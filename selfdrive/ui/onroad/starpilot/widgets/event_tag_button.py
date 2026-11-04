"""On-road TAG button: one tap saves a tag (route, segment, time, GPS, speed, state) for tools/event_tags_pack.py.

Everything here is wrapped: an exception escaping into the raylib loop would kill the UI (boot logo for ~13 s).
The tap itself does no file I/O (see starpilot/common/event_tags.py)."""
import time

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.onroad.starpilot.widgets.base import LayoutWidget
from openpilot.starpilot.common.event_tags import TagWriter, build_tag_record
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

SIZE = 192.0
RADIUS = 72.0
FLASH_S = 2.0          # how long the button shows "#N" after a tap
COOLDOWN_S = 1.5       # a second tap within this time is ignored (double taps on a bumpy road)
ALSO_BOOKMARK = True   # also flag the moment in the qlog / on connect (same path as a wheel-button bookmark)

BG = rl.Color(0, 0, 0, 166)
BG_PRESSED = rl.Color(0, 0, 0, 230)
RING = rl.Color(255, 255, 255, 90)
TEXT = rl.Color(255, 255, 255, 230)
DONE = rl.Color(22, 127, 64, 255)


def _enum_names():
  """{field: {int: name}} for a few enums, so tags.jsonl is readable. Optional: any failure gives {}."""
  try:
    from cereal import car, custom, log
    inv = lambda e: {v: k for k, v in e.schema.enumerants.items()}  # noqa: E731
    return {
      "state": inv(log.SelfdriveState.OpenpilotState),
      "gear": inv(car.CarState.GearShifter),
      "personality": inv(log.LongitudinalPersonality),
      "events": inv(custom.StarPilotOnroadEvent.EventName),
    }
  except Exception:
    return {}


class EventTagButtonWidget(LayoutWidget):
  def __init__(self, writer=None):
    super().__init__("event_tag", priority=3)
    self._writer = writer
    if self._writer is None:
      try:
        self._writer = TagWriter(params=ui_state.params)
      except Exception:
        self._writer = None
    self._names = _enum_names()
    self._font = gui_app.font(FontWeight.BOLD)
    self._tagged_at: float | None = None
    self._last_n: int | None = None
    self._count = 0

  @property
  def is_visible(self) -> bool:
    try:
      return bool(ui_state.started)
    except Exception:
      return False

  def get_size(self) -> tuple[float, float]:
    return SIZE, SIZE

  def _handle_mouse_release(self, _) -> None:
    self.tag()

  def tag(self, now: float | None = None) -> bool:
    """Save one tag. Returns True when a tag was queued. Never raises."""
    try:
      now = time.monotonic() if now is None else now
      if self._tagged_at is not None and now - self._tagged_at < COOLDOWN_S:
        return False
      self._tagged_at = now
      self._count += 1
      record = build_tag_record(ui_state.sm, ui_state, time.time(), time.monotonic(), self._names)  # noqa: TID251  (wall clock: matched to log file mtimes)
      self._last_n = self._writer.submit(record) if self._writer is not None else None
      if ALSO_BOOKMARK:
        try:
          pm = ui_state.params_memory
          pm.put_int("WheelButtonBookmarkCounter", (pm.get_int("WheelButtonBookmarkCounter") or 0) + 1)
        except Exception:
          pass
      return True
    except Exception:
      return False

  def _render(self, rect: rl.Rectangle) -> None:
    try:
      cx, cy = rect.x + rect.width / 2, rect.y + rect.height / 2
      flashing = self._tagged_at is not None and time.monotonic() - self._tagged_at < FLASH_S
      rl.draw_circle(int(cx), int(cy), RADIUS, BG_PRESSED if self.is_pressed else BG)
      ring = DONE if flashing else RING
      rl.draw_ring(rl.Vector2(cx, cy), RADIUS - 6, RADIUS, 0, 360, 48, ring)
      if flashing:
        label = f"#{self._last_n}" if self._last_n is not None else "OK"
      else:
        label = "TAG"
      size = 44 if len(label) <= 4 else 34
      ts = measure_text_cached(self._font, label, size)
      rl.draw_text_ex(self._font, label, rl.Vector2(cx - ts.x / 2, cy - ts.y / 2), size, 0, DONE if flashing else TEXT)
    except Exception:
      pass
