"""The Mazda speed control's TAG button on the Large driving screen (MazdaSpeedControlConfig `tag_button`, v7).

One tap saves one bookmark through the SAME path as 7.0's Bookmark action (the runtime's BOOKMARK favorite action:
`bookmarkButton` + `userBookmark`, the stock "Bookmark Saved"); the Mazda speed control turns each userBookmark into
a `tag` line in its event log (car/mazda/speed/caller.py).

Where (owner: a left-hand-drive car, the button within the driver's reach): top left, directly BELOW the set speed /
speed limit group (`cruise_limits`), left-aligned with it, GAP under the lowest thing that group can draw - its card
(taller with the speed-limit sources row or card), the limit sign's tap area, the SLC confirmation notice and its
Accept / Reject prompt - whether or not those show right now, so the button never moves under the driver's finger.
It must stay clear of every other enabled onroad widget (the CEM road gauge, the following distance, the driver
monitor icon, ...), of the Quick Select corner and of the alert area (a mid alert's top): when something is in the
way it slides right (still under the speed group), else down, else it is not shown. With right-hand drive (the
driver monitor's isRHD, as the driver monitor icon and the following distance mirror) it is mirrored to the right.
It is drawn after the alerts, so it stays visible and tappable while an alert (or a speed control notice) shows; it
hides only while Quick Select is open.

Geometry and the tap logic are pure (no raylib: tests/test_mazda_tag_button.py); only render() draws.
"""
import math
import time

from openpilot.starpilot.ui.large_speed_geometry import LIMIT_RECTS, WIDTH, confirmation_footprint
from openpilot.starpilot.ui.onroad_customization import PROFILES, widget_size
from openpilot.starpilot.ui.onroad_favorites_geometry import ROW_BODY, contains, intersects

SIZE = (208, 112)              # px at the Large (tizi / comma 3X) scale: the touch target is the whole pill
GAP = 16                       # px clear of every other thing
SIGN_PADDING = (19, 26)        # onroad_state.SIGN_PADDING[LARGE]: the limit sign's tap area around its rect
CONTENT_TOP, CONTENT_HEIGHT = 30, 1020
ALERT_TOP = CONTENT_TOP + CONTENT_HEIGHT - 420     # a mid alert's top (onroad_alerts: 420 px from the bottom)
QUICK_SELECT = ((30, 870, 340, 180), ROW_BODY)     # its corner (180 px * scale) / drawer zone and its open shelf
NOT_OBSTACLES = ("cruise_limits", "torque_bar", "pip_left", "pip_right", "nav_map")   # placed itself / underlays
MIRRORED = ("driver_monitor", "following_distance")   # drawn mirrored with right-hand drive at their defaults
REREAD = 2.0                   # s between two reads of the saved setting
FRESH = 0.25                   # s: a press counts only on a button drawn this recently
SLOP = 36.0                    # px a tap may travel
DEBOUNCE = 0.6                 # s: one tap, one bookmark
FEEDBACK = 0.9                 # s the "SAVED" / "NOT SAVED" feedback shows


def speed_group_bottom(document):
  """The lowest y the set speed / speed limit group can draw at its saved place (any of its states)."""
  at = document["layouts"]["large"]["cruise_limits"]
  default = PROFILES["large"]["widgets"]["cruise_limits"]["default"]
  dy = at["y"] - default["y"]
  card = at["y"] + widget_size(document, "large", "cruise_limits")[1]          # with the sources row / card
  sign = LIMIT_RECTS["large"][1] + dy + LIMIT_RECTS["large"][3] + SIGN_PADDING[1]
  notice = confirmation_footprint("large")
  confirm = at["y"] + notice["y"] + notice["height"]                        # the notice and the open prompt
  return max(card, sign, confirm)


def obstacles(document, viewport_width, rhd=False):
  """(x, y, w, h) of everything the button must stay clear of, in screen coordinates."""
  out = []
  layout = document["layouts"]["large"]
  widgets = PROFILES["large"]["widgets"]
  bounds = PROFILES["large"]["bounds"]
  for key, spec in widgets.items():
    saved = layout.get(key)
    if saved is None or not saved.get("enabled") or key in NOT_OBSTACLES:
      continue
    x = saved["x"] + spec.get("anchorShift", 0) * (viewport_width - 1860)
    y = saved["y"]
    width, height = widget_size(document, "large", key)
    if key in MIRRORED and rhd and (saved["x"], saved["y"]) == (spec["default"]["x"], spec["default"]["y"]):
      x = 2 * bounds["x"] + bounds["width"] - x - width       # (onroad_dm.monitor_bounds; the personality next to it)
    out.append((x, y, width, height))
  out += list(QUICK_SELECT)
  return out


def _place(blocks, x0, x1, y0, y1):
  """The first free (x, y) for the button: from (x0, y0) to the right within [x0, x1], then lower, never below y1."""
  w, h = SIZE
  y = y0
  while y + h <= y1 + 1e-6:
    x = x0
    while x + w <= x1 + 1e-6:
      hit = [b for b in blocks if intersects((x - GAP, y - GAP, w + 2 * GAP, h + 2 * GAP), b)]
      if not hit:
        return x, y
      x = max(b[0] + b[2] for b in hit) + GAP
    row = [b for b in blocks if b[1] < y + h + GAP and y - GAP < b[1] + b[3] and b[0] < x1 and x0 < b[0] + b[2]]
    if not row:
      return None
    y = min(b[1] + b[3] for b in row) + GAP
  return None


def tag_rect(document, viewport_width, rhd=False):
  """(x, y, w, h) of the TAG button on the Large driving screen, or None when it does not fit anywhere clear."""
  at = document["layouts"]["large"]["cruise_limits"]
  blocks = obstacles(document, viewport_width, rhd)
  if rhd:                                   # search in a mirrored screen, then mirror the answer back
    blocks = [(viewport_width - b[0] - b[2], b[1], b[2], b[3]) for b in blocks]
  y0 = math.ceil(speed_group_bottom(document)) + GAP
  y1 = ALERT_TOP - GAP
  spot = (_place(blocks, at["x"], at["x"] + WIDTH, y0, y1) or
          _place(blocks, CONTENT_TOP, viewport_width / 2, y0, y1))
  if spot is None:
    return None
  x, y = spot
  if rhd:
    x = viewport_width - x - SIZE[0]
  return (x, y, *SIZE)


class TagButton:
  """The button's state: whether it shows (the saved setting, the car), where, a press, the feedback.
  publish() -> bool saves one bookmark (the runtime's BOOKMARK action); read_config() -> Config; car() -> CarParams."""

  def __init__(self, publish, read_config, car, clock=time.monotonic):
    self.publish = publish
    self.read_config = read_config
    self.car = car
    self.clock = clock
    self._on, self._read_at = False, None
    self.rect = None                # where it was drawn last, and when
    self.drawn_at = None
    self.press_pos = None
    self.travel = 0.0
    self.last_fire = -math.inf
    self.feedback = None            # (True: saved / False: not saved, when)

  def setting_on(self, now):
    if self._read_at is None or now - self._read_at >= REREAD or now < self._read_at:
      self._read_at = now
      try:
        from openpilot.starpilot.car.mazda.speed.caller import eligible
        self._on = bool(self.read_config().tag_on and eligible(self.car()))
      except Exception:
        self._on = False
    return self._on

  def place(self, state, now):
    """The rect to draw this frame, or None (not shown)."""
    if not self.setting_on(now) or getattr(state, "quick_select_open", False):
      return None
    try:
      return tag_rect(state.customization, state.viewport_width, bool(getattr(state, "right_hand_drive", False)))
    except (KeyError, TypeError, ValueError):
      return None

  def shown(self, now):
    return self.rect is not None and self.drawn_at is not None and 0 <= now - self.drawn_at <= FRESH

  def press(self, x, y, now):
    """True: the press is the button's (nothing else gets this touch)."""
    if not self.shown(now) or not contains(self.rect, x, y):
      return False
    self.press_pos, self.travel = (x, y), 0.0
    return True

  def move(self, x, y):
    if self.press_pos is not None:
      self.travel = max(self.travel, math.hypot(x - self.press_pos[0], y - self.press_pos[1]))

  def release(self, x, y, now):
    """True when this release saved a bookmark."""
    if self.press_pos is None:
      return False
    self.move(x, y)
    self.press_pos = None
    if self.travel > SLOP or self.rect is None or not contains(self.rect, x, y) or now - self.last_fire < DEBOUNCE:
      return False
    self.last_fire = now
    try:
      ok = bool(self.publish())
    except Exception:
      ok = False
    self.feedback = (ok, now)
    return ok

  def cancel(self):
    self.press_pos = None

  def label(self, now):
    if self.feedback is not None and 0 <= now - self.feedback[1] <= FEEDBACK:
      return "SAVED" if self.feedback[0] else "NOT SAVED"
    return "TAG"

  def render(self, fonts, state, now=None):
    now = self.clock() if now is None else now
    rect = self.place(state, now)
    self.rect, self.drawn_at = rect, (now if rect is not None else None)
    if rect is None:
      self.press_pos = None
      return
    from openpilot.starpilot.ui.presentation import FontRole
    from openpilot.starpilot.ui.raylib import rl
    pressed = self.press_pos is not None
    text = self.label(now)
    saved = text == "SAVED"
    inset = 4 if pressed else 0
    box = rl.Rectangle(rect[0] + inset, rect[1] + inset, rect[2] - 2 * inset, rect[3] - 2 * inset)
    fill = (rl.Color(255, 196, 0, 245) if saved else rl.Color(70, 58, 18, 245) if pressed else
            rl.Color(15, 13, 23, 225))
    border = rl.Color(255, 120, 120, 255) if text == "NOT SAVED" else rl.Color(255, 196, 0, 255)
    rl.draw_rectangle_rounded(box, 1.0, 24, fill)
    rl.draw_rectangle_rounded_lines_ex(box, 1.0, 24, 5, border)
    size = 52 if text == "TAG" else 34
    while size > 20 and fonts.measure(text, FontRole.SEMI_BOLD, size).width > box.width - 28:
      size -= 2
    measured = fonts.measure(text, FontRole.SEMI_BOLD, size)
    ink = rl.Color(15, 13, 23, 255) if saved else rl.Color(255, 255, 255, 255)
    fonts.draw(text, FontRole.SEMI_BOLD, size, box.x + (box.width - measured.width) / 2,
               box.y + (box.height - measured.height) / 2, ink)
