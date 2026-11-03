"""
The "Map Build States" tile and dialog of the Mazda tiles (vehicle.py): a tick per state of StarPilot's map selection.

A ticked state is built in full by curve_mapd (starpilot/system/curve_mapd.py) while the car is parked, wherever it
is parked and whatever the Map Build Range says; an unticked one only within range of the car. The owner ticks the
states of a trip while parked at home, unticks them after. The ticks are the persistent param MazdaCurveMapStates
(a CSV of the same tokens as MapsSelected). The dialog lists the regions selected in the map settings (plus any ticked
one no longer selected, so it can be unticked), with what the device knows of each: how many of its map folders are
downloaded and how many of those have current curve files ("3 of 9 areas built", "Built", "Map not downloaded").
The daemon's status line (CurveMapBuildStatus) is shown under the list.

The folder scan (390 folders on the owner's device) runs in a background thread when the dialog opens and every few
seconds after; the dialog never blocks the UI on it.
"""
from __future__ import annotations

import re
import threading
import time

import pyray as rl

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.scroll_panel import GuiScrollPanel
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import gui_text_box
from openpilot.selfdrive.ui.layouts.settings.starpilot.aethergrid import (
  AetherListColors,
  DEFAULT_PANEL_STYLE,
  aether_begin_scissor_mode,
  aether_end_scissor_mode,
  draw_rounded_fill,
  draw_rounded_stroke,
  draw_selection_list_row,
  snap_rect,
  with_alpha,
)
from openpilot.starpilot.common.maps_catalog import MAP_TOKEN_LABELS
from openpilot.starpilot.common.maps_selection import COUNTRY_PREFIX

TICKED_PARAM = "MazdaCurveMapStates"
SELECTED_PARAM = "MapsSelected"
STATUS_PARAM = "CurveMapBuildStatus"
US_COUNTRY_TOKEN = f"{COUNTRY_PREFIX}US"
ROW_HEIGHT = 104.0
SCAN_INTERVAL = 5.0
STATUS_INTERVAL = 1.0
PANEL_STYLE = DEFAULT_PANEL_STYLE


def parse_tokens(raw) -> list[str]:
  if isinstance(raw, bytes):
    raw = raw.decode("utf-8", "ignore")
  out = []
  for t in (raw or "").split(","):
    t = t.strip()
    if t and t not in out:
      out.append(t)
  return out


def token_label(token: str) -> str:
  if token == US_COUNTRY_TOKEN:
    return tr("Whole U.S.")
  return tr(MAP_TOKEN_LABELS.get(token, token.split(".", 1)[-1]))


def listed_tokens(selected: list[str], ticked: list[str]) -> list[str]:
  """The regions the dialog lists: the map selection in its order, then ticked ones no longer selected."""
  return selected + [t for t in ticked if t not in selected]


def region_line(downloaded: int, built: int, placeable: bool, ticked: bool) -> str:
  """The subtitle of a region row from what the daemon knows: see region_status in curve_mapd."""
  if not placeable:
    return tr("Not on the device's map (no bounding box): not built")
  if downloaded == 0:
    return tr("Map not downloaded")
  if built >= downloaded:
    base = tr("Built, {} areas").format(downloaded) if downloaded != 1 else tr("Built, 1 area")
  else:
    base = tr("{} of {} areas built").format(built, downloaded)
  return base + (tr(" - whole state, always") if ticked else tr(" - only within range"))


def tile_value(ticked: list[str], status: str) -> str:
  """What the tile shows: the build in progress when there is one, else the ticked states."""
  m = re.match(r"Building area (\d+) of (\d+) \(now (.+?)\)", status or "")
  if m:
    return tr("Area {} of {} (now {})").format(m.group(1), m.group(2), m.group(3))
  if not ticked:
    return tr("None")
  first = token_label(ticked[0])
  return first if len(ticked) == 1 else tr("{} + {}").format(first, len(ticked) - 1)


def tick_summary(ticked: list[str], listed: list[str]) -> str:
  n = len(ticked)
  if n == 0:
    return tr("No state ticked: only map areas within range of the car are built.")
  return tr("{} of {} ticked: built in full while parked, wherever the car is.").format(n, len(listed))


class CurveMapStatesDialog(Widget):
  """Rows of the map selection with a tick each; tapping a row flips its tick and saves at once. 'Done' closes."""

  def __init__(self, params: Params | None = None, params_memory: Params | None = None, scan=None):
    super().__init__()
    self._params = params or Params()
    self._params_memory = params_memory or Params(memory=True)
    self._scan = scan or self._default_scan
    self._font_title = gui_app.font(FontWeight.BOLD)
    self._font_btn = gui_app.font(FontWeight.SEMI_BOLD)
    self._scroll = GuiScrollPanel()
    self._selected = parse_tokens(self._params.get(SELECTED_PARAM, encoding="utf-8"))
    self._ticked = parse_tokens(self._params.get(TICKED_PARAM, encoding="utf-8"))
    self._status_text = ""
    self._status_at = 0.0
    self._region_info: dict[str, tuple[int, int, bool]] = {}
    self._scan_at = 0.0
    self._scan_thread: threading.Thread | None = None
    self._lock = threading.Lock()
    self._row_rects: list[tuple[str, rl.Rectangle]] = []
    self._done_rect = rl.Rectangle(0, 0, 0, 0)
    self._list_rect = rl.Rectangle(0, 0, 0, 0)
    self._pressed: str | None = None
    self._done_pressed = False
    self.set_touch_valid_callback(self._scroll.is_touch_valid)

  # ---- data ----
  def _default_scan(self, tokens: list[str]) -> dict[str, tuple[int, int, bool]]:
    """(downloaded folders, built folders, placeable) per token, from curve_mapd's read-only helpers."""
    from openpilot.starpilot.system import curve_mapd as D
    folders = D.map_folders()
    regions = D.Regions(selected=tokens)
    return {t: D.region_status(t, folders, regions=regions) for t in tokens}

  def _start_scan(self):
    tokens = listed_tokens(self._selected, self._ticked)

    def worker():
      try:
        info = self._scan(tokens)
      except Exception:
        info = {}
      with self._lock:
        self._region_info = info

    self._scan_thread = threading.Thread(target=worker, daemon=True)
    self._scan_thread.start()
    self._scan_at = time.monotonic()

  def _refresh(self):
    now = time.monotonic()
    if now - self._status_at >= STATUS_INTERVAL:
      self._status_at = now
      try:
        self._status_text = self._params_memory.get(STATUS_PARAM, encoding="utf-8") or ""
      except Exception:
        self._status_text = ""
    if (self._scan_thread is None or not self._scan_thread.is_alive()) and now - self._scan_at >= SCAN_INTERVAL:
      self._start_scan()

  def _toggle(self, token: str):
    if token in self._ticked:
      self._ticked.remove(token)
    else:
      self._ticked.append(token)
    self._ticked = [t for t in listed_tokens(self._selected, self._ticked) if t in self._ticked]
    try:
      self._params.put(TICKED_PARAM, ",".join(self._ticked))
    except Exception:
      pass

  # ---- input ----
  def _handle_mouse_press(self, mouse_pos):
    if rl.check_collision_point_rec(mouse_pos, self._done_rect):
      self._done_pressed = True
      return
    for token, rect in self._row_rects:
      if rl.check_collision_point_rec(mouse_pos, rect) and rl.check_collision_point_rec(mouse_pos, self._list_rect):
        self._pressed = token
        return

  def _handle_mouse_release(self, mouse_pos):
    if self._done_pressed:
      self._done_pressed = False
      if rl.check_collision_point_rec(mouse_pos, self._done_rect):
        gui_app.pop_widget()
      return
    token, self._pressed = self._pressed, None
    if token is None or not self._scroll.is_touch_valid():
      return
    for t, rect in self._row_rects:
      if t == token and rl.check_collision_point_rec(mouse_pos, rect) and rl.check_collision_point_rec(mouse_pos, self._list_rect):
        self._toggle(token)
        return

  def _handle_mouse_event(self, mouse_event):
    pass

  # ---- drawing ----
  def _render(self, rect: rl.Rectangle):
    self._refresh()
    rl.draw_rectangle(0, 0, gui_app.width, gui_app.height, rl.Color(0, 0, 0, 160))
    dialog_w = min(1800, int(rect.width - 80))
    dialog_h = min(1000, int(rect.height - 40))
    dx = int(rect.x + (rect.width - dialog_w) / 2)
    dy = int(rect.y + (rect.height - dialog_h) / 2)
    d_rect = snap_rect(rl.Rectangle(dx, dy, dialog_w, dialog_h))
    draw_rounded_fill(d_rect, rl.Color(10, 12, 16, 255), radius_px=24)
    draw_rounded_stroke(d_rect, rl.Color(255, 255, 255, 16), radius_px=24)
    rl.draw_rectangle_rec(rl.Rectangle(d_rect.x, d_rect.y, d_rect.width, 3), PANEL_STYLE.accent)

    margin = 60
    title = tr("Map Build States")
    ts = measure_text_cached(self._font_title, title, 44)
    rl.draw_text_ex(self._font_title, title, rl.Vector2(dx + (dialog_w - ts.x) // 2, dy + 36), 44, 0, rl.WHITE)
    listed = listed_tokens(self._selected, self._ticked)
    gui_text_box(rl.Rectangle(dx + margin, dy + 96, dialog_w - 2 * margin, 40), tick_summary(self._ticked, listed), 26,
                 AetherListColors.SUBTEXT, font_weight=FontWeight.NORMAL)

    # the bottom: the daemon's status line and the Done button
    btn_h, btn_w = 96, 360
    self._done_rect = rl.Rectangle(dx + dialog_w - margin - btn_w, dy + dialog_h - btn_h - 40, btn_w, btn_h)
    status_rect = rl.Rectangle(dx + margin, self._done_rect.y - 4, dialog_w - 2 * margin - btn_w - 30, btn_h + 8)
    gui_text_box(status_rect, self._status_text or tr("Curve files: waiting for the parked build to report"), 24,
                 AetherListColors.SUBTEXT, font_weight=FontWeight.NORMAL, line_scale=0.95)
    pressed = self._done_pressed
    draw_rounded_fill(self._done_rect, with_alpha(PANEL_STYLE.accent, 255 if pressed else 220), radius_px=btn_h / 2)
    done = tr("Done")
    bs = measure_text_cached(self._font_btn, done, 36)
    rl.draw_text_ex(self._font_btn, done, rl.Vector2(self._done_rect.x + (btn_w - bs.x) // 2, self._done_rect.y + (btn_h - bs.y) // 2), 36, 0, rl.WHITE)

    # the list, scrolled
    list_top = dy + 150
    self._list_rect = rl.Rectangle(dx + margin, list_top, dialog_w - 2 * margin, self._done_rect.y - 24 - list_top)
    content = rl.Rectangle(self._list_rect.x, self._list_rect.y, self._list_rect.width, max(1.0, len(listed) * ROW_HEIGHT))
    offset = self._scroll.update(self._list_rect, content)
    with self._lock:
      info = dict(self._region_info)
    self._row_rects = []
    aether_begin_scissor_mode(int(self._list_rect.x), int(self._list_rect.y), int(self._list_rect.width), int(self._list_rect.height))
    if not listed:
      gui_text_box(rl.Rectangle(self._list_rect.x + 24, self._list_rect.y + 24, self._list_rect.width - 48, 120),
                   tr("No regions are selected in StarPilot's map settings. Select and download the states first; they appear here to tick."),
                   28, AetherListColors.SUBTEXT)
    mouse = gui_app.last_mouse_event.pos
    for k, token in enumerate(listed):
      row = rl.Rectangle(self._list_rect.x, self._list_rect.y + offset + k * ROW_HEIGHT, self._list_rect.width, ROW_HEIGHT)
      self._row_rects.append((token, row))
      if row.y + row.height < self._list_rect.y or row.y > self._list_rect.y + self._list_rect.height:
        continue
      ticked = token in self._ticked
      downloaded, built, placeable = info.get(token, (0, 0, True))
      subtitle = region_line(downloaded, built, placeable, ticked) if info else tr("Checking the map folders...")
      if token not in self._selected:
        subtitle += tr(" - not in the map selection any more")
      hovered = rl.check_collision_point_rec(mouse, row) and rl.check_collision_point_rec(mouse, self._list_rect)
      draw_selection_list_row(
        row,
        title=token_label(token),
        subtitle=subtitle,
        action_text=tr("Whole state") if ticked else tr("In range"),
        current=ticked,
        hovered=hovered,
        pressed=self._pressed == token,
        is_last=k == len(listed) - 1,
        action_width=200,
        action_pill=True,
        action_text_size=26,
        action_pill_height=56,
        action_pill_width=190,
        row_separator=PANEL_STYLE.divider_color,
        current_bg=PANEL_STYLE.current_fill,
        current_border=PANEL_STYLE.current_border,
        action_fill=with_alpha(AetherListColors.SUCCESS_SOFT, 36) if ticked else rl.Color(255, 255, 255, 8),
        action_border=with_alpha(AetherListColors.SUCCESS, 60) if ticked else rl.Color(255, 255, 255, 20),
        action_text_color=AetherListColors.HEADER,
      )
    aether_end_scissor_mode()
