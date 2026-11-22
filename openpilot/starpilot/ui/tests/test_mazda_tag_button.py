"""The Mazda TAG button on the Large driving screen (mazda_tag_button.py): where it goes (below the set speed / speed
limit group, clear of every other widget, Quick Select and the alert area, mirrored with right-hand drive), when it
shows (the setting with the master switch, a Mazda the speed control is for, Quick Select closed), and that one tap
publishes exactly one bookmark. Pure: no raylib needed (the drawing itself is not rendered here)."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.ui import mazda_tag_button as T
from openpilot.starpilot.ui.large_speed_geometry import confirmation_bounds, confirmation_buttons
from openpilot.starpilot.ui.onroad_customization import PROFILES, default_document, widget_size
from openpilot.starpilot.ui.onroad_favorites_geometry import intersects

ON = C.decode(b'{"version":7,"enabled":true,"tag_button":true}')


def mazda_cp():
  return CarInterface.get_params(CAR.MAZDA_CX5, gen_empty_fingerprint(), [], False, False, False)


def layouts():
  """The default document and the speed group's variants: sources inline / as a card, the group moved down, the CEM
  gauge off, a widget dropped right under it."""
  out = {"default": default_document()}
  for name, extra in (("sources", {"speedSources": True}), ("sources_card", {"speedSources": True, "speedSourcesCard": True})):
    d = default_document()
    d.update(extra)
    out[name] = d
  d = default_document()
  d["layouts"]["large"]["cruise_limits"]["y"] = 140
  out["moved_down"] = d
  d = default_document()
  d["layouts"]["large"]["cem_gauge"]["enabled"] = False
  d["speedSources"] = True
  out["no_gauge"] = d
  d = default_document()
  d["layouts"]["large"]["jetlink_status"].update(enabled=True, y=395)
  out["jetlink_under"] = d
  return out


def boxes(document, viewport, rhd):
  """Every rect the button must not touch: the speed group in all its states, the other widgets, Quick Select."""
  at = document["layouts"]["large"]["cruise_limits"]
  out = [(at["x"], at["y"], *widget_size(document, "large", "cruise_limits"))]
  for opened in (False, True):
    left, top, right, bottom = confirmation_bounds("large", at["x"], at["y"], opened=opened)
    out.append((left, top, right - left, bottom - top))
    for b in confirmation_buttons("large", (left, top, right, bottom)):
      out.append((b[0], b[1], b[2] - b[0], b[3] - b[1]))
  out += T.obstacles(document, viewport, rhd)     # (+ Quick Select's corner cue / drawer and its row shelf;
  return out                                       # its column shelf opens only when the row cannot: the button hides)


@pytest.mark.parametrize("viewport", (1860, 2160))
@pytest.mark.parametrize("rhd", (False, True))
@pytest.mark.parametrize("name", list(layouts()))
def test_the_button_is_clear_of_everything_and_above_the_alert_area(name, rhd, viewport):
  document = layouts()[name]
  rect = T.tag_rect(document, viewport, rhd)
  assert rect is not None, name
  x, y, w, h = rect
  assert (w, h) == T.SIZE and min(w, h) >= 90                       # a target to hit while driving
  assert 30 <= x and x + w <= viewport - 30 and 30 <= y
  assert y + h <= T.ALERT_TOP - T.GAP                               # above a small or mid alert
  for b in boxes(document, viewport, rhd):
    assert not intersects(rect, b), (name, rect, b)
  # under the lowest thing the speed group can draw, never higher
  assert y >= T.speed_group_bottom(document) + T.GAP - 1e-6


def test_the_spot_top_left_directly_under_the_speed_group():
  d = default_document()
  at = d["layouts"]["large"]["cruise_limits"]
  # the SLC confirmation notice is the lowest part of the group by default (75 - 12 + 321 = 384)
  assert abs(T.speed_group_bottom(d) - 383.92) < 0.01
  assert T.tag_rect(d, 2160) == (at["x"], 400, 208, 112) == T.tag_rect(d, 1860)
  # taller with the sources row / card: lower, and to the right of the CEM road gauge under the group
  for extra, want_y in (({"speedSources": True}, 457), ({"speedSources": True, "speedSourcesCard": True}, 465)):
    d = default_document()
    d.update(extra)
    x, y, _, _ = T.tag_rect(d, 2160)
    gauge = d["layouts"]["large"]["cem_gauge"]
    assert y == want_y and x == gauge["x"] + PROFILES["large"]["widgets"]["cem_gauge"]["width"] + T.GAP
  # the group moved: the button moves with it (and slides right of the gauge when the gap gets too small)
  d = default_document()
  d["layouts"]["large"]["cruise_limits"].update(x=120)
  assert T.tag_rect(d, 2160)[:2] == (120, 400)
  d["layouts"]["large"]["cruise_limits"].update(y=100)
  assert T.tag_rect(d, 2160)[:2] == (328, 425)


def test_right_hand_drive_mirrors_it():
  d = default_document()
  for viewport in (1860, 2160):
    x, y, w, _ = T.tag_rect(d, viewport, rhd=True)
    assert x > viewport / 2 and y >= T.speed_group_bottom(d) + T.GAP - 1e-6


def test_no_clear_spot_no_button():
  d = default_document()
  d["layouts"]["large"]["nav_map"].update(enabled=True, x=30, y=30, width=1800, height=1020)   # an underlay: ignored
  assert T.tag_rect(d, 2160) == (88, 400, 208, 112)
  d = default_document()
  d["layouts"]["large"]["stopped_timer"].update(enabled=True, x=30, y=380)    # in the way: it slides right of it
  x, y, _, _ = T.tag_rect(d, 2160)
  assert y == 400 and x == 30 + 660 + T.GAP
  assert T._place([(0, 0, 3000, 3000)], 88, 576, 400, 614) is None             # nothing clear: not shown
  assert T._place([(0, 380, 3000, 300)], 88, 576, 400, 614) is None             # (lower would reach the alerts)


# ---------------- showing and tapping ----------------

def state(document=None, quick_select_open=False, rhd=False, viewport=2160):
  return NS(customization=document or default_document(), viewport_width=viewport, quick_select_open=quick_select_open,
            right_hand_drive=rhd)


def button(cfg=ON, cp=None, publish=None):
  publish = publish or Mock(return_value=True)
  car = mazda_cp() if cp is None else cp
  return T.TagButton(publish, lambda: cfg, lambda: car), publish


def drawn(b, st, now):
  """What render() does before drawing (raylib is not needed for this)."""
  b.rect = b.place(st, now)
  b.drawn_at = now if b.rect is not None else None
  return b.rect


def test_shown_only_with_the_setting_and_the_master_switch_on_a_mazda():
  b, _ = button()
  assert drawn(b, state(), 1.0) == (88, 400, 208, 112)
  for raw in (b'{"version":7,"enabled":true}', b'{"version":7,"tag_button":true}', b'{"version":6,"enabled":true}'):
    b, _ = button(C.decode(raw))
    assert drawn(b, state(), 1.0) is None, raw
  b, _ = button(C.Config.off(valid=False))
  assert drawn(b, state(), 1.0) is None
  from opendbc.car.chrysler.interface import CarInterface as Chrysler
  from opendbc.car.chrysler.values import CAR as CHRYSLER
  b, _ = button(cp=Chrysler.get_params(CHRYSLER.JEEP_GRAND_CHEROKEE, gen_empty_fingerprint(), [], False, False, False))
  assert drawn(b, state(), 1.0) is None
  b, _ = button()
  assert drawn(b, state(quick_select_open=True), 1.0) is None     # Quick Select open: hidden, its taps are its own


def test_the_setting_is_read_again_every_two_seconds():
  cfg = {"now": ON}
  b = T.TagButton(Mock(), lambda: cfg["now"], mazda_cp)
  assert drawn(b, state(), 10.0) is not None
  cfg["now"] = C.Config()
  assert drawn(b, state(), 11.0) is not None
  assert drawn(b, state(), 12.1) is None
  cfg["now"] = ON
  assert drawn(b, state(), 14.2) is not None
  b.read_config = Mock(side_effect=OSError)                          # an unreadable setting: not shown
  assert drawn(b, state(), 16.3) is None


def test_one_tap_publishes_exactly_one_bookmark():
  b, publish = button()
  drawn(b, state(), 1.0)
  assert b.press(150, 450, 1.05)
  b.move(160, 455)
  assert b.release(160, 455, 1.12) and publish.call_count == 1
  assert b.label(1.2) == "SAVED" and b.label(1.12 + T.FEEDBACK + 0.05) == "TAG"
  # a second tap right after (a bounce) is not a second bookmark; a later one is
  drawn(b, state(), 1.3)
  assert b.press(150, 450, 1.3) and not b.release(150, 450, 1.35) and publish.call_count == 1
  drawn(b, state(), 2.0)
  assert b.press(150, 450, 2.0) and b.release(150, 450, 2.05) and publish.call_count == 2


def test_a_drag_a_release_outside_or_a_cancel_saves_nothing():
  b, publish = button()
  drawn(b, state(), 1.0)
  assert b.press(150, 450, 1.0)
  assert not b.release(150 + T.SLOP + 5, 450, 1.1)                  # dragged away
  drawn(b, state(), 2.0)
  assert b.press(100, 410, 2.0)
  b.cancel()
  assert not b.release(100, 410, 2.1)
  drawn(b, state(), 3.0)
  assert b.press(290, 505, 3.0) and not b.release(320, 520, 3.1)    # released off the button
  assert publish.call_count == 0


def test_a_refused_bookmark_says_so():
  b, publish = button(publish=Mock(return_value=False))
  drawn(b, state(), 1.0)
  assert b.press(150, 450, 1.0) and not b.release(150, 450, 1.1)
  assert publish.call_count == 1 and b.label(1.2) == "NOT SAVED"
  b, _ = button(publish=Mock(side_effect=RuntimeError))
  drawn(b, state(), 1.0)
  assert b.press(150, 450, 1.0) and not b.release(150, 450, 1.1) and b.label(1.2) == "NOT SAVED"


def test_it_takes_no_touch_outside_itself_or_when_not_drawn():
  b, _ = button()
  assert not b.press(150, 450, 1.0)                                 # never drawn
  drawn(b, state(), 1.0)
  assert not b.press(150, 450, 1.0 + T.FRESH + 0.1)                 # not drawn recently (another screen)
  drawn(b, state(), 2.0)
  # Quick Select's corner cue and drawer, the speed group, the gauge, the following distance, the driver monitor
  for x, y in ((78, 1002), (60, 960), (300, 1000), (200, 200), (400, 300), (150, 600), (150, 900), (330, 900),
               (87, 450), (296, 450), (150, 399), (150, 512)):
    assert not b.press(x, y, 2.0), (x, y)
  # off: no touch at all
  b, _ = button(C.Config())
  drawn(b, state(), 1.0)
  assert not b.press(150, 450, 1.0)


def test_the_favorites_corner_claim_never_overlaps_it():
  # (onroad_favorites.claims_corner / corner(): the cue at (30 + 48 s, 1050 - 48 s), the swipe zone 340 x 180 s)
  scale = 1.0
  cue = (30 + 48 * scale - 22 * scale, 1050 - 48 * scale - 22 * scale, 44 * scale, 44 * scale)
  zone = (30, 1050 - 180 * scale, 340, 180 * scale)
  for document in layouts().values():
    for rhd in (False, True):
      rect = T.tag_rect(document, 2160, rhd)
      assert not intersects(rect, cue) and not intersects(rect, zone)
