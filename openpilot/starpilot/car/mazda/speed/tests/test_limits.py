"""The speed limit follow (part f, F1/F2/F5): 7.0's slcState reading, the steady time with clean GPS trust, the anchor
rules (ours / engaged within the band / his), previews, the 43 mph fallback, test mode."""
from types import SimpleNamespace as N

from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed import limits as L
from openpilot.starpilot.car.mazda.speed.planner import Contribution, ENGAGED, HIS, OURS
from openpilot.starpilot.car.mazda.speed.tests.rig import MPH, Rig, driver, engaged, view

CFG = C.Config(enabled=True, speed_limit="on", limit_offset_mph=3)


def slc(mph, upcoming=0.0):
  return {"limit": None if mph is None else mph * MPH, "upcoming": upcoming * MPH, "t": 0.0}


def rig(cfg=CFG):
  return Rig([L.SpeedLimit(cfg, presses=cfg.presses("speed_limit"))])


def drive(r, seconds, limit, v=60.0, trust=0, chain=1, upcoming=0.0, events=(), **kw):
  n = int(round(seconds / r.dt))
  return r.step(v_mph=v, n=n, events=events, slc=slc(limit, upcoming), view=lambda t: view(t, trust=trust, chain=chain),
                **kw)


def test_the_reading_from_slcstate():
  row = lambda src, kind, mph: N(source=src, observationKind=kind, speedLimit=mph * MPH)  # noqa: E731
  st = N(observationKind="valid", source="map", speedLimit=45 * MPH, upcomingSpeedLimit=0.0, sourceReadings=[])
  assert abs(L.reading(st)[0] - 45 * MPH) < 1e-6
  # 7.0 selected vision: the map's own row is used; vision alone is no limit here
  st = N(observationKind="valid", source="vision", speedLimit=55 * MPH, upcomingSpeedLimit=35 * MPH,
         sourceReadings=[row("vision", "valid", 55), row("map", "valid", 45)])
  lim, up = L.reading(st)
  assert abs(lim - 45 * MPH) < 1e-6 and abs(up - 35 * MPH) < 1e-6
  st = N(observationKind="valid", source="vision", speedLimit=55 * MPH, upcomingSpeedLimit=0.0,
         sourceReadings=[row("vision", "valid", 55), row("map", "stale", 45)])
  assert L.reading(st) == (None, 0.0)
  assert L.reading(object()) == (None, 0.0)
  assert L.whole_limit(45 * MPH) == 45 and L.whole_limit(47 * MPH) is None and L.whole_limit(10 * MPH) is None
  assert L.whole_limit(70 / 3.6, metric=True) == 70


def test_his_own_80_on_a_70_road_is_never_pressed_down():
  r = rig()
  r.step(set_mph=80, events=[engaged(80)])
  drive(r, 8, 70, v=78)
  assert r.anchor_mph == 80 and r.p.owner == ENGAGED
  drive(r, 8, 55, v=78)                       # not in the 70 road's band (70-75) either
  assert r.anchor_mph == 80


def test_engaged_in_the_band_follows_a_change_and_then_it_is_ours():
  r = rig()
  drive(r, 6, 70, v=70)
  r.step(set_mph=73, events=[engaged(73)])
  drive(r, 2.5, 55, v=70)
  assert r.anchor_mph == 73                   # a lower limit needs its steady time first
  drive(r, 1.0, 55, v=70)
  assert r.anchor_mph == 58 and r.p.owner == OURS and r.p.ours_why == "limit"
  drive(r, 4.0, 65, v=58)
  assert r.anchor_mph == 58                   # a higher one waits longer
  drive(r, 1.5, 65, v=58)
  assert r.anchor_mph == 68
  # his SET+ / SET- make it his: no limit moves it again
  r.step(v_mph=66, events=[driver("SET_PLUS", 70)])
  assert r.p.owner == HIS
  drive(r, 8, 55, v=68)
  assert r.anchor_mph == 70


def test_engaging_within_the_band_alone_does_not_move_him():
  r = rig()
  drive(r, 6, 70, v=70)
  r.step(set_mph=75, events=[engaged(75)])
  drive(r, 5, 70, v=73)
  assert r.anchor_mph == 75                  # the same limit: his choice in the band stays


def test_a_bad_trust_frame_restarts_the_steady_time():
  r = rig()
  r.step(set_mph=48, events=[engaged(48)])
  r.p.owner, r.p.ours_why = OURS, "limit"
  drive(r, 6, 45, v=48)
  assert r.anchor_mph == 48
  drive(r, 2.5, 35, v=48)
  drive(r, 0.05, 35, v=48, trust=2)
  drive(r, 2.5, 35, v=48)
  assert r.anchor_mph == 48                  # 2.5 s clean since the bad frame: not yet
  drive(r, 0.6, 35, v=48)
  assert r.anchor_mph == 38


def test_a_lower_limit_after_an_exit_handover_recaps_our_anchor_in_any_band():
  r = rig()
  r.step(set_mph=75, events=[engaged(75)])
  # (the exit handover made 48 ours)
  r.p._set_anchor(48 * MPH, "handover")
  r.p.owner, r.p.ours_why = OURS, "handover"
  drive(r, 6, 45, v=46)
  assert r.anchor_mph == 48
  drive(r, 3.5, 25, v=46)
  assert r.anchor_mph == 28                  # 25 + 3, far out of the 45 road's band: ours, re-capped


def test_previews_hold_a_raise_only_on_the_road_they_were_seen_on_and_never_a_lowering():
  r = rig()
  r.step(set_mph=48, events=[engaged(48)])
  r.p.owner, r.p.ours_why = OURS, "limit"
  drive(r, 6, 45, v=48)
  drive(r, 6, 55, v=48, upcoming=40)        # a lower limit just ahead: the raise to 58 waits
  assert r.anchor_mph == 48
  drive(r, 6, 55, v=48, upcoming=40, chain=2)   # another road: the preview seen on the old one is stale
  assert r.anchor_mph == 58
  r2 = rig()
  r2.step(set_mph=58, events=[engaged(58)])
  r2.p.owner, r2.p.ours_why = OURS, "limit"
  drive(r2, 6, 55, v=58)
  drive(r2, 4, 45, v=58, upcoming=60)       # a higher preview never holds a lowering
  assert r2.anchor_mph == 48


def test_nothing_on_a_link_or_while_the_merge_or_exit_is_busy():
  r = rig()
  r.step(set_mph=48, events=[engaged(48)])
  r.p.owner, r.p.ours_why = OURS, "limit"
  r.step(v_mph=48, n=600, slc=slc(35), view=lambda t: view(t, link=True, cls=2))
  assert r.anchor_mph == 48
  r.step(v_mph=48, n=600, slc=slc(35), view=lambda t: view(t), merge_busy=True)
  assert r.anchor_mph == 48


def test_the_43_mph_fallback_only_unknown_soon_after_engaging_and_up():
  r = rig()
  r.step(v_mph=30, set_mph=25, events=[engaged(25)])
  drive(r, 2.5, None, v=36)
  assert r.anchor_mph == 25
  drive(r, 0.6, None, v=36)
  assert r.anchor_mph == 43 and r.p.ours_why == "fallback"
  # not after 120 s
  r = rig()
  r.step(v_mph=30, set_mph=25, events=[engaged(25)])
  drive(r, 121, None, v=30)
  drive(r, 4, None, v=40)
  assert r.anchor_mph == 25
  # never down, never his own (SET+) speed
  r = rig()
  r.step(v_mph=50, set_mph=50, events=[engaged(50)])
  drive(r, 4, None, v=50)
  assert r.anchor_mph == 50
  r = rig()
  r.step(v_mph=30, set_mph=25, events=[engaged(25)])
  r.step(v_mph=30, events=[driver("SET_PLUS", 28)])
  drive(r, 4, None, v=40)
  assert r.anchor_mph == 28
  # a known limit: no fallback (the first limit after he engaged in its band is followed: 25 + 3)
  r = rig()
  r.step(v_mph=30, set_mph=25, events=[engaged(25)])
  drive(r, 6, 25, v=40)
  assert r.anchor_mph == 28 and r.p.ours_why == "limit"


def test_test_mode_never_moves_the_anchor():
  cfg = C.Config(enabled=True, speed_limit="test")
  r = rig(cfg)
  drive(r, 6, 70, v=70)
  r.step(set_mph=73, events=[engaged(73)])
  out = drive(r, 4, 55, v=70)
  assert r.anchor_mph == 73 and abs(out.dry_anchor / MPH - 58) < 1e-6 and out.dry_anchor_kind == "limit"


def test_planner_rules_for_proposals():
  from openpilot.starpilot.car.mazda.speed.planner import Planner, PlanInputs
  p = Planner()

  def upd(contribs, events=()):
    return p.update(PlanInputs(v_ego=20.0, set_speed=p.anchor or 45 * MPH, cruise_enabled=True, main_on=True,
                               events=list(events), contributions=contribs))
  upd([], [engaged(45)])
  # a merge raises a street's speed; a handover only lowers; never in the frame of his own change
  upd([Contribution("merge", None, anchor=90 * MPH, kind="merge")], [driver("SET_PLUS", 46)])
  assert abs(p.anchor - 46 * MPH) < 1e-6
  upd([Contribution("merge", None, anchor=90 * MPH, kind="merge")])
  assert abs(p.anchor - 90 * MPH) < 1e-6 and p.ours_why == "merge"
  upd([Contribution("merge", None, anchor=78 * MPH, kind="merge")])
  assert abs(p.anchor - 78 * MPH) < 1e-6
  upd([Contribution("exit", None, anchor=80 * MPH, kind="handover")])
  assert abs(p.anchor - 78 * MPH) < 1e-6
  upd([Contribution("exit", None, anchor=48 * MPH, kind="handover")])
  assert abs(p.anchor - 48 * MPH) < 1e-6 and p.ours_why == "handover"
  # his own highway speed is not a street's: a merge does not raise it, and does not take it down
  upd([], [driver("SET_PLUS", 70)])
  upd([Contribution("merge", None, anchor=90 * MPH, kind="merge")])
  upd([Contribution("merge", None, anchor=60 * MPH, kind="merge")])
  assert abs(p.anchor - 70 * MPH) < 1e-6 and p.owner == HIS
  # with the cruise off nothing is taken
  p.update(PlanInputs(v_ego=20.0, set_speed=70 * MPH, cruise_enabled=False, main_on=True, events=[],
                      contributions=[Contribution("exit", None, anchor=40 * MPH, kind="handover")]))
  assert abs(p.anchor - 70 * MPH) < 1e-6
