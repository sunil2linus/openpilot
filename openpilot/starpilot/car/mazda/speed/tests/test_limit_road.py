"""The speed limit follow and the road (review of the limit change rule): a stale lower `matched` never turns a drop
into a raise (A1); the offset is a street's only (A2); after RES the snap happens only on a street, elsewhere his
speed stays his (A3); the steady time rides through 7.0's "not sure which road" fixes (A4); a highway drop and the
owner (A5)."""
import math
from types import SimpleNamespace as N

import pytest

from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed import limits as L
from openpilot.starpilot.car.mazda.speed.planner import ENGAGED, HIS, OURS
from openpilot.starpilot.car.mazda.speed.tests.rig import MPH, Rig, driver, engaged, view
from openpilot.starpilot.car.mazda.speed.tests.test_view_age import PHASE, at_hz

CFG = C.Config(enabled=True, speed_limit="on", limit_offset_mph=3)
STREET, HIGHWAY, UNKNOWN = 5, 1, 0


def slc(mph):
  return {"limit": None if mph is None else mph * MPH, "upcoming": 0.0, "t": 0.0}


def rig():
  sl = L.SpeedLimit(CFG)
  return Rig([sl]), sl


def drive(r, seconds, limit, v=50.0, cls=STREET, link=False, matched=True, **kw):
  return r.step(v_mph=v, n=int(round(seconds / r.dt)), slc=slc(limit),
                view=lambda t: view(t, cls=cls, link=link, matched=matched), **kw)


def resumed(mph):
  return {"kind": "resumed", "after": mph * MPH}


# ---------------- A1: a stale lower `matched` never turns a drop into a raise ----------------

def test_a_drop_after_his_own_speed_on_a_road_is_a_drop_from_that_road():
  # he cruised at 38 on a 35 street (matched 35), turned onto a 50 road and SET+ to 57 (his, out of the 50 road's
  # band: matched stays 35); within 90 s the road drops 50 -> 40: a drop from 50, re-taken on a street (40 + 3)
  r, sl = rig()
  drive(r, 6, 35, v=36)
  drive(r, 0.01, 35, v=36, set_mph=38, events=[engaged(38)])
  drive(r, 3, 35, v=38)
  assert r.anchor_mph == 38 and sl.matched[0] == 35
  drive(r, 2, 50, v=45)                                     # the 50 road, not steady yet: his SET+ to 57
  r.step(v_mph=50, set_mph=57, events=[driver("SET_PLUS", 57)])
  drive(r, 8, 50, v=56)
  assert r.anchor_mph == 57 and r.p.owner == HIS and sl.matched[0] == 35
  drive(r, 4.8, 40, v=56)
  assert r.anchor_mph == 57                                  # a lower limit re-takes his speed after 5 s
  drive(r, 0.5, 40, v=56)
  assert sl.prev == 50 and sl.steady == 40
  assert r.anchor_mph == 43 and r.p.owner == OURS and r.p.ours_why == "limit"


def test_matched_still_bridges_a_detour_when_the_change_is_a_raise():
  # (the case `matched` is for: his 48 on a 45 road, a 35 cross road while stopped, then 55: a raise from 45, so his
  # 48 is in the band of the limit it replaces and follows to 58)
  r, sl = rig()
  drive(r, 6, 45, v=45)
  drive(r, 0.01, 45, v=45, set_mph=45, events=[engaged(45)])
  drive(r, 2, 45, v=48)
  r.step(v_mph=48, set_mph=48, events=[driver("SET_PLUS", 48)])
  drive(r, 3.5, 35, v=10)                                    # the cross road, his speed kept (no street re-take yet)
  assert sl.steady == 35 and r.anchor_mph == 48
  drive(r, 5.5, 55, v=40)
  assert sl.prev == 45 and r.anchor_mph == 58


# ---------------- A2: the offset only on a street ----------------

@pytest.mark.parametrize("cls,link,want", ((STREET, False, 48), (7, False, 48), (HIGHWAY, False, 45), (3, False, 48),   # (a trunk road is a street here)
                                           (UNKNOWN, False, 45)))
def test_the_offset_is_a_streets_only(cls, link, want):
  r, sl = rig()
  drive(r, 0.01, None, v=40, cls=cls, set_mph=40, events=[engaged(40)])
  r.p.owner, r.p.ours_why = OURS, "limit"
  drive(r, 6, 45, v=40, cls=cls, link=link)
  assert r.anchor_mph == want, (cls, r.anchor_mph)


def test_the_street_is_the_last_known_class_class_0_frames_skipped():
  r, sl = rig()
  drive(r, 0.01, None, v=40, set_mph=40, events=[engaged(40)])
  r.p.owner, r.p.ours_why = OURS, "limit"
  drive(r, 2, 45, v=40, cls=STREET)
  drive(r, 4, 45, v=40, cls=UNKNOWN)                         # a stretch the matcher does not classify
  assert r.anchor_mph == 48
  drive(r, 1, 45, v=40, cls=HIGHWAY)                         # now a highway: the posted limit itself
  assert r.anchor_mph == 45
  assert sl.target(45, street=True) == 48 * MPH and sl.target(45, street=False) == 45 * MPH
  assert sl.band(45, street=False) == (45 * MPH, 50 * MPH)


# ---------------- A3: RES (his, or our auto-resume's) and match on engage ----------------

def _his_on(r, road_mph, his_mph, cls):
  drive(r, 6, road_mph, v=road_mph, cls=cls)
  drive(r, 0.01, road_mph, v=road_mph, cls=cls, set_mph=road_mph, events=[engaged(road_mph)])
  drive(r, 1, road_mph, v=road_mph, cls=cls)
  r.step(v_mph=road_mph, set_mph=his_mph, events=[driver("SET_PLUS", his_mph)], slc=slc(road_mph),
         view=lambda t: view(t, cls=cls))
  drive(r, 8, road_mph, v=his_mph, cls=cls)
  assert r.anchor_mph == his_mph and r.p.owner == HIS


def _res(r, road_mph, his_mph, cls, auto):
  drive(r, 1, road_mph, v=his_mph - 10, cls=cls, cruise=False)         # his brake: the cruise off
  if auto:                                                             # our auto-resume's RES (booked as his RES)
    drive(r, 0.01, road_mph, v=his_mph - 10, cls=cls, cruise=False,
          events=[{"kind": "driver_button", "button": "RES", "auto": True, "frame": 1}])
  drive(r, 0.01, road_mph, v=his_mph - 10, cls=cls, events=[resumed(his_mph)])
  drive(r, 2, road_mph, v=his_mph - 8, cls=cls)


@pytest.mark.parametrize("auto", (False, True))
def test_res_on_a_street_snaps_his_speed_in_the_band_to_the_limit_plus_offset(auto):
  r, _ = rig()
  _his_on(r, 45, 50, STREET)
  _res(r, 45, 50, STREET, auto)
  assert r.anchor_mph == 48 and r.p.owner == OURS            # (engaged in the 45 road's band: 45 + 3)
  # out of the band (his 52): his speed stays, as an engaged one
  r, _ = rig()
  _his_on(r, 45, 52, STREET)
  _res(r, 45, 52, STREET, auto)
  assert r.anchor_mph == 52 and r.p.owner == ENGAGED


@pytest.mark.parametrize("auto", (False, True))
@pytest.mark.parametrize("cls,link", ((HIGHWAY, False), (UNKNOWN, False), (2, True)))
def test_res_off_a_street_keeps_his_speed_his(auto, cls, link):
  # he SET+ to 75 on a 70 highway, taps the brake, RES: 75 stays his (was: 70)
  r, _ = rig()
  if link:
    _his_on(r, 70, 75, HIGHWAY)
    drive(r, 1, None, v=65, cls=cls, link=True, cruise=False)
    drive(r, 0.01, None, v=65, cls=cls, link=True, events=[resumed(75)])
    drive(r, 2, None, v=65, cls=cls, link=True)
  else:
    _his_on(r, 70, 75, cls)
    _res(r, 70, 75, cls, auto)
  assert r.anchor_mph == 75 and r.p.owner == HIS


def test_after_res_on_a_highway_only_a_new_limit_retakes_his_speed():
  r, _ = rig()
  _his_on(r, 70, 73, HIGHWAY)
  _res(r, 70, 73, HIGHWAY, False)
  drive(r, 10, 70, v=73, cls=HIGHWAY)
  assert r.anchor_mph == 73 and r.p.owner == HIS
  drive(r, 5.5, 75, v=73, cls=HIGHWAY)                       # a new limit, his 73 in the 70 road's band: re-taken
  assert r.anchor_mph == 75 and r.p.owner == OURS
  # a lower one never takes his own highway speed down
  r, _ = rig()
  _his_on(r, 70, 75, HIGHWAY)
  _res(r, 70, 75, HIGHWAY, True)
  drive(r, 12, 60, v=75, cls=HIGHWAY)
  assert r.anchor_mph == 75 and r.p.owner == HIS


def test_res_with_no_gps_position_is_still_an_engagement():
  # (no map view at all: nothing says the road; the RES engages as before, so the 43 mph fallback can still apply)
  r, _ = rig()
  _his_on(r, 70, 75, HIGHWAY)
  r.step(v_mph=60, n=100, slc=None, view=None, cruise=False)
  r.step(v_mph=60, slc=None, view=None, events=[resumed(75)])
  assert r.p.owner == ENGAGED


# ---------------- A4: riding through 7.0's "not sure which road" fixes (1 Hz) ----------------

def _pattern_drive(r, sl, pattern, values, cls=STREET, v=40.0):
  """One GPS fix (1 s) per letter: V a "current" fix (slcState valid with values[k]), P a "possible" fix (slcState
  not valid, mapdOut's limit values[k]), U "unknown" (nothing). Returns the steady limit after each fix."""
  fix0 = r.t

  def fix(t):
    return int(math.floor(t - fix0 - PHASE)) if t - fix0 >= PHASE else -1

  def slc_at(t):
    k = fix(t)
    return slc(values[k]) if 0 <= k < len(pattern) and pattern[k] == "V" else None

  def poss_at(t):
    k = fix(t)
    return {"limit": values[k] * MPH, "t": t} if 0 <= k < len(pattern) and pattern[k] == "P" else None
  out = []
  for _ in pattern:
    r.step(v_mph=v, n=100, slc=slc_at, slc_possible=poss_at, view=at_hz(lambda t: view(t, cls=cls), 1))
    out.append((sl.steady, round(sl.cand_t, 2)))
  return out


def _values(pattern, mph):
  return [mph] * len(pattern)


def test_two_unknown_fixes_hold_the_steady_time_three_restart_it():
  r, sl = rig()
  pat = "VUU" * 8
  got = _pattern_drive(r, sl, pat, _values(pat, 45))
  assert any(s == 45 for s, _ in got)                        # 1 s of growth per 3 fixes: steady after ~15 s
  assert got.index(next(x for x in got if x[0] == 45)) <= 16
  r, sl = rig()
  pat = "VVUUU" * 6
  got = _pattern_drive(r, sl, pat, _values(pat, 45))
  assert all(s is None for s, _ in got) and max(c for _, c in got) <= 2.0 + 1e-6


def test_possible_fixes_count_as_the_same_reading_never_as_a_new_one():
  # the real shape (Hall Rd): possible on the first fix of a new match, then current, unknown in between
  r, sl = rig()
  pat = "PVVUUPVVUUPVV"
  got = _pattern_drive(r, sl, pat, _values(pat, 45))
  assert got[0] == (None, 0.0)                               # a possible fix alone starts nothing
  assert any(s == 45 for s, _ in got)                        # V V . . P V V: 5 s of readings -> steady
  # a possible fix with ANOTHER value neither counts nor restarts (it is a gap fix): V V . p V V . p V
  r, sl = rig()
  pat = "VVUPVVUPVV"
  vals = [45, 45, 0, 40, 45, 45, 0, 40, 45, 45]
  got = _pattern_drive(r, sl, pat, vals)
  assert all(s in (None, 45) for s, _ in got) and sl.cand == 45
  assert got[8][1] > 4.5 and next(i for i, x in enumerate(got) if x[0] == 45) == 9   # only the V fixes counted


def test_a_different_value_restarts_and_a_real_change_still_happens():
  r, sl = rig()
  pat = "VVVVVV"
  _pattern_drive(r, sl, pat, _values(pat, 45))
  assert sl.steady == 45
  # a real change to 35 in the middle of dropouts: V U U V U U V -> steady after 3 s of readings (lower)
  pat = "VUUVUUVV"
  got = _pattern_drive(r, sl, pat, _values(pat, 35))
  assert sl.steady == 35 and got[-1][0] == 35
  # one fix of another value in a run restarts it
  r, sl = rig()
  pat = "VVVVUVVVVVVV"
  vals = [45, 45, 45, 45, 0, 50, 45, 45, 45, 45, 45, 45]
  got = _pattern_drive(r, sl, pat, vals)
  assert got[4][1] > 3.5                                     # held through the unknown fix ...
  assert got[5][1] < 1.0 and got[6][1] < 1.0                 # ... the 50 restarts it, and the 45 after it again
  assert got[10][0] is None and got[11][0] == 45             # steady 5 s after the 45 came back


def test_a_gap_still_restarts_on_bad_trust():
  r, sl = rig()
  drive(r, 3, 45)
  assert sl.cand_t > 2.9
  drive(r, 1.0, None)                                        # a gap: holds
  assert sl.cand == 45 and sl.cand_t > 2.9
  r.step(v_mph=40, n=5, slc=None, view=lambda t: view(t, trust=2, cls=STREET))
  assert sl.cand_t == 0.0                                    # bad trust in the gap: restarts as before


def test_map_possible_reads_only_a_v2_possible_match():
  road = lambda **kw: N(**{"sampleVersion": 2, "roadStatus": "matchedLimit", "waySelectionType": "possible",  # noqa: E731
                           "speedLimit": 45 * MPH, **kw})
  assert abs(L.map_possible(road()) - 45 * MPH) < 0.01
  for bad in ({"sampleVersion": 1}, {"roadStatus": "unknown"}, {"roadStatus": "matchedNoLimit"},
              {"waySelectionType": "current"}, {"speedLimit": 0.0}, {"speedLimit": float("nan")}):
    assert L.map_possible(road(**bad)) is None, bad
  assert L.map_possible(road(), valid=False) is None and L.map_possible(object()) is None


def test_sources_keep_the_possible_limit_through_the_private_submaster():
  from openpilot.starpilot.car.mazda.speed.contributors import Sources
  msgs = {"slcState": N(observationKind="unknown", source="none", speedLimit=0.0, upcomingSpeedLimit=0.0,
                        sourceReadings=[]),
          "mapdOut": N(sampleVersion=2, roadStatus="matchedLimit", waySelectionType="possible", speedLimit=45 * MPH,
                       previewSpeedLimit=0.0, previewDistance=0.0, previewWayId=0)}

  class SM(dict):
    updated = {"slcState": True, "mapdOut": True}
    valid = {"slcState": True, "mapdOut": True}

    def update(self, timeout):
      pass
  t = [0.0]
  src = Sources(want_model=False, want_view=False, sm=SM(msgs), read_view=lambda: None, clock=lambda: t[0],
                want_slc=True)
  ctx = src.context({"v_ego": 20.0})
  assert ctx["slc"]["limit"] is None and abs(ctx["slc_possible"]["limit"] - 45 * MPH) < 0.01
  SM.updated = {"slcState": False, "mapdOut": False}
  t[0] = 1.0
  assert src.context({"v_ego": 20.0})["slc_possible"] is None   # older than 0.5 s


# ---------------- A5: a highway drop and the owner ----------------

def test_a_highway_drop_lowers_ours_and_engaged_in_band_never_his():
  for owner in (OURS, ENGAGED):
    r, _ = rig()
    drive(r, 6, 70, v=70, cls=HIGHWAY)
    drive(r, 0.01, 70, v=70, cls=HIGHWAY, set_mph=72, events=[engaged(72)])
    if owner == OURS:
      r.p.owner, r.p.ours_why = OURS, "limit"
    drive(r, 0.5, 70, v=72, cls=HIGHWAY)
    assert r.anchor_mph == (70 if owner == OURS else 72) and r.p.owner == owner   # (ours: the limit itself at once)
    drive(r, L.LOWER_STEADY + 0.5, 60, v=72, cls=HIGHWAY)
    assert r.anchor_mph == (70 if owner == OURS else 72)     # off a street LOWER_STEADY is not enough (an overpass)
    drive(r, L.RETAKE_LOWER - L.LOWER_STEADY - 0.7, 60, v=72, cls=HIGHWAY)
    assert r.anchor_mph == (70 if owner == OURS else 72)
    drive(r, 0.4, 60, v=72, cls=HIGHWAY)
    assert r.anchor_mph == 60, owner                         # after RETAKE_LOWER, the posted limit itself
  r, _ = rig()
  drive(r, 6, 70, v=70, cls=HIGHWAY)
  drive(r, 0.01, 70, v=70, cls=HIGHWAY, set_mph=70, events=[engaged(70)])
  r.step(v_mph=70, set_mph=72, events=[driver("SET_PLUS", 72)], slc=slc(70), view=lambda t: view(t, cls=HIGHWAY))
  drive(r, 20, 60, v=72, cls=HIGHWAY)
  assert r.anchor_mph == 72 and r.p.owner == HIS


def test_a_big_highway_drop_of_ours_waits_the_long_time_and_a_street_drop_stays_quick():
  r, _ = rig()
  drive(r, 6, 70, v=70, cls=HIGHWAY)
  drive(r, 0.01, 70, v=70, cls=HIGHWAY, set_mph=70, events=[engaged(70)])
  r.p.owner, r.p.ours_why = OURS, "limit"
  drive(r, L.RETAKE_LOWER + 1.0, 45, v=70, cls=HIGHWAY)     # 70 -> 45: more than BIG_DROP
  assert r.anchor_mph == 70
  drive(r, L.RETAKE_LOWER_BIG - L.RETAKE_LOWER - 0.8, 45, v=70, cls=HIGHWAY)
  assert r.anchor_mph == 45
  r, _ = rig()                                              # a street (a trunk road too): LOWER_STEADY as before
  for cls in (STREET, 3):
    r, _ = rig()
    drive(r, 6, 45, v=45, cls=cls)
    drive(r, 0.01, 45, v=45, cls=cls, set_mph=45, events=[engaged(45)])
    r.p.owner, r.p.ours_why = OURS, "limit"
    drive(r, 0.5, 45, v=45, cls=cls)
    assert r.anchor_mph == 48
    drive(r, L.LOWER_STEADY + 0.3, 35, v=45, cls=cls)
    assert r.anchor_mph == 38, cls
