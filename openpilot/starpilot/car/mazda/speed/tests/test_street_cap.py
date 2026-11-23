"""The street cap (round 2, item 3b): a highway's speed carried onto a street is capped at the street's limit + offset
(43 with the limit unknown), paced, ours; his SET+ / SET- overrides; only after a motorway or a link from a highway;
never above 55 mph, never on a trunk road whose limit is unknown; the planner's rules for the "street" kind."""
from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed.merge import Merge
from openpilot.starpilot.car.mazda.speed.planner import Contribution, Planner, PlanInputs, OURS, HIS
from openpilot.starpilot.car.mazda.speed.street_cap import StreetCap
from openpilot.starpilot.car.mazda.speed.tests.rig import MPH, Rig, driver, engaged, view

CFG = C.Config(enabled=True, speed_limit="on", exits="on", limit_offset_mph=3)


def road(cls, link=False, trust=0, chain=1):
  return lambda t: view(t, cls=cls, link=link, trust=trust, chain=chain)


def slc(limit):
  return {"limit": None if limit is None else limit * MPH, "upcoming": 0.0, "t": 0.0}


def exit_to(r, cls=7, limit=45, v_street=40.0, trust=0, seconds=4.0, link_s=5.0, start=70):
  """70 on the motorway, an exit link (5 s at 45 mph), then the street."""
  r.step(v_mph=start, set_mph=start, events=[engaged(start)], view=road(1), slc=slc(70))
  r.step(v_mph=start, n=300, view=road(1), slc=slc(70))
  r.step(v_mph=45, n=int(link_s * 100), view=road(2, link=True, chain=2), slc=slc(None))
  return r.step(v_mph=v_street, n=int(seconds * 100), view=road(cls, trust=trust, chain=3), slc=slc(limit))


def rig(cfg=CFG):
  return Rig([StreetCap(cfg, presses=True)])


def test_an_exit_onto_a_45_street_caps_his_highway_70_at_48():
  r = rig()
  exit_to(r)
  assert r.anchor_mph == 48 and r.p.owner == OURS and r.p.ours_why == "street"


def test_the_limit_unknown_is_the_old_forks_43():
  r = rig()
  exit_to(r, limit=None)
  assert r.anchor_mph == 43


def test_paced_never_more_than_lower_gap_under_the_car():
  r = rig()
  exit_to(r, limit=None, v_street=54.0, seconds=2.5)
  first = r.anchor_mph
  assert 50 <= first <= 51                  # 54 + 2 dash - 5.4: the cruise slows gently, not 54 -> 43 at once
  for v in range(54, 40, -1):
    r.step(v_mph=v, n=50, view=road(7, chain=3), slc=slc(None))
  assert r.anchor_mph == 43


def test_his_set_plus_after_the_cap_is_his_and_is_not_capped_again():
  r = rig()
  exit_to(r)
  r.step(v_mph=45, events=[driver("SET_PLUS", 55)], view=road(7, chain=3), slc=slc(45))
  r.step(v_mph=45, n=500, view=road(7, chain=3), slc=slc(45))
  assert r.anchor_mph == 55 and r.p.owner == HIS


def test_after_his_brake_and_res_it_goes_on():
  r = rig()
  exit_to(r)
  r.step(v_mph=45, events=[driver("SET_PLUS", 60)], view=road(7, chain=3), slc=slc(45))    # his: done
  assert r.anchor_mph == 60
  r.step(v_mph=30, n=100, cruise=False, view=road(7, chain=3), slc=slc(45))                 # his brake
  r.step(v_mph=30, n=200, view=road(7, chain=3), slc=slc(45))                              # RES: it goes on
  assert r.anchor_mph == 48


def test_no_cap_without_a_highway_before_or_off_a_trusted_street():
  # street -> link -> street: a slip lane between two streets is no exit
  r = rig()
  r.step(v_mph=50, set_mph=60, events=[engaged(60)], view=road(5), slc=slc(45))
  r.step(v_mph=50, n=300, view=road(5), slc=slc(45))
  r.step(v_mph=40, n=500, view=road(6, link=True, chain=2), slc=slc(None))
  r.step(v_mph=40, n=500, view=road(7, chain=3), slc=slc(45))
  assert r.anchor_mph == 60
  # GPS trust 1 on the street: nothing
  r = rig()
  exit_to(r, trust=1)
  assert r.anchor_mph == 70


def test_a_trunk_road_counts_only_with_a_known_limit_under_55():
  r = rig()
  exit_to(r, cls=3, limit=None)
  assert r.anchor_mph == 70                 # a trunk with no limit may be a highway: no cap
  r = rig()
  exit_to(r, cls=3, limit=45)
  assert r.anchor_mph == 48                 # Hall Rd with its 45 is a street


def test_never_at_highway_speed_nor_for_a_limit_above_55():
  r = rig()
  exit_to(r, v_street=58.0)
  assert r.anchor_mph == 70                 # a wrong street match beside a highway is at highway speed
  r = rig()
  exit_to(r, limit=60)
  assert r.anchor_mph == 70


def test_an_anchor_already_under_the_cap_is_left_alone():
  r = rig()
  exit_to(r, start=45)
  assert r.anchor_mph == 45 and r.p.owner != OURS


def test_test_mode_never_moves_the_anchor():
  cfg = C.Config(enabled=True, speed_limit="test", limit_offset_mph=3)
  r = Rig([StreetCap(cfg, presses=False)])
  exit_to(r)
  assert r.anchor_mph == 70 and r.out.dry_anchor_kind == "street"


def test_the_window_closes_back_on_the_highway():
  r = rig()
  exit_to(r, seconds=1.0)                   # not steady yet
  r.step(v_mph=60, n=300, view=road(1), slc=slc(70))
  r.step(v_mph=40, n=400, view=road(7, chain=3), slc=slc(45))
  assert r.anchor_mph == 48                 # (the motorway again, then a street: an exit again)


def test_the_merge_drops_a_pending_raise_while_the_cap_is_open():
  cfg = C.Config(enabled=True, speed_limit="on", merge="on", limit_offset_mph=3)
  m = Merge(cfg, presses=True)
  r = Rig([StreetCap(cfg, presses=True), m])
  exit_to(r)
  m.pending = (70 * MPH, r.t + 2.0)
  r.step(v_mph=45, n=50, view=road(7, chain=3), slc=slc(45))
  assert m.pending is None and r.anchor_mph == 48


def test_planner_street_is_down_only_and_a_limit_raise_needs_a_new_limit():
  p = Planner()

  def step(contribs, events=()):
    return p.update(PlanInputs(v_ego=40 * MPH, set_speed=70 * MPH, cruise_enabled=True, main_on=True,
                               events=list(events), contributions=contribs))
  step([], [engaged(70)])
  step([Contribution("street", None, anchor=80 * MPH, kind="street")])
  assert round(p.anchor / MPH) == 70                       # never up
  step([Contribution("street", None, anchor=48 * MPH, kind="street"),
        Contribution("speed_limit", None, anchor=48 * MPH, kind="limit", seq=4)])
  assert round(p.anchor / MPH) == 48 and p.owner == OURS and p.ours_why == "street"
  step([Contribution("speed_limit", None, anchor=73 * MPH, kind="limit", seq=4)])
  assert round(p.anchor / MPH) == 48                       # the highway's old limit kept through a gap: no raise
  step([Contribution("speed_limit", None, anchor=40 * MPH, kind="limit", seq=4)])
  assert round(p.anchor / MPH) == 40                       # a lower one: taken
  step([Contribution("speed_limit", None, anchor=53 * MPH, kind="limit", seq=5)])
  assert round(p.anchor / MPH) == 53                       # a limit steady after it: taken


def resumed():
  return {"kind": "resumed", "after": None}


def res_on(r, cls=9, limit=None, v=20.0, anchor=55, trust=0, link=False, before_limit=None):
  """His set `anchor` (engaged), his brake, then a RES at v mph on this road."""
  r.step(v_mph=40, set_mph=anchor, events=[engaged(anchor)], view=road(cls, link=link, trust=trust), slc=slc(before_limit))
  r.step(v_mph=40, n=300, view=road(cls, link=link, trust=trust), slc=slc(before_limit))
  r.step(v_mph=v, n=200, cruise=False, view=road(cls, link=link, trust=trust), slc=slc(limit))
  r.step(v_mph=v, events=[dict(resumed(), after=anchor * MPH)], view=road(cls, link=link, trust=trust), slc=slc(limit))
  return r.step(v_mph=v, n=100, view=road(cls, link=link, trust=trust), slc=slc(limit))


def test_round2_res_on_a_25_street_caps_his_55_at_28():
  r = rig()
  res_on(r, limit=25)
  assert r.anchor_mph == 28 and r.p.owner == OURS and r.p.ours_why == "street"


def test_round2_res_with_the_limit_unknown_caps_at_the_speed_plus_20_at_least_45():
  r = rig()
  res_on(r, v=20)                                      # (r3 rt 402: 55 back at 21 mph, the limit unknown)
  assert r.anchor_mph == 45
  r = rig()
  res_on(r, cls=3, v=10, anchor=53)                    # (r2 rt 1030: Hall Rd, 53 at 10 mph)
  assert r.anchor_mph == 45
  r = rig()
  res_on(r, v=40, anchor=55)
  assert r.anchor_mph == 55                            # 40 + 20 is above his 55: his speed stays


def test_round2_res_uses_a_limit_read_on_this_road_just_before():
  r = rig()
  r.step(v_mph=40, set_mph=55, events=[engaged(55)], view=road(9), slc=slc(35))
  r.step(v_mph=40, n=300, view=road(9), slc=slc(35))
  r.step(v_mph=10, n=300, cruise=False, view=road(9), slc=slc(None))       # 7.0 says "unknown" at the stop
  r.step(v_mph=10, events=[dict(resumed(), after=55 * MPH)], view=road(9), slc=slc(None))
  r.step(v_mph=10, n=100, view=road(9), slc=slc(None))
  assert r.anchor_mph == 38


def test_round2_res_on_a_motorway_a_link_or_an_untrusted_road_keeps_his_speed():
  for kw in ({"cls": 1, "limit": 55}, {"cls": 2, "link": True, "limit": 25}, {"cls": 9, "trust": 1, "limit": 25}):
    r = rig()
    res_on(r, anchor=75, v=30, **kw)
    assert r.anchor_mph == 75, kw


def test_round2_his_set_plus_after_the_res_cap_is_his():
  r = rig()
  res_on(r, limit=25)
  r.step(v_mph=25, events=[driver("SET_PLUS", 40)], view=road(9), slc=slc(25))
  r.step(v_mph=25, n=300, view=road(9), slc=slc(25))
  assert r.anchor_mph == 40 and r.p.owner == HIS


# ---- round 3: the set stuck at 45 after a red light on a 45-55 mph road ----
HALL_RD = [0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0]   # ~18 % valid 1 s fixes


def both(cfg=CFG):
  from openpilot.starpilot.car.mazda.speed.limits import SpeedLimit
  return Rig([StreetCap(cfg, presses=True), SpeedLimit(cfg, presses=True)], dt=0.05)


def red_light(r, limit=55, set_mph=58, stop_s=20.0, cls=3, creep_m=0.0, pattern=HALL_RD, after_s=60, creep_mph=5):
  """(the reviewer's sequence) his set on a `limit` road read clean for a minute, a stop at a red light with 7.0's
  limit gone (slc None, his brake: cruise off), a RES at 15 mph, then the limit back in Hall Rd's dropout pattern (one
  1 s fix in ~5 valid). creep_m: metres crept in the stop (stop and go) before the RES."""
  rd = road(cls)
  r.step(v_mph=set_mph - 3, set_mph=set_mph, events=[engaged(set_mph)], view=rd, slc=slc(limit))
  r.step(v_mph=set_mph - 3, n=1200, view=rd, slc=slc(limit))
  r.step(v_mph=0, n=int(stop_s * 20), cruise=False, view=rd, slc=slc(None))
  if creep_m:
    r.step(v_mph=creep_mph, n=int(creep_m / (creep_mph * MPH) * 20), cruise=False, view=rd, slc=slc(None))
  r.step(v_mph=15, events=[dict(resumed(), after=set_mph * MPH)], view=rd, slc=slc(None))
  at_res = r.anchor_mph
  for k in range(after_s):
    r.step(v_mph=min(15 + 2 * k, set_mph - 3), n=20, view=rd, slc=slc(limit if pattern[k % len(pattern)] else None))
  return at_res


def test_round3_a_red_light_of_any_length_keeps_the_roads_limit_for_the_res():
  for stop_s in (20.0, 60.0, 300.0):                 # (SpeedLimit: steady kept <= 30 s, `known` 30-90 s, then gone)
    r = both()
    at_res = red_light(r, stop_s=stop_s)
    assert at_res == 58 and r.anchor_mph == 58, stop_s         # (was: 45 at the RES, 45 for good)
  r = both()
  at_res = red_light(r, limit=45, set_mph=53, stop_s=40.0, cls=9)
  assert at_res == 48 and r.anchor_mph == 48                   # a 45 street, his 53: limit + offset, not 45


def test_round3_the_remembered_limit_ages_by_distance_not_time():
  r = both()
  at_res = red_light(r, stop_s=5.0, creep_m=500.0, pattern=[0])   # 500 m of stop and go, no reading: not known now
  assert at_res == 45


def test_round3_a_limit_read_now_on_the_street_raises_our_street_cap_with_the_same_seq():
  p = Planner()

  def step(contribs, events=()):
    return p.update(PlanInputs(v_ego=15 * MPH, set_speed=58 * MPH, cruise_enabled=True, main_on=True,
                               events=list(events), contributions=contribs))
  step([Contribution("speed_limit", None, anchor=58 * MPH, kind="limit", seq=1)], [engaged(58)])
  step([Contribution("street", None, anchor=45 * MPH, kind="street"),
        Contribution("speed_limit", None, anchor=58 * MPH, kind="limit", seq=1)])
  assert round(p.anchor / MPH) == 45 and p.ours_why == "street" and p.ours_seq == 1
  step([Contribution("speed_limit", None, anchor=58 * MPH, kind="limit", seq=1)])
  assert round(p.anchor / MPH) == 45                       # held in a gap, not read now: no raise (round 2)
  step([Contribution("speed_limit", None, anchor=58 * MPH, kind="limit", seq=1, live=True)])
  assert round(p.anchor / MPH) == 58 and p.ours_why == "limit"   # read now on this street: raised


def test_round3_after_the_res_cap_the_roads_55_read_again_takes_the_set_back_to_58():
  # 30-90 s in stop and go (500 m at 15 mph, no reading): the RES caps at 45; SpeedLimit's steady 55 is forgotten,
  # its `known` 55 comes back as the same limit (no new seq); read again on this road it raises the cap to 58
  r = both()
  at_res = red_light(r, stop_s=5.0, creep_m=500.0, creep_mph=15, pattern=[0, 0] + [1] * 40, after_s=40)
  assert at_res == 45 and r.cs[1].seq == 1 and r.anchor_mph == 58     # (was: 45 for good)
  r = both()
  at_res = red_light(r, stop_s=5.0, creep_m=500.0, creep_mph=15, pattern=[0, 0] + HALL_RD * 3, after_s=90)
  assert at_res == 45 and r.anchor_mph == 45               # the dropout pattern alone never makes the limit steady


def test_round3_the_exit_cap_43_goes_to_the_streets_limit_once_it_is_read():
  # the street's 45 became steady on the exit link (its seq is older than our cap), then 2 s without a limit on the
  # street: the cap is 43 (limit unknown); then the 45 is read again: 48
  from openpilot.starpilot.car.mazda.speed.limits import SpeedLimit
  r = Rig([StreetCap(CFG, presses=True), SpeedLimit(CFG, presses=True)])
  r.step(v_mph=70, set_mph=70, events=[engaged(70)], view=road(1), slc=slc(70))
  r.step(v_mph=70, n=600, view=road(1), slc=slc(70))
  r.step(v_mph=45, n=1000, view=road(2, link=True, chain=2), slc=slc(45))
  r.step(v_mph=40, n=600, view=road(7, chain=3), slc=slc(None))
  assert r.anchor_mph == 43
  r.step(v_mph=40, n=500, view=road(7, chain=3), slc=slc(45))
  assert r.anchor_mph == 48                                # (was: 43 for good)


def test_round3_the_highways_70_on_the_street_never_lifts_the_cap():
  from openpilot.starpilot.car.mazda.speed.limits import SpeedLimit
  # (round 2) the motorway's steady 70 is held through the gap of readings on the street; mapd still reads it now
  # and then on the street: the same seq, and a 70 is no street's limit (never live): the cap stays
  r = Rig([StreetCap(CFG, presses=True), SpeedLimit(CFG, presses=True)])
  r.step(v_mph=70, set_mph=70, events=[engaged(70)], view=road(1), slc=slc(70))
  r.step(v_mph=70, n=600, view=road(1), slc=slc(70))
  r.step(v_mph=45, n=500, view=road(2, link=True, chain=2), slc=slc(None))
  r.step(v_mph=40, n=600, view=road(7, chain=3), slc=slc(None))
  assert r.anchor_mph == 43 and r.cs[1].steady == 70
  for _ in range(10):
    r.step(v_mph=40, n=100, view=road(7, chain=3), slc=slc(70))
    r.step(v_mph=40, n=200, view=road(7, chain=3), slc=slc(None))
  assert r.anchor_mph == 43


def test_round3_the_notice_number_is_the_winning_cap_when_both_start_on_one_frame():
  sc = StreetCap(CFG, presses=True)
  sc.since, sc.lim, sc.lim_t = 0.0, 0, 5.0                 # the exit window open, no limit for 5 s: its cap is 43
  sc.seen = [25, 1, 50.0]                                  # 25 read on this road 50 m back: the RES cap is 28
  ctx = {"now": 1.0, "v_ego": 20 * MPH, "cruise_enabled": True, "anchor": 55 * MPH, "owner": "engaged",
         "act_events": [dict(resumed(), after=55 * MPH)], "view": view(1.0, cls=9), "slc": None, "contribs": {}}
  sc.t = 0.99
  out = sc(ctx)
  assert sc.res is not None and sc.on                      # both started on this frame
  assert sc.cap_to == 28 and round(out.anchor / MPH) <= 45   # the RES cap's 28 (was: 43, the exit cap's)
