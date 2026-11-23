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
