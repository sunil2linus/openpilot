"""The limit change rule (planner.py, limits.py; the old fork's speed_limit_follow rules 1 and 9) and the engaging press
(actuator.py): a NEW limit re-takes his own speed (his SET+ / SET-, or his engaged speed), his next press owns it
again until the next limit change; never against his overrule of a slowdown, never a raise over a slowdown under way;
the press that engages the cruise (SET+, SET-, RES) is an engagement, not his change."""
import pytest

from openpilot.starpilot.car.mazda.speed import actuator as A
from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed import limits as L
from openpilot.starpilot.car.mazda.speed.planner import ENGAGED, HIS, OURS, Contribution, Planner, PlanInputs
from openpilot.starpilot.car.mazda.speed.tests.mrcc_sim import Mrcc, events, run
from openpilot.starpilot.car.mazda.speed.tests.rig import MPH, Rig, driver, engaged, view
from openpilot.starpilot.car.mazda.speed.tests.test_view_age import RATES, at_hz

CFG = C.Config(enabled=True, speed_limit="on", limit_offset_mph=3)
STREET, HIGHWAY = 5, 1


def slc(mph):
  return {"limit": None if mph is None else mph * MPH, "upcoming": 0.0, "t": 0.0}


class Slowdown:
  """A stand-in for a curve / stop slowdown: a target (mph true) while `on`, with an episode."""

  def __init__(self):
    self.mph, self.episode = None, "c1"

  def __call__(self, ctx):
    return Contribution("curve", None if self.mph is None else self.mph * MPH, episode=self.episode)


def rig(*extra):
  return Rig([*extra, L.SpeedLimit(CFG)])


def drive(r, seconds, limit, v=50.0, cls=STREET, hz=None, **kw):
  mk = (lambda t: view(t, cls=cls))  # noqa: E731
  return r.step(v_mph=v, n=int(round(seconds / r.dt)), slc=slc(limit), view=at_hz(mk, hz) if hz else mk, **kw)


def his_50_on_a_45_road(r, cls=STREET, hz=None):
  drive(r, 6, 45, cls=cls, hz=hz)
  r.step(v_mph=45, set_mph=45, events=[engaged(45)])
  drive(r, 1, 45, cls=cls, hz=hz)
  assert r.anchor_mph == 48 and r.p.owner == OURS            # (match on engage: 45 + 3)
  r.step(v_mph=48, set_mph=50, events=[driver("SET_PLUS", 50)])
  assert r.anchor_mph == 50 and r.p.owner == HIS
  drive(r, 8, 45, cls=cls, hz=hz)
  assert r.anchor_mph == 50                                  # the same limit: his speed stays


@pytest.mark.parametrize("cls", (STREET, HIGHWAY))
def test_a_new_higher_limit_retakes_his_speed_in_the_band_of_the_old_one(cls):
  r = rig()
  his_50_on_a_45_road(r, cls)
  drive(r, 4.5, 55, cls=cls)
  assert r.anchor_mph == 50                                  # a higher limit's steady time first (5 s)
  drive(r, 0.7, 55, cls=cls)
  assert r.anchor_mph == 58 and r.p.owner == OURS and r.p.ours_why == "limit"


def test_a_change_after_a_long_gap_of_readings_is_still_a_change_from_the_old_limit():
  # (drive 0000000c, the 45 -> 55 road: 7.0's map provider said "ambiguous road" for ~45 s in between)
  r = rig()
  his_50_on_a_45_road(r, cls=HIGHWAY)
  drive(r, 45, None, cls=HIGHWAY)
  drive(r, 5.2, 55, cls=HIGHWAY)
  assert r.anchor_mph == 58 and r.p.owner == OURS
  # ... but the same limit back after the gap is no change: his 46 stays
  r = rig()
  his_50_on_a_45_road(r)
  r.step(v_mph=48, set_mph=46, events=[driver("SET_MINUS", 46)])
  drive(r, 45, None)
  drive(r, 8, 45)
  assert r.anchor_mph == 46 and r.p.owner == HIS
  # ... and after more than MATCH_KEEP without it the old limit is unknown: a first limit, its own band
  r = rig()
  his_50_on_a_45_road(r)
  drive(r, L.MATCH_KEEP + 5, None)
  drive(r, 6, 55)
  assert r.anchor_mph == 50


def test_his_press_after_it_owns_the_speed_again_until_the_next_limit_change():
  r = rig()
  his_50_on_a_45_road(r)
  drive(r, 6, 55)
  assert r.anchor_mph == 58
  r.step(v_mph=56, set_mph=60, events=[driver("SET_PLUS", 60)])
  drive(r, 25, 55)
  assert r.anchor_mph == 60 and r.p.owner == HIS             # the same 55: his 60 stays
  drive(r, 6, 60)
  assert r.anchor_mph == 63 and r.p.owner == OURS            # the next change (his 60 in the 55 road's band 55-60)
  r.step(v_mph=60, set_mph=61, events=[driver("SET_MINUS", 61)])
  drive(r, 10, 60)
  assert r.anchor_mph == 61 and r.p.owner == HIS


def test_his_speed_out_of_the_old_band_is_not_raised():
  r = rig()
  drive(r, 6, 45)
  r.step(v_mph=45, set_mph=45, events=[engaged(45)])
  r.step(v_mph=48, set_mph=62, events=[driver("SET_PLUS", 62)])
  drive(r, 8, 55)
  assert r.anchor_mph == 62 and r.p.owner == HIS             # 62 is not the 45 road's speed (45-50)


def test_a_new_lower_limit_retakes_his_speed_on_a_street_any_band_after_the_old_steady_time():
  r = rig()
  his_50_on_a_45_road(r)
  drive(r, 4.8, 30)
  assert r.anchor_mph == 50                                  # a lower one re-takes his speed after 5 s (old rule 9)
  drive(r, 0.4, 30)
  assert r.anchor_mph == 33 and r.p.owner == OURS
  # a drop of more than 20 mph: 8 s
  r = rig()
  drive(r, 6, 55)
  r.step(v_mph=55, set_mph=55, events=[engaged(55)])
  r.step(v_mph=55, set_mph=40, events=[driver("SET_MINUS", 40)])
  drive(r, 7.8, 30)
  assert r.anchor_mph == 40
  drive(r, 0.4, 30)
  assert r.anchor_mph == 33


def test_a_lower_limit_never_retakes_his_speed_on_a_highway_or_an_unknown_road():
  for cls in (HIGHWAY, 0):
    r = rig()
    r.step(v_mph=70, set_mph=75, events=[engaged(75)])
    r.step(v_mph=72, set_mph=78, events=[driver("SET_PLUS", 78)])
    drive(r, 8, 70, v=76, cls=cls)
    drive(r, 12, 55, v=76, cls=cls)
    assert r.anchor_mph == 78 and r.p.owner == HIS, cls


def test_his_press_after_the_limit_became_steady_is_his_answer_to_it():
  r = rig()
  his_50_on_a_45_road(r)
  drive(r, 3.2, 30)                                          # steady (3 s), not yet re-taken (5 s)
  r.step(v_mph=48, set_mph=45, events=[driver("SET_MINUS", 45)])
  drive(r, 10, 30)
  assert r.anchor_mph == 45 and r.p.owner == HIS


def test_a_retake_waits_for_a_slowdown_under_way_and_ends_after_the_window():
  sd = Slowdown()
  r = rig(sd)
  his_50_on_a_45_road(r)
  sd.mph = 30                                                # a curve wants 30
  drive(r, 8, 55)
  assert r.anchor_mph == 50 and r.out.reason == "curve"      # no raise over the slowdown
  sd.mph = None
  drive(r, 0.1, 55)
  assert r.anchor_mph == 58                                  # once it is over (still fresh)
  # a slowdown longer than the window: the raise is gone, his speed stays
  sd = Slowdown()
  r = rig(sd)
  his_50_on_a_45_road(r)
  sd.mph = 30
  drive(r, L.RETAKE_WINDOW + 6, 55)
  sd.mph = None
  drive(r, 2, 55)
  assert r.anchor_mph == 50 and r.p.owner == HIS


def test_never_against_his_overrule_of_a_slowdown():
  sd = Slowdown()
  r = rig(sd)
  his_50_on_a_45_road(r)
  sd.mph = 40
  drive(r, 1, 45)
  assert r.out.reason == "curve"
  # his SET+ in the curve: the curve is his (blocked for its episode); a new lower limit waits too
  r.step(v_mph=40, set_mph=44, events=[driver("SET_PLUS", 44)])
  drive(r, 9, 30, v=42)
  assert r.p.blocked and r.anchor_mph == 50 and r.out.command / MPH >= 44 - 1e-6
  sd.mph = None                                              # the curve is over: the 30 (still fresh) is taken
  drive(r, 0.5, 30, v=42)
  assert r.anchor_mph == 33


def test_engaged_out_of_the_band_follows_a_new_street_limit_down_but_not_a_highway_one():
  r = rig()
  drive(r, 6, 45)
  r.step(v_mph=55, set_mph=58, events=[engaged(58)])         # 58 on a 45 street: out of the band, his
  drive(r, 2, 45)
  assert r.anchor_mph == 58 and r.p.owner == ENGAGED
  drive(r, 6, 35)
  assert r.anchor_mph == 38
  r = rig()
  r.step(v_mph=78, set_mph=80, events=[engaged(80)])
  drive(r, 8, 70, v=78, cls=HIGHWAY)
  drive(r, 12, 55, v=78, cls=HIGHWAY)
  assert r.anchor_mph == 80


@pytest.mark.parametrize("hz", RATES)
def test_the_limit_change_rule_at_the_gps_rate(hz):
  r = rig()
  his_50_on_a_45_road(r, hz=hz)
  drive(r, 6.5, 55, hz=hz)
  assert r.anchor_mph == 58
  r.step(v_mph=56, set_mph=56, events=[driver("SET_MINUS", 56)])
  drive(r, 10, 55, hz=hz)
  assert r.anchor_mph == 56
  drive(r, 6.5, 35, hz=hz)
  assert r.anchor_mph == 38


def test_speed_limit_off_nothing_but_his_own_speed():
  sd = Slowdown()
  r = Rig([sd])
  r.step(v_mph=45, set_mph=45, events=[engaged(45)])
  r.step(v_mph=45, set_mph=50, events=[driver("SET_PLUS", 50)])
  r.step(v_mph=50, n=600)
  assert r.anchor_mph == 50 and r.p.owner == HIS and abs(r.out.command / MPH - 50) < 1e-6
  r.step(v_mph=40, n=10, cruise=False)
  r.step(v_mph=40, set_mph=50, events=[{"kind": "resumed", "after": 50 * MPH}])
  r.step(v_mph=45, n=600)
  assert r.anchor_mph == 50 and r.p.owner == ENGAGED and abs(r.out.command / MPH - 50) < 1e-6
  assert r.p.own_seq == 0


# ---------------- the engaging press ----------------

def _planner_step(p, set_mph, events=(), cruise=True):
  return p.update(PlanInputs(v_ego=set_mph * MPH, set_speed=set_mph * MPH, cruise_enabled=cruise, main_on=True,
                             events=list(events), contributions=[]))


def test_resumed_makes_his_speed_an_engaged_one_and_keeps_ours():
  p = Planner()
  _planner_step(p, 45, [engaged(45)])
  _planner_step(p, 50, [driver("SET_PLUS", 50)])
  assert p.owner == HIS
  _planner_step(p, 50, cruise=False)
  _planner_step(p, 50, [{"kind": "resumed", "after": 50 * MPH}])
  assert p.owner == ENGAGED and abs(p.anchor - 50 * MPH) < 1e-6
  assert ("resumed (was his)", 50 * MPH, 50 * MPH) in p.changes
  p.owner, p.ours_why = OURS, "handover"
  _planner_step(p, 50, cruise=False)
  _planner_step(p, 50, [{"kind": "resumed", "after": 50 * MPH}])
  assert p.owner == OURS and p.ours_why == "handover"


def _engage(button, shown_after, new_mph=None):
  """Cruise off at 45 (main on), his `button` from standby; the car shows the cruise on and (new_mph) its new set
  speed `shown_after` frames later."""
  act, car = A.Actuator(), Mrcc(45, v_mph=40)
  run(act, car, 100)
  outs = run(act, car, 100, cruise=False)
  key = {"SET_PLUS": "set_plus", "SET_MINUS": "set_minus", "RES": "resume"}[button]
  outs += run(act, car, 8, cruise=False, **{key: True})       # his press (80 ms) engages the cruise
  on_frame = car.frame + 1
  for k in range(80):
    if new_mph is not None and k == shown_after:
      car.set = new_mph * MPH
    outs += run(act, car, 1)
  return outs, on_frame


@pytest.mark.parametrize("button", ("SET_PLUS", "SET_MINUS"))
def test_set_plus_or_minus_from_standby_is_an_engagement(button):
  for shown in (0, 3, 30):
    outs, _ = _engage(button, shown, new_mph=40)
    eng = events(outs, "engaged")
    assert eng and not events(outs, "driver"), (button, shown)
    assert abs(eng[-1]["after"] - 40 * MPH) < 1e-6
  # the planner: an engaged anchor at the new speed, not his
  outs, _ = _engage(button, 3, new_mph=40)
  p = Planner()
  _planner_step(p, 45, [engaged(45)])
  _planner_step(p, 47, [driver("SET_PLUS", 47)])
  _planner_step(p, 47, cruise=False)
  for e in events(outs):
    if e["kind"] in ("engaged", "resumed", "driver"):
      _planner_step(p, 40, [e])
  assert p.owner == ENGAGED and abs(p.anchor - 40 * MPH) < 1e-6
  # a set change later than ENGAGE_SETTLE without a press: unexplained, his (as before)
  outs, _ = _engage(button, 70, new_mph=40)
  assert len(events(outs, "driver")) == 1 and not any(e.get("settle") for e in events(outs, "engaged"))


def test_his_new_press_right_after_the_engagement_is_his():
  act, car = A.Actuator(), Mrcc(45, v_mph=40)
  run(act, car, 100)
  run(act, car, 100, cruise=False)
  run(act, car, 8, cruise=False, set_plus=True)
  outs = run(act, car, 20)                                   # engaged at 45 (resumed: the same set speed)
  outs += run(act, car, 8, set_plus=True)                    # his new SET+ 0.2 s later
  car.set = 46 * MPH
  outs += run(act, car, 5)
  assert events(outs, "resumed")
  d = events(outs, "driver")
  assert d and d[0]["direction"] == A.SET_PLUS and d[0]["why"] == "button"


def test_res_from_standby_is_resumed_and_the_planner_takes_it_as_an_engagement():
  outs, _ = _engage("RES", 0)
  assert events(outs, "resumed") and not events(outs, "driver") and not events(outs, "engaged")
  p = Planner()
  _planner_step(p, 45, [engaged(45)])
  _planner_step(p, 48, [driver("SET_PLUS", 48)])
  _planner_step(p, 48, cruise=False)
  for e in events(outs, "resumed"):
    _planner_step(p, 48, [dict(e, after=48 * MPH)])
  assert p.owner == ENGAGED and abs(p.anchor - 48 * MPH) < 1e-6
