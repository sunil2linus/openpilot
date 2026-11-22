"""The one map view age rule (view_age.py) and every reader with the GPS rate of the device: 1 Hz (qcomgpsd on the
comma 3X / tizi, e.g. the Konik A1M) and 10 Hz (ubloxd). The views carry the real timing: a fix every 1 / hz s, the
view's "t" = the fix minus GPS_LAG and its content the car's place at that time, card reading the file at 10 Hz
(contributors.Sources) or every frame (the rig). Before this rule a view older than 1.0 s from its "t" (0.5 s in the
auto-resume) was "no map": with 1 Hz that was ~90% of the frames."""
import math

import pytest

from openpilot.starpilot.car.mazda.speed import auto_resume as ar
from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed import exit_plan as E
from openpilot.starpilot.car.mazda.speed import limits as L
from openpilot.starpilot.car.mazda.speed import notices as N
from openpilot.starpilot.car.mazda.speed import view_age as VA
from openpilot.starpilot.car.mazda.speed.contributors import Sources
from openpilot.starpilot.car.mazda.speed.merge import Merge
from openpilot.starpilot.car.mazda.speed.planner import OURS, PlanOutput
from openpilot.starpilot.car.mazda.speed.tests.rig import Rig, engaged, view
from openpilot.starpilot.car.mazda.speed.tests.test_exit_plan import SPLIT, ExitRoad, at, run
from openpilot.starpilot.car.mazda.speed.tests.test_speed_control import Road

MPH = M.MPH
RATES = (1, 10)
PHASE = 0.037                     # s: the GPS fixes are not in step with card's frames


class GpsFeed:
  """read_view for a test road: a new view only at each GPS fix (every 1 / hz s), made from the car's place GPS_LAG
  before the fix and stamped with that time; between fixes the same view; `stop()`: no new view any more."""

  def __init__(self, road, hz):
    self.road, self.make, self.period = road, road.view, 1.0 / hz
    self.hist = []
    self.next_fix = PHASE
    self.cur = None
    self.stopped = False

  def stop(self):
    self.stopped = True

  def _s_at(self, t):
    h = self.hist
    if t <= h[0][0]:
      return h[0][1]
    for (t0, s0), (t1, s1) in zip(h, h[1:], strict=False):
      if t0 <= t <= t1:
        return s0 + (s1 - s0) * (t - t0) / max(t1 - t0, 1e-9)
    return h[-1][1]

  def __call__(self):
    r = self.road
    self.hist.append((r.t, r.s))
    if len(self.hist) > 400:
      del self.hist[:100]
    fix = None
    while r.t + 1e-9 >= self.next_fix:
      fix, self.next_fix = self.next_fix, self.next_fix + self.period
    if fix is not None and not self.stopped:
      ts = fix - VA.GPS_LAG
      now_t, now_s = r.t, r.s
      r.t, r.s = ts, self._s_at(ts)
      try:
        self.cur = self.make()
      finally:
        r.t, r.s = now_t, now_s
    return self.cur


def at_hz(make, hz):
  """For the rig (no positions in its views): the view of the last fix at time t, stamped fix - GPS_LAG."""
  def f(t):
    fix = math.floor((t - PHASE) * hz) / hz + PHASE
    return make(fix - VA.GPS_LAG)
  return f


# ---------------- the rule ----------------

def test_the_rule_alive_current_gone():
  v = {"gps": "ok", "t": 10.0}                       # its GPS message at 10.85
  assert VA.view_age(v, 11.85) == pytest.approx(1.0)
  assert VA.view_live(v, 10.9) == VA.CURRENT and VA.view_live(v, 10.85 + VA.VIEW_SPAN + 0.01) == VA.LATE
  assert VA.view_alive(v, 10.85 + VA.VIEW_ALIVE) and not VA.view_alive(v, 10.85 + VA.VIEW_ALIVE + 0.01)
  assert not VA.view_alive(dict(v, gps="none"), 10.9) and not VA.view_alive(None, 0.0)
  assert not VA.view_alive({"gps": "ok"}, 0.0)
  # the GPS period sets how long a view is current: one period + SPAN_SLACK, at most VIEW_SPAN
  for hz, span in ((1, 1.25), (10, 0.35)):
    clock = VA.ViewClock()
    for k in range(5):
      clock.update({"gps": "ok", "t": k / hz}, k / hz + VA.GPS_LAG)
      assert clock.new
      clock.update({"gps": "ok", "t": k / hz}, k / hz + VA.GPS_LAG + 0.01)
      assert not clock.new
    assert clock.span == pytest.approx(span)
  n = VA.ViewCount()
  n.add(1.0)
  n.add(1.0)
  assert not n
  n.add(2.0)
  assert n
  n.reset()
  assert not n


@pytest.mark.parametrize("hz", RATES)
def test_no_gps_false_with_steady_views_and_true_about_3_s_after_they_stop(hz):
  road = Road(set_mph=58)
  feed = GpsFeed(road, hz)
  src = Sources(want_model=False, sm=None, read_view=feed, clock=lambda: road.t)
  no_gps = []
  for _ in range(3000):
    road.tick()
    no_gps.append((road.t, src.context({"v_ego": road.v})["no_gps"]))
  assert not any(g for t, g in no_gps if t > 1.2)     # (the first fix comes PHASE + 0.1 s read in)
  feed.stop()
  last_fix = feed.next_fix - feed.period
  for _ in range(500):
    road.tick()
    no_gps.append((road.t, src.context({"v_ego": road.v})["no_gps"]))
  first = next(t for t, g in no_gps if g and t > 1.2)
  assert VA.VIEW_ALIVE - 0.01 <= first - last_fix <= VA.VIEW_ALIVE + 0.11       # ~2.5 s after its GPS message
  assert all(g for t, g in no_gps if t >= first)


@pytest.mark.parametrize("hz", RATES)
def test_map_covers_camera_at_the_gps_rate(hz):
  road = Road(set_mph=58)                              # a curve 1200 m on
  feed = GpsFeed(road, hz)
  src = Sources(want_model=False, sm=None, read_view=feed, clock=lambda: road.t, map_presses=True)
  covers = []
  while road.s < 1300.0:
    road.tick()
    ctx = src.context({"v_ego": road.v})
    if 1200.0 - road.s <= road.v * 2.5 - 5.0:          # the curve's entry within 2.5 s (the GPS lag kept in)
      covers.append(ctx["map_covers_camera"])
  assert covers and all(covers)


@pytest.mark.parametrize("hz", RATES)
def test_the_exit_confirms_on_the_link(hz):
  """Exit 48 (his blinker 370-300 m before the split, a 35 mph ramp curve 250 m past it). Before the rule: at 1 Hz
  the plan never left "hold" (LINK_CONFIRM / MATCH_CONFIRM built only from fresh frames) and the 12 mph bound stayed."""
  road = ExitRoad(set_mph=72, blink=lambda s: SPLIT - 370 <= s < SPLIT - 300)
  road.view = GpsFeed(road, hz)
  trace, ctl, plan = run(road)
  conf = next((x for x in trace if x[3] == "confirmed"), None)
  assert conf is not None, {x[3] for x in trace}
  # the shapes say link 40 m past the split; the fix is GPS_LAG old, the next fix up to 1 / hz later, then
  # LINK_CONFIRM (VIEWS_MIN views) and the 10 Hz read
  v_split = at(trace, SPLIT)[1]
  assert conf[0] <= SPLIT + 40.0 + v_split * (VA.GPS_LAG + 1.0 / hz + E.LINK_CONFIRM + 0.15), (conf[0] - SPLIT, v_split)
  # at the 35 mph curve: ~50 mph at 10 Hz, ~52 at 1 Hz (the fix's GPS_LAG and the 1 s between fixes confirm later than
  # test_exit_plan's lag-free views, <= 50; the driver brakes there, as the README says)
  assert at(trace, SPLIT + 250)[1] / MPH <= 53.0
  assert min(x[2] for x in trace) < 72 - 12 - 2      # below the 12 mph bound: the ramp's plan ran


def test_one_1hz_link_view_alone_never_confirms():
  plan = E.ExitPlan(C.Config(enabled=True, exits="on"))
  v, s, t = 29.0, SPLIT - 600.0, 0.0
  link_fix = None
  while s < SPLIT + 300:
    t += 0.01
    s += v * 0.01
    fix = math.floor(t - PHASE) + PHASE
    d = SPLIT - (s - v * (t - fix + VA.GPS_LAG))       # the exit's distance from the place at the fix
    sh = 0
    if d < -20.0 and link_fix is None:
      link_fix = fix
    if fix == link_fix:
      sh = 2                                           # one view says link, all the others nothing
    ex = {"id": "x0", "d": d, "side": 1, "hwy": False, "spd": 20.0, "m": 1, "sh": sh, "ly": None, "hy": None,
          "pts": [[d + 250 + x, (35 * MPH) ** 2 / 2.6] for x in (0.0, 40.0)]}
    vw = {"v": 2, "t": fix - VA.GPS_LAG, "gps": "ok", "trust": 0, "matched": True, "cls": 1, "link": False,
          "exits": [ex]}
    plan({"now": t, "v_ego": v, "view": vw, "act_events": [], "right_blinker": SPLIT - 380 <= s < SPLIT - 300})
    p = plan.plan or plan.ended
    assert not p.confirmed
  assert (plan.plan or plan.ended).link_t <= 0.0 or not (plan.plan or plan.ended).link_n


@pytest.mark.parametrize("hz", RATES)
def test_no_fallback_43_with_live_views_and_it_fires_once_they_are_gone(hz):
  road = Road(set_mph=40, curve=[])
  road.v = 40 * MPH
  feed = GpsFeed(road, hz)
  src = Sources(want_model=False, sm=None, read_view=feed, clock=lambda: road.t)
  sl = L.SpeedLimit(C.Config(enabled=True, speed_limit="on"))
  anchors = []
  for k in range(int(60 / 0.01)):
    road.tick()
    if k == int(30 / 0.01):
      feed.stop()
    ctx = src.context({"v_ego": road.v, "cruise_enabled": True, "anchor": 40 * MPH, "slc": None,
                       "act_events": [engaged(40)] if k == 0 else []})
    out = sl(ctx)
    anchors.append((road.t, out.kind if out.anchor is not None else None))
  assert not any(kind == "fallback" for t, kind in anchors if t < 30.0)
  first = next(t for t, kind in anchors if kind == "fallback")
  assert 30.0 + VA.VIEW_ALIVE - 1.0 + L.FALLBACK_AFTER <= first <= 30.0 + VA.VIEW_ALIVE + 0.2 + L.FALLBACK_AFTER


def test_the_fallback_timer_restarts_when_the_view_is_alive():
  """The old bug: fast_t was kept over the frames with a GPS position, so no_gps true 87% of the time (a 1 Hz GPS
  with the old rule) added up to 3 s and set 43 with a good fix."""
  sl = L.SpeedLimit(C.Config(enabled=True, speed_limit="on"))
  t = 0.0
  for k in range(1500):
    t += 0.01
    no_gps = (k % 100) < 87
    out = sl({"now": t, "v_ego": 40 * MPH, "cruise_enabled": True, "slc": None, "no_gps": no_gps,
              "act_events": [engaged(40)] if k == 0 else []})
    assert out.kind != "fallback" or out.anchor is None


@pytest.mark.parametrize("hz", RATES)
def test_a_limit_becomes_steady_at_the_gps_rate(hz):
  r = Rig([L.SpeedLimit(C.Config(enabled=True, speed_limit="on", limit_offset_mph=3))])
  r.step(v_mph=55, set_mph=55, events=[engaged(55)])
  r.step(v_mph=55, n=int(5.5 / r.dt), slc={"limit": 50 * MPH, "upcoming": 0.0, "t": 0.0},
         view=at_hz(lambda t: view(t), hz))
  assert r.anchor_mph == 53 and r.p.ours_why == "limit"


@pytest.mark.parametrize("hz", RATES)
def test_the_merge_boosts_at_the_gps_rate(hz):
  ramp = {"id": 7, "hwy": True, "spd": 70 * MPH, "from_hwy": False}
  r = Rig([Merge(C.Config(enabled=True, merge="on", limit_offset_mph=3))])
  r.step(v_mph=40, set_mph=45, events=[engaged(45)])
  r.step(v_mph=40, n=int(3.5 / r.dt), slc={"limit": 45 * MPH, "upcoming": 0.0, "t": 0.0}, preview=None,
         path=(list(range(0, 200, 5)), [0.0] * 40), model_age=0.0,
         view=at_hz(lambda t: view(t, link=True, cls=2, chain=7, ramp=ramp), hz))
  assert r.anchor_mph == 90 and r.p.owner == OURS and r.p.ours_why == "merge"


@pytest.mark.parametrize("hz", RATES)
def test_auto_resume_map_guard_passes_at_the_gps_rate(hz):
  lg, road = ar.AutoResumeLogic(), ar.ViewRoad()
  mk = at_hz(lambda t: {"v": 2, "t": t, "gps": "ok", "trust": 0, "matched": True, "cls": 5, "chain": 7}, hz)
  base = {"main_on": True, "set_speed": 45 * MPH, "v_ego": 45 * MPH, "angle": 0.0, "steer_rate": 0.0,
          "gear_drive": True, "cancel": False, "resume": False, "set_plus": False, "set_minus": False,
          "set_allowed": True, "can_valid": True}
  sent, waits = [], set()
  for f in range(1, 1600):
    now = f * 0.01
    d = dict(base, cruise_enabled=(f < 1000 and not sent) or bool(sent), brake=1000 - 3 <= f < 1100,
             gas=f >= 1200, wheel_counter=(f // 10) % 16)
    i = ar.inputs_from(d, {"now": now, "view": mk(now), "engageable": True}, road)
    if f > 300:
      assert i.plan_alive and lg.map_ok(i)
    b = lg.update(i)
    if lg.state in (ar.ARMED, ar.SENDING):
      waits.add(lg.wait_why)
    if b is not None:
      sent.append(f)
  assert sent and not {"map", "road just changed", "road name not fresh"} & waits, waits


def test_auto_resume_needs_the_road_in_two_views():
  road = ar.ViewRoad()
  v1 = {"v": 2, "t": 0.0, "gps": "ok", "trust": 0, "matched": True, "cls": 5, "chain": 7}
  road.update(v1, 1.0)
  assert road.views == 1
  road.update(v1, 1.5)
  assert road.views == 1                               # the same view again
  road.update(dict(v1, t=1.0), 2.0)
  assert road.views == 2
  road.update(dict(v1, t=2.0, chain=8), 3.0)
  assert road.views == 1                               # another road: counted again
  road.update(dict(v1, t=2.0, chain=8), 2.0 + VA.GPS_LAG + VA.VIEW_ALIVE + 0.1)
  assert road.views == 0                               # gone


@pytest.mark.parametrize("hz", RATES)
def test_gps_ok_for_10_s_is_reachable_curve_maps_not_built_here(hz):
  road = Road(set_mph=58, curve=[])
  make = road.view
  road.view = lambda: dict(make(), data=False)
  feed = GpsFeed(road, hz)
  src = Sources(want_model=False, sm=None, read_view=feed, clock=lambda: road.t)
  n = N.Notices(C.Config(enabled=True, curves="on", notices=True), writer=type("W", (), {"write": lambda *a, **k: True})())
  for _ in range(int(14 / 0.01)):
    road.tick()
    ctx = src.context({"v_ego": road.v})
    ctx["contribs"] = {}
    n.update(road.t, {"v_ego": road.v}, ctx, PlanOutput(command=None, reason=""), [], [], want_view=True)
  assert [k for _, k in n.sent] == ["no_data"]
