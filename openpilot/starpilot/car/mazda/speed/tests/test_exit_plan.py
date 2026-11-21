"""The exit plan (part f, F4) through the whole chain (map view entries -> exit plan [+ the curve plan once the match
is on the ramp] -> planner -> actuator -> the MRCC model of test_speed_control). Not a drive simulation: synthetic
views with the facts of the car (cruise lag 1.5 s, the measured decel bands). The regression scenarios in unit form:
exit 48 (his blinker 370-300 m before the split, a 35 mph curve 250 m past it), a false exit (his blinker for a lane
change 150-60 m before the split, he stays on), two exits close together, his gas / buttons, the handover."""
from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed import exit_plan as E
from openpilot.starpilot.car.mazda.speed.caller import SpeedControl
from openpilot.starpilot.car.mazda.speed.contributors import Sources
from openpilot.starpilot.car.mazda.speed.curve_plan import CurvePlan
from openpilot.starpilot.car.mazda.speed.tests.test_speed_control import Road

MPH = M.MPH
SPLIT = 2000.0
R35 = (35 * MPH) ** 2 / 2.6          # a 35 mph curve at the ramp side force


class ExitRoad(Road):
  def __init__(self, set_mph=72, exits=((SPLIT, 250.0),), take=True, shape_from=40.0, stay_from=60.0, blink=None,
               slc_mph=None, street_from=420.0, gps=True):
    super().__init__(set_mph=set_mph, curve=[])
    self.v = (set_mph - 2) * MPH
    self.exits = list(exits)            # (split, curve distance past it)
    self.take, self.shape_from, self.stay_from = take, shape_from, stay_from
    self.blink = blink or (lambda s: False)
    self.slc_mph = slc_mph
    self.street_from = street_from
    self.gps = gps

  def view(self):
    if not self.gps:
      return {"v": 2, "t": self.t, "gps": "none", "trust": 5, "curves": []}
    v = {"v": 2, "t": self.t, "gps": "ok", "trust": 0, "held": False, "matched": True, "cls": 1, "link": False,
         "chain": 1, "curves": [], "start_ok": False}
    out = []
    for k, (split, cd) in enumerate(self.exits):
      d = split - self.s
      if d > 1500 or d < -600:
        continue
      pts = [[round(split + cd + x - self.s, 1), R35] for x in (0.0, 40.0, 80.0)]
      past = -d
      taking = self.take is True or (type(self.take) is int and self.take == k)
      sh, m = 0, 1
      if taking and past >= self.shape_from:
        sh = 2
      if not taking and past >= self.stay_from:
        sh = 1
      if taking and past >= 120.0:
        m = 2
        v.update(cls=2, link=True, chain=100 + k)
        if past >= self.street_from:
          v.update(cls=5, link=False, chain=200 + k)
      if taking and 120.0 <= past < cd + 120.0:
        v["curves"] = [{"id": f"r{k}", "pts": [[p[0], p[1]] for p in pts], "ramp": True, "adv": 0.0}]
      out.append({"id": f"x{k}", "d": round(d, 1), "side": 1, "hwy": False, "spd": 45 * MPH, "pts": pts, "m": m,
                  "sh": sh, "ly": None, "hy": None})
    if out:
      v["exits"] = out
    return v


def run(road, cfg=None, frames=None, every=None, extra=()):
  cfg = cfg or C.Config(enabled=True, curves="on", exits="on")
  plan = E.ExitPlan(cfg, presses=cfg.presses("exits"))
  contribs = [CurvePlan(cfg, presses=cfg.presses("curves")), plan] if cfg.active("curves") else [plan]
  contribs += list(extra)
  src = Sources(want_model=False, sm=None, read_view=road.view, clock=lambda: road.t)
  ctl = SpeedControl(None, cfg, None, False, contributors=contribs, sources=src)
  trace = []
  frames = frames or int((SPLIT + 900) / max(road.v, 5.0) / 0.01)
  slc_at = road.slc_mph if callable(road.slc_mph) else (lambda s: road.slc_mph)
  for k in range(frames):
    road.tick()
    d = {"cruise_enabled": True, "main_on": True, "set_speed": road.car.set, "v_ego": road.v, "gas": False,
         "brake": False, "standstill": False, "set_plus": False, "set_minus": False, "resume": False, "cancel": False,
         "wheel_counter": road.car.counter, "other_button": False, "engaged": True, "lkas_blocked": False,
         "blinker": road.blink(road.s), "right_blinker": road.blink(road.s), "left_blinker": False}
    if slc_at(road.s) is not None:
      src.slc = {"limit": slc_at(road.s) * MPH, "upcoming": 0.0, "t": road.t}
    if every is not None:
      every(k, d, road)
    r = ctl.step(d)
    if r is not None:
      road.car.receive(*r)
    trace.append((road.s, road.v, road.car.set_units, plan.last[0] if plan.last else ""))
  return trace, ctl, plan


def at(trace, s):
  return min(trace, key=lambda x: abs(x[0] - s))


def reach_room(v, v_curve, decel=E.REACH_CHECK, dead=1.0):
  """m the cruise needs from v to v_curve at `decel` after `dead` s (old CHIME_DEAD: at the split the reach profile
  has the set below the car already, the cruise is braking)."""
  return v * dead + max(v * v - v_curve * v_curve, 0.0) / (2 * decel)


def test_exit_48_his_blinker_before_the_split_arrives_at_the_curve_speed():
  road = ExitRoad(set_mph=72, blink=lambda s: SPLIT - 370 <= s < SPLIT - 300)
  trace, ctl, plan = run(road)
  _, v_split, _, _ = at(trace, SPLIT)
  _, v_curve, _, _ = at(trace, SPLIT + 250)
  assert v_curve / MPH <= 38.5, (v_split / MPH, v_curve / MPH)
  # the reach bound: from the split the tight point was still reachable at <= 0.9 m/s^2
  assert reach_room(v_split, 35 * MPH) <= 250.0 + 1e-6, v_split / MPH
  whys = {w for s, _, _, w in trace if SPLIT - 300 <= s < SPLIT}
  assert "reach" in whys
  # without his blinker (the same exit, taken): no slowing before the split, and it comes in faster
  road0 = ExitRoad(set_mph=72)
  trace0, _, _ = run(road0)
  assert all(abs(x[2] - 72) < 1e-6 for x in trace0 if x[0] < SPLIT - 10)
  assert at(trace0, SPLIT + 250)[1] > v_curve + 2 * MPH


def test_a_false_exit_from_a_lane_change_blinker_is_bounded_and_released():
  road = ExitRoad(set_mph=72, take=False, stay_from=60.0, blink=lambda s: SPLIT - 150 <= s < SPLIT - 60)
  trace, ctl, plan = run(road, frames=int(3600 / 32 / 0.01))
  v0 = at(trace, SPLIT - 160)[1]
  v_split = at(trace, SPLIT)[1]
  low = min(x[1] for x in trace if x[0] >= SPLIT - 160)
  assert low >= v_split - E.DIP_MAX - 1.0 * MPH and low >= v0 - 15 * MPH, (v0 / MPH, v_split / MPH, low / MPH)
  assert plan.done and "x0" in plan.done
  assert abs(trace[-1][2] - 72) < 1e-6                              # his speed back
  assert plan.ended.id == "x0" and plan.ended.released == "highway" and not plan.ended.confirmed
  assert plan.ended.firm_t <= E.FIRM_MAX + 1e-6 and not plan.ended.reach_on   # armed too late for the reach profile
  assert all(abs(x[2] - 72) < 1e-6 for x in trace if x[0] < SPLIT)             # nothing before the split


def test_no_evidence_no_slowing_before_the_split_and_staying_on_costs_nothing():
  road = ExitRoad(set_mph=72, take=False)
  trace, _, plan = run(road, frames=int(3200 / 32 / 0.01))
  assert all(abs(x[2] - 72) < 1e-6 for x in trace)


def test_two_exits_close_together_carry_no_evidence():
  # his blinker for exit A (he stays on), exit B 450 m later: B gets a fresh plan, nothing carried
  road = ExitRoad(set_mph=72, exits=((SPLIT, 250.0), (SPLIT + 450, 250.0)), take=False,
                  blink=lambda s: SPLIT - 380 <= s < SPLIT - 250)
  trace, _, plan = run(road, frames=int(3600 / 32 / 0.01))
  assert "x0" in plan.done
  p = plan.plan if plan.plan is not None else plan.ended
  assert p.id == "x1" and not (p.signal or p.lane or p.model or p.reach_on), vars(p)
  # B itself: no slowing for B before its split
  before_b = [x for x in trace if SPLIT + 200 <= x[0] < SPLIT + 440]
  assert all(x[3] in ("watch", "", "released") or x[3] is None for x in before_b), {x[3] for x in before_b}


def test_his_gas_held_or_his_button_releases_the_plan():
  def gas(k, d, road):
    if SPLIT - 250 <= road.s < SPLIT - 150:
      d["gas"] = True
  road = ExitRoad(set_mph=72, take=False, blink=lambda s: SPLIT - 380 <= s < SPLIT - 300)
  _, _, plan = run(road, frames=int(2600 / 32 / 0.01), every=gas)
  assert plan.done.get("x0") is not None
  road = ExitRoad(set_mph=72, take=False, blink=lambda s: SPLIT - 380 <= s < SPLIT - 300)

  def button(k, d, road):
    if SPLIT - 200 <= road.s < SPLIT - 198:
      d["cancel"] = True
  _, _, plan = run(road, frames=int(2600 / 32 / 0.01), every=button)
  assert plan.done.get("x0") is not None


def test_the_handover_to_the_streets_limit():
  road = ExitRoad(set_mph=72, blink=lambda s: SPLIT - 370 <= s < SPLIT - 300, slc_mph=35, street_from=420.0)
  trace, ctl, plan = run(road, frames=int(2950 / 30 / 0.01) + 2500)
  assert abs(ctl.planner.anchor / MPH - 38) < 1e-6 and ctl.planner.ours_why == "handover", ctl.planner.anchor / MPH
  assert abs(trace[-1][2] - 38) < 1e-6


def test_a_lower_limit_after_the_handover_recaps_our_anchor():
  from openpilot.starpilot.car.mazda.speed.limits import SpeedLimit
  cfg = C.Config(enabled=True, curves="on", exits="on", speed_limit="on")
  road = ExitRoad(set_mph=72, blink=lambda s: SPLIT - 370 <= s < SPLIT - 300,
                  slc_mph=lambda s: 70 if s < SPLIT + 420 else (35 if s < SPLIT + 800 else 25))
  trace, ctl, plan = run(road, cfg=cfg, frames=int(2950 / 30 / 0.01) + 6000, extra=[SpeedLimit(cfg)])
  assert ctl.planner.owner == "ours" and ctl.planner.ours_why == "limit"
  assert abs(ctl.planner.anchor / MPH - 28) < 1e-6 and abs(trace[-1][2] - 28) < 1e-6


def test_handover_floor_is_ten_under_the_car_until_the_street():
  cfg = C.Config(enabled=True, exits="on")
  plan = E.ExitPlan(cfg)
  p = E.Plan("x0", 0.0, 1)
  p.confirmed = p.handover = True
  p.spd = 45 * MPH
  plan.plan = p
  ctx = {"now": 1.0, "v_ego": 60 * MPH, "anchor": 72 * MPH,
         "view": {"gps": "ok", "t": 1.0, "trust": 0, "matched": True, "link": True, "cls": 2}}
  out = plan._handover(ctx, E.Contribution("exit", None, kind="handover"))
  assert abs(out.anchor / MPH - 52) < 1e-6                         # 60 + 2 (dash) - 10, not the street's 48
  ctx["view"]["trust"] = 1
  out = plan._handover(ctx, E.Contribution("exit", None, kind="handover"))
  assert out.anchor is None                                         # only on a confident match


def test_no_gps_drops_an_unconfirmed_plan():
  road = ExitRoad(set_mph=72, blink=lambda s: SPLIT - 370 <= s < SPLIT - 300)

  def lose(k, d, road):
    road.gps = road.s < SPLIT - 200
  trace, _, plan = run(road, frames=int(2400 / 32 / 0.01), every=lose)
  assert plan.done.get("x0") is not None or plan.plan is None


def test_test_mode_never_presses():
  road = ExitRoad(set_mph=72, blink=lambda s: SPLIT - 370 <= s < SPLIT - 300)
  trace, ctl, plan = run(road, cfg=C.Config(enabled=True, exits="test"))
  assert road.car.sent == [] and all(abs(x[2] - 72) < 1e-6 for x in trace)
  assert any(x[3] == "reach" for x in trace)


def test_lane_view_edge_lane():
  lv = E.LaneView()
  right_edge = ([0.9, 0.9, 0.9, 0.1], [-5.4, -1.8, 1.8, 5.4], [-9.0, 3.5], [0.3, 0.3])
  for _ in range(25):
    lv.update(0.05, right_edge)
  assert lv.edge[1] and not lv.edge[-1]
  for _ in range(100):
    lv.update(0.05, None)
  assert not lv.edge[1]
  assert E.path_y_at(([0.0, 50.0, 100.0], [0.0, 1.0, 3.0]), 75.0) == 2.0 and E.path_y_at(None, 10.0) is None


def test_his_blinker_for_the_second_of_two_close_exits_counts_for_it():
  road = ExitRoad(set_mph=72, exits=((SPLIT, 250.0), (SPLIT + 450, 250.0)), take=1,
                  blink=lambda s: SPLIT + 150 <= s < SPLIT + 220)
  trace, _, plan = run(road, frames=int(3600 / 32 / 0.01))
  assert plan.done.get("x0") is not None and plan.ended is not None
  b = plan.plan if plan.plan is not None and plan.plan.id == "x1" else plan.ended
  assert b is not None and b.id == "x1" and b.signal and b.reach_on and b.confirmed
  assert at(trace, SPLIT + 450)[1] < at(trace, SPLIT + 100)[1] - 5 * MPH     # slowed before B's split


def test_the_edge_lane_alone_only_holds_the_speed_at_the_split():
  cfg = C.Config(enabled=True, exits="on")
  plan = E.ExitPlan(cfg)
  plan.lanes.edge[1] = True
  plan.lanes.update = lambda dt, lanes: None
  v, t, s = 30.0, 0.0, 0.0
  whys = []
  while s < SPLIT + 30:
    t += 0.01
    s += v * 0.01
    d = SPLIT - s
    vw = {"v": 2, "t": t, "gps": "ok", "trust": 0, "matched": True, "cls": 1, "link": False,
          "exits": [{"id": "x0", "d": d, "side": 1, "hwy": False, "spd": 20.0, "m": 1, "sh": 0, "ly": None, "hy": None,
                     "pts": [[d + 250 + x, R35] for x in (0.0, 40.0)]}]}
    out = plan({"now": t, "v_ego": v, "view": vw, "model_new": True, "act_events": []})
    whys.append((s, plan.last[0], out.target))
  pre = [w for w in whys if w[0] < SPLIT]
  assert all(w[2] is None for w in pre)                              # nothing before the split
  post = [w for w in whys if w[0] >= SPLIT + 1]
  assert post and all(w[1] == "hold" and abs(w[2] - v) < 1e-6 for w in post), post[:3]
  assert plan.plan.lane and not plan.plan.signal


def test_the_model_path_against_the_maps_laterals():
  plan = E.ExitPlan(C.Config(enabled=True, exits="on"))
  path = ([0.0, 30.0, 60.0, 90.0], [0.0, 1.0, 4.5, 9.0])           # bending right: 4.5 m right 60 m ahead
  ctx = {"path_y": path, "model_age": 0.1}
  assert plan._model_verdict(ctx, {"ly": 5.0, "hy": 0.0}) == 2
  assert plan._model_verdict(dict(ctx, path_y=([0.0, 90.0], [0.0, 0.2])), {"ly": 5.0, "hy": 0.0}) == 1
  assert plan._model_verdict(ctx, {"ly": 2.0, "hy": 0.0}) is None   # not apart enough
  assert plan._model_verdict(dict(ctx, model_age=1.0), {"ly": 5.0, "hy": 0.0}) is None
  assert plan._model_verdict(ctx, {"ly": None, "hy": 0.0}) is None
