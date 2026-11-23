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
      past = -d
      taking = self.take is True or (type(self.take) is int and self.take == k)
      if taking and past >= max(self.street_from, 600.0):
        v.update(cls=5, link=False, chain=200 + k)     # (still on the exit's street once it is out of the list)
      if d > 1500 or d < -600:
        continue
      pts = [[round(split + cd + x - self.s, 1), R35] for x in (0.0, 40.0, 80.0)]
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
  lane = getattr(road, "lane", None)
  if lane is not None:
    plan.lanes.update = lambda dt, lanes: None    # the camera's edge lane given by the scenario
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
    if lane is not None:
      plan.lanes.edge[1] = bool(lane(road.s))
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
  whys = {w for s, _, _, w in trace if SPLIT - 300 <= s < SPLIT}
  assert "reach" in whys
  # His blinker 370-300 m, then off (beyond BLINKER_GRACE) with the ramp not confirmed: the 12 mph bound from where
  # our slowing began applies again (70 here) and the plan holds where it is (no SET+ back up in the exit area).
  # The car is ~60 mph at the split and ~48 mph at the 35 mph curve: the DRIVER MUST BRAKE. (README / settings.)
  conf = next(x for x in trace if x[3] == "confirmed")
  assert min(x[2] for x in trace if x[0] < conf[0]) - 2.0 >= 70.0 - 12.0 - 0.5          # set (dash - 2) vs the bound
  off = SPLIT - 300 + 32.0                                                                # off + the grace
  held = [x[2] for x in trace if off <= x[0] < conf[0]]
  assert all(b <= a + 1e-6 for a, b in zip(held, held[1:], strict=False))                 # never raised in the area
  assert 50.0 <= v_split / MPH <= 63.0 and v_curve / MPH <= 50.0, (v_split / MPH, v_curve / MPH)
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


def test_his_button_far_before_the_exit_does_not_cancel_it():
  # r3 (M-59): his SET+ ~500 m before the split released the exit for good; at the split nothing slowed.
  # Now a press while the exit is only looked at (beyond SIGNAL_REACH, no target yet) is ignored by the exit plan.
  for far in (1400.0, 900.0, E.SIGNAL_REACH + 100.0):
    def button(k, d, road, far=far):
      if SPLIT - far <= road.s < SPLIT - far + 2:
        d["set_plus"] = True
    road = ExitRoad(set_mph=72, blink=lambda s: SPLIT - 370 <= s < SPLIT - 300)
    trace, ctl, plan = run(road, every=button)
    p = plan.plan if plan.plan is not None and plan.plan.id == "x0" else plan.ended
    assert p is not None and p.confirmed and not p.cancelled and p.released != "his button", (far, vars(p))
    whys = {x[3] for x in trace if SPLIT - 300 <= x[0] < SPLIT}
    assert "reach" in whys or "reach signal" in whys, (far, whys)
    assert at(trace, SPLIT + 250)[1] / MPH <= 50.0                    # the same slowdown as without his press


def test_his_gas_far_before_the_exit_does_not_cancel_it():
  def gas(k, d, road):
    if SPLIT - 1200 <= road.s < SPLIT - 900:
      d["gas"] = True                                                  # held ~9 s, far ahead
  road = ExitRoad(set_mph=72, blink=lambda s: SPLIT - 370 <= s < SPLIT - 300)
  _, _, plan = run(road, every=gas)
  p = plan.plan if plan.plan is not None and plan.plan.id == "x0" else plan.ended
  assert p is not None and p.confirmed and p.released != "his gas", vars(p)


def test_his_button_releases_an_acting_plan_as_before():
  # his blinker latched for the exit (a target set), and past the split
  for at_m, blink in ((250.0, lambda s: SPLIT - 380 <= s < SPLIT - 300), (-20.0, None)):
    def button(k, d, road, at_m=at_m):
      if SPLIT - at_m <= road.s < SPLIT - at_m + 2:
        d["cancel"] = True
    road = ExitRoad(set_mph=72, take=False, blink=blink)
    _, _, plan = run(road, frames=int(2600 / 32 / 0.01), every=button)
    assert plan.done.get("x0") is not None and plan.ended.released == "his button", at_m
  # acting: its target was set (the flag stays once set), his blinker, the reach profile, confirmed, past the split
  cfg = C.Config(enabled=True, exits="on")
  ex = E.ExitPlan(cfg)
  p = E.Plan("x0", 1000.0, 1)
  assert not ex._acting(p, 1000.0) and not ex._acting(p, E.SIGNAL_REACH) and not ex._acting(p, 1.0)
  p.acted = True
  assert ex._acting(p, 1000.0) and ex._acting(E.Plan("x1", 0.0, 1), 0.0)
  p = E.Plan("x2", 1000.0, 1)
  p.signal = True
  assert ex._acting(p, 300.0)


def test_round3_his_press_or_gas_inside_signal_reach_before_the_plan_acts_does_not_cancel_the_exit():
  # r2 rt 1044 (SET+ 227 m before the split), r3 rt 255.6 (163 m): no blinker, no slowing yet: about his own speed.
  # Here he then takes the exit: it is still confirmed and handed over after the split.
  for at_m in (227.0, 163.0, 30.0):
    def button(k, d, road, at_m=at_m):
      if SPLIT - at_m <= road.s < SPLIT - at_m + 2:
        d["set_plus"] = True
    road = ExitRoad(set_mph=72, slc_mph=35, street_from=420.0)
    _, _, plan = run(road, frames=int(2950 / 30 / 0.01) + 2500, every=button)
    p = plan.plan if plan.plan is not None and plan.plan.id == "x0" else plan.ended
    assert p is not None and p.confirmed and not p.cancelled and p.released != "his button", (at_m, vars(p))
  # r2 rt 987: his gas held across the 400 m line (from 450 m to 330 m) no longer releases it at the line
  def gas(k, d, road):
    if SPLIT - 450 <= road.s < SPLIT - 330:
      d["gas"] = True
  road = ExitRoad(set_mph=72, slc_mph=35, street_from=420.0)
  _, _, plan = run(road, frames=int(2950 / 30 / 0.01) + 2500, every=gas)
  p = plan.plan if plan.plan is not None and plan.plan.id == "x0" else plan.ended
  assert p is not None and p.confirmed and p.released != "his gas", vars(p)


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



# ---- review items 1-4: a direct frame loop on the exit plan (no planner) ----

def frames(plan, seconds_or_until, v=29.0, s0=SPLIT - 600.0, view_fn=None, ctx_fn=None, t0=0.0):
  """Drive the exit plan at v from s0 until `until(s)`; view_fn(s, t) -> the exit entry fields, ctx_fn(s) -> ctx."""
  s, t, log = s0, t0, []
  until = seconds_or_until
  while not until(s):
    t += 0.01
    s += v * 0.01
    d = SPLIT - s
    ex = {"id": "x0", "d": d, "side": 1, "hwy": False, "spd": 20.0, "m": 1, "sh": 0, "ly": None, "hy": None,
          "pts": [[d + 250 + x, R35] for x in (0.0, 40.0)]}
    vw = {"v": 2, "t": round(t, 1), "gps": "ok", "trust": 0, "matched": True, "cls": 1, "link": False}
    if view_fn is not None:
      view_fn(s, t, ex, vw)
    vw["exits"] = [ex]
    ctx = {"now": t, "v_ego": v, "view": vw, "act_events": []}
    if ctx_fn is not None:
      ctx.update(ctx_fn(s))
    out = plan(ctx)
    log.append((s, plan.last[0] if plan.last else "", out.target, plan.plan, ctx))
  return log


def blink_right(lo, hi):
  return lambda s: {"right_blinker": lo <= s < hi}


def test_review_1a_scattered_link_frames_never_add_up_to_a_confirmation():
  plan = E.ExitPlan(C.Config(enabled=True, exits="on"))

  def jitter(s, t, ex, vw):
    if s > SPLIT + 10 and int(round(t * 10)) % 8 == 0:
      ex["sh"] = 2                                  # one view in eight says "link", the rest nothing
  log = frames(plan, lambda s: s > SPLIT + 300, view_fn=jitter, ctx_fn=blink_right(SPLIT - 380, SPLIT - 300))
  assert not any(p is not None and p.confirmed for *_, p, _c in log)


def test_review_1b_one_link_frame_then_silence_does_not_block_the_highway_release():
  plan = E.ExitPlan(C.Config(enabled=True, exits="on"))

  def once_then_highway_match(s, t, ex, vw):
    if SPLIT + 20 <= s < SPLIT + 23:
      ex["sh"] = 2                                  # one view of "link"
    # (then silent shapes; the trusted match stays on the highway)
  log = frames(plan, lambda s: s > SPLIT + 300, view_fn=once_then_highway_match,
               ctx_fn=blink_right(SPLIT - 380, SPLIT - 300))
  ended = plan.ended
  assert ended is not None and ended.released == "highway" and not ended.confirmed
  assert all(entry[2] is None for entry in log if entry[0] > SPLIT + 250)


def test_review_1c_a_false_confirmation_is_overturned_by_highway_evidence_or_his_gas():
  def false_link_then_highway(s, t, ex, vw):
    if SPLIT + 20 <= s < SPLIT + 60:
      ex["sh"] = 2                                  # a false "link" long enough to confirm ...
    elif s >= SPLIT + 80:
      ex["sh"] = 1                                  # ... then the shapes say highway
  plan = E.ExitPlan(C.Config(enabled=True, exits="on"))
  log = frames(plan, lambda s: s > SPLIT + 200, view_fn=false_link_then_highway,
               ctx_fn=blink_right(SPLIT - 380, SPLIT - 300))
  assert plan.ended.released == "highway after the confirmation" and plan.ended.cancelled   # (it was confirmed)
  assert all(entry[2] is None for entry in log if entry[0] > SPLIT + 110)

  def false_link_then_silent(s, t, ex, vw):
    if SPLIT + 20 <= s < SPLIT + 60:
      ex["sh"] = 2
  gas = lambda s: {"gas": s > SPLIT + 70, **blink_right(SPLIT - 380, SPLIT - 300)(s)}  # noqa: E731
  plan = E.ExitPlan(C.Config(enabled=True, exits="on"))
  log = frames(plan, lambda s: s > SPLIT + 200, view_fn=false_link_then_silent, ctx_fn=gas)
  assert plan.ended.released == "his gas after the confirmation"
  assert all(entry[2] is None for entry in log if entry[0] > SPLIT + 130)
  # his gas held while matched on the link with trust 0 (a real ramp): the ramp's slowdown stays
  def on_ramp(s, t, ex, vw):
    false_link_then_silent(s, t, ex, vw)
    if s > SPLIT + 60:
      ex["m"] = 2
      vw.update(link=True, cls=2)
  plan = E.ExitPlan(C.Config(enabled=True, exits="on"))
  log = frames(plan, lambda s: s > SPLIT + 150, view_fn=on_ramp, ctx_fn=gas)
  p = plan.plan
  assert p is not None and p.confirmed and not p.released and log[-1][2] is not None


def test_review_1c_an_overturned_handover_restores_his_speed():
  plan = E.ExitPlan(C.Config(enabled=True, exits="on"))
  p = E.Plan("x0", 0.0, 1)
  p.confirmed = p.handover = True
  p.before = 72 * MPH
  plan.plan = p
  out = plan._overturn({"anchor": 52 * MPH}, E.Contribution("exit", None, kind="handover"), "highway", 1.0)
  assert out.kind == "restore" and abs(out.anchor - 72 * MPH) < 1e-6 and p.cancelled
  from openpilot.starpilot.car.mazda.speed.planner import Planner, PlanInputs, OURS
  pl = Planner()
  pl._set_anchor(52 * MPH, "handover")
  pl.owner, pl.ours_why = OURS, "handover"
  pl.update(PlanInputs(v_ego=25.0, set_speed=52 * MPH, cruise_enabled=True, main_on=True, events=[], contributions=[out]))
  assert abs(pl.anchor - 72 * MPH) < 1e-6
  pl.ours_why = "limit"                             # a restore never raises anything but our handover anchor
  pl._set_anchor(52 * MPH, "limit")
  pl.update(PlanInputs(v_ego=25.0, set_speed=52 * MPH, cruise_enabled=True, main_on=True, events=[], contributions=[out]))
  assert abs(pl.anchor - 52 * MPH) < 1e-6


def test_review_2_the_splits_place_follows_the_view_and_the_plan_keeps_its_evidence():
  plan = E.ExitPlan(C.Config(enabled=True, exits="on"))

  def chord(s, t, ex, vw):
    d = SPLIT - s
    ex["d"] = d * 0.93 if d > 0 else d              # the straight distance: up to ~100 m short far out
  frames(plan, lambda s: s > SPLIT - 20, s0=SPLIT - 1450, view_fn=chord, ctx_fn=blink_right(SPLIT - 380, SPLIT - 300))
  p = plan.plan
  assert p is not None and p.signal and plan.ended is None          # never "another exit"
  assert abs((p.at - plan.odo) - 20.0) < 6.0                        # its split where the view says, 20 m ahead


def test_review_3a_his_other_blinker_cancels_the_exit_blinker():
  plan = E.ExitPlan(C.Config(enabled=True, exits="on"))
  ctx = lambda s: {"right_blinker": SPLIT - 380 <= s < SPLIT - 330, "left_blinker": SPLIT - 250 <= s < SPLIT - 220}  # noqa: E731
  log = frames(plan, lambda s: s > SPLIT - 5, ctx_fn=ctx)
  assert any(entry[1] == "reach" for entry in log if entry[0] < SPLIT - 260)
  assert all(entry[2] is None for entry in log if entry[0] > SPLIT - 215)
  assert not plan.plan.signal and not plan.plan.reach_on
  # his exit blinker simply going off keeps it
  plan = E.ExitPlan(C.Config(enabled=True, exits="on"))
  log = frames(plan, lambda s: s > SPLIT - 5, ctx_fn=blink_right(SPLIT - 380, SPLIT - 330))
  assert plan.plan.signal and log[-1][1] == "reach"


def test_review_4_a_handover_onto_a_faster_street_is_done_and_frees_the_limit():
  plan = E.ExitPlan(C.Config(enabled=True, exits="on"))
  p = E.Plan("x0", 0.0, 1)
  p.confirmed = p.handover = True
  p.spd = 55 * MPH
  p.street_t = 5.0
  plan.plan = p
  ctx = {"now": 1.0, "v_ego": 35 * MPH, "anchor": 45 * MPH, "slc": {"limit": 55 * MPH, "upcoming": 0.0, "t": 1.0},
         "view": {"gps": "ok", "t": 1.0, "trust": 0, "matched": True, "link": False, "cls": 5}}
  plan.dt = 0.01
  plan._handover(ctx, E.Contribution("exit", None, kind="handover"))
  assert p.handed                                   # his 45 is under the street's 58: nothing to lower, done
  ctx2 = {}
  plan._handover(ctx2, E.Contribution("exit", None, kind="handover"))
  assert "exit_busy" not in ctx2



# ---- trust a blinker that is still on (owner's decision after review #3b) ----

def peak_decel(trace):
  return max((trace[k - 100][1] - trace[k][1]) for k in range(100, len(trace)))


def test_blinker_1_kept_on_through_the_split_slows_as_much_as_the_ramp_needs():
  road = ExitRoad(set_mph=72, blink=lambda s: SPLIT - 370 <= s < SPLIT + 20)
  trace, _, plan = run(road)
  v_split, v_curve = at(trace, SPLIT)[1] / MPH, at(trace, SPLIT + 250)[1] / MPH
  assert v_curve <= 38.5 and v_split <= 57.5, (v_split, v_curve)          # ~56 / ~38
  assert peak_decel(trace) <= E.REACH_CHECK + 0.05                       # ~0.8 m/s^2
  assert any(x[3] == "reach signal" for x in trace)
  assert min(x[1] for x in trace if x[0] < SPLIT) / MPH < 70 - 12         # deeper than the bound: allowed while on


def test_blinker_2_his_habit_320_270_with_the_camera_edge_lane_still_counts():
  # (drive 75 as driven: his blinker 320-270 m, the car in the lane next to the right edge = the exit lane)
  road = ExitRoad(set_mph=72, blink=lambda s: SPLIT - 320 <= s < SPLIT - 270)
  road.lane = lambda s: SPLIT - 450 <= s < SPLIT + 60
  trace, _, plan = run(road)
  assert at(trace, SPLIT + 250)[1] / MPH <= 40.0                          # ~38
  assert any(x[3] == "reach edge lane" for x in trace)
  # the same blink without the edge lane: bounded (~48 mph at the curve: he must brake)
  road = ExitRoad(set_mph=72, blink=lambda s: SPLIT - 320 <= s < SPLIT - 270)
  trace, _, _ = run(road)
  assert at(trace, SPLIT + 250)[1] / MPH > 44.0
  # the false cost of the edge-lane rule: he signals into the right lane near an exit and stays in it
  road = ExitRoad(set_mph=72, take=False, blink=lambda s: SPLIT - 320 <= s < SPLIT - 270)
  road.lane = lambda s: SPLIT - 450 <= s < SPLIT + 300
  trace, _, plan = run(road, frames=int(3600 / 32 / 0.01))
  low = min(x[1] for x in trace) / MPH
  assert 45.0 <= low and plan.ended.released == "highway" and abs(trace[-1][2] - 72) < 1e-6, low   # ~48 (-22 mph)


def test_blinker_3_a_quick_lane_change_blink_stays_within_12_mph():
  road = ExitRoad(set_mph=72, take=False, blink=lambda s: SPLIT - 150 <= s < SPLIT - 60)
  trace, _, plan = run(road, frames=int(3600 / 32 / 0.01))
  assert min(x[1] for x in trace) / MPH >= 70 - 12 and plan.ended.released == "highway"
  assert abs(trace[-1][2] - 72) < 1e-6


def test_blinker_4_held_to_the_split_for_a_lane_change_then_stays_on():
  road = ExitRoad(set_mph=72, take=False, blink=lambda s: SPLIT - 350 <= s < SPLIT)
  trace, _, plan = run(road, frames=int(3600 / 32 / 0.01))
  low = min(x[1] for x in trace) / MPH
  assert 70 - 23 <= low < 70 - 12, low                                    # the price of trusting it: ~21 mph
  assert plan.ended.released == "highway" and abs(trace[-1][2] - 72) < 1e-6


def test_blinker_grace_bridges_flash_gaps_and_a_blinker_back_on_deepens_again():
  # flashing: on 0.35 s, off 0.35 s (the lamp) -> one continuous "on" for the plan
  road = ExitRoad(set_mph=72, blink=lambda s: SPLIT - 370 <= s < SPLIT and int((s - SPLIT) / 11.0) % 2 == 0)
  trace, _, _ = run(road)
  assert all(x[3] == "reach signal" for x in trace if SPLIT - 340 <= x[0] < SPLIT - 40)
  assert at(trace, SPLIT + 250)[1] / MPH <= 38.5
  # off at 300 m, back on from 150 m: deep again
  road = ExitRoad(set_mph=72, blink=lambda s: SPLIT - 370 <= s < SPLIT - 300 or SPLIT - 150 <= s < SPLIT)
  trace, _, _ = run(road)
  assert any(x[3] == "reach signal" for x in trace if SPLIT - 140 <= x[0] < SPLIT)
