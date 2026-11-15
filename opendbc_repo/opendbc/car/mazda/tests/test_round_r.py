"""Round R (drive 76, round Q's build): the fixes R1-R7. Synthetic geometry and speeds only."""
import importlib.util
import os

HERE = os.path.dirname(__file__)


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, path))
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


SS = _load("tss_round_r", "test_street_speed.py")
cs, MPH = SS.cs, SS.MPH


# ---------------- R4: the street speed lands on its target ----------------

def test_r4_the_street_target_is_reached_with_the_dashs_x989_readings():
  # drive 76 t 1857 (W Utica Rd): rule 8's match on engage, a 45 street + 3 = 48; the dash reads 49.989 for 49. Before: one
  # SET- to 48.989, then nothing (0.989 of a step "is no step") and the match never finished
  for set0 in (49.989, 53.989, 50.0):
    r = SS.street_sim(4000, lambda f: 45, set0, v_mph=36, street_mode=1, **SS._engage())
    assert abs(r["sets"][-1] - 48) < 0.02, (set0, r["sets"][-1])
    assert "done" in SS._reasons(r) and r["logic"].anchor is None, (set0, SS._reasons(r))
    assert min(r["sets"][SS.ENGAGE:]) >= 48 - 0.02                 # never under it
  # at the target already (49.989 for a +5 street's 50): nothing
  r = SS.street_sim(3000, lambda f: 45, 49.989, v_mph=36, street_mode=2, **SS._engage())
  assert r["presses"] == []


def test_r4_no_margin_on_a_street_target_and_curves_keep_theirs():
  base = dict(enabled=True, cruise_enabled=True, v_ego=40 * MPH, gas_pressed=False, brake_pressed=False, standstill=False,
              plan_alive=True, csc_active=True, other_button_active=False, curve_floor=25 * MPH, curve_budget=30)

  def first_press(set_mph, csc_mph, **kw):
    lg2 = cs.CurveSlowdownLogic(False)
    for _ in range(100):
      b, _ = lg2.update(cs.SlowdownInputs(set_speed=set_mph * MPH, csc_speed=csc_mph * MPH, **base, **kw))
      if b is not None:
        return b
    return None
  # a street target 48 with the set at 48.989: one SET- (whole steps); a camera curve's 48 keeps its margin: none
  assert first_press(48.989, 48.0, street_lower=True, curve_track=True) == cs.SET_MINUS
  assert first_press(48.989, 48.0) is None
  # the street target never goes under itself: at 47.989 nothing more
  assert first_press(47.989, 48.0, street_lower=True, curve_track=True) is None


# ---------------- R2: the merge taper only by the speed ----------------

TP = _load("trp_round_r", "test_round_p.py")


def test_r2_the_highway_match_right_after_the_loop_does_not_taper_at_34_mph():
  # drive 76 t 525.6: the motorway matched with the car at ~34 mph, 41 under the target 75 (before: the anchor 89 -> 80)
  car, lg, log = TP._boost73(inp=lambda f, car, lg: {"merge_taper": f >= 60})
  f65 = next(f for f, s, v, *_ in log if v >= 65)
  assert max(s for f, s, *_ in log if f < f65) >= 90 - 1e-6           # the full boost until 10 mph under the target
  assert all(a is None or a >= 90 * MPH - 1e-6 for f, s, v, b, act, a, _ in log if 100 <= f < f65 - 50)
  assert abs(car.set_speed / MPH - 75) < 1e-6


def test_r2_a_boost_ended_by_its_time_behind_a_moving_lead_goes_to_the_target():
  # a long ramp behind a car at 57 mph (it does not block the boost: moving, not closing): MERGE_BOOST_TIME ends it with
  # the car 18 mph under the target; the set comes down to the target 75 (not left high, not left low)
  def inp(f, car, lg):
    car.v_ego = min(car.v_ego, 57 * MPH)
    return {"radar_lead": True, "lead_v": 57 * MPH}
  car, lg, log = TP._boost73(frames=6000, inp=inp)
  ends = [f for f, s, v, b, *_ in log if b == cs.SET_MINUS]
  assert ends and ends[0] >= int(cs.MERGE_BOOST_TIME / 0.01), ends[:3]
  assert abs(car.set_speed / MPH - 75) < 1e-6 and min(s for f, s, *_ in log if f >= ends[0]) >= 75 - 1e-6


# ---------------- R3: the fast raise after our auto resume's RES ----------------

TCS = _load("tcs_round_r", "test_curve_slowdown.py")


def _acc(g):
  return 0.39 if g < 5 else (0.50 if g < 10 else (0.71 if g < 20 else 0.82))


def _res_after_our_stop(auto=True, gas_s=2.5, set0=19, anchor=53, goal=53, clean=25, stop_episode=True, extra=None,
                        frames=2000, lose=None):
  """Stopped at a light at the set our stop lowering left (set0 under his anchor), his brake, then the RES (ours when
  `auto`) at 10 mph on his gas, held gas_s; the car's speed follows his foot (+2 m/s^2), then the cruise."""
  car = TCS.Car(set0, dash_offset_mph=0.0, lag=10, accel_fn=_acc, cruise_gain=0.19, step=MPH, lose=lose)
  car.set_speed, car.v_ego = (set0 - 0.005) * MPH, 10 * MPH
  lg = cs.CurveSlowdownLogic(False)
  lg.anchor, lg.last_set_speed, lg.stop_episode, lg.cruise_gap = anchor * MPH, car.set_speed, stop_episode, True
  lg.lock_measured, lg.lock_overrun, lg.lock_clean = True, {cs.SET_MINUS: 0, cs.SET_PLUS: 1}, clean
  log, reach = [], None
  for f in range(frames):
    gas = f < gas_s * 100
    if gas:
      car.gas_frame = car.frame
    d = dict(enabled=True, cruise_enabled=True, set_speed=car.set_speed, v_ego=car.v_ego, gas_pressed=gas, brake_pressed=False,
             standstill=False, plan_alive=True, csc_active=False, csc_speed=0.0, other_button_active=f < 100,
             btn_counter=(f // 10) % 16, stop_clear_ahead=True, stop_pulling_away=True, engage_goal=goal * MPH,
             auto_resumed=auto and f == 0)
    if extra:
      d.update(extra(f, car))
    btn, _ = lg.update(cs.SlowdownInputs(**d))
    if btn is not None:
      car.hold_frame(btn) if lg.last_action == "hold" else car.press(btn)
    log.append((f, car.set_speed / MPH, car.v_ego / MPH, btn, lg.last_action))
    v0 = car.v_ego
    car.tick()
    if gas:
      car.v_ego = v0 + 2.0 * 0.01
    if reach is None and car.set_speed >= (goal - 0.6) * MPH:
      reach = f
  return car, lg, log, reach


def test_r3_our_auto_resume_raises_at_once_with_a_hold_under_his_gas_to_the_goal():
  car, lg, log, reach = _res_after_our_stop()
  assert reach is not None and reach <= 700, reach                      # (drive 76: 11-14 s; here <= 7 s)
  holds = [f for f, s, v, b, a in log if a == "hold"]
  assert holds and holds[0] < 250, holds[:3]                            # a SET+ hold under his gas (he is on it 2.5 s)
  assert max(s for _, s, *_ in log) <= 53 + 1e-6                        # never above the goal
  # the hold to 50 (no overrun margin: this drive's holds are clean), the last 3 mph by taps
  end = max(holds)
  assert abs(log[end + 160][1] - 50) < 0.6, log[end + 160][1]


def test_r3_his_own_res_is_unchanged_and_none_without_our_lowering():
  # his RES (not our auto resume's): exactly as before round R (the logic with ENGAGE_AUTO off)
  _, _, log1, r1 = _res_after_our_stop(auto=False)
  old = cs.ENGAGE_AUTO
  try:
    cs.ENGAGE_AUTO = False
    _, _, log0, r0 = _res_after_our_stop(auto=True)
  finally:
    cs.ENGAGE_AUTO = old
  assert [x[:4] for x in log1] == [x[:4] for x in log0] and r1 == r0
  assert r0 is None or r0 > 700
  # the set at his brake his own (no lowering of ours: no stop episode): no ENGAGE_AUTO raise
  _, lg, log, _ = _res_after_our_stop(stop_episode=False)
  assert lg.engage is None or not lg.engage.get("auto")
  # the street's target under his anchor: the goal is the lower one
  car, lg, log, reach = _res_after_our_stop(goal=48)
  assert max(s for _, s, *_ in log) <= 48 + 1e-6 and reach is not None


def test_r3_the_guards_stay_red_light_a_slow_car_ahead_a_curve_and_the_hold_margin_unless_measured():
  # the stop-light detector on: nothing up while it is (RED_STOP_BLOCK_MAX), then the raise
  car, lg, log, reach = _res_after_our_stop(extra=lambda f, car: dict(red_stop=f < 300, red_light=f < 300))
  assert not any(b == cs.SET_PLUS for f, s, v, b, a in log if f < 300)
  assert reach is not None
  # a slower car ahead (the stop trigger's clear / pulling away off): no raise of this kind
  car, lg, log, reach = _res_after_our_stop(extra=lambda f, car: dict(stop_clear_ahead=False, stop_pulling_away=False))
  assert not any(a == "hold" for *_, a in log)
  # a curve target under the goal: no raise to the goal
  car, lg, log, reach = _res_after_our_stop(extra=lambda f, car: dict(csc_active=True, csc_speed=30 * MPH, curve_floor=25 * MPH))
  assert max(s for _, s, *_ in log) <= 31 + 1e-6
  # this drive's holds not measured clean: the hold keeps its quantum of margin (stops at 45), the rest by taps
  car, lg, log, reach = _res_after_our_stop(clean=0)
  holds = [f for f, s, v, b, a in log if a == "hold"]
  assert holds and abs(log[max(holds) + 160][1] - 45) < 0.6, log[max(holds) + 160][1]


# ---------------- R5: an exit that ends on its ramp with no curve does not keep the highway's set ----------------

TE = _load("tep_round_r", "test_exit_plan.py")


def _exit_no_curve(ramp_to=45, set_mph=75, frames=12000, lead=40):
  """Navigation's exit with no curve on its ramp, the link matched with GPS trust 2 (unmatched) after it, the car behind
  a car at 40 mph that leaves 400 m past the exit point (drive 76 t 700-755, exit to Dobry Dr; synthetic)."""
  exit_x = 1500.0
  F, cs_ = TE.F, TE.cs

  def fields(x):
    hwy = dict(road_class=1, limit=70 * MPH, gps_trust=0)
    if x < exit_x - 700:
      return F(**hwy)
    if x < exit_x - 40:
      return F(nav_exit=1, nav_exit_distance=exit_x - x, nav_exit_side=1, exit_shape=1, exit_start=exit_x - x,
               ramp_type=1, ramp_to_speed=ramp_to * MPH, **hwy)
    if x < exit_x + 60:
      return F(exit_shape=0, **{**hwy, "gps_trust": 2})
    return F(exit_shape=2, road_class=2, gps_trust=2)
  real = cs_.adopt_forgets

  def wrapper_adopt(lg, mo, set_speed):            # (the wrapper's adopt: curve_slowdown.CurveSlowdown.update)
    if not real(lg, mo, set_speed):
      return False
    return not cs_.exit_ramp_handover(lg, mo, set_speed)
  cs_.adopt_forgets = wrapper_adopt
  try:
    r = TE.drive_exit(fields, frames, set_mph, v_mph=62, lane=lambda x: 1 if x < exit_x else None,
                      signal=lambda x: 1 if exit_x - 300 <= x < exit_x - 250 else 0,
                      lead=lambda x: lead * MPH if x < exit_x + 400 else None)
  finally:
    cs_.adopt_forgets = real
  return r, exit_x


def test_r5_the_exit_ending_on_its_ramp_hands_over_to_the_streets_target():
  r, exit_x = _exit_no_curve()
  assert (r["ctl"].last_plan.why == "on the ramp, no curve"), r["ctl"].last_plan.why
  end = next(k for k, xx in enumerate(r["xs"]) if xx >= exit_x)
  assert r["sets"][end] >= 59                                          # (the exit's own lowering left ~60)
  assert abs(r["sets"][-1] - 50) < 0.6, r["sets"][-1]                  # the announced 45 street + 5, by SET- taps
  assert not TE._plus(r, exit_x, 10 ** 6)
  assert r["logic"].anchor_ours or r["logic"].anchor is None
  # the car does not speed up toward the old set once the car ahead is gone: at most the street's target
  assert max(r["vs"][end:]) <= 50.5, max(r["vs"][end:])
  # (before round R: the 60 kept as his speed)
  old = TE.CTL.EXIT_RAMP_HANDOVER
  try:
    TE.CTL.EXIT_RAMP_HANDOVER = False
    r0, _ = _exit_no_curve()
  finally:
    TE.CTL.EXIT_RAMP_HANDOVER = old
  assert r0["sets"][-1] >= 59 and max(r0["vs"][end:]) > 55


def test_r5_the_streets_limit_unknown_takes_the_43_fallback_and_never_a_raise():
  r, exit_x = _exit_no_curve(ramp_to=0)
  assert abs(r["sets"][-1] - 43) < 0.6, r["sets"][-1]
  # the set already under the street's target (his 45 on the highway, a 55 street): left as it was
  r, exit_x = _exit_no_curve(ramp_to=55, set_mph=50)
  assert not TE._plus(r, exit_x, 10 ** 6) and r["sets"][-1] <= 50 + 1e-6
