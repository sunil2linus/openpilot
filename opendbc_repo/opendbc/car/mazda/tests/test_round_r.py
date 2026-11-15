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
W = _load("wcp_round_r", "test_car_profile_wrapper.py")


def _handover_run(after, set_mph=75, v_mph=62, frames=12000, lead=40, gas=None, driver=None, ramp_to=45, flag=True):
  """Navigation's exit with no curve on its ramp; after(d, F, hwy) gives the planner's fields d m past the exit point. The
  wrapper's adopt and its RampHandover (curve_slowdown.CurveSlowdown.update) on drive_exit's loop."""
  exit_x = 1500.0
  F, cs_ = TE.F, TE.cs
  st = {"f": None, "rh": None, "frame": 0}

  def fields(x):
    hwy = dict(road_class=1, limit=70 * MPH, gps_trust=0)
    if x < exit_x - 700:
      f = F(**hwy)
    elif x < exit_x - 40:
      f = F(nav_exit=1, nav_exit_distance=exit_x - x, nav_exit_side=1, exit_shape=1, exit_start=exit_x - x,
            ramp_type=1, ramp_to_speed=ramp_to * MPH, **hwy)
    else:
      f = after(x - exit_x, F, hwy)
    st["f"] = f
    return f
  real_adopt, real_update = cs_.adopt_forgets, cs_.CurveSlowdownLogic.update

  def wrapper_adopt(lg, mo, set_speed):
    if not real_adopt(lg, mo, set_speed):
      return False
    rh = cs_.exit_ramp_handover(lg, mo, set_speed, lg.last_set_speed and st["v"], lg.frame)
    if rh is not None:
      st["rh"] = rh
    return True                                      # (the set his, as before)

  def update(self, i):
    st["v"] = i.v_ego
    f = st["f"]
    if st["rh"] is not None and f is not None:
      if not st["rh"].step(self, self.frame, i.set_speed, i.v_ego, f.road_class, f.gps_trust, f.on_ramp, i.cruise_enabled,
                           i.gas_pressed, i.brake_pressed, i.driver_button is not None):
        st["rh"] = None
    return real_update(self, i)
  cs_.adopt_forgets, cs_.CurveSlowdownLogic.update = wrapper_adopt, update
  old = TE.CTL.EXIT_RAMP_HANDOVER
  TE.CTL.EXIT_RAMP_HANDOVER = flag
  try:
    r = TE.drive_exit(fields, frames, set_mph, v_mph=v_mph, lane=lambda x: 1 if x < exit_x else None,
                      signal=lambda x: 1 if exit_x - 300 <= x < exit_x - 250 else 0,
                      lead=(lambda x: lead * MPH if x < exit_x + 400 else None) if lead else None, gas=gas)
  finally:
    cs_.adopt_forgets, cs_.CurveSlowdownLogic.update = real_adopt, real_update
    TE.CTL.EXIT_RAMP_HANDOVER = old
  return r, exit_x


def _trusted_ramp(street_at=600.0, street_limit=45):
  """The exit's ramp matched (a link, the planner's ramp flag, GPS trust 0), a 45 street from street_at m past the exit point."""
  def after(d, F, hwy):
    if d < 60:
      return F(exit_shape=0, **{**hwy, "gps_trust": 2})
    if d < street_at:
      return F(exit_shape=2, road_class=2, gps_trust=0, on_ramp=True, ramp_type=1, ramp_to_speed=street_limit * MPH,
               ramp_end_distance=max(street_at - d, 0.0))
    return F(road_class=7, gps_trust=0, limit=street_limit * MPH)
  return after


def test_r5_the_exit_ending_on_its_ramp_hands_over_to_the_streets_target_on_a_trusted_ramp():
  r, exit_x = _handover_run(_trusted_ramp())
  assert r["ctl"].last_plan.why == "on the ramp, no curve", r["ctl"].last_plan.why
  k_exit = next(k for k, xx in enumerate(r["xs"]) if xx >= exit_x)
  assert r["sets"][k_exit] >= 59                                       # (the exit's own lowering left ~60)
  # on the ramp: never more than RAMP_HANDOVER_DROP under the speed at its start (~40 behind the car ahead: 30, so the 50)
  k_street = next(k for k, xx in enumerate(r["xs"]) if xx >= exit_x + 600)
  assert min(r["sets"][k_exit:k_street]) >= 50 - 0.6
  assert abs(r["sets"][-1] - 50) < 0.6, r["sets"][-1]                  # the announced 45 street + 5
  assert max(r["sets"][k_exit + 200:]) <= 60 + 0.6                     # never a raise by it
  # (before round R: the 60 kept as his speed on the ramp until the street cap 200 m before its end, the car speeding up
  # toward it once the car ahead left)
  r0, _ = _handover_run(_trusted_ramp(), flag=False)
  k0 = next(k for k, xx in enumerate(r0["xs"]) if xx >= exit_x + 380)
  k1 = next(k for k, xx in enumerate(r["xs"]) if xx >= exit_x + 380)
  assert r0["sets"][k0] >= 59 and r["sets"][k1] <= 51 + 0.6, (r0["sets"][k0], r["sets"][k1])
  # the street's limit unknown: the 43 fallback once on the street
  r, _ = _handover_run(_trusted_ramp(street_limit=0), ramp_to=0)
  assert abs(r["sets"][-1] - 43) < 0.6, r["sets"][-1]


# review of round R, F1: false exit ends (the car stays on the highway) and an untrusted link: no presses beyond the base's
FALSE_EXITS = {
  "shapes link, hwy trust2": lambda d, F, h: F(exit_shape=2 if d < 300 else 0, **{**h, "gps_trust": 2}),
  "shapes link, hwy trust2 class0": lambda d, F, h: F(exit_shape=2 if d < 300 else 0, road_class=0, gps_trust=2),
  "shapes link 100m, then hwy trust1": lambda d, F, h: F(exit_shape=2 if d < 100 else 0, **{**h, "gps_trust": 1}),
  "parallel ramp matched as link": lambda d, F, h: F(exit_shape=2, road_class=2, gps_trust=0, limit=70 * MPH),
  "a real ramp, GPS trust 2": lambda d, F, h: F(exit_shape=2, road_class=2, gps_trust=2, on_ramp=d > 100),
}


def test_r5_false_exit_ends_and_untrusted_links_press_nothing_beyond_the_base():
  for tag, aft in FALSE_EXITS.items():
    for gas in (None, lambda x: 72 * MPH if 2100 <= x < 2300 else None):
      r, exit_x = _handover_run(aft, set_mph=75, v_mph=70, lead=None, gas=gas, frames=9000)
      r0, _ = _handover_run(aft, set_mph=75, v_mph=70, lead=None, gas=gas, frames=9000, flag=False)
      assert [p[2] for p in r["presses"]] == [p[2] for p in r0["presses"]], tag
      assert abs(r["sets"][-1] - r0["sets"][-1]) < 1e-6, (tag, r["sets"][-1], r0["sets"][-1])


def test_r5_his_gas_or_his_button_cancels_it_for_good():
  # his gas held on the trusted ramp (> GAS_OVERTURN_TIME) before the street: nothing more of the handover afterwards
  r, exit_x = _handover_run(_trusted_ramp(street_at=900.0), gas=lambda x: 45 * MPH if 1650 <= x < 1750 else None)
  k = next(k for k, xx in enumerate(r["xs"]) if xx >= 1760)
  assert not [p for p in r["presses"] if p[0] > k + 100 and p[2] == TE.cs.SET_MINUS], r["presses"][-3:]
  # any button of his, his brake, cruise off: over at once (the set stays where it is)
  for kw in (dict(buttons=True), dict(brake=True), dict(cruise=False)):
    lg = cs.CurveSlowdownLogic(False)
    lg.last_set_speed = 60 * MPH
    rh = cs.RampHandover(48 * MPH, 40 * MPH, 0)
    args = dict(set_speed=60 * MPH, v_ego=40 * MPH, road_class=7, trust=0, on_ramp=False, cruise=True, gas=False, brake=False,
                buttons=False)
    args.update(kw)
    assert not rh.step(lg, 1, **args) and lg.anchor is None, kw


# ---------------- R1: the merge boost opened by the model at a loop's end ----------------

def _loop_end(gate_at=400, model_from=100, lat=lambda f: 2.1, pk=lambda f: 0.5, frames=1500, extra=None, v0=31.0):
  """The merge (75, boost +20) offered inside a loop, the set 31 at 31 mph; the turn gate ok only from gate_at (its pause
  latched from the loop until then); the model's conditions (merge_model_open) from model_from; the loop's own camera
  target at the car's speed meanwhile."""
  car = TP.MaxCar(31, dash_offset_mph=0.0, lag=10, accel_fn=TP._acc, cruise_gain=0.19, step=MPH)
  car.set_speed, car.v_ego = 31 * MPH, v0 * MPH
  lg = cs.CurveSlowdownLogic(False)
  log = []
  for f in range(frames):
    d = dict(set_speed=car.set_speed, v_ego=car.v_ego, btn_counter=(f // 10) % 16,
             limit_raise=75 * MPH if lg.boost is None and lg.limit_raised is None else 0.0,
             limit_boost=20 * MPH if lg.boost is None and lg.limit_raised is None else 0.0,
             merge_turn_ok=f >= gate_at, merge_turn_pause=f < gate_at, merge_turning=f < gate_at,
             csc_active=f < gate_at, csc_speed=v0 * MPH,
             merge_model_open=f >= model_from, merge_lat=lat(f), merge_model_pk=pk(f))
    if extra:
      d.update(extra(f, car))
    btn, _ = lg.update(TP._inp(**d))
    if btn is not None:
      car.hold_frame(btn) if lg.last_action == "hold" else car.press(btn)
    log.append((f, car.set_speed / MPH, car.v_ego / MPH, btn, lg.last_action, lg.anchor))
    car.tick()
  return car, lg, log


def _first_up(log):
  return next((f for f, s, v, b, *_ in log if b == cs.SET_PLUS), None)


def test_r1_the_model_opens_the_boost_before_the_turn_gate():
  car, lg, log = _loop_end()
  f_up = _first_up(log)
  assert f_up is not None and 100 <= f_up < 160, f_up                      # (the gate only at 400: 3 s later)
  assert max(s for f, s, *_ in log if f < 400) >= 45                        # the boost's SET+ hold goes on in the loop's end
  old = cs.MERGE_MODEL_OPEN
  try:
    cs.MERGE_MODEL_OPEN = False
    _, _, log0 = _loop_end()
  finally:
    cs.MERGE_MODEL_OPEN = old
  assert _first_up(log0) >= 400                                             # (before round R: at the gate)


def test_r1_a_second_bend_stops_it_and_takes_the_set_back():
  # the lateral rises 0.2 over its value at the start (an S-curve): stop, the set back to max(the speed, the set at the
  # start), the boost opens again on the turn gate
  car, lg, log = _loop_end(lat=lambda f: 2.1 if f < 250 else 2.4, gate_at=900)
  f_up = _first_up(log)
  peak = max(s for f, s, *_ in log if f < 400)
  assert f_up is not None and peak >= 35 - 1e-6
  assert any(b == cs.SET_MINUS for f, s, v, b, *_ in log if 250 <= f < 700)
  assert max(s for f, s, *_ in log if 700 <= f < 900) <= round(max(v for f, s, v, *_ in log if 250 <= f < 300)) + 1.0
  assert max(s for f, s, *_ in log if f >= 900) >= 75                       # up again once the gate is ok
  # the model's 0.5-3 s peak rising over the start lateral + 0.2: the same
  car, lg, log = _loop_end(pk=lambda f: 0.5 if f < 250 else 2.5, gate_at=900)
  assert any(b == cs.SET_MINUS for f, s, v, b, *_ in log if 250 <= f < 700)


def test_r1_its_guards_red_light_a_slow_lead_a_tighter_curve_target():
  for extra in (lambda f, car: dict(red_light=True),
                lambda f, car: dict(radar_lead=True, lead_v=8.0),
                lambda f, car: dict(csc_speed=24 * MPH)):                    # an unmapped tighter turn the camera sees
    car, lg, log = _loop_end(extra=extra)
    f_up = _first_up(log)
    assert f_up is None or f_up >= 400, f_up


def _merge_model_wrapper(tmp_path, monkeypatch, kappa=0.0, lat_deg=-36.0, lane=0.9, link=True, plan=None, rend=15.0):
  from types import SimpleNamespace as NS
  c = W.make_cs(tmp_path, W.FakeParams({"IsMetric": False}), monkeypatch)
  m = c.sm.msgs["modelV2"]
  xs = [190.0 * (j / 32) ** 2 for j in range(33)]
  m.position = NS(x=xs, y=[0.0] * 33)
  m.velocity = NS(x=[14.0] * 33, t=[0.0] * 33)
  m.orientationRate = NS(z=[(2.0 / 14.0 ** 2 if x < 20 else kappa) * 14.0 for x in xs])
  m.laneLineProbs = [0.5, lane, lane, 0.5]
  c.sm.updated["modelV2"] = True
  c.map_control.link_now = link
  c.map_control.hwy_ramp_left = rend
  p = c.sm.msgs["starpilotPlan"]
  for k, v in (plan or {}).items():
    setattr(p, k, v)
  CS = W.car_state(31.0, 31.0)
  CS.out.steeringAngleDeg, CS.out.steeringRateDeg = lat_deg, 0.0
  out = None
  for _ in range(60):
    c.frame += 1
    c.turn_gate.lat = 2.0
    c.steer_cmds.append(0.6)
    out = c._merge_model(CS, None, p, 14.0)
  return c, out


def test_r1_the_wrappers_model_view_and_its_guards(tmp_path, monkeypatch):
  _, out = _merge_model_wrapper(tmp_path, monkeypatch)
  assert out["merge_model_open"] and abs(out["merge_lat"] - 2.0) < 1e-6
  _, out = _merge_model_wrapper(tmp_path, monkeypatch, kappa=1.5 / 14.0 ** 2)     # still turning 2.5 s ahead: no
  assert not out["merge_model_open"]
  _, out = _merge_model_wrapper(tmp_path, monkeypatch, lane=0.2)
  assert not out["merge_model_open"]
  _, out = _merge_model_wrapper(tmp_path, monkeypatch, link=False)
  assert not out["merge_model_open"]
  # a tight ramp curve just ahead: no; a map curve we are in (the planner has not let the loop go: the next one is hidden): no
  near = dict(mapCurveStatus=2, mapCurveRadius=60.0, mapCurveDistance=30.0)
  assert not _merge_model_wrapper(tmp_path, monkeypatch, plan=near)[1]["merge_model_open"]
  inside = dict(mapCurveStatus=2, mapCurveRadius=70.0, mapCurveDistance=0.0, mapCurveApexValid=True, mapCurveApexDistance=-40.0)
  assert not _merge_model_wrapper(tmp_path, monkeypatch, plan=inside)[1]["merge_model_open"]
  # a gentle mapped curve far ahead (its plan above the speed + 5): yes
  far = dict(mapCurveStatus=2, mapCurveRadius=400.0, mapCurveDistance=300.0)
  assert _merge_model_wrapper(tmp_path, monkeypatch, plan=far)[1]["merge_model_open"]
  # a mapped curve ahead whose plan is under the speed + 5: no
  ahead = dict(mapCurveStatus=1, mapCurveRadius=60.0, mapCurveDistance=120.0)
  assert not _merge_model_wrapper(tmp_path, monkeypatch, plan=ahead)[1]["merge_model_open"]
  # the ramp's end far (a long ramp left: a turn the map lacks may come) or unknown: no
  assert not _merge_model_wrapper(tmp_path, monkeypatch, rend=300.0)[1]["merge_model_open"]
  # (F4) the ramp's end beyond the model's trusted horizon (31 mph x 3 s = 42 m; the end 30 m by the straight line, 1.3 x
  # that + 20 m to be seen): no
  assert not _merge_model_wrapper(tmp_path, monkeypatch, rend=30.0)[1]["merge_model_open"]
  assert not _merge_model_wrapper(tmp_path, monkeypatch, rend=None)[1]["merge_model_open"]
  # the stock LKAS blocked: no
  c, out = _merge_model_wrapper(tmp_path, monkeypatch)
  c.lkas_blocked = True
  assert not c._merge_model(W.car_state(31.0, 31.0), None, c.sm.msgs["starpilotPlan"], 14.0)["merge_model_open"]


def test_r1_the_ramps_end_distance_is_kept_when_the_ramp_flag_drops():
  # (drive 76 t 521.5: the planner's ramp flag dropped with the end 50 m ahead) the map control counts it on by the speed
  CTLm = TE.CTL
  ctl = CTLm.MapCurveControl(2.2, CTLm.MAP_DECEL)
  for _ in range(300):
    ctl.update(TE.F(road_class=2, on_ramp=True, ramp_type=2, ramp_to_highway=True, ramp_end_distance=50.0), 31 * MPH, 0,
               v_ego=14.0)
  for _ in range(100):
    ctl.update(TE.F(road_class=2, gps_trust=2), 31 * MPH, 0, v_ego=14.0)
  assert ctl.hwy_ramp_left is not None and abs(ctl.hwy_ramp_left - (50.0 - 14.0)) < 1.0, ctl.hwy_ramp_left


# ---------------- R7: the loop's second part, past the apex ----------------

CCm = _load("camera_curve_round_r", os.path.join("..", "camera_curve.py"))
TMC = _load("tmc_round_r", "test_map_curves.py")


def _loop_cam(seconds=6.0, v_mph=30.5, set_mph=30.0, lat=2.0, apex=-30.0, torque=0.6, peak_first=2.6, **kw):
  """In a loop's gentler part at v_mph, its measured lateral `lat` (after peak_first in its tight part for 2 s), the model's
  plan the same curve on, the governing map plan at the set speed (C6)."""
  cc = CCm.CameraCurve()
  v = v_mph * MPH
  k0 = lat / (v * v)
  xs = [190.0 * (j / 32) ** 2 for j in range(33)]
  ks = [k0] * 33
  set_v, taps = set_mph * MPH, []
  for f in range(int((2.0 + seconds) / 0.01)):
    tight = f < 200
    args = dict(v_ego=v, set_speed=set_v, lat=peak_first if tight else lat, model_x=xs, model_k=ks, lane_prob=0.8,
                torque=1.0 if tight else torque, curve_on=True, map_in_curve=True, map_curve_id=(2, 72), on_ramp=True,
                apex_d=(20.0 if tight else apex), gov_plan=set_mph * MPH, cam_target=set_mph * MPH, hwy_ramp=True)
    args.update(kw)
    o = cc.update(CCm.CamInputs(**args))
    if o.tap:
      taps.append(f)
      set_v = o.tap_goal
  return taps, set_v / MPH


def test_r7_past_the_apex_and_the_peak_taps_up_to_3_mph_over_the_plan():
  taps, s = _loop_cam()
  assert taps and taps[0] >= 300, taps[:2]                              # (the tight part 2 s, then the peak + 1 s)
  assert s <= 30.0 + 3 + 1e-6 and s >= 31.0 - 1e-6, s                    # up to the map plan + 3, the speed + 1 at most
  assert all(b - a >= 50 for a, b in zip(taps, taps[1:], strict=False))   # at most every 0.5 s


def test_r7_no_taps_before_the_apex_unknown_apex_too_little_reserve_or_the_ti_alone():
  # (F3) a ramp that does not lead to a highway: never
  for kw in (dict(apex=15.0), dict(apex=None), dict(torque=0.9), dict(lat=2.3), dict(ti_only=True), dict(hwy_ramp=False)):
    taps, s = _loop_cam(**kw)
    assert not taps, kw
  # the lateral not past its peak (the loop's second part as tight as the first): none
  taps, s = _loop_cam(peak_first=2.0)
  assert not taps
  # (round Q's rule without the apex passed: no tap above the governing map plan)
  old = CCm.CAM_POST_APEX
  try:
    CCm.CAM_POST_APEX = False
    taps, s = _loop_cam()
  finally:
    CCm.CAM_POST_APEX = old
  assert not taps


def test_r7_the_planner_publishes_a_ramp_curve_s_apex_once_inside_it():
  from types import SimpleNamespace as N
  import math
  m = TMC.MC.MapCurves()
  m.odo_ok, m.odo = True, 0.0
  # a 270 deg loop of radius 60 m around a synthetic origin, entered at its start; the apex at its middle
  R, lat0, lon0 = 60.0, 10.0, 20.0

  def ll(th):
    x, y = R * math.cos(th), R * math.sin(th)
    return lat0 + math.degrees(y / 6371000.0), lon0 + math.degrees(x / (6371000.0 * math.cos(math.radians(lat0))))
  c = {"s_a": 100.0, "s_b": 100.0 + 1.5 * math.pi * R, "len": 1.5 * math.pi * R, "ramp": True, "apex": ll(0.75 * math.pi),
       "a": ll(0.0), "b": ll(1.5 * math.pi), "r": R}
  m.store = N(curves={7: [c]}, ramps={})
  m.entered = {100.0: 0.0}
  seen = []
  for k in range(1, 28):
    th = 1.5 * math.pi * k / 28
    m.odo = R * th
    la, lo = ll(th)
    m.geom = {}
    m._curves_ahead(7, True, 0.0, 1000.0, la, lo, (90.0 - math.degrees(th + math.pi / 2)) % 360)
    g = m.geom.get(id(c))
    seen.append((round(th / (1.5 * math.pi), 2), None if g is None else round(g[0], 1)))
  before = [g for fr, g in seen if fr < 0.45]
  after = [g for fr, g in seen if fr > 0.75]
  assert before and all(g is not None and g > 0 for g in before), seen       # ahead, closing in
  assert after and all(g is not None and g < 0 for g in after), seen         # passed (the loop turns the bearing round)


# ---------------- R8: engaged with no GPS at all: the 43 fallback ----------------

_LI0 = SS.slf.LimitInputs


def _nogps_run(monkeypatch, set_mph, no_gps_until=3000, street_from=3000, limit=45, gps_bad=True, frames=6000, driver=None,
               gas_v=38):
  """Cruise on at frame ENGAGE (his SET at set_mph); no position (no_gps) until no_gps_until, nothing matched (road class 0)
  until street_from, then a street with `limit` (gps_bad False: GPS fine all along, the road simply not in our map)."""
  orig = _LI0
  st = {"f": 0}

  def li(*a, **k):
    return orig(*a, no_gps=bool(gps_bad and st["f"] < no_gps_until), **k)
  monkeypatch.setattr(SS.slf, "LimitInputs", li)

  def road(f):
    st["f"] = f
    return 7 if f >= street_from else 0
  # (his gas to a road's speed, 38 mph, for 6 s after his SET: review of round R, F2, NOGPS_MIN_SPEED)
  return SS.street_sim(frames, lambda f: limit if f >= street_from else 0, set_mph, v_mph=24, road=road, driver=driver,
                       gas=lambda f: gas_v * MPH if SS.ENGAGE <= f < SS.ENGAGE + 600 else None, **SS._engage())


def test_r8_no_gps_at_the_engagement_raises_to_43_then_the_streets_rules_take_over(monkeypatch):
  r = _nogps_run(monkeypatch, 19)
  at = r["sets"][2900]
  assert abs(at - 43) < 0.6, at                                         # the fallback, before any road was matched
  assert any("no GPS" in x for x in SS._reasons(r))
  assert abs(r["sets"][-1] - 50) < 0.6, r["sets"][-1]                   # the 45 street matched: + 5 (Street Speed)
  # a 25 street matched later: down to 30 (the 43 was ours)
  r = _nogps_run(monkeypatch, 19, limit=25)
  assert abs(r["sets"][2900] - 43) < 0.6 and abs(r["sets"][-1] - 30) < 0.6, (r["sets"][2900], r["sets"][-1])


def test_r8_never_with_gps_fine_on_an_unmapped_street_nor_over_his_higher_set_nor_after_his_set(monkeypatch):
  r = _nogps_run(monkeypatch, 19, gps_bad=False, street_from=10 ** 6)
  assert r["presses"] == [] and abs(r["sets"][-1] - 19) < 0.6           # a 25 mph subdivision street: his 19 stays
  r = _nogps_run(monkeypatch, 50, street_from=10 ** 6)
  assert not SS._plus(r) and not SS._minus(r)                           # his own higher set: never lowered by this
  # (F2) his gas only to 33 (drive 76's 23 -> 33 mph after his SET at 19): never at a road's speed for 3 s, nothing
  r = _nogps_run(monkeypatch, 19, gas_v=33, street_from=10 ** 6)
  assert not SS._plus(r), r["presses"][:3]
  # his SET+ after the fallback: his; the street matched later does not decide rule 8 again (rules 1 / 2 as always)
  r = _nogps_run(monkeypatch, 19, driver=lambda f: cs.SET_PLUS if f == 2000 else None)
  assert sum(1 for x in SS._reasons(r) if x.startswith("start: cruise on")) == 1, SS._reasons(r)
  assert not SS._minus(r, 3000)


def test_r8_the_wrapper_tells_no_gps_from_a_road_not_in_the_map(tmp_path, monkeypatch):
  from types import SimpleNamespace as NS
  c = W.make_cs(tmp_path, W.FakeParams({"IsMetric": False}), monkeypatch)
  P = lambda **k: NS(**{**dict(mapCurveMode=2, mapRoadClass=0, mapCurveStatus=0, mapGpsAccuracy=0.0), **k})  # noqa: E731
  assert c._no_gps(P(mapCurveStatus=3), True)                          # no fix yet: the planner has no position
  assert c._no_gps(P(mapGpsAccuracy=150.0), True)                      # a fix far too poor
  assert not c._no_gps(P(mapGpsAccuracy=6.0), True)                    # GPS fine, the road not in our map
  assert not c._no_gps(P(mapCurveStatus=3, mapGpsAccuracy=6.0), True)  # outside the curve files with a good fix
  assert not c._no_gps(P(mapCurveStatus=3, mapRoadClass=7), True)
  assert not c._no_gps(P(mapCurveStatus=3, mapCurveMode=0), True)      # Map Curves off: nothing known, no raise
  assert not c._no_gps(P(mapCurveStatus=3), False)


T8 = _load("tcs8_round_r", "test_curve_slowdown.py")


def _nogps_wrapper(tmp_path, monkeypatch, v0, engage_f=2000, frames=4000, acc_m=0.0, status=3, gas_until=None, gas_v=None,
                   gas_from=0):
  """(review of round R, F2) the real CurveSlowdown.update with Speed Limit Follow and Map Curves On: the drive's start with no
  fix (Map Curves On, status no data, accuracy 0, road class 0); his SET at v0 in a subdivision / lot, then maybe his gas."""
  from types import SimpleNamespace as NS
  from opendbc.car.mazda import mazdacan
  from opendbc.car.mazda.values import Buttons
  sent = []
  monkeypatch.setattr(mazdacan, "create_button_cmd", lambda packer, CP, ctr, btn: sent.append(btn) or ("btn", btn))
  monkeypatch.setattr(W.messaging, "SubMaster", lambda services: W.FakeSM())
  monkeypatch.setattr(W.cp, "PROFILE_PATH", str(tmp_path / "profile.json"))
  CP0 = NS(flags=W.MazdaSafetyFlags.GEN1 | W.MazdaSafetyFlags.TORQUE_INTERCEPTOR, carFingerprint="MAZDA_CX5", maxLateralAccel=1.09)
  c = cs.CurveSlowdown(CP0, W.FakeParams({"IsMetric": False}), curve_enabled=True, stop_mode=2, learning=False, map_mode=2,
                       limit_mode=2)
  if c.recorder is not None:
    c.recorder.root = str(tmp_path / "events")
  car = T8.Car(v0, dash_offset_mph=0.0, lag=10, accel_fn=_acc, cruise_gain=0.19, step=MPH)
  car.v_ego = v0 * MPH
  plan, m = c.sm.msgs["starpilotPlan"], c.sm.msgs["modelV2"]
  m.laneLineProbs, m.meta = [0.5, 0.9, 0.9, 0.5], NS(laneChangeState=0)
  xs = [190.0 * (j / 32) ** 2 for j in range(33)]
  CP = NS(wheelbase=2.7, steerRatio=15.5, carFingerprint="MAZDA_CX5")
  CCm = NS(enabled=True, cruiseControl=NS(cancel=False, resume=False), latActive=True, actuators=NS(torque=0.0))
  out, presses = [], []
  for f in range(frames):
    on = f >= engage_f
    if f == engage_f:
      car.set_speed = round(car.v_ego / MPH) * MPH
    plan.mapCurveMode, plan.mapCurveStatus, plan.mapGpsAccuracy, plan.mapRoadClass, plan.mapGpsTrust = 2, status, acc_m, 0, 0
    if f % 5 == 0:
      v = max(car.v_ego, 1.0)
      m.position, m.velocity, m.orientationRate = NS(x=xs, y=[0.0] * 33), NS(x=[v] * 33, t=[0.0] * 33), NS(z=[0.0] * 33)
      c.sm.updated["modelV2"] = c.sm.updated["starpilotPlan"] = True
    else:
      c.sm.updated["modelV2"] = c.sm.updated["starpilotPlan"] = False
    gas = gas_until is not None and engage_f + gas_from <= f < gas_until and on
    CS = W.car_state(car.v_ego / MPH, car.set_speed / MPH if on else 0.0, cruise=on, set_minus=engage_f <= f < engage_f + 10,
                     gas=gas)
    CS.crz_btns_counter = (f // 10) % 16
    nb = len(sent)
    c.update(CCm, CS, None, CP, False)
    if len(sent) > nb:
      btn = cs.SET_MINUS if sent[-1] == Buttons.SET_MINUS else cs.SET_PLUS
      presses.append((f, btn))
      car.hold_frame(btn) if c.logic.last_action == "hold" else car.press(btn)
    if gas:
      car.gas_frame = car.frame
    v_prev = car.v_ego
    car.tick()
    if not on:
      car.v_ego = v0 * MPH
    if gas and gas_v is not None:
      car.v_ego = min(gas_v * MPH, v_prev + 1.5 * 0.01)       # (his foot: +1.5 m/s^2 up to gas_v)
    out.append((f, car.set_speed / MPH if on else 0.0, car.v_ego / MPH))
  return out, presses


def test_r8_f2_no_raise_in_a_subdivision_or_a_lot_only_at_a_roads_speed(tmp_path, monkeypatch):
  # the reviewer's cases: his SET at 12-25 mph, or at 15 and then his gas to 26 / 30 mph: nothing
  for v0 in (12, 15, 19, 25):
    out, presses = _nogps_wrapper(tmp_path, monkeypatch, v0)
    assert not [p for p in presses if p[1] == cs.SET_PLUS], (v0, presses[:3])
  for gv in (26, 30):
    for gf in (500, 3000):
      out, presses = _nogps_wrapper(tmp_path, monkeypatch, 15, frames=gf + 4000, gas_until=2000 + gf + 1500, gas_v=gv,
                                    gas_from=gf)
      assert not [p for p in presses if p[1] == cs.SET_PLUS], (gv, gf, presses[:3])
  # his gas to 40 mph for 15 s (a road's speed): then the fallback, never above 43
  out, presses = _nogps_wrapper(tmp_path, monkeypatch, 19, frames=6000, gas_until=3500, gas_v=40)
  ups = [p for p in presses if p[1] == cs.SET_PLUS]
  assert ups and ups[0][0] >= 2000 + 700, ups[:2]
  assert max(s for _, s, _ in out) <= 43 + 0.6 and abs(out[-1][1] - 43) < 0.6, out[-1]
  # a usable fix with no road matched (outside the curve files): nothing
  out, presses = _nogps_wrapper(tmp_path, monkeypatch, 19, frames=6000, gas_until=3500, gas_v=40, acc_m=5.0)
  assert not [p for p in presses if p[1] == cs.SET_PLUS]


def test_r3_f7_no_fast_hold_with_a_short_model_path_or_a_lead_we_close_on(tmp_path, monkeypatch):
  # the guard off: taps only (no hold of the raise), never faster than before round R's taps
  car, lg, log, reach = _res_after_our_stop(extra=lambda f, car: dict(auto_hold_ok=f < 0))
  assert not [f for f, s, v, b, a in log if a == "hold"]
  _, _, log0, reach0 = _res_after_our_stop(auto=False)
  assert reach is None or reach0 is None or reach >= reach0 - 200
  # coming on during the hold: it stops there (taps from then)
  car, lg, log, reach = _res_after_our_stop(extra=lambda f, car: dict(auto_hold_ok=f < 250))
  assert not [f for f, s, v, b, a in log if a == "hold" and f >= 260]
  # the wrapper: the model's path under v x 4 s, or a lead within 80 m we close on (even one the cruise does not track)
  from types import SimpleNamespace as NS
  c = W.make_cs(tmp_path, W.FakeParams({"IsMetric": False}), monkeypatch)
  v = 15.0
  assert c._auto_hold_ok(190.0, NS(status=False, dRel=0.0, vLead=0.0), v)
  assert not c._auto_hold_ok(50.0, NS(status=False, dRel=0.0, vLead=0.0), v)
  assert not c._auto_hold_ok(190.0, NS(status=True, dRel=70.0, vLead=0.0), v)
  assert c._auto_hold_ok(190.0, NS(status=True, dRel=70.0, vLead=18.0), v)        # pulling away
  assert c._auto_hold_ok(190.0, NS(status=True, dRel=120.0, vLead=0.0), v)
