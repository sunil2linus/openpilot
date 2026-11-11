"""Round k (drives 6a / 6c / 6d): the logged scenarios as tests."""
import math

from opendbc.car.mazda import curve_slowdown as cs
from opendbc.car.mazda import speed_limit_follow as slf
from opendbc.car.mazda import stopped_car_slowdown as scs

MPH = cs.MS_PER_MPH


def _inp(**k):
  d = dict(enabled=True, cruise_enabled=True, set_speed=20 * MPH, v_ego=25 * MPH, gas_pressed=False, brake_pressed=False,
           standstill=False, plan_alive=True, csc_active=False, csc_speed=0.0, other_button_active=False)
  d.update(k)
  return cs.SlowdownInputs(**d)


# ---- 1: a SET+ tap of ours that jumps to the current speed is ours (drive 6a t 1084.6) ----

def _jump_run(land_mph_fn, frames=1500, anchor=45, curve=43):
  """Cruise back on at 20 with the anchor at 45 (our stop lowered it), the driver on the gas to 30 mph, then off it; a
  followed map curve at 43 (the street speed). Our first SET+ tap lands as a jump to land_mph_fn(v_mph); later taps
  step 1 mph."""
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp())
  lg.anchor = anchor * MPH
  for _ in range(300):
    lg.update(_inp(cruise_enabled=False, set_speed=0.0, enabled=False))
  set_mph, v, due, jumped, log = 20.0, 27.0, None, False, []
  for f in range(frames):
    gas = f < 100
    v = min(v + 0.012, 30.4) if gas else max(v - 0.004, 29.55)
    b, _ = lg.update(_inp(set_speed=set_mph * MPH, v_ego=v * MPH, gas_pressed=gas, csc_active=True, csc_speed=curve * MPH,
                          curve_track=True, curve_raise=True))
    if b == cs.SET_PLUS and due is None:
      due = f + 9
    if due is not None and f == due:
      set_mph = land_mph_fn(v) if not jumped else set_mph + 1
      jumped, due = True, None
    log.append((f, set_mph, lg.disabled))
  return lg, set_mph, log


def test_a_jump_just_under_the_current_speed_is_booked_as_ours():
  # the logged landing: 20 -> 28.993 with the car at ~29.5 (round(vEgo) 30: 1.007 steps under it)
  lg, set_mph, log = _jump_run(lambda v: 28.993)
  assert not lg.disabled
  assert any(s > 28 for _, s, _ in log)
  assert abs(set_mph - 43) < 0.5, set_mph            # carries on to the curve's (street) speed, never above it
  assert lg.anchor is not None and abs(lg.anchor / MPH - 45) < 0.5


def test_a_jump_one_step_above_the_current_speed_is_ours_more_is_not():
  lg, set_mph, _ = _jump_run(lambda v: math.floor(v) + 1.0)          # 30 at 29.55
  assert not lg.disabled and abs(set_mph - 43) < 0.5
  lg, set_mph, _ = _jump_run(lambda v: math.floor(v) + 2.0)    # 31 at 29.55: past the current speed + a step: still a disable
  assert lg.disabled


def test_a_jump_that_cannot_be_ours_is_his_never_a_disable():
  # nothing anchored (the controller forgot meanwhile): his speed from there, never switched off for the drive
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=20 * MPH, v_ego=29.5 * MPH))
  lg.inflight.append(cs.Press(cs.SET_PLUS, lg.frame))
  lg.update(_inp(set_speed=28.993 * MPH, v_ego=29.5 * MPH))
  assert not lg.disabled and lg.anchor is None and not lg.inflight


# ---- 2: stop with car: released once the car ahead moves on (no stop planned, no red light); red light protection ----


def _sw(**kw):
  base = dict(enabled=True, cruise_enabled=True, brake_pressed=False, v_ego=45 * MPH, lead_alive=True, lead_status=True,
              lead_prob=0.9, lead_d=110.0, lead_v=12 * MPH, lead_y=0.1, model_alive=True, model_accel=-0.6)
  base.update(kw)
  return scs.StoppedCarInputs(**base)


def _started():
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(_sw(lead_status=False, plan_stop=True, red_light=True))
  n = 0
  while not tr.update(_sw(plan_stop=True, red_light=True)):
    n += 1
    assert n < 200
  assert tr.kind == scs.STOP_WITH_CAR
  return tr


def test_6c_the_car_ahead_moving_on_after_a_green_releases_the_floor():
  # drive 6c t 772-784: the light off at 772.3, the plan off from 775, the model's desired acceleration still < 0 as we
  # slow at the floor; the car ahead 70-95 m away speeds up to 27-35 mph; we come down 46 -> 22.5 mph (driver's gas)
  tr = _started()
  v, lead, rel = 46.0, 12.0, None
  for f in range(1500):
    v = max(v - 0.024, 22.5)                        # ~1.07 m/s^2 at the floor
    lead = min(lead + 0.03, 34.0)
    if not tr.update(_sw(v_ego=v * MPH, lead_v=lead * MPH, lead_d=80.0, model_accel=-0.3, plan_stop=f < 300,
                         red_light=False)):
      rel = (f, v)
      break
  assert rel is not None and tr.release_reason == scs.CAR_AHEAD_MOVING and tr.restore_now, rel
  assert rel[1] > 28, rel                           # well before the 22.5 mph of the drive


def test_the_car_ahead_moving_does_not_release_while_a_stop_is_planned_the_light_is_on_or_it_is_much_slower():
  for kw in (dict(plan_stop=True), dict(red_light=True), dict(lead_v=20 * MPH), dict(lead_v=9 * MPH, v_ego=10 * MPH)):
    tr = _started()
    for _ in range(400):
      assert tr.update(_sw(**{**dict(v_ego=30 * MPH, lead_v=29 * MPH, model_accel=-0.3), **kw})), (kw, tr.release_reason)


def test_6a_584_no_traffic_moving_release_under_a_red_light_and_no_restore_until_it_has_gone():
  # drive 6a t 580-585: the plan blinking off under the light, the model's go: released "traffic moving again" with the
  # light on (and the set speed went back up toward the red light)
  tr = _started()
  for _ in range(500):                              # 5 s of the model's go, the light on, no plan: held
    assert tr.update(_sw(v_ego=30 * MPH, lead_v=29 * MPH, model_accel=0.4, red_light=True))
  for _ in range(200):                              # the plan gone STOP_AHEAD_RED_HOLD with the light on: released ...
    if not tr.update(_sw(v_ego=30 * MPH, lead_v=29 * MPH, model_accel=0.4, red_light=True)):
      break
  assert not tr.active and tr.release_reason == scs.TRAFFIC_MOVING
  assert not tr.restore_now and tr.hold_restore      # ... but the set speed stays down while the light is on
  restored = False
  for _ in range(150):                              # the light goes: the restore at once
    tr.update(_sw(v_ego=30 * MPH, lead_v=29 * MPH, model_accel=0.4, red_light=False))
    restored = restored or tr.restore_now
  assert restored and not tr.hold_restore


# ---- 3: a far camera-only stop ahead is staged on its extrapolated distance ----

def _sa(v_mph, **kw):
  base = dict(enabled=True, cruise_enabled=True, brake_pressed=False, v_ego=v_mph * MPH, lead_alive=True, lead_status=False,
              lead_prob=0.0, lead_d=0.0, lead_v=0.0, lead_y=0.0, model_alive=True, model_accel=-0.6, plan_stop=True,
              plan_ratio=0.4, plan_stop_d=120.0, set_speed=v_mph * MPH, red_light=True)
  base.update(kw)
  return scs.StoppedCarInputs(**base)


def _sa_run(v_mph, plan_fn, frames=600, **kw):
  tr = scs.StoppedCarTrigger()
  for _ in range(100):
    tr.update(_sa(v_mph, plan_stop=False, red_light=False))
  out, f0 = [], None
  for f in range(frames):
    act = tr.update(_sa(v_mph, plan_stop=plan_fn(f), **kw))
    if act and f0 is None:
      f0 = f
    out.append((f, act, tr.target / MPH, tr.floor_mode))
    if f0 is not None and not act:
      break
  return tr, out, f0


def test_a_real_red_far_away_takes_the_distance_target_then_the_floor_by_2_5_s():
  tr, out, f0 = _sa_run(50, lambda f: True)
  assert f0 is not None and tr.kind == scs.STOP_AHEAD and tr.far_staged
  first = out[f0][2]
  assert 30 <= first <= 38, first                  # d_stop ~142 m: ~34 mph, not the floor at once
  at = next(f for f, act, t, _ in out if act and abs(t * MPH - scs.FLOOR_SPEED) < 1e-6)
  assert at - f0 <= int(scs.FAR_STOP_STAGE / scs.DT_CTRL) + 1, at - f0
  assert tr.active


def _red_trace(v_mph, plan_fn, red_fn, frames, decel=0.0):
  """Replay-shaped trace (review of round k, rk/rep_far.py): a camera-only stop ahead at v_mph, the plan's stop on
  plan_fn(f), redLight red_fn(f); the car slowing at `decel` m/s^2 while an episode is on. Rows (f, active, target mph,
  restore_now, hold_restore, release reason, far stage)."""
  tr = scs.StoppedCarTrigger()
  for _ in range(100):
    tr.update(_sa(v_mph, plan_stop=False, red_light=False))
  v, out = v_mph * MPH, []
  for f in range(frames):
    act = tr.update(_sa(v / MPH, plan_stop=plan_fn(f), red_light=red_fn(f), set_speed=v_mph * MPH))
    out.append((f, act, tr.target / MPH, tr.restore_now, tr.hold_restore, tr.release_reason, tr.far_stage))
    if act:
      v = max(v - decel * 0.01, 8.0)
  return tr, out


def _share(pattern):
  """plan_fn: on for the first 60 frames (the start), then `pattern` (a string of 0/1, one char per 0.1 s, repeated)."""
  return lambda f: f < 60 or pattern[(f // 10) % len(pattern)] == "1"


def test_review_6a_580_a_vanishing_plan_under_a_red_light_never_restores_while_it_is_on():
  # the plan on 3 % of the frames after the start, the light on for 15 s: no restore (and never the floor) until it
  # goes off, then the restore at once
  tr, out = _red_trace(50, _share("0" * 33 + "1"), lambda f: f < 1500, 2500)
  assert not any(r for f, _, _, r, _, _, _ in out if f < 1500), [f for f, _, _, r, _, _, _ in out if r][:3]
  assert all(t > 25 for _, act, t, _, _, _, _ in out if act)            # (FAR_STOP_RED_SPEED: never the floor)
  assert any(r for f, _, _, r, _, _, _ in out if 1500 <= f < 1700)       # the light gone: back at once


def test_review_6c_768_a_third_of_the_plan_under_a_red_light_goes_to_the_floor_at_the_stage_end():
  # (second review, V2) the light on, started at 45 mph or more: a quarter of the plan confirms it (FAR_STOP_RED_PLAN_SHARE)
  tr, out = _red_trace(52, _share("100"), lambda f: True, 600)
  stage_end = next(f for f, act, _, _, _, _, far in out if act and not far and f > 100)
  assert all(act for f, act, *_ in out if 60 <= f < 600)
  assert abs(out[stage_end][2] - scs.FLOOR_SPEED / MPH) < 0.01, out[stage_end]
  assert 250 <= stage_end <= 320, stage_end                    # (the start ~0.5 s in, + FAR_STOP_STAGE)
  assert not any(r for _, _, _, r, _, _, _ in out)


def test_review_v2_the_stage_target_under_a_red_light_is_from_the_plans_own_distance():
  tr = scs.StoppedCarTrigger()
  for _ in range(100):
    tr.update(_sa(50, plan_stop=False, red_light=False))
  for _ in range(60):
    tr.update(_sa(50, red_light=True, plan_stop_d=100.0))
  assert tr.far_staged
  assert abs(tr.target - tr._far_target(100.0)) < 1e-6           # not d_stop's (farther, higher)
  assert tr._far_target(tr._d_stop(_sa(50, plan_stop_d=100.0))) > tr.target


def test_review_69_1193_a_blinking_plan_under_a_red_light_re_arms_and_holds():
  # the plan on 58 % (short gaps): an early release "gone before the warning" must not restore; it re-arms and the stage
  # target holds until the car ahead is found
  tr, out = _red_trace(53, _share("1101101100110"), lambda f: True, 800, decel=0.8)
  assert not any(r for _, _, _, r, _, _, _ in out)
  last_on = max(f for f, act, *_ in out if act)
  assert last_on >= 700 and out[-1][1], [(f, why) for f, act, _, _, _, why, _ in out if not act and why][:3]


def test_review_a_vanishing_plan_with_no_red_light_at_50_mph_restores_after_the_stage():
  tr, out, f0 = _sa_run(50, lambda f: f < 55 or f == 200, red_light=False)
  assert not tr.active and tr.restore_now and not tr.hold_restore
  assert out[-1][0] - f0 <= int(scs.FAR_STOP_STAGE / scs.DT_CTRL) + 1
  assert min(t for _, act, t, _ in out if act) > 25    # never the floor


def test_a_near_stop_ahead_goes_to_the_floor_at_once():
  tr, out, f0 = _sa_run(50, lambda f: True, frames=200, plan_stop_d=50.0, plan_ratio=0.0)
  assert not tr.far_staged and abs(out[f0][2] * MPH - scs.FLOOR_SPEED) < 1e-6


def test_the_same_vanishing_plan_at_40_mph_with_the_light_on_is_still_held():
  tr, out, f0 = _sa_run(40, lambda f: f < 55, frames=400)
  assert tr.active and abs(tr.target - scs.FLOOR_SPEED) < 1e-6     # the red light rules, as before


# ---- 4: a curve pass that ends in a stop after the curve still counts ----

def _tmc():
  import importlib.util
  import os
  spec = importlib.util.spec_from_file_location("tmc_rk", os.path.join(os.path.dirname(__file__), "test_map_curves.py"))
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


def test_6a_west_utica_a_pass_ending_at_a_red_light_after_the_curve_is_full_and_clean(tmp_path):
  TMC = _tmc()
  MC = TMC.MC
  arc = math.radians(40) * 250
  c = TMC._curve(0, 150, arc, 40, 116.6)
  end = 150 + arc

  def speed(x):                           # 38 mph through the curve, then braking hard for the light 5 m past its end
    return 17.0 if x < end + 5 else max(3.0, 17.0 - (x - end - 5) * 2.5)
  mp = TMC._drive_road([(150, 0.0), (arc, 1 / 250), (120, 0.0)], [c], tmp_path, speed=speed)
  e = mp.mc.memory._find(*c["apex"])
  assert e and len(e["passes"]) == 1, getattr(mp, "pass_drops", [])
  r, _, v, flags = e["passes"][0]
  assert abs(r - 250) < 15 and "full" in flags and abs(v - 17.0) < 0.5, e["passes"]
  # ... so a cap on it can lift (CAP_LIFT_MARGIN): the same pass with a cap of 37 mph
  mem = MC.CurveMemory(str(tmp_path / "cap.json"))
  mem.add_saturated_pass(c, 41.1 * MC.MPH, t=1000.0)
  cap0 = mem.speed_cap(c)
  mem.add_pass(c, r, t=5000.0, v=v, flags=flags)
  assert mem.speed_cap(c) >= cap0 + MC.CAP_JUMP - 0.02


def test_6a_243_the_in_curve_raise_reaches_the_cap_not_a_step_under_it():
  # set 35.99 (dash), the followed curve's command 36.977 (the cap 37): raised to 37, never to 38
  lg = cs.CurveSlowdownLogic(False, 10)
  st = {"set": 35.99}
  kw = dict(v_ego=36.5 * MPH, csc_active=True, csc_speed=36.977 * MPH, curve_track=True, curve_raise=True)
  lg.update(_inp(set_speed=st["set"] * MPH, **kw))
  lg.anchor = 44.993 * MPH
  due = None
  for f in range(1500):
    b, _ = lg.update(_inp(set_speed=st["set"] * MPH, **kw))
    if b == cs.SET_PLUS and due is None:
      due = f + 9
    if due == f:
      st["set"] += 1.0
      due = None
  assert abs(st["set"] - 36.99) < 1e-6, st


# ---- 5: hold misses only for holds that ran and were not cut; landings credited to the hold being sent ----

def _stop_hold_run(cut_after=None, frames=600):
  lg = cs.CurveSlowdownLogic(False, 10)
  st = dict(set_speed=45 * MPH, v_ego=45 * MPH, stop_active=True, stop_target=scs.FLOOR_SPEED, stop_urgent=True)
  lg.update(_inp(**st))
  started = None
  for f in range(frames):
    brake = cut_after is not None and started is not None and f >= started + cut_after
    lg.update(_inp(brake_pressed=brake, cruise_enabled=not brake, **st))
    if started is None and lg.hold is not None:
      started = f
  return lg, started


def test_6c_134_a_hold_cut_by_the_brake_after_0_1_s_is_no_miss():
  lg, started = _stop_hold_run(cut_after=10)
  assert started is not None and lg.hold_misses == 0 and lg.holds_ok, lg.holds_off_why


def test_a_hold_that_ran_its_second_with_nothing_landing_is_still_a_miss():
  lg, started = _stop_hold_run()
  assert started is not None and lg.hold_misses >= 1 and "no effect" in lg.holds_off_why


def test_a_hold_that_never_sent_a_frame_leaves_nothing_in_flight():
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=45 * MPH, v_ego=45 * MPH))
  lg._start_hold(cs.SET_MINUS)
  lg._stop_hold()
  assert not lg.inflight and lg.pending is None and lg.hold_log[-1].get("never_sent")


def test_6a_1031_the_hold_being_sent_takes_its_own_landings():
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=43 * MPH, v_ego=40 * MPH))
  lg.anchor = 45 * MPH
  a = cs.Press(cs.SET_MINUS, lg.frame, reserve=11, remaining=11, hold=True, frames_sent=3, release_frame=lg.frame, landed=1)
  b = cs.Press(cs.SET_MINUS, lg.frame, reserve=11, remaining=11, hold=True, frames_sent=2, release_frame=lg.frame)
  lg.inflight = [a, b]
  lg.hold = b
  lg.update(_inp(set_speed=38 * MPH, v_ego=40 * MPH, stop_active=True, stop_target=scs.FLOOR_SPEED, stop_urgent=True))
  assert b.landed == 5 and a.landed == 1 and not lg.disabled


# ---- 6: the ramp start (rule 5: any engagement / SET+ / RES on the link below the merge's target; the turn gate) ----


WB = 2.7


def _angle_for(lat, v):
  """Steering wheel angle (deg) that reads `lat` m/s^2 at v m/s (cs.steer_lateral_accel inverted)."""
  return math.degrees(lat * cs.MERGE_TURN_RATIO * WB * (1 + cs.MERGE_TURN_SLIP * v * v) / max(v * v, 0.1))


def _acc(g):
  return 0.33 if g < 5 else (0.51 if g < 10 else (0.75 if g < 15 else (0.91 if g < 20 else 1.06)))


def _ramp_run(frames, lat_fn=lambda f, v: 0.0, ev=None, anchor=45, resume_at=150, red=lambda f: False, plan=lambda f: False):
  """A street, then from frame 100 a link to a 70 highway (next limit 70 known on it). Cruise off at set 19 with our old
  anchor 45 (a stop lowering it), RES at resume_at on the link; lat_fn(f, v): the car's lateral acceleration (the
  steering). ev(f, st, car): the driver. The follower, the controller and the turn gate as the wrapper couples them."""
  T = _tmc().T
  car = T.Car(19, dash_offset_mph=0.0, lag=10, accel_fn=_acc, cruise_gain=0.19)
  car.v_ego = 21 * MPH
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=19 * MPH, v_ego=car.v_ego))
  lg.anchor = anchor * MPH
  fol = slf.SpeedLimitFollower(False)
  gate = cs.MergeTurnGate()
  st = dict(cruise=False, btn=None, brake=False)
  log, offers, prev_angle = [], [], 0.0
  for f in range(frames):
    if f == resume_at:
      st["cruise"] = True
    if ev:
      ev(f, st, car)
    btn_now = st["btn"] or car.driver_button_now()
    lat = lat_fn(f, car.v_ego)
    angle = _angle_for(lat, car.v_ego)
    ok, pause = gate.update(car.v_ego, angle, (angle - prev_angle) / 0.01, WB)
    prev_angle = angle
    if lg.limit_adopted is not None:
      fol.mark_adopted()
    on = st["cruise"]
    d = (lg.anchor if lg.anchor is not None else car.set_speed) if on else 0.0
    road = dict(road_class=7) if f < 100 else dict(road_class=2, ramp_to_highway=True, next_limit=70 * MPH,
                                                    next_distance=max(900.0 - (f - 100) * car.v_ego * 0.01, 0.0))
    want = fol.update(slf.LimitInputs(0.0, "Map Data", 0, d, on, v_ego=car.v_ego, driver_set=btn_now is not None,
                                      driver_minus=btn_now == cs.SET_MINUS, **road))
    o = fol.offer
    if o is not None and (not offers or offers[-1] is not o):
      offers.append(o)
    boost = cs.MERGE_BOOST if (o is not None and o.kind == "merge") else 0.0
    b, _ = lg.update(_inp(cruise_enabled=on, set_speed=car.set_speed if on else 0.0, v_ego=car.v_ego,
                          brake_pressed=st["brake"], driver_button=btn_now, limit_raise=want, limit_boost=boost,
                          merge_turn_ok=ok, merge_turn_pause=pause, red_light=red(f), ramp_start=fol.ramp_start,
                          model_plan_stop=plan(f)))
    if b is not None and on:
      car.hold_frame(b) if lg.last_action == "hold" else car.press(b)
    if on:
      car.tick()
    else:
      car.frame += 1
    log.append((f, car.set_speed / MPH, car.v_ego / MPH, b, lg.last_action, ok, pause))
  return car, lg, log, offers


def _driver_hold_to(top_mph, start):
  """His SET+ held from `start` until the set speed reaches top_mph (the car's long press: 5 mph every 0.6 s)."""
  def ev(f, st, car):
    if f >= start and car.set_speed < (top_mph - 0.5) * MPH and not st.get("held"):
      st["btn"] = cs.SET_PLUS
      if (f - start) % 10 == 0:
        car.hold_frame(cs.SET_PLUS)
    elif f >= start and st["btn"] is not None:
      st["btn"], st["held"] = None, True
  return ev


def test_6a_329_resume_at_19_on_the_link_then_his_set_plus_hold_to_55_gives_the_merge():
  car, lg, log, offers = _ramp_run(7000, ev=_driver_hold_to(55, 200))
  merges = [o for o in offers if o.kind == "merge"]
  assert merges and merges[0].target == 75, [(o.kind, o.target) for o in offers]
  sets = [s for _, s, _, _, _, _, _ in log]
  assert max(sets) >= 90 - 1e-6 and max(sets) <= 95 + 1e-6, max(sets)     # the boost (75 + 20)
  assert abs(sets[-1] - 75) < 1e-6, sets[-1]                               # back to the merge's target
  assert not lg.disabled


def test_engaged_mid_turn_at_22_mph_the_merge_waits_for_a_calm_wheel():
  # a 90 degree turn onto the ramp, 3 m/s^2 until frame 400, then straight: nothing of the merge before the wheel has
  # been calm (< MERGE_TURN_LAT) for MERGE_TURN_CALM
  car, lg, log, offers = _ramp_run(3000, lat_fn=lambda f, v: 3.0 if f < 400 else 0.0, resume_at=150)
  first = next((f for f, _, _, b, _, _, _ in log if b == cs.SET_PLUS), None)
  assert first is not None and first >= 400 + 99, first
  assert first <= 400 + 100 + 60, first                                    # ... and then at once


def test_a_90_degree_turn_peaking_at_3_4_then_straight_starts_about_a_second_after_the_wheel_is_straight():
  gate = cs.MergeTurnGate()
  v, prev, ok_at = 23 * MPH, 0.0, None
  for f in range(600):
    t = f * 0.01
    lat = 3.4 * math.sin(math.pi * t / 3.0) if t < 3.0 else 0.0           # 3 s turn, then straight
    a = _angle_for(lat, v)
    ok, _ = gate.update(v, a, (a - prev) / 0.01, WB)
    prev = a
    if ok and ok_at is None:
      ok_at = t
  straight = next(f * 0.01 for f in range(300) if 3.4 * math.sin(math.pi * f * 0.01 / 3.0) < 1.0 and f > 150)
  assert ok_at is not None and 3.0 + 0.0 <= ok_at <= 3.0 + 1.0 and ok_at >= straight + 1.0 - 0.02, (ok_at, straight)


def test_turn_then_2_s_straight_then_a_loop_the_merge_pauses_at_the_loop_and_goes_on_after_it():
  def lat(f, v):
    t = f * 0.01
    if t < 3.0:
      return 3.0                                     # the turn onto the ramp
    if t < 5.0:
      return 0.0                                     # 2 s straight: the merge starts
    if t < 15.0:
      return min(3.5, (t - 5.0) * 3.0)               # the loop
    return 0.0
  car, lg, log, offers = _ramp_run(4000, lat_fn=lat, resume_at=100)
  ups = [f for f, _, _, b, _, _, _ in log if b == cs.SET_PLUS]
  assert any(f < 500 for f in ups), ups[:5]                                # started on the straight
  loop = [f for f in ups if 560 <= f < 1500]                               # from the loop's onset to its end
  assert loop == [], loop[:5]
  assert any(f >= 1500 for f in ups)                                       # goes on after it
  sets = [s for _, s, _, _, _, _, _ in log]
  assert all(sets[k] >= sets[k - 1] - 1e-6 for k in range(560, 1500))      # paused, never lowered


def test_a_long_loop_ramp_no_merge_until_it_ends():
  car, lg, log, offers = _ramp_run(5000, lat_fn=lambda f, v: 2.0 + 0.9 * math.sin(f * 0.01) if f < 150 + 2800 else 0.0,
                                   ev=_driver_hold_to(30, 160))
  ups = [f for f, _, _, b, act, _, _ in log if b == cs.SET_PLUS and f > 400]
  assert ups and ups[0] >= 150 + 2800 + 99, ups[:3]


def test_his_set_minus_on_the_link_ends_the_merge_and_a_red_light_holds_it():
  def ev(f, st, car):
    if f == 600:
      car.press(cs.SET_MINUS, who="driver")
  car, lg, log, offers = _ramp_run(3000, ev=ev)
  assert not any(o.kind == "merge" and offers.index(o) >= 0 and o.frame > 600 for o in offers)
  assert max(s for f, s, *_ in log if f > 700) <= 45 + 1e-6
  # the light on: no merge start; it goes once the light is off
  car, lg, log, offers = _ramp_run(3000, red=lambda f: f < 1500)
  ups = [f for f, _, _, b, _, _, _ in log if b == cs.SET_PLUS]
  assert ups and ups[0] >= 1500, ups[:3]


# ---- 7: RES after our own stop lowering brings his speed back with the fast restore ----

def _resume_run(frames=4000, off_for=3000, red=lambda f: False, stop_off_ago=10.0, clear=True, changed=False):
  T = _tmc().T
  car = T.Car(19, dash_offset_mph=0.0, lag=10, accel_fn=_acc, cruise_gain=0.19)
  car.v_ego = 15 * MPH
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=19 * MPH, v_ego=car.v_ego))
  lg.anchor, lg.stop_episode = 45 * MPH, True                 # our stop slowdown left 19 (his 45)
  lg.stop_off_frame = lg.frame - int(stop_off_ago * 100)
  lg.pending_from = None
  log = []
  for f in range(frames):
    on = f >= off_for
    b, _ = lg.update(_inp(cruise_enabled=on, set_speed=car.set_speed if on else 0.0, v_ego=car.v_ego, red_light=red(f),
                          red_stop=red(f), road_changed=changed,
                          enabled=on, stop_clear_ahead=clear))
    if b is not None and on:
      car.hold_frame(b) if lg.last_action == "hold" else car.press(b)
    if on:
      car.tick()
    else:
      car.frame += 1
    log.append((f, car.set_speed / MPH, b, lg.last_action))
    if lg.restore_pending and lg.pending_from is None:
      lg.pending_from = f
  return car, lg, log


def test_res_long_after_our_stop_lowering_restores_his_speed_with_a_hold():
  car, lg, log = _resume_run()                                # 30 s cruise off (his brake), then RES at 19
  first = next(f for f, _, b, _ in log if b == cs.SET_PLUS)
  assert first - 3000 <= 60, first                             # at once (cruise settle)
  assert any(act == "hold" for _, _, b, act in log if b == cs.SET_PLUS)
  assert abs(car.set_speed / MPH - 45) < 1e-6 and max(s for _, s, _, _ in log) <= 45 + 1e-6
  car, lg, log = _resume_run(clear=False)                     # a slower car in our lane: the gentle taps, no hold
  assert not any(act == "hold" for _, _, b, act in log if b == cs.SET_PLUS)


def test_res_under_a_red_light_or_right_after_the_stop_waits():
  car, lg, log = _resume_run(red=lambda f: 2950 <= f < 3500)
  assert next(f for f, _, b, _ in log if b == cs.SET_PLUS) >= 3500
  car, lg, log = _resume_run(off_for=200, stop_off_ago=1.0)    # RES 3 s after the stop's release: the post-stop wait
  assert lg.pending_from is not None and lg.pending_from >= 400 - 2, lg.pending_from
  assert abs(car.set_speed / MPH - 45) < 1e-6
  car, lg, log = _resume_run(red=lambda f: 2950 <= f < 3500)
  assert lg.pending_from is not None and lg.pending_from >= 3500, lg.pending_from


# ---- 8 / 9: logging (wheel button, driverSetChange with no anchor, unattributed changes, the speed limit events) ----

class _Rec:
  def __init__(self):
    self.events = []

  def record(self, kind, t, summary, min_gap=None, signals=True):
    self.events.append((kind, summary))
    return True


def _wrapper(limit=70, road_class=1):
  from types import SimpleNamespace as N
  w = cs.CurveSlowdown.__new__(cs.CurveSlowdown)
  w.logic = cs.CurveSlowdownLogic(False, 10)
  w.limit_follow = slf.SpeedLimitFollower(False)
  w.limit_follow.stable = limit
  w.recorder = _Rec()
  w.unattributed_high_frames = 0
  w.unattributed_high_last = -1_000_000
  w.limit_mode, w.limit_want = 2, 0.0
  w.limit_cap_logged = w.limit_ceiling_logged = None
  return w, N(mapRoadClass=road_class, slcSpeedLimit=limit * MPH)


def test_8a_the_wheel_button_state_is_the_cars_own_switch():
  from types import SimpleNamespace as N
  from opendbc.car.interfaces import mazda_wheel_button
  assert mazda_wheel_button(N(crz_set_plus=False, crz_set_minus=False, crz_resume=False, crz_cancel=False)) == 0
  assert mazda_wheel_button(N(crz_set_plus=True, crz_set_minus=False, crz_resume=False, crz_cancel=False)) == 1
  assert mazda_wheel_button(N(crz_set_plus=False, crz_set_minus=True, crz_resume=False, crz_cancel=False)) == 2
  assert mazda_wheel_button(N(crz_set_plus=False, crz_set_minus=False, crz_resume=True, crz_cancel=False)) == 3
  assert mazda_wheel_button(N(crz_set_plus=True, crz_set_minus=False, crz_resume=True, crz_cancel=True)) == 4
  assert mazda_wheel_button(N()) == 0                              # (not a Mazda)


def test_8b_driver_set_changes_are_logged_with_no_anchor_at_most_every_second():
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=40 * MPH, v_ego=40 * MPH))
  n = 0
  for f in range(300):
    btn = cs.SET_PLUS if f % 20 < 2 else None
    s = 40 + (f // 20) + (1 if f % 20 >= 1 else 0)
    lg.update(_inp(set_speed=s * MPH, v_ego=40 * MPH, driver_button=btn))
    n += len(lg.driver_events)
    lg.driver_events = []
  assert 2 <= n <= 4, n                                            # 15 taps in 3 s: one a second


def test_8c_9_an_unexplained_raise_is_logged_and_chimes_when_high_on_a_highway():
  w, plan = _wrapper(limit=70, road_class=1)
  lg = w.logic
  lg.update(_inp(set_speed=72 * MPH, v_ego=70 * MPH))
  for _ in range(100):
    lg.update(_inp(set_speed=72 * MPH, v_ego=70 * MPH))
  i = _inp(set_speed=80 * MPH, v_ego=70 * MPH)
  lg.update(i)                                                     # 72 -> 80: nothing of ours, no button
  assert lg.unattributed == (72 * MPH, 80 * MPH)
  w._unattributed_change(i, plan)
  assert w.recorder.events and w.recorder.events[-1][0] == "unattributedSetChange"
  assert w.recorder.events[-1][1]["high_alert"] and w.set_speed_high
  for _ in range(150):
    lg.update(i)
    w._unattributed_change(i, plan)
  assert not w.set_speed_high                                      # the flag drops (chime once), nothing pressed
  # his own SET+ tap, or our press: not unexplained; on a street: no chime
  w, plan = _wrapper(limit=45, road_class=5)
  lg = w.logic
  lg.update(_inp(set_speed=45 * MPH, v_ego=45 * MPH))
  for _ in range(100):
    lg.update(_inp(set_speed=45 * MPH, v_ego=45 * MPH))
  lg.update(_inp(set_speed=45 * MPH, v_ego=45 * MPH, driver_button=cs.SET_PLUS))
  lg.update(_inp(set_speed=46 * MPH, v_ego=45 * MPH))
  assert lg.unattributed is None
  for _ in range(100):
    lg.update(_inp(set_speed=46 * MPH, v_ego=45 * MPH))
  i = _inp(set_speed=60 * MPH, v_ego=45 * MPH)
  lg.update(i)
  w._unattributed_change(i, plan)
  assert lg.unattributed is not None and not w.set_speed_high


def test_8d_6a_329_a_follower_event_with_no_new_limit_no_longer_blocks_the_speed_limit_log():
  w, _ = _wrapper()
  lf = w.limit_follow
  lf.events = [("speedLimitNotFollowed", {"old": None, "new": None, "source": "Map Data",
                                          "reason": "merge: the driver set his speed on the link"})]
  i = _inp(set_speed=45 * MPH, v_ego=20 * MPH)
  w._limit_bookkeeping(i)
  assert lf.events == [] and w.recorder.events[-1][0] == "speedLimitNotFollowed"
  assert w.recorder.events[-1][1]["new_mph"] is None


def test_review_s1_a_red_light_on_the_ramp_pauses_the_merge_a_stuck_flag_does_not():
  # (rk/ramp_red.py) a ramp meter's light comes on mid-merge and stays: the way up pauses (the set kept, no SET-);
  # on continuously MERGE_RED_STUCK at speed with no stop planned: a stuck flag, the merge goes on
  car, lg, log, offers = _ramp_run(3200, red=lambda f: f >= 500)
  sets = {f: s for f, s, *_ in log}
  assert not [b for f, _, _, b, _, _, _ in log if b == cs.SET_MINUS]
  assert abs(sets[2400] - sets[620]) < 1e-6, (sets[620], sets[2400])
  assert sets[3100] > sets[2400] + 10, sets[3100]


def test_review_s2_his_steering_in_the_curves_tail_still_spoils_the_pass(tmp_path):
  # (rk/pass_cut.py) the map curve's end, then 30 m tighter where he grabs the wheel: as before, no pass (the stop cut
  # is for braking / slowing only)
  from types import SimpleNamespace as N
  TMC = _tmc()
  MC = TMC.MC

  class P:
    def get(self, k, encoding=None):
      return None
  arc = math.radians(40) * 250
  c = TMC._curve(0, 150, arc, 40, 116.6)
  end = 150 + arc
  mp = MC.MapCurvesPlanner(P(), P())
  mp.mc = TMC._FakeMC(MC.CurveMemory(str(tmp_path / "m.json")))
  mp.trust = N(shaky=False)
  mp.pass_curve, mp.pass_samples = None, []
  v, x, frame, gx = 17.0, 0.0, 0, -1e9
  segs, s0 = [], 0.0
  for length, k in ((150, 0.0), (arc, 1 / 250), (30, 1 / 80), (120, 0.0)):
    segs.append((s0, s0 + length, k))
    s0 += length
  while x < s0:
    k = next(kk for a, b, kk in segs if a <= x < b)
    if frame % 20 == 0:
      gx = x - v * MC.PASS_GPS_LAG
    inside = c if c["x0"] + MC.NEAR < gx < c["x1"] - MC.NEAR else None
    mp.mc.inside, mp.mc.inside_exit = inside, (c["x1"] - gx) if inside is not None else float("inf")
    mp.counter = frame
    lat = c["apex"][0] + (gx - (c["x0"] + c["x1"]) / 2) / 111320.0
    sm = {"carState": N(vEgo=v, steeringPressed=end + 3 <= x < end + 33, leftBlinker=False, rightBlinker=False,
                        brakePressed=False),
          "carControl": N(latActive=True, actuators=N(torque=0.4)), "controlsState": N(curvature=k)}
    mp._measure_pass(sm, inside, lat, c["apex"][1], dt=MC.PASS_DT)
    x += v * MC.PASS_DT
    frame += 1
  e = mp.mc.memory._find(*c["apex"])
  assert not (e and e.get("passes")), e


def test_review_s3_a_red_light_flag_stuck_on_blocks_the_restore_at_most_20_s():
  # (rk/red_stuck.py) the flag on from just before his RES and never off: no SET+ for RED_STOP_BLOCK_MAX from its onset,
  # then the restore (to his 45) goes on
  car, lg, log = _resume_run(frames=3000 + 4000, off_for=3000, red=lambda f: f >= 2900)
  ups = [f for f, _, b, _ in log if b == cs.SET_PLUS]
  assert ups and ups[0] >= 2900 + int(cs.RED_STOP_BLOCK_MAX * 100) - 2, ups[:3]
  assert abs(car.set_speed / MPH - 45) < 1e-6


def test_review_s4_res_after_a_turn_onto_another_road_gets_the_gentle_restore():
  car, lg, log = _resume_run(changed=True)
  assert lg.pending_from is None                      # no fast restore of ours (the baseline restore rules only)
  car2, lg2, log2 = _resume_run(changed=True, clear=False)
  assert lg2.pending_from is None and not any(a == "hold" for *_, a in log2)
  # the wrapper: another road name, or the heading watch's turn, since the stop episode began
  w, _ = _wrapper()
  name = {"n": "West Utica Road"}
  w._road_name = lambda: name["n"]
  w.stop_road = None
  w.logic.stop_episode = True
  assert not w._road_changed()
  assert not w._road_changed()
  name["n"] = "Dequindre Road"
  assert w._road_changed()
  name["n"] = "West Utica Road"
  w.limit_follow.frame += 10
  w.limit_follow.turned_frame = w.limit_follow.frame
  assert w._road_changed()
  w.logic.stop_episode = False
  assert not w._road_changed() and w.stop_road is None


def test_review_m1_a_hold_that_never_sent_a_frame_does_not_use_up_the_episodes_holds():
  for kind, attr in (("stop", "holds_this_episode"), ("curve", "curve_holds")):
    lg = cs.CurveSlowdownLogic(False, 10)
    lg.update(_inp(set_speed=45 * MPH, v_ego=45 * MPH))
    setattr(lg, attr, 1)
    lg.hold_kind = kind
    lg._start_hold(cs.SET_MINUS)
    lg._stop_hold(cut=True)
    assert getattr(lg, attr) == 0, kind


def test_review_m2_a_queue_on_the_ramp_mid_boost_then_res_at_5_mph_takes_the_boost_down_at_once():
  # (rk/ramp_sc.py E / F) the boost at 95, his brake for a queue (5 mph), RES 10 s later: the stored 95 comes back;
  # one SET- hold at once, then taps, to the merge's 75 (it waited for 12 mph and took ~7 s)
  def ev(f, st, car):
    if f == 2500:
      st["cruise"], st["brake"] = False, True
    if 2500 <= f < 3500:
      car.v_ego = max(car.v_ego - 0.05, 5 * MPH)
    if f == 3400:
      st["brake"] = False
    if f == 3500:
      st["cruise"] = True
  car, lg, log, offers = _ramp_run(5000, ev=ev)
  assert max(s for f, s, *_ in log if f < 2500) >= 90
  downs = [f for f, _, _, b, act, _, _ in log if b == cs.SET_MINUS and act == "hold" and f >= 3500]
  assert downs and downs[0] - 3500 <= 60, downs[:2]
  back = next(f for f, s, *_ in log if f > 3500 and abs(s - 75) < 1e-6)
  assert back - 3500 <= 550, back


def test_review_m3_a_reopened_merge_refused_again_is_logged_once_per_link():
  fol = slf.SpeedLimitFollower(False)

  def step(f, on, d_mph, btn=None):
    road = dict(road_class=7) if f < 100 else dict(road_class=2, ramp_to_highway=True, next_limit=70 * MPH,
                                                    next_distance=900.0)
    fol.update(slf.LimitInputs(0.0, "Map Data", 0, d_mph * MPH if on else 0.0, on, v_ego=30 * MPH,
                               driver_set=btn is not None, **road))
  for f in range(2000):
    on = not (600 <= f < 700 or 1100 <= f < 1200)     # two brakes / RES at his 80 (above the merge's 75): his, each time
    step(f, on, 80)
  why = [e for e in fol.events if "not below the merge" in e[1]["reason"]]
  assert len(why) == 1, fol.events


def test_review_m4_the_unexplained_raise_chime_at_most_once_per_10_s():
  w, plan = _wrapper(limit=70, road_class=1)
  lg = w.logic
  set_mph, rises, prev = 72.0, 0, False
  lg.update(_inp(set_speed=set_mph * MPH, v_ego=70 * MPH))
  for f in range(1500):
    if f % 200 == 100:
      set_mph += 2.0 if set_mph < 90 else -2.0          # an unexplained step every 2 s, all of them high
    i = _inp(set_speed=set_mph * MPH, v_ego=70 * MPH)
    lg.update(i)
    w._unattributed_change(i, plan)
    rises += w.set_speed_high and not prev
    prev = w.set_speed_high
  assert rises == 2, rises                             # 15 s: at f 100 and once 10 s later


def test_second_review_a_red_light_he_waited_at_with_cruise_off_still_blocks_at_his_res():
  # (rk/red_stuck2.py) the light on for 30 s while stopped with cruise off, then his RES under it: the 20 s count
  # starts at the RES
  car, lg, log = _resume_run(frames=3000 + 3000, off_for=3000, red=lambda f: True)
  ups = [f for f, _, b, _ in log if b == cs.SET_PLUS]
  assert ups and ups[0] >= 3000 + int(cs.RED_STOP_BLOCK_MAX * 100) - 2, ups[:3]
  assert abs(car.set_speed / MPH - 45) < 1e-6


def test_second_review_a_red_light_on_the_ramp_with_a_planned_stop_never_counts_as_stuck():
  # (M-b) the light on and the model planning a stop (a ramp meter's queue): the pause never expires
  # (MERGE_BOOST_TIME does not run out meanwhile)
  car, lg, log, offers = _ramp_run(4500, red=lambda f: f >= 500, plan=lambda f: f >= 500 and f % 300 < 250)
  sets = {f: s for f, s, *_ in log}
  assert abs(sets[4499] - sets[620]) < 1e-6, (sets[620], sets[4499])
