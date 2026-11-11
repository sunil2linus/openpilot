"""Round k (drives 6a / 6c / 6d): the logged scenarios as tests."""
import math

from opendbc.car.mazda import curve_slowdown as cs

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

from opendbc.car.mazda import stopped_car_slowdown as scs  # noqa: E402


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


def test_6a_580_a_vanishing_plan_with_the_light_on_at_50_mph_restores_after_2_5_s():
  tr, out, f0 = _sa_run(50, lambda f: f < 55 or f == 200)
  assert not tr.active and tr.release_reason in (scs.FAR_STOP_UNCONFIRMED, scs.STOP_AHEAD_CLEARED, scs.STOP_AHEAD_UNCONFIRMED)
  assert tr.restore_now and not tr.hold_restore
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
