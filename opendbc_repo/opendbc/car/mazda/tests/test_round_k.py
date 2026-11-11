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
