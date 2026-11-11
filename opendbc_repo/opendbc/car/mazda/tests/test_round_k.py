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
