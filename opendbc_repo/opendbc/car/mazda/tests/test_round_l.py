"""Round L (drive 6f): the logged scenarios as tests."""
import importlib.util
import math
import os

from opendbc.car.mazda import curve_slowdown as cs

MPH = cs.MS_PER_MPH
HERE = os.path.dirname(__file__)
_spec = importlib.util.spec_from_file_location("tep_round_l", os.path.join(HERE, "test_exit_plan.py"))
TE = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(TE)
CTL = TE.CTL


def _inp(**k):
  d = dict(enabled=True, cruise_enabled=True, set_speed=20 * MPH, v_ego=25 * MPH, gas_pressed=False, brake_pressed=False,
           standstill=False, plan_alive=True, csc_active=False, csc_speed=0.0, other_button_active=False)
  d.update(k)
  return cs.SlowdownInputs(**d)


# ---- 1a: his gas just past the split does not decline an exit navigation takes (drive 6f t 776.9) ----

def _e6f(**kw):
  """Drive 6f, the exit to E South Blvd: 75 set, the 70 highway, navigation's exit point at 1600 (it clears 5 m before),
  the shapes silent past it and "link" from 135 m on, the 113 m curve 270 m after it; the camera's edge lane from 400 m
  before (exit lane evidence)."""
  exit_x, curve_x = 1600.0, 1870.0
  fields = TE.exit_fields(exit_x, curve_x, r=113.0, ann=1500.0, clear=5.0, link_at=135.0, ramp_at=180.0,
                          curve_held=60.0, **kw)
  return fields, exit_x, curve_x


def test_6f_gas_past_the_split_with_navigation_on_the_exit_does_not_decline_it():
  fields, exit_x, curve_x = _e6f()
  lane = lambda x: 1 if x >= exit_x - 400 else None  # noqa: E731
  # his foot on the gas 30 m past the split for ~90 m (4 s), the exit tentative (the shapes silent)
  gas = lambda x: 0.0 if exit_x + 30 <= x < exit_x + 120 else None  # noqa: E731
  sig = lambda x: 1 if exit_x - 700 <= x < exit_x - 640 else 0  # noqa: E731
  base = TE.drive_exit(fields, 9000, 75, lane=lane, signal=sig)
  r = TE.drive_exit(fields, 9000, 75, lane=lane, gas=gas, signal=sig)
  p = r["ctl"].plan or r["ctl"].last_plan
  assert p.nav_near and not p.declined and "overturned" not in p.why, p.why
  assert "confirmed" in r["state"]
  # no SET+ of ours past the split above the speed there (was: the restore 47 -> 64 under a red light)
  v_split = TE._at(r, exit_x)
  assert max(s for s, xx in zip(r["sets"], r["xs"], strict=True) if xx >= exit_x) <= v_split + 1.0
  v_c = math.sqrt(2.2 * 113.0) / MPH
  assert TE._at(r, curve_x) <= min(v_c, TE._at(base, curve_x)) + 1.0, (TE._at(r, curve_x), TE._at(base, curve_x))
  # (before round L: declined by the gas, restored at once, far too fast at the curve)
  old = CTL.NAV_GAS_KEEPS
  try:
    CTL.NAV_GAS_KEEPS = False
    r0 = TE.drive_exit(fields, 9000, 75, lane=lane, gas=gas, signal=sig)
  finally:
    CTL.NAV_GAS_KEEPS = old
  assert (r0["ctl"].plan or r0["ctl"].last_plan).declined and TE._at(r0, curve_x) > v_c + 8.0, TE._at(r0, curve_x)
  # without navigation's route on it near the split (it dropped the exit 600 m before: no nav_near) his gas is still his
  # decision (declined, restored)
  ctl = CTL.MapCurveControl(3.0)
  ctl.update(CTL.MapFields(mode=CTL.ON, road_class=1, nav_exit=1, nav_exit_distance=900.0, nav_exit_side=1), 33.0, 0,
             v_ego=30.0)
  p = ctl.plan
  assert p is not None and not p.nav_near
  ctl.odo = p.exit_at - 150.0
  p.slowing = True
  ctl.update(CTL.MapFields(mode=CTL.ON, road_class=1), 33.0, 0, v_ego=30.0, gas=True)
  assert p.declined


def test_6f_gas_held_with_the_trusted_highway_match_still_declines():
  # he stays on the highway against navigation: his gas held with the road match on the highway (GPS trusted) past the
  # split is his decision (the shapes silent: no missed exit answer)
  ctl = CTL.MapCurveControl(3.0)
  ctl.update(CTL.MapFields(mode=CTL.ON, road_class=1, nav_exit=1, nav_exit_distance=200.0, nav_exit_side=1), 33.0, 0,
             v_ego=30.0)
  p = ctl.plan
  assert p.nav_near
  ctl.odo = p.exit_at + 20.0
  n_tap = int(0.5 / CTL.DT_CTRL)
  for _ in range(n_tap):
    ctl.update(CTL.MapFields(mode=CTL.ON, road_class=1, gps_trust=2), 33.0, 0, v_ego=30.0, gas=True)
  assert not p.declined                       # a tap, GPS not trusted: nothing
  for _ in range(int((CTL.HWY_MATCH_STAY + CTL.GAS_OVERTURN_TIME) / CTL.DT_CTRL) + 5):
    ctl.update(CTL.MapFields(mode=CTL.ON, road_class=1, gps_trust=0), 33.0, 0, v_ego=30.0, gas=True)
  assert p.declined


# ---- 1b: no SET+ back up of ours under the stop-light detector, whatever the restore (drive 6f t 780.3-782.1) ----

def _restore_run(frames, red_fn, gas_fn=lambda f: False, set0=47, anchor=75, v=52, fast=True):
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=set0 * MPH, v_ego=v * MPH))
  lg.anchor = anchor * MPH
  set_mph, ups = float(set0), []
  for f in range(frames):
    b, _ = lg.update(_inp(set_speed=set_mph * MPH, v_ego=v * MPH, gas_pressed=gas_fn(f), red_stop=red_fn(f),
                          fast_restore=fast and f < 100, btn_counter=(f // 10) % 16))
    if b == cs.SET_PLUS:
      ups.append(f)
      if lg.last_action != "hold" or (f % 10) == 0:
        set_mph = min(set_mph + 1, anchor)
  return lg, set_mph, ups


def test_6f_no_set_plus_of_a_restore_under_the_red_light_also_on_the_gas():
  assert not cs.CurveSlowdownLogic(False, 10).stop_episode
  # the exit's fast restore (no stop episode of ours), his gas on and off, the light on from frame 0 to 600
  lg, set_mph, ups = _restore_run(900, lambda f: f < 600, gas_fn=lambda f: 100 <= f < 300)
  assert ups and min(ups) >= 600, ups[:5]
  assert set_mph > 47
  # without the light: up at once
  lg, set_mph, ups = _restore_run(300, lambda f: False)
  assert ups and min(ups) < 100
  # the restore after a reason ended (no fast restore): the same
  lg, _, ups = _restore_run(900, lambda f: f < 600, fast=False)
  assert all(f >= 600 for f in ups)
  # a stuck flag blocks for RED_STOP_BLOCK_MAX (with cruise on) at most
  n = int(cs.RED_STOP_BLOCK_MAX / cs.DT_CTRL)
  lg, _, ups = _restore_run(n + 400, lambda f: True)
  assert ups and all(f >= n for f in ups)


def test_6f_a_restore_hold_already_going_stops_under_the_red_light():
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=40 * MPH, v_ego=40 * MPH))
  lg.anchor = 75 * MPH
  set_mph, hold_frames, ups = 40.0, [], []
  for f in range(400):
    b, _ = lg.update(_inp(set_speed=set_mph * MPH, v_ego=40 * MPH, fast_restore=f < 50, red_stop=f > 60,
                          btn_counter=(f // 10) % 16))
    if lg.hold is not None:
      hold_frames.append(f)
    if b == cs.SET_PLUS:
      ups.append(f)
      if lg.last_action == "hold" and f % 20 == 0:
        set_mph += 5
  # the hold began before the light (frame 28) and ends with it (was: on to 70 by frame 140, then taps)
  assert hold_frames and hold_frames[0] < 60 and hold_frames[-1] <= 61, hold_frames[-3:]
  assert ups and max(ups) <= 61 and set_mph <= 55, (ups[-3:], set_mph)

