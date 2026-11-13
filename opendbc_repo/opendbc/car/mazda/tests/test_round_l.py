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


# ---- 2: our SET- right after his gas jumps UP to the current speed (drive 6f t 535.69) ----

def _gas_release_run(jump_window=1.0, frames=1500, driver_tap_at=None):
  """Set 39 under his 75 (a ramp curve at 33 followed), the car at 47.2 on his gas until frame 100. The car: a SET tap
  within jump_window s after the gas, the car 2+ mph over the set speed, sets the current speed; else 1 mph steps.
  driver_tap_at: his own SET+ tap at that frame (a jump of his)."""
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=39 * MPH, v_ego=47.2 * MPH))
  lg.anchor = 75 * MPH
  set_mph, v, last_gas, due, log, minus = 39.0, 47.2, 0, [], [], []
  for f in range(frames):
    gas = f < 100
    if gas:
      last_gas = f
    v = v if gas else max(v - 0.0025, 33.0)
    drv = cs.SET_PLUS if driver_tap_at is not None and driver_tap_at <= f < driver_tap_at + 5 else None
    b, _ = lg.update(_inp(set_speed=set_mph * MPH, v_ego=v * MPH, gas_pressed=gas, csc_active=True, csc_speed=33 * MPH,
                          curve_track=True, curve_floor=25 * MPH, driver_button=drv, btn_counter=(f // 10) % 16))
    if drv is not None and f == driver_tap_at:
      due.append((f + 8, "jump"))
    if b is not None:
      if b == cs.SET_MINUS:
        minus.append(f)
      jump = (f - last_gas) * cs.DT_CTRL <= jump_window and v >= set_mph + 2
      due.append((f + 8, "jump" if jump else b))
    for d in [d for d in due if d[0] == f]:
      due.remove(d)
      set_mph = float(round(v)) if d[1] == "jump" else set_mph + (1 if d[1] == cs.SET_PLUS else -1)
    log.append((f, set_mph))
  return lg, set_mph, minus, log


def test_6f_no_set_minus_right_after_the_gas_while_it_may_jump():
  # (the car jumps on a tap up to 0.5 s after the gas: drives 4c-4f, every jump had the gas within 0.5 s)
  lg, set_mph, minus, log = _gas_release_run(jump_window=0.5)
  assert cs.GAS_RELEASE_SETTLE >= 0.5 and minus and min(minus) >= 150, minus[:3]
  assert max(s for _, s in log) <= 39 + 1e-6            # never a jump up
  assert abs(set_mph - 33) <= 1.0 and lg.anchor is not None and abs(lg.anchor / MPH - 75) < 0.5


def test_6f_a_set_minus_of_ours_landing_as_a_jump_up_is_ours_and_lowering_goes_on():
  # the car jumps on a SET tap up to 1 s after the gas: our first SET- (0.5 s after) lands 39 -> 47
  lg, set_mph, minus, log = _gas_release_run(jump_window=1.0)
  assert max(s for _, s in log) >= 46, "the jump happened"
  assert lg.jump_down_claims == 1 and not lg.disabled
  assert lg.anchor is not None and abs(lg.anchor / MPH - 75) < 0.5
  assert abs(set_mph - 33) <= 1.0, set_mph              # lowered on to the curve's target (was: 47, blocked)
  # his own SET+ edge then: his (the old rule)
  lg, set_mph, _, _ = _gas_release_run(jump_window=1.0, driver_tap_at=150)
  assert lg.jump_down_claims == 0 and lg.blocked


# ---- 3b / 3c: the loop on-ramp's merge after a rolling stop, the unwind path (drive 6f t 676-681) ----

WB = 2.7


def _loop_merge(profile, frames=4000, v_min=13.4):
  """Set 30 on the loop (13.4 m/s), the angle -36 deg, a stop latched (frames 0-2000), released; 1 s later the wheel
  unwinds as profile(t) gives (t s from the onset, deg). limit_raise 75, limit_boost 20 throughout. Returns the frame of
  the first SET+ of ours relative to the unwind's onset (s), or None."""
  lg = cs.CurveSlowdownLogic(False, 10)
  gate = cs.MergeTurnGate()
  release, onset = 2000, 2100
  set_mph, prev = 30.0, -36.0
  lg.update(_inp(set_speed=30 * MPH, v_ego=13.4))
  for f in range(frames):
    t = (f - onset) * cs.DT_CTRL
    ang = -36.0 if t < 0 else profile(t)
    rate = (ang - prev) / cs.DT_CTRL
    prev = ang
    v = v_min if f < release else 13.4
    ok, pause = gate.update(v, ang, rate, WB)
    stop = f < release
    b, _ = lg.update(_inp(set_speed=set_mph * MPH, v_ego=v, stop_active=stop, stop_target=30 * MPH,
                          limit_raise=75 * MPH,
                          limit_boost=20 * MPH, merge_turn_ok=ok, merge_turn_pause=pause, btn_counter=(f // 10) % 16))
    if b == cs.SET_PLUS:
      return t
  return None


def _pw(points):
  def f(t):
    for (t0, a0), (t1, a1) in zip(points, points[1:], strict=False):
      if t <= t1:
        return a0 + (a1 - a0) * (t - t0) / (t1 - t0)
    return points[-1][1]
  return f


SIXF = _pw([(0.0, -36.0), (0.25, -27.0), (1.0, -20.0), (2.0, -8.0), (3.0, -3.0)])       # as logged (676.9-679.0)
EVEN = _pw([(0.0, -36.0), (2 / 3, -27.0), (4 / 3, -20.0), (2.0, -8.0), (3.0, -3.0)])


def test_6f_loop_merge_starts_as_the_wheel_unwinds_after_a_rolling_stop():
  # the unwind path (|lat| under MERGE_TURN_UNWIND_LAT = MERGE_TURN_LAT, at -19 deg here, while it unwinds): ~1.1 s after the
  # onset, the calm path's 1 s wait saved (the analysis asked for 0.5 s, which needs |lat| < 1.5 at -29 deg: that also
  # starts test_round_k's long loop easing to 1.1 m/s^2 mid-loop)
  t = _loop_merge(SIXF)
  assert t is not None and 0.9 <= t <= 1.2, t
  t = _loop_merge(EVEN)
  assert t is not None and t <= 1.5, t
  old = cs.MERGE_TURN_UNWIND
  try:
    cs.MERGE_TURN_UNWIND = False
    t_calm = _loop_merge(SIXF)
  finally:
    cs.MERGE_TURN_UNWIND = old
  assert t_calm is not None and t_calm >= _loop_merge(SIXF) + 0.8, t_calm
  # (before round L: the calm gate and the post-stop wait, RAISE_FAST_AFTER_STOP from the release: 4 s after the onset)
  old = (cs.MERGE_TURN_UNWIND, cs.MERGE_STOP_ROLLING_SPEED)
  try:
    cs.MERGE_TURN_UNWIND, cs.MERGE_STOP_ROLLING_SPEED = False, 1e9
    t_old = _loop_merge(SIXF)
  finally:
    cs.MERGE_TURN_UNWIND, cs.MERGE_STOP_ROLLING_SPEED = old
  assert t_old is not None and t_old >= cs.RAISE_FAST_AFTER_STOP - 1.0 - 0.05, t_old


def test_the_post_stop_wait_stays_after_a_real_stop_or_his_brake_to_a_stop():
  # the stop took the car below MERGE_STOP_ROLLING_SPEED (a queue, a light): the 5 s wait as before
  t = _loop_merge(SIXF, v_min=3.0)
  assert t is not None and t >= cs.RAISE_FAST_AFTER_STOP - 1.0 - 0.05, t


def test_the_unwind_path_needs_the_car_not_slowing_and_no_tight_curve_ahead():
  def run(decel=0.0, ahead=False, n=180):   # (the calm path alone: from ~2.05 s)
    g = cs.MergeTurnGate()
    v, prev, oks = 13.4, -36.0, []
    for f in range(n):
      t = f * cs.DT_CTRL
      a = SIXF(t)
      v -= decel * cs.DT_CTRL
      ok, _ = g.update(v, a, (a - prev) / cs.DT_CTRL, WB, False, curve_ahead=ahead)
      prev = a
      oks.append(ok)
    return any(oks)
  assert run()
  # drive 65 t 651: a 120 deg turn unwinding while the car slows 1 m/s^2, then a turn the other way: not ok
  assert not run(decel=1.0)
  # a map curve under 90 m within 100 m ahead (a loop after a short straight): not ok
  assert not run(ahead=True)
  # winding up again pauses
  g = cs.MergeTurnGate()
  prev = -36.0
  for f in range(130):
    a = SIXF(f * cs.DT_CTRL)
    ok, pz = g.update(13.4, a, (a - prev) / cs.DT_CTRL, WB)
    prev = a
  assert ok
  ok, pz = g.update(13.4, -16.0, -30.0, WB)
  assert pz and not ok


# ---- 3a: no camera-only stop just before the turn onto an on-ramp, nor through a HELD flicker (drive 6f t 647.7-649.7) ----

def _wrapper(tmp_path, monkeypatch):
  import pytest
  pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)
  spec = importlib.util.spec_from_file_location("twi_round_l", os.path.join(HERE, "test_wrapper_inputs.py"))
  WI = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(WI)
  c, sm, _ = WI._make(tmp_path, monkeypatch, limit_mode=0, map_mode=2)
  seen = []
  orig = c.trigger.update
  monkeypatch.setattr(c.trigger, "update", lambda i: (seen.append(i), orig(i))[1])
  return c, sm, seen, WI


def test_6f_no_red_light_stop_just_before_the_on_ramp_and_through_its_held_flicker(tmp_path, monkeypatch):
  c, sm, seen, WI = _wrapper(tmp_path, monkeypatch)
  plan = sm.msgs["starpilotPlan"]
  plan.mapCurveMode, plan.mapRoadClass = 2, 5            # a street
  WI._drive(c, 20, set_mph=35, v_mph=32)
  plan.redLight = True
  # the on-ramp's first curve governs before the turn onto it (navigation / his signal), 150 m ahead: as before
  plan.mapOnRampAhead, plan.mapCurveStatus, plan.mapCurveRadius, plan.mapCurveDistance = True, 2, 84.0, 150.0
  WI._drive(c, 3, set_mph=35, v_mph=32)
  assert seen[-1].red_light
  # within ONRAMP_RED_OFF_DIST: no camera-only stop (64 m: still as before since the review's 50 m)
  plan.mapCurveDistance = 64.0
  WI._drive(c, 3, set_mph=35, v_mph=32)
  assert seen[-1].red_light
  plan.mapCurveDistance = 40.0
  WI._drive(c, 3, set_mph=35, v_mph=32)
  assert not seen[-1].red_light and not seen[-1].plan_stop
  # the map's status HELD (5) for 2 s before the link is matched: still off; ONRAMP_RED_OFF_HOLD later back
  plan.mapOnRampAhead, plan.mapCurveStatus, plan.mapCurveRadius, plan.mapCurveDistance = False, 5, 212.0, 37.0
  WI._drive(c, int(2.0 / cs.DT_CTRL), set_mph=35, v_mph=32)
  assert not any(i.red_light for i in seen[-200:])
  WI._drive(c, int(1.2 / cs.DT_CTRL), set_mph=35, v_mph=32)
  assert seen[-1].red_light


def test_6f_a_map_curve_target_that_drops_still_counts_for_the_red_head_start(tmp_path, monkeypatch):
  c, sm, seen, WI = _wrapper(tmp_path, monkeypatch)
  from opendbc.car.mazda.map_curve_control import MapOutput
  plan = sm.msgs["starpilotPlan"]
  plan.mapCurveMode, plan.mapRoadClass = 2, 5
  out = {"mo": MapOutput(active=True, speed=30 * MPH, why="ramp")}
  monkeypatch.setattr(c.map_control, "update", lambda *a, **k: out["mo"])
  WI._drive(c, 5, set_mph=35, v_mph=32)
  assert seen[-1].curve_active
  out["mo"] = MapOutput()
  WI._drive(c, int(2.9 / cs.DT_CTRL), set_mph=35, v_mph=32)
  assert all(i.curve_active for i in seen[-280:])
  WI._drive(c, int(0.3 / cs.DT_CTRL), set_mph=35, v_mph=32)
  assert not seen[-1].curve_active


# ---- 4: navigation's "on ramp" second in allManeuvers (drive 6f t 625-646) ----

TM = TE.TM


def test_6f_the_on_ramp_named_second_in_navigation_is_planned_from_far_out():
  root, curves, ramps = TE.write_files(TM._onramp_scene())
  la, lo = TM.pos(0, -390)
  street = {"dt": 0.05, "v_ego": 22.0, "street": True}
  prim = ("turn", "slight right", 380.0)
  alls = [("end of road", "right", 380.0), ("on ramp", "slight right", 380.0), ("off ramp", "slight right", 3600.0)]
  m = TM.planner(root)
  m.update(la, lo, 0.0, 30, True, nav=prim, gps=dict(street, signal=0), nav_all=alls)
  assert m.onramp_ahead and m.status == TM.MC.STATUS_RAMP_CURVE and 380 <= m.distance <= 480, (m.onramp_ahead, m.distance)
  # (before round L the primary maneuver alone was read: nothing until his signal ~150 m out)
  m = TM.planner(root)
  m.update(la, lo, 0.0, 30, True, nav=("turn", "slight right", 150.0), gps=dict(street, signal=0),
           nav_all=[("turn", "slight right", 150.0), ("on ramp", "slight right", 400.0)])
  assert not m.onramp_ahead                         # the on ramp 250 m from the primary's distance: not this one
  # the primary alone, a turn toward the ramp's side at its start's distance, the ramp leaving our road: also
  m = TM.planner(root)
  m.update(la, lo, 0.0, 30, True, nav=prim, gps=dict(street, signal=0))
  assert m.onramp_ahead
  # ... not toward the other side, nor 100 m off
  for nav in (("turn", "slight left", 380.0), ("turn", "slight right", 280.0), ("merge", "slight right", 380.0)):
    m = TM.planner(root)
    m.update(la, lo, 0.0, 30, True, nav=nav, gps=dict(street, signal=0))
    assert not m.onramp_ahead, nav


def test_drive_69_turn_onto_the_crossing_road_with_its_on_ramp_still_no_on_ramp_ahead():
  root, curves, ramps = TE.write_files(TM._junction_scene())
  la, lo = TM.pos(0, 160)
  street = {"dt": 0.05, "v_ego": 12.0, "street": True}
  m = TM.planner(root)
  m.update(la, lo, 0.0, 40, True, nav=("turn", "left", 130.0), gps=dict(street, signal=0),
           nav_all=[("turn", "left", 130.0), ("continue", "straight", 2000.0)])
  assert not m.onramp_ahead


def test_6f_on_ramp_ahead_from_380_m_slows_to_its_speed_with_the_cruise_alone():
  entry = 430.0
  v_c = math.sqrt(2.6 * 70.0) / MPH                 # the loop at RAMP_LAT (published 84 m at PLAN_LAT)

  def fields(x):
    if x < entry:
      return TE.F(status=2, radius=84.0, distance=entry - x, road_class=5, onramp_ahead=True)
    return TE.F(status=2, radius=84.0, distance=0.0, road_class=2, on_ramp=True, ramp_type=2, ramp_to_highway=True,
                ramp_end_distance=400.0)
  r = TM.drive_53(fields, 3000, 50, v_mph=50)
  v_in = TM._at(r, entry)
  assert v_in <= v_c + 2.0, (v_in, v_c)
  vs, xs = r["vs"], r["xs"]
  n = int(1.0 / cs.DT_CTRL)
  dec = max((vs[k - n] - vs[k]) * MPH for k in range(n, len(vs)) if xs[k] <= entry)
  assert dec <= 0.85, dec                           # the cruise's own slowdown, no brake needed


def test_no_street_raise_while_an_on_ramp_ahead_below_the_set_speed_is_pending():
  def run(pending):
    lg = cs.CurveSlowdownLogic(False, 10)
    ups = 0
    for _ in range(800):
      b, _ = lg.update(_inp(set_speed=47 * MPH, v_ego=47 * MPH, limit_raise=53 * MPH, onramp_pending=pending))
      ups += b == cs.SET_PLUS
    return ups, lg
  ups, lg = run(True)
  assert ups == 0 and lg.limit_adopted is None and lg.anchor is None
  ups, lg = run(False)
  assert ups > 0


# ---- 5: ramp curve memory (drive 6f, the exit to S Rochester Rd: map 85 m, measured 113-132 m) ----

MC = TM.MC


def _ramp_c(r=85.0):
  return {"apex": (42.6, -83.1), "r": r, "ramp": {"type": "exit"}, "s_a": 0.0, "name": ""}


DIR = 90.0


def test_ramp_memory_radius_rules(tmp_path):
  # (review of round L, F2) two clean ramp passes agreeing, at most 1.15 x the map's, never below it, a direction known
  mem = MC.CurveMemory(str(tmp_path / "mem.json"))
  c = _ramp_c()
  assert mem.ramp_radius(c, DIR) is None                               # no memory: the map's (unchanged)
  mem.add_ramp_pass(c, 200.0, DIR, t=1000.0, v=15.0, flags=["full"])
  assert mem.ramp_radius(c, DIR) is None                               # one pass (rl/s5.py: a wrong 200 m one): the map's
  mem.add_ramp_pass(c, 115.0, DIR, t=2000.0, v=19.0, flags=["full"])
  assert mem.ramp_radius(c, DIR) is None                               # the last two disagree (200 / 115)
  mem.add_ramp_pass(c, 115.0, DIR, t=3000.0, v=19.0, flags=["full"])
  assert abs(mem.ramp_radius(c, DIR) - 97.75) < 1e-6                   # 0.9 x 115 = 103.5, capped at 1.15 x 85
  assert mem.ramp_radius(c, None) is None and mem.ramp_radius(c, DIR + 180.0) is None   # direction unknown / the other way
  c3 = {**_ramp_c(), "apex": (42.8, -83.3)}
  for k in range(2):
    mem.add_ramp_pass(c3, 70.0, DIR, t=1.0 + 500 * k, v=19.0, flags=["full"])
  assert abs(mem.ramp_radius(c3, DIR) - 85.0) < 1e-6                   # never below the map's
  c4 = {**_ramp_c(), "apex": (42.9, -83.4)}
  for k in range(2):
    mem.add_ramp_pass(c4, 115.0, DIR, t=1.0 + 500 * k, v=19.0, flags=[])
  assert mem.ramp_radius(c4, DIR) is None                              # not full passes: nothing
  c5 = {**_ramp_c(), "apex": (43.0, -83.5)}
  for k in range(2):
    mem.add_pass(c5, 115.0, t=1.0 + 500 * k, v=19.0, flags=["full"], direction=DIR)   # untagged (a road record)
  assert mem.ramp_radius(c5, DIR) is None
  mem.add_ramp_pass(c, 115.0, None, t=9000.0, flags=["full"])          # no direction: not stored
  assert len(mem._find(*c["apex"], DIR)["passes"]) == 3


def test_review_l_f2_a_saturated_or_hand_steered_ramp_pass_clears_its_memory(tmp_path):
  # rl/s5.py: the saturated pass ("Turn Exceeds Steering Limit") and the pass with his steering / brake changed nothing
  for kind in ("sat", "steer", "brake"):
    mem = MC.CurveMemory(str(tmp_path / f"{kind}.json"))
    c = _ramp_c()
    for k in range(2):
      mem.add_ramp_pass(c, 115.0, DIR, t=1000.0 * (k + 1), v=19.0, flags=["full"])
    assert abs(mem.ramp_radius(c, DIR) - 97.75) < 1e-6
    v = math.sqrt(MC.RAMP_LAT * 97.75)
    if kind == "sat":
      samples = [(v, 1 / 95.0, True, False, 0.99)] * 30 + [(v, 1 / 85.0, True, False, 0.6)] * 50
    elif kind == "steer":
      samples = [(v, 1 / 85.0, True, True, 0.6)] * 80
    else:
      samples = [(v, 1 / 85.0, True, False, 0.6)] * 80
    _ramp_pass_on(tmp_path, mem, c, samples, brake=kind == "brake")
    assert mem.ramp_radius(c, DIR) is None, kind                       # back to the map's radius


def _ramp_pass_on(tmp_path, mem, c, samples, blinker=False, stop=False, brake=False):
  from types import SimpleNamespace as N
  mp = MC.MapCurvesPlanner(None, None)
  mp.mc = MC.MapCurves(MC.MapCurveStore(str(tmp_path), background=False), memory=mem)
  mp.trust = N(shaky=False)
  mp.pass_curve, mp.pass_samples, mp.pass_dir = c, samples, DIR
  mp.pass_blinker, mp.pass_apex, mp.pass_exit, mp.pass_vetoed, mp.pass_shaky = blinker, 5.0, 16.0, False, False
  mp.pass_stop, mp.pass_brake = stop, brake
  mp._finish_pass()


def test_6f_a_remembered_ramp_curve_is_planned_faster_and_a_road_curve_is_unchanged(tmp_path):
  root, curves, _ = TE.write_files(TM._onramp_scene())
  ramp_c = next(c for c in curves if c.get("ramp"))
  mem = MC.CurveMemory(str(tmp_path / "mem.json"))
  m = MC.MapCurves(MC.MapCurveStore(root, background=False), memory=mem)
  r_map = float(ramp_c["r"])
  assert abs(m._ramp_r(ramp_c, DIR) - MC.ramp_radius(ramp_c)) < 1e-6
  for k in range(2):
    mem.add_ramp_pass(ramp_c, r_map * 1.1, DIR, t=1000.0 * (k + 1), v=19.0, flags=["full"])
  want = max(r_map, 0.9 * r_map * 1.1) * MC.RAMP_LAT / MC.PLAN_LAT
  assert abs(m._ramp_r(ramp_c, DIR) - want) < 0.2
  assert abs(m._ramp_r(ramp_c, None) - MC.ramp_radius(ramp_c)) < 1e-6   # direction unknown: the map's
  # drive 6f's numbers: map 85 m, measured 115 m twice: the curve's speed at RAMP_LAT on 97.75 m, >= 35 mph (was 33.3)
  c = _ramp_c()
  for k in range(2):
    mem.add_ramp_pass(c, 115.0, DIR, t=1000.0 * (k + 1), v=19.0, flags=["full"])
  v = MC.curve_speed_limit(m._ramp_r(c, DIR)) / MPH
  assert v >= 35.5 and abs(MC.curve_speed_limit(MC.ramp_radius(c)) / MPH - 33.3) < 0.2, v
  # a road curve: the road memory as before (two agreeing passes)
  road = {"apex": (43.0, -83.0), "r": 100.0, "s_a": 0.0}
  mem.add_pass(road, 150.0, t=1.0, v=19.0, flags=["full"])
  assert m._radius(road) == 100.0


def test_6f_ramp_curve_from_61_mph_with_its_memory_on_the_car_side():
  entry = 430.0

  def run(r_pub):
    def fields(x):
      return TE.F(status=2, radius=r_pub, distance=max(entry - x, 0.0), road_class=2, on_ramp=True, ramp_type=1,
                  ramp_end_distance=900.0 - x)
    return TM.drive_53(fields, 3000, 61, v_mph=61)
  r0 = run(85.0 * MC.RAMP_LAT / MC.PLAN_LAT)
  r1 = run(1.15 * 85.0 * MC.RAMP_LAT / MC.PLAN_LAT)
  v0, v1 = TM._at(r0, entry), TM._at(r1, entry)
  assert v1 >= 35.0 and v1 >= v0 + 1.5, (v0, v1)
  assert min(r1["sets"]) >= min(r0["sets"]) + 1.5


def test_a_ramp_pass_with_a_lane_change_or_a_stop_is_not_stored(tmp_path):
  samples = [(19.0, 1 / 115.0, True, False, 0.4)] * 80
  mem = MC.CurveMemory(str(tmp_path / "clean.json"))
  _ramp_pass_on(tmp_path, mem, _ramp_c(), samples)
  p = mem.entries[0]["passes"][0]
  assert abs(p[0] - 115.0) < 1.0 and "full" in p[3] and "ramp" in p[3] and mem.entries[0].get("ramp")
  for kw in (dict(blinker=True), dict(stop=True)):
    mem = MC.CurveMemory(str(tmp_path / f"x{len(kw)}{list(kw)[0]}.json"))
    _ramp_pass_on(tmp_path, mem, _ramp_c(), samples, **kw)
    assert not any(e.get("passes") for e in mem.entries), kw


def test_the_ramp_curve_we_are_in_is_measured(tmp_path):
  # the on-ramp loop of test_map_curves._onramp_scene, driven along its shape: MapCurves gives it as the curve we are in
  root, curves, _ = TE.write_files(TM._onramp_scene())
  ramp_c = next(c for c in curves if c.get("ramp"))
  m = MC.MapCurves(MC.MapCurveStore(root, background=False))
  pts = TM.G.straight(0, 0, 90, 40) + TM.G.arc(40, -70, 70, 90, -90)[1:]
  seen = False
  for (x0, y0), (x1, y1) in zip(pts, pts[1:], strict=False):
    brg = math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0
    for k in range(5):
      x, y = x0 + (x1 - x0) * k / 5, y0 + (y1 - y0) * k / 5
      m.update(*TM.pos(x, y), brg, 31, True, gps={"dt": 0.1, "v_ego": 10.0})
      seen |= m.inside is not None and m.inside.get("ramp") is not None and m.inside["s_a"] == ramp_c["s_a"]
  assert seen


# ---- 6: the merge boost behind a moving lead (owner; drive 6f, the Dequindre ramp 419-451) ----

def _boost_run(lead_fn, frames=600, v_mph=40.0):
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=40 * MPH, v_ego=v_mph * MPH))
  set_mph, log = 40.0, []
  for f in range(frames):
    lv = lead_fn(f)
    b, _ = lg.update(_inp(set_speed=set_mph * MPH, v_ego=v_mph * MPH, limit_raise=55 * MPH, limit_boost=20 * MPH,
                          radar_lead=True, lead_v=lv, btn_counter=(f // 10) % 16))
    if b == cs.SET_PLUS:
      set_mph += 1
    elif b == cs.SET_MINUS:
      set_mph -= 1
    log.append((f, lg.anchor, None if lg.boost is None else dict(lg.boost), set_mph))
  return lg, log


def test_merge_boost_behind_a_lead_moving_away_and_its_end_when_it_slows():
  v = 40 * MPH
  # the lead 2.5 m/s faster than us: the boost opens, anchor at the target + 20
  lg, log = _boost_run(lambda f: v + 2.5)
  assert any(a is not None and abs(a / MPH - 75) < 0.5 for _, a, _, _ in log), [a for _, a, _, _ in log][-1]
  assert lg.boost is not None and "ended" not in lg.boost
  # ... then it slows to 5 m/s (a queue): the boost ends, the anchor back to the target
  lg, log = _boost_run(lambda f: v + 2.5 if f < 300 else 5.0, frames=2000)
  ended = next(f for f, a, b, _ in log if b is not None and "ended" in b)
  assert 300 <= ended <= 302, ended
  assert abs(log[ended][1] / MPH - 55) < 0.5 and max(s for _, _, _, s in log[:300]) > 55
  assert abs(log[-1][3] - 55) < 1e-6, log[-1]                  # the set speed down to the target
  # closing at 3 m/s or more: blocked (a raise to the target only)
  lg, log = _boost_run(lambda f: v - 3.0)
  assert all(bb is None for _, _, bb, _ in log) and max(st for _, _, _, st in log) <= 55 + 1e-6
  # the lead's speed unknown: blocked, as before
  lg, log = _boost_run(lambda f: -1.0)
  assert all(bb is None for _, _, bb, _ in log) and max(st for _, _, _, st in log) <= 55 + 1e-6
  # (no lead at all: the boost as before)
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=40 * MPH, v_ego=v))
  for _ in range(50):
    lg.update(_inp(set_speed=40 * MPH, v_ego=v, limit_raise=55 * MPH, limit_boost=20 * MPH))
  assert lg.boost is not None and abs(lg.anchor / MPH - 75) < 0.5


# ---- review of round L, F1: weak GPS past the split, a short false "link" or none, his gas held (rl/s1c.py, s1b.py) ----

def _false_link_weak(shape_fn, trust_fn, gas=None, frames=20000):
  """test_exit_plan._false_link with the GPS trust given: he stays on (set 80), navigation's exit at 3000."""
  ex = 3000.0
  F = TE.F

  def fields(x):
    hw = dict(road_class=1, limit=70 * MPH, gps_trust=trust_fn(x))
    if x < ex - 2500:
      return F(**hw)
    if x < ex - 46:
      return F(nav_exit=1, nav_exit_distance=ex - x, nav_curve_radius=85.0, nav_curve_distance=ex + 500 - x,
               nav_exit_side=1, exit_shape=shape_fn(x), **hw)
    return F(exit_shape=shape_fn(x), **hw)
  return TE.drive_exit(fields, frames, 80, lane=lambda x: 1 if ex - 300 < x < ex + 300 else None, gas=gas), ex


WEAK = lambda x: 2 if x > 2950 else 0  # noqa: E731


def test_review_l_f1_gas_held_overturns_a_quiet_false_link_with_weak_gps():
  sh = lambda x: 2 if 3050 < x < 3110 else 0  # noqa: E731
  for rel in (200, 400, 600):
    r, ex = _false_link_weak(sh, WEAK, gas=lambda x, rel=rel: 36.0 if 3150 <= x < 3000 + rel else None)
    p = r["ctl"].plan or r["ctl"].last_plan
    i0 = next(i for i, xx in enumerate(r["xs"]) if xx >= ex + rel)
    assert p.why == "overturned: gas" and min(r["vs"][i0:]) >= 77.0, (rel, p.why, min(r["vs"][i0:]))
    assert abs(r["sets"][-1] - 80) < 1e-6
  # (was: 81 -> 39 mph on the highway after the release at +200)
  old = CTL.GAS_QUIET_LINK
  try:
    CTL.GAS_QUIET_LINK = 1e9
    r, ex = _false_link_weak(sh, WEAK, gas=lambda x: 36.0 if 3150 <= x < 3200 else None)
  finally:
    CTL.GAS_QUIET_LINK = old
  assert min(r["vs"]) < 60.0


def test_review_l_f1_silent_shapes_and_his_gas_held_restore_his_speed_not_adopt():
  for sh in (lambda x: 0, lambda x: 2 if 3100 < x < 3130 else 0):
    r, ex = _false_link_weak(sh, WEAK, gas=lambda x: 36.0 if 3100 <= x < 4000 else None)
    p = r["ctl"].plan or r["ctl"].last_plan
    assert p.why == "not confirmed, his gas" and abs(r["sets"][-1] - 80) < 1e-6, (p.why, r["sets"][-1])
  # no gas: as before (held, then adopted)
  r, ex = _false_link_weak(lambda x: 0, WEAK)
  assert (r["ctl"].plan or r["ctl"].last_plan).why == "not confirmed"


def test_review_l_f1_the_rereview_case_with_weak_gps_too():
  # test_exit_plan.test_rereview_his_gas_held_past_the_split_is_his_decision_even_after_a_link, trusted and weak
  link_then_none = lambda x: 2 if 3000 < x < 3200 else 0  # noqa: E731
  for trust in (lambda x: 0, WEAK):
    r, ex = _false_link_weak(link_then_none, trust, gas=lambda x: 36.0 if 3300 <= x < 3360 else None, frames=17000)
    assert r["ctl"].last_plan.why == "overturned: gas", r["ctl"].last_plan.why
    assert TE._at(r, ex + 500, "sets") >= 79.0 and abs(r["sets"][-1] - 80) < 1e-6


def test_review_l_f1_nav_near_clears_when_navigation_drops_or_reroutes():
  def plan_at(rem_drop, other=False):
    ctl = CTL.MapCurveControl(3.0)
    base = dict(mode=CTL.ON, road_class=1, nav_exit_side=1)
    ctl.update(CTL.MapFields(nav_exit=1, nav_exit_distance=250.0, **base), 33.0, 0, v_ego=30.0)
    p = ctl.plan
    assert p.nav_near
    ctl.odo = p.exit_at - rem_drop
    f = CTL.MapFields(nav_exit=1, nav_exit_distance=rem_drop + 1500.0, **base) if other else CTL.MapFields(**base)
    ctl.update(f, 33.0, 0, v_ego=30.0)
    return p
  assert not plan_at(150.0).nav_near                  # cancelled 150 m before the exit point
  assert not plan_at(150.0, other=True).nav_near      # rerouted to an exit farther on
  assert plan_at(20.0).nav_near                       # the usual clearing at the split


# ---- review of round L, F3: a red light at a signalled turn onto an on-ramp (Dequindre type): the stop keeps running ----

def test_review_l_f3_a_red_light_stop_started_before_the_on_ramp_keeps_running(tmp_path, monkeypatch):
  c, sm, seen, WI = _wrapper(tmp_path, monkeypatch)
  plan = sm.msgs["starpilotPlan"]
  plan.mapCurveMode, plan.mapRoadClass = 2, 5
  WI._drive(c, 20, set_mph=35, v_mph=34)
  # the light seen 150 m before the turn onto the on-ramp (his signal: mapOnRampAhead), its first curve just past the turn
  plan.redLight = True
  plan.mapOnRampAhead, plan.mapCurveStatus, plan.mapCurveRadius = True, 2, 400.0
  d, log = 150.0, []
  while d > 0.0:
    plan.mapCurveDistance = d
    c.update(WI.W.CC, WI.W.car_state(34, 35), None, None, False)
    log.append((d, c.trigger.active, seen[-1].red_light))
    d -= 34 * MPH * cs.DT_CTRL
  start = next((dd for dd, act, _ in log if act), None)
  print("stop episode from", start)
  assert start is not None and start > cs.ONRAMP_RED_OFF_DIST, start
  # ... and it keeps the light and runs on to the turn (within ONRAMP_RED_OFF_DIST too)
  assert all(act and red for dd, act, red in log if dd <= start), [x for x in log if x[0] <= start and not (x[1] and x[2])][:3]
  # without an episode the camera stops stay off within ONRAMP_RED_OFF_DIST (test_6f_no_red_light_stop_...)
  assert cs.ONRAMP_RED_OFF_DIST == 50.0
