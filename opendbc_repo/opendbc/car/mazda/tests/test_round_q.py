"""Round Q (drives 74 / 75): the logged scenarios as tests (synthetic geometry only)."""
import dataclasses
import importlib.util
import os

HERE = os.path.dirname(__file__)


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


TE = _load("tep_round_q", os.path.join(HERE, "test_exit_plan.py"))
CTL, cs, F = TE.CTL, TE.cs, TE.F
MPH = cs.MS_PER_MPH
DT = cs.DT_CTRL


# ---- Q1: exit 48 (drive 75 t 1227-1260) ----

def _stale(fields, exit_x, every=62.0):
  """Navigation's maneuverDistance as it comes (~1 Hz, sometimes 2 s apart: held for `every` m, never counted down
  between); the map's link start (mapExitStart) fresh every frame."""
  def f(x):
    fl = fields(x)
    if fl.nav_exit_distance > 0:
      held = exit_x - (x - (x % every))
      fl = dataclasses.replace(fl, nav_exit_distance=held, exit_start=max(exit_x - x, 0.0))
    return fl
  return f


def _e48(reach=True, signal=True, every=62.0, stay=False, shape_from=150.0, gas=None):
  """Exit 48: 75 set at 70 mph, the 113 m exit curve (35 mph) 262 m past the split, his right signal 370-300 m before it,
  GPS trust 2 from the split, the shapes silent past the split until `shape_from` m ("link"; "highway" when he stays)."""
  exit_x, curve_x = 2000.0, 2262.0
  base = TE.exit_fields(exit_x, curve_x, r=113.0, ann=1000.0, clear=10.0, link_at=shape_from, ramp_at=shape_from + 40,
                        curve_held=100.0, stay=stay, stay_from=shape_from)
  fields = _stale(base, exit_x, every)
  sig = (lambda x: 1 if exit_x - 370 <= x < exit_x - 300 else 0) if signal else None
  old = CTL.TENT_REACH
  CTL.TENT_REACH = reach
  try:
    r = TE.drive_exit(fields, 9000, 75, v_mph=70, signal=sig, lane=lambda x: 1 if exit_x - 450 <= x < exit_x + 60 else None,
                      gas=gas)
  finally:
    CTL.TENT_REACH = old
  return r, exit_x, curve_x


def test_q1_navigations_stale_distance_does_not_reroute_and_his_signal_is_kept():
  r, exit_x, curve_x = _e48()
  ends = [p for p in (r["ctl"].last_plan,) if p is not None and p.why in CTL.NAV_REVIVE]
  assert not ends, ends
  # (base: start_off took up each held value's error, the next fresh one was a whole update off: "rerouted" ~380 m out)
  p = r["ctl"].last_plan or r["ctl"].plan
  assert p.signalled and p.signal_odo is not None and p.exit_at - p.signal_odo <= CTL.NAV_SIGNAL_REACH


def test_q1_a_rerouted_plan_is_taken_on_again_with_his_signal():
  exit_x, curve_x = 2000.0, 2262.0
  base = TE.exit_fields(exit_x, curve_x, r=113.0, ann=1000.0, clear=10.0, link_at=150.0, ramp_at=190.0, curve_held=100.0)

  def fields(x):
    f = base(x)
    if exit_x - 290 <= x < exit_x - 250:                 # navigation shows another exit 1 km on for ~1.3 s
      f = dataclasses.replace(f, nav_exit_distance=exit_x + 1000 - x, exit_start=0.0)
    return f
  r = TE.drive_exit(fields, 9000, 75, v_mph=70, signal=lambda x: 1 if exit_x - 370 <= x < exit_x - 300 else 0)
  p = r["ctl"].plan or r["ctl"].last_plan
  assert p.revived >= 1 and p.signalled, (p.revived, p.signalled, p.why)


def test_q1_exit_48_arrives_at_the_curve_at_its_speed():
  r, exit_x, curve_x = _e48()
  v_c = TE._at(r, curve_x)
  assert v_c <= 39.0, v_c
  assert "exit reach" in r["why"] or "exit reach hold" in r["why"]
  # without the reachable profile (round P's tentative bounds alone) it comes in faster
  r0, _, _ = _e48(reach=False)
  assert TE._at(r0, curve_x) > v_c + 1.5, (TE._at(r0, curve_x), v_c)


def test_q1_the_reachable_profile_needs_his_signal_and_lets_go_at_highway():
  # the camera's edge lane alone (the ordinary right lane): never
  r, exit_x, _ = _e48(signal=False)
  assert "exit reach" not in r["why"] and "exit reach hold" not in r["why"]
  # he stays on: the shapes say "highway" 60 m past the split; the profile let go, his speed back
  r, exit_x, _ = _e48(stay=True, shape_from=60.0)
  p = r["ctl"].last_plan
  assert p.why == "missed", p.why
  assert abs(r["sets"][-1] - 75) < 1e-6
  past = [v for v, xx in zip(r["vs"], r["xs"], strict=True) if xx >= exit_x]
  assert min(past) >= TE._at(r, exit_x) - CTL.TENT_REACH_DIP / MPH - 1.0
  # nothing answers at all (GPS trust 2, no shapes): never more than TENT_REACH_DIP under the speed at the split (+ lag)
  r, exit_x, _ = _e48(shape_from=5000.0)
  past = [v for v, xx in zip(r["vs"], r["xs"], strict=True) if exit_x <= xx]
  assert min(past) >= TE._at(r, exit_x) - CTL.TENT_REACH_DIP / MPH - 3.0, (TE._at(r, exit_x), min(past))


def test_q1_his_gas_held_lets_the_reachable_profile_go():
  exit_x = 2000.0
  r, exit_x, _ = _e48(shape_from=5000.0, gas=lambda x: 62 * MPH if exit_x + 40 <= x < exit_x + 300 else None)
  p = r["ctl"].plan or r["ctl"].last_plan
  assert p.reach_off == "his gas", p.reach_off
  later = [w for w, xx in zip(r["why"], r["xs"], strict=True) if exit_x + 120 <= xx < exit_x + 300]
  assert "exit reach" not in later and "exit reach hold" not in later


# ---- Q2: the merge boost at a loop's end (drive 75 t 1158.6-1159.1; drive 73's loop start) ----

TP = _load("trp_round_q", os.path.join(HERE, "test_round_p.py"))


def _loop_end(csc_fn, frames=1500):
  def inp(f, car, lg):
    v = csc_fn(f, car)
    return {"csc_active": v is not None, "csc_speed": (v or 0.0) * MPH}
  return TP._boost73(frames=frames, inp=inp)


def test_q2_a_curve_target_rising_out_of_the_loop_keeps_the_boost():
  # drive 75: the camera target 67 -> 69 mph at 33-34 mph for 1.5 s from the merge's adoption (was: ended after 0.5 s,
  # anchor 75)
  car, lg, log = _loop_end(lambda f, car: 67.0 + 2.0 * min(max(f - 50, 0) / 150.0, 1.0) if 53 <= f < 200 else None)
  assert max(s for _, s, *_ in log) >= 89.5, max(s for _, s, *_ in log)
  # drive 73's loop start (the 72.8 mph release ramp, 0.8 s): the same
  car, lg, log = _loop_end(lambda f, car: 72.8 if 53 <= f < 133 else None)
  assert max(s for _, s, *_ in log) >= 89.5


def test_q2_a_real_tight_curve_still_ends_it():
  # a curve asking for no more than the speed + 3 mph, for MERGE_CSC_END: ended, the anchor at the target
  car, lg, log = _loop_end(lambda f, car: car.v_ego / MPH + 3.0 if 60 <= f < 400 else None)
  assert lg.boost is None or "ended" in lg.boost
  assert max(s for _, s, *_ in log) <= 75 + 1e-6
  # a target far above the speed that comes down to it later (a curve ahead coming into view) ends it then
  car, lg, log = _loop_end(lambda f, car: (72.0 if f < 300 else car.v_ego / MPH + 2.0) if 60 <= f < 600 else None)
  assert lg.boost is None or "ended" in lg.boost


# ---- Q3: the camera-led curve handling (camera_curve.py) ----

CC = _load("camera_curve_round_q", os.path.join(HERE, "..", "camera_curve.py"))


def _path(kappa_fn, n=33, x_max=190.0):
  xs = [x_max * (k / (n - 1)) ** 2 for k in range(n)]      # (modelV2's points crowd near the car)
  return xs, [kappa_fn(x) for x in xs]


def _cam_run(seconds, v_mph=35.0, set_mph=33.0, lat=1.2, kappa=None, **kw):
  """A steady curve: the measured lateral `lat`, the model's plan with kappa(x) (default: the same curve all along)."""
  cc = CC.CameraCurve()
  v = v_mph * MPH
  k0 = (lat(10 ** 6) if callable(lat) else lat) / (v * v)
  xs, ks = _path(kappa or (lambda x: k0))
  set_v, taps, outs = set_mph * MPH, [], []
  for f in range(int(seconds / DT)):
    args = dict(v_ego=v, set_speed=set_v, lat=lat(f) if callable(lat) else lat, model_x=xs, model_k=ks, lane_prob=0.8, torque=0.4, curve_on=True,
                map_in_curve=True, map_curve_id=(1, 100))
    args.update({k: (val(f) if callable(val) else val) for k, val in kw.items()})
    o = cc.update(CC.CamInputs(**args))
    outs.append(o)
    if o.tap:
      taps.append(f)
      set_v = o.tap_goal
  return taps, set_v / MPH, outs


def test_q3_model_peak_and_its_horizon():
  xs, ks = _path(lambda x: 0.002 if 40 <= x <= 60 else 0.0)
  v = 20.0
  assert abs(CC.model_peak(xs, ks, v, 0.5, 3.0) - 0.002 * v * v) < 1e-9
  assert CC.model_peak(xs, ks, v, 0.5, 1.5) == 0.0           # (40-60 m is past 1.5 s at 20 m/s)
  assert CC.model_peak(xs[:5], ks[:5], v, 0.5, 3.0) is None   # (the path does not reach that far: no answer)


def test_q3_in_curve_taps_every_half_second_up_to_the_speed_plus_one():
  taps, set_end, _ = _cam_run(5.0)
  assert taps and all(b - a >= int(CC.CAM_TAP_DT / DT) for a, b in zip(taps, taps[1:], strict=False))
  assert abs(set_end - 36.0) < 1e-6                         # 33 -> 36: round(35) + 1 at most


def test_q3_no_tap_without_the_margin_or_the_reserve():
  # 1.75 m/s^2 now: at +1 mph 1.85 >= CAM_LAT_LKAS
  taps, _, outs = _cam_run(3.0, lat=1.75)
  assert not taps and outs[-1].why == "lateral"
  # the TI alone (LKAS blocked): CAM_LAT_TI
  taps, _, outs = _cam_run(3.0, lat=1.35, ti_only=True)
  assert not taps
  # tightening ahead (the model's 0.5-2 s peak well above now)
  v = 35 * MPH
  taps, _, outs = _cam_run(3.0, kappa=lambda x: 1.6 / (v * v) if x > 20 else 1.2 / (v * v))
  assert not taps and outs[-1].why == "tightening" and not outs[-1].keep
  # the steering command near its limit
  taps, _, outs = _cam_run(3.0, torque=0.9)
  assert not taps and outs[-1].why == "steering reserve"
  # a map curve ahead whose plan is under the next step (drive 56: a loop just beyond the model's horizon)
  taps, _, outs = _cam_run(3.0, map_ahead=33.2 * MPH)
  assert not taps and outs[-1].why == "map curve ahead"
  # the street / limit / merge target
  taps, _, outs = _cam_run(3.0, cap=33.5 * MPH)
  assert not taps and outs[-1].why == "cap"
  # no lane lines, too slow
  assert not _cam_run(3.0, lane_prob=0.2)[0] and not _cam_run(3.0, v_mph=24.0, set_mph=22.0)[0]


def test_q3_never_with_a_car_ahead_a_stop_his_steering_a_signal_a_fault_or_a_test_mode():
  for kw in (dict(lead_slow=True), dict(stop=True), dict(steer_override=True), dict(blinker=True), dict(cam_fault=True),
             dict(test_mode=True), dict(blocked=True)):
    taps, _, outs = _cam_run(3.0, **kw)
    assert not taps and not outs[-1].keep, kw
  # a signal on a ramp is not a lane change
  assert _cam_run(3.0, blinker=True, on_ramp=True)[0]


def test_q3_no_tap_while_the_lateral_rises_and_nothing_after_a_curve_driven_too_fast():
  # the curve entry: the measured lateral 0.6 -> 1.5 in 1 s (the 2 Hz model under-reads a tightening, drive 74's exit 46)
  taps, _, outs = _cam_run(1.5, lat=lambda f: min(0.6 + 0.009 * f, 1.5))
  assert not [t for t in taps if t < 110], taps
  # the turnaround at 3.6 m/s^2 (drive 74 t 1111): no tap, no end for the rest of that slowdown
  cc = CC.CameraCurve()
  v = 27 * MPH
  xs, ks = _path(lambda x: 0.0)
  outs = []
  for f in range(400):
    lat = 3.6 if f < 100 else 0.4
    outs.append(cc.update(CC.CamInputs(v_ego=v, set_speed=25 * MPH, lat=lat, model_x=xs, model_k=ks, lane_prob=0.8,
                                       torque=0.5, curve_on=True, map_in_curve=True)))
  assert not any(o.tap or o.ending for o in outs)


def test_q3_the_curve_ending_from_the_model_ahead():
  v = 35 * MPH

  # in the curve 2 s, then the model sees the road straighten 2 s ahead (x > 2 s at v)
  def kap(f):
    return (lambda x: 1.4 / (v * v)) if f < 200 else (lambda x: 1.4 / (v * v) if x < 1.5 * v else 0.0)
  cc = CC.CameraCurve()
  ends = []
  for f in range(400):
    xs, ks = _path(kap(f))
    o = cc.update(CC.CamInputs(v_ego=v, set_speed=30 * MPH, lat=1.4, model_x=xs, model_k=ks, lane_prob=0.8, torque=0.5,
                               curve_on=True, map_in_curve=True))
    if o.ending:
      ends.append(f)
  assert ends and 200 <= ends[0] <= 205, ends[:1]
  # never without a curve actually driven (the approach: the lateral still low)
  cc = CC.CameraCurve()
  xs, ks = _path(lambda x: 0.0)
  outs = [cc.update(CC.CamInputs(v_ego=v, set_speed=30 * MPH, lat=0.3, model_x=xs, model_k=ks, lane_prob=0.8,
                                 curve_on=True)) for _ in range(300)]
  assert not any(o.ending for o in outs)


def test_q3_phantom_release_on_a_straight_road_past_the_map_curves_start():
  v = 40 * MPH
  xs, ks = _path(lambda x: 0.0)
  cc = CC.CameraCurve()
  outs = [cc.update(CC.CamInputs(v_ego=v, set_speed=36 * MPH, lat=0.2, model_x=xs, model_k=ks, lane_prob=0.8,
                                 curve_on=True, map_in_curve=True, map_curve_id=(1, 142))) for _ in range(80)]
  assert not outs[30].phantom and outs[-1].phantom
  assert [k for o in outs for k, _ in o.events if k == "camPhantomRelease"] == ["camPhantomRelease"]
  # another map curve: not released
  o = cc.update(CC.CamInputs(v_ego=v, set_speed=36 * MPH, lat=0.2, model_x=xs, model_k=ks, lane_prob=0.8, curve_on=True,
                             map_in_curve=True, map_curve_id=(1, 90)))
  assert not o.phantom
  # a real curve (the lateral up): never
  cc = CC.CameraCurve()
  outs = [cc.update(CC.CamInputs(v_ego=v, set_speed=36 * MPH, lat=1.3, model_x=xs, model_k=ks, lane_prob=0.8,
                                 curve_on=True, map_in_curve=True, map_curve_id=(1, 142))) for _ in range(200)]
  assert not any(o.phantom for o in outs)


def test_q3_the_wrapper_raises_the_curve_target_by_its_taps_and_logs_them(tmp_path, monkeypatch):
  import pytest
  pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)
  from types import SimpleNamespace as N
  W = _load("tcpw_round_q", os.path.join(HERE, "test_car_profile_wrapper.py"))
  c = W.make_cs(tmp_path, W.FakeParams({"IsMetric": False}), monkeypatch)
  assert c.cam_curve is not None                               # (MazdaCameraCurveRaise: on by default)
  records = []
  if c.recorder is not None:
    monkeypatch.setattr(c.recorder, "record", lambda kind, t, data, **kw: records.append(kind))
  v = 35 * MPH
  xs = [190.0 * (k / 32) ** 2 for k in range(33)]
  m = c.sm.msgs["modelV2"]
  m.position, m.velocity = N(x=xs, y=[0.0] * 33), N(x=[v] * 33, t=[0.0] * 33)
  m.orientationRate = N(z=[1.2 / v] * 33)                      # 1.2 m/s^2 at 35 mph all along
  m.laneLineProbs = [0.5, 0.9, 0.9, 0.5]
  c.sm.updated["modelV2"] = True
  cs_ = W.car_state(35, 33)
  cs_.out.steeringAngleDeg = 1.2 * 18.9 * 2.7 * (1 + 0.001 * v * v) / (v * v) * 57.29578
  c.logic.anchor = 45 * MPH
  plan = c.sm.msgs["starpilotPlan"]
  got = []
  for _ in range(120):
    c.frame += 1
    r = c._camera_curve(W.CC, cs_, N(wheelbase=2.7), plan, True, v, 33 * MPH, 45 * MPH, True, 33 * MPH, 0.0, 0, False,
                        False, False, False, False, False, True, 33 * MPH)
    got.append(r)
  assert any(r[1] >= 34 * MPH - 1e-3 and r[4] and r[5] for r in got), got[-1]
  assert "camCurveTap" in records
  # his steering: nothing raised, the target as it came
  cs_.out.steeringPressed = True
  r = c._camera_curve(W.CC, cs_, N(wheelbase=2.7), plan, True, v, 33 * MPH, 45 * MPH, True, 33 * MPH, 0.0, 0, False,
                      False, False, False, False, False, True, 33 * MPH)
  assert abs(r[1] - 33 * MPH) < 1e-6 and not r[4]


# ---- Q5: a known lower limit caps every raise of ours (drive 74 t 1104-1142, t 532) ----

SLF = _load("slf_round_q", os.path.join(HERE, "..", "speed_limit_follow.py"))


def _sli(limit, d, trust=0, pending=False, name="Rochester Road", nav=0.0, turned=False):
  return SLF.LimitInputs(limit=limit * MPH, source="Map Data", gps_trust=trust, driver_speed=d * MPH, cruise_enabled=True,
                         road_class=7, road_name=name, pending=pending, nav_limit=nav * MPH, v_ego=d * MPH, turned=turned,
                         nav_dist=1000.0)


def _sl_run(steps, f=None):
  f = f or SLF.SpeedLimitFollower(False)
  for n, kw in steps:
    for _ in range(int(n / DT)):
      f.update(_sli(**kw))
  return f


def test_q5_a_pending_restore_is_capped_at_a_lower_limit_in_any_band():
  # drive 74: our exit handover's 53 on a 45 road (out of its band), the set lowered to 30 for the turnaround; the 35 road
  f = _sl_run([(12, dict(limit=45, d=53)), (12, dict(limit=35, d=53, pending=True))])
  assert abs(f.cap_speed / MPH - 40) < 1e-6, f.cap_speed / MPH       # 35 + the Street Speed offset (+5)
  assert f.cap is not None and not f.cap.band
  # nothing pending (his own 53): no cap; rule 9 may slow him as before (MazdaSlowLowerLimit)
  f = _sl_run([(12, dict(limit=45, d=53)), (12, dict(limit=35, d=53))])
  assert f.cap_speed == 0.0


def test_q5_one_second_of_gps_trust_2_no_longer_disqualifies_the_lower_limit():
  # (his 50 on the 45 road: at the street's target already, nothing for rule 8)
  steps = [(12, dict(limit=45, d=50)), (1.0, dict(limit=35, d=50, trust=2)), (10, dict(limit=35, d=50))]
  f = _sl_run(steps)
  assert f.lower is not None and f.lower.kind == "lower" and abs(f.lower.tgt - 40) < 0.6, f.lower
  # (still: trust 2 within the last LIMIT_TRUST_RECENT of its steady time keeps it waiting)
  f = _sl_run([(12, dict(limit=45, d=50)), (3.5, dict(limit=35, d=50)), (0.5, dict(limit=35, d=50, trust=2)),
               (2.0, dict(limit=35, d=50))])
  assert f.lower is None


def test_q5_a_stale_navigation_limit_after_a_road_change_does_not_block_slowing():
  # drive 74 t 532: navigation's 50 from the route step before the turn, the map's 45 on the new road: 53 was kept
  def steps(nav_after):
    return [(12, dict(limit=50, d=55, nav=50.0, name="Old Road")), (1, dict(limit=50, d=55, nav=50.0, name="New Road", turned=True)),
            (5, dict(limit=50, d=55, nav=50.0, name="New Road")), (20, dict(limit=45, d=55, nav=nav_after, name="New Road"))]
  f = _sl_run(steps(50.0))
  assert f.lower is not None and abs(f.lower.tgt - 50) < 0.6, f.lower       # 45 + 5 (was: max(nav 50, map 45) + 5 = 55)
  # navigation's limit changed after the road change: it still counts (the higher of the two, as before)
  f = _sl_run(steps(48.0))
  assert f.lower is not None and abs(f.lower.tgt - 53) < 0.6, f.lower


# ---- Q4: the phantom curve where a road splits into its two one-way sides (synthetic geometry) ----

TMC = _load("tmc_round_q", os.path.join(HERE, "test_map_curves.py"))
MC = TMC.MC
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
BCM = _load("bcm_round_q", os.path.join(ROOT, "tools", "curve_map", "build_curve_map.py"))


def test_q4_a_later_way_is_never_followed_against_its_one_way_direction():
  from types import SimpleNamespace as N
  from array import array
  mcs = MC.MapCurves(MC.MapCurveStore(__import__("tempfile").mkdtemp(), background=False))
  mcs.store = N(grid=[N(way=array("q", [1, 2]), oneway=array("b", [1, 0]))])     # (the store's RoadIndex list)
  c = {"oneway": True, "wdir": [[1, 1]]}
  assert mcs._against_oneway(c, increasing=False)            # b -> a on a way one-way a -> b
  assert not mcs._against_oneway(c, increasing=True)
  assert not mcs._against_oneway({"oneway": True, "wdir": [[2, 1]]}, increasing=False)   # (the two-way way)
  assert not mcs._against_oneway({"oneway": False, "wdir": [[1, 1]]}, increasing=False)


def test_q4_the_builder_drops_the_jog_where_a_road_splits_into_its_one_way_sides():
  from types import SimpleNamespace as N
  ways = {1: N(name="Main Road", ref="", oneway=False), 2: N(name="Main Road", ref="", oneway=True),
          3: N(name="Other Road", ref="", oneway=True)}

  def cv(s_a, s_b, d, turn, wset):
    return {"s_a": s_a, "s_b": s_b, "dir": d, "turn": turn, "ways": wset}
  jog = [cv(0, 30, "L", 22.0, [1, 2]), cv(70, 100, "R", 18.0, [2])]
  assert BCM.split_doublets(jog, ways) == []
  # a real S-bend (same name, both two-way), one far apart, one not cancelling, another road: kept
  assert len(BCM.split_doublets([cv(0, 30, "L", 22.0, [1]), cv(70, 100, "R", 18.0, [1])], ways)) == 2
  assert len(BCM.split_doublets([cv(0, 30, "L", 22.0, [1, 2]), cv(120, 150, "R", 18.0, [2])], ways)) == 2
  assert len(BCM.split_doublets([cv(0, 30, "L", 40.0, [1, 2]), cv(70, 100, "R", 18.0, [2])], ways)) == 2
  assert len(BCM.split_doublets([cv(0, 30, "L", 22.0, [1, 3]), cv(70, 100, "R", 18.0, [3])], ways)) == 2


def test_q4_two_trusted_straight_passes_make_a_ghost_and_a_real_pass_clears_it(tmp_path):
  class P:
    def get(self, key, encoding=None):
      return None
  mem = MC.CurveMemory(str(tmp_path / "mem.json"))
  c = {"apex": (10.0, 10.0), "r": 142.0, "s_a": 0.0, "turn": 40.0, "name": "Main Road"}

  def run(samples, trust0=True, blinker=False):
    mp = MC.MapCurvesPlanner(P(), P())
    mp.mc = MC.MapCurves(MC.MapCurveStore(__import__("tempfile").mkdtemp(), background=False), memory=mem)
    mp.pass_curve, mp.pass_samples = c, samples
    mp.pass_blinker, mp.pass_apex, mp.pass_exit, mp.pass_vetoed, mp.pass_shaky = blinker, 5.0, 16.0, False, False
    mp.pass_trust0 = trust0
    mp._finish_pass()
  straight = TMC._pass(0, seconds=6.0)
  run(straight, trust0=False)
  assert mem.ghost(c) == 0                                    # (GPS trust not 0 throughout: no mark)
  run(straight, blinker=True)
  assert mem.ghost(c) == 0                                    # (his signal: no mark)
  run(straight)
  run(straight)
  assert mem.ghost(c) == MC.PASS_GHOST_COUNT
  import math
  run(TMC._pass(142, seconds=math.radians(40) * 142 / 18.0))  # it really turned through it
  assert mem.ghost(c) == 0


def test_q4_a_lead_on_a_straight_path_in_our_lane_does_not_blind_the_camera_check():
  import math
  s = [float(x) for x in range(0, 200, 10)]
  cam = lambda psi, y: MC.CameraView(s, psi, [0.1] * len(s), lead_d=40.0, lead_y=y, v_ego=15.0)  # noqa: E731
  assert cam([0.0] * len(s), 0.2).blind(60.0) is None
  assert cam([0.0] * len(s), 2.0).blind(60.0) == "lead"                       # (the lead beside our lane's middle)
  assert cam([math.radians(x / 10) for x in s], 0.2).blind(60.0) == "lead"    # (the path curving)
  assert MC.BLIND_CODES["lead"] == 6
