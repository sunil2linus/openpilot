"""Round U (drive 7e, round T's build): the fixes U1-U10. Unit tests on synthetic inputs only (no drive data)."""
import importlib.util
import os
from types import SimpleNamespace as NS

HERE = os.path.dirname(__file__)


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, path))
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


MPH = 0.44704
KPH = 1 / 3.6
DT = 0.01
CCm = _load("camera_curve_round_u", os.path.join("..", "camera_curve.py"))
W = _load("wcp_round_u", "test_car_profile_wrapper.py")
cs = W.cs


# ---------------- U1: no camera taps / keep on the approach to a governing map curve; capped at the follower's command ----

def _bend_cam(seconds=4.0, v_mph=40.0, set_mph=38.0, lat=1.2, **kw):
  """A bend at v_mph with a steady measured lateral `lat` (the model the same ahead): CameraCurve's taps and keep."""
  cc = CCm.CameraCurve()
  v = v_mph * MPH
  k0 = lat / (v * v)
  xs = [190.0 * (j / 32) ** 2 for j in range(33)]
  set_v, taps, keeps = set_mph * MPH, [], []
  for f in range(int(seconds / DT)):
    args = dict(v_ego=v, set_speed=set_v, lat=lat, model_x=xs, model_k=[k0] * 33, lane_prob=0.8, torque=0.3, curve_on=True)
    args.update(kw)
    o = cc.update(CCm.CamInputs(**args))
    keeps.append(o.keep)
    if o.tap:
      taps.append((f, o.tap_goal / MPH))
      set_v = o.tap_goal
  return taps, keeps, set_v / MPH, o


def test_u1_no_taps_and_no_keep_from_the_bend_before_while_a_map_curve_is_ahead():
  # the bend before the map curve: taps before round U (the lateral >= CAM_PHANTOM_LAT counted as "in the curve")
  taps0, keeps0, _, _ = _bend_cam(map_curve_id=(1, 90), gov_plan=45 * MPH)
  assert taps0 and any(keeps0)
  taps, keeps, s, o = _bend_cam(map_curve_id=(1, 90), gov_plan=45 * MPH, map_approach=True, gov_cmd=36 * MPH)
  assert not taps and not any(keeps) and s == 38.0 and o.why == "map curve approach"
  old = CCm.CAM_MAP_APPROACH
  try:
    CCm.CAM_MAP_APPROACH = False
    assert _bend_cam(map_curve_id=(1, 90), gov_plan=45 * MPH, map_approach=True)[0]
  finally:
    CCm.CAM_MAP_APPROACH = old


def test_u1_in_the_map_curve_taps_stop_at_the_followers_command_not_its_distance_plan():
  # in the curve: the distance plan 45, the follower's command 40
  taps, _, s, _ = _bend_cam(seconds=8.0, set_mph=37.0, map_in_curve=True, map_curve_id=(1, 90), gov_plan=45 * MPH,
                            gov_cmd=40 * MPH)
  assert taps and abs(s - 40.0) < 1e-6
  taps, _, s, o = _bend_cam(set_mph=40.0, map_in_curve=True, map_curve_id=(1, 90), gov_plan=45 * MPH, gov_cmd=40 * MPH)
  assert not taps and o.why == "map command"


def test_u1_post_apex_taps_unaffected():
  TR = _load("trr_round_u", "test_round_r.py")
  base_taps, base_s = TR._loop_cam()
  taps, s = TR._loop_cam(gov_cmd=30 * MPH, map_approach=False)
  assert taps and abs(s - base_s) < 1e-6 and base_taps == taps
  # (post-apex: the approach flag cannot stop them either)
  assert TR._loop_cam(gov_cmd=30 * MPH, map_approach=True)[0]
