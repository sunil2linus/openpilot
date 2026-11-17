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


# ---------------- U2: inside a map curve the set comes up to the curve's own target ----------------

def _inp(**k):
  d = dict(enabled=True, cruise_enabled=True, set_speed=20 * MPH, v_ego=25 * MPH, gas_pressed=False, brake_pressed=False,
           standstill=False, plan_alive=True, csc_active=False, csc_speed=0.0, other_button_active=False)
  d.update(k)
  return cs.SlowdownInputs(**d)


def _in_curve(frames=800, set0=30.0, v=31.0, own=33.25, own_fn=None, anchor=45.0, **kw):
  """Inside a followed loop: the curve target 30 (the camera's lowest), the car at v mph; returns the set speeds."""
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=set0 * MPH, v_ego=v * MPH))
  lg.anchor = anchor * MPH
  set_mph, sets = set0, []
  for f in range(frames):
    o = own_fn(f) if own_fn else own
    b, _ = lg.update(_inp(set_speed=set_mph * MPH, v_ego=v * MPH, csc_active=True, csc_speed=30 * MPH, curve_track=True, curve_floor=24 * MPH,
                          curve_budget=60, curve_own=o * MPH, btn_counter=(f // 10) % 16, **kw))
    if b == cs.SET_PLUS and lg.last_action != "hold":
      set_mph += 1
    elif b == cs.SET_MINUS and lg.last_action != "hold":
      set_mph -= 1
    sets.append(set_mph)
  return sets, lg


def test_u2_inside_the_curve_the_set_comes_up_to_its_own_target_rounded_down():
  sets, _ = _in_curve(own=0.0)
  assert max(sets) == 30.0                                              # before: left at the floor
  sets, _ = _in_curve()
  assert max(sets) == 33.0 and sets[-1] == 33.0                         # 33.25 -> 33, never 34
  sets, _ = _in_curve(own=33.0)
  assert max(sets) == 33.0
  # not while the car is more than 2 mph above the curve's own target
  assert max(_in_curve(v=35.5)[0]) == 30.0
  # never above his speed
  assert max(_in_curve(anchor=32.0)[0]) == 32.0


def test_u2_the_guards():
  for kw in (dict(stop_active=True, stop_target=20 * MPH), dict(hold_restore=True), dict(red_stop=True), dict(red_light=True),
             dict(model_plan_stop=True), dict(radar_lead=True, lead_v=20 * MPH), dict(driver_button=cs.SET_MINUS)):
    sets, _ = _in_curve(frames=300, **kw)
    assert max(sets) <= 30.0, kw
  old = cs.CURVE_OWN_RAISE
  try:
    cs.CURVE_OWN_RAISE = False
    assert max(_in_curve()[0]) == 30.0
  finally:
    cs.CURVE_OWN_RAISE = old


def test_u2_a_second_after_our_set_minus_and_the_lowest_target_held():
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=30 * MPH, v_ego=31 * MPH))
  lg.anchor = 45 * MPH
  lg.last_minus_frame = lg.frame
  first = None
  for f in range(300):
    b, _ = lg.update(_inp(set_speed=30 * MPH, v_ego=31 * MPH, csc_active=True, csc_speed=30 * MPH, curve_track=True,
                          curve_own=33.25 * MPH, btn_counter=(f // 10) % 16))
    if b == cs.SET_PLUS and first is None:
      first = f
  assert first is not None and first + 1 >= cs.MIN_DWELL / DT
  # the curve tightens (own 31.4 from 2 s), then opens again (34): held at 31, no SET- / SET+ churn
  sets, _ = _in_curve(frames=1200, own_fn=lambda f: 33.25 if f < 200 else (31.4 if f < 600 else 34.0))
  assert max(sets[:200]) == 33.0 and sets[-1] == 31.0 and max(sets[700:]) == 31.0


def test_u2_the_wrapper_gives_the_own_target_only_inside_a_followed_curve():
  def w(**k):
    d = dict(ti_only=False, lkas_blocked=False, ti_fault=False, cam_curve=NS(over=False),
             map_out=NS(why="ramp", curve_speed=33.25 * MPH, active=True, track=True), _followed_map_curve=lambda: True)
    d.update(k)
    return NS(**d)
  plan = NS(mapCurveStatus=2, mapCurveRadius=80.0, mapCurveDistance=0.0)
  own = lambda o, p=plan, act=True, street=False: cs.CurveSlowdown._curve_own(o, p, act, street)  # noqa: E731
  assert abs(own(w()) - 33.25 * MPH) < 1e-9
  assert own(w(), NS(mapCurveStatus=2, mapCurveRadius=80.0, mapCurveDistance=40.0)) == 0.0     # still ahead
  assert abs(own(w(map_out=NS(why="ramp hold", curve_speed=33.25 * MPH)), NS()) - 33.25 * MPH) < 1e-9
  for k in (dict(ti_only=True), dict(lkas_blocked=True), dict(ti_fault=True), dict(cam_curve=NS(over=True)),
            dict(_followed_map_curve=lambda: False)):
    assert own(w(**k)) == 0.0, k
  assert own(w(), street=True) == 0.0 and own(w(), act=False) == 0.0


# ---------------- U3: his SET- in a ramp curve is a curve speed, not a merge decision ----------------

slf = _load("slf_round_u", os.path.join("..", "speed_limit_follow.py"))
STREET, LINK = 5, 2


def _minus_run(minus_at, curve=(0, 500), next_from=600, frames=2200, link_at=600, flag=True):
  """A street (45), a link to a highway from link_at; his SET- (45 -> 30) at link + minus_at; a ramp curve on the link from
  link + curve[0] to link + curve[1]; the highway's 70 the next limit from link + next_from."""
  f = slf.SpeedLimitFollower(False)
  offers = []
  old = slf.RAMP_CURVE_MINUS
  slf.RAMP_CURVE_MINUS = flag
  try:
    for k in range(frames):
      kl = k - link_at
      on_link = k >= link_at
      d = 30 if (on_link and kl >= minus_at) else 45
      li = slf.LimitInputs(limit=45 * MPH, source="Map Data", gps_trust=0, driver_speed=d * MPH, cruise_enabled=True,
                           road_class=LINK if on_link else STREET, ramp_to_highway=on_link,
                           next_limit=(70 * MPH if (on_link and kl >= next_from) else 0.0),
                           next_distance=400.0 if on_link else -1.0, v_ego=d * MPH, driver_set=on_link and kl == minus_at,
                           driver_minus=on_link and kl == minus_at, ramp_curve=on_link and curve[0] <= kl < curve[1])
      f.update(li)
      if f.offer is not None and f.offer.kind == "merge" and not offers:
        offers.append((kl, f.offer.target, f.offer.driver))
  finally:
    slf.RAMP_CURVE_MINUS = old
  reasons = [e[1]["reason"] for e in f.events if e[0] == "speedLimitNotFollowed"]
  return offers, reasons, f


def test_u3_his_set_minus_in_the_ramp_curve_keeps_the_merge_which_opens_after_the_curve_from_his_speed():
  ofs, reasons, _ = _minus_run(300, flag=False)
  assert not ofs and "merge: the driver set his speed lower on the link" in reasons      # before: off for the link
  ofs, reasons, f = _minus_run(300)
  assert ofs and ofs[0][0] >= 600 + 200 and ofs[0][1] == 75 and ofs[0][2] == 30, (ofs, reasons)
  assert "merge: the driver set his speed lower on the link" not in reasons
  # the highway's limit known early: the merge waits for the curve's end, then its usual steady time
  ofs, reasons, _ = _minus_run(300, curve=(0, 900), next_from=100)
  assert ofs and ofs[0][0] >= 900 + 200, ofs
  assert "merge: his SET- in the ramp curve (waiting for its end)" in reasons


def test_u3_his_set_minus_after_the_curve_or_with_no_curve_still_declines():
  ofs, reasons, _ = _minus_run(550)                         # the curve ended at 500
  assert not ofs and "merge: the driver set his speed lower on the link" in reasons
  ofs, reasons, _ = _minus_run(300, curve=(0, 0))
  assert not ofs and "merge: the driver set his speed lower on the link" in reasons
  # (the SET- in the curve is marked as a curve speed, not his decision on the link)
  f = _minus_run(300, frames=600 + 520)[2]
  assert f._curve_minus_on_link() and not f._his_minus_on_link()
