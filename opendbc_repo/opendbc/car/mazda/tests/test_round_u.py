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


# ---------------- U4: a map ramp curve that "ends" while the loop goes on keeps its target ----------------

HELD = (True, 30 * MPH, 24 * MPH, 60, True, True, True, False)
FREE = (False, 100.0, 0.0, 0, False, False, False, False)


def _tail(lat_fn, frames=2000, angle=40.0, rate=0.0, cam_end_at=None, cam=None):
  t = cs.RampTail()
  for _ in range(100):
    t.update(True, HELD, 2.0, angle, 0.0, False)
  out, whys = [], []
  for f in range(frames):
    inp = FREE if cam is None else (True, cam * MPH, 0.0, 0, False, False, False, False)
    o = t.update(False, inp, lat_fn(f), angle, rate, cam_end_at is not None and f >= cam_end_at)
    out.append(o)
    if t.why:
      whys.append((f, t.why))
  return t, out, whys


def test_u4_the_loop_going_on_keeps_the_curves_target_until_the_lateral_is_down_1_s():
  t, out, whys = _tail(lambda f: 2.0 if f < 700 else 0.4)
  assert whys[0] == (0, "start") and whys[-1][1] == "lateral" and 700 + 100 - 2 <= whys[-1][0] <= 700 + 101, whys
  held = out[:whys[-1][0]]
  assert all(o[0] and abs(o[1] - 30 * MPH) < 1e-9 and o[4] and not o[5] for o in held)    # kept, no raise
  assert out[-1] == FREE
  # a lower camera curve target in the tail still counts
  _, out, _ = _tail(lambda f: 2.0, frames=50, cam=27.0)
  assert abs(out[-1][1] - 27 * MPH) < 1e-9


def test_u4_a_real_end_lets_go_at_once_and_the_other_ends():
  # the lateral low and the wheel unwinding at the map's end: no tail (as before)
  t, out, whys = _tail(lambda f: 0.3, rate=-20.0)
  assert not whys and out[0] == FREE and not t.active
  # the lateral low but the wheel held (not unwinding): a tail, over after 1 s under the limit
  t, out, whys = _tail(lambda f: 0.6)
  assert whys[0][1] == "start" and whys[-1][1] == "lateral" and whys[-1][0] <= 101, whys
  # the camera's curve end
  _, _, whys = _tail(lambda f: 2.0, cam_end_at=300)
  assert whys[-1] == (300, "camera end")
  # at most 15 s
  _, _, whys = _tail(lambda f: 2.0, frames=2000)
  assert whys[-1][1] == "max" and abs(whys[-1][0] - (cs.RAMP_TAIL_MAX / DT - 1)) <= 1
  old = cs.RAMP_TAIL
  try:
    cs.RAMP_TAIL = False
    assert not _tail(lambda f: 2.0)[2]
  finally:
    cs.RAMP_TAIL = old


def test_u4_the_tail_is_the_ramp_curve_for_the_merge():
  on = lambda **k: cs.CurveSlowdown._ramp_curve_on(NS(**k), NS(), False)  # noqa: E731
  assert on(map_out=None, ramp_tail=NS(active=True))
  assert not on(map_out=None, ramp_tail=NS(active=False))
  assert on(map_out=NS(active=True, track=True, why="ramp hold"), ramp_tail=None)
  assert not on(map_out=NS(active=True, track=True, why="curve"), ramp_tail=None)
  assert cs.CurveSlowdown._ramp_curve_on(NS(map_out=None, ramp_tail=None), NS(mapCurveStatus=2, mapCurveRadius=70.0), True)


# ---------------- U5: the on-ramp's raise does not wait for the map's next limit ----------------

def _ramp_to_run(next_fn, ramp_to=70, trust=0, why=0, acc=0.0, driver=35, frames=2000, link_at=600):
  f = slf.SpeedLimitFollower(False)
  offers = []
  for k in range(frames):
    on_link = k >= link_at
    li = slf.LimitInputs(limit=45 * MPH, source="Map Data", gps_trust=0 if not on_link else trust, driver_speed=driver * MPH,
                         cruise_enabled=True, road_class=LINK if on_link else STREET, ramp_to_highway=on_link,
                         next_limit=(next_fn(k - link_at) * MPH if on_link else 0.0), next_distance=-1.0, v_ego=driver * MPH,
                         gps_shaky_why=why if on_link else 0, gps_acc=acc, ramp_to_speed=ramp_to * MPH if on_link else 0.0)
    f.update(li)
    if f.offer is not None and f.offer.kind == "merge" and not offers:
      offers.append((k - link_at, f.offer.target))
  return offers, [e[1]["reason"] for e in f.events if e[0] == "speedLimitNotFollowed"]


def test_u5a_the_planners_ramp_to_speed_is_the_highways_limit_when_the_next_limit_is_missing_or_a_streets():
  for nxt in (lambda k: 0, lambda k: 45):
    ofs, _ = _ramp_to_run(nxt, ramp_to=0)
    assert not ofs                                                         # before: waits for mapd's next limit
    ofs, reasons = _ramp_to_run(nxt)
    assert ofs and ofs[0][1] == 75 and ofs[0][0] <= 2.0 / DT + 5, (ofs, reasons)
  # a real highway next limit wins; a ramp-to speed that is no highway limit is not used
  ofs, _ = _ramp_to_run(lambda k: 65, ramp_to=70)
  assert ofs and ofs[0][1] == 70
  assert not _ramp_to_run(lambda k: 0, ramp_to=35)[0]
  # the trust rules as for the normal merge: trust 2 waits; the start of the drive's trust 1 with good accuracy opens
  assert not _ramp_to_run(lambda k: 0, trust=2)[0]
  MCm = _load("mc_round_u", os.path.join("..", "..", "..", "..", "..", "starpilot", "controls", "lib", "map_curves.py"))
  assert _ramp_to_run(lambda k: 0, trust=1, why=MCm.SHAKY_START, acc=8.0)[0]
  old = slf.MERGE_RAMP_TO
  try:
    slf.MERGE_RAMP_TO = False
    assert not _ramp_to_run(lambda k: 0)[0]
  finally:
    slf.MERGE_RAMP_TO = old


def _res_run(frames=400, hwy_link=True, auto=True, v=31.6, set0=20.0, gas=False, **kw):
  """Cruise off (his brake), then cruise back at the old set 20 by our auto resume's RES on the link at 31.6 mph."""
  lg = cs.CurveSlowdownLogic(False, 10)
  for f in range(100):
    lg.update(_inp(set_speed=set0 * MPH, v_ego=20 * MPH, btn_counter=(f // 10) % 16))
  for f in range(200):
    lg.update(_inp(cruise_enabled=False, set_speed=0.0, v_ego=v * MPH, btn_counter=(f // 10) % 16))
  set_mph, presses = set0, []
  for f in range(frames):
    b, _ = lg.update(_inp(set_speed=set_mph * MPH, v_ego=v * MPH, auto_resumed=auto, hwy_link=hwy_link, gas_pressed=gas,
                          btn_counter=(f // 10) % 16, **kw))
    if b is not None:
      presses.append((f, b, lg.last_action))
      if b == cs.SET_PLUS:
        set_mph = (int(v) if lg.last_action == "jump" else set_mph + (5 if lg.last_action == "hold" and f % 10 == 0 else
                                                                      (1 if lg.last_action != "hold" else 0)))
  return lg, presses, set_mph


def test_u5b_our_res_on_the_link_with_an_old_low_set_takes_the_set_to_the_cars_speed():
  lg, presses, s = _res_run()
  assert presses and presses[0][1] == cs.SET_PLUS and presses[0][0] <= 100, presses[:3]
  assert abs(s - 32) < 1e-6, (s, presses[:5])                          # the car's 31.6 as a whole step
  # on his gas: the jump to the car's speed
  lg, presses, s = _res_run(gas=True)
  assert presses and any(a == "jump" for _, _, a in presses), presses[:3]
  # not: his own RES, not on a link to a highway, the set near the car's speed, a curve target or a slow car ahead
  for kw in (dict(auto=False), dict(hwy_link=False), dict(set0=28.0), dict(radar_lead=True, lead_v=20 * MPH),
             dict(red_light=True)):
    lg, presses, s = _res_run(**kw)
    assert not any(b == cs.SET_PLUS for _, b, _ in presses), kw
