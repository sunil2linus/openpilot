"""Round S, Curve Speed Offset (MazdaCurveSpeedOffset; map_curves.CURVE_OFFSET_*, map_curve_control.OFFSET_*): the car side
adds the planner's granted offset where the map curve's speed is made; the planner's real publish path. Synthetic geometry."""
import importlib.util
import math
import os
import tempfile

HERE = os.path.dirname(__file__)


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, path))
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


TMC = _load("tmc_round_s", "test_map_curves.py")
MC, CTL, G = TMC.MC, TMC.CTL, TMC.G
CCm = _load("camera_curve_round_s", os.path.join("..", "camera_curve.py"))
MPH = 0.44704
DT = 0.01


def F(**kw):
  f = CTL.MapFields(mode=CTL.ON)
  for k, v in kw.items():
    setattr(f, k, v)
  return f


def _ctl(ok=True, ti=True):
  c = CTL.MapCurveControl(3.0)
  c.offset_ok, c.ti_plan = ok, ti
  return c


# ---------------- the car side: one number for the curve's speed ----------------

def test_the_offset_is_added_to_the_curves_speed_its_plan_and_its_floor():
  g = 0.9 * MPH
  for st, ramp_frames in ((1, 0), (2, 200)):
    base, new = _ctl(), _ctl()
    for c, ofs in ((base, 0.0), (new, g)):
      for _ in range(ramp_frames + 1):
        o = c.update(F(status=st, radius=131.0, distance=0.0, on_ramp=st == 2, speed_offset=ofs), 45 * MPH, 0, v_ego=40 * MPH,
                     max_drop=10 * MPH)
      if c is base:
        ob = o
    assert ob.active and o.active
    assert abs(o.curve_speed - (ob.curve_speed + g)) < 1e-6, (st, ob.curve_speed, o.curve_speed)
    assert abs(new.offset_now - g) < 1e-9 and base.offset_now == 0.0
    # the plan ahead (approach timing) is the offset curve's
    rb = _ctl().update(F(status=1, radius=131.0, distance=120.0), 50 * MPH, 0)
    rn = _ctl().update(F(status=1, radius=131.0, distance=120.0, speed_offset=g), 50 * MPH, 0)
    assert rb.active and rn.active and rn.speed > rb.speed
  # a road curve under the Max Drop floor: the floor stays the floor (the offset only lifts a curve above it)
  o = _ctl().update(F(status=1, radius=60.0, distance=0.0, speed_offset=g), 45 * MPH, 0, v_ego=36 * MPH, max_drop=10 * MPH)
  assert abs(o.curve_speed - 35 * MPH) < 1e-6
  # the camera's view of the plan ahead (map_ahead) the same
  c = _ctl()
  c.update(F(status=1, radius=131.0, distance=300.0, speed_offset=g), 50 * MPH, 0)
  assert abs(math.sqrt(c.curve.lat_accel * c.with_offset(131.0)) - (math.sqrt(2.2 * 131.0) + g)) < 1e-6


def test_nothing_in_test_mode_ti_only_without_a_ti_or_with_the_setting_off_and_at_most_8_km_h():
  g = 1.0 * MPH
  base = _ctl().update(F(status=1, radius=131.0, distance=0.0), 45 * MPH, 0, v_ego=40 * MPH).curve_speed
  for why in ("test", "ti_only", "no_ti", "off", "status5"):
    c = _ctl(ok=why != "off", ti=why != "no_ti")
    c.ti_only = why == "ti_only"
    f = F(status=5 if why == "status5" else 1, radius=131.0, distance=0.0, speed_offset=g)
    if why == "test":
      f.mode = CTL.TEST
    o = c.update(f, 45 * MPH, 0, v_ego=40 * MPH)
    assert c.offset_now == 0.0, why
    if why in ("off", "no_ti"):
      assert abs(o.curve_speed - base) < 1e-9, why
  c = _ctl()
  c.update(F(status=1, radius=131.0, distance=0.0, speed_offset=50.0), 45 * MPH, 0, v_ego=40 * MPH)
  assert c.offset_now == CTL.OFFSET_MAX
  c.update(F(status=1, radius=131.0, distance=0.0, speed_offset=float("nan")), 45 * MPH, 0, v_ego=40 * MPH)
  assert c.offset_now == 0.0
  # the exit plan's curve is never offset (mapExitCurve* / mapNavCurve*: planned on the map's radius at RAMP_LAT)
  assert CTL.MapFields.from_plan(type("Pl", (), {k: 0 for k in (
    "mapCurveMode", "mapCurveStatus", "mapCurveDistance", "mapCurveRadius", "mapOnRamp", "mapRampType", "mapRampToSpeed",
    "mapRampToHighway", "mapRampEndDistance", "mapNavExit", "mapNavExitDistance", "mapNavCurveRadius",
    "mapNavCurveDistance")})()).speed_offset == 0.0


def _post_apex_cam(gov_mph, offset_mph):
  """A loop's gentler part past the apex and the peak (test_round_r._loop_cam), the governing plan gov_mph with offset_mph in it."""
  cc = CCm.CameraCurve()
  v = 30.5 * MPH
  k0 = 1.2 / (v * v)
  xs = [190.0 * (j / 32) ** 2 for j in range(33)]
  set_v = 30.0 * MPH
  for f in range(1500):                 # (the tight part 2 s, its too-fast latch CAM_OVER_HOLD, then the gentler part)
    tight = f < 200
    o = cc.update(CCm.CamInputs(v_ego=v, set_speed=set_v, lat=2.6 if tight else k0 * v * v, model_x=xs, model_k=[k0] * 33,
                                lane_prob=0.8, torque=1.0 if tight else 0.6, curve_on=True, map_in_curve=True,
                                map_curve_id=(2, 72), on_ramp=True, apex_d=20.0 if tight else -30.0, gov_plan=gov_mph * MPH,
                                cam_target=40 * MPH, hwy_ramp=True, map_offset=offset_mph * MPH))
    if o.tap:
      set_v = o.tap_goal
    v = max(v, set_v + 0.2 * MPH)         # (the car follows the set)
  return set_v / MPH


def test_the_post_apex_taps_and_the_offset_together_never_pass_the_plan_plus_3():
  # (round T, T4: the allowance is now 5 mph) base: the plan 30 (no offset), taps to 35; with +2 in the plan (32): still 35
  # at most (plan + max(+5, offset))
  assert abs(_post_apex_cam(30.0, 0.0) - 35.0) < 1e-6
  assert _post_apex_cam(32.0, 2.0) <= 35.0 + 1e-6
  assert _post_apex_cam(34.0, 4.0) <= 35.0 + 1e-6
  assert abs(_post_apex_cam(32.0, 0.0) - 37.0) < 1e-6     # (what the same plan without the offset in it would allow)


# ---------------- the planner's real publish path ----------------

def _road(tmp, setting, r=142.0):
  root, curves, _ = TMC.write_files(G.curve_road(r=r, turn=40))
  store = MC.MapCurveStore(root, background=False)
  m = MC.MapCurves(store, memory=MC.CurveMemory(os.path.join(tmp, "mem.json")))
  m.offset_set = MC.curve_offset_setting(setting)
  return m, MC.RoadMatcher(store)


def _publish(m, rm, y):
  from types import SimpleNamespace as N
  la, lo = TMC.pos(0, y)
  for _ in range(3):
    w, f = rm.update(la, lo, 0.0, 0.0)
    m.update(la, lo, 0.0, w, f, gps={"dt": 0.05, "v_ego": 20.0})
  mp = MC.MapCurvesPlanner(None, None)
  mp.mode, mp.mc = 2, m
  plan = N()
  mp.publish(plan)
  return plan


def test_the_planner_publishes_the_granted_offset_its_state_and_cap_radius_with_the_radius_unchanged():
  m0, rm0 = _road(tempfile.mkdtemp(), 0)
  m4, rm4 = _road(tempfile.mkdtemp(), 4)
  p0, p4 = _publish(m0, rm0, -150), _publish(m4, rm4, -150)
  assert p0.mapCurveStatus == p4.mapCurveStatus == MC.STATUS_CURVE
  assert p0.mapCurveRadius == p4.mapCurveRadius and p0.mapCurveDistance == p4.mapCurveDistance
  assert p0.mapCurveSpeedOffset == 0.0 and p0.mapCurveOffsetState == MC.OFS_OFF
  r = p4.mapCurveRadius
  assert p4.mapCurveOffsetState == MC.OFS_MAP and abs(p4.mapCurveOffsetCapRadius - r) < 1e-6
  assert abs((math.sqrt(2.2 * r) + p4.mapCurveSpeedOffset) ** 2 / r - MC.CURVE_OFFSET_LAT) < 1e-6
  assert abs(p4.mapCurveTarget - (p0.mapCurveTarget + p4.mapCurveSpeedOffset)) < 1e-6     # (the notice's speed too)
  # the car side reads it
  f = CTL.MapFields.from_plan(type("Pl", (), dict(vars(p4), **{k: 0 for k in (
    "mapRampType", "mapRampToSpeed", "mapRampToHighway", "mapRampEndDistance", "mapNavExit", "mapNavExitDistance",
    "mapNavCurveRadius", "mapNavCurveDistance") if not hasattr(p4, k)}))())
  assert f.speed_offset == p4.mapCurveSpeedOffset
  # a "no offset" mark on it (this direction): nothing, state 3
  c = m4.curve
  m4.memory.add_no_offset(c, m4.direction(c), way=MC.curve_way(c), reason="lateral")
  p = _publish(m4, rm4, -140)
  assert p.mapCurveSpeedOffset == 0.0 and p.mapCurveOffsetState == MC.OFS_MARK
  # no governing curve: nothing
  m4.curve = None
  p = __import__("types").SimpleNamespace()
  mp = MC.MapCurvesPlanner(None, None)
  mp.mode, mp.mc = 2, m4
  mp.publish(p)
  assert p.mapCurveSpeedOffset == 0.0 and p.mapCurveOffsetState == MC.OFS_OFF


def test_gps_trust_not_ok_publishes_no_offset():
  m, rm = _road(tempfile.mkdtemp(), 4)
  from types import SimpleNamespace as N
  la, lo = TMC.pos(0, -150)
  trust = MC.GpsTrust()                       # (a fresh drive: shaky until STEADY_TIME of matched driving)
  for _ in range(3):
    w, f = rm.update(la, lo, 0.0, 0.0)
    m.update(la, lo, 0.0, w, f, trust=trust, gps={"dt": 0.05, "v_ego": 20.0, "acc": 0.0, "rel": 0, "ambiguous": False})
  mp = MC.MapCurvesPlanner(None, None)
  mp.mode, mp.mc = 2, m
  plan = N()
  mp.publish(plan)
  assert plan.mapCurveStatus == MC.STATUS_CURVE and m.trust_status != MC.TRUST_OK
  assert plan.mapCurveSpeedOffset == 0.0 and plan.mapCurveOffsetState == MC.OFS_NOT


# ---------------- closed loop through the real CurveSlowdown.update (wsim's harness) ----------------

def _wsim(tmp_path, monkeypatch, R, offset_mph, setting="4", anchor=45, lkas_block=False, ramp=False):
  import pytest
  pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)
  from types import SimpleNamespace as N
  from opendbc.car.mazda import mazdacan
  from opendbc.car.mazda.values import Buttons
  W = _load("tcpw_round_s_w", "test_car_profile_wrapper.py")
  T = _load("tcs_round_s_w", "test_curve_slowdown.py")
  sent = []
  monkeypatch.setattr(mazdacan, "create_button_cmd", lambda packer, CP, ctr, btn: sent.append(btn) or ("btn", btn))
  c = W.make_cs(tmp_path, W.FakeParams({"IsMetric": False, "MazdaCurveSpeedOffset": setting}), monkeypatch)
  c.opened = []
  if c.recorder is not None:
    open0 = c.recorder.open
    c.recorder.open = lambda kind, t, summary, *a, **kw: (c.opened.append((kind, dict(summary))), open0(kind, t, summary, *a, **kw))[1]
  A, B = 500.0, 500.0 + math.radians(60) * R

  def k(x):
    return 1.0 / R if A <= x < B else 0.0
  acc = lambda g: 0.33 if g < 5 else (0.51 if g < 10 else (0.75 if g < 15 else 0.91))  # noqa: E731
  car = T.Car(anchor, dash_offset_mph=0.0, lag=10, accel_fn=acc, cruise_gain=0.19, step=MPH)
  car.v_ego = anchor * MPH
  plan, m = c.sm.msgs["starpilotPlan"], c.sm.msgs["modelV2"]
  m.laneLineProbs, m.meta = [0.5, 0.9, 0.9, 0.5], N(laneChangeState=0)
  xs = [190.0 * (j / 32) ** 2 for j in range(33)]
  CP = N(wheelbase=2.7, steerRatio=15.5, carFingerprint="MAZDA_CX5")
  cc = N(enabled=True, cruiseControl=N(cancel=False, resume=False), latActive=True, actuators=N(torque=0.0))
  x, trace, presses = 0.0, [], 0
  r_pub = R * 2.6 / 2.2 if ramp else R
  for f in range(20000):
    v = car.v_ego
    plan.mapCurveMode, plan.mapRoadClass, plan.mapGpsTrust, plan.mapOnRamp = 2, (2 if ramp else 5), 0, ramp
    on = x < B and A - x < 400
    plan.mapCurveStatus, plan.mapCurveRadius = ((2 if ramp else 1), r_pub) if on else (0, 0.0)
    plan.mapCurveDistance = max(A - x, 0.0) if on else 0.0
    plan.mapCurveSpeedOffset = offset_mph * MPH if on else 0.0
    plan.mapCurveApexValid, plan.mapCurveApexDistance = on, ((A + B) / 2 - x) if on else 0.0
    upd = f % 5 == 0
    if upd:
      m.position = N(x=xs, y=[0.0] * 33)
      m.velocity = N(x=[max(v, 1.0)] * 33, t=[0.0] * 33)
      m.orientationRate = N(z=[(k(x + xx) if xx <= 70.0 else 0.0) * max(v, 1.0) for xx in xs])
    c.sm.updated["modelV2"] = c.sm.updated["starpilotPlan"] = upd
    lat = k(x) * v * v
    cc.actuators.torque = min(abs(lat) / 2.5, 1.0)
    CS = W.car_state(v / MPH, car.set_speed / MPH)
    CS.out.steeringAngleDeg = math.degrees(k(x) * 18.9 * 2.7 * (1 + 0.001 * v * v))
    CS.out.steeringRateDeg = 0.0
    CS.crz_btns_counter = (f // 10) % 16
    CS.lkas_block = lkas_block
    nb = len(sent)
    c.update(cc, CS, None, CP, False)
    if len(sent) > nb:
      btn = T.cs.SET_MINUS if sent[-1] == Buttons.SET_MINUS else T.cs.SET_PLUS
      presses += 1
      car.hold_frame(btn) if c.logic.last_action == "hold" else car.press(btn)
    trace.append((round(x, 3), round(v / MPH, 4), round(car.set_speed / MPH, 2), abs(lat)))
    car.tick()
    x += car.v_ego * DT
    if x > B + 150:
      break
  entry = next(t[1] for t in trace if t[0] >= A)
  return trace, presses, entry, max(t[3] for t in trace if A <= t[0] < B), c


def test_closed_loop_the_offset_lifts_a_road_curve_by_what_it_granted_and_the_setting_0_or_ti_only_is_the_base(tmp_path, monkeypatch):
  R = 131.0
  g = math.sqrt(2.3 * R) / MPH - math.sqrt(2.2 * R) / MPH          # what the planner grants at setting 4 (0.85 mph)
  tb, pb, eb, lb, _ = _wsim(tmp_path, monkeypatch, R, 0.0)
  tn, pn, en, ln, cn = _wsim(tmp_path, monkeypatch, R, g)
  assert 0.4 < en - eb <= g + 0.3 and ln <= lb + 0.2 and pn <= pb, (eb, en, lb, ln, pb, pn)
  assert cn.curve_offset_set == 4 and cn.map_control.offset_ok
  # the mapCurve event carries the setting, the granted and the applied offset
  ev = [d for k, d in cn.opened if k == "mapCurve"]
  assert cn.recorder is not None
  assert ev and ev[0]["offset_set"] == 4 and abs(ev[0]["offset_granted_mph"] - g) < 0.01, ev[:1]
  assert abs(ev[0]["offset_applied_mph"] - g) < 0.01 and "offset_state" in ev[0] and "offset_cap_r_m" in ev[0]
  # the setting 0 on the car side (whatever the planner sent), TI-only (the LKAS blocked): the base, frame for frame
  t0, p0, *_ = _wsim(tmp_path, monkeypatch, R, g, setting="0")
  assert t0 == tb and p0 == pb
  tbt, pbt, *_ = _wsim(tmp_path, monkeypatch, R, 0.0, lkas_block=True)
  tnt, pnt, *_ = _wsim(tmp_path, monkeypatch, R, g, lkas_block=True)
  assert tnt == tbt and pnt == pbt
