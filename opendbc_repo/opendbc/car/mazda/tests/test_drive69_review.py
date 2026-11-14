"""Review of drive 69's fixes (ramp holds, merge boost, exit street, curve caps): the reviewer's probes as tests."""
import importlib.util
import math
import os

HERE = os.path.dirname(__file__)


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


TMC = _load("tmc_r69", os.path.join(HERE, "test_map_curves.py"))
TSC = _load("tscs_r69", os.path.join(HERE, "test_stopped_car_slowdown.py"))
TSL = _load("tslf_r69", os.path.join(HERE, "test_speed_limit_follow.py"))
T, cs, CTL, MC, F = TMC.T, TMC.cs, TMC.CTL, TMC.MC, TMC.F
slf = TSL.slf
MPH = TMC.MPH
DT = 0.01


def _acc(g):
  return 0.33 if g < 5 else (0.51 if g < 10 else (0.75 if g < 15 else (0.91 if g < 20 else 1.06)))


def _boost_run(frames, ev=None, limit=75, boost=20, raise_frames=(400, 401)):
  """The reviewer's boost_probe: a merge raise at frame 400 on the measured cruise; ev(f, st, car, logic) changes the
  inputs (cruise, enabled, gas, brake, undo, driver button, radar lead, block: SET presses never reach the car)."""
  car = T.Car(35, dash_offset_mph=0.0, lag=10, accel_fn=_acc, cruise_gain=0.19)
  car.v_ego = 35 * MPH
  lg = cs.CurveSlowdownLogic(False, 10)
  st = dict(cruise=True, enabled=True, gas=False, brake=False, undo=0.0, btn=None, radar=False, block=False, road=True)
  log, rises, prev = [], 0, False
  for f in range(frames):
    if ev:
      ev(f, st, car, lg)
    inp = cs.SlowdownInputs(enabled=st["enabled"], cruise_enabled=st["cruise"], set_speed=car.set_speed if st["cruise"] else 0.0,
                            v_ego=car.v_ego, gas_pressed=st["gas"], brake_pressed=st["brake"], standstill=False, plan_alive=True,
                            other_button_active=False, driver_button=st["btn"], csc_active=False, csc_speed=0.0,
                            limit_raise=limit * MPH if raise_frames[0] <= f < raise_frames[1] else 0.0,
                            limit_boost=boost * MPH, limit_undo=st["undo"], radar_lead=st["radar"], boost_road_ok=st["road"])
    btn, _ = lg.update(inp)
    if btn is not None and st["cruise"] and not st["block"]:
      car.hold_frame(btn) if lg.last_action == "hold" else car.press(btn)
    if st["cruise"]:
      car.tick()
    else:
      car.frame += 1
      car.v_ego = max(0.0, car.v_ego - 0.5 * DT)
    rises += lg.boost_stuck and not prev
    prev = lg.boost_stuck
    log.append((f, car.set_speed / MPH, car.v_ego / MPH))
  return car, lg, log, rises


# ---- 2: a gas jump (SET+ to the current speed) is ours, never "more steps than our presses" ----

def test_a_gas_jump_never_switches_the_controller_off():
  # merge damage: the jump's booking (p.jump, its reserve) was nested under the boost's `held` line
  for v, extra in ((35, 0), (35, 1), (35, 2), (40, 2), (30, 3)):
    r, _ = TSC._gas_run(v, top_mph=v, jump_extra=extra)
    assert not r["logic"].disabled, (v, extra)
  for lag in (15, 30, 45, 60):
    orig = TSC._GasCar.__init__

    def init(self, *a, lag=lag, orig=orig, **kw):
      kw["lag"] = lag
      orig(self, *a, **kw)
    TSC._GasCar.__init__ = init
    try:
      r, _ = TSC._gas_run(30, top_mph=60)
    finally:
      TSC._GasCar.__init__ = orig
    assert not r["logic"].disabled, lag


# ---- 1: camera-only stops off only once on an on-ramp's link; a running stop episode keeps them ----

def test_an_on_ramp_ahead_on_the_street_never_turns_the_camera_stops_off():
  c = CTL.MapCurveControl(3.0)
  for _ in range(200):
    c.update(F(status=2, radius=70.0, distance=120.0, road_class=5, onramp_ahead=True), 45 * MPH, 1, v_ego=12.0)
  assert not c.onramp_now                                        # his signal toward an on-ramp, still on the street
  for _ in range(100):
    c.update(F(road_class=2), 45 * MPH, 1, v_ego=12.0)           # on its link
  assert c.onramp_now


# ---- 3: his SET- during the boost ----

def test_his_set_minus_during_the_boost_lowers_it_and_the_restore_takes_it_on_down():
  def ev(f, st, car, lg):
    if f == 1300:
      car.press(cs.SET_MINUS, who="driver")
    st["btn"] = car.driver_button_now()
  car, lg, log, _ = _boost_run(5000, ev)
  assert abs(car.set_speed / MPH - 75) < 1e-6 and max(v for _, _, v in log) <= 79, car.set_speed / MPH
  # his SET+ instead: his speed (the boost ends, nothing taken back)

  def ev2(f, st, car, lg):
    if f == 1300:
      car.press(cs.SET_PLUS, who="driver")
    st["btn"] = car.driver_button_now()
  car, lg, log, _ = _boost_run(5000, ev2)
  assert car.set_speed / MPH > 85 and lg.boost is None


# ---- 4: rule 4's undo while the boost comes down ----

def test_an_undo_while_the_boost_comes_down_ends_it_with_one_bounded_set_minus_hold():
  for uf in (2250, 2600, 3200):
    def ev(f, st, car, lg, uf=uf):
      st["undo"] = 35 * MPH if uf <= f < uf + 5 else 0.0
    car, lg, log, _ = _boost_run(7000, ev)
    assert abs(car.set_speed / MPH - 35) < 1e-6, (uf, car.set_speed / MPH)
    assert all(s >= 35 - 1e-6 for _, s, _ in log), uf

  def ev(f, st, car, lg):
    st["undo"] = 35 * MPH if 2250 <= f < 2255 else 0.0
  car, lg, log, _ = _boost_run(7000, ev)
  downs = [h for h in lg.hold_log if h["direction"] == cs.SET_MINUS]
  assert downs and abs(downs[-1]["bound_mph"] - 35) < 1e-6, lg.hold_log


# ---- 5: a boost left high: re-anchored, not forgotten, the alert repeated ----

def test_a_boost_left_high_is_taken_back_and_the_alert_repeats():
  # openpilot off 38 s with cruise on (PAUSE_ABANDON would forget it): back to the target afterwards
  def ev(f, st, car, lg):
    st["enabled"] = not (1200 <= f < 5000)
  car, lg, log, _ = _boost_run(8000, ev)
  assert abs(car.set_speed / MPH - 75) < 1e-6, car.set_speed / MPH
  # the car answers no SET- after the end: the alert, again every MERGE_HIGH_REPEAT while 10+ above the target

  def ev2(f, st, car, lg):
    st["block"] = lg.boost is not None and ("ended" in lg.boost or "taper" in lg.boost)   # (round P: from the taper)
  car, lg, log, rises = _boost_run(9000, ev2)
  assert car.set_speed / MPH >= 85 and rises >= 3, rises
  # forgotten (the car not answering, his cancel and RES): the boost's ended branch anchors the target again
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.boost = {"target": 75 * MPH, "frame": 0, "ended": 0}
  lg.last_set_speed, lg.anchor = 92 * MPH, None
  lg._boost_update(cs.SlowdownInputs(enabled=True, cruise_enabled=True, set_speed=92 * MPH, v_ego=74 * MPH, gas_pressed=False,
                                     brake_pressed=False, standstill=False, plan_alive=True, other_button_active=False,
                                     driver_button=None, csc_active=False, csc_speed=0.0), True)
  assert abs(lg.anchor / MPH - 75) < 1e-6 and lg.restore_pending


# ---- 7: the boost on the link only, capped ----

def test_the_boost_only_on_the_link_and_never_above_100():
  car, lg, log, _ = _boost_run(3000, lambda f, st, car, lg: st.update(road=False))
  assert max(s for _, s, _ in log) <= 75 + 1e-6                 # a street: no boost

  def ev(f, st, car, lg):
    st["road"] = f < 1000                                         # the road becomes a street mid-boost
  car, lg, log, _ = _boost_run(6000, ev)
  assert abs(car.set_speed / MPH - 75) < 1e-6 and max(v for _, _, v in log) <= 79
  car, lg, log, _ = _boost_run(1500, limit=85)
  assert lg.anchor is None or lg.anchor / MPH <= 100 + 1e-6
  assert max(s for _, s, _ in log) <= 100 + 1e-6


# ---- 11: a raise right after a stop episode's release taps first ----

def test_a_raise_right_after_a_stop_release_taps_first_then_holds():
  # a stop episode lowered the set (45 -> 30); it releases at frame 600 and a posted limit raise (55) comes at 650:
  # taps (the gentle restore) until RAISE_FAST_AFTER_STOP after the release, a hold only from then
  car = T.Car(45, dash_offset_mph=0.0, lag=10, accel_fn=_acc, cruise_gain=0.19)
  car.v_ego = 45 * MPH
  lg = cs.CurveSlowdownLogic(False, 10)
  acts = []
  for f in range(4000):
    stop = 100 <= f < 600
    inp = cs.SlowdownInputs(enabled=True, cruise_enabled=True, set_speed=car.set_speed, v_ego=car.v_ego, gas_pressed=False,
                            brake_pressed=False, standstill=False, plan_alive=True, other_button_active=False,
                            driver_button=None, csc_active=False, csc_speed=0.0, stop_active=stop, stop_target=30 * MPH,
                            limit_raise=55 * MPH if f >= 650 else 0.0)
    btn, _ = lg.update(inp)
    if btn is not None:
      acts.append((f, btn, lg.last_action))
      car.hold_frame(btn) if lg.last_action == "hold" else car.press(btn)
    car.tick()
  ups = [(f, a) for f, b, a in acts if b == cs.SET_PLUS]
  assert ups and ups[0][0] < 600 + cs.RAISE_FAST_AFTER_STOP * 100, ups[:3]
  assert all(a != "hold" for f, a in ups if f < 600 + cs.RAISE_FAST_AFTER_STOP * 100), ups
  assert abs(car.set_speed / MPH - 55) < 1e-6, car.set_speed / MPH


# ---- 6 + owner: the held ramp command, the camera, the distance / time fallback ----

def _held_loop(cam=None, frames=3000, v=13.4, end0=400.0):
  """On a ramp onto a highway: its ramp curve governs (30 mph), then status 5 from frame 300 with the ramp flag off.
  cam(f): the camera curve speed (m/s; None: no camera). Returns the frames the hold lasted (from 300)."""
  c = CTL.MapCurveControl(3.0)
  ramp = dict(on_ramp=True, ramp_type=2, ramp_to_highway=True, road_class=2)
  left = end0
  held = 0
  for f in range(frames):
    left -= v * DT
    if f < 300:
      out = c.update(F(status=2, radius=84.0, distance=0.0, ramp_end_distance=left, **ramp), 50 * MPH, 0, v_ego=v,
                     cam_speed=cam(f) if cam else None)
    else:
      out = c.update(F(status=5, radius=212.0, distance=10.0, road_class=2), 50 * MPH, 1, v_ego=v,
                     cam_speed=cam(f) if cam else None)
      if out.why == "ramp hold":
        held += 1
      elif held:
        break
  return held


def test_the_held_ramp_command_lets_go_by_the_distance_left_or_a_max_time_without_the_camera():
  # the ramp flag is off in status 5: the end distance is counted on (v x dt) and the hold lets go 120 m before it
  held = _held_loop(end0=250.0)
  left_at = 250.0 - 13.4 * DT * (300 + held)
  assert abs(left_at - CTL.RAMP_STATUS_HWY_RELEASE) < 2.0, left_at
  # no end distance known (a long ramp): RAMP_STATUS_HOLD_MAX
  held = _held_loop(end0=5000.0, frames=4000)
  assert abs(held - CTL.RAMP_STATUS_HOLD_MAX / DT) <= 2, held


def test_the_camera_decides_while_the_ramp_command_is_held():
  # drive 69 t 529: the loop seen by the camera (~31 mph) while the GPS is weak: held, past the distance rule too;
  # the camera path straight from frame 1500: let go within ~1 s
  loop = math.sqrt(2.6 * 90.0)
  held = _held_loop(cam=lambda f: loop if f < 1500 else 100.0, end0=300.0)
  assert 1500 - 300 <= held <= 1500 - 300 + CTL.RAMP_CAM_OPEN_TIME / DT + 2, held
  # a camera speed between the loop band and the open band: still held (no release counted)
  held = _held_loop(cam=lambda f: (14.0 + 9 * MPH) if f < 1500 else 100.0, end0=300.0)
  assert held >= 1500 - 300
  # straight at once: let go after ~1 s, earlier than the distance rule
  held = _held_loop(cam=lambda f: 100.0, end0=600.0)
  assert held <= CTL.RAMP_CAM_OPEN_TIME / DT + 2, held


def test_the_loop_with_his_signal_and_the_gps_dropping_never_raises_while_the_camera_sees_it():
  # drive 69: right signal through the loop, status 5 mid-loop. The camera curve stays on on the link (the signal does
  # not turn it off there), the held command stands while it sees the loop, and the restore follows its opening
  loop_v = math.sqrt(2.6 * 90.0)
  fields_at = lambda x: (F(status=2, radius=84.0, distance=0.0, road_class=2, on_ramp=True, ramp_type=2,   # noqa: E731
                           ramp_to_highway=True, ramp_end_distance=max(600 - x, 1.0)) if x < 150 else
                         F(status=5, radius=212.0, distance=10.0, road_class=2))
  car = T.Car(31, dash_offset_mph=0.0, lag=10, accel=0.6, step=MPH, **TMC.MEASURED_CRUISE)
  car.v_ego = 31 * MPH
  logic = cs.CurveSlowdownLogic(False, 10)
  logic.anchor, logic.last_set_speed = 50 * MPH, 31 * MPH
  ctl = CTL.MapCurveControl(3.0, CTL.MAP_DECEL)
  mix = CTL.CurveMix(cs.CURVE_MARGIN)
  md = logic.max_drop_steps * logic.step
  x, plus_in_loop, first_plus, released = 0.0, 0, None, None
  for _ in range(4000):
    cam_v = loop_v if x < 330 else 100.0                         # the loop opens up at 330 m
    driver = car.set_speed + logic.offset * logic.step
    mo = ctl.update(fields_at(x), driver, 1, v_ego=car.v_ego, max_drop=md, set_speed=car.set_speed, cam_speed=cam_v)
    if released is None and x > 150 and mo.why != "ramp hold":
      released = x
    active, speed, floor, budget, track, raise_ok = mix.update(cam_v < 50, cam_v, mo, drop_floor=max(driver - md, 25 * MPH))
    inp = cs.SlowdownInputs(enabled=True, cruise_enabled=True, set_speed=car.set_speed, v_ego=car.v_ego, gas_pressed=False,
                            brake_pressed=False, standstill=False, plan_alive=True, csc_active=active, csc_speed=min(speed, 100.0),
                            other_button_active=False, driver_button=None, curve_floor=floor, curve_budget=budget,
                            curve_track=track, curve_raise=raise_ok, curve_no_accel=mix.no_accel, curve_hold=mix.firm)
    btn, _ = logic.update(inp)
    if btn == cs.SET_PLUS:
      if x < 330:
        plus_in_loop += 1
      elif first_plus is None:
        first_plus = x
    if btn is not None:
      car.hold_frame(btn) if logic.last_action == "hold" else car.press(btn)
    car.tick()
    x += car.v_ego * DT
  assert plus_in_loop == 0, plus_in_loop
  assert released is not None and 330 <= released <= 330 + 31 * MPH * (CTL.RAMP_CAM_OPEN_TIME + 0.2), released
  assert first_plus is not None and first_plus <= 330 + 31 * MPH * 3.5, first_plus     # (the restore's own delays)


# ---- 8: rule 1's retry ----

def test_a_retried_rise_expires_and_ends_with_his_set():
  RAISE = TSL.RAISE
  f = TSL._settled(45)
  TSL._feed(f, RAISE + 10, 60, gps_trust=2)
  assert f.rise_retry is not None
  TSL._feed(f, int(slf.LIMIT_OFFER_TIME / slf.DT_CTRL) + 10, 60, gps_trust=2)   # trust 2 the whole offer time
  assert f.rise_retry is None
  f = TSL._settled(45)
  TSL._feed(f, RAISE + 10, 60, gps_trust=2)
  li = TSL._li(60, gps_trust=2)
  li.driver_set = True
  f.update(li)                                                   # his SET after the refusal
  out = TSL._feed(f, RAISE + 20, 60)
  assert all(x == 0.0 for x in out) and f.rise_retry is None


# ---- 9: the exit's street raises wait for a stop / red light, a queue, a sure street ----

def test_the_exits_street_raises_wait_for_no_stop_no_red_a_moving_car_and_a_sure_street():
  ex = TSL._Exit(link=1000, street=3000, limit=10 ** 9)

  def run(**kw):
    f = slf.SpeedLimitFollower(False, street_mode=1)
    got = []
    for k in range(7000):
      r = ex.road(k, 0.0)
      li = slf.LimitInputs(limit=ex.slc(k) * MPH, source="Map Data", gps_trust=0, driver_speed=27 * MPH, cruise_enabled=True,
                           road_class=r["road_class"], v_ego=kw.get("v", 27) * MPH, road_name=kw.get("name", lambda k: "Dobry")(k),
                           stop_active=kw.get("stop", lambda k: False)(k), red_light=kw.get("red", lambda k: False)(k))
      out = f.update(li)
      if out > 0 and not got:
        got.append(k)
    return got
  base = run()
  assert base and base[0] <= 3100 + slf.EXIT_STREET_MATCH * 100 + 20, base
  assert run(v=12) == []                                          # a queue at the ramp's light
  got = run(stop=lambda k: k < 4500)
  assert got and got[0] >= 4500, got                              # a stop reason on: waits
  got = run(red=lambda k: k < 4500)
  assert got and got[0] >= 4500, got
  got = run(name=lambda k: "Cross Road" if k < 3300 else "Dobry")  # the cross street snapped to first
  assert got and got[0] >= 3300 + slf.EXIT_STREET_MATCH * 100, got


# ---- second review ----

def test_one_straight_camera_frame_never_ends_the_held_ramp_command():
  # review69/cam_glitch.py: CurveTarget.speed is the least demanding of its 1 s window, so one straight frame (glare, a
  # lead in the way) read straight for a whole second and let go of the hold in the loop. "Open" needs every frame's own
  # value straight for RAMP_CAM_OPEN_TIME; "the loop still" counts the window's most demanding value
  CT = _load("curve_target_r69", os.path.join(HERE, "..", "curve_target.py"))
  k_loop = 2.2 / (30 * MPH) ** 2

  def run(straight):
    t = CT.CurveTarget(2.2)
    c = CTL.MapCurveControl(3.0)
    ramp = dict(on_ramp=True, ramp_type=2, ramp_to_highway=True, road_class=2, ramp_end_distance=900.0)
    released = None
    for f in range(1200):
      t.update(0.0 if straight(f) else k_loop, 45 * MPH)
      if f < 200:
        c.update(F(status=2, radius=84.0, distance=0.0, **ramp), 50 * MPH, 0, v_ego=13.4, cam_speed=t.raw, cam_low=t.low)
        continue
      out = c.update(F(status=5, radius=212.0, distance=10.0, road_class=2), 50 * MPH, 1, v_ego=13.4,
                     cam_speed=t.raw, cam_low=t.low)
      if released is None and out.why != "ramp hold":
        released = f
    return released
  assert run(lambda f: f in (350, 351, 600)) is None             # single straight frames: held
  assert run(lambda f: 500 <= f < 600) is None                   # straight for 1 s only: held
  rel = run(lambda f: f >= 500)                                  # the loop opens at 500
  assert rel is not None and abs(rel - 500 - CTL.RAMP_CAM_OPEN_TIME / DT) <= 2, rel


def _boost_engage(ev):
  return _boost_run(6000, ev)


def test_his_set_at_engage_after_a_boost_is_his_but_res_of_our_boosted_set_comes_down():
  # his brake during the boost, then SET at 85: his choice (not pressed down to the target)
  def ev(f, st, car, lg):
    if f == 1300:
      st["cruise"], st["brake"] = False, True
    if f == 1320:
      st["brake"] = False
    if f == 1500:
      car.set_speed = 85 * MPH                                    # SET engages at his speed
      st["btn"] = cs.SET_MINUS
    if f == 1510:
      st["btn"] = None
      st["cruise"] = True
  car, lg, log, _ = _boost_engage(ev)
  assert abs(car.set_speed / MPH - 85) < 1e-6 and lg.boost is None, car.set_speed / MPH
  # RES: the set speed comes back at the boosted value we put there: ours, down to the target

  def ev2(f, st, car, lg):
    if f == 1300:
      st["cruise"], st["brake"] = False, True
    if f == 1320:
      st["brake"] = False
    if f == 1600:
      st["cruise"] = True
  car, lg, log, _ = _boost_engage(ev2)
  assert abs(car.set_speed / MPH - 75) < 1e-6, car.set_speed / MPH
  # RES at another set speed than we left (his own stored choice): his

  def ev3(f, st, car, lg):
    if f == 1300:
      st["cruise"], st["brake"] = False, True
    if f == 1320:
      st["brake"] = False
    if f == 1590:
      car.set_speed = 82 * MPH
    if f == 1600:
      st["cruise"] = True
  car, lg, log, _ = _boost_engage(ev3)
  assert abs(car.set_speed / MPH - 82) < 1e-6 and lg.boost is None, car.set_speed / MPH
