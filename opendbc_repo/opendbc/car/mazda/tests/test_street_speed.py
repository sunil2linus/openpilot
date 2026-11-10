"""The street speed (owner after drives 54 / 55): speed_limit_follow rules 8 (match on engage) and 9 (slow for a lower
limit, MazdaSlowLowerLimit), navigation's limit as a second opinion, carried out by map_curve_control (limit_lower):
the follower, the map control, CurveMix and the controller in the loop on the measured cruise."""
import importlib.util
import os

HERE = os.path.dirname(__file__)
_spec = importlib.util.spec_from_file_location("tmc_street", os.path.join(HERE, "test_map_curves.py"))
TM = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(TM)
_spec = importlib.util.spec_from_file_location("slf_street", os.path.join(HERE, "..", "speed_limit_follow.py"))
slf = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(slf)
T, cs, CTL, F, MPH = TM.T, TM.cs, TM.CTL, TM.F, TM.MPH
KPH = 1 / 3.6
DT = CTL.DT_CTRL


def street_sim(frames, limit, set_mph, v_mph=None, road=lambda f: 7, name=lambda f: "A Road", trust=lambda f: 0,
               nav=lambda f: (0.0, 0.0, False), cruise=lambda f: True, vcap=lambda f: None, gas=lambda f: None,
               driver=None, slow_lower=True, street_mode=2, metric=False, settled=True, max_drop=10, follower=True,
               street_active=True):
  """limit(f) mph (0 none, km/h when metric), road(f) class, name(f) mapd's road name, nav(f) (its limit, m to its next
  maneuver, a turn), cruise(f) on, vcap(f) the car ahead (m/s or None), gas(f) his foot (m/s he takes the car to, or
  None), driver(f) his button (SET_MINUS / SET_PLUS / None). settled: the follower has seen the road (cruise on) 20 s
  before frame 0 (no fresh engage). street_active (bool or f -> bool): Map Curves On with Speed Limit Follow On (False:
  Map Curves Test, the planner's mode not On: the wrapper's _street_active; a paused slowdown holds the restore)."""
  street_on = street_active if callable(street_active) else (lambda f, a=street_active: a)
  step = KPH if metric else MPH
  car = T.Car(set_mph * step / MPH, dash_offset_mph=0.0, lag=10, accel=0.6, step=step, **TM.MEASURED_CRUISE)
  car.set_speed, car.v_ego = set_mph * step, (v_mph if v_mph is not None else set_mph) * step
  logic = cs.CurveSlowdownLogic(metric, max_drop)
  ctl = CTL.MapCurveControl(3.0, CTL.MAP_DECEL, metric=metric, street_mode=street_mode)
  mix = CTL.CurveMix(cs.CURVE_MARGIN)
  f_ = slf.SpeedLimitFollower(metric, street_mode=street_mode, slow_lower=slow_lower)
  md = logic.max_drop_steps * logic.step
  r = dict(sets=[], vs=[], presses=[], holds=[], logs=[], adopted=[], f=f_, ctl=ctl, logic=logic, accel=[])

  def li(fr, on, drv, nl):
    lim = limit(fr)
    return slf.LimitInputs(lim * step if lim else 0.0, "Map Data" if lim else "None", trust(fr), drv, on,
                           road_class=road(fr), road_name=name(fr), v_ego=car.v_ego, nav_limit=nl[0] * step,
                           nav_dist=nl[1], nav_turn=nl[2], driver_set=car.driver_button_now() is not None,
                           pending=bool(on and logic.anchor is not None and car.set_speed < logic.anchor - logic.half_step))
  if settled:
    for _ in range(2000):
      f_.update(li(0, True, car.set_speed, nav(0)))
    f_.engage_frame, f_.events = None, []
  for fr in range(frames):
    on = cruise(fr)
    b = driver(fr) if driver else None
    if b is not None:
      car.driver_button, car.driver_button_until = b, car.frame + 10
      car.set_speed += step if b == cs.SET_PLUS else -step
    drv = (logic.anchor if logic.anchor is not None else car.set_speed) if on else 0.0
    f_.street_active = street_on(fr)
    want = f_.update(li(fr, on, drv, nav(fr))) if follower else 0.0
    lim = limit(fr)
    fld = F(road_class=road(fr), limit=lim * step if lim else 0.0, gps_trust=trust(fr))
    driver_speed = car.set_speed + logic.offset * logic.step if on else 0.0
    foot = gas(fr)
    mo = ctl.update(fld, driver_speed, 0, v_ego=car.v_ego, max_drop=md, gas=foot is not None,
                    set_speed=car.set_speed if on else None, driver_set=car.driver_button_now() is not None,
                    limit_lower=f_.lower_speed if follower and street_on(fr) else 0.0,
                    limit_lower_gas=bool(follower and f_.lower is not None and f_.lower.kind == "lower"))
    if ctl.lower_event is not None:
      f_.lower_finished(ctl.lower_event, car.set_speed / step if on else None)
      ctl.lower_event = None
    active, speed, floor, budget, track, raise_ok = mix.update(False, CTL.NO_CURVE, mo,
                                                               drop_floor=max(driver_speed - md, cs.MIN_SET_SPEED))
    if mo.adopt and cs.adopt_forgets(logic, mo, car.set_speed):
      logic._forget()
      r["adopted"].append(fr)
    inp = cs.SlowdownInputs(enabled=True, cruise_enabled=on, set_speed=car.set_speed, v_ego=car.v_ego,
                            gas_pressed=foot is not None, brake_pressed=False, standstill=car.v_ego < 0.3, plan_alive=True,
                            csc_active=active, csc_speed=min(speed, 100.0), other_button_active=False,
                            driver_button=car.driver_button_now(), curve_floor=floor, curve_budget=budget,
                            curve_track=track, curve_raise=raise_ok, curve_no_accel=mix.no_accel, curve_hold=mix.firm,
                            fast_restore=mo.fast_restore, btn_counter=(fr // 10) % 16, limit_raise=want,
                            hold_restore=bool(follower and f_.paused),
                            limit_cap=f_.cap_speed if follower else 0.0, limit_undo=f_.undo_speed if follower else 0.0,
                            limit_ceiling=f_.ceiling_speed if follower else 0.0,
                            street_lower=bool(mo.street and track and abs(speed - mo.speed) < 1e-3))
    btn, _ = logic.update(inp)
    if logic.limit_adopted is not None:
      f_.mark_adopted()
    if f_.cap is not None and f_.cap.driver is not None and logic.limit_cap is None:
      f_.mark_capped()
    if f_.undo is not None:
      f_.mark_undone()
    if btn is not None and on:
      r["presses"].append((fr, btn, logic.last_action))
      if logic.last_action == "hold":
        car.hold_frame(btn)
        r["holds"].append(fr)
      else:
        car.press(btn)
    if foot is not None:
      car.gas_frame = car.frame
    v0 = car.v_ego
    car.tick()
    if foot is not None:
      car.v_ego = max(car.v_ego, foot)
    cap = vcap(fr)
    if cap is not None:
      car.v_ego = min(car.v_ego, cap)
    r["accel"].append((car.v_ego - v0) / DT)
    r["logs"] += [(fr, k, d["reason"]) for k, d in f_.events if k in ("speedLimitMatch", "speedLimitSlow")]
    f_.events = []
    r["sets"].append(car.set_speed / step)
    r["vs"].append(car.v_ego / step)
  return r


def _minus(r, lo=0, hi=10 ** 9):
  return [p for p in r["presses"] if lo <= p[0] < hi and p[1] == cs.SET_MINUS]


def _plus(r, lo=0, hi=10 ** 9):
  return [p for p in r["presses"] if lo <= p[0] < hi and p[1] == cs.SET_PLUS]


def _reasons(r):
  return [x[2] for x in r["logs"]]


ENGAGE = 300       # frame his SET / RES turns cruise on (off before)


def _engage(**kw):
  return dict(cruise=lambda f: f >= ENGAGE, settled=False, **kw)


# ---------------- rule 8: match on engage ----------------

def test_engage_on_a_known_street_matches_its_limit_up_and_down():
  # RES after a stop at a red light on a 45 street (drive 55, t 581: his highway 75 came back): down to 50 (+5) once
  # moving, from the controller's KEEP_SPEED (12 mph; was: 25): a SET- hold while the set speed is 10+ mph above
  # max(50, his speed + 5), never under that, then taps; then his speed
  r = street_sim(4000, lambda f: 45, 75, v_mph=0, vcap=lambda f: 0.0 if f < 700 else None, **_engage())
  assert abs(r["sets"][-1] - 50) < 1e-6 and r["logic"].anchor is None, (r["sets"][-1], _reasons(r))
  assert _minus(r, ENGAGE) and not _minus(r, 0, 700), r["presses"][:5]                      # (none at a standstill)
  first = _minus(r)[0][0]
  assert r["vs"][first] >= cs.KEEP_SPEED / MPH - 0.5 and r["vs"][first] < 20, r["vs"][first]   # (was: 25 mph)
  assert r["holds"] and all(r["sets"][k] >= max(50, r["vs"][k] + 5) - 1 for k in r["holds"]), r["holds"]
  assert min(r["sets"][first:]) >= 50 - 1e-6 and r["adopted"] and len(r["adopted"]) <= 2, r["adopted"]
  assert any("start: cruise on, down" in x for x in _reasons(r)) and "done" in _reasons(r)
  # SET at 30 on the 45 street: up to 50 (the controller's raise: taps / a hold)
  r = street_sim(4000, lambda f: 45, 30, **_engage())
  assert abs(r["sets"][-1] - 50) < 1e-6 and _plus(r, ENGAGE) and not _minus(r, ENGAGE)
  # the legal limit setting: 45
  r = street_sim(4000, lambda f: 45, 30, street_mode=0, **_engage())
  assert abs(r["sets"][-1] - 45) < 1e-6


def test_engage_with_no_limit_known_only_down_to_43():
  r = street_sim(4000, lambda f: 0, 50, **_engage())
  assert abs(r["sets"][-1] - 43) < 1e-6 and r["logic"].anchor is None, r["sets"][-1]
  r = street_sim(3000, lambda f: 0, 38, **_engage())
  assert r["presses"] == [] and abs(r["sets"][-1] - 38) < 1e-6                       # never raised to 43
  # (second review) above 50 mph: not a street's speed, no fallback (a 65 road with no limit in the map)
  r = street_sim(3000, lambda f: 0, 70, v_mph=70, **_engage())
  assert r["presses"] == [] and abs(r["sets"][-1] - 70) < 1e-6
  # ... navigation's 65 alone there: its 65 + 5, never under it, never up on it alone
  r = street_sim(3000, lambda f: 0, 75, v_mph=70, nav=lambda f: (65.0, 2000.0, False), **_engage())
  assert abs(r["sets"][-1] - 70) < 1e-6 and _plus(r) == [], (r["sets"][-1], _reasons(r))
  r = street_sim(3000, lambda f: 0, 60, nav=lambda f: (65.0, 2000.0, False), **_engage())
  assert r["presses"] == [] and abs(r["sets"][-1] - 60) < 1e-6
  # metric: 70 km/h (from 75; above 80 km/h nothing)
  r = street_sim(4000, lambda f: 0, 78, metric=True, **_engage())
  assert abs(r["sets"][-1] - 70) < 1e-6, r["sets"][-1]
  r = street_sim(3000, lambda f: 0, 100, metric=True, **_engage())
  assert r["presses"] == [], r["presses"][:3]


def test_engage_his_set_after_it_wins_and_a_highway_or_unknown_road_is_unchanged():
  # his SET- 3 s after the engage (before the road was steady 2 s ... the follower waits for the road): his
  r = street_sim(3000, lambda f: 45, 30, road=lambda f: 0 if f < ENGAGE + 400 else 7,
                 driver=lambda f: cs.SET_MINUS if f == ENGAGE + 300 else None, **_engage())
  assert _plus(r) == [] and abs(r["sets"][-1] - 29) < 1e-6, r["sets"][-1]
  # after the match, his SET+: his
  r = street_sim(5000, lambda f: 45, 30, driver=lambda f: cs.SET_PLUS if f == 3500 else None, **_engage())
  assert abs(r["sets"][-1] - 51) < 1e-6 and _minus(r, 3500) == [], r["sets"][-1]
  # a highway (class 1) or the road class unknown (Map Curves off): nothing
  for cls in (1, 2, 0):
    r = street_sim(3000, lambda f, c=cls: 70 if c else 45, 80, road=lambda f, c=cls: c, **_engage())
    assert r["presses"] == [] and abs(r["sets"][-1] - 80) < 1e-6, cls
  # GPS trust 1: waits (nothing while it stays)
  r = street_sim(3000, lambda f: 45, 75, trust=lambda f: 1, **_engage())
  assert r["presses"] == []


# ---------------- rule 9: slow for a lower limit ----------------

def test_a_lower_limit_steady_5_s_slows_gently_to_it():
  # 45 -> 25 (a school zone from a 45 road), his 50 (45 + 5): to 30 (25 + 5), paced: the set never more than ~6 mph
  # under the speed, no hold, the cruise's slowdown ~0.5 m/s^2 at most
  at = 500
  r = street_sim(9000, lambda f: 45 if f < at else 25, 50)
  first = _minus(r)[0][0]
  assert at + 500 <= first <= at + 600, first                                        # 5 s steady first
  assert r["holds"] == [] and abs(r["sets"][-1] - 30) < 1e-6 and r["logic"].anchor is None, r["sets"][-1]
  assert max(v - s for v, s in zip(r["vs"], r["sets"], strict=True)) <= 7.0
  assert min(r["accel"][first:]) >= -0.6, min(r["accel"][first:])
  t_30 = next(k for k, v in enumerate(r["vs"]) if v <= 31.0)
  assert 15 <= (t_30 - first) * DT <= 30, (t_30 - first) * DT                         # 50 -> 31 mph: ~20 s
  assert any(x.startswith("start: 45 -> 25") for x in _reasons(r)) and "done" in _reasons(r)
  # the setting Off: never a SET- for a lower limit (the old behaviour)
  r = street_sim(9000, lambda f: 45 if f < at else 25, 50, slow_lower=False)
  assert _minus(r) == [] and abs(r["sets"][-1] - 50) < 1e-6
  # metric: 70 -> 40 km/h, his 78 (70 + 8): to 48
  r = street_sim(9000, lambda f: 70 if f < at else 40, 78, metric=True)
  assert abs(r["sets"][-1] - 48) < 1e-6, r["sets"][-1]


def test_a_junction_flap_is_ignored_and_a_big_drop_needs_8_s():
  # drive 55: the cross road's 35 for 4 s at a junction (also as its road name): nothing
  r = street_sim(5000, lambda f: 35 if 500 <= f < 900 else 45, 50,
                 name=lambda f: "Cross Road" if 500 <= f < 900 else "A Road")
  assert r["presses"] == [] and r["logs"] == []
  # a drop of more than 20 mph (50 -> 25): 8 s
  r = street_sim(9000, lambda f: 50 if f < 500 else 25, 55)
  assert 500 + 800 <= _minus(r)[0][0] <= 500 + 900, _minus(r)[0][0]
  # GPS trust 1 for a moment in it: not
  r = street_sim(5000, lambda f: 45 if f < 500 else 25, 50, trust=lambda f: 1 if f == 600 else 0)
  assert _minus(r) == []
  # the name changed since it appeared, not by a turn (a flap still going): not
  r = street_sim(5000, lambda f: 45 if f < 500 else 25, 50, name=lambda f: "B Road" if f >= 550 else "A Road")
  assert _minus(r, 0, 1100) == []


def test_his_gas_or_set_ends_it_and_the_limit_back_up_undoes_it():
  at = 500
  # his gas held 2 s while it slows: ends, what it reached is his (no restore to 50, no more SET-)
  r = street_sim(9000, lambda f: 45 if f < at else 25, 50, gas=lambda f: (40 * MPH) if 1500 <= f < 1700 else None)
  assert "cancel: his gas" in _reasons(r) and _minus(r, 1700) == [] and _plus(r, 1700) == [], r["presses"][-3:]
  assert r["logic"].anchor is None and 30 < r["sets"][-1] < 50
  # his SET+ while it slows: his
  r = street_sim(9000, lambda f: 45 if f < at else 25, 50, driver=lambda f: cs.SET_PLUS if f == 1500 else None)
  assert "cancel: his SET" in _reasons(r) and _minus(r, 1600) == []
  # the limit back up 20 s after it got there: his 50 back (rule 4's way)
  r = street_sim(12000, lambda f: 25 if at <= f < 4500 else 45, 50)
  done = next(fr for fr, k, x in r["logs"] if x == "done")
  assert done < 4500 - 100 and any("undo" in x for x in _reasons(r)) and abs(r["sets"][-1] - 50) < 1e-6, (done, _reasons(r))
  # ... before it got there: the slowing just ends, his 50 comes back
  r = street_sim(9000, lambda f: 25 if at <= f < 1600 else 45, 50)
  assert any("before it got there" in x for x in _reasons(r)) and abs(r["sets"][-1] - 50) < 1e-6


def test_never_on_a_highway_or_a_link_and_a_curve_still_wins():
  for cls in (1, 3, 2, 0):
    r = street_sim(5000, lambda f: 70 if f < 500 else 55, 75, road=lambda f, c=cls: c)
    assert _minus(r) == [], cls


# ---------------- navigation's limit (a second opinion) ----------------

def test_navigation_agreeing_slows_sooner_disagreeing_never_past_either():
  at = 500
  # agreeing (25 both): 2.5 s
  r = street_sim(9000, lambda f: 45 if f < at else 25, 50, nav=lambda f: (45 if f < at else 25, 900.0, False))
  assert at + 250 <= _minus(r)[0][0] <= at + 350 and abs(r["sets"][-1] - 30) < 1e-6, _minus(r)[:1]
  # navigation says 35 (the map 25): only down to 40 (the higher one + 5)
  r = street_sim(9000, lambda f: 45 if f < at else 25, 50, nav=lambda f: (35, 900.0, False))
  assert abs(r["sets"][-1] - 40) < 1e-6, r["sets"][-1]
  # navigation's junk 5 mph (drive 55: 9 messages) is no limit: the map alone (5 s, to 30)
  r = street_sim(9000, lambda f: 45 if f < at else 25, 50, nav=lambda f: (5, 900.0, False))
  assert at + 500 <= _minus(r)[0][0] <= at + 600 and abs(r["sets"][-1] - 30) < 1e-6
  # stale / no navigation (0): the map alone
  r = street_sim(9000, lambda f: 45 if f < at else 25, 50, nav=lambda f: (0, 0.0, False))
  assert at + 500 <= _minus(r)[0][0] <= at + 600


def test_navigation_a_lower_limit_at_its_next_turn_waits_for_the_turn():
  # the cross road's 25 appears with navigation's right turn 60 m ahead (still in the through lane): not slowed for
  nav = lambda d: (lambda f: (45 if f < 500 else 25, d, True))  # noqa: E731
  r = street_sim(4000, lambda f: 45 if f < 500 else 25, 50, nav=nav(60.0))
  assert _minus(r) == []
  # (the turn more than NAV_TURN_NEAR away: slowed for)
  r = street_sim(4000, lambda f: 45 if f < 500 else 25, 50, nav=nav(400.0))
  assert _minus(r)


def test_navigation_engage_between_the_two():
  # the map 45, navigation 35: his 75 at RES -> 50 (the higher + 5); his 42 -> left (inside 40 .. 50)
  r = street_sim(4000, lambda f: 45, 75, nav=lambda f: (35, 900.0, False), **_engage())
  assert abs(r["sets"][-1] - 50) < 1e-6
  r = street_sim(4000, lambda f: 45, 42, nav=lambda f: (35, 900.0, False), **_engage())
  assert r["presses"] == []
  # his 30: up to 40 (the lower + 5)
  r = street_sim(4000, lambda f: 45, 30, nav=lambda f: (35, 900.0, False), **_engage())
  assert abs(r["sets"][-1] - 40) < 1e-6, r["sets"][-1]


def test_navigation_straight_through_a_short_name_change_is_no_turn():
  f = slf.SpeedLimitFollower(False)
  li = lambda name, nd: slf.LimitInputs(45 * MPH, "Map Data", 0, 50 * MPH, True, road_class=7, road_name=name,  # noqa: E731
                                        nav_dist=nd)
  for _ in range(600):
    f.update(li("A Road", 900.0))
  for _ in range(300):                                          # 3 s of the cross road's name, straight on: a flap
    f.update(li("Cross Road", 900.0))
  assert f.name == "A Road"
  for _ in range(300):                                          # the same with no navigation: a turn (2 s)
    f.update(li("Cross Road", 0.0))
  assert f.name == "Cross Road"


# ---------------- review of merge-f: the exit's street speed never left half way ----------------

def _exit_road(f):
  return 1 if f < 300 else (2 if f < 1300 else 7)


def _exit_limit(f):
  return 70 if f < 300 else (0 if f < 1300 else 45)


def test_review_a_queue_on_the_street_never_ends_the_exits_lowering_half_way():
  # off the exit onto the 45 street into a queue (stopped 70 s; the taps wait for 25 mph): the window used to expire
  # while lowering and the restore took the set back to his highway 75 at 27 mph (Speed Limit Follow off)
  vcap = lambda f: 40 * MPH if f < 1300 else (0.0 if f < 1300 + 7000 else 30 * MPH)  # noqa: E731
  for follower in (False, True):
    r = street_sim(1300 + 7000 + 4000, _exit_limit, 75, road=_exit_road, vcap=vcap, follower=follower)
    assert abs(r["sets"][-1] - 50) < 1e-6 and r["logic"].anchor is None, (follower, r["sets"][-1])
    assert max(r["sets"][1300:]) <= 75 + 1e-6 and _plus(r, 1300 + 7000) == [], follower


def test_review_his_brake_mid_way_then_res_goes_on_to_the_streets_speed():
  # lowering 75 -> 50 on the street, his brake (cruise off) at ~60, RES: on to 50 (was: left at 60, his 75 the anchor)
  vcap = lambda f: 40 * MPH if f < 1300 else 30 * MPH  # noqa: E731
  for follower in (False, True):
    base = street_sim(1300 + 3000, _exit_limit, 75, road=_exit_road, vcap=vcap, follower=follower)
    half = next(k for k, v in enumerate(base["sets"]) if k > 1300 and v <= 62)
    r = street_sim(half + 4000, _exit_limit, 75, road=_exit_road, vcap=vcap, follower=follower,
                   cruise=lambda f, h=half: not (h <= f < h + 500))
    assert abs(r["sets"][-1] - 50) < 1e-6 and r["logic"].anchor is None, (follower, r["sets"][half - 1], r["sets"][-1])
    assert _plus(r, half) == [], follower


def test_review_a_one_frame_limit_dropout_is_not_the_fallback_and_trust_1_is_no_street():
  c = CTL.MapCurveControl(3.0)
  for _ in range(100):
    c.update(F(road_class=1, limit=70 * MPH), 75 * MPH, 0, v_ego=30.0)
  for _ in range(300):
    c.update(F(road_class=2), 75 * MPH, 0, v_ego=25.0)
  for _ in range(300):
    c.update(F(road_class=7, limit=45 * MPH), 75 * MPH, 0, v_ego=12.0, set_speed=75 * MPH)
  seen = set()
  for k in range(400):
    o = c.update(F(road_class=7, limit=0.0 if k == 10 else 45 * MPH), 75 * MPH, 0, v_ego=12.0, set_speed=60 * MPH)
    if o.active:
      seen.add(round(o.speed / MPH, 1))
  assert seen == {50.0}, seen
  # "no limit" steady 2 s: the fallback
  c = CTL.MapCurveControl(3.0)
  for _ in range(100):
    c.update(F(road_class=1, limit=70 * MPH), 75 * MPH, 0, v_ego=30.0)
  for _ in range(300):
    c.update(F(road_class=2), 75 * MPH, 0, v_ego=25.0)
  outs = [c.update(F(road_class=7), 75 * MPH, 0, v_ego=12.0, set_speed=75 * MPH) for _ in range(500)]
  assert any(o.active and abs(o.speed - 43 * MPH) < 1e-6 for o in outs)
  # the street matched with GPS trust 1 only: no exit street (a wrong match beside a congested highway)
  c = CTL.MapCurveControl(3.0)
  for _ in range(100):
    c.update(F(road_class=1, limit=70 * MPH), 75 * MPH, 0, v_ego=30.0)
  for _ in range(300):
    c.update(F(road_class=2), 75 * MPH, 0, v_ego=25.0)
  outs = [c.update(F(road_class=7, limit=45 * MPH, gps_trust=1), 75 * MPH, 0, v_ego=12.0, set_speed=75 * MPH)
          for _ in range(800)]
  assert not any(o.active for o in outs) and c.xs_since is None


# ---------------- second review ----------------

def test_review2_a_lower_limit_twice_slows_twice():
  # 45 -> 35 (slowed to 40), 35 -> 45 (rule 9's undo: his 50 back; his SET+ to 60 after), later 45 -> 35 again:
  # slowed again (was: the second 35 skipped for the rest of the drive)
  lim = lambda f: 35 if (1000 <= f < 6000 or f >= 12000) else 45  # noqa: E731
  drv = lambda f: cs.SET_PLUS if f in range(8000, 11000, 300) else None  # noqa: E731
  r = street_sim(20000, lim, 50, driver=drv)
  starts = [x for x in r["logs"] if x[2].startswith("start")]
  assert len(starts) == 2 and abs(r["sets"][7999] - 50) < 1e-6 and abs(r["sets"][11999] - 60) < 1e-6, starts
  assert abs(r["sets"][-1] - 40) < 1e-6, r["sets"][-1]


def test_review2_a_zone_under_25_mph_ends_at_25():
  # 45 -> 15 (+5: 20, under the controller's 25 mph MIN_SET_SPEED): to 25, then done, his (was: never finished)
  r = street_sim(9000, lambda f: 45 if f < 500 else 15, 50)
  assert abs(r["sets"][-1] - 25) < 1e-6 and "done" in _reasons(r) and r["f"].lower is None and not r["ctl"].ll_on
  assert r["logic"].anchor is None
  # a 20 mph zone the same; metric: a 30 km/h zone (+8: 38) ends at 41 (25 mph rounded up)
  r = street_sim(9000, lambda f: 45 if f < 500 else 20, 50)
  assert abs(r["sets"][-1] - 25) < 1e-6 and "done" in _reasons(r)
  r = street_sim(9000, lambda f: 50 if f < 500 else 30, 58, metric=True)
  assert abs(r["sets"][-1] - 41) < 1e-6 and "done" in _reasons(r), r["sets"][-1]
  # rule 8 too: RES on a 15 street at 40: down to 25
  r = street_sim(5000, lambda f: 15, 40, **_engage())
  assert abs(r["sets"][-1] - 25) < 1e-6 and "done" in _reasons(r)


def test_review2_his_gas_with_the_set_lowered_by_something_else_is_not_adopted():
  # the paced lower is on (target 40, his 50); something else (a stop) has the set at 19; his gas 1.5 s: it ends, the
  # 19 is not made his (only a set speed at the target is)
  c = CTL.MapCurveControl(3.0)
  adopt, ev = [], None
  for k in range(300):
    o = c.update(F(road_class=7, limit=35 * MPH), 50 * MPH, 0, v_ego=8 * MPH, gas=k >= 50, set_speed=19 * MPH,
                 limit_lower=40 * MPH)
    adopt.append(o.adopt)
    ev = ev or c.lower_event
  assert ev == "cancel: his gas" and not any(adopt)
  # rule 8's match: his gas does not end it
  c = CTL.MapCurveControl(3.0)
  for k in range(300):
    c.update(F(road_class=7, limit=35 * MPH), 50 * MPH, 0, v_ego=8 * MPH, gas=k >= 50, set_speed=50 * MPH,
             limit_lower=40 * MPH, limit_lower_gas=False)
  assert c.lower_event is None and c.ll_on


def test_review2_rule_8_gas_at_the_launch_does_not_end_the_match():
  # drive 55 (608-611): RES at a red light on the 45 street (his highway 75), then his gas pulling away 3 s: still
  # down to 50, adopted there (was: his gas ended it at the launch, the 75 left his)
  r = street_sim(5000, lambda f: 45, 75, v_mph=0, vcap=lambda f: 0.0 if f < 700 else None,
                 gas=lambda f: (12 + (f - 700) * 0.04) * MPH if 700 <= f < 1000 else None, **_engage())
  assert "cancel: his gas" not in _reasons(r) and "done" in _reasons(r), _reasons(r)
  assert abs(r["sets"][-1] - 50) < 1e-6 and r["logic"].anchor is None, r["sets"][-1]
  assert not any(p[1] == cs.SET_MINUS for p in r["presses"] if p[0] < 1000)     # (nothing pressed on his gas)
  # rule 9's lower limit: his gas still ends it (the existing case: test_his_gas_or_set_ends_it_...)


def test_review2_rule_8_at_highway_speed_on_a_street_match_is_no_fallback():
  # a street match (class 5, trust 0) at 70 mph with no limit known (a 65 road missing from the map): nothing (was:
  # lowered to 43)
  r = street_sim(6000, lambda f: 0, 70, road=lambda f: 5, **_engage())
  assert r["presses"] == [] and abs(r["sets"][-1] - 70) < 1e-6
  # navigation's 65 (up to 70 with no map limit counts): 75 -> 70, never up
  r = street_sim(6000, lambda f: 0, 80, v_mph=70, road=lambda f: 5, nav=lambda f: (65.0, 3000.0, False), **_engage())
  assert abs(r["sets"][-1] - 70) < 1e-6 and _plus(r) == [], r["sets"][-1]
  # with a map limit, navigation's 65 on a street is a route step's highway limit: not taken (NAV_STREET_MAX)
  f_ = slf.SpeedLimitFollower(False)
  assert f_.nav_street_max == 55 and f_.nav_alone_max == 70


def test_review2_map_curves_test_never_slows():
  # Map Curves Test (Speed Limit Follow On): no lower, no alert, no rule 8 press
  r = street_sim(9000, lambda f: 45 if f < 500 else 25, 50, street_active=False)
  assert _minus(r) == [] and r["f"].lower is None and abs(r["sets"][-1] - 50) < 1e-6
  r = street_sim(4000, lambda f: 45, 75, street_active=False, **_engage())
  assert r["presses"] == [] and abs(r["sets"][-1] - 75) < 1e-6


# ---------------- third review ----------------

def test_review3_the_fallback_guard_is_the_cars_speed_not_his_set_speed():
  # RES at 75 with the car at 20 mph on a street with no limit (was: nothing, his 75 stayed): down to 43
  r = street_sim(6000, lambda f: 0, 75, v_mph=20, vcap=lambda f: 20 * MPH if f < 1500 else None, **_engage())
  assert abs(r["sets"][-1] - 43) < 1e-6 and r["logic"].anchor is None, (r["sets"][-1], _reasons(r))
  # the car at 60 (a street match beside a highway): no fallback
  r = street_sim(4000, lambda f: 0, 60, v_mph=60, vcap=lambda f: 60 * MPH, **_engage())
  assert r["presses"] == [] and abs(r["sets"][-1] - 60) < 1e-6


def test_review3_navigation_alone_never_takes_him_under_the_fallback():
  # drive 54 (t 1155-1225): navigation's 25 lingering with no map limit; SET at 45: 43 (was: 30)
  r = street_sim(6000, lambda f: 0, 45, v_mph=40, nav=lambda f: (25.0, 800.0, False), **_engage())
  assert abs(r["sets"][-1] - 43) < 1e-6, (r["sets"][-1], _reasons(r))
  # ... at 70 mph: not a street's speed, nothing
  r = street_sim(4000, lambda f: 0, 75, v_mph=70, vcap=lambda f: 70 * MPH, nav=lambda f: (25.0, 800.0, False),
                 **_engage())
  assert r["presses"] == [] and abs(r["sets"][-1] - 75) < 1e-6
  # navigation's 70 lingering, his 75 at 30 mph: its 75 is not under his speed: nothing (never a raise either)
  r = street_sim(6000, lambda f: 0, 75, v_mph=30, nav=lambda f: (70.0, 800.0, False), **_engage())
  assert r["presses"] == [] and abs(r["sets"][-1] - 75) < 1e-6
  r = street_sim(6000, lambda f: 0, 40, v_mph=30, nav=lambda f: (45.0, 800.0, False), **_engage())
  assert _plus(r) == [] and abs(r["sets"][-1] - 40) < 1e-6


def test_review3_adopt_forgets_an_anchor_just_under_the_reached_street_speed_not_rule_4s_undo():
  lg = cs.CurveSlowdownLogic(False, 10)
  mo = CTL.MapOutput(adopt=True, adopt_target=43 * MPH)
  lg.anchor, lg.last_set_speed = 42 * MPH, 43 * MPH          # (our landing re-anchored one step under: drive 54, 745.2)
  assert cs.adopt_forgets(lg, mo, 43 * MPH)
  # rule 4's undo (the anchor his speed before our raise, under the set speed): kept with no street speed just reached
  assert not cs.adopt_forgets(lg, CTL.MapOutput(adopt=True), 43 * MPH)
  assert not cs.adopt_forgets(lg, mo, 47 * MPH)
  lg.anchor, lg.last_set_speed = 50 * MPH, 43 * MPH          # a restore pending above it: forgotten (as before)
  assert cs.adopt_forgets(lg, CTL.MapOutput(adopt=True), 43 * MPH)
  # the map control: the target while its adopt lasts (2 frames), then 0
  c = CTL.MapCurveControl(3.0)
  outs = [c.update(F(road_class=7, limit=45 * MPH), 50 * MPH, 0, v_ego=20.0, set_speed=40 * MPH,
                   limit_lower=40 * MPH if k == 0 else 0.0) for k in range(4)]
  assert [round(o.adopt_target / MPH) for o in outs] == [40, 40, 0, 0], [o.adopt_target / MPH for o in outs]


def test_review3_one_frame_of_a_link_class_does_not_end_the_match():
  # RES at 75 on the 45 street; one frame of class 2 (a link, near a ramp: drive 53 t 443) mid-way: still to 50
  base = dict(v_mph=30, vcap=lambda f: 30 * MPH)
  r = street_sim(5000, lambda f: 45, 75, road=lambda f: 2 if f == 800 else 7, **base, **_engage())
  assert "cancel: not on a street" not in _reasons(r) and abs(r["sets"][-1] - 50) < 1e-6, _reasons(r)
  # 3 s of it: cancelled (his 75 comes back); his next RES (cruise off / on) re-opens the match: 50
  link = lambda f: 2 if 800 <= f < 1100 else 7  # noqa: E731
  r = street_sim(4000, lambda f: 45, 75, road=link, **base, **_engage())
  assert "cancel: not on a street" in _reasons(r) and r["sets"][-1] > 70, (r["sets"][-1], _reasons(r))
  r = street_sim(8000, lambda f: 45, 75, road=link, cruise=lambda f: f >= ENGAGE and not (4000 <= f < 4100),
                 settled=False, **base)
  assert abs(r["sets"][-1] - 50) < 1e-6 and _reasons(r).count("start: cruise on, down") == 2, _reasons(r)


def test_review3_a_map_curve_mode_flip_mid_way_pauses_the_slowdown():
  # RES at 75 on the 45 street; the planner's map curve mode not On for 5 s mid-way: nothing pressed meanwhile (no
  # restore back up), then on to 50 (was: cancelled, his 75 restored)
  base = dict(v_mph=30, vcap=lambda f: 30 * MPH)
  r = street_sim(800, lambda f: 45, 75, **base, **_engage())
  mid = r["sets"][-1]
  assert 52 <= mid < 75, mid
  pause = lambda f: not (800 <= f < 1300)  # noqa: E731
  r = street_sim(5000, lambda f: 45, 75, street_active=pause, **base, **_engage())
  assert not [p for p in r["presses"] if 815 <= p[0] < 1300], [p for p in r["presses"] if 815 <= p[0] < 1300]
  assert max(r["sets"][800:1300]) <= mid + 1e-6
  assert abs(r["sets"][-1] - 50) < 1e-6 and "done" in _reasons(r), (r["sets"][-1], _reasons(r))
  # his SET while paused: his
  r = street_sim(5000, lambda f: 45, 75, street_active=pause, driver=lambda f: cs.SET_MINUS if f == 1000 else None,
                 **base, **_engage())
  assert "cancel: his SET" in _reasons(r) and _minus(r, 1300) == [] and _plus(r, 1000) == [], _reasons(r)


def test_final_no_set_minus_of_rules_8_9_while_off_a_street_and_a_long_pause_ends():
  # a link class for 3 s mid-way (a real move onto a ramp): rules 8 / 9 press nothing while it says "not a street"
  # (the restore held meanwhile), then end (his speed comes back)
  base = dict(v_mph=30, vcap=lambda f: 30 * MPH)
  link = lambda f: 2 if 800 <= f < 1100 else 7  # noqa: E731
  r = street_sim(4000, lambda f: 45, 75, road=link, **base, **_engage())
  assert _minus(r, 815, 1100) == [], _minus(r, 815, 1100)
  assert "cancel: not on a street" in _reasons(r)
  # Map Curves not On for good mid-way: the paused slowdown ends after LOWER_PAUSE_MAX, the restore is not held for good
  off = lambda f: f < 800  # noqa: E731
  r = street_sim(800 + int((slf.LOWER_PAUSE_MAX + 2) / DT), lambda f: 45, 75, street_active=off, **base, **_engage())
  assert "cancel: paused too long" in _reasons(r), _reasons(r)
  assert not r["f"].paused


def test_drive_56_rule_8_goes_up_when_the_set_speed_is_already_under_the_streets_speed():
  # drive 56 t 729: RES on a 45 street with the stored highway 80 (a restore pending): rule 8 started "down" to 50, the
  # set speed was 43 already ("his speed under it") and stayed 43. Rule 8 is up or down: it now goes up to 50
  f = slf.SpeedLimitFollower(False, street_mode=2)
  f.lower = slf.Offer(None, 45, "Map Data", 0, "match", 80, tgt=50)
  f.lower_finished("his speed under it", 43)
  assert f.lower is None and f.offer is not None and f.offer.kind == "match" and f.offer.target == 50 and f.offer.driver == 43
  # done at it, or his own SET: nothing more
  for why, d in (("done", 50), ("cancel: his SET", 43)):
    f = slf.SpeedLimitFollower(False, street_mode=2)
    f.lower = slf.Offer(None, 45, "Map Data", 0, "match", 80, tgt=50)
    f.lower_finished(why, d)
    assert f.offer is None, why
  # rule 9 (a lower limit) with his speed under it: his
  f = slf.SpeedLimitFollower(False, street_mode=2)
  f.lower = slf.Offer(50, 45, "Map Data", 0, "lower", 55, tgt=50)
  f.lower_finished("his speed under it", 43)
  assert f.offer is None


def test_drive_66_a_lower_limit_is_counted_with_cruise_on_and_longer_after_a_turn():
  # drive 66 t 366-384: cruise turned on just after a turn, the map said 25 for a few seconds on a 35 / 45 road and he
  # was slowed 33 -> 29. A lower limit is counted with cruise on only (5 s), and 10 s after a turn onto the road
  f = slf.SpeedLimitFollower(False, street_mode=0)
  assert f.cruise_frames == 0
  f.update(slf.LimitInputs(25 * MPH, "Map Data", 0, 33 * MPH, True, road_class=7, road_name="A"))
  f.update(slf.LimitInputs(25 * MPH, "Map Data", 0, 0.0, False, road_class=7, road_name="A"))
  assert f.cruise_frames == 0
  assert slf.LOWER_NEW_ROAD_STEADY >= 2 * slf.LOWER_STEADY


def _drive69_t866(**kw):
  """Drive 69, t 860-920 (frame 0 = t 860): a 45 street, a 4 s "35" from mapd's next limit at the junction (t 866-870),
  then the road's 50 (t 871) with GPS trust 2 for 5 s (t 874-879), 0 after; his own speed 41; he braked behind a car the
  radar did not see (cruise off t 883.7, stopped), RES at a standstill at t 890.5, moving again at t 900 (t 903.7 on the
  drive; the simulated cruise pulls away slower than the car did, and the match must be taken within LIMIT_OFFER_TIME)."""
  limit = lambda f: 45 if f < 600 else (35 if f < 1000 else 50)   # noqa: E731
  trust = lambda f: 2 if 1400 <= f < 1900 else 0                                        # noqa: E731
  return street_sim(7000, limit, 41, trust=trust, cruise=lambda f: not (2370 <= f < 3050),
                    vcap=lambda f: 0.0 if 2370 <= f < 4000 else None, **kw)


def test_drive_69_t866_the_match_on_engage_judges_the_recent_gps_trust():
  # the limit's worst trust ever seen (2, eleven seconds before his RES) kept rule 8 from matching: the restore ended at
  # his 41 and he set 50 himself. Trust 0 over the last ENGAGE_LIMIT_STEADY now counts: up to 50 + the street offset
  r = _drive69_t866()
  want = 50 + (r["f"]._add() * r["f"].step / MPH)
  assert any("start: cruise on, up" in x for x in _reasons(r)), _reasons(r)
  assert abs(r["sets"][-1] - want) < 1e-6, (r["sets"][-1], want, _reasons(r))
  assert not _minus(r, 3050), _minus(r, 3050)[:3]
  # trust 2 until after the RES: it waits for ENGAGE_LIMIT_STEADY of trust 0 (within ENGAGE_WINDOW), then matches ...
  r2 = street_sim(5000, lambda f: 50, 41, trust=lambda f: 2 if f < 3000 else 0, cruise=lambda f: f >= 2950,
                  settled=False)
  starts = [fr for fr, k, x in r2["logs"] if "start: cruise on, up" in x]
  assert starts and starts[0] >= 3000 + slf.ENGAGE_LIMIT_STEADY / DT - 2, starts
  # ... and never while the trust stays low through the window
  r3 = street_sim(5000, lambda f: 50, 41, trust=lambda f: 2 if f < 4700 else 0, cruise=lambda f: f >= 2950,
                  settled=False)
  assert not any("start: cruise on, up" in x for x in _reasons(r3)) and abs(r3["sets"][-1] - 41) < 1e-6
