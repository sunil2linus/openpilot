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
               driver=None, slow_lower=True, street_mode=2, metric=False, settled=True, max_drop=10, follower=True):
  """limit(f) mph (0 none, km/h when metric), road(f) class, name(f) mapd's road name, nav(f) (its limit, m to its next
  maneuver, a turn), cruise(f) on, vcap(f) the car ahead (m/s or None), gas(f) his foot (m/s he takes the car to, or
  None), driver(f) his button (SET_MINUS / SET_PLUS / None). settled: the follower has seen the road (cruise on) 20 s
  before frame 0 (no fresh engage)."""
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
                           nav_dist=nl[1], nav_turn=nl[2], driver_set=car.driver_button_now() is not None)
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
    want = f_.update(li(fr, on, drv, nav(fr))) if follower else 0.0
    lim = limit(fr)
    fld = F(road_class=road(fr), limit=lim * step if lim else 0.0, gps_trust=trust(fr))
    driver_speed = car.set_speed + logic.offset * logic.step if on else 0.0
    foot = gas(fr)
    mo = ctl.update(fld, driver_speed, 0, v_ego=car.v_ego, max_drop=md, gas=foot is not None,
                    set_speed=car.set_speed if on else None, driver_set=car.driver_button_now() is not None,
                    limit_lower=f_.lower_speed if follower else 0.0)
    if ctl.lower_event is not None:
      f_.lower_finished(ctl.lower_event, car.set_speed / step if on else None)
      ctl.lower_event = None
    active, speed, floor, budget, track, raise_ok = mix.update(False, CTL.NO_CURVE, mo,
                                                               drop_floor=max(driver_speed - md, cs.MIN_SET_SPEED))
    if mo.adopt and logic.offset > 0:
      logic._forget()
      r["adopted"].append(fr)
    inp = cs.SlowdownInputs(enabled=True, cruise_enabled=on, set_speed=car.set_speed, v_ego=car.v_ego,
                            gas_pressed=foot is not None, brake_pressed=False, standstill=car.v_ego < 0.3, plan_alive=True,
                            csc_active=active, csc_speed=min(speed, 100.0), other_button_active=False,
                            driver_button=car.driver_button_now(), curve_floor=floor, curve_budget=budget,
                            curve_track=track, curve_raise=raise_ok, curve_no_accel=mix.no_accel, curve_hold=mix.firm,
                            fast_restore=mo.fast_restore, btn_counter=(fr // 10) % 16, limit_raise=want,
                            limit_cap=f_.cap_speed if follower else 0.0, limit_undo=f_.undo_speed if follower else 0.0,
                            limit_ceiling=f_.ceiling_speed if follower else 0.0)
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
  # RES after a stop at a red light on a 45 street (drive 55, t 581: his highway 75 came back): down to 50 (+5) by taps
  # once moving, then his speed
  r = street_sim(4000, lambda f: 45, 75, v_mph=0, vcap=lambda f: 0.0 if f < 700 else None, **_engage())
  assert abs(r["sets"][-1] - 50) < 1e-6 and r["logic"].anchor is None, (r["sets"][-1], _reasons(r))
  assert _minus(r, ENGAGE) and not _minus(r, 0, 700) and r["holds"] == [], r["presses"][:5]     # (none at a standstill)
  assert any("start: cruise on, down" in x for x in _reasons(r)) and "done" in _reasons(r)
  # SET at 30 on the 45 street: up to 50 (the controller's raise: taps / a hold)
  r = street_sim(4000, lambda f: 45, 30, **_engage())
  assert abs(r["sets"][-1] - 50) < 1e-6 and _plus(r, ENGAGE) and not _minus(r, ENGAGE)
  # the legal limit setting: 45
  r = street_sim(4000, lambda f: 45, 30, street_mode=0, **_engage())
  assert abs(r["sets"][-1] - 45) < 1e-6


def test_engage_with_no_limit_known_only_down_to_43():
  r = street_sim(4000, lambda f: 0, 60, **_engage())
  assert abs(r["sets"][-1] - 43) < 1e-6 and r["logic"].anchor is None, r["sets"][-1]
  r = street_sim(3000, lambda f: 0, 38, **_engage())
  assert r["presses"] == [] and abs(r["sets"][-1] - 38) < 1e-6                       # never raised to 43
  # metric: 70 km/h
  r = street_sim(4000, lambda f: 0, 100, metric=True, **_engage())
  assert abs(r["sets"][-1] - 70) < 1e-6, r["sets"][-1]


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
