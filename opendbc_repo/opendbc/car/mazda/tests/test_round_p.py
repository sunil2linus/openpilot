"""Round P (drives 72 / 73): the logged scenarios as tests."""
import dataclasses
import importlib.util
import math
import os

HERE = os.path.dirname(__file__)


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


TMC = _load("tmc_round_p", os.path.join(HERE, "test_map_curves.py"))
T, cs, CTL = TMC.T, TMC.cs, TMC.CTL
MPH = cs.MS_PER_MPH
KPH = cs.MS_PER_KPH
DT = cs.DT_CTRL


def _inp(**k):
  d = dict(enabled=True, cruise_enabled=True, set_speed=20 * MPH, v_ego=25 * MPH, gas_pressed=False, brake_pressed=False,
           standstill=False, plan_alive=True, csc_active=False, csc_speed=0.0, other_button_active=False)
  d.update(k)
  return cs.SlowdownInputs(**d)


def _acc(g):
  return 0.33 if g < 5 else (0.51 if g < 10 else (0.75 if g < 15 else (0.91 if g < 20 else 1.06)))


class MaxCar(T.Car):
  """The car with its own maximum set speed: a SET+ there does nothing (drive 73: 90 mph, the set stayed 90.1)."""
  def __init__(self, *a, max_set=90 * MPH, **kw):
    super().__init__(*a, **kw)
    self.max_set = max_set

  def tick(self):
    super().tick()
    self.set_speed = min(self.set_speed, self.max_set)


def _boost73(frames=4000, ev=None, set0=42, v0=34.0, limit=75, step=MPH, max_set=90 * MPH, inp=None):
  """Drive 73 t 409-447: the merge onto M-59 (the target 70 + 5), the set 42 at 34 mph, the car's maximum 90 mph."""
  car = MaxCar(set0 * step / MPH, dash_offset_mph=0.0, lag=10, accel_fn=_acc, cruise_gain=0.19, step=step, max_set=max_set)
  car.v_ego = v0 * MPH
  lg = cs.CurveSlowdownLogic(step < MPH, 10)
  log = []
  for f in range(frames):
    extra = inp(f, car, lg) if inp else {}
    i = _inp(set_speed=car.set_speed, v_ego=car.v_ego, limit_raise=limit * MPH if 50 <= f < 55 else 0.0,
             limit_boost=(32 * KPH if step < MPH else 20 * MPH), btn_counter=(f // 10) % 16, **extra)
    btn, _ = lg.update(i)
    if ev:
      ev(f, car, lg)
    if btn is not None:
      car.hold_frame(btn) if lg.last_action == "hold" else car.press(btn)
    log.append((f, car.set_speed / MPH, car.v_ego / MPH, btn, lg.last_action, lg.anchor, lg.backoff_until))
    car.tick()
  return car, lg, log


# ---- fix 1: the boost never aimed above the car's maximum set speed (drive 73 t 405-447) ----

def test_73_the_boosted_anchor_is_capped_at_90_no_set_plus_there_no_back_off():
  car, lg, log = _boost73()
  anchors = [a for *_, a, _ in log if a is not None]
  assert anchors and max(anchors) / MPH <= 90 + 1e-6, max(anchors) / MPH
  # never a SET+ with the set at the car's maximum, so no no-effect presses and no back-off at all
  assert not any(b == cs.SET_PLUS and s >= 90 - 0.5 for _, s, _, b, *_ in log)
  assert lg.no_effect_trips == 0 and all(bu == 0 for *_, bu in log)
  assert abs(car.set_speed / MPH - 75) < 1e-6, car.set_speed / MPH        # down to the target
  # (before round P: the anchor 95, SET+ taps at 90 ignored, a 10 s back-off)
  old = cs.MERGE_BOOST_MAX
  try:
    cs.MERGE_BOOST_MAX = (100, 160)
    _, lg0, log0 = _boost73()
  finally:
    cs.MERGE_BOOST_MAX = old
  assert any(b == cs.SET_PLUS and s >= 90 - 0.5 for _, s, _, b, *_ in log0)


def test_a_lower_maximum_is_learned_and_its_refused_presses_are_no_no_effect():
  # a metric dash on a car whose maximum is 143 km/h (under the assumed 145): one SET+ there that does not land
  car, lg, log = _boost73(step=KPH, max_set=143 * KPH, set0=42 * MPH / KPH)
  assert lg.set_max is not None and abs(lg.set_max / KPH - 143) < 0.6, lg.set_max
  ups_at_max = [f for f, s, _, b, *_ in log if b == cs.SET_PLUS and s * MPH / KPH >= 143 - 0.5]
  assert len(ups_at_max) <= 1, ups_at_max
  assert lg.no_effect_trips == 0 and all(bu == 0 for *_, bu in log)


def test_a_back_off_never_blocks_the_way_down_and_a_third_one_does_not_forget_it():
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=90 * MPH, v_ego=74 * MPH))
  lg.boost = {"target": 75 * MPH, "frame": 0, "ended": 1, "held": True}
  lg.anchor = 75 * MPH
  lg.no_effect_trips = cs.NO_EFFECT_TRIPS - 1
  lg.backoff_until = lg.frame + 10_000
  set_mph, minus = 90.0, []
  for f in range(400):
    lg.no_effect = cs.NO_EFFECT_LIMIT if f == 5 else lg.no_effect     # the third back-off comes now
    b, _ = lg.update(_inp(set_speed=set_mph * MPH, v_ego=74 * MPH, btn_counter=(f // 10) % 16))
    if b == cs.SET_MINUS:
      minus.append(f)
      if lg.last_action != "hold":
        set_mph -= 1
  assert minus and minus[0] < 40, minus[:3]
  assert abs(set_mph - 75) < 1e-6, (set_mph, lg.anchor)             # (not forgotten on the way)
  # (without the rule the back-off held the way down and the third back-off forgot it)
  old = cs.BOOST_DOWN_NO_BACKOFF
  try:
    cs.BOOST_DOWN_NO_BACKOFF = False
    lg = cs.CurveSlowdownLogic(False, 10)
    lg.update(_inp(set_speed=90 * MPH, v_ego=74 * MPH))
    lg.boost = {"target": 75 * MPH, "frame": 0, "ended": 1, "held": True}
    lg.anchor, lg.no_effect_trips, lg.backoff_until = 75 * MPH, cs.NO_EFFECT_TRIPS - 1, lg.frame + 10_000
    lg.no_effect = cs.NO_EFFECT_LIMIT
    outs = [lg.update(_inp(set_speed=90 * MPH, v_ego=74 * MPH))[0] for _ in range(300)]
  finally:
    cs.BOOST_DOWN_NO_BACKOFF = old
  assert cs.SET_MINUS not in outs


# ---- fix 2: the earlier come-down (owner): a taper to the target + 5 from 10 under it (a72/sim1.py) ----

def _after(log, f0):
  return [x for x in log if x[0] >= f0]


def test_73_the_taper_holds_90_to_80_from_65_mph_then_75_from_73_and_no_set_plus_after():
  car, lg, log = _boost73()
  f65 = next(f for f, s, v, *_ in log if v >= 65)
  f73 = next(f for f, s, v, *_ in log if v >= 73)
  downs = [x for x in _after(log, f65) if x[3] == cs.SET_MINUS]
  assert downs and downs[0][0] - f65 <= 60 and downs[0][4] == "hold", downs[:2]     # one SET- hold at once
  assert all(s >= 80 - 1e-6 for f, s, *_ in log if f65 <= f < f73), "never below the target + 5 before the end"
  assert any(abs(s - 80) < 1e-6 for f, s, *_ in log if f65 <= f < f73)
  assert not any(b == cs.SET_PLUS for f, s, v, b, *_ in log if f >= f65), "no SET+ after the taper"
  back = next(f for f, s, *_ in log if f > f73 and s <= 75 + 1e-6)
  assert back - f73 <= 300 and abs(car.set_speed / MPH - 75) < 1e-6
  peak = max(v for _, _, v, *_ in log)
  # (without the taper: the set 90 until 73 mph, the peak higher)
  old = cs.MERGE_TAPER
  try:
    cs.MERGE_TAPER = False
    _, _, log0 = _boost73()
  finally:
    cs.MERGE_TAPER = old
  t73, t73_0 = f73, next(f for f, s, v, *_ in log0 if v >= 73)
  # (a72/sim1.py: ~1.2-1.5 s more to 73, the peak ~75; on the drive the back-off made it 81)
  assert peak <= 76.0 and 0 <= t73 - t73_0 <= 200, (peak, t73, t73_0)


def test_the_taper_also_when_the_highway_is_matched_or_the_ramp_flag_drops():
  # the ramp flag drops at 55 mph (the planner's merge_taper): the taper at once, the set never under 80 before the end
  state = {}

  def inp(f, car, lg):
    if car.v_ego >= 55 * MPH and "f" not in state:
      state["f"] = f
    return {"merge_taper": "f" in state}
  car, lg, log = _boost73(inp=inp)
  f55 = state["f"]
  f73 = next(f for f, s, v, *_ in log if v >= 73)
  assert next(f for f, s, v, b, *_ in log if f >= f55 and b == cs.SET_MINUS) - f55 <= 60
  assert all(s >= 80 - 1e-6 for f, s, *_ in log if f55 <= f < f73)
  assert not any(b == cs.SET_PLUS for f, s, v, b, *_ in log if f >= f55 + 5)
  assert abs(car.set_speed / MPH - 75) < 1e-6


def test_the_wrapper_tapers_on_the_highway_match_or_when_the_ramp_flag_drops():
  from types import SimpleNamespace
  w = SimpleNamespace(logic=SimpleNamespace(boost=None), boost_ramp_seen=False)
  taper = lambda plan: cs.CurveSlowdown._merge_taper(w, plan)  # noqa: E731
  assert not taper(SimpleNamespace(mapRoadClass=1, mapOnRamp=False))           # no boost: nothing
  w.logic.boost = {"target": 75 * MPH, "frame": 0}
  assert not taper(SimpleNamespace(mapRoadClass=2, mapOnRamp=False))           # a link, the flag never seen: not yet
  assert not taper(SimpleNamespace(mapRoadClass=2, mapOnRamp=True))
  assert taper(SimpleNamespace(mapRoadClass=2, mapOnRamp=False))               # the ramp flag dropped
  w.boost_ramp_seen = False
  assert taper(SimpleNamespace(mapRoadClass=1, mapOnRamp=False))               # the highway matched
  w.logic.boost = None
  assert not taper(None) and not w.boost_ramp_seen


# ---- fix 3: navigation's exit with his signal toward it: the ramp's follow from the split (drive 73 t 491-524, exit 46) ----

TE = _load("tep_round_p", os.path.join(HERE, "test_exit_plan.py"))


def _e46(signal_on=True):
  """Drive 73, exit 46: 75 set on the 70 highway, the 85 m exit curve (100 m as published, map_curves.RAMP_LAT) 476 m past
  the split, the shapes saying "link" only 150 m past it, his right signal ~400-60 m before the split."""
  exit_x, curve_x = 2000.0, 2476.0
  base = TE.exit_fields(exit_x, curve_x, r=100.0, ann=900.0, clear=10.0, link_at=150.0, ramp_at=190.0, curve_held=200.0)
  # (navigation's notice had no curve: only the planner's exit curve, so no short ramp split speed; 69 mph at the split)
  fields = lambda x: dataclasses.replace(base(x), nav_curve_radius=0.0, nav_curve_distance=0.0)  # noqa: E731
  sig = (lambda x: 1 if exit_x - 400 <= x < exit_x - 60 else 0) if signal_on else None
  lane = lambda x: 1 if x >= exit_x - 300 else None  # noqa: E731
  r = TE.drive_exit(fields, 9000, 75, signal=sig, lane=lane)
  return r, exit_x, curve_x


def test_73_exit_46_with_his_signal_the_firm_follow_starts_at_the_split():
  r, exit_x, curve_x = _e46()
  v_split = TE._at(r, exit_x)
  assert 67 <= v_split <= 72, v_split
  # a firm SET- hold within 0.5 s of the split (was: the tentative TENT_GAP taps, the confirmation 6 s later)
  f_split = next(k for k, xx in enumerate(r["xs"]) if xx >= exit_x)
  holds = [p for p in r["presses"] if p[0] >= f_split and p[2] == cs.SET_MINUS and p[3] == "hold"]
  assert holds and holds[0][0] - f_split <= 50, (holds[:1], f_split)
  assert "exit signalled" in r["why"]
  v_c = TE._at(r, curve_x)
  assert v_c <= 35.0, v_c
  # without his signal: as before (the tentative slowdown, the confirmation later, faster at the curve)
  r0, _, _ = _e46(signal_on=False)
  assert "exit signalled" not in r0["why"] and "exit tentative" in r0["why"]
  assert TE._at(r0, curve_x) > v_c + 2.0, (TE._at(r0, curve_x), v_c)


# ---- fix 5: a curve target blip does not end the boost in the frame it began (drive 73 t 1160.7, the loop on-ramp) ----

def _blip_run(on_frames, csc_mph=72.8):
  def inp(f, car, lg):
    on = f in on_frames
    return {"csc_active": on, "csc_speed": csc_mph * MPH if on else 0.0}
  return _boost73(frames=1500, inp=inp)


def test_73_a_curve_release_blip_of_0_2_s_under_the_anchor_keeps_the_boost_and_its_hold():
  car, lg, log = _blip_run(range(53, 73))            # (the merge adopted at frame 50, the blip from 53 for 0.2 s)
  assert lg.boost is not None and "ended" not in lg.boost
  assert all(a is None or abs(a / MPH - 90) < 1e-6 for f, *_, a, _ in log if 50 <= f < 1400)
  ups = [h for h in lg.hold_log if h["direction"] == cs.SET_PLUS]
  assert ups and ups[0]["frame"] < 120, ups[:1]        # the boost's hold goes on right after the blip
  assert max(s for _, s, *_ in log) >= 89.5            # ... and the set to the cap
  # a curve target under the target for MERGE_CSC_END or more still ends it (as before)
  car, lg, log = _blip_run(range(53, 53 + int(cs.MERGE_CSC_END / DT) + 5))
  assert lg.boost is None or "ended" in lg.boost
  assert max(s for _, s, *_ in log) <= 75 + 1e-6
  # (before round P: ended in the blip's first frame)
  old = cs.MERGE_CSC_END
  try:
    cs.MERGE_CSC_END = 0.0
    car, lg, log = _blip_run(range(53, 73))
  finally:
    cs.MERGE_CSC_END = old
  assert max(s for _, s, *_ in log) <= 75 + 1e-6


# ---- fix 4: his RES after our lowering: the fast raise at once (drive 73 t 540.9, drive 72 t 406.6) ----

def _res_run(set0, v0, anchor, frames=1500, res=300, gas_until=None, extra=lambda f, car: {}, stop_episode=False):
  """Cruise off (his brake) until his RES at `res` with the set our slowdown left (set0) and his `anchor`; his gas held
  from frame 100 until `gas_until` (the car speeding up on it)."""
  car = T.Car(set0, dash_offset_mph=0.0, lag=10, accel_fn=_acc, cruise_gain=0.19)
  car.v_ego = v0 * MPH
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=car.set_speed, v_ego=car.v_ego))
  lg.anchor, lg.stop_episode = anchor * MPH, stop_episode
  log = []
  for f in range(frames):
    cruise = f >= res
    gas = gas_until is not None and 100 <= f < gas_until
    kw = dict(stop_clear_ahead=True, stop_pulling_away=True)
    kw.update(extra(f, car))
    btn, _ = lg.update(_inp(cruise_enabled=cruise, set_speed=car.set_speed if cruise else 0.0, v_ego=car.v_ego,
                            gas_pressed=gas, btn_counter=(f // 10) % 16, **kw))
    if btn is not None and cruise:
      car.hold_frame(btn) if lg.last_action == "hold" else car.press(btn)
    if gas:
      car.gas_frame = car.frame
    if cruise and not gas:
      car.tick()
    else:
      car.frame += 1
      car.tick() if cruise else None
      car.v_ego += (1.5 if gas else 0.0) * DT
    log.append((f, car.set_speed / MPH, car.v_ego / MPH, btn, lg.last_action))
  return car, lg, log


def _street43(f, car):
  # the exit's street target (map_curve_control "exit street": the fallback 43, settled)
  return dict(csc_active=True, csc_speed=43 * MPH, curve_track=True, curve_raise=True, street_lower=True,
              curve_floor=25 * MPH)


def test_73_res_at_29_with_the_set_at_25_raises_at_once_after_his_gas_to_the_street_target():
  res, gas_end = 300, 360                        # his RES on the gas at 29 mph, the gas released 0.6 s later
  car, lg, log = _res_run(25, 29, 75, res=res, gas_until=gas_end, extra=_street43)
  ups = [f for f, s, v, b, a in log if b == cs.SET_PLUS]
  assert ups and ups[0] - gas_end <= 30, (ups[:3], gas_end)        # within 0.3 s of the gas release
  assert log[ups[0]][4] in ("jump", "hold"), log[ups[0]]             # a jump to the speed or a hold, not a 1 mph tap
  assert max(s for f, s, *_ in log) <= 43 + 1e-6                     # never above the street's target
  t43 = next(f for f, s, *_ in log if s >= 43 - 1e-6)
  assert t43 - gas_end <= 400, t43 - gas_end                        # (was: 14.7 s to 53 by taps)
  assert min(v for f, s, v, *_ in log if f >= gas_end) >= 28.5
  # (before round P: the first SET+ 2 s later, taps only)
  old = cs.ENGAGE_RAISE
  try:
    cs.ENGAGE_RAISE = False
    car0, lg0, log0 = _res_run(25, 29, 75, res=res, gas_until=gas_end, extra=_street43)
  finally:
    cs.ENGAGE_RAISE = old
  t43_0 = next((f for f, s, *_ in log0 if s >= 43 - 1e-6), 10**9)
  assert t43_0 > t43 + 150, (t43_0, t43)


def _lead46(f, car):
  return dict(radar_lead=True, lead_v=20.6)


def test_72_res_at_45_with_the_set_at_20_raises_to_his_speed_at_once():
  res = 300
  car, lg, log = _res_run(20, 45, 48, res=res, gas_until=res - 100, extra=_lead46, stop_episode=True)
  ups = [f for f, s, v, b, a in log if b == cs.SET_PLUS]
  assert ups and ups[0] - res <= 50, ups[:3]
  assert log[ups[0]][4] == "hold"                                    # the hold at once (was: 1 mph taps)
  # (the hold to 40, a quantum of margin under his 48, then taps: the measured hold margins)
  t40 = next(f for f, s, *_ in log if f >= res and s >= 40 - 1e-6)
  assert t40 - res <= 300, t40 - res
  v_res = log[res][2]                                                # (his gas took the car to ~48.4 by then)
  assert max(s for f, s, *_ in log) <= max(48, math.ceil(v_res - 0.25)) + 1e-6
  v_low = min(v for f, s, v, *_ in log if f >= res)
  old = cs.ENGAGE_RAISE
  try:
    cs.ENGAGE_RAISE = False
    _, _, log0 = _res_run(20, 45, 48, res=res, gas_until=res - 100, extra=_lead46, stop_episode=True)
  finally:
    cs.ENGAGE_RAISE = old
  v_low0 = min(v for f, s, v, *_ in log0 if f >= res)
  assert v_low >= v_low0 + 1.0 and v_low >= 41.0, (v_low, v_low0)


def test_res_with_a_red_light_or_a_slower_lead_or_a_lower_curve_no_set_plus():
  res = 300
  for extra in (lambda f, car: dict(red_stop=True),
                lambda f, car: dict(radar_lead=True, lead_v=8.0, stop_clear_ahead=False, stop_pulling_away=False),
                lambda f, car: dict(stop_slower_ahead=True),
                lambda f, car: dict(stop_active=True, stop_target=20 * MPH),
                lambda f, car: dict(csc_active=True, csc_speed=30 * MPH)):
    car, lg, log = _res_run(20, 45, 48, frames=900, res=res, gas_until=res - 100, extra=extra, stop_episode=True)
    assert not lg._engage_go()
    assert not any(b == cs.SET_PLUS and a == "hold" for f, s, v, b, a in log if f >= res)


def test_res_after_a_turn_takes_the_old_speed_only_up_to_the_streets_target():
  res = 300
  extra = lambda f, car: dict(road_changed=True, engage_goal=40 * MPH)  # noqa: E731
  car, lg, log = _res_run(25, 30, 75, res=res, extra=extra, stop_episode=True)
  assert abs(car.set_speed / MPH - 40) < 1e-6 and max(s for _, s, *_ in log) <= 40 + 1e-6
  # nothing known about the street: the car's speed at least, never his old highway speed
  extra = lambda f, car: dict(road_changed=True)  # noqa: E731
  car, lg, log = _res_run(25, 33, 75, res=res, extra=extra, stop_episode=True)
  assert 33 - 1e-6 <= max(s for _, s, *_ in log) <= 34 + 1e-6, max(s for _, s, *_ in log)


def test_res_with_nothing_lowered_never_raises_his_own_set():
  # (review of round P, B1) his own set 25 (nothing of ours anchored), RES at 40 mph, a 45 street: his set stays
  for extra in ({}, {"engage_goal": 50 * MPH}):
    car = T.Car(25, dash_offset_mph=0.0, lag=10, accel_fn=_acc, cruise_gain=0.19)
    car.v_ego = 40 * MPH
    lg = cs.CurveSlowdownLogic(False, 10)
    lg.update(_inp(set_speed=car.set_speed, v_ego=car.v_ego))
    for f in range(800):
      cruise = not (50 <= f < 200)
      btn, _ = lg.update(_inp(cruise_enabled=cruise, set_speed=car.set_speed if cruise else 0.0, v_ego=car.v_ego,
                              stop_clear_ahead=True, stop_pulling_away=True, btn_counter=(f // 10) % 16, **extra))
      assert btn is None, (f, extra)
      if cruise:
        car.tick()
    assert lg.engage is None and abs(car.set_speed / MPH - 25) < 1e-6


def test_review_b1_his_own_set_on_a_street_is_never_raised_test_or_on(tmp_path, monkeypatch):
  import pytest
  pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)
  WI = _load("twi_round_pb1", os.path.join(HERE, "test_wrapper_inputs.py"))
  for limit_mode, gas_v in ((1, None), (2, None), (0, 45)):
    c, sm, _ = WI._make(tmp_path, monkeypatch, limit_mode=limit_mode, stop_mode=2)
    sm.msgs["starpilotPlan"].mapRoadClass = 5                    # his own 30 on the 45 street
    for k in range(1200):
      cruise = not (300 <= k < 400)                              # his CANCEL, then RES at his own 30
      gas = gas_v is not None and 380 <= k < 460
      st = WI.W.car_state(gas_v if gas else 30, 30 if cruise else 0.0, cruise=cruise, gas=gas)
      out = c.update(WI.W.CC, st, None, None, False)
      assert not (out and k >= 400), (limit_mode, k, c.logic.last_action)


# ---- fix 6: the exit's adoption after a stop hands over to the street's target (drive 73 t 1296.2, South Blvd) ----

F = CTL.MapFields


def test_73_south_blvd_the_exits_street_cap_is_the_handover_target():
  ctl = CTL.MapCurveControl(3.0, street_mode=1)                 # (his Street Speed +3: the street cap 48 for the 45 road)
  ramp = dict(road_class=2, on_ramp=True, ramp_type=1, ramp_to_speed=45 * MPH, ramp_to_highway=False)
  for k in range(400):
    ctl.update(F(mode=CTL.ON, ramp_end_distance=max(300.0 - k, 0.0), **ramp), 75 * MPH, 0, v_ego=9.0, set_speed=26 * MPH)
  mo = ctl.update(F(mode=CTL.ON, road_class=5), 75 * MPH, 0, v_ego=9.0, set_speed=26 * MPH)
  assert mo.adopt and abs(ctl.handover_target() / MPH - 48) < 1e-6, ctl.handover_target() / MPH
  # (the street's limit unknown: the fallback, 43)
  ctl = CTL.MapCurveControl(3.0, street_mode=1)
  ramp["ramp_to_speed"] = 0.0
  for k in range(400):
    ctl.update(F(mode=CTL.ON, ramp_end_distance=max(300.0 - k, 0.0), **ramp), 75 * MPH, 0, v_ego=9.0, set_speed=26 * MPH)
  assert abs(ctl.handover_target() / MPH - 43) < 1e-6


def _handover_run(stopped=True, set0=26, target=48, extra=lambda f: {}, frames=1500):
  """The exit's slowdown and a queue left the set at 26 (his 75); at frame 10 (before anything of ours) the exit ends by
  adoption."""
  car = T.Car(set0, dash_offset_mph=0.0, lag=10, accel_fn=_acc, cruise_gain=0.19)
  car.v_ego = 20 * MPH
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=car.set_speed, v_ego=car.v_ego))
  lg.anchor = 75 * MPH
  goals, log = [], []
  for f in range(frames):
    if f == 10:
      g = cs.exit_handover(lg, car.set_speed, target * MPH, stopped)
      goals.append(g)
      lg.handover(g) if g > 0 else lg._forget()
    kw = dict(stop_clear_ahead=True, stop_pulling_away=True)
    kw.update(extra(f))
    btn, _ = lg.update(_inp(set_speed=car.set_speed, v_ego=car.v_ego, btn_counter=(f // 10) % 16, **kw))
    if btn is not None:
      car.hold_frame(btn) if lg.last_action == "hold" else car.press(btn)
    car.tick()
    log.append((f, car.set_speed / MPH, btn))
  return car, lg, goals, log


def test_73_south_blvd_after_the_queue_the_set_goes_to_the_street_cap_not_left_at_26():
  car, lg, goals, log = _handover_run()
  assert abs(goals[0] / MPH - 48) < 1e-6
  assert abs(car.set_speed / MPH - 48) < 1e-6 and max(s for _, s, _ in log) <= 48 + 1e-6
  first = next(f for f, s, b in log if b == cs.SET_PLUS)
  assert first - 10 <= 50, first
  # (before round P: adopted, 26 his speed, nothing raised)
  old = cs.EXIT_HANDOVER
  try:
    cs.EXIT_HANDOVER = False
    car0, _, goals0, log0 = _handover_run()
  finally:
    cs.EXIT_HANDOVER = old
  assert goals0 == [0.0] and abs(car0.set_speed / MPH - 26) < 1e-6
  # the same guards as his RES: a red light: no SET+ now (the target stays his speed for later)
  car, lg, goals, log = _handover_run(extra=lambda f: dict(red_stop=True), frames=800)
  assert not any(b == cs.SET_PLUS for _, _, b in log) and abs(lg.anchor / MPH - 48) < 1e-6
  car, lg, goals, log = _handover_run(extra=lambda f: dict(stop_slower_ahead=True, stop_clear_ahead=False,
                                                           stop_pulling_away=False), frames=800)
  # (a slower car: not the fast raise; the usual restore to the street's target, the radar keeping the gap)
  assert not lg._engage_go()
  assert max(s for _, s, _ in log) <= 48 + 1e-6


def test_an_exit_without_a_stop_and_the_set_near_the_streets_target_is_still_adopted():
  car, lg, goals, log = _handover_run(stopped=False, set0=45)
  assert goals == [0.0] and lg.anchor is None and abs(car.set_speed / MPH - 45) < 1e-6
  # far under it without a stop: handed over too
  car, lg, goals, log = _handover_run(stopped=False, set0=30)
  assert abs(goals[0] / MPH - 48) < 1e-6 and abs(car.set_speed / MPH - 48) < 1e-6


def test_the_wrapper_hands_the_exit_over_to_the_street_cap_after_a_stop(tmp_path, monkeypatch):
  import pytest
  pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)
  WI = _load("twi_round_p", os.path.join(HERE, "test_wrapper_inputs.py"))
  c, sm, _ = WI._make(tmp_path, monkeypatch, limit_mode=0, map_mode=2)
  plan = sm.msgs["starpilotPlan"]
  plan.mapCurveMode, plan.mapRoadClass, plan.mapOnRamp, plan.mapRampType, plan.mapRampToSpeed = 2, 2, True, 1, 45 * MPH
  plan.mapRampToHighway, plan.mapRampEndDistance = False, 300.0
  WI._drive(c, 50, set_mph=26, v_mph=20)
  c.logic.anchor = 75 * MPH                                      # (his highway 75, our exit slowdown left 26)
  for k in range(250):
    plan.mapRampEndDistance = max(300.0 - k, 0.0)
    cs_ = WI.W.car_state(0.0 if 100 <= k < 150 else 9.0, 26)
    cs_.out.standstill = 100 <= k < 150                          # (the queue on the ramp)
    c.update(WI.W.CC, cs_, None, None, False)
  plan.mapOnRamp, plan.mapRoadClass = False, 5                   # off the ramp onto the street: the adoption
  WI._drive(c, 3, set_mph=26, v_mph=9)
  assert c.logic.anchor is not None and abs(c.logic.anchor / MPH - 50) < 1e-6, c.logic.anchor   # (the default +5)
  assert c.logic.engage is not None and c.logic.engage.get("fixed")


# ---- fix 8: never under the stock LKAS cut-off for a curve that does not need it; TI-only planning when it does ----

CT = _load("ct_round_p", os.path.join(HERE, "..", "curve_target.py"))


def test_map_curves_ti_only_lateral_by_branch():
  c = CTL.MapCurveControl(3.0)
  c.ti_plan = True
  # drive 73's loop (71 m on the map, published 84 m for RAMP_LAT): its target 30.3 mph needs no TI-only plan
  assert abs(c._lat(84.0, scaled=True) - CTL.MAP_LATERAL_ACCEL) < 1e-9
  assert CTL.map_curve_target(84.0, c._lat(84.0, scaled=True)) / MPH >= 28
  # (review of round P, S3) a 50 m road curve (23.5 mph at 2.2) with the LKAS working: its normal plan, no TI-only re-plan
  assert abs(c._lat(50.0) - CTL.MAP_LATERAL_ACCEL) < 1e-9
  # the LKAS blocked: every curve TI-only; a ramp curve's published radius at TI_ONLY_LAT for its real radius
  c.ti_only = True
  assert abs(c._lat(84.0, scaled=True) - CTL.TI_ONLY_LAT * CTL.TI_ONLY_RAMP_SCALE) < 1e-9
  assert abs(CTL.map_curve_target(84.0, c._lat(84.0, scaled=True)) - math.sqrt(1.8 * 71.0)) < 0.05
  assert abs(c._lat(90.0) - CTL.TI_ONLY_LAT) < 1e-9                              # (West Utica, a road curve: 28.5 mph)
  # no TI: never
  c.ti_plan = False
  assert c._lat(50.0) == c.lat_accel


def test_the_camera_target_ti_only_while_blocked():
  t = CT.CurveTarget(3.0)
  t.ti_lat = CTL.TI_ONLY_LAT
  k_tight, k_mild = 3.0 / (25 * MPH) ** 2, 3.0 / (40 * MPH) ** 2      # 25 mph and 40 mph at 3.0 m/s^2
  for _ in range(150):
    t.update(k_tight, 60 * MPH)
  assert abs(t.speed / MPH - 25) < 1e-6                                 # (S3) under 28 with the LKAS working: as before
  t.reset()
  for _ in range(150):
    t.update(k_mild, 60 * MPH)
  assert abs(t.speed / MPH - 40) < 1e-6                                 # 40 mph: as before
  t.ti_only = True
  for _ in range(150):
    t.update(k_mild, 60 * MPH)
  assert abs(t.speed - math.sqrt(1.8 / k_mild)) < 1e-6                  # blocked: at 1.8


def test_the_controller_floors_curve_targets_but_never_a_stop_at_lkas_keep():
  keep = cs.LKAS_KEEP_SPEED[0]

  def run(**kw):
    lg = cs.CurveSlowdownLogic(False, 10)
    lg.update(_inp(set_speed=45 * MPH, v_ego=45 * MPH))
    set_mph = 45.0
    for f in range(1500):
      b, _ = lg.update(_inp(set_speed=set_mph * MPH, v_ego=max(set_mph - 1, 31) * MPH, btn_counter=(f // 10) % 16, **kw))
      if b == cs.SET_MINUS and lg.last_action != "hold":
        set_mph -= 1
      elif b == cs.SET_MINUS and f % 10 == 0:
        set_mph -= 1
    return set_mph
  # the firm ramp follow asks 24.3 (30.3 - 6) on a link: floored at 30
  firm = dict(csc_active=True, csc_speed=24.3 * MPH, curve_track=True, curve_hold=True, curve_floor=24.3 * MPH,
              curve_budget=60)
  assert run(**firm) <= 26 and run(lkas_keep=keep, **firm) >= 30 - 1e-6
  # (review of round P, B3) a stop's dip, also inside a followed curve: not floored, under 30 as before
  stop = dict(stop_active=True, stop_target=20 * MPH, stop_urgent=True)
  inside = dict(csc_active=True, csc_speed=31 * MPH, curve_track=True, curve_floor=25 * MPH)
  assert run(lkas_keep=keep, **stop, **inside) <= 25
  assert run(**stop, **inside) <= 25
  assert run(lkas_keep=keep, **stop) <= 25


def _loop73(keep_on=True, frames=4000):
  """Drive 73's loop on-ramp (t 1112-1150): 45 mph with his 53 on the street, the 84 m (published) ramp curve from 300 m,
  its target 30.3 mph; the wrapper's LKAS_KEEP floor while the LKAS is not blocked."""
  car = T.Car(49, dash_offset_mph=0.0, lag=10, accel=0.6, **TE.TM.MEASURED_CRUISE)
  car.v_ego = 45 * MPH
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=car.set_speed, v_ego=car.v_ego))
  lg.anchor = 53 * MPH
  ctl = CTL.MapCurveControl(3.0)
  ctl.ti_plan = True
  mix = CTL.CurveMix(cs.CURVE_MARGIN)
  x, sets, vs = 0.0, [], []
  for f in range(frames):
    d = 300.0 - x
    fld = F(mode=CTL.ON, status=2, radius=84.0, distance=max(d, 0.0), on_ramp=d < 120, ramp_type=2, ramp_to_highway=True,
            road_class=2 if d < 120 else 5, onramp_ahead=d >= 120) if d > -150 else F(mode=CTL.ON, road_class=2)
    driver = car.set_speed + lg.offset * lg.step
    mo = ctl.update(fld, driver, 0, v_ego=car.v_ego, max_drop=10 * MPH, set_speed=car.set_speed)
    act, sp, fl, bud, tr, ra = mix.update(False, CTL.NO_CURVE, mo, drop_floor=max(driver - 10 * MPH, cs.MIN_SET_SPEED))
    keep = min(cs.LKAS_KEEP_SPEED[0], mo.curve_speed) if (keep_on and act and mo.curve_speed < 100) else 0.0
    b, _ = lg.update(_inp(set_speed=car.set_speed, v_ego=car.v_ego, csc_active=act, csc_speed=min(sp, 100.0), curve_floor=fl,
                          curve_budget=bud, curve_track=tr, curve_raise=ra, curve_hold=mix.firm, curve_no_accel=mix.no_accel,
                          lkas_keep=keep, btn_counter=(f // 10) % 16))
    if b is not None:
      car.hold_frame(b) if lg.last_action == "hold" else car.press(b)
    car.tick()
    x += car.v_ego * DT
    if -150 < d < 300:
      sets.append(car.set_speed / MPH)
      vs.append(car.v_ego / MPH)
  return sets, vs


def test_73_loop_the_set_never_under_30_for_its_30_3_mph_target():
  sets, vs = _loop73()
  assert min(sets) >= 30 - 1e-6, min(sets)
  assert min(vs) >= 28.5, min(vs)                       # (45 km/h = 28 mph: the LKAS stays on)
  sets0, vs0 = _loop73(keep_on=False)
  assert min(sets0) < 29, min(sets0)                    # (before: the firm undershoot under the cut-off)


# ---- fix 9: the camera's fault / LKAS_BLOCK above the cut-off: TI-only (and the merge's early raise off) ----

def _carstate():
  from opendbc.can import CANPacker
  from opendbc.car import Bus, structs
  from opendbc.car.mazda.carstate import CarState
  from opendbc.car.mazda.values import CAR, MazdaSafetyFlags
  CP = structs.CarParams()
  CP.carFingerprint = CAR.MAZDA_CX5
  CP.flags = int(MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR)
  c = CarState(CP, None)
  ps = CarState.get_can_parsers(CP)
  before = {b: set(ps[b].message_states) for b in (Bus.pt, Bus.cam)}
  c.update(ps, None)                     # (the parsers register the messages read)
  return c, ps, CANPacker("mazda_2017"), Bus, before


def test_carstate_reads_lkas_block_and_the_cameras_err_bits_from_messages_already_parsed():
  c, ps, packer, Bus, _ = _carstate()
  pt_msgs, cam_msgs = set(ps[Bus.pt].message_states), set(ps[Bus.cam].message_states)
  assert 0x241 in pt_msgs and 0x243 in cam_msgs
  t = 2_000_000_000
  for blk, e1, e2, fault in ((1, 0, 1, 2), (0, 1, 0, 2), (0, 0, 0, 0)):
    a1, d1, _ = packer.make_can_msg("STEER_RATE", 0, {"LKAS_BLOCK": blk, "LKAS_EFFECTIVE": 37})
    a2, d2, _ = packer.make_can_msg("CAM_LKAS", 2, {"ERR_BIT_1": e1, "ERR_BIT_2": e2})
    ps[Bus.pt].update([(t, [(a1, d1, 0)])])
    ps[Bus.cam].update([(t, [(a2, d2, 2)])])
    t += 10_000_000
    c.update(ps, None)
    assert c.lkas_block == bool(blk) and c.lkas_effective == 37 and c.cam_err == bool(e1 or e2) and c.cam_fault == fault
  # no new message for either parser (canValid unchanged)
  assert set(ps[Bus.pt].message_states) == pt_msgs and set(ps[Bus.cam].message_states) == cam_msgs


def test_the_wrapper_plans_ti_only_while_the_camera_reports_a_fault_and_reverts_after_2_s(tmp_path, monkeypatch):
  import pytest
  pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)
  WI = _load("twi_round_p9", os.path.join(HERE, "test_wrapper_inputs.py"))
  c, sm, _ = WI._make(tmp_path, monkeypatch, limit_mode=0, map_mode=2)

  def step(n, v_mph=50, **flags):
    for _ in range(n):
      s = WI.W.car_state(v_mph, 55)
      for k, val in flags.items():
        setattr(s, k, val)
      c.update(WI.W.CC, s, None, None, False)
  step(10)
  assert not c.ti_only and not c.ti_fault
  step(10, cam_err=True)
  assert c.ti_only and c.ti_fault and c.map_control.ti_only and c.curve_target.ti_only
  step(150)                                                  # cleared: held TI_ONLY_HYST
  assert c.ti_fault
  step(60)
  assert not c.ti_fault and not c.ti_only and not c.map_control.ti_only
  step(10, stock_lkas_off=True)
  assert c.ti_fault
  step(250)
  # LKAS_BLOCK above the cut-off is a fault; under it (the EPS's own low-speed block) only TI-only planning
  step(10, v_mph=40, lkas_block=True)
  assert c.ti_fault and c.ti_only
  step(250, v_mph=40)
  step(10, v_mph=20, lkas_block=True)
  assert c.ti_only and not c.ti_fault


# ---- fix 11: logging (starpilotCarState, the signal rows, camFault / camFaultCleared) ----

def test_starpilot_car_state_carries_the_cameras_fault_the_lkas_block_and_the_ti_state():
  from types import SimpleNamespace as N
  from cereal import custom
  from opendbc.car.interfaces import mazda_lkas_state
  assert mazda_lkas_state(N()) == (0, False, 0)                                     # (not a Mazda)
  assert mazda_lkas_state(N(cam_fault=2, lkas_block=True, ti_state_log=3)) == (2, True, 3)
  m = custom.StarPilotCarState.new_message()
  m.mazdaCamFault, m.mazdaLkasBlock, m.mazdaTiState = mazda_lkas_state(N(cam_fault=1, lkas_block=False, ti_state_log=2))
  assert (m.mazdaCamFault, m.mazdaLkasBlock, m.mazdaTiState) == (1, False, 2)


def test_the_signal_rows_and_the_cam_fault_events(tmp_path, monkeypatch):
  import pytest
  pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)
  WI = _load("twi_round_p11", os.path.join(HERE, "test_wrapper_inputs.py"))
  c, sm, records = WI._make(tmp_path, monkeypatch, limit_mode=0, map_mode=2)
  rows = []
  monkeypatch.setattr(c.recorder, "add_row", lambda t, row: rows.append(row))

  def step(n, **flags):
    for _ in range(n):
      s = WI.W.car_state(50, 55)
      for k, val in flags.items():
        setattr(s, k, val)
      c.update(WI.W.CC, s, None, None, False)
  step(20)
  step(50, cam_fault=2, cam_err=True, lkas_block=True, ti_state_log=3)
  step(50)
  cols = cs.CurveSlowdown.EVENT_COLUMNS
  assert rows and all(len(r) == len(cols) for r in rows)
  k = cols.index("cam_fault")
  assert {r[k] for r in rows} == {0, 2} and any(r[cols.index("lkas_block")] for r in rows)
  assert any(r[cols.index("ti_state")] == 3 for r in rows) and any(r[cols.index("ti_only")] for r in rows)
  kinds = [kd for kd, _ in records]
  assert kinds.count("camFault") == 1 and kinds.count("camFaultCleared") == 1
  fault = next(d for kd, d in records if kd == "camFault")
  assert fault["kind"] == "ERR bits" and fault["panda_can"] == []        # (no pandaStates in the test's SubMaster)


# ---- fix 10: the camera fault notice (text by kind, again after clearing, "cleared") ----

def _events_modules():
  import pytest
  try:
    from openpilot.selfdrive.selfdrived import events
    from openpilot.starpilot.controls.lib import starpilot_events as spe
  except Exception as e:                                          # (needs the device's native modules, or their stubs)
    pytest.skip(f"events.py not importable here: {e}")
  return events, spe


def test_the_camera_fault_notice_text_by_kind():
  from types import SimpleNamespace as N
  events, _ = _events_modules()
  a = events.mazda_cam_fault_alert(None, None, {"starpilotCarState": N(mazdaCamFault=2)}, False, 0, None, N())
  assert a.alert_text_1 == "Car's camera reports a fault"
  # (the UI's width, test_alerts.test_alert_text_length: 1809 of 1860 px at Inter-SemiBold 66)
  assert a.alert_text_2 == "Collision warning, lane keep may be off · steering weaker"
  b = events.mazda_cam_fault_alert(None, None, {"starpilotCarState": N(mazdaCamFault=1)}, False, 0, None, N())
  assert b.alert_text_1 == "Car's Lane Keep Assist is off" and "Restart" not in b.alert_text_2
  assert events.mazda_cam_fault_alert(None, None, {}, False, 0, None, N()).alert_text_1 == "Car's Lane Keep Assist is off"
  E = events.STARPILOT_EVENTS
  assert E[events.StarPilotEventName.mazdaStockLkasOff][events.ET.PERMANENT] is events.mazda_cam_fault_alert
  assert E[events.StarPilotEventName.mazdaCamFaultCleared][events.ET.PERMANENT].alert_text_1 == "Car's camera fault cleared"


def test_the_camera_fault_notice_comes_back_after_clearing_at_most_every_60_s():
  _, spe = _events_modules()
  n = spe.MazdaCamNotice(dt=0.05)
  run = lambda fault, s: [x for x in (n.update(fault) for _ in range(int(round(s / 0.05)))) if x]  # noqa: E731
  assert run(2, 5.0) == ["fault"]                           # once while it lasts
  assert run(0, 1.0) == [] and run(2, 1.0) == []            # a short dropout: the same episode
  assert run(0, 3.0) == ["cleared"]                         # gone 2 s: "cleared"
  assert run(1, 5.0) == [] and run(0, 3.0) == []            # back within 60 s of the notice: held back (and no "cleared")
  run(0, 60.0)
  assert run(2, 1.0) == ["fault"] and run(0, 3.0) == ["cleared"]   # back after 60 s: again


# ---- fix 7: the early raise at the end of a loop on-ramp (owner: "start raising when the curve is about to end") ----

def _early_run(early_from=100, gate_from=300, frames=600, **kw):
  """On a loop's link at 30 mph with the set 30, a merge raise to 75 (+ the boost) waiting for the turn gate (ok from
  `gate_from`); the wrapper's MERGE_EARLY conditions from `early_from`."""
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=30 * MPH, v_ego=30 * MPH))
  set_mph, log = 30.0, []
  for f in range(frames):
    b, _ = lg.update(_inp(set_speed=set_mph * MPH, v_ego=30 * MPH, limit_raise=75 * MPH, limit_boost=20 * MPH,
                          merge_turn_ok=f >= gate_from, merge_early=f >= early_from, btn_counter=(f // 10) % 16, **kw))
    if b == cs.SET_PLUS and (lg.last_action != "hold" or f % 10 == 0):
      set_mph += 1 if lg.last_action != "hold" else 5
    log.append((f, set_mph, b, lg.last_action))
  return lg, log


def test_the_early_raise_taps_to_the_speed_plus_5_then_the_boost_when_the_gate_opens():
  lg, log = _early_run()
  ups = [(f, a) for f, s, b, a in log if b == cs.SET_PLUS]
  assert ups and 100 <= ups[0][0] <= 130 and ups[0][1] == "early", ups[:2]
  assert max(s for f, s, *_ in log if f < 300) == 35, max(s for f, s, *_ in log if f < 300)   # the speed + 5, no more
  assert all(a == "early" for f, a in ups if f < 300)
  assert lg.boost is not None and max(s for f, s, *_ in log) > 40                              # the boost from the gate
  # without the conditions: nothing before the gate (as before)
  lg, log = _early_run(early_from=10_000)
  assert not any(b == cs.SET_PLUS for f, s, b, a in log if f < 300)


def test_no_early_raise_under_a_red_light_a_stop_or_a_slow_car_ahead():
  for kw in (dict(red_light=True), dict(stop_active=True, stop_target=20 * MPH), dict(radar_lead=True, lead_v=5.0),
             dict(hold_restore=True)):
    lg, log = _early_run(frames=290, **kw)
    assert not any(b == cs.SET_PLUS for f, s, b, a in log), kw


def test_the_wrappers_early_conditions(tmp_path, monkeypatch):
  import pytest
  pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)
  from types import SimpleNamespace as N
  WI = _load("twi_round_p7", os.path.join(HERE, "test_wrapper_inputs.py"))
  c, sm, _ = WI._make(tmp_path, monkeypatch, limit_mode=0, map_mode=2)
  plan = N(mapOnRamp=True, mapRampToHighway=True, mapRampEndDistance=35.0, mapCurveStatus=0, mapCurveRadius=0.0,
           mapCurveDistance=0.0)
  c.map_control.hwy_ramp_left = 35.0
  cc = N(actuators=N(torque=0.5))
  c.turn_gate.lat = -1.8
  for _ in range(20):
    ok = c._merge_early(cc, plan)
  assert ok
  c.turn_gate.lat = -2.4                                            # still turning hard
  assert not c._merge_early(cc, plan)
  c.turn_gate.lat = -1.0
  c.map_control.hwy_ramp_left = 60.0                                # the ramp's end still 60 m away
  assert not c._merge_early(cc, plan)
  c.map_control.hwy_ramp_left = 30.0
  plan.mapCurveStatus, plan.mapCurveRadius, plan.mapCurveDistance = 2, 70.0, 40.0   # a tight ramp curve near
  assert not c._merge_early(cc, plan)
  plan.mapCurveStatus = 0
  c.lkas_blocked = True                                             # the stock LKAS blocked: only the TI steers
  assert not c._merge_early(cc, plan)
  c.lkas_blocked, c.ti_fault = False, True                          # the camera's fault
  assert not c._merge_early(cc, plan)
  c.ti_fault = False
  sat = N(actuators=N(torque=1.0))
  for _ in range(200):
    ok = c._merge_early(sat, plan)
  assert not ok                                                     # the steering saturated: no reserve
  for _ in range(200):
    ok = c._merge_early(cc, plan)
  assert ok


# ---- review of round P, B2: a false exit with his signal near the split (he stays on the highway) ----

def _false_exit(shape=False, untrusted=True, stay_from=200.0):
  exit_x, curve_x = 2000.0, 2300.0
  base = TE.exit_fields(exit_x, curve_x, r=60.0, ann=900.0, clear=10.0, stay=True, shape=shape, stay_from=stay_from)
  fields = (lambda x: dataclasses.replace(base(x), gps_trust=2) if x > exit_x else base(x)) if untrusted else base
  sig = lambda x: 1 if exit_x - 150 <= x < exit_x - 60 else 0  # noqa: E731
  lane = lambda x: 1 if exit_x - 300 <= x < exit_x + 30 else None  # noqa: E731
  r = TE.drive_exit(fields, 11000, 75, signal=sig, lane=lane)
  past = [(v, s, w, xx) for v, s, w, xx in zip(r["vs"], r["sets"], r["why"], r["xs"], strict=True) if xx >= exit_x]
  return r, exit_x, past


def test_review_b2_a_false_exit_without_a_link_answer_is_firm_for_tent_max_at_most():
  r, exit_x, past = _false_exit()
  firm_x = [xx for v, s, w, xx in past if w == "exit signalled"]
  v_split = TE._at(r, exit_x)
  assert firm_x and (max(firm_x) - exit_x) / (v_split * MPH) <= CTL.TENT_MAX + 0.5, max(firm_x) - exit_x
  assert min(v for v, *_ in past) >= 50.0, min(v for v, *_ in past)          # (was 38.2; baseline 53.5)
  assert r["sets"][-1] >= 54 - 1e-6, r["sets"][-1]                           # the adopted set: not under baseline's 54
  r, exit_x, past = _false_exit(shape=True)
  assert min(v for v, *_ in past) >= 50.0 and abs(r["sets"][-1] - 75) < 1e-6  # (was 40.3)


def test_review_s2_no_hold_with_the_car_faster_than_a_goal_under_its_speed():
  # after a turn the street's 40 bounds his old 75; RES with the set 20 at 45 mph (off the gas): taps only (a hold's first
  # frame jumping to 45 would land above the 40)
  extra = lambda f, car: dict(road_changed=True, engage_goal=40 * MPH)  # noqa: E731
  car, lg, log = _res_run(20, 45, 75, frames=900, extra=extra, stop_episode=True)
  assert lg.engage is None or abs(lg.engage["goal"] / MPH - 40) < 1e-6
  assert not any(b == cs.SET_PLUS and a == "hold" for f, s, v, b, a in log if f >= 300 and v > 41)
  assert max(s for _, s, *_ in log) <= 40 + 1e-6


def test_review_s3_the_floor_is_continuous_and_never_raises_the_set():
  keep = cs.LKAS_KEEP_SPEED[0]

  def run(csc_mph, keep_mph, set0=45, frames=1500):
    lg = cs.CurveSlowdownLogic(False, 10)
    lg.update(_inp(set_speed=set0 * MPH, v_ego=set0 * MPH))
    lg.anchor = 50 * MPH
    set_mph, ups = float(set0), 0
    for f in range(frames):
      b, _ = lg.update(_inp(set_speed=set_mph * MPH, v_ego=max(set_mph - 1, 20) * MPH, csc_active=True, csc_speed=csc_mph * MPH,
                            curve_track=True, curve_raise=True, curve_hold=True, curve_floor=20 * MPH, curve_budget=60,
                            lkas_keep=keep_mph * MPH, btn_counter=(f // 10) % 16))
      if b == cs.SET_MINUS and (lg.last_action != "hold" or f % 10 == 0):
        set_mph -= 1
      elif b == cs.SET_PLUS:
        ups += 1
        set_mph += 1
    return set_mph, ups
  # a 27.9 mph curve (its own target; the follower asks 21.9): floored at its own speed, not 30, not re-planned to 21.6
  s, _ = run(21.9, min(keep / MPH, 27.9))
  assert 27 <= s <= 28 + 1e-6, s
  # a 30.4 mph curve asking 24.4: floored at 30
  s, _ = run(24.4, min(keep / MPH, 30.4))
  assert 30 - 1e-6 <= s <= 30 + 1e-6, s
  # the set already under the floor (26, the floor 30): never a SET+ for the floor
  s, ups = run(24.4, 30.0, set0=26)
  assert ups == 0 and s <= 26 + 1e-6, (s, ups)


def test_review_s1_a_mazda_without_a_ti_has_no_camera_fault_notice_or_events():
  from opendbc.car.mazda.values import MazdaSafetyFlags
  c, ps, packer, Bus, _ = _carstate()
  c.CP.flags = int(MazdaSafetyFlags.GEN1)                            # (no Torque Interceptor)
  t = 3_000_000_000
  a2, d2, _ = packer.make_can_msg("CAM_LKAS", 2, {"ERR_BIT_1": 1, "ERR_BIT_2": 1})
  ps[Bus.cam].update([(t, [(a2, d2, 2)])])
  c.update(ps, None)
  assert c.cam_err and c.cam_fault == 0


def test_review_m1_no_early_raise_while_the_wheel_winds_up():
  lg, log = _early_run(frames=290, merge_turn_pause=True)
  assert not any(b == cs.SET_PLUS for f, s, b, a in log)
