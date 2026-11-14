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


def test_res_with_nothing_lowered_but_the_set_far_under_the_car_raises_it_to_the_speed():
  # his own set 25 (nothing of ours anchored), RES at 40 mph: the set never left under the car's speed
  car = T.Car(25, dash_offset_mph=0.0, lag=10, accel_fn=_acc, cruise_gain=0.19)
  car.v_ego = 40 * MPH
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=car.set_speed, v_ego=car.v_ego))
  for f in range(800):
    cruise = not (50 <= f < 200)
    btn, _ = lg.update(_inp(cruise_enabled=cruise, set_speed=car.set_speed if cruise else 0.0, v_ego=car.v_ego,
                            stop_clear_ahead=True, stop_pulling_away=True, btn_counter=(f // 10) % 16))
    if btn is not None and cruise:
      car.hold_frame(btn) if lg.last_action == "hold" else car.press(btn)
    car.tick() if cruise else None
  assert car.set_speed / MPH >= 39 - 1e-6, car.set_speed / MPH
