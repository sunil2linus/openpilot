"""Round P (drives 72 / 73): the logged scenarios as tests."""
import importlib.util
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
