"""Offline tests for the Mazda curve slowdown decision logic (no car, no openpilot runtime needed).

The simulated car models the stock MRCC: a SET-/SET+ press changes the dash set speed one step
after `lag` frames, and the true speed follows (set speed - dash offset). The simulated planner
mimics FrogPilot's CurveSpeedController.update_target and csc_active = target < v_cruise, where
v_cruise is the set speed we are changing.
"""
import importlib.util
import os

_spec = importlib.util.spec_from_file_location(
  "curve_slowdown", os.path.join(os.path.dirname(__file__), "..", "curve_slowdown.py"))
cs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cs)

MPH = cs.MS_PER_MPH
KPH = cs.MS_PER_KPH
DT = 0.01


class Car:
  def __init__(self, set_mph, dash_offset_mph=2.0, lag=15, accel=1.0, ignore=False, step=MPH):
    self.set_speed = set_mph * MPH
    self.dash_offset = dash_offset_mph * MPH
    self.v_ego = self.set_speed - self.dash_offset
    self.lag, self.accel, self.ignore, self.step = lag, accel, ignore, step
    self.pending, self.frame = [], 0

  def press(self, button, who="op"):
    if self.ignore and who == "op":
      return
    d = -self.step if button == cs.SET_MINUS else self.step
    self.pending.append((self.frame + self.lag, d))

  def tick(self):
    self.frame += 1
    for f, d in [p for p in self.pending if p[0] <= self.frame]:
      self.set_speed += d
    self.pending = [p for p in self.pending if p[0] > self.frame]
    tgt = self.set_speed - self.dash_offset
    self.v_ego += max(-self.accel * DT, min(self.accel * DT, tgt - self.v_ego))


class CSC:
  """Mimics FrogPilot's curve speed target (20 Hz) and its active flag."""
  def __init__(self):
    self.target, self.active, self.speed = None, False, 0.0

  def update(self, curve_speed, v_cruise, v_ego, time_to_curve=3.0):
    if curve_speed is None:
      self.target, self.speed, self.active = None, v_cruise, False
      return
    if self.target is None:
      self.target = max(v_ego, curve_speed)
    if curve_speed < self.target:
      decel = max(v_ego - curve_speed, 0) / max(time_to_curve, 1)
      self.target = max(min(self.target, v_ego) - decel * 0.05, curve_speed)
    else:
      self.target = min(self.target + 1.2 * 0.05, curve_speed)
      if self.target >= v_cruise:
        self.target = curve_speed
    self.speed = self.target
    self.active = self.target < v_cruise


def run(car, curve_fn, frames, events=None, plan_lag=10, metric=False, max_drop=5):
  logic = cs.CurveSlowdownLogic(metric, max_drop)
  csc = CSC()
  st = dict(enabled=True, cruise=True, gas=False, brake=False, plan_alive=True)
  hist, presses = [], []
  start = car.set_speed
  min_set = max_set = car.set_speed
  for f in range(frames):
    if events and f in events:
      events[f](st, car)
    if f % 5 == 0:
      csc.update(curve_fn(f), car.set_speed, car.v_ego)
    hist.append((csc.active, csc.speed))
    pa, ps = hist[max(0, len(hist) - 1 - plan_lag)]
    inp = cs.SlowdownInputs(enabled=st["enabled"], cruise_enabled=st["cruise"], set_speed=car.set_speed,
                            v_ego=car.v_ego, gas_pressed=st["gas"], brake_pressed=st["brake"], standstill=False,
                            plan_alive=st["plan_alive"], csc_active=pa, csc_speed=ps, other_button_active=False)
    btn, off = logic.update(inp)
    if btn is not None:
      assert off in (0, 1)
      presses.append((f, btn))
      car.press(btn)
    car.tick()
    min_set, max_set = min(min_set, car.set_speed), max(max_set, car.set_speed)
  return dict(presses=presses, min=min_set / MPH, max=max_set / MPH, final=car.set_speed / MPH,
              start=start / MPH, logic=logic)


def between(a, b, mph):
  return lambda f: mph * MPH if a <= f < b else None


def near(x, y):
  return abs(x - y) < 0.01


def minus_count(r):
  return sum(1 for _, b in r["presses"] if b == cs.SET_MINUS)


# ---------------- basic behaviour ----------------

def test_gentle_curve_drops_then_restores():
  r = run(Car(65), between(100, 1500, 61), 4000)
  assert 60 - 1e-6 <= r["min"] <= 63, r["min"]
  assert near(r["final"], 65), r["final"]
  assert near(r["max"], 65)


def test_sharp_curve_capped_at_5_mph():
  r = run(Car(65), between(100, 1500, 45), 4000)
  assert near(r["min"], 60), r["min"]
  assert near(r["final"], 65)


def test_never_below_25_mph_and_restores_at_low_speed():
  # dash 2 mph high: lowered to 25 dash = ~23 true; must still restore to 29
  r = run(Car(29), between(100, 1500, 15), 4000)
  assert r["min"] >= 25 - 1e-6, r["min"]
  assert near(r["final"], 29), r["final"]


def test_does_not_start_below_25_mph():
  r = run(Car(25, dash_offset_mph=2.0), between(100, 1500, 15), 2500)
  assert r["presses"] == []


def test_tiny_curve_needs_full_step():
  r = run(Car(65, dash_offset_mph=0.0), between(100, 1500, 64.5), 3000)
  assert r["presses"] == []


def test_dash_offset_zero():
  r = run(Car(65, dash_offset_mph=0.0), between(100, 1500, 61), 4000)
  assert r["min"] >= 60 - 1e-6 and near(r["final"], 65)


def test_metric_step_and_cap():
  car = Car(100 / 1.609344, step=KPH, dash_offset_mph=1.0)
  r = run(car, between(100, 1500, 55), 4000, metric=True)
  drop_kph = (r["start"] - r["min"]) * 1.609344
  assert drop_kph <= 8 + 0.01, drop_kph
  assert abs(r["final"] - r["start"]) < 0.01


def test_button_spacing_at_least_0_2_s():
  frames = [f for f, _ in run(Car(65), between(100, 1500, 45), 4000)["presses"]]
  assert all(b - a > cs.BUTTON_GAP_FRAMES for a, b in zip(frames, frames[1:]))


def test_no_ping_pong_inside_a_curve():
  # Targets at and just above step boundaries used to flip FrogPilot's flag and ping-pong.
  for curve in (61.0, 61.05, 61.2, 61.3, 62.0, 62.02):
    for offset in (0.0, 2.0):
      r = run(Car(65, dash_offset_mph=offset), between(100, 2000, curve), 5000)
      plus_in_curve = [f for f, b in r["presses"] if b == cs.SET_PLUS and f < 2000]
      assert plus_in_curve == [], (curve, offset, r["presses"])
      assert len(r["presses"]) <= 10, (curve, offset, len(r["presses"]))
      assert near(r["final"], 65)


def test_gradual_curve_exit_restores():
  rising = lambda f: (55 * MPH if 100 <= f < 900 else (55 * MPH + (f - 900) * 0.01 if 900 <= f < 1700 else None))
  r = run(Car(65), rising, 5000)
  assert near(r["final"], 65) and r["min"] >= 60 - 1e-6


def test_two_curves_back_to_back():
  curve = lambda f: 61 * MPH if 100 <= f < 700 else (58 * MPH if 900 <= f < 1600 else None)
  r = run(Car(65), curve, 5000)
  assert r["min"] >= 60 - 1e-6 and near(r["final"], 65) and near(r["max"], 65)


# ---------------- hard invariants across car response delays ----------------

def test_invariants_for_any_car_delay():
  for lag in (5, 15, 30, 45, 52, 60, 75, 95, 110, 140, 160, 200, 250, 320):
    for curve in (45, 58, 61, 63.5):
      r = run(Car(65, lag=lag), between(100, 1500, curve), 6000)
      assert r["max"] <= 65 + 1e-6, ("above driver's speed", lag, curve, r["max"])
      assert r["min"] >= 60 - 1e-6, ("dropped more than 5 mph", lag, curve, r["min"])
      if lag <= 300:
        assert near(r["final"], 65), ("not restored", lag, curve, r["final"])


# ---------------- driver and system interactions ----------------

def _first_press_frame(curve=61):
  return run(Car(65), between(100, 1500, curve), 3000)["presses"][0][0]


def test_driver_set_minus_during_lowering_is_respected():
  first = _first_press_frame()
  r = run(Car(65), between(100, 1500, 61), 4000,
          events={first + 40: lambda s, c: c.press(cs.SET_MINUS, who="driver")})
  assert near(r["final"], 64), r["final"]             # driver's one-step reduction kept, ours restored


def test_driver_hold_minus_5_is_respected():
  first = _first_press_frame()
  def hold(s, c):
    c.set_speed -= 5 * MPH
  r = run(Car(65), between(100, 1500, 61), 4000, events={first + 30: hold})
  assert r["final"] <= 60 + 0.01, r["final"]


def test_driver_set_plus_mid_curve_we_stop():
  r = run(Car(65), between(100, 1500, 61), 5000, events={800: lambda s, c: c.press(cs.SET_PLUS, who="driver")})
  assert [f for f, b in r["presses"] if b == cs.SET_MINUS and f >= 800] == [], r["presses"]
  assert r["max"] <= 66 + 1e-6
  assert near(r["final"], 66), r["final"]             # driver's +1 respected, our drop restored


def test_disengage_mid_curve_no_presses_while_off():
  def off(s, c):
    s["enabled"] = False
  r = run(Car(65), between(100, 1500, 61), 4000, events={300: off})
  assert all(f < 300 for f, _ in r["presses"])


def test_brake_during_restore_then_resume_restores():
  base = run(Car(65), between(100, 1000, 61), 4000)
  plus = [f for f, b in base["presses"] if b == cs.SET_PLUS]
  def brake(s, c):
    s["brake"], s["cruise"] = True, False
  def resume(s, c):
    s["brake"], s["cruise"] = False, True
  r = run(Car(65), between(100, 1000, 61), 6000, events={plus[0] + 5: brake, plus[0] + 300: resume})
  assert near(r["final"], 65), r["final"]
  assert all(not (plus[0] + 5 <= f < plus[0] + 300) for f, _ in r["presses"])


def test_driver_sets_new_speed_while_cruise_off_is_respected():
  def brake(s, c):
    s["brake"], s["cruise"] = True, False
  def set_new(s, c):
    s["brake"], s["cruise"] = False, True
    c.set_speed = 58 * MPH          # driver pressed SET at a new speed
  r = run(Car(65), between(100, 1500, 61), 5000, events={500: brake, 800: set_new})
  assert near(r["final"], 58), r["final"]


def test_gas_pauses_presses_then_restores():
  def gas_on(s, c):
    s["gas"] = True
  def gas_off(s, c):
    s["gas"] = False
  r = run(Car(65), between(100, 1500, 61), 4000, events={90: gas_on, 600: gas_off})
  assert all(not (90 <= f < 600) for f, _ in r["presses"])
  assert near(r["final"], 65)


def test_plan_hiccup_mid_curve_still_restores():
  def dead(s, c):
    s["plan_alive"] = False
  def back(s, c):
    s["plan_alive"] = True
  r = run(Car(65), between(100, 1500, 61), 4000, events={700: dead, 760: back})
  assert near(r["final"], 65), r["final"]


def test_long_pause_forgets_and_leaves_speed_alone():
  def off(s, c):
    s["enabled"] = False
  r = run(Car(65), between(100, 1500, 61), 8000, events={700: off})
  assert all(f < 700 for f, _ in r["presses"])


def test_car_ignores_buttons_we_give_up():
  r = run(Car(65, ignore=True), between(100, 9000, 61), 12000)
  assert len(r["presses"]) <= cs.NO_EFFECT_LIMIT * cs.NO_EFFECT_TRIPS, len(r["presses"])
  assert near(r["final"], 65)


def test_flicker_short_does_not_spam():
  flick = lambda f: 61 * MPH if (100 <= f < 1600 and (f // 80) % 2 == 0) else None
  r = run(Car(65), flick, 5000)
  assert near(r["final"], 65) and len(r["presses"]) <= 12, len(r["presses"])


def test_flicker_long_is_bounded():
  flick = lambda f: 61 * MPH if (100 <= f < 2100 and (f // 200) % 2 == 0) else None
  r = run(Car(65), flick, 6000)
  assert near(r["final"], 65)
  assert len(r["presses"]) <= 40, len(r["presses"])   # ~10 separate 2 s curves; bounded rate
  assert r["max"] <= 65 + 1e-6 and r["min"] >= 60 - 1e-6


# ---------------- review round 2 (driver speed must never drift) ----------------

def test_repeated_curves_never_drift():
  # 7.5 s curves every 11 s for 2 minutes
  curve = lambda f: 50 * MPH if (f % 1100) < 750 and f < 12000 else None
  r = run(Car(65), curve, 16000)
  assert r["min"] >= 60 - 1e-6, r["min"]
  assert r["max"] <= 65 + 1e-6, r["max"]
  assert near(r["final"], 65), r["final"]


def test_flicker_3_5_s_never_drifts():
  flick = lambda f: 61 * MPH if (f < 6000 and (f // 350) % 2 == 0) else None
  r = run(Car(65), flick, 10000)
  assert r["min"] >= 60 - 1e-6 and r["max"] <= 65 + 1e-6
  assert near(r["final"], 65), r["final"]


def test_random_curves_lags_never_break_limits():
  import random
  rng = random.Random(1234)
  for _ in range(60):
    set_mph = rng.choice([35, 45, 55, 65, 75])
    lag = rng.choice([5, 15, 40, 80, 150, 250])
    segs, f = [], 100
    while f < 9000:
      length = rng.randint(100, 900)
      segs.append((f, f + length, rng.uniform(set_mph - 15, set_mph + 2)))
      f += length + rng.randint(50, 900)
    def curve(fr, segs=segs):
      for a, b, v in segs:
        if a <= fr < b:
          return v * MPH
      return None
    r = run(Car(set_mph, lag=lag, dash_offset_mph=rng.choice([0, 1, 2])), curve, 14000)
    floor = max(set_mph - 5, 25)
    assert r["max"] <= set_mph + 1e-6, (set_mph, lag, r["max"])
    assert r["min"] >= floor - 1e-6, (set_mph, lag, r["min"])
    assert near(r["final"], set_mph), (set_mph, lag, r["final"])


def test_gas_release_does_not_restore_mid_curve():
  def gas_on(s, c):
    s["gas"] = True
  def gas_off(s, c):
    s["gas"] = False
  r = run(Car(65), between(100, 3000, 58), 6000, events={600: gas_on, 900: gas_off})
  assert [f for f, b in r["presses"] if b == cs.SET_PLUS and f < 3000] == [], r["presses"]
  assert near(r["final"], 65)


# ---------------- review round 3 ----------------

def test_driver_tap_during_lowering_next_curve_respects_limits():
  # N1: driver taps SET- once while we lower; later curves must stay within (64 - 5)
  for lag in (15, 60):
    first = run(Car(65, lag=lag), between(100, 1500, 50), 3000)["presses"][0][0]
    curve = lambda f: 45 * MPH if (100 <= f < 1500 or 3000 <= f < 4400) else None
    r = run(Car(65, lag=lag), curve, 8000, events={first + 5: lambda s, c: c.press(cs.SET_MINUS, who="driver")})
    assert r["min"] >= 59 - 1e-6, (lag, r["min"])
    assert near(r["final"], 64), (lag, r["final"])


def test_step_mismatch_disables():
  # N2: device set to km/h but the dash steps in mph
  r = run(Car(65), between(100, 3000, 45), 6000, metric=True)
  assert r["logic"].disabled
  assert len(r["presses"]) <= 2, r["presses"]


def test_long_cruise_off_then_resume_restores():
  # N3: traffic stop with cruise off for 2 minutes, then RES at the lowered speed
  def off(s, c):
    s["cruise"], s["brake"] = False, True
  def resume(s, c):
    s["cruise"], s["brake"] = True, False
  r = run(Car(45), between(100, 1500, 30), 20000, events={1000: off, 13000: resume})
  assert near(r["final"], 45), r["final"]
  assert r["min"] >= 40 - 1e-6


# ---------------- review round 4 ----------------

def test_cancel_while_press_in_flight_then_resume_later():
  # P1: our SET- is in flight when the driver cancels; cruise off for 10 s; then RES.
  for lag in (15, 60, 120):
    first = run(Car(65, lag=lag), between(100, 1500, 50), 3000)["presses"][0][0]
    def off(s, c):
      s["cruise"], s["brake"] = False, True
    def resume(s, c):
      s["cruise"], s["brake"] = True, False
    curve = lambda f: 45 * MPH if (100 <= f < 1500 or 3500 <= f < 4900) else None
    r = run(Car(65, lag=lag), curve, 9000, events={first + 2: off, first + 1000: resume})
    assert r["min"] >= 60 - 1e-6, (lag, r["min"])
    assert near(r["final"], 65), (lag, r["final"])


def test_car_stepping_twice_per_press_disables():
  # P3: a car whose set speed moves 2 steps per press
  r = run(Car(65, step=2 * MPH), between(100, 3000, 45), 6000)
  assert r["logic"].disabled
  assert len(r["presses"]) <= 1, r["presses"]


# ---------------- v2: adjustable drop, restore while exiting ----------------

def test_max_drop_settings():
  for max_drop, expect in ((5, 60), (10, 55), (15, 50)):
    r = run(Car(65), between(100, 2500, 30), 6000, max_drop=max_drop)
    assert near(r["min"], expect), (max_drop, r["min"])
    assert near(r["final"], 65) and near(r["max"], 65)


def test_45_road_with_30_curve():
  # the yellow-sign case: 45 mph road, curve needs ~30 true; dash reads 2 mph high
  r = run(Car(45), between(100, 2500, 30), 6000, max_drop=15)
  assert r["min"] <= 31 + 1e-6 and r["min"] >= 30 - 1e-6, r["min"]   # target 30 true + margin
  assert near(r["final"], 45)
  r = run(Car(45), between(100, 2500, 30), 6000, max_drop=10)
  assert near(r["min"], 35), r["min"]


def test_drop_only_as_much_as_curve_needs():
  r = run(Car(65, dash_offset_mph=0.0), between(100, 2500, 62.5), 6000, max_drop=15)
  assert r["min"] >= 62 - 1e-6, r["min"]


def test_restore_starts_quickly_after_curve():
  r = run(Car(65), between(100, 1500, 58), 5000, max_drop=10)
  plus = [f for f, b in r["presses"] if b == cs.SET_PLUS]
  assert plus and plus[0] - 1500 <= 150, plus[:3]      # within ~1.5 s of the curve ending
  assert near(r["final"], 65)


def test_rising_exit_raises_during_exit_without_ping_pong():
  # curve target rises gradually from 50 to 70 over 8 s while exiting
  exit_curve = lambda f: (50 * MPH if 100 <= f < 1000 else
                          (50 * MPH + (f - 1000) / 800 * 20 * MPH if 1000 <= f < 1800 else None))
  r = run(Car(65), exit_curve, 6000, max_drop=15)
  plus = [f for f, b in r["presses"] if b == cs.SET_PLUS]
  assert plus and plus[0] < 1800, plus[:3]            # starts raising during the exit
  seq = "".join("M" if b == cs.SET_MINUS else "P" for _, b in r["presses"])
  assert seq.count("PM") <= 1, seq                    # no ping-pong
  assert near(r["final"], 65) and near(r["max"], 65)


def test_random_drives_all_drop_settings():
  import random
  rng = random.Random(99)
  for _ in range(45):
    max_drop = rng.choice([5, 10, 15])
    set_mph = rng.choice([30, 45, 55, 65, 80])
    lag = rng.choice([5, 15, 40, 80, 150])
    segs, f = [], 100
    while f < 9000:
      length = rng.randint(100, 900)
      segs.append((f, f + length, rng.uniform(set_mph - 25, set_mph + 2)))
      f += length + rng.randint(50, 900)
    def curve(fr, segs=segs):
      for a, b, v in segs:
        if a <= fr < b:
          return v * MPH
      return None
    r = run(Car(set_mph, lag=lag, dash_offset_mph=rng.choice([0, 1, 2])), curve, 14000, max_drop=max_drop)
    floor = min(set_mph, max(set_mph - max_drop, 25))
    assert r["max"] <= set_mph + 1e-6, (max_drop, set_mph, lag, r["max"])
    assert r["min"] >= floor - 1e-6, (max_drop, set_mph, lag, r["min"])
    assert near(r["final"], set_mph), (max_drop, set_mph, lag, r["final"])


if __name__ == "__main__":
  import sys
  failed = 0
  for name, fn in sorted(globals().items()):
    if name.startswith("test_"):
      try:
        fn()
        print("PASS", name)
      except Exception as e:  # noqa: BLE001
        failed += 1
        print("FAIL", name, e)
  print(f"{failed} failed")
  sys.exit(1 if failed else 0)
