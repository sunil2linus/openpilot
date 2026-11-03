"""Offline tests for the per-car profile (car_profile.py): the file, what a drive starts with, the hold overrun
across drives, the cruise decel learner, and that the planner reads the same decel as the car side."""
import importlib.util
import json
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(HERE, ".."))   # curve_target for map_curve_control's offline import


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


CP = _load("car_profile", os.path.join(HERE, "..", "car_profile.py"))
CT = _load("curve_target", os.path.join(HERE, "..", "curve_target.py"))
CTL = _load("map_curve_control", os.path.join(HERE, "..", "map_curve_control.py"))
MC = _load("map_curves", os.path.join(ROOT, "starpilot", "controls", "lib", "map_curves.py"))
T = _load("tcs", os.path.join(HERE, "test_curve_slowdown.py"))
S = _load("tscs", os.path.join(HERE, "test_stopped_car_slowdown.py"))
cs = T.cs
MPH = CP.MPH


def profile(tmp_path, fp="MAZDA_CX5", ti=True, name="p.json"):
  return CP.CarProfile(path=str(tmp_path / name), fingerprint=fp, ti=ti)


# ---------------- the file ----------------

def test_round_trip(tmp_path):
  p = profile(tmp_path)
  assert not p.loaded and p.discarded is None and p.latencies == [] and p.decel_use is None
  assert p.hold_start(1) == (1, False)
  p.note_drive()
  p.set_latencies([12, 9, 11, 10])
  p.add_hold(1)
  p.add_hold(2, driver=True)
  for _ in range(12):
    p.add_decel_episode(0.22)
  assert p.dirty
  p.save(in_thread=False)
  assert p.writes == 1 and not p.dirty and os.path.exists(p.path) and not os.path.exists(p.path + ".tmp")
  q = profile(tmp_path)
  assert q.loaded and q.discarded is None
  assert q.latencies == [12, 9, 11, 10] and q.data["latency_samples"] == 4 and q.data["drives"] == 1
  assert q.data["hold"] == {**q.data["hold"], "worst_after_release": 2, "holds": 2, "driver_holds": 1}
  assert q.hold_start(1) == (2, False)                 # 2 holds: not measured yet, but never less margin than seen
  assert q.decel_use == 0.22 and q.decel_episodes == 12 and q.decel() == 0.22
  with open(p.path) as f:
    d = json.load(f)
  assert d["version"] == CP.PROFILE_VERSION and d["fingerprint"] == "MAZDA_CX5" and d["ti"] is True
  assert d["created"] <= d["updated"]
  assert "12/" not in q.summary() and "0.22" in q.summary()


def test_other_car_or_ti_change_discards_the_profile(tmp_path):
  p = profile(tmp_path)
  p.set_latencies([10, 10, 10])
  p.add_hold(2)
  p.save(in_thread=False)
  other = profile(tmp_path, fp="MAZDA_3")
  assert not other.loaded and other.discarded.startswith("other car") and other.latencies == [] and other.hold_start(1) == (1, False)
  assert other.dirty                                       # the fresh profile replaces the file
  other.save(in_thread=False)
  assert profile(tmp_path, fp="MAZDA_3").loaded
  no_ti = profile(tmp_path, fp="MAZDA_3", ti=False)
  assert not no_ti.loaded and no_ti.discarded == "torque interceptor changed"
  with open(str(tmp_path / "p.json"), "w") as f:
    f.write("{not json")
  broken = profile(tmp_path, fp="MAZDA_3")
  assert not broken.loaded and broken.discarded is None     # unreadable: start fresh, never raise
  with open(str(tmp_path / "p.json"), "w") as f:
    json.dump({"version": CP.PROFILE_VERSION, "fingerprint": "MAZDA_3", "ti": True, "latencies": "x", "hold": 5}, f)
  odd = profile(tmp_path, fp="MAZDA_3")
  assert not odd.loaded and odd.discarded == "unreadable values"


def test_reset_deletes_the_file(tmp_path):
  p = profile(tmp_path)
  p.set_latencies([10])
  p.add_hold(3)
  p.save(in_thread=False)
  assert os.path.exists(p.path)
  p.delete()
  assert not os.path.exists(p.path) and p.latencies == [] and p.hold_start(1) == (1, False) and not p.dirty
  assert not profile(tmp_path).loaded


def wait_written(p):
  import time
  for _ in range(300):
    if not p.saving:
      return
    time.sleep(0.01)
  raise AssertionError("writer did not finish")


def test_saves_in_the_background_when_due(tmp_path):
  clock = [0.0]
  p = CP.CarProfile(path=str(tmp_path / "p.json"), fingerprint="X", ti=False, clock=lambda: clock[0])
  assert not p.save_if_due()                               # nothing changed
  p.set_latencies([10])
  assert p.save_if_due()                                   # first change: at once
  wait_written(p)
  assert p.writes == 1
  p.set_latencies([10, 11])
  clock[0] = 10.0
  assert not p.save_if_due()                               # too soon
  assert p.save_if_due(force=True)                         # cruise went off: now
  wait_written(p)
  assert p.writes == 2
  clock[0] = 10.0 + CP.SAVE_INTERVAL + 1
  p.set_latencies([10, 11, 12])
  assert p.save_if_due()                                   # the interval passed
  wait_written(p)
  assert p.writes == 3 and profile(tmp_path, fp="X", ti=False).latencies == [10, 11, 12]
  # a change made while the writer is busy is written right after, not lost
  p.set_latencies([1])
  p.save()
  p.set_latencies([1, 2])
  wait_written(p)
  assert not p.dirty and profile(tmp_path, fp="X", ti=False).latencies == [1, 2]


# ---------------- drives: what the logic starts with ----------------

def start_drive(p, logic):
  """What CurveSlowdown._load_profile does with the logic at the start of a drive."""
  logic.latencies = p.latencies
  logic.hold_overrun, logic.hold_measured = p.hold_start(cs.HOLD_OVERRUN_DEFAULT)
  p.note_drive()
  return logic


def end_drive(p, logic, seen=0):
  """What CurveSlowdown._learn_profile feeds back (here once, at the end)."""
  p.set_latencies(logic.latencies)
  while seen < logic.holds_learned:
    seen += 1
    after, who = logic.last_hold_learned
    p.add_hold(after, driver=(who == "driver"))
  p.save(in_thread=False)
  return seen


def test_saved_answer_times_are_used_from_the_first_press_of_the_next_drive(tmp_path):
  p = profile(tmp_path)
  r = T.run(T.Car(45, lag=12), T.between(100, 1500, 35), 3000, setup=lambda lg: start_drive(p, lg))
  lg = r["logic"]
  assert lg.latencies and all(10 <= x <= 14 for x in lg.latencies), lg.latencies
  end_drive(p, lg)
  # next drive: the timeout logic is tuned at once (window = LATE_FACTOR x answer time, not the 3.3 s default)
  q = profile(tmp_path)
  fresh = cs.CurveSlowdownLogic(False, 10)
  assert fresh.latency is None and fresh._in_flight_window() == cs._frames(cs.PENDING_TIMEOUT + cs.LATE_WINDOW)
  start_drive(q, fresh)
  assert fresh.latencies == lg.latencies[-CP.LATENCY_KEEP:] and fresh.latency == lg.latency
  assert fresh._in_flight_window() == max(cs._frames(cs.MIN_IN_FLIGHT_WINDOW), cs.LATE_FACTOR * lg.latency)
  r2 = T.run(T.Car(45, lag=12), T.between(100, 1500, 35), 3000, setup=lambda lg: start_drive(q, lg))
  assert T.near(r2["final"], 45) and r2["min"] >= 35 - 1e-6


def holds_drive(p, hold_latency, frames=2500):
  """One drive with a stopped car approach that uses SET- holds on a car that lands `hold_latency` s worth of quanta
  after a release (0.0: none, 0.6: one, 1.2: two)."""
  lead = T.StoppedLead(60)
  car = T.Car(45, dash_offset_mph=0.0, hold_latency=hold_latency)
  r = T.run(car, S.NO_CURVE, frames, lead=lead, stop_mode=2, max_drop=10, setup=lambda lg: start_drive(p, lg))
  return r["logic"]


def test_hold_overrun_is_kept_across_drives_and_never_gives_less_margin(tmp_path):
  for latency, quanta in ((0.0, 0), (0.6, 1), (1.2, 2)):
    path = tmp_path / f"q{quanta}"
    path.mkdir()
    worst_seen = 0
    for drive in range(6):
      p = profile(path)
      lg = holds_drive(p, latency)
      assert lg.hold_log and any(h["done"] for h in lg.hold_log), (quanta, drive)
      measured = [h for h in lg.hold_log if h["done"] and h["landed"] > lg._quantum_steps()]
      for h in measured:
        q = -(-h["landed_after_release"] // lg._quantum_steps())
        assert q <= quanta, (latency, h)                    # the simulated car never lands more than that
        worst_seen = max(worst_seen, q)
      # within the drive the margin is at least what was seen, and the SET- overrun starts at the profile's worst
      assert lg.hold_overrun >= max(1, worst_seen), (quanta, drive, lg.hold_overrun, worst_seen)
      end_drive(p, lg)
      q2 = profile(path)
      assert q2.data["hold"]["worst_after_release"] == worst_seen, (quanta, drive, q2.data["hold"])
      assert q2.data["hold"]["holds"] >= drive + 1
    q2 = profile(path)
    assert q2.holds >= CP.HOLD_MIN_MEASURED
    overrun, measured_start = q2.hold_start(cs.HOLD_OVERRUN_DEFAULT)
    assert measured_start and overrun == max(cs.HOLD_OVERRUN_DEFAULT, quanta), (quanta, overrun)
    lg = start_drive(q2, cs.CurveSlowdownLogic(False, 10))
    assert lg.hold_measured and lg.hold_overrun == max(1, quanta) and lg._up_overrun() == lg.hold_overrun + 1
    # a worse landing than ever seen raises it at once, and is remembered
    lg._finish_hold(cs.Press(cs.SET_MINUS, 0, reserve=20, remaining=0, landed=12, hold=True, release_frame=0,
                             landings=[(0, 1), (10, 5), (200, 5), (260, 1)]))
    assert lg.hold_overrun == 2 and lg.holds_learned == 1 and lg.last_hold_learned == (2, "ours")
    end_drive(q2, lg)
    assert profile(path).hold_start(cs.HOLD_OVERRUN_DEFAULT) == (2, True)


def test_the_drivers_own_hold_measures_the_car(tmp_path):
  """Test mode: nothing of ours is pressed; the driver holds SET- for 1.5 s and the car steps 5 mph at a time. What
  lands after the release is learned like one of our holds."""
  for latency, quanta in ((0.0, 0), (0.6, 1), (1.2, 2)):
    car = T.Car(55, dash_offset_mph=0.0, hold_latency=latency)
    events = {}

    def hold(st, c, f=None):
      c.driver_button, c.driver_button_until = cs.SET_MINUS, c.frame + 11
      c.hold_frame(cs.SET_MINUS)
    for f in range(500, 650, 10):
      events[f] = hold
    r = T.run(car, S.NO_CURVE, 1500, events=events)
    lg = r["logic"]
    assert r["presses"] == [] and r["holds"] == []            # nothing of ours
    assert r["min"] <= 46, r["min"]                           # the driver's hold stepped the car down (55 -> 54 -> 50 -> 45)
    assert lg.holds_learned == 1 and lg.last_hold_learned == (quanta, "driver"), (latency, lg.last_hold_learned)
    assert lg.hold_measured and lg.hold_overrun == max(1, quanta)
  # a single tap by the driver is not a hold
  car = T.Car(55, dash_offset_mph=0.0)
  r = T.run(car, S.NO_CURVE, 1000, events={500: lambda st, c: c.press(cs.SET_MINUS, who="driver")})
  assert r["logic"].holds_learned == 0 and not r["logic"].hold_measured


# ---------------- cruise decel learner ----------------

class CruiseCar:
  """A stock cruise: the true speed follows (set speed - dash offset) after a reaction time, slowing at `decel` and
  speeding up at `accel`; optional lead car, pedals and curve flag."""

  def __init__(self, set_mph, decel, accel=0.5, dash_mph=2.0, reaction=1.5, noise=0.0, seed=0):
    self.set_speed = set_mph * MPH
    self.dash = dash_mph * MPH
    self.v = self.set_speed - self.dash
    self.decel, self.accel, self.reaction, self.noise = decel, accel, reaction, noise
    self.react_left = 0.0
    self.rng = random.Random(seed)
    self.lead_d, self.brake, self.gas, self.csc = 0.0, False, False, False

  def set(self, mph):
    if abs(self.v - (self.set_speed - self.dash)) < 0.2:
      self.react_left = self.reaction        # the cruise takes a moment to react when it was cruising at the set speed
    self.set_speed = mph * MPH

  def tick(self, dt=0.01):
    self.react_left -= dt
    target = self.set_speed - self.dash
    if self.react_left <= 0:
      self.v += max(-self.decel * dt, min(self.accel * dt, target - self.v))
    return CP.DecelInputs(v_ego=self.v + self.rng.gauss(0, self.noise), set_speed=self.set_speed, cruise_enabled=True,
                          brake_pressed=self.brake, gas_pressed=self.gas, lead_status=self.lead_d > 0, lead_d=self.lead_d,
                          csc_active=self.csc)


def episode(learner, car, drop_mph=3, seconds=25.0, taps=False, during=None):
  """One slowdown: the set speed goes down drop_mph (at once, or 1 mph a second as our staircase does), the car
  follows; returns the decel the learner reported, if any."""
  out = []
  start_mph = round(car.set_speed / MPH)
  for _ in range(50):                       # cruising at the set speed first (the gap opens from there)
    learner.update(car.tick())
  for k in range(int(seconds / 0.01)):
    if taps:
      if k % 100 == 0 and start_mph - round(car.set_speed / MPH) < drop_mph:
        car.set(round(car.set_speed / MPH) - 1)
    elif k == 0:
      car.set(start_mph - drop_mph)
    if during is not None:
      during(k, car)
    d = learner.update(car.tick())
    if d is not None:
      out.append(d)
  # back up to the start speed (cruising again), so the next episode starts clean
  car.set(start_mph)
  for _ in range(int(20.0 / 0.01)):
    d = learner.update(car.tick())
    if d is not None:
      out.append(d)
  return out


def test_decel_learner_converges_on_0_2_and_0_35(tmp_path):
  for true in (0.2, 0.35):
    p = profile(tmp_path, name=f"d{true}.json")
    learner = CP.DecelLearner()
    car = CruiseCar(45, true, noise=0.05, seed=int(true * 100))
    for k in range(12):
      for d in episode(learner, car, drop_mph=4 if k % 2 else 5, taps=(k % 3 == 0)):
        p.add_decel_episode(d)
      if k < CP.DECEL_MIN_EPISODES - 1:
        assert p.decel_use is None, (true, k)            # today's constant until enough episodes
    assert p.decel_episodes >= CP.DECEL_MIN_EPISODES and p.decel_use is not None
    assert abs(p.decel_use - true) <= 0.03, (true, p.decel_use, p.data["decel"]["episodes"])
    assert learner.rejected == 0, learner.rejected
    assert abs(p.decel() - true) <= 0.03


def test_decel_learner_ignores_lead_brake_gas_and_curve_episodes():
  learner = CP.DecelLearner()

  def with_lead(k, car):
    car.lead_d = 50.0

  def far_lead(k, car):
    car.lead_d = 120.0

  def braking(k, car):
    car.brake = 200 <= k <= 400          # a brake tap early in the slowdown (the fit would be too short before it)

  def gas(k, car):
    car.gas = k >= 150

  def in_curve(k, car):
    car.csc = True
  car = CruiseCar(45, 0.25)
  assert episode(learner, car, during=with_lead) == []
  assert episode(learner, car, during=braking) == []
  assert episode(learner, car, during=gas) == []
  assert episode(learner, car, during=in_curve) == []
  assert learner.episodes == 0
  car.lead_d, car.brake, car.gas, car.csc = 0.0, False, False, False
  # a car far ahead does not count as a lead; a clean episode is learned
  got = episode(learner, car, during=far_lead)
  assert len(got) == 1 and abs(got[0] - 0.25) < 0.02, got
  # a big drop (a hold's 5 mph quanta, the gap opens past GAP_MAX) is not the staircase pace
  car2 = CruiseCar(55, 0.7)
  learner2 = CP.DecelLearner()
  got = episode(learner2, car2, drop_mph=15)
  assert got == [], got


def test_decel_is_clamped_to_the_safe_range(tmp_path):
  p = profile(tmp_path)
  for _ in range(12):
    p.add_decel_episode(0.9)
  assert p.decel_use == CP.DECEL_RANGE[1]
  q = profile(tmp_path, name="lo.json")
  for _ in range(12):
    q.add_decel_episode(0.05)
  assert q.decel_use == CP.DECEL_RANGE[0]
  assert CP.decel_from_episodes([0.2] * 9) is None
  assert CP.decel_from_episodes([0.1, 0.2, 0.3, 0.2, 0.25, 0.22, 0.21, 0.23, 0.9, 0.2]) == 0.215  # median: one bad pass does nothing


# ---------------- the planner and the car side agree ----------------

class FakeParams:
  def __init__(self, v):
    self.v = v

  def get(self, key, encoding=None):
    return self.v.get(key)

  def get_bool(self, key):
    return bool(self.v.get(key))


def test_planner_and_car_side_use_the_same_decel(tmp_path):
  assert CT.MAP_DECEL == MC.PLAN_DECEL == CP.DECEL_DEFAULT
  assert tuple(MC.PROFILE_DECEL_RANGE) == tuple(CP.DECEL_RANGE)
  assert MC.PROFILE_PATH == CP.PROFILE_PATH
  path = str(tmp_path / "p.json")
  # no profile yet: both use the constant
  assert MC.read_profile_decel(path) == CP.read_profile_decel(path) == CT.MAP_DECEL
  for episodes in ([0.2] * 12, [0.33] * 15, [0.9] * 12, [0.01] * 12, [0.2] * 5):
    p = CP.CarProfile(path=path, fingerprint="MAZDA_CX5", ti=True)
    for d in episodes:
      p.add_decel_episode(d)
    p.save(in_thread=False)
    # the car side (CurveSlowdown._load_profile -> MapCurveControl)
    car_side = CP.CarProfile(path=path, fingerprint="MAZDA_CX5", ti=True).decel(CT.MAP_DECEL)
    ctl = CTL.MapCurveControl(3.0, car_side)
    # the planner (MapCurvesPlanner._read_mode -> MapCurves.decel)
    pl = MC.MapCurvesPlanner(FakeParams({"MazdaMapCurves": "2"}), FakeParams({}))
    pl.profile_path = path
    pl._read_mode()
    assert ctl.decel == ctl.curve.decel == pl.decel == MC.read_profile_decel(path) == CP.read_profile_decel(path), episodes
    expected = CP.clamp_decel(sorted(episodes)[len(episodes) // 2]) if len(episodes) >= CP.DECEL_MIN_EPISODES else CT.MAP_DECEL
    assert abs(pl.decel - expected) < 1e-9, (episodes, pl.decel, expected)
    # and the plan itself: the same set speed for the same curve on both sides (reaction speed = the driver's speed)
    v = MC.plan_speed(150.0, 400.0, 0.0, reaction_speed=20.0, decel=pl.decel)
    assert abs(v - CT.map_curve_speed(150.0, 400.0, 2.2, 0.0, reaction_speed=20.0, decel=ctl.decel)) < 1e-9
    os.remove(path)
  # the planner object keeps the value it read for the drive and gives it to MapCurves
  p = CP.CarProfile(path=path, fingerprint="MAZDA_CX5", ti=True)
  for _ in range(12):
    p.add_decel_episode(0.3)
  p.save(in_thread=False)
  pl = MC.MapCurvesPlanner(FakeParams({"MazdaMapCurves": "2"}), FakeParams({}))
  pl.profile_path = path
  pl._read_mode()
  pl.mc = MC.MapCurves(MC.MapCurveStore(str(tmp_path / "nocurves"), background=False), decel=pl.decel)
  assert pl.mc.decel == 0.3
  pl._read_mode()
  assert pl.mc.decel == 0.3


def test_learned_decel_changes_the_plan_consistently():
  """A slower cruise (0.2) is planned earlier than the default; the car side and the planner move together."""
  lo = CTL.MapCurveControl(3.0, 0.2)
  hi = CTL.MapCurveControl(3.0, 0.4)
  f = CTL.MapFields(mode=2, status=1, distance=400.0, radius=120.0)
  a_lo, sp_lo = lo.curve.update(f.radius, f.distance, 25.0, 0.0, lo.decel)
  a_hi, sp_hi = hi.curve.update(f.radius, f.distance, 25.0, 0.0, hi.decel)
  assert sp_lo < sp_hi
  assert MC.plan_speed(120.0, 400.0, 0.0, 25.0, decel=0.2) < MC.plan_speed(120.0, 400.0, 0.0, 25.0, decel=0.4)
  assert abs(MC.plan_speed(120.0, 400.0, 0.0, 25.0, decel=0.2) - sp_lo) < 1e-9
