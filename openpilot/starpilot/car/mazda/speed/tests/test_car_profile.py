"""Per-car profile (car_profile.py), ported from the old fork's tests: the file, a change of car or TI, reset,
background saves, the cruise decel learner and its clamp. (The old drive tests that ran the old controller are
replaced by tests/test_actuator.py, which starts the new actuator from a saved profile.)"""
import json
import os
import random

from openpilot.starpilot.car.mazda.speed import car_profile as CP

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
