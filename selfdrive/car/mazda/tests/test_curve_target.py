"""Offline tests for the Mazda curve target (road curvature -> curve speed) and the curve learner."""
import importlib.util
import math
import os


def _load(name):
  spec = importlib.util.spec_from_file_location(name, os.path.join(os.path.dirname(__file__), "..", name + ".py"))
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


ct = _load("curve_target")
cl = _load("curve_learner")
MPH = ct.MS_PER_MPH


def feed(target, curvature_fn, frames, driver_mph=45, blinker_fn=lambda f: False):
  out = []
  for f in range(frames):
    out.append(target.update(curvature_fn(f), driver_mph * MPH, True, blinker_fn(f)))
  return out


# ---------------- curve target ----------------

def test_lateral_limit():
  assert ct.lateral_limit(True, 1.089) == 3.0                 # Torque Interceptor
  assert abs(ct.lateral_limit(False, 1.089) - 1.089) < 1e-9   # no TI: the car's own steering limit (FrogPilot's cap)
  assert ct.lateral_limit(False, 0.0) == 2.0
  assert ct.lateral_limit(False, 5.0) == 3.0
  assert ct.lateral_limit(False, 0.1) == 0.8


def test_30_mph_curve_with_ti():
  # radius ~115 m (the owner's 30 mph curve): 3.0 m/s^2 -> 41.5 mph, active after 1 s
  t = ct.CurveTarget(3.0)
  out = feed(t, lambda f: 1 / 115.0, 300)
  assert not any(a for a, _ in out[:99])
  assert out[100][0]
  assert abs(out[200][1] / MPH - math.sqrt(3.0 * 115) / MPH) < 0.01
  assert 41 < out[200][1] / MPH < 42


def test_gentle_bends_do_nothing():
  # 7:10 bend (radius ~480 m), 4:54 curve (~210 m), chevron curve (~150 m): none needs 2 mph below 45
  for radius in (480.0, 210.0, 155.0):
    t = ct.CurveTarget(3.0)
    assert not any(a for a, _ in feed(t, lambda f, r=radius: 1 / r, 400)), radius


def test_brief_spike_ignored():
  # a sharp curvature for 0.5 s inside a straight never activates (the curve must be seen for 1 s)
  t = ct.CurveTarget(3.0)
  out = feed(t, lambda f: 1 / 40.0 if 200 <= f < 250 else 1e-4, 600)
  assert not any(a for a, _ in out)


def test_gap_and_hysteresis():
  # driver 45 mph: a target 1.5 mph below does nothing; 2.5 below acts and stays until < 1 mph below
  k_for = lambda mph: 3.0 / (mph * MPH) ** 2
  t = ct.CurveTarget(3.0)
  assert not any(a for a, _ in feed(t, lambda f: k_for(43.5), 300))
  t = ct.CurveTarget(3.0)
  out = feed(t, lambda f: k_for(42.5) if f < 300 else (k_for(43.5) if f < 600 else k_for(44.5)), 900)
  assert out[250][0] and out[550][0]           # 1.5 below: still active (hysteresis)
  assert not out[899][0]                       # 0.5 below: over


def test_exit_is_quick():
  # the least demanding of the last second rises at once when the curve ends
  t = ct.CurveTarget(3.0)
  out = feed(t, lambda f: 1 / 100.0 if f < 300 else 1e-5, 320)
  assert out[299][0] and not out[301][0]


def test_blinker_and_dead_plan():
  t = ct.CurveTarget(3.0)
  assert not any(a for a, _ in feed(t, lambda f: 1 / 30.0, 300, blinker_fn=lambda f: True))
  t = ct.CurveTarget(3.0)
  feed(t, lambda f: 1 / 60.0, 200)
  assert t.active
  assert t.update(1 / 60.0, 45 * MPH, False, False) == (False, ct.NO_CURVE)
  assert not t.update(1 / 60.0, 45 * MPH, True, False)[0]      # needs a full second again


def test_without_ti_matches_frogpilot_limit():
  # no TI: 1.09 m/s^2, a 250 m curve wants ~37 mph (FrogPilot's CSC without the roll term)
  t = ct.CurveTarget(ct.lateral_limit(False, 1.089))
  out = feed(t, lambda f: 1 / 250.0, 200)
  assert out[150][0] and abs(out[150][1] / MPH - math.sqrt(1.089 * 250) / MPH) < 0.01


def test_cruise_off_never_active():
  t = ct.CurveTarget(3.0)
  assert not any(a for a, _ in feed(t, lambda f: 1 / 50.0, 300, driver_mph=0))


# ---------------- curve learner ----------------

def drive_curve(learner, lat, frames=300, speed_mph=40, **kw):
  """A curve: lead-in at 0.2, `frames` at `lat` (callable or number), then 1 s straight. Returns the records."""
  recs = []
  base = dict(v_ego=speed_mph * MPH, lat_active=True, steering_pressed=False, steer_cmd=0.3, cruise_enabled=True,
              gas_pressed=False, brake_pressed=False, blinker=False, our_offset=0, driver_set_minus=False)
  for f in range(50 + frames + 100):
    inside = 50 <= f < 50 + frames
    a = (lat(f - 50) if callable(lat) else lat) if inside else 0.2
    d = dict(base, lat_accel=a)
    for k, v in kw.items():
      d[k] = v(f - 50) if callable(v) else v
      if not inside and k in ("steering_pressed", "brake_pressed", "gas_pressed", "driver_set_minus"):
        d[k] = False
    r = learner.update(cl.LearnerInputs(**d))
    if r is not None:
      recs.append(r)
  return recs


def test_one_record_per_curve():
  L = cl.CurveLearner(3.0)
  recs = drive_curve(L, 2.0)
  assert len(recs) == 1 and abs(recs[0]["lat_p90"] - 2.0) < 1e-6
  assert recs[0]["used"] == "nothing to learn"     # engaged, below the capability, cruise on: nothing to learn


def test_capability_rises_slowly_on_clean_curves():
  L = cl.CurveLearner(3.0)
  drive_curve(L, 3.2)
  assert abs(L.capability - 3.02) < 1e-6
  for _ in range(100):
    drive_curve(L, 3.4)
  assert 3.35 < L.capability <= 3.4


def test_capability_drops_when_the_driver_helps_near_the_limit():
  L = cl.CurveLearner(3.0)
  recs = drive_curve(L, 2.8, steering_pressed=lambda f: f > 100)
  assert abs(L.capability - 2.52) < 1e-6 and "capability down" in recs[0]["used"]
  # a nudge on a gentle curve says nothing about the limit
  L = cl.CurveLearner(3.0)
  drive_curve(L, 1.3, steering_pressed=lambda f: f > 100)
  assert L.capability == 3.0


def test_saturation_counts_like_help():
  L = cl.CurveLearner(3.0)
  drive_curve(L, 2.6, steer_cmd=lambda f: 1.0 if f > 100 else 0.3)
  assert abs(L.capability - 2.34) < 1e-6


def test_no_ti_start_is_the_low_limit():
  L = cl.CurveLearner(1.089)
  drive_curve(L, 1.0, steer_cmd=lambda f: 1.0 if f > 100 else 0.3)
  assert L.capability == cl.LAT_MIN          # bounded below


def test_preference_from_manual_speed_and_gas_only():
  L = cl.CurveLearner(3.0)
  drive_curve(L, 2.3, cruise_enabled=False, lat_active=False)
  assert len(L.pref_samples) == 1 and abs(L.pref_samples[0] - 2.3) < 1e-6
  assert abs(L.preference - (3.0 + (2.3 - 3.0) / cl.PREF_SAMPLES)) < 1e-6
  drive_curve(L, 2.5, gas_pressed=lambda f: 50 <= f < 100)
  assert len(L.pref_samples) == 2
  drive_curve(L, 2.0)                       # cruise on, no gas: not the driver's choice
  assert len(L.pref_samples) == 2


def test_our_slowdown_curves_never_teach_the_preference():
  L = cl.CurveLearner(3.0)
  recs = drive_curve(L, 2.3, cruise_enabled=False, our_offset=lambda f: 1 if f > 20 else 0)
  assert not L.pref_samples and recs[0]["our_slowdown"]
  recs = drive_curve(L, 2.6, brake_pressed=lambda f: f == 100, our_offset=3)
  assert not L.pref_samples


def test_brake_or_set_minus_in_a_curve_is_too_fast():
  L = cl.CurveLearner(3.0)
  drive_curve(L, 2.6, brake_pressed=lambda f: f == 100)
  assert abs(L.pref_samples[-1] - 2.34) < 1e-6
  drive_curve(L, 2.4, driver_set_minus=lambda f: f == 100)
  assert abs(L.pref_samples[-1] - 2.16) < 1e-6 and L.counts["too_fast"] == 2


def test_ignored_curves():
  L = cl.CurveLearner(3.0)
  assert drive_curve(L, 2.3, cruise_enabled=False, blinker=True)[0]["used"] == "ignored: turn signal"
  assert drive_curve(L, 2.3, frames=30, cruise_enabled=False)[0]["used"] == "ignored: too short"
  assert drive_curve(L, 2.3, cruise_enabled=False,
                     v_ego=lambda f: 8.0 if 100 < f < 200 else 40 * MPH)[0]["used"] == "ignored: below 25 mph (turn)"
  assert not L.pref_samples


def test_state_round_trip_and_bounds():
  L = cl.CurveLearner(3.0)
  for _ in range(60):
    drive_curve(L, 2.2, cruise_enabled=False, lat_active=False)
  s = L.state()
  assert len(s["preference_samples"]) == cl.PREF_SAMPLES and s["preference_confidence"] == 1.0
  assert abs(L.preference - 2.2) < 1e-6 and abs(L.suggested - 2.2) < 1e-6
  L2 = cl.CurveLearner(3.0, s)
  assert L2.state()["preference"] == s["preference"] and L2.capability == L.capability
  L3 = cl.CurveLearner(3.0, {"capability": 99, "preference_samples": [0.1, "x"]})
  assert L3.capability == cl.LAT_MAX and not L3.pref_samples
  L4 = cl.CurveLearner(3.0, {"capability": 2.0, "preference_samples": [0.1, 9.0]})
  assert L4.pref_samples == [cl.LAT_MIN, cl.LAT_MAX]


def test_invalid_lateral_accel_is_safe():
  L = cl.CurveLearner(3.0)
  recs = drive_curve(L, lambda f: float("nan") if f == 150 else 2.0)
  assert len(recs) >= 1
  assert L.update(cl.LearnerInputs(v_ego=20, lat_accel=None, lat_active=True, steering_pressed=False, steer_cmd=0,
                                   cruise_enabled=True, gas_pressed=False, brake_pressed=False, blinker=False)) is None
