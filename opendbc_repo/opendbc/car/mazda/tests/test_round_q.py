"""Round Q (drives 74 / 75): the logged scenarios as tests (synthetic geometry only)."""
import dataclasses
import importlib.util
import os

HERE = os.path.dirname(__file__)


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


TE = _load("tep_round_q", os.path.join(HERE, "test_exit_plan.py"))
CTL, cs, F = TE.CTL, TE.cs, TE.F
MPH = cs.MS_PER_MPH
DT = cs.DT_CTRL


# ---- Q1: exit 48 (drive 75 t 1227-1260) ----

def _stale(fields, exit_x, every=62.0):
  """Navigation's maneuverDistance as it comes (~1 Hz, sometimes 2 s apart: held for `every` m, never counted down
  between); the map's link start (mapExitStart) fresh every frame."""
  def f(x):
    fl = fields(x)
    if fl.nav_exit_distance > 0:
      held = exit_x - (x - (x % every))
      fl = dataclasses.replace(fl, nav_exit_distance=held, exit_start=max(exit_x - x, 0.0))
    return fl
  return f


def _e48(reach=True, signal=True, every=62.0, stay=False, shape_from=150.0, gas=None):
  """Exit 48: 75 set at 70 mph, the 113 m exit curve (35 mph) 262 m past the split, his right signal 370-300 m before it,
  GPS trust 2 from the split, the shapes silent past the split until `shape_from` m ("link"; "highway" when he stays)."""
  exit_x, curve_x = 2000.0, 2262.0
  base = TE.exit_fields(exit_x, curve_x, r=113.0, ann=1000.0, clear=10.0, link_at=shape_from, ramp_at=shape_from + 40,
                        curve_held=100.0, stay=stay, stay_from=shape_from)
  fields = _stale(base, exit_x, every)
  sig = (lambda x: 1 if exit_x - 370 <= x < exit_x - 300 else 0) if signal else None
  old = CTL.TENT_REACH
  CTL.TENT_REACH = reach
  try:
    r = TE.drive_exit(fields, 9000, 75, v_mph=70, signal=sig, lane=lambda x: 1 if exit_x - 450 <= x < exit_x + 60 else None,
                      gas=gas)
  finally:
    CTL.TENT_REACH = old
  return r, exit_x, curve_x


def test_q1_navigations_stale_distance_does_not_reroute_and_his_signal_is_kept():
  r, exit_x, curve_x = _e48()
  ends = [p for p in (r["ctl"].last_plan,) if p is not None and p.why in CTL.NAV_REVIVE]
  assert not ends, ends
  # (base: start_off took up each held value's error, the next fresh one was a whole update off: "rerouted" ~380 m out)
  p = r["ctl"].last_plan or r["ctl"].plan
  assert p.signalled and p.signal_odo is not None and p.exit_at - p.signal_odo <= CTL.NAV_SIGNAL_REACH


def test_q1_a_rerouted_plan_is_taken_on_again_with_his_signal():
  exit_x, curve_x = 2000.0, 2262.0
  base = TE.exit_fields(exit_x, curve_x, r=113.0, ann=1000.0, clear=10.0, link_at=150.0, ramp_at=190.0, curve_held=100.0)

  def fields(x):
    f = base(x)
    if exit_x - 290 <= x < exit_x - 250:                 # navigation shows another exit 1 km on for ~1.3 s
      f = dataclasses.replace(f, nav_exit_distance=exit_x + 1000 - x, exit_start=0.0)
    return f
  r = TE.drive_exit(fields, 9000, 75, v_mph=70, signal=lambda x: 1 if exit_x - 370 <= x < exit_x - 300 else 0)
  p = r["ctl"].plan or r["ctl"].last_plan
  assert p.revived >= 1 and p.signalled, (p.revived, p.signalled, p.why)


def test_q1_exit_48_arrives_at_the_curve_at_its_speed():
  r, exit_x, curve_x = _e48()
  v_c = TE._at(r, curve_x)
  assert v_c <= 39.0, v_c
  assert "exit reach" in r["why"] or "exit reach hold" in r["why"]
  # without the reachable profile (round P's tentative bounds alone) it comes in faster
  r0, _, _ = _e48(reach=False)
  assert TE._at(r0, curve_x) > v_c + 1.5, (TE._at(r0, curve_x), v_c)


def test_q1_the_reachable_profile_needs_his_signal_and_lets_go_at_highway():
  # the camera's edge lane alone (the ordinary right lane): never
  r, exit_x, _ = _e48(signal=False)
  assert "exit reach" not in r["why"] and "exit reach hold" not in r["why"]
  # he stays on: the shapes say "highway" 60 m past the split; the profile let go, his speed back
  r, exit_x, _ = _e48(stay=True, shape_from=60.0)
  p = r["ctl"].last_plan
  assert p.why == "missed", p.why
  assert abs(r["sets"][-1] - 75) < 1e-6
  past = [v for v, xx in zip(r["vs"], r["xs"], strict=True) if xx >= exit_x]
  assert min(past) >= TE._at(r, exit_x) - CTL.TENT_REACH_DIP / MPH - 1.0
  # nothing answers at all (GPS trust 2, no shapes): never more than TENT_REACH_DIP under the speed at the split (+ lag)
  r, exit_x, _ = _e48(shape_from=5000.0)
  past = [v for v, xx in zip(r["vs"], r["xs"], strict=True) if exit_x <= xx]
  assert min(past) >= TE._at(r, exit_x) - CTL.TENT_REACH_DIP / MPH - 3.0, (TE._at(r, exit_x), min(past))


def test_q1_his_gas_held_lets_the_reachable_profile_go():
  exit_x = 2000.0
  r, exit_x, _ = _e48(shape_from=5000.0, gas=lambda x: 62 * MPH if exit_x + 40 <= x < exit_x + 300 else None)
  p = r["ctl"].plan or r["ctl"].last_plan
  assert p.reach_off == "his gas", p.reach_off
  later = [w for w, xx in zip(r["why"], r["xs"], strict=True) if exit_x + 120 <= xx < exit_x + 300]
  assert "exit reach" not in later and "exit reach hold" not in later


# ---- Q2: the merge boost at a loop's end (drive 75 t 1158.6-1159.1; drive 73's loop start) ----

TP = _load("trp_round_q", os.path.join(HERE, "test_round_p.py"))


def _loop_end(csc_fn, frames=1500):
  def inp(f, car, lg):
    v = csc_fn(f, car)
    return {"csc_active": v is not None, "csc_speed": (v or 0.0) * MPH}
  return TP._boost73(frames=frames, inp=inp)


def test_q2_a_curve_target_rising_out_of_the_loop_keeps_the_boost():
  # drive 75: the camera target 67 -> 69 mph at 33-34 mph for 1.5 s from the merge's adoption (was: ended after 0.5 s,
  # anchor 75)
  car, lg, log = _loop_end(lambda f, car: 67.0 + 2.0 * min(max(f - 50, 0) / 150.0, 1.0) if 53 <= f < 200 else None)
  assert max(s for _, s, *_ in log) >= 89.5, max(s for _, s, *_ in log)
  # drive 73's loop start (the 72.8 mph release ramp, 0.8 s): the same
  car, lg, log = _loop_end(lambda f, car: 72.8 if 53 <= f < 133 else None)
  assert max(s for _, s, *_ in log) >= 89.5


def test_q2_a_real_tight_curve_still_ends_it():
  # a curve asking for no more than the speed + 3 mph, for MERGE_CSC_END: ended, the anchor at the target
  car, lg, log = _loop_end(lambda f, car: car.v_ego / MPH + 3.0 if 60 <= f < 400 else None)
  assert lg.boost is None or "ended" in lg.boost
  assert max(s for _, s, *_ in log) <= 75 + 1e-6
  # a target far above the speed that comes down to it later (a curve ahead coming into view) ends it then
  car, lg, log = _loop_end(lambda f, car: (72.0 if f < 300 else car.v_ego / MPH + 2.0) if 60 <= f < 600 else None)
  assert lg.boost is None or "ended" in lg.boost

