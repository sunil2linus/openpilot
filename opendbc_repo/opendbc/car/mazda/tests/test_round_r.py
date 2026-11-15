"""Round R (drive 76, round Q's build): the fixes R1-R7. Synthetic geometry and speeds only."""
import importlib.util
import os

HERE = os.path.dirname(__file__)


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, path))
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


SS = _load("tss_round_r", "test_street_speed.py")
cs, MPH = SS.cs, SS.MPH


# ---------------- R4: the street speed lands on its target ----------------

def test_r4_the_street_target_is_reached_with_the_dashs_x989_readings():
  # drive 76 t 1857 (W Utica Rd): rule 8's match on engage, a 45 street + 3 = 48; the dash reads 49.989 for 49. Before: one
  # SET- to 48.989, then nothing (0.989 of a step "is no step") and the match never finished
  for set0 in (49.989, 53.989, 50.0):
    r = SS.street_sim(4000, lambda f: 45, set0, v_mph=36, street_mode=1, **SS._engage())
    assert abs(r["sets"][-1] - 48) < 0.02, (set0, r["sets"][-1])
    assert "done" in SS._reasons(r) and r["logic"].anchor is None, (set0, SS._reasons(r))
    assert min(r["sets"][SS.ENGAGE:]) >= 48 - 0.02                 # never under it
  # at the target already (49.989 for a +5 street's 50): nothing
  r = SS.street_sim(3000, lambda f: 45, 49.989, v_mph=36, street_mode=2, **SS._engage())
  assert r["presses"] == []


def test_r4_no_margin_on_a_street_target_and_curves_keep_theirs():
  base = dict(enabled=True, cruise_enabled=True, v_ego=40 * MPH, gas_pressed=False, brake_pressed=False, standstill=False,
              plan_alive=True, csc_active=True, other_button_active=False, curve_floor=25 * MPH, curve_budget=30)

  def first_press(set_mph, csc_mph, **kw):
    lg2 = cs.CurveSlowdownLogic(False)
    for _ in range(100):
      b, _ = lg2.update(cs.SlowdownInputs(set_speed=set_mph * MPH, csc_speed=csc_mph * MPH, **base, **kw))
      if b is not None:
        return b
    return None
  # a street target 48 with the set at 48.989: one SET- (whole steps); a camera curve's 48 keeps its margin: none
  assert first_press(48.989, 48.0, street_lower=True, curve_track=True) == cs.SET_MINUS
  assert first_press(48.989, 48.0) is None
  # the street target never goes under itself: at 47.989 nothing more
  assert first_press(47.989, 48.0, street_lower=True, curve_track=True) is None


# ---------------- R2: the merge taper only by the speed ----------------

TP = _load("trp_round_r", "test_round_p.py")


def test_r2_the_highway_match_right_after_the_loop_does_not_taper_at_34_mph():
  # drive 76 t 525.6: the motorway matched with the car at ~34 mph, 41 under the target 75 (before: the anchor 89 -> 80)
  car, lg, log = TP._boost73(inp=lambda f, car, lg: {"merge_taper": f >= 60})
  f65 = next(f for f, s, v, *_ in log if v >= 65)
  assert max(s for f, s, *_ in log if f < f65) >= 90 - 1e-6           # the full boost until 10 mph under the target
  assert all(a is None or a >= 90 * MPH - 1e-6 for f, s, v, b, act, a, _ in log if 100 <= f < f65 - 50)
  assert abs(car.set_speed / MPH - 75) < 1e-6


def test_r2_a_boost_ended_by_its_time_behind_a_moving_lead_goes_to_the_target():
  # a long ramp behind a car at 57 mph (it does not block the boost: moving, not closing): MERGE_BOOST_TIME ends it with
  # the car 18 mph under the target; the set comes down to the target 75 (not left high, not left low)
  def inp(f, car, lg):
    car.v_ego = min(car.v_ego, 57 * MPH)
    return {"radar_lead": True, "lead_v": 57 * MPH}
  car, lg, log = TP._boost73(frames=6000, inp=inp)
  ends = [f for f, s, v, b, *_ in log if b == cs.SET_MINUS]
  assert ends and ends[0] >= int(cs.MERGE_BOOST_TIME / 0.01), ends[:3]
  assert abs(car.set_speed / MPH - 75) < 1e-6 and min(s for f, s, *_ in log if f >= ends[0]) >= 75 - 1e-6
