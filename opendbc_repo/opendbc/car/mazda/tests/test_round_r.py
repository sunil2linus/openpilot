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

