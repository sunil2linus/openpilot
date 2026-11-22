"""The target arbitrator: min(his own speed, contributors), his anchor rules, test mode, and the cruise model's
undershoot / LKAS floor / braking envelope."""
import math

from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed.planner import Planner, PlanInputs, Contribution

MPH = M.MPH


def step(p, set_mph, v_mph, events=(), target_mph=None, name="curve", presses=True, main=True, cruise=True, floor=False,
         extra=()):
  contribs = [Contribution(name, None if target_mph is None else target_mph * MPH, lkas_floor=floor, presses=presses),
              *extra]
  return p.update(PlanInputs(v_ego=v_mph * MPH, set_speed=set_mph * MPH, cruise_enabled=cruise, main_on=main,
                             events=list(events), contributions=contribs))


def engaged(set_mph):
  return {"kind": "engaged", "before": None, "after": set_mph * MPH, "changed": False}


def driver(direction, after_mph):
  return {"kind": "driver", "direction": direction, "after": after_mph * MPH}


def mph(x):
  return None if x is None else x / MPH


def test_anchor_from_engage_and_min_of_contributors():
  p = Planner()
  out = step(p, 55, 53, [engaged(55)])
  assert abs(mph(p.anchor) - 55) < 1e-6 and abs(mph(out.command) - 55) < 1e-6 and out.reason == ""
  out = step(p, 55, 53, target_mph=40)
  assert out.reason == "curve"
  # car 53 > target 40 + 1.5: the undershoot is on, dash offset added
  assert abs(mph(out.command) - (40 + 2 - 4)) < 1e-6
  out = step(p, 45, 41, target_mph=40)
  assert abs(mph(out.command) - 42) < 1e-6                  # slowed: back to the target (+ dash offset)
  out = step(p, 45, 41, target_mph=60)
  assert abs(mph(out.command) - 55) < 1e-6 and out.reason == ""   # never above his speed


def test_lkas_floor_only_for_curves_that_allow_it():
  p = Planner()
  step(p, 50, 48, [engaged(50)])
  assert abs(mph(step(p, 50, 48, target_mph=30, floor=True).command) - 29) < 1e-6
  assert mph(step(p, 50, 48, target_mph=30, floor=False).command) < 29


def test_his_set_minus_is_his_speed_and_blocks_that_reason_until_over():
  p = Planner()
  step(p, 60, 58, [engaged(60)])
  step(p, 55, 56, target_mph=40)
  out = step(p, 52, 50, [driver("SET_MINUS", 52)], target_mph=40)
  assert abs(mph(p.anchor) - 52) < 1e-6 and "curve" in p.blocked
  assert abs(mph(out.command) - 52) < 1e-6                  # held where he put it
  out = step(p, 52, 45, target_mph=40)
  assert abs(mph(out.command) - 52) < 1e-6
  out = step(p, 52, 45, target_mph=None)                    # reason over
  assert not p.blocked and abs(mph(out.command) - 52) < 1e-6   # never back up to the old 60
  out = step(p, 52, 50, target_mph=40)                      # the next curve may lower again
  assert out.reason == "curve"


def test_his_set_plus_under_the_anchor_keeps_it_and_stops_our_lowering():
  p = Planner()
  step(p, 60, 58, [engaged(60)])
  step(p, 50, 50, target_mph=40)
  assert p.lowered
  out = step(p, 46, 45, [driver("SET_PLUS", 46)], target_mph=40)
  assert abs(mph(p.anchor) - 60) < 1e-6 and p.blocked
  assert abs(mph(out.command) - 46) < 1e-6                  # not lowered again, not raised either
  out = step(p, 46, 45, target_mph=None)
  assert abs(mph(out.command) - 60) < 1e-6 and not p.blocked   # restored after the curve
  # his SET+ to (or past) the anchor is his new speed
  out = step(p, 62, 50, [driver("SET_PLUS", 62)])
  assert abs(mph(p.anchor) - 62) < 1e-6


def test_main_off_forgets_and_cruise_off_leaves_alone():
  p = Planner()
  step(p, 60, 58, [engaged(60)])
  out = step(p, 60, 58, target_mph=40, cruise=False)
  assert out.command is None and p.anchor is not None
  out = step(p, 60, 58, main=False)
  assert out.command is None and p.anchor is None
  out = step(p, 60, 58, target_mph=40)
  assert out.command is None                                # nothing anchored yet: nothing to do


def test_resume_keeps_the_anchor_engage_elsewhere_replaces_it():
  p = Planner()
  step(p, 60, 58, [engaged(60)])
  step(p, 50, 50, target_mph=40, cruise=False)
  out = step(p, 50, 50, [{"kind": "resumed", "after": 50 * MPH}])
  assert abs(mph(p.anchor) - 60) < 1e-6 and abs(mph(out.command) - 60) < 1e-6    # we restore from where we left it
  step(p, 35, 34, [{"kind": "engaged", "before": 50 * MPH, "after": 35 * MPH, "changed": True}])
  assert abs(mph(p.anchor) - 35) < 1e-6


def test_test_mode_contributors_never_command_but_are_reported():
  p = Planner()
  step(p, 60, 58, [engaged(60)])
  out = step(p, 60, 58, target_mph=40, presses=False)
  assert abs(mph(out.command) - 60) < 1e-6 and out.reason == ""
  assert out.dry_reason == "curve" and mph(out.dry_command) < 45
  extra = (Contribution("camera", 50 * MPH),)
  out = step(p, 60, 58, target_mph=40, presses=False, extra=extra)
  assert out.reason == "camera" and out.dry_reason == "curve"


def test_braking_envelope():
  v = 25.0
  assert M.envelope(15.0, 0.0, v) == 15.0
  assert M.envelope(15.0, v * (M.LAG + M.BUTTON_TIME), v) == 15.0
  d = 200.0
  got = M.envelope(15.0, d, v)
  room = d - v * (M.LAG + M.BUTTON_TIME)
  assert abs(got - math.sqrt(15 ** 2 + 2 * M.PLAN_DECEL * room)) < 1e-9 and got > 15.0
  assert M.cruise_decel(4 * MPH) == 0.54 and M.cruise_decel(9 * MPH) == 0.80 and M.cruise_decel(1 * MPH) == 0.0
  assert M.cruise_accel(12 * MPH) == 0.71 and M.cruise_accel(1 * MPH) == 0.0
  # the undershoot gives at least the planning decel's band once the car is faster than the target
  target = 40 * MPH
  cmd_true = M.set_command(target, 50 * MPH) - M.DASH_OFFSET
  assert M.cruise_decel(50 * MPH - cmd_true) >= M.PLAN_DECEL


def test_a_ceiling_caps_the_command_in_dash_units_and_is_a_wish_like_a_target():
  p = Planner()
  step(p, 53, 51, [engaged(53)])
  out = step(p, 53, 51, name="other", extra=[Contribution("stop", None, ceiling=51 * MPH)])
  assert out.reason == "stop" and abs(mph(out.command) - 51) < 1e-6           # exactly 2 steps, no undershoot
  out = step(p, 53, 51, target_mph=40, extra=[Contribution("stop", None, ceiling=51 * MPH)])
  assert out.reason == "curve" and mph(out.command) < 51                       # the lowest command wins
  out = step(p, 53, 51, name="other", extra=[Contribution("stop", None, ceiling=60 * MPH)])
  assert out.reason == "" and abs(mph(out.command) - 53) < 1e-6                # never above his anchor
  out = step(p, 53, 51, name="other", extra=[Contribution("stop", None, ceiling=50 * MPH, presses=False)])
  assert out.reason == "" and out.dry_reason == "stop" and abs(mph(out.dry_command) - 50) < 1e-6   # test: logged only
  # his SET+ under the anchor while the ceiling governs: blocked for the episode (no lowering, no raise)
  step(p, 50, 50, name="other", extra=[Contribution("stop", None, ceiling=50 * MPH, episode="s1")])
  out = step(p, 51, 50, [driver("SET_PLUS", 51)], name="other",
             extra=[Contribution("stop", None, ceiling=50 * MPH, episode="s1")])
  out = step(p, 51, 50, name="other", extra=[Contribution("stop", None, ceiling=49 * MPH, episode="s1")])
  assert "his choice" in out.reason and abs(mph(out.command) - 51) < 1e-6
  out = step(p, 51, 50, name="other", extra=[Contribution("stop", None, ceiling=49 * MPH, episode="s2")])
  assert out.reason == "stop"                                                  # a new episode: his overrule is over
