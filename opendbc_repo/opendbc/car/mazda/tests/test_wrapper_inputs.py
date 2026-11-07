"""The card-side wrapper (curve_slowdown.CurveSlowdown), drive 4c additions: it feeds the speed limit follower and logs
an adoption, and builds the new stopped car trigger inputs (offset from the model path, model length, lane change).
Needs the native-module stubs (run with -p stub_native); skipped without them."""
import importlib.util
import os
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)

import cereal.messaging as messaging
from opendbc.car.mazda import curve_slowdown as cs
from opendbc.car.mazda import car_profile as cp
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda import stopped_car_slowdown as scs
from opendbc.car.mazda.values import MazdaSafetyFlags

_spec = importlib.util.spec_from_file_location("tcpw", os.path.join(os.path.dirname(__file__), "test_car_profile_wrapper.py"))
W = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(W)
MPH = W.MPH


def _make(tmp_path, monkeypatch, limit_mode=2, stop_mode=2, map_mode=0):
  sm = W.FakeSM()
  plan = sm.msgs["starpilotPlan"]
  plan.slcSpeedLimit, plan.slcSpeedLimitSource, plan.mapGpsTrust = 45 * MPH, "Map Data", 0
  monkeypatch.setattr(messaging, "SubMaster", lambda services: sm)
  monkeypatch.setattr(cp, "PROFILE_PATH", str(tmp_path / "profile.json"))
  monkeypatch.setattr(cp.CarProfile.__init__, "__defaults__", (str(tmp_path / "profile.json"), "", False, time.monotonic))
  monkeypatch.setattr(mazdacan, "create_button_cmd", lambda *a, **k: "button")
  CP = SimpleNamespace(flags=MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR, carFingerprint="MAZDA_CX5",
                       maxLateralAccel=1.09)
  c = cs.CurveSlowdown(CP, W.FakeParams({"IsMetric": False}), curve_enabled=False, stop_mode=stop_mode, learning=False,
                       map_mode=map_mode, limit_mode=limit_mode)
  records = []
  if c.recorder is not None:
    c.recorder.root = str(tmp_path / "events")
    monkeypatch.setattr(c.recorder, "record", lambda kind, t, data, **kw: records.append((kind, data)))
  return c, sm, records


def _drive(c, n, set_mph=45, v_mph=44):
  out = []
  for _ in range(n):
    out.append(c.update(W.CC, W.car_state(v_mph, set_mph), None, None, False))
  return out


def test_wrapper_feeds_the_limit_follower_and_logs_the_adoption(tmp_path, monkeypatch):
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=2)
  _drive(c, 400)
  sm.msgs["starpilotPlan"].slcSpeedLimit = 50 * MPH
  sent = _drive(c, 600)                                           # (a higher limit: steady 5 s)
  assert c.logic.anchor is not None and abs(c.logic.anchor / MPH - 50) < 1e-6
  assert any(s for s in sent)                                     # the first SET+ went out
  adopt = [d for k, d in records if k == "speedLimitAdopt"]
  assert adopt and adopt[0]["mode"] == "on" and abs(adopt[0]["new_mph"] - 50) < 1e-6 and adopt[0]["how"] == "taps"
  # Test mode: logged, nothing changes
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=1)
  _drive(c, 400)
  sm.msgs["starpilotPlan"].slcSpeedLimit = 50 * MPH
  sent = _drive(c, 600)
  assert c.logic.anchor is None and not any(sent)
  adopt = [d for k, d in records if k == "speedLimitAdopt"]
  assert len(adopt) == 1 and adopt[0]["mode"] == "test"
  # Off: no follower at all
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=0)
  assert c.limit_follow is None


def _hold_brake(c, n, set_mph=45, v_mph=44):
  return [c.update(W.CC, W.car_state(v_mph, set_mph, brake=True), None, None, False) for _ in range(n)]


def test_wrapper_caps_a_pending_restore_at_a_lower_limit_and_logs_it(tmp_path, monkeypatch):
  # drive 52, tag 450: on a 50 road the set was lowered to 45 (our anchor 50); the limit falls to 45: the restore stops
  for mode in (2, 1):
    c, sm, records = _make(tmp_path, monkeypatch, limit_mode=mode)
    plan = sm.msgs["starpilotPlan"]
    plan.slcSpeedLimit = 50 * MPH
    c.logic.anchor, c.logic.last_set_speed = 50 * MPH, 45 * MPH
    _hold_brake(c, 400)
    plan.slcSpeedLimit = 45 * MPH
    sent = _hold_brake(c, 400)
    cap = [d for k, d in records if k == "speedLimitCap"]
    assert len(cap) == 1 and cap[0]["mode"] == ("on" if mode == 2 else "test"), (mode, records)
    assert abs(cap[0]["old_mph"] - 50) < 1e-6 and abs(cap[0]["new_mph"] - 45) < 1e-6
    assert abs(cap[0]["driver_mph_before"] - 50) < 1e-6 and abs(cap[0]["restore_to_mph"] - 45) < 1e-6
    assert not any(sent)
    if mode == 2:
      assert c.logic.anchor is None and c.logic.limit_cap is None          # nothing left to restore
    else:
      assert abs(c.logic.anchor / MPH - 50) < 1e-6                         # Test: logged only
  # nothing pending (the set at the driver's speed): logged as left alone, nothing pressed
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=2)
  plan = sm.msgs["starpilotPlan"]
  plan.slcSpeedLimit = 50 * MPH
  _drive(c, 400, set_mph=50, v_mph=49)
  plan.slcSpeedLimit = 45 * MPH
  sent = _drive(c, 400, set_mph=50, v_mph=49)
  assert not any(sent) and not [d for k, d in records if k == "speedLimitCap"]
  left = [d for k, d in records if k == "speedLimitNotFollowed"]
  assert left and "no restore pending" in left[-1]["reason"]


def test_wrapper_gives_the_follower_the_anchor_not_set_plus_last_frames_offset(tmp_path, monkeypatch):
  # on the frame the set speed changes (our press landing) set + offset is a step off for a frame; the follower would
  # take that for the driver changing his speed and drop its offer
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=2)
  seen = []
  orig = c.limit_follow.update
  monkeypatch.setattr(c.limit_follow, "update", lambda li: (seen.append(li.driver_speed), orig(li))[1])
  c.logic.anchor, c.logic.last_set_speed = 50 * MPH, 45 * MPH
  _hold_brake(c, 10, set_mph=45)
  _hold_brake(c, 10, set_mph=46)
  assert all(abs(v / MPH - 50) < 1e-6 for v in seen), [round(v / MPH, 2) for v in seen]


def test_wrapper_builds_the_new_trigger_inputs(tmp_path, monkeypatch):
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=0)
  m = sm.msgs["modelV2"]
  xs = [float(k * 10) for k in range(20)]
  m.position.x, m.position.y = xs, [0.0002 * x * x for x in xs]       # a left curve: y left positive
  m.meta = SimpleNamespace(laneChangeState="laneChangeStarting")
  lead = sm.msgs["radarState"].leadOne
  lead.status, lead.modelProb, lead.dRel, lead.vLead, lead.yRel = True, 0.9, 100.0, 0.0, -2.0   # on the path
  seen = []
  orig = c.trigger.update
  monkeypatch.setattr(c.trigger, "update", lambda i: (seen.append(i), orig(i))[1])
  _drive(c, 3)
  i = seen[-1]
  assert abs(i.lead_y_path) < 1e-6 and abs(i.model_length - 190.0) < 1e-6 and i.lane_change
  assert abs(scs.lead_path_offset(100.0, -2.0, xs, m.position.y)) < 1e-6
  # without a path the offset is the raw yRel
  m.position.x, m.position.y = [], []
  _drive(c, 2)
  assert seen[-1].lead_y_path == -2.0 and seen[-1].model_length == 0.0


def _mapd(sm, limit_mph, dist):
  sm.msgs["mapdOut"] = SimpleNamespace(nextSpeedLimit=limit_mph * MPH, nextSpeedLimitDistance=dist)
  sm.alive["mapdOut"], sm.recv_frame["mapdOut"], sm.updated["mapdOut"] = True, 1, True


def test_wrapper_feeds_the_merge_and_exit_inputs(tmp_path, monkeypatch):
  # drive 53 (rules 5, 6): the road class, "this link leads to a highway", mapd's next limit and its distance (else
  # starpilotPlan's value, no distance), the true speed and the driver's SET switch
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=2)
  seen = []
  orig = c.limit_follow.update
  monkeypatch.setattr(c.limit_follow, "update", lambda li: (seen.append(li), orig(li))[1])
  plan = sm.msgs["starpilotPlan"]
  plan.mapRoadClass, plan.mapOnRamp, plan.mapRampToHighway, plan.slcNextSpeedLimit = 2, True, True, 65 * MPH
  _mapd(sm, 70, 400.0)
  _drive(c, 2, v_mph=30)
  li = seen[-1]
  assert li.road_class == 2 and li.ramp_to_highway and abs(li.next_limit / MPH - 70) < 1e-6 and li.next_distance == 400.0
  assert abs(li.v_ego / MPH - 30) < 1e-6 and not li.driver_set
  plan.mapOnRamp = False                                          # (the ramp match lost: not known to lead anywhere)
  sm.alive["mapdOut"] = False
  c.update(W.CC, W.car_state(30, 45, set_plus=True), None, None, False)
  li = seen[-1]
  assert not li.ramp_to_highway and abs(li.next_limit / MPH - 65) < 1e-6 and li.next_distance < 0 and li.driver_set


def test_wrapper_merge_on_and_test(tmp_path, monkeypatch):
  for mode in (2, 1):
    c, sm, records = _make(tmp_path, monkeypatch, limit_mode=mode)
    _drive(c, 400)                                                # the 45 street
    plan = sm.msgs["starpilotPlan"]
    plan.slcSpeedLimit, plan.mapRoadClass, plan.mapOnRamp, plan.mapRampToHighway = 0.0, 2, True, True
    _mapd(sm, 70, 250.0)                                          # (already less than the cruise needs: at once)
    sent = _drive(c, 600)
    adopt = [d for k, d in records if k == "speedLimitAdopt"]
    assert len(adopt) == 1 and adopt[0]["kind"] == "merge" and abs(adopt[0]["new_mph"] - 70) < 1e-6, (mode, records)
    if mode == 2:
      assert adopt[0]["mode"] == "on" and abs(c.logic.anchor / MPH - 70) < 1e-6 and any(s for s in sent)
    else:
      assert adopt[0]["mode"] == "test" and c.logic.anchor is None and not any(sent)


def test_wrapper_gives_the_provisional_ceiling_in_on_mode_only_and_logs_it(tmp_path, monkeypatch):
  for mode in (2, 1):
    c, sm, records = _make(tmp_path, monkeypatch, limit_mode=mode)
    seen = []
    orig = c.logic.update
    monkeypatch.setattr(c.logic, "update", lambda i, seen=seen, orig=orig: (seen.append(i.limit_ceiling), orig(i))[1])
    plan = sm.msgs["starpilotPlan"]
    c.logic.anchor, c.logic.last_set_speed = 50 * MPH, 40 * MPH  # a restore 40 -> 50 pending, his 50 on the 45 road
    _hold_brake(c, 400, set_mph=40)
    plan.slcSpeedLimit = 40 * MPH
    _hold_brake(c, 50, set_mph=40)
    assert (abs(seen[-1] / MPH - 40) < 1e-6) if mode == 2 else seen[-1] == 0.0, (mode, seen[-1])
    hold = [d for k, d in records if k == "speedLimitHold"]
    assert len(hold) == 1 and abs(hold[0]["limit_mph"] - 40) < 1e-6 and hold[0]["mode"] == ("on" if mode == 2 else "test")


def test_wrapper_counts_the_exit_plan_on_through_a_planner_gap_and_passes_the_gas(tmp_path, monkeypatch):
  # review: the carried exit distance was counted only while starpilotPlan was alive (a gap stretched the slowdown):
  # the exit plan's odometer goes on by the car's own speed (vEgo x dt) meanwhile
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=0, map_mode=2)
  plan = sm.msgs["starpilotPlan"]
  plan.mapCurveMode, plan.mapRoadClass, plan.mapGpsTrust = 2, 1, 0
  plan.mapNavExit, plan.mapNavExitDistance, plan.mapNavExitSide = 1, 250.0, 1
  plan.mapNavCurveRadius, plan.mapNavCurveDistance = 95.0, 520.0
  c.update(W.CC, W.car_state(44, 45), None, None, False)
  rem = c.map_control.exit_rem
  assert c.map_control.plan is not None and abs(rem - 250.0) < 1.0
  sm.alive["starpilotPlan"] = False
  _drive(c, 100, v_mph=44)
  moved = 44 * MPH * 100 * cs.DT_CTRL
  assert abs(c.map_control.exit_rem - (rem - moved)) < 1e-3, (c.map_control.exit_rem, rem - moved)
  # planner back, near the exit point, his foot on the gas: his decision (the exit slowing ends)
  sm.alive["starpilotPlan"] = True
  c.update(W.CC, W.car_state(44, 45), None, None, False)
  assert not c.map_control.plan.declined
  c.update(W.CC, W.car_state(44, 45, gas=True), None, None, False)
  assert c.map_control.plan.declined


def _lane_model(m, right_edge=True):
  xs = [0.0, 3.0, 6.75, 12.0, 18.75, 27.0, 48.0]
  line = lambda y: SimpleNamespace(x=xs, y=[y] * len(xs))  # noqa: E731
  m.laneLines = [line(-5.4), line(-1.8), line(1.8), line(5.4)]
  m.laneLineProbs = [0.8, 0.9, 0.9, 0.0 if right_edge else 0.8]
  m.roadEdges = [line(-8.0), line(2.8 if right_edge else 8.0)]
  m.roadEdgeStds = [0.3, 0.3]


def test_wrapper_reads_the_exit_setting_feeds_the_lane_view_and_raises_the_brake_alert(tmp_path, monkeypatch):
  for raw, want in ((None, 0), ("2", 2), ("3", 3), ("x", 0), ("9", 0)):
    sm = W.FakeSM()
    monkeypatch.setattr(messaging, "SubMaster", lambda services, sm=sm: sm)
    CP = SimpleNamespace(flags=MazdaSafetyFlags.GEN1, carFingerprint="MAZDA_CX5", maxLateralAccel=1.09)
    c = cs.CurveSlowdown(CP, W.FakeParams({"IsMetric": False, "MazdaExitSplitSpeed": raw}), curve_enabled=False, map_mode=2)
    assert c.map_control.split_mode == want, (raw, c.map_control.split_mode)
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=0, map_mode=2)
  _lane_model(sm.msgs["modelV2"])
  sm.updated["modelV2"] = True
  _drive(c, 150)
  assert c.map_control.lane.edge[1] and not c.map_control.lane.edge[-1]
  _lane_model(sm.msgs["modelV2"], right_edge=False)
  _drive(c, 250)
  assert not c.map_control.lane.edge[1]
  # the model path's heading 80 m ahead against our lane (map_curve_control.ModelPath): here the path turns off 5 deg
  # toward the right from 40 m on (this model's y: right positive, as its lane lines say)
  import math
  m = sm.msgs["modelV2"]
  xs = [float(k * 5) for k in range(25)]
  m.position.x = xs
  m.position.y = [0.0 if x < 40 else (x - 40) * math.tan(math.radians(5.0)) for x in xs]
  _lane_model(m, right_edge=True)
  _drive(c, 5)
  assert c.map_control.model.rel is not None and abs(c.map_control.model.rel - 5.0) < 0.5, c.map_control.model.rel
  # a model without lane lines (or a malformed one) never breaks the wrapper
  sm.msgs["modelV2"].laneLines = []
  _drive(c, 10)
  # "Exit curve - brake": the flag for EXIT_CHIME_HOLD (starpilotCarState.mazdaExitCurveBrake), one record
  plan = sm.msgs["starpilotPlan"]
  plan.mapCurveMode = 2
  c.map_control.chime_count += 1
  c.map_control.chime_why = "before the split"
  c.update(W.CC, W.car_state(44, 45), None, None, False)
  assert c.exit_brake_alert
  _drive(c, int(c.EXIT_CHIME_HOLD / cs.DT_CTRL))
  assert not c.exit_brake_alert
  brake = [d for k, d in records if k == "mapExitBrake"]
  assert len(brake) == 1 and brake[0]["mode"] == "on" and brake[0]["why"] == "before the split"
  # Test mode: logged, no alert
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=0, map_mode=1)
  sm.msgs["starpilotPlan"].mapCurveMode = 1
  c.map_control.chime_count += 1
  c.update(W.CC, W.car_state(44, 45), None, None, False)
  assert not c.exit_brake_alert and [d for k, d in records if k == "mapExitBrake"][0]["mode"] == "test"


def test_wrapper_logs_each_navigation_exit_with_the_lane_and_shape_checks(tmp_path, monkeypatch):
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=0, map_mode=2)
  opened, closed = [], []
  monkeypatch.setattr(c.recorder, "open", lambda kind, t, summary, **kw: (opened.append((kind, summary)), {"kind": kind})[1])
  monkeypatch.setattr(c.recorder, "close", lambda ev, t, extra=None: closed.append((ev["kind"], extra)))
  rows = []
  monkeypatch.setattr(c.recorder, "add_row", lambda t, values: rows.append(values))
  plan = sm.msgs["starpilotPlan"]
  plan.mapCurveMode, plan.mapRoadClass, plan.mapGpsTrust, plan.slcSpeedLimit = 2, 1, 0, 70 * MPH
  plan.mapNavExit, plan.mapNavExitDistance, plan.mapNavExitSide = 1, 1500.0, 1
  plan.mapNavCurveRadius, plan.mapNavCurveDistance = 95.0, 1770.0
  plan.mapExitShape, plan.mapExitLinkDist, plan.mapExitHighwayDist = 1, 0.0, 5.5
  _drive(c, 400)                                                  # (the highway's limit steady LIMIT_STEADY far from it)
  c.map_control.odo = c.map_control.plan.exit_at - 900.0         # (600 m driven)
  plan.mapNavExitDistance, plan.mapNavCurveDistance = 900.0, 1170.0
  _drive(c, 10)
  assert not [k for k, _ in opened if k == "mapExit"]             # 900 m out: not yet (EXIT_LOG_BEFORE)
  c.map_control.odo = c.map_control.plan.exit_at - 500.0
  plan.mapNavExitDistance, plan.mapNavCurveDistance = 500.0, 770.0
  _drive(c, 10)
  ex = [s for k, s in opened if k == "mapExit"]
  assert len(ex) == 1 and abs(ex[0]["exit_m"] - 500) < 2 and ex[0]["side"] == 1 and abs(ex[0]["limit_mph"] - 70) < 1e-6
  assert not any(k in s for s in ex for k in ("lat", "lon", "latitude", "longitude"))
  cols = c.EVENT_COLUMNS
  assert len(rows[-1]) == len(cols) and rows[-1][cols.index("exit_shape")] == 1 and rows[-1][cols.index("exit_state")] == "planned"
  # missed: the plan ends, the event closes with its result
  c.map_control._end("missed", restore=True)
  _drive(c, 5)
  done = [e for k, e in closed if k == "mapExit"]
  assert len(done) == 1 and done[0]["result"] == "missed" and done[0]["ended"] == "missed"


def test_the_writer_threads_leave_cards_real_time_priority_first(tmp_path, monkeypatch):
  # outside review of e1be62454: the follow summary and curve learning writers inherited card's SCHED_FIFO (drives 3b /
  # 44: "System Lagging" from file writes at card's priority); each now leaves it before any file work
  calls = []
  monkeypatch.setattr(cp, "leave_caller_priority", lambda: calls.append("leave"))
  monkeypatch.setattr(cs.CurveSlowdown, "_learning_dir", classmethod(lambda cls: (calls.append("dir"), str(tmp_path))[1]))
  c = cs.CurveSlowdown.__new__(cs.CurveSlowdown)
  c.follow_params = W.FakeParams({})
  c._write_follow_line({"ticks": 1})
  assert calls[:2] == ["leave", "dir"], calls
  calls.clear()
  c._write_learning({"duration_s": 1.0}, {"x": 1})
  assert calls[:2] == ["leave", "dir"], calls
  assert (tmp_path / cs.CurveSlowdown.FOLLOW_SUMMARY_CSV).exists() and (tmp_path / cs.CurveSlowdown.LEARNING_CSV).exists()


def test_the_wheels_cancel_or_resume_stops_our_frames(tmp_path, monkeypatch):
  # outside review of e1be62454: the wheel's CANCEL (CRZ_BTNS CAN_OFF) and RES were never read, so our 0x9D frames during
  # a hold could hide his CANCEL. While either is down, and DRIVER_QUIET after, the logic is told another button is on
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=0)
  seen = []
  monkeypatch.setattr(c.logic, "update", lambda i: (seen.append(i.other_button_active), (None, None))[1])

  def state(**kw):
    s = W.car_state(44, 45)
    s.crz_cancel, s.crz_resume = kw.get("cancel", False), kw.get("resume", False)
    return s
  for kw in (dict(cancel=True), dict(resume=True)):
    seen.clear()
    c.update(W.CC, state(), None, None, False)
    c.update(W.CC, state(**kw), None, None, False)
    for _ in range(int(cs.DRIVER_QUIET / 0.01) - 1):
      c.update(W.CC, state(), None, None, False)
    c.update(W.CC, state(), None, None, False)
    assert seen[0] is False and all(seen[1:-1]) and seen[-1] is False, (kw, seen[:3], seen[-3:])
  # carstate reads them from CRZ_BTNS
  src = open(os.path.join(os.path.dirname(cs.__file__), "carstate.py")).read()
  assert 'cp.vl["CRZ_BTNS"]["CAN_OFF"]' in src and 'cp.vl["CRZ_BTNS"]["RES"]' in src
