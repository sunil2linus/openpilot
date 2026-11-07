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


def test_wrapper_counts_a_carried_exit_on_through_a_planner_gap_and_passes_the_gas(tmp_path, monkeypatch):
  # review: the carried exit distance was counted only while starpilotPlan was alive (a gap stretched the slowdown):
  # now by the car's own speed (vEgo x dt) meanwhile
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=0, map_mode=2)
  c.map_control.carry = [30.0, 300.0]
  sm.alive["starpilotPlan"] = False
  _drive(c, 100, v_mph=44)
  moved = 44 * MPH * 100 * cs.DT_CTRL
  assert abs(c.map_control.carry[0] - (30.0 - moved)) < 1e-3 and abs(c.map_control.carry[1] - (300.0 - moved)) < 1e-3
  # planner back, past the exit point on the motorway (not yet EXIT_STAY_MARGIN), his foot on the gas: the exit is dropped
  sm.alive["starpilotPlan"] = True
  plan = sm.msgs["starpilotPlan"]
  plan.mapCurveMode, plan.mapRoadClass, plan.mapGpsTrust = 2, 1, 0
  c.map_control.carry = [-20.0, 250.0]
  c.update(W.CC, W.car_state(44, 45), None, None, False)
  assert c.map_control.carry is not None
  c.update(W.CC, W.car_state(44, 45, gas=True), None, None, False)
  assert c.map_control.carry is None
