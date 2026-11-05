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


def _make(tmp_path, monkeypatch, limit_mode=2, stop_mode=2):
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
                       map_mode=0, limit_mode=limit_mode)
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
  sent = _drive(c, 400)
  assert c.logic.anchor is not None and abs(c.logic.anchor / MPH - 50) < 1e-6
  assert any(s for s in sent)                                     # the first SET+ went out
  adopt = [d for k, d in records if k == "speedLimitAdopt"]
  assert adopt and adopt[0]["mode"] == "on" and abs(adopt[0]["new_mph"] - 50) < 1e-6 and adopt[0]["how"] == "taps"
  # Test mode: logged, nothing changes
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=1)
  _drive(c, 400)
  sm.msgs["starpilotPlan"].slcSpeedLimit = 50 * MPH
  sent = _drive(c, 400)
  assert c.logic.anchor is None and not any(sent)
  adopt = [d for k, d in records if k == "speedLimitAdopt"]
  assert len(adopt) == 1 and adopt[0]["mode"] == "test"
  # Off: no follower at all
  c, sm, records = _make(tmp_path, monkeypatch, limit_mode=0)
  assert c.limit_follow is None


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
