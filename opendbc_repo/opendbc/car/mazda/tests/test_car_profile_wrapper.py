"""The card-side wrapper (curve_slowdown.CurveSlowdown) with the car profile: loads it at start, applies it to the
logic, learns from the frames, saves in the background, honours the reset param, and puts the learned values in the
event summaries. Needs the native-module stubs (run with -p stub_native); skipped without them."""
import json
import os
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)

import cereal.messaging as messaging
from opendbc.car.mazda import curve_slowdown as cs
from opendbc.car.mazda import car_profile as cp
from opendbc.car.mazda.values import MazdaSafetyFlags
from opendbc.car.mazda.curve_target import MAP_DECEL

MPH = 0.44704


class FakeParams:
  def __init__(self, v=None):
    self.v = dict(v or {})

  def get(self, key, encoding=None):
    return self.v.get(key)

  def get_bool(self, key):
    return bool(self.v.get(key))

  def put_bool(self, key, val):
    self.v[key] = bool(val)


class FakeSM:
  def __init__(self):
    self.msgs = {
      "starpilotPlan": SimpleNamespace(cscControllingSpeed=False, cscSpeed=0.0, mapCurveMode=0, mapCurveStatus=0, mapCurveDistance=0.0,
                                       mapCurveRadius=0.0, mapCurveName="", mapOnRamp=False, mapRampType=0, mapRampToSpeed=0.0,
                                       mapRampToHighway=False, mapRampEndDistance=0.0, mapNavExit=0, mapNavExitDistance=0.0,
                                       mapNavCurveRadius=0.0, mapNavCurveDistance=0.0, mapNavExitSide=0, mapCurveAdvisory=0.0,
                                       mapNavCurveAdvisory=0.0, mapRoadClass=0),
      "radarState": SimpleNamespace(leadOne=SimpleNamespace(status=False, modelProb=0.0, dRel=0.0, vLead=0.0, yRel=0.0)),
      "modelV2": SimpleNamespace(action=SimpleNamespace(desiredAcceleration=0.0), velocity=SimpleNamespace(x=[], t=[]),
                                 orientationRate=SimpleNamespace(z=[]), position=SimpleNamespace(x=[], y=[])),
    }
    self.alive = dict.fromkeys(self.msgs, True)
    self.recv_frame = dict.fromkeys(self.msgs, 1)
    self.updated = dict.fromkeys(self.msgs, False)

  def update(self, timeout):
    pass

  def __getitem__(self, k):
    return self.msgs[k]


def make_cs(tmp_path, params, monkeypatch, fingerprint="MAZDA_CX5", ti=True, map_mode=2):
  monkeypatch.setattr(messaging, "SubMaster", lambda services: FakeSM())
  monkeypatch.setattr(cp, "PROFILE_PATH", str(tmp_path / "profile.json"))
  monkeypatch.setattr(cp.CarProfile.__init__, "__defaults__", (str(tmp_path / "profile.json"), "", False, time.monotonic))
  flags = MazdaSafetyFlags.GEN1 | (MazdaSafetyFlags.TORQUE_INTERCEPTOR if ti else 0)
  CP = SimpleNamespace(flags=flags, carFingerprint=fingerprint, maxLateralAccel=1.09)
  c = cs.CurveSlowdown(CP, params, curve_enabled=True, stop_mode=2, learning=False, map_mode=map_mode)
  if c.recorder is not None:
    c.recorder.root = str(tmp_path / "events")
  return c


def car_state(v_mph, set_mph, cruise=True, set_minus=False, set_plus=False, brake=False, gas=False):
  out = SimpleNamespace(cruiseState=SimpleNamespace(enabled=cruise, speed=set_mph * MPH), vEgo=v_mph * MPH, aEgo=0.0,
                        gasPressed=gas, brakePressed=brake, standstill=False, leftBlinker=False, rightBlinker=False, steeringPressed=False)
  return SimpleNamespace(out=out, crz_set_minus=set_minus, crz_set_plus=set_plus, crz_btns_counter=0, radar_has_lead=False, radar_lead_distance=0)


CC = SimpleNamespace(enabled=True, cruiseControl=SimpleNamespace(cancel=False, resume=False), latActive=True,
                     actuators=SimpleNamespace(torque=0.0))


def test_profile_loads_applies_learns_and_saves(tmp_path, monkeypatch):
  params = FakeParams({"IsMetric": False})
  # a profile from earlier drives
  p = cp.CarProfile(path=str(tmp_path / "profile.json"), fingerprint="MAZDA_CX5", ti=True)
  p.set_latencies([11, 10, 12])
  for _ in range(6):
    p.add_hold(2)
  for _ in range(12):
    p.add_decel_episode(0.2)
  p.save(in_thread=False)
  c = make_cs(tmp_path, params, monkeypatch)
  assert c.profile is not None and c.profile.loaded
  lg = c.logic
  assert lg.latencies == [11, 10, 12] and lg.latency == 11
  assert lg.hold_measured and lg.hold_overrun == 2 and lg._up_overrun() == 3
  assert c.decel_used == 0.2 and c.map_control.decel == 0.2 and c.map_control.curve.decel == 0.2
  assert c.profile.data["drives"] == 1
  summary = c._profile_summary()
  assert summary["answer_frames"] == 11 and summary["hold_overrun"] == 2 and summary["decel_used"] == 0.2 and "0.20" in summary["profile"]
  # a few frames of driving: the learner runs, nothing raises, the profile stays
  for _ in range(50):
    c.update(CC, car_state(43, 45), None, None, False)
  assert c.profile is not None and c.decel_learner is not None
  # the driver holds SET- (stock switch) and the car steps: the hold is learned into the profile
  lg.holds_learned += 1
  lg.last_hold_learned = (3, "driver")
  c.update(CC, car_state(43, 45), None, None, False)
  assert c.profile.holds == 7 and c.profile.data["hold"]["worst_after_release"] == 3 and c.profile.data["hold"]["driver_holds"] == 1
  assert c.profile.dirty
  # cruise off: saved now (in the background)
  c.update(CC, car_state(43, 45, cruise=False), None, None, False)
  for _ in range(200):
    if not c.profile.saving:
      break
    time.sleep(0.01)
  with open(str(tmp_path / "profile.json")) as f:
    saved = json.load(f)
  assert saved["hold"]["holds"] == 7 and saved["hold"]["worst_after_release"] == 3 and saved["drives"] == 1


def test_reset_param_deletes_the_profile_and_clears_itself(tmp_path, monkeypatch):
  params = FakeParams({"IsMetric": False, "MazdaResetCarProfile": True})
  p = cp.CarProfile(path=str(tmp_path / "profile.json"), fingerprint="MAZDA_CX5", ti=True)
  p.set_latencies([11, 10, 12])
  p.save(in_thread=False)
  c = make_cs(tmp_path, params, monkeypatch)
  assert not os.path.exists(str(tmp_path / "profile.json"))
  assert params.v["MazdaResetCarProfile"] is False
  assert c.logic.latencies == [] and not c.logic.hold_measured and c.decel_used == MAP_DECEL


def test_other_car_starts_fresh(tmp_path, monkeypatch):
  params = FakeParams({"IsMetric": False})
  p = cp.CarProfile(path=str(tmp_path / "profile.json"), fingerprint="MAZDA_3", ti=True)
  p.set_latencies([11, 10, 12])
  p.save(in_thread=False)
  c = make_cs(tmp_path, params, monkeypatch)
  assert c.profile is not None and not c.profile.loaded and c.profile.discarded.startswith("other car")
  assert c.logic.latencies == [] and c._profile_summary()["profile_discarded"].startswith("other car")
  c2 = make_cs(tmp_path, params, monkeypatch, ti=False)
  assert c2.profile.discarded.startswith("other car")       # fingerprint first; the TI flag is checked too


def test_profile_failure_never_reaches_the_controller(tmp_path, monkeypatch):
  params = FakeParams({"IsMetric": False})
  c = make_cs(tmp_path, params, monkeypatch)
  assert c.profile is not None
  monkeypatch.setattr(c.decel_learner, "update", lambda i: 1 / 0)
  c.update(CC, car_state(43, 45), None, None, False)
  assert c.profile is None                                   # switched off for the drive, nothing raised
  c.update(CC, car_state(43, 45), None, None, False)
