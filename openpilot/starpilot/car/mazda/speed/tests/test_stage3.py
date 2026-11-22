"""Stage 3 on the car side: the safety param bits at car start (speed_flags.py through vehicle_preferences.py), the TI
steering limit test (STEER_MAX and the torque tune), carstate's auto-resume reads, the actuator's RES path, the caller
(one sender, the planner and the stop slowdown see it as his RES) and the map view adapter of the auto-resume."""
import json
import math
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace as N

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.mazda import speed_flags
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.torque_interceptor import prepare as prepare_ti
from opendbc.car.mazda.values import CAR, CarControllerParams, MazdaFlags, MazdaSafetyFlags
from openpilot.common.params import Params
from openpilot.starpilot.car.mazda.speed import auto_resume as ar, caller, config as C
from openpilot.starpilot.car.mazda.speed.actuator import Actuator, ActInputs, DRIVER_QUIET, RESUME, frames
from openpilot.starpilot.car.mazda.speed.stops import StopSlowdown
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences

MPH = 0.44704
AR = int(MazdaSafetyFlags.AUTO_RESUME)
S700 = int(MazdaSafetyFlags.TI_STEER_700)
TI = int(MazdaSafetyFlags.TORQUE_INTERCEPTOR)


def stock_cp(car=CAR.MAZDA_CX5_2022):
  return CarInterface.get_params(car, gen_empty_fingerprint(), [], False, False, False)


def ti_cp():
  return prepare_ti(stock_cp(CAR.MAZDA_CX5), True)


def saved_params(d, doc):
  params = Params(d)
  if doc is not None:
    Path(params.get_param_path(C.KEY)).write_bytes(json.dumps(doc).encode() if isinstance(doc, dict) else doc)
  return params


def started(params, cp):
  return VehicleStartupPreferences.read(params, enabled=True).prepare(cp)


# ---------------- the bits at car start ----------------

@pytest.mark.parametrize("doc", [None, {"version": 4}, {"version": 5}, {"version": 5, "enabled": True},
                                 {"version": 5, "auto_resume": True}, {"version": 5, "enabled": True, "curves": "on"},
                                 b'{"version":5,"auto_resume":true,"ti_steer_700":true,"bad":1}', b"not json"])
def test_off_the_safety_param_and_the_limits_are_stage2(doc):
  # with the Stage 3 settings off (or the document invalid) the CarParams are exactly the Stage 2 ones
  with tempfile.TemporaryDirectory() as d:
    params = saved_params(d, doc)
    for make, param, steer in ((ti_cp, TI, 600), (stock_cp, 0, 800)):
      reference, cp = make(), started(params, make())
      assert int(cp.safetyConfigs[0].safetyParam) == param
      assert cp.flags == reference.flags and cp.lateralTuning.torque.latAccelFactor == reference.lateralTuning.torque.latAccelFactor
      assert cp.lateralTuning.torque.friction == reference.lateralTuning.torque.friction
      assert CarControllerParams(cp).STEER_MAX == steer
      assert 0x21B not in CarState.get_can_parsers(cp)[Bus.pt].message_states    # no CRZ_INFO read


def test_auto_resume_bit_needs_the_master_switch_and_its_own_switch():
  with tempfile.TemporaryDirectory() as d:
    params = saved_params(d, {"version": 5, "enabled": True, "auto_resume": True})
    cp = started(params, ti_cp())
    assert int(cp.safetyConfigs[0].safetyParam) == TI | AR and int(cp.flags) == int(MazdaFlags.GEN1 | MazdaFlags.TORQUE_INTERCEPTOR)
    assert CarControllerParams(cp).STEER_MAX == 600 and speed_flags.auto_resume(cp) and not speed_flags.ti_steer_700(cp)
    cp = started(params, stock_cp())
    assert int(cp.safetyConfigs[0].safetyParam) == AR and int(cp.flags) == int(MazdaFlags.GEN1)
    # a stock AOL car with the bit is no AOL car (mazda.h blocks all TX with alt exp 32 and a param != 0)
    from opendbc.car.mazda.stock_aol import qualified
    assert not qualified(cp) and qualified(stock_cp())


def test_steer_700_only_with_the_ti_and_the_torque_for_a_lateral_acceleration_is_unchanged():
  with tempfile.TemporaryDirectory() as d:
    params = saved_params(d, {"version": 5, "ti_steer_700": True})   # (not under the master switch)
    base, cp = ti_cp(), started(params, ti_cp())
    assert int(cp.safetyConfigs[0].safetyParam) == TI | S700 and int(cp.flags) == int(base.flags)
    assert CarControllerParams(base).STEER_MAX == 600 and CarControllerParams(cp).STEER_MAX == 700
    t0, t1 = base.lateralTuning.torque, cp.lateralTuning.torque
    assert math.isclose(t1.latAccelFactor, t0.latAccelFactor * 7 / 6, rel_tol=1e-6)
    assert math.isclose(t1.friction, t0.friction * 6 / 7, rel_tol=1e-6)
    for lat in (0.5, 1.0, 1.5):
      assert math.isclose(lat / t0.latAccelFactor * 600, lat / t1.latAccelFactor * 700, rel_tol=1e-6)
    assert math.isclose(t0.friction * 600, t1.friction * 700, rel_tol=1e-6)
    # idempotent: a second prepare changes nothing
    again = speed_flags.prepare(cp, steer_700_requested=True)
    assert again.lateralTuning.torque.latAccelFactor == t1.latAccelFactor
    # without the TI: nothing (800, no bit)
    cp = started(params, stock_cp())
    assert int(cp.safetyConfigs[0].safetyParam) == 0 and CarControllerParams(cp).STEER_MAX == 800
    # the bit alone in a hand-made CP without the TI flag: ignored
    hand = stock_cp()
    hand.safetyConfigs[0].safetyParam = S700
    assert CarControllerParams(hand).STEER_MAX == 800


def test_not_for_other_cars_or_openpilot_long():
  from opendbc.car.toyota.interface import CarInterface as ToyotaInterface
  from opendbc.car.toyota.values import CAR as TOYOTA
  cp = ToyotaInterface.get_params(TOYOTA.TOYOTA_RAV4, gen_empty_fingerprint(), [], False, False, False)
  before = int(cp.safetyConfigs[0].safetyParam)
  assert int(speed_flags.prepare(cp, auto_resume_requested=True, steer_700_requested=True).safetyConfigs[0].safetyParam) == before
  cp = ti_cp()
  cp.openpilotLongitudinalControl = True
  assert int(speed_flags.prepare(cp, auto_resume_requested=True).safetyConfigs[0].safetyParam) == TI


def test_the_controller_sends_up_to_700_with_the_test():
  with tempfile.TemporaryDirectory() as d:
    cp = started(saved_params(d, {"version": 5, "ti_steer_700": True}), ti_cp())
  from opendbc.car.mazda.carcontroller import CarController
  cc = CarController({Bus.pt: "mazda_2017"}, cp)
  assert cc.params.STEER_MAX == 700


# ---------------- carstate ----------------

def test_carstate_reads_crz_info_only_with_the_bit_never_in_can_valid():
  cp = speed_flags.prepare(ti_cp(), auto_resume_requested=True)
  parser = CarState.get_can_parsers(cp)[Bus.pt]
  assert 0x21B in parser.message_states and parser.message_states[0x21B].ignore_alive
  packer = CANPacker("mazda_2017")
  addr, dat, _ = packer.make_can_msg("CRZ_INFO", 0, {"ACC_SET_ALLOWED": 1})
  t = 1_000_000_000
  parser.update([(t, [(addr, dat, 0)])])
  assert parser.vl["CRZ_INFO"]["ACC_SET_ALLOWED"] == 1 and parser.ts_nanos["CRZ_INFO"]["ACC_SET_ALLOWED"] == t
  # byte 4 of CRZ_CTRL: the raw "not allowed" byte (0x0c after P)
  addr, dat, _ = packer.make_can_msg("CRZ_CTRL", 0, {"CRZ_BYTE_4": 0x0C, "CRZ_AVAILABLE": 1})
  assert dat[4] == 0x0C and dat[2] & 0x02
  parser2 = CANParser("mazda_2017", [], 0)
  assert parser2.vl["CRZ_CTRL"]["CRZ_BYTE_4"] == 0       # (registered on first read, as carstate's first update does)
  parser2.update([(t, [(addr, dat, 0)])])
  assert parser2.vl["CRZ_CTRL"]["CRZ_BYTE_4"] == 0x0C


def test_carstate_set_allowed_goes_stale():
  cp = speed_flags.prepare(ti_cp(), auto_resume_requested=True)
  cs = CarState(cp)
  assert cs.auto_resume and not cs.crz_set_allowed
  # the staleness rule alone (update() needs the whole bus): a value not refreshed for CRZ_INFO_STALE updates is "no"
  from opendbc.car.mazda import carstate as M
  assert M.CRZ_INFO_STALE == 20
  assert not CarState(ti_cp()).auto_resume


# ---------------- the actuator's RES path ----------------

def act_in(**kw):
  base = dict(cruise_enabled=False, set_speed=45 * MPH, v_ego=40 * MPH, gas=True, wheel_counter=7)
  base.update(kw)
  return ActInputs(**base)


def test_actuator_res_only_with_the_cruise_off_and_nothing_else_pressed():
  for kw in ({"cruise_enabled": True}, {"other_button": True}, {"brake": True}, {"set_plus": True}, {"set_minus": True},
             {"resume": True}, {"cancel": True}, {"dry_run": True}):
    a = Actuator()
    out = a.update(act_in(auto_resume=True, **kw))
    assert out.button is None and any(e["kind"] == "auto_res_refused" for e in out.events), kw
  a = Actuator()
  out = a.update(act_in(auto_resume=True))
  assert out.button == RESUME and out.counter == 7
  assert {"kind": "driver_button", "frame": a.frame, "button": "RES", "auto": True} in out.events
  # without the request: never a RES, and nothing at all with the cruise off
  a = Actuator()
  for _ in range(50):
    assert a.update(act_in()).button is None


def test_after_our_res_the_actuator_waits_as_after_his_res():
  a = Actuator()
  for _ in range(100):
    a.update(act_in(cruise_enabled=True, gas=False, command=45 * MPH))
  for _ in range(10):
    a.update(act_in(gas=False))
  out = a.update(act_in(auto_resume=True))
  assert out.button == RESUME
  # the cruise comes back at the same set speed: "resumed"; SET+/SET- wait DRIVER_QUIET like after his RES
  events, pressed = [], []
  for k in range(frames(DRIVER_QUIET) + 60):
    out = a.update(act_in(cruise_enabled=True, gas=False, command=40 * MPH))
    events += out.events
    if out.button is not None:
      pressed.append(k)
  assert any(e["kind"] == "resumed" for e in events)
  assert pressed and pressed[0] >= frames(DRIVER_QUIET) - 2


def test_the_stop_slowdown_sees_our_res_as_his():
  stops = StopSlowdown(C.Config(enabled=True, stops="on"))
  res, plus, engaged = stops._his({"act_events": [{"kind": "driver_button", "button": "RES", "auto": True, "frame": 1}]})
  assert res and not plus and not engaged


# ---------------- the map view adapter ----------------

def test_view_road_and_inputs():
  road = ar.ViewRoad()
  now = 100.0
  assert road.update(None, now)[:3] == (False, 0, "")
  view = {"v": 2, "t": now - 0.1, "gps": "ok", "trust": 0, "matched": True, "cls": 5, "chain": 42}
  alive, cls, name, age, last, trust = road.update(view, now)
  assert (alive, cls, name, age, last, trust) == (True, 5, "42", 0.0, "42", 0)
  # gone (view_age.py: older than VIEW_ALIVE after its GPS message; the old rule was 0.5 s after the stamp): no map
  assert road.update(dict(view, t=now - 0.85 - 2.6), now)[0] is False
  assert road.update(dict(view, t=now - 1.85), now)[0] is True                  # a 1 Hz GPS's view: alive
  assert road.update(dict(view, gps="none"), now)[0] is False
  got = road.update(dict(view, matched=False, t=now + 0.9), now + 1.0)
  assert got[:3] == (True, 0, "") and got[4] == "42" and got[3] == 1.0
  d = {"cruise_enabled": False, "main_on": True, "set_speed": 20.0, "v_ego": 10.0, "gas": True, "brake": False,
       "angle": 3.0, "steer_rate": 1.0, "gear_drive": True, "cancel": False, "resume": False, "set_plus": False,
       "set_minus": False, "set_allowed": True, "can_valid": True, "wheel_counter": 3}
  i = ar.inputs_from(d, {"now": now, "view": view, "engageable": True, "lead": {"status": True, "d": 20.0, "v": 3.0, "t": now}},
                     ar.ViewRoad())
  assert i.plan_alive and i.road_class == 5 and i.gps_trust == 0 and i.road_name == "42" and i.engageable
  assert i.lead_status and i.lead_d == 20.0 and i.btn_counter == 3 and i.set_allowed and i.gear_drive
  i = ar.inputs_from(d, {"now": now, "view": None}, ar.ViewRoad())
  assert not i.plan_alive and i.gps_trust == 4 and not i.engageable and not i.lead_status


def test_unsend_takes_a_refused_frame_back():
  lg = ar.AutoResumeLogic()
  lg.state, lg.button, lg.burst_sent, lg.total_sent, lg.last_frame = ar.SENDING, ar.RES, 1, 1, 5
  lg._prev_frame_state = (0, 0, -10_000)
  lg.unsend()
  assert (lg.burst_sent, lg.total_sent, lg.last_frame) == (0, 0, -10_000)
  lg.unsend()                                   # nothing to take back twice
  assert (lg.burst_sent, lg.total_sent) == (0, 0)


# ---------------- the caller: one sender ----------------

def _files_to(tmp_path, monkeypatch):
  import openpilot.starpilot.car.mazda.speed.events as E
  import openpilot.starpilot.car.mazda.speed.car_profile as P
  monkeypatch.setattr(E, "LOG_DIR", str(tmp_path / "log"))
  monkeypatch.setattr(P, "PROFILE_PATH", str(tmp_path / "profile.json"))
  monkeypatch.setattr(P.CarProfile.__init__, "__defaults__", (str(tmp_path / "profile.json"), "", False, P.time.monotonic))


class FakeSources:
  want_view = True

  def __init__(self, view=True, engageable=True):
    self.view, self.engageable = view, engageable

  def context(self, d):
    now = time.monotonic()
    view = {"v": 2, "t": now, "gps": "ok", "trust": 0, "matched": True, "cls": 5, "chain": 7} if self.view else None
    return dict(d, now=now, view=view, lead=None, engageable=self.engageable, contribs={})


class Drive:
  """Card frames at 100 Hz into caller.update(): the wheel's CRZ_BTNS counter steps every 10 frames."""
  def __init__(self, params, cp, sources=None):
    self.card = N(params=params, CP=cp, CI=CarInterface(cp), is_metric=False)
    self.ocs = self.card.CI.CS
    self.ocs.crz_set_allowed = True
    self.frame = 0
    self.state = dict(enabled=True, speed=45 * MPH, v=45 * MPH, gas=False, brake=False)
    self.cancel = False
    self.sent = []
    ctl = caller.make(self.card)
    self.card.mazda_speed = ctl
    if ctl and sources is not None:
      ctl.sources = sources

  def step(self, n=1, **kw):
    self.state.update(kw)
    s = self.state
    for _ in range(n):
      self.frame += 1
      self.ocs.crz_btns_counter = (self.frame // 10) % 16
      cs = N(cruiseState=N(enabled=s["enabled"], available=True, speed=s["speed"], standstill=False), vEgo=s["v"],
             gasPressed=s["gas"], brakePressed=s["brake"], standstill=False, leftBlinker=False, rightBlinker=False,
             steeringAngleDeg=0.0, steeringRateDeg=0.0, steeringPressed=False, gearShifter=structs.CarState.GearShifter.drive,
             doorOpen=False, seatbeltUnlatched=False, canValid=True)
      cc = N(enabled=s["enabled"], latActive=s["enabled"], cruiseControl=N(cancel=self.cancel, resume=False),
             actuators=N(torque=0.0), hudControl=N(visualAlert="none"))
      frames_in = []
      out = caller.update(self.card, cs, cc, frames_in, 0)
      assert len(out) <= 1
      if out:
        self.sent.append((self.frame, out[0], self.ocs.crz_btns_counter))

  def brake_cancel(self):
    self.step(300, enabled=True, gas=False, brake=False)
    self.step(3, brake=True)
    self.step(1, enabled=False)
    self.step(100)
    self.step(1, brake=False)


def _ctr(frame):
  parser = CANParser("mazda_2017", [("CRZ_BTNS", 10)], 0)
  parser.update([(1, [(frame[0], frame[1], 0)])])
  return int(parser.vl["CRZ_BTNS"]["CTR"]), int(parser.vl["CRZ_BTNS"]["RES"])


def test_caller_sends_our_res_through_the_one_actuator(tmp_path, monkeypatch):
  _files_to(tmp_path, monkeypatch)
  with tempfile.TemporaryDirectory() as d:
    params = saved_params(d, {"version": 5, "enabled": True, "auto_resume": True})
    cp = started(params, ti_cp())
    drive = Drive(params, cp, FakeSources())
    assert drive.card.mazda_speed is not False and drive.card.mazda_speed.auto_resume is not None
    ctl = drive.card.mazda_speed
    drive.brake_cancel()
    anchor = ctl.planner.anchor
    assert drive.sent == [] and anchor is not None and ctl.auto_resume.state == ar.ARMED
    for _ in range(150):
      drive.step(1, gas=True)
      if drive.sent:
        break
    assert len(drive.sent) == 1
    # the car does not answer at once: the burst goes on, every frame RES alone right after a new wheel frame
    drive.step(25)
    assert 2 <= len(drive.sent) <= ar.BURST_FRAMES
    for frame, can, wheel in drive.sent:
      assert can[0] == 0x09D and bytes(can[1])[0] == 0x04          # RES alone: what mazda.h lets through
      ctr, res = _ctr(can)
      assert res == 1 and ctr == (wheel + 1) % 16 and frame % 10 == 0   # CTR = wheel + 1
    # the car takes it: the cruise comes back at his set speed; the planner keeps his anchor (as after his RES)
    n = len(drive.sent)
    drive.step(100, enabled=True, gas=False)
    assert ctl.auto_resume.state == ar.DONE and len(drive.sent) == n and ctl.planner.anchor == anchor
    # no new episode of anything: the actuator did not press after it (nothing asked)
    drive.step(100)
    assert len(drive.sent) == n
    # never more than the panda's 8 frames when the car never answers
    drive2 = Drive(drive.card.params, cp, FakeSources())
    drive2.brake_cancel()
    drive2.step(600, gas=True)
    assert len(drive2.sent) == ar.MAX_FRAMES and drive2.card.mazda_speed.auto_resume.state == ar.GAVE_UP


def test_caller_not_without_the_panda_bit_or_with_openpilots_cancel(tmp_path, monkeypatch):
  _files_to(tmp_path, monkeypatch)
  with tempfile.TemporaryDirectory() as d:
    params = saved_params(d, {"version": 5, "enabled": True, "auto_resume": True})
    # the setting on but the car started without the bit (e.g. set after the car started): nothing
    drive = Drive(params, ti_cp(), FakeSources())
    assert drive.card.mazda_speed is not False and drive.card.mazda_speed.auto_resume is None
    drive.brake_cancel()
    drive.step(200, gas=True)
    assert drive.sent == []
    # openpilot asks for CANCEL: none of ours
    drive = Drive(params, started(params, ti_cp()), FakeSources())
    drive.brake_cancel()
    drive.cancel = True
    drive.step(200, gas=True)
    assert drive.sent == []
    # no map view: none
    drive = Drive(params, started(params, ti_cp()), FakeSources(view=False))
    drive.brake_cancel()
    drive.step(200, gas=True)
    assert drive.sent == [] and drive.card.mazda_speed.auto_resume.wait_why == "map"
