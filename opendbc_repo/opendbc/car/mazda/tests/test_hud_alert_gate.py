"""Tests for the Mazda TI dash HUD alert speed gate (old build)."""
from types import SimpleNamespace

import pytest

pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)

import cereal.messaging as messaging
from cereal import custom
from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda import carcontroller
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState, stock_lkas_speed_allowed
from opendbc.car.mazda.values import CAR, LKAS_LIMITS, MazdaSafetyFlags, TI_STATE

VisualAlert = structs.CarControl.HUDControl.VisualAlert


def test_stock_lkas_speed_allowed_hysteresis():
  assert not stock_lkas_speed_allowed(False, 35.0, True)
  assert not stock_lkas_speed_allowed(False, 48.0, True)
  assert not stock_lkas_speed_allowed(False, 55.0, True)
  assert stock_lkas_speed_allowed(False, 55.0, False)
  assert stock_lkas_speed_allowed(True, 48.0, False)
  assert stock_lkas_speed_allowed(True, 48.0, True)
  assert not stock_lkas_speed_allowed(True, 40.0, True)


def _decode_hud(can_sends, hud_parser, t_ns):
  msgs = [(addr, dat, bus) for addr, dat, bus in can_sends if addr == 0x440 and bus == 0]
  assert len(msgs) == 1
  hud_parser.update([(t_ns, msgs)])
  return hud_parser.vl["CAM_LANEINFO"]


def _feed(cs, parsers, packer, tick, *, speed=35.0, lkas_block=1, lane_lines=1, ti_state=TI_STATE.RUN, with_ti=True):
  t_ns = tick * 10_000_000
  msgs = [
    packer.make_can_msg("ENGINE_DATA", 0, {"SPEED": speed}),
    packer.make_can_msg("STEER_RATE", 0, {"LKAS_BLOCK": lkas_block}),
    packer.make_can_msg("STEER_TORQUE", 0, {"STEER_TORQUE_SENSOR": 0}),
    packer.make_can_msg("CRZ_CTRL", 0, {"CRZ_ACTIVE": 1, "CRZ_AVAILABLE": 1}),
    packer.make_can_msg("CAM_LANEINFO", 2, {"LANE_LINES": lane_lines}),
  ]
  if with_ti:
    msgs.append(packer.make_can_msg("TI_FEEDBACK", 1, {"TI_TORQUE_SENSOR": 0, "STATE": ti_state,
                                                       "VERSION_NUMBER": 2, "RAMP_DOWN": 0}))
  for cp in parsers.values():
    if cp is not None:
      cp.update([(t_ns, msgs)])
  ret, _ = cs.update(parsers, None)
  cs.out = ret
  return ret, t_ns


def _make_cp(model, flags, min_steer_speed):
  CP = structs.CarParams()
  CP.carFingerprint = model
  CP.flags = int(flags)
  CP.minSteerSpeed = min_steer_speed
  CP.openpilotLongitudinalControl = False
  return CP


def test_carcontroller_ti_hides_dash_steer_required_below_stock_lkas_speed(monkeypatch):
  class StubParams:
    def __init__(self, *a, **k):
      pass

    def get(self, key, encoding=None):
      return None

    def get_bool(self, key):
      return False

  monkeypatch.setattr(carcontroller, "Params", StubParams)
  monkeypatch.setattr(messaging, "SubMaster", lambda services: SimpleNamespace(update=lambda t: None))

  packer = CANPacker("mazda_2017")
  hud_parser = CANParser("mazda_2017", [("CAM_LANEINFO", float("nan"))], 0)
  CC = structs.CarControl()
  CC.latActive = True
  CC.hudControl.visualAlert = VisualAlert.steerRequired
  cc_reader = CC.as_reader()

  # Gen1 + TI (minSteerSpeed = 0): suppress dash steer_required below stock LKAS speed, keep low-speed TI fault detection
  cp_ti = _make_cp(CAR.MAZDA_CX5, MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR, 0.0)
  cs = CarState(cp_ti, custom.StarPilotCarParams.new_message())
  parsers = CarState.get_can_parsers(cp_ti)
  cc = CarController({Bus.pt: "mazda_2017"}, cp_ti)

  tick = 1
  for speed, blk, want_warn in ((35.0, 1, 0), (48.0, 1, 0), (55.0, 1, 0), (55.0, 0, 1), (48.0, 0, 1), (40.0, 1, 0)):
    _, t_ns = _feed(cs, parsers, packer, tick, speed=speed, lkas_block=blk)
    tick += 1
    assert cs.lkas_allowed_speed is True
    assert cs.hud_lkas_allowed_speed is bool(want_warn)
    cc.frame = 0
    _, can_sends = cc.update(cc_reader, cs, t_ns, None)
    hud = _decode_hud(can_sends, hud_parser, t_ns)
    assert hud["HANDS_ON_STEER_WARN"] == want_warn
    assert hud["HANDS_ON_STEER_WARN_2"] == want_warn
    assert hud["HANDS_WARN_3_BITS"] == (0b111 if want_warn else 0)

  ret, _ = _feed(cs, parsers, packer, tick, speed=20.0, lkas_block=1, ti_state=TI_STATE.OFF)
  assert ret.steerFaultTemporary

  # Non-TI Gen1 CX-5 2022+ (minSteerSpeed = 0): dash alert stays enabled at low speed
  cp_22 = _make_cp(CAR.MAZDA_CX5_2022, MazdaSafetyFlags.GEN1, 0.0)
  cs22 = CarState(cp_22, custom.StarPilotCarParams.new_message())
  parsers22 = CarState.get_can_parsers(cp_22)
  cc22 = CarController({Bus.pt: "mazda_2017"}, cp_22)
  _, t_ns = _feed(cs22, parsers22, packer, 1, speed=35.0, lkas_block=1, with_ti=False)
  assert cs22.hud_lkas_allowed_speed is True
  cc22.frame = 0
  _, can_sends22 = cc22.update(cc_reader, cs22, t_ns, None)
  assert _decode_hud(can_sends22, hud_parser, t_ns)["HANDS_ON_STEER_WARN"] == 1

  # Stock Gen1 CX-5 (minSteerSpeed > 0): follows stock LKAS speed gate as before
  cp_st = _make_cp(CAR.MAZDA_CX5, MazdaSafetyFlags.GEN1, LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS)
  cs_st = CarState(cp_st, custom.StarPilotCarParams.new_message())
  parsers_st = CarState.get_can_parsers(cp_st)
  cc_st = CarController({Bus.pt: "mazda_2017"}, cp_st)
  for idx, (speed, blk, want_warn) in enumerate(((35.0, 1, 0), (55.0, 0, 1)), start=1):
    _, t_ns = _feed(cs_st, parsers_st, packer, idx, speed=speed, lkas_block=blk, with_ti=False)
    cc_st.frame = 0
    _, can_sends_st = cc_st.update(cc_reader, cs_st, t_ns, None)
    assert _decode_hud(can_sends_st, hud_parser, t_ns)["HANDS_ON_STEER_WARN"] == want_warn
