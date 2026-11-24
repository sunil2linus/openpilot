import copy

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.torque_interceptor import installed, prepare, supported
from opendbc.car.mazda.values import CAR, CarControllerParams, LKAS_LIMITS, MazdaFlags, MazdaSafetyFlags, TI_STATE

KEY = 3294744160


def stock_params(model):
  return CarInterface.get_params(model, gen_empty_fingerprint(), [], False, False, False)


def ti_params(model):
  return prepare(stock_params(model), True)


@pytest.mark.parametrize("model", list(CAR))
def test_not_requested_is_stock(model):
  stock = stock_params(model)
  prepared = prepare(stock_params(model), False)
  assert prepared.to_dict() == stock.to_dict()
  assert int(prepared.flags) == int(MazdaFlags.GEN1)
  assert prepared.safetyConfigs[0].safetyParam == 0
  assert not installed(prepared)
  assert prepared.dashcamOnly == (model not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021))
  params = CarControllerParams(prepared)
  assert (params.STEER_MAX, params.STEER_DRIVER_MULTIPLIER) == (800, 1)
  assert Bus.body not in CarInterface.CarState.get_can_parsers(prepared)


@pytest.mark.parametrize("model", list(CAR))
def test_requested(model):
  cp = ti_params(model)
  assert int(cp.flags) == int(MazdaFlags.GEN1 | MazdaFlags.TORQUE_INTERCEPTOR)
  assert cp.safetyConfigs[0].safetyParam == int(MazdaSafetyFlags.TORQUE_INTERCEPTOR) == 8
  assert cp.minSteerSpeed == 0.0
  assert not cp.dashcamOnly
  assert not cp.steerAtStandstill
  assert installed(cp)
  # idempotent and the rest of CarParams is stock
  again = prepare(copy.deepcopy(cp), True)
  assert again.to_dict() == cp.to_dict()
  stock = stock_params(model).to_dict()
  changed = {k for k, v in cp.to_dict().items() if stock[k] != v}
  assert changed <= {"flags", "safetyConfigs", "minSteerSpeed", "dashcamOnly"}
  # lateral tuning is the stock one (torque, CX-9 2021 substitute data)
  assert cp.lateralTuning.to_dict() == stock["lateralTuning"]
  params = CarControllerParams(cp)
  assert (params.STEER_MAX, params.STEER_DELTA_UP, params.STEER_DELTA_DOWN, params.STEER_DRIVER_ALLOWANCE,
          params.STEER_DRIVER_MULTIPLIER) == (600, 10, 25, 15, 40)


def test_not_applied_to_other_configurations():
  cp = stock_params(CAR.MAZDA_CX5)
  cp.safetyConfigs[0].safetyParam = 1
  assert not supported(cp) and not installed(prepare(cp, True))
  cp = stock_params(CAR.MAZDA_CX5)
  cp.notCar = True
  assert not installed(prepare(cp, True))
  cp = stock_params(CAR.MAZDA_CX5)
  cp.brand = "toyota"
  assert not installed(prepare(cp, True))


def ti_feedback(packer, torque=0, state=TI_STATE.RUN, version=2, ramp_down=0):
  return packer.make_can_msg("TI_FEEDBACK", 1, {"TI_TORQUE_SENSOR": torque, "STATE": state, "VERSION_NUMBER": version,
                                                "RAMP_DOWN": ramp_down})


def frames(packer, *, steer_torque=0, lkas_block=0, lane_lines=1, cruise=1, speed=30.0):
  return [
    packer.make_can_msg("STEER_TORQUE", 0, {"STEER_TORQUE_SENSOR": steer_torque}),
    packer.make_can_msg("STEER_RATE", 0, {"LKAS_BLOCK": lkas_block}),
    packer.make_can_msg("ENGINE_DATA", 0, {"SPEED": speed}),
    packer.make_can_msg("CRZ_CTRL", 0, {"CRZ_ACTIVE": cruise, "CRZ_AVAILABLE": 1}),
    packer.make_can_msg("CAM_LANEINFO", 2, {"LANE_LINES": lane_lines}),
  ]


def run(ci, packer, ticks=5, ti=None, **kwargs):
  state = None
  for tick in range(1, ticks + 1):
    msgs = frames(packer, **kwargs) + ([ti] if ti is not None else [])
    state = ci.update([(tick * 10_000_000, msgs)])
  return state


def test_carstate_ti_driver_torque_and_faults():
  packer = CANPacker("mazda_2017")
  ci = CarInterface(ti_params(CAR.MAZDA_CX5))
  assert ci.can_parsers[Bus.body].bus == 1
  # driver torque comes from the TI sensor, not from the EPS
  state = run(ci, packer, steer_torque=-100, ti=ti_feedback(packer, torque=LKAS_LIMITS.TI_STEER_THRESHOLD + 1))
  assert state.steeringTorque == LKAS_LIMITS.TI_STEER_THRESHOLD + 1 and state.steeringPressed
  state = run(ci, packer, steer_torque=-100, ti=ti_feedback(packer, torque=LKAS_LIMITS.TI_STEER_THRESHOLD))
  assert not state.steeringPressed
  # low speed with stock LKAS blocked and lane lines off: the TI is running, so no fault and no low speed alert
  state = run(ci, packer, speed=20.0, lkas_block=1, lane_lines=0, ti=ti_feedback(packer))
  assert not state.steerFaultTemporary and not state.invalidLkasSetting and not state.lowSpeedAlert
  # TI not running (or ramping down) while LKAS is blocked: temporary fault
  state = run(ci, packer, speed=20.0, lkas_block=1, ti=ti_feedback(packer, state=TI_STATE.OFF))
  assert state.steerFaultTemporary
  state = run(ci, packer, speed=20.0, lkas_block=1, ti=ti_feedback(packer, ramp_down=1))
  assert state.steerFaultTemporary


def test_carstate_stock_unchanged():
  packer = CANPacker("mazda_2017")
  ci = CarInterface(stock_params(CAR.MAZDA_CX5))
  state = run(ci, packer, steer_torque=LKAS_LIMITS.STEER_THRESHOLD + 1, ti=ti_feedback(packer, torque=50))
  assert state.steeringTorque == LKAS_LIMITS.STEER_THRESHOLD + 1 and state.steeringPressed
  state = run(ci, packer, lane_lines=0)
  assert state.invalidLkasSetting


def control(latActive=True, torque=1.0, visual_alert=structs.CarControl.HUDControl.VisualAlert.none):
  CC = structs.CarControl()
  CC.latActive = latActive
  CC.actuators.torque = torque
  CC.hudControl.visualAlert = visual_alert
  return CC


def sent(can_sends, addr, bus):
  return [dat for a, dat, b in can_sends if a == addr and b == bus]


def lkas_torque(dat):
  return (((dat[0] & 0x0F) << 8) | dat[1]) - 2048


def test_carcontroller_ti_sends_same_command_limited_to_600():
  packer = CANPacker("mazda_2017")
  ci = CarInterface(ti_params(CAR.MAZDA_CX5))
  run(ci, packer, ti=ti_feedback(packer))
  parser = CANParser("mazda_2017", [("CAM_LKAS2", float("nan"))], 1)
  last = None
  for frame in range(100):
    actuators, can_sends = ci.apply(control().as_reader(), (frame + 10) * 10_000_000)
    lkas, ti = sent(can_sends, 0x243, 0), sent(can_sends, 0x249, 1)
    assert len(lkas) == 1 and len(ti) == 1
    assert lkas_torque(lkas[0]) == lkas_torque(ti[0]) == actuators.torqueOutputCan
    parser.update([((frame + 10) * 10_000_000, [(0x249, ti[0], 1)])])
    assert parser.vl["CAM_LKAS2"]["LKAS_REQUEST"] == actuators.torqueOutputCan
    assert parser.vl["CAM_LKAS2"]["CHKSUM"] == actuators.torqueOutputCan
    assert parser.vl["CAM_LKAS2"]["KEY"] == KEY
    last = actuators.torqueOutputCan
  assert last == 600  # ramped at 10 per frame up to the TI limit
  # the TI message is never sent on the main or camera bus
  assert not sent(can_sends, 0x249, 0) and not sent(can_sends, 0x249, 2)


def test_carcontroller_stock_no_ti_message():
  packer = CANPacker("mazda_2017")
  ci = CarInterface(stock_params(CAR.MAZDA_CX5_2022))
  run(ci, packer)
  last = None
  for frame in range(100):
    actuators, can_sends = ci.apply(control().as_reader(), (frame + 10) * 10_000_000)
    assert not [m for m in can_sends if m[0] == 0x249]
    last = actuators.torqueOutputCan
  assert last == 800


def test_carcontroller_ti_hides_dash_steer_required_below_stock_lkas_speed():
  packer = CANPacker("mazda_2017")
  hud_parser = CANParser("mazda_2017", [("CAM_LANEINFO", float("nan"))], 0)
  alert = structs.CarControl.HUDControl.VisualAlert.steerRequired
  tick = [1]

  def apply_hud(ci, **kwargs):
    msgs = frames(packer, **kwargs) + ([ti_feedback(packer)] if ci.CS.torque_interceptor else [])
    t_ns = tick[0] * 10_000_000
    tick[0] += 1
    state = ci.update([(t_ns, msgs)])
    ci.CC.frame = 0
    _, can_sends = ci.apply(control(visual_alert=alert).as_reader(), t_ns)
    hud = sent(can_sends, 0x440, 0)
    assert len(hud) == 1
    hud_parser.update([(t_ns, [(0x440, hud[0], 0)])])
    return state, hud_parser.vl["CAM_LANEINFO"]

  # TI car (minSteerSpeed = 0): dash steer_required chime suppressed below stock LKAS speed while
  # lkas_allowed_speed stays True so low-speed TI-dropout steerFaultTemporary still works.
  ci = CarInterface(ti_params(CAR.MAZDA_CX5))
  for speed, blk, want in ((35.0, 1, 0), (48.0, 1, 0), (55.0, 1, 0), (55.0, 0, 1), (48.0, 0, 1), (40.0, 1, 0)):
    _, hud = apply_hud(ci, speed=speed, lkas_block=blk)
    assert ci.CS.lkas_allowed_speed is True
    assert ci.CS.hud_lkas_allowed_speed is bool(want)
    assert hud["HANDS_ON_STEER_WARN"] == want
    assert hud["HANDS_ON_STEER_WARN_2"] == want
    assert hud["HANDS_WARN_3_BITS"] == (0b111 if want else 0)
  state = ci.update([(tick[0] * 10_000_000, frames(packer, speed=20.0, lkas_block=1) +
                      [ti_feedback(packer, state=TI_STATE.OFF)])])
  tick[0] += 1
  assert state.steerFaultTemporary

  # Stock CX-5 2022+ (no TI, minSteerSpeed = 0): dash chime stays active at low speed
  ci_22 = CarInterface(stock_params(CAR.MAZDA_CX5_2022))
  _, hud_22 = apply_hud(ci_22, speed=35.0, lkas_block=1)
  assert ci_22.CS.hud_lkas_allowed_speed is True
  assert hud_22["HANDS_ON_STEER_WARN"] == 1

  # Stock Gen1 CX-5 (no TI, minSteerSpeed > 0): keeps stock speed hysteresis
  ci_st = CarInterface(stock_params(CAR.MAZDA_CX5))
  _, hud_low = apply_hud(ci_st, speed=35.0, lkas_block=1)
  assert hud_low["HANDS_ON_STEER_WARN"] == 0
  _, hud_hi = apply_hud(ci_st, speed=55.0, lkas_block=0)
  assert hud_hi["HANDS_ON_STEER_WARN"] == 1
