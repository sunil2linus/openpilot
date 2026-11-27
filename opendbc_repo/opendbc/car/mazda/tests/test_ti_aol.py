"""Always On Lateral with the Torque Interceptor: qualification (ti_aol.py) and the car code while steering with the cruise
off (carstate, carcontroller with real CANPacker / CANParser, the frames checked by the built panda safety)."""
import pytest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda import ti_aol
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.speed_flags import prepare as prepare_speed
from opendbc.car.mazda.stock_aol import aol_qualified, qualified as stock_qualified, temporary_restriction
from opendbc.car.mazda.torque_interceptor import prepare
from opendbc.car.mazda.values import CAR, MazdaFlags, MazdaSafetyFlags, TI_STATE
from opendbc.car.mazda.tests.test_torque_interceptor import control, run, sent, ti_feedback, lkas_torque

TI = int(MazdaSafetyFlags.TORQUE_INTERCEPTOR)
TI_AOL = int(MazdaSafetyFlags.TI_AOL)
Fixed = structs.CarParams.FingerprintSource.fixed


def ti_params(model=CAR.MAZDA_CX5, *, auto_resume=False, steer_700=False):
  cp = CarInterface.get_params(model, gen_empty_fingerprint(), [], False, False, False)
  return prepare_speed(prepare(cp, True), auto_resume_requested=auto_resume, steer_700_requested=steer_700)


def mark(cp):
  """What card does at car start for a qualified car with Always On Lateral on."""
  cp.safetyConfigs[0].safetyParam |= ti_aol.ADDITION
  cp.alternativeExperience |= 32
  return cp


def test_flag_value():
  assert TI_AOL == 1024 and not TI_AOL & (TI | int(MazdaSafetyFlags.AUTO_RESUME) | int(MazdaSafetyFlags.TI_STEER_700))
  assert not any(int(f) & TI_AOL for f in MazdaFlags)


@pytest.mark.parametrize("model", list(CAR))
@pytest.mark.parametrize("auto_resume, steer_700", ((False, False), (True, False), (False, True), (True, True)))
def test_qualified_unmarked_and_marked(model, auto_resume, steer_700):
  cp = ti_params(model, auto_resume=auto_resume, steer_700=steer_700)
  cp.fingerprintSource = Fixed   # the owner's car is chosen in settings
  assert ti_aol.qualified(cp) and not ti_aol.qualified(cp, marked_only=True)
  assert aol_qualified(cp) and not stock_qualified(cp)
  mark(cp)
  assert ti_aol.qualified(cp) and ti_aol.qualified(cp, marked_only=True) and aol_qualified(cp, marked_only=True)
  assert ti_aol.native_param(int(cp.safetyConfigs[0].safetyParam))
  assert not stock_qualified(cp, marked_only=True)


@pytest.mark.parametrize("change", (
  "no_ti", "passive", "notCar", "dashcamOnly", "manual", "no_pcm", "op_long", "two_configs", "wrong_model",
  "unknown_bit", "flag_extra", "brand", "aol_bit_without_32", "32_without_aol_bit", "alt_other",
))
def test_not_qualified(change):
  cp = ti_params()
  if change == "no_ti":
    cp = CarInterface.get_params(CAR.MAZDA_CX5, gen_empty_fingerprint(), [], False, False, False)
  elif change == "passive":
    cp.passive = True
  elif change == "notCar":
    cp.notCar = True
  elif change == "dashcamOnly":
    cp.dashcamOnly = True
  elif change == "manual":
    cp.transmissionType = structs.CarParams.TransmissionType.manual
  elif change == "no_pcm":
    cp.pcmCruise = False
  elif change == "op_long":
    cp.openpilotLongitudinalControl = True
  elif change == "two_configs":
    first = cp.safetyConfigs[0]
    cp.safetyConfigs = [structs.CarParams.SafetyConfig(safetyModel=first.safetyModel, safetyParam=first.safetyParam),
                        structs.CarParams.SafetyConfig(safetyModel=first.safetyModel, safetyParam=first.safetyParam)]
  elif change == "wrong_model":
    cp.safetyConfigs[0].safetyModel = structs.CarParams.SafetyModel.noOutput
  elif change == "unknown_bit":
    cp.safetyConfigs[0].safetyParam |= 2048
  elif change == "flag_extra":
    cp.flags |= 2
  elif change == "brand":
    cp.brand = "toyota"
  elif change == "aol_bit_without_32":
    cp.safetyConfigs[0].safetyParam |= TI_AOL
  elif change == "32_without_aol_bit":
    cp.alternativeExperience = 32
  elif change == "alt_other":
    cp.alternativeExperience = 96
  assert not ti_aol.qualified(cp) and not ti_aol.qualified(cp, marked_only=True)
  assert not ti_aol.qualified(mark(cp.copy()), marked_only=True) or change in ("32_without_aol_bit", "aol_bit_without_32")


def test_stock_qualification_unchanged():
  # the stock cars are still stock-only, and never TI
  for model in CAR:
    cp = CarInterface.get_params(model, gen_empty_fingerprint(), [], False, False, False)
    assert stock_qualified(cp) == (model in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021))
    assert not ti_aol.qualified(cp)


def test_temporary_restriction_for_ti():
  cp = mark(ti_params())
  cs = structs.CarState(canValid=True, gearShifter=structs.CarState.GearShifter.drive, vEgo=10.)
  assert not temporary_restriction(cp, cs)
  # minSteerSpeed is 0 with a TI: slow is no restriction
  cs.vEgo = 2.
  assert not temporary_restriction(cp, cs)
  for field, value in (("standstill", True), ("steerFaultTemporary", True), ("gearShifter", structs.CarState.GearShifter.reverse)):
    state = structs.CarState(**cs.to_dict())
    setattr(state, field, value)
    assert temporary_restriction(cp, state), field
  # not before car start marking, and not for an unqualified car
  assert not temporary_restriction(ti_params(), structs.CarState(canValid=True, standstill=True))


def test_carstate_lkas_block_is_no_fault_with_the_ti_running_cruise_off():
  packer = CANPacker("mazda_2017")
  ci = CarInterface(mark(ti_params()))
  state = run(ci, packer, cruise=0, speed=20.0, lkas_block=1, lane_lines=0, ti=ti_feedback(packer))
  assert not state.cruiseState.enabled and state.cruiseState.available
  assert not state.steerFaultTemporary and not state.steerFaultPermanent and not state.invalidLkasSetting
  state = run(ci, packer, cruise=0, speed=20.0, lkas_block=1, ti=ti_feedback(packer, state=TI_STATE.OFF))
  assert state.steerFaultTemporary
  # no cancel button events for a TI car (unchanged)
  assert not [e for e in state.buttonEvents if e.type == structs.CarState.ButtonEvent.Type.cancel]


def _safety(param):
  from opendbc.safety.tests.libsafety import libsafety_py
  from opendbc.safety.tests.common import CANPackerSafety
  safety = libsafety_py.libsafety
  safety.init_tests()
  safety.set_alternative_experience(32)
  safety.set_safety_hooks(structs.CarParams.SafetyModel.mazda, param)
  safety.set_aol_test_heartbeat(True)
  packer = CANPackerSafety("mazda_2017")
  now = [1_000_000]

  def feed(main=True):
    now[0] += 10_000
    safety.set_timer(now[0])
    for name, bus, fields in (
      ('CRZ_CTRL', 0, {'CRZ_AVAILABLE': int(main), 'CRZ_ACTIVE': 0}), ('CRZ_BTNS', 0, {}), ('STEER_TORQUE', 0, {}),
      ('ENGINE_DATA', 0, {'SPEED': 3000}), ('PEDALS', 0, {}), ('TI_FEEDBACK', 1, {'TI_TORQUE_SENSOR': 0}),
      ('GEAR', 0, {'GEAR': 4}), ('STEER_RATE', 0, {'LKAS_BLOCK': 1}), ('CAM_LKAS', 2, {})):
      safety.safety_rx_hook(packer.make_can_msg_safety(name, bus, fields))
    safety.aol_set_host_request(1)
  for _ in range(10):
    feed()
  feed(main=False)
  feed()
  assert safety.aol_get_permission_mask() == 1 and not safety.get_controls_allowed()
  return safety, feed, libsafety_py


def test_carcontroller_frames_with_the_cruise_off_pass_the_panda():
  """latActive with the cruise off (CC.enabled False): CAM_LKAS and CAM_LKAS2 carry the same torque, the HUD is sent, no
  button is pressed; every frame passes the TI AOL panda safety; with latActive False only zero torque goes out."""
  packer = CANPacker("mazda_2017")
  cp = mark(ti_params())
  ci = CarInterface(cp)
  run(ci, packer, cruise=0, speed=30.0, lkas_block=1, ti=ti_feedback(packer))
  safety, feed, lib = _safety(int(cp.safetyConfigs[0].safetyParam))
  huds = 0
  last = 0
  for frame in range(120):
    CC = control(latActive=frame < 100, torque=0.5)
    CC.enabled = False
    actuators, can_sends = ci.apply(CC.as_reader(), (frame + 10) * 10_000_000)
    lkas, ti = sent(can_sends, 0x243, 0), sent(can_sends, 0x249, 1)
    assert len(lkas) == 1 and len(ti) == 1 and lkas_torque(lkas[0]) == lkas_torque(ti[0]) == actuators.torqueOutputCan
    assert not sent(can_sends, 0x09D, 0)
    huds += len(sent(can_sends, 0x440, 0))
    feed()
    for addr, dat, bus in can_sends:
      assert safety.safety_tx_hook(lib.make_CANPacket(addr, bus, dat)), (frame, hex(addr))
    last = actuators.torqueOutputCan
    if frame < 100:
      assert actuators.torqueOutputCan == min(300, 10 * (frame + 1))
  assert last == 0 and huds >= 2
