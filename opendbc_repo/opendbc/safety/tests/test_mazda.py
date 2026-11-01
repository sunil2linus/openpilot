#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.mazda.values import MazdaSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  # Gen1 (2017-2021 CX-5 / CX-9 / Mazda3 / Mazda6), stock MRCC, no interceptors
  SAFETY_PARAM = MazdaSafetyFlags.GEN1

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  FWD_BLACKLISTED_ADDRS = {2: [0x243, 0x440]}

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [800]

  MAX_RT_DELTA = 300

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 1

  # Mazda actually does not set any bit when requesting torque
  NO_STEER_REQ_BIT = True

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, int(self.SAFETY_PARAM))
    self.safety.init_tests()

  def _torque_meas_msg(self, torque):
    values = {"STEER_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"STEER_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_REQUEST": torque}
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_ON": brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRZ_ACTIVE": enable}
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, values)

  def _button_msg(self, resume=False, cancel=False):
    values = {
      "CAN_OFF": cancel,
      "CAN_OFF_INV": (cancel + 1) % 2,
      "RES": resume,
      "RES_INV": (resume + 1) % 2,
    }
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)

  def test_buttons(self):
    # only cancel allows while controls not allowed
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertTrue(self._tx(self._button_msg(resume=True)))

  def _toggle_aol(self, toggle_on):
    # CRZ_CTRL, CRZ_AVAILABLE is the main on button
    values = {"CRZ_AVAILABLE": 1 if toggle_on else 0}
    return self.packer.make_can_msg_panda("CRZ_CTRL", 0, values)


class TestMazdaTorqueInterceptorSafety(TestMazdaSafety):
  """Gen1 + MoreTorque Torque Interceptor (TI). The TI command CAM_LKAS2 (0x249) is sent on bus 1, driver torque
  comes from TI_FEEDBACK (0x24A) on bus 1, and both steering messages are limited to 600 (carcontroller STEER_MAX)."""
  SAFETY_PARAM = MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR
  TX_MSGS = [[0x243, 0], [0x249, 1], [0x09d, 0], [0x440, 0]]
  MAX_TORQUE_LOOKUP = [0], [600]

  def _torque_driver_msg(self, torque):
    values = {"TI_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("TI_FEEDBACK", 1, values)

  def _ti_cmd_msg(self, torque):
    values = {"LKAS_REQUEST": torque, "CHKSUM": torque, "KEY": 3294744160}
    return self.packer.make_can_msg_safety("CAM_LKAS2", 1, values)

  def test_ti_cmd_same_as_lkas(self):
    # carcontroller sends the same value on CAM_LKAS and CAM_LKAS2 every frame: both must pass
    # ramp up at 100 Hz like carcontroller: both messages carry the same value in the same frame
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(0)
    self._set_prev_torque(0)
    torque = 0
    for frame in range(1, 81):
      self.safety.set_timer(frame * 10000)
      torque = min(torque + self.MAX_RATE_UP, 600)
      self.assertTrue(self._tx(self._torque_cmd_msg(torque)), f"CAM_LKAS {torque=}")
      self.assertTrue(self._tx(self._ti_cmd_msg(torque)), f"CAM_LKAS2 {torque=}")

  def test_ti_cmd_blocked_when_not_allowed(self):
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._ti_cmd_msg(0)))
    self.assertFalse(self._tx(self._ti_cmd_msg(self.MAX_RATE_UP)))

  def test_ti_cmd_max_torque(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(600)
    self.assertTrue(self._tx(self._ti_cmd_msg(600)))
    self._set_prev_torque(600)
    self.assertFalse(self._tx(self._ti_cmd_msg(601)))
    self._set_prev_torque(-600)
    self.assertTrue(self._tx(self._ti_cmd_msg(-600)))
    self._set_prev_torque(-600)
    self.assertFalse(self._tx(self._ti_cmd_msg(-601)))

  def test_ti_cmd_rate_limit(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._ti_cmd_msg(self.MAX_RATE_UP + 1)))
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._ti_cmd_msg(self.MAX_RATE_UP)))

  def test_ti_cmd_only_on_bus_1(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    for bus in (0, 2):
      values = {"LKAS_REQUEST": 0, "CHKSUM": 0, "KEY": 3294744160}
      self.assertFalse(self._tx(self.packer.make_can_msg_safety("CAM_LKAS2", bus, values)))

  def test_stock_eps_torque_is_not_driver_torque(self):
    # with a TI the driver torque comes from the TI sensor, not from the EPS STEER_TORQUE message
    self.safety.set_controls_allowed(True)
    for _ in range(6):
      self._rx(self._torque_driver_msg(0))
      self._rx(self.packer.make_can_msg_safety("STEER_TORQUE", 0, {"STEER_TORQUE_SENSOR": -100}))
    self._set_prev_torque(600)
    self.assertTrue(self._tx(self._torque_cmd_msg(600)))


class TestMazdaTorqueInterceptorCmdSafety(TestMazdaTorqueInterceptorSafety):
  """Runs the generic torque steering tests on the TI command (CAM_LKAS2, bus 1) instead of CAM_LKAS."""
  def _torque_cmd_msg(self, torque, steer_req=1):
    return self._ti_cmd_msg(torque)


def _rx_checks_valid(param, include_crz_ctrl, include_ti):
  packer = CANPackerSafety("mazda_2017")
  safety = libsafety_py.libsafety
  safety.set_safety_hooks(CarParams.SafetyModel.mazda, int(param))
  safety.init_tests()
  msgs = [
    packer.make_can_msg_safety("CRZ_BTNS", 0, {}),
    packer.make_can_msg_safety("STEER_TORQUE", 0, {}),
    packer.make_can_msg_safety("ENGINE_DATA", 0, {}),
    packer.make_can_msg_safety("PEDALS", 0, {}),
  ]
  if include_crz_ctrl:
    msgs.append(packer.make_can_msg_safety("CRZ_CTRL", 0, {}))
  if include_ti:
    msgs.append(packer.make_can_msg_safety("TI_FEEDBACK", 1, {"TI_TORQUE_SENSOR": 0}))
  for msg in msgs:
    assert safety.safety_rx_hook(msg)
  return safety.safety_config_valid()


def test_mazda_stock_mrcc_requires_crz_ctrl():
  for param in (MazdaSafetyFlags.GEN1, MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR):
    ti = bool(param & MazdaSafetyFlags.TORQUE_INTERCEPTOR)
    assert not _rx_checks_valid(param, include_crz_ctrl=False, include_ti=ti)
    assert _rx_checks_valid(param, include_crz_ctrl=True, include_ti=ti)


def test_mazda_ti_requires_ti_feedback():
  param = MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR
  assert not _rx_checks_valid(param, include_crz_ctrl=True, include_ti=False)


def test_mazda_no_mrcc_and_ri_do_not_require_crz_ctrl():
  for extra in (MazdaSafetyFlags.NO_MRCC, MazdaSafetyFlags.RADAR_INTERCEPTOR):
    for ti in (False, True):
      param = MazdaSafetyFlags.GEN1 | extra | (MazdaSafetyFlags.TORQUE_INTERCEPTOR if ti else 0)
      assert _rx_checks_valid(param, include_crz_ctrl=False, include_ti=ti)

if __name__ == "__main__":
  unittest.main()
