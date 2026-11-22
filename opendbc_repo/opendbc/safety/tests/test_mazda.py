#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.mazda.values import MazdaSafetyFlags
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg
from opendbc.safety.tests.libsafety import libsafety_py


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  DBC = "mazda_2017"
  SAFETY_MODEL = CarParams.SafetyModel.mazda

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


class TestMazdaTorqueInterceptorSafety(TestMazdaSafety):
  """Gen1 + MoreTorque Torque Interceptor (TI). The TI command CAM_LKAS2 (0x249) is sent on bus 1, driver torque
  comes from TI_FEEDBACK (0x24A) on bus 1, and both steering messages are limited to 600 (carcontroller STEER_MAX)."""
  SAFETY_PARAM = int(MazdaSafetyFlags.TORQUE_INTERCEPTOR)
  TX_MSGS = [[0x243, 0], [0x249, 1], [0x09d, 0], [0x440, 0]]
  MAX_TORQUE_LOOKUP = [0], [600]
  # the TI command (0x249) is never passed between the car and the camera
  FWD_BLACKLISTED_ADDRS = {0: [0x249], 2: [0x243, 0x440, 0x249]}

  def _torque_driver_msg(self, torque):
    values = {"TI_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("TI_FEEDBACK", 1, values)

  def _ti_cmd_msg(self, torque):
    values = {"LKAS_REQUEST": torque, "CHKSUM": torque, "KEY": 3294744160}
    return self.packer.make_can_msg_safety("CAM_LKAS2", 1, values)

  def test_ti_cmd_same_as_lkas(self):
    # carcontroller sends the same value on CAM_LKAS and CAM_LKAS2 every frame, at 100 Hz: both must pass
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
    for limit in (600, -600):
      self._set_prev_torque(limit)
      self.assertTrue(self._tx(self._ti_cmd_msg(limit)))
      self._set_prev_torque(limit)
      self.assertFalse(self._tx(self._ti_cmd_msg(limit + (1 if limit > 0 else -1))))
    # the stock 800 limit does not apply with a TI, on either message
    self._set_prev_torque(700)
    self.assertFalse(self._tx(self._torque_cmd_msg(700)))
    self._set_prev_torque(700)
    self.assertFalse(self._tx(self._ti_cmd_msg(700)))

  def test_ti_cmd_rate_limit(self):
    # (Stage 3, C) the rate limit is checked on CAM_LKAS, which carcontroller sends first in the same frame; the TI
    # command then has to repeat that value (it no longer steps the shared limiter by itself)
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._ti_cmd_msg(self.MAX_RATE_UP + 1)))
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))
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

  def test_ti_driver_torque_limits(self):
    # a driver pushing against the command on the TI sensor limits the command
    self.safety.set_controls_allowed(True)
    for _ in range(6):
      self._rx(self._torque_driver_msg(-100))
    self._set_prev_torque(600)
    self.assertFalse(self._tx(self._torque_cmd_msg(600)))
    # (Stage 3, C) the blocked CAM_LKAS reset the limiter: the TI command of the same frame is blocked too
    self.assertFalse(self._tx(self._ti_cmd_msg(600)))

  # (Stage 3, C) these talk to the hook directly: the CmdSafety subclass pairs each TI command with a CAM_LKAS frame
  def _tx_raw(self, msg):
    return self.safety.safety_tx_hook(msg)

  def _lkas_msg(self, torque):
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, {"LKAS_REQUEST": torque})

  def test_ti_cmd_cannot_double_the_rate_limit(self):
    # (Stage 3, C) one shared rate limiter for two messages: in Stage 1 the TI command was checked as a second torque
    # command and moved the limiter's state too, so CAM_LKAS and CAM_LKAS2 taking turns each stepped MAX_RATE_UP from
    # the other one's value: twice the rate per frame. The TI command must now repeat the last checked CAM_LKAS value.
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(0)
    self._set_prev_torque(0)
    torque = 0
    for frame in range(1, 6):
      self.safety.set_timer(frame * 10000)
      torque += self.MAX_RATE_UP
      self.assertTrue(self._tx_raw(self._lkas_msg(torque)))
      self.assertFalse(self._tx_raw(self._ti_cmd_msg(torque + self.MAX_RATE_UP)), f"{frame=}")
      self.assertTrue(self._tx_raw(self._ti_cmd_msg(torque)))

  def test_ti_cmd_must_match_the_last_lkas_cmd(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertTrue(self._tx_raw(self._lkas_msg(self.MAX_RATE_UP)))
    for other in (0, self.MAX_RATE_UP - 1, self.MAX_RATE_UP + 1, -self.MAX_RATE_UP):
      self.assertFalse(self._tx_raw(self._ti_cmd_msg(other)), other)
    self.assertTrue(self._tx_raw(self._ti_cmd_msg(self.MAX_RATE_UP)))
    # checking the TI command never moves the limiter: CAM_LKAS still steps from its own last value
    self.assertTrue(self._tx_raw(self._lkas_msg(2 * self.MAX_RATE_UP)))
    # a blocked CAM_LKAS resets the limiter to 0: the TI command cannot carry the blocked value
    self.assertFalse(self._tx_raw(self._lkas_msg(5 * self.MAX_RATE_UP)))
    self.assertFalse(self._tx_raw(self._ti_cmd_msg(5 * self.MAX_RATE_UP)))
    self.assertFalse(self._tx_raw(self._ti_cmd_msg(2 * self.MAX_RATE_UP)))
    self.assertTrue(self._tx_raw(self._ti_cmd_msg(0)))

  def test_ti_cmd_after_controls_lost_is_zero(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertTrue(self._tx_raw(self._lkas_msg(self.MAX_RATE_UP)))
    self.safety.set_controls_allowed(False)
    # controls lost between the two messages of one frame: the TI command is blocked (it is not 0, and a 0 has to
    # follow a CAM_LKAS 0 too); carcontroller's next frame (CAM_LKAS 0, then CAM_LKAS2 0) passes
    self.assertFalse(self._tx_raw(self._ti_cmd_msg(self.MAX_RATE_UP)))
    self.assertFalse(self._tx_raw(self._ti_cmd_msg(0)))
    self.assertTrue(self._tx_raw(self._lkas_msg(0)))
    self.assertTrue(self._tx_raw(self._ti_cmd_msg(0)))

  def test_ti_cmd_bounded_on_its_own(self):
    # even with the limiter's last value out of bounds (only possible by hand), the TI command stays within max_torque
    top = self.MAX_TORQUE_LOOKUP[1][0]
    self.safety.set_controls_allowed(True)
    for t in (top + 1, -top - 1, 2047, -2048):
      self.safety.set_desired_torque_last(t)
      self.assertFalse(self._tx_raw(self._ti_cmd_msg(t)), t)


class TestMazdaTorqueInterceptorCmdSafety(TestMazdaTorqueInterceptorSafety):
  """Runs the generic torque steering tests on the TI command (CAM_LKAS2, bus 1) instead of CAM_LKAS. As carcontroller
  sends them, each TI command goes out right after the same value on CAM_LKAS (bus 0); the TI command's own result is
  what the generic test checks."""
  def _torque_cmd_msg(self, torque, steer_req=1):
    return self._ti_cmd_msg(torque)

  def _tx(self, msg):
    if msg.addr == 0x249:
      raw = ((msg.data[0] & 0x0F) << 8) | msg.data[1]
      super()._tx(self.packer.make_can_msg_safety("CAM_LKAS", 0, {"LKAS_REQUEST": raw - 2048}))
    return super()._tx(msg)


def _rx_checks_valid(param, include_ti):
  packer = CANPackerSafety("mazda_2017")
  safety = libsafety_py.libsafety
  safety.set_safety_hooks(CarParams.SafetyModel.mazda, int(param))
  safety.init_tests()
  msgs = [packer.make_can_msg_safety(name, 0, {}) for name in ("CRZ_CTRL", "CRZ_BTNS", "STEER_TORQUE", "ENGINE_DATA", "PEDALS")]
  if include_ti:
    msgs.append(packer.make_can_msg_safety("TI_FEEDBACK", 1, {"TI_TORQUE_SENSOR": 0}))
  for msg in msgs:
    assert safety.safety_rx_hook(msg)
  return safety.safety_config_valid()


def test_mazda_ti_requires_ti_feedback():
  param = int(MazdaSafetyFlags.TORQUE_INTERCEPTOR)
  assert not _rx_checks_valid(param, include_ti=False)
  assert _rx_checks_valid(param, include_ti=True)


def test_mazda_stock_does_not_require_ti_feedback():
  # without the TI flag the RX checks are stock: TI_FEEDBACK is not needed
  assert _rx_checks_valid(0, include_ti=False)


def test_mazda_stock_ti_messages_not_allowed():
  # without the TI flag (stock param 0): no CAM_LKAS2 TX, TI_FEEDBACK is not driver torque, stock limit 800
  packer = CANPackerSafety("mazda_2017")
  safety = libsafety_py.libsafety
  safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
  safety.init_tests()
  safety.set_controls_allowed(True)
  assert not safety.safety_tx_hook(packer.make_can_msg_safety("CAM_LKAS2", 1, {"LKAS_REQUEST": 0, "CHKSUM": 0, "KEY": 3294744160}))
  for _ in range(6):
    safety.safety_rx_hook(packer.make_can_msg_safety("TI_FEEDBACK", 1, {"TI_TORQUE_SENSOR": -100}))
    safety.safety_rx_hook(packer.make_can_msg_safety("STEER_TORQUE", 0, {"STEER_TORQUE_SENSOR": 0}))
  safety.set_desired_torque_last(800)
  safety.set_rt_torque_last(800)
  safety.set_torque_meas(800, 800)
  assert safety.safety_tx_hook(packer.make_can_msg_safety("CAM_LKAS", 0, {"LKAS_REQUEST": 800}))
  # the TI command is forwarded like any other message without a TI (stock behaviour)
  assert safety.safety_fwd_hook(0, 0x249) == 2
  assert safety.safety_fwd_hook(2, 0x249) == 0


class TestMazdaIgnition(common.SafetyTestBase):
  SAFETY_MODEL = None

  TX_MSGS: list = []

  def _msg(self, byte0):
    return make_msg(0, 0x9E, dat=bytes([byte0]) + b"\x00" * 7)

  # 0x9E byte 0 high 3 bits == 6 (0xC0)
  def test_ignition_on(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0x20))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
