#!/usr/bin/env python3
import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car.structs import CarParams
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import Buttons, MazdaSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  # Gen1 (2017-2021 CX-5 / CX-9 / Mazda3 / Mazda6), stock MRCC, no interceptors
  SAFETY_PARAM = MazdaSafetyFlags.GEN1

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  # the TI command (0x249) is never passed between the car and the camera (forwarded to the camera, the camera
  # turns its LKAS off and the EPS then ignores CAM_LKAS)
  FWD_BLACKLISTED_ADDRS = {0: [0x249], 2: [0x243, 0x440, 0x249]}

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


class AutoResumeHelpers:
  """The gas auto-resume window (MazdaBrakeAutoResume, opendbc/car/mazda/auto_resume.py, mazda.h)."""
  def _crz(self, active, main=True):
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, {"CRZ_ACTIVE": active, "CRZ_AVAILABLE": 1 if main else 0})

  def _engine(self, kph, gas=0):
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, {"SPEED": kph, "PEDAL_GAS": gas})

  def _btn(self, button):
    # the very frame the car code sends (mazdacan.create_button_cmd)
    addr, dat, bus = mazdacan.create_button_cmd(CANPacker("mazda_2017"), SimpleNamespace(flags=MazdaSafetyFlags.GEN1), 3,
                                                button)
    return libsafety_py.make_CANPacket(addr, bus, dat)

  def _brake_cancel(self, kph=50, brake_at_fall=True, main=True, t=0):
    """Cruise on, the driver's brake, cruise off (brake still down), brake released, gas down."""
    self.safety.set_timer(t)
    self._rx(self._crz(1, main))
    self._rx(self._engine(kph))
    self.assertTrue(self.safety.get_controls_allowed() or not main)
    self._rx(self._user_brake_msg(1 if brake_at_fall else 0))
    self._rx(self._crz(0, main))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(0))
    self._rx(self._engine(kph, gas=40))


class AutoResumeTests(AutoResumeHelpers):
  def test_res_and_set_minus_frames_are_the_ones_the_car_code_sends(self):
    self.assertEqual(self._btn(Buttons.RESUME).data[0], 0x04)
    self.assertEqual(self._btn(Buttons.SET_MINUS).data[0], 0x20)
    self.assertEqual(self._btn(Buttons.SET_PLUS).data[0], 0x10)
    self.assertEqual(self._btn(Buttons.CANCEL).data[0], 0x01)

  def test_allowed_only_in_the_window(self):
    self.safety.set_controls_allowed(0)
    for b in (Buttons.RESUME, Buttons.SET_MINUS, Buttons.SET_PLUS):
      self.assertFalse(self._tx(self._btn(b)))
    self._brake_cancel()
    self.assertTrue(self._tx(self._btn(Buttons.RESUME)))
    self.assertTrue(self._tx(self._btn(Buttons.CANCEL)))

  def test_set_plus_and_set_minus_never(self):
    self._brake_cancel()
    for _ in range(10):
      self.assertFalse(self._tx(self._btn(Buttons.SET_PLUS)))
      self.assertFalse(self._tx(self._btn(Buttons.SET_MINUS)))
    self.assertTrue(self._tx(self._btn(Buttons.RESUME)))     # (they did not use the budget)

  def test_needs_the_gas_and_no_brake(self):
    self._brake_cancel()
    self._rx(self._engine(50, gas=0))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))
    self._rx(self._engine(50, gas=40))
    self._rx(self._user_brake_msg(1))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))
    self.assertFalse(self._tx(self._btn(Buttons.SET_MINUS)))
    # a new brake press does not close the window (a stop sign, then the gas)
    self._rx(self._user_brake_msg(0))
    self.assertTrue(self._tx(self._btn(Buttons.RESUME)))

  def test_not_after_a_cancel_without_the_brake(self):
    self._brake_cancel(brake_at_fall=False)
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))
    self.assertFalse(self._tx(self._btn(Buttons.SET_MINUS)))

  def test_not_with_the_main_switch_off(self):
    self._brake_cancel()
    self._rx(self._crz(0, main=False))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))
    self._rx(self._crz(0, main=True))     # main back on: the stored speed is gone, the window stays closed
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))

  def test_frame_budget_not_refilled_by_a_new_brake_press(self):
    self._brake_cancel()
    for _ in range(8):
      self.assertTrue(self._tx(self._btn(Buttons.RESUME)))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))
    self._rx(self._user_brake_msg(1))
    self._rx(self._user_brake_msg(0))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))
    self.assertFalse(self._tx(self._btn(Buttons.SET_MINUS)))
    # a new cruise on -> brake cancel cycle refills it
    self._brake_cancel()
    self.assertTrue(self._tx(self._btn(Buttons.RESUME)))

  def test_window_closes_when_the_cruise_comes_on(self):
    self._brake_cancel()
    self.assertTrue(self._tx(self._btn(Buttons.RESUME)))
    self._rx(self._crz(1))
    self.assertTrue(self._tx(self._btn(Buttons.RESUME)))   # controls allowed now: as before
    # controls lost with the cruise still on (e.g. an RX check): the closed window allows nothing
    self.safety.set_controls_allowed(0)
    self._rx(self._crz(1))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))
    self.safety.set_controls_allowed(1)
    # off again without the brake (his CANCEL): closed
    self._rx(self._crz(0))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))

  def test_180_s_of_moving_time_closes_it_standing_still_never_counts(self):
    self._brake_cancel(kph=0)
    # 10 minutes standing at a red light
    for s in range(1, 601):
      self.safety.set_timer(s * 1_000_000)
      self._rx(self._engine(0, gas=40))
    self.assertTrue(self._tx(self._btn(Buttons.RESUME)))
    # then 179 s moving: still open; past 180 s: closed for good
    t0 = 600
    for s in range(1, 180):
      self.safety.set_timer((t0 + s) * 1_000_000)
      self._rx(self._engine(30, gas=40))
    self.assertTrue(self._tx(self._btn(Buttons.RESUME)))
    for s in range(180, 183):
      self.safety.set_timer((t0 + s) * 1_000_000)
      self._rx(self._engine(30, gas=40))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))
    self._rx(self._engine(0, gas=40))
    self.assertFalse(self._tx(self._btn(Buttons.SET_MINUS)))


class AutoResumeReviewTests(AutoResumeHelpers):
  """Review of round n (S/rn/mut): the negatives the first tests missed."""
  def _btn_raw(self, b0):
    p = self._btn(Buttons.RESUME)
    p.data[0] = b0
    return p

  def test_only_res_alone(self):
    self._brake_cancel()
    for b0 in (0x24, 0x14, 0x05, 0x84, 0x44, 0x0C, 0x06, 0x20, 0x10):
      self.assertFalse(self._tx(self._btn_raw(b0)), hex(b0))
    self.assertTrue(self._tx(self._btn_raw(0x04)))

  def test_init_resets_the_window(self):
    self._brake_cancel()
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, int(self.SAFETY_PARAM))
    self._rx(self._crz(0))
    self._rx(self._engine(50, gas=40))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))

  def test_init_with_the_cruise_on_forgets_it(self):
    # re-init while the cruise is on: the first CRZ_CTRL frame with it off is not a falling edge of this config
    self._rx(self._crz(1))
    self._rx(self._engine(50))
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, int(self.SAFETY_PARAM))
    self._rx(self._user_brake_msg(1))
    self._rx(self._crz(0))
    self._rx(self._user_brake_msg(0))
    self._rx(self._engine(50, gas=40))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))

  def test_the_moving_time_never_wraps_around(self):
    # 71.7 minutes of moving after a brake cancel (a uint32 microsecond count wraps at 71.6 min): still closed
    self._brake_cancel(kph=30, t=1_000_000)
    for s in range(2, 4302, 2):              # (4299 s: a wrapped count would read ~4 s)
      self.safety.set_timer((s * 1_000_000) & 0xFFFFFFFF)   # (the panda's timer wraps too)
      self._rx(self._engine(30, gas=40))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))

  def test_a_brake_with_the_cruise_already_off_opens_nothing(self):
    # the cruise never on: his brake with CRZ_CTRL frames during it, released, gas
    self._rx(self._crz(0))
    self._rx(self._user_brake_msg(1))
    self._rx(self._crz(0))
    self._rx(self._crz(0))
    self._rx(self._user_brake_msg(0))
    self._rx(self._engine(50, gas=40))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))

  def test_a_new_brake_with_crz_frames_never_refills_the_budget(self):
    self._brake_cancel()
    for _ in range(8):
      self.assertTrue(self._tx(self._btn(Buttons.RESUME)))
    self._rx(self._user_brake_msg(1))
    self._rx(self._crz(0))
    self._rx(self._crz(0))
    self._rx(self._user_brake_msg(0))
    self._rx(self._crz(0))
    self._rx(self._engine(50, gas=40))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))

  def test_moving_time_counts_from_a_falling_edge_at_a_later_time(self):
    t0 = 500                               # s: the brake cancel well into the drive
    self._brake_cancel(kph=30, t=t0 * 1_000_000)
    for s in range(1, 180):
      self.safety.set_timer((t0 + s) * 1_000_000)
      self._rx(self._engine(30, gas=40))
    self.assertTrue(self._tx(self._btn(Buttons.RESUME)))
    for s in range(180, 182):
      self.safety.set_timer((t0 + s) * 1_000_000)
      self._rx(self._engine(30, gas=40))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))

  def test_moving_state_at_the_falling_edge_is_the_cars(self):
    # standing at the falling edge: the first interval after it is not counted as moving
    t0 = 100
    self._brake_cancel(kph=0, t=t0 * 1_000_000)
    self.safety.set_timer((t0 + 175) * 1_000_000)
    self._rx(self._engine(0, gas=40))      # 175 s standing (one interval: counted, the window would close)
    for s in range(176, 184):
      self.safety.set_timer((t0 + s) * 1_000_000)
      self._rx(self._engine(30, gas=40))
    self.assertTrue(self._tx(self._btn(Buttons.RESUME)))

  def test_not_with_the_rx_checks_failing(self):
    self._brake_cancel(t=1_000_000)
    self.assertTrue(self._tx(self._btn(Buttons.RESUME)))
    # ENGINE_DATA / PEDALS stop: 3 s later the RX checks are invalid (gas / brake could be stale)
    self.safety.set_timer(4_000_000)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.safety_config_valid())
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))

  def test_res_with_controls_allowed_does_not_use_the_budget(self):
    self._brake_cancel()
    self._rx(self._crz(1))
    for _ in range(20):
      self.assertTrue(self._tx(self._btn(Buttons.RESUME)))
    self._rx(self._user_brake_msg(1))
    self._rx(self._crz(0))
    self._rx(self._user_brake_msg(0))
    self._rx(self._engine(50, gas=40))
    for _ in range(8):
      self.assertTrue(self._tx(self._btn(Buttons.RESUME)))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))


class TestMazdaAutoResumeSafety(AutoResumeReviewTests, AutoResumeTests, TestMazdaTorqueInterceptorSafety):
  """Gen1 + TI with the flag: every test of the classes above runs with it too (nothing else changes)."""
  SAFETY_PARAM = MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR | MazdaSafetyFlags.AUTO_RESUME


class TestMazdaAutoResumeNoTiSafety(AutoResumeReviewTests, AutoResumeTests, TestMazdaSafety):
  """Gen1 without a Torque Interceptor, with the flag (the window does not depend on the TI)."""
  SAFETY_PARAM = MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.AUTO_RESUME


class TestMazdaFlagOffSafety(AutoResumeHelpers, TestMazdaTorqueInterceptorSafety):
  """Without the flag the brake cancel opens no window: only CANCEL while controls are not allowed (as before)."""
  def test_no_window_with_the_flag_off(self):
    self._brake_cancel()
    for b in (Buttons.RESUME, Buttons.SET_MINUS, Buttons.SET_PLUS):
      self.assertFalse(self._tx(self._btn(b)))
    self.assertTrue(self._tx(self._btn(Buttons.CANCEL)))


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
