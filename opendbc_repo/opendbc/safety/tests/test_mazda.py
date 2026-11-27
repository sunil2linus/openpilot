#!/usr/bin/env python3
import re
import unittest
from pathlib import Path
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car.structs import CarParams
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import Buttons, MazdaFlags, MazdaSafetyFlags
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
    # TI_TORQUE_SENSOR is 8 bits with offset -127: a driver torque outside -127..128 is sent as the nearest it can be
    # (the packer would wrap it; the generic "arbitrary high driver torque" for the 700 test is 716, read as -52)
    values = {"TI_TORQUE_SENSOR": max(-127, min(128, torque))}
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
      torque = min(torque + self.MAX_RATE_UP, self.MAX_TORQUE_LOOKUP[1][0])
      self.assertTrue(self._tx(self._torque_cmd_msg(torque)), f"CAM_LKAS {torque=}")
      self.assertTrue(self._tx(self._ti_cmd_msg(torque)), f"CAM_LKAS2 {torque=}")

  def test_ti_cmd_blocked_when_not_allowed(self):
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._ti_cmd_msg(0)))
    self.assertFalse(self._tx(self._ti_cmd_msg(self.MAX_RATE_UP)))

  def test_ti_cmd_max_torque(self):
    top = self.MAX_TORQUE_LOOKUP[1][0]       # 600 (700 with the steering limit test's flag)
    self.safety.set_controls_allowed(True)
    for limit in (top, -top):
      self._set_prev_torque(limit)
      self.assertTrue(self._tx(self._ti_cmd_msg(limit)))
      self._set_prev_torque(limit)
      self.assertFalse(self._tx(self._ti_cmd_msg(limit + (1 if limit > 0 else -1))))
    # the stock 800 limit does not apply with a TI, on either message
    self._set_prev_torque(top + 100)
    self.assertFalse(self._tx(self._torque_cmd_msg(top + 100)))
    self._set_prev_torque(top + 100)
    self.assertFalse(self._tx(self._ti_cmd_msg(top + 100)))

  def test_steer_700_only_with_the_test_flag(self):
    # (Stage 3, B) 601..700 pass only with TI_STEER_700 (and the TI); 701 never; the 600 path is unchanged
    allowed = bool(self.SAFETY_PARAM & MazdaSafetyFlags.TI_STEER_700)
    self.safety.set_controls_allowed(True)
    for t in (601, 650, 700, -700):
      self._set_prev_torque(t)
      self.assertEqual(allowed, self._tx(self._torque_cmd_msg(t)), ("CAM_LKAS", t))
      self.assertEqual(allowed, self._tx(self._ti_cmd_msg(t)), ("CAM_LKAS2", t))
    for t in (600, -600):
      self._set_prev_torque(t)
      self.assertTrue(self._tx(self._torque_cmd_msg(t)))
      self.assertTrue(self._tx(self._ti_cmd_msg(t)))
    self._set_prev_torque(701)
    self.assertFalse(self._tx(self._torque_cmd_msg(701)))
    self._set_prev_torque(701)
    self.assertFalse(self._tx(self._ti_cmd_msg(701)))

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
    top = self.MAX_TORQUE_LOOKUP[1][0]
    for _ in range(6):
      self._rx(self._torque_driver_msg(-100))
    self._set_prev_torque(top)
    self.assertFalse(self._tx(self._torque_cmd_msg(top)))
    # (Stage 3, C) the blocked CAM_LKAS reset the limiter: the TI command of the same frame is blocked too
    self.assertFalse(self._tx(self._ti_cmd_msg(top)))

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


class TestMazdaTiSteer700Safety(TestMazdaTorqueInterceptorSafety):
  """(Stage 3, B) Gen1 + TI with the steering limit test: both steering messages limited to 700; the rate limits, the
  real-time delta and the driver allowance as without it. Every TI test runs with it."""
  SAFETY_PARAM = int(MazdaSafetyFlags.TORQUE_INTERCEPTOR | MazdaSafetyFlags.TI_STEER_700)
  MAX_TORQUE_LOOKUP = [0], [700]


class TestMazdaTiSteer700CmdSafety(TestMazdaTorqueInterceptorCmdSafety):
  """The generic torque steering tests on the TI command with the test's 700."""
  SAFETY_PARAM = int(MazdaSafetyFlags.TORQUE_INTERCEPTOR | MazdaSafetyFlags.TI_STEER_700)
  MAX_TORQUE_LOOKUP = [0], [700]


class TestMazdaSteer700FlagWithoutTiSafety(TestMazdaSafety):
  """(Stage 3, B) the test's flag without a TI is ignored: the stock 800 on CAM_LKAS as always, no CAM_LKAS2."""
  SAFETY_PARAM = int(MazdaSafetyFlags.TI_STEER_700)

  def test_no_ti_command_with_the_flag_alone(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    values = {"LKAS_REQUEST": 0, "CHKSUM": 0, "KEY": 3294744160}
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("CAM_LKAS2", 1, values)))


class AutoResumeHelpers:
  """(Stage 3, A) the gas auto-resume window (mazda.h; car side openpilot/starpilot/car/mazda/speed/auto_resume.py)."""
  def _crz(self, active, main=True):
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, {"CRZ_ACTIVE": active, "CRZ_AVAILABLE": 1 if main else 0})

  def _engine(self, kph, gas=0):
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, {"SPEED": kph, "PEDAL_GAS": gas})

  def _btn(self, button):
    # the very frame the car code sends (mazdacan.create_button_cmd)
    addr, dat, bus = mazdacan.create_button_cmd(CANPacker("mazda_2017"), SimpleNamespace(flags=MazdaFlags.GEN1), 3, button)
    return libsafety_py.make_CANPacket(addr, bus, dat)

  def _brake_cancel(self, kph=50, brake_at_fall=True, main=True, t=0):
    """Cruise on, the driver's brake, cruise off (brake still down), brake released, gas down."""
    self.safety.set_timer(t)
    self._rx(self._crz(1, main))
    self._rx(self._engine(kph))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(1 if brake_at_fall else 0))
    self._rx(self._crz(0, main))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(0))
    self._rx(self._engine(kph, gas=40))


class AutoResumeTests(AutoResumeHelpers):
  def test_res_and_set_frames_are_the_ones_the_car_code_sends(self):
    self.assertEqual(self._btn(Buttons.RESUME).data[0], 0x04)
    self.assertEqual(self._btn(Buttons.SET_MINUS).data[0], 0x20)
    self.assertEqual(self._btn(Buttons.SET_PLUS).data[0], 0x10)
    self.assertEqual(self._btn(Buttons.CANCEL).data[0], 0x01)

  def test_res_blocked_outside_the_window(self):
    self.safety.set_controls_allowed(0)
    for b in (Buttons.RESUME, Buttons.SET_MINUS, Buttons.SET_PLUS):
      self.assertFalse(self._tx(self._btn(b)))
    # the gas and the main switch alone open nothing: no brake cancel yet
    self._rx(self._crz(0))
    self._rx(self._engine(50, gas=40))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))
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

  def test_no_window_without_main_on_at_the_fall(self):
    # the brake cancel with the main switch off in that CRZ_CTRL frame opens nothing, even when it comes back on
    self.safety.set_timer(0)
    self._rx(self._crz(1, main=True))
    self._rx(self._engine(50))
    self._rx(self._user_brake_msg(1))
    self._rx(self._crz(0, main=False))
    self._rx(self._user_brake_msg(0))
    self._rx(self._crz(0, main=True))
    self._rx(self._engine(50, gas=40))
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))

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

  def test_window_closed_by_180_s_of_moving_time_standing_still_never_counts(self):
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
    self.assertFalse(self._tx(self._btn(Buttons.RESUME)))


class AutoResumeReviewTests(AutoResumeHelpers):
  """The negatives (the old fork's review of the window)."""
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
    self._rx(self._engine(0, gas=40))
    for s in range(176, 184):
      self.safety.set_timer((t0 + s) * 1_000_000)
      self._rx(self._engine(30, gas=40))
    self.assertTrue(self._tx(self._btn(Buttons.RESUME)))

  def test_not_with_the_rx_checks_failing(self):
    self._brake_cancel(t=1_000_000)
    self.assertTrue(self._tx(self._btn(Buttons.RESUME)))
    # ENGINE_DATA / PEDALS stop: 3 s later the RX checks are invalid (gas / brake could be stale)
    self.safety.set_timer(4_000_000)
    self.safety.safety_tick()
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
  """Gen1 + TI with the auto-resume flag: every TI test runs with it too (nothing else changes)."""
  SAFETY_PARAM = int(MazdaSafetyFlags.TORQUE_INTERCEPTOR | MazdaSafetyFlags.AUTO_RESUME)


class TestMazdaAutoResumeNoTiSafety(AutoResumeReviewTests, AutoResumeTests, TestMazdaSafety):
  """Gen1 without a TI, with the flag (the window does not depend on the TI)."""
  SAFETY_PARAM = int(MazdaSafetyFlags.AUTO_RESUME)


class TestMazdaAutoResumeSteer700Safety(AutoResumeTests, TestMazdaTiSteer700Safety):
  """All three Stage 3 flags together: 700 and the window."""
  SAFETY_PARAM = int(MazdaSafetyFlags.TORQUE_INTERCEPTOR | MazdaSafetyFlags.TI_STEER_700 | MazdaSafetyFlags.AUTO_RESUME)


class TestMazdaAutoResumeFlagOffSafety(AutoResumeHelpers, TestMazdaTorqueInterceptorSafety):
  """Without the flag a brake cancel opens no window: only CANCEL while controls are not allowed (as before)."""
  def test_no_window_with_the_flag_off(self):
    self._brake_cancel()
    for b in (Buttons.RESUME, Buttons.SET_MINUS, Buttons.SET_PLUS):
      self.assertFalse(self._tx(self._btn(b)))
    self.assertTrue(self._tx(self._btn(Buttons.CANCEL)))


class TestMazdaStockAutoResumeFlagOffSafety(AutoResumeHelpers, TestMazdaSafety):
  def test_no_window_with_the_flag_off(self):
    TestMazdaAutoResumeFlagOffSafety.test_no_window_with_the_flag_off(self)


def test_safety_flags_match_mazda_h():
  # one place each: mazda.h MAZDA_PARAM_* and values.py MazdaSafetyFlags, the same names and bits, no bit shared
  src = (Path(__file__).resolve().parents[1] / "modes" / "mazda.h").read_text()
  c_flags = {name: int(v) for name, v in re.findall(r"^#define MAZDA_PARAM_(\w+) (\d+)U", src, re.MULTILINE)}
  assert c_flags == {f.name: int(f) for f in MazdaSafetyFlags}
  assert c_flags == {"TORQUE_INTERCEPTOR": 8, "AUTO_RESUME": 256, "TI_STEER_700": 512, "TI_AOL": 1024}
  bits = list(c_flags.values())
  assert all(b & (b - 1) == 0 for b in bits) and len(set(bits)) == len(bits)
  # never in CP.flags: the car flags stay GEN1 (| TORQUE_INTERCEPTOR), which the dashboard / AOL checks compare exactly
  assert not any(int(f) & (int(MazdaSafetyFlags.AUTO_RESUME) | int(MazdaSafetyFlags.TI_STEER_700) |
                           int(MazdaSafetyFlags.TI_AOL)) for f in MazdaFlags)


def test_ti_steer_700_needs_both_flags_and_auto_resume_keeps_the_tx_list():
  packer = CANPackerSafety("mazda_2017")
  safety = libsafety_py.libsafety
  ti = packer.make_can_msg_safety("CAM_LKAS2", 1, {"LKAS_REQUEST": 0, "CHKSUM": 0, "KEY": 3294744160})
  for param, ti_ok, top in ((0, False, 800), (int(MazdaSafetyFlags.TI_STEER_700), False, 800),
                            (int(MazdaSafetyFlags.TORQUE_INTERCEPTOR), True, 600),
                            (int(MazdaSafetyFlags.TORQUE_INTERCEPTOR | MazdaSafetyFlags.TI_STEER_700), True, 700),
                            (int(MazdaSafetyFlags.AUTO_RESUME), False, 800),
                            (int(MazdaSafetyFlags.AUTO_RESUME | MazdaSafetyFlags.TI_STEER_700), False, 800)):
    safety.set_safety_hooks(CarParams.SafetyModel.mazda, param)
    safety.init_tests()
    safety.set_controls_allowed(True)
    assert bool(safety.safety_tx_hook(ti)) == ti_ok, param
    for t, ok in ((top, True), (top + 1, False)):
      safety.set_desired_torque_last(t)
      safety.set_rt_torque_last(t)
      safety.set_torque_meas(t, t)
      assert bool(safety.safety_tx_hook(packer.make_can_msg_safety("CAM_LKAS", 0, {"LKAS_REQUEST": t}))) == ok, (param, t)


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
