"""Always On Lateral with the MoreTorque Torque Interceptor (mazda.h MAZDA_PARAM_TI_AOL, mazda_stock_aol.h TI rules).

Steering with the cruise off is allowed only with every condition read by the panda itself; the steering limits, the
CAM_LKAS2 rule and the cruise button rules are those of the TI without AOL; every other param / alternative experience
behaves as before (see also test_mazda.py, test_mazda_stock_aol.py, test_mazda_engagement_aol.py).
"""
import random
import unittest

from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.can.dbc import DBC
from opendbc.car.mazda.values import Buttons, MazdaFlags, MazdaSafetyFlags
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.ti_aol import native_param
from opendbc.safety.tests.common import CANPackerSafety, make_msg
from opendbc.safety.tests.libsafety import libsafety_py

TI = int(MazdaSafetyFlags.TORQUE_INTERCEPTOR)
TI_AOL = int(MazdaSafetyFlags.TI_AOL)
AR = int(MazdaSafetyFlags.AUTO_RESUME)
S700 = int(MazdaSafetyFlags.TI_STEER_700)
TI_AOL_PARAMS = (TI | TI_AOL, TI | TI_AOL | AR, TI | TI_AOL | S700, TI | TI_AOL | AR | S700)
SOURCES = ('CRZ_CTRL', 'CRZ_BTNS', 'STEER_TORQUE', 'ENGINE_DATA', 'PEDALS', 'TI_FEEDBACK', 'GEAR', 'STEER_RATE', 'CAM_LKAS')


class MazdaTiAolBase(unittest.TestCase):
  PARAM = TI | TI_AOL

  def tearDown(self):
    self.safety.init_tests()
    self.safety.set_alternative_experience(0)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)

  def configure(self, experience=32, param=None):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.safety.set_alternative_experience(experience)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.mazda, self.PARAM if param is None else param)
    self.time = 1_000_000
    self.safety.set_timer(self.time)
    self.safety.set_aol_test_heartbeat(True)
    self.packer = CANPackerSafety('mazda_2017')
    self.dbc = DBC('mazda_2017')

  def packet(self, name, fields, bus=0):
    self.assertFalse(fields.keys() - self.dbc.name_to_msg[name].sigs.keys())
    return self.packer.make_can_msg_safety(name, bus, fields)

  def lkas(self, torque):
    return self.packet('CAM_LKAS', {'LKAS_REQUEST': torque})

  def ti(self, torque):
    return self.packet('CAM_LKAS2', {'LKAS_REQUEST': torque, 'CHKSUM': torque, 'KEY': 3294744160}, 1)

  def steer(self, torque):
    """One carcontroller frame: CAM_LKAS then the same value on CAM_LKAS2. Both verdicts."""
    return bool(self.safety.safety_tx_hook(self.lkas(torque))), bool(self.safety.safety_tx_hook(self.ti(torque)))

  def feed(self, *, main=True, active=False, gear=4, speed=8000, brake=False, gas=0, lkas_block=False, cam_err=False,
           buttons=0, driver=0, omit=(), request=1, step=10_000):
    self.time += step
    self.safety.set_timer(self.time)
    sources = (
      ('CRZ_CTRL', 0, {'CRZ_AVAILABLE': int(main), 'CRZ_ACTIVE': int(active)}),
      ('CRZ_BTNS', 0, {'RES': int(buttons == 1), 'SET_M': int(buttons == 2), 'CAN_OFF': int(buttons == 3)}),
      ('STEER_TORQUE', 0, {'STEER_TORQUE_SENSOR': 0}),
      ('ENGINE_DATA', 0, {'SPEED': speed, 'PEDAL_GAS': gas}),
      ('PEDALS', 0, {'BRAKE_ON': int(brake)}),
      ('TI_FEEDBACK', 1, {'TI_TORQUE_SENSOR': driver}),
      ('GEAR', 0, {'GEAR': gear}),
      ('STEER_RATE', 0, {'LKAS_BLOCK': int(lkas_block)}),
      ('CAM_LKAS', 2, {'ERR_BIT_1': int(cam_err), 'LKAS_REQUEST': 0}),
    )
    for name, bus, fields in sources:
      if name not in omit:
        self.safety.safety_rx_hook(self.packet(name, fields, bus))
    self.safety.safety_tick()
    if request is not None:
      self.safety.aol_set_host_request(request)

  def arm(self, **kwargs):
    """The driver's fresh action: the main switch off, then on (the same rule as stock AOL)."""
    for _ in range(10):
      self.feed(**kwargs)
    self.feed(**{**kwargs, 'main': False})
    self.feed(**kwargs)
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)
    self.assertFalse(self.safety.get_controls_allowed())

  def permission(self):
    return self.safety.aol_get_permission_mask() & 1


class TestMazdaTiAolGrant(MazdaTiAolBase):
  def test_granted_with_every_condition_and_cruise_off(self):
    for param in TI_AOL_PARAMS:
      with self.subTest(param=param):
        self.PARAM = param
        self.configure()
        self.feed()
        self.assertEqual(self.permission(), 0)  # no fresh driver action yet
        self.assertEqual(self.steer(10), (False, False))
        self.assertEqual(self.steer(0), (True, True))
        self.arm()
        self.assertEqual(self.steer(10), (True, True))
        self.assertEqual(self.steer(20), (True, True))
        # never longitudinal without the stock cruise
        self.safety.aol_set_host_request(3)
        self.assertEqual(self.safety.aol_get_permission_mask(), 1)

  def test_lkas_block_is_not_a_condition(self):
    self.configure()
    self.arm(lkas_block=True)
    for _ in range(50):
      self.feed(lkas_block=True)
    self.assertEqual(self.permission(), 1)
    self.assertEqual(self.steer(10), (True, True))

  def test_each_condition_missing_blocks_torque(self):
    cases = {
      'main off': {'main': False}, 'reverse': {'gear': 2}, 'park': {'gear': 1}, 'neutral': {'gear': 3},
      'gear unknown': {'gear': 0}, 'standstill': {'speed': 0}, 'no host request': {'request': 0},
      'camera fault': {'cam_err': True},
    }
    for name, fault in cases.items():
      with self.subTest(case=name):
        self.configure()
        self.arm()
        self.assertEqual(self.steer(10), (True, True))
        self.feed(**fault)
        self.assertEqual(self.permission(), 0)
        self.assertEqual(self.steer(20), (False, False))
        self.assertEqual(self.steer(0), (True, True))

  def test_main_off_with_the_cruise_active_blocks_torque(self):
    # an inconsistent frame (cruise active, main off) must not steer through the controls_allowed path
    self.configure()
    self.arm()
    self.feed(active=True)
    self.feed(active=True, main=False)
    self.assertEqual(self.permission(), 0)
    self.assertEqual(self.steer(10), (False, False))

  def test_rx_check_older_than_its_age_limit_blocks_torque(self):
    # older than the RX check's own age limit (ENGINE_DATA 100 Hz: 100 ms; PEDALS / TI_FEEDBACK 50 Hz: 200 ms), not
    # lagging yet (1 s): not healthy
    for name, frames in (('ENGINE_DATA', 15), ('PEDALS', 25), ('TI_FEEDBACK', 25)):
      with self.subTest(source=name):
        self.configure()
        self.arm()
        for _ in range(frames):
          self.feed(omit=(name,))
        self.assertEqual(self.permission(), 0)
        self.assertEqual(self.steer(10), (False, False))

  def test_rx_check_older_than_its_age_limit_blocks_engaged_torque_too(self):
    self.configure()
    for _ in range(20):
      self.feed(request=0)
    self.feed(active=True)
    self.assertEqual(self.permission(), 1)
    for _ in range(25):
      self.feed(active=True, omit=('PEDALS',))
    self.assertEqual(self.permission(), 0)
    self.assertEqual(self.steer(10), (False, False))

  def test_boot_grant_with_lkas_block_below_45_kph(self):
    # the EPS blocks the stock path at low speed (LKAS_BLOCK): the TI profile still grants at boot
    self.configure()
    for _ in range(10):
      self.feed(speed=2000, lkas_block=True)
    self.time += 1
    self.safety.set_timer(self.time)
    self.safety.aol_set_host_request(1)
    self.assertEqual(self.permission(), 1)

  def test_engagement_grant_with_lkas_block_below_45_kph(self):
    self.configure()
    self.safety.set_aol_test_heartbeat(False)
    for _ in range(20):
      self.feed(speed=2000, lkas_block=True, active=True)  # boot with the cruise active: no boot grant
    self.feed(speed=2000, lkas_block=True)                 # cancel
    self.safety.set_aol_test_heartbeat(True)
    for _ in range(3):
      self.feed(speed=2000, lkas_block=True)
    self.assertEqual(self.permission(), 0)
    self.feed(speed=2000, lkas_block=True, active=True)    # a normal engagement
    self.feed(speed=2000, lkas_block=True, brake=True, active=True)
    self.feed(speed=2000, lkas_block=True, brake=True)     # brake cancel: the engagement token keeps steering
    self.assertEqual(self.permission(), 1)

  def test_heartbeat_lost_blocks_torque(self):
    self.configure()
    self.arm()
    self.safety.set_aol_test_heartbeat(False)
    self.assertEqual(self.permission(), 0)
    self.assertEqual(self.steer(10), (False, False))

  def test_host_request_stale_blocks_torque(self):
    self.configure()
    self.arm()
    self.time += 300_001
    self.safety.set_timer(self.time)
    self.assertEqual(self.permission(), 0)
    self.assertEqual(self.steer(10), (False, False))

  def test_short_gaps_keep_it(self):
    # every source here is at most 90 ms old (the RX checks' own age limit is 100 ms at 100 Hz)
    self.configure()
    self.arm()
    self.time += 90_000
    self.safety.set_timer(self.time)
    self.assertEqual(self.permission(), 1)

  def test_each_source_stale_blocks_torque(self):
    for name in SOURCES:
      with self.subTest(source=name):
        self.configure()
        self.arm()
        for _ in range(120):  # 1.2 s without it: past the 300 ms freshness and every RX check's own age (1 s at 10 Hz)
          self.feed(omit=(name,))
        self.assertEqual(self.permission(), 0)
        self.assertEqual(self.steer(10), (False, False))

  def test_each_source_wrong_bus_or_length_does_not_count(self):
    for name in SOURCES:
      for fault in ('bus', 'length'):
        with self.subTest(source=name, fault=fault):
          self.configure()
          self.arm()
          bus = {'TI_FEEDBACK': 1, 'CAM_LKAS': 2}.get(name, 0)
          for _ in range(120):
            self.feed(omit=(name,))
            message = self.dbc.name_to_msg[name]
            if fault == 'bus':
              self.safety.safety_rx_hook(make_msg((bus + 1) % 3, message.address, dat=bytes(8)))
            else:
              self.safety.safety_rx_hook(make_msg(bus, message.address, dat=bytes(7)))
          self.assertEqual(self.permission(), 0)

  def test_relay_malfunction_blocks_everything(self):
    self.configure()
    self.arm()
    self.safety.set_relay_malfunction(True)
    self.assertEqual(self.permission(), 0)
    self.assertEqual(self.steer(10), (False, False))
    self.assertEqual(self.steer(0), (False, False))

  def test_stock_camera_frame_on_bus_0_is_a_relay_malfunction(self):
    # check_relay stays on for CAM_LKAS / the HUD with the TI profile (openpilot always sends them)
    self.configure()
    self.arm()
    self.safety.init_tests()  # the harness's safety_mode_cnt: past the relay's 1 s transition time
    self.safety.set_timer(self.time)
    self.feed()
    self.safety.safety_rx_hook(self.lkas(0))
    self.assertTrue(self.safety.get_relay_malfunction())
    self.assertEqual(self.permission(), 0)

  def test_rx_checks_invalid_blocks_torque(self):
    self.configure()
    self.arm()
    for _ in range(150):  # 1.5 s without PEDALS: lagging, RX checks invalid at the next tick
      self.feed(omit=('PEDALS',))
    self.assertTrue(self.safety.get_controls_allowed() is False)
    self.feed()
    self.assertEqual(self.permission(), 0)
    self.assertEqual(self.steer(10), (False, False))

  def test_safety_param_bits_both_required(self):
    for param in (TI_AOL, TI_AOL | AR, TI_AOL | S700, TI, TI | AR, TI | S700, TI | AR | S700):
      with self.subTest(param=param):
        self.configure(param=param)
        for _ in range(10):
          self.feed()
        self.feed(main=False)
        self.feed()
        self.assertEqual(self.permission(), 0)
        # alternative experience 32 with an unqualified param: all TX off, as before
        self.assertEqual(self.steer(0), (False, False))
        self.assertFalse(self.safety.safety_tx_hook(button(Buttons.CANCEL)))

  def test_unknown_bits_never_qualify(self):
    for extra in (1, 2, 4, 16, 32, 64, 128, 2048, 4096, 32768):
      with self.subTest(extra=extra):
        self.configure(param=TI | TI_AOL | extra)
        self.feed()
        self.assertFalse(self.safety.safety_tx_hook(self.lkas(0)))

  def test_alternative_experience_must_be_exactly_32(self):
    for experience in (0, 1, 8, 16, 33, 96):
      with self.subTest(experience=experience):
        self.configure(experience=experience)
        for _ in range(10):
          self.feed()
        self.feed(main=False)
        self.feed()
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.assertEqual(self.steer(10), (False, False))


def button(which):
  # the very frame the car code sends (mazdacan.create_button_cmd)
  addr, dat, bus = mazdacan.create_button_cmd(CANPacker('mazda_2017'), SimpleNamespace(flags=MazdaFlags.GEN1), 3, which)
  return libsafety_py.make_CANPacket(addr, bus, dat)


class TestMazdaTiAolBehaviour(MazdaTiAolBase):
  def test_permission_lost_mid_steer_blocks_the_next_non_zero_frame(self):
    for fault in ({'main': False}, {'gear': 2}, {'speed': 0}, {'request': 0}, {'cam_err': True}):
      with self.subTest(fault=fault):
        self.configure()
        self.arm()
        torque = 0
        for _ in range(10):
          torque += 10
          self.assertEqual(self.steer(torque), (True, True))
          self.feed()
        self.feed(**fault)
        self.assertEqual(self.steer(torque), (False, False))
        self.assertEqual(self.steer(torque - 25), (False, False))
        self.assertEqual(self.steer(0), (True, True))

  def test_brake_and_gas_do_not_drop_it(self):
    self.configure()
    self.arm()
    for kwargs in ({'brake': True}, {'gas': 50}, {'brake': True, 'gas': 50}):
      with self.subTest(**kwargs):
        for _ in range(30):
          self.feed(**kwargs)
          self.assertEqual(self.permission(), 1)
        self.assertEqual(self.steer(10), (True, True))
        self.assertEqual(self.steer(0), (True, True))

  def test_cruise_engage_and_brake_cancel(self):
    self.configure()
    for _ in range(20):
      self.feed(request=0)
    self.feed(active=True)                    # a normal engagement after a healthy inactive frame: the token
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.aol_set_host_request(3)
    self.assertEqual(self.safety.aol_get_permission_mask(), 3)
    self.feed(active=True, brake=True)
    self.feed(active=False, brake=True)       # the brake cancels the cruise
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.permission(), 1)
    self.assertEqual(self.steer(10), (True, True))
    self.feed(active=True)                    # engaged again
    self.assertEqual(self.safety.aol_get_permission_mask(), 1 | (self.safety.get_controls_allowed() << 1))
    self.feed(active=False)                   # cancel without the brake (CANCEL button)
    self.assertEqual(self.permission(), 1)

  def test_boot_with_the_cruise_active_needs_a_fresh_action_after_a_cancel(self):
    self.configure()
    for _ in range(20):
      self.feed(active=True)
    self.assertEqual(self.permission(), 1)    # engaged: the controls_allowed path
    self.feed(active=False, brake=True)
    self.assertEqual(self.permission(), 0)
    self.feed(buttons=0)
    self.feed(buttons=1)                       # a cruise button after a neutral frame
    self.assertEqual(self.permission(), 1)

  def _boot(self, *, gas=0, brake=False):
    """Boot with the main switch on, parked, then driving off; returns the permission after the host's next request."""
    self.safety.set_aol_test_heartbeat(False)
    for _ in range(10):
      self.feed(speed=0, gear=1)
    self.safety.set_aol_test_heartbeat(True)
    for _ in range(3):
      self.feed(speed=3000, gas=gas, brake=brake)
    self.time += 1
    self.safety.set_timer(self.time)
    self.safety.aol_set_host_request(1)
    return self.permission()

  def test_boot_grant_moving_in_drive(self):
    self.configure()
    self.assertEqual(self._boot(), 1)

  def test_boot_grant_with_the_gas_pressed(self):
    # (drive 00000003: the boot request came while driving off with the gas down and was never granted)
    self.configure()
    self.assertEqual(self._boot(gas=40), 1)

  def test_no_boot_grant_with_the_brake_pressed(self):
    self.configure()
    self.assertEqual(self._boot(brake=True), 0)

  def test_no_boot_grant_with_the_gas_on_the_stock_profile(self):
    # the stock profile (param 0) is unchanged: the gas still waits
    self.PARAM = 0
    self.configure()
    self.assertEqual(self._boot(gas=40), 0)

  def _engage_after_brake_cancel(self, *, gas):
    """Cruise on (token), brake cancel, then the cruise engaged again with or without the gas (as the gas auto-resume)."""
    self.feed(active=True, request=0)
    for _ in range(20):
      self.feed(request=0)
    self.feed(active=True)
    self.feed(active=True, brake=True)
    self.feed(brake=True)
    self.safety.set_aol_test_heartbeat(False)  # the host withdrew: the token is cleared (as after any loss)
    self.feed(brake=True, request=0)
    self.safety.set_aol_test_heartbeat(True)
    for _ in range(3):
      self.feed(gas=gas)
    self.assertEqual(self.permission(), 0)
    self.feed(active=True, gas=gas)              # the engagement edge
    self.feed(active=True)
    self.feed(active=True, brake=True)
    self.feed(brake=True)                        # the next brake cancel
    return self.permission()

  def test_engagement_with_the_gas_pressed_grants_the_token(self):
    # (drive 00000002: every engagement by the gas auto-resume RES gave no token, steering stopped at the next brake)
    self.configure()
    self.assertEqual(self._engage_after_brake_cancel(gas=50), 1)

  def test_engagement_without_the_gas_grants_the_token(self):
    self.configure()
    self.assertEqual(self._engage_after_brake_cancel(gas=0), 1)

  def test_engagement_with_the_gas_on_the_stock_profile_gives_no_token(self):
    from opendbc.safety.tests.test_mazda_stock_aol import TestMazdaStockAol
    stock = TestMazdaStockAol(methodName='run')
    stock.configure()
    for _ in range(20):
      stock.feed()
    stock.safety.safety_rx_hook(stock.packet('ENGINE_DATA', {'SPEED': 80, 'PEDAL_GAS': 50}))
    stock.safety.safety_rx_hook(stock.packet('CRZ_CTRL', {'CRZ_AVAILABLE': 1, 'CRZ_ACTIVE': 0}))
    stock.safety.safety_rx_hook(stock.packet('CRZ_CTRL', {'CRZ_AVAILABLE': 1, 'CRZ_ACTIVE': 1}))
    stock.safety.safety_rx_hook(stock.packet('PEDALS', {'BRAKE_ON': 1}))
    stock.safety.safety_rx_hook(stock.packet('CRZ_CTRL', {'CRZ_AVAILABLE': 1, 'CRZ_ACTIVE': 0}))
    self.safety = stock.safety
    self.assertEqual(stock.safety.aol_get_permission_mask() & 1, 0)

  def test_engagement_with_the_brake_pressed_gives_no_token(self):
    self.configure()
    for _ in range(20):
      self.feed(request=0)
    self.feed(brake=True)                        # (feed sends CRZ_CTRL before PEDALS: the brake is seen from the next frame)
    self.feed(active=True, brake=True)
    self.feed(brake=True)
    self.assertEqual(self.permission(), 0)

  def test_camera_fault_with_the_cruise_off_needs_a_fresh_action(self):
    self.configure()
    self.arm()
    self.feed(cam_err=True)
    self.assertEqual(self.permission(), 0)
    for _ in range(20):
      self.feed()
    self.assertEqual(self.permission(), 0)    # cleared: not back by itself
    self.feed(main=False)
    self.feed()
    self.assertEqual(self.permission(), 1)

  def test_camera_fault_with_the_cruise_engaged_keeps_steering(self):
    # as without AOL: the TI keeps steering through a camera fault while engaged
    self.configure()
    for _ in range(20):
      self.feed(request=0)
    self.feed(active=True)
    for _ in range(100):
      self.feed(active=True, cam_err=True)
      self.assertEqual(self.permission(), 1)
    self.assertEqual(self.steer(10), (True, True))
    self.feed(active=False, cam_err=True)      # the cruise goes off during the fault: no token
    self.assertEqual(self.permission(), 0)
    self.assertEqual(self.steer(20), (False, False))

  def test_main_off_clears_and_main_on_restores(self):
    self.configure()
    self.arm()
    self.feed(main=False)
    self.assertEqual(self.permission(), 0)
    self.feed()
    self.assertEqual(self.permission(), 1)

  def test_nothing_is_forwarded_from_the_camera(self):
    self.configure()
    self.arm()
    self.assertEqual(self.steer(10), (True, True))
    for address in (0x243, 0x440, 0x249):
      self.assertEqual(self.safety.safety_fwd_hook(2, address), -1)
    self.safety.aol_set_host_request(0)
    for address in (0x243, 0x440, 0x249):
      self.assertEqual(self.safety.safety_fwd_hook(2, address), -1)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x249), -1)


class TestMazdaTiAolSameLimits(MazdaTiAolBase):
  """Differential: the same command sequence on the TI without AOL (cruise engaged, controls allowed) and with the TI
  AOL profile (cruise off, the AOL token): every TX verdict must be the same."""

  def _run(self, aol, param, seed, buttons=False):
    rng = random.Random(seed)
    self.PARAM = param | (TI_AOL if aol else 0)
    self.configure(experience=32 if aol else 0)
    if aol:
      self.arm()
    else:
      for _ in range(10):
        self.feed(request=None)
      self.feed(active=True, request=None)
      self.assertTrue(self.safety.get_controls_allowed())
    verdicts = []
    torque = 0
    for _ in range(400):
      driver = rng.choice((0, 0, 0, 5, 20, -20, 60, -60))
      self.feed(active=not aol, driver=driver, request=1 if aol else None)
      torque = max(-800, min(800, torque + rng.choice((-30, -25, -10, -5, 0, 5, 10, 11, 26, 300))))
      ti_value = torque if rng.random() < 0.9 else torque + rng.choice((-1, 1, 10))
      verdicts.append((bool(self.safety.safety_tx_hook(self.lkas(torque))), bool(self.safety.safety_tx_hook(self.ti(ti_value)))))
      if buttons:
        for which in (Buttons.CANCEL, Buttons.RESUME, Buttons.SET_PLUS, Buttons.SET_MINUS):
          verdicts.append(bool(self.safety.safety_tx_hook(button(which))))
    return verdicts

  def test_torque_rate_driver_and_ti_rule_unchanged(self):
    for param in (TI, TI | AR, TI | S700, TI | AR | S700):
      for seed in range(4):
        with self.subTest(param=param, seed=seed):
          self.assertEqual(self._run(False, param, seed), self._run(True, param, seed))

  def test_limits_explicit(self):
    for param, top in ((TI | TI_AOL, 600), (TI | TI_AOL | S700, 700)):
      with self.subTest(param=param):
        self.PARAM = param
        self.configure()
        self.arm()
        for t in (top, -top):
          self.safety.set_desired_torque_last(t)
          self.safety.set_rt_torque_last(t)
          self.assertTrue(self.safety.safety_tx_hook(self.lkas(t)))
          self.assertTrue(self.safety.safety_tx_hook(self.ti(t)))
          self.safety.set_desired_torque_last(t)
          self.safety.set_rt_torque_last(t)
          self.assertFalse(self.safety.safety_tx_hook(self.lkas(t + (1 if t > 0 else -1))))
        self.safety.set_desired_torque_last(0)
        self.safety.set_rt_torque_last(0)
        self.assertFalse(self.safety.safety_tx_hook(self.lkas(11)))   # rate up 10
        self.assertTrue(self.safety.safety_tx_hook(self.lkas(10)))
        self.assertFalse(self.safety.safety_tx_hook(self.ti(9)))      # CAM_LKAS2 repeats the last CAM_LKAS value
        self.assertTrue(self.safety.safety_tx_hook(self.ti(10)))

  def test_buttons_unchanged(self):
    # with the cruise engaged (controls allowed) and with the cruise off: the same button verdicts with and without AOL
    for param in (TI, TI | AR):
      with self.subTest(param=param):
        self.assertEqual(self._button_script(param, False), self._button_script(param, True))

  def _button_script(self, param, aol):
    self.PARAM = param | (TI_AOL if aol else 0)
    self.configure(experience=32 if aol else 0)
    out = []

    def presses():
      return [bool(self.safety.safety_tx_hook(button(b)))
              for b in (Buttons.CANCEL, Buttons.RESUME, Buttons.SET_PLUS, Buttons.SET_MINUS)]
    for _ in range(10):
      self.feed(request=1 if aol else None)
    out.append(presses())                                       # cruise off, main on
    self.feed(active=True, request=1 if aol else None)
    out.append(presses())                                       # engaged
    self.feed(active=True, brake=True, speed=5000, request=1 if aol else None)
    self.feed(active=False, brake=True, speed=5000, request=1 if aol else None)   # brake cancel: auto-resume window
    out.append(presses())
    for _ in range(12):
      self.feed(gas=50, speed=5000, request=1 if aol else None)
      out.append(presses())                                     # gas: RES inside the window (bit 256 only)
    self.feed(main=False, request=1 if aol else None)
    out.append(presses())
    return out


class TestMazdaTiAolOtherCombinationsUnchanged(MazdaTiAolBase):
  """With alternative experience 0 the TI AOL bit changes nothing: every param with it behaves exactly as without it.
  With alternative experience 32 only the TI AOL params (and stock 0) have TX."""

  def _trace(self, param, experience):
    self.configure(experience=experience, param=param)
    out = []
    for step in range(60):
      active = 10 <= step < 30
      self.feed(active=active, main=step != 40, brake=step == 30, gas=50 if step > 32 else 0, driver=(step % 7) - 3,
                request=1 if experience == 32 else None)
      out.append((self.steer((step % 12) * 10 - 40), self.safety.get_controls_allowed(),
                  self.safety.aol_get_permission_mask(), self.safety.safety_fwd_hook(2, 0x243),
                  self.safety.safety_fwd_hook(0, 0x249), self.safety.safety_fwd_hook(2, 0x249)))
      out.append(bool(self.safety.safety_tx_hook(self.packet('CRZ_BTNS', {'RES': 1, 'RES_INV': 0}))))
    return out

  def test_ti_aol_bit_is_inert_without_alternative_experience_32(self):
    for base in (0, TI, AR, S700, TI | AR, TI | S700, TI | AR | S700, AR | S700):
      with self.subTest(param=base):
        self.assertEqual(self._trace(base, 0), self._trace(base | TI_AOL, 0))

  def test_alternative_experience_32_tx_only_for_qualified_params(self):
    for param in (0, 1, 2, TI, TI | AR, TI | S700, AR, S700, TI_AOL, TI_AOL | AR, *TI_AOL_PARAMS, TI | TI_AOL | 1,
                  TI | TI_AOL | 2048, 0xFFFF):
      with self.subTest(param=param):
        self.configure(param=param)
        self.feed()
        self.assertEqual(bool(self.safety.safety_tx_hook(self.lkas(0))), param == 0 or param in TI_AOL_PARAMS)


if __name__ == "__main__":
  unittest.main()


def test_param_values_match_python():
  # mazda.h mazda_ti_aol_param, ti_aol.native_param and pandad's mazda_aol_param (test_aol_protocol.cc) pin the same set
  assert [p for p in range(0x10000) if native_param(p)] == sorted(TI_AOL_PARAMS) == [1032, 1288, 1544, 1800]
