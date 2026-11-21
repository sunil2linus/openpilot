"""Gen1 Mazda + Torque Interceptor startup through card and the pandad safety handshake.

FakePandad follows openpilot/selfdrive/pandad/panda_safety.cc (updateMultiplexingMode / fetchCarParams /
setSafetyMode) and panda/board/main.c (which CAN mode each safety mode selects). The car side is the real Mazda
CarInterface fed real CAN frames; TI_FEEDBACK only reaches bus 1 while the panda routes CAN2 to the OBD-II pins.
"""
import tempfile
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.chrysler.interface import CarInterface as ChryslerInterface
from opendbc.car.chrysler.values import CAR as CHRYSLER
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.torque_interceptor import prepare
from opendbc.car.mazda.values import CAR, TI_STATE
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.selfdrive.car.card import Car, EventName, obd_callback
from openpilot.starpilot.car.mazda.ti_startup import TorqueInterceptorStartup, panda_ti_configured, required
from openpilot.starpilot.vehicle_startup import VehicleStartupOwner

DT_NS = 10_000_000  # 100 Hz card loop


class FakePandad:
  def __init__(self, params, cp):
    self.params, self.cp = params, cp
    self.initialized = self.configured = False
    self.prev_obd = False
    self.mode = ('noOutput', 0)

  def loop(self):
    # PandaSafety::configureSafetyMode while onroad and not yet configured
    if self.configured:
      return
    if not self.initialized:
      self.prev_obd = False
      self.mode = ('elm327', 1)
      self.initialized = True
    requested = self.params.get_bool("ObdMultiplexingEnabled")
    if requested != self.prev_obd:
      self.mode = ('elm327', 0 if requested else 1)
      self.prev_obd = requested
      self.params.put_bool("ObdMultiplexingChanged", True)
    if self.params.get_bool("FirmwareQueryDone") and self.params.get_bool("ControlsReady"):
      config = self.cp.safetyConfigs[0]
      self.mode = ('mazda' if config.safetyModel == structs.CarParams.SafetyModel.mazda else str(config.safetyModel),
                   int(config.safetyParam))
      self.configured = True

  def obd_can2(self):
    # main.c: ELM327 param 0 -> CAN_MODE_OBD_CAN2; SAFETY_MAZDA with param bit 8 -> CAN_MODE_OBD_CAN2
    return self.mode == ('elm327', 0) or (self.mode[0] == 'mazda' and bool(self.mode[1] & 8))

  def panda_states(self):
    return [SimpleNamespace(safetyModel=self.mode[0], safetyParam=self.mode[1])]


def car_frames(ci, packer):
  ci.update([(DT_NS, [])])
  frames = []
  for bus, parser in ci.can_parsers.items():
    if bus == Bus.body:
      continue
    for state in parser.message_states.values():
      frames.append(packer.make_can_msg(state.name, parser.bus, {}))
  return frames


class Harness:
  def __init__(self, cp, *, ti_present=True, ti_needs_wakeup=False, obd_car_traffic=False):
    self.directory = tempfile.TemporaryDirectory()
    self.params = Params(self.directory.name)
    self.cp = cp
    self.pandad = FakePandad(self.params, cp)
    self.ci = CarInterface(cp)
    self.packer = CANPacker("mazda_2017")
    self.frames = car_frames(self.ci, self.packer)
    self.ti_present = ti_present
    self.ti_needs_wakeup = ti_needs_wakeup
    self.obd_car_traffic = obd_car_traffic
    self.ti_awake = not ti_needs_wakeup
    self.sendcan = []
    self.t = 2 * DT_NS
    self.events = [SimpleNamespace(name=EventName.selfdriveInitializing)]

  def can(self):
    frames = list(self.frames)
    if self.obd_car_traffic and self.pandad.obd_can2():
      frames.append(self.packer.make_can_msg("ENGINE_DATA", 1, {}))
    if self.ti_present and self.ti_awake and self.pandad.obd_can2():
      frames.append(self.packer.make_can_msg("TI_FEEDBACK", 1, {"STATE": TI_STATE.RUN, "VERSION_NUMBER": 2}))
    self.t += DT_NS
    return self.ci.update([(self.t, frames)])

  def card(self, startup):
    card = Car.__new__(Car)
    card.CP = self.cp
    card.vehicle_startup = VehicleStartupOwner()
    card.ci_initialized = False
    card.initialized_prev = False
    card.aol_replay = False
    card.ioniq6_long_prearmed = False
    card.ioniq6_long_selected = False
    card.can_callbacks = (lambda wait_for_one=False: [], lambda frames: None)
    card.mazda_ti_startup = startup
    card.CI = SimpleNamespace(
      CS=self.ci.CS,
      CC=self.ci.CC,
      can_parsers=self.ci.can_parsers,
      init=Mock(),
      apply=Mock(side_effect=self.ci.apply),
    )
    card.params = self.params
    harness = self

    def pm_send(service, packet):
      if service != 'sendcan':
        return
      msg = messaging.log_from_bytes(packet)
      frames = [(f.address, bytes(f.dat), f.src) for f in msg.sendcan]
      harness.sendcan.append((msg.valid, frames))
      if harness.ti_present and not harness.ti_awake and panda_ti_configured(harness.pandad.panda_states()):
        for addr, dat, src in frames:
          if src == 1 and addr == 0x249 and len(dat) == 8 and int.from_bytes(dat[4:8], 'big') == 3294744160:
            harness.ti_awake = True
            break

    card.pm = SimpleNamespace(send=pm_send)
    card.state_publish = Mock()
    card._deferred_radar_can = None
    card.radar_update = Mock(return_value=None)
    card.radar_publish = Mock()

    def state_update(*, defer_radar=False):
      cs = harness.can()
      harness.cs = cs
      card.sm.valid['carControl'] = bool(cs.canValid)
      return cs, None
    card.state_update = state_update
    card.control = structs.CarControl.new_message().as_reader()

    class SubMaster:
      valid = {'carControl': False}
      alive = {'carControl': True}
      seen = {'onroadEvents': True, 'carControl': True, 'pandaStates': True}
      logMonoTime = {'carControl': 0}
      recv_time = {'carControl': 0.0}

      def __getitem__(self, key):
        return {'onroadEvents': harness.events, 'carControl': card.control, 'pandaStates': harness.pandad.panda_states()}[key]

      def all_alive(self, services):
        return True
    card.sm = SubMaster()
    return card

  def step(self, card):
    card.sm.logMonoTime['carControl'] = time.monotonic_ns()
    card.sm.recv_time['carControl'] = time.monotonic()
    card.step()
    self.pandad.loop()

  def after_fingerprint(self):
    # get_car() ends with fingerprint()'s set_obd_multiplexing(False); pandad answers it
    self.pandad.loop()
    self.params.put_bool("ObdMultiplexingEnabled", False, block=True)
    self.pandad.loop()
    assert self.pandad.mode == ('elm327', 1)


def ti_cp(model=CAR.MAZDA_CX5):
  return prepare(CarInterface.get_params(model, gen_empty_fingerprint(), [], False, False, False), True)


def test_without_the_request_the_ti_car_never_leaves_elm327():
  h = Harness(ti_cp())
  h.after_fingerprint()
  h.params.put_bool("FirmwareQueryDone", True, block=True)
  card = h.card(None)
  for _ in range(300):
    h.step(card)
  h.events = []
  for _ in range(300):
    h.step(card)
  assert not h.cs.canValid and not h.params.get_bool("ControlsReady")
  assert h.pandad.mode == ('elm327', 1)


def test_full_startup_sequence():
  h = Harness(ti_cp())
  h.after_fingerprint()
  startup = TorqueInterceptorStartup(h.params, sleep=lambda _: h.pandad.loop())
  assert startup.request(h.cp, h.pandad.panda_states()) == 'requested'
  assert h.pandad.mode == ('elm327', 0) and h.pandad.obd_can2()  # relay open, no car TX in ELM327
  h.params.put_bool("FirmwareQueryDone", True, block=True)
  card = h.card(startup)
  steps_to_valid = None
  for i in range(1, 50):
    h.step(card)
    if h.cs.canValid and steps_to_valid is None:
      steps_to_valid = i
  assert steps_to_valid is not None and steps_to_valid * DT_NS <= 100_000_000  # well under selfdrived's 6 s
  assert not h.cs.canTimeout
  assert not h.params.get_bool("ControlsReady")  # still selfdriveInitializing
  h.events = []  # selfdrived initialized (carState valid)
  h.step(card)
  assert h.params.get_bool("ControlsReady") and card.ci_initialized
  assert h.pandad.mode == ('mazda', 8) and h.pandad.obd_can2()
  assert h.params.get_bool("ObdMultiplexingEnabled")
  h.step(card)  # pandaStates shows Mazda + TI: the multiplexing request is withdrawn (asynchronous write)
  assert not startup.pending
  assert wait_for(lambda: not h.params.get_bool("ObdMultiplexingEnabled"))
  for _ in range(20):
    h.step(card)
    assert h.cs.canValid and h.pandad.mode == ('mazda', 8)
  card.CI.init.assert_called_once()


def test_fail_closed_without_a_ti():
  h = Harness(ti_cp(), ti_present=False)
  h.after_fingerprint()
  startup = TorqueInterceptorStartup(h.params, sleep=lambda _: h.pandad.loop())
  assert startup.request(h.cp, h.pandad.panda_states()) == 'requested'
  h.params.put_bool("FirmwareQueryDone", True, block=True)
  card = h.card(startup)
  for _ in range(600):  # past selfdrived's 6 s initialization timeout
    h.step(card)
  h.events = []
  for _ in range(100):
    h.step(card)
  assert not h.cs.canValid and h.cs.canTimeout
  assert not h.params.get_bool("ControlsReady") and not card.ci_initialized
  assert h.pandad.mode == ('elm327', 0)  # never the car safety mode: relay open, stock car
  card.CI.apply.assert_not_called()


def wait_for(condition, timeout=1.0):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if condition():
      return True
    time.sleep(0.005)
  return condition()


def call_with_timeout(function, timeout=2.0):
  done = threading.Event()
  thread = threading.Thread(target=lambda: (function(), done.set()), daemon=True)
  thread.start()
  return done.wait(timeout)


@pytest.mark.parametrize("stale_request", (False, True))
def test_card_restart_mid_drive(stale_request):
  h = Harness(ti_cp())
  h.after_fingerprint()
  for key in ("FirmwareQueryDone", "ControlsReady"):
    h.params.put_bool(key, True, block=True)
  h.params.put_bool("ObdMultiplexingEnabled", True, block=True)
  h.pandad.loop()
  assert h.pandad.configured and h.pandad.mode == ('mazda', 8)
  if not stale_request:
    h.params.put_bool("ObdMultiplexingEnabled", False, block=True)  # withdrawn by the first card run
  sleep = Mock()
  startup = TorqueInterceptorStartup(h.params, sleep=sleep)
  assert startup.request(h.cp, h.pandad.panda_states()) == 'already_configured'
  sleep.assert_not_called()
  assert h.params.get_bool("ObdMultiplexingEnabled") is stale_request
  startup.maintain(h.pandad.panda_states())
  assert wait_for(lambda: not h.params.get_bool("ObdMultiplexingEnabled"))
  # the next card restart's fingerprint() set_obd_multiplexing(False) returns at once (pandad no longer answers)
  assert call_with_timeout(lambda: obd_callback(h.params)(False))
  assert h.pandad.mode == ('mazda', 8)


def test_restart_without_panda_states_uses_controls_ready():
  h = Harness(ti_cp())
  h.params.put_bool("ControlsReady", True, block=True)
  sleep = Mock()
  startup = TorqueInterceptorStartup(h.params, sleep=sleep)
  assert startup.request(h.cp, None) == 'already_configured'
  sleep.assert_not_called()
  assert not h.params.get_bool("ObdMultiplexingEnabled")


def test_request_never_waits_forever():
  with tempfile.TemporaryDirectory() as directory:
    params = Params(directory)
    now = [0.0]
    startup = TorqueInterceptorStartup(params, clock=lambda: now[0], sleep=lambda dt: now.__setitem__(0, now[0] + dt))
    assert startup.request(ti_cp(), [SimpleNamespace(safetyModel='elm327', safetyParam=1)]) == 'timeout'
    assert 2.0 <= now[0] < 2.1


def test_only_ti_mazdas_are_affected():
  for model in CAR:
    stock = CarInterface.get_params(model, gen_empty_fingerprint(), [], False, False, False)
    assert not required(stock)
    assert required(prepare(CarInterface.get_params(model, gen_empty_fingerprint(), [], False, False, False), True))
  passive = ti_cp()
  passive.passive = True
  assert not required(passive)
  assert not required(ChryslerInterface.get_params(CHRYSLER.JEEP_GRAND_CHEROKEE, gen_empty_fingerprint(), [], False, False, False))
  with tempfile.TemporaryDirectory() as directory:
    params = Params(directory)
    startup = TorqueInterceptorStartup(params, sleep=Mock(side_effect=AssertionError))
    assert startup.request(CarInterface.get_params(CAR.MAZDA_CX5, gen_empty_fingerprint(), [], False, False, False), None) == 'not_required'
    assert params.get("ObdMultiplexingEnabled") is None


def test_panda_state_check():
  assert panda_ti_configured([SimpleNamespace(safetyModel='mazda', safetyParam=8)])
  assert panda_ti_configured([SimpleNamespace(safetyModel=SimpleNamespace(raw=13), safetyParam=8)])
  assert not panda_ti_configured([SimpleNamespace(safetyModel='mazda', safetyParam=0)])
  assert not panda_ti_configured([SimpleNamespace(safetyModel='elm327', safetyParam=8)])
  assert not panda_ti_configured([])
  assert not panda_ti_configured(None)


def _ti_wakeup_frames(frames):
  return [(addr, dat, src) for addr, dat, src in frames if src == 1 and addr == 0x249]


def _cam_lkas2_raw_torque(dat):
  return ((dat[0] & 0x0F) << 8) | dat[1]


def test_full_startup_sequence_with_ti_wakeup():
  h = Harness(ti_cp(), ti_present=True, ti_needs_wakeup=True, obd_car_traffic=True)
  try:
    h.after_fingerprint()
    startup = TorqueInterceptorStartup(h.params, sleep=lambda _: h.pandad.loop())
    assert startup.request(h.cp, h.pandad.panda_states()) == 'requested'
    assert h.pandad.mode == ('elm327', 0) and h.pandad.obd_can2()
    h.params.put_bool("FirmwareQueryDone", True, block=True)
    card = h.card(startup)

    # Step 1: bus 1 has OBD traffic but no TI_FEEDBACK; card sets ControlsReady early while panda was elm327
    h.step(card)
    assert wait_for(lambda: h.params.get_bool("ControlsReady"))
    h.pandad.loop()
    assert not h.cs.canValid
    assert card.ci_initialized
    assert h.pandad.mode == ('mazda', 8)
    assert panda_ti_configured(h.pandad.panda_states())
    assert h.sendcan == []
    assert not h.ti_awake

    # Step 2: panda is in SAFETY_MAZDA(8); card sends inactive zero-torque 0x249 wakeup with valid=False
    h.step(card)
    assert not h.cs.canValid
    assert len(h.sendcan) == 1
    valid, frames = h.sendcan[0]
    assert valid is False
    ti_frames = _ti_wakeup_frames(frames)
    assert len(ti_frames) == 1
    assert _cam_lkas2_raw_torque(ti_frames[0][1]) == 2048
    assert int.from_bytes(ti_frames[0][1][4:8], 'big') == 3294744160
    assert h.ti_awake
    assert not startup.pending
    assert wait_for(lambda: not h.params.get_bool("ObdMultiplexingEnabled"))

    # Step 3: TI is now awake and emitting TI_FEEDBACK, but selfdrived is still initializing -> keepalive with valid=True
    h.step(card)
    assert h.cs.canValid
    assert len(h.sendcan) == 2
    valid, frames = h.sendcan[1]
    assert valid is True
    ti_frames = _ti_wakeup_frames(frames)
    assert len(ti_frames) == 1
    assert _cam_lkas2_raw_torque(ti_frames[0][1]) == 2048
    assert int.from_bytes(ti_frames[0][1][4:8], 'big') == 3294744160

    # Step 4: selfdrived initialization finishes -> normal controls_update takes over
    h.events = []
    cc = structs.CarControl.new_message()
    cc.latActive = True
    cc.actuators.torque = 0.5
    card.control = cc.as_reader()
    h.step(card)
    assert h.cs.canValid
    assert len(h.sendcan) == 3
    assert h.sendcan[2][0] is True
    assert card.CI.apply.call_args_list[-1].args[0].latActive is True
    card.CI.init.assert_called_once()
  finally:
    h.directory.cleanup()


def test_fail_closed_bus1_traffic_without_ti():
  h = Harness(ti_cp(), ti_present=False, obd_car_traffic=True)
  try:
    h.after_fingerprint()
    startup = TorqueInterceptorStartup(h.params, sleep=lambda _: h.pandad.loop())
    assert startup.request(h.cp, h.pandad.panda_states()) == 'requested'
    assert h.pandad.mode == ('elm327', 0) and h.pandad.obd_can2()
    h.params.put_bool("FirmwareQueryDone", True, block=True)
    card = h.card(startup)

    # Even if onroadEvents lacks selfdriveInitializing and CarControl requests torque, controls_update must not run
    h.events = []
    cc = structs.CarControl.new_message()
    cc.latActive = True
    cc.actuators.torque = 0.5
    card.control = cc.as_reader()

    h.step(card)
    assert wait_for(lambda: h.params.get_bool("ControlsReady"))
    h.pandad.loop()
    for _ in range(5):
      h.step(card)

    assert not h.cs.canValid
    assert len(h.sendcan) >= 5
    assert all(valid is False for valid, _ in h.sendcan)
    assert all(_cam_lkas2_raw_torque(dat) == 2048 for _, frames in h.sendcan for _, dat, _ in _ti_wakeup_frames(frames))
    assert all(not call.args[0].latActive for call in card.CI.apply.call_args_list)
  finally:
    h.directory.cleanup()
