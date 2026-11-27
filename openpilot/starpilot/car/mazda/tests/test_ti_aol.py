"""Always On Lateral with the Mazda Torque Interceptor: the openpilot side (policy, car start composition, native
admission, intent, and every place that consults the Mazda AOL qualification)."""
import re
import tempfile
from pathlib import Path

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, MazdaSafetyFlags
from openpilot.common.params import Params
from openpilot.starpilot.aol.intent import AolSettings, independent_axis_requested, read_settings
from openpilot.starpilot.aol.vehicle import native_matches_cp, native_profile_supported, policy_for
from openpilot.starpilot.car.mazda import aol as mazda_aol
from openpilot.starpilot.car.mazda.stock_intent import MazdaStockCardIntent
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences

ROOT = Path(__file__).resolve().parents[5]
TI = int(MazdaSafetyFlags.TORQUE_INTERCEPTOR)
TI_AOL = int(MazdaSafetyFlags.TI_AOL)
MAZDA = int(structs.CarParams.SafetyModel.mazda)


def put(params, key, value: bool):
  Path(params.get_param_path(key)).write_bytes(b'1' if value else b'0')


def startup(model=CAR.MAZDA_CX5, *, ti=True, aol=True):
  """CarParams as card builds them at car start: startup preferences, then (card.py) the AOL composition."""
  with tempfile.TemporaryDirectory() as directory:
    params = Params(directory)
    put(params, 'TorqueInterceptorEnabled', ti)
    put(params, 'AlwaysOnLateral', aol)
    preferences = VehicleStartupPreferences.read(params, enabled=True)
    cp = preferences.prepare(CarInterface.get_params(model, gen_empty_fingerprint(), [], False, False, False))
    cp.fingerprintSource = structs.CarParams.FingerprintSource.fixed
    cp.alternativeExperience = 0          # card.py
    settings = read_settings(params)
  policy = policy_for(cp)
  # card.py: aol_qualified -> the vehicle's safety config gains the policy's additions
  if independent_axis_requested(settings) and policy.intent_supported and not cp.passive:
    cp.safetyConfigs[0].safetyParam |= policy.safety_param_addition
    cp.alternativeExperience |= policy.alternative_experience_addition
  return cp, policy, settings


def test_ti_with_aol_on_is_marked():
  cp, policy, settings = startup()
  assert settings.enabled
  assert policy.intent_supported and policy.settings_supported and policy.runtime_supported and policy.explicit_latch
  assert (policy.safety_param_addition, policy.alternative_experience_addition) == (TI_AOL, 32)
  assert (int(cp.safetyConfigs[0].safetyParam), int(cp.alternativeExperience)) == (TI | TI_AOL, 32)
  assert policy_for(cp).ordinary_axis_ack_required


def test_aol_off_is_unchanged():
  """With Always On Lateral off the CarParams of a TI car are exactly those of today (no bit, no alternative experience)."""
  cp, policy, settings = startup(aol=False)
  assert not settings.enabled
  assert (int(cp.safetyConfigs[0].safetyParam), int(cp.alternativeExperience)) == (TI, 0)
  with tempfile.TemporaryDirectory() as directory:
    params = Params(directory)
    put(params, 'TorqueInterceptorEnabled', True)
    today = VehicleStartupPreferences.read(params, enabled=True).prepare(
      CarInterface.get_params(CAR.MAZDA_CX5, gen_empty_fingerprint(), [], False, False, False))
  today.fingerprintSource = structs.CarParams.FingerprintSource.fixed
  assert cp.to_dict() == today.to_dict()


def test_without_the_ti_nothing_changes():
  for model in (CAR.MAZDA_CX5, CAR.MAZDA_6):
    cp, policy, _ = startup(model, ti=False)
    assert not policy.intent_supported
    assert (int(cp.safetyConfigs[0].safetyParam), int(cp.alternativeExperience)) == (0, 0)


def test_stock_aol_cars_unchanged():
  for model in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021):
    cp = CarInterface.get_params(model, gen_empty_fingerprint(), [], False, False, False)
    policy = policy_for(cp)
    assert policy.intent_supported and policy.safety_param_addition == 0 and policy.alternative_experience_addition == 32


def test_native_profiles():
  for param in range(0x10000):
    expected = param in (0, 1032, 1288, 1544, 1800)
    assert mazda_aol.native_profile_supported(MAZDA, param) == expected, param
  cp, _, _ = startup()
  assert mazda_aol.native_accepts_cp(cp, MAZDA, TI | TI_AOL)
  assert not mazda_aol.native_accepts_cp(cp, MAZDA, 0)                       # a TI car never takes the stock profile
  assert native_matches_cp(cp, MAZDA, TI | TI_AOL)
  assert not native_matches_cp(cp, MAZDA, TI | TI_AOL | int(MazdaSafetyFlags.AUTO_RESUME))   # must be its own param
  assert not native_matches_cp(cp, int(structs.CarParams.SafetyModel.noOutput), TI | TI_AOL)
  stock = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], False, False, False)
  stock.alternativeExperience = 32
  assert mazda_aol.native_accepts_cp(stock, MAZDA, 0) and not mazda_aol.native_accepts_cp(stock, MAZDA, TI | TI_AOL)
  assert native_profile_supported(MAZDA, TI | TI_AOL)


def test_unmarked_car_has_no_native_admission():
  cp, _, _ = startup(aol=False)
  assert not mazda_aol.native_accepts_cp(cp, MAZDA, TI | TI_AOL)
  assert not mazda_aol.ordinary_axis_request_allowed(cp, structs.CarState(canValid=True))


def settings(**changes):
  values = dict(enabled=True, pause_brake_mps=0., lkas_action=0, main_action=0, distance_actions=(0, 0, 0),
                cancel_actions=(0, 0, 0))
  values.update(changes)
  return AolSettings(**values)


def cs(available=True, **fields):
  state = structs.CarState(canValid=True, gearShifter=structs.CarState.GearShifter.drive, vEgo=10., **fields)
  state.cruiseState.available = available
  return state.as_reader()


def test_intent_follows_the_main_switch_and_ignores_brake_and_gas():
  cp, _, _ = startup()
  intent = mazda_aol.create_intent(cp, settings())
  assert isinstance(intent, MazdaStockCardIntent)
  now = 1_000_000_000
  for available in (False, True):
    intent.update(cs(available), fault_active=False, now_ns=now)
    now += 10_000_000
  intent.update(cs(True), fault_active=False, now_ns=now)
  assert intent.allowed_latch
  for fields in ({'brakePressed': True}, {'gasPressed': True}, {'standstill': True}):
    now += 10_000_000
    intent.update(cs(True, **fields), fault_active=False, now_ns=now)
    assert intent.allowed_latch, fields
  now += 10_000_000
  intent.update(cs(False), fault_active=False, now_ns=now)
  assert not intent.allowed_latch


def test_intent_off_when_the_setting_is_off():
  cp, _, _ = startup()
  intent = mazda_aol.create_intent(cp, settings(enabled=False))
  now = 1_000_000_000
  for available in (False, True, True):
    intent.update(cs(available), fault_active=False, now_ns=now)
    now += 10_000_000
  assert not intent.allowed_latch


def test_pause_on_brake_setting_default_is_zero():
  with tempfile.TemporaryDirectory() as directory:
    params = Params(directory)
    put(params, 'AlwaysOnLateral', True)
    assert read_settings(params).pause_brake_mps == 0.


# Every place that consults the Mazda AOL qualification and what it uses for a TI car.
WIRING = {
  'openpilot/selfdrive/car/card.py': 'aol_qualified as qualified_mazda_aol',
  'openpilot/selfdrive/selfdrived/selfdrived.py': 'aol_qualified as qualified_mazda',
  'openpilot/starpilot/aol/temporary_steering.py': 'aol_qualified as mazda_qualified',
  'openpilot/starpilot/aol/transport_pause.py': 'aol_qualified as mazda_qualified',
  'openpilot/starpilot/car/mazda/stock_intent.py': 'import aol_qualified',
  'openpilot/starpilot/car/mazda/aol.py': 'aol_qualified',
}
# Stock-only on purpose: the Cancel button events and their wheel mapping (a TI car emits no Cancel button event).
STOCK_ONLY = {
  'openpilot/starpilot/controllers/wheel_actions.py': 1,
  'opendbc_repo/opendbc/car/mazda/carstate.py': 1,
}


def test_every_mazda_qualification_use_is_wired():
  pattern = re.compile(r"mazda(?:\.stock_aol| import stock_aol)")
  users = set()
  for path in list(ROOT.joinpath('openpilot').rglob('*.py')) + list(ROOT.joinpath('opendbc_repo/opendbc').rglob('*.py')):
    if '/tests/' in str(path) or '__pycache__' in str(path):
      continue
    text = path.read_text(errors='ignore')
    if pattern.search(text):
      users.add(str(path.relative_to(ROOT)))
  assert users == set(WIRING) | set(STOCK_ONLY), users
  for path, needle in WIRING.items():
    text = (ROOT / path).read_text()
    assert needle in text, path
    assert 'mazda.stock_aol import qualified as' not in text and 'mazda.stock_aol import (qualified' not in text, path
  for path, count in STOCK_ONLY.items():
    assert (ROOT / path).read_text().count('mazda.stock_aol import qualified') == count, path


def test_pandad_profile_source_matches():
  text = (ROOT / 'openpilot/starpilot/car/mazda/aol_policy.h').read_text()
  assert '(param == 0U) || ((param & static_cast<uint16_t>(~optional)) == (8U | 1024U))' in text
  assert 'const uint16_t optional = 256U | 512U;' in text
  mazda_h = (ROOT / 'opendbc_repo/opendbc/safety/modes/mazda.h').read_text()
  assert '#define MAZDA_PARAM_TI_AOL 1024U' in mazda_h


def test_policy_is_frozen_per_car_start():
  cp, _, _ = startup()
  # an unrelated live CarState never changes the policy (fixed at car start)
  assert policy_for(cp) == policy_for(cp.copy())
