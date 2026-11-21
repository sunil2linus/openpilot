"""The Torque Interceptor (Mazda) row: shown only for Gen1 Mazda, saved parked with confirmation, read at startup."""
from dataclasses import replace
import tempfile

from opendbc.car import gen_empty_fingerprint
from opendbc.car.chrysler.interface import CarInterface as ChryslerInterface
from opendbc.car.chrysler.values import CAR as CHRYSLER
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.torque_interceptor import prepare
from opendbc.car.mazda.values import CAR
from openpilot.common.params import Params
from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
from openpilot.starpilot.ui.feature_settings_state import row_change
from openpilot.starpilot.ui.mazda_ti_feature import capable
from openpilot.starpilot.ui.vehicle_bool import VEHICLE_BOOL_KEYS, confirmation_question
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences

KEY = "TorqueInterceptorEnabled"


def owner_for(saved, cp, groups):
  return FeatureSettingsOwner(saved, lambda group: groups.get(group, True), vehicle_fingerprint=lambda: cp.carFingerprint,
                              vehicle_params=lambda: cp)


def vehicle_rows(owner, parked=True):
  return owner.snapshot('vehicle', parked=parked, system_long=False, lateral_context=True, metric=False).rows


def ti_row(owner, parked=True):
  return next((row for row in vehicle_rows(owner, parked) if row.key == KEY), None)


def test_row_saves_and_reaches_startup():
  with tempfile.TemporaryDirectory() as directory:
    saved = Params(directory)
    cp = CarInterface.get_params(CAR.MAZDA_CX5, gen_empty_fingerprint(), [], False, False, False)
    groups = {'vehicle': True, 'parked_preferences': True}
    owner = owner_for(saved, cp, groups)
    row = ti_row(owner)
    assert row is not None and row.value == 'Off' and row.label == 'Torque Interceptor (Mazda)'
    request = row_change(row)
    assert request is not None and request.value == 'On'
    assert not owner.apply(request)  # confirmation required
    assert KEY in VEHICLE_BOOL_KEYS and 'Torque Interceptor' in confirmation_question(request)
    groups['parked_preferences'] = False
    assert not owner.apply(replace(request, confirmation=True))
    groups['parked_preferences'] = True
    assert owner.apply(replace(request, confirmation=True))
    assert saved.get_bool(KEY)
    assert VehicleStartupPreferences.read(saved, enabled=True).mazda_torque_interceptor

    # after the next startup the running CarParams carry the TI: the row stays, shown On, and can be turned off
    owner = owner_for(saved, prepare(cp, True), groups)
    row = ti_row(owner)
    assert row is not None and row.value == 'On'
    assert owner.apply(replace(row_change(row), confirmation=True))
    assert not saved.get_bool(KEY)


def test_row_unavailable_while_driving():
  with tempfile.TemporaryDirectory() as directory:
    saved = Params(directory)
    cp = CarInterface.get_params(CAR.MAZDA_CX5, gen_empty_fingerprint(), [], False, False, False)
    row = ti_row(owner_for(saved, cp, {}), parked=False)
    assert row is not None and not row.available


def test_row_only_for_mazda():
  with tempfile.TemporaryDirectory() as directory:
    saved = Params(directory)
    cp = ChryslerInterface.get_params(CHRYSLER.JEEP_GRAND_CHEROKEE, gen_empty_fingerprint(), [], False, False, False)
    assert ti_row(owner_for(saved, cp, {})) is None


def passive_cp(model=CAR.MAZDA_CX5):
  """What card.py saves for a 2017-21 Gen1 Mazda without the TI: dashcam, passive, [noOutput, 0]."""
  from opendbc.car import structs
  cp = CarInterface.get_params(model, gen_empty_fingerprint(), [], False, False, False)
  assert cp.dashcamOnly
  cp.passive = True
  config = structs.CarParams.SafetyConfig()
  config.safetyModel = structs.CarParams.SafetyModel.noOutput
  cp.safetyConfigs = [config]
  return cp


def card_passive(cp, openpilot_enabled=True):
  # card.py: controller_available = CC is not None and enabled and not dashcamOnly; passive = not available or dashcamOnly
  return not (openpilot_enabled and not cp.dashcamOnly) or cp.dashcamOnly


def test_row_shown_for_dashcam_mazda_and_toggle_works_from_dashcam_state():
  with tempfile.TemporaryDirectory() as directory:
    saved = Params(directory)
    for model in (CAR.MAZDA_CX5, CAR.MAZDA_CX9, CAR.MAZDA_3, CAR.MAZDA_6):
      assert capable(passive_cp(model))
    cp = passive_cp()
    groups = {'vehicle': True, 'parked_preferences': True}
    owner = owner_for(saved, cp, groups)
    from openpilot.starpilot.ui.feature_settings_state import FeaturePage
    main = owner.snapshot(FeaturePage.HUB, parked=True, system_long=False, lateral_context=True, metric=False).rows
    assert any(row.label == 'Vehicle Settings' for row in main)
    row = ti_row(owner)
    assert row is not None and row.value == 'Off'
    assert owner.apply(replace(row_change(row), confirmation=True))
    assert saved.get_bool(KEY)
    # next startup: prepare() runs on the fresh interface params (safetyModel mazda), not on the saved passive CP
    fresh = CarInterface.get_params(CAR.MAZDA_CX5, gen_empty_fingerprint(), [], False, False, False)
    started = VehicleStartupPreferences.read(saved, enabled=True).prepare(fresh)
    assert not started.dashcamOnly and not card_passive(started)
    assert [(int(getattr(c.safetyModel, 'raw', c.safetyModel)), int(c.safetyParam)) for c in started.safetyConfigs] == [(13, 8)]
    assert started.minSteerSpeed == 0.0


def test_row_through_galaxy_configuration_context():
  from opendbc.car.structs import car
  from openpilot.starpilot.galaxy.vehicle_configuration import configuration_context
  with tempfile.TemporaryDirectory() as directory:
    saved = Params(directory)
    physical = car.CarParams(**passive_cp().to_dict()).as_reader()
    cp, _ = configuration_context(saved, physical, b'raw')
    assert cp is not None and not cp.passive and str(cp.safetyConfigs[0].safetyModel) == 'noOutput'
    assert capable(cp)
    assert ti_row(owner_for(saved, cp, {})) is not None


def test_not_capable_configurations():
  cp = CarInterface.get_params(CAR.MAZDA_CX5, gen_empty_fingerprint(), [], False, False, False)
  cp.safetyConfigs[0].safetyParam = 1
  assert not capable(cp)
  cp = passive_cp()
  cp.safetyConfigs[0].safetyParam = 2
  assert not capable(cp)
  cp = passive_cp()
  cp.notCar = True
  assert not capable(cp)
  assert not capable(None)


def test_invalid_saved_value_is_off_and_repairable():
  with tempfile.TemporaryDirectory() as directory:
    saved = Params(directory)
    with open(saved.get_param_path(KEY), 'wb') as f:
      f.write(b"1\n")  # what `echo 1 > ...` writes
    assert not VehicleStartupPreferences.read(saved, enabled=True).mazda_torque_interceptor  # invalid -> Off
    owner = owner_for(saved, passive_cp(), {'vehicle': True, 'parked_preferences': True})
    row = ti_row(owner)
    assert row is not None and row.value == 'Invalid saved choice' and row.repair_value == 'Off' and row.available
    request = row_change(row)
    assert request is not None and request.value == 'Off'
    assert not owner.apply(replace(request, value='On', confirmation=True))  # never On from an invalid value
    assert owner.apply(replace(request, confirmation=True))
    with open(saved.get_param_path(KEY), 'rb') as f:
      assert f.read() == b"0"
