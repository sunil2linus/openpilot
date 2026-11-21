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
