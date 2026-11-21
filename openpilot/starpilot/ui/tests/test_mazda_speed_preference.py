"""The Mazda speed control (buttons) rows: Gen1 Mazda only, master switch off by default and confirmed to turn on,
sub-options only while it is on, all saved into the one JSON document; invalid documents are off and repairable."""
from dataclasses import replace
import json
import tempfile
from pathlib import Path

from opendbc.car import gen_empty_fingerprint
from opendbc.car.chrysler.interface import CarInterface as ChryslerInterface
from opendbc.car.chrysler.values import CAR as CHRYSLER
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.torque_interceptor import prepare
from opendbc.car.mazda.values import CAR
from openpilot.common.params import Params
from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.galaxy.setting_descriptions import purpose
from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
from openpilot.starpilot.ui.feature_settings_state import row_change
from openpilot.starpilot.ui.mazda_speed_feature import MASTER, PREFIX, RESET
from openpilot.starpilot.ui.vehicle_bool import VEHICLE_BOOL_KEYS, confirmation_question


def owner_for(saved, cp, groups):
  return FeatureSettingsOwner(saved, lambda group: groups.get(group, True), vehicle_fingerprint=lambda: cp.carFingerprint,
                              vehicle_params=lambda: cp)


def rows(owner, parked=True):
  return [r for r in owner.snapshot('vehicle', parked=parked, system_long=False, lateral_context=True, metric=False).rows
          if r.key.startswith(PREFIX)]


def ti_cp():
  return prepare(CarInterface.get_params(CAR.MAZDA_CX5, gen_empty_fingerprint(), [], False, False, False), True)


def test_master_off_by_default_confirmed_on_and_sub_options():
  with tempfile.TemporaryDirectory() as d:
    saved = Params(d)
    groups = {'vehicle': True, 'parked_preferences': True}
    owner = owner_for(saved, ti_cp(), groups)
    shown = rows(owner)
    assert [r.key for r in shown] == [MASTER] and shown[0].value == 'Off'
    assert all(purpose(r) for r in shown)
    request = row_change(shown[0])
    assert request.value == 'On' and MASTER in VEHICLE_BOOL_KEYS and 'SET+' in confirmation_question(request)
    assert not owner.apply(request)                       # needs the confirmation
    groups['parked_preferences'] = False
    assert not owner.apply(replace(request, confirmation=True))
    groups['parked_preferences'] = True
    assert owner.apply(replace(request, confirmation=True))
    assert C.read(saved).enabled and not C.read(saved).any_active
    shown = {r.key: r for r in rows(owner)}
    assert set(shown) == {PREFIX + k for k in C.SCHEMA}
    for r in shown.values():
      assert purpose(r), r.key
    # a mode row cycles Off -> Test -> On, saved in the one document
    curves = shown[PREFIX + 'curves']
    assert curves.value == 'Off'
    assert owner.apply(row_change(curves))
    assert C.read(saved).mode('curves') == 'test'
    # a number row steps and stays in bounds
    cap = {r.key: r for r in rows(owner)}[PREFIX + 'side_force_cap']
    assert cap.value == '3.0'
    assert owner.apply(row_change(cap, 1)) and C.read(saved).side_force_cap == 3.1
    offset = {r.key: r for r in rows(owner)}[PREFIX + 'curve_offset_mph']
    assert owner.apply(row_change(offset, 1)) and C.read(saved).curve_offset_mph == 1
    # a stale source is refused
    assert not owner.apply(row_change(offset, 1))
    # off again: the sub-options keep their values but are hidden and inactive
    master = {r.key: r for r in rows(owner)}[MASTER]
    assert owner.apply(replace(row_change(master), confirmation=True))
    cfg = C.read(saved)
    assert not cfg.enabled and cfg.curves == 'test' and not cfg.any_active
    assert [r.key for r in rows(owner)] == [MASTER]


def test_unavailable_while_driving_and_only_for_mazda():
  with tempfile.TemporaryDirectory() as d:
    saved = Params(d)
    shown = rows(owner_for(saved, ti_cp(), {}), parked=False)
    assert shown and not shown[0].available
    cp = ChryslerInterface.get_params(CHRYSLER.JEEP_GRAND_CHEROKEE, gen_empty_fingerprint(), [], False, False, False)
    assert rows(owner_for(saved, cp, {})) == []


def test_invalid_document_is_off_and_repairable():
  with tempfile.TemporaryDirectory() as d:
    saved = Params(d)
    Path(saved.get_param_path(C.KEY)).write_bytes(json.dumps({"version": 1, "enabled": True, "curves": "bad"}).encode())
    assert not C.read(saved).enabled
    owner = owner_for(saved, ti_cp(), {'vehicle': True, 'parked_preferences': True})
    shown = rows(owner)
    assert [r.key for r in shown] == [RESET] and shown[0].repair_value == 'Off'
    request = row_change(shown[0])
    assert not owner.apply(replace(request, value='On'))
    assert owner.apply(request)
    assert Path(saved.get_param_path(C.KEY)).read_bytes() == b'{"version":1}'
    assert [r.key for r in rows(owner)] == [MASTER]
