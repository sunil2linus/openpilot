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
    # (Stage 3) the steering limit test row is shown without the master switch on a TI car
    assert [r.key for r in shown] == [MASTER, PREFIX + 'ti_steer_700'] and shown[0].value == 'Off'
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
    lat = {r.key: r for r in rows(owner)}[PREFIX + 'plan_lat']
    assert lat.value == '2.2' and lat.minimum == 1.8 and lat.maximum == 3.5
    assert owner.apply(row_change(lat, 1)) and C.read(saved).plan_lat == 2.3
    # a stale source is refused
    assert not owner.apply(row_change(lat, 1))
    # (v4) the stop slowdown is a mode row, the notices and their chime On / Off rows, all off by default
    shown = {r.key: r for r in rows(owner)}
    assert shown[PREFIX + 'stops'].value == 'Off' and shown[PREFIX + 'stops'].choices == ('Off', 'Test (log only)', 'On')
    assert "cannot stop the car" in shown[PREFIX + 'stops'].reason and "You brake" in shown[PREFIX + 'stops'].reason
    for k in ('notices', 'notice_sound'):
      assert shown[PREFIX + k].value == 'Off' and shown[PREFIX + k].choices == ('Off', 'On')
    assert "Bookmark" in shown[PREFIX + 'notices'].reason and "Quick Select" in shown[PREFIX + 'notices'].reason
    assert owner.apply(row_change(shown[PREFIX + 'notices']))
    assert C.read(saved).notices is True and C.read(saved).stops == 'off'
    assert owner.apply(row_change({r.key: r for r in rows(owner)}[PREFIX + 'stops']))
    assert C.read(saved).mode('stops') == 'test' and C.read(saved).shows_notices
    # off again: the sub-options keep their values but are hidden and inactive
    master = {r.key: r for r in rows(owner)}[MASTER]
    assert owner.apply(replace(row_change(master), confirmation=True))
    cfg = C.read(saved)
    assert not cfg.enabled and cfg.curves == 'test' and not cfg.any_active
    assert [r.key for r in rows(owner)] == [MASTER, PREFIX + 'ti_steer_700']


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
    assert Path(saved.get_param_path(C.KEY)).read_bytes() == b'{"version":6}'
    assert [r.key for r in rows(owner)] == [MASTER, PREFIX + 'ti_steer_700']


def test_speed_limit_merge_and_exit_rows_say_when_the_map_service_will_not_run(monkeypatch):
  import openpilot.starpilot.ui.mazda_speed_feature as F
  from openpilot.starpilot.maps import shadow_lifecycle as S

  from openpilot.starpilot.car.mazda.speed import status as ST

  def reasons(saved):
    owner = owner_for(saved, ti_cp(), {'vehicle': True, 'parked_preferences': True})
    return {r.key: r.reason for r in rows(owner)}
  with tempfile.TemporaryDirectory() as d, tempfile.TemporaryDirectory() as status_dir:
    monkeypatch.setattr(ST, "STATUS_DIR", status_dir)      # no status file from the two helpers
    saved = Params(d)
    Path(saved.get_param_path(C.KEY)).write_bytes(json.dumps({"version": 3, "enabled": True}).encode())
    monkeypatch.setattr(S, "_owned_root", lambda root, anchor: True)
    monkeypatch.setattr(S, "_selected_snapshot", lambda root: False)
    got = reasons(saved)                                  # 7.0's defaults: Show Speed Limits on, Map Data a source
    for k in F.MAP_ROWS:
      assert "Now: no map speed limits: download offline maps in Maps settings." in got[PREFIX + k], got[PREFIX + k]
    assert "Now:" not in got[PREFIX + "curves"]
    monkeypatch.setattr(S, "_selected_snapshot", lambda root: True)
    assert "Now:" not in reasons(saved)[PREFIX + "merge"]
    put = lambda key, raw: Path(saved.get_param_path(key)).write_bytes(raw)  # noqa: E731
    put("ShowSpeedLimits", b"0")
    assert "turn on Show Speed Limits" in reasons(saved)[PREFIX + "speed_limit"]
    put("ShowSpeedLimits", b"1")
    for key, value in (("SLCPriority1", b"Vision"), ("SLCPriority2", b"Off"), ("SLCPriority3", b"Off")):
      put(key, value)
    got = reasons(saved)[PREFIX + "exits"]
    assert "add Map Data to the speed limit sources" in got and "Show Speed Limits" not in got
    assert all("MapdShadow" not in r for r in reasons(saved).values())


def test_rows_show_the_curve_map_build_and_the_last_drives_gps(monkeypatch):
  from openpilot.starpilot.car.mazda.speed import status as ST
  import openpilot.starpilot.ui.mazda_speed_feature as F
  with tempfile.TemporaryDirectory() as d, tempfile.TemporaryDirectory() as status_dir:
    monkeypatch.setattr(ST, "STATUS_DIR", status_dir)
    monkeypatch.setattr(F, "map_service_status", lambda params, cp: "")
    saved = Params(d)
    Path(saved.get_param_path(C.KEY)).write_bytes(json.dumps({"version": 3, "enabled": True}).encode())
    ST.write(ST.BUILD_NAME, {"state": ST.BUILDING, "built": 3, "total": 5, "failed": 0})
    ST.write(ST.GPS_NAME, {"state": ST.GPS_NO_MESSAGES, "src": ""})
    owner = owner_for(saved, ti_cp(), {'vehicle': True, 'parked_preferences': True})
    got = {r.key: r.reason for r in rows(owner)}
    for k in ("curves", "merge", "exits"):
      assert "Now: curve maps: building… 3 of 5 map areas built" in got[PREFIX + k], got[PREFIX + k]
    for k in ("curves", "speed_limit", "merge", "exits"):
      assert "last drive: no GPS position: no message from the GPS" in got[PREFIX + k], got[PREFIX + k]
    assert "curve maps" not in got[PREFIX + "speed_limit"] and "Now:" not in got[PREFIX + "camera_curves"]


def test_stage3_rows_confirmed_on_and_the_steering_test_only_with_the_ti():
  with tempfile.TemporaryDirectory() as d:
    saved = Params(d)
    groups = {'vehicle': True, 'parked_preferences': True}
    owner = owner_for(saved, ti_cp(), groups)
    # the steering limit test: shown with the master switch off, Off by default, confirmed to turn on
    steer = {r.key: r for r in rows(owner)}[PREFIX + 'ti_steer_700']
    assert steer.value == 'Off' and steer.choices == ('Off', 'On') and 'Test only' in steer.reason
    assert '600 to 700' in steer.reason and 'next drive' in steer.reason
    request = row_change(steer)
    assert request.key in VEHICLE_BOOL_KEYS and '700' in confirmation_question(request)
    assert not owner.apply(request)
    assert owner.apply(replace(request, confirmation=True))
    assert C.read(saved).steer_700_on and not C.read(saved).any_active
    # off needs no question
    assert owner.apply(row_change({r.key: r for r in rows(owner)}[PREFIX + 'ti_steer_700']))
    assert not C.read(saved).ti_steer_700
    # the gas auto-resume: a sub-option of the master switch, confirmed to turn on, says it applies at the next drive
    master = {r.key: r for r in rows(owner)}[MASTER]
    assert owner.apply(replace(row_change(master), confirmation=True))
    ar = {r.key: r for r in rows(owner)}[PREFIX + 'auto_resume']
    assert ar.value == 'Off' and 'RES' in ar.reason and 'Panda' in ar.reason and 'next drive' in ar.reason
    request = row_change(ar)
    assert request.key in VEHICLE_BOOL_KEYS and 'RES' in confirmation_question(request)
    assert not owner.apply(request)
    assert owner.apply(replace(request, confirmation=True))
    assert C.read(saved).auto_resume_on and C.read(saved).any_active
    # without the TI: no steering test row, and it cannot be turned on
    stock = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], False, False, False)
    stock_owner = owner_for(saved, stock, groups)
    keys = [r.key for r in rows(stock_owner)]
    assert PREFIX + 'ti_steer_700' not in keys and PREFIX + 'auto_resume' in keys
    fake = replace(row_change(steer), confirmation=True, expected=Path(saved.get_param_path(C.KEY)).read_bytes(),
                   capability=('MAZDA_CX5_2022', 'mazda', 'mazda-gen1-speed-buttons'), vehicle_fingerprint='MAZDA_CX5_2022')
    assert not stock_owner.apply(fake) and not C.read(saved).ti_steer_700


def test_rows_stay_with_a_stage3_bit_in_the_saved_carparams():
  # (Stage 3) the auto-resume bit in the safety param must not hide the Mazda rows (mazda_ti_feature.capable)
  from opendbc.car.mazda.values import MazdaSafetyFlags
  with tempfile.TemporaryDirectory() as d:
    saved = Params(d)
    cp = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], False, False, False)
    cp.safetyConfigs[0].safetyParam = int(MazdaSafetyFlags.AUTO_RESUME)
    assert [r.key for r in rows(owner_for(saved, cp, {}))] == [MASTER]
    cp = ti_cp()
    cp.safetyConfigs[0].safetyParam |= int(MazdaSafetyFlags.AUTO_RESUME | MazdaSafetyFlags.TI_STEER_700)
    assert [r.key for r in rows(owner_for(saved, cp, {}))] == [MASTER, PREFIX + 'ti_steer_700']


def test_curve_map_build_range_row(monkeypatch):
  from openpilot.starpilot.car.mazda.speed import status as ST
  import openpilot.starpilot.ui.mazda_speed_feature as F
  with tempfile.TemporaryDirectory() as d, tempfile.TemporaryDirectory() as status_dir:
    monkeypatch.setattr(ST, "STATUS_DIR", status_dir)
    monkeypatch.setattr(F, "map_service_status", lambda params, cp: "")
    saved = Params(d)
    Path(saved.get_param_path(C.KEY)).write_bytes(json.dumps({"version": 5, "enabled": True}).encode())  # an old document
    owner = owner_for(saved, ti_cp(), {'vehicle': True, 'parked_preferences': True})
    shown = rows(owner)
    keys = [r.key for r in shown]
    assert keys.index(PREFIX + 'curve_range_mi') == keys.index(PREFIX + 'curves') + 1     # next to Curves from the map
    row = {r.key: r for r in shown}[PREFIX + 'curve_range_mi']
    assert row.label == 'Curve map build range' and row.value == '100 mi' and row.default_value == '100 mi'
    assert row.choices == ('30 mi', '50 mi', '100 mi', '200 mi', '300 mi', 'Whole map')
    assert '2-3 minutes' in row.reason and 'parked with power' in row.reason and 'no reboot' in row.reason
    assert 'next drive' not in row.reason and purpose(row)
    # each press steps to the next choice, saved in the one document (version 6); Whole map is 0
    want = [200, 300, 0, 30, 50, 100]
    for miles in want:
      row = {r.key: r for r in rows(owner)}[PREFIX + 'curve_range_mi']
      assert owner.apply(row_change(row))
      assert C.read(saved).curve_range_mi == miles
    assert Path(saved.get_param_path(C.KEY)).read_bytes() == b'{"enabled":true,"version":6}'
    row = {r.key: r for r in rows(owner)}[PREFIX + 'curve_range_mi']
    assert owner.apply(row_change(row, -1))
    assert C.read(saved).curve_range_mi == 50 and {r.key: r for r in rows(owner)}[PREFIX + 'curve_range_mi'].value == '50 mi'
    for bad in ('75 mi', '100', 'whole map'):
      assert not owner.apply(replace(row_change({r.key: r for r in rows(owner)}[PREFIX + 'curve_range_mi']), value=bad))
    # the build status shows on the row, both counts
    ST.write(ST.BUILD_NAME, {"state": ST.UP_TO_DATE, "built": 5, "total": 5, "failed": 0, "range": 100, "all_built": 5,
                             "all_total": 33})
    owner = owner_for(saved, ti_cp(), {'vehicle': True, 'parked_preferences': True})
    got = {r.key: r.reason for r in rows(owner)}
    for k in ('curves', 'curve_range_mi'):
      assert "Now: curve maps: within 100 miles 5 of 5 built; all downloaded 5 of 33." in got[PREFIX + k], got[PREFIX + k]
