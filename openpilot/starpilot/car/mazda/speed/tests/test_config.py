"""MazdaSpeedControlConfig: strict decoding (anything wrong = everything off), defaults off, the registry, the
first-start migration keep-list."""
import json
import tempfile
from pathlib import Path

import pytest

from openpilot.common.params import Params, ParamKeyFlag
from openpilot.starpilot import state_migration as migration
from openpilot.starpilot.car.mazda.speed import config as C


def doc(**kw):
  return json.dumps({"version": 1, **kw}).encode()


def test_defaults_are_off():
  cfg = C.decode(None)
  assert not cfg.enabled and cfg.valid
  for name, (kind, default, _) in C.SCHEMA.items():
    assert getattr(cfg, name) == default
    if kind == "mode":
      assert default == "off" and cfg.mode(name) == "off"
  assert not cfg.any_active


def test_sub_features_need_the_master_switch():
  cfg = C.decode(doc(curves="on", camera_curves="test"))
  assert cfg.curves == "on" and cfg.mode("curves") == "off" and not cfg.presses("curves") and not cfg.any_active
  cfg = C.decode(doc(enabled=True, curves="on", camera_curves="test"))
  assert cfg.presses("curves") and cfg.active("camera_curves") and not cfg.presses("camera_curves") and cfg.any_active
  assert not C.decode(doc(enabled=True)).any_active   # master on, nothing chosen: nothing runs


@pytest.mark.parametrize("raw", [
  b"", b"{", b"[]", b"null", doc()[:-1], json.dumps({"version": 9}).encode(), json.dumps({"version": 0}).encode(),
  json.dumps({"version": 5, "curve_range_mi": 200}).encode(), json.dumps({"version": 1, "curve_range_mi": 0}).encode(),
  json.dumps({"version": 6, "curve_range_mi": 150}).encode(), json.dumps({"version": 6, "curve_range_mi": -1}).encode(),
  json.dumps({"version": 6, "curve_range_mi": "100"}).encode(), json.dumps({"version": 6, "curve_range_mi": 100.0}).encode(),
  json.dumps({"version": 6, "curve_range_mi": True}).encode(), json.dumps({"version": 6, "curve_range_mi": None}).encode(),
  json.dumps({"version": 6, "curve_map_range": 100}).encode(),
  json.dumps({"version": 4, "auto_resume": True}).encode(), json.dumps({"version": 4, "ti_steer_700": True}).encode(),
  json.dumps({"version": 5, "auto_resume": 1}).encode(), json.dumps({"version": 5, "ti_steer_700": "on"}).encode(),
  json.dumps({"version": 5, "auto_resume_after_brake": True}).encode(), json.dumps({"version": 1, "curve_offset_mph": 2}).encode(),
  json.dumps({"version": 2, "speed_limit": "on"}).encode(), json.dumps({"version": 1, "exits": "test"}).encode(),
  json.dumps({"version": 3, "merge": "yes"}).encode(), json.dumps({"version": 3, "limit_offset_mph": 11}).encode(),
  json.dumps({"version": 3, "limit_offset_mph": 2.5}).encode(), json.dumps({"version": 3, "stops": "on"}).encode(),
  json.dumps({"version": 3, "notices": True}).encode(), json.dumps({"version": 2, "notice_sound": True}).encode(),
  json.dumps({"version": 4, "stops": "yes"}).encode(), json.dumps({"version": 4, "notices": "on"}).encode(),
  json.dumps({"version": 4, "notice_sound": 1}).encode(), json.dumps({"version": 4, "stop_warnings": "on"}).encode(),
  json.dumps({"version": 7, "steer_smoothing": False}).encode(), json.dumps({"version": 8, "steer_smoothing": 0}).encode(),
  json.dumps({"version": 8, "steer_smoothing": "off"}).encode(), json.dumps({"version": 8, "steer_smoothing": None}).encode(),
  json.dumps({"version": 8, "model_smoothing": False}).encode(),
  json.dumps({"version": "1"}).encode(),
  json.dumps({"version": True}).encode(), doc(enabled=1), doc(enabled="true"), doc(curves="yes"), doc(nope=1),
  doc(curve_offset_mph=6), doc(curve_offset_lat=4.5), doc(side_force_cap=3.0), doc(speed_limit_follow="on"), doc(plan_lat=3.6), doc(ti_only_lat=1.79),
  doc(plan_lat=1.7), doc(ramp_lat=3.6), doc(ti_only_lat="2"), b'{"version":1,"plan_lat":NaN}',
  b'{"version":1,"enabled":true,"enabled":false}', b"\xff", b"x" * 5000,
])
def test_invalid_documents_raise(raw):
  with pytest.raises(C.ConfigError):
    C.decode(raw)


def test_read_never_raises_and_invalid_is_off():
  with tempfile.TemporaryDirectory() as d:
    params = Params(d)
    assert C.read(params) == C.Config()
    Path(params.get_param_path(C.KEY)).write_bytes(b'{"version":1,"enabled":true,"curves":"maybe"}')
    cfg = C.read(params)
    assert not cfg.valid and not cfg.enabled and not cfg.any_active
    Path(params.get_param_path(C.KEY)).write_bytes(doc(enabled=True, curves="on", plan_lat=2.4))
    cfg = C.read(params)
    assert cfg.valid and cfg.presses("curves") and cfg.plan_lat == 2.4


def test_encode_roundtrip_is_canonical():
  cfg = C.with_value(C.Config(), "enabled", True)
  cfg = C.with_value(cfg, "curves", "test")
  raw = C.encode(cfg)
  assert raw == b'{"curves":"test","enabled":true,"version":8}'
  assert C.decode(b'{"curves":"test","enabled":true,"version":1}') == cfg     # a version 1 document still reads
  assert C.decode(b'{"curves":"test","enabled":true,"version":2}') == cfg     # ... and a version 2 one
  assert C.decode(b'{"curves":"test","enabled":true,"version":3}') == cfg     # ... and a version 3 one
  assert C.decode(b'{"curves":"test","enabled":true,"version":4}') == cfg     # ... and a version 4 one
  assert C.decode(b'{"curves":"test","enabled":true,"version":5}') == cfg     # ... and a version 5 one
  assert C.decode(raw) == cfg
  assert C.encode(C.Config()) == b'{"version":8}'
  with pytest.raises(C.ConfigError):
    C.with_value(cfg, "plan_lat", 9.0)
  with pytest.raises(C.ConfigError):
    C.with_value(cfg, "not_a_field", 1)


def test_registry_knows_the_key():
  with tempfile.TemporaryDirectory() as d:
    params = Params(d)
    assert params.check_key(C.KEY)
    assert params.get_type(C.KEY).name == "JSON"
  assert C.KEY in migration.OPERATIONAL_KEYS and C.KEY not in migration.VEHICLE_PREFERENCE_KEYS  # (those are booleans)


def manager_start(params, storage):
  migration.prepare_manager_start(params, storage, auto_migrate=True)
  for flag in (ParamKeyFlag.CLEAR_ON_MANAGER_START, ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION,
               ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION, ParamKeyFlag.CLEAR_ON_IGNITION_ON):
    params.clear_all(flag)


def test_saved_config_survives_manager_starts_and_old_fork_keys_do_not():
  with tempfile.TemporaryDirectory() as root:
    params = Params(str(Path(root) / "params"))
    storage = Path(root) / "recovery"
    raw = doc(enabled=True, curves="test")
    # first start from the old fork: its own Mazda speed keys are archived, ours (if present) is kept
    old = {C.KEY: raw, "MazdaCurveSlowdown": b"1", "MazdaMapCurves": b"2", "IsMetric": b"0"}
    for k, v in old.items():
      Path(params.get_param_path(k)).write_bytes(v)
    manager_start(params, storage)
    values = migration._read_namespace(Path(params.get_param_path()))
    assert values[C.KEY] == raw
    assert "MazdaCurveSlowdown" not in values and "MazdaMapCurves" not in values
    for _ in range(2):
      manager_start(params, storage)
      assert C.read(params).active("curves")


def test_version_3_options_default_off_and_need_the_master_switch():
  cfg = C.Config()
  assert (cfg.speed_limit, cfg.limit_offset_mph, cfg.merge, cfg.exits) == ("off", 3, "off", "off")
  assert not cfg.needs_view and not cfg.any_active
  on = C.decode(json.dumps({"version": 3, "speed_limit": "on", "merge": "test", "exits": "on",
                            "limit_offset_mph": 5}).encode())
  assert on.mode("speed_limit") == "off" and not on.any_active and not on.needs_view     # master off
  on = C.decode(json.dumps({"version": 3, "enabled": True, "speed_limit": "on", "merge": "test", "exits": "on",
                            "limit_offset_mph": 0}).encode())
  assert on.presses("speed_limit") and on.active("merge") and not on.presses("merge") and on.presses("exits")
  assert on.limit_offset_mph == 0 and on.needs_view and on.any_active and on.mode("curves") == "off"
  assert C.decode(C.encode(on)) == on



def test_version_4_stops_and_notices_default_off_and_need_the_master_switch():
  cfg = C.Config()
  assert (cfg.stops, cfg.notices, cfg.notice_sound) == ("off", False, False) and not cfg.shows_notices
  off = C.decode(json.dumps({"version": 4, "stops": "on", "notices": True, "notice_sound": True}).encode())
  assert off.mode("stops") == "off" and not off.any_active and not off.shows_notices      # master off
  on = C.decode(json.dumps({"version": 4, "enabled": True, "stops": "test", "notices": True}).encode())
  assert on.active("stops") and not on.presses("stops") and on.any_active and on.shows_notices and not on.needs_view
  assert not on.notice_sound and C.decode(C.encode(on)) == on
  # notices alone (nothing that computes) show nothing
  assert not C.decode(json.dumps({"version": 4, "enabled": True, "notices": True}).encode()).shows_notices
  # an older document still reads, with the new options off
  old = C.decode(json.dumps({"version": 3, "enabled": True, "merge": "on"}).encode())
  assert old.presses("merge") and old.stops == "off" and not old.notices


def test_version_5_stage3_options_default_off():
  cfg = C.Config()
  assert cfg.auto_resume is False and cfg.ti_steer_700 is False
  assert not cfg.auto_resume_on and not cfg.steer_700_on and not cfg.any_active and not cfg.needs_view
  # the gas auto-resume needs the master switch; it then runs the speed control and the map view (its map guard)
  cfg = C.decode(json.dumps({"version": 5, "auto_resume": True}).encode())
  assert not cfg.auto_resume_on and not cfg.any_active and not cfg.needs_view
  cfg = C.decode(json.dumps({"version": 5, "enabled": True, "auto_resume": True}).encode())
  assert cfg.auto_resume_on and cfg.any_active and cfg.needs_view and not cfg.shows_notices
  # the steering limit test is its own switch (not under the master switch) and starts nothing of the speed control
  cfg = C.decode(json.dumps({"version": 5, "ti_steer_700": True}).encode())
  assert cfg.steer_700_on and not cfg.any_active and not cfg.needs_view
  assert C.encode(cfg) == b'{"ti_steer_700":true,"version":8}'


def test_version_6_curve_map_build_range():
  # default 100 miles: what the builder always did (a document without the field, of any version)
  for version in C.VERSIONS:
    cfg = C.decode(json.dumps({"version": version}).encode())
    assert cfg.curve_range_mi == 100 and cfg.curve_range_miles == 100.0
  assert C.CURVE_RANGES == (30, 50, 100, 200, 300, 0)
  for miles in C.CURVE_RANGES:
    cfg = C.decode(json.dumps({"version": 6, "enabled": True, "curves": "on", "curve_range_mi": miles}).encode())
    assert cfg.curve_range_mi == miles and C.decode(C.encode(cfg)) == cfg
    assert cfg.curve_range_miles == (None if miles == C.WHOLE_MAP else float(miles))
    assert cfg.presses("curves") and cfg.needs_view
  assert C.encode(C.with_value(C.Config(), "curve_range_mi", 0)) == b'{"curve_range_mi":0,"version":8}'
  assert C.encode(C.with_value(C.Config(), "curve_range_mi", 100)) == b'{"version":8}'    # the default is not written
  # the range alone starts nothing
  assert not C.decode(b'{"version":6,"enabled":true,"curve_range_mi":300}').any_active
  with pytest.raises(C.ConfigError):
    C.with_value(C.Config(), "curve_range_mi", 75)
  with tempfile.TemporaryDirectory() as d:
    params = Params(d)
    Path(params.get_param_path(C.KEY)).write_bytes(b'{"version":6,"curve_range_mi":0,"extra":1}')
    assert not C.read(params).valid and C.read(params).curve_range_mi == 100       # unknown field: invalid, default


def test_version_8_steering_smoothing_is_on_by_default_and_its_own_switch():
  for version in C.VERSIONS:                                   # every document without the field: on
    assert C.decode(json.dumps({"version": version}).encode()).steer_smoothing_on
  assert C.Config().steer_smoothing_on and C.Config.off(valid=False).steer_smoothing_on   # missing / invalid: on
  cfg = C.decode(b'{"version":8,"steer_smoothing":false}')
  assert not cfg.steer_smoothing_on and not cfg.any_active and not cfg.needs_view and not cfg.enabled
  assert C.encode(cfg) == b'{"steer_smoothing":false,"version":8}' and C.decode(C.encode(cfg)) == cfg
  assert C.encode(C.with_value(cfg, "steer_smoothing", True)) == b'{"version":8}'          # the default is not written
  cfg = C.decode(b'{"version":8,"enabled":true,"steer_smoothing":false,"curves":"on"}')     # independent of the master
  assert not cfg.steer_smoothing_on and cfg.presses("curves")
  assert C.decode(b'{"version":8,"enabled":true}').steer_smoothing_on
  with tempfile.TemporaryDirectory() as d:
    params = Params(d)
    Path(params.get_param_path(C.KEY)).write_bytes(b'{"version":8,"steer_smoothing":false,"extra":1}')
    assert not C.read(params).valid and C.read(params).steer_smoothing_on    # invalid: speed control off, smoothing on
