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
  b"", b"{", b"[]", b"null", doc()[:-1], json.dumps({"version": 2}).encode(), json.dumps({"version": "1"}).encode(),
  json.dumps({"version": True}).encode(), doc(enabled=1), doc(enabled="true"), doc(curves="yes"), doc(nope=1),
  doc(curve_offset_mph=2), doc(side_force_cap=3.0), doc(speed_limit_follow="on"), doc(plan_lat=3.6), doc(ti_only_lat=1.79),
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
  assert raw == b'{"curves":"test","enabled":true,"version":1}'
  assert C.decode(raw) == cfg
  assert C.encode(C.Config()) == b'{"version":1}'
  with pytest.raises(C.ConfigError):
    C.with_value(cfg, "plan_lat", 9.0)
  with pytest.raises(C.ConfigError):
    C.with_value(cfg, "not_a_field", 1)


def test_registry_knows_the_key():
  with tempfile.TemporaryDirectory() as d:
    params = Params(d)
    assert params.check_key(C.KEY)
    assert params.get_type(C.KEY).name == "JSON"
  assert C.KEY in migration.VEHICLE_PREFERENCE_KEYS and C.KEY in migration.OPERATIONAL_KEYS


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
