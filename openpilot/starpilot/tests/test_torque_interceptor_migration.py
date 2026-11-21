"""TorqueInterceptorEnabled through the manager-start settings migration, with the real Params registry.

(i) First start coming from the owner's previous Mazda fork: the old namespace has the fork's own keys, the
    TI setting on, and old CarParamsCache / CarParamsPersistent bytes. The manager start must not raise, the
    TI setting must be kept, the fork's other keys and the old caches archived.
(ii) An existing 7.0 install: the TI setting survives every later manager start.
The stale-registry case shows why libparams_c.so must know the key: an initialized namespace with an unknown
saved key and an incompatible vehicle cache is refused.
"""
import tempfile
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.common.params import Params, ParamKeyFlag
from openpilot.starpilot import state_migration as migration

KEY = "TorqueInterceptorEnabled"
LEGACY_CACHE = b"\x00\x00\x00\x00\x05\x00\x00\x00legacy capnp CarParams from the old fork"
OLD_FORK = {
  KEY: b"1",
  "CarParamsCache": LEGACY_CACHE,
  "CarParamsPersistent": LEGACY_CACHE,
  "CarParamsPrevRoute": LEGACY_CACHE,
  # keys only the old starpilot-mazda fork had
  "RadarInterceptorEnabled": b"0", "NoMRCC": b"0", "NoFSC": b"0", "BlendedACC": b"1", "ManualTransmission": b"0",
  "MazdaTiSteerMaxTest": b"0",
  # operational keys that a first start keeps
  "IsMetric": b"0", "DongleId": b"0123456789abcdef", "SshEnabled": b"1",
}


def write(params, values):
  for key, value in values.items():
    Path(params.get_param_path(key)).write_bytes(value)


def saved(params):
  return migration._read_namespace(Path(params.get_param_path()))


def manager_start(params, storage):
  # the same call and the same order as manager_init(): migration first, then the startup clears
  migration.prepare_manager_start(params, storage, auto_migrate=True)
  params.clear_all(ParamKeyFlag.CLEAR_ON_MANAGER_START)
  params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
  params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)
  params.clear_all(ParamKeyFlag.CLEAR_ON_IGNITION_ON)


@pytest.fixture
def env():
  with tempfile.TemporaryDirectory() as root:
    params = Params(str(Path(root) / "params"))
    storage = Path(root) / "recovery"
    yield params, storage


def test_registry_knows_the_key(env):
  params, _ = env
  assert params.check_key(KEY)
  assert params.get_type(KEY).name == "BOOL"
  assert KEY in migration.OPERATIONAL_KEYS


def test_first_start_from_old_fork_keeps_ti(env):
  params, storage = env
  write(params, OLD_FORK)
  manager_start(params, storage)
  values = saved(params)
  assert values[KEY] == b"1"
  assert params.get_bool(KEY)
  for key in ("CarParamsCache", "CarParamsPersistent", "CarParamsPrevRoute", "RadarInterceptorEnabled", "NoMRCC",
              "NoFSC", "BlendedACC", "ManualTransmission", "MazdaTiSteerMaxTest"):
    assert key not in values
  assert values["IsMetric"] == b"0" and values["DongleId"] == b"0123456789abcdef"
  # the old state is kept in the raw recovery snapshot
  snapshot = next((storage / "snapshots").iterdir())
  assert migration.load_snapshot(snapshot) == OLD_FORK


def test_first_start_invalid_ti_value_is_reset(env):
  params, storage = env
  write(params, OLD_FORK | {KEY: b"yes"})
  manager_start(params, storage)
  assert KEY not in saved(params)
  assert not params.get_bool(KEY)


def test_existing_install_keeps_ti_across_restarts(env):
  params, storage = env
  manager_start(params, storage)  # fresh 7.0 install
  params.put_bool(KEY, True, block=True)
  for _ in range(3):
    manager_start(params, storage)
    assert saved(params)[KEY] == b"1"
  # an old vehicle cache on an existing install is archived, the TI setting stays
  write(params, {"CarParamsCache": LEGACY_CACHE, "CarParamsPersistent": LEGACY_CACHE})
  manager_start(params, storage)
  values = saved(params)
  assert values[KEY] == b"1"
  assert "CarParamsCache" not in values and "CarParamsPersistent" not in values


def stale_registry(params):
  # the registry of a libparams_c.so built before the key existed
  return SimpleNamespace(get_param_path=params.get_param_path, get_type=params.get_type,
                         all_keys=lambda: [k for k in params.all_keys() if (k.decode() if isinstance(k, bytes) else k) != KEY])


def test_stale_registry_first_start_archives_ti(env):
  params, storage = env
  write(params, OLD_FORK)
  migration.prepare_manager_start(stale_registry(params), storage, auto_migrate=True)
  assert KEY not in saved(params)


def test_stale_registry_initialized_install_with_old_cache_is_refused(env):
  params, storage = env
  manager_start(params, storage)
  write(params, {KEY: b"1", "CarParamsCache": LEGACY_CACHE})
  with pytest.raises(migration.MigrationRequired):
    migration.prepare_manager_start(stale_registry(params), storage, auto_migrate=True)
  # with the rebuilt registry the same state starts
  manager_start(params, storage)
  assert saved(params)[KEY] == b"1"
