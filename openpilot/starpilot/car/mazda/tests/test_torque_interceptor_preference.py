"""The saved TorqueInterceptorEnabled setting reaches the Mazda CarParams at startup, and only then."""
import tempfile

import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.chrysler.interface import CarInterface as ChryslerInterface
from opendbc.car.chrysler.values import CAR as CHRYSLER
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, MazdaFlags
from openpilot.common.params import Params
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences


def params_for(model):
  return CarInterface.get_params(model, gen_empty_fingerprint(), [], False, False, False)


@pytest.mark.parametrize("raw, expected", ((None, False), (b"0", False), (b"1", True), (b"bad", False), (b"11", False)))
def test_saved_setting(raw, expected):
  with tempfile.TemporaryDirectory() as directory:
    saved = Params(directory)
    if raw is not None:
      with open(saved.get_param_path("TorqueInterceptorEnabled"), "wb") as f:
        f.write(raw)
    for enabled in (True, False):
      preferences = VehicleStartupPreferences.read(saved, enabled=enabled)
      assert preferences.mazda_torque_interceptor is expected
      for model in CAR:
        stock = params_for(model)
        cp = preferences.prepare(params_for(model))
        assert bool(cp.flags & MazdaFlags.TORQUE_INTERCEPTOR) is expected
        assert cp.safetyConfigs[0].safetyParam == (8 if expected else 0)
        if not expected:
          assert cp.to_dict() == stock.to_dict()


def test_default_is_off():
  with tempfile.TemporaryDirectory() as directory:
    saved = Params(directory)
    assert saved.get_bool("TorqueInterceptorEnabled") is False


def test_other_brands_unchanged():
  with tempfile.TemporaryDirectory() as directory:
    saved = Params(directory)
    saved.put_bool("TorqueInterceptorEnabled", True, block=True)
    preferences = VehicleStartupPreferences.read(saved, enabled=True)
    stock = ChryslerInterface.get_params(CHRYSLER.JEEP_GRAND_CHEROKEE, gen_empty_fingerprint(), [], False, False, False)
    cp = preferences.prepare(ChryslerInterface.get_params(CHRYSLER.JEEP_GRAND_CHEROKEE, gen_empty_fingerprint(), [], False, False, False))
    assert cp.to_dict() == stock.to_dict()
