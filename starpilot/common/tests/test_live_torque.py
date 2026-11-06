import importlib.util
import os

from cereal import log

_spec = importlib.util.spec_from_file_location("live_torque", os.path.join(os.path.dirname(__file__), "..", "live_torque.py"))
lt = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(lt)


def _cache(use_params, live_valid, mono=123456789):
  e = log.Event.new_message()
  e.init("liveTorqueParameters")
  e.logMonoTime = mono
  e.liveTorqueParameters.useParams = use_params
  e.liveTorqueParameters.liveValid = live_valid
  return e.to_bytes()


def test_the_cache_is_read_as_the_event_torqued_stores():
  for up in (False, True):
    for lv in (False, True):
      for mono in (0, 123456789, 2**40 + 7):
        assert lt.read_live_torque_cache(_cache(up, lv, mono)) == (up, lv), (up, lv, mono)


def test_missing_or_bad_cache():
  assert lt.read_live_torque_cache(None) == (False, False)
  assert lt.read_live_torque_cache(b"") == (False, False)
  assert lt.read_live_torque_cache(b"\x00\x01garbage") == (False, False)
  other = log.Event.new_message()
  other.init("carState")
  assert lt.read_live_torque_cache(other.to_bytes()) == (False, False)
