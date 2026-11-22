"""With the master switch off (missing, off, invalid, or on with nothing chosen) or on a car it is not for, the hook
card.py calls returns the very same frames list, creates no log or profile file and never looks again: the drive
is the Stage 1 drive. card.py only calls it for brand == "mazda", after CI.apply()."""
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace as N

import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.torque_interceptor import prepare
from opendbc.car.mazda.values import CAR
from openpilot.common.params import Params
from openpilot.starpilot.car.mazda.speed import caller, config as C

ROOT = Path(__file__).resolve().parents[6]


def ti_cp():
  return prepare(CarInterface.get_params(CAR.MAZDA_CX5, gen_empty_fingerprint(), [], False, False, False), True)


def fake_card(params, cp=None):
  cp = cp or ti_cp()
  ci = CarInterface(cp)
  return N(params=params, CP=cp, CI=ci, is_metric=False)


def cs(enabled=True, speed=25.0, v=24.0):
  return N(cruiseState=N(enabled=enabled, available=True, speed=speed, standstill=False), vEgo=v, gasPressed=False,
           brakePressed=False, standstill=False, leftBlinker=False, rightBlinker=False, steeringAngleDeg=0.0,
           steeringPressed=False)


def cc():
  return N(enabled=True, latActive=True, cruiseControl=N(cancel=False, resume=False), actuators=N(torque=0.0),
           hudControl=N(visualAlert="none"))


@pytest.fixture
def no_files(monkeypatch):
  made = []
  import openpilot.starpilot.car.mazda.speed.events as E
  import openpilot.starpilot.car.mazda.speed.car_profile as P
  monkeypatch.setattr(E.EventLog, "__init__", lambda self, *a, **k: made.append("log"))
  monkeypatch.setattr(P.CarProfile, "__init__", lambda self, *a, **k: made.append("profile"))
  return made


@pytest.mark.parametrize("raw", [None, b'{"version":1}', b'{"version":1,"enabled":false,"curves":"on"}',
                                 b'{"version":1,"enabled":true}', b'{"version":1,"enabled":true,"curves":"bad"}',
                                 b'not json', b'{"version":3}',
                                 b'{"version":3,"enabled":false,"speed_limit":"on","merge":"on","exits":"on"}',
                                 b'{"version":3,"enabled":true,"limit_offset_mph":5}',
                                 b'{"version":2,"enabled":true,"exits":"on"}',
                                 b'{"version":3,"enabled":true,"stops":"on"}',
                                 b'{"version":4,"enabled":false,"stops":"on","notices":true,"notice_sound":true}',
                                 b'{"version":4,"enabled":true,"notices":true,"notice_sound":true}',
                                 b'{"version":4,"enabled":true,"stops":"maybe"}',
                                 # (Stage 3) the gas auto-resume without the master switch, the steering test alone
                                 # (not a speed feature), a Stage 3 field in a version 4 document
                                 b'{"version":5,"enabled":false,"auto_resume":true}',
                                 b'{"version":5,"ti_steer_700":true}', b'{"version":5,"enabled":true,"ti_steer_700":true}',
                                 b'{"version":4,"enabled":true,"auto_resume":true}'])
def test_off_returns_the_same_frames_and_makes_nothing(raw, no_files):
  with tempfile.TemporaryDirectory() as d:
    params = Params(d)
    if raw is not None:
      Path(params.get_param_path(C.KEY)).write_bytes(raw)
    card = fake_card(params)
    frames = [(0x243, b"\x00" * 8, 0)]
    for _ in range(50):
      assert caller.update(card, cs(), cc(), frames, 0) is frames
    assert card.mazda_speed is False and no_files == []


def test_not_for_passive_or_long_cars(no_files):
  with tempfile.TemporaryDirectory() as d:
    params = Params(d)
    Path(params.get_param_path(C.KEY)).write_bytes(b'{"version":1,"enabled":true,"curves":"on"}')
    for change in ({"passive": True}, {"openpilotLongitudinalControl": True}, {"pcmCruise": False}):
      cp = ti_cp()
      for k, v in change.items():
        setattr(cp, k, v)
      card = fake_card(params, cp)
      frames = []
      assert caller.update(card, cs(), cc(), frames, 0) is frames and card.mazda_speed is False
    assert no_files == []


def _files_to(tmp_path, monkeypatch):
  import openpilot.starpilot.car.mazda.speed.events as E
  import openpilot.starpilot.car.mazda.speed.car_profile as P
  import openpilot.starpilot.car.mazda.speed.curve_memory as CM
  monkeypatch.setattr(E, "LOG_DIR", str(tmp_path / "log"))
  monkeypatch.setattr(P, "PROFILE_PATH", str(tmp_path / "profile.json"))
  monkeypatch.setattr(P.CarProfile.__init__, "__defaults__", (str(tmp_path / "profile.json"), "", False, P.time.monotonic))
  monkeypatch.setattr(CM.CurveMemory.__init__, "__defaults__", (str(tmp_path / "memory.json"), CM.time.time, True))


@pytest.mark.parametrize("doc", [{"speed_limit": "test"}, {"merge": "test"}, {"exits": "test"},
                                 {"speed_limit": "on"}, {"merge": "on"}, {"exits": "on"}, {"stops": "test"},
                                 {"stops": "on"},
                                 {"speed_limit": "test", "merge": "test", "exits": "test", "curves": "test"}])
def test_the_part_f_options_without_a_map_or_a_limit_never_press(doc, tmp_path, monkeypatch):
  _files_to(tmp_path, monkeypatch)
  with tempfile.TemporaryDirectory() as d:
    params = Params(d)
    Path(params.get_param_path(C.KEY)).write_bytes(json.dumps({"version": 4, "enabled": True, **doc}).encode())
    card = fake_card(params)
    for k in range(600):
      frames = []
      out = caller.update(card, cs(speed=25.0 + (k > 200) * 0.44704, v=24.0 + (k > 300) * 3.0), cc(), frames, 0)
      assert out is frames
    assert card.mazda_speed is not False
    names = {type(c).__name__ for c in card.mazda_speed.contributors}
    want = {"speed_limit": "SpeedLimit", "merge": "Merge", "exits": "ExitPlan", "curves": "CurvePlan",
            "stops": "StopSlowdown"}
    assert card.mazda_speed.notices is None          # notices off: nothing decides or writes a notice
    assert names == {want[k] for k in doc} | ({"CurvePlan"} if "merge" in doc else set())
    for c in card.mazda_speed.contributors:
      if type(c).__name__ == "CurvePlan" and "curves" not in doc:
        assert c.presses is False             # the merge's curve guard only: never pressed for


def test_on_without_a_map_never_presses(tmp_path, monkeypatch):
  import openpilot.starpilot.car.mazda.speed.events as E
  import openpilot.starpilot.car.mazda.speed.car_profile as P
  monkeypatch.setattr(E, "LOG_DIR", str(tmp_path / "log"))
  monkeypatch.setattr(P, "PROFILE_PATH", str(tmp_path / "profile.json"))
  monkeypatch.setattr(P.CarProfile.__init__, "__defaults__", (str(tmp_path / "profile.json"), "", False, P.time.monotonic))
  import openpilot.starpilot.car.mazda.speed.curve_memory as CM
  monkeypatch.setattr(CM.CurveMemory.__init__, "__defaults__", (str(tmp_path / "memory.json"), CM.time.time, True))
  with tempfile.TemporaryDirectory() as d:
    params = Params(d)
    Path(params.get_param_path(C.KEY)).write_bytes(json.dumps({"version": 1, "enabled": True, "curves": "on"}).encode())
    card = fake_card(params)
    for k in range(500):
      frames = []
      out = caller.update(card, cs(speed=25.0 + (k > 200) * 0.44704), cc(), frames, 0)
      assert out is frames
    assert card.mazda_speed is not False


def test_card_hook_is_the_only_call_and_only_for_mazda():
  src = (ROOT / "openpilot/selfdrive/car/card.py").read_text()
  assert src.count("update_mazda_speed") == 2
  hook = src[src.index("self.CI.apply(CC, now_nanos)\n      if self.CP.brand == \"mazda\""):]
  lines = hook.splitlines()[:5]
  assert lines[1].strip() == 'if self.CP.brand == "mazda":'
  assert lines[3].strip() == "can_sends = update_mazda_speed(self, CS, CC, can_sends, now_nanos)"
  assert lines[4].strip() == "self.publish_sendcan(can_sends, valid=CS.canValid)"


@pytest.mark.parametrize("raw", [None, b'{"version":4}', b'{"version":4,"enabled":false,"stops":"on","notices":true}',
                                 b'{"version":4,"enabled":true,"stops":"on"}', b'{"version":4,"enabled":true,"notices":true}',
                                 b'not json'])
def test_selfdrived_notice_hook_is_inert_when_off(raw):
  from openpilot.starpilot.car.mazda.speed.notice_alert import make
  with tempfile.TemporaryDirectory() as d:
    params = Params(d)
    if raw is not None:
      Path(params.get_param_path(C.KEY)).write_bytes(raw)
    assert make(ti_cp(), params) is None


def test_notices_on_write_only_through_the_notice_file(tmp_path, monkeypatch):
  _files_to(tmp_path, monkeypatch)
  import openpilot.starpilot.car.mazda.speed.notices as N
  monkeypatch.setattr(N, "NOTICE_PATH", str(tmp_path / "notice.json"))
  monkeypatch.setattr(N.NoticeWriter.__init__, "__defaults__", (str(tmp_path / "notice.json"), N.time.monotonic))
  with tempfile.TemporaryDirectory() as d:
    params = Params(d)
    Path(params.get_param_path(C.KEY)).write_bytes(b'{"version":4,"enabled":true,"stops":"test","notices":true}')
    card = fake_card(params)
    for _ in range(300):
      frames = []
      assert caller.update(card, cs(), cc(), frames, 0) is frames
    assert card.mazda_speed is not False and card.mazda_speed.notices is not None
    assert not (tmp_path / "notice.json").exists()     # nothing happened: no notice
