"""(v7) The Tag button setting, car side: `tag_button` in MazdaSpeedControlConfig (older versions still read, unknown
fields still invalid); with the master switch on and every other feature off the speed control runs "tag only": it
reads userBookmark, writes the tag line and never presses a button."""
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace as N

import pytest

from openpilot.common.params import Params
from openpilot.starpilot.car.mazda.speed import caller, config as C
from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed.caller import SpeedControl
from openpilot.starpilot.car.mazda.speed.contributors import build
from openpilot.starpilot.car.mazda.speed.events import EventLog
from openpilot.starpilot.car.mazda.speed.tests.mrcc_sim import Mrcc
from openpilot.starpilot.car.mazda.speed.tests.test_off_is_stage1 import fake_card

MPH = M.MPH


def test_version_7_tag_button_field():
  assert C.VERSION == 8 and 7 in C.VERSIONS and C.Config().tag_button is False
  cfg = C.decode(b'{"version":7,"enabled":true,"tag_button":true}')
  assert cfg.tag_button and cfg.tag_on and not cfg.any_active and not cfg.needs_view and not cfg.shows_notices
  assert C.encode(cfg) == b'{"enabled":true,"tag_button":true,"version":8}' and C.decode(C.encode(cfg)) == cfg
  assert not C.decode(b'{"version":7,"tag_button":true}').tag_on               # needs the master switch
  for version in C.VERSIONS:                                                  # every older version still reads
    assert C.decode(json.dumps({"version": version, "enabled": True}).encode()).tag_button is False
  for raw in (b'{"version":6,"tag_button":true}', b'{"version":7,"tag_button":1}', b'{"version":7,"tag":true}',
              b'{"version":7,"tag_button":"on"}', b'{"version":9,"tag_button":true}'):
    with pytest.raises(C.ConfigError):
      C.decode(raw)
  with tempfile.TemporaryDirectory() as d:
    params = Params(d)
    Path(params.get_param_path(C.KEY)).write_bytes(b'{"version":7,"enabled":true,"tag_button":true,"extra":1}')
    assert not C.read(params).valid and not C.read(params).tag_on


def test_build_tag_only_reads_the_bookmark_and_nothing_else():
  cfg = C.decode(b'{"version":7,"enabled":true,"tag_button":true}')
  contributors, sources = build(cfg, None)
  assert contributors == [] and sources is not None
  assert sources.want_bookmark and not (sources.want_model or sources.want_view or sources.want_slc or
                                        sources.want_radar or sources.want_selfdrive)
  # the setting off (or the master switch off): no speed control at all, as before
  assert build(C.decode(b'{"version":7,"enabled":true}'), None) == ([], None)
  assert build(C.decode(b'{"version":7,"tag_button":true}'), None) == ([], None)


@pytest.fixture
def made(monkeypatch):
  out = []
  import openpilot.starpilot.car.mazda.speed.events as E
  import openpilot.starpilot.car.mazda.speed.car_profile as P
  monkeypatch.setattr(E.EventLog, "__init__", lambda self, *a, **k: out.append("log"))
  monkeypatch.setattr(E.EventLog, "log", lambda self, *a, **k: None)
  monkeypatch.setattr(P.CarProfile, "__init__", lambda self, *a, **k: out.append("profile"))
  return out


def test_make_tag_only_runs_without_a_profile_or_notices(made):
  with tempfile.TemporaryDirectory() as d:
    params = Params(d)
    Path(params.get_param_path(C.KEY)).write_bytes(b'{"version":7,"enabled":true,"tag_button":true}')
    ctl = caller.make(fake_card(params))
    assert isinstance(ctl, SpeedControl) and ctl.tag_only and ctl.contributors == [] and ctl.notices is None
    assert ctl.profile is None and made == ["log"] and ctl.sources.want_bookmark
  # with a real feature on it is the usual speed control (not tag only)
  cfg = C.decode(b'{"version":7,"enabled":true,"tag_button":true,"stops":"test"}')
  assert not SpeedControl(None, cfg, None, False).tag_only


class BookmarkSources:
  want_view = False

  def __init__(self, at):
    self.at, self.k = set(at), 0

  def context(self, d):
    self.k += 1
    return dict(d, now=d["_t"], view=None, gps_ok=True, no_gps=False, model_age=1e9, lead=None,
                bookmark=self.k in self.at)


def test_tag_only_logs_each_bookmark_and_never_presses(tmp_path):
  cfg = C.decode(b'{"version":7,"enabled":true,"tag_button":true}')
  log = EventLog(directory=str(tmp_path), name="events-20261122-130000.jsonl", background=False)
  ctl = SpeedControl(None, cfg, N(get=lambda key: None), False, log=log, sources=BookmarkSources((50, 700, 1500)))
  car = Mrcc(45, latency=12, v_mph=45)
  pressed = []
  for k in range(2000):
    car.tick()
    cruise = not 600 <= k < 650                   # his brake, then his RES at the same set speed
    if k == 300:
      car.set = 50 * MPH                          # his SET+ ...
    if k == 1000:
      car.set = 40 * MPH                          # ... his SET- (the planner's anchor follows him)
    d = {"cruise_enabled": cruise, "main_on": True, "set_speed": car.set, "v_ego": car.v, "gas": False,
         "brake": k == 600, "standstill": False, "set_plus": k == 299, "set_minus": k == 999,
         "resume": k == 649, "cancel": False, "wheel_counter": car.counter, "other_button": False, "engaged": cruise,
         "lkas_blocked": False, "blinker": False, "radar_lead": False, "_t": 2000.0 + k * 0.01}
    r = ctl.step(d, now=2000.0 + k * 0.01)
    if r is not None:
      pressed.append((k, r))
  log.flush()
  rows = [json.loads(x) for x in open(log.path)]
  assert pressed == []
  assert [r["n"] for r in rows if r["kind"] == "tag"] == [1, 2, 3]
