"""The on-screen notices (part H): the file writer, each trigger with its once-per-event rule and the global gap,
the selfdrived reader (new seq -> one alert; stale / garbled -> none; never over a stock alert; sound gating;
non-Mazda / off -> inert) and the hook in selfdrived's update_alerts."""
import json
import os
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed import notice_alert as A
from openpilot.starpilot.car.mazda.speed import notices as N
from openpilot.starpilot.car.mazda.speed.planner import Contribution, PlanOutput

MPH = M.MPH
CFG = C.Config(enabled=True, curves="on", exits="on", speed_limit="on", merge="on", stops="on", notices=True,
               notice_sound=True)


class Writer:
  def __init__(self):
    self.rows = []

  def write(self, kind, text1, text2="", sound=False):
    self.rows.append((kind, text1, text2, sound))
    return True


def notices(cfg=CFG, ti=False):
  w = Writer()
  return N.Notices(cfg, writer=w, ti=ti), w


def plan(reason=""):
  return PlanOutput(command=None, reason=reason)


def run(n, now, *, d=None, ctx=None, reason="", contributors=(), changes=(), want_view=False):
  n.update(now, d or {"v_ego": 20.0}, ctx or {"contribs": {}}, plan(reason), list(contributors), list(changes),
           want_view=want_view)


# ---------------- the writer ----------------
def test_writer_writes_one_atomic_small_json_and_never_raises(tmp_path):
  path = str(tmp_path / "notice.json")
  w = N.NoticeWriter(path=path, clock=lambda: 100.0)
  assert w.write("stop_ahead", "Stop ahead", "Cruise may not stop · brake", True)
  doc = json.loads(open(path).read())
  assert set(doc) == A.FIELDS and doc["seq"] == 1 and doc["kind"] == "stop_ahead" and doc["sound"] is True
  assert doc["until"] == 100.0 + N.SHOW_WITHIN and not os.path.exists(path + ".tmp")
  assert w.write("tagged", "Tagged") and json.loads(open(path).read())["seq"] == 2
  assert not any(k in json.dumps(doc) for k in ("lat", "lon"))
  bad = N.NoticeWriter(path=str(tmp_path / "missing" / "x.json"))
  assert bad.write("tagged", "Tagged") is False and bad.failed == 1


# ---------------- the triggers ----------------
class Curve:
  def __init__(self, vmin):
    self.last_vmin = vmin
    self.presses = True


Curve.__name__ = "CurvePlan"


def test_curve_notice_once_per_curve_with_the_curve_speed():
  n, w = notices()
  c = Curve(35 * MPH)
  ctx = {"contribs": {"map_curve": Contribution("map_curve", 30.0, episode="c1")}}
  run(n, 0.0, ctx=ctx, reason="map_curve", contributors=[c])
  run(n, 0.1, ctx=ctx, reason="map_curve", contributors=[c])
  run(n, 5.0, ctx=ctx, reason="", contributors=[c])
  run(n, 10.0, ctx=ctx, reason="map_curve", contributors=[c])              # the same curve again: no
  assert [r[1] for r in w.rows] == ["Curve ahead — slowing to 37"]          # + the dash offset (2 mph)
  ctx2 = {"contribs": {"map_curve": Contribution("map_curve", 30.0, episode="c2")}}
  run(n, 11.0, ctx=ctx2, reason="", contributors=[c])
  run(n, 20.0, ctx=ctx2, reason="map_curve", contributors=[c])
  assert len(w.rows) == 2 and w.rows[-1][3] is False                        # no sound for a curve


def test_global_gap_and_a_pending_slot_where_a_warning_wins():
  n, w = notices()
  n.fire("merge", 0.0, key="m")
  n.fire("no_gps", 1.0, key="g")              # inside the gap: pending
  n.fire("stop_ahead", 1.5, key="s")          # a warning: may go after WARN_GAP
  assert [r[0] for r in w.rows] == ["merge", "stop_ahead"]
  n2, w2 = notices()
  n2.fire("merge", 0.0, key="m")
  n2.fire("no_gps", 1.0, key="g")
  run(n2, 2.0)
  assert [r[0] for r in w2.rows] == ["merge"]
  run(n2, 3.05)                               # pending for 2.05 s: dropped (PENDING_MAX)
  assert [r[0] for r in w2.rows] == ["merge"]
  n3, w3 = notices()
  n3.fire("merge", 0.0, key="m")
  n3.fire("no_gps", 1.5, key="g")
  run(n3, 3.1)
  assert [r[0] for r in w3.rows] == ["merge", "no_gps"]
  assert n3.fire("no_gps", 50.0, key="g") is False                          # once per its event


class Stops:
  def __init__(self, notice, presses):
    self.notice = notice
    self.presses = presses
    self.active = {"id": "stop1"}


Stops.__name__ = "StopSlowdown"


@pytest.mark.parametrize("presses", [True, False])
def test_stop_warnings_fire_once_per_episode_with_sound_also_in_test_mode(presses):
  n, w = notices()
  s = Stops("stopped_car", presses)
  run(n, 0.0, contributors=[s])
  run(n, 2.0, contributors=[s])
  s.notice = "stop_ahead"
  run(n, 2.5, contributors=[s])
  assert w.rows == [("stopped_car", "Stopped car ahead", "Cruise may not stop · brake", True),
                    ("stop_ahead", "Stop ahead", "Cruise may not stop · brake", True)]
  n2, w2 = notices(C.Config(enabled=True, stops="test", notices=True))
  run(n2, 0.0, contributors=[Stops("stop_ahead", False)])
  assert w2.rows[0][3] is False                                             # notice_sound off: no sound


class Limit:
  steady = 45


Limit.__name__ = "SpeedLimit"


class MergeC:
  link = 7


MergeC.__name__ = "Merge"


def test_limit_and_merge_notices_come_from_our_anchor_changes():
  n, w = notices()
  run(n, 0.0, contributors=[Limit()], changes=[("limit", 50 * MPH, 48 * MPH)])
  run(n, 0.5, contributors=[Limit()], changes=[("limit", 50 * MPH, 48 * MPH)])
  run(n, 4.0, contributors=[MergeC()], changes=[("merge", 40 * MPH, 75 * MPH)])
  run(n, 8.0, contributors=[MergeC()], changes=[("merge", 75 * MPH, 70 * MPH)])     # the taper: no notice
  run(n, 12.0, changes=[("driver SET_PLUS", 40 * MPH, 50 * MPH)])                   # his own: no notice
  assert [r[1] for r in w.rows] == ["Speed limit 45 — set to 48", "Merging — set speed high"]
  assert all(r[3] is False for r in w.rows)


class ExitC:
  presses = True
  brake_needed = False


ExitC.__name__ = "ExitPlan"


def test_exit_notices_slowing_and_brake_once_per_exit():
  n, w = notices()
  ex = ExitC()
  ctx = {"contribs": {"exit": Contribution("exit", 25.0, episode="x1")}}
  run(n, 0.0, ctx=ctx, reason="exit", contributors=[ex])
  ex.brake_needed = True
  run(n, 2.0, ctx=ctx, reason="exit", contributors=[ex])
  run(n, 5.0, ctx=ctx, reason="exit", contributors=[ex])
  assert [(r[0], r[3]) for r in w.rows] == [("exit", False), ("exit_brake", True)]


def test_no_gps_and_no_curve_data_once_per_drive_while_moving():
  n, w = notices()
  t = 0.0
  for _ in range(400):
    t += 0.05
    run(n, t, d={"v_ego": 20.0}, ctx={"contribs": {}, "no_gps": True, "gps_ok": False, "view": None}, want_view=True)
  assert [r[0] for r in w.rows] == ["no_gps"]
  for _ in range(400):
    t += 0.05
    run(n, t, d={"v_ego": 20.0}, ctx={"contribs": {}, "no_gps": False, "gps_ok": True, "view": {"data": False}},
        want_view=True)
  assert [r[0] for r in w.rows] == ["no_gps", "no_data"]
  n2, w2 = notices()
  for k in range(400):
    run(n2, k * 0.05, d={"v_ego": 1.0}, ctx={"contribs": {}, "no_gps": True}, want_view=True)   # parked: nothing
    run(n2, 30 + k * 0.05, d={"v_ego": 20.0}, ctx={"contribs": {}, "no_gps": True}, want_view=False)  # no view needed
  assert w2.rows == []


def test_camera_fault_lane_lines_and_cleared_on_a_ti_car_only():
  n, w = notices(ti=True)
  t = 0.0

  def frames(sec, **d):
    nonlocal t
    for _ in range(int(sec / 0.05)):
      t += 0.05
      run(n, t, d=dict({"v_ego": 20.0, "cam_err": False, "lane_lines": 2}, **d))
  frames(1.0, cam_err=True)
  frames(3.0)
  frames(6.0, lane_lines=0)                     # 5 s above 35 mph: the latch
  frames(6.0, lane_lines=2)                     # lines back 5 s, then 2 s clear
  frames(3.0)
  assert [r[0] for r in w.rows] == ["cam_fault", "cam_cleared", "lines_off", "cam_cleared"]
  assert w.rows[0][1:] == ("Car's camera reports a fault", "Lane keep may be off · steering weaker", True)
  n2, w2 = notices(ti=False)
  for k in range(100):
    run(n2, k * 0.05, d={"v_ego": 20.0, "cam_err": True, "lane_lines": 0})
  assert w2.rows == []


def test_tagged():
  n, w = notices()
  n.tagged(1.0)
  assert w.rows == [("tagged", "Tagged", "", False)]


# ---------------- the selfdrived reader ----------------
def put(path, **kw):
  raw = kw.pop("_raw", None)
  doc = {"v": 1, "sid": "abc", "seq": 1, "kind": "stop_ahead", "text1": "Stop ahead", "text2": "Cruise may not stop",
         "sound": True, "until": 100.0, **kw}
  with open(path, "wb") as f:
    f.write(raw if raw is not None else json.dumps(doc).encode())
  st = os.stat(path)
  os.utime(path, ns=(st.st_atime_ns, st.st_mtime_ns + 1_000_000))   # a new mtime each write


EMPTY = NS(alert_type="")


def test_reader_new_seq_makes_one_alert_and_the_same_seq_none(tmp_path):
  path = str(tmp_path / "n.json")
  r = A.NoticeReader(path=path, sound=True)
  assert r.poll(EMPTY, now=1.0) == []                                       # no file
  put(path)
  got = r.poll(EMPTY, now=1.2)
  assert len(got) == 1 and got[0].alert_text_1 == "Stop ahead" and got[0].alert_type == A.ALERT_TYPE
  from openpilot.selfdrive.selfdrived.events import AudibleAlert, Priority
  assert got[0].priority == Priority.LOWER and got[0].audible_alert == AudibleAlert.prompt
  assert r.poll(EMPTY, now=1.4) == []
  put(path)                                                                 # rewritten, the same seq
  assert r.poll(EMPTY, now=1.6) == []
  put(path, seq=2, kind="tagged", text1="Tagged", text2="", sound=False)
  got = r.poll(EMPTY, now=1.8)
  assert got[0].alert_text_1 == "Tagged" and got[0].audible_alert == AudibleAlert.none
  put(path, sid="new", seq=1, kind="merge", text1="Merging — set speed high", text2="", sound=False)  # next drive
  assert len(r.poll(EMPTY, now=2.0)) == 1


@pytest.mark.parametrize("bad", [
  {"_raw": b"{not json"}, {"_raw": b"x" * 2000}, {"kind": "unknown"}, {"seq": 0}, {"seq": "2"}, {"sound": 1},
  {"text1": ""}, {"text1": "x" * 49}, {"v": 2}, {"until": 0.5}, {"extra": 1}, {"_raw": b"[]"},
  {"_raw": b'{"v":1,"sid":"abc","seq":1,"kind":"tagged","text1":"T","text2":"","sound":false,"until":NaN}'},
])
def test_reader_stale_or_garbled_file_makes_no_alert(tmp_path, bad):
  path = str(tmp_path / "n.json")
  r = A.NoticeReader(path=path, sound=True)
  put(path, **bad)
  assert r.poll(EMPTY, now=1.0) == []


def test_reader_polls_at_10_hz_parses_only_on_change_and_waits_for_a_free_screen(tmp_path):
  path = str(tmp_path / "n.json")
  r = A.NoticeReader(path=path, sound=False)
  put(path)
  with patch.object(A.os, "stat", wraps=os.stat) as st, patch("builtins.open", wraps=open) as op:
    for k in range(100):
      r.poll(NS(alert_type="startup/permanent"), now=1.0 + k * 0.01)          # a stock alert on screen
    assert st.call_count == 10 and op.call_count == 1
  got = r.poll(EMPTY, now=2.5)
  assert len(got) == 1
  from openpilot.selfdrive.selfdrived.events import AudibleAlert
  assert got[0].audible_alert == AudibleAlert.none                          # notice_sound off: no sound
  put(path, seq=5, until=3.0)
  r.poll(NS(alert_type="startup/permanent"), now=2.7)
  assert r.poll(EMPTY, now=3.1) == []                                       # past its "until": dropped


def test_make_is_inert_unless_a_mazda_with_the_notices_on(tmp_path):
  from openpilot.common.params import Params
  params = Params(str(tmp_path / "p"))
  mazda, other = NS(brand="mazda"), NS(brand="toyota")
  assert A.make(mazda, params) is None                                      # nothing saved
  for doc in ({"version": 4, "enabled": False, "stops": "on", "notices": True},
              {"version": 4, "enabled": True, "notices": True},
              {"version": 4, "enabled": True, "stops": "on"},
              {"version": 3, "enabled": True, "curves": "on", "notices": True}):
    with open(params.get_param_path(C.KEY), "wb") as f:
      f.write(json.dumps(doc).encode())
    assert A.make(mazda, params) is None, doc
  with open(params.get_param_path(C.KEY), "wb") as f:
    f.write(json.dumps({"version": 4, "enabled": True, "stops": "test", "notices": True}).encode())
  assert isinstance(A.make(mazda, params), A.NoticeReader) and A.make(other, params) is None
  assert A.make(None, params) is None and A.make(mazda, None) is None


# ---------------- the hook in selfdrived ----------------
def selfdrived_fixture():
  from openpilot.cereal import log
  from opendbc.car import structs
  from openpilot.common.params import Params
  from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
  from openpilot.selfdrive.selfdrived.events import Events, ET
  from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
  from openpilot.selfdrive.selfdrived.tests.test_force_stop_alert import NOW, Sources
  from openpilot.starpilot.controllers.mode_actions import SwitchbackCooldown
  from openpilot.starpilot.longitudinal.force_stop_alert import HoldAlertState
  drive = SelfdriveD.__new__(SelfdriveD)
  drive.CP = structs.CarParams(brand="mazda")
  drive.enabled, drive.personality, drive.is_metric = True, log.LongitudinalPersonality.standard, False
  vars(drive)['state_machine'] = NS(current_alert_types=[ET.PERMANENT], soft_disable_timer=0)
  drive.events, drive.AM = Events(), AlertManager()
  sm = Sources()
  sm.data['longitudinalPlan'].shouldStop = False
  sm.data['starpilotLongitudinalPlan'].forceStopHolding = False
  vars(drive)['sm'] = sm
  drive.params = Mock(spec=Params, get=lambda _key: None)
  drive.switchback_capable = False
  drive.switchback_setting_ns = NOW
  drive.switchback_cooldown_ns = 300_000_000_000
  drive.switchback_cooldown = SwitchbackCooldown()
  drive.force_stop_hold_alert = HoldAlertState()
  drive.aol_car_state_log_ns, drive.conditional_car_state_valid = NOW - 10_000_000, True
  return drive, structs.CarState(canValid=True), NOW


def test_selfdrived_hook_turns_a_new_notice_into_the_current_alert_and_is_inert_without_it(tmp_path):
  drive, cs, now = selfdrived_fixture()
  with patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns', return_value=now):
    drive.update_alerts(cs)                                                 # no attribute at all: nothing
    assert drive.AM.current_alert.alert_type == ""
    drive.mazda_speed_notice = None
    drive.update_alerts(cs)
    assert drive.AM.current_alert.alert_type == ""
    path = str(tmp_path / "n.json")
    drive.mazda_speed_notice = A.NoticeReader(path=path, sound=True, clock=lambda: 10.0)
    put(path, until=20.0)
    drive.sm.frame += 1
    drive.update_alerts(cs)
    assert drive.AM.current_alert.alert_text_1 == "Stop ahead"
    for _ in range(50):                                                     # it stays for its duration, once
      drive.sm.frame += 1
      drive.update_alerts(cs)
    assert drive.AM.current_alert.alert_text_1 == "Stop ahead" and drive.mazda_speed_notice.made == 1


def test_selfdrived_hook_is_small_and_marked():
  from pathlib import Path
  src = (Path(__file__).resolve().parents[5] / "selfdrive/selfdrived/selfdrived.py").read_text()
  assert src.count("mazda_speed_notice") == 5         # import, init, the check, the poll
  assert "from openpilot.starpilot.car.mazda.speed.notice_alert import make as make_mazda_speed_notice" in src


def test_curve_notice_through_the_chain_once_with_the_curves_set_speed():
  import math
  from openpilot.starpilot.car.mazda.speed.caller import SpeedControl
  from openpilot.starpilot.car.mazda.speed.contributors import Sources
  from openpilot.starpilot.car.mazda.speed.curve_plan import CurvePlan
  from openpilot.starpilot.car.mazda.speed.tests.test_speed_control import Road
  cfg = C.Config(enabled=True, curves="on", notices=True)
  road = Road()
  w = Writer()
  plan_c = CurvePlan(cfg, presses=True)
  src = Sources(want_model=False, sm=None, read_view=road.view, clock=lambda: road.t)
  ctl = SpeedControl(None, cfg, None, False, contributors=[plan_c], sources=src, notices=N.Notices(cfg, writer=w))
  for _ in range(9000):
    road.tick()
    d = {"cruise_enabled": True, "main_on": True, "set_speed": road.car.set, "v_ego": road.v, "gas": False,
         "brake": False, "standstill": False, "set_plus": False, "set_minus": False, "resume": False, "cancel": False,
         "wheel_counter": road.car.counter, "other_button": False, "engaged": True, "lkas_blocked": False,
         "blinker": False}
    r = ctl.step(d, now=road.t)
    if r is not None:
      road.car.receive(*r)
  curve = [row for row in w.rows if row[0] == "curve"]
  want = round((math.sqrt(2.2 * 72) + M.DASH_OFFSET) / MPH)
  assert curve == [("curve", f"Curve ahead — slowing to {want}", "", False)]
