"""Offline tests for the on-road TAG button records, the writer and the pack tool."""
import builtins
import importlib.util
import json
import os
import tarfile
import time
from types import SimpleNamespace

import pytest

_here = os.path.dirname(__file__)
_spec = importlib.util.spec_from_file_location("event_tags", os.path.join(_here, "..", "event_tags.py"))
et = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(et)

_pspec = importlib.util.spec_from_file_location(
  "event_tags_pack", os.path.join(_here, "..", "..", "..", "tools", "event_tags_pack.py"))
pk = importlib.util.module_from_spec(_pspec)
_pspec.loader.exec_module(pk)

ROUTE = "00000042--abcdef1234"


class Enum:
  def __init__(self, raw):
    self.raw = raw

  def __str__(self):
    raise RuntimeError("Member was null")   # pycapnp on an out-of-range enum

  __repr__ = __str__


class FakeSM:
  def __init__(self, msgs, valid=None, alive=None, recv_time=None):
    self.msgs = msgs
    self.valid = valid if valid is not None else dict.fromkeys(msgs, True)
    self.alive = alive if alive is not None else dict.fromkeys(msgs, True)
    self.recv_time = recv_time or dict.fromkeys(msgs, 100.0)

  def __getitem__(self, k):
    return self.msgs[k]


def full_sm():
  ns = SimpleNamespace
  return FakeSM({
    "gpsLocation": ns(hasFix=True, latitude=42.1, longitude=-83.0, altitude=200.0, bearingDeg=90.0, speed=20.0,
                      horizontalAccuracy=4.5, satelliteCount=12, unixTimestampMillis=1_700_000_000_000),
    "gpsLocationExternal": ns(hasFix=False, latitude=0.0, longitude=0.0),
    "carState": ns(vEgo=20.1, aEgo=-0.5, standstill=False, steeringAngleDeg=3.2, steeringPressed=False,
                   gasPressed=False, brakePressed=True, leftBlinker=False, rightBlinker=False, gearShifter=Enum(2),
                   cruiseState=ns(available=True, enabled=True, speed=22.0), vCruise=72.0, vCruiseCluster=72.0),
    "selfdriveState": ns(enabled=True, active=True, engageable=True, state=Enum(1), experimentalMode=False,
                         personality=Enum(1), alertText1="Stop Ahead", alertText2="", alertType="x/warning",
                         alertSize=Enum(1), alertStatus=Enum(0)),
    "starpilotCarState": ns(alwaysOnLateralEnabled=True, pauseLateral=False, pauseLongitudinal=False, forceCoast=False,
                            dashboardSpeedLimit=0.0, curveSlowdownOffset=-2.0, mazdaStockLkasOff=False),
    "starpilotSelfdriveState": ns(alertText1="", alertText2="", alertSound=Enum(25)),
    "radarState": ns(leadOne=ns(status=True, dRel=35.0, vRel=-3.0, vLead=17.0, yRel=0.2, aLeadK=-1.0, modelProb=0.9)),
    "starpilotRadarState": ns(adjacentStopped=ns(status=False, dRel=0.0, yRel=0.0)),
    "longitudinalPlan": ns(aTarget=-0.8, hasLead=True, longitudinalPlanSource=Enum(1), shouldStop=False),
    "starpilotPlan": ns(vCruise=20.0, redLight=True, forcingStop=False, forcingStopLength=0.0, approachStopLength=80.0,
                        stopSignConfirmed=False, trackingLead=True, slcSpeedLimit=0.0, slcSpeedLimitSource="",
                        cscSpeed=0.0, cscControllingSpeed=False, mapCurveMode=1, mapCurveStatus=2,
                        mapCurveDistance=150.0, mapCurveRadius=80.0, mapCurveName="", mapCurveVeto=0,
                        mapCurveSource=1, mapGpsTrust=0, mapGpsAccuracy=4.5,
                        starpilotEvents=[ns(name=Enum(2)), ns(name=Enum(99))]),
    "controlsState": ns(curvature=0.001, desiredCurvature=0.0012, longControlState=Enum(1)),
    "mapdOut": ns(roadName="Main St", speedLimit=20.1),
  }, recv_time=dict.fromkeys(["gpsLocation", "gpsLocationExternal"], 99.0))


STATE = SimpleNamespace(always_on_lateral_active=False, has_longitudinal_control=False, traffic_mode_enabled=False)


def test_full_record_has_every_section_and_enums_as_ints():
  r = et.build_tag_record(full_sm(), STATE, 1_700_000_000.0, 100.5,
                          names={"state": {1: "enabled"}, "gear": {2: "drive"}, "events": {2: "forcingStop"}})
  assert r["gps"]["src"] == "gpsLocation" and r["gps"]["fix"] is True and r["gps"]["h_acc"] == 4.5
  assert r["gps"]["age_s"] == 1.5
  assert r["car"]["v_ego"] == 20.1 and r["car"]["gear"] == 2 and r["car"]["gear_name"] == "drive"
  assert r["op"]["state"] == 1 and r["op"]["state_name"] == "enabled" and r["op"]["curve_offset"] == -2.0
  assert r["alert"]["text1"] == "Stop Ahead" and r["alert"]["sp_sound"] == 25
  assert r["lead"]["d_rel"] == 35.0 and r["lead"]["adj_stopped"]["status"] is False
  assert r["plan"]["red_light"] is True and r["plan"]["events"] == [2, 99]
  assert r["plan"]["event_names"] == ["forcingStop", 99]          # unknown enum stays an int
  assert r["plan"]["map_curve"]["status"] == 2
  assert r["map"]["road"] == "Main St"
  json.dumps(r)


def test_gps_falls_back_to_any_valid_source_and_none_without_gps():
  sm = full_sm()
  sm.msgs["gpsLocation"].hasFix = False
  r = et.build_tag_record(sm, STATE, 0.0, 100.0)
  assert r["gps"]["src"] == "gpsLocation"                          # first valid when none has a fix
  sm.valid["gpsLocation"] = False
  assert et.build_tag_record(sm, STATE, 0.0, 100.0)["gps"]["src"] == "gpsLocationExternal"
  sm.alive["gpsLocationExternal"] = False
  assert et.build_tag_record(sm, STATE, 0.0, 100.0)["gps"] is None


def test_nothing_valid_still_gives_a_record():
  sm = full_sm()
  sm.valid = dict.fromkeys(sm.msgs, False)
  sm.alive = dict.fromkeys(sm.msgs, False)
  r = et.build_tag_record(sm, STATE, 5.0, 6.0)
  assert r["v"] == 1 and r["unix"] == 5.0
  assert r["gps"] is None and r["car"] is None and r["lead"] is None
  json.dumps(r)


class Hostile:
  """Every attribute read raises, like a capnp reader on a broken message."""

  def __getattr__(self, k):
    raise RuntimeError("Member was null")


class HostileSM:
  valid = alive = recv_time = property(lambda self: (_ for _ in ()).throw(RuntimeError("boom")))

  def __getitem__(self, k):
    raise RuntimeError("boom")


def test_hostile_messages_never_raise():
  sm = full_sm()
  for k in list(sm.msgs):
    sm.msgs[k] = Hostile()
  r = et.build_tag_record(sm, Hostile(), 1.0, 2.0)
  json.dumps(r)
  r = et.build_tag_record(HostileSM(), Hostile(), 1.0, 2.0)
  assert r["v"] == 1
  json.dumps(r)
  # enum whose .raw raises, list that raises while iterating
  sm = full_sm()
  sm.msgs["carState"].gearShifter = Hostile()
  sm.msgs["starpilotPlan"].starpilotEvents = Hostile()
  r = et.build_tag_record(sm, STATE, 1.0, 2.0, names=Hostile())
  assert r["car"]["gear"] is None and r["plan"]["events"] == []
  json.dumps(r)


def test_ui_thread_does_no_file_io(monkeypatch, tmp_path):
  w = et.TagWriter(root=str(tmp_path), write_in_thread=True)
  w._thread = object()                     # pretend the writer thread runs; nothing will consume the queue
  w._next = 7

  def boom(*a, **k):
    raise AssertionError("file I/O on the UI thread")
  for name in ("listdir", "stat", "scandir", "makedirs"):
    monkeypatch.setattr(os, name, boom)
  monkeypatch.setattr(builtins, "open", boom)
  monkeypatch.setattr(os.path, "exists", boom)
  r = et.build_tag_record(full_sm(), STATE, time.time(), time.monotonic())  # noqa: TID251  (wall clock: matched to log file mtimes)
  assert w.submit(r) == 7 and w.queue.qsize() == 1 and w.failed == 0
  assert w.submit(et.build_tag_record(full_sm(), STATE, time.time(), time.monotonic())) == 8  # noqa: TID251  (wall clock: matched to log file mtimes)


def test_submit_never_raises_when_the_queue_is_full(tmp_path):
  w = et.TagWriter(root=str(tmp_path), write_in_thread=True)
  w._thread = object()
  w.queue.maxsize = 1
  w.submit({})
  assert w.submit({}) is None or w.failed == 1     # dropped, counted, no exception
  assert w.failed == 1


def make_seg(root, route, n, end, lock=None):
  d = os.path.join(root, f"{route}--{n}")
  os.makedirs(d, exist_ok=True)
  for name in ("qlog.zst", "qcamera.ts", "rlog.zst"):
    p = os.path.join(d, name)
    with open(p, "wb") as f:
      f.write(b"x" * 10)
    os.utime(p, (end, end))
  if lock is not None:
    p = os.path.join(d, et.LOCK_FILE)
    open(p, "w").close()
    os.utime(p, (lock, lock))
  return d


class FakeParams:
  def get(self, key, encoding=None):
    return ROUTE.encode() if key == "CurrentRoute" else None


def test_writer_finds_the_open_segment_by_its_lock_and_numbers_tags(tmp_path):
  logs = str(tmp_path / "realdata")
  make_seg(logs, ROUTE, 3, 1000.0)
  make_seg(logs, ROUTE, 4, 1030.0, lock=1000.0)
  root = str(tmp_path / "tags")
  w = et.TagWriter(root=root, params=FakeParams(), write_in_thread=False, roots=(logs,))
  w.submit({"unix": 1025.0})
  w.submit({"unix": 1030.0})
  tags = et.read_tags(os.path.join(root, et.TAGS_FILE))
  assert [t["n"] for t in tags] == [1, 2]
  assert tags[0]["route"] == ROUTE and tags[0]["segment"] == 4 and tags[0]["segment_s"] == 25.0
  assert tags[0]["segment_from"] == "lock"
  # a new writer (UI restart, next drive) continues the numbering
  w2 = et.TagWriter(root=root, params=FakeParams(), write_in_thread=False, roots=(logs,))
  w2.submit({"unix": 1040.0})
  assert et.read_tags(os.path.join(root, et.TAGS_FILE))[-1]["n"] == 3


def test_writer_survives_an_unwritable_root(tmp_path):
  blocker = tmp_path / "file"
  blocker.write_text("x")
  w = et.TagWriter(root=str(blocker / "tags"), params=FakeParams(), write_in_thread=False, roots=())
  w.submit({"unix": 1.0})
  assert w.failed == 1 and w.written == 0


def test_writer_thread_writes(tmp_path):
  w = et.TagWriter(root=str(tmp_path), params=FakeParams(), write_in_thread=True, roots=())
  w.submit({"unix": 1.0})
  for _ in range(100):
    if w.written:
      break
    time.sleep(0.02)
  assert w.written == 1
  assert et.read_tags(os.path.join(str(tmp_path), et.TAGS_FILE))[0]["n"] == 1


def test_scan_segments_spans(tmp_path):
  logs = str(tmp_path)
  make_seg(logs, ROUTE, 0, 1060.0)
  make_seg(logs, ROUTE, 1, 1120.0)
  make_seg(logs, ROUTE, 3, 1240.0)               # 2 deleted: no chain
  make_seg(logs, ROUTE, 4, 1250.0, lock=1240.0)
  sp = et.scan_segments((logs,), now=1255.0)
  assert (sp[(ROUTE, 0)].start, sp[(ROUTE, 0)].end) == (1000.0, 1060.0)
  assert (sp[(ROUTE, 1)].start, sp[(ROUTE, 1)].end) == (1060.0, 1120.0)
  assert sp[(ROUTE, 3)].start == 1180.0
  assert sp[(ROUTE, 4)].in_progress and (sp[(ROUTE, 4)].start, sp[(ROUTE, 4)].end) == (1240.0, 1255.0)


def spans_0_to_9():
  return {(ROUTE, n): et.SegSpan(f"/x/{n}", 1000.0 + 60 * n, 1060.0 + 60 * n) for n in range(10)}


def test_select_segments_covers_10_s_before_and_20_s_after():
  sp = spans_0_to_9()
  found, miss = et.select_segments({"route": ROUTE, "unix": 1425.0, "segment": 7, "segment_s": 5.0}, sp)
  assert found == [(ROUTE, 6), (ROUTE, 7)] and miss == []
  found, _ = et.select_segments({"route": ROUTE, "unix": 1470.0, "segment": 7, "segment_s": 50.0}, sp)
  assert found == [(ROUTE, 7), (ROUTE, 8)]
  found, _ = et.select_segments({"route": ROUTE, "unix": 1450.0, "segment": 7, "segment_s": 30.0}, sp)
  assert found == [(ROUTE, 7)]
  # no route / segment recorded: by time only
  found, _ = et.select_segments({"unix": 1425.0}, sp)
  assert found == [(ROUTE, 6), (ROUTE, 7)]
  # recorded segment deleted from the device
  del sp[(ROUTE, 7)]
  found, miss = et.select_segments({"route": ROUTE, "unix": 1425.0, "segment": 7, "segment_s": 5.0}, sp)
  assert found == [(ROUTE, 6)] and miss == [(ROUTE, 7)]


def test_filter_tags():
  tags = [{"n": i, "unix": 1000.0 + i * 1000, "route": ROUTE if i % 2 else "other--r"} for i in range(1, 9)]
  n = lambda s: [t["n"] for t in s]  # noqa: E731
  assert n(et.filter_tags(tags, last=2)) == [7, 8]
  assert n(et.filter_tags(tags, since=6)) == [6, 7, 8]
  assert n(et.filter_tags(tags, numbers=[2, 5])) == [2, 5]
  assert n(et.filter_tags(tags, after_n=5)) == [6, 7, 8]
  assert n(et.filter_tags(tags, route="abcdef")) == [1, 3, 5, 7]
  assert n(et.filter_tags(tags, hours=1, now=11000.0)) == [7, 8]
  assert pk._parse_numbers("3,5,8-10") == [3, 5, 8, 9, 10]


def write_event(root, name, t):
  d = os.path.join(root, name)
  os.makedirs(d)
  with open(os.path.join(d, "summary.txt"), "w") as f:
    f.write(f"kind: x\ntime_unix: {t}\n")


def test_pack_end_to_end(tmp_path):
  logs = str(tmp_path / "realdata")
  for n in range(3):
    make_seg(logs, ROUTE, n, 1060.0 + 60 * n)
  events = str(tmp_path / "fe")
  write_event(events, "a_stopWarning", 1070.0)
  write_event(events, "b_curve", 1500.0)
  tags_root = str(tmp_path / "tags")
  os.makedirs(tags_root)
  tags = [{"n": 1, "unix": 1065.0, "route": ROUTE, "segment": 1, "segment_s": 5.0}]
  tags_path = os.path.join(tags_root, et.TAGS_FILE)
  with open(tags_path, "w") as f:
    f.write(json.dumps(tags[0]) + "\n")
  out = str(tmp_path / "out.tgz")
  _, packed, missing = pk.pack(tags, tags_path, out, roots=(logs,), events_root=events, log=lambda *a: None)
  assert packed == [(ROUTE, 0), (ROUTE, 1)] and missing == []
  with tarfile.open(out) as tar:
    names = tar.getnames()
  assert f"event_tags/logs/{ROUTE}--1/qlog.zst" in names and f"event_tags/logs/{ROUTE}--0/qcamera.ts" in names
  assert not any(n.endswith("rlog.zst") for n in names)
  assert "event_tags/feature_events/a_stopWarning/summary.txt" in names
  assert not any("b_curve" in n for n in names)
  assert "event_tags/tags/README.txt" in names and "event_tags/tags/tags.jsonl" in names
  pk.pack(tags, tags_path, out, roots=(logs,), events_root=events, rlog=True, log=lambda *a: None)
  with tarfile.open(out) as tar:
    assert f"event_tags/logs/{ROUTE}--1/rlog.zst" in tar.getnames()


def test_main_marks_packed_tags(tmp_path, monkeypatch, capsys):
  root = str(tmp_path / "tags")
  os.makedirs(root)
  with open(os.path.join(root, et.TAGS_FILE), "w") as f:
    for i in (1, 2, 3):
      f.write(json.dumps({"n": i, "unix": 1000.0 + i, "route": ROUTE}) + "\n")
  monkeypatch.setattr(pk, "pack", lambda sel, *a, **k: open(k.get("out") or a[1], "wb").close() or (None, [], []))
  out = str(tmp_path / "o.tgz")
  assert pk.main(["--root", root, "--out", out]) == 0
  assert pk._read_mark(root) == 3
  capsys.readouterr()
  pk.main(["--root", root, "--out", out])
  assert "no tags selected" in capsys.readouterr().out
  pk.main(["--root", root, "--list"])
  assert capsys.readouterr().out.count("#") == 3
  pk.main(["--root", root, "--out", out, "--last", "1", "--no-mark"])
  assert pk._read_mark(root) == 3


@pytest.mark.parametrize("bad", ["", "not json\n", "[1,2]\n"])
def test_read_tags_skips_bad_lines(tmp_path, bad):
  p = tmp_path / "t.jsonl"
  p.write_text(bad + '{"n": 1}\n')
  assert et.read_tags(str(p)) == [{"n": 1}]
