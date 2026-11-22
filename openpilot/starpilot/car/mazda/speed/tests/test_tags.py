"""The Bookmark tag (part T): a new userBookmark writes one "tag" line with the speed control's snapshot (never a
position) and, with notices on, a "Tagged" notice; the tag tool lists the tags and packs the lines around them
(dry run by default). Plus the caller's new reads (the dashboard stop sign, the camera bits)."""
import json
import os
from types import SimpleNamespace as NS

from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed import tag_tool as T
from openpilot.starpilot.car.mazda.speed.caller import SpeedControl
from openpilot.starpilot.car.mazda.speed.events import EventLog
from openpilot.starpilot.car.mazda.speed.notices import Notices
from openpilot.starpilot.car.mazda.speed.stops import StopSlowdown
from openpilot.starpilot.car.mazda.speed.tests.mrcc_sim import Mrcc
from openpilot.starpilot.car.mazda.speed.tests.test_notices import Writer

MPH = M.MPH


class Sources:
  want_view = True

  def __init__(self):
    self.bookmark_at = set()
    self.k = 0

  def context(self, d):
    self.k += 1
    view = {"t": d["_t"], "gps": "ok", "trust": 0, "cls": 1, "link": False, "matched": True, "data": True,
            "curves": [{"id": "c", "pts": [[300.0, 200.0]], "name": "Main St"}]}
    return dict(d, now=d["_t"], view=view, path=([0.0, 300.0], [0.0, 0.0]), model_age=0.0, lead=None, gps_ok=True,
                no_gps=False, bookmark=self.k in self.bookmark_at)


def run(tmp_path, cfg, frames=600, bookmarks=(100, 400)):
  log = EventLog(directory=str(tmp_path), name="events-20261122-120000.jsonl", background=False)
  src = Sources()
  src.bookmark_at = set(bookmarks)
  w = Writer()
  notices = Notices(cfg, writer=w) if cfg.shows_notices else None
  car = Mrcc(45, latency=12, v_mph=45)
  params = NS(get=lambda key: b"0000004a--1a2b3c4d5e" if key == "CurrentRoute" else None)
  ctl = SpeedControl(None, cfg, params, False, log=log, contributors=[StopSlowdown(cfg, presses=cfg.presses("stops"))],
                     sources=src, notices=notices)
  for k in range(frames):
    car.tick()
    d = {"cruise_enabled": True, "main_on": True, "set_speed": car.set, "v_ego": car.v, "gas": False, "brake": False,
         "standstill": False, "set_plus": False, "set_minus": False, "resume": False, "cancel": False,
         "wheel_counter": car.counter, "other_button": False, "engaged": True, "lkas_blocked": False,
         "blinker": False, "radar_lead": False, "_t": 1000.0 + k * 0.01}
    r = ctl.step(d, now=1000.0 + k * 0.01)
    if r is not None:
      car.receive(*r)
  log.flush()
  rows = [json.loads(x) for x in open(log.path)]
  return rows, w, log


def test_a_bookmark_writes_a_tag_line_with_the_snapshot_and_no_position(tmp_path):
  rows, w, log = run(tmp_path, C.Config(enabled=True, stops="on", notices=True))
  tags = [r for r in rows if r["kind"] == "tag"]
  assert [t["n"] for t in tags] == [1, 2]
  t = tags[0]
  for key in ("wall", "route", "drive_s", "segment_approx", "v", "set", "anchor", "owner", "reason", "states",
              "contribs", "view", "gps_ok", "presses", "lead", "radar_lead"):
    assert key in t, key
  assert t["route"] == "0000004a--1a2b3c4d5e" and t["view"]["name"] is True and t["view"]["gps"] == "ok"
  assert t["contribs"]["stop"]["presses"] is True and abs(t["anchor"] - 45 * MPH) < 1e-6
  text = json.dumps(rows)
  assert '"lat"' not in text and '"lon"' not in text and "latitude" not in text
  assert [r[0] for r in w.rows] == ["tagged", "tagged"]


def test_a_tag_without_notices_is_logged_only(tmp_path):
  rows, w, log = run(tmp_path, C.Config(enabled=True, stops="test"))
  assert sum(r["kind"] == "tag" for r in rows) == 2 and w.rows == []


def test_tag_tool_lists_and_packs_with_a_dry_run_by_default(tmp_path, capsys):
  rows, w, log = run(tmp_path, C.Config(enabled=True, stops="test"), frames=9000, bookmarks=(100, 8500))
  assert T.main(["list", "--dir", str(tmp_path)]) == 0
  out = capsys.readouterr().out
  assert "tag 1" in out and "tag 2" in out and "2 tag(s)" in out and "0000004a--1a2b3c4d5e" in out
  assert T.main(["pack", "--dir", str(tmp_path)]) == 0
  assert "dry run" in capsys.readouterr().out
  assert sorted(os.listdir(tmp_path)) == ["events-20261122-120000.jsonl"]         # nothing written
  dest = str(tmp_path / "packed.jsonl")
  assert T.main(["pack", "--dir", str(tmp_path), "--out", dest, "--write"]) == 0
  packed = [json.loads(x) for x in open(dest)]
  heads = [p for p in packed if p["kind"] == "pack_tag"]
  assert [h["n"] for h in heads] == [1, 2]
  first = packed[1:packed.index(heads[1])]
  t1 = next(r["t"] for r in rows if r["kind"] == "tag" and r["n"] == 1)
  assert first and all(abs(r["t"] - t1) <= 60.0 for r in first)
  assert any(r["kind"] == "start" for r in first) and not any(r["t"] > t1 + 60 for r in first)


def test_pack_drops_position_like_keys(tmp_path):
  path = tmp_path / "events-20261122-130000.jsonl"
  path.write_text(json.dumps({"t": 1.0, "kind": "tag", "n": 1}) + "\n" +
                  json.dumps({"t": 2.0, "kind": "x", "lat": 1.5, "nested": {"lon": 2.5, "ok": 1}}) + "\n" +
                  "not json\n")
  packed = T.pack(str(tmp_path))
  assert packed[2] == {"t": 2.0, "kind": "x", "nested": {"ok": 1}}


def test_caller_reads_the_dashboard_stop_sign_and_the_camera_bits():
  ocs = NS(crz_set_plus=False, crz_set_minus=False, crz_resume=False, crz_cancel=False, crz_btns_counter=3,
           lkas_blocked=False, radar_has_lead=True, dashboard_stop_sign=(True, 123),
           cam_lkas={"ERR_BIT_1": 0, "ERR_BIT_2": 1}, cam_laneinfo={"LANE_LINES": 0})
  card = NS(CI=NS(CS=ocs))
  cs = NS(cruiseState=NS(enabled=True, available=True, speed=20.0, standstill=False), vEgo=19.0, gasPressed=False,
          brakePressed=False, standstill=False, leftBlinker=False, rightBlinker=False, steeringAngleDeg=0.0,
          steeringPressed=False)
  cc = NS(enabled=True, latActive=True, cruiseControl=NS(cancel=False, resume=False), actuators=NS(torque=0.0),
          hudControl=NS(visualAlert="none"))
  ctl = SpeedControl(None, C.Config(enabled=True, stops="test"), None, False)
  d = ctl.inputs(card, cs, cc, [])
  assert d["stop_sign_raw"] == (True, 123) and d["cam_err"] is True and d["lane_lines"] == 0
  ocs.cam_lkas, ocs.cam_laneinfo, ocs.dashboard_stop_sign = None, None, None
  d = ctl.inputs(card, cs, cc, [])
  assert d["stop_sign_raw"] is None and d["cam_err"] is False and d["lane_lines"] is None
