"""The parked curve builder daemon: which folders it builds (older builder/format, changed tiles, broken files),
MemoryError back-off, the selected 7.0 generation as its source, and a real build from a synthetic tile."""
import hashlib
import json
import math
import os

import pytest

from openpilot.starpilot.car.mazda.speed import curve_builder as B
from openpilot.starpilot.car.mazda.speed import curve_mapd as D

FOLDER = "42/-84"
TILES = {"42.250000_-83.750000_42.500000_-83.500000": "ab" * 32}


def built_state(root, **kw):
  d = os.path.join(root, FOLDER)
  os.makedirs(d, exist_ok=True)
  name = "42.250000_-83.750000_42.500000_-83.500000"
  B.write_json_atomic(os.path.join(d, name + ".json"), {"version": B.FORMAT_VERSION, "builder": B.BUILDER_VERSION,
                                                        "curves": [], "ramps": [], "chains": [], "roads": []})
  state = {"format": B.FORMAT_VERSION, "builder": B.BUILDER_VERSION, "signature": D.signature(TILES), "files": [name]}
  state.update(kw)
  B.write_json_atomic(os.path.join(d, D.STATE_NAME), state)
  return d, name


def test_up_to_date_folder_is_left_alone(tmp_path):
  built_state(str(tmp_path))
  assert D.check_folder(FOLDER, TILES, str(tmp_path)) is None


def test_older_builder_or_format_is_rebuilt(tmp_path):
  built_state(str(tmp_path), builder=1)
  assert D.check_folder(FOLDER, TILES, str(tmp_path)).startswith("builder")
  built_state(str(tmp_path), builder=B.BUILDER_VERSION, format=4)
  assert D.check_folder(FOLDER, TILES, str(tmp_path)).startswith("format")
  # the old fork's own folders (format 4, builder 1) are always rebuilt
  built_state(str(tmp_path), builder=1, format=4)
  assert D.check_folder(FOLDER, TILES, str(tmp_path)) is not None


def test_changed_tiles_missing_state_and_broken_files_are_rebuilt(tmp_path):
  root = str(tmp_path)
  assert D.check_folder(FOLDER, TILES, root) == "no state"
  d, name = built_state(root)
  assert D.check_folder(FOLDER, dict.fromkeys(TILES, "cd" * 32), root) == "map tiles changed"
  open(os.path.join(d, name + ".json"), "w").close()            # 0 bytes after a power cut
  assert "missing or broken" in D.check_folder(FOLDER, TILES, root)
  with open(os.path.join(d, name + ".json"), "w") as f:
    f.write('{"version": 5, "curves": [')                         # torn
  assert "missing or broken" in D.check_folder(FOLDER, TILES, root)
  built_state(root)
  open(os.path.join(d, D.STATE_NAME), "w").close()
  assert D.check_folder(FOLDER, TILES, root) == "state unreadable"


def test_plan_is_nearest_first_within_range_and_skips_failed(tmp_path):
  folders = {f: {"dir": "/x", "tiles": TILES} for f in ("42/-84", "40/-84", "30/-90")}
  near = (42.5, -83.0)
  got = D.plan(folders, str(tmp_path), near=near)
  assert [f for f, _ in got] == ["42/-84", "40/-84"]               # 30/-90 is far out of range
  assert [f for f, _ in D.plan(folders, str(tmp_path), near=None)] == sorted(folders)
  assert [f for f, _ in D.plan(folders, str(tmp_path), near=near, failed={"42/-84"})] == ["40/-84"]


def test_memory_error_retries_once_then_skips_for_the_session(tmp_path):
  calls = []

  def boom(*a, **k):
    calls.append(1)
    raise MemoryError
  folders = {FOLDER: {"dir": "/x", "tiles": TILES}}
  failed, lines = {}, []
  assert D.run_once(folders, str(tmp_path), log=lines.append, failed=failed, builder=boom) == 0
  assert len(calls) == 2 and failed[FOLDER] == 2 and "skipped" in lines[-1]
  assert D.run_once(folders, str(tmp_path), log=lines.append, failed=failed, builder=boom) == 0
  assert len(calls) == 2                                           # not again this parked session


def test_parse_position_capnp_json_and_junk():
  from openpilot.cereal import messaging
  msg = messaging.new_message("gpsLocationExternal")
  msg.gpsLocationExternal.latitude, msg.gpsLocationExternal.longitude = 42.6, -83.1
  msg.gpsLocationExternal.hasFix = True
  assert D.parse_position(msg.to_bytes()) == (42.6, -83.1)
  msg.gpsLocationExternal.hasFix = False
  assert D.parse_position(msg.to_bytes()) is None
  assert D.parse_position(json.dumps({"latitude": 1.5, "longitude": 2.5}).encode()) == (1.5, 2.5)
  assert D.parse_position(b"junk") is None and D.parse_position(None) is None


# ---------------- a real build from a synthetic tile in a selected 7.0 generation ----------------

def write_tile(path, ways):
  schema = B.load_schema()
  t = schema.Offline.new_message()
  t.minLat, t.minLon, t.maxLat, t.maxLon = 42.25, -83.75, 42.5, -83.5
  lst = t.init("ways", len(ways))
  for w, (wid, pts) in zip(lst, ways, strict=True):
    w.id, w.name, w.highwayClass, w.maxSpeed = wid, "Test Road", 7, 45 * B.MPH
    ns = w.init("nodes", len(pts))
    for n, (la, lo) in zip(ns, pts, strict=True):
      n.latitude, n.longitude = la, lo
  with open(path, "wb") as f:
    f.write(t.to_bytes_packed())


def curve_pts():
  lat0, lon0 = 42.3, -83.6
  mlat, mlon = 1 / 111195.0, 1 / (111195.0 * math.cos(math.radians(lat0)))
  pts = [(0.0, -300.0 + 20 * i) for i in range(15)]
  r = 100.0
  pts += [(-r + r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in range(0, 91, 5)]
  pts += [(-r - 20 * i, r) for i in range(1, 15)]
  return [(lat0 + y * mlat, lon0 + x * mlon) for x, y in pts]


def make_generation(root):
  name = "42.250000_-83.750000_42.500000_-83.500000"
  stage = os.path.join(root, "stage")
  os.makedirs(os.path.join(stage, "42", "-84"))
  tile = os.path.join(stage, "42", "-84", name)
  write_tile(tile, [(7, curve_pts())])
  data = open(tile, "rb").read()
  receipt = {"version": 1, "tileSchema": "offline.capnp:0xda3a0d9284ca402f",
             "attribution": "© OpenStreetMap contributors; https://www.openstreetmap.org/copyright (ODbL)",
             "sourceVerified": False, "sourceKind": "local-pbf", "inputs": [{"x": 1}],
             "tiles": [{"path": f"42/-84/{name}", "size": len(data), "sha256": hashlib.sha256(data).hexdigest()}],
             "bounds": [42, -84, 44, -82]}
  raw = json.dumps(receipt).encode()
  gen = hashlib.sha256(raw).hexdigest()
  os.makedirs(os.path.join(root, "generations"))
  os.rename(stage, os.path.join(root, "generations", gen))
  with open(os.path.join(root, "generations", gen, "receipt.json"), "wb") as f:
    f.write(raw)
  with open(os.path.join(root, "current.json"), "w") as f:
    json.dump({"version": 1, "generation": gen}, f)
  return name


def test_selected_generation_is_built_and_then_up_to_date(tmp_path):
  offline, curves = str(tmp_path / "offline"), str(tmp_path / "curves")
  os.makedirs(offline)
  name = make_generation(offline)
  folders = D.selected_folders(offline)
  assert list(folders) == [FOLDER] and name in folders[FOLDER]["tiles"]
  lines = []
  assert D.run_once(folders, curves, log=lines.append) == 1
  out = json.load(open(os.path.join(curves, FOLDER, name + ".json")))
  assert out["version"] == B.FORMAT_VERSION and out["builder"] == B.BUILDER_VERSION
  assert len(out["curves"]) == 1 and out["curves"][0]["prof"] and abs(out["curves"][0]["r"] - 100) < 12
  assert D.check_folder(FOLDER, folders[FOLDER]["tiles"], curves) is None
  assert D.run_once(folders, curves, log=lines.append) == 0
  assert all("42.3" not in line and "-83.6" not in line for line in lines)


def test_the_builder_runs_for_every_option_that_needs_the_map_view(tmp_path):
  import json
  from pathlib import Path
  from openpilot.common.params import Params
  from openpilot.starpilot.car.mazda.speed import config as C
  from openpilot.starpilot.car.mazda.speed import processes
  params = Params(str(tmp_path))
  cp = type("CP", (), {"brand": "mazda", "notCar": False})()
  for doc, want in (({"enabled": True, "curves": "on"}, True), ({"enabled": True, "speed_limit": "test"}, True),
                    ({"enabled": True, "merge": "on"}, True), ({"enabled": True, "exits": "on"}, True),
                    ({"enabled": True, "camera_curves": "on"}, False), ({"enabled": False, "exits": "on"}, False)):
    Path(params.get_param_path(C.KEY)).write_bytes(json.dumps({"version": 3, **doc}).encode())
    assert D.wanted(params, cp) is want, doc
    assert processes.curve_build(False, params, cp) is want       # the daemon starts by the same rule


# ---------------- (config v6) the curve map build range ----------------

NEAR = (42.5, -83.0)
# folder -> miles from NEAR to its box: 0, 34.5, 61.6, 172.7, about 255, 310.9, 768.9
RANGE_FOLDERS = ("42/-84", "40/-84", "40/-86", "38/-84", "42/-78", "36/-84", "30/-90")
IN_RANGE = {30: RANGE_FOLDERS[:1], 50: RANGE_FOLDERS[:2], 100: RANGE_FOLDERS[:3], 200: RANGE_FOLDERS[:4],
            300: RANGE_FOLDERS[:5], None: RANGE_FOLDERS}


def range_folders():
  return {f: {"dir": "/x", "tiles": TILES} for f in reversed(RANGE_FOLDERS)}    # dict order is not the build order


def fake_builder(calls):
  def build(src, out, **kw):
    calls.append(os.path.relpath(out, os.path.dirname(os.path.dirname(out))))
    return {"files": [], "curves": 0, "ramps": 0}
  return build


def test_folder_distances_used_by_the_range_tests():
  miles = [D.folder_box_distance_miles(NEAR, f) for f in RANGE_FOLDERS]
  assert miles == sorted(miles) and miles[0] == 0
  assert 30 < miles[1] < 50 < miles[2] < 100 < miles[3] < 200 < miles[4] < 300 < miles[5]


@pytest.mark.parametrize("miles", [30, 50, 100, 200, 300, None])
def test_plan_with_each_range_nearest_first(tmp_path, miles):
  got = [f for f, _ in D.plan(range_folders(), str(tmp_path), near=NEAR, range_miles=miles)]
  assert got == list(IN_RANGE[miles])
  # no position: every folder, whatever the range (as before)
  assert sorted(f for f, _ in D.plan(range_folders(), str(tmp_path), near=None, range_miles=miles)) == sorted(RANGE_FOLDERS)


@pytest.mark.parametrize("miles", [30, 50, 100, 200, 300, None])
def test_run_once_with_each_range_builds_in_range_and_reports_both_counts(tmp_path, miles):
  calls, reports = [], []
  n = D.run_once(range_folders(), str(tmp_path), near=NEAR, log=lambda _: None, builder=fake_builder(calls),
                 report=reports.append, range_miles=miles)
  want = list(IN_RANGE[miles])
  assert n == len(want) and calls == want
  last = reports[-1]
  assert set(last) == {"state", "built", "total", "failed", "range", "all_built", "all_total"}   # counts only
  assert (last["built"], last["total"], last["all_built"], last["all_total"]) == (len(want), len(want), len(want), 7)
  assert last["range"] == (miles or 0) and last["state"] == "up_to_date"
  first = reports[0]
  assert first["state"] == "building" and first["built"] == 0 and first["all_built"] == 0


def test_the_default_range_is_100_miles_as_before(tmp_path):
  from openpilot.starpilot.car.mazda.speed import config as C
  assert D.RANGE_MILES == C.Config().curve_range_miles == 100.0
  calls = []
  D.run_once(range_folders(), str(tmp_path), near=NEAR, log=lambda _: None, builder=fake_builder(calls))
  assert calls == list(IN_RANGE[100])


def test_folders_out_of_range_count_as_built_from_their_state(tmp_path):
  calls, reports = [], []
  folders = range_folders()
  D.run_once(folders, str(tmp_path), near=NEAR, log=lambda _: None, builder=fake_builder(calls), range_miles=None)
  assert len(calls) == 7
  D.run_once(folders, str(tmp_path), near=NEAR, log=lambda _: None, builder=fake_builder(calls), report=reports.append,
             range_miles=30)
  assert len(calls) == 7                                               # nothing to build
  assert {k: reports[-1][k] for k in ("built", "total", "range", "all_built", "all_total")} == \
    {"built": 1, "total": 1, "range": 30, "all_built": 7, "all_total": 7}
  # a stale folder out of range is not counted as built, and is not built
  folders["30/-90"] = {"dir": "/x", "tiles": dict.fromkeys(TILES, "cd" * 32)}
  D.run_once(folders, str(tmp_path), near=NEAR, log=lambda _: None, builder=fake_builder(calls), report=reports.append,
             range_miles=30)
  assert len(calls) == 7 and reports[-1]["all_built"] == 6


def test_no_position_builds_everything_and_says_one_count(tmp_path):
  from openpilot.starpilot.car.mazda.speed import status as ST
  calls, reports = [], []
  D.run_once(range_folders(), str(tmp_path), near=None, log=lambda _: None, builder=fake_builder(calls),
             report=reports.append, range_miles=100)
  assert sorted(calls) == sorted(RANGE_FOLDERS)
  assert reports[-1]["range"] == 0 and reports[-1]["total"] == 7
  assert ST.build_text(reports[-1]) == "curve maps: up to date (7 of 7 map areas built)."


def test_a_range_change_stops_the_pass_after_the_folder_being_built(tmp_path):
  calls = []
  D.run_once(range_folders(), str(tmp_path), near=NEAR, log=lambda _: None, builder=fake_builder(calls),
             range_miles=None, should_stop=lambda: len(calls) >= 1)
  assert calls == ["42/-84"]


def test_build_range_is_read_from_the_saved_config(tmp_path):
  from pathlib import Path
  from openpilot.common.params import Params
  from openpilot.starpilot.car.mazda.speed import config as C
  params = Params(str(tmp_path))
  assert D.build_range(params) == 100.0                               # nothing saved: the default
  for raw, want in ((b'{"version":6,"curve_range_mi":300}', 300.0), (b'{"version":6,"curve_range_mi":0}', None),
                    (b'{"version":5,"enabled":true}', 100.0), (b'{"version":6,"curve_range_mi":7}', 100.0)):
    Path(params.get_param_path(C.KEY)).write_bytes(raw)
    assert D.build_range(params) == want, raw


def test_status_texts_say_the_range_and_the_whole_map():
  from openpilot.starpilot.car.mazda.speed import status as ST
  t = ST.build_text
  assert t(D.build_status(5, 5, range_miles=100, all_built=5, all_total=33)) == \
    "curve maps: within 100 miles 5 of 5 built; all downloaded 5 of 33."
  assert t(D.build_status(3, 5, building=True, range_miles=100, all_built=3, all_total=33)) == \
    "curve maps: building… within 100 miles 3 of 5 built; all downloaded 3 of 33 (parked, on power)."
  assert t(D.build_status(3, 5, failed=1, range_miles=30, all_built=4, all_total=33)) == \
    "curve maps: within 30 miles 3 of 5 built, 1 failed (see build_log.txt); all downloaded 4 of 33; " + \
    "the rest within 30 miles are built while parked."
  # the whole map (range 0): one count
  assert t(D.build_status(12, 33, range_miles=None, all_built=12, all_total=33)) == \
    "curve maps: 12 of 33 map areas built; the rest are built while parked."
  assert t(D.build_status(12, 33, building=True, range_miles=None, all_built=12, all_total=33)) == \
    "curve maps: building… 12 of 33 map areas built (parked, on power)."
  assert t(D.build_status(33, 33, range_miles=None, all_built=33, all_total=33)) == \
    "curve maps: up to date (33 of 33 map areas built)."
  assert t(D.build_status(0, None)) == "curve maps: no offline map downloaded (download one in Maps settings)."
  # an older status file (no range / all-downloaded counts) still reads
  assert t({"state": "up_to_date", "built": 5, "total": 5, "failed": 0}) == "curve maps: up to date (5 of 5 map areas built)."
  assert t({"state": "building", "built": 3, "total": 5, "failed": 0}) == \
    "curve maps: building… 3 of 5 map areas built (parked, on power)."
  assert t({"state": "partial", "built": 3, "total": 5, "failed": 2}) == \
    "curve maps: 3 of 5 map areas built, 2 failed (see build_log.txt); the rest are built while parked."
  assert t({"state": "up_to_date", "built": 0, "total": 0}) == "curve maps: up to date."
  assert t({"state": "building", "built": "x"}) == "curve maps: building… 0 of 0 map areas built (parked, on power)."
  assert t(None) == "" and t({"state": "odd"}) == ""
  assert all("folder" not in t(v) for v in ({"state": s, "built": 1, "total": 2} for s in ("building", "partial", "up_to_date")))


def test_a_curve_file_that_parsed_is_only_looked_at_again_when_it_changes(tmp_path, monkeypatch):
  import builtins
  import json as _json
  import os as _os
  p = tmp_path / "a.json"
  p.write_text(_json.dumps({"version": D.B.FORMAT_VERSION}))
  D._FILE_OK.clear()
  opened = []
  real = builtins.open

  def counting(path, *a, **k):
    if str(path) == str(p):
      opened.append(1)
    return real(path, *a, **k)
  monkeypatch.setattr(builtins, "open", counting)
  assert D.file_ok(str(p)) and D.file_ok(str(p)) and D.file_ok(str(p))
  assert len(opened) == 1                                   # read once, then only its time and size
  monkeypatch.setattr(builtins, "open", real)
  p.write_text("{broken")                                   # changed (torn): read again, not ok, forgotten
  _os.utime(p, ns=(1, 1))
  assert not D.file_ok(str(p)) and str(p) not in D._FILE_OK
  p.write_text("")
  assert not D.file_ok(str(p))
  assert not D.file_ok(str(tmp_path / "missing.json"))
