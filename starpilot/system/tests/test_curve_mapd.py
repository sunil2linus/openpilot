"""Offline tests for curve_mapd (building the curve files on the device) on synthetic offline map tiles: a small
packed capnp file per 0.25 deg square, made here with the same schema the device's map download uses. No GPS data
of anyone: the roads are drawn at a fixed test position."""
import importlib.util
import json
import math
import os
import time

import capnp
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


D = _load("curve_mapd", os.path.join(ROOT, "starpilot", "system", "curve_mapd.py"))
B = D.B
G = _load("tbcm", os.path.join(ROOT, "tools", "curve_map", "test_build_curve_map.py"))
MC = _load("map_curves", os.path.join(ROOT, "starpilot", "controls", "lib", "map_curves.py"))
capnp.remove_import_hook()
SCHEMA = capnp.load(os.path.join(ROOT, "tools", "curve_map", "offline_maps.capnp"))

LAT0, LON0 = 42.6, -83.0          # the test roads sit around here (the 42/-84 folder, square 42.5_-83.0)


def write_tile(path, ways):
  """A packed offline map tile with the given Way objects (as the map download writes them)."""
  lats = np.concatenate([w.lat for w in ways.values()])
  lons = np.concatenate([w.lon for w in ways.values()])
  msg = SCHEMA.Offline.new_message()
  msg.minLat, msg.maxLat = float(lats.min()), float(lats.max())
  msg.minLon, msg.maxLon = float(lons.min()), float(lons.max())
  lst = msg.init("ways", len(ways))
  for k, w in enumerate(ways.values()):
    ww = lst[k]
    ww.id, ww.name, ww.ref, ww.highwayClass = int(w.id), w.name, w.ref, int(w.cls)
    ww.maxSpeed, ww.advisorySpeed, ww.oneWay = float(w.speed), float(w.advisory), bool(w.oneway)
    ww.minLat, ww.maxLat, ww.minLon, ww.maxLon = float(w.lat.min()), float(w.lat.max()), float(w.lon.min()), float(w.lon.max())
    nodes = ww.init("nodes", len(w.lat))
    for m in range(len(w.lat)):
      nodes[m].latitude, nodes[m].longitude = float(w.lat[m]), float(w.lon[m])
  os.makedirs(os.path.dirname(path), exist_ok=True)
  with open(path, "wb") as f:
    f.write(msg.to_bytes_packed())


def square_of(lat, lon):
  return B.tile_of(lat, lon)


def make_osm(osm_root, folder="42/-84", n_squares=1, roads_per_square=1, mtime=None):
  """A map folder with n_squares tiles, each with a curvy test road (and a residential street a ramp ends on in the
  first), at distinct positions inside the square. Returns the tile paths."""
  paths = []
  wid = 1
  for k in range(n_squares):
    lat = LAT0 + 0.25 * k
    ways = {}
    for r in range(roads_per_square):
      G.LAT0, G.LON0 = lat + 0.02 * r, LON0 + 0.03 * r
      for w in G.curve_road(r=115, turn=40, split=True, name=f"Road {k}-{r}").values():
        w.id = wid
        wid += 1
        ways[w.id] = w
    if k == 0:
      G.LAT0, G.LON0 = lat + 0.1, LON0 + 0.1
      for w in G.ramp_scene().values():
        w.id = wid
        wid += 1
        ways[w.id] = w
    G.LAT0, G.LON0 = 42.6, -83.0
    path = os.path.join(osm_root, folder, square_of(lat + 0.01, LON0 + 0.01))
    write_tile(path, ways)
    if mtime is not None:
      os.utime(path, (mtime, mtime))
    paths.append(path)
  return paths


def test_builds_missing_folder_and_skips_it_next_time(tmp_path):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, n_squares=2)
  log = []
  built, total = D.run_once(osm, cm, log=log.append)
  assert (built, total) == (1, 1)
  files = D.square_files(os.path.join(cm, "42", "-84"))
  assert len(files) >= 2 and all(f.startswith("42.") for f in files), files
  for name in files:
    with open(os.path.join(cm, "42", "-84", name + ".json")) as f:
      d = json.load(f)
    assert d["version"] == B.FORMAT_VERSION and d["builder"] == B.BUILDER_VERSION and d["source"] == "device"
    assert d["curves"] or d["ramps"] or d["roads"]
  state = D.read_state(os.path.join(cm, "42", "-84"))
  assert state["builder"] == B.BUILDER_VERSION and sorted(state["files"]) == files and len(state["tiles"]) == 2
  assert any("done in" in line for line in log) and any("square" in line for line in log)
  assert not any(p.endswith(".tmp") for p in os.listdir(os.path.join(cm, "42", "-84")))
  # up to date: nothing happens
  assert D.run_once(osm, cm, log=log.append) == (0, 1)
  assert D.plan(osm, cm).todo == []


def test_newer_map_tile_rebuilds_the_folder(tmp_path):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  paths = make_osm(osm, n_squares=1, mtime=1_600_000_000)
  D.run_once(osm, cm, log=lambda m: None)
  before = os.stat(os.path.join(cm, "42", "-84", D.square_files(os.path.join(cm, "42", "-84"))[0] + ".json")).st_mtime_ns
  assert D.plan(osm, cm).todo == []
  os.utime(paths[0], (1_700_000_000, 1_700_000_000))        # the map download replaced the tile
  todo = D.plan(osm, cm).todo
  assert todo == [("42/-84", "map data changed")]
  time.sleep(0.01)
  assert D.run_once(osm, cm, log=lambda m: None)[0] == 1
  after = os.stat(os.path.join(cm, "42", "-84", D.square_files(os.path.join(cm, "42", "-84"))[0] + ".json")).st_mtime_ns
  assert after > before
  assert D.read_state(os.path.join(cm, "42", "-84"))["tiles"][os.path.basename(paths[0])][1] == 1_700_000_000


def test_builder_version_bump_rebuilds(tmp_path, monkeypatch):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm)
  D.run_once(osm, cm, log=lambda m: None)
  assert D.plan(osm, cm).todo == []
  monkeypatch.setattr(D, "BUILDER_VERSION", B.BUILDER_VERSION + 1)
  monkeypatch.setattr(B, "BUILDER_VERSION", B.BUILDER_VERSION + 1)
  todo = D.plan(osm, cm).todo
  assert todo and todo[0][1].startswith("older builder")
  assert D.run_once(osm, cm, log=lambda m: None)[0] == 1
  assert D.read_state(os.path.join(cm, "42", "-84"))["builder"] == D.BUILDER_VERSION
  assert D.plan(osm, cm).todo == []


def test_laptop_files_are_adopted_not_rebuilt(tmp_path):
  """Files the owner built on the laptop (no state record, no builder field) stay when they are newer than the map
  tiles; they are rebuilt when a map tile is newer or the format is old."""
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  paths = make_osm(osm, mtime=1_600_000_000)
  folder = os.path.join(cm, "42", "-84")
  os.makedirs(folder)
  laptop = os.path.join(folder, square_of(LAT0, LON0) + ".json")
  with open(laptop, "w") as f:
    json.dump({"version": B.FORMAT_VERSION, "keep_lat": 2.2, "curves": [{"fake": 1}], "ramps": [], "chains": [], "roads": []}, f,
              separators=(",", ":"))
  assert D.file_versions(laptop) == (B.FORMAT_VERSION, D.LEGACY_BUILDER)
  assert D.plan(osm, cm).todo == []                      # adopted
  state = D.read_state(folder)
  assert state["source"] == "adopted" and state["files"] == [square_of(LAT0, LON0)]
  with open(laptop) as f:
    assert json.load(f)["curves"] == [{"fake": 1}]     # untouched
  # a newer map tile: rebuilt (and the fake file, which the new build does not produce, is removed)
  os.utime(paths[0], (1_700_000_000, 1_700_000_000))
  todo = D.plan(osm, cm).todo
  assert todo == [("42/-84", "map data changed")]
  D.run_once(osm, cm, log=lambda m: None)
  with open(laptop) as f:
    assert json.load(f)["source"] == "device"
  # an old format without a state record is rebuilt at once
  os.remove(os.path.join(folder, D.STATE_NAME))
  with open(laptop, "w") as f:
    json.dump({"version": 3, "curves": []}, f)
  os.utime(laptop, None)
  assert D.plan(osm, cm).todo == [("42/-84", "old format 3")]


def test_write_is_atomic_and_a_kill_leaves_no_half_file(tmp_path, monkeypatch):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, n_squares=2)
  folder = os.path.join(cm, "42", "-84")
  seen = []
  real_replace = os.replace

  def replace(src, dst):
    assert src.endswith(".tmp") and os.path.getsize(src) > 0
    with open(src) as f:
      json.load(f)                                      # complete before it gets its name
    real_replace(src, dst)
    if dst.endswith(".json"):
      seen.append(dst)
      if len(seen) == 1:
        raise KeyboardInterrupt                         # the car started: manager kills the process
  monkeypatch.setattr(B.os, "replace", replace)
  try:
    D.run_once(osm, cm, log=lambda m: None)
  except KeyboardInterrupt:
    pass
  names = os.listdir(folder)
  assert not any(n.endswith(".tmp") for n in names) and D.read_state(folder).get("building")
  assert len([n for n in names if n.endswith(".json")]) == 1
  monkeypatch.setattr(B.os, "replace", real_replace)
  # next run: the folder is built again (resume = finished folders are skipped, this one was not finished)
  todo = D.plan(osm, cm).todo
  assert todo == [("42/-84", "unfinished build")]
  assert D.run_once(osm, cm, log=lambda m: None)[0] == 1
  assert D.read_state(folder) is not None and D.plan(osm, cm).todo == []


def test_nothing_under_osm_is_changed(tmp_path):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  paths = make_osm(osm, n_squares=2)
  before = {p: (os.path.getsize(p), os.stat(p).st_mtime_ns, open(p, "rb").read()) for p in paths}
  D.run_once(osm, cm, log=lambda m: None)
  assert {p: (os.path.getsize(p), os.stat(p).st_mtime_ns, open(p, "rb").read()) for p in paths} == before
  assert sorted(os.listdir(os.path.join(osm, "42", "-84"))) == sorted(os.path.basename(p) for p in paths)


def test_plan_builds_the_folder_around_the_car_first(tmp_path):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, folder="42/-84")
  make_osm(osm, folder="40/-90")
  p = D.plan(osm, cm, near=(41.0, -89.0))
  assert p.total == 2 and [f for f, _ in p.todo] == ["40/-90", "42/-84"]
  p = D.plan(osm, cm, near=(42.6, -83.0))
  assert [f for f, _ in p.todo] == ["42/-84", "40/-90"]


def test_build_log_appends_and_rotates(tmp_path):
  log = D.BuildLog(str(tmp_path), max_bytes=200)
  for k in range(20):
    log(f"line {k} " + "x" * 30)
  assert os.path.exists(log.path) and os.path.exists(log.path + ".1")
  with open(log.path) as f:
    assert "line 19" in f.read()


def test_store_reindexes_when_the_folder_changes(tmp_path):
  """The car side picks up squares built while parked: the folder signature changes, the index is remade and the new
  square loads; nothing is rescanned while the folder is unchanged."""
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, n_squares=1)
  D.run_once(osm, cm, log=lambda m: None)
  store = MC.MapCurveStore(cm, background=False)
  first = set(store.index)
  assert store.covers(LAT0, LON0)
  mc = MC.MapCurves(store)
  mc.update(LAT0, LON0, 0.0, 0, True)
  assert store.loaded and store.grid
  assert not store.rescan_if_changed(now=0.0, force=True)          # unchanged: no rescan
  # a second square appears (built later, or copied over): detected on the forced check (drive start) ...
  time.sleep(0.02)
  make_osm(osm, n_squares=2)
  D.run_once(osm, cm, log=lambda m: None)
  assert len(set(store.index)) == len(first)                        # not yet looked
  assert not store.rescan_if_changed(now=1.0)                       # too soon for the periodic check
  assert store.rescan_if_changed(now=1.0, force=True)
  assert set(store.index) > first and store.covers(LAT0 + 0.25, LON0)
  assert store.loaded == {} and store.wanted == []                  # loaded squares dropped (may be rebuilt)
  mc.update(LAT0 + 0.25, LON0, 0.0, 0, True)
  assert any(n.startswith(f"{math.floor((LAT0 + 0.25) * 4) / 4:.6f}") for n in store.loaded)
  # ... and on the periodic check once RESCAN_SECONDS passed
  time.sleep(0.02)
  os.utime(os.path.join(cm, "42", "-84"), None)
  assert not store.rescan_if_changed(now=1.0 + MC.RESCAN_SECONDS - 1)
  assert store.rescan_if_changed(now=1.0 + MC.RESCAN_SECONDS + 1)


def test_planner_wrapper_forces_a_rescan_at_drive_start(tmp_path):
  class P:
    def __init__(self, v):
      self.v = v

    def get(self, key, encoding=None):
      return self.v.get(key)

    def get_bool(self, key):
      return bool(self.v.get(key))

  class DS:
    def __init__(self, started):
      self.started = started

  class SM(dict):
    alive = {"navInstruction": False}
    valid = {"navInstruction": False}
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, n_squares=1)
  D.run_once(osm, cm, log=lambda m: None)
  MC.CURVE_DIR = cm
  pl = MC.MapCurvesPlanner(P({"MazdaMapCurves": "2"}), P({}))
  sm = SM(deviceState=DS(True))
  pl.update(sm, None, 0.0, 20.0)
  assert pl.mc is not None
  store = pl.mc.store
  store.background = False
  n_before = len(store.index)
  time.sleep(0.02)
  make_osm(osm, n_squares=2)
  D.run_once(osm, cm, log=lambda m: None)
  pl.update(sm, None, 0.0, 20.0)                     # same drive: not yet due
  assert len(store.index) == n_before
  sm["deviceState"] = DS(False)
  pl.update(sm, None, 0.0, 20.0)
  sm["deviceState"] = DS(True)
  pl.update(sm, None, 0.0, 20.0)                     # drive start: forced
  assert len(store.index) > n_before


# ---------------- the build range (MazdaCurveMapRange) ----------------

class FakeParams:
  def __init__(self, vals=None):
    self.vals = dict(vals or {})

  def get(self, key, encoding=None, **kw):
    return self.vals.get(key)

  def get_int(self, key, return_default=False, default=0, **kw):
    v = self.vals.get(key)
    return default if v is None else int(v)

  def get_bool(self, key, **kw):
    return bool(self.vals.get(key))


def gps(lat, lon, **extra):
  return json.dumps(dict({"latitude": lat, "longitude": lon, "hasFix": True}, **extra))


def test_folder_box_naming_matches_the_builder():
  """'42/-84' is lat 42..44, lon -84..-82: the folder build_curve_map.folders_for names for a point in that box."""
  assert D.folder_box("42/-84") == (42, 44, -84, -82)
  assert D.folder_box("-2/-2") == (-2, 0, -2, 0)
  assert D.folder_box("build_log.txt") is None
  assert B.folders_for(42.6, -83.1, 1) == ["42/-84"]
  assert B.folders_for(43.9, -82.1, 1) == ["42/-84"]


def test_box_distance_inside_at_the_edge_and_near_a_corner():
  here = (42.6, -83.1)
  assert D.box_distance_miles(here, "42/-84") == 0.0                      # inside
  assert D.box_distance_miles((42.0, -83.1), "42/-84") == 0.0             # on the edge
  # straight south across the edge of 40/-84 (box lat 40..42): 0.6 deg of latitude, whatever the longitude
  south = D.box_distance_miles(here, "40/-84")
  assert abs(south - 0.6 * D.DEG_M / D.MILE_M) < 0.01
  # 40/-86 (lat 40..42, lon -86..-84): the nearest point is its corner (42, -84)
  corner = D.box_distance_miles(here, "40/-86")
  dlat, dlon = 0.6 * D.DEG_M, 0.9 * D.DEG_M * math.cos(math.radians(42.3))
  assert abs(corner - math.hypot(dlat, dlon) / D.MILE_M) < 0.01
  assert 60 < corner < 63
  # from the box, not from its centre: the centre of 40/-86 (41, -85) is much further away
  centre = math.hypot(1.6 * D.DEG_M, 1.9 * D.DEG_M * math.cos(math.radians(41.8))) / D.MILE_M
  assert corner < centre - 60
  assert D.box_distance_miles(here, "not/a/folder") == float("inf")


def test_in_range_selection_keeps_nearest_first_and_the_home_folder_always():
  folders = ["40/-90", "40/-86", "42/-86", "40/-84", "42/-84", "44/-84", "42/-82", "30/-100"]
  here = (42.6, -83.1)
  assert D.in_range(folders, here, 30) == ["42/-84"]                       # only the folder around the car
  assert D.in_range(folders, here, 50) == ["42/-84", "40/-84", "42/-86"]   # 0.6 deg south (~41 mi), 0.9 deg west (~46 mi)
  r100 = D.in_range(folders, here, 100)
  assert r100[:3] == ["42/-84", "40/-84", "42/-86"]
  assert set(r100) == {"42/-84", "40/-84", "42/-86", "42/-82", "40/-86", "44/-84"}   # 44/-84: 1.4 deg north, ~97 mi
  assert [D.box_distance_miles(here, f) for f in r100] == sorted(D.box_distance_miles(here, f) for f in r100)
  # the folder around the position always qualifies, even at the smallest range and with the car in a corner
  assert D.in_range(folders, (43.99, -82.01), 30)[0] == "42/-84"
  # no range: everything, nearest first; no position: nothing with a range, everything without
  assert D.in_range(folders, here, None)[0] == "42/-84" and len(D.in_range(folders, here, None)) == len(folders)
  assert D.in_range(folders, None, 100) == [] and D.in_range(folders, None, None) == folders


def test_plan_builds_only_the_folders_in_range_nearest_first(tmp_path):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  for f in ("42/-84", "40/-84", "40/-90"):
    make_osm(osm, folder=f)
  here = (42.6, -83.1)
  p = D.plan(osm, cm, near=here, range_miles=100)
  assert (p.total, p.in_range, p.skipped) == (3, 2, 1)
  assert [f for f, _ in p.todo] == ["42/-84", "40/-84"]
  assert p.summary().startswith("2 of 2 map folders within 100 mi of 42.6,-83.1 need building (1 of 3 out of range): 42/-84, 40/-84")
  log = []
  status = []
  assert D.run_once(osm, cm, near=here, range_miles=100, log=log.append, status=status.append) == (2, 3)
  assert os.path.isdir(os.path.join(cm, "42", "-84")) and os.path.isdir(os.path.join(cm, "40", "-84"))
  assert not os.path.exists(os.path.join(cm, "40", "-90"))                 # out of range: not built
  assert log[0] == p.summary() and "42.6,-83.1" in log[0] and "-83.0" not in log[0]
  assert "1 out of range" in status[-1] and "100 mi" in status[-1]
  # the next pass: up to date within range, the far folder still not built
  assert D.run_once(osm, cm, near=here, range_miles=100, log=log.append, status=status.append) == (0, 3)
  assert status[-1].startswith("Curve files up to date: 2 map folders within 100 mi") and "1 out of range" in status[-1]
  assert not os.path.exists(os.path.join(cm, "40", "-90"))


def test_summary_lists_at_most_ten_names():
  todo = [(f"{la}/-84", "no curve files") for la in range(0, 30, 2)]
  p = D.Plan(todo, 390, 15, (42.6, -83.1), 100)
  s = p.summary()
  assert s.startswith("15 of 15 map folders within 100 mi of 42.6,-83.1 need building (375 of 390 out of range): 0/-84, 2/-84")
  assert s.endswith("18/-84, ...") and s.count("/") == 10 and len(s) < 200
  assert D.Plan(todo[:1], 1, 1, (42.6, -83.1), 100).summary() == "1 of 1 map folders within 100 mi of 42.6,-83.1 need building: 0/-84"


def test_out_of_range_folders_are_never_touched_even_when_stale(tmp_path):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  paths = make_osm(osm, folder="42/-84", mtime=1_600_000_000)
  make_osm(osm, folder="40/-90", mtime=1_600_000_000)
  here = (42.6, -83.1)
  D.run_once(osm, cm, near=here, range_miles=30, log=lambda m: None)
  folder = os.path.join(cm, "42", "-84")
  files = D.square_files(folder)
  assert files and not os.path.exists(os.path.join(cm, "40", "-90"))
  before = {n: open(os.path.join(folder, n + ".json"), "rb").read() for n in files}
  state_before = D.read_state(folder)
  # the owner parks far away; meanwhile the 42/-84 map tile is replaced (stale curve files)
  os.utime(paths[0], (1_700_000_000, 1_700_000_000))
  far = (34.0, -118.2)
  log = []
  assert D.run_once(osm, cm, near=far, range_miles=100, log=log.append) == (0, 2)
  assert {n: open(os.path.join(folder, n + ".json"), "rb").read() for n in files} == before
  assert D.read_state(folder) == state_before                             # not even its record was rewritten
  assert D.plan(osm, cm, near=far, range_miles=100).skipped == 2
  # back home: the stale folder is rebuilt
  time.sleep(0.01)
  p = D.plan(osm, cm, near=here, range_miles=30)
  assert p.todo == [("42/-84", "map data changed")]
  assert D.run_once(osm, cm, near=here, range_miles=30, log=lambda m: None) == (1, 2)
  assert D.read_state(folder)["tiles"][os.path.basename(paths[0])][1] == 1_700_000_000


def test_no_position_builds_nothing_and_says_so_once(tmp_path):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, folder="42/-84")
  log, status, idle = [], [], {}
  assert D.run_once(osm, cm, near=None, range_miles=100, log=log.append, status=status.append, idle=idle) == (0, 1)
  assert not os.path.exists(cm) or not os.listdir(cm)
  assert len(log) == 1 and "no GPS position yet" in log[0] and "drive once" in log[0]
  assert "no GPS position" in status[-1]
  # the next pass, still no position: the status is refreshed, the log is not repeated
  assert D.run_once(osm, cm, near=None, range_miles=100, log=log.append, status=status.append, idle=idle) == (0, 1)
  assert len(log) == 1 and len(status) == 2
  # the position becomes known while the process keeps running: the next pass builds
  assert D.run_once(osm, cm, near=(42.6, -83.1), range_miles=100, log=log.append, status=status.append, idle=idle) == (1, 1)
  assert D.square_files(os.path.join(cm, "42", "-84")) and idle == {}
  # without a range there is no filtering at all (the laptop-style run_once of the earlier tests)
  assert D.run_once(osm, cm, near=None, range_miles=None, log=log.append) == (0, 1)


def test_no_maps_in_range_logs_once_per_position(tmp_path):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, folder="42/-84")
  log, status, idle = [], [], {}
  la = (34.06, -118.25)                                   # parked in Los Angeles, maps downloaded for Michigan only
  assert D.run_once(osm, cm, near=la, range_miles=100, log=log.append, status=status.append, idle=idle) == (0, 1)
  assert len(log) == 1 and log[0].startswith("no downloaded maps within 100 mi of 34.1,-118.2") and "1 map folder elsewhere" in log[0]
  assert "download this area in StarPilot's map settings" in log[0]
  assert status[-1] == "No downloaded maps within 100 mi of here: download this area in StarPilot's map settings"
  assert not os.path.exists(cm) or not os.listdir(cm)
  # same place, the next passes (GPS jitter of a parked car, even across a rounding line): status only
  for pos in ((34.0601, -118.2502), (34.0499, -118.2499), (34.0601, -118.2502)):
    D.run_once(osm, cm, near=pos, range_miles=100, log=log.append, status=status.append, idle=idle)
  assert len(log) == 1 and len(status) == 4
  # a wider range at the same place: said again (it is a new situation), still nothing built
  D.run_once(osm, cm, near=la, range_miles=300, log=log.append, status=status.append, idle=idle)
  assert len(log) == 2 and "within 300 mi" in log[1]
  # parked somewhere else without maps: said again, once
  sf = (37.77, -122.42)
  D.run_once(osm, cm, near=sf, range_miles=100, log=log.append, status=status.append, idle=idle)
  D.run_once(osm, cm, near=sf, range_miles=100, log=log.append, status=status.append, idle=idle)
  assert len(log) == 3 and "37.8,-122.4" in log[2]
  # back in Michigan: builds, and the idle note is forgotten so the next trip west says it again
  assert D.run_once(osm, cm, near=(42.6, -83.1), range_miles=100, log=log.append, status=status.append, idle=idle)[0] == 1
  assert idle == {}
  D.run_once(osm, cm, near=sf, range_miles=100, log=log.append, status=status.append, idle=idle)
  assert log[-1].startswith("no downloaded maps within 100 mi of 37.8,-122.4")
  # no maps downloaded at all
  empty = str(tmp_path / "osm_empty")
  os.makedirs(empty)
  log2 = []
  assert D.run_once(empty, cm, near=sf, range_miles=100, log=log2.append, status=status.append, idle={}) == (0, 0)
  assert log2 == ["no offline maps downloaded (within 100 mi of 37.8,-122.4): download this area in StarPilot's map settings"]


def test_range_setting_and_position_are_read_from_params():
  # the setting: index -> miles, default 100, out of range -> default
  assert D.range_miles(FakeParams()) == 100
  assert [D.range_miles(FakeParams({D.RANGE_PARAM: i})) for i in range(5)] == [30, 50, 100, 200, 300]
  assert D.range_miles(FakeParams({D.RANGE_PARAM: 9})) == 100
  # the position: the memory copy first (fresh), the persistent copy when the memory one is gone (a reboot)
  mem, disk = FakeParams(), FakeParams({D.POSITION_PARAM: gps(42.6, -83.1)})
  assert D.last_position(mem, disk) == (42.6, -83.1)
  mem.vals[D.POSITION_PARAM] = gps(34.05, -118.25)
  assert D.last_position(mem, disk) == (34.05, -118.25)
  assert D.last_position(FakeParams(), FakeParams()) is None
  # not a fix / null island / garbage: no position
  assert D.parse_position(gps(0.0, 0.0)) is None
  assert D.parse_position(gps(42.6, -83.1, hasFix=False)) is None
  assert D.parse_position('{"latitude": "x"}') is None and D.parse_position("") is None and D.parse_position(None) is None
  assert D.parse_position(json.dumps({"latitude": 42.6, "longitude": -83.1})) == (42.6, -83.1)   # an older payload without hasFix


def test_range_change_is_picked_up_on_the_next_pass(tmp_path):
  """The daemon reads the setting and the position every pass: 30 mi leaves the folder 0.6 deg south alone, the
  owner picks 50 mi, the next pass builds it, with no restart."""
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, folder="42/-84")
  make_osm(osm, folder="40/-84")
  params, mem, idle = FakeParams({D.RANGE_PARAM: 0, D.POSITION_PARAM: gps(42.6, -83.1)}), FakeParams(), {}

  def one_pass():
    return D.run_once(osm, cm, near=D.last_position(mem, params), range_miles=D.range_miles(params), log=lambda m: None, idle=idle)
  assert one_pass() == (1, 2) and not os.path.exists(os.path.join(cm, "40", "-84"))
  assert one_pass() == (0, 2)
  params.vals[D.RANGE_PARAM] = 1                          # the settings tile: 50 mi
  assert one_pass() == (1, 2) and D.square_files(os.path.join(cm, "40", "-84"))
  # a drive to Chicago and back to parked: the memory position (written while driving) wins over the persistent one
  make_osm(osm, folder="40/-88")
  mem.vals[D.POSITION_PARAM] = gps(41.9, -87.6)
  assert one_pass() == (1, 3) and D.square_files(os.path.join(cm, "40", "-88"))


def test_planner_on_the_road_without_curve_data_reports_no_data(tmp_path):
  """Parked (or driving) where no maps were downloaded: the curve folder is empty or does not cover the car. The
  planner says STATUS_NO_DATA and nothing fails, with and without any curve files at all."""
  import types

  class SM(dict):
    alive = {"navInstruction": False}
    valid = {"navInstruction": False}

  class DS:
    started = True
  for curve_root in (str(tmp_path / "missing"), str(tmp_path / "curve_map")):
    if curve_root.endswith("curve_map"):
      osm = str(tmp_path / "osm")
      make_osm(osm, folder="42/-84")
      D.run_once(osm, curve_root, near=(42.6, -83.1), range_miles=100, log=lambda m: None)
    MC.CURVE_DIR = curve_root
    pl = MC.MapCurvesPlanner(FakeParams({"MazdaMapCurves": "2"}), FakeParams({}))
    sm = SM(deviceState=DS())
    here = {"latitude": 34.05, "longitude": -118.25, "bearing": 90.0, "location_mono_time": 0}
    for _ in range(3):
      pl.update(sm, here, 20.0, 30.0)
    assert pl.mc is not None and not pl.mc.data_here and pl.mc.curve is None and pl.notice is None
    plan = types.SimpleNamespace()
    pl.publish(plan)
    assert plan.mapCurveStatus == MC.STATUS_NO_DATA and plan.mapCurveMode == 2 and plan.mapNavExit == MC.NAV_NONE


def test_owner_example_folder_counts_per_range_choice():
  """The owner's maps cover the lower 48. On a 2 deg grid lat 24..48, lon -126..-68 (348 folders), parked near
  42.6,-83.1: how many folders each setting builds. 30 mi is just the home folder here, 300 mi reaches ~30."""
  folders = [f"{la}/{lo}" for la in range(24, 48, 2) for lo in range(-126, -68, 2)]
  assert len(folders) == 348
  counts = {r: len(D.in_range(folders, (42.6, -83.1), r)) for r in D.RANGE_MILES}
  assert counts[30] == 1 and counts[50] == 3
  assert counts[100] < counts[200] < counts[300] <= 40
  assert all(D.in_range(folders, (42.6, -83.1), r)[0] == "42/-84" for r in D.RANGE_MILES)


# ---------------- the map selection and the ticked states (MapsSelected, MazdaCurveMapStates) ----------------

R = D.R
MI, IL, IN, KY, WA, DE = "us_state.MI", "us_state.IL", "us_state.IN", "us_state.KY", "us_state.WA", "nation.DE"
LABELS = {MI: "Michigan", IL: "Illinois", IN: "Indiana", KY: "Kentucky", WA: "Washington", DE: "Germany", "nation.US": "United States"}
HOME = (42.6, -83.1)                 # parked near Detroit: the 42/-84 folder


def regions(selected=(), ticked=(), monkeypatch=None):
  if monkeypatch is not None:
    monkeypatch.setattr(D, "_LABELS", dict(LABELS))
  return D.Regions(selected=list(selected), ticked=list(ticked))


def test_token_to_boxes():
  """A state token has a box, the US is the union of its states, Alaska crosses 180 (two boxes), a country or a
  token this file does not know has none; the CSV parser takes prefixed tokens and bare state codes only."""
  assert R.region_boxes(MI) == ((41.69, 48.31, -90.42, -82.41),)
  assert len(R.region_boxes("us_state.AK")) == 2 and R.region_boxes("us_state.AK")[1][2] > 170
  assert len(R.region_boxes("nation.US")) == 52 and R.region_boxes("nation.CA")[0][1] > 80
  assert R.region_boxes(DE) is None and R.region_boxes("us_state.XX") is None and R.region_boxes("pkg:midwest") is None
  assert R.region_boxes(None) is None and R.region_boxes("") is None
  for code, boxes in R.US_STATE_BOXES.items():
    for lat_min, lat_max, lon_min, lon_max in boxes:
      assert -90 < lat_min < lat_max < 90 and -180 <= lon_min < lon_max <= 180, code
      assert lat_max - lat_min < 21 and lon_max - lon_min < 51, code
  assert len(R.US_STATE_BOXES) == 56                                       # 50 states, DC, 5 territories
  placed, unknown = R.place_tokens([MI, DE, IL])
  assert list(placed) == [MI, IL] and unknown == [DE]
  assert D.parse_tokens("us_state.MI,us_state.IL,,nation.DE") == [MI, IL, DE]
  assert D.parse_tokens(b"us_state.MI, MI ,IL,junk,pkg:midwest,us_state.MI") == [MI, IL]   # bytes, bare codes, once each
  assert D.parse_tokens("") == [] and D.parse_tokens(None) == []
  assert D.token_label("us_state.ZZ") == "ZZ"


def test_folder_overlap_with_the_region_boxes():
  mi = R.region_boxes(MI)
  assert R.box_overlaps("42/-84", mi) and R.box_overlaps("46/-90", mi) and R.box_overlaps("40/-90", mi)   # SW corner, with the margin
  assert not R.box_overlaps("38/-84", mi) and not R.box_overlaps("42/-82", mi)                           # Kentucky, Lake Huron / Ontario
  assert not R.box_overlaps("build_log.txt", mi) and not R.box_overlaps("42/-84", None) and not R.box_overlaps("42/-84", ())
  # the margin: Michigan's east edge is -82.41; without a margin the -84 folder still overlaps, the -82 folder never
  assert R.box_overlaps("42/-84", mi, margin=0.0) and not R.box_overlaps("42/-82", mi, margin=0.3)
  # Indiana's west edge (-88.1) is just past the -88 line: the -90 folder is Indiana's too, margin or not
  assert R.box_overlaps("38/-90", R.region_boxes(IN), margin=0.0)
  hits = R.folders_in_regions(["42/-84", "38/-84", "40/-88", "36/-90"], {MI: mi, IL: R.region_boxes(IL)})
  assert hits == {"42/-84": [MI], "40/-88": [MI, IL], "36/-90": [IL]}
  assert set(R.folders_of_box(((41.69, 48.31, -90.42, -82.41),))) == {f"{la}/{lo}" for la in (40, 42, 44, 46, 48) for lo in (-92, -90, -88, -86, -84)}
  assert R.folders_of_box(R.region_boxes("us_state.DC")) == ["38/-78"]
  assert D.folder_box("42/-84") == R.folder_box("42/-84") == (42, 44, -84, -82)


def test_regions_placement_and_fallbacks(monkeypatch):
  folders = ["42/-84", "38/-84", "36/-90"]
  rg = regions([MI, IL], [MI], monkeypatch)
  assert not rg.empty and rg.restricts() and rg.unknown == [] and not rg.own_everything
  assert rg.folders_of(folders, [MI]) == {"42/-84": [MI]}
  assert rg.folders_of(folders, [MI, IL]) == {"42/-84": [MI], "36/-90": [IL]}
  # a token without a box among placed ones: skipped (its folders cannot be told from leftovers)
  rg = regions([MI, DE], [DE], monkeypatch)
  assert rg.unknown == [DE] and rg.restricts() and not rg.own_everything and rg.folders_of(folders, [DE]) == {}
  assert rg.ticked_unknown() == [DE] and rg.ticked_placed() == []
  # only unplaceable tokens: every downloaded folder is theirs (the old behaviour), range mode is not restricted
  rg = regions([DE], [DE], monkeypatch)
  assert rg.own_everything and not rg.restricts() and set(rg.folders_of(folders, [DE])) == set(folders)
  assert D.Regions().empty and not D.Regions().restricts()


def test_ticked_state_is_built_in_full_whatever_the_range(tmp_path, monkeypatch):
  """Michigan ticked, parked in Detroit with a 30 mi range: every downloaded Michigan folder is built, the Illinois
  folder (selected, out of range) and the Kentucky folder (a leftover) are not. A stale Michigan folder is rebuilt
  on a later pass while it stays ticked; unticked, it is left alone again."""
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  for f in ("42/-84", "44/-88", "36/-90", "38/-84"):
    make_osm(osm, folder=f, mtime=1_600_000_000)
  rg = regions([MI, IL], [MI], monkeypatch)
  p = D.plan(osm, cm, near=HOME, range_miles=30, regions=rg)
  assert [f for f, _ in p.todo] == ["42/-84", "44/-88"] and p.full == {"42/-84": [MI], "44/-88": [MI]}
  assert (p.total, p.in_range, p.skipped) == (4, 2, 2) and p.leftovers == (1, sum(s for s, _ in D.map_folders(osm)["38/-84"].values()))
  assert p.where() == "within 30 mi of 42.6,-83.1 or in Michigan"
  expect = "2 of 2 map folders within 30 mi of 42.6,-83.1 or in Michigan need building (2 of 4 out of range or outside your selected maps)"
  assert p.summary() == expect + ": 42/-84, 44/-88"
  log, status, memo = [], [], {}
  assert D.run_once(osm, cm, near=HOME, range_miles=30, log=log.append, status=status.append, regions=rg, memo=memo) == (2, 4)
  assert D.square_files(os.path.join(cm, "44", "-88")) and D.square_files(os.path.join(cm, "42", "-84"))
  assert not os.path.exists(os.path.join(cm, "36", "-90")) and not os.path.exists(os.path.join(cm, "38", "-84"))
  assert status[0] == "Building area 1 of 2 (now Michigan), about 3 min left (guessing 1.5 min per area)"
  assert any(s.startswith("Building area 2 of 2 (now Michigan), ") and s.endswith("squares, 0 s") and " - 44/-88: " in s for s in status)
  assert status[-1].startswith("Built 2 folders in") and "in Michigan" in status[-1] and "2 out of range or outside your selected maps" in status[-1]
  assert sum("outside your selected maps: leftovers" in line for line in log) == 1
  # the next pass: up to date, the leftover note is not repeated
  assert D.run_once(osm, cm, near=HOME, range_miles=30, log=log.append, status=status.append, regions=rg, memo=memo) == (0, 4)
  assert status[-1].startswith("Curve files up to date: 2 map folders within 30 mi of 42.6,-83.1 or in Michigan")
  assert sum("leftovers" in line for line in log) == 1
  # a map update of the far Michigan folder: rebuilt because the state stays ticked
  tile = os.listdir(os.path.join(osm, "44", "-88"))[0]
  os.utime(os.path.join(osm, "44", "-88", tile), (1_700_000_000, 1_700_000_000))
  assert D.plan(osm, cm, near=HOME, range_miles=30, regions=rg).todo == [("44/-88", "map data changed")]
  time.sleep(0.01)
  assert D.run_once(osm, cm, near=HOME, range_miles=30, log=log.append, regions=rg, memo=memo) == (1, 4)
  # unticked after the trip: the far folder is out of range again, left alone even when stale
  os.utime(os.path.join(osm, "44", "-88", tile), (1_700_000_100, 1_700_000_100))
  p = D.plan(osm, cm, near=HOME, range_miles=30, regions=regions([MI, IL], [], monkeypatch))
  assert p.todo == [] and p.in_range == 1 and p.full == {} and p.where() == "within 30 mi of 42.6,-83.1"


def test_ticked_state_is_built_without_a_gps_position(tmp_path, monkeypatch):
  """A new install, never driven: no position, but the ticked state is built; range mode waits for a drive."""
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, folder="42/-84")
  make_osm(osm, folder="36/-90")
  rg = regions([MI, IL], [MI], monkeypatch)
  log, status, idle = [], [], {}
  assert D.run_once(osm, cm, near=None, range_miles=100, log=log.append, status=status.append, idle=idle, regions=rg) == (1, 2)
  assert D.square_files(os.path.join(cm, "42", "-84")) and not os.path.exists(os.path.join(cm, "36", "-90"))
  assert not any("no GPS position" in line for line in log) and idle == {}
  # nothing ticked and no position: the old notice
  rg = regions([MI, IL], [], monkeypatch)
  assert D.run_once(osm, cm, near=None, range_miles=100, log=log.append, status=status.append, idle=idle, regions=rg) == (0, 2)
  assert "no GPS position yet" in log[-1] and idle["kind"] == "no position"


def test_range_mode_skips_folders_outside_the_selected_regions(tmp_path, monkeypatch):
  """Kentucky is within 300 mi of Detroit but the owner never selected it: its folder is a leftover of the old
  whole-US download and is not built (nor deleted). The leftover count and size are logged once."""
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, folder="42/-84")
  make_osm(osm, folder="38/-84")
  assert D.box_distance_miles(HOME, "38/-84") < 300
  rg = regions([MI, IL], [], monkeypatch)
  p = D.plan(osm, cm, near=HOME, range_miles=300, regions=rg)
  assert [f for f, _ in p.todo] == ["42/-84"] and (p.total, p.in_range, p.skipped) == (2, 1, 1) and p.full == {}
  log, memo = [], {}
  assert D.run_once(osm, cm, near=HOME, range_miles=300, log=log.append, regions=rg, memo=memo) == (1, 2)
  assert not os.path.exists(os.path.join(cm, "38", "-84"))
  leftover_lines = [line for line in log if "leftovers" in line]
  assert len(leftover_lines) == 1 and leftover_lines[0].startswith("1 map folder (") and "MB) are outside your selected maps" in leftover_lines[0]
  assert sorted(os.listdir(os.path.join(osm, "38", "-84")))                    # nothing under osm touched
  D.run_once(osm, cm, near=HOME, range_miles=300, log=log.append, regions=rg, memo=memo)
  assert sum("leftovers" in line for line in log) == 1
  # a selected state with no downloaded maps within range: said so, once
  log, status, idle = [], [], {}
  assert D.run_once(osm, cm, near=(37.0, -95.0), range_miles=30, log=log.append, status=status.append, idle=idle, regions=rg, memo=memo) == (0, 2)
  assert log[-1].startswith("no downloaded maps of your selected regions within 30 mi of 37.0,-95.0 (2 map folders elsewhere)")


def test_empty_selection_keeps_the_old_behaviour(tmp_path):
  """No MapsSelected (or no regions passed at all): everything downloaded within range is built, Kentucky included."""
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, folder="42/-84")
  make_osm(osm, folder="38/-84")
  for rg in (None, D.Regions()):
    p = D.plan(osm, cm, near=HOME, range_miles=300, regions=rg)
    assert [f for f, _ in p.todo] == ["42/-84", "38/-84"] and p.leftovers is None and not p.restricted
    assert p.summary().startswith("2 of 2 map folders within 300 mi of 42.6,-83.1 need building: 42/-84, 38/-84")
  log = []
  assert D.run_once(osm, cm, near=HOME, range_miles=300, log=log.append, regions=D.Regions(), memo={}) == (2, 2)
  assert os.path.isdir(os.path.join(cm, "38", "-84")) and not any("leftovers" in line for line in log)


def test_unknown_token_handling(tmp_path, monkeypatch):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, folder="42/-84")
  make_osm(osm, folder="38/-84")
  # only Germany selected: nothing can be placed, so every downloaded folder is taken to be its own (old behaviour)
  log, status, memo = [], [], {}
  rg = regions([DE], [], monkeypatch)
  p = D.plan(osm, cm, near=HOME, range_miles=300, regions=rg)
  assert len(p.todo) == 2 and p.unknown == [] and p.leftovers is None
  assert D.run_once(osm, cm, near=HOME, range_miles=300, log=log.append, status=status.append, regions=rg, memo=memo) == (2, 2)
  assert not any("cannot place" in line for line in log)
  # ... and ticked: all of it is built in full, no position needed
  p = D.plan(osm, cm, near=None, range_miles=30, regions=regions([DE], [DE], monkeypatch))
  assert p.in_range == 2 and set(p.full) == {"42/-84", "38/-84"} and p.where() == "within 30 mi (no position yet) or in Germany"
  # Germany next to Michigan: cannot be told from the leftovers, skipped and said once; Michigan still restricts
  import shutil
  shutil.rmtree(cm)
  log, status, memo = [], [], {}
  rg = regions([MI, DE], [DE], monkeypatch)
  p = D.plan(osm, cm, near=HOME, range_miles=300, regions=rg)
  assert [f for f, _ in p.todo] == ["42/-84"] and p.unknown == [DE] and p.full == {} and p.leftovers[0] == 1
  assert D.run_once(osm, cm, near=HOME, range_miles=300, log=log.append, status=status.append, regions=rg, memo=memo) == (1, 2)
  assert sum(line.startswith("cannot place Germany on the map") for line in log) == 1
  assert "can't place Germany" in status[-1] and not os.path.exists(os.path.join(cm, "38", "-84"))
  D.run_once(osm, cm, near=HOME, range_miles=300, log=log.append, status=status.append, regions=rg, memo=memo)
  assert sum("cannot place" in line for line in log) == 1 and "can't place Germany" in status[-1]
  # the selection changes to one that places: the note is forgotten, so a later Germany is said again
  D.run_once(osm, cm, near=HOME, range_miles=300, log=log.append, status=status.append, regions=regions([MI], [], monkeypatch), memo=memo)
  assert "unknown" not in memo and "can't place" not in status[-1]


def test_ticked_state_without_downloaded_maps(tmp_path, monkeypatch):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, folder="42/-84")
  # Washington ticked, nothing of it downloaded, nothing else in scope: the status says what to do, logged once
  rg = regions([MI, WA], [WA], monkeypatch)
  log, status, idle, memo = [], [], {}, {}
  assert D.run_once(osm, cm, near=None, range_miles=100, log=log.append, status=status.append, idle=idle, regions=rg, memo=memo) == (0, 1)
  assert log == ["no map tiles for Washington: download them in StarPilot's map settings"]
  assert status[-1] == "Map not downloaded for Washington: download it in StarPilot's map settings"
  D.run_once(osm, cm, near=None, range_miles=100, log=log.append, status=status.append, idle=idle, regions=rg, memo=memo)
  assert len(log) == 1 and len(status) == 2
  # Michigan ticked too: Michigan is built, Washington is noted once in the log and in the status
  rg = regions([MI, WA], [MI, WA], monkeypatch)
  log, status, idle, memo = [], [], {}, {}
  assert D.run_once(osm, cm, near=None, range_miles=100, log=log.append, status=status.append, idle=idle, regions=rg, memo=memo) == (1, 1)
  assert log[0] == "no map tiles for Washington: download them in StarPilot's map settings"
  assert status[-1].startswith("Built 1 folder") and "Washington not downloaded" in status[-1] and "in Michigan" in status[-1]
  D.run_once(osm, cm, near=None, range_miles=100, log=log.append, status=status.append, idle=idle, regions=rg, memo=memo)
  assert sum("no map tiles" in line for line in log) == 1
  assert status[-1].startswith("Curve files up to date: 1 map folder within 100 mi (no position yet) or in Michigan")


def test_progress_and_time_left_text():
  assert D.time_left_text(0, 9, 0.0) == "about 14 min left (guessing 1.5 min per area)"
  assert D.time_left_text(3, 9, 3 * 84.0) == "about 8 min left (avg 1.4 min per area)"
  assert D.time_left_text(8, 9, 8 * 84.0) == "about 80 s left (avg 1.4 min per area)"
  assert D.time_left_text(9, 9, 9 * 84.0) == "almost done (avg 1.4 min per area)"
  assert D.time_left_text(2, 66, 2 * 90.0) == "about 1.6 h left (avg 1.5 min per area)"
  assert D.progress_text("Illinois", 3, 9, 3 * 84.0) == "Building area 4 of 9 (now Illinois), about 8 min left (avg 1.4 min per area)"
  assert D.progress_text("map folder 40/-90", 0, 1, 0.0, "40/-90: 3 squares, 40 s").endswith("(guessing 1.5 min per area) - 40/-90: 3 squares, 40 s")
  p = D.Plan([], 4, 2, HOME, 30, full={"42/-84": [MI, IL]})
  assert p.folder_label("42/-84") in ("Michigan", "MI") and p.folder_label("38/-84") == "map folder 38/-84"


def test_region_status_for_the_ui(tmp_path, monkeypatch):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  make_osm(osm, folder="42/-84")
  make_osm(osm, folder="44/-88")
  make_osm(osm, folder="36/-90")
  folders = D.map_folders(osm)
  rg = regions([MI, IL, WA, DE], [], monkeypatch)
  assert D.region_status(MI, folders, cm, rg) == (2, 0, True)
  assert D.region_status(WA, folders, cm, rg) == (0, 0, True)
  assert D.region_status(DE, folders, cm, rg) == (0, 0, False)
  D.run_once(osm, cm, near=HOME, range_miles=30, log=lambda m: None, regions=rg)
  assert D.region_status(MI, folders, cm, rg) == (2, 1, True) and D.region_status(IL, folders, cm, rg) == (1, 0, True)
  assert D.folder_is_built("42/-84", folders["42/-84"], cm) and not D.folder_is_built("44/-88", folders["44/-88"], cm)
  assert not D.folder_is_built("42/-84", dict(folders["42/-84"], extra=[1, 2]), cm)       # a new tile: stale
  assert not os.path.exists(os.path.join(cm, "36", "-90", D.STATE_NAME))                 # read-only: nothing written
  assert D.region_status(MI, folders, cm) == (2, 1, True)                                 # without a Regions object


def test_regions_are_read_from_params(monkeypatch):
  monkeypatch.setattr(D, "_LABELS", dict(LABELS))
  rg = D.read_regions(FakeParams({D.SELECTED_PARAM: "us_state.IL,us_state.MI", D.TICKED_PARAM: b"us_state.MI"}))
  assert rg.selected == [IL, MI] and rg.ticked == [MI]
  assert D.read_regions(FakeParams()).empty
  # a ticked state no longer in the map selection still counts (the UI lists it so it can be unticked)
  rg = D.read_regions(FakeParams({D.SELECTED_PARAM: "us_state.IL", D.TICKED_PARAM: "us_state.MI"}))
  assert rg.ticked == [MI] and MI in rg.placed and rg.restricts()


def test_owner_example_eight_states():
  """The owner's selection (MI, IL, IN, MO, ND, SD, WA, WI) on a 2 deg grid of the lower 48 (lat 24..48, lon
  -126..-68, 348 folders): all eight ticked is 66 folders; Kentucky never qualifies; range mode at 300 mi from
  Detroit builds fewer folders than without the selection."""
  grid = [f"{la}/{lo}" for la in range(24, 48, 2) for lo in range(-126, -68, 2)]
  toks = [f"us_state.{c}" for c in ("MI", "IL", "IN", "MO", "ND", "SD", "WA", "WI")]
  rg = D.Regions(selected=toks, ticked=toks)
  full = rg.folders_of(grid, toks)
  assert len(full) == 66
  assert all(f not in full for f in R.folders_in_regions(grid, {KY: R.region_boxes(KY)}) if "38/" in f and f not in ("38/-88", "38/-90", "38/-92", "38/-86"))
  per_state = {t: sum(1 for toks_ in full.values() if t in toks_) for t in toks}
  assert per_state["us_state.MI"] == 20 and per_state["us_state.IN"] == 9 and per_state["us_state.WA"] == 10
  near, r300 = (42.6, -83.1), D.in_range(grid, (42.6, -83.1), 300)
  restricted = [f for f in r300 if f in rg.folders_of(grid, toks)]
  assert len(restricted) == 17 < len(r300) and "38/-84" in r300 and "38/-84" not in restricted
  assert restricted[0] == "42/-84" and D.box_distance_miles(near, restricted[-1]) <= 300
