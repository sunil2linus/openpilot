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
  assert D.plan(osm, cm)[0] == []


def test_newer_map_tile_rebuilds_the_folder(tmp_path):
  osm, cm = str(tmp_path / "osm"), str(tmp_path / "curve_map")
  paths = make_osm(osm, n_squares=1, mtime=1_600_000_000)
  D.run_once(osm, cm, log=lambda m: None)
  before = os.stat(os.path.join(cm, "42", "-84", D.square_files(os.path.join(cm, "42", "-84"))[0] + ".json")).st_mtime_ns
  assert D.plan(osm, cm)[0] == []
  os.utime(paths[0], (1_700_000_000, 1_700_000_000))        # the map download replaced the tile
  todo, _ = D.plan(osm, cm)
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
  assert D.plan(osm, cm)[0] == []
  monkeypatch.setattr(D, "BUILDER_VERSION", B.BUILDER_VERSION + 1)
  monkeypatch.setattr(B, "BUILDER_VERSION", B.BUILDER_VERSION + 1)
  todo, _ = D.plan(osm, cm)
  assert todo and todo[0][1].startswith("older builder")
  assert D.run_once(osm, cm, log=lambda m: None)[0] == 1
  assert D.read_state(os.path.join(cm, "42", "-84"))["builder"] == D.BUILDER_VERSION
  assert D.plan(osm, cm)[0] == []


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
  assert D.plan(osm, cm)[0] == []                      # adopted
  state = D.read_state(folder)
  assert state["source"] == "adopted" and state["files"] == [square_of(LAT0, LON0)]
  with open(laptop) as f:
    assert json.load(f)["curves"] == [{"fake": 1}]     # untouched
  # a newer map tile: rebuilt (and the fake file, which the new build does not produce, is removed)
  os.utime(paths[0], (1_700_000_000, 1_700_000_000))
  todo, _ = D.plan(osm, cm)
  assert todo == [("42/-84", "map data changed")]
  D.run_once(osm, cm, log=lambda m: None)
  with open(laptop) as f:
    assert json.load(f)["source"] == "device"
  # an old format without a state record is rebuilt at once
  os.remove(os.path.join(folder, D.STATE_NAME))
  with open(laptop, "w") as f:
    json.dump({"version": 3, "curves": []}, f)
  os.utime(laptop, None)
  assert D.plan(osm, cm)[0] == [("42/-84", "old format 3")]


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
  todo, _ = D.plan(osm, cm)
  assert todo == [("42/-84", "unfinished build")]
  assert D.run_once(osm, cm, log=lambda m: None)[0] == 1
  assert D.read_state(folder) is not None and D.plan(osm, cm)[0] == []


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
  todo, total = D.plan(osm, cm, near=(41.0, -89.0))
  assert total == 2 and [f for f, _ in todo] == ["40/-90", "42/-84"]
  todo, _ = D.plan(osm, cm, near=(42.6, -83.0))
  assert [f for f, _ in todo] == ["42/-84", "40/-90"]


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
