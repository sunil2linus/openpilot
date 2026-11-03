#!/usr/bin/env python3
"""
curve_mapd: builds the Mazda map curve files on the device, while the car is parked.

The map curve slowdown (MazdaMapCurves) reads curve files from /data/media/0/curve_map/<lat>/<lon>/<square>.json,
made from StarPilot's offline map tiles (/data/media/0/osm/offline/<lat>/<lon>/<square>) by tools/curve_map/
build_curve_map.py. Until now the owner ran that script on a laptop and copied the files over. This process does it
on the device, so a new city only needs the map download in the settings:

  * It runs only offroad (manager starts it when the car is off and kills it when a drive starts), only when the
    map curve setting is Test or On, and only on a Mazda (CarParamsPersistent). It is niced to the lowest priority.
  * Every map folder (2 x 2 degrees) is checked: no curve files, map tiles newer than or different from the ones the
    curve files were built from, or files from an older builder (BUILDER_VERSION) -> the folder is built again.
    Curve files the owner built on the laptop are adopted as they are when they are newer than the map tiles and of
    the current format; they are only rebuilt when the map data or the builder changes. Nothing under osm/ is ever
    changed.
  * A folder is one unit of work: its roads join across the squares inside it, so the squares cannot be built one at
    a time, but they are WRITTEN one at a time through a temporary name, so a kill never leaves a half-written file.
    The folder's record (build_state.meta: which tiles it was built from, by which builder) is written last; a
    folder killed half way is built again on the next run, finished folders are skipped (that is the resume).
  * Memory: see build_curve_map.load_ways (about 170 MB peak for the Detroit folder, the biggest the owner has).
  * Progress and timings go to /data/media/0/curve_map/build_log.txt (appended, size capped) and to cloudlog; a one
    line status goes to the memory param CurveMapBuildStatus for the UI.

The pure parts (folder state, planning) have no openpilot imports so they can be unit tested; main() has them.
"""
import datetime
import glob
import importlib.util
import json
import math
import os
import re
import time

OSM_ROOT = "/data/media/0/osm/offline"
CURVE_ROOT = "/data/media/0/curve_map"
LOG_NAME = "build_log.txt"
LOG_MAX_BYTES = 300_000
STATE_NAME = "build_state.meta"        # not .json: the car side indexes every *.json in the folder as a square
STATUS_PARAM = "CurveMapBuildStatus"
CHECK_INTERVAL = 60.0                  # s between two looks at the map folders while parked
NOT_MAZDA_INTERVAL = 300.0
LEGACY_BUILDER = 1                     # curve files without a "builder" field were made by this builder (the laptop)
HEAD_BYTES = 200                       # a curve file starts with {"version":4,"builder":1,... : enough to read both


def _load_builder():
  here = os.path.dirname(os.path.abspath(__file__))
  path = os.path.join(here, "..", "..", "tools", "curve_map", "build_curve_map.py")
  spec = importlib.util.spec_from_file_location("build_curve_map", path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


B = _load_builder()
BUILDER_VERSION = B.BUILDER_VERSION
FORMAT_VERSION = B.FORMAT_VERSION


def map_folders(osm_root=OSM_ROOT):
  """{"42/-84": {"42.000000_-84.000000_42.250000_-83.750000": [size, mtime], ...}} for every map folder with tiles."""
  out = {}
  for d in sorted(glob.glob(os.path.join(osm_root, "*", "*"))):
    if not os.path.isdir(d):
      continue
    tiles = {}
    try:
      with os.scandir(d) as it:
        for e in it:
          if e.is_file() and not e.name.startswith(".") and "_" in e.name:
            st = e.stat()
            tiles[e.name] = [st.st_size, int(st.st_mtime)]
    except OSError:
      continue
    if tiles:
      out[os.path.relpath(d, osm_root)] = tiles
  return out


def file_versions(path):
  """(format version, builder version) from the head of a curve file, without parsing the whole (16 MB a folder)."""
  try:
    with open(path, "rb") as f:
      head = f.read(HEAD_BYTES).decode("utf-8", "ignore")
  except OSError:
    return 0, 0
  v = re.search(r'"version"\s*:\s*(\d+)', head)
  b = re.search(r'"builder"\s*:\s*(\d+)', head)
  return (int(v.group(1)) if v else 0), (int(b.group(1)) if b else LEGACY_BUILDER)


def read_state(curve_dir):
  try:
    with open(os.path.join(curve_dir, STATE_NAME)) as f:
      st = json.load(f)
    return st if isinstance(st, dict) else None
  except (OSError, ValueError):
    return None


def write_state(curve_dir, state):
  os.makedirs(curve_dir, exist_ok=True)
  B.write_json_atomic(os.path.join(curve_dir, STATE_NAME), state)


def square_files(curve_dir):
  return sorted(os.path.basename(p)[:-5] for p in glob.glob(os.path.join(curve_dir, "*.json")))


def check_folder(folder, tiles, curve_root=CURVE_ROOT):
  """Why the folder's curve files must be (re)built, or None when they are up to date. Adopts files the owner built
  on the laptop (no state record yet) by writing their record, when they are of the current format and newer than
  every map tile of the folder."""
  curve_dir = os.path.join(curve_root, folder)
  state = read_state(curve_dir)
  if state is None:
    files = square_files(curve_dir)
    if not files:
      return "no curve files"
    # no record but files: made on the laptop (or the record was deleted); adopt them when they are current
    newest_tile = max(m for _s, m in tiles.values())
    worst_builder = BUILDER_VERSION
    for name in files:
      path = os.path.join(curve_dir, name + ".json")
      fmt, builder = file_versions(path)
      if fmt < FORMAT_VERSION:
        return f"old format {fmt}"
      try:
        if os.stat(path).st_mtime < newest_tile:
          return "map data newer than the curve files"
      except OSError:
        return "curve file missing"
      worst_builder = min(worst_builder, builder)
    if worst_builder < BUILDER_VERSION:
      return f"older builder {worst_builder}"
    write_state(curve_dir, {"builder": worst_builder, "format": FORMAT_VERSION, "tiles": tiles, "files": files,
                            "source": "adopted", "built": datetime.datetime.now().isoformat(timespec="seconds")})
    return None
  if state.get("building"):
    return "unfinished build"
  if int(state.get("builder", 0)) < BUILDER_VERSION:
    return f"older builder {state.get('builder')}"
  if int(state.get("format", 0)) < FORMAT_VERSION:
    return f"old format {state.get('format')}"
  if state.get("tiles") != tiles:
    return "map data changed"
  for name in state.get("files", []):
    if not os.path.exists(os.path.join(curve_dir, name + ".json")):
      return "curve file missing"
  return None


def plan(osm_root=OSM_ROOT, curve_root=CURVE_ROOT, near=None):
  """[(folder, reason)] to build, the folder around `near` (lat, lon) first, then by distance."""
  folders = map_folders(osm_root)
  todo = []
  for folder, tiles in folders.items():
    reason = check_folder(folder, tiles, curve_root)
    if reason is not None:
      todo.append((folder, reason))

  def dist(item):
    if near is None:
      return 0.0
    try:
      la, lo = (float(x) for x in item[0].split("/"))
    except ValueError:
      return 1e9
    return math.hypot(la + 1.0 - near[0], (lo + 1.0 - near[1]) * math.cos(math.radians(near[0])))
  todo.sort(key=dist)
  return todo, len(folders)


class BuildLog:
  """Appends to build_log.txt (size capped: the file is rotated to .1) and mirrors to a callback (cloudlog)."""

  def __init__(self, root=CURVE_ROOT, mirror=None, max_bytes=LOG_MAX_BYTES):
    self.path = os.path.join(root, LOG_NAME)
    self.mirror = mirror
    self.max_bytes = max_bytes

  def __call__(self, msg):
    line = f"{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} {msg}"
    try:
      os.makedirs(os.path.dirname(self.path), exist_ok=True)
      if os.path.exists(self.path) and os.path.getsize(self.path) > self.max_bytes:
        os.replace(self.path, self.path + ".1")
      with open(self.path, "a") as f:
        f.write(line + "\n")
    except OSError:
      pass
    if self.mirror is not None:
      try:
        self.mirror(f"curve_mapd: {msg}")
      except Exception:
        pass


def build_folder(folder, tiles, osm_root=OSM_ROOT, curve_root=CURVE_ROOT, log=print, status=None):
  """Build one map folder's curve files; returns the builder's stats. Files of the previous build that the new
  build did not produce (the map changed) are removed; nothing else in the folder is touched."""
  osm_dir, curve_dir = os.path.join(osm_root, folder), os.path.join(curve_root, folder)
  previous = read_state(curve_dir)
  os.makedirs(curve_dir, exist_ok=True)
  # mark the folder as being built first: killed half way (the car started), it is built again next time
  write_state(curve_dir, dict(previous or {}, building=True))
  for tmp in glob.glob(os.path.join(curve_dir, "*.tmp")):
    try:
      os.remove(tmp)
    except OSError:
      pass
  t0 = time.monotonic()
  n = {"files": 0}

  def progress(name, nbytes, seconds):
    n["files"] += 1
    log(f"  {folder}: square {name} written, {nbytes / 1e3:.0f} kB, {seconds:.0f} s into the folder")
    if status is not None:
      status(f"Building {folder}: {n['files']} squares, {seconds:.0f} s")

  built = datetime.datetime.now().isoformat(timespec="seconds")
  stats = B.build(osm_dir, curve_dir, quiet=True, progress=progress, extra={"built": built, "source": "device"})
  if previous is not None:
    for name in previous.get("files", []):
      if name not in stats["files"]:
        try:
          os.remove(os.path.join(curve_dir, name + ".json"))
          log(f"  {folder}: removed {name} (not in the new map data)")
        except OSError:
          pass
  write_state(curve_dir, {"builder": BUILDER_VERSION, "format": FORMAT_VERSION, "tiles": tiles, "files": stats["files"],
                          "source": "device", "built": built, "seconds": round(time.monotonic() - t0, 1),
                          "curves": stats["curves"], "ramps": stats["ramps"], "roads": stats["roads"]})
  msg = f"{folder}: done in {time.monotonic() - t0:.0f} s: {stats['tiles']} map tiles, {stats['roads']} roads, {stats['curves']} curves, "
  msg += f"{stats['ramps']} ramps, {len(stats['files'])} squares, {stats['bytes'] / 1e6:.1f} MB "
  msg += f"(loading {stats['load_s']:.0f} s, curves {stats['curves_s']:.0f} s)"
  log(msg)
  return stats


def run_once(osm_root=OSM_ROOT, curve_root=CURVE_ROOT, near=None, log=print, status=None, should_stop=lambda: False):
  """Build every folder that needs it. Returns (folders built, folders checked)."""
  todo, total = plan(osm_root, curve_root, near)
  if not todo:
    if status is not None:
      status(f"Curve files up to date ({total} map folder{'s' if total != 1 else ''})")
    return 0, total
  log(f"{len(todo)} of {total} map folders to build: " + ", ".join(f"{f} ({why})" for f, why in todo))
  t0 = time.monotonic()
  built = 0
  for folder, why in todo:
    if should_stop():
      break
    log(f"{folder}: building ({why})")
    try:
      tiles = map_folders(osm_root).get(folder)
      if not tiles:
        continue
      build_folder(folder, tiles, osm_root, curve_root, log, status)
      built += 1
    except FileNotFoundError as e:
      log(f"{folder}: skipped: {e}")
    except MemoryError:
      log(f"{folder}: out of memory, giving up on this folder for now")
      break
  if built:
    log(f"all done: {built} folder{'s' if built != 1 else ''} in {time.monotonic() - t0:.0f} s")
  if status is not None:
    status(f"Built {built} folder{'s' if built != 1 else ''} in {time.monotonic() - t0:.0f} s")
  return built, total


# ---------------- the daemon ----------------

def _lowest_priority():
  try:
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  except (AttributeError, OSError, PermissionError):
    pass
  try:
    os.nice(19)
  except OSError:
    pass


def _is_mazda(params):
  """True when the car fingerprinted so far is a Mazda; None when no car is known yet."""
  try:
    raw = params.get("CarParamsPersistent")
    if not raw:
      return None
    from cereal import car
    with car.CarParams.from_bytes(raw) as cp:
      return str(cp.brand) == "mazda"
  except Exception:
    return None


def _last_position(params):
  try:
    gps = json.loads(params.get("LastGPSPosition", encoding="utf8") or "null")
    return float(gps["latitude"]), float(gps["longitude"])
  except (TypeError, ValueError, KeyError):
    return None


def main():
  from openpilot.common.params import Params
  from openpilot.common.swaglog import cloudlog
  _lowest_priority()
  params = Params()
  params_memory = Params(memory=True)
  log = BuildLog(mirror=cloudlog.info)

  def status(text):
    try:
      params_memory.put(STATUS_PARAM, text)
    except Exception:
      pass

  while True:
    mazda = _is_mazda(params)
    if not mazda:
      status("Curve files: waiting for a Mazda" if mazda is None else "Curve files: not a Mazda")
      time.sleep(NOT_MAZDA_INTERVAL)
      continue
    try:
      run_once(near=_last_position(params), log=log, status=status)
    except Exception:
      cloudlog.exception("curve_mapd failed")
      log("failed, see the system log")
    time.sleep(CHECK_INTERVAL)


if __name__ == "__main__":
  main()
