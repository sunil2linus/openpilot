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
  * Range: a folder takes 1.5 to 2 minutes whatever its density (loading the tiles ~25 s, the curve search ~50 s),
    and the owner's offline maps cover the whole lower 48 (390 folders: 6 to 10 hours). So only the folders whose
    2 x 2 degree box comes within MazdaCurveMapRange miles (30/50/100/200/300, default 100) of the last known position
    are built, nearest first. The position is LastGPSPosition: the memory copy (fresh, written while driving and
    while parked with a fix) first, the persistent copy (written when a drive ends, survives a reboot) as fallback.
    Folders out of range are left alone: not built, never deleted; built ones are refreshed only once in range
    again. Both the position and the setting are read again on every pass, so parking somewhere else or changing the
    range takes effect on the next pass without a reboot. No position known yet (a new install): nothing is built.
  * Progress and timings go to /data/media/0/curve_map/build_log.txt (appended, size capped) and to cloudlog; a one
    line status goes to the memory param CurveMapBuildStatus for the UI. The log may be shared, so the position in
    it is rounded to 0.1 degree.

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
RANGE_PARAM = "MazdaCurveMapRange"
RANGE_MILES = (30, 50, 100, 200, 300)  # MazdaCurveMapRange index -> miles
RANGE_DEFAULT = 2                      # 100 miles
POSITION_PARAM = "LastGPSPosition"
CHECK_INTERVAL = 60.0                  # s between two looks at the map folders while parked
NOT_MAZDA_INTERVAL = 300.0
LEGACY_BUILDER = 1                     # curve files without a "builder" field were made by this builder (the laptop)
HEAD_BYTES = 200                       # a curve file starts with {"version":4,"builder":1,... : enough to read both
LOG_NAMES = 10                         # folder names listed in the summary line, the rest is "..."
IDLE_MOVE_MILES = 5.0                  # "no maps here" is said again only after the car moved this far (not GPS jitter)
MILE_M = 1609.344
DEG_M = 111_195.0                      # metres per degree of latitude


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


def folder_box(folder):
  """(lat0, lat1, lon0, lon1) of a map folder: '42/-84' is the box lat 42..44, lon -84..-82 (as build_curve_map.
  folders_for names them: the lower corner, floored to an even degree). None for a name that is not a folder."""
  try:
    la, lo = (int(x) for x in folder.split("/"))
  except ValueError:
    return None
  return la, la + 2, lo, lo + 2


def box_distance_miles(near, folder):
  """Miles from the position `near` (lat, lon) to the nearest point of the folder's box: 0 inside it. A flat-earth
  distance with the longitude scaled at the latitude half way to that point: good to a percent at these sizes."""
  box = folder_box(folder)
  if box is None:
    return float("inf")
  lat, lon = near
  la = min(max(lat, box[0]), box[1])              # the nearest point of the box
  lo = min(max(lon, box[2]), box[3])
  dlat = (la - lat) * DEG_M
  dlon = (lo - lon) * DEG_M * math.cos(math.radians((la + lat) / 2.0))
  return math.hypot(dlat, dlon) / MILE_M


def in_range(folders, near, range_miles):
  """The folders (names) whose box comes within range_miles of `near`, nearest first; the one around the position
  always first. No range: all of them, nearest first. No position: none when there is a range, all otherwise."""
  names = list(folders)
  if near is None:
    return [] if range_miles is not None else names
  if range_miles is not None:
    names = [f for f in names if box_distance_miles(near, f) <= range_miles]
  names.sort(key=lambda f: (box_distance_miles(near, f), f))
  return names


class Plan:
  """What one pass found: todo = [(folder, reason)] to build, nearest first; total = map folders downloaded;
  in_range = how many of them are within range (all when there is no range); skipped = out of range (untouched)."""

  def __init__(self, todo, total, in_range, near, range_miles):
    self.todo, self.total, self.in_range = todo, total, in_range
    self.near, self.range_miles = near, range_miles
    self.skipped = total - in_range

  def where(self):
    """'within 100 mi of 42.6,-83.1': the position rounded to 0.1 degree, this may be shared in a log."""
    if self.near is None:
      return "" if self.range_miles is None else f"within {self.range_miles:g} mi (no position yet)"
    where = f"of {self.near[0]:.1f},{self.near[1]:.1f}"
    return f"within {self.range_miles:g} mi {where}" if self.range_miles is not None else f"nearest first {where}"

  def summary(self):
    """One short line: '3 of 12 map folders within 100 mi of 42.6,-83.1 need building (378 of 390 out of range):
    42/-84, 42/-86, ...' (at most LOG_NAMES names)."""
    names = [f for f, _ in self.todo[:LOG_NAMES]] + (["..."] if len(self.todo) > LOG_NAMES else [])
    s = f"{len(self.todo)} of {self.in_range} map folders {self.where()} need building".replace("  ", " ")
    if self.skipped:
      s += f" ({self.skipped} of {self.total} out of range)"
    return s + ": " + ", ".join(names)


def plan(osm_root=OSM_ROOT, curve_root=CURVE_ROOT, near=None, range_miles=None):
  """The folders to build: those within range_miles of `near` (lat, lon) whose curve files are missing or stale,
  nearest first. Folders out of range are not even checked (checking a laptop-built folder writes its record)."""
  folders = map_folders(osm_root)
  wanted = in_range(folders, near, range_miles)
  todo = []
  for folder in wanted:
    reason = check_folder(folder, folders[folder], curve_root)
    if reason is not None:
      todo.append((folder, reason))
  return Plan(todo, len(folders), len(wanted), near, range_miles)


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


def _plural(n, word="folder"):
  return f"{n} {word}{'s' if n != 1 else ''}"


def idle_notice(p):
  """Why nothing can be built this pass, as (kind, log line, status line), or None when there is work or everything
  is up to date."""
  if p.near is None and p.range_miles is not None:
    return ("no position", "no GPS position yet: drive once, the map folders around the car are built after that",
            "Curve files: no GPS position yet, drive once")
  if p.in_range == 0 and p.range_miles is not None:
    where = p.where()
    if p.total == 0:
      line = f"no offline maps downloaded ({where}): download this area in StarPilot's map settings"
    else:
      line = f"no downloaded maps {where} ({_plural(p.total, 'map folder')} elsewhere): download this area in StarPilot's map settings"
    return ("no maps", line, f"No downloaded maps within {p.range_miles:g} mi of here: download this area in StarPilot's map settings")
  return None


def _miles_between(a, b):
  dlat = (a[0] - b[0]) * DEG_M
  dlon = (a[1] - b[1]) * DEG_M * math.cos(math.radians((a[0] + b[0]) / 2.0))
  return math.hypot(dlat, dlon) / MILE_M


def same_situation(idle, kind, p):
  """True when the idle notice `kind` was already logged for this range and (within IDLE_MOVE_MILES, GPS jitter
  while parked must not count) this position."""
  if idle.get("kind") != kind or idle.get("range") != p.range_miles:
    return False
  prev = idle.get("near")
  if prev is None or p.near is None:
    return prev is None and p.near is None
  return _miles_between(prev, p.near) <= IDLE_MOVE_MILES


def run_once(osm_root=OSM_ROOT, curve_root=CURVE_ROOT, near=None, log=print, status=None, should_stop=lambda: False,
             range_miles=None, idle=None):
  """Build every folder within range that needs it. Returns (folders built, folders downloaded). `idle`, a dict the
  caller keeps between passes, rate-limits the "nothing to do here" notices to once per situation (position, range)
  instead of once a minute; the status line is refreshed every pass."""
  p = plan(osm_root, curve_root, near, range_miles)
  notice = idle_notice(p)
  if notice is not None:
    kind, line, text = notice
    if idle is None or not same_situation(idle, kind, p):
      log(line)
    if idle is not None:
      idle.update(kind=kind, near=p.near, range=p.range_miles)
    if status is not None:
      status(text)
    return 0, p.total
  if idle is not None:
    idle.clear()
  scope = f"{_plural(p.in_range, 'map folder')} {p.where()}".replace("  ", " ").strip()
  if p.skipped:
    scope += f", {p.skipped} out of range"
  if not p.todo:
    if status is not None:
      status(f"Curve files up to date: {scope}")
    return 0, p.total
  log(p.summary())
  t0 = time.monotonic()
  built = 0
  for folder, why in p.todo:
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
    log(f"all done: {_plural(built)} in {time.monotonic() - t0:.0f} s")
  if status is not None:
    status(f"Built {_plural(built)} in {time.monotonic() - t0:.0f} s ({scope})")
  return built, p.total


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


def parse_position(raw):
  """(lat, lon) from a LastGPSPosition JSON string, or None when there is none or it is not a fix."""
  try:
    gps = json.loads(raw or "null")
    lat, lon = float(gps["latitude"]), float(gps["longitude"])
  except (TypeError, ValueError, KeyError):
    return None
  if not (math.isfinite(lat) and math.isfinite(lon)) or (lat == 0.0 and lon == 0.0) or abs(lat) > 90 or abs(lon) > 180:
    return None
  if isinstance(gps, dict) and gps.get("hasFix") is False:
    return None
  return lat, lon


def last_position(*stores):
  """The last known position from the first param store that has one: the memory copy (fresh, lost at a reboot) is
  given first, the persistent copy (written when a drive ends) second."""
  for store in stores:
    try:
      pos = parse_position(store.get(POSITION_PARAM, encoding="utf8"))
    except Exception:
      pos = None
    if pos is not None:
      return pos
  return None


def range_miles(params):
  """The MazdaCurveMapRange setting in miles (read every pass: a change needs no reboot)."""
  try:
    idx = int(params.get_int(RANGE_PARAM, return_default=True, default=RANGE_DEFAULT))
  except Exception:
    idx = RANGE_DEFAULT
  return RANGE_MILES[idx] if 0 <= idx < len(RANGE_MILES) else RANGE_MILES[RANGE_DEFAULT]


def main():
  from openpilot.common.params import Params
  from openpilot.common.swaglog import cloudlog
  _lowest_priority()
  params = Params()
  params_memory = Params(memory=True)
  log = BuildLog(mirror=cloudlog.info)
  idle = {}

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
      # position and range are read every pass: a car parked somewhere new, a GPS fix that arrived while parked, or a
      # changed setting all take effect on the next pass
      run_once(near=last_position(params_memory, params), range_miles=range_miles(params), log=log, status=status, idle=idle)
    except Exception:
      cloudlog.exception("curve_mapd failed")
      log("failed, see the system log")
    time.sleep(CHECK_INTERVAL)


if __name__ == "__main__":
  main()
