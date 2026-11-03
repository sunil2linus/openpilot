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
  * Which folders (a folder takes 1.5 to 2 minutes whatever its density; the owner's offline maps cover the whole
    lower 48, 390 folders, 6 to 10 hours):
      - Ticked states (MazdaCurveMapStates, the "Map Build States" tile: a tick per state of the map selection) are
        built in full: every downloaded folder overlapping the state's box, wherever the car is, and kept up to date
        when a map update makes a folder stale. The owner ticks the states of a trip while parked at home, unticks
        them after.
      - Everything else: only the folders whose box comes within MazdaCurveMapRange miles (30/50/100/200/300,
        default 100) of the last known position, measured to the nearest point of the box (the folder around the
        car always qualifies). The position is LastGPSPosition: the memory copy (fresh, written while driving and
        while parked with a fix) first, the persistent copy (written when a drive ends, survives a reboot) as
        fallback. No position known yet (a new install): only ticked states are built.
      - Both are restricted to the regions selected in StarPilot's map settings (MapsSelected) when there is a
        selection: the owner's osm folder holds the whole lower 48 from an older download, but only 8 states are
        selected, so Kentucky (within 300 mi of Detroit) is never built. The leftovers are counted and their size
        logged once; nothing is ever deleted here. No selection: everything downloaded within range, as before.
      - Where a region is comes from starpilot/common/map_region_boxes.py (real outlines of the US states, US/CA/MX).
        A token it cannot place is skipped and said so, unless it is the only kind of region selected: then every
        downloaded folder is taken to be its own.
    Nearest first. Folders out of scope are left alone: not built, never deleted; built ones are refreshed only once
    in scope again. Position, range and both selections are read again on every pass, so parking somewhere else,
    changing the range or ticking a state takes effect on the next pass without a reboot.
  * Progress and timings go to /data/media/0/curve_map/build_log.txt (appended, size capped) and to cloudlog; a one
    line status with a time estimate ("Building area 4 of 9 (now Illinois), about 8 min left (avg 1.4 min per
    area)") goes to the memory param CurveMapBuildStatus for the UI. The log may be shared, so the position in it
    is rounded to 0.1 degree.

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
SELECTED_PARAM = "MapsSelected"        # StarPilot's map settings: the regions downloaded (CSV of tokens)
TICKED_PARAM = "MazdaCurveMapStates"   # the states to build in full, whatever the range (CSV of the same tokens)
CHECK_INTERVAL = 60.0                  # s between two looks at the map folders while parked
NOT_MAZDA_INTERVAL = 300.0
LEGACY_BUILDER = 1                     # curve files without a "builder" field were made by this builder (the laptop)
HEAD_BYTES = 200                       # a curve file starts with {"version":4,"builder":1,... : enough to read both
LOG_NAMES = 10                         # folder names listed in the summary line, the rest is "..."
IDLE_MOVE_MILES = 5.0                  # "no maps here" is said again only after the car moved this far (not GPS jitter)
MILE_M = 1609.344
DEG_M = 111_195.0                      # metres per degree of latitude
FOLDER_SECONDS_GUESS = 90.0            # time per folder assumed before the first one of a pass is timed


def _load_by_path(name, *parts):
  here = os.path.dirname(os.path.abspath(__file__))
  path = os.path.join(here, "..", "..", *parts)
  spec = importlib.util.spec_from_file_location(name, path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


B = _load_by_path("build_curve_map", "tools", "curve_map", "build_curve_map.py")
R = _load_by_path("map_region_boxes", "starpilot", "common", "map_region_boxes.py")
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


def folder_is_built(folder, tiles, curve_root=CURVE_ROOT):
  """Read-only: True when the folder has a finished record for exactly these tiles by the current builder (for the
  UI; check_folder, which may write an adopted record, is for the daemon)."""
  state = read_state(os.path.join(curve_root, folder))
  if state is None or state.get("building"):
    return False
  try:
    return (int(state.get("builder", 0)) >= BUILDER_VERSION and int(state.get("format", 0)) >= FORMAT_VERSION
            and state.get("tiles") == tiles)
  except (TypeError, ValueError):
    return False


# ---------------- where the folders are: range, regions ----------------

def folder_box(folder):
  """(lat0, lat1, lon0, lon1) of a map folder: '42/-84' is the box lat 42..44, lon -84..-82 (as build_curve_map.
  folders_for names them: the lower corner, floored to an even degree). None for a name that is not a folder."""
  return R.folder_box(folder)


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


def parse_tokens(raw):
  """The region tokens of a MapsSelected / MazdaCurveMapStates CSV ('us_state.MI,us_state.IL'), in order, once each.
  A bare state code (an old selection) counts as a state. Anything else is dropped."""
  if isinstance(raw, bytes):
    raw = raw.decode("utf-8", "ignore")
  out = []
  for t in (raw or "").split(","):
    t = t.strip()
    if not t:
      continue
    if not t.startswith((R.STATE_PREFIX, R.COUNTRY_PREFIX)):
      if len(t) == 2 and t.isalpha() and t.isupper():
        t = R.STATE_PREFIX + t
      else:
        continue
    if t not in out:
      out.append(t)
  return out


_LABELS = None


def token_label(token):
  """'Michigan' for 'us_state.MI': from StarPilot's map catalog when it imports, else the code."""
  global _LABELS
  if _LABELS is None:
    try:
      from openpilot.starpilot.common.maps_catalog import MAP_TOKEN_LABELS
      _LABELS = dict(MAP_TOKEN_LABELS)
    except Exception:
      _LABELS = {}
  return _LABELS.get(token) or token.split(".", 1)[-1]


class Regions:
  """The selected (MapsSelected) and ticked (MazdaCurveMapStates) regions placed on the map.
  placed: {token: Region (outline)}; unknown: tokens that cannot be placed; ticked: the ticked tokens (a subset of placed or unknown).
  own_everything: no token could be placed but some were given: every downloaded folder is taken to be theirs."""

  def __init__(self, selected=(), ticked=()):
    self.selected = list(selected)
    self.ticked = list(ticked)
    all_tokens = self.selected + [t for t in self.ticked if t not in self.selected]
    self.placed, self.unknown = R.place_tokens(all_tokens)
    self.own_everything = bool(all_tokens) and not self.placed

  @property
  def empty(self):
    return not self.selected and not self.ticked

  def restricts(self):
    """True when range mode must be limited to the selected regions' folders: a selection exists and at least one of
    its regions could be placed. A selection of only unplaceable regions keeps the old behaviour (everything
    downloaded within range): nothing can be told apart from it."""
    return any(t in self.placed for t in self.selected)

  def folders_of(self, folders, tokens):
    """{folder: [token...]} of the downloaded folders belonging to these tokens (placed ones; or all folders when
    nothing at all could be placed)."""
    boxes = {t: self.placed[t] for t in tokens if t in self.placed}
    if boxes:
      return R.folders_in_regions(folders, boxes)
    if self.own_everything and any(t in self.unknown for t in tokens):
      return {f: [t for t in tokens if t in self.unknown] for f in folders}
    return {}

  def ticked_placed(self):
    return [t for t in self.ticked if t in self.placed]

  def ticked_unknown(self):
    return [t for t in self.ticked if t in self.unknown]


def _names(tokens, limit=4):
  labels = [token_label(t) for t in tokens]
  return ", ".join(labels[:limit]) + (f" + {len(labels) - limit} more" if len(labels) > limit else "")


def _size(nbytes):
  return f"{nbytes / 1e9:.1f} GB" if nbytes >= 1e9 else f"{nbytes / 1e6:.0f} MB"


class Plan:
  """What one pass found: todo = [(folder, reason)] to build, nearest first; total = map folders downloaded;
  in_range = how many of them are within scope (range and ticked states; all when there is no range);
  skipped = out of scope (untouched). With regions: full = {folder: [ticked tokens]} of the ticked states' folders,
  leftovers = (count, bytes) of downloaded folders outside every selected or ticked region, missing = ticked tokens
  with no downloaded folder, unknown = tokens that could not be placed (skipped)."""

  def __init__(self, todo, total, in_range, near, range_miles, regions=None, full=None, leftovers=None, missing=(),
               unknown=(), restricted=False):
    self.todo, self.total, self.in_range = todo, total, in_range
    self.near, self.range_miles = near, range_miles
    self.skipped = total - in_range
    self.regions = regions
    self.full = dict(full or {})
    self.leftovers = leftovers
    self.missing = list(missing)
    self.unknown = list(unknown)
    self.restricted = restricted

  def ticked_labels(self):
    seen = []
    for toks in self.full.values():
      for t in toks:
        if t not in seen:
          seen.append(t)
    if self.regions is not None:
      for t in self.regions.ticked:
        if t not in seen and t not in self.missing and t not in self.unknown:
          seen.append(t)
    return [token_label(t) for t in seen]

  def where(self):
    """'within 100 mi of 42.6,-83.1' (+ ' or in Michigan, Illinois' for ticked states): the position rounded to 0.1
    degree, this may be shared in a log."""
    if self.near is None:
      s = "" if self.range_miles is None else f"within {self.range_miles:g} mi (no position yet)"
    else:
      where = f"of {self.near[0]:.1f},{self.near[1]:.1f}"
      s = f"within {self.range_miles:g} mi {where}" if self.range_miles is not None else f"nearest first {where}"
    ticked = self.ticked_labels()
    if ticked:
      s = (s + " or " if s else "") + "in " + _names(ticked)
    return s

  def skipped_why(self):
    return "out of range or outside your selected maps" if self.restricted else "out of range"

  def summary(self):
    """One short line: '3 of 12 map folders within 100 mi of 42.6,-83.1 need building (378 of 390 out of range):
    42/-84, 42/-86, ...' (at most LOG_NAMES names)."""
    names = [f for f, _ in self.todo[:LOG_NAMES]] + (["..."] if len(self.todo) > LOG_NAMES else [])
    s = f"{len(self.todo)} of {self.in_range} map folders {self.where()} need building".replace("  ", " ")
    if self.skipped:
      s += f" ({self.skipped} of {self.total} {self.skipped_why()})"
    return s + ": " + ", ".join(names)

  def folder_label(self, folder):
    """'Illinois' for a folder of a ticked state (the first one it overlaps), 'map folder 40/-90' otherwise."""
    toks = self.full.get(folder)
    return token_label(toks[0]) if toks else f"map folder {folder}"


def plan(osm_root=OSM_ROOT, curve_root=CURVE_ROOT, near=None, range_miles=None, regions=None):
  """The folders to build: every downloaded folder of a ticked state, plus those within range_miles of `near`
  (lat, lon), both limited to the selected regions when there is a selection; only the ones whose curve files are
  missing or stale; nearest first. Folders out of scope are not even checked (checking a laptop-built folder writes
  its record)."""
  folders = map_folders(osm_root)
  if regions is None or regions.empty:
    wanted = in_range(folders, near, range_miles)
    todo = [(f, r) for f in wanted for r in [check_folder(f, folders[f], curve_root)] if r is not None]
    return Plan(todo, len(folders), len(wanted), near, range_miles, regions)
  full = regions.folders_of(folders, regions.ticked)
  restricted = regions.restricts()
  if restricted:
    # range mode only builds folders of the selected regions (or of a ticked state); the rest are leftovers
    in_selection = set(regions.folders_of(folders, regions.selected)) | set(full)
    wanted = [f for f in in_range(folders, near, range_miles) if f in in_selection]
    outside = [f for f in folders if f not in in_selection]
    leftovers = (len(outside), sum(s for f in outside for s, _m in folders[f].values()))
  else:
    wanted = in_range(folders, near, range_miles)
    leftovers = None
  missing = [t for t in regions.ticked if t not in regions.unknown or regions.own_everything]
  missing = [t for t in missing if not any(t in toks for toks in full.values())]
  for f in full:
    if f not in wanted:
      wanted.append(f)
  if near is not None:
    wanted.sort(key=lambda f: (box_distance_miles(near, f), f))
  todo = []
  for folder in wanted:
    reason = check_folder(folder, folders[folder], curve_root)
    if reason is not None:
      todo.append((folder, reason))
  unknown = [] if regions.own_everything else list(regions.unknown)
  return Plan(todo, len(folders), len(wanted), near, range_miles, regions, full, leftovers, missing, unknown, restricted)


def region_status(token, folders, curve_root=CURVE_ROOT, regions=None):
  """For the UI: (downloaded folders of the region, built ones, placeable) — read-only."""
  rg = regions or Regions(selected=[token])
  mine = rg.folders_of(folders, [token])
  built = sum(1 for f in mine if folder_is_built(f, folders[f], curve_root))
  return len(mine), built, token in rg.placed or rg.own_everything


# ---------------- the build log, building ----------------

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
      status(f"{folder}: {n['files']} squares, {seconds:.0f} s")

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


def time_left_text(done, total, seconds_so_far, guess=FOLDER_SECONDS_GUESS):
  """'about 12 min left (avg 1.4 min per area)' from the folders finished this pass; before the first one is timed
  the guess per folder is used and said so."""
  left = max(0, total - done)
  if done > 0 and seconds_so_far > 0:
    per = seconds_so_far / done
    avg = f"avg {per / 60:.1f} min per area"
  else:
    per = guess
    avg = f"guessing {per / 60:.1f} min per area"
  secs = left * per
  if secs < 90:
    when = f"about {max(1, round(secs / 10) * 10)} s left" if secs >= 10 else "almost done"
  elif secs < 3600 * 1.5:
    when = f"about {round(secs / 60)} min left"
  else:
    when = f"about {secs / 3600:.1f} h left"
  return f"{when} ({avg})"


def progress_text(label, done, total, seconds_so_far, detail=""):
  """'Building area 4 of 9 (now Illinois), about 8 min left (avg 1.4 min per area)' (+ ' — <detail>'). The count is
  this build session's queue (every ticked state and the range together); the state is the one the current area is
  for, so it is never read as "Illinois: 4 of 9"."""
  s = f"Building area {done + 1} of {total} (now {label}), {time_left_text(done, total, seconds_so_far)}"
  return s + (f" - {detail}" if detail else "")


def idle_notice(p):
  """Why nothing can be built this pass, as (kind, log line, status line), or None when there is work or everything
  is up to date."""
  if p.in_range > 0:
    return None
  if p.missing:
    names = _names(p.missing)
    return ("not downloaded", f"no map tiles for {names}: download them in StarPilot's map settings",
            f"Map not downloaded for {names}: download it in StarPilot's map settings")
  if p.near is None and p.range_miles is not None:
    return ("no position", "no GPS position yet: drive once, the map folders around the car are built after that",
            "Curve files: no GPS position yet, drive once")
  if p.range_miles is not None:
    where = p.where()
    if p.total == 0:
      line = f"no offline maps downloaded ({where}): download this area in StarPilot's map settings"
    elif p.restricted:
      line = f"no downloaded maps of your selected regions {where} ({_plural(p.total, 'map folder')} elsewhere): download this area in StarPilot's map settings"
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


def region_notices(p, memo, log):
  """Log, once per situation (`memo` is kept between passes), what the regions tell: tokens that cannot be placed,
  ticked states without map tiles, downloaded folders outside every selected region (the leftovers, never
  deleted here). Returns a short status prefix for the unknown/missing ones, or ''."""
  notes = []
  if p.unknown:
    key = ("unknown", tuple(p.unknown))
    if memo is not None and memo.get("unknown") != key:
      log(f"cannot place {_names(p.unknown)} on the map (no bounding box here): skipped, its folders cannot be told from leftovers")
    if memo is not None:
      memo["unknown"] = key
    notes.append(f"can't place {_names(p.unknown, 2)}")
  elif memo is not None:
    memo.pop("unknown", None)
  if p.missing and p.in_range > 0:
    key = ("missing", tuple(p.missing))
    if memo is not None and memo.get("missing") != key:
      log(f"no map tiles for {_names(p.missing)}: download them in StarPilot's map settings")
    if memo is not None:
      memo["missing"] = key
    notes.append(f"{_names(p.missing, 2)} not downloaded")
  elif memo is not None:
    memo.pop("missing", None)
  if p.leftovers and p.leftovers[0]:
    key = ("leftovers", p.leftovers[0])
    if memo is not None and memo.get("leftovers") != key:
      n, nbytes = p.leftovers
      log(f"{_plural(n, 'map folder')} ({_size(nbytes)}) are outside your selected maps: leftovers from an older download, left alone")
    if memo is not None:
      memo["leftovers"] = key
  elif memo is not None:
    memo.pop("leftovers", None)
  return "; ".join(notes)


def run_once(osm_root=OSM_ROOT, curve_root=CURVE_ROOT, near=None, log=print, status=None, should_stop=lambda: False,
             range_miles=None, idle=None, regions=None, memo=None):
  """Build every folder in scope (ticked states in full, the rest within range) that needs it. Returns (folders
  built, folders downloaded). `idle`, a dict the caller keeps between passes, rate-limits the "nothing to do here"
  notices to once per situation (position, range) instead of once a minute; `memo` does the same for the region
  notices; the status line is refreshed every pass."""
  p = plan(osm_root, curve_root, near, range_miles, regions)
  notes = region_notices(p, memo, log)
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
    scope += f", {p.skipped} {p.skipped_why()}"
  if notes:
    scope += f"; {notes}"
  if not p.todo:
    if status is not None:
      status(f"Curve files up to date: {scope}")
    return 0, p.total
  log(p.summary())
  t0 = time.monotonic()
  built = 0
  total = len(p.todo)
  for folder, why in p.todo:
    if should_stop():
      break
    log(f"{folder}: building ({why})")
    label = p.folder_label(folder)
    if status is not None:
      status(progress_text(label, built, total, time.monotonic() - t0))

    def folder_status(detail, _label=label, _built=built):
      status(progress_text(_label, _built, total, time.monotonic() - t0, detail))
    try:
      tiles = map_folders(osm_root).get(folder)
      if not tiles:
        continue
      build_folder(folder, tiles, osm_root, curve_root, log, folder_status if status is not None else None)
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


def read_regions(params):
  """The selected (MapsSelected) and ticked (MazdaCurveMapStates) regions, read every pass."""
  def tokens(key):
    try:
      return parse_tokens(params.get(key, encoding="utf8"))
    except Exception:
      return []
  return Regions(selected=tokens(SELECTED_PARAM), ticked=tokens(TICKED_PARAM))


def main():
  from openpilot.common.params import Params
  from openpilot.common.swaglog import cloudlog
  _lowest_priority()
  params = Params()
  params_memory = Params(memory=True)
  log = BuildLog(mirror=cloudlog.info)
  idle, memo = {}, {}

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
      # position, range and the selections are read every pass: a car parked somewhere new, a GPS fix that arrived
      # while parked, a changed setting or a newly ticked state all take effect on the next pass
      run_once(near=last_position(params_memory, params), range_miles=range_miles(params), log=log, status=status,
               idle=idle, regions=read_regions(params), memo=memo)
    except Exception:
      cloudlog.exception("curve_mapd failed")
      log("failed, see the system log")
    time.sleep(CHECK_INTERVAL)


if __name__ == "__main__":
  main()
