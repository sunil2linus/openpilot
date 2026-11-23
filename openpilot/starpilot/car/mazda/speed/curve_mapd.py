#!/usr/bin/env python3
"""
mazda_curve_mapd: builds the Mazda curve files on the device while the car is parked (ported from the old fork's
starpilot/system/curve_mapd.py, pointed at 7.0's offline maps).

  * Runs only offroad (the manager starts it parked and stops it when a drive starts), only on a Mazda with the speed
    control master switch on and map curves Test or On; lowest priority (nice 19).
  * Source: 7.0's SELECTED offline map generation (<offline root>/current.json -> generations/<id>/, its receipt lists
    every tile with its SHA-256). That selection is the owner's map choice in 7.0's settings, so there is no separate
    region list. A 2 x 2 degree folder is one unit of work (its roads join across its squares).
  * Output: /data/media/0/mazda_curve_map/<lat>/<lon>/<square>.json + build_state.meta (which tiles, by which
    builder/format). A folder is rebuilt when: it has no state, the state or a listed file is missing, empty or not
    JSON, its tiles changed (their SHA-256 in the receipt), or it was built by an older BUILDER_VERSION or
    FORMAT_VERSION (curve_builder.py: bumped on every geometry change). Files are written with fsync and the state
    last, so a folder killed half way is simply built again.
  * Which folders: those within the "Curve map build range" (config curve_range_mi: 30 / 50 / 100 / 200 / 300 miles,
    default 100 = RANGE_MILES, or the whole map) of the last known position (LastGPSPosition from either GPS service,
    else 7.0's navigation last-position store: on a comma 3X / tizi only ubloxd writes LastGPSPosition, and those
    devices run qcomgpsd), nearest first; the whole map or no position yet: every folder of the selection (nearest
    first when there is a position). The range is read again on every pass (every CHECK_INTERVAL while parked) and
    before each folder: a change stops the pass after the folder being built, so it applies without a reboot.
  * Status: <mazda_speed>/curve_build_status.json (status.py: "within 100 miles 3 of 5 built; all downloaded 3 of 33",
    "building", "no offline map downloaded"), read by the settings rows; counts and the range only, never a position.
    Folders out of range are counted as built from their state alone (no curve file is opened for them). A folder
    built by an older builder / format the car still reads (curve_builder.READ_FORMATS / READ_MIN_BUILDER: format 5,
    before the tagged limits) is rebuilt like any stale one, nearest first, and counted as "older, still in use until
    rebuilt" meanwhile (its curves, ramps and shapes keep working; only the map's own speed limit is missing there).
  * A MemoryError on a dense folder: gc.collect() and one retry; a second failure skips that folder for the rest of
    this parked session (logged) instead of retrying it forever. gc.collect() between folders.
  * Log: <curve root>/build_log.txt (size capped), never a GPS position.
Time: the old builder took 1.5-2 minutes per folder on the device whatever its density (lower 48: 390 folders, 6-10
hours); the spline adds work per chain, so expect roughly 2-3 minutes per folder until timed on the device.

The planning parts are pure (tests/test_curve_mapd.py); main() has the openpilot imports.
"""
import gc
import hashlib
import json
import math
import os
import re
import shutil
import time

from openpilot.starpilot.car.mazda.speed import curve_builder as B

CURVE_ROOT = "/data/media/0/mazda_curve_map"
STATE_NAME = "build_state.meta"        # not .json: the car side reads every *.json in a folder as a square
LOG_NAME = "build_log.txt"
LOG_MAX_BYTES = 300_000
RANGE_MILES = 100.0
CHECK_INTERVAL = 60.0                  # s between two looks while parked
IDLE_INTERVAL = 300.0                  # s while it is not for this car / setting
MILE_M = 1609.344
DEG_M = 111_195.0


# ---------------- the selected generation ----------------

def selected_folders(offline_root):
  """{"<lat>/<lon>": {"dir": path, "tiles": {name: sha256}}} of the selected generation, or None (nothing selected
  or the selection does not check out: 7.0's own receipt rules, maps/shadow_lifecycle.py)."""
  from pathlib import Path
  from openpilot.starpilot.maps.shadow_lifecycle import _selected_receipt
  sel = _selected_receipt(Path(offline_root))
  if sel is None:
    return None
  generation, receipt, _ = sel
  base = os.path.join(str(offline_root), "generations", generation)
  out = {}
  for t in receipt["tiles"]:
    parts = t["path"].split("/")
    if len(parts) != 3:
      continue
    folder = parts[0] + "/" + parts[1]
    if B.folder_box(folder) is None:
      continue
    d = out.setdefault(folder, {"dir": os.path.join(base, parts[0], parts[1]), "tiles": {}})
    d["tiles"][parts[2]] = t["sha256"]
  return out


def signature(tiles):
  h = hashlib.sha256()
  for name in sorted(tiles):
    h.update(f"{name}={tiles[name]}\n".encode())
  return h.hexdigest()


# ---------------- folder state ----------------

def write_json_atomic(path, data):
  B.write_json_atomic(path, data)


def read_state(curve_dir):
  try:
    with open(os.path.join(curve_dir, STATE_NAME)) as f:
      s = json.load(f)
    return s if isinstance(s, dict) else None
  except (OSError, ValueError):
    return None


_FILE_OK = {}     # path -> (mtime_ns, size, format) of a file that parsed: the idle look every minute only stats it


def file_ok(path):
  """A curve file is there when it is not empty and parses (a power cut could leave a 0-byte or torn file).
  A file that parsed is not read again while its time and size stay the same (in this process): with the whole map
  built, the look every CHECK_INTERVAL would otherwise parse every curve file of every map area."""
  try:
    st = os.stat(path)
    if st.st_size == 0:
      return False
    key = (st.st_mtime_ns, st.st_size, B.FORMAT_VERSION)
    if _FILE_OK.get(path) == key:
      return True
    with open(path) as f:
      d = json.load(f)
    ok = isinstance(d, dict) and d.get("version") == B.FORMAT_VERSION
    if ok:
      _FILE_OK[path] = key
    else:
      _FILE_OK.pop(path, None)
    return ok
  except (OSError, ValueError):
    _FILE_OK.pop(path, None)
    return False


def check_folder(folder, tiles, curve_root=CURVE_ROOT):
  """None when the folder's curve files are up to date, else why it must be built."""
  d = os.path.join(curve_root, folder)
  state = read_state(d)
  if state is None:
    return "no state" if not os.path.exists(os.path.join(d, STATE_NAME)) else "state unreadable"
  if int(state.get("format", 0)) < B.FORMAT_VERSION:
    return f"format {state.get('format')}"
  if int(state.get("builder", 0)) < B.BUILDER_VERSION:
    return f"builder {state.get('builder')}"
  if state.get("signature") != signature(tiles):
    return "map tiles changed"
  for name in state.get("files", []):
    if not file_ok(os.path.join(d, name + ".json")):
      return f"file {name} missing or broken"
  return None


def folder_box_distance_miles(near, folder):
  """Miles from (lat, lon) to the nearest point of the folder's 2 x 2 degree box (0 inside)."""
  la0, lo0 = B.folder_box(folder)
  lat = min(max(near[0], la0), la0 + 2)
  lon = min(max(near[1], lo0), lo0 + 2)
  dy = (near[0] - lat) * DEG_M
  dx = (near[1] - lon) * DEG_M * math.cos(math.radians(near[0]))
  return math.hypot(dx, dy) / MILE_M


def state_current(folder, tiles, curve_root=CURVE_ROOT):
  """Built by the current builder / format for these tiles, from the folder's state alone (the light check used to
  count folders out of range: their curve files are not opened every pass)."""
  try:
    state = read_state(os.path.join(curve_root, folder))
    return (state is not None and int(state.get("format", 0)) >= B.FORMAT_VERSION and
            int(state.get("builder", 0)) >= B.BUILDER_VERSION and state.get("signature") == signature(tiles))
  except (TypeError, ValueError):
    return False


def older_in_use(folder, tiles, curve_root=CURVE_ROOT):
  """Built for these tiles by an older builder / format that the car still reads (format 5 before the tagged limits):
  it is rebuilt, and works as before until then. From the state alone."""
  try:
    state = read_state(os.path.join(curve_root, folder))
    return (state is not None and int(state.get("format", 0)) in B.READ_FORMATS and
            int(state.get("builder", 0)) >= B.READ_MIN_BUILDER and state.get("signature") == signature(tiles) and
            not state_current(folder, tiles, curve_root))
  except (TypeError, ValueError):
    return False


def in_scope(near, folder, range_miles):
  """The folder is built: no position or no range (whole map), or its box within range_miles."""
  return near is None or range_miles is None or folder_box_distance_miles(near, folder) <= range_miles


def plan(folders, curve_root=CURVE_ROOT, near=None, range_miles=RANGE_MILES, failed=()):
  """[(folder, why)] to build, nearest first. folders: selected_folders(). range_miles None: the whole map."""
  todo = []
  for folder, info in folders.items():
    if folder in failed:
      continue
    dist = folder_box_distance_miles(near, folder) if near is not None else 0.0
    if not in_scope(near, folder, range_miles):
      continue
    why = check_folder(folder, info["tiles"], curve_root)
    if why is not None:
      todo.append((dist, folder, why))
  return [(f, w) for _, f, w in sorted(todo)]


def build_folder(folder, info, curve_root=CURVE_ROOT, log=print, builder=B.build):
  """Build one folder into its curve folder; the state is written last. Squares the new build did not write are
  removed (old files would otherwise stay loaded)."""
  out = os.path.join(curve_root, folder)
  os.makedirs(out, exist_ok=True)
  state_path = os.path.join(out, STATE_NAME)
  if os.path.exists(state_path):
    os.remove(state_path)                  # a half-built folder has no state: built again next time
  t0 = time.monotonic()
  stats = builder(info["dir"], out, quiet=True, box=B.folder_box(folder),
                  extra={"built": round(time.time(), 1)})  # noqa: TID251
  written = set(stats["files"])
  for name in os.listdir(out):
    if name.endswith(".json") and name[:-5] not in written:
      try:
        os.remove(os.path.join(out, name))
      except OSError:
        pass
  write_json_atomic(state_path, {"format": B.FORMAT_VERSION, "builder": B.BUILDER_VERSION,
                                 "signature": signature(info["tiles"]), "files": sorted(written),
                                 "curves": stats["curves"], "ramps": stats["ramps"],
                                 "seconds": round(time.monotonic() - t0, 1)})
  log(f"built {folder}: {stats['curves']} curves, {stats['ramps']} ramps, {len(written)} files, " +
      f"{stats.get('outside', 0)} squares left to their own folder, {time.monotonic() - t0:.0f} s")
  return stats


def run_once(folders, curve_root=CURVE_ROOT, near=None, log=print, should_stop=lambda: False, failed=None,
             builder=B.build, report=None, range_miles=RANGE_MILES):
  """One pass over the folders in scope (range_miles None: the whole map). failed: {folder: tries} kept by the
  caller for the parked session. report(status dict): told before each folder and after the pass (the settings
  rows' status file)."""
  failed = {} if failed is None else failed
  skip = {f for f, n in failed.items() if n >= 2}
  stale = plan(folders, curve_root, near, range_miles)    # every folder in scope that is not up to date
  todo = [(f, w) for f, w in stale if f not in skip]
  scope = [f for f in folders if in_scope(near, f, range_miles)]
  total = len(scope)
  built = total - len(stale)
  scope = set(scope)
  outside = sum(state_current(f, folders[f]["tiles"], curve_root) for f in folders if f not in scope)
  shown_range = range_miles if (near is not None and range_miles is not None) else None   # None: all in scope
  older = {f for f, _w in stale if older_in_use(f, folders[f]["tiles"], curve_root)}

  def status(n, **kw):
    return build_status(n, total, range_miles=shown_range, all_built=n + outside, all_total=len(folders),
                        older=len(older - built_now), **kw)

  done = 0
  built_now = set()
  for folder, why in todo:
    if should_stop():
      break
    if report is not None:
      report(status(built + done, building=True, failed=len(stale) - len(todo)))
    log(f"building {folder} ({why})")
    for attempt in (1, 2):
      try:
        build_folder(folder, folders[folder], curve_root, log, builder)
        failed.pop(folder, None)
        built_now.add(folder)
        done += 1
        break
      except MemoryError:
        gc.collect()
        failed[folder] = failed.get(folder, 0) + 1
        log(f"{folder}: out of memory (try {attempt})" + ("; skipped for this parked session" if attempt == 2 else ""))
      except FileNotFoundError as e:
        failed[folder] = 2
        log(f"{folder}: {e}")
        break
    gc.collect()
  if report is not None:
    report(status(built + done, failed=sum(f in failed and failed[f] >= 2 for f, _ in stale)))
  return done


class BuildLog:
  def __init__(self, curve_root=CURVE_ROOT, mirror=None):
    self.path = os.path.join(curve_root, LOG_NAME)
    self.mirror = mirror

  def __call__(self, text):
    line = time.strftime("%Y-%m-%d %H:%M:%S ") + re.sub(r"\s+", " ", str(text)) + "\n"
    try:
      os.makedirs(os.path.dirname(self.path), exist_ok=True)
      if os.path.exists(self.path) and os.path.getsize(self.path) > LOG_MAX_BYTES:
        shutil.move(self.path, self.path + ".1")
      with open(self.path, "a") as f:
        f.write(line)
    except OSError:
      pass
    if self.mirror is not None:
      try:
        self.mirror(line.strip())
      except Exception:
        pass


GPS_EVENTS = ("gpsLocationExternal", "gpsLocation")


def _position(lat, lon):
  lat, lon = float(lat), float(lon)
  if not (math.isfinite(lat) and math.isfinite(lon)) or (lat == 0.0 and lon == 0.0) or abs(lat) > 90 or abs(lon) > 180:
    return None
  return lat, lon


def parse_position(raw):
  """(lat, lon) from LastGPSPosition, or None. 7.0's ubloxd stores an Event holding gpsLocationExternal; an Event
  holding gpsLocation (qcomgpsd's service) is read the same way; older versions stored JSON."""
  if not raw:
    return None
  try:
    from openpilot.cereal import log as capnp_log
    with capnp_log.Event.from_bytes(raw) as ev:
      which = ev.which()
      if which not in GPS_EVENTS:
        return None
      g = getattr(ev, which)
      if not g.hasFix:
        return None
      return _position(g.latitude, g.longitude)
  except AttributeError as e:
    from openpilot.starpilot.car.mazda.speed.events import schema_miss
    schema_miss(None, "LastGPSPosition gpsLocation(External).hasFix/latitude/longitude", e)
    return None
  except Exception:                # not a capnp Event: the older JSON form
    try:
      g = json.loads(raw)
      return _position(g["latitude"], g["longitude"])
    except (TypeError, ValueError, KeyError):
      return None


def navigation_position(root=None):
  """(lat, lon) from 7.0's own last known position (starpilot/navigation/position.LastPositionStore, written by
  navigationd from either GPS service), or None. On a comma 3X / tizi (qcomgpsd) nothing writes LastGPSPosition:
  only ubloxd does."""
  try:
    from pathlib import Path
    from openpilot.starpilot.navigation.position import LastPositionStore
    if root is None:
      from openpilot.starpilot.storage import starpilot_storage_root
      root = starpilot_storage_root() / "navigation"
    saved = LastPositionStore(Path(root)).read()
    return None if saved is None else _position(saved["latitude"], saved["longitude"])
  except Exception:
    return None


def last_position(params, nav_root=None):
  """The last known position for the nearest-first order: LastGPSPosition (either service), else 7.0's navigation
  store. None: every folder of the selection is built."""
  try:
    near = parse_position(params.get("LastGPSPosition"))
  except Exception:
    near = None
  return near if near is not None else navigation_position(nav_root)


def wanted(params, CP=None):
  """Built only for a Mazda with the master switch on and an option that needs the map view (map curves, speed
  limit, merge or exits) Test or On: the same rule processes.py starts this daemon with (config.needs_view)."""
  from openpilot.starpilot.car.mazda.speed import config as C
  from openpilot.starpilot.car.mazda.speed import processes
  try:
    if CP is not None and processes.car_brand(params, CP)[0] != "mazda":
      return False
    return C.read(params).needs_view
  except Exception:
    return False


def build_range(params):
  """The configured "Curve map build range" in miles, or None for the whole map (config.curve_range_mi)."""
  from openpilot.starpilot.car.mazda.speed import config as C
  return C.read(params).curve_range_miles


def build_status(built, total, building=False, failed=0, range_miles=None, all_built=None, all_total=None, older=0):
  """The status file's dict (status.build_text says it in words); counts and the range only, never a position.
  built / total: the folders in scope; range: its miles (0: everything is in scope: the whole map or no position);
  all_built / all_total: every folder of the selected map; older: folders in scope not built yet that still work
  from an older format (older_in_use)."""
  from openpilot.starpilot.car.mazda.speed import status as ST
  if total is None:
    return {"state": ST.NO_MAP, "built": 0, "total": 0, "failed": 0}
  state = ST.UP_TO_DATE if built >= total else ST.BUILDING if building else ST.PARTIAL
  out = {"state": state, "built": int(built), "total": int(total), "failed": int(failed)}
  if all_total is not None:
    out.update({"range": int(range_miles or 0), "all_built": int(all_built), "all_total": int(all_total)})
  if older and state != ST.UP_TO_DATE:
    out["older"] = int(older)
  return out


def _lowest_priority():
  try:
    os.nice(19)
  except OSError:
    pass


def main():
  from openpilot.common.params import Params
  from openpilot.common.swaglog import cloudlog
  from openpilot.starpilot.maps.storage import offline_root
  from openpilot.starpilot.car.mazda.speed import status as ST
  _lowest_priority()
  params = Params()
  log = BuildLog(mirror=cloudlog.info)
  failed = {}
  last = [None]

  def report(value):
    if value != last[0] and ST.write(ST.BUILD_NAME, value):
      last[0] = value

  while True:
    if not wanted(params):
      time.sleep(IDLE_INTERVAL)
      continue
    try:
      folders = selected_folders(offline_root())
      if folders is None:
        report(build_status(0, None))
        time.sleep(IDLE_INTERVAL)
        continue
      rng = build_range(params)                           # read every pass: a change needs no reboot
      run_once(folders, near=last_position(params), log=log, failed=failed, report=report, range_miles=rng,
               should_stop=lambda rng=rng: build_range(params) != rng)
    except Exception:
      cloudlog.exception("mazda_curve_mapd failed")
      log("failed, see the system log")
    time.sleep(CHECK_INTERVAL)


if __name__ == "__main__":
  main()
