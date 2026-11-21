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
  * Which folders: those within RANGE_MILES of the last known position (LastGPSPosition), nearest first; with no
    position yet, every folder of the selection.
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


def file_ok(path):
  """A curve file is there when it is not empty and parses (a power cut could leave a 0-byte or torn file)."""
  try:
    if os.path.getsize(path) == 0:
      return False
    with open(path) as f:
      d = json.load(f)
    return isinstance(d, dict) and d.get("version") == B.FORMAT_VERSION
  except (OSError, ValueError):
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


def plan(folders, curve_root=CURVE_ROOT, near=None, range_miles=RANGE_MILES, failed=()):
  """[(folder, why)] to build, nearest first. folders: selected_folders()."""
  todo = []
  for folder, info in folders.items():
    if folder in failed:
      continue
    dist = folder_box_distance_miles(near, folder) if near is not None else 0.0
    if near is not None and dist > range_miles:
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
             builder=B.build):
  """One pass over the folders in scope. failed: {folder: tries} kept by the caller for the parked session."""
  failed = {} if failed is None else failed
  todo = plan(folders, curve_root, near, failed={f for f, n in failed.items() if n >= 2})
  done = 0
  for folder, why in todo:
    if should_stop():
      break
    log(f"building {folder} ({why})")
    for attempt in (1, 2):
      try:
        build_folder(folder, folders[folder], curve_root, log, builder)
        failed.pop(folder, None)
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


def parse_position(raw):
  """(lat, lon) from LastGPSPosition (7.0: an Event with gpsLocationExternal; older: JSON), or None."""
  if not raw:
    return None
  lat = lon = None
  try:
    from openpilot.cereal import log as capnp_log
    with capnp_log.Event.from_bytes(raw) as ev:
      g = ev.gpsLocationExternal
      if not g.hasFix:
        return None
      lat, lon = float(g.latitude), float(g.longitude)
  except Exception:
    try:
      g = json.loads(raw)
      lat, lon = float(g["latitude"]), float(g["longitude"])
    except (TypeError, ValueError, KeyError):
      return None
  if not (math.isfinite(lat) and math.isfinite(lon)) or (lat == 0.0 and lon == 0.0) or abs(lat) > 90 or abs(lon) > 180:
    return None
  return lat, lon


def wanted(params, CP=None):
  """Built only for a Mazda with the master switch on and an option that needs the map view (map curves, speed
  limit, merge or exits) Test or On: the same rule processes.py starts this daemon with (config.needs_view)."""
  from openpilot.starpilot.car.mazda.speed import config as C
  try:
    if CP is not None and CP.brand != "mazda":
      return False
    return C.read(params).needs_view
  except Exception:
    return False


def _lowest_priority():
  try:
    os.nice(19)
  except OSError:
    pass


def main():
  from openpilot.common.params import Params
  from openpilot.common.swaglog import cloudlog
  from openpilot.starpilot.maps.storage import offline_root
  _lowest_priority()
  params = Params()
  log = BuildLog(mirror=cloudlog.info)
  failed = {}
  while True:
    if not wanted(params):
      time.sleep(IDLE_INTERVAL)
      continue
    try:
      folders = selected_folders(offline_root())
      if folders is None:
        time.sleep(IDLE_INTERVAL)
        continue
      run_once(folders, near=parse_position(params.get("LastGPSPosition")), log=log, failed=failed)
    except Exception:
      cloudlog.exception("mazda_curve_mapd failed")
      log("failed, see the system log")
    time.sleep(CHECK_INTERVAL)


if __name__ == "__main__":
  main()
