"""Two tiny status files the settings rows read (no Params key): how far the parked curve-map builder got
(mazda_curve_mapd) and whether the onroad map view had a GPS position on the last drive (mazda_mapviewd).
Both live in the event-log folder, are written atomically, never hold a position, and are read with a size cap.
Never raises."""
import json
import os
import time

STATUS_DIR = "/data/media/0/mazda_speed"
BUILD_NAME = "curve_build_status.json"
GPS_NAME = "gps_status.json"
MAX_BYTES = 4096

# curve build states
NO_MAP, BUILDING, UP_TO_DATE, PARTIAL = "no_map", "building", "up_to_date", "partial"
# GPS states (the view's "why" when there is no position)
GPS_OK, GPS_NO_MESSAGES, GPS_NO_FIX, GPS_STALE = "ok", "no_messages", "no_fix", "stale"


def _path(name, directory=None):
  return os.path.join(directory or STATUS_DIR, name)


def write(name, value, directory=None):
  path = _path(name, directory)
  try:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
      json.dump(dict(value, wall=round(time.time(), 1)), f, separators=(",", ":"))  # noqa: TID251
    os.replace(tmp, path)
    return True
  except (OSError, TypeError, ValueError):
    return False


def read(name, directory=None):
  try:
    with open(_path(name, directory), "rb") as f:
      raw = f.read(MAX_BYTES + 1)
    if len(raw) > MAX_BYTES:
      return None
    value = json.loads(raw)
    return value if isinstance(value, dict) else None
  except (OSError, ValueError):
    return None


def build_text(value):
  """The curve-map builder's state in plain words, or "" when it has not written one."""
  if not value:
    return ""
  state = value.get("state")
  try:
    built, total = int(value.get("built", 0)), int(value.get("total", 0))
  except (TypeError, ValueError):
    built, total = 0, 0
  if state == NO_MAP:
    return "curve maps: no offline map downloaded (download one in Maps settings)."
  if state == UP_TO_DATE:
    return f"curve maps: up to date ({total} of {total} folders built)." if total else "curve maps: up to date."
  if state == BUILDING:
    return f"curve maps: building… {built} of {total} folders built (parked, on power)."
  if state == PARTIAL:
    failed = int(value.get("failed", 0) or 0)
    return (f"curve maps: {built} of {total} folders built" + (f", {failed} failed (see build_log.txt)" if failed else "") +
            "; the rest are built while parked.")
  return ""


GPS_WORDS = {GPS_NO_MESSAGES: "no GPS position: no message from the GPS (gpsLocation / gpsLocationExternal)",
             GPS_NO_FIX: "no GPS position: the GPS has no fix", GPS_STALE: "no GPS position: the GPS messages stopped"}


def gps_text(value):
  """The last drive's GPS state for the map features, or "" when it had a position (or nothing was written).
  mazda_mapviewd writes "ok" once the drive has had a position; until then the reason it has none."""
  if not value:
    return ""
  state = value.get("state")
  if state in GPS_WORDS:
    return "last drive: " + GPS_WORDS[state] + "; map curves, exits, merges and speed limits need it."
  return ""
