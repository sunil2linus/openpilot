"""The parked curve builder daemon: which folders it builds (older builder/format, changed tiles, broken files),
MemoryError back-off, the selected 7.0 generation as its source, and a real build from a synthetic tile."""
import hashlib
import json
import math
import os

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
  assert D.check_folder(FOLDER, {k: "cd" * 32 for k in TILES}, root) == "map tiles changed"
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
