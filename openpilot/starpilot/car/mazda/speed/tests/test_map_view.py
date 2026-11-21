"""The onroad map view: curve files -> matched road -> profile points ahead (distances, direction, one-way rule),
old-format files ignored, GPS trust, and the view file protocol."""
import math
import os

from openpilot.starpilot.car.mazda.speed import curve_builder as B
from openpilot.starpilot.car.mazda.speed import map_match as MM
from openpilot.starpilot.car.mazda.speed import map_view as V
from openpilot.starpilot.car.mazda.speed.tests.test_curve_builder import LAT0, LON0, M_LAT, M_LON, way, straight, arc_m


def write_folder(root, ways, version=B.FORMAT_VERSION, builder=B.BUILDER_VERSION):
  chains = B.build_chains(ways)
  curves, recs = [], {}
  for ch in chains:
    found = B.find_curves(ch, ways)
    curves += found
    if found:
      rec = B.chain_record(ch)
      recs[rec["id"]] = rec
  chain_of = {w.id: int(ch[0][0].id) for ch in chains for w, _ in ch}
  shapes = B.road_shapes(ways, chain_of)
  d = os.path.join(root, "42", "-84")
  os.makedirs(d, exist_ok=True)
  for name, roads in shapes.items():
    cs = [c for c in curves if B.tile_of(*c["apex"]) == name]
    B.write_json_atomic(os.path.join(d, name + ".json"), {"version": version, "builder": builder, "curves": cs,
                        "ramps": [], "chains": list(recs.values()), "roads": roads})
  return curves


def loop_road():
  """North 400 m, a left 80 m radius curve of 90 deg, then west 400 m."""
  a = straight(0, -400, 0, 400, step=10)
  c = arc_m(-80, 0, 80, 0, 90, 5.0)
  ex, ey = c[-1]
  b = [(ex - 10 * i, ey) for i in range(1, 41)]
  return a + c[1:] + b


def gps_at(x, y, bearing, speed=20.0):
  return {"lat": LAT0 + y * M_LAT, "lon": LON0 + x * M_LON, "bearing": bearing, "speed": speed, "acc": 5.0, "fix": True}


def make_view(tmp_path, ways, **kw):
  write_folder(str(tmp_path), ways, **kw)
  store = MM.MapCurveStore(root=str(tmp_path), background=False)
  return V.MapView(store), store


def test_curve_ahead_distance_and_profile(tmp_path):
  mv, _ = make_view(tmp_path, {1: way(1, loop_road(), speed_mph=45)})
  out = None
  for k, y in enumerate(range(-380, -150, 2)):
    out = mv.update(k * 0.1, gps_at(0.0, float(y), 0.0))
  assert out["gps"] == "ok" and out["matched"] and len(out["curves"]) == 1
  c = out["curves"][0]
  d_entry = c["pts"][0][0]
  assert 130 <= d_entry <= 160                        # entry near y = 0 minus the spline's run-in; car at y = -152
  assert all(b[0] > a[0] for a, b in zip(c["pts"], c["pts"][1:], strict=False))
  assert abs(min(r for _, r in c["pts"]) - 80) / 80 < 0.12
  assert abs(out["t"] - (len(range(-380, -150, 2)) - 1) * 0.1 + V.GPS_LAG) < 1e-9


def test_inside_the_curve_points_go_negative_and_the_other_direction_mirrors(tmp_path):
  mv, _ = make_view(tmp_path, {1: way(1, loop_road(), speed_mph=45)})
  t = 0.0
  for y in range(-200, 0, 2):
    t += 0.1
    out = mv.update(t, gps_at(0.0, float(y), 0.0))
  for a in range(5, 40, 2):                           # into the curve
    t += 0.1
    x, y = -80 + 80 * math.cos(math.radians(a)), 80 * math.sin(math.radians(a))
    out = mv.update(t, gps_at(x, y, (360 - a) % 360))
  c = out["curves"][0]
  assert c["pts"][0][0] < 0 < c["pts"][-1][0]
  # the other way round (westbound end -> south): the same curve, its profile mirrored
  mv2, _ = make_view(tmp_path, {1: way(1, loop_road(), speed_mph=45)})
  t = 0.0
  for x in range(-480, -150, 3):
    t += 0.1
    out2 = mv2.update(t, gps_at(float(x), 80.0, 90.0))
  c2 = out2["curves"][0]
  assert c2["id"].endswith(":ba") and c["id"].endswith(":ab") and c2["id"].split(":")[:2] == c["id"].split(":")[:2]


def test_old_format_files_are_ignored_until_rebuilt(tmp_path):
  mv, store = make_view(tmp_path, {1: way(1, loop_road(), speed_mph=45)}, version=4, builder=1)
  out = None
  for k, y in enumerate(range(-380, -150, 2)):
    out = mv.update(k * 0.1, gps_at(0.0, float(y), 0.0))
  assert out["curves"] == [] and not out["matched"] and store.stale_files > 0


def test_never_a_one_way_backwards(tmp_path):
  # two ways of one chain: the second (with the curve) is one-way against our direction
  pts = loop_road()
  ways = {1: way(1, pts[:30], speed_mph=45), 2: way(2, list(reversed(pts[29:])), speed_mph=45, oneway=True)}
  mv, _ = make_view(tmp_path, ways)
  out = None
  for k, y in enumerate(range(-380, -150, 2)):
    out = mv.update(k * 0.1, gps_at(0.0, float(y), 0.0))
  assert out["matched"] and out["curves"] == []
  ways[2].oneway = False                              # the same road two-way: the curve is ours
  mv, _ = make_view(tmp_path / "two", ways)
  for k, y in enumerate(range(-380, -150, 2)):
    out = mv.update(k * 0.1, gps_at(0.0, float(y), 0.0))
  assert len(out["curves"]) == 1


def test_no_fix_and_far_off_gps(tmp_path):
  mv, _ = make_view(tmp_path, {1: way(1, loop_road(), speed_mph=45)})
  out = mv.update(0.0, None)
  assert out["gps"] == "none" and out["trust"] == MM.TRUST_NO_GPS and out["curves"] == []
  t = 0.0
  for _ in range(int(MM.LOST_TIME / 0.1) + 5):          # 300 m beside the road at speed: lost, paused
    t += 0.1
    out = mv.update(t, gps_at(300.0, -100.0, 0.0, speed=20.0))
  assert out["trust"] == MM.TRUST_PAUSED_LOST and out["curves"] == []


def test_view_file_roundtrip(tmp_path):
  p = str(tmp_path / "view.json")
  assert V.read_view(p) is None
  V.write_view({"v": 1, "t": 5.0, "gps": "none", "curves": []}, p)
  assert V.read_view(p)["t"] == 5.0
  V.write_view({"v": 2, "t": 5.0}, p)
  assert V.read_view(p) is None
  with open(p, "w") as f:
    f.write("{")
  assert V.read_view(p) is None
