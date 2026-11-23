"""Curve map builder: the old fork's tests (ported), the spline acceptance shapes, distance profiles, split
doublets, the folder box, file writing and versions."""
import json
import math
import os

import numpy as np

from openpilot.starpilot.car.mazda.speed import curve_builder as B

LAT0, LON0 = 42.6, -83.0
M_LAT = 1 / 111195.0
M_LON = 1 / (111195.0 * math.cos(math.radians(LAT0)))


def way(wid, pts_m, name="Test Road", cls=7, speed_mph=45, adv=0.0, oneway=False):
  w = B.Way()
  w.id, w.name, w.ref, w.cls = wid, name, "", cls
  w.speed, w.advisory, w.oneway = speed_mph * B.MPH, adv, oneway
  w.lat = np.array([LAT0 + y * M_LAT for x, y in pts_m])
  w.lon = np.array([LON0 + x * M_LON for x, y in pts_m])
  w.start, w.end = B.key(w.lat[0], w.lon[0]), B.key(w.lat[-1], w.lon[-1])
  return w


def straight(x0, y0, heading_deg, length, step=20.0):
  h = math.radians(heading_deg)
  n = int(length // step)
  return [(x0 + math.sin(h) * step * i, y0 + math.cos(h) * step * i) for i in range(n + 1)]


def arc(cx, cy, r, a0, a1, step_deg=4.0):
  n = max(2, int(abs(a1 - a0) / step_deg))
  return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)), cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n)))
          for i in range(n + 1)]


def curve_road(r=115.0, turn=40.0, split=False, name="Test Road", cls=7, speed=45):
  """Straight north 300 m, a left curve of radius r turning `turn` degrees, straight 300 m."""
  a = straight(0, -300, 0, 300)
  c = arc(-r, 0, r, 0, turn)
  ex, ey = c[-1]
  hd = -turn
  b = straight(ex, ey, hd, 300)[1:]
  pts = a + c[1:] + b
  if not split:
    return {1: way(1, pts, name, cls, speed)}
  mid = len(a) + len(c) // 2
  return {1: way(1, pts[:mid + 1], name, cls, speed), 2: way(2, pts[mid:], name, cls, speed)}


def curves_of(ways):
  out = []
  for ch in B.build_chains(ways):
    out.extend(B.find_curves(ch, ways))
  return out


def test_radius_of_a_clean_arc():
  for r in (60.0, 115.0, 250.0):
    cs = curves_of(curve_road(r=r, turn=60))
    assert len(cs) == 1, (r, cs)
    assert abs(cs[0]["r"] - r) / r < 0.12, (r, cs[0]["r"])
    assert cs[0]["dir"] == "L" and 50 <= cs[0]["turn"] <= 70


def test_curve_split_across_two_ways_is_one_curve():
  cs = curves_of(curve_road(r=115, turn=40, split=True))
  assert len(cs) == 1 and abs(cs[0]["r"] - 115) / 115 < 0.12 and cs[0]["ways"] == [1, 2]


def test_gentle_or_tiny_bends_are_ignored():
  assert curves_of(curve_road(r=900, turn=30)) == []        # gentler than MAX_RADIUS
  assert curves_of(curve_road(r=115, turn=6)) == []         # turns too little


def test_curves_that_do_not_matter_at_the_speed_limit_are_dropped():
  # 300 m radius needs 30 m/s (67 mph) at 3.0 m/s^2: kept on a 55 mph road, dropped on a 25 mph one
  assert len(curves_of(curve_road(r=300, turn=40, speed=55))) == 1
  assert curves_of(curve_road(r=300, turn=40, speed=25)) == []


def test_turn_at_an_intersection_is_not_a_curve():
  # Road A goes north, road B (another name) leaves the same node going east: a 90 degree junction turn
  a = way(1, straight(0, -300, 0, 300), name="North Road")
  b = way(2, straight(0, 0, 90, 300), name="East Road")
  assert curves_of({1: a, 2: b}) == []
  # the same corner as ONE road that bends (same name, sharp corner) is a curve
  c = way(3, straight(0, -300, 0, 300) + arc(30, 0, 30, 180, 90)[1:] + straight(30, 30, 90, 300)[1:], name="Bend Road")
  assert len(curves_of({3: c})) == 1


def test_same_road_does_not_join_through_a_sharp_turn():
  # the same name continuing 80 degrees off at a node (e.g. a road that turns at a T-junction) is not joined
  a = way(1, straight(0, -300, 0, 300), name="Same Road")
  b = way(2, straight(0, 0, 80, 300), name="Same Road")
  chains = B.build_chains({1: a, 2: b})
  assert len(chains) == 2


def test_both_ends_and_headings():
  cs = curves_of(curve_road(r=115, turn=40))
  c = cs[0]
  # travelling a -> b we head north at the entry; travelling b -> a we enter heading south-east (turn 40 left)
  assert abs(B.angle_diff(c["a_hdg"], 0.0)) < 10, c["a_hdg"]
  assert abs(B.angle_diff(c["b_hdg"], (-40 + 180) % 360)) < 10, c["b_hdg"]
  assert c["v"]["3.0"] == round(math.sqrt(3.0 * c["r"]), 2)


def ramp_scene():
  """A motorway going north; an exit ramp leaves it at (0, 0), loops right with a 50 m radius and ends on a
  primary road; an entrance ramp leaves the primary road and joins the motorway further north."""
  mw = way(10, straight(0, -500, 0, 1500), name="", cls=1, speed_mph=70)
  mw.ref = "I 75"
  loop = straight(0, 0, 20, 100)[:-1] + arc(34 + 50, 94, 50, 180, 0)[0:] + straight(134, 94, 180, 100)[1:]
  exit_r = way(11, loop, name="", cls=2, speed_mph=45, oneway=True)
  prim = way(12, straight(134, -6, 90, 600), name="Main Street", cls=5, speed_mph=45)
  # the exit ends on the primary road's first node
  prim.lat[0], prim.lon[0] = exit_r.lat[-1], exit_r.lon[-1]
  prim.start = exit_r.end = B.key(exit_r.lat[-1], exit_r.lon[-1])
  ent = way(13, straight(400, -6, 340, 900), name="", cls=2, speed_mph=45, oneway=True)
  ent.lat[0], ent.lon[0] = prim.lat[3], prim.lon[3]
  ent.start = B.key(prim.lat[3], prim.lon[3])
  # it joins the motorway at one of its interior nodes
  k = len(mw.lat) - 10
  ent.lat[-1], ent.lon[-1] = mw.lat[k], mw.lon[k]
  ent.end = B.key(mw.lat[k], mw.lon[k])
  exit_r.lat[0], exit_r.lon[0] = mw.lat[25], mw.lon[25]
  exit_r.start = B.key(mw.lat[25], mw.lon[25])
  return {w.id: w for w in (mw, exit_r, prim, ent)}


def test_ramps_know_where_they_lead():
  ways = ramp_scene()
  nidx = B.node_index(ways)
  recs = {r["wdir"][0][0]: r for r in (B.ramp_record(ch, nidx, ways) for ch in B.build_chains(ways)) if r}
  assert recs[11]["type"] == "exit" and recs[11]["to"]["name"] == "Main Street" and recs[11]["from"]["cls"] == 1
  assert recs[13]["type"] == "entrance" and recs[13]["to"]["cls"] == 1 and recs[13]["from"]["name"] == "Main Street"
  assert recs[11]["travel"] == "ab"


def test_curve_on_an_exit_carries_the_ramp_and_distance_to_its_end():
  ways = ramp_scene()
  nidx = B.node_index(ways)
  for ch in B.build_chains(ways):
    rec = B.ramp_record(ch, nidx, ways)
    if rec and rec["wdir"][0][0] == 11:
      cs = B.find_curves(ch, ways, rec)
      assert len(cs) >= 1
      c = min(cs, key=lambda c: c["r"])                      # the loop (the bend off the motorway is wide)
      assert abs(c["r"] - 50) / 50 < 0.15, c["r"]
      assert c["ramp"]["type"] == "exit" and c["ramp"]["to"]["name"] == "Main Street"
      assert 60 <= c["ramp"]["end_dist"] <= 130, c["ramp"]["end_dist"]
      assert c["wdir"] == [[11, 1]]
      return
  raise AssertionError("exit ramp not found")


def test_chain_records_give_way_order_and_curve_position():
  ways = curve_road(r=115, turn=40, split=True)
  ch = B.build_chains(ways)[0]
  rec = B.chain_record(ch)
  assert [w[0] for w in rec["w"]] == [1, 2] and rec["w"][0][2] == 0.0 and rec["w"][1][2] > 300
  c = B.find_curves(ch, ways)[0]
  assert c["chain"] == rec["id"] and 280 <= c["s_a"] < c["s_b"] <= rec["len"]


def test_road_shapes_keep_node_order_and_decode_back():
  ways = curve_road(r=115, turn=40, split=True)
  shapes = [r for lst in B.road_shapes(ways).values() for r in lst]
  assert {r["w"] for r in shapes} == {1, 2}
  for wid, w in ways.items():
    pieces = []
    for r in shapes:
      if r["w"] != wid:
        continue
      p = r["p"]
      lat, lon = p[0], p[1]
      pts = [(lat, lon)]
      for k in range(2, len(p), 2):
        lat, lon = lat + p[k], lon + p[k + 1]
        pts.append((lat, lon))
      pieces.append(pts)
    # the way crosses the -83.0 deg square boundary: split in two squares, each piece in node order
    starts = [abs(pc[0][0] / 1e6 - w.lat[0]) < 2e-6 and abs(pc[0][1] / 1e6 - w.lon[0]) < 2e-6 for pc in pieces]
    ends = [abs(pc[-1][0] / 1e6 - w.lat[-1]) < 2e-6 and abs(pc[-1][1] / 1e6 - w.lon[-1]) < 2e-6 for pc in pieces]
    assert any(starts) and any(ends)
    pts = [q for pc in pieces for q in pc]
    assert len(set(pts)) < len(w.lat)                  # straight parts simplified
    # every original node lies on the simplified shape (within SHAPE_TOLERANCE of a segment)
    for la, lo in zip(w.lat, w.lon, strict=False):
      best = 1e9
      for pc in pieces:
        for (a1, b1), (a2, b2) in zip(pc, pc[1:], strict=False):
          ax, ay = (b1 / 1e6 - lo) / M_LON, (a1 / 1e6 - la) / M_LAT
          bx, by = (b2 / 1e6 - lo) / M_LON, (a2 / 1e6 - la) / M_LAT
          dx, dy = bx - ax, by - ay
          t = max(0.0, min(1.0, -(ax * dx + ay * dy) / max(dx * dx + dy * dy, 1e-9)))
          best = min(best, math.hypot(ax + t * dx, ay + t * dy))
      assert best <= B.SHAPE_TOLERANCE + 0.2, best


def test_simplify_keeps_corners_and_drops_straight_points():
  x = [0, 10, 20, 30, 30, 30]
  y = [0, 0, 0, 0, 10, 20]
  assert B.simplify(x, y, 2.0) == [0, 3, 5]
  assert B.simplify([0, 1], [0, 0], 2.0) == [0, 1]


def test_short_sharp_bends_are_widened_real_curves_are_not():
  # Dobry Drive: 17.4 deg over 34 m reads as 99 m; a car cuts it within its lane (measured ~370 m)
  assert B.effective_radius(98.7, 17.4) > 120
  # West Utica Road: 40 deg over 112 m, 117 m (measured 104-116 m): unchanged
  assert B.effective_radius(116.6, 39.9) == 116.6
  assert B.effective_radius(50.0, 90.0) == 50.0


# ---------------- Stage 2: spline + profiles ----------------

def arc_m(cx, cy, r, a0, a1, step_m):
  n = max(2, int(abs(math.radians(a1 - a0)) * r / step_m))
  return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)), cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n)))
          for i in range(n + 1)]


def radii_along(pts, use_spline=True):
  x, y = np.array([p[0] for p in pts]), np.array([p[1] for p in pts])
  if use_spline:
    x, y = B.spline(x, y)
  rx, ry, rs = B.resample(x, y, B.STEP)
  k = B.curvature(rx, ry, B.STEP, B.HALF_WINDOW)
  return rs, 1.0 / np.maximum(np.abs(k), 1e-6)


def pinch_loop():
  """North 300 m, a left 72 m radius entry pinch (50 deg), then a 92 m body (130 deg), out straight."""
  a = straight(0, -300, 0, 300)
  c1 = arc_m(-72, 0, 72, 0, 50, 6.0)
  ex, ey = c1[-1]
  h = math.radians(50)
  c2 = arc_m(ex - 92 * math.cos(h), ey - 92 * math.sin(h), 92, 50, 180, 6.0)
  ex, ey = c2[-1]
  return a + c1[1:] + c2[1:] + [(ex, ey - 20 * i) for i in range(1, 15)]


def test_spline_keeps_a_real_sustained_pinch():
  rs, r = radii_along(pinch_loop())
  pinch = (rs > 300) & (rs < 300 + 72 * math.radians(50))
  body = (rs > 300 + 72 * math.radians(50) + 30) & (rs < 300 + 72 * math.radians(50) + 92 * math.radians(130) - 30)
  assert abs(r[pinch].min() - 72) / 72 < 0.05
  assert abs(np.median(r[body]) - 92) / 92 < 0.05
  # and the curve's profile keeps it as its tightest point, then the wider body
  cs = curves_of({1: way(1, pinch_loop(), speed_mph=45)})
  assert len(cs) == 1
  prof = cs[0]["prof"]
  assert 3 <= len(prof) <= B.PROFILE_MAX_POINTS
  r_min = min(r for _, r in prof)
  assert abs(r_min - 72) / 72 < 0.05 and abs(cs[0]["r"] - r_min) < 1.0
  s_min = next(s for s, r in prof if r == r_min)
  assert s_min < 0.5 * cs[0]["len"]                     # the pinch is at the entry
  assert any(abs(r - 92) / 92 < 0.08 and s > s_min for s, r in prof)
  assert all(b[0] > a[0] for a, b in zip(prof, prof[1:], strict=False)) and prof[0][0] == 0.0


def test_spline_widens_a_three_node_corner_toward_its_arc():
  sp = 60.0
  a = [(0, -sp * i) for i in range(5, 0, -1)] + [(0, 0)]
  h = math.radians(20)
  pts = a + [(math.sin(h) * sp * i, math.cos(h) * sp * i) for i in range(1, 6)]
  _, r_poly = radii_along(pts, use_spline=False)
  _, r_spl = radii_along(pts)
  assert r_spl.min() > 1.25 * r_poly.min()


def test_spline_stops_a_polygonal_loop_from_beating():
  r0, side = 80.0, 22.0
  dth = 2 * math.asin(side / 2 / r0)
  n = int(math.radians(270) / dth)
  pts = straight(0, -300, 0, 300) + [(-r0 + r0 * math.cos(i * dth), r0 * math.sin(i * dth)) for i in range(1, n + 1)]
  for use, limit in ((False, None), (True, 0.06)):
    rs, r = radii_along(pts, use_spline=use)
    body = (rs > 340) & (rs < 300 + r0 * math.radians(270) - 40)
    spread = (r[body].max() - r[body].min()) / r0
    if use:
      assert spread < limit, spread
    else:
      assert spread > 0.1, spread                       # the polygon alone beats


def test_profile_is_simplified_and_bounded():
  s = np.arange(0, 400, 2.0)
  k = 1 / 300 + (1 / 60 - 1 / 300) * np.exp(-((s - 120) / 40) ** 2) + 0.0002 * np.sin(s / 3)
  prof = B.profile(s, k)
  assert 3 <= len(prof) <= B.PROFILE_MAX_POINTS
  assert min(r for _, r in prof) == round(1 / k.max(), 1)
  assert prof[0][0] == 0.0 and prof[-1][0] == s[-1]
  # every original point is within the tolerance (or the bound forced a coarser one)
  ps, pc = np.array([p[0] for p in prof]), 1 / np.array([p[1] for p in prof])
  err = np.abs(np.interp(s, ps, pc) - k)
  assert err.max() < 4 * max(B.PROFILE_TOL, B.PROFILE_REL * k.max())
  # a short sharp bend's floor (effective_radius) holds on every point
  assert min(r for _, r in B.profile(s, k, r_floor=150.0)) >= 150.0


def test_split_doublets_drop_a_carriageway_jog_keep_a_real_s_bend():
  def jog(bend, off_name="Rochester Rd", oneway_second=True):
    a = straight(0, -400, 0, 400, step=10)
    c1 = arc_m(-150, 0, 150, 0, bend, 3.0)
    ex, ey = c1[-1]
    h = math.radians(bend)
    cx, cy = ex + 150 * math.cos(h), ey + 150 * math.sin(h)
    c2 = arc_m(cx, cy, 150, 180 + bend, 180, 3.0)
    ex, ey = c2[-1]
    b = straight(ex, ey, 0, 400, step=10)
    first = a + c1[1:]
    second = c1[-1:] + c2[1:] + b[1:]
    return {1: way(1, first, name="Rochester Rd", speed_mph=45),
            2: way(2, second, name=off_name, speed_mph=45, oneway=oneway_second)}
  assert curves_of(jog(14)) == []                       # ~14 deg each way, ~7 m sideways: the split jog
  assert len(curves_of(jog(14, oneway_second=False))) == 2   # no one-way meets two-way: kept
  assert len(curves_of(jog(60))) >= 1                  # a real S-bend is kept


def test_squares_only_in_their_own_folder_and_versions(tmp_path):
  assert B.folder_box("/x/generations/abc/42/-84") == (42, -84)
  assert B.folder_box("/x/43/-84") is None and B.folder_box("/x/y") is None
  assert B.in_box(B.tile_of(42.1, -83.9), (42, -84)) and B.in_box(B.tile_of(43.9, -82.1), (42, -84))
  assert not B.in_box(B.tile_of(44.05, -83.0), (42, -84)) and not B.in_box(B.tile_of(42.5, -81.9), (42, -84))
  assert B.FORMAT_VERSION == 6 and B.BUILDER_VERSION >= 4
  assert B.READ_FORMATS == (5, 6) and B.READ_MIN_BUILDER == 3         # a format 5 area works until it is rebuilt


def test_files_are_written_atomically_and_synced(tmp_path, monkeypatch):
  synced = []
  real = os.fsync
  monkeypatch.setattr(os, "fsync", lambda fd: (synced.append(fd), real(fd)))
  path = str(tmp_path / "a.json")
  B.write_json_atomic(path, {"version": B.FORMAT_VERSION})
  assert json.load(open(path)) == {"version": B.FORMAT_VERSION} and not os.path.exists(path + ".tmp")
  assert len(synced) >= 1
