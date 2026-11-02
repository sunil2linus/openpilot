"""Offline tests for the curve map generator (synthetic roads, no map files or capnp needed)."""
import importlib.util
import math
import os

import numpy as np

_spec = importlib.util.spec_from_file_location("bcm", os.path.join(os.path.dirname(__file__), "build_curve_map.py"))
B = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(B)

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
      assert len(cs) == 1
      c = cs[0]
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
    for la, lo in zip(w.lat, w.lon):
      best = 1e9
      for pc in pieces:
        for (a1, b1), (a2, b2) in zip(pc, pc[1:]):
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
