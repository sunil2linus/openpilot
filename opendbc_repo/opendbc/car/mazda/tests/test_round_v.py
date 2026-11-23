"""Round V (7.0 drives r2 / r3 replayed on the old logic): the four fixes as tests (synthetic geometry only)."""
import importlib.util
import json
import math
import os

HERE = os.path.dirname(__file__)


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


TM = _load("tmc_round_v", os.path.join(HERE, "test_map_curves.py"))
MC, G, B = TM.MC, TM.G, TM.B


def _along(x0, y0, heading, d):
  h = math.radians(heading)
  return x0 + math.sin(h) * d, y0 + math.cos(h) * d


def _set_node(w, i, other, j):
  """Make node i of way w the same node as node j of way `other` (a shared OSM node: the same position)."""
  w.lat[i], w.lon[i] = other.lat[j], other.lon[j]
  w.start, w.end = B.key(w.lat[0], w.lon[0]), B.key(w.lat[-1], w.lon[-1])


# ---- V1: an exit ramp to a street joined with the on-ramp across it (M-53 to 23 Mile Rd) ----

def junction_scene(shared=True, split_street=False, loop=True):
  """A motorway north; an exit ramp (way 21) leaves it heading 20 deg and after 300 m reaches a street (way 30, east-west,
  45 mph); the on-ramp (way 22) goes on straight across and (loop) turns right onto another motorway (way 40). The builder
  joins 21 and 22 into one chain ("interchange"). shared=False: the street passes 2.5 m beside the joint with no node
  there (a bridge). split_street: the street is two ways that end at the junction (its node kept in the shapes)."""
  mw = G.way(10, G.straight(0, -500, 0, 1500), name="", cls=1, speed_mph=70)
  mw.ref = "M 53"
  jx, jy = _along(0, 0, 20, 300)
  ex = G.way(21, G.straight(0, 0, 20, 300), name="", cls=2, speed_mph=45, oneway=True)
  _set_node(ex, 0, mw, 25)
  on_pts = G.straight(jx, jy, 20, 100)
  if loop:
    ox, oy = on_pts[-1]
    cxy = (ox + 60 * math.cos(math.radians(20)), oy - 60 * math.sin(math.radians(20)))
    arc = G.arc(cxy[0], cxy[1], 60, 180 - 20, 90 - 20)[1:]
    on_pts = on_pts + arc + G.straight(arc[-1][0], arc[-1][1], 110, 200)[1:]
  on = G.way(22, on_pts, name="", cls=2, speed_mph=45, oneway=True)
  on.lat[0], on.lon[0] = ex.lat[-1], ex.lon[-1]
  on.start = B.key(on.lat[0], on.lon[0])
  ex.end = on.start
  ex2 = on.lat[-1], on.lon[-1]
  mw2 = G.way(40, G.straight(0, 0, 110, 600), name="", cls=1, speed_mph=70)
  mw2.ref = "M 59"
  mw2.lat = mw2.lat - mw2.lat[0] + ex2[0]
  mw2.lon = mw2.lon - mw2.lon[0] + ex2[1]
  mw2.start, mw2.end = B.key(mw2.lat[0], mw2.lon[0]), B.key(mw2.lat[-1], mw2.lon[-1])
  on.end = mw2.start
  street_pts = [(jx - 300 + 20 * i, jy + (0.0 if shared else 2.5)) for i in range(31)]
  ways = {w.id: w for w in (mw, ex, on, mw2)}
  if split_street:
    a, b = G.way(30, street_pts[:16], name="23 Mile Road", cls=7), G.way(31, street_pts[15:], name="23 Mile Road", cls=7)
    if shared:
      _set_node(a, len(a.lat) - 1, ex, len(ex.lat) - 1)
      _set_node(b, 0, ex, len(ex.lat) - 1)
    ways.update({30: a, 31: b})
  else:
    st = G.way(30, street_pts, name="23 Mile Road", cls=7)
    if shared:
      _set_node(st, 15, ex, len(ex.lat) - 1)
    ways[30] = st
  return ways


def _write(ways, drop_cuts=False):
  root, curves, ramps = TM.write_files(ways)
  if drop_cuts:                    # a builder 1 file: no "cuts" in the ramp records
    for dirpath, _d, files in os.walk(root):
      for f in files:
        p = os.path.join(dirpath, f)
        d = json.load(open(p))
        for r in d["ramps"]:
          r.pop("cuts", None)
        json.dump(d, open(p, "w"))
  return root, curves, ramps


def _on(m, way_id, d_along, heading=20.0, x0=0.0, y0=0.0):
  x, y = _along(x0, y0, heading, d_along)
  la, lo = TM.pos(x, y)
  m.update(la, lo, heading, way_id, True)
  return m


def test_v1_the_builder_finds_the_street_at_the_shared_node():
  ways = junction_scene()
  nidx = B.node_index(ways)
  recs = [r for r in (B.ramp_record(ch, nidx, ways) for ch in B.build_chains(ways)) if r]
  r = [x for x in recs if x["wdir"][0][0] == 21][0]
  assert [w for w, _f in r["wdir"]] == [21, 22] and r["type"] == "interchange"
  assert len(r["cuts"]) == 1
  k, cls, spd, s, la, lo = r["cuts"][0]
  assert (k, cls) == (0, 7) and abs(spd - 45 * B.MPH) < 0.01 and abs(s - 300.0) < 1.0
  assert MC.dist_m(la, lo, ways[21].lat[-1], ways[21].lon[-1]) < 0.2
  # a street passing beside the joint with no node there (a bridge): no cut
  ways = junction_scene(shared=False)
  nidx = B.node_index(ways)
  recs = [r for r in (B.ramp_record(ch, nidx, ways) for ch in B.build_chains(ways)) if r]
  assert [x for x in recs if x["wdir"][0][0] == 21][0]["cuts"] == []


def _check_exit_then_entrance(root):
  m = TM.planner(root)
  _on(m, 21, 100.0)
  assert m.on_ramp and m.ramp["type"] == "exit" and m.ramp["to"]["cls"] == 7 and not MC.ramp_to_highway(m.ramp)
  assert abs(m.ramp_end_dist - 200.0) < 5.0        # to the street, not to the far motorway
  assert m.curve is None                            # the on-ramp's loop past the street is not on this exit
  m2 = TM.planner(root)
  jx, jy = _along(0, 0, 20, 300)
  _on(m2, 22, 50.0, x0=jx, y0=jy)
  assert m2.on_ramp and m2.ramp["type"] == "entrance" and m2.ramp["from"]["cls"] == 7 and MC.ramp_to_highway(m2.ramp)
  assert m2.curve is not None and m2.status == MC.STATUS_RAMP_CURVE   # its own loop is still planned
  return m, m2


def test_v1_on_the_exit_part_it_is_an_exit_to_the_street_and_past_it_an_on_ramp_from_it():
  root, _, ramps = _write(junction_scene())
  r = [x for x in ramps if x["wdir"][0][0] == 21][0]
  assert r["to"]["cls"] == 1 and r["type"] == "interchange"          # as built
  m, _m2 = _check_exit_then_entrance(root)
  assert abs(m.ramp["to"]["speed"] - 45 * B.MPH) < 0.01             # the street's own speed (builder 2)


def test_v1_older_files_fall_back_to_a_shared_street_vertex_in_the_shapes():
  # (builder 1 files: no "cuts") the street split at the junction keeps its node in the shapes: found
  root, _, _ = _write(junction_scene(split_street=True), drop_cuts=True)
  m, _m2 = _check_exit_then_entrance(root)
  assert abs(m.ramp["to"]["speed"] - MC.STREET_SPEED_MPH[7] * MC.MPH) < 0.01   # its class default
  # a street merely passing close (2.5 m, a bridge over it) is no junction
  root, _, _ = _write(junction_scene(shared=False, split_street=True), drop_cuts=True)
  m = _on(TM.planner(root), 21, 100.0)
  assert m.on_ramp and m.ramp["type"] == "interchange" and MC.ramp_to_highway(m.ramp)
  # (the fallback's limit: a straight street's node is simplified away in the shapes, so it is not found there)
  root, _, _ = _write(junction_scene(), drop_cuts=True)
  m = _on(TM.planner(root), 21, 100.0)
  assert m.ramp["type"] == "interchange"


def test_v1_navigation_and_the_on_ramp_search_see_the_exit_to_the_street():
  root, _, ramps = _write(junction_scene())
  m = TM.planner(root)
  r = [x for x in ramps if x["wdir"][0][0] == 21][0]
  la, lo = TM.pos(0, -300)
  d = MC.dist_m(la, lo, *r["start"])
  m.update(la, lo, 0.0, 10, True, ("off ramp", "slight right", d))
  assert m.nav_ramp is not None and m.nav_ramp["chain"] == r["chain"] and m.nav_ramp["type"] == "exit"
  assert not MC.ramp_to_highway(m.nav_ramp)
  # an ordinary on-ramp record still leads to the highway
  assert MC.ramp_to_highway({"to": {"cls": 1}}) and not MC.ramp_to_highway({"to": {"cls": 7}})
