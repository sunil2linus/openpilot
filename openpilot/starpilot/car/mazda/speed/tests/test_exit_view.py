"""The map view's part for the exit plan and the merge: an exit's link start distance, side, the ramp's curve profile,
"to highway" and destination speed, the match and the shapes past the split, the matched link's ramp record. Synthetic
roads only (the builder's ramp scene: a motorway north, an exit leaving right at y = 0 into a 50 m loop)."""
import os

from openpilot.starpilot.car.mazda.speed import curve_builder as B
from openpilot.starpilot.car.mazda.speed import map_match as MM
from openpilot.starpilot.car.mazda.speed import map_view as V
from openpilot.starpilot.car.mazda.speed.tests.test_curve_builder import ramp_scene, way, straight
from openpilot.starpilot.car.mazda.speed.tests.test_map_view import gps_at


def write_scene(root, ways):
  chains = B.build_chains(ways)
  nidx = B.node_index(ways)
  curves, ramps, recs = [], [], {}
  for ch in chains:
    ramp = B.ramp_record(ch, nidx, ways)
    found = B.find_curves(ch, ways, ramp)
    curves += found
    if ramp is not None:
      ramps.append(ramp)
    if found or ramp is not None:
      rec = B.chain_record(ch)
      recs[rec["id"]] = rec
  chain_of = {w.id: int(ch[0][0].id) for ch in chains for w, _ in ch}
  shapes = B.road_shapes(ways, chain_of)
  d = os.path.join(root, "42", "-84")
  os.makedirs(d, exist_ok=True)
  for name, roads in shapes.items():
    B.write_json_atomic(os.path.join(d, name + ".json"), {
      "version": B.FORMAT_VERSION, "builder": B.BUILDER_VERSION,
      "curves": [c for c in curves if B.tile_of(*c["apex"]) == name],
      "ramps": [r for r in ramps if B.tile_of(*r["start"]) == name], "chains": list(recs.values()), "roads": roads})
  return ramps


def scene_view(tmp_path, ways=None):
  ramps = write_scene(str(tmp_path), ways or ramp_scene())
  store = MM.MapCurveStore(root=str(tmp_path), background=False)
  return V.MapView(store), ramps


def drive(mv, pts, t0=0.0):
  out = []
  for k, (x, y, b) in enumerate(pts):
    out.append(mv.update(t0 + k * 0.1, gps_at(x, y, b, speed=25.0)))
  return out


def test_an_exit_ahead_has_its_start_side_profile_and_destination(tmp_path):
  mv, ramps = scene_view(tmp_path)
  exit_rec = next(r for r in ramps if r["type"] == "exit")
  views = drive(mv, [(0.0, float(y), 0.0) for y in range(-480, -300, 3)])
  v = views[-1]
  assert v["matched"] and v["cls"] == 1 and not v["link"] and "ramp" not in v
  ex = v["exits"][0]
  assert ex["id"] == f"x{exit_rec['chain']}"
  assert 290 <= ex["d"] <= 310, ex["d"]                          # the car at y = -303, the split at y = 0
  assert ex["side"] == 1 and ex["m"] == 1 and ex["sh"] == 0
  assert not ex["hwy"] and abs(ex["spd"] - 45 * B.MPH) < 0.1      # an exit onto the 45 mph primary road
  # the ramp's loop (50 m) is in the profile, placed past the split
  r_min = min(r for _, r in ex["pts"])
  assert abs(r_min - 50) / 50 < 0.2
  d_tight = min(d for d, r in ex["pts"] if r == r_min)
  assert ex["d"] + 20 < d_tight < ex["d"] + 250


def test_past_the_split_the_shapes_tell_link_from_highway(tmp_path):
  # taking the exit: along the link's first straight (20 deg right of north)
  mv, ramps = scene_view(tmp_path)
  rid = next(r for r in ramps if r["type"] == "exit")["chain"]
  import math
  pre = [(0.0, float(y), 0.0) for y in range(-200, 0, 3)]
  on = [(math.sin(math.radians(20)) * s, math.cos(math.radians(20)) * s, 20.0) for s in range(3, 90, 3)]
  views = drive(mv, pre + on)
  past = [e for v in views for e in v.get("exits", []) if e["id"] == f"x{rid}" and e["d"] < -15]
  assert past and all(e["sh"] in (0, 2) for e in past) and any(e["sh"] == 2 for e in past)
  # staying on: north past the split
  mv, _ = scene_view(tmp_path)
  views = drive(mv, [(0.0, float(y), 0.0) for y in range(-200, 150, 3)])
  past = [e for v in views for e in v.get("exits", []) if e["id"] == f"x{rid}" and e["d"] < -15]
  assert past and all(e["sh"] in (0, 1) for e in past) and any(e["sh"] == 1 for e in past)
  assert all(e["m"] == 1 for e in past)


def test_the_matched_link_carries_its_ramp_record(tmp_path):
  ways = ramp_scene()
  mv, ramps = scene_view(tmp_path, ways)
  ent = next(r for r in ramps if r["type"] == "entrance")
  # along the entrance ramp (way 13: from (400, -6) heading 340 deg)
  import math
  pts = [(400 + math.sin(math.radians(340)) * s, -6 + math.cos(math.radians(340)) * s, 340.0) for s in range(20, 300, 3)]
  v = drive(mv, pts)[-1]
  assert v["matched"] and v["link"] and v["ramp"]["id"] == ent["chain"]
  assert v["ramp"]["hwy"] and not v["ramp"]["from_hwy"] and v["ramp"]["spd"] > 0
  assert v["chain"] == ent["chain"] and v["acc"] == 5.0


def test_no_exit_for_the_other_direction_or_a_street(tmp_path):
  # driving south on the motorway: the exit starts behind (and is for the northbound direction)
  mv, _ = scene_view(tmp_path)
  views = drive(mv, [(0.0, float(y), 180.0) for y in range(600, 300, -3)])
  assert not views[-1].get("exits")
  # a street with a link: no exits are watched (only from a highway)
  ways = {w.id: w for w in (way(20, straight(0, -500, 0, 1500), cls=5, speed_mph=45),)}
  mv, _ = scene_view(tmp_path / "s", ways)
  views = drive(mv, [(0.0, float(y), 0.0) for y in range(-480, -300, 3)])
  assert not views[-1].get("exits")


def test_the_view_is_small(tmp_path):
  import json
  mv, _ = scene_view(tmp_path)
  v = drive(mv, [(0.0, float(y), 0.0) for y in range(-480, -10, 3)])[-1]
  assert len(json.dumps(v, separators=(",", ":"))) < 4096
  assert V.VIEW_VERSION == 2


def pass_through_scene():
  """(r2, M-53 at 23 Mile Rd) a motorway north; its exit ramp (no name) leaves at y = 0 heading 20 deg right, meets a
  primary street running east-west at one node, and runs straight on across it as the on-ramp back to the motorway:
  the builder joins both ramp ways into ONE chain whose record says motorway to motorway."""
  import math
  mw = way(30, straight(0, -500, 0, 2000), name="", cls=1, speed_mph=70)
  mw.ref = "M 53"
  h = math.radians(20)
  a_pts = [(math.sin(h) * s, math.cos(h) * s) for s in range(0, 201, 20)]
  jx, jy = a_pts[-1]
  b_pts = [(jx - math.sin(h) * s, jy + math.cos(h) * s) for s in range(0, 201, 20)]
  ex = way(31, a_pts, name="", cls=2, speed_mph=45, oneway=True)
  on = way(32, b_pts, name="", cls=2, speed_mph=45, oneway=True)
  st_pts = [(jx + d, jy) for d in range(-300, 301, 20)]
  st = way(33, st_pts, name="23 Mile Road", cls=5, speed_mph=45)
  # shared nodes: the exit's start on the motorway, the joint on the street, the on-ramp's end on the motorway
  k0 = 25
  ex.lat[0], ex.lon[0] = mw.lat[k0], mw.lon[k0]
  ex.start = B.key(mw.lat[k0], mw.lon[k0])
  j = st_pts.index((jx + 0, jy))
  on.lat[0], on.lon[0] = ex.lat[-1], ex.lon[-1]
  st.lat[j], st.lon[j] = ex.lat[-1], ex.lon[-1]
  on.start = ex.end
  k1 = min(range(len(mw.lat)), key=lambda k: abs(mw.lat[k] - on.lat[-1]))
  on.lat[-1], on.lon[-1] = mw.lat[k1], mw.lon[k1]
  on.end = B.key(mw.lat[k1], mw.lon[k1])
  return {w.id: w for w in (mw, ex, on, st)}


def test_an_exit_whose_chain_runs_on_across_a_street_leads_to_that_street(tmp_path):
  import math
  ways = pass_through_scene()
  mv, ramps = scene_view(tmp_path, ways)
  rec = next(r for r in ramps if r["chain"] in (31, 32))
  assert rec["to"]["cls"] == 1 and rec["from"]["cls"] == 1 and len(rec["wdir"]) == 2   # (the record as built)
  views = drive(mv, [(0.0, float(y), 0.0) for y in range(-480, -300, 3)])
  info = mv.links.info(rec)
  assert [c for _k, c in info["cuts"]] == [5] and 190 < info["cut_s"] < 210
  # on the motorway before the split: the exit leads to the street (no "to highway"), its profile ends there
  ex = views[-1]["exits"][0]
  assert not ex["hwy"] and abs(ex["spd"] - V.STREET_SPEED_MPH[5] * B.MPH) < 0.1
  assert all(d <= ex["d"] + info["cut_s"] + 1 for d, _r in ex["pts"])
  # on the exit half of the link: to the street; on the on-ramp half past the street: to the motorway, from a street
  h = math.radians(20)
  pts = [(math.sin(h) * s, math.cos(h) * s, 20.0) for s in range(20, 180, 3)]
  v = drive(mv, pts)[-1]
  assert v["link"] and v["ramp"]["street"] == 5 and not v["ramp"]["hwy"] and v["ramp"]["from_hwy"]
  jx, jy = math.sin(h) * 200, math.cos(h) * 200
  pts = [(jx - math.sin(h) * s, jy + math.cos(h) * s, 340.0) for s in range(20, 180, 3)]
  mv, _ = scene_view(tmp_path / "b", ways)
  v = drive(mv, pts)[-1]
  assert v["link"] and v["ramp"]["hwy"] and not v["ramp"]["from_hwy"] and "street" not in v["ramp"]


def test_an_ordinary_exit_and_entrance_keep_their_records(tmp_path):
  mv, ramps = scene_view(tmp_path)
  drive(mv, [(0.0, float(y), 0.0) for y in range(-480, -300, 3)])
  for r in ramps:
    assert mv.links.info(r)["cuts"] == []
  ent = next(r for r in ramps if r["type"] == "entrance")
  assert mv.links.leads(ent)[0] is True
