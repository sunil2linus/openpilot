#!/usr/bin/env python3
"""
Build a curve map from FrogPilot's offline OpenStreetMap tiles.

FrogPilot keeps OSM road data on the device in /data/media/0/osm/offline/<lat>/<lon>/<file>, one
packed capnp file (offline_maps.capnp) per 0.25 x 0.25 degree square, grouped in 2 x 2 degree folders.
This script finds the curves on those roads and writes one small JSON file per square, which the car
side loads around the current position (a few dozen curves per square).

A curve is found ALONG a road, never at an intersection: ways are joined into chains only where the
same road (same name or ref, or the same unnamed link class) continues end to end with a small
heading change; a turn from one road into another is not part of any chain. Along each chain the
heading change over a 40 m window gives the curvature (robust to the spacing and small wobbles of
OSM points), and a curve is a stretch with radius below MAX_RADIUS that turns enough to matter.

Each curve records its tightest radius, both ends (with the heading in each direction of travel), the
apex, the road's speed limit (or a default for its class), any advisory speed sign tagged in OSM, and
the speed it needs at 2.5 / 3.0 / 3.5 m/s^2 of lateral acceleration (the car uses its own limit). Only
curves that matter are kept: the speed they need at KEEP_LAT is below the road speed + KEEP_OVER.

Runs anywhere with Python 3.9+ and `pip install pycapnp numpy`, e.g. on a laptop:

  python build_curve_map.py --tiles <folder with the 0.25 deg files> --out <output folder>
  python build_curve_map.py --tiles osm_offline/42/-84 --out curves/42/-84

The schema (offline_maps.capnp) is read from FrogPilot's frogpilot/system/ next to this script's repo
root, or from --schema.
"""
import argparse
import glob
import json
import math
import os
import sys
import time

import numpy as np

MPH = 0.44704
EARTH_R = 6371008.8

MAX_RADIUS = 600.0          # m: anything gentler is not a curve for us
MIN_TURN_DEG = 12.0         # total heading change of a curve
MIN_LENGTH = 15.0           # m
STEP = 2.0                  # m: resampling step along a chain
HALF_WINDOW = 30.0          # m: heading change measured between chords of this length on each side (checked
                            # against radii measured on the road: 117 / 131 / 216 m vs 104-116 / 138-164 / 231 m)
JOIN_MAX_TURN = 50.0        # deg: ways only join into one road when they continue this straight
KEEP_LAT = 3.0              # m/s^2: a curve is kept if sqrt(KEEP_LAT * R) < road speed + KEEP_OVER
KEEP_OVER = 15 * MPH
LAT_LEVELS = (2.5, 3.0, 3.5)

# FrogPilot highwayClass values (from the offline tiles): 1 motorway, 2 motorway_link, 3 trunk,
# 4 trunk_link, 5 primary, 6 primary_link, 7 secondary, 8 secondary_link, 9 tertiary, 10 tertiary_link,
# 11 unclassified, 12 residential. Default speeds (Michigan-style) where OSM has no maxspeed.
CLASS_NAMES = {1: "motorway", 2: "motorway_link", 3: "trunk", 4: "trunk_link", 5: "primary", 6: "primary_link",
               7: "secondary", 8: "secondary_link", 9: "tertiary", 10: "tertiary_link", 11: "unclassified",
               12: "residential"}
DEFAULT_SPEED_MPH = {1: 70, 2: 45, 3: 65, 4: 45, 5: 50, 6: 35, 7: 45, 8: 35, 9: 45, 10: 30, 11: 40, 12: 25}
LINK_CLASSES = {2, 4, 6, 8, 10}
SKIP_CLASSES = {12}         # residential streets: 25 mph, cul-de-sacs and courts; the slowdown works from 25 mph up


def load_schema(path=None):
  import capnp
  capnp.remove_import_hook()
  if path is None:
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, "..", "..", "frogpilot", "system", "offline_maps.capnp"),
                 os.path.join(here, "offline_maps.capnp")):
      if os.path.exists(cand):
        path = cand
        break
  if path is None:
    sys.exit("offline_maps.capnp not found: pass --schema")
  return capnp.load(path)


def read_tile(schema, path):
  data = open(path, "rb").read()
  if len(data) < 64:
    return None
  return schema.Offline.from_bytes_packed(data, traversal_limit_in_words=2 ** 62)


# ---------------- geometry ----------------

def to_xy(lat, lon, lat0, lon0):
  x = np.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
  y = np.radians(lat - lat0) * EARTH_R
  return x, y


def to_latlon(x, y, lat0, lon0):
  lat = lat0 + math.degrees(y / EARTH_R)
  lon = lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0))))
  return lat, lon


def bearing_deg(dx, dy):
  return (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0


def angle_diff(a, b):
  return (a - b + 180.0) % 360.0 - 180.0


def resample(x, y, step):
  d = np.hypot(np.diff(x), np.diff(y))
  keep = np.concatenate(([True], d > 0.05))
  x, y = x[keep], y[keep]
  if len(x) < 2:
    return None, None, None
  s = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
  n = int(s[-1] // step) + 1
  if n < 3:
    return None, None, None
  si = np.arange(n) * step
  return np.interp(si, s, x), np.interp(si, s, y), si


def curvature(x, y, step, half):
  """Signed curvature (1/m, + = left) at each resampled point: heading change between the chord over
  the previous `half` metres and the chord over the next `half` metres, divided by `half`."""
  n = len(x)
  k = int(round(half / step))
  out = np.zeros(n)
  if n < 2 * k + 1:
    return out
  i = np.arange(k, n - k)
  h1 = np.arctan2(y[i] - y[i - k], x[i] - x[i - k])
  h2 = np.arctan2(y[i + k] - y[i], x[i + k] - x[i])
  dh = (h2 - h1 + np.pi) % (2 * np.pi) - np.pi
  out[i] = dh / half
  return out


# ---------------- road chains ----------------

class Way:
  __slots__ = ("id", "name", "ref", "cls", "speed", "advisory", "oneway", "lat", "lon", "start", "end")


def key(lat, lon):
  return (round(lat * 1e7), round(lon * 1e7))


def load_ways(schema, files):
  ways = {}
  for f in files:
    t = read_tile(schema, f)
    if t is None:
      continue
    for w in t.ways:
      if w.id in ways or w.highwayClass not in CLASS_NAMES or w.highwayClass in SKIP_CLASSES or len(w.nodes) < 2:
        continue
      wy = Way()
      wy.id, wy.name, wy.ref, wy.cls = w.id, w.name, w.ref, w.highwayClass
      wy.speed = w.maxSpeed if w.maxSpeed > 0 else DEFAULT_SPEED_MPH[w.highwayClass] * MPH
      wy.advisory = w.advisorySpeed if w.advisorySpeed > 0 else 0.0
      wy.oneway = bool(w.oneWay)
      wy.lat = np.array([n.latitude for n in w.nodes])
      wy.lon = np.array([n.longitude for n in w.nodes])
      wy.start, wy.end = key(wy.lat[0], wy.lon[0]), key(wy.lat[-1], wy.lon[-1])
      ways[w.id] = wy
  return ways


def same_road(a, b):
  if a.cls in LINK_CLASSES or b.cls in LINK_CLASSES:
    return a.cls == b.cls and (a.ref == b.ref or not a.ref or not b.ref) and (a.name == b.name or not a.name or not b.name)
  if a.name and a.name == b.name:
    return True
  if a.ref and a.ref == b.ref:
    return True
  return not a.name and not b.name and not a.ref and not b.ref and a.cls == b.cls


def end_heading(w, at_start, flip=False):
  """Heading (deg) of travel leaving the way's end `at_start` towards its interior, or arriving."""
  lat0, lon0 = w.lat[0], w.lon[0]
  x, y = to_xy(w.lat, w.lon, lat0, lon0)
  if at_start:   # direction of travel from the start into the way
    j = 1
    while j < len(x) - 1 and math.hypot(x[j] - x[0], y[j] - y[0]) < 10:
      j += 1
    h = bearing_deg(x[j] - x[0], y[j] - y[0])
  else:          # direction of travel arriving at the end
    j = len(x) - 2
    while j > 0 and math.hypot(x[-1] - x[j], y[-1] - y[j]) < 10:
      j -= 1
    h = bearing_deg(x[-1] - x[j], y[-1] - y[j])
  return (h + 180.0) % 360.0 if flip else h


def build_chains(ways):
  """Chains of (way, reversed) joined end to end along the same road, never through a turn."""
  ends = {}
  for w in ways.values():
    ends.setdefault(w.start, []).append((w, True))
    ends.setdefault(w.end, []).append((w, False))
  used = set()

  def leave_heading(w, rev):
    # heading arriving at the chain's current far end
    return end_heading(w, at_start=rev, flip=rev)

  def next_way(w, rev):
    far = w.start if rev else w.end
    arrive = leave_heading(w, rev)
    best, best_turn = None, JOIN_MAX_TURN
    cands = [(o, at_start) for o, at_start in ends.get(far, []) if o.id != w.id and o.id not in used and same_road(w, o)]
    for o, at_start in cands:
      depart = end_heading(o, at_start=at_start, flip=not at_start)
      turn = abs(angle_diff(depart, arrive))
      if turn < best_turn:
        best, best_turn = (o, not at_start), turn
    return best

  chains = []
  for w in ways.values():
    if w.id in used:
      continue
    used.add(w.id)
    chain = [(w, False)]
    # forward
    cur = (w, False)
    while True:
      nxt = next_way(*cur)
      if nxt is None:
        break
      used.add(nxt[0].id)
      chain.append(nxt)
      cur = nxt
    # backward: walk from the start of w (reversed)
    cur = (w, True)
    back = []
    while True:
      nxt = next_way(*cur)
      if nxt is None:
        break
      used.add(nxt[0].id)
      back.append((nxt[0], not nxt[1]))
      cur = nxt
    chains.append(list(reversed(back)) + chain)
  return chains


# ---------------- curves ----------------

def chain_geometry(chain):
  lats, lons, wid = [], [], []
  for w, rev in chain:
    la, lo = (w.lat[::-1], w.lon[::-1]) if rev else (w.lat, w.lon)
    if lats:
      la, lo = la[1:], lo[1:]
    lats.append(la)
    lons.append(lo)
    wid.append(np.full(len(la), w.id))
  return np.concatenate(lats), np.concatenate(lons), np.concatenate(wid)


def find_curves(chain, ways_by_id):
  lat, lon, wids = chain_geometry(chain)
  if len(lat) < 3:
    return []
  lat0, lon0 = float(lat.mean()), float(lon.mean())
  x, y = to_xy(lat, lon, lat0, lon0)
  # cumulative distance at the original points, to map resampled points back to ways
  s_pts = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
  rx, ry, rs = resample(x, y, STEP)
  if rx is None:
    return []
  k = curvature(rx, ry, STEP, HALF_WINDOW)
  kmin = 1.0 / MAX_RADIUS
  curves = []
  n = len(k)
  i = 0
  while i < n:
    if abs(k[i]) < kmin:
      i += 1
      continue
    sign = np.sign(k[i])
    j = i
    while j + 1 < n and k[j + 1] * sign >= kmin:
      j += 1
    seg = slice(i, j + 1)
    length = (j - i) * STEP
    # total heading change over the curve (chords just outside it)
    a = max(i - 2, 0)
    b = min(j + 2, n - 1)
    turn = 0.0
    if b - a >= 2:
      h1 = math.atan2(ry[a + 1] - ry[a], rx[a + 1] - rx[a])
      h2 = math.atan2(ry[b] - ry[b - 1], rx[b] - rx[b - 1])
      turn = abs(math.degrees((h2 - h1 + math.pi) % (2 * math.pi) - math.pi))
      turn = max(turn, abs(math.degrees(float(np.sum(k[seg]) * STEP))))
    if length >= MIN_LENGTH and turn >= MIN_TURN_DEG:
      apex = i + int(np.argmax(np.abs(k[seg])))
      r_min = 1.0 / abs(k[apex])
      # the ways the curve lies on
      wset = sorted({int(wids[min(np.searchsorted(s_pts, rs[m]), len(wids) - 1)]) for m in range(i, j + 1, 5)} |
                    {int(wids[min(np.searchsorted(s_pts, rs[j]), len(wids) - 1)])})
      wl = [ways_by_id[w] for w in wset if w in ways_by_id]
      speed = min(w.speed for w in wl) if wl else 0.0
      adv = min([w.advisory for w in wl if w.advisory > 0] or [0.0])
      cls = min(w.cls for w in wl) if wl else 0
      if math.sqrt(KEEP_LAT * r_min) < speed + KEEP_OVER:
        def pt(m):
          la, lo = to_latlon(float(rx[m]), float(ry[m]), lat0, lon0)
          return round(la, 6), round(lo, 6)
        h_in = bearing_deg(rx[min(i + 1, n - 1)] - rx[i], ry[min(i + 1, n - 1)] - ry[i])
        h_out = bearing_deg(rx[j] - rx[max(j - 1, 0)], ry[j] - ry[max(j - 1, 0)])
        name = next((w.name or w.ref for w in wl if w.name or w.ref), "")
        curves.append({
          "a": pt(i), "a_hdg": round(h_in, 1),            # entry when travelling a -> b
          "b": pt(j), "b_hdg": round((h_out + 180) % 360, 1),  # entry when travelling b -> a
          "apex": pt(apex), "r": round(r_min, 1), "len": round(length, 1), "turn": round(turn, 1),
          "dir": "L" if sign > 0 else "R",                 # turning direction when travelling a -> b
          "speed": round(speed, 2), "adv": round(adv, 2), "cls": cls, "name": name,
          "oneway": any(w.oneway for w in wl), "ways": wset,
          "v": {str(L): round(math.sqrt(L * r_min), 2) for L in LAT_LEVELS},
        })
    i = j + 1
  return curves


def tile_of(lat, lon):
  la = math.floor(lat * 4) / 4
  lo = math.floor(lon * 4) / 4
  return f"{la:.6f}_{lo:.6f}_{la + 0.25:.6f}_{lo + 0.25:.6f}"


def build(tiles_dir, out_dir, schema_path=None, quiet=False):
  schema = load_schema(schema_path)
  files = sorted(f for f in glob.glob(os.path.join(tiles_dir, "*")) if os.path.isfile(f))
  if not files:
    sys.exit(f"no tile files in {tiles_dir}")
  t0 = time.time()
  ways = load_ways(schema, files)
  chains = build_chains(ways)
  curves = []
  for ch in chains:
    curves.extend(find_curves(ch, ways))
  # one file per 0.25 deg square, by the curve's apex
  by_tile = {}
  for c in curves:
    by_tile.setdefault(tile_of(*c["apex"]), []).append(c)
  os.makedirs(out_dir, exist_ok=True)
  for name, cs in by_tile.items():
    with open(os.path.join(out_dir, name + ".json"), "w") as f:
      json.dump({"version": 1, "keep_lat": KEEP_LAT, "curves": cs}, f, separators=(",", ":"))
  if not quiet:
    size = sum(os.path.getsize(os.path.join(out_dir, n + ".json")) for n in by_tile)
    print(f"{len(files)} tiles, {len(ways)} roads, {len(chains)} chains, {len(curves)} curves, "
          f"{len(by_tile)} output files, {size / 1e6:.2f} MB, {time.time() - t0:.0f} s")
  return curves, ways


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--tiles", required=True, help="folder with FrogPilot offline map files (one 2x2 degree folder)")
  p.add_argument("--out", required=True, help="output folder for the curve files")
  p.add_argument("--schema", help="path to offline_maps.capnp (default: from the FrogPilot repo)")
  a = p.parse_args()
  build(a.tiles, a.out, a.schema)


if __name__ == "__main__":
  main()
