#!/usr/bin/env python3
"""
Curve map builder (ported from the old fork's tools/curve_map/build_curve_map.py, geometry rewritten).

Reads 7.0's offline map tiles (offline.capnp:0xda3a0d9284ca402f, one packed file per 0.25 x 0.25 degree square in
2 x 2 degree folders: <root>/generations/<selected>/<lat>/<lon>/<square>) and writes one small JSON file per square
with the curves along each road, the ramps and the road shapes the car matches itself to.

What changed against the old builder (FORMAT_VERSION 4 / BUILDER_VERSION 1):
  * Geometry: a centripetal (alpha 0.5, chord-length) Catmull-Rom spline through the OSM nodes BEFORE the 2 m
    resampling. It removes the 3-node corner kinks and the phase beating of polygonal loops (a heading change only
    at the nodes), and keeps real sustained pinches (tests/test_curve_builder.py has the acceptance shapes).
  * Every curve carries a distance profile "prof": [[s, r], ...] along a -> b (m from its entry a, radius m),
    simplified so each point is within PROFILE_TOL of the curvature (5-12 points for a long curve). The car plans
    every point with the same braking envelope; "r" (the tightest) stays for matching, memory and logs.
  * split_doublets (carriageway split jogs), the chain joining (never through a turn) and the ramp records are
    kept as they were; the car side keeps the "never follow a one-way backwards" rule.
  * Files are written with flush + fsync before the rename (a power cut left 0-byte files); a square is written
    only into its own 2 x 2 degree folder (squares that overhang the folder edge were orphan files).

Format 6 / builder 4 (the owner's one-time rebuild of his map areas):
  * Every road shape carries its way's TAGGED speed limit "l" (tag_code: N mph, -N km/h, the tag's own unit and
    whole number; [forward, backward] when a two-way way's directions differ; no field when untagged). Never the
    class default (DEFAULT_SPEED_MPH only decides which curves are kept). The conditional tags are not read. The car
    uses it only where 7.0's map service has no valid limit (limits.MapLimit). About +3 % file size.
  * Every ramp record carries "j": the joints between its ways where a street (STREET_CLASSES) shares the NODE (an
    at-grade junction; a bridge shares none), found from the tiles' exact node coordinates (key()); and its "to" /
    "from" roads carry "h", their travel heading at the link's own end node when one-way. map_view.LinkShapes uses
    them instead of searching the simplified shapes (the 2 m simplification dropped ~40 % of the street nodes at link
    ends). A format 5 file (an area not rebuilt yet) still loads; the car then uses the read-time rules there.

VERSIONS: bump BUILDER_VERSION on ANY change to how curves, profiles, ramps or road shapes are computed (geometry,
thresholds, smoothing, simplification), and FORMAT_VERSION on any change to the file layout. curve_mapd rebuilds
every folder whose files are older in either; the car ignores files whose FORMAT_VERSION is not its own (no map
curves there until rebuilt). (The old builder added split_doublets without a bump, so existing folders were never
rebuilt with it.)

Known limit (not fixed here): curvature() leaves the first and last HALF_WINDOW of each chain at zero, and a folder
only knows its own tiles, so a curve lying exactly on a 2-degree folder border is clipped; a one-square halo from
the neighbouring folder would fix it.

Runs on the device (curve_mapd.py, parked) or anywhere with numpy + pycapnp:
  python -m openpilot.starpilot.car.mazda.speed.curve_builder --tiles <folder of 0.25 deg files> --out <folder>
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
KEEP_LAT = 2.2              # m/s^2: a curve is kept if sqrt(KEEP_LAT * R) < road speed + KEEP_OVER (the car plans at 2.2)
KINK_SLACK = 1.5            # m: a car can cut a short sharp bend this far inside its lane (see effective_radius)
KEEP_OVER = 15 * MPH
ADVISORY_OVER = 10 * MPH     # the car never plans a curve faster than its yellow advisory sign + this
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
SHAPE_TOLERANCE = 2.0       # m: road shapes for the car's own road matching are simplified to this
FORMAT_VERSION = 6          # 3: curves, ramps, chains; 4: + road shapes; 5: + distance profiles ("prof");
                            # 6: + the tagged speed limit on road shapes ("l") and the link joints ("j", to / from "h")
READ_FORMATS = (5, 6)       # the car reads these (a format 5 file has no limits and no joints: the read-time rules)
# BUILDER_VERSION: how the curves are found. Written into every file ("builder"); the device rebuilds a folder whose
# files were made by an older builder. Bump it when a change here should rebuild the existing curve files (files
# without the field were made by builder 1: the same algorithm, before the field existed).
BUILDER_VERSION = 4          # 1: old fork; 2: (old fork, split_doublets, never bumped); 3: spline + profiles;
                             # 4: tagged limits + link joints by shared node
READ_MIN_BUILDER = 3         # the car reads files of this builder and newer (curve_mapd rebuilds anything older than
                             # BUILDER_VERSION / FORMAT_VERSION; until then a format 5 / builder 3 file still works)
KEY_SCALE = 4_000_000_000   # node key = round(lat * 1e7) * KEY_SCALE + round(lon * 1e7) (one int, lon fits below the scale)


SCHEMA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "offline_tiles.capnp")


def load_schema(path=None):
  import capnp
  return capnp.load(path or SCHEMA_PATH)


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


SPLINE_STEP = 1.0           # m between spline samples (then resampled to STEP)
SPLINE_ALPHA = 0.5          # centripetal Catmull-Rom: no cusps or self-intersections at uneven node spacing


def spline(x, y, max_step=SPLINE_STEP, alpha=SPLINE_ALPHA):
  """Centripetal Catmull-Rom spline through the points (vectorized). It passes through every node, so a real
  sustained pinch (many nodes) keeps its shape, while a lone corner node gets a tangent along its neighbours' chord
  (a rounded arc instead of a kink) and a polygon's heading change is spread along its sides."""
  x, y = np.asarray(x, dtype=np.float64), np.asarray(y, dtype=np.float64)
  d = np.hypot(np.diff(x), np.diff(y))
  keep = np.concatenate(([True], d > 0.05))
  x, y = x[keep], y[keep]
  n = len(x)
  if n < 3:
    return x, y
  pts = np.stack([x, y], axis=1)
  P = np.vstack([2 * pts[0] - pts[1], pts, 2 * pts[-1] - pts[-2]])
  P0, P1, P2, P3 = P[:-3], P[1:-2], P[2:-1], P[3:]
  def knot(a, b):
    return np.maximum(np.hypot(*(b - a).T), 1e-6) ** alpha
  t1 = knot(P0, P1)
  t2 = t1 + knot(P1, P2)
  t3 = t2 + knot(P2, P3)
  seg_len = np.hypot(*(P2 - P1).T)
  m = np.maximum(1, np.ceil(seg_len / max_step)).astype(np.int64)
  idx = np.repeat(np.arange(len(m)), m)
  first = np.repeat(np.cumsum(m) - m, m)
  u = (np.arange(int(m.sum())) - first + 1) / m[idx]
  T0, T1, T2, T3 = 0.0, t1[idx], t2[idx], t3[idx]
  t = (T1 + u * (T2 - T1))[:, None]
  T1, T2, T3 = T1[:, None], T2[:, None], T3[:, None]
  p0, p1, p2, p3 = P0[idx], P1[idx], P2[idx], P3[idx]
  A1 = ((T1 - t) * p0 + (t - T0) * p1) / (T1 - T0)
  A2 = ((T2 - t) * p1 + (t - T1) * p2) / (T2 - T1)
  A3 = ((T3 - t) * p2 + (t - T2) * p3) / (T3 - T2)
  B1 = ((T2 - t) * A1 + (t - T0) * A2) / (T2 - T0)
  B2 = ((T3 - t) * A2 + (t - T1) * A3) / (T3 - T1)
  C = ((T2 - t) * B1 + (t - T1) * B2) / (T2 - T1)
  out = np.vstack([pts[:1], C])
  return out[:, 0], out[:, 1]


PROFILE_TOL = 0.0004        # 1/m: a profile point is kept where the curvature leaves the straight line between kept
PROFILE_REL = 0.04          # points by more than this, or by this share of the curvature there
PROFILE_MAX_POINTS = 12
PROFILE_MAX_R = 2000.0      # m: a profile point never reads wider than this


def profile(s, k, r_floor=0.0):
  """[[s, r], ...]: the curvature along a curve (s m from its entry, |k| 1/m) simplified to a few points (vertical
  Douglas-Peucker on the curvature, the tightest point always kept). r is at least r_floor (effective_radius)."""
  s = np.asarray(s, dtype=np.float64)
  c = np.minimum(np.abs(np.asarray(k, dtype=np.float64)), 1.0 / max(r_floor, 1e-3) if r_floor > 0 else np.inf)
  c = np.maximum(c, 1.0 / PROFILE_MAX_R)
  n = len(s)
  if n == 0:
    return []
  tol_scale = 1.0
  while True:
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    keep[int(np.argmax(c))] = True
    stack = [(0, n - 1)]
    while stack:
      a, b = stack.pop()
      if b - a < 2:
        continue
      seg = slice(a + 1, b)
      line = c[a] + (c[b] - c[a]) * (s[seg] - s[a]) / max(s[b] - s[a], 1e-9)
      err = np.abs(c[seg] - line)
      tol = tol_scale * np.maximum(PROFILE_TOL, PROFILE_REL * c[seg])
      m = int(np.argmax(err - tol))
      if err[m] > tol[m]:
        keep[a + 1 + m] = True
      inner = [a] + [a + 1 + q for q in np.nonzero(keep[a + 1:b])[0].tolist()] + [b]
      if len(inner) > 2:
        stack.extend(zip(inner, inner[1:], strict=False))
    if keep.sum() <= PROFILE_MAX_POINTS or tol_scale > 64:
      break
    tol_scale *= 1.5
  return [[round(float(s[q]), 1), round(1.0 / float(c[q]), 1)] for q in np.nonzero(keep)[0]]


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
  __slots__ = ("id", "name", "ref", "cls", "speed", "advisory", "oneway", "lat", "lon", "start", "end", "tag")


def key(lat, lon):
  """One int per node position (1e-7 deg, about 1 cm): ways that share a node share the key."""
  return int(round(float(lat) * 1e7)) * KEY_SCALE + int(round(float(lon) * 1e7))


def node_keys(lat, lon):
  """key() of every node, as an int64 array (the same numbers: both round the same double, ties to even)."""
  la = np.rint(np.asarray(lat, dtype=np.float64) * 1e7).astype(np.int64)
  lo = np.rint(np.asarray(lon, dtype=np.float64) * 1e7).astype(np.int64)
  return la * KEY_SCALE + lo


def keys_in(keys, sorted_keys):
  """Positions in `keys` (int64 array) whose value is in `sorted_keys` (sorted int64 array)."""
  if len(sorted_keys) == 0 or len(keys) == 0:
    return np.zeros(0, dtype=np.int64)
  pos = np.searchsorted(sorted_keys, keys)
  pos[pos >= len(sorted_keys)] = 0
  return np.nonzero(sorted_keys[pos] == keys)[0]


# The tagged speed limit (format 6). 7.0's tile maker stores maxspeed in m/s: "45 mph" as 45 x 0.44704, a number
# without a unit or with km/h as N x 0.277778 (mapd maps/generate_offline.go ParseMaxSpeed; knots and anything it
# cannot parse: 0). The code keeps the tag's own unit and whole number: N for N mph, -N for N km/h, 0 untagged (or not
# a whole mph / km/h, e.g. knots). Never the class default (DEFAULT_SPEED_MPH is only for the curves' keep test).
KPH_TILE = 0.277778         # the tile maker's km/h factor (1 / 3.6 = 0.2777778 reads the same within TAG_TOL)
TAG_TOL = 0.005             # whole units


def tag_code(mps):
  """The limit code (above) of a tile's maxSpeed value in m/s."""
  try:
    v = float(mps)
  except (TypeError, ValueError):
    return 0
  if not math.isfinite(v) or v <= 0:
    return 0
  best = None
  for unit, sign in ((MPH, 1), (KPH_TILE, -1)):
    n = v / unit
    err = abs(n - round(n))
    if err <= TAG_TOL and 1 <= round(n) <= 200 and (best is None or err < best[0]):
      best = (err, sign * int(round(n)))
  return best[1] if best else 0


def code_mps(code):
  """m/s of a limit code (0: None)."""
  code = int(code or 0)
  if code == 0:
    return None
  return code * MPH if code > 0 else -code / 3.6


def way_tags(max_speed, forward, backward):
  """(forward code, backward code) of a way: the direction's own tag (maxspeed:forward / :backward) when set, else
  maxspeed (mapd's CurrentWay._maxSpeed rule; the conditional tags are not read)."""
  base = tag_code(max_speed)
  f, b = tag_code(forward), tag_code(backward)
  return (f or base, b or base)


def _make_way(meta, lat, lon):
  wy = Way()
  wy.id, wy.name, wy.ref, wy.cls, speed, advisory, wy.oneway = meta[:7]
  wy.tag = meta[7] if len(meta) > 7 else (0, 0)
  wy.speed = speed if speed > 0 else DEFAULT_SPEED_MPH[wy.cls] * MPH
  wy.advisory = advisory if advisory > 0 else 0.0
  wy.lat, wy.lon = lat, lon
  wy.start, wy.end = key(lat[0], lon[0]), key(lat[-1], lon[-1])
  return wy


def link_end_keys(ways):
  """The nodes ramps (link roads) start and end on: the only nodes the ramp records look up (connected())."""
  out = set()
  for w in ways.values():
    if w.cls in LINK_CLASSES:
      out.add(w.start)
      out.add(w.end)
  return out


def load_ways(schema, files):
  """Way objects for the roads the curves use. Roads of SKIP_CLASSES (residential streets) are only kept when a ramp
  ends on one of their nodes (the ramp records need the road a ramp leads to); nothing else uses them. Memory: the
  nodes of a tile are one flat array and each Way's lat/lon is a view into it. A metro folder (Detroit, 2 x 2 deg) has
  204k roads with 2M nodes, 60 % residential: a Way object per road with its own arrays took 240 MB, this about 60."""
  wanted = set()        # nodes ramps start or end on (first pass: cheap, the tiles are read lazily)
  for f in files:
    t = read_tile(schema, f)
    if t is None:
      continue
    for w in t.ways:
      if w.highwayClass in LINK_CLASSES and len(w.nodes) >= 2:
        nodes = w.nodes
        wanted.add(key(nodes[0].latitude, nodes[0].longitude))
        wanted.add(key(nodes[-1].latitude, nodes[-1].longitude))
    del t
  wanted_arr = np.array(sorted(wanted), dtype=np.int64)
  ways, seen = {}, set()
  for f in files:
    t = read_tile(schema, f)
    if t is None:
      continue
    la, lo, pend = [], [], []          # nodes of the ways kept, and (meta, start, count) per way
    rla, rlo, rpend = [], [], []       # skipped classes: only to find the nodes a ramp ends on
    for w in t.ways:
      cls = w.highwayClass
      if w.id in seen or cls not in CLASS_NAMES or len(w.nodes) < 2:
        continue
      nodes = w.nodes
      meta = (w.id, sys.intern(w.name), sys.intern(w.ref), cls, w.maxSpeed, w.advisorySpeed, bool(w.oneWay),
              way_tags(w.maxSpeed, w.maxSpeedForward, w.maxSpeedBackward))
      if cls in SKIP_CLASSES:
        if len(wanted_arr):
          rpend.append((meta, len(rla), len(nodes)))
          rla.extend(n.latitude for n in nodes)
          rlo.extend(n.longitude for n in nodes)
      else:
        seen.add(w.id)
        pend.append((meta, len(la), len(nodes)))
        la.extend(n.latitude for n in nodes)
        lo.extend(n.longitude for n in nodes)
    if rpend:
      hits = keys_in(node_keys(rla, rlo), wanted_arr)
      if len(hits):
        starts = np.array([s for _, s, _ in rpend], dtype=np.int64)
        for k in sorted(set((np.searchsorted(starts, hits, side="right") - 1).tolist())):
          meta, s, n = rpend[k]
          if meta[0] not in seen:
            seen.add(meta[0])
            pend.append((meta, len(la), n))
            la.extend(rla[s:s + n])
            lo.extend(rlo[s:s + n])
    del rla, rlo, rpend
    lat_arr, lon_arr = np.array(la, dtype=np.float64), np.array(lo, dtype=np.float64)
    del la, lo
    for meta, s, n in pend:
      ways[meta[0]] = _make_way(meta, lat_arr[s:s + n], lon_arr[s:s + n])
    del t
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
  ways = {k: w for k, w in ways.items() if w.cls not in SKIP_CLASSES}
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

def chain_record(chain):
  """A road as the car follows it: [[way id, 1 if the chain runs in the way's node order, start (m)], ...]."""
  out, s0 = [], 0.0
  for w, rev in chain:
    x, y = to_xy(w.lat, w.lon, float(w.lat[0]), float(w.lon[0]))
    out.append([int(w.id), 0 if rev else 1, round(s0, 1)])
    s0 += float(np.sum(np.hypot(np.diff(x), np.diff(y))))
  return {"id": int(chain[0][0].id), "w": out, "len": round(s0, 1)}


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


def effective_radius(r_map, turn_deg, slack=KINK_SLACK):
  """Radius a car actually needs for a bend: a short sharp bend (two map pieces meeting at an angle, e.g. Dobry
  Drive: 17 deg over 34 m reads as 99 m, the car measured ~370 m) can be cut within the lane. A circle tangent to
  both sides of a corner of `turn_deg` that cuts it by `slack` m has radius slack / (1/cos(turn/2) - 1); a real
  curve (long arc) is already wider than that and keeps its own radius."""
  half = math.radians(min(max(turn_deg, 0.1), 170.0)) / 2
  return max(r_map, slack / (1.0 / math.cos(half) - 1.0))


def find_curves(chain, ways_by_id, ramp=None):
  """Curves along one chain. `ramp`: the chain's ramp record (link roads), added to each curve with the
  distance from the curve's end to the end of the ramp in the direction of travel."""
  fwd = {w.id: not rev for w, rev in chain}
  lat, lon, wids = chain_geometry(chain)
  if len(lat) < 3:
    return []
  lat0, lon0 = float(lat.mean()), float(lon.mean())
  x, y = to_xy(lat, lon, lat0, lon0)
  # cumulative distance at the original points, to map resampled points back to ways
  s_pts = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
  sx, sy = spline(x, y)
  rx, ry, rs = resample(sx, sy, STEP)
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
      r_map = 1.0 / abs(k[apex])
      r_min = effective_radius(r_map, turn)
      # the ways the curve lies on
      wset = sorted({int(wids[min(np.searchsorted(s_pts, rs[m]), len(wids) - 1)]) for m in range(i, j + 1, 5)} |
                    {int(wids[min(np.searchsorted(s_pts, rs[j]), len(wids) - 1)])})
      wl = [ways_by_id[w] for w in wset if w in ways_by_id]
      speed = min(w.speed for w in wl) if wl else 0.0
      adv = min([w.advisory for w in wl if w.advisory > 0] or [0.0])
      cls = min(w.cls for w in wl) if wl else 0
      need = math.sqrt(KEEP_LAT * r_min)
      if adv > 0:
        need = min(need, adv + ADVISORY_OVER)   # a signed curve is kept even if its shape alone looks easy
      if need < speed + KEEP_OVER:
        def pt(m):
          la, lo = to_latlon(float(rx[m]), float(ry[m]), lat0, lon0)
          return round(la, 6), round(lo, 6)
        h_in = bearing_deg(rx[min(i + 1, n - 1)] - rx[i], ry[min(i + 1, n - 1)] - ry[i])
        h_out = bearing_deg(rx[j] - rx[max(j - 1, 0)], ry[j] - ry[max(j - 1, 0)])
        name = next((w.name or w.ref for w in wl if w.name or w.ref), "")
        c = {
          "a": pt(i), "a_hdg": round(h_in, 1),            # entry when travelling a -> b
          "b": pt(j), "b_hdg": round((h_out + 180) % 360, 1),  # entry when travelling b -> a
          "apex": pt(apex), "r": round(r_min, 1), "r_map": round(r_map, 1), "len": round(length, 1), "turn": round(turn, 1),
          "dir": "L" if sign > 0 else "R",                 # turning direction when travelling a -> b
          "speed": round(speed, 2), "adv": round(adv, 2), "cls": cls, "name": name,
          "oneway": any(w.oneway for w in wl), "ways": wset,
          "v": {str(L): round(math.sqrt(L * r_min), 2) for L in LAT_LEVELS},
          # travelling a -> b follows each way's node order when fwd is 1 (FrogPilot's map match: is_forward)
          "wdir": [[w, 1 if fwd.get(w, True) else 0] for w in wset],
          # the distance profile along a -> b (m from a, radius m); r_min is its floor (short sharp bends)
          "prof": profile(rs[seg] - rs[i], k[seg], r_floor=r_min if r_min > r_map else 0.0),
        }
        c["chain"] = int(chain[0][0].id)
        c["s_a"], c["s_b"] = round(float(rs[i]), 1), round(float(rs[j]), 1)   # along the chain, from its start
        if ramp is not None:
          total = float(rs[-1])
          end_dist = total - float(rs[j]) if ramp["travel"] != "ba" else float(rs[i])
          c["ramp"] = {"type": ramp["type"], "travel": ramp["travel"], "to": ramp["to"], "from": ramp["from"],
                       "end_dist": round(end_dist, 1), "len": ramp["len"]}
        curves.append(c)
    i = j + 1
  return split_doublets(curves, ways_by_id)


# Round Q (Rochester Rd construction, drives 74 / 75: a road splitting into its two one-way sides): two opposite bends
# within SPLIT_PAIR_GAP whose heading changes cancel (net under SPLIT_NET_DEG), where a one-way way meets a two-way way of
# the same name, are the jog onto a carriageway, not curves: dropped
SPLIT_PAIR_GAP = 60.0      # m between the first's exit and the second's entry
SPLIT_NET_DEG = 8.0        # deg
# (review of round Q, S5) ... each bend at most SPLIT_BEND_MAX and the road moved sideways at most SPLIT_OFFSET_MAX (the
# Rochester Rd case: ~23-26 deg, ~9 m): a real S-bend (70 / 65 deg) is kept
SPLIT_BEND_MAX = 30.0      # deg
SPLIT_OFFSET_MAX = 12.0    # m


def _sideways(c1, c2):
  """m: how far the road after c2's exit lies beside the line through c1's entry along its heading."""
  (la1, lo1), (la2, lo2) = c1["a"], c2["b"]
  dy = (la2 - la1) * 111195.0
  dx = (lo2 - lo1) * 111195.0 * math.cos(math.radians(la1))
  h = math.radians(c1["a_hdg"])
  return abs(dx * math.cos(h) - dy * math.sin(h))


def split_doublets(curves, ways_by_id):
  """curves of one chain (along it) without the split jogs (SPLIT_*)."""
  drop = set()
  for k in range(len(curves) - 1):
    c1, c2 = curves[k], curves[k + 1]
    if c1["dir"] == c2["dir"] or c2["s_a"] - c1["s_b"] > SPLIT_PAIR_GAP or abs(c1["turn"] - c2["turn"]) >= SPLIT_NET_DEG:
      continue
    if max(c1["turn"], c2["turn"]) > SPLIT_BEND_MAX or _sideways(c1, c2) > SPLIT_OFFSET_MAX:
      continue
    wl = [ways_by_id[w] for w in sorted(set(c1["ways"]) | set(c2["ways"])) if w in ways_by_id]
    names = {(w.name or w.ref) for w in wl if (w.name or w.ref)}
    if len(names) == 1 and any(w.oneway for w in wl) and any(not w.oneway for w in wl):
      drop.update((k, k + 1))
  return [c for k, c in enumerate(curves) if k not in drop]


# ---------------- ramps ----------------

HIGHWAY_CLASSES = {1, 3}
LINK_FOLLOW_HOPS = 4
STREET_CLASSES = (5, 7, 9, 11, 12)   # primary .. residential: a link joint on one of these is a junction (map_view)
HEADING_SPAN = 10.0                  # m along a road from the node where its travel heading ("h") is measured


def node_index(ways, only=None):
  """Node -> [(way, position)], for all classes (a ramp may end on a road's interior node). With `only` (a set of
  node keys) just those nodes are indexed: the ramp records only ever look up link_end_keys(ways), and the full
  index of a metro folder (2M nodes) took 500 MB."""
  idx = {}
  if only is None:
    for w in ways.values():
      for m in range(len(w.lat)):
        idx.setdefault(key(w.lat[m], w.lon[m]), []).append((w, m))
    return idx
  only_arr = np.array(sorted(only), dtype=np.int64)
  for w in ways.values():
    ks = node_keys(w.lat, w.lon)
    for m in keys_in(ks, only_arr).tolist():
      idx.setdefault(int(ks[m]), []).append((w, m))
  return idx


def road_info(w, m=None):
  """The road a ramp leads to / comes from. With m (the node's position on w, found at the link's own end) and w
  one-way: "h", its travel heading there (deg; format 6: the turnaround test without a shape search)."""
  out = {"cls": w.cls, "speed": round(w.speed, 2), "name": w.name or w.ref}
  if m is not None and w.oneway:
    h = heading_at(w, m)
    if h is not None:
      out["h"] = round(h)
  return out


def heading_at(w, m):
  """Travel heading (deg, node order) of way w at its node m: towards the first node HEADING_SPAN on (or from the
  one HEADING_SPAN back at the way's last node), or None (a way too short)."""
  x, y = to_xy(w.lat, w.lon, float(w.lat[m]), float(w.lon[m]))
  n = len(x)
  if m < n - 1:
    j = m + 1
    while j < n - 1 and math.hypot(x[j] - x[m], y[j] - y[m]) < HEADING_SPAN:
      j += 1
    dx, dy = x[j] - x[m], y[j] - y[m]
  else:
    j = m - 1
    while j > 0 and math.hypot(x[m] - x[j], y[m] - y[j]) < HEADING_SPAN:
      j -= 1
    dx, dy = x[m] - x[j], y[m] - y[j]
  if math.hypot(dx, dy) < 0.5:
    return None
  return bearing_deg(float(dx), float(dy))


def connected(nidx, node, exclude, ways, leaving, hops=LINK_FOLLOW_HOPS):
  """The road reached at `node`: the most important non-link road through it (motorway first). When only
  ramps continue there, follow them (in their direction of travel, up to `hops` ramps) to where they lead.
  leaving=True: roads we can continue onto from this node (destination); False: roads arriving (origin)."""
  best = best_m = None
  links = []
  for o, m in nidx.get(node, []):
    if o.id in exclude:
      continue
    if o.cls in LINK_CLASSES:
      last = len(o.lat) - 1
      if leaving and (m == 0 or not o.oneway):
        links.append((o, o.end if m == 0 else (o.start if m == last else None)))
      elif not leaving and (m == last or not o.oneway):
        links.append((o, o.start if m == last else (o.end if m == 0 else None)))
      continue
    if best is None or o.cls < best.cls:
      best, best_m = o, m
  if best is not None:
    return road_info(best, best_m if hops == LINK_FOLLOW_HOPS else None)
  if hops > 0:
    for o, far in links:
      if far is None:
        continue
      r = connected(nidx, far, exclude | {o.id}, ways, leaving, hops - 1)
      if r is not None:
        return r
  return None


def ramp_record(chain, nidx, ways):
  if not all(w.cls in LINK_CLASSES for w, _ in chain):
    return None
  ids = {w.id for w, _ in chain}
  oneway = all(w.oneway for w, _ in chain)
  fwds = [not rev for _, rev in chain]
  travel = ("ab" if all(fwds) else "ba" if not any(fwds) else "both") if oneway else "both"
  first, last = chain[0], chain[-1]
  node_a = first[0].end if first[1] else first[0].start
  node_b = last[0].start if last[1] else last[0].end
  lat, lon, _ = chain_geometry(chain)
  x, y = to_xy(lat, lon, float(lat.mean()), float(lon.mean()))
  length = float(np.sum(np.hypot(np.diff(x), np.diff(y))))
  start_node, end_node = (node_b, node_a) if travel == "ba" else (node_a, node_b)
  to = connected(nidx, end_node, ids, ways, leaving=True)
  frm = connected(nidx, start_node, ids, ways, leaving=False)
  if travel == "both":
    kind = "two-way"
  elif to and frm and to["cls"] in HIGHWAY_CLASSES and frm["cls"] in HIGHWAY_CLASSES:
    kind = "interchange"
  elif to and to["cls"] in HIGHWAY_CLASSES:
    kind = "entrance"
  elif frm and frm["cls"] in HIGHWAY_CLASSES:
    kind = "exit"
  else:
    kind = "link"
  # the joints between the link's ways (chain order: k = the joint after the k-th way) where a street shares the NODE:
  # an at-grade junction (a bridge over a street shares none). [[k, the most important street class there], ...]
  joints = []
  for k in range(len(chain) - 1):
    w, rev = chain[k]
    node = w.start if rev else w.end
    cls = min((o.cls for o, _m in nidx.get(node, []) if o.id not in ids and o.cls in STREET_CLASSES), default=0)
    if cls:
      joints.append([k, int(cls)])
  s_pt = (round(float(lat[0]), 6), round(float(lon[0]), 6))
  e_pt = (round(float(lat[-1]), 6), round(float(lon[-1]), 6))
  return {"type": kind, "travel": travel, "len": round(length, 1), "to": to, "from": frm,
          "start": e_pt if travel == "ba" else s_pt, "end": s_pt if travel == "ba" else e_pt,
          "wdir": [[w.id, 0 if rev else 1] for w, rev in chain], "cls": min(w.cls for w, _ in chain),
          "chain": int(chain[0][0].id), "j": joints,
          "name": next((w.name or w.ref for w, _ in chain if w.name or w.ref), "")}


def simplify(x, y, tol):
  """Indices of the points kept by Douglas-Peucker with tolerance `tol` (m); first and last always kept."""
  n = len(x)
  if n <= 2:
    return list(range(n))
  keep = [False] * n
  keep[0] = keep[-1] = True
  stack = [(0, n - 1)]
  while stack:
    a, b = stack.pop()
    if b - a < 2:
      continue
    dx, dy = x[b] - x[a], y[b] - y[a]
    seg = math.hypot(dx, dy)
    best, best_d = None, tol
    for i in range(a + 1, b):
      if seg < 1e-6:
        d = math.hypot(x[i] - x[a], y[i] - y[a])
      else:
        d = abs(dy * (x[i] - x[a]) - dx * (y[i] - y[a])) / seg
      if d > best_d:
        best, best_d = i, d
    if best is not None:
      keep[best] = True
      stack.append((a, best))
      stack.append((best, b))
  return [i for i in range(n) if keep[i]]


def limit_field(w):
  """The road shape's "l" (format 6): the way's tagged limit code (tag_code) for both directions, or [forward,
  backward] when a two-way way's directions differ (0: that direction untagged); None (no field): untagged."""
  f, b = getattr(w, "tag", None) or (0, 0)
  if w.oneway:
    b = f
  if not f and not b:
    return None
  return int(f) if f == b else [int(f), int(b)]


def road_shapes(ways, chain_of=None):
  """Road shapes for matching the car to a road: {tile name: [{"w": way id, "c": chain id, "k": road class,
  "o": 1 if one-way, "l": tagged limit (limit_field; absent: untagged), "p": [lat0, lon0, dlat, dlon, ...]}]} with
  each way in its OSM node order (so the direction of travel gives
  FrogPilot's is_forward), simplified to SHAPE_TOLERANCE, in micro-degrees (first point absolute, then
  differences), split by 0.25 deg square with one point of overlap. All classes the chains use (no residential
  streets). The chain id (the road the way belongs to) lets the car stay on a road across its way splits."""
  chain_of = chain_of or {}
  out = {}
  for w in ways.values():
    if w.cls in SKIP_CLASSES or len(w.lat) < 2:
      continue
    x, y = to_xy(w.lat, w.lon, float(w.lat[0]), float(w.lon[0]))
    idx = simplify(list(map(float, x)), list(map(float, y)), SHAPE_TOLERANCE)
    pts = [(int(round(float(w.lat[i]) * 1e6)), int(round(float(w.lon[i]) * 1e6))) for i in idx]
    runs = {}
    for k in range(len(pts) - 1):
      for t in {tile_of(pts[k][0] / 1e6, pts[k][1] / 1e6), tile_of(pts[k + 1][0] / 1e6, pts[k + 1][1] / 1e6)}:
        lst = runs.setdefault(t, [])
        if lst and lst[-1][-1] == k:
          lst[-1].append(k + 1)
        else:
          lst.append([k, k + 1])
    for t, lst in runs.items():
      for run in lst:
        flat = [pts[run[0]][0], pts[run[0]][1]]
        for a, b in zip(run, run[1:], strict=False):
          flat += [pts[b][0] - pts[a][0], pts[b][1] - pts[a][1]]
        rec = {"w": int(w.id), "c": int(chain_of.get(w.id, w.id)), "k": int(w.cls), "p": flat}
        if w.oneway:
          rec["o"] = 1
        lim = limit_field(w)
        if lim is not None:
          rec["l"] = lim
        out.setdefault(t, []).append(rec)
  return out


def tile_of(lat, lon):
  la = math.floor(lat * 4) / 4
  lo = math.floor(lon * 4) / 4
  return f"{la:.6f}_{lo:.6f}_{la + 0.25:.6f}_{lo + 0.25:.6f}"


def tiles_of_way(w):
  """The 0.25 deg squares a way's road shape can fall in (from its bounding box, in the micro-degrees the shape
  is written in, so a point rounded across a square line still finds its square)."""
  la = np.rint(w.lat * 1e6) / 1e6
  lo = np.rint(w.lon * 1e6) / 1e6
  la0, la1 = math.floor(float(la.min()) * 4), math.floor(float(la.max()) * 4)
  lo0, lo1 = math.floor(float(lo.min()) * 4), math.floor(float(lo.max()) * 4)
  return [tile_of(a / 4 + 0.001, o / 4 + 0.001) for a in range(la0, la1 + 1) for o in range(lo0, lo1 + 1)]


def write_json_atomic(path, data):
  """Write through a temporary name and rename: a reader (or a kill) never sees a half-written file."""
  tmp = path + ".tmp"
  with open(tmp, "w") as f:
    json.dump(data, f, separators=(",", ":"))
    f.flush()
    os.fsync(f.fileno())          # (a power cut before the data reached the disk left 0-byte files)
  os.replace(tmp, path)
  try:
    fd = os.open(os.path.dirname(path) or ".", os.O_RDONLY)
    try:
      os.fsync(fd)
    finally:
      os.close(fd)
  except OSError:
    pass


def folder_box(path):
  """(lat0, lon0) of a 2 x 2 degree folder from its '<lat>/<lon>' path end, or None."""
  try:
    la, lo = (int(p) for p in os.path.normpath(path).split(os.sep)[-2:])
  except ValueError:
    return None
  return (la, lo) if la % 2 == 0 and lo % 2 == 0 else None


def in_box(name, box):
  """A square's file belongs to the folder whose 2 x 2 degree box holds the square."""
  if box is None:
    return True
  la, lo = (float(v) for v in name.split('_')[:2])
  return box[0] <= la + 0.125 < box[0] + 2 and box[1] <= lo + 0.125 < box[1] + 2


def build(tiles_dir, out_dir, schema_path=None, quiet=False, progress=None, extra=None, box="auto"):
  """Build the curve files of one map folder into out_dir, one square at a time (each written atomically).
  progress(square name, bytes written, seconds since the start) is called after each file. `extra`: more fields for
  every file (the device adds when it built them). box: the folder's (lat0, lon0) ("auto": from tiles_dir's
  '<lat>/<lon>' end; None: no filter): squares outside it are not written (they belong to the next folder, which
  writes them itself). Returns the counts as a dict (see the end)."""
  box = folder_box(tiles_dir) if box == "auto" else box
  schema = load_schema(schema_path)
  files = sorted(f for f in glob.glob(os.path.join(tiles_dir, "*")) if os.path.isfile(f))
  if not files:
    raise FileNotFoundError(f"no tile files in {tiles_dir}")
  t0 = time.monotonic()
  ways = load_ways(schema, files)
  t_load = time.monotonic() - t0
  chains = build_chains(ways)
  nidx = node_index(ways, link_end_keys(ways))
  curves, ramps, chain_recs = [], [], {}
  for ch in chains:
    ramp = ramp_record(ch, nidx, ways)
    if ramp is not None:
      ramps.append(ramp)
    found = find_curves(ch, ways, ramp)
    curves.extend(found)
    if found or ramp is not None:
      rec = chain_record(ch)
      chain_recs[rec["id"]] = rec
  del nidx
  t_curves = time.monotonic() - t0
  # one file per 0.25 deg square: curves by their apex, ramps by their start, road shapes where they lie
  by_tile = {}
  def tile(name):
    return by_tile.setdefault(name, {"curves": [], "ramps": [], "chains": set()})
  for c in curves:
    t = tile(tile_of(*c["apex"]))
    t["curves"].append(c)
    t["chains"].add(c["chain"])
  for r in ramps:
    t = tile(tile_of(*r["start"]))
    t["ramps"].append(r)
    t["chains"].add(r["chain"])
  chain_of = {w.id: int(ch[0][0].id) for ch in chains for w, _rev in ch}
  n_chains = len(chains)
  del chains
  # the road shapes are made square by square (all at once they took more memory than the roads themselves)
  ways_of_tile = {}
  for w in ways.values():
    if w.cls in SKIP_CLASSES or len(w.lat) < 2:
      continue
    for name in tiles_of_way(w):
      ways_of_tile.setdefault(name, []).append(w.id)
  os.makedirs(out_dir, exist_ok=True)
  written, size = [], 0
  outside = 0
  for name in sorted(set(by_tile) | set(ways_of_tile)):
    if not in_box(name, box):
      outside += 1
      continue
    d = by_tile.get(name, {"curves": [], "ramps": [], "chains": set()})
    roads = road_shapes({wid: ways[wid] for wid in ways_of_tile.get(name, [])}, chain_of).get(name, [])
    if not (d["curves"] or d["ramps"] or roads):
      continue
    data = {"version": FORMAT_VERSION, "builder": BUILDER_VERSION, "keep_lat": KEEP_LAT, "curves": d["curves"], "ramps": d["ramps"],
            "chains": [chain_recs[c] for c in sorted(d["chains"]) if c in chain_recs], "roads": roads}
    if extra:
      data.update(extra)
    path = os.path.join(out_dir, name + ".json")
    write_json_atomic(path, data)
    n = os.path.getsize(path)
    size += n
    written.append(name)
    if progress is not None:
      progress(name, n, time.monotonic() - t0)
  kinds = {}
  for r in ramps:
    kinds[r["type"]] = kinds.get(r["type"], 0) + 1
  tagged = sum(1 for w in ways.values() if w.cls not in SKIP_CLASSES and limit_field(w) is not None)
  stats = {"tiles": len(files), "roads": len(ways), "chains": n_chains, "curves": len(curves), "ramps": len(ramps),
           "kinds": kinds, "files": written, "outside": outside, "bytes": size, "seconds": round(time.monotonic() - t0, 1),
           "load_s": round(t_load, 1), "curves_s": round(t_curves - t_load, 1), "tagged": tagged,
           "joints": sum(len(r["j"]) for r in ramps)}
  if not quiet:
    msg = f"{stats['tiles']} tiles, {stats['roads']} roads, {n_chains} chains, {stats['curves']} curves, {stats['ramps']} ramps {kinds}, "
    msg += f"{len(written)} output files, {size / 1e6:.2f} MB, {stats['seconds']:.0f} s (load {stats['load_s']:.0f}, curves {stats['curves_s']:.0f})"
    print(msg)
  return stats



def folders_for(lat, lon, miles):
  """The 2 x 2 degree map folders (e.g. '42/-84') that cover `miles` around (lat, lon)."""
  dlat = miles * 1609.344 / 111195.0
  dlon = dlat / max(math.cos(math.radians(lat)), 0.1)
  out = []
  for la in range(int(math.floor((lat - dlat) / 2) * 2), int(math.floor((lat + dlat) / 2) * 2) + 1, 2):
    for lo in range(int(math.floor((lon - dlon) / 2) * 2), int(math.floor((lon + dlon) / 2) * 2) + 1, 2):
      out.append(f"{la}/{lo}")
  return out


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--tiles", required=True, help="one 2 x 2 degree folder of offline map tiles")
  p.add_argument("--out", required=True, help="output folder for the curve files")
  p.add_argument("--schema", help="tile schema (default: offline_tiles.capnp next to this file)")
  a = p.parse_args()
  try:
    build(a.tiles, a.out, a.schema)
  except FileNotFoundError as e:
    sys.exit(str(e))


if __name__ == "__main__":
  main()
