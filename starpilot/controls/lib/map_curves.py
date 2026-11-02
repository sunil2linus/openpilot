"""
Map curves: the curve (or ramp) ahead from the curve files made by tools/curve_map/build_curve_map.py.

Runs in the FrogPilot planner (frogpilot_process) at the planner rate, and publishes the governing curve
ahead in frogpilotPlan; the Mazda car controller turns it into a set speed target with its own lateral
limit and acts on it when the setting is On (MazdaMapCurves: 0 off, 1 test, 2 on).

  * Files: /data/media/0/curve_map/<lat>/<lon>/<0.25 deg square>.json. The squares around the car are
    loaded in a background thread (never in the planner loop); curves, ramps and road chains of all loaded
    squares are merged (the same curve from two folders is kept once).
  * Where we are: the road (way id) and the direction along it (is_forward). FrogPilot's map program
    publishes this (MapMatchedWay); where it does not (StarPilot's newer map program), the road shapes in
    the curve files (format 4) are matched here (RoadMatcher). The road chain that way belongs to gives every
    curve on the road ahead, in order, also on later ways of the same road. Distances are straight lines from
    the GPS position (a little short on a winding road, i.e. on the early side).
  * Which curve: of the curves within LOOKAHEAD ahead (and the one we are in), the one that needs the
    lowest speed now with a gentle slowdown (PLAN_DECEL) to reach its speed PLAN_MARGIN before its entry.
  * Ramps: when the road is a ramp (exit / entrance / interchange) we know where it leads: the road class,
    its speed limit and the distance to the end of the ramp.
  * Navigation: when the route says an exit ("off ramp" / "fork") is coming, the exit ramp is matched by
    distance and direction, so its curves are known before the split (used for the on-screen notice and
    chime, and, with the turn signal on, to start early).

No openpilot imports at module level (MapCurveStore / MapCurves are unit tested offline).
"""
import glob
import json
import math
import os
import threading
from array import array

CURVE_DIR = "/data/media/0/curve_map"
MPH = 0.44704
EARTH_R = 6371008.8

LOOKAHEAD = 1000.0        # m along the road
PLAN_LAT = 3.0            # m/s^2: only to choose the governing curve; the car uses its own limit
PLAN_DECEL = 0.6          # m/s^2: the slowdown the stock cruise gives after a set speed drop (0.6-0.7 measured)
PLAN_MARGIN = 30.0        # m before the entry where the curve speed should be reached
ADVISORY_OVER = 10 * 0.44704   # m/s: a curve with a yellow advisory sign in the map is never planned faster than sign + this
AHEAD_COS = 0.2           # a point is ahead when it is within ~78 deg of our heading
NEAR = 15.0               # m
TILE_KEEP = 25            # squares kept in memory
MATCH_DIST = 20.0         # m: the car is on a road shape within this distance (GPS error + road width)
MATCH_HEADING = 35.0      # deg: and heading along it (either direction) within this
MATCH_KEEP = 8.0          # m: another road must be this much closer to take over from the matched one
MATCH_CELL = 500          # grid cells of 1/500 deg (~220 m) for the road shape index
NAV_EXIT_TYPES = ("off ramp", "fork")
NAV_EXIT_RANGE = 1500.0   # m: exits matched from this far
NAV_MATCH_TOL = 0.25      # match tolerance as a share of the announced distance (at least NAV_MATCH_MIN)
NAV_MATCH_MIN = 150.0
HIGHWAY_CLASSES = (1, 3)
RAMP_TYPES = {"exit": 1, "entrance": 2, "interchange": 3, "link": 4, "two-way": 4}

# frogpilotPlan.mapCurveStatus
STATUS_NONE, STATUS_CURVE, STATUS_RAMP_CURVE, STATUS_NO_DATA = 0, 1, 2, 3
# frogpilotPlan.mapNavExit
NAV_NONE, NAV_EXIT_CURVES, NAV_EXIT_NO_CURVES, NAV_EXIT_NO_DATA = 0, 1, 2, 3


def nav_side(modifier):
  """Side of a navigation maneuver from its modifier ("slight right", "left", ...): -1 left, 1 right, 0 unknown."""
  m = (modifier or "").lower()
  return 1 if "right" in m else (-1 if "left" in m else 0)


def tile_name(lat, lon):
  la = math.floor(lat * 4) / 4
  lo = math.floor(lon * 4) / 4
  return f"{la:.6f}_{lo:.6f}_{la + 0.25:.6f}_{lo + 0.25:.6f}"


def tiles_around(lat, lon):
  return [tile_name(lat + dla * 0.25, lon + dlo * 0.25) for dla in (-1, 0, 1) for dlo in (-1, 0, 1)]


def dist_m(a_lat, a_lon, b_lat, b_lon):
  x = math.radians(b_lon - a_lon) * EARTH_R * math.cos(math.radians((a_lat + b_lat) / 2))
  y = math.radians(b_lat - a_lat) * EARTH_R
  return math.hypot(x, y)


def bearing_to(a_lat, a_lon, b_lat, b_lon):
  x = math.radians(b_lon - a_lon) * math.cos(math.radians((a_lat + b_lat) / 2))
  y = math.radians(b_lat - a_lat)
  return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def ahead(lat, lon, bearing, p_lat, p_lon):
  d = dist_m(lat, lon, p_lat, p_lon)
  if d < NEAR:
    return True, d
  return math.cos(math.radians(bearing_to(lat, lon, p_lat, p_lon) - bearing)) > AHEAD_COS, d


def curve_speed_limit(radius, advisory=0.0, lat_limit=PLAN_LAT):
  """Speed (m/s) a curve allows: its shape at `lat_limit`, and at most its advisory sign + ADVISORY_OVER."""
  v = math.sqrt(lat_limit * radius)
  return min(v, advisory + ADVISORY_OVER) if advisory > 0 else v


def plan_speed(radius, distance, advisory=0.0, lat_limit=PLAN_LAT, decel=PLAN_DECEL, margin=PLAN_MARGIN):
  """Highest speed (m/s) from which a `decel` slowdown reaches the curve speed `margin` m before its entry."""
  v_curve = curve_speed_limit(radius, advisory, lat_limit)
  return math.sqrt(v_curve ** 2 + 2 * decel * max(distance - margin, 0.0))


def cell_key(cla, clo):
  return cla * 10_000_000 + clo     # one int per grid cell (|clo| < 180 * MATCH_CELL)


class RoadIndex:
  """Road shape segments of one square in compact arrays (micro-degrees), with a grid of cells -> segment numbers.
  About 1 MB per square instead of the ~10 MB the decoded JSON lists would take."""

  def __init__(self, shapes):
    self.lat1, self.lon1, self.lat2, self.lon2 = array("i"), array("i"), array("i"), array("i")
    self.way = array("q")
    cells = {}
    for shape in shapes:
      p = shape.get("p", [])
      if len(p) < 4:
        continue
      wid = int(shape.get("w", 0))
      lat, lon = p[0], p[1]
      for k in range(2, len(p) - 1, 2):
        la2, lo2 = lat + p[k], lon + p[k + 1]
        i = len(self.way)
        self.lat1.append(lat)
        self.lon1.append(lon)
        self.lat2.append(la2)
        self.lon2.append(lo2)
        self.way.append(wid)
        n = max(1, int(dist_m(lat / 1e6, lon / 1e6, la2 / 1e6, lo2 / 1e6) / 100.0) + 1)
        seen = set()
        for j in range(n + 1):
          f = j / n
          c = cell_key(int(math.floor((lat + (la2 - lat) * f) * MATCH_CELL / 1e6)), int(math.floor((lon + (lo2 - lon) * f) * MATCH_CELL / 1e6)))
          if c not in seen:
            seen.add(c)
            cells.setdefault(c, array("i")).append(i)
        lat, lon = la2, lo2
    self.cells = cells

  def segments(self, cell):
    for i in self.cells.get(cell, ()):
      yield self.lat1[i] / 1e6, self.lon1[i] / 1e6, self.lat2[i] / 1e6, self.lon2[i] / 1e6, self.way[i]


class RoadMatcher:
  """The road (way id) the car is on and its direction along the way's node order (is_forward), from the
  road shapes in the curve files: the nearest shape segment within MATCH_DIST whose direction (either way)
  is within MATCH_HEADING of the car's heading. The road matched last keeps the match unless another is
  MATCH_KEEP closer (no flicker between a ramp and the main road at a split)."""

  def __init__(self, store):
    self.store = store
    self.way_id = 0
    self.is_forward = True

  def update(self, lat, lon, bearing):
    grid = self.store.grid
    if not grid or lat is None:
      self.way_id = 0
      return 0, True
    cla, clo = int(math.floor(lat * MATCH_CELL)), int(math.floor(lon * MATCH_CELL))
    kx = math.radians(1.0) * EARTH_R * math.cos(math.radians(lat))
    ky = math.radians(1.0) * EARTH_R
    best = None
    for dla in (-1, 0, 1):
      for dlo in (-1, 0, 1):
        for la1, lo1, la2, lo2, wid in (seg for ri in grid for seg in ri.segments(cell_key(cla + dla, clo + dlo))):
          ax, ay = (lo1 - lon) * kx, (la1 - lat) * ky
          bx, by = (lo2 - lon) * kx, (la2 - lat) * ky
          dx, dy = bx - ax, by - ay
          ll = dx * dx + dy * dy
          if ll < 1e-6:
            continue
          t = max(0.0, min(1.0, -(ax * dx + ay * dy) / ll))
          d = math.hypot(ax + t * dx, ay + t * dy)
          if d > MATCH_DIST:
            continue
          diff = abs((math.degrees(math.atan2(dx, dy)) - bearing + 180.0) % 360.0 - 180.0)
          if diff <= MATCH_HEADING:
            fwd = True
          elif diff >= 180.0 - MATCH_HEADING:
            fwd = False
          else:
            continue
          score = d - (MATCH_KEEP if wid == self.way_id else 0.0)
          if best is None or score < best[0]:
            best = (score, wid, fwd)
    if best is None:
      self.way_id = 0
      return 0, True
    self.way_id, self.is_forward = best[1], best[2]
    return self.way_id, self.is_forward


class MapCurveStore:
  """Curve files around the car, loaded in the background."""

  def __init__(self, root=CURVE_DIR, background=True):
    self.root = root
    self.background = background
    self.index = {}            # square name -> [paths]
    self.loaded = {}           # square name -> (curves, ramps, chains)
    self.order = []
    self.lock = threading.Lock()
    self.loading = False
    self.curves = {}           # chain id -> [curve] (sorted by s_a)
    self.ramps = {}            # chain id -> ramp
    self.ramps_all = []
    self.chains = {}           # chain id -> chain record
    self.way_chain = {}        # way id -> (chain id, index in the chain)
    self.grid = []             # RoadIndex of every loaded square that has road shapes
    self.wanted = []
    self.scan()

  def scan(self):
    self.index = {}
    for p in glob.glob(os.path.join(self.root, "*", "*", "*.json")):
      self.index.setdefault(os.path.basename(p)[:-5], []).append(p)

  @property
  def has_files(self):
    return bool(self.index)

  def covers(self, lat, lon):
    return tile_name(lat, lon) in self.index

  def update(self, lat, lon):
    want = [t for t in tiles_around(lat, lon) if t in self.index]
    if want == self.wanted or self.loading:
      return
    missing = [t for t in want if t not in self.loaded]
    self.wanted = want
    if not missing:
      return
    if self.background:
      self.loading = True
      threading.Thread(target=self._load, args=(missing, want), daemon=True).start()
    else:
      self._load(missing, want)

  def _load(self, names, want):
    try:
      new = {}
      for n in names:
        curves, ramps, chains, roads = [], [], [], []
        for p in self.index.get(n, []):
          try:
            with open(p) as f:
              d = json.load(f)
          except (OSError, ValueError):
            continue
          if d.get("version", 0) < 3:
            continue
          curves += d.get("curves", [])
          ramps += d.get("ramps", [])
          chains += d.get("chains", [])
          roads += d.get("roads", [])
        new[n] = (curves, ramps, chains, RoadIndex(roads) if roads else None)
      with self.lock:
        self.loaded.update(new)
        self.order = [t for t in self.order if t not in new] + list(new)
        keep = set(want) | set(self.order[-TILE_KEEP:])
        for t in list(self.loaded):
          if t not in keep:
            del self.loaded[t]
        self.order = [t for t in self.order if t in self.loaded]
        self._rebuild()
    finally:
      self.loading = False

  def _rebuild(self):
    curves, ramps, chains, seen = {}, {}, {}, set()
    grid = [ri for _cs, _rs, _chs, ri in self.loaded.values() if ri is not None]
    for cs, rs, chs, _ri in self.loaded.values():
      for ch in chs:
        if ch["id"] not in chains or len(ch["w"]) > len(chains[ch["id"]]["w"]):
          chains[ch["id"]] = ch
      for c in cs:
        k = (c.get("chain"), c.get("s_a"))
        if k in seen:
          continue
        seen.add(k)
        curves.setdefault(c.get("chain"), []).append(c)
      for r in rs:
        ramps[r.get("chain")] = r
    for lst in curves.values():
      lst.sort(key=lambda c: c["s_a"])
    way_chain = {}
    for cid, ch in chains.items():
      for k, (wid, _fwd, _s0) in enumerate(ch["w"]):
        if wid not in way_chain or len(ch["w"]) > len(chains[way_chain[wid][0]]["w"]):
          way_chain[wid] = (cid, k)
    self.curves, self.ramps, self.chains, self.way_chain, self.grid = curves, ramps, chains, way_chain, grid
    self.ramps_all = list(ramps.values())


class MapCurves:
  """The governing curve ahead, the ramp we are on, and the matched navigation exit."""

  def __init__(self, store=None):
    self.store = store if store is not None else MapCurveStore()
    self.reset()

  def reset(self):
    self.status = STATUS_NONE
    self.curve = None            # governing curve dict
    self.distance = 0.0          # m to its entry (0 = in it)
    self.radius = 0.0
    self.advisory = 0.0
    self.on_ramp = False
    self.ramp = None
    self.ramp_end_dist = 0.0
    self.nav_exit = NAV_NONE
    self.nav_exit_dist = 0.0
    self.nav_ramp = None
    self.nav_curve = None
    self.nav_curve_dist = 0.0
    self.nav_side = 0
    self.data_here = False
    self.chain_id = 0

  # ---- the road ahead ----
  def _road(self, way_id, is_forward):
    hit = self.store.way_chain.get(way_id)
    if hit is None:
      return None
    cid, k = hit
    ch = self.store.chains.get(cid)
    if ch is None:
      return None
    increasing = bool(ch["w"][k][1]) == bool(is_forward)
    s0 = ch["w"][k][2]
    s1 = ch["w"][k + 1][2] if k + 1 < len(ch["w"]) else ch["len"]
    return cid, increasing, s0, s1

  def _curves_ahead(self, cid, increasing, s0, s1, lat, lon, bearing):
    out = []
    for c in self.store.curves.get(cid, []):
      entry, exit_ = (c["a"], c["b"]) if increasing else (c["b"], c["a"])
      s_entry, s_exit = (c["s_a"], c["s_b"]) if increasing else (c["s_b"], c["s_a"])
      if increasing and (s_exit < s0 or s_entry > s1 + LOOKAHEAD):
        continue
      if not increasing and (s_exit > s1 or s_entry < s0 - LOOKAHEAD):
        continue
      on_this_way = (s0 <= s_entry <= s1)
      ent_ahead, d_entry = ahead(lat, lon, bearing, *entry)
      ex_ahead, d_exit = ahead(lat, lon, bearing, *exit_)
      later_way = (s_entry > s1) if increasing else (s_entry < s0)
      if later_way or (on_this_way and ent_ahead):
        if d_entry <= LOOKAHEAD:
          out.append((c, d_entry))
      elif ex_ahead and d_exit > NEAR and d_exit <= c["len"] + 60:
        out.append((c, 0.0))      # in the curve
    return out

  def update(self, lat, lon, bearing, way_id, is_forward, nav=None):
    """nav: (maneuver type, modifier, distance m) of the next maneuver, or None."""
    self.reset()
    if lat is None:
      return
    self.store.update(lat, lon)
    self.data_here = self.store.covers(lat, lon)
    road = self._road(way_id, is_forward) if way_id else None
    if road is not None:
      cid, increasing, s0, s1 = road
      self.chain_id = cid
      cands = self._curves_ahead(cid, increasing, s0, s1, lat, lon, bearing)
      if cands:
        c, d = min(cands, key=lambda cd: plan_speed(cd[0]["r"], cd[1], cd[0].get("adv", 0.0)))
        self.curve, self.distance, self.radius = c, d, c["r"]
        self.advisory = float(c.get("adv", 0.0) or 0.0)
        self.status = STATUS_RAMP_CURVE if c.get("ramp") else STATUS_CURVE
      ramp = self.store.ramps.get(cid)
      if ramp is not None and (ramp["travel"] == "both" or (ramp["travel"] == "ab") == increasing):
        self.on_ramp, self.ramp = True, ramp
        end = ramp["end"] if ramp["travel"] != "both" or increasing else ramp["start"]
        self.ramp_end_dist = dist_m(lat, lon, *end)
    if nav is not None:
      self._nav_exit(lat, lon, bearing, *nav)

  def _nav_exit(self, lat, lon, bearing, mtype, modifier, distance):
    if mtype not in NAV_EXIT_TYPES or not (0 < distance <= NAV_EXIT_RANGE) or self.on_ramp:
      return
    self.nav_side = nav_side(modifier)
    if not self.data_here:
      self.nav_exit, self.nav_exit_dist = NAV_EXIT_NO_DATA, distance
      return
    tol = max(NAV_MATCH_MIN, NAV_MATCH_TOL * distance)
    best, best_err = None, tol
    for r in self.store.ramps_all:
      if r["type"] not in ("exit", "interchange") or r["travel"] == "both":
        continue
      ok, d = ahead(lat, lon, bearing, *r["start"])
      if not ok or abs(d - distance) > best_err:
        continue
      # the ramp must leave roughly in our direction (not an exit of the other carriageway)
      first_dir = bearing_to(*r["start"], *r["end"])
      if math.cos(math.radians(first_dir - bearing)) < -0.2:
        continue
      best, best_err = r, abs(d - distance)
    if best is None:
      self.nav_exit, self.nav_exit_dist = NAV_EXIT_NO_DATA, distance
      return
    self.nav_ramp, self.nav_exit_dist = best, distance
    curves = self.store.curves.get(best["chain"], [])
    if best["travel"] == "ba":
      curves = sorted(curves, key=lambda c: -c["s_b"])
    if curves:
      chain_len = self.store.chains.get(best["chain"], {}).get("len", 0.0)
      def to_entry(c):
        return c["s_a"] if best["travel"] == "ab" else chain_len - c["s_b"]
      c = min(curves, key=lambda c: plan_speed(c["r"], distance + to_entry(c), c.get("adv", 0.0)))
      self.nav_curve, self.nav_curve_dist = c, distance + to_entry(c)
      self.nav_exit = NAV_EXIT_CURVES
    else:
      self.nav_exit = NAV_EXIT_NO_CURVES


class MapCurvesPlanner:
  """Planner-side wrapper: reads the setting, GPS position, map match and navigation, runs MapCurves and
  fills frogpilotPlan; also says when to show the notices (frogpilot_events)."""
  PARAM = "MazdaMapCurves"

  def __init__(self, params, params_memory):
    self.params = params
    self.params_memory = params_memory
    self.mode = 0
    self.counter = 0
    self.mc = None
    self.announced_curve = None
    self.announced_exit = None
    self.notice = None          # "curve" / "exit_data" / "exit_no_data" for one planner cycle

  def _read_mode(self):
    try:
      self.mode = int(self.params.get(self.PARAM, encoding="utf8") or 0)
    except (TypeError, ValueError):
      self.mode = 0

  def update(self, sm, gps_position, v_ego, set_speed):
    self.notice = None
    if self.counter % 20 == 0:
      self._read_mode()
    self.counter += 1
    if self.mode == 0:
      if self.mc is not None:
        self.mc.reset()
      return
    if self.mc is None:
      self.mc = MapCurves()
      self.matcher = RoadMatcher(self.mc.store)
    if gps_position is None:
      self.mc.reset()
      return
    way_id, is_forward = 0, True
    try:
      mm = json.loads(self.params_memory.get("MapMatchedWay") or "{}")
      age = gps_position["location_mono_time"] - mm.get("location_mono_time", 0)
      if mm.get("way_id") and isinstance(mm.get("is_forward"), bool) and 0 <= age <= 2_000_000_000:
        way_id, is_forward = int(mm["way_id"]), mm["is_forward"]
    except (TypeError, ValueError, KeyError):
      pass
    if not way_id:
      # no map match from the map program (StarPilot): match the road shapes of the curve files ourselves
      way_id, is_forward = self.matcher.update(gps_position["latitude"], gps_position["longitude"], gps_position["bearing"])
    nav = None
    try:
      if sm.alive["navInstruction"] and sm.valid["navInstruction"]:
        ni = sm["navInstruction"]
        if ni.maneuverType and ni.maneuverDistance > 0:
          nav = (str(ni.maneuverType), str(ni.maneuverModifier), float(ni.maneuverDistance))
    except Exception:
      nav = None
    self.mc.update(gps_position["latitude"], gps_position["longitude"], gps_position["bearing"], way_id, is_forward, nav)
    mc = self.mc
    # notices: once per curve that would need a slowdown, once per announced exit
    if mc.curve is not None and set_speed > 0:
      key = (mc.chain_id, mc.curve["s_a"])
      if key != self.announced_curve and plan_speed(mc.radius, mc.distance, mc.advisory) < set_speed - 2 * MPH and \
         curve_speed_limit(mc.radius, mc.advisory) < set_speed - 2 * MPH:
        self.announced_curve = key
        self.notice = "curve"
    if mc.nav_exit != NAV_NONE:
      key = (mc.nav_ramp or {}).get("chain", -1)
      if key != self.announced_exit:
        self.announced_exit = key
        self.notice = "exit_data" if mc.nav_exit in (NAV_EXIT_CURVES, NAV_EXIT_NO_CURVES) else "exit_no_data"
    elif not mc.on_ramp:
      self.announced_exit = None

  def publish(self, plan):
    mc = self.mc
    plan.mapCurveMode = self.mode
    if mc is None or self.mode == 0:
      return
    plan.mapCurveStatus = mc.status if mc.curve is not None else (STATUS_NO_DATA if not mc.data_here else STATUS_NONE)
    plan.mapCurveDistance = float(mc.distance)
    plan.mapCurveRadius = float(mc.radius)
    plan.mapCurveAdvisory = float(mc.advisory)
    plan.mapCurveName = (mc.curve or {}).get("name", "") or ""
    plan.mapOnRamp = bool(mc.on_ramp)
    r = mc.ramp if mc.on_ramp else mc.nav_ramp
    if r is not None:
      plan.mapRampType = RAMP_TYPES.get(r["type"], 4)
      to = r.get("to") or {}
      plan.mapRampToSpeed = float(to.get("speed", 0.0) or 0.0)
      plan.mapRampToHighway = to.get("cls") in HIGHWAY_CLASSES
      plan.mapRampEndDistance = float(mc.ramp_end_dist)
    plan.mapNavExit = mc.nav_exit
    plan.mapNavExitDistance = float(mc.nav_exit_dist)
    plan.mapNavExitSide = int(mc.nav_side)
    if mc.nav_curve is not None:
      plan.mapNavCurveRadius = float(mc.nav_curve["r"])
      plan.mapNavCurveDistance = float(mc.nav_curve_dist)
      plan.mapNavCurveAdvisory = float(mc.nav_curve.get("adv", 0.0) or 0.0)
