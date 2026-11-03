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
    lowest speed now with a gentle slowdown (PLAN_DECEL, after the cruise reacts) to reach its speed
    PLAN_LEAD_TIME before its entry.
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
import time
from array import array

CURVE_DIR = "/data/media/0/curve_map"
MPH = 0.44704
EARTH_R = 6371008.8

LOOKAHEAD = 1000.0        # m along the road
PLAN_LAT = 2.2            # m/s^2: as the car side (curve_target.MAP_LATERAL_ACCEL)
PLAN_DECEL = 0.25         # m/s^2: as the car side for road curves (curve_target.MAP_DECEL), until the car profile has measured it
# The car profile (opendbc/car/mazda/car_profile.py, written by the car controller in card) holds the cruise decel
# measured on this car as decel.use; both sides read it once per drive and clamp it the same way, so the planner's
# "Slowing to" notices and the car side's set speed target agree.
PROFILE_PATH = "/data/media/0/mazda_car_profile.json"
PROFILE_DECEL_RANGE = (0.15, 0.40)
PLAN_LEAD_TIME = 1.0      # s at the curve's speed before its entry
PLAN_REACTION_TIME = 1.5  # s for the stock cruise to start slowing after a SET- press
ADVISORY_OVER = 10 * 0.44704   # m/s: a curve with a yellow advisory sign in the map is never planned faster than sign + this
AHEAD_COS = 0.2           # a point is ahead when it is within ~78 deg of our heading
NEAR = 15.0               # m
APEX_PASSED = 10.0        # m: a road curve stops governing once its apex is this far behind (slow in, speed up from
                          # the apex), when the camera curve slowdown is on to hold the speed for what is left of it
TILE_KEEP = 25            # squares kept in memory
RESCAN_SECONDS = 120.0    # how often the curve_map folder is checked for new files (curve_mapd builds them while parked)
MATCH_DIST = 20.0         # m: the car is on a road shape within this distance (GPS error + road width)
MATCH_HEADING = 35.0      # deg: and heading along it (either direction) within this
MATCH_STAY = 12.0         # m: the car stays on the road it is on until that road is this far away (or turns away)
LINK_CLASSES = (2, 4, 6, 8, 10)   # ramps and other link roads (highwayClass)
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


def plan_speed(radius, distance, advisory=0.0, reaction_speed=0.0, lat_limit=PLAN_LAT, decel=PLAN_DECEL):
  """Highest speed (m/s) from which the stock cruise (reacting for PLAN_REACTION_TIME at `reaction_speed`, then
  slowing at `decel`) reaches the curve speed PLAN_LEAD_TIME before its entry (the car side's map_curve_speed)."""
  v_curve = curve_speed_limit(radius, advisory, lat_limit)
  room = distance - v_curve * PLAN_LEAD_TIME - PLAN_REACTION_TIME * max(reaction_speed, v_curve)
  return math.sqrt(v_curve ** 2 + 2 * decel * max(room, 0.0))


def read_profile_decel(path=PROFILE_PATH, default=PLAN_DECEL):
  """The decel to plan with: the car profile's measured value (clamped to PROFILE_DECEL_RANGE), else the default.
  A few KB once per drive; never raises. Mirrors car_profile.read_profile_decel (this module stays dependency free)."""
  try:
    with open(path) as f:
      use = json.load(f).get("decel", {}).get("use")
    v = float(use)
  except (OSError, ValueError, TypeError, AttributeError):
    return default
  if v != v:
    return default
  return min(max(v, PROFILE_DECEL_RANGE[0]), PROFILE_DECEL_RANGE[1])


def leave_caller_priority():
  """Background threads here inherit starpilot_process's real-time priority and core (SCHED_FIFO 51 on core 5, the
  same as plannerd and radard, which cannot preempt it: drive 3e). Give this thread normal priority and the other
  cores (as starpilot/common/feature_events.py does; a copy so this module stays dependency free)."""
  try:
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  except (AttributeError, OSError, PermissionError):
    pass
  try:
    cores = os.sched_getaffinity(0)
    others = set(range(os.cpu_count() or 1)) - cores
    if others:
      os.sched_setaffinity(0, others)
  except (AttributeError, OSError):
    pass
  try:
    os.nice(10)
  except OSError:
    pass


_DECODER = json.JSONDecoder()
_SPACE = " \t\n\r"
JSON_STEP = 200            # list items parsed between two yields of the GIL


def _skip(text, i):
  while i < len(text) and text[i] in _SPACE:
    i += 1
  return i


def load_json_in_steps(text):
  """json.loads for a curve tile, one top-level list item at a time, letting other threads run in between.
  json.load of a big tile (3 MB, 17k road shapes) holds the GIL for 0.3 s on a PC and longer on the device: the
  planner loop then stalls and selfdrived raises "Communication Issue" (drive 3e, first GPS fix). Same result."""
  i = _skip(text, 0)
  if i >= len(text) or text[i] != "{":
    return _DECODER.decode(text)
  out = {}
  i = _skip(text, i + 1)
  if i < len(text) and text[i] == "}":
    return out
  count = 0
  while True:
    key, i = _DECODER.raw_decode(text, _skip(text, i))
    i = _skip(text, i)
    if text[i] != ":":
      raise ValueError("expected ':'")
    i = _skip(text, i + 1)
    if text[i] == "[":
      items = []
      i = _skip(text, i + 1)
      if text[i] == "]":
        i += 1
      else:
        while True:
          item, i = _DECODER.raw_decode(text, i)
          items.append(item)
          count += 1
          if count % JSON_STEP == 0:
            time.sleep(0)
          i = _skip(text, i)
          if text[i] == ",":
            i = _skip(text, i + 1)
          elif text[i] == "]":
            i += 1
            break
          else:
            raise ValueError("expected ',' or ']'")
      value = items
    else:
      value, i = _DECODER.raw_decode(text, i)
    out[key] = value
    i = _skip(text, i)
    if text[i] == ",":
      i += 1
    elif text[i] == "}":
      return out
    else:
      raise ValueError("expected ',' or '}'")


def cell_key(cla, clo):
  return cla * 10_000_000 + clo     # one int per grid cell (|clo| < 180 * MATCH_CELL)


class RoadIndex:
  """Road shape segments of one square in compact arrays (micro-degrees), with a grid of cells -> segment numbers.
  About 1 MB per square instead of the ~10 MB the decoded JSON lists would take."""

  def __init__(self, shapes):
    self.lat1, self.lon1, self.lat2, self.lon2 = array("i"), array("i"), array("i"), array("i")
    self.way, self.chain = array("q"), array("q")
    self.oneway, self.cls = array("b"), array("b")
    cells = {}
    for shape in shapes:
      p = shape.get("p", [])
      if len(p) < 4:
        continue
      wid = int(shape.get("w", 0))
      cid = int(shape.get("c", wid))
      one = 1 if shape.get("o") else 0
      cls = int(shape.get("k", 0))
      lat, lon = p[0], p[1]
      for k in range(2, len(p) - 1, 2):
        la2, lo2 = lat + p[k], lon + p[k + 1]
        i = len(self.way)
        self.lat1.append(lat)
        self.lon1.append(lon)
        self.lat2.append(la2)
        self.lon2.append(lo2)
        self.way.append(wid)
        self.chain.append(cid)
        self.oneway.append(one)
        self.cls.append(cls)
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
      yield (self.lat1[i] / 1e6, self.lon1[i] / 1e6, self.lat2[i] / 1e6, self.lon2[i] / 1e6, self.way[i], self.chain[i],
             self.oneway[i], self.cls[i])


class RoadMatcher:
  """The road (way id) the car is on and its direction along the way's node order (is_forward), from the
  road shapes in the curve files: shape segments within MATCH_DIST whose direction is within MATCH_HEADING of
  the car's heading (either way along it, but only the open direction of a one-way road).

  The car stays on the road (chain) it is on while that road is within MATCH_STAY and still runs its way, also
  where OSM starts a new way at an exit: a car in the right lane past an exit is closer to the start of the
  ramp than to the middle of the freeway for a while, but it has not left the freeway until the freeway is
  MATCH_STAY away. Where the main road itself changes at the split (a new name or ref), or before anything
  is matched, a main road within MATCH_STAY is taken before a ramp. Only then the nearest road is taken (so
  from an entrance ramp the car joins the freeway it merges into, not the next ramp). Taking an exit is therefore seen once the ramp is about MATCH_STAY away from the main road (typically
  50-150 m after the split); passing it is not mistaken for taking it."""

  def __init__(self, store):
    self.store = store
    self.way_id = 0
    self.chain_id = 0
    self.on_link = False
    self.road_class = 0           # highwayClass of the matched road (0 unknown)
    self.is_forward = True

  def update(self, lat, lon, bearing):
    grid = self.store.grid
    if not grid or lat is None:
      self.way_id = 0
      return 0, True
    cla, clo = int(math.floor(lat * MATCH_CELL)), int(math.floor(lon * MATCH_CELL))
    kx = math.radians(1.0) * EARTH_R * math.cos(math.radians(lat))
    ky = math.radians(1.0) * EARTH_R
    best, stay, kind = None, None, None
    for dla in (-1, 0, 1):
      for dlo in (-1, 0, 1):
        for la1, lo1, la2, lo2, wid, cid, one, cls in (seg for ri in grid for seg in ri.segments(cell_key(cla + dla, clo + dlo))):
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
          elif diff >= 180.0 - MATCH_HEADING and not one:
            fwd = False
          else:
            continue
          link = cls in LINK_CLASSES
          cand = (d, wid, fwd, cid, link, cls)
          if best is None or d < best[0]:
            best = cand
          if d <= MATCH_STAY:
            if self.chain_id and cid == self.chain_id and (stay is None or d < stay[0]):
              stay = cand
            if not link and not self.on_link and (kind is None or d < kind[0]):
              kind = cand
    pick = stay or kind or best
    if pick is None:
      self.way_id, self.chain_id, self.road_class = 0, 0, 0
      return 0, True
    _, self.way_id, self.is_forward, self.chain_id, self.on_link, self.road_class = pick
    return self.way_id, self.is_forward


class MapCurveStore:
  """Curve files around the car, loaded in the background."""

  def __init__(self, root=None, background=True):
    self.root = CURVE_DIR if root is None else root
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
    self.signature = None      # folder mtimes the index was made from
    self.last_rescan = None
    self.scan()

  def _signature(self):
    """Modification times of the curve_map folder and its two levels of subfolders (a few stats): a file added,
    replaced or removed changes its folder's mtime, so this says whether the index is stale without listing files."""
    sig = []
    for d in [self.root] + sorted(glob.glob(os.path.join(self.root, "*"))) + sorted(glob.glob(os.path.join(self.root, "*", "*"))):
      try:
        sig.append((d, os.stat(d).st_mtime_ns))
      except OSError:
        pass
    return tuple(sig)

  def _index(self):
    index = {}
    for p in glob.glob(os.path.join(self.root, "*", "*", "*.json")):
      index.setdefault(os.path.basename(p)[:-5], []).append(p)
    return index

  def scan(self):
    self.signature = self._signature()
    self.index = self._index()
    self.last_rescan = time.monotonic()

  def rescan_if_changed(self, now=None, force=False):
    """Pick up curve files built since the index was made (curve_mapd, or files copied over): when the folder
    signature changed, re-index and drop the loaded squares (they may have been rebuilt) in the background thread.
    Checked at most every RESCAN_SECONDS (or when `force`), never while a load is running. Returns True when a
    rescan was started."""
    if self.loading:
      return False
    now = time.monotonic() if now is None else now
    if not force and self.last_rescan is not None and now - self.last_rescan < RESCAN_SECONDS:
      return False
    self.last_rescan = now
    if self._signature() == self.signature:
      return False
    if self.background:
      self.loading = True
      threading.Thread(target=self._rescan, daemon=True).start()
    else:
      self._rescan()
    return True

  def _rescan(self):
    if self.background:
      leave_caller_priority()
    try:
      sig = self._signature()
      index = self._index()
      with self.lock:
        self.signature, self.index = sig, index
        self.loaded, self.order, self.wanted = {}, [], []
        self._rebuild()
    finally:
      self.loading = False

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
    if self.background:
      leave_caller_priority()
    try:
      new = {}
      for n in names:
        curves, ramps, chains, roads = [], [], [], []
        for p in self.index.get(n, []):
          try:
            with open(p) as f:
              text = f.read()
            d = load_json_in_steps(text) if self.background else json.loads(text)
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


MEMORY_PATH = "/data/media/0/curve_memory.json"
MEMORY_MATCH = 20.0        # m between a curve's apex and a remembered one to be the same curve (curve files rebuilt)
MEMORY_SAME_PASS = 60.0    # s: passes this close in time are one pass (a divided road is two curve records ~20 m apart)
MEMORY_MIN_PASSES = 2      # clean passes before the measured radius is used
MEMORY_AGREE = 1.35        # ... and the last ones must agree within this factor
MEMORY_KEEP = 6            # passes kept per curve
MEMORY_RANGE = (0.7, 5.0)  # measured radius used within this factor of the map's (a bad pass cannot do much)
MEMORY_MAX_R = 2000.0      # m: a pass that never really turned counts as this
PASS_MIN_SPEED = 8.0       # m/s
PASS_MIN_TIME = 2.0        # s of samples in the curve
PASS_SATURATED = 0.98      # |steer command|: the car could not follow (it goes wide): the pass does not count
PASS_FILTER = 1.0          # s moving average of the measured curvature
# Steering at its limit: the Torque Interceptor could not steer this curve at that speed (drive 3d: the 30 mph West
# Utica curve maxed the steering at 37 mph on every pass, FrogPilot's too). Such a pass caps the curve's speed.
SAT_MIN_TIME = 1.0         # s with the steering command at PASS_SATURATED or more for a pass to count as maxed out
SAT_LAT_SHARE = 0.9        # share of the pass with openpilot steering
# Saturated samples with the driver's hands on the wheel do not count: the command also saturates when the driver
# steers against openpilot's path (another line through the curve), which says nothing about the curve.
SAT_FACTOR = 0.9           # a maxed-out pass caps the curve at this share of the speed the steering maxed out at
CAP_FLOOR = 25 * MPH       # never below this (the stock cruise's lowest set speed) ...
CAP_RISE = 0.5 * MPH       # a clean pass at (or within CAP_NEAR of) the cap raises it this much ...
CAP_NEAR = 2.5 * MPH       # (true speed: the dash reads ~2 mph above it, and the car side keeps a margin above the cap)
CAP_JUMP = 5 * MPH         # ... a clean pass well above the cap raises it to that pass's speed, at most this much
# One trusted gentle pass: a map curve much sharper than the road really is (a kink where two map ways join) is
# corrected after a single clean pass, when that pass was full, fast enough and right through the curve's apex.
MEMORY_ONE_PASS = 1.6      # the newest pass measured at least this many times the map's radius ...
PASS_TRUST_MIN_SPEED = 10.0  # m/s ... at this median speed or more ...
PASS_APEX_NEAR = 15.0      # m ... the car came this close to the curve's apex (else: another road, the pass counts for nothing)
PASS_EXIT_NEAR = 30.0      # m ... and the pass ended at the curve's exit (not by losing the road match)

# Camera check of map curves (CurveVeto): the driving model's path ahead is compared with the map curve's radius.
VETO_FACTOR = 2.0          # the camera sees the curve at least this many times gentler than planned ...
VETO_CONFIRM = 20          # ... on this many frames in a row where it can see the curve (1 s at 20 Hz): ignored
REARM_FACTOR = 1.3         # an ignored curve governs again when the camera sees it within this factor ...
REARM_CONFIRM = 10         # ... on this many seeing frames in a row (0.5 s)
VETO_MAX_FLIPS = 2         # after this many take-backs a curve is never ignored again this drive
VETO_FORGET = 40           # planner frames a curve may be missing (road match flicker) before its check starts over
VETO_CHORD = 30.0          # m: heading change over +-this along the path (the curve builder's HALF_WINDOW)
VETO_PAD = 20.0            # m added around the curve's span
VETO_STEP = 5.0            # m between curvature samples
VETO_SEE_MAX = 110.0       # m: the camera judges a curve only when its apex + VETO_PAD is this close ...
VETO_PATH_MARGIN = 10.0    # ... and this far inside the end of the model's path
VETO_YSTD_MAX = 1.5        # m: the path's sideways uncertainty at the apex
VETO_MIN_SPEED = 8.0       # m/s (at low speed the model's path is short)


def curve_pass_radius(samples, dt=0.05, max_r=MEMORY_MAX_R):
  """Radius (m) a clean pass through a curve measured: the tightest PASS_FILTER moving average of the car's own path
  curvature, or None when the pass does not count. samples: (v m/s, curvature 1/m, lat active, driver steering,
  |steer command|) per planner cycle while in the curve."""
  if not samples or len(samples) * dt < PASS_MIN_TIME:
    return None
  if any((not lat) or driver or cmd >= PASS_SATURATED for _, _, lat, driver, cmd in samples):
    return None
  ks = [abs(k) for v, k, _, _, _ in samples if v >= PASS_MIN_SPEED]
  n = max(int(round(PASS_FILTER / dt)), 1)
  if len(ks) < n:
    return None
  best = max(sum(ks[j:j + n]) / n for j in range(len(ks) - n + 1))
  return max_r if best <= 1.0 / max_r else min(1.0 / best, max_r)


def curve_pass_saturation(samples, dt=0.05):
  """Speed (m/s) at which the steering first stayed at its limit through the curve (SAT_MIN_TIME in all), or None.
  samples as for curve_pass_radius."""
  if not samples or len(samples) * dt < PASS_MIN_TIME:
    return None
  if sum(1 for _, _, lat, _, _ in samples if lat) < SAT_LAT_SHARE * len(samples):
    return None
  sat = [lat and not driver and cmd >= PASS_SATURATED and v >= PASS_MIN_SPEED for v, _, lat, driver, cmd in samples]
  if sum(sat) * dt < SAT_MIN_TIME:
    return None
  # the speed at the start of the longest run at the limit (a short blip on entry does not set the speed)
  best_len, best_start, run_start = 0, 0, None
  for k, s in enumerate(sat + [False]):
    if s and run_start is None:
      run_start = k
    elif not s and run_start is not None:
      if k - run_start > best_len:
        best_len, best_start = k - run_start, run_start
      run_start = None
  return samples[best_start][0]


def curve_pass_speed(samples):
  """Median speed (m/s) of a pass."""
  vs = sorted(v for v, _, _, _, _ in samples)
  return vs[len(vs) // 2] if vs else 0.0


class CurveMemory:
  """Per map curve (by its apex position, so it survives rebuilt curve files), kept in MEMORY_PATH (a small JSON list):
    * the radius the car actually drove: used instead of the map's once MEMORY_MIN_PASSES clean passes agree;
    * a speed cap ("cap", m/s) after a pass where the steering maxed out: SAT_FACTOR of that speed, never below the
      curve's yellow sign or CAP_FLOOR. Each clean pass near the cap raises it by CAP_RISE (searching back up)."""

  def __init__(self, path=MEMORY_PATH):
    self.path = path
    self.entries = []
    try:
      with open(path) as f:
        data = json.load(f)
      self.entries = [e for e in data.get("curves", []) if "lat" in e and "lon" in e]
    except (OSError, ValueError, AttributeError):
      self.entries = []
    self.saving = False
    self.dirty = False

  def _find(self, lat, lon):
    best, best_d = None, MEMORY_MATCH
    for e in self.entries:
      d = dist_m(lat, lon, e["lat"], e["lon"])
      if d <= best_d:
        best, best_d = e, d
    return best

  def radius(self, c):
    """Learned radius for curve dict `c`, or None: the middle of the last passes when MEMORY_MIN_PASSES or more
    agree; else the newest pass alone when it was a trusted gentle pass (one_pass_ok). A sharper newest pass
    therefore goes back to the map's radius by itself, and a later gentle pass earns it again."""
    try:
      e = self._find(*c["apex"])
      if e is None:
        return None
      passes = [p for p in e.get("passes", []) if isinstance(p, list) and len(p) >= 2]
      lo, hi = MEMORY_RANGE
      if len(passes) >= MEMORY_MIN_PASSES:
        last = sorted(float(p[0]) for p in passes[-3:])
        if last[-1] <= last[0] * MEMORY_AGREE:
          return min(max(last[len(last) // 2], c["r"] * lo), c["r"] * hi)
      if passes and self.one_pass_ok(c, passes[-1]):
        return min(max(float(passes[-1][0]), c["r"] * lo), c["r"] * hi)
      return None
    except (KeyError, TypeError, IndexError, ValueError):
      return None

  @staticmethod
  def one_pass_ok(c, p):
    """Pass p ([r, t, v, flags]) alone shows the curve is much gentler than the map says: a full pass through the
    apex, fast enough, MEMORY_ONE_PASS x the map's radius or more, and a real turn (MEMORY_MAX_R means it never
    turned: another road, not a gentle curve). Old 2-value passes never qualify."""
    try:
      if len(p) < 4 or p[2] is None or "full" not in (p[3] or []):
        return False
      r = float(p[0])
      return float(p[2]) >= PASS_TRUST_MIN_SPEED and MEMORY_ONE_PASS * float(c["r"]) <= r < MEMORY_MAX_R
    except (TypeError, ValueError, KeyError):
      return False

  def speed_cap(self, c):
    """Speed cap (m/s) for curve dict `c` from passes where the steering maxed out, or None."""
    try:
      e = self._find(*c["apex"])
      cap = None if e is None else e.get("cap")
      return float(cap) if cap else None
    except (KeyError, TypeError, ValueError):
      return None

  def _entry(self, c):
    e = self._find(*c["apex"])
    if e is None:
      e = {"lat": round(c["apex"][0], 6), "lon": round(c["apex"][1], 6), "r_map": c.get("r"),
           "name": c.get("name", ""), "passes": []}
      self.entries.append(e)
    return e

  def add_saturated_pass(self, c, v, t=None):
    """A pass through curve `c` where the steering maxed out at speed v (m/s): cap the curve's speed."""
    t = time.time() if t is None else t   # noqa: TID251  wall time, as add_pass
    e = self._entry(c)
    if abs(t - float(e.get("cap_t", -1e12))) < MEMORY_SAME_PASS:
      return                                  # the same pass again (the other carriageway's record)
    floor = max(CAP_FLOOR, float(c.get("adv", 0.0) or 0.0))
    old = float(e["cap"]) if e.get("cap") else float("inf")
    e["cap"] = round(max(floor, min(old, SAT_FACTOR * v)), 2)
    e["cap_t"] = round(t, 1)
    e["cap_hits"] = int(e.get("cap_hits", 0)) + 1
    self.save()

  def add_pass(self, c, r, t=None, v=None, flags=None):
    """A clean pass through curve `c` that measured radius r (m), at wall time t (s) and median speed v (m/s).
    flags: "full" (through the apex to the exit), "vetoed" (the camera check ignored the curve: audit only)."""
    t = time.time() if t is None else t   # noqa: TID251  wall time: passes are compared across drives and reboots
    e = self._entry(c)
    if v is not None and e.get("cap") and v >= float(e["cap"]) - CAP_NEAR and \
       abs(t - float(e.get("cap_t", -1e12))) >= MEMORY_SAME_PASS:
      # it took the curve cleanly near the cap (try a bit faster), or well above it (up to that speed)
      cap = float(e["cap"])
      e["cap"] = round(max(cap + CAP_RISE, min(v, cap + CAP_JUMP)), 2)
      e["cap_t"] = round(t, 1)
    passes = [p for p in e["passes"] if isinstance(p, list) and len(p) >= 2]
    if passes and abs(t - passes[-1][1]) < MEMORY_SAME_PASS:
      passes = passes[:-1]                    # the same pass again (the other carriageway's record): keep the newest
    new = [round(float(r), 1), round(t, 1), None if v is None else round(float(v), 2), sorted(set(flags or []))]
    e["passes"] = (passes + [new])[-MEMORY_KEEP:]
    self.save()

  def note_veto(self, c):
    """The camera check ignored curve `c` (counted for the record only, never read)."""
    e = self._entry(c)
    e["veto_hits"] = int(e.get("veto_hits", 0)) + 1
    self.save()

  def save(self):
    """Write the file in the background; a pass added while writing is written right after."""
    self.dirty = True
    if self.saving:
      return
    self.saving = True

    def write():
      leave_caller_priority()
      try:
        while self.dirty:
          self.dirty = False
          data = {"version": 1, "curves": [dict(e) for e in self.entries]}
          tmp = self.path + ".tmp"
          with open(tmp, "w") as f:
            json.dump(data, f)
          os.replace(tmp, self.path)
      except OSError:
        pass
      finally:
        self.saving = False
    threading.Thread(target=write, daemon=True).start()


class CameraView:
  """What the driving model sees ahead in one frame (camera_view() builds it from modelV2, radarState, carState):
  per path point s (m along the path), psi (heading, rad, relative to the car) and ystd (m, sideways uncertainty);
  red (model confidence red), lane_change, blinker, lead_d (m to the lead, None without one), v_ego (m/s)."""

  def __init__(self, s, psi, ystd, red=False, lane_change=False, blinker=False, lead_d=None, v_ego=0.0):
    self.s, self.psi, self.ystd = list(s), list(psi), list(ystd)
    self.red, self.lane_change, self.blinker = bool(red), bool(lane_change), bool(blinker)
    self.lead_d, self.v_ego = lead_d, float(v_ego)

  @property
  def end(self):
    return self.s[-1] if self.s else 0.0

  def _at(self, values, x):
    s = self.s
    if x <= s[0]:
      return values[0]
    for i in range(1, len(s)):
      if x <= s[i]:
        w = (x - s[i - 1]) / max(s[i] - s[i - 1], 1e-6)
        return values[i - 1] + w * (values[i] - values[i - 1])
    return values[-1]

  def radius(self, lo, hi):
    """Tightest radius (m) of the path between lo and hi m along it: heading change over +-VETO_CHORD (shorter at
    the ends of the path, never under VETO_CHORD in all). inf when straight."""
    end = self.end
    hi = min(hi, end)
    worst = 0.0
    x = max(lo, 0.0)
    while x <= hi + 1e-6:
      a, b = max(x - VETO_CHORD, 0.0), min(x + VETO_CHORD, end)
      if b - a >= VETO_CHORD:
        worst = max(worst, abs(self._at(self.psi, b) - self._at(self.psi, a)) / (b - a))
      x += VETO_STEP
    return 1.0 / worst if worst > 1e-6 else float("inf")

  def blind(self, s_apex):
    """Why the camera cannot judge a curve whose apex is s_apex m ahead now (None: it can)."""
    if self.v_ego < VETO_MIN_SPEED:
      return "slow"
    if self.red:
      return "red"
    if self.lane_change or self.blinker:
      return "signal"
    if len(self.s) < 2 or s_apex + VETO_PAD > min(VETO_SEE_MAX, self.end - VETO_PATH_MARGIN):
      return "far"
    if self._at(self.ystd, s_apex) > VETO_YSTD_MAX:
      return "ystd"
    if self.lead_d is not None and self.lead_d < s_apex + VETO_PAD:
      return "lead"
    return None


def _raw(v):
  return int(getattr(v, "raw", v))


def camera_view(sm):
  """CameraView of this planner frame, or None when the model is not there (never raises)."""
  try:
    if not (sm.alive["modelV2"] and sm.valid["modelV2"]):
      return None
    md = sm["modelV2"]
    xs, ys = list(md.position.x), list(md.position.y)
    n = len(xs)
    if n < 2 or len(ys) != n:
      return None
    s = [0.0]
    for i in range(1, n):
      s.append(s[-1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]))
    psi = list(md.orientation.z)
    if len(psi) != n:
      psi = [0.0] + [math.atan2(ys[i] - ys[i - 1], xs[i] - xs[i - 1]) for i in range(1, n)]
    ystd = list(md.position.yStd)
    if len(ystd) != n:
      ystd = [float("inf")] * n            # no uncertainty from the model: never judge
    cs = sm["carState"]
    lead = sm["radarState"].leadOne
    lead_d = float(lead.dRel) if lead.status else None
    return CameraView(s, psi, ystd, red=_raw(md.confidence) == 0, lane_change=_raw(md.meta.laneChangeState) != 0,
                      blinker=bool(cs.leftBlinker or cs.rightBlinker), lead_d=lead_d, v_ego=float(cs.vEgo))
  except Exception:
    return None


class CurveVeto:
  """Camera check of map curves. A map curve the driving model sees as at least VETO_FACTOR x gentler, on
  VETO_CONFIRM frames in a row where it can see the curve's apex well (CameraView.blind), stops governing: the
  next curve governs, or none (the speed comes back as for any curve). While ignored it is still watched: seen
  sharp again (REARM_FACTOR) it governs again; after VETO_MAX_FLIPS take-backs it is never ignored again this drive.
  Never ignored: ramp curves, curves with a yellow sign, curves the curve memory knows (a cap or a learned radius:
  the car has driven them). Frames where the camera cannot judge leave the counters as they are.
  The car side only sees the curve gone; the camera curve slowdown keeps running as the safety net.
  Honest limit: the map slows early (~290 m before the curve at 45 mph), the camera judges at ~110 m, so this saves
  about the second half of a false slowdown; the curve memory then corrects the curve after one clean pass."""

  def __init__(self):
    self.reset_drive()

  def reset_drive(self):
    self.state = {}           # (chain, s_a) -> {"vetoed", "n", "flips", "r_cam", "seen"}
    self.given_up = set()     # keys never ignored again this drive
    self.frame = 0
    self.events = []          # for the planner's log (it empties the list)
    self.hidden = None        # (key, r_cam) of an ignored curve this frame, or None

  def tick(self):
    self.frame += 1
    self.hidden = None
    for key in [k for k, st in self.state.items() if self.frame - st["seen"] > VETO_FORGET]:
      del self.state[key]

  def is_vetoed(self, key):
    st = self.state.get(key)
    return bool(st and st["vetoed"])

  def flips(self, key):
    st = self.state.get(key)
    return st["flips"] if st else (VETO_MAX_FLIPS if key in self.given_up else 0)

  @staticmethod
  def vetoable(mc, c):
    if c.get("ramp") or float(c.get("adv", 0.0) or 0.0) > 0 or "apex" not in c:
      return False
    if mc.memory is not None and (mc.memory.speed_cap(c) is not None or mc.memory.radius(c) is not None):
      return False
    return True

  def filter(self, mc, cid, cands, camera):
    """cands: [(curve, m to its entry)] -> the same without the ignored curves."""
    out = []
    for c, d in cands:
      key = (cid, c["s_a"])
      if key in self.given_up or not self.vetoable(mc, c):
        self.state.pop(key, None)
        out.append((c, d))
        continue
      st = self.state.get(key)
      if st is None:
        st = self.state[key] = {"vetoed": False, "n": 0, "flips": 0, "r_cam": 0.0}
      st["seen"] = self.frame
      self._check(mc, key, st, c, d, camera)
      if st["vetoed"]:
        if self.hidden is None:
          self.hidden = (key, st["r_cam"])
      else:
        out.append((c, d))
    return out

  def _check(self, mc, key, st, c, d_entry, camera):
    g = mc.geom.get(id(c))
    if camera is None or g is None:
      return
    d_apex, d_exit = g
    s_apex = max(d_apex, 0.0)               # behind the apex: what is left of the curve
    if camera.blind(s_apex) is not None:
      return
    r_eff = mc._radius(c)
    s_exit = d_exit if d_entry <= 0.0 else d_entry + float(c.get("len", 0.0))
    lo = max(d_entry - VETO_PAD, 0.0)
    hi = min(s_exit + VETO_PAD, VETO_SEE_MAX, camera.end - VETO_PATH_MARGIN)
    r_cam = camera.radius(lo, hi)
    st["r_cam"] = r_cam
    if not st["vetoed"]:
      st["n"] = st["n"] + 1 if r_cam >= VETO_FACTOR * r_eff else 0
      if st["n"] >= VETO_CONFIRM:
        st["vetoed"], st["n"] = True, 0
        self._event("veto", c, key, st, d_entry, d_apex, r_eff, camera)
    else:
      st["n"] = st["n"] + 1 if r_cam <= REARM_FACTOR * r_eff else 0
      if st["n"] >= REARM_CONFIRM:
        st["vetoed"], st["n"] = False, 0
        st["flips"] += 1
        self._event("rearm", c, key, st, d_entry, d_apex, r_eff, camera)
        if st["flips"] >= VETO_MAX_FLIPS:
          self.given_up.add(key)
          self._event("give_up", c, key, st, d_entry, d_apex, r_eff, camera)

  def _event(self, what, c, key, st, d_entry, d_apex, r_eff, camera):
    r_cam = st["r_cam"]
    self.events.append({
      "what": what, "curve": c, "name": c.get("name", "") or "", "chain": key[0], "s_a": key[1],
      "r_map": float(c.get("r", 0.0)), "r_eff": round(r_eff, 1), "turn": float(c.get("turn", 0.0) or 0.0),
      "len": float(c.get("len", 0.0) or 0.0), "d_entry": round(d_entry, 1), "d_apex": round(d_apex, 1),
      "r_cam": round(r_cam, 1) if r_cam != float("inf") else -1.0,
      "ystd_apex": round(camera._at(camera.ystd, max(d_apex, 0.0)), 2), "lead_d": camera.lead_d,
      "v_ego": round(camera.v_ego, 2), "frames": VETO_CONFIRM if what == "veto" else REARM_CONFIRM, "flips": st["flips"]})


class MapCurves:
  """The governing curve ahead, the ramp we are on, and the matched navigation exit."""

  def __init__(self, store=None, release_after_apex=False, memory=None, decel=PLAN_DECEL):
    self.store = store if store is not None else MapCurveStore()
    self.release_after_apex = release_after_apex
    self.memory = memory
    self.decel = decel           # the cruise's slowdown for the plan (PLAN_DECEL, or the car profile's measured value)
    self.veto = CurveVeto()      # camera check (acts only when update() gets a camera view)
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
    self.inside = None           # the road curve we are driving through (also after its apex), for curve memory
    self.inside_exit = float("inf")  # m to its exit
    self.geom = {}               # id(curve) -> (m to its apex, negative when behind; m to its exit) this frame
    self.veto_status = 0         # starpilotPlan.mapCurveVeto
    self.veto_radius = 0.0
    self.source = 0              # starpilotPlan.mapCurveSource

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
    inside_exit = float("inf")
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
      if "apex" in c:
        ap_ok, ap_d = ahead(lat, lon, bearing, *c["apex"])
        self.geom[id(c)] = (ap_d if ap_ok else -ap_d, d_exit)
      later_way = (s_entry > s1) if increasing else (s_entry < s0)
      if later_way or (on_this_way and ent_ahead):
        if d_entry <= LOOKAHEAD:
          out.append((c, d_entry))
      elif ex_ahead and d_exit > NEAR and d_exit <= c["len"] + 60:
        if not c.get("ramp") and d_exit < inside_exit:
          # the curve we are in is the one we leave first (the next one may already match up to 60 m before it)
          self.inside, inside_exit = c, d_exit
          self.inside_exit = d_exit
        if self.release_after_apex and not c.get("ramp") and "apex" in c:
          ap_ahead, d_apex = ahead(lat, lon, bearing, *c["apex"])
          if not ap_ahead and d_apex > APEX_PASSED:
            continue              # past the apex: the next curve (if any) governs, or none (the speed comes back)
        out.append((c, 0.0))      # in the curve
    return out

  def _radius(self, c):
    """The curve's radius: what the car measured on clean passes (curve memory) if known, else the map's; made
    tighter when the memory caps the curve's speed (steering maxed out there), so that the curve's speed at PLAN_LAT
    is the cap. The car side plans at PLAN_LAT or less, so it never goes faster than the cap."""
    r = c["r"]
    if self.memory is not None and not c.get("ramp"):
      learned = self.memory.radius(c)
      if learned is not None:
        r = learned
      cap = self.memory.speed_cap(c)
      if cap is not None:
        r = min(r, cap * cap / PLAN_LAT)
    return r

  def _source(self, c):
    """Where the curve's radius comes from: 0 the map, 1 curve memory (driven passes), 2 a memory speed cap."""
    if self.memory is None or c.get("ramp"):
      return 0
    learned = self.memory.radius(c)
    cap = self.memory.speed_cap(c)
    r = learned if learned is not None else c["r"]
    if cap is not None and cap * cap / PLAN_LAT < r:
      return 2
    return 1 if learned is not None else 0

  def update(self, lat, lon, bearing, way_id, is_forward, nav=None, camera=None):
    """nav: (maneuver type, modifier, distance m) of the next maneuver, or None. camera: CameraView or None
    (no camera check)."""
    self.reset()
    self.veto.tick()
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
        cands = self.veto.filter(self, cid, cands, camera)
      if cands:
        c, d = min(cands, key=lambda cd: plan_speed(self._radius(cd[0]), cd[1], cd[0].get("adv", 0.0), decel=self.decel))
        self.curve, self.distance, self.radius = c, d, self._radius(c)
        self.advisory = float(c.get("adv", 0.0) or 0.0)
        self.status = STATUS_RAMP_CURVE if c.get("ramp") else STATUS_CURVE
        self.source = self._source(c)
      if self.veto.hidden is not None:
        self.veto_status = 1
        r_cam = self.veto.hidden[1]
        self.veto_radius = r_cam if r_cam != float("inf") else MEMORY_MAX_R
      elif self.curve is not None and self.veto.flips((cid, self.curve["s_a"])) > 0:
        self.veto_status = 2
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
      c = min(curves, key=lambda c: plan_speed(c["r"], distance + to_entry(c), c.get("adv", 0.0), decel=self.decel))
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
    self.profile_path = PROFILE_PATH
    self.decel = PLAN_DECEL     # read with the mode at the start of every drive (the car side reads the same file then)
    self.recorder = None        # camera check events (feature logs), started on first use
    self.recorder_failed = False

  def _read_mode(self):
    try:
      self.mode = int(self.params.get(self.PARAM, encoding="utf8") or 0)
    except (TypeError, ValueError):
      self.mode = 0
    self.decel = read_profile_decel(self.profile_path)
    if self.mc is not None:
      self.mc.decel = self.decel

  def update(self, sm, gps_position, v_ego, set_speed):
    self.notice = None
    # The car side reads the setting once when a drive starts, so read it here only then too (first call, after
    # a pause in calls, or while not started): the screen never says "Slowing" when the car would not act.
    import time
    now = time.monotonic()
    try:
      started = bool(sm["deviceState"].started)
    except Exception:
      started = True
    drive_start = self.counter == 0 or now - getattr(self, "last_call", now) > 5.0 or (started and not getattr(self, "prev_started", False))
    if drive_start or (not started and self.counter % 20 == 0):
      self._read_mode()
    if drive_start and self.mc is not None:
      self.mc.veto.reset_drive()  # a curve ignored or taken back last drive is checked afresh
    self.prev_started = started
    self.last_call = now
    self.counter += 1
    if self.mode == 0:
      if self.mc is not None:
        self.mc.reset()
      return
    if self.mc is None:
      # the speed comes back from a road curve's apex only with the camera curve slowdown on (it keeps the speed
      # for whatever is left of the curve, and the next curve keeps governing when it is close)
      self.mc = MapCurves(release_after_apex=self.params.get_bool("MazdaCurveSlowdown"), memory=CurveMemory(), decel=self.decel)
      self.pass_curve, self.pass_samples = None, []
      self.matcher = RoadMatcher(self.mc.store)
    elif started:
      # curve files built while parked (curve_mapd) or copied over: re-index at the start of a drive, then every
      # RESCAN_SECONDS (a few folder stats here; the file listing runs in the background)
      self.mc.store.rescan_if_changed(now, force=drive_start)
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
      self.road_class = self.matcher.road_class
    else:
      self.road_class = 0
    nav = None
    try:
      if sm.alive["navInstruction"] and sm.valid["navInstruction"]:
        ni = sm["navInstruction"]
        if ni.maneuverType and ni.maneuverDistance > 0:
          nav = (str(ni.maneuverType), str(ni.maneuverModifier), float(ni.maneuverDistance))
    except Exception:
      nav = None
    camera = camera_view(sm)
    self.mc.update(gps_position["latitude"], gps_position["longitude"], gps_position["bearing"], way_id, is_forward, nav,
                   camera=camera)
    mc = self.mc
    self._veto_log(mc, v_ego, set_speed, now)
    self._measure_pass(sm, mc.inside, gps_position["latitude"], gps_position["longitude"])
    # notices: once per curve that would need a slowdown, once per announced exit
    if mc.curve is not None and set_speed > 0:
      key = (mc.chain_id, mc.curve["s_a"])
      if key != self.announced_curve and plan_speed(mc.radius, mc.distance, mc.advisory, set_speed, decel=self.decel) < set_speed - 2 * MPH and \
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

  VETO_COLUMNS = ["t_wall", "vEgo_mph", "set_mph", "map_status", "map_dist", "map_r", "veto", "veto_r"]

  def _veto_log(self, mc, v_ego, set_speed, now):
    """Feature log of the camera check: a row per frame, an event when a curve is ignored, taken back or given up
    on (never raises into the planner)."""
    try:
      events, mc.veto.events = mc.veto.events, []
      if self.recorder is None and not self.recorder_failed:
        try:
          from openpilot.starpilot.common.feature_events import EventRecorder
          self.recorder = EventRecorder(self.VETO_COLUMNS, hz=20, params=self.params)
        except Exception:
          self.recorder_failed = True
      for ev in events:
        if ev["what"] == "veto" and mc.memory is not None:
          mc.memory.note_veto(ev["curve"])
      rec = self.recorder
      if rec is None:
        return
      rec.add_row(now, (f"{time.time():.2f}", v_ego / MPH, set_speed / MPH, mc.status if mc.curve is not None else 0,  # noqa: TID251
                        mc.distance, mc.radius, mc.veto_status, mc.veto_radius))
      for ev in events:
        rec.record("mapCurveVeto", now, {**{k: v for k, v in ev.items() if k != "curve"}, "set_mph": set_speed / MPH})
      rec.tick(now)
    except Exception:
      pass

  def _measure_pass(self, sm, inside, lat=None, lon=None):
    """Curve memory: collect the car's own path through the road curve we are in; when we leave it, a clean pass
    adds its measured radius (never raises into the planner). A pass counts for nothing when the turn signal was
    on in it (a lane change) or the car never came within PASS_APEX_NEAR of the apex (another road)."""
    try:
      if inside is not self.pass_curve:
        if self.pass_curve is not None and self.mc.memory is not None:
          self._finish_pass()
        self.pass_curve, self.pass_samples = inside, []
        self.pass_blinker, self.pass_apex, self.pass_exit, self.pass_vetoed = False, float("inf"), float("inf"), False
      if inside is not None and len(self.pass_samples) < 2400:
        cs, cc = sm["carState"], sm["carControl"]
        self.pass_samples.append((float(cs.vEgo), float(sm["controlsState"].curvature), bool(cc.latActive),
                                  bool(cs.steeringPressed), abs(float(cc.actuators.torque))))
        self.pass_blinker |= bool(cs.leftBlinker or cs.rightBlinker)
        if lat is not None and "apex" in inside:
          self.pass_apex = min(self.pass_apex, dist_m(lat, lon, *inside["apex"]))
        self.pass_exit = self.mc.inside_exit
        self.pass_vetoed |= self.mc.veto.is_vetoed((self.mc.chain_id, inside["s_a"]))
    except Exception:
      self.pass_curve, self.pass_samples = None, []

  def _finish_pass(self):
    c, samples = self.pass_curve, self.pass_samples
    if self.pass_blinker or self.pass_apex > PASS_APEX_NEAR:
      return
    v_sat = curve_pass_saturation(samples)
    if v_sat is not None:
      self.mc.memory.add_saturated_pass(c, v_sat)
      return
    r = curve_pass_radius(samples)
    if r is not None:
      flags = (["full"] if self.pass_exit <= PASS_EXIT_NEAR else []) + (["vetoed"] if self.pass_vetoed else [])
      self.mc.memory.add_pass(c, r, v=curve_pass_speed(samples), flags=flags)

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
    plan.mapRoadClass = int(getattr(self, "road_class", 0))
    plan.mapCurveVeto = int(mc.veto_status)
    plan.mapCurveVetoRadius = float(mc.veto_radius)
    plan.mapCurveSource = int(mc.source)
    plan.mapNavExit = mc.nav_exit
    plan.mapNavExitDistance = float(mc.nav_exit_dist)
    plan.mapNavExitSide = int(mc.nav_side)
    if mc.nav_curve is not None:
      plan.mapNavCurveRadius = float(mc.nav_curve["r"])
      plan.mapNavCurveDistance = float(mc.nav_curve_dist)
      plan.mapNavCurveAdvisory = float(mc.nav_curve.get("adv", 0.0) or 0.0)
