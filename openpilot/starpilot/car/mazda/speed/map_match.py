"""Onroad map side of the Mazda speed control, ported from the old fork's starpilot/controls/lib/map_curves.py:
the curve files around the car (MapCurveStore), the road the car is on (RoadIndex / RoadMatcher, from the road
shapes in the curve files, never through a one-way road's closed direction) and whether the GPS is good enough to
slow early (GpsTrust). Behaviour unchanged except: curve files of another FORMAT_VERSION or an older builder are
ignored (no map curves there until curve_mapd rebuilds them: the camera curves still work), the folder is 7.0's
curve root. The GPS error estimate is used as before (only with USE_GPS_ACCURACY; logged otherwise).
Pure Python + threads, no openpilot imports (tests/test_map_match.py)."""
import glob
import json
import math
import os
import threading
import time
from array import array

from openpilot.starpilot.car.mazda.speed.curve_builder import FORMAT_VERSION, BUILDER_VERSION
from openpilot.starpilot.car.mazda.speed.events import leave_caller_priority

CURVE_DIR = "/data/media/0/mazda_curve_map"
EARTH_R = 6371008.8
NEAR = 15.0               # m
AHEAD_COS = 0.2           # a point is ahead when it is within ~78 deg of our heading
TILE_KEEP = 25            # squares kept in memory
RESCAN_SECONDS = 120.0    # how often the curve folder is checked for new files (curve_mapd builds them while parked)
MATCH_DIST = 20.0         # m: the car is on a road shape within this distance (GPS error + road width)
MATCH_HEADING = 35.0      # deg: and heading along it (either direction) within this
MATCH_STAY = 12.0         # m: the car stays on the road it is on until that road is this far away (or turns away)
LINK_CLASSES = (2, 4, 6, 8, 10)   # ramps and other link roads (highwayClass)
MATCH_CELL = 500          # grid cells of 1/500 deg (~220 m) for the road shape index


# 3f: the track had the road's exact shape, 90-180 m to one side): then nothing matches, or worse, a parallel road
# matches. Safe by default: when the match is doubtful the map does not slow early; the camera curve slowdown is
# the safety net.
# starpilotPlan.mapGpsTrust
TRUST_OK, TRUST_SHAKY, TRUST_LOW, TRUST_PAUSED_ACCURACY, TRUST_PAUSED_LOST = 0, 1, 2, 3, 4
TRUST_NO_GPS = 5           # (round T, T1b) paused: no GPS position (every car-side check reads trust > 0 as not OK)
# (round T, T1c) starpilotPlan.mapGpsShakyWhy, why the trust is 1 (shaky): 0 it is not 1; 1 only the start of the drive's
# STEADY_TIME (no pause, no suspicious gap restarted it); 2 the steady time restarted after a pause or a suspicious gap
SHAKY_NONE, SHAKY_START, SHAKY_RESTART = 0, 1, 2
# (round T, T1b) "Map curves off: no GPS yet": once per drive, before the first position, once cruise is engaged or the car
# is above this
NOGPS_NOTICE_SPEED = 15 * 0.44704   # m/s
# The GPS chip's own error estimate (qcomgpsd fills gpsLocation.horizontalAccuracy since StarPilot; openpilot left
# it 0). Its values have not been checked against a drive yet: until then it is only logged, and the pause and
# the ambiguity distance work without it (USE_GPS_ACCURACY). Turn on once a drive shows ~3-15 m on open roads.
USE_GPS_ACCURACY = False
ACC_PAUSE = 50.0           # m: an error estimate above this (or reliability "very low") for BAD_CONFIRM pauses
BAD_CONFIRM = 2.0          # s (a one-second blip under a bridge does not cancel a slowdown)
ACC_LOW = 20.0             # m: from this up to ACC_PAUSE (or reliability "low") a curve is held, not slowed for
ACC_RESUME = 30.0          # m: a paused gate resumes only with the error at or below this ...
RESUME_TIME = 5.0          # s ... for this long, matched on a road (the first match after a blind spell is suspect)
LOST_SPEED = 15.0          # m/s (34 mph): unmatched below this is normal (residential streets are not in the files)
LOST_TIME = 10.0           # s unmatched at speed where curve data exists: paused ("GPS not on a known road")
STEADY_TIME = 60.0         # s of matched driving (no suspicious gap) before the match is fully trusted ...
STEADY_GAP = 2.0           # s ... a gap longer than this unmatched at speed with data starts it over
AMB_MIN, AMB_MAX = 15.0, 45.0   # m: another road (heading within MATCH_HEADING) closer than clamp(2 x error + 5)
AMB_DEFAULT_ACC = 5.0           # m: the error assumed when the GPS gives none (or USE_GPS_ACCURACY is off): good
                                # drives were 1-4 m from the road; a far-off GPS is caught by the lost clock instead
TIMER_EPS = 1e-6                # s: timers summed from dt steps reach their limit on the expected frame
NEAREST_SEARCH = 220.0          # m: the nearest road is measured this far (logs: how far off the GPS is)


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


def gps_flags(flags):
  """Decode gpsLocation.flags of source qcomdiag (system/qcomgpsd/structs.position_quality): reliability 0-4 (0 not
  set, 1 very low .. 4 high), sensor aided, sensor data used, ellipse confidence %."""
  flags = int(flags or 0)
  return {"reliability": flags & 0x7, "sensorAided": bool(flags & 0x8), "sensorData": bool(flags & 0x10),
          "confidence": (flags >> 8) & 0x7F}


def gps_quality(gps_location):
  """The fix quality keys added to the planner's gps_position (never raises: fakes and other sources give 0)."""
  try:
    q = gps_flags(getattr(gps_location, "flags", 0))
    acc = float(getattr(gps_location, "horizontalAccuracy", 0.0) or 0.0)
    return {"accuracy": acc if math.isfinite(acc) and acc > 0 else 0.0,
            "satellites": int(getattr(gps_location, "satelliteCount", 0) or 0),
            "reliability": q["reliability"], "confidence": q["confidence"], "sensorAided": q["sensorAided"]}
  except (TypeError, ValueError):
    return {"accuracy": 0.0, "satellites": 0, "reliability": 0, "confidence": 0, "sensorAided": False}


class GpsTrust:
  """Whether the GPS position is good enough for the map to slow the car early. Pure; fed once per planner frame.

    * paused ("accuracy"): the GPS's own error above ACC_PAUSE (or reliability very low) for BAD_CONFIRM;
      paused ("lost"): RoadMatcher matched no road for LOST_TIME at LOST_SPEED or more where curve data exists.
      Resumes after RESUME_TIME of good, matched frames. While paused nothing is planned from the map.
    * low (per frame): another road is close enough to be the one we are on (RoadMatcher.ambiguous), or the error
      is between ACC_LOW and ACC_PAUSE (reliability low): a curve is held (logged, not slowed for).
    * shaky: low, paused, or not yet STEADY_TIME of matched driving (drive start, after a pause or a suspicious gap):
      the camera check may end a slowdown for any curve, and curve memory does not learn from passes.
  Honest limit: a constant offset onto a parallel road farther than the ambiguity distance, with the GPS saying it
  is accurate, looks like the right road until the curve; the camera check and the camera curve slowdown are the
  protection there, and curve memory is protected by the driven-turn check."""

  def __init__(self):
    self.reset_drive()

  def reset_drive(self):
    self.paused, self.reason = False, ""
    self.bad_t = self.lost_t = self.good_t = self.steady_t = self.gap_t = 0.0
    self.low = False
    self.shaky = True
    self.restarted = False                 # (T1c) the steady time was restarted this drive (a pause, a suspicious gap)
    self.events = []                       # (what, info) for the planner's log

  @property
  def status(self):
    if self.paused:
      return TRUST_PAUSED_LOST if self.reason == "lost" else TRUST_PAUSED_ACCURACY
    if self.low:
      return TRUST_LOW
    return TRUST_SHAKY if self.shaky else TRUST_OK

  @property
  def shaky_why(self):
    """(T1c, SHAKY_*) why the status is TRUST_SHAKY."""
    if self.status != TRUST_SHAKY:
      return SHAKY_NONE
    return SHAKY_RESTART if self.restarted else SHAKY_START

  def update(self, dt, v_ego, acc, rel, data_here, matched, ambiguous):
    """acc: m (0 unknown), rel: reliability 0-4 (0 not set), matched: RoadMatcher found a road."""
    bad = acc > ACC_PAUSE or rel == 1
    self.bad_t = self.bad_t + dt if bad else 0.0
    suspicious = v_ego >= LOST_SPEED and data_here and not matched
    if matched:
      self.lost_t = 0.0
    elif suspicious:
      self.lost_t += dt
    if not self.paused:
      if self.bad_t >= BAD_CONFIRM - TIMER_EPS:
        self._pause("accuracy", acc, rel)
      elif self.lost_t >= LOST_TIME - TIMER_EPS:
        self._pause("lost", acc, rel)
    else:
      good = (acc <= ACC_RESUME or acc <= 0.0) and rel != 1 and (matched or not data_here)
      self.good_t = self.good_t + dt if good else 0.0
      if self.good_t >= RESUME_TIME - TIMER_EPS:
        self.paused, self.good_t, self.lost_t, self.bad_t = False, 0.0, 0.0, 0.0
        self.events.append(("resume", {"reason": self.reason, "acc": acc, "rel": rel}))
        self.reason = ""
    self.low = (not self.paused) and (bool(ambiguous) or ACC_LOW <= acc < ACC_PAUSE or rel == 2)
    self.gap_t = self.gap_t + dt if suspicious else 0.0
    if self.paused or self.gap_t > STEADY_GAP:
      self.restarted = self.restarted or self.paused or self.steady_t > 0.0
      self.steady_t = 0.0
    elif matched:
      self.steady_t += dt
    self.shaky = self.paused or self.low or self.steady_t < STEADY_TIME - TIMER_EPS

  def _pause(self, reason, acc, rel):
    self.paused, self.reason, self.good_t, self.steady_t = True, reason, 0.0, 0.0
    self.restarted = True
    self.events.append(("pause", {"reason": reason, "acc": acc, "rel": rel, "unmatched_s": round(self.lost_t, 1)}))


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
  from an entrance ramp the car joins the freeway it merges into, not the next ramp).
  Taking an exit is therefore seen once the ramp is about MATCH_STAY away from the main road (typically
  50-150 m after the split); passing it is not mistaken for taking it."""

  def __init__(self, store):
    self.store = store
    self.way_id = 0
    self.chain_id = 0
    self.on_link = False
    self.road_class = 0           # highwayClass of the matched road (0 unknown)
    self.is_forward = True
    self.ambiguous = False        # another road (heading within MATCH_HEADING) closer than ambiguity_distance(acc)
    self.other_d = 0.0            # m to that nearest other road (0 none)
    self.nearest_d = 0.0          # m to the nearest road shape of any kind within NEAREST_SEARCH (0 none): for the logs

  @staticmethod
  def ambiguity_distance(acc):
    acc = acc if acc > 0 else AMB_DEFAULT_ACC
    return min(max(2.0 * acc + 5.0, AMB_MIN), AMB_MAX)

  def update(self, lat, lon, bearing, acc=0.0):
    """acc: the GPS's error estimate (m, 0 unknown) for the ambiguity check."""
    self.ambiguous, self.other_d, self.nearest_d = False, 0.0, 0.0
    grid = self.store.grid
    if not grid or lat is None:
      self.way_id = 0
      return 0, True
    amb = self.ambiguity_distance(acc)
    other = {}                    # chain -> nearest heading-compatible distance alongside (projection inside a segment)
    nearest = None
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
          t_raw = -(ax * dx + ay * dy) / ll
          t = max(0.0, min(1.0, t_raw))
          d = math.hypot(ax + t * dx, ay + t * dy)
          if nearest is None or d < nearest:
            nearest = d
          if d > max(MATCH_DIST, amb):
            continue
          diff = abs((math.degrees(math.atan2(dx, dy)) - bearing + 180.0) % 360.0 - 180.0)
          if diff <= MATCH_HEADING:
            fwd = True
          elif diff >= 180.0 - MATCH_HEADING and not one:
            fwd = False
          else:
            continue
          if 0.0 < t_raw < 1.0 and d < other.get(cid, float("inf")):
            other[cid] = d          # alongside (a road that only starts or ends here, e.g. at a junction, is not)
          if d > MATCH_DIST:
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
    if nearest is not None and nearest <= NEAREST_SEARCH:
      self.nearest_d = nearest
    pick = stay or kind or best
    if pick is None:
      self.way_id, self.chain_id, self.road_class = 0, 0, 0
      return 0, True
    _, self.way_id, self.is_forward, self.chain_id, self.on_link, self.road_class = pick
    others = [d for c, d in other.items() if c != self.chain_id]
    if others:
      self.other_d = min(others)
      self.ambiguous = self.other_d <= amb
    return self.way_id, self.is_forward


class MapCurveStore:
  """Curve files around the car, loaded in the background."""

  def __init__(self, root=None, background=True):
    self.root = CURVE_DIR if root is None else root
    self.stale_files = 0
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
          if d.get("version") != FORMAT_VERSION or int(d.get("builder", 0)) < BUILDER_VERSION:
            self.stale_files += 1      # (an old builder's file: ignored until curve_mapd rebuilds the folder)
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


