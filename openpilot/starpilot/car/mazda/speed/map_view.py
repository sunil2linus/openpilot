"""
mazda_mapviewd: the onroad map side. From the GPS and the curve files it writes a small "map view" for card (no capnp
change): the curve profiles ahead on the matched road as [distance ahead (m), radius (m)] points at the GPS sample's
time, the GPS trust state and the matched road's class. card (curve_plan.py) moves the points by the distance it
drove since that time.

Protocol: VIEW_PATH on tmpfs, one JSON object, replaced atomically (tmp + rename) at the GPS rate (1 Hz qcomgpsd .. 10 Hz ubloxd):
  {"v": 2, "t": monotonic s of the position (the GPS sample minus GPS_LAG), "gps": "ok" | "none" (no GPS position),
   "trust": GpsTrust status (0 ok ...), "start_ok": bool, "held": bool, "matched": bool, "cls": road class,
   "link": bool, "data": bool (curve files cover here), "chain": matched road (chain id, 0 none), "acc": GPS error m,
   "src": the GPS service used ("gpsLocation" qcomgpsd / "gpsLocationExternal" ubloxd), "why": with "gps": "none",
          why there is no position (status.GPS_NO_MESSAGES / GPS_NO_FIX / GPS_STALE),
   "curves": [{"id", "pts": [[d, r], ...], "ramp", "adv", "len"}],
   "ramp": the matched link's ramp record {"id", "hwy": leads to a highway, "spd": m/s of the road it leads to,
           "from_hwy": leaves a highway, "street": the class of the street the link meets ahead (LinkShapes; absent:
           none)} or absent,
   "exits": [{"id", "d": m to the exit link's start (negative: behind), "side": -1 left / 1 right / 0 unknown,
              "hwy": the link leads to a highway, "spd": m/s of the road it leads to, "pts": [[d, r], ...] the ramp's
              curve profile from the car, "m": the match (1 our highway, 2 this link, 0 other), "sh": the map shapes
              past the split (2 link, 1 highway, 0 no answer), "ly" / "hy": m right of the car where the link / the
              highway is MODEL_AT m ahead (null unknown)}]}  (at most EXITS_MAX, only while on or just past a highway)
The GPS: both services 7.0's GPS daemons publish, gpsLocation (qcomgpsd: comma 3X / tizi, e.g. the Konik A1M) and
gpsLocationExternal (ubloxd); the newest valid fix wins (7.0's own rule, starpilot/gps/source.select_source, without
its 25 m accuracy cut: GpsTrust pauses on a bad accuracy itself). Both are GpsLocationData: degrees, m/s, degrees,
m. flags are decoded (GpsTrust's reliability) only for qcomdiag; ublox flags mean something else.
The reader (read_view) never raises and returns None for a missing, unreadable or other-version view; how old a view
may be is the readers' one rule in view_age.py (alive up to VIEW_ALIVE s after its GPS message).

Exits ahead (part f, the exit plan in exit_plan.py): a ramp record of type exit / interchange whose start lies on the
matched highway road (within ON_CHAIN of its shape) ahead within EXIT_LOOK; kept (watched) until EXIT_KEEP behind,
also once the match has moved onto the link. Its side is where the link's shape lies 40-120 m on from its start,
relative to the highway's direction there. The shape answer past the split is the old fork's (map_curves.near_shapes /
shape_verdict): "link" when the car is closer to the link's shape and heads closer to it by SHAPE_LINK_GAP; "highway"
when closer to the highway and heading closer, or heading SHAPE_HWY_HEADING_GAP closer to the highway within
SHAPE_HWY_NEAR of it (a car going on in the right lane is near the link's first shape). No navigation is used.

Curves ahead (ported from map_curves.MapCurves._curves_ahead, simplified): the curves of the matched chain in our
direction within LOOKAHEAD, never one on a later way of the chain driven against its one-way direction; a curve
ahead is placed by the straight distance to its entry (never more than the road distance: slows early, not late);
a curve we are in by the distance driven since its entry (or, entered unseen, its length minus the distance to its
exit). The profile is mirrored for travel b -> a.
"""
import json
import math
import os
import time

from openpilot.starpilot.car.mazda.speed import map_match as MM
from openpilot.starpilot.car.mazda.speed import status as ST
from openpilot.starpilot.car.mazda.speed.curve_memory import way_place
from openpilot.starpilot.car.mazda.speed.view_age import GPS_LAG, VIEW_ALIVE

VIEW_PATH = "/dev/shm/mazda_speed_view.json"
VIEW_VERSION = 2
# GPS_LAG (view_age.py): the position is this old when it arrives (0.7-1.0 s measured)
GPS_STALE = VIEW_ALIVE        # s without a new fix: "none" (as long as a reader keeps a view alive: one missed 1 Hz fix)
GPS_SERVICES = ("gpsLocationExternal", "gpsLocation")   # ubloxd, qcomgpsd (7.0 runs one of them per device)
LOOKAHEAD = 1000.0            # m
START_ACC = 15.0              # m: start-of-drive trust on a matched link needs the GPS error at or below this
IN_CURVE_SLACK = 60.0         # m past an exit a curve still counts as the one we are in (the old value)
HIGHWAY_CLASSES = (1, 3)      # motorway, trunk
EXIT_TYPES = ("exit", "interchange")
EXIT_LOOK = 1500.0            # m: exits watched from this far ahead
EXIT_KEEP = 600.0             # m behind their start: forgotten
EXITS_MAX = 2
ON_CHAIN = 6.0                # m: the link's start lies on the matched highway's shape
SIDE_FROM, SIDE_TO = 40.0, 120.0   # m along the link from its start: where its side is measured
SIDE_MIN = 2.0                # m to one side at least (else 0: unknown)
SHAPE_FROM = 10.0             # m past the split before the shapes answer (at the split itself they may say "highway")
SHAPE_TO = 400.0
SHAPE_RANGE = 40.0            # m
SHAPE_HEADING = 60.0          # deg
SHAPE_MIN_SPEED = 10.0        # m/s (the GPS heading is noise in a jam)
SHAPE_LINK_GAP = 1.0          # deg
SHAPE_HWY_HEADING_GAP, SHAPE_HWY_NEAR, SHAPE_HWY_HEADING_MAX = 3.5, 12.0, 1.0
MODEL_AT = 60.0               # m ahead of the car (now): where the link / highway laterals are given for the camera check
LATERAL_FROM, LATERAL_TO = -150.0, 200.0   # m to the split: the laterals are given only here
COARSE = 50                   # ramp index cells of 1/50 deg (~2.2 km)
# Where a link really leads (read from the road shapes already in the files; no rebuild): see LinkShapes
STREET_CLASSES = (5, 7, 9, 11, 12)    # primary .. residential (not links, not motorway / trunk)
STREET_SPEED_MPH = {5: 50, 7: 45, 9: 45, 11: 40, 12: 25}   # (curve_builder.DEFAULT_SPEED_MPH: the map has no limit here)
JUNCTION_NEAR = 3.0           # m: a street shape this close to a joint of the link's ways: the link meets it there
SHAPE_SEARCH = 150.0          # m between the points where the link's own shapes are looked up


def directed_profile(c, increasing):
  prof = c.get("prof") or [[0.0, c["r"]], [c.get("len", 0.0), c["r"]]]
  if increasing:
    return [(float(s), float(r)) for s, r in prof]
  length = float(c.get("len", prof[-1][0]))
  return [(length - float(s), float(r)) for s, r in reversed(prof)]


class CurvesAhead:
  def __init__(self, store):
    self.store = store
    self.entered = {}           # curve id -> odometer at its entry
    self.chain = None
    self.odo = 0.0
    self._ow = None

  def oneway_ways(self):
    key = tuple(id(ri) for ri in self.store.grid)
    if self._ow is None or self._ow[0] != key:
      self._ow = (key, {int(ri.way[k]) for ri in self.store.grid for k in range(len(ri.way)) if ri.oneway[k]})
    return self._ow[1]

  def against_oneway(self, c, increasing):
    if not c.get("oneway") or not self.store.grid:
      return False
    ow = self.oneway_ways()
    return any(int(w) in ow and bool(fwd) != bool(increasing) for w, fwd in c.get("wdir", []))

  def way_place(self, cid, c, increasing):
    """(curve_memory.way_place of the curve's start along its first way, our direction along that way's node order)
    from the chain's way distances; (None, None) unknown."""
    ways = c.get("ways") or []
    ch = self.store.chains.get(cid)
    if not ways or ch is None:
      return None, None
    w = ch["w"]
    for k, (wid, fwd, s0) in enumerate(w):
      if int(wid) == int(ways[0]):
        s1 = w[k + 1][2] if k + 1 < len(w) else ch["len"]
        off = (c["s_a"] - s0) if fwd else (s1 - c["s_b"])
        return way_place(off), ("ab" if bool(fwd) == bool(increasing) else "ba")
    return None, None

  def road(self, way_id, is_forward):
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

  def update(self, lat, lon, bearing, way_id, is_forward, dist_driven):
    self.odo += max(dist_driven, 0.0)
    road = self.road(way_id, is_forward) if way_id else None
    if road is None:
      return []
    cid, increasing, s0, s1 = road
    if cid != self.chain:
      self.chain, self.entered = cid, {}
    out = []
    for c in self.store.curves.get(cid, []):
      key = f"{cid}:{c['s_a']}:{'ab' if increasing else 'ba'}"
      entry, exit_ = (c["a"], c["b"]) if increasing else (c["b"], c["a"])
      s_entry, s_exit = (c["s_a"], c["s_b"]) if increasing else (c["s_b"], c["s_a"])
      if increasing and (s_exit < s0 or s_entry > s1 + LOOKAHEAD):
        continue
      if not increasing and (s_exit > s1 or s_entry < s0 - LOOKAHEAD):
        continue
      length = float(c.get("len", 0.0))
      later_way = (s_entry > s1) if increasing else (s_entry < s0)
      if later_way and self.against_oneway(c, increasing):
        continue
      on_this_way = (s0 <= s_entry <= s1) if increasing else (s0 <= s_entry <= s1)
      ent_ahead, d_entry = MM.ahead(lat, lon, bearing, *entry)
      ex_ahead, d_exit = MM.ahead(lat, lon, bearing, *exit_)
      if key in self.entered:
        s_in = self.odo - self.entered[key]
        if s_in > length + IN_CURVE_SLACK:
          continue
        d0 = -s_in
      elif later_way or (on_this_way and ent_ahead):
        if d_entry > LOOKAHEAD:
          continue
        d0 = d_entry
        if d_entry <= MM.NEAR:
          self.entered[key] = self.odo + d_entry
      elif ex_ahead and d_exit <= length + IN_CURVE_SLACK:
        s_in = max(length - d_exit, 0.0)
        self.entered[key] = self.odo - s_in
        d0 = -s_in
      else:
        continue
      pts = [[round(d0 + s, 1), r] for s, r in directed_profile(c, increasing)]
      place, wdir = self.way_place(cid, c, increasing)
      out.append({"id": key, "pts": pts, "ramp": bool(c.get("ramp")) or int(c.get("cls", 0)) in MM.LINK_CLASSES,
                  "adv": float(c.get("adv", 0.0) or 0.0), "len": length, "r": float(c["r"]),
                  "way": int((c.get("ways") or [0])[0]), "name": str(c.get("name", "")),
                  **({"place": place, "wdir": wdir} if place is not None else {})})
    return out


def _local(lat0, lon0):
  kx = math.radians(1.0) * MM.EARTH_R * math.cos(math.radians(lat0))
  ky = math.radians(1.0) * MM.EARTH_R
  return lambda la, lo: ((lo - lon0) * kx, (la - lat0) * ky)


def chain_segments(store, chain_id, lat, lon, cells=1):
  """[(lat1, lon1, lat2, lon2, oneway)] of one chain's road shape around a point (the 3 x 3 match cells: +-330 m)."""
  out = []
  cla, clo = int(math.floor(lat * MM.MATCH_CELL)), int(math.floor(lon * MM.MATCH_CELL))
  seen = set()
  for dla in range(-cells, cells + 1):
    for dlo in range(-cells, cells + 1):
      for ri in store.grid:
        for la1, lo1, la2, lo2, _wid, cid, one, _cls in ri.segments(MM.cell_key(cla + dla, clo + dlo)):
          if cid != chain_id:
            continue
          k = (la1, lo1, la2, lo2)
          if k not in seen:
            seen.add(k)
            out.append((la1, lo1, la2, lo2, one))
  return out


def _nearest(segs, to_xy, heading=None, max_turn=SHAPE_HEADING):
  """(m, deg off `heading`) of the nearest segment to the origin (either way along it unless one-way), or None."""
  best = None
  for la1, lo1, la2, lo2, one in segs:
    ax, ay = to_xy(la1, lo1)
    bx, by = to_xy(la2, lo2)
    dx, dy = bx - ax, by - ay
    ll = dx * dx + dy * dy
    if ll < 1e-6:
      continue
    t = max(0.0, min(1.0, -(ax * dx + ay * dy) / ll))
    d = math.hypot(ax + t * dx, ay + t * dy)
    diff = 0.0
    if heading is not None:
      diff = abs((math.degrees(math.atan2(dx, dy)) - heading + 180.0) % 360.0 - 180.0)
      if not one:
        diff = min(diff, 180.0 - diff)
      if diff > max_turn:
        continue
    if best is None or d < best[0]:
      best = (d, diff)
  return best


def shape_verdict(a, b, gap=0.0, b_heading=None):
  """(ported) 1 when shape a is closer AND heads closer (by `gap` deg) than b, -1 when b is closer and heads closer (or
  b_heading: heads b_heading[0] closer within b_heading[1] m and b_heading[2] deg of ours), 0 neither."""
  if a is None and b is None:
    return 0
  if b is None:
    return 1
  if a is None:
    return -1
  if a[0] < b[0] and a[1] + gap <= b[1]:
    return 1
  if b[0] < a[0] and b[1] < a[1]:
    return -1
  if b_heading is not None and b[1] + b_heading[0] <= a[1] and b[0] <= b_heading[1] and b[1] <= b_heading[2]:
    return -1
  return 0


def lateral_at(segs, to_xy, bearing, along):
  """m right of the car where the shape crosses the line `along` m ahead (the crossing nearest the car's line), or
  None. to_xy: the car's local frame (x east, y north)."""
  h = math.radians(bearing)
  fx, fy = math.sin(h), math.cos(h)        # forward
  rx, ry = math.cos(h), -math.sin(h)       # right
  best = None
  for la1, lo1, la2, lo2, _one in segs:
    ax, ay = to_xy(la1, lo1)
    bx, by = to_xy(la2, lo2)
    fa, fb = ax * fx + ay * fy, bx * fx + by * fy
    if (fa - along) * (fb - along) > 0 or abs(fb - fa) < 1e-6:
      continue
    f = (along - fa) / (fb - fa)
    x = (ax * rx + ay * ry) + f * ((bx * rx + by * ry) - (ax * rx + ay * ry))
    if best is None or abs(x) < abs(best):
      best = x
  return best


class LinkShapes:
  """Where a link (ramp chain) really leads, from the road shapes the curve files already carry (cached per ramp):

  * `cuts`: the joints between the link's ways (in travel order) where a street (STREET_CLASSES) passes within
    JUNCTION_NEAR: the link meets that street there. The builder's chain joining (same class, no name) runs straight
    through such a junction, so an exit ramp and the next on-ramp across the street are ONE chain whose record says
    "to the motorway" (r2: M-53's exit to 23 Mile Rd; the merge armed toward 70 on the exit). Before the first cut the
    link leads to that street; past the last cut it comes from a street.
  Never raises: a link whose shapes are not loaded gives no cuts (the record as built)."""

  def __init__(self, store):
    self.store = store
    self.cache = {}
    self._key = None

  def _segs(self, wids, pts):
    """{way id: set of (a, b) micro-degree segments} of the given ways near the given points."""
    out = {w: set() for w in wids}
    cells = set()
    for la, lo in pts:
      cla, clo = int(math.floor(la * MM.MATCH_CELL)), int(math.floor(lo * MM.MATCH_CELL))
      for dla in (-1, 0, 1):
        for dlo in (-1, 0, 1):
          cells.add(MM.cell_key(cla + dla, clo + dlo))
    for ri in self.store.grid:
      for cell in cells:
        for i in ri.cells.get(cell, ()):
          w = int(ri.way[i])
          if w in out:
            out[w].add(((ri.lat1[i], ri.lon1[i]), (ri.lat2[i], ri.lon2[i])))
    return out

  @staticmethod
  def _polyline(segs):
    """The way's points in its node order from its (directed) segments, or []."""
    if not segs:
      return []
    nxt = {a: b for a, b in segs}
    ends = {b for _a, b in segs}
    starts = [a for a in nxt if a not in ends]
    p = starts[0] if starts else next(iter(nxt))
    out, seen = [p], {p}
    while p in nxt and nxt[p] not in seen:
      p = nxt[p]
      out.append(p)
      seen.add(p)
    return out

  def _street_near(self, p, exclude):
    """The class of a street shape within JUNCTION_NEAR of micro-degree point p (0 none)."""
    la, lo = p[0] / 1e6, p[1] / 1e6
    to_xy = _local(la, lo)
    cla, clo = int(math.floor(la * MM.MATCH_CELL)), int(math.floor(lo * MM.MATCH_CELL))
    for ri in self.store.grid:
      for dla in (-1, 0, 1):
        for dlo in (-1, 0, 1):
          for la1, lo1, la2, lo2, wid, cid, _one, cls in ri.segments(MM.cell_key(cla + dla, clo + dlo)):
            if cls not in STREET_CLASSES or int(wid) in exclude:
              continue
            near = _nearest([(la1, lo1, la2, lo2, 0)], to_xy)
            if near is not None and near[0] <= JUNCTION_NEAR:
              return int(cls)
    return 0

  def info(self, r):
    """{"cuts": [(chain-order index k of the way before the joint, street class)], "cut_s": m along
    the link (travel) to the first cut or None} for ramp record r."""
    key = tuple(id(ri) for ri in self.store.grid)
    if key != self._key:
      self.cache, self._key = {}, key
    cid = r.get("chain")
    if cid in self.cache:
      return self.cache[cid]
    out = {"cuts": [], "cut_s": None}
    try:
      out = self._info(r) or out
    except Exception:
      pass
    self.cache[cid] = out
    return out

  def _info(self, r):
    ch = self.store.chains.get(r.get("chain"))
    if ch is None or not r.get("start") or not r.get("end") or r.get("travel") == "both":
      return None
    wids = [int(w) for w, _f, _s in ch["w"]]
    (la0, lo0), (la1, lo1) = r["start"], r["end"]
    n = max(1, int(float(r.get("len", 0.0) or 0.0) / SHAPE_SEARCH) + 1)
    pts = [(la0 + (la1 - la0) * k / n, lo0 + (lo1 - lo0) * k / n) for k in range(n + 1)]
    segs = self._segs(set(wids), pts)
    lines = []
    for w, fwd, _s0 in ch["w"]:
      line = self._polyline(segs.get(int(w)))
      if len(line) < 2:
        return None
      lines.append(line if fwd else line[::-1])
    increasing = r.get("travel") != "ba"
    cuts = []
    for k in range(len(lines) - 1):
      cls = self._street_near(lines[k][-1], set(wids))
      if cls:
        cuts.append((k, cls))
    cut_s = None
    if cuts:
      s0 = [float(s) for _w, _f, s in ch["w"]]
      js = [s0[k + 1] for k, _c in cuts]
      length = float(ch.get("len", 0.0) or 0.0)
      cut_s = min(js) if increasing else length - max(js)
    return {"cuts": cuts, "cut_s": cut_s}

  def leads(self, r, way_index=None):
    """(to a highway, m/s of the road it leads to, from a highway, the street class it meets ahead (0 none))
    of ramp r for a car on the link's way `way_index` (chain order; None: at the link's start)."""
    to, frm = r.get("to") or {}, r.get("from") or {}
    to_cls, to_spd = int(to.get("cls", 0) or 0), float(to.get("speed", 0.0) or 0.0)
    hwy = to_cls in HIGHWAY_CLASSES
    from_hwy = int(frm.get("cls", 0) or 0) in HIGHWAY_CLASSES
    inf = self.info(r)
    increasing = r.get("travel") != "ba"
    cuts = sorted(inf["cuts"], reverse=not increasing)          # in travel order
    ahead = [cls for k, cls in cuts if way_index is None or (way_index <= k if increasing else way_index > k)]
    street = ahead[0] if ahead else 0
    if street:
      hwy, to_spd = False, STREET_SPEED_MPH.get(street, 45) * 0.44704
    if len(ahead) < len(cuts):
      from_hwy = False                     # a street behind on the link: it comes from that street
    return hwy, to_spd, from_hwy, street


class ExitsAhead:
  """The exits ahead on the matched highway, watched past their split (see the module doc)."""

  def __init__(self, store, links=None):
    self.store = store
    self.links = links if links is not None else LinkShapes(store)
    self.watch = {}             # ramp chain -> {"rec", "hwy": highway chain, "side", "pts": [(s, r)]}
    self._idx = None
    self._no = set()            # (ramp chain, highway chain) checked: not on that highway

  def _index(self):
    key = id(self.store.ramps_all)
    if self._idx is None or self._idx[0] != key:
      cells = {}
      for r in self.store.ramps_all:
        if r.get("type") in EXIT_TYPES and r.get("start"):
          la, lo = r["start"]
          cells.setdefault((int(math.floor(la * COARSE)), int(math.floor(lo * COARSE))), []).append(r)
      self._idx = (key, cells)
      self._no = set()
    return self._idx[1]

  def _profile(self, r):
    ch = self.store.chains.get(r.get("chain"))
    length = float(ch["len"]) if ch else 0.0
    increasing = r.get("travel") != "ba"
    pts = []
    for c in self.store.curves.get(r.get("chain"), []):
      s_entry = c["s_a"] if increasing else length - c["s_b"]
      if s_entry < -1.0:
        continue
      pts += [(s_entry + s, rr) for s, rr in directed_profile(c, increasing)]
    pts.sort()
    return pts

  def _side(self, r, hwy_segs, bearing):
    """-1 / 1: the link lies left / right of the highway's direction SIDE_FROM-SIDE_TO m on (0: unknown); and the
    link's own departure heading at its start (deg, None unknown)."""
    la0, lo0 = r["start"]
    to_xy = _local(la0, lo0)
    best, heading = None, None
    for la1, lo1, la2, lo2, _one in hwy_segs:
      ax, ay = to_xy(la1, lo1)
      bx, by = to_xy(la2, lo2)
      dx, dy = bx - ax, by - ay
      ll = dx * dx + dy * dy
      if ll < 1e-6:
        continue
      t = max(0.0, min(1.0, -(ax * dx + ay * dy) / ll))
      d = math.hypot(ax + t * dx, ay + t * dy)
      if best is None or d < best:
        best, heading = d, math.degrees(math.atan2(dx, dy))
    link = chain_segments(self.store, r.get("chain"), la0, lo0)
    depart = None
    for la1, lo1, la2, lo2, _one in link:
      (ax, ay), (bx, by) = to_xy(la1, lo1), to_xy(la2, lo2)
      if math.hypot(ax, ay) <= ON_CHAIN:
        depart = math.degrees(math.atan2(bx - ax, by - ay))
      elif math.hypot(bx, by) <= ON_CHAIN and depart is None:
        depart = math.degrees(math.atan2(ax - bx, ay - by))
    if heading is None:
      return 0, depart
    if abs((heading - bearing + 180.0) % 360.0 - 180.0) > 90.0:
      heading += 180.0             # the shape's node order runs against our travel
    lats = [x for x in (lateral_at(link, to_xy, heading, a) for a in range(int(SIDE_FROM), int(SIDE_TO) + 1, 20))
            if x is not None]
    if not lats:
      return 0, depart
    m = sum(lats) / len(lats)
    return (0 if abs(m) < SIDE_MIN else (1 if m > 0 else -1)), depart

  def _find(self, lat, lon, bearing, hwy_chain):
    cells = self._index()
    cla, clo = int(math.floor(lat * COARSE)), int(math.floor(lon * COARSE))
    for dla in (-1, 0, 1):
      for dlo in (-1, 0, 1):
        for r in cells.get((cla + dla, clo + dlo), []):
          rid = r.get("chain")
          if rid in self.watch or (rid, hwy_chain) in self._no:
            continue
          ahead, dist = MM.ahead(lat, lon, bearing, *r["start"])
          if not ahead or dist > EXIT_LOOK:
            continue
          la0, lo0 = r["start"]
          hwy = chain_segments(self.store, hwy_chain, la0, lo0)
          near = _nearest(hwy, _local(la0, lo0))
          if near is None or near[0] > ON_CHAIN:
            self._no.add((rid, hwy_chain))
            continue
          # the link must leave in our direction (a two-way trunk road: the other direction's ramp)
          side, depart = self._side(r, hwy, bearing)
          if depart is not None and abs((depart - bearing + 180.0) % 360.0 - 180.0) > 60.0:
            self._no.add((rid, hwy_chain))
            continue
          hwy_to, spd, _from, _street = self.links.leads(r)
          pts = self._profile(r)
          cut = self.links.info(r)["cut_s"]
          if cut is not None:
            pts = [(s, rr) for s, rr in pts if s <= cut]      # the link ends at the street it meets (LinkShapes)
          self.watch[rid] = {"rec": r, "hwy": hwy_chain, "side": side, "pts": pts, "to_hwy": hwy_to, "spd": spd}

  def update(self, lat, lon, bearing, speed, chain_id, road_class):
    if chain_id and road_class in HIGHWAY_CLASSES and self.store.grid:
      self._find(lat, lon, bearing, chain_id)
    out = []
    for rid, w in list(self.watch.items()):
      r = w["rec"]
      ahead, dist = MM.ahead(lat, lon, bearing, *r["start"])
      d = dist if ahead else -dist
      if d < -EXIT_KEEP or d > EXIT_LOOK + 200.0 or (d > 0 and chain_id and chain_id != w["hwy"]):
        del self.watch[rid]          # (an exit ahead on a highway the car has left; found again when back on it)
        continue
      e = {"id": f"x{rid}", "d": round(d, 1), "side": w["side"], "hwy": bool(w["to_hwy"]),
           "spd": float(w["spd"]), "pts": [[round(d + s, 1), rr] for s, rr in w["pts"]],
           "m": 1 if chain_id == w["hwy"] else (2 if chain_id == rid else 0), "sh": 0, "ly": None, "hy": None}
      to_xy = _local(lat, lon)
      link = hwy = None
      if -SHAPE_TO <= d <= LATERAL_TO:
        link = chain_segments(self.store, rid, lat, lon)
        hwy = chain_segments(self.store, w["hwy"], lat, lon)
      if -SHAPE_TO <= d <= -SHAPE_FROM and speed >= SHAPE_MIN_SPEED:
        v = shape_verdict(_nearest(link, to_xy, bearing) if link else None, _nearest(hwy, to_xy, bearing) if hwy else None,
                          SHAPE_LINK_GAP, (SHAPE_HWY_HEADING_GAP, SHAPE_HWY_NEAR, SHAPE_HWY_HEADING_MAX))
        e["sh"] = 2 if v > 0 else (1 if v < 0 else 0)
      if LATERAL_FROM <= d <= LATERAL_TO:
        along = MODEL_AT + max(speed, 0.0) * GPS_LAG
        ly = lateral_at(link, to_xy, bearing, along) if link else None
        hy = lateral_at(hwy, to_xy, bearing, along) if hwy else None
        e["ly"] = None if ly is None else round(ly, 2)
        e["hy"] = None if hy is None else round(hy, 2)
      out.append(e)
    out.sort(key=lambda e: (e["d"] < -SHAPE_TO, abs(e["d"])))
    return out[:EXITS_MAX]


def write_view(view, path=VIEW_PATH):
  tmp = path + ".tmp"
  with open(tmp, "w") as f:
    json.dump(view, f, separators=(",", ":"))
  os.replace(tmp, path)


def read_view(path=VIEW_PATH):
  try:
    with open(path) as f:
      v = json.load(f)
  except (OSError, ValueError):
    return None
  if not isinstance(v, dict) or v.get("v") != VIEW_VERSION or not isinstance(v.get("t"), (int, float)):
    return None
  return v


class MapView:
  """The daemon's per-GPS-sample work, without the sockets (tests drive it directly)."""

  def __init__(self, store):
    self.store = store
    self.matcher = MM.RoadMatcher(store)
    self.trust = MM.GpsTrust()
    self.ahead = CurvesAhead(store)
    self.links = LinkShapes(store)
    self.exits = ExitsAhead(store, self.links)
    self.last_t = None

  def update(self, t, gps):
    """t: monotonic s of the GPS sample; gps: dict(lat, lon, bearing, speed, acc, fix). Returns the view dict."""
    view = {"v": VIEW_VERSION, "t": t - GPS_LAG, "gps": "none", "trust": MM.TRUST_NO_GPS, "curves": [],
            "matched": False, "cls": 0, "link": False, "held": False, "start_ok": False, "data": False}
    if gps is None or not gps.get("fix"):
      self.last_t = None
      return view
    dt = 0.1 if self.last_t is None else min(max(t - self.last_t, 0.0), 1.0)
    self.last_t = t
    lat, lon, bearing, speed = gps["lat"], gps["lon"], gps["bearing"], max(gps.get("speed", 0.0), 0.0)
    acc = float(gps.get("acc", 0.0) or 0.0)
    self.store.update(lat, lon)
    self.store.rescan_if_changed()
    data = self.store.covers(lat, lon)
    way_id, fwd = self.matcher.update(lat, lon, bearing, acc if MM.USE_GPS_ACCURACY else 0.0)
    matched = bool(way_id)
    self.trust.update(dt, speed, acc if MM.USE_GPS_ACCURACY else 0.0, int(gps.get("rel", 0) or 0), data, matched,
                      self.matcher.ambiguous)
    view.update(gps="ok", trust=self.trust.status, matched=matched, cls=self.matcher.road_class,
                link=bool(self.matcher.on_link), data=data, chain=int(self.matcher.chain_id) if matched else 0,
                acc=round(acc, 1))
    ramp = self.store.ramps.get(self.matcher.chain_id) if matched and self.matcher.on_link else None
    if ramp is not None:
      hit = self.store.way_chain.get(way_id)
      k = hit[1] if hit is not None and hit[0] == self.matcher.chain_id else None
      hwy, spd, from_hwy, street = self.links.leads(ramp, k)
      view["ramp"] = {"id": int(self.matcher.chain_id), "hwy": hwy, "spd": spd, "from_hwy": from_hwy}
      if street:
        view["ramp"]["street"] = street
    # start of the drive: a matched highway link with a good fix is trusted for stepping up in its curves
    view["start_ok"] = bool(self.trust.status == MM.TRUST_SHAKY and self.trust.shaky_why == MM.SHAKY_START and
                            matched and self.matcher.on_link and 0.0 < acc <= START_ACC)
    if self.trust.paused:
      view["paused"] = True
      return view
    curves = self.ahead.update(lat, lon, bearing, way_id, fwd, speed * dt) if matched else []
    if self.trust.low and curves:
      view["held"] = True        # another road this close may be ours: logged, not slowed for
    view["curves"] = curves
    exits = self.exits.update(lat, lon, bearing, speed, self.matcher.chain_id if matched else 0,
                              self.matcher.road_class if matched else 0)
    if exits:
      view["exits"] = exits
    return view


def gps_sample(g, stamp, now, valid=True):
  """One GpsLocationData of either service (gpsLocation from qcomgpsd, gpsLocationExternal from ubloxd) -> the dict
  MapView.update takes. stamp: the message's monotonic s. Never raises on odd values (a non-finite number is no fix)."""
  def num(v, default=0.0):
    try:
      v = float(v)
    except (TypeError, ValueError):
      return default
    return v if math.isfinite(v) else default
  lat, lon = num(g.latitude, math.nan), num(g.longitude, math.nan)
  try:
    src = str(g.source)
  except AttributeError as e:
    from openpilot.starpilot.car.mazda.speed.events import schema_miss
    schema_miss(None, "gpsLocation.source", e)      # (7.0 has it on both services: tests/test_cereal_fields.py)
    src = ""
  fix = bool(valid and g.hasFix and 0.0 <= now - stamp < GPS_STALE and math.isfinite(lat) and math.isfinite(lon) and
             abs(lat) <= 90.0 and abs(lon) <= 180.0 and not (lat == 0.0 and lon == 0.0))
  acc = num(g.horizontalAccuracy)
  rel = MM.gps_flags(g.flags)["reliability"] if src == "qcomdiag" else 0
  return {"lat": lat, "lon": lon, "bearing": num(g.bearingDeg) % 360.0, "speed": max(num(g.speed), 0.0),
          "acc": acc if acc > 0.0 else 0.0, "rel": rel, "fix": fix, "src": src}


def pick_gps(sm, now):
  """(stamp s, sample, service) to process this cycle, or None. The newest valid fix of the two services wins and is
  processed when its service sent it this cycle; with no fix anywhere the newest new message (no fix) is processed."""
  best = nofix = None
  for s in GPS_SERVICES:
    if not sm.seen[s]:
      continue
    stamp = sm.logMonoTime[s] * 1e-9
    g = gps_sample(sm[s], stamp, now, sm.valid[s])
    if g["fix"]:
      if best is None or stamp > best[0]:
        best = (stamp, g, s)
    elif sm.updated[s] and (nofix is None or stamp > nofix[0]):
      nofix = (stamp, g, s)
  if best is not None:
    return best if sm.updated[best[2]] else None
  return nofix


def no_gps_why(sm, now):
  """Why there is no position: no message from either GPS service yet, the messages stopped, or no fix."""
  seen = [sm.logMonoTime[s] * 1e-9 for s in GPS_SERVICES if sm.seen[s]]
  if not seen:
    return ST.GPS_NO_MESSAGES
  return ST.GPS_STALE if now - max(seen) >= GPS_STALE else ST.GPS_NO_FIX


class DriveGps:
  """This drive's GPS state for the settings rows (status.GPS_NAME): "ok" once the drive has had a position, until
  then why it has none. Returns the dict to write when it changed (a handful of writes per drive)."""

  def __init__(self):
    self.state, self.src, self.fix_s, self.last = None, "", 0.0, None

  def update(self, now, fix, why, src=""):
    if fix and self.last is not None:
      self.fix_s += min(max(now - self.last, 0.0), 1.0)
    self.last = now if fix else None
    state = ST.GPS_OK if (fix or self.state == ST.GPS_OK) else why
    if fix and src:
      self.src = src
    if state != self.state:
      self.state = state
      return {"state": state, "src": self.src}
    return None


def main():
  from openpilot.cereal import messaging
  from openpilot.common.params import Params
  from openpilot.common.swaglog import cloudlog
  from openpilot.starpilot.car.mazda.speed import config as C
  try:
    os.nice(5)
  except OSError:
    pass
  if not C.read(Params()).needs_view:
    while True:                  # (the manager only starts it with the setting on; settings apply per drive)
      time.sleep(60)
  store = MM.MapCurveStore()
  mv = MapView(store)
  sm = messaging.SubMaster(list(GPS_SERVICES))
  drive = DriveGps()
  last_write = 0.0
  while True:
    sm.update(200)
    now = time.monotonic()
    try:
      got = pick_gps(sm, now)
      if got is not None:
        t, gps, service = got
        view = mv.update(t, gps)
        view["src"] = service
        if not gps["fix"]:
          view["why"] = ST.GPS_NO_FIX
        write_view(view)
        last_write = now
        changed = drive.update(now, gps["fix"], ST.GPS_NO_FIX, service)
      elif now - last_write > GPS_STALE:
        view = mv.update(now, None)
        view["why"] = no_gps_why(sm, now)
        write_view(view)
        last_write = now
        changed = drive.update(now, False, view["why"])
      else:
        changed = None
      if changed is not None:
        ST.write(ST.GPS_NAME, changed)
        cloudlog.warning(f"mazda_mapviewd: GPS {changed['state']} {changed['src']}")
    except Exception:
      cloudlog.exception("mazda_mapviewd failed")
      time.sleep(1.0)


if __name__ == "__main__":
  main()
