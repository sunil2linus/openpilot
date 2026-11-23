"""
The speed limit contributor (part f, F1/F2/F5): the posted limit -> ONE anchor proposal (planner.py takes it only by
its anchor rules). It never gives a slowing target and never presses.

Where the limit comes from (F1): 7.0's Speed Limit Controller runs DISPLAY-ONLY on this car (feature_runtime.
slc_runtime_settings: no openpilot longitudinal) and plannerd publishes its arbitration as `slcState` (20 Hz):
`speedLimit` with `observationKind` "valid" and `source`, each source's own reading in `sourceReadings`, and
(`upcomingSpeedLimit` is NOT used: on this car's display-only SLC it is never filled). It runs with "Show Speed Limits" on (the default) and a map / dash
source in the SLC source priority ("Map Data": the offline maps provider must be running). card reads slcState
through the speed control's private SubMaster (contributors.Sources), only when an option that needs it is on.
Only "map" and "dashboard" readings count (LIMIT_SOURCES): vision sign readings were never checked on this car (the
old fork did not follow them either); when 7.0 selected vision, the map / dash row of sourceReadings is used.

Rules (ported from the old fork's speed_limit_follow, written as one contributor):
  * A reading is a limit only as a whole multiple of 5 mph (10 km/h) between LIMIT_MIN and LIMIT_MAX.
  * A new limit must be reported unchanged with clean GPS trust (the map view's trust 0) for the whole steady time
    (LOWER_STEADY for a lower one, RAISE_STEADY for a higher or first one); a bad-trust frame restarts it, and so does
    a frame without a live map view (view_age.py); while the view is LATE (a missed GPS fix) it holds. A gap in the
    readings keeps the last steady limit for LIMIT_KEEP.
  * "Not sure which road" (drive 0000000c, Hall Rd: slcState's map row valid on ~18% of the samples): 7.0's mapd says
    "unknown" for whole 1 s fixes while ways compete and "possible" on the first fix of a new match; slcState is
    "valid" only for a "current" one. So the steady time HOLDS (neither grows nor restarts) through a gap of readings
    up to GAP_HOLD (two missed 1 Hz fixes, like view_age's LATE) and restarts only on a DIFFERENT value or a gap
    longer than that. mapdOut's own limit on a "possible" fix (map_possible, the private SubMaster) counts as the
    same reading when it equals the candidate; it never starts a candidate or a different value on its own.
  * The proposal: the steady limit (dash, whole steps, no margin), + limit_offset_mph only on a STREET (the owner's
    "street speed: limit + 3"; the old fork's Street Speed): the last known road class of the map view (frames with
    class 0 skipped, as the old fork) is known and not a highway nor a link. On a highway, a link or a road whose
    class was never known the target is the posted limit itself. At most TARGET_MAX.
    The planner takes it when the anchor is ours (our limit raise, merge, exit handover: re-capped by a lower limit in
    any band), or when he engaged (SET / RES) at a speed within the band of the limit it replaces (the old limit up to
    + max(LIMIT_BAND, the offset on a street); at a first limit: the new limit's band, on a street only). His own
    higher speed (80 on a 70 road) is out of the band: never pressed down.
  * The limit change rule (the old fork's rules 1 and 9): his own speed (his SET+ / SET-, or his engaged speed out of
    the band) is re-taken only by a NEW limit, one that became steady after his last press / engagement (`seq`,
    planner.py). The proposal is marked `retake` while it is fresh (RETAKE_WINDOW after it became steady). Under his
    speed only for a limit that went down, only on a street (a live view, road class known, not a highway nor a
    link), once that limit has been reported unchanged with clean trust for RETAKE_LOWER (RETAKE_LOWER_BIG for a drop
    of more than BIG_DROP): his 62 on a new 55 road stays. A raise needs his speed in the band of the limit it
    replaces (the planner checks it). His next SET+ / SET- after that is his again, until the next limit change.
  * The limit it replaces (as the old fork): the steady one before, or, after a gap of readings longer than
    LIMIT_KEEP (7.0's map provider says "ambiguous road" for whole stretches), the last steady one within MATCH_KEEP;
    a lower limit his speed was not at in between (a cross road while stopped) does not count when his speed was in
    the band of an older one (`matched`, within MATCH_KEEP) - only when no old limit is known or the change from the
    known old one is itself a raise: a stale lower `matched` never turns a drop into a raise (his 35 road, then a 50
    road at his own 57, then 50 -> 40 is a drop from 50). The same limit back after a gap is no change.
  * Live (round 3): the proposal says whether its limit is being read NOW on the street the car is on (a valid
    reading this frame equal to the steady limit, clean trust, a matched street, not a link, at most LIVE_MAX) and has
    been read on this road chain for LIVE_STEADY in all (gaps of any length hold it; a new chain starts over). The
    planner lets such a limit raise our street cap / exit handover anchor even with the same seq (the 55 of the
    trunk road he stopped on, after a cap at the RES); a highway's limit kept through a gap is never live.
  * Only on roads: nothing on a link (the merge / exit own it) nor while the merge or the exit plan are busy.
  * Match on engage (the old fork's rule 8): only on a STREET (the last known class a street, a live view with trust
    0, not a link): he engages (SET / RES) with the set inside the band of an already-known steady limit -> the
    limit + offset is proposed at once, whether the limit became steady before or after the press; outside the band
    his speed stays his. On a highway, a link or an unknown road nothing is snapped: his SET speed stays (engaged: a
    later limit CHANGE in the old limit's band still moves it), and after his RES (or our auto-resume's) a speed his
    SET+/SET- had made his stays HIS (`keep_his` on this frame's contribution: the planner keeps the owner), so only
    a limit change after the RES re-takes it by the limit change rule. (No GPS position at all: the RES is an
    engagement as before, so the 43 mph fallback can still apply.)
  * A raise waits while the map's preview limit ahead is lower (mapdOut previewSpeedLimit / previewDistance, read
    directly: published whenever 7.0's map provider runs, checked as map_observation does), but only a preview
    seen on the current road (a road change, the matched chain, drops it); a preview never holds a lowering.
  * Our map as a FALLBACK source (format 6 curve files carry each way's TAGGED maxspeed; never a class default):
    7.0's mapd says "unknown" for whole seconds where ways compete (ErrAmbiguousWay on divided roads: on Hall Rd
    slcState's map row was valid on ~5-18 % of the samples). The map view then gives the matched way's tagged limit
    for the direction of travel ("lim", only with trust 0 and a match; a link only with its own tag). MapLimit
    (once per frame, ctx["limit_in"]) decides the reading every limit rule here uses:
      1. 7.0's valid reading (slcState map / dash row) always wins.
      2. Missing / unknown: our map's limit counts as a reading of the same kind: the same steady times, clean trust,
         view-alive rules, GAP_HOLD, "live", seq / re-take and match on engage.
      3. 7.0 valid with a DIFFERENT value than our map: 7.0 wins, and our map is not used on that road chain for
         DISAGREE_HOLD after it (no flipping between the two values in 7.0's gaps); logged once per road chain.
      4. 7.0 only "possible" (mapdOut, the first fix of a new match) with a different value than our map: neither
         is used on that frame (a gap: the steady time holds up to GAP_HOLD). The same value: one reading.
    The map's preview (a lower limit ahead) stays 7.0's only. Event log: "map_limit" lines (start / stop with the
    value and the road class; "disagree" with both values). Never a position.
  * F5 fallback: no GPS position (the map view's "none": ctx no_gps) and the limit unknown, the car at
    FALLBACK_FROM or more for FALLBACK_AFTER without a break (any frame with the view alive, a known limit, below
    FALLBACK_FROM restarts it), within FALLBACK_WINDOW of engaging: FALLBACK (43 mph) as a "fallback" proposal (up
    only). With GPS an untagged street keeps his speed.
Pure (tests/test_limits.py).
"""
import math

from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed.planner import Contribution
from openpilot.starpilot.car.mazda.speed.view_age import GONE, LATE, ViewClock

MPH, KPH = M.MPH, M.KPH
LIMIT_SOURCES = ("map", "dashboard")
LIMIT_MIN = (15, 20)                 # mph / km/h
LIMIT_MAX = (85, 140)
MULTIPLE = (5, 10)
WHOLE_TOL = 0.15                     # steps
LOWER_STEADY = 3.0                   # s
RAISE_STEADY = 5.0                   # s
LIMIT_KEEP = 30.0                    # s without readings before the steady limit is forgotten
LIMIT_BAND = (5, 10)                 # one limit multiple above the limit (drive 52: his 50 on a 45 road)
TARGET_MAX = (85, 140)               # dash units
TRUST_OK = 0
FALLBACK = (43, 70)                  # dash units
FALLBACK_FROM = 35 * MPH             # m/s true
FALLBACK_AFTER = 3.0                 # s
FALLBACK_WINDOW = 120.0              # s after engaging
SLC_STALE = 0.5                      # s
MATCH_KEEP = 90.0                    # s: the old limit (and the one his speed was at) for a change after a gap
RETAKE_WINDOW = 30.0                 # s after a limit became steady it may re-take his speed (the old offer time)
RETAKE_LOWER = 5.0                   # s unchanged with clean trust before a lower limit re-takes his speed (old rule 9)
RETAKE_LOWER_BIG = 8.0               # s ... for a drop of more than BIG_DROP (old LOWER_BIG_STEADY)
BIG_DROP = (20, 32)                  # mph / km/h
HIGHWAY_CLASSES = (1,)               # for the limit rules only a motorway is a highway. A trunk road (class 3: Hall Rd,
                                     # a divided surface road with lights) is a STREET here: the offset, the match on
                                     # engage and the lower-limit re-take apply. Exits and merges keep map_view's (1, 3).
LINK_CLASSES = (2, 4, 6, 8, 10)      # map_match.LINK_CLASSES
GAP_HOLD = 2.5                       # s without a reading the steady time holds (two missed 1 Hz fixes; view_age)
LIVE_STEADY = 3.0                    # s the steady limit has been read on this road chain (any gaps): "live" (planner)
LIVE_MAX = (55, 90)                  # dash units: a limit above this is no street's: never "live"


def map_preview(road, valid=True, warn=None):
  """{"speed": m/s, "dist": m} of the map's next limit ahead from a mapdOut reader (v2, as
  speed_limits.map_observation checks it: finite >= 1 m/s, 0 < distance <= 500 m, a way, not the current limit),
  or None; never raises."""
  try:
    from openpilot.starpilot.speed_limits.map_observation import repair_legacy_map_speed
    if not valid or int(road.sampleVersion) != 2:
      return None
    speed = repair_legacy_map_speed(float(road.previewSpeedLimit))
    dist, way, cur = float(road.previewDistance), int(road.previewWayId), float(road.speedLimit)
  except (AttributeError, ImportError) as e:
    _miss(warn, "mapdOut.preview", e)
    return None
  except (TypeError, ValueError):
    return None
  if not (math.isfinite(speed) and speed >= 1.0 and math.isfinite(dist) and 0 < dist <= 500 and way > 0 and
          abs(speed - cur) > 0.02):
    return None
  return {"speed": speed, "dist": dist}


def map_possible(road, valid=True, warn=None):
  """mapdOut's own limit (m/s) on a "possible" fix (the first fix of a new match: slcState is not "valid" then), or
  None; never raises. Only a v2 sample with roadStatus matchedLimit and waySelectionType possible."""
  try:
    from openpilot.starpilot.speed_limits.map_observation import repair_legacy_map_speed
    if (not valid or int(road.sampleVersion) != 2 or str(road.roadStatus) != "matchedLimit" or
        str(road.waySelectionType) != "possible"):
      return None
    speed = repair_legacy_map_speed(float(road.speedLimit))
  except (AttributeError, ImportError) as e:
    _miss(warn, "mapdOut.roadStatus/waySelectionType", e)
    return None
  except (TypeError, ValueError):
    return None
  return speed if math.isfinite(speed) and speed >= 1.0 else None


def reading(state, warn=None):
  """(limit m/s or None, upcoming m/s or 0.0) from an slcState reader; never raises. (The upcoming value is
  always 0 on this car's display-only SLC; it is not used.)"""
  try:
    limit = None
    if str(state.observationKind) == "valid" and str(state.source) in LIMIT_SOURCES:
      limit = float(state.speedLimit)
    else:
      rows = {str(r.source): r for r in state.sourceReadings}
      for src in LIMIT_SOURCES:
        r = rows.get(src)
        if r is not None and str(r.observationKind) == "valid":
          limit = float(r.speedLimit)
          break
    if limit is not None and not (math.isfinite(limit) and limit > 0):
      limit = None
    up = float(state.upcomingSpeedLimit)
    return limit, (up if math.isfinite(up) and up > 0 else 0.0)
  except AttributeError as e:
    _miss(warn, "slcState", e)
    return None, 0.0
  except (TypeError, ValueError):
    return None, 0.0


def _miss(warn, what, err):
  from openpilot.starpilot.car.mazda.speed.events import schema_miss
  schema_miss(warn, what, err)


DISAGREE_HOLD = 15.0                 # s our map is not used on a road chain after 7.0 read another value there
DIFFERENT = 0.3                      # m/s: two readings this far apart are different limits (< 1 mph / 2 km/h)
STOP_AFTER = 10.0                    # s without our map supplying the limit: the "stop" line


def view_limit(view):
  """m/s of our map's tagged limit in the view (map_view "lim": trust 0, matched), or None. Not the view's age."""
  if not view or view.get("gps") != "ok" or view.get("trust") != TRUST_OK or not view.get("matched"):
    return None
  lim = view.get("lim")
  if not isinstance(lim, (int, float)) or not math.isfinite(lim) or lim <= 0:
    return None
  return float(lim)


def limit_in(ctx):
  """{"limit": m/s or None, "src": "7.0" | "map" | None} for this frame: MapLimit's (ctx["limit_in"], set by
  contributors.Sources) or, without it (tests, replays), the same rules without the disagreement memory."""
  got = ctx.get("limit_in")
  if got is not None:
    return got
  return MapLimit().decide(ctx, log=False)


class MapLimit:
  """Which reading the limit rules use this frame (module doc: our map as a fallback). Holds the disagreement memory
  and writes the "map_limit" event-log lines through `log(kind, **fields)`."""

  def __init__(self, log=None):
    self.log = log
    self.view_clock = ViewClock()
    self.disagree = None           # (chain, our m/s, until s): 7.0 read another value on this chain
    self.logged = set()            # chains whose disagreement was logged (once per road chain per drive)
    self.supplying = None          # (our m/s, road class) while our map supplies the limit
    self.used_t = None             # s our map last supplied it

  def _log(self, kind, **kw):
    if self.log is not None:
      try:
        self.log(kind, **kw)
      except Exception:
        pass

  def decide(self, ctx, log=True):
    now = ctx.get("now", 0.0)
    view = ctx.get("view")
    alive = self.view_clock.update(view, now) != GONE
    slc = ctx.get("slc")
    theirs = slc.get("limit") if slc else None
    ours = view_limit(view) if alive else None
    chain = view.get("chain") if alive else None
    cls = int(view.get("cls", 0) or 0) if alive else 0
    out = {"limit": None, "src": None, "ours": ours}
    if theirs is not None:
      out.update(limit=theirs, src="7.0")
      if ours is not None and abs(theirs - ours) > DIFFERENT:
        self.disagree = (chain, ours, now + DISAGREE_HOLD)
        if log and chain not in self.logged:
          self.logged.add(chain)
          self._log("map_limit", event="disagree", theirs_mph=round(theirs / MPH, 1), ours_mph=round(ours / MPH, 1),
                    unit=view.get("limu", ""), cls=cls)
      elif ours is not None:
        self.disagree = None       # they agree here
    else:
      use = ours
      d = self.disagree
      if use is not None and d is not None and d[0] == chain and now <= d[2] and abs(d[1] - use) <= DIFFERENT:
        use = None                 # 7.0 read another value on this road just now: it wins
      poss = ctx.get("slc_possible")
      if use is not None and poss is not None and abs(float(poss["limit"]) - use) > DIFFERENT:
        use = None                 # 7.0's "possible" fix says another value: no reading this frame (a gap)
      if use is not None:
        out.update(limit=use, src="map")
    if log:
      if out["src"] == "map":
        self.used_t = now
        key = (round(out["limit"], 2), cls)
        if self.supplying is None or self.supplying[0] != key[0]:
          self._log("map_limit", event="start", mph=round(out["limit"] / MPH, 1), unit=view.get("limu", ""), cls=cls)
        self.supplying = key
      elif self.supplying is not None and (self.used_t is None or now - self.used_t > STOP_AFTER):
        self._log("map_limit", event="stop", mph=round(self.supplying[0] / MPH, 1), cls=self.supplying[1],
                  why="7.0" if theirs is not None else ("no view" if not alive else "no tag"))
        self.supplying = None
    return out


def whole_limit(mps, metric=False):
  """The limit in dash units (int) when it is a whole multiple within bounds, else None."""
  if mps is None:
    return None
  u = mps / (KPH if metric else MPH)
  k = 1 if metric else 0
  n = round(u / MULTIPLE[k]) * MULTIPLE[k]
  if abs(u - n) > WHOLE_TOL or not LIMIT_MIN[k] <= n <= LIMIT_MAX[k]:
    return None
  return int(n)


class SpeedLimit:
  def __init__(self, cfg, presses=True, metric=False):
    self.cfg = cfg
    self.presses = presses
    self.metric = bool(metric)
    self.k = 1 if self.metric else 0
    self.unit = KPH if self.metric else MPH
    self.offset = int(cfg.limit_offset_mph)
    self.steady = None             # dash units
    self.prev = None               # the steady limit before (its band)
    self.cand, self.cand_t = None, 0.0
    self.last_seen = -1e9
    self.change_band = None        # (lo, hi) dash units of the last change, for an engaged anchor
    self.preview = None            # (dash units, chain it was seen on)
    self.t = None
    self.engaged_t = None
    self.fast_t = 0.0
    self.last = None               # for the log
    self.view_clock = ViewClock()
    self.seq = 0                   # grows each time the steady limit changes (planner: the limit change rule)
    self.steady_t = None           # when the steady limit last changed
    self.known = None              # (dash units, s): the last steady limit and when it was last reported
    self.matched = None            # (dash units, s): the last steady limit his anchor was in the band of, and when
    self.gap_t = 0.0               # s without a reading (the steady time holds up to GAP_HOLD)
    self.road_cls = 0              # the last known road class of a live map view (0: never known)
    self.road_link = False         # ... and whether the view says the car is on a link
    self.live_key = None           # (steady limit, chain) the live time is for
    self.live_t = 0.0              # s that limit has been read with clean trust on that street chain (gaps hold it)
    self.src = None                # this frame's reading: "7.0", "map" (our map's fallback) or None

  def street(self):
    """On a street (the offset applies): the last known road class known, not a highway, not a link."""
    return bool(self.road_cls and self.road_cls not in HIGHWAY_CLASSES + LINK_CLASSES and not self.road_link)

  def band(self, limit, street=None):
    street = self.street() if street is None else street
    return (limit * self.unit, (limit + max(LIMIT_BAND[self.k], self.offset if street else 0)) * self.unit)

  def target(self, limit, street=None):
    street = self.street() if street is None else street
    return min(limit + (self.offset if street else 0), TARGET_MAX[self.k]) * self.unit

  def _retake(self, now, tgt, anchor, street):
    """The limit change rule (module doc): may this proposal re-take his own speed now?"""
    if self.steady_t is None or now - self.steady_t > RETAKE_WINDOW + 1e-6:
      return False
    if anchor is None or tgt >= anchor - 0.5 * self.unit:
      return True                  # a raise (the planner checks the band of the limit it replaces)
    if self.prev is None or self.steady >= self.prev:
      return False                 # under his speed only for a limit that went DOWN (his 62 on a new 55 road stays)
    big = self.prev - self.steady > BIG_DROP[self.k]
    need = RETAKE_LOWER_BIG if big else RETAKE_LOWER
    return bool(street and self.cand == self.steady and self.cand_t >= need - 1e-6)

  def __call__(self, ctx):
    now, v = ctx.get("now", 0.0), ctx.get("v_ego", 0.0)
    dt = 0.0 if self.t is None else min(max(now - self.t, 0.0), 0.5)
    self.t = now
    view = ctx.get("view")
    live = self.view_clock.update(view, now)
    alive = live != GONE
    clean = bool(alive and view.get("trust") == TRUST_OK)
    chain = view.get("chain", 0) if alive else None
    if alive:
      cls = int(view.get("cls", 0) or 0)
      if cls:
        self.road_cls = cls        # (class 0 frames skipped, as the old fork)
      self.road_link = bool(view.get("link"))
    # match on engage only on a street with a live view and clean trust (the old fork's rule 8)
    engage_street = bool(clean and self.street())
    keep_his = False
    for e in ctx.get("act_events", ()):
      if e.get("kind") in ("engaged", "resumed"):
        # match on engage: the band of the steady limit already known (none known: the first one's band, below)
        self.engaged_t, self.fast_t = now, 0.0
        self.change_band = self.band(self.steady) if self.steady is not None and engage_street else None
        if e.get("kind") == "resumed" and alive and not engage_street:
          keep_his = True          # his RES (or ours) off a street: a speed of his stays his (planner)
    if not ctx.get("cruise_enabled", False):
      self.engaged_t = None
    lin = limit_in(ctx)            # 7.0's valid reading, else our map's tagged limit (MapLimit)
    limit_mps = lin["limit"]
    self.src = lin["src"]
    pv = ctx.get("preview")
    up_mps = pv["speed"] if pv else 0.0
    raw = whole_limit(limit_mps, self.metric)
    if raw is None and self.cand is not None:
      poss = ctx.get("slc_possible")
      if poss is not None and whole_limit(poss["limit"], self.metric) == self.cand:
        raw = self.cand            # mapd's "possible" fix with the candidate's value: the same reading
    if raw is not None:
      self.last_seen, self.gap_t = now, 0.0
      if raw != self.cand:
        self.cand, self.cand_t = raw, 0.0
      elif not (clean and live == LATE):
        self.cand_t = self.cand_t + dt if clean else 0.0   # restarts at the last bad-trust frame (holds while LATE)
    else:
      self.gap_t += dt
      if self.gap_t > GAP_HOLD + 1e-6:
        self.cand, self.cand_t = None, 0.0                 # a longer gap: the candidate is gone
      elif not clean:
        self.cand_t = 0.0                                  # holds through the gap, but trust / view rules still apply
    if self.cand is not None and self.cand != self.steady:
      need = RAISE_STEADY
      if self.steady is not None and self.cand < self.steady:
        need = LOWER_STEADY
        if not self.street():      # off a street (a motorway, a link, unknown): the re-take times, also for a speed of
          # ours (a wrong reading under an overpass or beside a ramp must not drop 70 to 55 after 3 s)
          need = RETAKE_LOWER_BIG if self.steady - self.cand > BIG_DROP[self.k] else RETAKE_LOWER
      if self.cand_t >= need - 1e-6:
        new, old = self.cand, self.steady
        if old is None and self.known is not None and now - self.known[1] <= MATCH_KEEP:
          old = self.known[0]      # forgotten in a gap of readings (LIMIT_KEEP), still the road's old limit
        m = self.matched
        if (m is not None and now - m[1] <= MATCH_KEEP and m[0] != old and m[0] < new and
            (old is None or old < new)):
          old = m[0]               # the limit his speed was at, before a short detour (the old fork's "matched")
        self.prev, self.steady = old, new
        # a first limit: its own band, a match on engage only on a street (rule 8); a change: the old limit's band
        self.change_band = self.band(old) if old is not None else (self.band(new) if self.street() else None)
        if old is None or new != old:
          self.seq, self.steady_t = self.seq + 1, now    # a real change of the road's limit (not one back after a gap)
    if self.steady is not None and raw == self.steady:
      self.known = (self.steady, now)
      a = ctx.get("anchor")
      lo, hi = self.band(self.steady)
      if a is not None and lo - 0.5 * self.unit <= a <= hi + 0.5 * self.unit:
        self.matched = (self.steady, now)
    if raw is None and now - self.last_seen > LIMIT_KEEP:
      self.steady = self.prev = self.steady_t = None
    # "live": the steady limit is being read NOW on the street the car is on (a valid reading this frame, clean trust,
    # the matched road a street, not a link), and has been read there for LIVE_STEADY in all (any gaps: Hall Rd's
    # mapd is valid on ~1 fix in 5; a chain change starts over: a highway's limit still read at the exit is not it)
    on_street = bool(clean and view.get("matched", True) and not view.get("link") and
                     int(view.get("cls", 0) or 0) not in (0,) + HIGHWAY_CLASSES + LINK_CLASSES)
    key = (self.steady, chain if alive else (self.live_key[1] if self.live_key else None))
    if self.live_key != key:
      self.live_key, self.live_t = key, 0.0
    reading_now = bool(on_street and self.steady is not None and raw == self.steady)
    if reading_now and live != LATE:
      self.live_t += dt
    live_now = bool(reading_now and self.steady <= LIVE_MAX[self.k] and self.live_t >= LIVE_STEADY - 1e-6)
    up = whole_limit(up_mps, self.metric) if up_mps else None
    if up is not None and (self.preview is None or self.preview[0] != up):
      self.preview = (up, chain)
    elif up is None:
      self.preview = None
    if self.preview is not None and chain is not None and self.preview[1] != chain:
      self.preview = None          # seen on the road before: stale, never holds anything
    busy = bool(ctx.get("merge_busy") or ctx.get("exit_busy") or (alive and view.get("link")))
    # F5: the fallback's time at speed needs every condition on every frame (no_gps from contributors: view not alive)
    fallback = bool(self.steady is None and raw is None and ctx.get("no_gps") and self.engaged_t is not None and
                    now - self.engaged_t <= FALLBACK_WINDOW and not busy)
    self.fast_t = self.fast_t + dt if (fallback and v >= FALLBACK_FROM) else 0.0
    out = Contribution("speed_limit", None, presses=self.presses, kind="limit", seq=self.seq, keep_his=keep_his,
                       live=live_now)
    if self.steady is not None:
      if busy:
        self.last = ("busy", self.steady)
        return out
      tgt = self.target(self.steady)
      anchor = ctx.get("anchor")
      if (anchor is not None and tgt > anchor + 0.5 * self.unit and self.preview is not None and
          self.preview[0] < self.steady):
        self.last = ("preview lower", self.steady)
        return out                 # a raise waits for the lower limit just ahead
      out.anchor, out.band = tgt, self.change_band
      out.retake = self._retake(now, tgt, anchor, alive and not view.get("link") and
                                int(view.get("cls", 0) or 0) not in (0,) + HIGHWAY_CLASSES)
      self.last = ("limit", self.steady)
      return out
    # F5: the limit unknown
    if fallback and self.fast_t >= FALLBACK_AFTER - 1e-6:
      out.anchor, out.kind = FALLBACK[self.k] * self.unit, "fallback"
      self.last = ("fallback", None)
      return out
    self.last = ("unknown", None)
    return out
