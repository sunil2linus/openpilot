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
HIGHWAY_CLASSES = (1, 3)             # map_view.HIGHWAY_CLASSES
LINK_CLASSES = (2, 4, 6, 8, 10)      # map_match.LINK_CLASSES
GAP_HOLD = 2.5                       # s without a reading the steady time holds (two missed 1 Hz fixes; view_age)


def map_preview(road, valid=True):
  """{"speed": m/s, "dist": m} of the map's next limit ahead from a mapdOut reader (v2, as
  speed_limits.map_observation checks it: finite >= 1 m/s, 0 < distance <= 500 m, a way, not the current limit),
  or None; never raises."""
  try:
    from openpilot.starpilot.speed_limits.map_observation import repair_legacy_map_speed
    if not valid or int(road.sampleVersion) != 2:
      return None
    speed = repair_legacy_map_speed(float(road.previewSpeedLimit))
    dist, way, cur = float(road.previewDistance), int(road.previewWayId), float(road.speedLimit)
  except (AttributeError, TypeError, ValueError, ImportError):
    return None
  if not (math.isfinite(speed) and speed >= 1.0 and math.isfinite(dist) and 0 < dist <= 500 and way > 0 and
          abs(speed - cur) > 0.02):
    return None
  return {"speed": speed, "dist": dist}


def map_possible(road, valid=True):
  """mapdOut's own limit (m/s) on a "possible" fix (the first fix of a new match: slcState is not "valid" then), or
  None; never raises. Only a v2 sample with roadStatus matchedLimit and waySelectionType possible."""
  try:
    from openpilot.starpilot.speed_limits.map_observation import repair_legacy_map_speed
    if (not valid or int(road.sampleVersion) != 2 or str(road.roadStatus) != "matchedLimit" or
        str(road.waySelectionType) != "possible"):
      return None
    speed = repair_legacy_map_speed(float(road.speedLimit))
  except (AttributeError, TypeError, ValueError, ImportError):
    return None
  return speed if math.isfinite(speed) and speed >= 1.0 else None


def reading(state):
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
  except (AttributeError, TypeError, ValueError):
    return None, 0.0


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
    slc = ctx.get("slc")
    limit_mps = slc["limit"] if slc else None
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
      need = LOWER_STEADY if self.steady is not None and self.cand < self.steady else RAISE_STEADY
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
    out = Contribution("speed_limit", None, presses=self.presses, kind="limit", seq=self.seq, keep_his=keep_his)
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
