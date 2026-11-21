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
    (LOWER_STEADY for a lower one, RAISE_STEADY for a higher or first one); a bad-trust frame restarts it. A gap in
    the readings keeps the last steady limit for LIMIT_KEEP.
  * The proposal: the steady limit + limit_offset_mph (dash, whole steps, no margin), at most TARGET_MAX.
    The planner takes it when the anchor is ours (our limit raise, merge, exit handover: re-capped by a lower limit in
    any band), or when he engaged (SET / RES) at a speed within the band of the limit it replaces (the old limit up to
    + max(LIMIT_BAND, offset); at a first limit: the new limit's band). His SET+ / SET- make the speed his: no more
    proposals take. His own higher speed (80 on a 70 road) is out of the band: never pressed down.
  * Only on roads: nothing on a link (the merge / exit own it) nor while the merge or the exit plan are busy.
  * Match on engage: he engages (SET / RES) with the set inside the band of an already-known steady limit -> the
    limit + offset is proposed at once (the old fork's "match on engage"; owner: limit + 3), whether the limit became
    steady before or after the press; outside the band his speed stays his.
  * A raise waits while the map's preview limit ahead is lower (mapdOut previewSpeedLimit / previewDistance, read
    directly: published whenever 7.0's map provider runs, checked as map_observation does), but only a preview
    seen on the current road (a road change, the matched chain, drops it); a preview never holds a lowering.
  * F5 fallback: no GPS position (the map view's "none": ctx no_gps) and the limit unknown, the car at
    FALLBACK_FROM or more for FALLBACK_AFTER, within FALLBACK_WINDOW of engaging: FALLBACK (43 mph) as a "fallback"
    proposal (up only). With GPS an untagged street keeps his speed.
Pure (tests/test_limits.py).
"""
import math

from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed.planner import Contribution

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

  def band(self, limit):
    return (limit * self.unit, (limit + max(LIMIT_BAND[self.k], self.offset)) * self.unit)

  def target(self, limit):
    return min(limit + self.offset, TARGET_MAX[self.k]) * self.unit

  def __call__(self, ctx):
    now, v = ctx.get("now", 0.0), ctx.get("v_ego", 0.0)
    dt = 0.0 if self.t is None else min(max(now - self.t, 0.0), 0.5)
    self.t = now
    view = ctx.get("view")
    fresh = view is not None and view.get("gps") == "ok" and now - view.get("t", -1e9) <= 1.0
    clean = bool(fresh and view.get("trust") == TRUST_OK)
    chain = view.get("chain", 0) if fresh else None
    for e in ctx.get("act_events", ()):
      if e.get("kind") in ("engaged", "resumed"):
        # match on engage: the band of the steady limit already known (none known: the first one's band, below)
        self.engaged_t, self.fast_t = now, 0.0
        self.change_band = self.band(self.steady) if self.steady is not None else None
    if not ctx.get("cruise_enabled", False):
      self.engaged_t = None
    slc = ctx.get("slc")
    limit_mps = slc["limit"] if slc else None
    pv = ctx.get("preview")
    up_mps = pv["speed"] if pv else 0.0
    raw = whole_limit(limit_mps, self.metric)
    if raw is not None:
      self.last_seen = now
    if raw != self.cand:
      self.cand, self.cand_t = raw, 0.0
    elif raw is not None:
      self.cand_t = self.cand_t + dt if clean else 0.0     # restarts at the last bad-trust frame
    if self.cand is not None and self.cand != self.steady:
      need = LOWER_STEADY if self.steady is not None and self.cand < self.steady else RAISE_STEADY
      if self.cand_t >= need - 1e-6:
        self.prev, self.steady = self.steady, self.cand
        self.change_band = self.band(self.prev if self.prev is not None else self.steady)
    if raw is None and now - self.last_seen > LIMIT_KEEP:
      self.steady = self.prev = None
    up = whole_limit(up_mps, self.metric) if up_mps else None
    if up is not None and (self.preview is None or self.preview[0] != up):
      self.preview = (up, chain)
    elif up is None:
      self.preview = None
    if self.preview is not None and chain is not None and self.preview[1] != chain:
      self.preview = None          # seen on the road before: stale, never holds anything
    busy = bool(ctx.get("merge_busy") or ctx.get("exit_busy") or (fresh and view.get("link")))
    out = Contribution("speed_limit", None, presses=self.presses, kind="limit")
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
      self.last = ("limit", self.steady)
      return out
    # F5: the limit unknown
    if (raw is None and ctx.get("no_gps") and self.engaged_t is not None and now - self.engaged_t <= FALLBACK_WINDOW
        and not busy):
      self.fast_t = self.fast_t + dt if v >= FALLBACK_FROM else 0.0
      if self.fast_t >= FALLBACK_AFTER - 1e-6:
        out.anchor, out.kind = FALLBACK[self.k] * self.unit, "fallback"
        self.last = ("fallback", None)
        return out
    self.last = ("unknown", None)
    return out
