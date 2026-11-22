"""
The curve contributor: the map's curve profiles ahead (from the map view, map_view.py) -> one true-speed target.

  * Each profile point i (distance d_i ahead, radius r_i) allows v_i = sqrt(lat * r_i), lat = plan_lat on roads,
    ramp_lat on ramps/links, ti_only_lat when the stock LKAS was already blocked before the curve (latched per curve;
    LKAS lost inside the curve because the car slowed below its cut-off does NOT re-plan lower). The curve speed
    offset (a later stage) raises v_i, never above sqrt(cap * r_i); an advisory sign caps it at sign + 10 mph.
  * target = min over the points still ahead of cruise_model.envelope(v_i, d_i, v_ego): the same braking envelope
    for every point, so a tight entry governs the approach and the target rises by itself once only wider points
    remain. A point counts until KEEP_PAST m after it (GPS lag ~0.7-1.0 s).
  * Curve memory (curve_memory.py): the offset and a learned cap act on the curve's tight points; each pass through a
    curve (entered: its first point behind the car; over: its last point KEEP_PAST behind) is recorded.
  * Mid-curve the target only steps UP with GPS trust OK; otherwise it holds the lowest it had in that curve.
  * The camera's veto (camera.py) drops a curve the camera sees straight through its tight points.
  * If the view drops out mid-curve, the last good one is followed by the distance driven (DEAD_RECKON): the set speed
    is never raised in a curve just because the GPS went away.
  * The LKAS floor is asked for when the curve's own lowest point speed is at least 30 mph.
Pure (tests/test_curve_plan.py).
"""
import math

from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed.curve_memory import MARK_OVER, PassRecorder, apply_memory, curve_key, offset_on
from openpilot.starpilot.car.mazda.speed.planner import Contribution

MPH = M.MPH
KEEP_PAST = 18.0              # m a point keeps governing after the car passed it (GPS lag)
VIEW_STALE = 1.0              # s: an older map view is no map
DEAD_RECKON = 10.0            # s: when the map view drops out (stale, no GPS, paused, held), the last good view is
DEAD_RECKON_M = 300.0         # m: followed by the distance driven this long / this far, so a curve being slowed for
                              # (or driven) is finished; no new curve can start from it
ADVISORY_OVER = 10 * MPH
TRUST_OK = 0                  # map_match.TRUST_OK


def point_speed(r, lat, offset=0.0, cap_lat=None, advisory=0.0):
  v = math.sqrt(max(lat, 0.1) * max(r, 1.0))
  if offset > 0:
    v = min(v + offset, math.sqrt(cap_lat * r)) if cap_lat else v + offset
    v = max(v, math.sqrt(lat * r))
  if advisory > 0:
    v = min(v, advisory + ADVISORY_OVER)
  return v


class CurvePlan:
  def __init__(self, cfg, presses=True, metric=False, memory=None):
    self.cfg = cfg
    self.presses = presses
    self.memory = memory
    self.offset = cfg.curve_offset_mph * MPH
    self.recording = {}          # curve id -> PassRecorder while the car is in it
    self.ti_only = {}            # curve id -> True when LKAS was blocked before it (latched)
    self.held = {}               # curve id -> lowest target while inside without GPS trust OK
    self.last = None             # (target, curve id, why) for the log
    self.vetoed = set()
    self.good = None             # the last good view (DEAD_RECKON)
    self.last_vmin = None

  def curve_lat(self, c, lkas_blocked):
    lat = self.cfg.ramp_lat if c.get("ramp") else self.cfg.plan_lat
    cid = c["id"]
    if cid not in self.ti_only:
      ahead = min(p[0] for p in c["pts"]) > 0 if c["pts"] else False
      if ahead:
        self.ti_only[cid] = bool(lkas_blocked)
    elif not self.ti_only[cid] and lkas_blocked and min(p[0] for p in c["pts"]) > 0:
      self.ti_only[cid] = True   # still before the curve: blocked now counts
    if self.ti_only.get(cid):
      lat = min(lat, self.cfg.ti_only_lat)
    return lat

  def __call__(self, ctx):
    view = ctx.get("view")
    now, v = ctx.get("now", 0.0), ctx.get("v_ego", 0.0)
    bad = (view is None or view.get("gps") != "ok" or now - view.get("t", -1e9) > VIEW_STALE or
           view.get("held") or view.get("paused"))
    if not bad:
      self.good = view
    else:
      g = self.good
      if g is None or now - g["t"] > DEAD_RECKON or max(v, 0.0) * (now - g["t"]) > DEAD_RECKON_M:
        self.good = None
        self.last = (None, None, "no map")
        return Contribution("map_curve", None, presses=self.presses)
      # only what was already ahead and planned: never a new curve from an old view
      view = dict(g, trust=1, start_ok=False)
    driven = max(v, 0.0) * max(now - view["t"], 0.0)
    trust_ok = view.get("trust") == TRUST_OK or view.get("start_ok", False)
    veto = ctx.get("veto")
    best, best_c, floor, best_vmin = None, None, False, None
    seen = set()
    for c in view.get("curves", []):
      cid = c["id"]
      pts = [(d - driven, r) for d, r in c["pts"]]
      if not pts or max(d for d, _ in pts) < -KEEP_PAST:
        continue
      cc = dict(c, pts=pts)
      if veto is not None and veto.vetoed(cc, ctx):
        continue
      seen.add(cid)
      lat = self.curve_lat(cc, ctx.get("lkas_blocked", False))
      speeds = [point_speed(r, lat, 0.0, None, c.get("adv", 0.0)) for _, r in pts]
      key = curve_key(c["way"], c["place"], c["wdir"]) if c.get("way") and "place" in c else None
      if key is not None and (self.memory is not None or self.offset > 0):
        speeds = apply_memory(pts, speeds, key, cc.get("ramp", False), self.memory, self.offset,
                              self.cfg.curve_offset_lat, lat)
      self._record(cid, key, c, pts, ctx)
      target = None
      for (d, _), vp in zip(pts, speeds, strict=True):
        if d < -KEEP_PAST:
          continue
        e = M.envelope(vp, max(d, 0.0), v)
        target = e if target is None else min(target, e)
      if target is None:
        continue
      inside = min(d for d, _ in pts) < 0
      if inside and not trust_ok:
        held = self.held.get(cid)
        target = target if held is None else min(target, held)
        self.held[cid] = target
      elif not inside:
        self.held.pop(cid, None)
      if best is None or target < best:
        best, best_c, floor, best_vmin = target, cid, min(speeds) >= M.LKAS_FLOOR_FROM, min(speeds)
    for cid in list(self.held):
      if cid not in seen:
        del self.held[cid]
    for cid in list(self.recording):
      if cid not in seen:
        self._finish(cid, now)
    for cid in list(self.ti_only):
      if cid not in seen:
        del self.ti_only[cid]
    self.last = (best, best_c, "curve")
    self.last_vmin = best_vmin   # true m/s of the governing curve's tightest point (the notice's "slowing to")
    return Contribution("map_curve", best, lkas_floor=floor, presses=self.presses, episode=best_c)

  # ---- curve memory passes ----
  def _record(self, cid, key, c, pts, ctx):
    if self.memory is None or key is None:
      return
    first, last = min(d for d, _ in pts), max(d for d, _ in pts)
    rec = self.recording.get(cid)
    if rec is None and first <= 0 < last:
      ramp = bool(c.get("ramp"))
      rec = self.recording[cid] = PassRecorder(key, c.get("name", ""), ramp, ctx.get("plan_reason") == "map_curve",
                                               offset_on=offset_on(key, ramp, self.memory, self.offset))
    if rec is None:
      return
    if last < -KEEP_PAST:
      self._finish(cid, ctx.get("now", 0.0))
      return
    r_min = min(r for _, r in pts)
    apex_d = next(d for d, r in pts if r == r_min)
    rec.sample(ctx, before_apex=apex_d > 0)

  def _finish(self, cid, now):
    rec = self.recording.pop(cid, None)
    if rec is None:
      return
    p = rec.finish(self.memory.clock())
    if p is not None:
      self.memory.add_pass(rec.key, rec.name, rec.ramp, p, mark_lat=self.cfg.curve_offset_lat + MARK_OVER)
