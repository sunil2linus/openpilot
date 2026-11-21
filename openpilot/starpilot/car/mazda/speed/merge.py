"""
The on-ramp merge contributor (part f, F3): on a link to a highway, once nothing ahead asks for a lower speed, the
set speed goes up toward the highway's limit + limit_offset_mph with a boost. It only PROPOSES anchors ("merge",
planner.py takes them by its rules: up from our anchor or a street's, down only from our merge anchor); it never
presses and gives no slowing target.

Ported from the old fork (speed_limit_follow rule 5, curve_slowdown MERGE_*), without its patch layers:
  * Where: the map view matched a link whose ramp record leads to a highway, with GPS trust 0, or the start of the
    drive's trust on that link with the GPS error at most START_ACC (the view's start_ok); kept HWY_KEEP after the
    car is on the highway; a street matched for STREET_END ends it.
  * The highway's limit H (whole multiple, at least H_MIN): 7.0's upcoming (preview) limit, else the current reading
    when it already is a highway's, else the ramp's destination speed from the map (the next limit is often missing
    or still the street's at the link's start: it keeps waiting on the link, never gives up on a street limit). H must
    stay the same for H_STEADY with that trust.
  * Target T = H + offset (dash). Boost: T + BOOST (at most the car's max, 90 mph); taper to T + TAPER_OVER once the
    car is within TAPER_GAP of T; then T once within END_GAP (the merge is done: the anchor stays ours at T, so the
    limit follow carries on from there). At most BOOST_MAX in the boost.
  * Waits (no raise; a running boost drops to T) while: a mapped / camera curve ahead plans below the car's speed +
    CURVE_NEAR (the ramp curve is not behind the car yet: the curve contributor says when its target rises; with map
    curves Off the merge gets a log-only curve plan for this, contributors.build), the
    model's path is short (a stop, a queue: shorter than v x SHORT_T), a radar lead that is slow, closing or of
    unknown speed, the stock LKAS blocked.
  * His SET- on the link while a curve holds the merge is a curve speed (it opens again after the curve, from his
    speed); his SET- otherwise declines the merge for this link. His SET+ / RES / SET: decided again from his speed
    (the planner raises only a street's speed, not his own highway speed).
Pure (tests/test_merge.py).
"""
from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed.limits import whole_limit
from openpilot.starpilot.car.mazda.speed.planner import Contribution

MPH, KPH = M.MPH, M.KPH
TRUST_OK = 0
H_MIN = (50, 80)                 # mph / km/h: a highway's limit
H_STEADY = 2.0                   # s
BOOST = (20, 32)                 # dash units over the target
CAR_MAX = (90, 145)
TAPER_GAP = (10, 16)             # the car within this of the target: down to target + TAPER_OVER
TAPER_OVER = (5, 8)
END_GAP = (2, 3)
BOOST_MAX = 30.0                 # s
HWY_KEEP = 20.0                  # s on the highway after the link
STREET_END = 1.0                 # s
CURVE_NEAR = 5 * MPH             # m/s
SHORT_T = 4.0                    # s
SHORT_MIN = 30.0                 # m
LEAD_MIN_SPEED = 13.0            # m/s
LEAD_CLOSING = 3.0               # m/s
LEAD_STALE = 0.5                 # s
TARGET_MAX = (85, 140)


class Merge:
  def __init__(self, cfg, presses=True, metric=False):
    self.cfg = cfg
    self.presses = presses
    self.metric = bool(metric)
    self.k = 1 if self.metric else 0
    self.unit = KPH if self.metric else MPH
    self.offset = int(cfg.limit_offset_mph)
    self.link = None               # the ramp (chain id) of this episode
    self.reset_episode()
    self.t = None
    self.last = None

  def reset_episode(self):
    self.declined = self.done = False
    self.h, self.h_t = None, 0.0
    self.boost_t = 0.0
    self.boosting = False
    self.hwy_t = None              # s since the car left the link onto the highway
    self.street_t = 0.0

  def guards(self, ctx):
    v = ctx.get("v_ego", 0.0)
    why = []
    for name in ("map_curve", "camera_curve", "exit"):
      c = (ctx.get("contribs") or {}).get(name)
      if c is not None and c.target is not None and c.target < v + CURVE_NEAR:
        why.append("curve")
        break
    path = ctx.get("path")
    if path is None or ctx.get("model_age", 1e9) > 0.5 or path[0][-1] < max(v * SHORT_T, SHORT_MIN):
      why.append("short path")
    if ctx.get("radar_lead"):
      lead = ctx.get("lead")
      if (lead is None or ctx.get("now", 0.0) - lead.get("t", -1e9) > LEAD_STALE or not lead.get("status") or
          lead["v"] < LEAD_MIN_SPEED or lead["vrel"] <= -LEAD_CLOSING):
        why.append("lead")
    if ctx.get("lkas_blocked"):
      why.append("lkas")
    return why

  def highway_limit(self, ctx, view):
    slc = ctx.get("slc")
    for mps in ((slc["upcoming"], slc["limit"]) if slc else ()):
      h = whole_limit(mps, self.metric) if mps else None
      if h is not None and h >= H_MIN[self.k]:
        return h, "slc"
    ramp = view.get("ramp") or {}
    h = whole_limit(ramp.get("spd"), self.metric)
    if h is not None and h >= H_MIN[self.k]:
      return h, "map"
    return None, ""

  def __call__(self, ctx):
    now, v = ctx.get("now", 0.0), ctx.get("v_ego", 0.0)
    dt = 0.0 if self.t is None else min(max(now - self.t, 0.0), 0.5)
    self.t = now
    out = Contribution("merge", None, presses=self.presses, kind="merge")
    view = ctx.get("view")
    fresh = view is not None and view.get("gps") == "ok" and now - view.get("t", -1e9) <= 1.0
    ramp = (view.get("ramp") or {}) if fresh else {}
    on_link = bool(fresh and view.get("link") and ramp.get("hwy"))
    if on_link and ramp.get("id") != self.link:
      self.link = ramp.get("id")
      self.reset_episode()
    if self.link is None:
      self.last = ("idle", None)
      return out
    if not on_link:
      if fresh and view.get("cls") in (1, 3) and not view.get("link"):
        self.hwy_t = (self.hwy_t or 0.0) + dt
        self.street_t = 0.0
      elif fresh and view.get("matched") and not view.get("link"):
        self.street_t += dt
      if (self.hwy_t is not None and self.hwy_t > HWY_KEEP) or self.street_t >= STREET_END or not fresh:
        end = self.boosting and not self.done
        self.link = None
        self.last = ("ended", None)
        if end and self.h is not None:
          out.anchor = min(self.target(self.h), max(v / self.unit + M.DASH_OFFSET / self.unit, 0) * self.unit) \
            if self.street_t >= STREET_END else self.target(self.h)
        return out
    ctx["merge_busy"] = True
    trust = bool(fresh and (view.get("trust") == TRUST_OK or view.get("start_ok")))
    guards = self.guards(ctx)
    for e in ctx.get("act_events", ()):
      if e.get("kind") == "driver" and e.get("direction") == "SET_MINUS" and on_link:
        if "curve" not in guards:
          self.declined = True     # his own lower speed on the link, no curve: his decision for this link
    if self.declined or self.done:
      self.last = ("declined" if self.declined else "done", self.h)
      return out
    h, src = self.highway_limit(ctx, view) if fresh else (None, "")
    if h != self.h:
      if self.h is not None and self.boosting:
        out.anchor = self.target(self.h)       # the target moved: start from the old one
      self.h, self.h_t = h, 0.0
      self.boosting = False
    elif h is not None:
      self.h_t = self.h_t + dt if trust else 0.0
    if self.h is None or self.h_t < H_STEADY - 1e-6:
      self.last = ("wait limit", self.h)
      return out
    t_dash = self.target(self.h)
    car_dash = v + M.DASH_OFFSET
    if guards:
      self.last = ("wait " + ",".join(guards), self.h)
      if self.boosting:
        out.anchor = t_dash
        self.boosting = False
      return out
    if car_dash >= t_dash - END_GAP[self.k] * self.unit or self.boost_t >= BOOST_MAX:
      out.anchor, self.done, self.boosting = t_dash, True, False
      self.last = ("end", self.h)
      return out
    self.boost_t += dt
    self.boosting = True
    if car_dash >= t_dash - TAPER_GAP[self.k] * self.unit:
      out.anchor = min(t_dash + TAPER_OVER[self.k] * self.unit, CAR_MAX[self.k] * self.unit)
      self.last = ("taper", self.h)
    else:
      out.anchor = min(t_dash + BOOST[self.k] * self.unit, CAR_MAX[self.k] * self.unit)
      self.last = ("boost " + src, self.h)
    return out

  def target(self, h):
    return min(h + self.offset, TARGET_MAX[self.k]) * self.unit
