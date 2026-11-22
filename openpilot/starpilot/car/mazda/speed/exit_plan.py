"""
The exit plan contributor (part f, F4): slows for an exit ramp's curves BEFORE the split, bounded when the car stays
on the highway, and hands the speed over to the exit street's limit. It gives a true-speed target ("exit") and an
anchor proposal ("handover"); it never presses.

Why (the old fork): a 35 mph exit curve was entered at 52 mph because the exit was only recognised once the road
match moved onto the ramp (50-150 m past the split). Here the map view (map_view.ExitsAhead) gives the exit link's
start ahead, its side, the ramp's curve profile, the match and the map shapes past the split; this plan decides.

Evidence and decisions, per exit (a fresh plan for each exit: a new link, or the same link whose start moved by
more than SAME_EXIT, starts with nothing; a view that is gone is followed by the distance driven and is not a new exit).
The view (view_age.py): alive up to VIEW_ALIVE after its GPS message (a 1 Hz GPS); the evidence timers below grow only
while it is CURRENT and hold (neither grow nor restart) while it is LATE (a missed fix); every confirmation and release
from the map also needs VIEWS_MIN different views saying it; gone, the plan follows the last view (DEAD_RECKON) with
silent shapes and no match, as without GPS:
  * Before the split: his blinker toward the exit's side within SIGNAL_REACH of the split (latched for this exit);
    the camera's lane: the car in the lane next to the road edge on that side (LaneView, DECIDE_NEAR); the model path
    bending toward the link near the split (the map's laterals MODEL_AT m ahead; MODEL_SEP apart at least).
    No navigation: 7.0's navInstruction (navd, 1 Hz, an online Mapbox route only with NavigationEnabled) and its own
    starpilotNavigation are not a qualified offline exit signal here; neither is read.
  * Reach bound: with his blinker, until the exit is resolved the car is kept on a speed profile from which the
    ramp's profile points are still reachable: each point planned at REACH_DECEL (0.7, REACH_LEAD before it, so that
    tracking it with the cruise's lag leaves the ramp's first tight point reachable at <= 0.9 m/s^2, REACH_CHECK)
    after the cruise's lag. It takes over only once the car is within REACH_START of it (latched): armed too late,
    it does not slow before the split.
  * At the split with his blinker (or the camera's lane AND the model path together): the full ramp plan (the curve
    plan's braking envelope, PLAN_DECEL; with the car well above it that is the cruise's strongest) for at most
    FIRM_MAX without a "link" confirmation; then no more slowing than the reach bound (or, without his blinker, the
    speed then is held) until it is resolved. The camera's lane or the model path alone only hold the speed at the
    split (the edge lane is also the ordinary right lane: no slowdown at every exit passed in it).
  * Confirmed ("link"): the map shapes say link for LINK_CONFIRM, or the match on the link with GPS trust 0 for
    MATCH_CONFIRM, or the model path says link for MODEL_CONFIRM while the shapes have said link. Then the full plan
    (the curve contributor takes the ramp's curves over once the match is on the link).
  * Released at once (the speed comes back): "highway" from the shapes for HWY_CONFIRM, the match on the highway with
    GPS trust 0 for HWY_MATCH past HWY_MATCH_FROM with the shapes' last answer not link, the model path straight
    (along the highway, MODEL_CONFIRM) with the shapes' last answer not link; his gas held GAS_HELD; any button of his;
    PASSED past the split unresolved. A "link" answer is forgotten after SHAPE_QUIET_T / SHAPE_QUIET_M of silent
    shapes (scattered link frames never add up to a confirmation, one frame never blocks the release).
  * A confirmation is overturned (everything released: the ramp target, the handover, his speed restored) by the
    shapes saying highway for HWY_CONFIRM, the trusted highway match (as above), or his gas held GAS_HELD unless the
    car is matched on the link with GPS trust 0 right now.
  * His blinker to the OTHER side cancels his exit blinker and the reach profile (his blinker going off does not: he
    cancels it once in the exit lane).
  * The split's place is refreshed from every new view of the same exit (the straight distance shortens toward the
    road distance as the car gets close; a moved link start beyond SAME_EXIT is another exit: a fresh plan). The map
    view gives no along-road distance (the matcher has no position along the road): straight, so never late.
  * Trust a blinker that is still on: while his exit-side blinker is on (BLINKER_GRACE after its last on-frame)
    before the split, or it was on for this exit and the camera says the car is in the lane next to the road edge
    on that side right now, the reach profile slows as much as the ramp's first tight point needs: no DIP_MAX bound.
  * Otherwise, until confirmed: the cruise's set (true speed) never more than DIP_MAX under the speed where OUR
    slowing began (the reach profile taking over, else the split), so the car does not go lower either. The bound
    never RAISES the plan: when the blinker goes off with the plan already deeper than the bound, the plan holds
    where it is (no further lowering, no SET+ burst back toward the highway speed inside the exit area) until the
    exit is resolved (confirmed: the ramp plan; released: his speed back by the release). Firm slowing past the
    split at most FIRM_MAX. Before the split only the reach profile slows (his blinker, once the car is on it).
  * Exit handover (an exit onto a street, after the exit was confirmed): with a confident (trust 0) match of the ramp
    or the street, the anchor is proposed down ("handover") to the street's limit + offset (7.0's reading once the
    street is matched, else the ramp's destination speed from the map, at most STREET_MAX), never more than
    HANDOVER_GAP under the car's speed until the street is matched (STREET_STEADY with trust 0). Cancelled for good by
    his gas held, any button of his, the brake. An interchange (to a highway) has no handover (the merge's job).
Pure (tests/test_exit_plan.py).
"""
import math

from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed.curve_plan import point_speed
from openpilot.starpilot.car.mazda.speed.limits import whole_limit
from openpilot.starpilot.car.mazda.speed.planner import Contribution
from openpilot.starpilot.car.mazda.speed.view_age import GONE, LATE, ViewClock, ViewCount

MPH, KPH = M.MPH, M.KPH
TRUST_OK = 0
EXIT_LOOK = 1500.0          # m
SAME_EXIT = 60.0            # m: the same link whose start moved more than this is another exit (a fresh plan)
SIGNAL_REACH = 400.0        # m before the split: his blinker toward the exit side counts from here
DECIDE_NEAR = 300.0         # m: the camera's lane counts from here
MODEL_NEAR = 120.0          # m before the split: the model path's bend counts from here
MODEL_SEP = 4.0             # m: the link and the highway this far apart MODEL_AT ahead (else no model answer)
MODEL_GAP = 1.0             # m closer to one of them
MODEL_CONFIRM = 0.5         # s
REACH_DECEL = 0.7           # m/s^2 the reach profile is planned at (old TENT_REACH_DECEL) ...
REACH_LEAD = 1.0            # s ... to be at the point's speed this long before it (old MAP_LEAD_TIME)
REACH_START = 5 * MPH       # m/s: it only takes over once the car is within this of it (old TENT_REACH_START, 1 mph,
                            # widened so his blinker 300 m out at 70 mph still counts): an exit armed too late for
                            # it (a blinker 150 m out) gets no deep slowdown before the split
REACH_CHECK = 0.9           # m/s^2: the tight point stays reachable at this (tests)
FIRM_MAX = 3.0              # s of the full plan without a "link" confirmation
DIP_MAX = 12 * MPH          # m/s under the speed at the split until confirmed
LINK_CONFIRM = 1.0          # s of the shapes saying link
MATCH_CONFIRM = 1.0         # s matched on the link with GPS trust 0
HWY_CONFIRM = 0.5           # s of the shapes saying highway
HWY_MATCH = 2.0             # s matched on the highway with GPS trust 0 ...
HWY_MATCH_FROM = 90.0       # m ... this far past the split
GAS_HELD = 1.5              # s
SHAPE_QUIET_T = 0.75        # s / ...
SHAPE_QUIET_M = 40.0        # m of the shapes silent after a "link": that "link" is forgotten (old GAS_QUIET_LINK)
AT_GAIN = 0.3               # per new view: the split's place follows the view's fresh distance (same exit)
RESTORE_MAX = 2.0           # s the restore of his speed is proposed after a handover was overturned
BLINKER_GRACE = 1.0         # s his blinker counts as still on after its last on-frame (carState's blinker on this car
                            # is the flashing lamp held 0.4 s: flash gaps, a self-cancel at the lane change)
PASSED = 400.0              # m past the split unresolved: released
HANDOVER_MAX = 3000.0       # m past the split: a handover not done by then is given up
DEAD_RECKON = 5.0           # s a plan is followed by the distance driven without a live view
HANDOVER_GAP = 10 * MPH     # m/s
STREET_STEADY = 2.0         # s
STREET_MAX = (55, 90)       # dash units: a street's limit
STREET_MIN = (25, 40)
TARGET_MIN = 15 * MPH
BRAKE_GAP = 3 * MPH           # m/s: the bound holds the plan this much above what the ramp needs: "brake" notice

# ---- the camera's lane (ported LaneView: in the lane next to the road edge on a side) ----
LANE_LINE_PROB = 0.3
LANE_OWN_PROB = 0.3
EDGE_NEAR = 4.0
EDGE_STD_MAX = 1.0
LANE_ON_TIME = 1.0
LANE_OFF_TIME = 2.0
LANE_DROPOUT = 3.0
LANE_HALF = 1.8
LANE_NEAR_X = 20.0


def model_lanes(model):
  """(probs[4], lines_y[4], edges_y[2], edge_stds[2]): the y (right positive) of each at the car (mean up to
  LANE_NEAR_X), from a modelV2 reader; None when missing."""
  try:
    probs = [float(p) for p in model.laneLineProbs]
    lines = [_near_y(line) for line in model.laneLines]
    edges = [_near_y(e) for e in model.roadEdges]
    stds = [float(s) for s in model.roadEdgeStds]
  except (AttributeError, TypeError, ValueError):
    return None
  if len(probs) < 4 or len(lines) < 4 or len(edges) < 2 or None in lines or None in edges:
    return None
  return probs[:4], lines[:4], edges[:2], (stds[:2] if len(stds) >= 2 else [EDGE_STD_MAX] * 2)


def _near_y(line):
  xs, ys = list(line.x), list(line.y)
  vals = [y for x, y in zip(xs, ys, strict=False) if x <= LANE_NEAR_X]
  if not vals or not all(math.isfinite(v) for v in vals):
    return None
  return sum(vals) / len(vals)


def path_y_at(path_y, x):
  """The model path's y (m, right positive) x m ahead, or None."""
  if not path_y:
    return None
  xs, ys = path_y
  if not xs or xs[-1] < x:
    return None
  for k in range(1, len(xs)):
    if xs[k] >= x:
      f = (x - xs[k - 1]) / (xs[k] - xs[k - 1]) if xs[k] > xs[k - 1] else 0.0
      return ys[k - 1] + (ys[k] - ys[k - 1]) * f
  return None


class LaneView:
  """In the lane next to the left / right road edge (debounced), from the camera's lane lines and road edges."""

  def __init__(self):
    self.edge = {-1: False, 1: False}
    self.on = {-1: 0.0, 1: 0.0}
    self.off = {-1: 0.0, 1: 0.0}
    self.blind = LANE_DROPOUT + 1.0

  def update(self, dt, lanes):
    if lanes is None:
      self.blind += dt
      if self.blind > LANE_DROPOUT:
        for s in (-1, 1):
          self.edge[s] = False
          self.on[s] = self.off[s] = 0.0
      return
    self.blind = 0.0
    probs, lines_y, edges_y, stds = lanes
    for s, own, beyond, e in ((-1, 1, 0, 0), (1, 2, 3, 1)):
      ref = lines_y[own] if probs[own] > LANE_OWN_PROB else s * LANE_HALF
      raw = probs[beyond] < LANE_LINE_PROB and abs(edges_y[e] - ref) < EDGE_NEAR and stds[e] < EDGE_STD_MAX
      if raw:
        self.on[s], self.off[s] = self.on[s] + dt, 0.0
      else:
        self.on[s], self.off[s] = 0.0, self.off[s] + dt
      if raw and self.on[s] >= LANE_ON_TIME - 1e-6:
        self.edge[s] = True
      elif not raw and self.off[s] >= LANE_OFF_TIME - 1e-6:
        self.edge[s] = False


class Plan:
  """One exit, by where its split is on the contributor's odometer."""

  def __init__(self, eid, at, side):
    self.id, self.at, self.side = eid, at, side
    self.signal = self.lane = self.model = False   # evidence (latched for this exit)
    self.v_split = None
    self.split_t = None
    self.firm_t = 0.0
    self.hold = None
    self.confirmed = False
    self.released = ""
    self.link_t = self.match_t = self.model_link_t = 0.0
    self.hwy_t = self.hwy_match_t = self.model_hwy_t = 0.0
    self.shape_link_seen = False
    self.last_shape = 0
    self.gas_t = 0.0
    self.reach_on = False       # the car got onto the reach profile (REACH_START)
    self.v_ref = None           # m/s where our slowing began (DIP_MAX)
    self.quiet_t = self.quiet_m = 0.0
    self.view_t = None          # the last view the split's place was refreshed from
    self.before = None          # his anchor before the handover lowered it (a restore when overturned)
    self.restore = None         # (anchor, until)
    self.link_n, self.match_n = ViewCount(), ViewCount()           # views saying link / matched on the link
    self.hwy_n, self.hwy_match_n = ViewCount(), ViewCount()        # views saying highway / matched on it past HWY_MATCH_FROM
    self.blink_t = None         # s of his last exit-side blinker frame before the split (BLINKER_GRACE)
    self.last_target = None     # the plan's target of the frame before (the bound never raises it)
    self.pts = []               # [(odometer m, radius)] of the ramp's profile
    self.hwy = False
    self.spd = 0.0
    # handover
    self.handover = False
    self.cancelled = False
    self.street_t = 0.0
    self.handed = False


class ExitPlan:
  def __init__(self, cfg, presses=True, metric=False):
    self.cfg = cfg
    self.presses = presses
    self.metric = bool(metric)
    self.k = 1 if self.metric else 0
    self.unit = KPH if self.metric else MPH
    self.offset = int(cfg.limit_offset_mph)
    self.odo = 0.0
    self.t = None
    self.plan = None
    self.done = {}              # exit id -> split odometer of exits already resolved (not planned again)
    self.lanes = LaneView()
    self.good = None            # (view, odometer at its time)
    self.dt = 0.0
    self.ended = None           # the last plan that ended (logs, tests)
    self.last = None
    self.brake_needed = False
    self.view_clock = ViewClock()
    self.live = GONE            # this frame's view state (view_age.py)

  # ---- helpers ----
  def lat(self, ctx):
    lat = self.cfg.ramp_lat
    if ctx.get("lkas_blocked"):
      lat = min(lat, self.cfg.ti_only_lat)
    return lat

  def targets(self, p, v, lat):
    """(full plan, reach profile) true m/s over the ramp's points still ahead, or (None, None)."""
    full = reach = None
    for s, r in p.pts:
      d = s - self.odo
      if d < -18.0:
        continue
      vp = max(point_speed(r, lat), TARGET_MIN)
      f = M.envelope(vp, max(d, 0.0), v)
      g = M.envelope(vp, max(d - vp * REACH_LEAD, 0.0), v, decel=REACH_DECEL)
      full = f if full is None else min(full, f)
      reach = g if reach is None else min(reach, g)
    return full, reach

  def _his(self, ctx):
    for e in ctx.get("act_events", ()):
      if e.get("kind") in ("driver_button", "driver"):
        return True
    return bool(ctx.get("set_plus") or ctx.get("set_minus") or ctx.get("resume") or ctx.get("cancel"))

  def _exit_in_view(self, view, driven):
    exits = [e for e in view.get("exits", []) if e.get("d", 1e9) - driven <= EXIT_LOOK]
    p = self.plan
    if p is not None and p.handover and not (p.handed or p.cancelled):
      same = [e for e in exits if e["id"] == p.id]
      return same[0] if same else None      # off the highway: no other exit replaces the handover
    if p is not None and not p.released:
      same = [e for e in exits if e["id"] == p.id]
      if same:
        return same[0]
    ahead = [e for e in exits if e["d"] - driven > 0 and not self._resolved(e, driven)]
    return min(ahead, key=lambda e: e["d"]) if ahead else None

  def _resolved(self, e, driven):
    at = self.done.get(e["id"])
    return at is not None and abs(self.odo + e["d"] - driven - at) <= SAME_EXIT

  def _end(self, why):
    p = self.plan
    if p is not None:
      p.released = p.released or why
      self.ended = p
      self.done[p.id] = p.at
      if len(self.done) > 16:
        self.done.pop(next(iter(self.done)))

  # ---- the frame ----
  def __call__(self, ctx):
    now, v = ctx.get("now", 0.0), max(ctx.get("v_ego", 0.0), 0.0)
    dt = 0.0 if self.t is None else min(max(now - self.t, 0.0), 0.5)
    self.t, self.dt = now, dt
    self.brake_needed = False
    self.odo += v * dt
    if ctx.get("model_new"):
      self.lanes.update(0.05, ctx.get("lanes"))
    out = Contribution("exit", None, presses=self.presses, kind="handover")
    view = ctx.get("view")
    live = self.view_clock.update(view, now)
    if live != GONE and view.get("paused"):
      live = GONE
    self.live = live
    alive = live != GONE
    if alive and (self.good is None or self.good[0]["t"] != view["t"]):
      self.good = (view, self.odo - v * max(now - view["t"], 0.0))   # a new view: where it was on our odometer
    elif alive:
      self.good = (view, self.good[1])
    elif self.good is not None and now - self.good[0]["t"] > DEAD_RECKON:
      self.good = None
    p = self.plan
    if self.good is None:
      if p is not None and not p.confirmed and not p.handover:
        self._end("no map")
        self.plan = None
      self.last = ("no map", None)
      return out
    gview, gt_odo = self.good
    driven = self.odo - gt_odo
    e = self._exit_in_view(gview, driven) if alive or p is not None else None
    if e is not None:
      at = gt_odo + e["d"]
      if p is None or p.id != e["id"] or abs(at - p.at) > SAME_EXIT:
        if p is not None and not p.released:
          self._end("another exit")
        if p is None or not p.handover or p.id != e["id"]:
          p = self.plan = Plan(e["id"], at, e.get("side", 0))
          p.view_t = gview["t"]
      elif alive and gview["t"] != p.view_t and p.at - self.odo > 0.0:
        p.at += AT_GAIN * (at - p.at)          # the same exit: its split's place from the new view
        p.view_t = gview["t"]
      if alive:
        p.pts = [(gt_odo + d, r) for d, r in e.get("pts", [])]
        p.hwy, p.spd = bool(e.get("hwy")), float(e.get("spd", 0.0) or 0.0)
        if e.get("side"):
          p.side = e["side"]
    if p is None:
      self.last = ("watch", None)
      return out
    d = p.at - self.odo              # m to the split (negative: past it)
    p.gas_t = p.gas_t + dt if ctx.get("gas") else 0.0
    ctx["exit_busy"] = bool((p.handover and not (p.handed or p.cancelled)) or
                            (not p.released and (p.confirmed or (d <= SIGNAL_REACH and (p.signal or p.lane or p.model)))))
    if p.confirmed and not p.released and d < -PASSED:
      self._end("ramp done")         # (the curve plan has the ramp's curves; the handover goes on)
    if p.released:
      if p.restore is not None:
        return self._restore(ctx, out, now)
      if p.handover and not (p.cancelled or p.handed) and d > -HANDOVER_MAX:
        return self._handover(ctx, out)
      if d < -PASSED or (e is None and alive):
        self.plan = None
      return out
    # ---- his decisions ----
    if self._his(ctx) or ctx.get("brake"):
      p.cancelled = True             # (a handover is cancelled for good; the ramp's curves stay the curve plan's)
      self._end("his button" if self._his(ctx) else "brake")
      self.last = ("released", p.released)
      return out
    if p.gas_t >= GAS_HELD - 1e-6 and not p.confirmed:
      self._end("his gas")
      return out
    # ---- evidence ----
    if 0.0 < d <= SIGNAL_REACH and p.side and (ctx.get("right_blinker") if p.side > 0 else ctx.get("left_blinker")):
      p.signal = True
      p.blink_t = now
    if p.side and not p.confirmed and (ctx.get("left_blinker") if p.side > 0 else ctx.get("right_blinker")):
      p.signal = p.reach_on = False            # his blinker to the other side: not this exit
      p.blink_t = None
      if d > 0.0:
        p.v_ref = None
    if 0.0 < d <= DECIDE_NEAR and p.side and self.lanes.edge.get(p.side):
      p.lane = True
    ex = e if (e is not None and alive) else {}
    if live != LATE:
      self._evidence(p, ctx, ex, gview, d, v, dt)
    trust_ok = bool(ex and gview.get("trust") == TRUST_OK)
    m = ex.get("m", 0) if ex else 0
    # ---- a confirmation overturned ----
    if p.confirmed and d <= 0.0:
      on_link_now = m == 2 and trust_ok
      if self._hwy_shapes(p) or self._hwy_match(p):
        return self._overturn(ctx, out, "highway after the confirmation", now)
      if p.gas_t >= GAS_HELD - 1e-6 and not on_link_now:
        return self._overturn(ctx, out, "his gas after the confirmation", now)
    # ---- resolve past the split ----
    if d <= 0.0:
      if p.v_split is None:
        p.v_split, p.split_t = v, now
      if not p.confirmed:
        if ((p.link_t >= LINK_CONFIRM - 1e-6 and p.link_n) or (p.match_t >= MATCH_CONFIRM - 1e-6 and p.match_n) or
            (p.shape_link_seen and p.model_link_t >= MODEL_CONFIRM - 1e-6)):
          p.confirmed = True
        elif (self._hwy_shapes(p) or self._hwy_match(p) or
              (p.model_hwy_t >= MODEL_CONFIRM - 1e-6 and p.last_shape != 2)):
          self._end("highway")
          self.last = ("released", "highway")
          return out
        elif d < -PASSED:
          self._end("passed")
          return out
    # ---- the target ----
    full, reach = self.targets(p, v, self.lat(ctx))
    target, why = None, "watch"
    unbounded = None                           # what the ramp needs where the DIP_MAX bound holds the plan higher
    if p.signal and reach is not None and v <= reach + REACH_START:
      if not p.reach_on and p.v_ref is None:
        p.v_ref = v                            # our slowing begins here: DIP_MAX from this speed
      p.reach_on = True
    if p.confirmed:
      target, why = full, "confirmed"
    elif d > 0.0:
      if p.reach_on and reach is not None:
        blink_on = p.blink_t is not None and now - p.blink_t <= BLINKER_GRACE + 1e-6
        edge_lane = bool(p.signal and p.side and self.lanes.edge.get(p.side))
        if blink_on or edge_lane:
          target, why = reach, ("reach signal" if blink_on else "reach edge lane")   # as deep as the ramp needs
        else:
          target, why, unbounded = self._bounded(p, reach), "reach", reach
    else:
      evidence = p.signal or p.lane or p.model
      firm_ok = p.signal or (p.lane and p.model)     # the edge lane alone is also the ordinary right lane
      if firm_ok and full is not None and p.firm_t < FIRM_MAX - 1e-6:
        target, why = full, "firm"
        if full < v - 1.0 * MPH:
          p.firm_t += dt
        p.hold = None
      elif p.reach_on and reach is not None:
        target, why, unbounded = reach, "reach", reach
      elif evidence:
        if p.hold is None:
          p.hold = v
        target, why = p.hold, "hold"
      if target is not None:
        target = self._bounded(p, target)
    if target is not None and full is not None and p.confirmed:
      target = full
    p.last_target = target
    # (part H) the bound keeps the set higher than the ramp needs: the exit brake notice (notices.py)
    self.brake_needed = bool(target is not None and unbounded is not None and not p.confirmed and
                             target > unbounded + BRAKE_GAP)
    out.target = target
    out.episode = p.id
    out.no_raise = bool(target is not None and not p.confirmed)   # unresolved: no SET+ inside the exit area
    self.last = (why, target)
    if p.confirmed and not p.hwy and not p.cancelled:
      p.handover = True
    return self._handover(ctx, out) if p.handover else out

  @staticmethod
  def _hwy_shapes(p):
    return p.hwy_t >= HWY_CONFIRM - 1e-6 and bool(p.hwy_n)

  @staticmethod
  def _hwy_match(p):
    return p.hwy_match_t >= HWY_MATCH - 1e-6 and bool(p.hwy_match_n) and p.last_shape != 2

  def _evidence(self, p, ctx, ex, gview, d, v, dt):
    """The evidence timers of this frame (a CURRENT view, or none: silent shapes, no match). Not called while the view
    is LATE: they hold."""
    vt = gview["t"]
    verdict = self._model_verdict(ctx, ex)
    if -SAME_EXIT <= d <= MODEL_NEAR and verdict == 2:
      p.model_link_t += dt
      if d > 0 and p.model_link_t >= MODEL_CONFIRM - 1e-6:
        p.model = True
    else:
      p.model_link_t = 0.0 if verdict != 2 else p.model_link_t
    p.model_hwy_t = p.model_hwy_t + dt if verdict == 1 else 0.0
    sh = ex.get("sh", 0) if ex else 0
    if sh:
      p.last_shape = sh
    if sh == 2:
      p.quiet_t = p.quiet_m = 0.0
    elif sh == 0:
      p.quiet_t += dt
      p.quiet_m += v * dt
      p.link_t = max(p.link_t - dt, 0.0)       # scattered "link" frames do not add up
      if p.quiet_t >= SHAPE_QUIET_T - 1e-6 or p.quiet_m >= SHAPE_QUIET_M:
        p.link_t = 0.0
        p.shape_link_seen = False
        if p.last_shape == 2:
          p.last_shape = 0                     # an old "link" no longer blocks the release
    p.link_t = p.link_t + dt if sh == 2 else (0.0 if sh == 1 else p.link_t)
    p.shape_link_seen = p.shape_link_seen or sh == 2
    p.hwy_t = p.hwy_t + dt if sh == 1 else (0.0 if sh == 2 else p.hwy_t)
    trust_ok = bool(ex and gview.get("trust") == TRUST_OK)
    m = ex.get("m", 0) if ex else 0
    p.match_t = p.match_t + dt if (m == 2 and trust_ok) else 0.0
    p.hwy_match_t = p.hwy_match_t + dt if (m == 1 and trust_ok and d <= -HWY_MATCH_FROM) else 0.0
    # the views behind each timer (VIEWS_MIN): counted while it grows, forgotten when it restarts
    for t, n, said in ((p.link_t, p.link_n, sh == 2), (p.hwy_t, p.hwy_n, sh == 1), (p.match_t, p.match_n, p.match_t > 0.0),
                       (p.hwy_match_t, p.hwy_match_n, p.hwy_match_t > 0.0)):
      if t <= 0.0:
        n.reset()
      elif said:
        n.add(vt)

  def _bounded(self, p, target):
    """The target with the DIP_MAX bound (the set, true, never more than DIP_MAX under where our slowing began; the
    planner's undershoot taken into account). The bound never raises the plan above its last target: deeper already
    (his blinker was on), it holds there."""
    ref = p.v_ref if p.v_ref is not None else p.v_split
    if ref is None:
      return target
    bound = ref - DIP_MAX + M.UNDERSHOOT
    if target >= bound:
      return target
    hold = p.last_target if p.last_target is not None else bound
    return max(target, min(bound, hold))

  def _model_verdict(self, ctx, ex):
    """2 the model path follows the link, 1 the highway, 0 no answer, None not available."""
    ly, hy = ex.get("ly"), ex.get("hy")
    if ly is None or hy is None or abs(ly - hy) < MODEL_SEP or ctx.get("model_age", 1e9) > 0.5:
      return None
    from openpilot.starpilot.car.mazda.speed.map_view import MODEL_AT
    my = path_y_at(ctx.get("path_y"), MODEL_AT)
    if my is None:
      return None
    dl, dh = abs(my - ly), abs(my - hy)
    return 2 if dl + MODEL_GAP <= dh else (1 if dh + MODEL_GAP <= dl else 0)

  # ---- the handover to the street ----
  def _handover(self, ctx, out):
    p = self.plan
    if p is None or p.cancelled or p.handed:
      return out
    now, v = ctx.get("now", 0.0), max(ctx.get("v_ego", 0.0), 0.0)
    if self._his(ctx) or ctx.get("brake") or p.gas_t >= GAS_HELD - 1e-6:
      p.cancelled = True
      self.last = ("handover cancelled", None)
      return out
    ctx["exit_busy"] = True
    view = ctx.get("view")
    live = self.view_clock.update(view, now)
    trust_ok = bool(live != GONE and view.get("trust") == TRUST_OK and view.get("matched"))
    on_link = bool(trust_ok and view.get("link"))
    street = bool(trust_ok and not view.get("link") and view.get("cls") not in (1, 3))
    if live != LATE or not street:
      p.street_t = p.street_t + self.dt if street else 0.0   # (STREET_STEADY: grows on CURRENT views, holds while LATE)
    if not (on_link or street):
      return out                     # only on a confident match of the ramp or the street
    # (exit_busy: the limit follow waits until the handover is done)
    limit = None
    if street:
      slc = ctx.get("slc")
      limit = whole_limit(slc["limit"], self.metric) if slc else None
      if limit is not None and limit > STREET_MAX[self.k]:
        limit = None
    if limit is None:
      limit = whole_limit(p.spd, self.metric)
    if limit is None or limit > STREET_MAX[self.k]:
      limit = STREET_MAX[self.k] if limit is not None else None
    if limit is None:
      return out
    street_target = max(min(limit + self.offset, STREET_MAX[self.k] + self.offset), STREET_MIN[self.k]) * self.unit
    if p.street_t >= STREET_STEADY - 1e-6:
      prop = street_target
      anchor = ctx.get("anchor")
      if anchor is not None and anchor <= prop + 0.5 * self.unit:
        p.handed = True             # at or under the street's target (a faster street: never raised): done
    else:
      floor = math.ceil((v + M.DASH_OFFSET - HANDOVER_GAP) / self.unit) * self.unit
      prop = max(street_target, floor)
    if p.before is None and ctx.get("anchor") is not None and not (ctx.get("owner") == "ours" and
                                                                   ctx.get("ours_why") == "handover"):
      p.before = ctx.get("anchor")
    out.anchor, out.kind = prop, "handover"
    self.last = ("handover", prop)
    return out

  def _overturn(self, ctx, out, why, now):
    """A confirmation taken back: the ramp target, the handover and our lowered anchor go; his speed comes back."""
    p = self.plan
    p.confirmed = False
    p.cancelled = True
    self._end(why)
    if p.before is not None:
      p.restore = (p.before, now + RESTORE_MAX)
    self.last = ("released", why)
    return self._restore(ctx, out, now) if p.restore is not None else out

  def _restore(self, ctx, out, now):
    p = self.plan
    value, until = p.restore
    anchor = ctx.get("anchor")
    if now > until or (anchor is not None and anchor >= value - 0.5 * self.unit):
      p.restore = None
      return out
    out.anchor, out.kind = value, "restore"
    return out
