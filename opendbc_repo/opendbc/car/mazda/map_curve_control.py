"""
Map curves on the Mazda set speed controller: turns the map curve fields of starpilotPlan (published by
starpilot/controls/lib/map_curves.py) into the inputs of CurveSlowdownLogic.

  * Curve ahead (or the one we are in): the set speed allowed now so that a firm cruise slowdown
    (MAP_DECEL, after the cruise's MAP_REACTION_TIME) reaches the curve's speed (MAP_LATERAL_ACCEL, or the car's
    lateral limit if lower, and never below the driver's Curve Max Drop floor) MAP_LEAD_TIME before its entry.
    It starts at the first 1 mph step and ends with the camera curve target's 1 mph end gap. While it is on, the
    set speed asked for is the follower's (curve_target.MapCurveFollower): below the plan by the gap that makes the
    stock cruise slow at the needed rate, at most MAP_UNDERSHOOT below the curve's speed (the floor given to the
    controller allows exactly that), and back up to the curve's speed once the car is nearly there; the controller
    tracks it both ways (SlowdownInputs.curve_track). Ramp curves and navigation exits use the same plan and their
    own follower since drive 53 (their old 0.5 m/s^2 staircase sat only 5-9 mph under the speed and the car entered
    an 85 m exit curve at 38-39 mph for 31), with the ramp floor and budget below instead of the Max Drop.
  * Ramps (we are on an exit / entrance / interchange ramp and the curve is on it): may go past the
    driver's Max Drop, down to 25 mph, with a bigger SET- budget. On a link road connected to a highway (the map
    match's road class, also while its curve is held or not matched yet; HIGHWAY_LINK_CLASSES) the camera curve
    slowdown gets the same floor and budget (CurveMix; drive 53: an on-ramp loop held at the Max Drop floor 11 mph
    above the car's speed); a street to street slip lane keeps the Max Drop.
  * Navigation exit (the owner's exit design; drive 53): navigation is the base. An exit navigation's route takes is a
    plan (ExitPlan) tied to the exit navigation counts down to (where its exit point is by the distance driven), so a
    second exit close by or a reroute never carries the wrong exit's plan; a notice that clears close to the split does
    not end it (drive 53: 46 m before the exit point), the split decides. The turn signal is not needed (he often stays
    in the lane that becomes the exit lane without signalling).
      - Before the split: a gentle slowdown so that the car is at the "exit speed at the split" there (setting
        MazdaExitSplitSpeed: the highway's posted limit, +5, +10, or Off), never raising the speed, only while the
        car is in an exit lane: the turn signal toward the exit, or the camera says we are in the lane next to the
        exit side (LaneView). Map lane data (which lanes exit) is not in the curve files.
      - At the split: confirmed on the ramp by the map's shapes (starpilotPlan.mapExitShape: our position and heading
        closer to the link's than to the highway's, sustained), or the road match on the link with GPS trusted. Until
        then the command holds (no SET-, no SET+). Then the full slowdown toward the exit's curve (the planner's
        exit curve, the lower of the branches of a fork not decided yet; else the announced curve carried on by the
        distance driven), firm (the cruise's strongest) when the room left needs it.
      - Missed (past the split, the shapes keep matching the highway): the plan ends and the speed comes back at once
        (SET+ holds). The driver decides too: his gas, a signal to the other side, or a lane change away from the exit
        side near the split end the exit slowing at once; a last-minute dive into the ramp is still confirmed.
      - "Exit curve - brake" (chime_count): before the split, in an exit lane, when the exit's curve cannot be reached
        from the exit speed at the split with the cruise's ~0.9 m/s^2; also at the confirmation when the room left is
        too short.
    Without navigation there is no plan: ramps are handled once on them.
  * Ramp rules (ramp curves, fast restore, street cap) only start after the car has been on the ramp for
    RAMP_HOLD, and the street rules only after STREET_PROGRESS along the exit, so a short wrong ramp match
    (passing an exit in the right lane) changes nothing.
  * End of a ramp onto a highway: restore at once with SET+ holds (merge).
  * End of an exit onto a street with a known speed limit: within STREET_CAP_DISTANCE of the end, cap the
    set speed at the street's limit; once off the ramp, that lowered set speed becomes the driver's speed
    (never restored to the highway speed on a street).

Pure logic (no openpilot imports) so it can be unit tested; the wrapper in curve_slowdown.py feeds it.
"""
from dataclasses import dataclass

try:
  from opendbc.car.mazda.curve_target import (MapCurveTarget, MapCurveFollower, map_curve_target, NO_CURVE, START_GAP,
                                              END_GAP, MAP_LATERAL_ACCEL, MAP_DECEL, MAP_UNDERSHOOT, MAP_FOLLOW_BUDGET,
                                              MAP_START_GAP, MAP_LEAD_TIME)
except ImportError:          # offline tests load this file directly
  from curve_target import (MapCurveTarget, MapCurveFollower, map_curve_target, NO_CURVE, START_GAP, END_GAP,
                            MAP_LATERAL_ACCEL, MAP_DECEL, MAP_UNDERSHOOT, MAP_FOLLOW_BUDGET, MAP_START_GAP, MAP_LEAD_TIME)

MPH = 0.44704
KPH = 1 / 3.6
OFF, TEST, ON = 0, 1, 2
RAMP_FLOOR = 25 * MPH
RAMP_BUDGET = 60                # SET- presses for one ramp (70 -> 25 mph plus ignored presses)
DT_CTRL = 0.01
STREET_CAP_DISTANCE = 200.0     # m before the end of an exit onto a street
RAMP_HOLD = 1.0                 # s on the ramp before ramp rules apply (a short wrong ramp match does nothing)
STREET_PROGRESS = 50.0          # m driven along an exit (its end getting closer) before the street rules apply
STATUS_CURVE, STATUS_RAMP_CURVE = 1, 2
RAMP_EXIT = 1
HIGHWAY_CLASSES = (1, 3)        # motorway, trunk
LINK_CLASSES = (2, 4, 6, 8, 10)  # motorway / trunk / primary / secondary / tertiary links (map_curves.LINK_CLASSES)
# A link road matched this long counts as a ramp for the camera curve slowdown's floor and budget, also while the map
# holds its curve (the GPS match is doubtful near a split: drive 53 held every ramp curve there for 2-4 s) or has not
# matched one yet (the on-ramp loop at 1019: class 2 one second before the loop, its curve matched 8 m before it)
LINK_HOLD = 0.5                 # s
# ... but only a link connected to a highway (review: street to street slip lanes, primary / secondary links, took the
# ramp floor and CURVE_NO_ACCEL too): one that leads to a highway (mapRampToHighway), one entered from a highway, or a
# motorway / trunk link (these only join a motorway or a trunk). A street slip lane keeps the Max Drop floor.
HIGHWAY_LINK_CLASSES = (2, 4)
TRUST_LOW = 2                   # starpilotPlan.mapGpsTrust (map_curves.TRUST_LOW): another road close by, or worse

# ---- navigation exits (ExitPlan) ----
NAV_PLAN_EXITS = (1, 2)         # starpilotPlan.mapNavExit: matched on the map, with or without curves (3, no data: no plan)
# The exit navigation counts down to is the same exit while its exit point (the distance driven plus the announced
# distance) stays within this of where the plan has it (navigation's distance comes every few seconds and is counted down
# between; drive 53: within ~20 m), else it is another exit (a reroute, the next maneuver).
NAV_SAME_MIN = 60.0             # m
NAV_SAME_SHARE = 0.15           # ... or this share of the announced distance, if more
NAV_GRACE = 3.0                 # s a plan is kept while navigation drops it far from its exit point (then: rerouted)
# Drive 53, both exits: the GPS match turns doubtful at the split (the exit is not computed then) and navigation moves on to
# the next maneuver; the exit dropped 46 m and 9 m before the exit point. A plan that clears this close is the split
# being reached (farther: a reroute, NAV_GRACE), and a new exit announced then waits until the split has decided.
NAV_CLOSE = 200.0               # m
# The shape check (mapExitShape) and the road match on the link count from this far before navigation's exit point
# (where navigation puts the maneuver and where the map's link starts differ by a few tens of m)
EXIT_EARLY = 60.0               # m
EXIT_CONFIRM_TIME = 0.5         # s of the shapes saying "link" (gaps of no answer do not reset it)
EXIT_STAY_TIME = 1.0            # s of the shapes saying "highway" past the exit point: we stayed on (missed exit)
# Planner without the shape check (an older build): past the exit point by this, on the highway with the match not
# doubtful, the exit is missed (twice drive 53's 34 m link match at 400). The last resort for everything:
# EXIT_PASS_MARGIN past the exit point and still not confirmed.
EXIT_STAY_MARGIN = 90.0         # m
EXIT_PASS_MARGIN = 250.0        # m
# m past the carried curve's entry on the link: it is behind us (drive 53: the 95 m ramp curve took ~127 m), the ramp's
# own rules from there
EXIT_CURVE_PAST = 150.0
# The slowdown before the split (owner: traffic on a 70 mph highway runs ~80; arrive at the split at about the limit,
# not lower): planned gently, 1 mph steps. The follower's gap for 0.3 m/s^2 is ~4 mph; it may undershoot the split
# speed by SPLIT_UNDERSHOOT.
SPLIT_DECEL = 0.3               # m/s^2
SPLIT_MAX_DECEL = 0.45          # m/s^2: the follower's most before the split (a late lane change into the exit lane)
SPLIT_UNDERSHOOT = 0.0          # m/s: never below the split speed (no SET+ back up before the split)
# "It never raises the speed": a car already slower than the split speed DECIDE_NEAR before it in an exit lane (behind
# traffic) is not let speed up into the exit either: the set speed is kept at its speed then plus SPLIT_CAP_MARGIN (drive 53, E1:
# 57 mph behind a car with his 75, the car ahead gone 250 m before the split: the cruise would have taken it to 70 with
# a 32 mph curve 270 m after the split).
SPLIT_CAP_MARGIN = 1 * MPH      # m/s
SPLIT_OFFSETS = (0, 5, 10)      # MazdaExitSplitSpeed 0/1/2: the highway's limit + this (mph, km/h when metric); 3: Off
SPLIT_OFF = 3
# The early brake chime: the cruise's strongest slowdown with the set speed held far below (drives 44-53: 0.9-1.0 m/s^2,
# never more) after its ~1 s dead time; the curve's speed reached MAP_LEAD_TIME before its entry.
CHIME_DECEL = 0.9               # m/s^2
CHIME_DEAD = 1.0                # s
# ... and the ramp is only confirmed (the full slowdown starts) a few s past the exit point: drive 53 (its GPS through the
# planner), the shapes said "link" from 3.4 s (E1) and 3.9 s (E2) after navigation's exit point (the car's heading turns
# toward the link's only once the exit lane leaves the highway), confirmed EXIT_CONFIRM_TIME later
CHIME_CONFIRM = 4.0             # s at the split speed
CHIME_AHEAD = 12.0              # s before the split (at least CHIME_AHEAD_MIN): the chime before the split comes from here
CHIME_AHEAD_MIN = 250.0         # m
DECIDE_NEAR = 300.0             # m before the exit point: his gas / a lane change away there is his decision
# A new posted limit on the highway must be steady this long before it is the highway's (not a flicker to the ramp's
# limit near the split)
LIMIT_STEADY = 3.0              # s
RESTORE_PULSE = 1.0             # s the fast restore is asked for after a plan ends by his decision or a missed exit

# ---- the camera's view of our lane (LaneView) ----
# modelV2 lane lines 0-3 (left-left, left, right, right-right; y positive to the right: radard reads model y = -yRel) and
# road edges 0 / 1 (left, right). In the lane next to the right edge: no usable lane line to our right (laneLineProbs[3])
# and the right road edge close to our right lane line (a lane is ~3.6 m; a shoulder adds up to ~3 m). Mirrored for left.
# Not verified on a drive: the qlogs have no lane lines (the exit log, MapExitLog, records them for the next drive).
LANE_LINE_PROB = 0.3            # a lane line beyond ours counts above this probability
LANE_OWN_PROB = 0.3             # our own line on that side counts above this (else the edge is measured from the car)
EDGE_NEAR = 4.0                 # m from our line on that side (or 1.8 m from the car) to the road edge: no other lane fits
EDGE_STD_MAX = 1.0              # m: the edge's own uncertainty must be below this
LANE_NEAR_X = 20.0              # m ahead: the lines and edges are read up to this far (the mean of their points)
LANE_ON_TIME = 1.0              # s of "next to the edge" before it counts
LANE_OFF_TIME = 2.0             # s of "not" before it no longer counts (a gore, a dashed line, a merge)
LANE_DROPOUT = 3.0              # s with no model lines: the last state is kept that long
LANE_JUMP = 2.0                 # m: both our lines jumping this much between two model frames is a lane change
LANE_HALF = 1.8                 # m: half a lane, from the car to its lane line


class LaneView:
  """Which edge lane the car is in (the camera's lane lines and road edges), with hysteresis, and its lane changes.
  update() at the model's rate with (dt, valid, probs[4], lines_y[4], edges_y[2], edge_stds[2]): the y of each at
  the car (the mean up to LANE_NEAR_X)."""

  def __init__(self):
    self.edge = {-1: False, 1: False}     # in the lane next to the left / right road edge (debounced)
    self.raw = {-1: False, 1: False}
    self.on = {-1: 0.0, 1: 0.0}
    self.off = {-1: 0.0, 1: 0.0}
    self.blind = LANE_DROPOUT + 1.0       # s without lines
    self.prev = None                      # (left line y, right line y) last frame both were seen
    self.change = 0                       # the lane change seen on the last update: -1 left, 1 right
    self.gaps = (0.0, 0.0)                # m to the left / right edge from our lines (logs)
    self.probs = (0.0, 0.0, 0.0, 0.0)

  def update(self, dt, valid, probs=None, lines_y=None, edges_y=None, edge_stds=None):
    self.change = 0
    if not valid or not probs or len(probs) < 4 or not lines_y or len(lines_y) < 4 or not edges_y or len(edges_y) < 2:
      self.blind += dt
      self.prev = None
      if self.blind > LANE_DROPOUT:
        for s in (-1, 1):
          self.edge[s] = self.raw[s] = False
          self.on[s] = self.off[s] = 0.0
      return
    self.blind = 0.0
    self.probs = tuple(float(p) for p in probs[:4])
    stds = edge_stds if edge_stds and len(edge_stds) >= 2 else (0.0, 0.0)
    gaps = []
    for s, own, beyond, e in ((-1, 1, 0, 0), (1, 2, 3, 1)):
      ref = lines_y[own] if probs[own] > LANE_OWN_PROB else s * LANE_HALF
      gap = abs(edges_y[e] - ref)
      gaps.append(gap)
      self.raw[s] = probs[beyond] < LANE_LINE_PROB and gap < EDGE_NEAR and stds[e] < EDGE_STD_MAX
      if self.raw[s]:
        self.on[s], self.off[s] = self.on[s] + dt, 0.0
      else:
        self.on[s], self.off[s] = 0.0, self.off[s] + dt
      if self.raw[s] and self.on[s] >= LANE_ON_TIME - 1e-6:
        self.edge[s] = True
      elif not self.raw[s] and self.off[s] >= LANE_OFF_TIME - 1e-6:
        self.edge[s] = False
    self.gaps = (gaps[0], gaps[1])
    both = probs[1] > 0.5 and probs[2] > 0.5
    if both and self.prev is not None:
      jump = ((lines_y[1] - self.prev[0]) + (lines_y[2] - self.prev[1])) / 2
      # (moving left the lines drift right, then at the crossing both lines become the ones left of them: a jump to
      # the left, negative y; moving right the opposite)
      if jump < -LANE_JUMP:
        self.change = -1
      elif jump > LANE_JUMP:
        self.change = 1
    self.prev = (lines_y[1], lines_y[2]) if both else None


@dataclass
class MapFields:
  mode: int = OFF
  status: int = 0
  distance: float = 0.0
  radius: float = 0.0
  on_ramp: bool = False
  ramp_type: int = 0
  ramp_to_speed: float = 0.0
  ramp_to_highway: bool = False
  ramp_end_distance: float = 0.0
  nav_exit: int = 0
  nav_exit_distance: float = 0.0
  nav_curve_radius: float = 0.0
  nav_curve_distance: float = 0.0
  nav_exit_side: int = 0          # -1 left, 1 right, 0 unknown
  advisory: float = 0.0           # m/s, yellow sign speed of the curve (0 none)
  nav_curve_advisory: float = 0.0
  road_class: int = 0             # matched road (0 unknown); a freeway never takes the street rules
  veto: int = 0                   # planner's camera check (logged only): 1 a map curve is ignored, 2 taken back
  veto_radius: float = 0.0        # m, the camera's radius of that curve
  source: int = 0                 # governing curve's radius: 0 map, 1 curve memory, 2 memory speed cap
  gps_trust: int = 0              # planner's GPS trust (logged only): 0 ok, 1 shaky, 2 low, 3/4 paused
  limit: float = 0.0              # m/s, the posted limit (slcSpeedLimit, 0 unknown)
  exit_shape: int = 0             # mapExitShape: 0 no answer, 1 the highway's shape, 2 the exit link's
  exit_curve_radius: float = 0.0  # mapExitCurve*: the watched exit's next curve from the car (forks decided), 0 none
  exit_curve_distance: float = 0.0
  exit_curve_advisory: float = 0.0
  exit_fork: int = 0              # mapExitFork: 0 none, 1 not decided yet, 2 / 3 decided (navigation's / the other branch)

  @staticmethod
  def from_plan(plan):
    g = lambda k, d=0.0: getattr(plan, k, d)  # noqa: E731  (an older planner has no such field)
    return MapFields(int(plan.mapCurveMode), int(plan.mapCurveStatus), float(plan.mapCurveDistance),
                     float(plan.mapCurveRadius), bool(plan.mapOnRamp), int(plan.mapRampType),
                     float(plan.mapRampToSpeed), bool(plan.mapRampToHighway), float(plan.mapRampEndDistance),
                     int(plan.mapNavExit), float(plan.mapNavExitDistance), float(plan.mapNavCurveRadius),
                     float(plan.mapNavCurveDistance), int(g("mapNavExitSide", 0)),
                     float(g("mapCurveAdvisory")), float(g("mapNavCurveAdvisory")),
                     int(g("mapRoadClass", 0)), int(g("mapCurveVeto", 0)),
                     float(g("mapCurveVetoRadius")), int(g("mapCurveSource", 0)),
                     int(g("mapGpsTrust", 0)), float(g("slcSpeedLimit") or 0.0), int(g("mapExitShape", 0)),
                     float(g("mapExitCurveRadius")), float(g("mapExitCurveDistance")), float(g("mapExitCurveAdvisory")),
                     int(g("mapExitFork", 0)))


@dataclass
class MapOutput:
  active: bool = False          # a map reason wants the set speed lowered
  speed: float = NO_CURVE       # m/s, true speed
  floor: float = 0.0            # > 0: a ramp, may go this low past the max drop
  budget: int = 0
  fast_restore: bool = False
  adopt: bool = False           # off an exit onto a street: keep the lowered set speed as the driver's speed
  why: str = ""                 # curve / ramp / nav exit / exit split / exit hold / street cap (for the logs)
  track: bool = False           # a follower's command (road / ramp curve, exit): tracked both ways (SET+ when it comes back up)
  plan: float = NO_CURVE        # m/s, the plan the follower works under (logs)
  curve_speed: float = NO_CURVE # m/s, the speed the governing curve is taken at (shape, sign, a road curve's max drop floor)
  settled: bool = False         # the follower is (nearly) at the curve's speed: its command is the curve's speed
  link: bool = False            # on a link road (ramp): the camera curve slowdown takes the ramp floor and budget (CurveMix)
  firm: bool = False            # the governing follower needs the cruise's strongest slowdown: a SET- hold may take it there
  chime: bool = False           # "Exit curve - brake" this frame


class ExitPlan:
  """The exit navigation's route takes, by where it is (m of the control's odometer)."""

  def __init__(self, odo, f):
    self.exit_at = odo + f.nav_exit_distance
    self.side = f.nav_exit_side
    self.curve = (0.0, 0.0, 0.0)  # radius, m of the odometer at its entry, advisory
    self.limit = 0.0              # m/s, the highway's posted limit (last known while on the highway)
    self.limit_new, self.limit_s = 0.0, 0.0   # a new one and how long it has been steady
    self.confirmed = False
    self.declined = False         # his decision (gas, the other signal, a lane change away): no exit slowing before the ramp
    self.signalled = False        # he signalled toward the exit (kept: the signal may go off once in the exit lane)
    self.chimed = False           # the brake chime before the split
    self.slowing = False          # the slowdown before the split is on
    self.v_cap = 0.0              # m/s: the car was slower than the split speed when it began: the speed is kept at this
    self.hold = None              # m/s: the command held from the split to the confirmation
    self.link_s = self.hwy_s = 0.0  # s of the shapes saying link / highway
    self.grace = 0.0              # s navigation has not shown it (far from it)
    self.seen_shape = False       # the planner gives the shape check
    self.why = ""                 # how it ended (logs)
    self.update_nav(odo, f)

  def update_nav(self, odo, f):
    self.exit_at = odo + f.nav_exit_distance
    if f.nav_exit_side:
      self.side = f.nav_exit_side
    if f.nav_curve_radius > 0:
      self.curve = (f.nav_curve_radius, odo + f.nav_curve_distance, f.nav_curve_advisory)
    self.grace = 0.0

  def same(self, odo, f):
    d = f.nav_exit_distance
    return abs(odo + d - self.exit_at) <= max(NAV_SAME_MIN, NAV_SAME_SHARE * d)


def reach_needed(v, v_curve, decel=CHIME_DECEL, dead=CHIME_DEAD, lead=MAP_LEAD_TIME):
  """m the cruise needs from v (m/s) to be at v_curve MAP_LEAD_TIME before a curve's entry at its strongest slowdown."""
  if v <= v_curve:
    return 0.0
  return v * dead + (v * v - v_curve * v_curve) / (2 * decel) + v_curve * lead


class MapCurveControl:
  def __init__(self, lat_accel, decel=MAP_DECEL, split_mode=0, metric=False):
    """decel: the cruise's planned slowdown, m/s^2 (MAP_DECEL; the planner plans road curves with the same constant),
    for road curves, ramp curves and navigation exits alike. split_mode: MazdaExitSplitSpeed (0 the highway's limit, 1
    +5, 2 +10, 3 Off); metric: the offsets are km/h."""
    lat_accel = min(lat_accel, MAP_LATERAL_ACCEL)
    self.lat_accel = lat_accel
    self.decel = decel
    self.split_mode = split_mode if split_mode in (0, 1, 2, SPLIT_OFF) else 0
    self.unit = KPH if metric else MPH
    self.curve = MapCurveTarget(lat_accel, decel)
    self.follower = MapCurveFollower(firm=True)          # (firm only for a ramp curve, see update)
    self.road_follower = MapCurveFollower()
    self.nav = MapCurveTarget(lat_accel, decel)
    self.nav_follower = MapCurveFollower(firm=True)
    self.split_follower = MapCurveFollower(max_decel=SPLIT_MAX_DECEL, undershoot=SPLIT_UNDERSHOOT, lead_time=0.0)
    self.split_active = False
    self.lane = LaneView()          # fed by the wrapper from modelV2
    self.odo = 0.0                  # m driven (vEgo x dt; drive() through a planner gap)
    self.plan = None                # ExitPlan
    self.last_plan = None           # the plan that ended last (logs; its exit is never taken on again)
    self.done_at = None             # m of the odometer: the exit point of a plan that ended (no re-adoption)
    self.restore_frames = 0         # fast restore asked for this many frames more (RESTORE_PULSE)
    self.chime_count = 0            # "Exit curve - brake" chimes so far (the wrapper raises the alert on a change)
    self.chime_why = ""
    self.on_street_exit = False
    self.ramp_frames = 0            # consecutive updates on a ramp
    self.ramp_end_first = 0.0       # distance to the ramp's end when it was first matched
    self.link_frames = 0            # consecutive updates matched to a link road
    self.road_before = 0            # the last road class matched that is not a link (0 unknown): where a link was entered
    self.link_hwy = False           # the link we are on is connected to a highway (HIGHWAY_LINK_CLASSES)

  @property
  def exit_rem(self):
    """m to the planned exit point (negative past it), None without a plan."""
    return None if self.plan is None else self.plan.exit_at - self.odo

  def drive(self, step):
    """m driven (the car's own speed x dt) while the planner is missing (the wrapper calls update only while it is
    alive): the exit plan is counted on by it, so a planner gap does not stretch the slowdown past the exit point."""
    self.odo += step

  def _end(self, why, restore=False):
    p = self.plan
    if p is not None:
      p.why = why
      self.last_plan, self.done_at = p, p.exit_at
    self.plan = None
    self.split_active = False
    self.split_follower.reset()
    if restore:
      self.restore_frames = int(RESTORE_PULSE / DT_CTRL)

  def _chime(self, why):
    self.chime_count += 1
    self.chime_why = why

  def split_speed(self, p):
    """m/s, the chosen exit speed at the split (0: none, Off or the highway's limit unknown)."""
    if self.split_mode == SPLIT_OFF or p.limit <= 0:
      return 0.0
    return p.limit + SPLIT_OFFSETS[self.split_mode] * self.unit

  def _nav_plan(self, f, on_link, signal):
    """Adopt / follow / drop the plan from navigation's exit."""
    p = self.plan
    announced = f.nav_exit in NAV_PLAN_EXITS and f.nav_exit_distance > 0 and not on_link
    if announced:
      if p is not None and p.same(self.odo, f):
        p.update_nav(self.odo, f)
      elif p is not None and not p.confirmed and p.exit_at - self.odo > NAV_CLOSE:
        self._end("rerouted")                       # another exit far from this one's split: navigation changed its route
        p = None
      if p is None and f.road_class not in LINK_CLASSES and not (
          self.done_at is not None and abs(self.odo + f.nav_exit_distance - self.done_at) <= NAV_SAME_MIN):
        p = self.plan = ExitPlan(self.odo, f)
    elif p is not None and not p.confirmed and p.exit_at - self.odo > NAV_CLOSE:
      p.grace += DT_CTRL
      if p.grace > NAV_GRACE:
        self._end("navigation dropped it")
    p = self.plan
    if p is not None:
      # the highway's own limit: one steady LIMIT_STEADY on the highway (kept through a gap or at the split, where the
      # posted limit may already be the ramp's)
      if f.road_class in HIGHWAY_CLASSES and f.limit > 0 and not on_link:
        p.limit_s = p.limit_s + DT_CTRL if abs(f.limit - p.limit_new) < 0.1 else 0.0
        p.limit_new = f.limit
        if p.limit_s >= LIMIT_STEADY - 1e-6 or p.limit <= 0:
          p.limit = f.limit
      else:
        p.limit_s = 0.0
    return p

  def _exit(self, f, driver_speed, v_ego, on_link, ramp_held, signal, gas):
    """The plan's command: (speed, plan, curve speed, why, follower or None, firm). NO_CURVE when it asks for nothing."""
    p = self._nav_plan(f, on_link, signal)
    none = (NO_CURVE, NO_CURVE, NO_CURVE, "", None, False)
    if p is None:
      return none
    rem = p.exit_at - self.odo
    side = p.side
    # the shape check (the planner's), the road match on the link with GPS trusted, the ramp itself
    if f.exit_shape:
      p.seen_shape = True
    # (the link's from EXIT_EARLY before the exit point, the highway's past it; no answer keeps both counts)
    if f.exit_shape == 2:
      p.link_s, p.hwy_s = (p.link_s + DT_CTRL if rem <= EXIT_EARLY else 0.0), 0.0
    elif f.exit_shape == 1:
      p.link_s, p.hwy_s = 0.0, (p.hwy_s + DT_CTRL if rem < 0 else 0.0)
    link_match = (on_link and f.gps_trust < TRUST_LOW) or (f.on_ramp and ramp_held)
    if not p.confirmed:
      if on_link and f.gps_trust < TRUST_LOW and rem > EXIT_EARLY + NAV_SAME_MIN:
        self._end("an earlier exit")              # on a link well before this exit: its own ramp rules
        return none
      if rem <= EXIT_EARLY and (p.link_s >= EXIT_CONFIRM_TIME - 1e-6 or link_match):
        p.confirmed = True
        self.split_active = False
        self.split_follower.reset()
        r, at, adv = self._curve_now(f, p)
        if r > 0 and reach_needed(v_ego, map_curve_target(r, self.lat_accel, adv)) > at:
          self._chime("confirmed, too little room")
    # his decision: the other signal, a lane change away from the exit side near it, his gas while we slow for it
    near = rem <= DECIDE_NEAR
    if signal and side and not p.confirmed:
      if signal == side and rem > 0:
        p.signalled, p.declined = True, False      # (a new signal toward it takes a decision back)
      elif signal == -side and (near or p.slowing):
        p.declined, p.signalled = True, False
        self._end_slowing(p)
    elif signal and not side and not p.confirmed and rem > 0:
      p.signalled = True
    if not p.confirmed and side and self.lane.change == -side and near and not p.declined:
      p.declined = True
      self._end_slowing(p)
    if gas and not p.confirmed and (near or p.slowing) and not p.declined:
      p.declined = True
      self._end_slowing(p)
    if p.confirmed:
      return self._ramp_command(f, p, driver_speed, v_ego, on_link)
    # missed: past the exit point the shapes keep matching the highway; an older planner: on the highway, not doubtful,
    # past EXIT_STAY_MARGIN; anything: past EXIT_PASS_MARGIN
    if rem < 0 and (p.hwy_s >= EXIT_STAY_TIME - 1e-6 or rem < -EXIT_PASS_MARGIN or
                    (not p.seen_shape and f.road_class in HIGHWAY_CLASSES and f.gps_trust < TRUST_LOW and
                     rem < -EXIT_STAY_MARGIN)):
      self._end("missed", restore=True)
      return none
    if p.declined:
      return none
    in_lane = p.signalled or (side != 0 and self.lane.edge[side])
    vs = self.split_speed(p)
    # slower than the split speed (or his speed, the highway's limit unknown) near it: kept from speeding up into the exit
    if (rem > 0 and in_lane and self.split_mode != SPLIT_OFF and not p.v_cap and rem <= DECIDE_NEAR and
        v_ego + SPLIT_CAP_MARGIN < (vs if vs > 0 else driver_speed)):
      p.v_cap = v_ego + SPLIT_CAP_MARGIN
    if p.v_cap:
      vs = min(vs, p.v_cap) if vs > 0 else p.v_cap
    if rem > 0:
      if in_lane and not p.chimed and p.curve[0] > 0 and rem <= max(CHIME_AHEAD * v_ego, CHIME_AHEAD_MIN):
        # the speed at the split: the chosen one (slowed to it, or the cruise may still speed up to it), else the current
        v_at = min(vs, driver_speed) if vs > 0 else v_ego
        r, at, adv = p.curve
        if v_at * CHIME_CONFIRM + reach_needed(v_at, map_curve_target(r, self.lat_accel, adv)) > at - p.exit_at:
          p.chimed = True
          self._chime("before the split")
      if vs <= 0 or not in_lane or driver_speed <= vs + START_GAP:
        if p.slowing and not in_lane:
          self._end_slowing(p)                    # out of the exit lane: the speed comes back as after any curve
        return none
      plan = (vs * vs + 2 * SPLIT_DECEL * max(rem - v_ego * 1.5, 0.0)) ** 0.5
      if not self.split_active:
        self.split_active = plan <= driver_speed - MAP_START_GAP
      else:
        self.split_active = plan < driver_speed - END_GAP
      if not self.split_active:
        return none
      p.slowing = True
      cmd = min(self.split_follower.update(v_ego, vs, rem, plan) if v_ego > 0 else plan, driver_speed)
      p.hold = cmd
      return cmd, plan, vs, "exit split", self.split_follower, False
    # past the exit point, not confirmed yet: hold what the slowdown had (no SET-, no SET+)
    if p.hold is not None and p.slowing:
      return p.hold, p.hold, p.hold, "exit hold", None, False
    return none

  def _end_slowing(self, p):
    if p.slowing:
      self.restore_frames = int(RESTORE_PULSE / DT_CTRL)
    p.slowing, p.hold = False, None
    self.split_active = False
    self.split_follower.reset()

  def _curve_now(self, f, p):
    """(radius, m to its entry, advisory) of the exit's curve to plan now: the planner's watched exit curve (forks
    decided: the lower of the branches until then), else the announced one carried on by the distance driven."""
    if f.exit_curve_radius > 0:
      return f.exit_curve_radius, max(f.exit_curve_distance, 0.0), f.exit_curve_advisory
    r, at, adv = p.curve
    return r, (at - self.odo if r > 0 else 0.0), adv

  def _ramp_command(self, f, p, driver_speed, v_ego, on_link):
    """Confirmed on the ramp: the full slowdown toward its curve."""
    none = (NO_CURVE, NO_CURVE, NO_CURVE, "", None, False)
    ramp_curve = f.on_ramp and self.ramp_frames >= RAMP_HOLD / DT_CTRL and f.status == STATUS_RAMP_CURVE
    r, d, adv = self._curve_now(f, p)
    carried_past = r > 0 and f.exit_curve_radius <= 0 and d <= 0 and p.curve[1] - self.odo < -EXIT_CURVE_PAST
    if (ramp_curve and f.exit_fork in (0, 2, 3)) or carried_past or (r <= 0 and on_link):
      self._end("on the ramp")                    # the ramp's own curves from here
      return none
    a, sp = self.nav.update(r, max(d, 0.0), driver_speed, adv)
    if not a:
      self.nav_follower.reset()
      return none
    v_c = map_curve_target(r, self.lat_accel, adv)
    if v_ego <= 0:
      return sp, sp, v_c, "nav exit", None, False
    cmd = self.nav_follower.update(v_ego, v_c, max(d, 0.0), sp)
    return cmd, sp, v_c, "nav exit", self.nav_follower, self.nav_follower.firm

  def update(self, f: MapFields, driver_speed, signal, v_ego=0.0, max_drop=0.0, gas=False):
    """signal: turn signal side, -1 left, 1 right, 0 none. v_ego: the true speed (m/s) for the followers (0: plan
    only). max_drop: the driver's Curve Max Drop in m/s (0: unknown, no floor in the plan). gas: the driver is on the
    gas (his decision near an exit)."""
    out = MapOutput()
    self.odo += max(v_ego, 0.0) * DT_CTRL
    if self.restore_frames > 0:
      self.restore_frames -= 1
    if f.on_ramp:
      if self.ramp_frames == 0:
        self.ramp_end_first = f.ramp_end_distance
      self.ramp_frames += 1
    else:
      self.ramp_frames = 0
    ramp_held = self.ramp_frames >= RAMP_HOLD / DT_CTRL
    self.link_frames = self.link_frames + 1 if f.road_class in LINK_CLASSES else 0
    on_link = (f.on_ramp and ramp_held) or self.link_frames >= LINK_HOLD / DT_CTRL
    if f.road_class and f.road_class not in LINK_CLASSES and not f.on_ramp:
      self.road_before, self.link_hwy = f.road_class, False
    elif (f.road_class in LINK_CLASSES or f.on_ramp) and (f.ramp_to_highway or f.road_class in HIGHWAY_LINK_CLASSES or
                                                          self.road_before in HIGHWAY_CLASSES):
      self.link_hwy = True
    if f.mode == OFF or driver_speed <= 0:
      self.curve.active = self.nav.active = False
      self.follower.reset()
      self.road_follower.reset()
      self.nav_follower.reset()
      if self.plan is not None:
        self._end("off")
      self.restore_frames = 0
      self.on_street_exit = f.on_ramp and self.on_street_exit
      return out
    out.link = on_link and self.link_hwy
    # only 1 and 2 are curves to act on: 4 (paused, GPS not trusted) and 5 (held, road match not trusted) are "no
    # curve" on purpose, so the speed comes back as for any curve that ends (a navigation exit plan is the exception)
    radius = f.radius if f.status == STATUS_CURVE or (f.status == STATUS_RAMP_CURVE and ramp_held) else 0.0
    road_curve = f.status == STATUS_CURVE and not f.on_ramp
    # a road curve is never planned below the driver's Curve Max Drop (the controller would not go there anyway, and
    # planning for a lower speed only starts the slowdown earlier than that speed needs)
    floor = max(driver_speed - max_drop, 0.0) if (road_curve and max_drop > 0) else 0.0
    a, sp = self.curve.update(radius, f.distance, driver_speed, f.advisory, floor=floor)
    ramp_curve = a and f.status == STATUS_RAMP_CURVE and f.on_ramp
    speed, why = NO_CURVE, ""
    if a:
      speed, why = sp, ("ramp" if ramp_curve else "curve")
      out.plan = sp
    track, settled, firm = False, False, False
    if a and (road_curve or ramp_curve) and v_ego > 0:
      # the set speed to ask for so the cruise really slows at the planned rate (see curve_target.MapCurveFollower);
      # a ramp curve's may turn firm (the cruise's strongest) when the room left needs it, a road curve's never
      fol, other = (self.follower, self.road_follower) if ramp_curve else (self.road_follower, self.follower)
      other.reset()
      out.curve_speed = map_curve_target(radius, self.lat_accel, f.advisory, floor)
      speed = fol.update(v_ego, out.curve_speed, f.distance, sp)
      track, settled, firm = True, fol.settled, fol.firm
      low = fol.low
    else:
      self.follower.reset()
      self.road_follower.reset()
      low = NO_CURVE
    cmd, nplan, ncurve, nwhy, nfol, nfirm = self._exit(f, driver_speed, v_ego, on_link, ramp_held, int(signal), gas)
    na = cmd < NO_CURVE
    if na and cmd < speed:
      speed, why = cmd, nwhy
      track = nfol is not None or nwhy == "exit hold"
      settled = nfol.settled if nfol is not None else nwhy == "exit hold"
      firm = nfirm
      out.plan, out.curve_speed = nplan, ncurve
      low = nfol.low if nfol is not None else cmd
    exit_to_street = f.on_ramp and ramp_held and f.ramp_type == RAMP_EXIT and not f.ramp_to_highway
    progress = self.ramp_end_first - f.ramp_end_distance >= STREET_PROGRESS
    street_exit = exit_to_street and progress and f.ramp_to_speed > 0
    capped = False
    if street_exit and f.ramp_end_distance <= STREET_CAP_DISTANCE and f.ramp_to_speed < driver_speed - START_GAP:
      if f.ramp_to_speed < speed:
        speed, why, track, settled, firm = f.ramp_to_speed, "street cap", False, False, False
      capped = True
    out.active = a or na or capped
    out.speed = speed
    out.track = track
    out.settled = track and settled
    out.firm = firm and track
    if ramp_curve or na or capped:
      # ramps and exits may go past the Max Drop, to the ramp floor; a followed one's undershoot stays as for a road
      # curve (MAP_UNDERSHOOT below the curve's speed, FIRM_UNDERSHOOT when firm), never under the ramp floor
      out.floor = max(RAMP_FLOOR, low) if track else RAMP_FLOOR
      out.budget = RAMP_BUDGET
    elif track:
      # the follower may ask up to MAP_UNDERSHOOT below the curve's speed (so at most that far below the max drop
      # floor, which the curve's speed never goes under): that is the floor the controller gets, and nothing lower
      out.floor, out.budget = out.curve_speed - MAP_UNDERSHOOT, MAP_FOLLOW_BUDGET
    out.fast_restore = (f.on_ramp and ramp_held and f.ramp_to_highway and not out.active) or \
      (self.restore_frames > 0 and not out.active)
    out.adopt = self.on_street_exit and not f.on_ramp and f.road_class not in HIGHWAY_CLASSES
    # keep the lowered speed after an exit onto a street: known when the road after it is matched as not a freeway
    # (own road matching), otherwise only after driving along the exit
    self.on_street_exit = (exit_to_street and (progress or f.road_class != 0)) or (self.on_street_exit and f.on_ramp)
    out.why = why
    out.chime = self.chime_count != getattr(self, "_chimes_out", 0)
    self._chimes_out = self.chime_count
    return out

  def state(self):
    """The exit plan's state for the logs."""
    p = self.plan
    if p is None:
      return ""
    if p.confirmed:
      return "confirmed"
    if p.declined:
      return "declined"
    return "split hold" if (p.exit_at - self.odo) <= 0 else ("slowing" if p.slowing else "planned")


class CurveMix:
  """The wrapper's combination of the camera curve target with the map control's output (CurveSlowdown.update; the
  offline tests drive it too). Returns (active, speed, floor, budget, track, raise_ok) for SlowdownInputs.

  Without a followed curve: the camera's target, or the map's when it is lower, as before the follower (the
  controller adds its margin, the driver's Curve Max Drop is the floor). On a link road (MapOutput.link: a ramp, also
  while the map holds its curve or has not matched it yet) the camera's target takes the ramp floor and budget instead
  of the Max Drop (drive 53, the on-ramp loop onto M-59 at 1019-1030: the map's ramp curve was matched 8 m before the
  71 m loop and lost inside it, the camera's slowdown stopped at the Max Drop floor, 40 mph with the driver's 50, and
  the car sped up from 27 to 32 mph in the loop behind a car: "Turn Exceeds Steering Limit" twice).
  `no_accel` is True while the camera's target governs on a link (SlowdownInputs.curve_no_accel): the controller then
  never lets the set speed sit above the current speed in the curve. On a normal road the camera curve slowdown is as
  before (no extra SET- behind a slower car in a curve).
  While a follower runs, the camera's target counts at the lowest it has been since the follower started, with the
  margin and the Max Drop floor the controller gives it (`drop_floor`; the ramp floor on a link). The camera's target
  is the least demanding curvature of the last second, so it pops up for a second at a time (review sC: a camera
  target alternating 27 / 38 inside a 32 mph map curve made the step back up follow it, 3-4 SET+ then 3-4 SET- over
  and over). Counted at its lowest it can only lower the target, as it always could, and never raise it: the only SET+
  in the curve is the follower's own undershoot being taken back, up to the lower of the follower's command and that
  camera target (review: with the camera governing and no step back up at all, the undershoot stayed 4 mph below the
  Max Drop floor through the whole curve). The governing target is passed with its margin already in (track: no
  margin added), the camera's with the Max Drop floor (floor 0) or the ramp floor, the follower's with its undershoot
  floor.
  raise_ok: the follower has settled at the curve's speed; the only other way its command goes up is a restart
  (another curve governs, or its speed changed), which is no reason to raise (review s4c: a radius re-matched 91 ->
  100 m mid-approach raised 32 -> 33 at 38 mph, then three SET- again)."""

  def __init__(self, margin):
    self.margin = margin
    self.cam_low = NO_CURVE       # the camera's lowest target (margin and Max Drop floor in) while the follower runs
    self.no_accel = False         # the camera's target governs on a link: the set speed never above the current speed
    self.firm = False             # the governing follower is firm (MapOutput.firm): a SET- hold may take the set speed there

  def update(self, cam_active, cam_speed, mo, drop_floor=0.0):
    link = mo.link
    self.no_accel = False
    self.firm = False
    if not (mo.active and mo.track):
      self.cam_low = NO_CURVE
      active, speed = cam_active, cam_speed
      floor, budget = (RAMP_FLOOR, RAMP_BUDGET) if (cam_active and link) else (0.0, 0)
      self.no_accel = cam_active and link
      if mo.active and (not cam_active or mo.speed < cam_speed):
        speed, floor, budget = mo.speed, mo.floor, mo.budget
        self.no_accel = False
      return active or mo.active, speed, floor, budget, False, False
    if cam_active:
      self.cam_low = min(self.cam_low, max(cam_speed + self.margin, RAMP_FLOOR if link else drop_floor))
    if mo.speed <= self.cam_low:
      self.firm = mo.firm
      return True, mo.speed, mo.floor, mo.budget, True, mo.settled
    return True, self.cam_low, (RAMP_FLOOR if link else 0.0), mo.budget, True, mo.settled
