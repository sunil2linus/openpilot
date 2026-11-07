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
    driver's Max Drop, down to 25 mph, with a bigger SET- budget. On a link road (the map match's road class, also
    while its curve is held or not matched yet) the camera curve slowdown gets the same floor and budget (CurveMix;
    drive 53: an on-ramp loop held at the Max Drop floor 11 mph above the car's speed).
  * Navigation exit before the split: never on navigation alone. Once the driver signals toward the exit's side
    (any side if navigation does not say) while navigation announces it (up to 1500 m ahead), the exit is
    remembered: the signal may go off again after moving into the exit lane. The exit's tightest curve is then
    planned the same way (so the slowdown starts only as late as that curve allows). When the announcement clears
    close to the split (the GPS match is doubtful there, and navigation moves on to the next maneuver), the exit and
    its curve are carried on by the distance driven: on the link the plan holds until the ramp's own curve is
    matched and trusted (or the carried curve is well behind); past the exit point by EXIT_PASS_MARGIN still on the
    motorway, it is dropped and the speed comes back. Forgotten when the car is on the ramp (the ramp's own curves
    take over), when navigation no longer shows that exit for NAV_GRACE while still far from it (rerouted), or
    when the driver signals the other way.
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
                                              MAP_LATERAL_ACCEL, MAP_DECEL, MAP_UNDERSHOOT, MAP_FOLLOW_BUDGET)
except ImportError:          # offline tests load this file directly
  from curve_target import (MapCurveTarget, MapCurveFollower, map_curve_target, NO_CURVE, START_GAP,
                            MAP_LATERAL_ACCEL, MAP_DECEL, MAP_UNDERSHOOT, MAP_FOLLOW_BUDGET)

MPH = 0.44704
OFF, TEST, ON = 0, 1, 2
RAMP_FLOOR = 25 * MPH
RAMP_BUDGET = 60                # SET- presses for one ramp (70 -> 25 mph plus ignored presses)
NAV_GRACE = 3.0                 # s the exit plan is kept while navigation drops it (map match catching up at the split)
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
# Navigation exit carried on after its announcement clears (drive 53, both exits): the GPS match turns doubtful at the
# split (the exit is not computed then) and navigation moves on to the next maneuver; the exit dropped 46 m and 9 m
# before the exit point. The restore that followed took the set speed 49 -> 70 while the car took the exit at 1165
# (the owner braked from 56 mph before the 95 m ramp curve), and the slowdown stalled for 5 s on the link at 400-405.
NAV_CLOSE = 200.0               # m: an exit cleared this close is the split being reached (farther: a reroute, NAV_GRACE)
# Past the exit point by this, still not on a link: the car stayed on the motorway. The map match sees an exit taken
# only once the ramp is MATCH_STAY away from the main road (drive 53: the link was matched 34 m past the exit point at
# 400 and 137 m past it at 1168), so this is well over that; the price of staying on is a few seconds at the exit's
# speed.
EXIT_PASS_MARGIN = 250.0        # m
# m past the carried curve's entry on the link: it is behind us (drive 53: the 95 m ramp curve took ~127 m), the ramp's
# own rules from there
EXIT_CURVE_PAST = 150.0


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

  @staticmethod
  def from_plan(plan):
    return MapFields(int(plan.mapCurveMode), int(plan.mapCurveStatus), float(plan.mapCurveDistance),
                     float(plan.mapCurveRadius), bool(plan.mapOnRamp), int(plan.mapRampType),
                     float(plan.mapRampToSpeed), bool(plan.mapRampToHighway), float(plan.mapRampEndDistance),
                     int(plan.mapNavExit), float(plan.mapNavExitDistance), float(plan.mapNavCurveRadius),
                     float(plan.mapNavCurveDistance), int(getattr(plan, "mapNavExitSide", 0)),
                     float(getattr(plan, "mapCurveAdvisory", 0.0)), float(getattr(plan, "mapNavCurveAdvisory", 0.0)),
                     int(getattr(plan, "mapRoadClass", 0)), int(getattr(plan, "mapCurveVeto", 0)),
                     float(getattr(plan, "mapCurveVetoRadius", 0.0)), int(getattr(plan, "mapCurveSource", 0)),
                     int(getattr(plan, "mapGpsTrust", 0)))


@dataclass
class MapOutput:
  active: bool = False          # a map reason wants the set speed lowered
  speed: float = NO_CURVE       # m/s, true speed
  floor: float = 0.0            # > 0: a ramp, may go this low past the max drop
  budget: int = 0
  fast_restore: bool = False
  adopt: bool = False           # off an exit onto a street: keep the lowered set speed as the driver's speed
  why: str = ""                 # curve / ramp / nav exit / street cap (for the logs)
  track: bool = False           # a follower's command (road / ramp curve, exit): tracked both ways (SET+ when it comes back up)
  plan: float = NO_CURVE        # m/s, the plan the follower works under (logs)
  curve_speed: float = NO_CURVE # m/s, the speed the governing curve is taken at (shape, sign, a road curve's max drop floor)
  settled: bool = False         # the follower is (nearly) at the curve's speed: its command is the curve's speed
  link: bool = False            # on a link road (ramp): the camera curve slowdown takes the ramp floor and budget (CurveMix)


class MapCurveControl:
  def __init__(self, lat_accel, decel=MAP_DECEL):
    """decel: the cruise's planned slowdown, m/s^2 (MAP_DECEL; the planner plans road curves with the same constant),
    for road curves, ramp curves and navigation exits alike."""
    lat_accel = min(lat_accel, MAP_LATERAL_ACCEL)
    self.lat_accel = lat_accel
    self.decel = decel
    self.curve = MapCurveTarget(lat_accel, decel)
    self.follower = MapCurveFollower()
    self.nav = MapCurveTarget(lat_accel, decel)
    self.nav_follower = MapCurveFollower()
    self.on_street_exit = False
    self.exit_intent = False      # the driver signalled for the announced exit
    self.nav_grace = 0
    self.last_nav = (0.0, 0.0, 0.0)  # radius, distance, advisory of the remembered exit curve
    self.last_exit = 0.0          # m to the exit point when navigation last announced it
    self.last_side = 0            # its side
    self.since_nav = 0.0          # m driven since then
    self.carry = None             # [m to the exit point, m to its curve] after the announcement cleared at the split
    self.ramp_frames = 0          # consecutive updates on a ramp
    self.ramp_end_first = 0.0     # distance to the ramp's end when it was first matched
    self.link_frames = 0          # consecutive updates matched to a link road

  def _forget_exit(self):
    self.exit_intent = False
    self.carry = None

  def _exit_intent(self, f, signal, ramp_held=True, on_link=False, step=0.0):
    """signal: -1 left, 1 right, 0 none; step: m driven since the last update. Returns (radius, distance, advisory) of
    the exit curve to plan, or zeros."""
    nav_ok = f.nav_exit == 1 and f.nav_curve_radius > 0 and not f.on_ramp
    side = f.nav_exit_side or (0 if nav_ok else self.last_side)
    if signal != 0 and side != 0 and signal == -side and not on_link:
      self._forget_exit()         # signalling the other way: not this exit
    elif nav_ok and signal != 0:
      self.exit_intent = True
    if not self.exit_intent:
      self.carry = None
      return 0.0, 0.0, 0.0
    if nav_ok:
      self.nav_grace, self.since_nav, self.carry = 0, 0.0, None
      self.last_nav = (f.nav_curve_radius, f.nav_curve_distance, f.nav_curve_advisory)
      self.last_exit, self.last_side = f.nav_exit_distance, f.nav_exit_side
      return self.last_nav
    self.since_nav += step
    if self.carry is None and self.last_exit - self.since_nav <= NAV_CLOSE:
      # cleared at the split: carry the exit and its curve on by the distance driven
      self.carry = [self.last_exit - self.since_nav, self.last_nav[1] - self.since_nav]
    elif self.carry is not None:
      self.carry[0] -= step
      self.carry[1] -= step
    if self.carry is not None:
      ramp_curve = f.on_ramp and ramp_held and f.status == STATUS_RAMP_CURVE
      if ramp_curve or (on_link and self.carry[1] < -EXIT_CURVE_PAST) or (not on_link and self.carry[0] < -EXIT_PASS_MARGIN):
        # the ramp's own curve is matched and trusted / the carried curve is behind us / we stayed on the motorway
        self._forget_exit()
        return 0.0, 0.0, 0.0
      return self.last_nav[0], max(self.carry[1], 0.0), self.last_nav[2]
    self.nav_grace += 1
    if (f.on_ramp and ramp_held) or self.nav_grace > NAV_GRACE / DT_CTRL:
      self._forget_exit()
      return 0.0, 0.0, 0.0
    return self.last_nav          # brief drop-out: keep the last plan (a ceiling, the ramp curve is lower)

  def update(self, f: MapFields, driver_speed, signal, v_ego=0.0, max_drop=0.0):
    """signal: turn signal side, -1 left, 1 right, 0 none. v_ego: the true speed (m/s) for the followers (0: plan
    only). max_drop: the driver's Curve Max Drop in m/s (0: unknown, no floor in the plan)."""
    out = MapOutput()
    if f.on_ramp:
      if self.ramp_frames == 0:
        self.ramp_end_first = f.ramp_end_distance
      self.ramp_frames += 1
    else:
      self.ramp_frames = 0
    ramp_held = self.ramp_frames >= RAMP_HOLD / DT_CTRL
    self.link_frames = self.link_frames + 1 if f.road_class in LINK_CLASSES else 0
    on_link = (f.on_ramp and ramp_held) or self.link_frames >= LINK_HOLD / DT_CTRL
    if f.mode == OFF or driver_speed <= 0:
      self.curve.active = self.nav.active = False
      self.follower.reset()
      self.nav_follower.reset()
      self._forget_exit()
      self.on_street_exit = f.on_ramp and self.on_street_exit
      return out
    out.link = on_link
    # only 1 and 2 are curves to act on: 4 (paused, GPS not trusted) and 5 (held, road match not trusted) are "no
    # curve" on purpose, so the speed comes back as for any curve that ends (a carried navigation exit is the exception)
    radius = f.radius if f.status == STATUS_CURVE or (f.status == STATUS_RAMP_CURVE and ramp_held) else 0.0
    road_curve = f.status == STATUS_CURVE and not f.on_ramp
    # a road curve is never planned below the driver's Curve Max Drop (the controller would not go there anyway, and
    # planning for a lower speed only starts the slowdown earlier than that speed needs)
    floor = max(driver_speed - max_drop, 0.0) if (road_curve and max_drop > 0) else 0.0
    a, sp = self.curve.update(radius, f.distance, driver_speed, f.advisory, floor=floor)
    ramp_curve = a and f.status == STATUS_RAMP_CURVE and f.on_ramp
    nav_r, nav_d, nav_adv = self._exit_intent(f, int(signal), ramp_held, on_link, max(v_ego, 0.0) * DT_CTRL)
    na, nsp = self.nav.update(nav_r, nav_d, driver_speed, nav_adv)
    speed, why = NO_CURVE, ""
    if a:
      speed, why = sp, ("ramp" if ramp_curve else "curve")
      out.plan = sp
    track, settled = False, False
    if a and (road_curve or ramp_curve) and v_ego > 0:
      # the set speed to ask for so the cruise really slows at the planned rate (see curve_target.MapCurveFollower)
      out.curve_speed = map_curve_target(radius, self.lat_accel, f.advisory, floor)
      speed = self.follower.update(v_ego, out.curve_speed, f.distance, sp)
      track, settled = True, self.follower.settled
    else:
      self.follower.reset()
    if na and v_ego > 0:
      # the exit's curve gets its own follower (drive 53: the 0.5 m/s^2 staircase toward an 85 m curve 930 m ahead
      # left the set speed 5-9 mph under the speed and the car 7-8 mph too fast at the entry)
      nav_speed = map_curve_target(nav_r, self.lat_accel, nav_adv)
      cmd = self.nav_follower.update(v_ego, nav_speed, nav_d, nsp)
      if cmd < speed:
        speed, why, track, settled = cmd, "nav exit", True, self.nav_follower.settled
        out.plan, out.curve_speed = nsp, nav_speed
    else:
      self.nav_follower.reset()
      if na and nsp < speed:
        speed, why, track, settled = nsp, "nav exit", False, False
    exit_to_street = f.on_ramp and ramp_held and f.ramp_type == RAMP_EXIT and not f.ramp_to_highway
    progress = self.ramp_end_first - f.ramp_end_distance >= STREET_PROGRESS
    street_exit = exit_to_street and progress and f.ramp_to_speed > 0
    capped = False
    if street_exit and f.ramp_end_distance <= STREET_CAP_DISTANCE and f.ramp_to_speed < driver_speed - START_GAP:
      if f.ramp_to_speed < speed:
        speed, why, track, settled = f.ramp_to_speed, "street cap", False, False
      capped = True
    out.active = a or na or capped
    out.speed = speed
    out.track = track
    out.settled = track and settled
    if ramp_curve or na or capped:
      # ramps and exits may go past the Max Drop, to the ramp floor; a followed one's undershoot stays as for a road
      # curve (MAP_UNDERSHOOT below the curve's speed), never under the ramp floor
      out.floor = max(RAMP_FLOOR, out.curve_speed - MAP_UNDERSHOOT) if track else RAMP_FLOOR
      out.budget = RAMP_BUDGET
    elif track:
      # the follower may ask up to MAP_UNDERSHOOT below the curve's speed (so at most that far below the max drop
      # floor, which the curve's speed never goes under): that is the floor the controller gets, and nothing lower
      out.floor, out.budget = out.curve_speed - MAP_UNDERSHOOT, MAP_FOLLOW_BUDGET
    out.fast_restore = f.on_ramp and ramp_held and f.ramp_to_highway and not out.active
    out.adopt = self.on_street_exit and not f.on_ramp and f.road_class not in HIGHWAY_CLASSES
    # keep the lowered speed after an exit onto a street: known when the road after it is matched as not a freeway
    # (own road matching), otherwise only after driving along the exit
    self.on_street_exit = (exit_to_street and (progress or f.road_class != 0)) or (self.on_street_exit and f.on_ramp)
    out.why = why
    return out


class CurveMix:
  """The wrapper's combination of the camera curve target with the map control's output (CurveSlowdown.update; the
  offline tests drive it too). Returns (active, speed, floor, budget, track, raise_ok) for SlowdownInputs.

  Without a followed curve: the camera's target, or the map's when it is lower, as before the follower (the
  controller adds its margin, the driver's Curve Max Drop is the floor). On a link road (MapOutput.link: a ramp, also
  while the map holds its curve or has not matched it yet) the camera's target takes the ramp floor and budget instead
  of the Max Drop (drive 53, the on-ramp loop onto M-59 at 1019-1030: the map's ramp curve was matched 8 m before the
  71 m loop and lost inside it, the camera's slowdown stopped at the Max Drop floor, 40 mph with the driver's 50, and
  the car sped up from 27 to 32 mph in the loop behind a car: "Turn Exceeds Steering Limit" twice).
  `no_accel` is True while the camera's target governs (SlowdownInputs.curve_no_accel): the controller then never lets
  the set speed sit above the current speed in the curve.
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
    self.no_accel = False         # the camera's target governs: the set speed never above the current speed in the curve

  def update(self, cam_active, cam_speed, mo, drop_floor=0.0):
    link = mo.link
    self.no_accel = False
    if not (mo.active and mo.track):
      self.cam_low = NO_CURVE
      active, speed = cam_active, cam_speed
      floor, budget = (RAMP_FLOOR, RAMP_BUDGET) if (cam_active and link) else (0.0, 0)
      self.no_accel = cam_active
      if mo.active and (not cam_active or mo.speed < cam_speed):
        speed, floor, budget = mo.speed, mo.floor, mo.budget
        self.no_accel = False
      return active or mo.active, speed, floor, budget, False, False
    if cam_active:
      self.cam_low = min(self.cam_low, max(cam_speed + self.margin, RAMP_FLOOR if link else drop_floor))
    if mo.speed <= self.cam_low:
      return True, mo.speed, mo.floor, mo.budget, True, mo.settled
    return True, self.cam_low, (RAMP_FLOOR if link else 0.0), mo.budget, True, mo.settled
