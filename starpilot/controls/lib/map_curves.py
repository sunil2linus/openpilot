"""
Map curves: the curve (or ramp) ahead from the curve files made by tools/curve_map/build_curve_map.py.

Runs in the FrogPilot planner (frogpilot_process) at the planner rate, and publishes the governing curve
ahead in frogpilotPlan; the Mazda car controller turns it into a set speed target with its own lateral
limit and acts on it when the setting is On (MazdaMapCurves: 0 off, 1 test, 2 on).

  * Files: /data/media/0/curve_map/<lat>/<lon>/<0.25 deg square>.json. The squares around the car are
    loaded in a background thread (never in the planner loop); curves, ramps and road chains of all loaded
    squares are merged (the same curve from two folders is kept once).
  * Where we are: the road (way id) and the direction along it (is_forward). FrogPilot's map program
    publishes this (MapMatchedWay); where it does not (StarPilot's newer map program), the road shapes in
    the curve files (format 4) are matched here (RoadMatcher). The road chain that way belongs to gives every
    curve on the road ahead, in order, also on later ways of the same road. Distances are straight lines from
    the GPS position (a little short on a winding road, i.e. on the early side).
  * Which curve: of the curves within LOOKAHEAD ahead (and the one we are in), the one that needs the
    lowest speed now with a gentle slowdown (PLAN_DECEL, after the cruise reacts) to reach its speed
    PLAN_LEAD_TIME before its entry.
  * Ramps: when the road is a ramp (exit / entrance / interchange) we know where it leads: the road class,
    its speed limit and the distance to the end of the ramp.
  * Navigation: when the route says an exit ("off ramp" / "fork") is coming, the exit ramp is matched by
    distance and direction, so its curves are known before the split (used for the on-screen notice and
    chime, and, with the turn signal on, to start early).

No openpilot imports at module level (MapCurveStore / MapCurves are unit tested offline).
"""
import glob
import json
import math
import os
import threading
import time
from array import array

CURVE_DIR = "/data/media/0/curve_map"
MPH = 0.44704
EARTH_R = 6371008.8

LOOKAHEAD = 1000.0        # m along the road
PLAN_LAT = 2.2            # m/s^2: as the car side (curve_target.MAP_LATERAL_ACCEL)
# Ramp curves (ramps and exits) are taken faster (owner, drives 61 / 64: "we are the bottleneck"; the same ramps by hand:
# the 68 m loop at 34-35 mph, exit curves entered at 45-47; with our 2.2 m/s^2: 27 and 32-33). The car's steering has
# measured ~2.7 m/s^2 as what it holds (curve learning, Torque Interceptor), so ramp curves are planned at RAMP_LAT: the
# radius the planner gives for a ramp curve is scaled so that the car side's PLAN_LAT speed is the RAMP_LAT speed
# (ramp_radius; the notices, which show the speed at PLAN_LAT for the published radius, agree). Road curves keep
# PLAN_LAT and the curve memory.
RAMP_LAT = 2.6            # m/s^2


def ramp_radius(c):
  """The radius to plan curve record `c` with: a ramp curve's is scaled for RAMP_LAT (see above)."""
  r = float(c["r"])
  return r * RAMP_LAT / PLAN_LAT if c.get("ramp") else r
# PLAN_DECEL: as the car side for road curves (curve_target.MAP_DECEL, the drive evidence is there): the slowdown is
# planned firm and the car side commands the set speed below the plan so the stock cruise really slows that hard
# (drive 50: the old gentle 0.25 plan started 380-395 m before West Utica's 30 mph curve). The car profile's learned
# decel (car_profile.py decel.use, the pace of the small-step staircase) is still measured and logged but no longer
# planned with on either side: it would only start the slowdown earlier again. Both sides use this constant, so the
# planner's "Slowing to" notices and the car side's set speed target agree.
PLAN_DECEL = 0.45         # m/s^2
PROFILE_PATH = "/data/media/0/mazda_car_profile.json"
PROFILE_DECEL_RANGE = (0.15, 0.40)
PROFILE_DECEL_DEFAULT = 0.25   # car_profile.DECEL_DEFAULT: the staircase pace assumed until measured (logs only)
PLAN_LEAD_TIME = 1.0      # s at the curve's speed before its entry (exits; the car side's curve_target.MAP_LEAD_TIME)
# Round L (drive 6f: the ramp curves planned 1 s early at the curve's speed; by hand he is at it at the entry): ramp
# curves 0.5 s (the car side's curve_target.MAP_RAMP_LEAD_TIME), and the cruise's reaction counted at the car's own speed
# (the car side's reaction_speed for ramp curves: max(vEgo, the curve's speed), not his set speed)
PLAN_RAMP_LEAD_TIME = 0.5  # s
PLAN_ROAD_LEAD_TIME = 0.5  # s, road curves (the car side's curve_target.MAP_ROAD_LEAD_TIME, drive 54 West Utica)
PLAN_REACTION_TIME = 1.5  # s for the stock cruise to start slowing after a SET- press
# The driver's Curve Max Drop (MazdaCurveSlowdownMaxDrop, an index into MAX_DROP_CHOICES_MPH, read as the car side's
# curve_slowdown.py reads it): the car side never takes a road curve below the driver's speed less this, so the plan
# aims for that speed, not lower (drive 50: planning for 32 mph when the car would only go to 35 started the slowdown
# ~50 m earlier than the 35 needed). Whole 1 mph / 1 km/h steps, as CurveSlowdownLogic counts them.
MAX_DROP_CHOICES_MPH = (5, 10, 15)
DEFAULT_MAX_DROP_MPH = 10
ADVISORY_OVER = 10 * 0.44704   # m/s: a curve with a yellow advisory sign in the map is never planned faster than sign + this
AHEAD_COS = 0.2           # a point is ahead when it is within ~78 deg of our heading
NEAR = 15.0               # m
# On a ramp onto a highway a ramp curve stops governing this far before its end (owner, drives 56 / 59: "the moment we
# are almost at the end of the curve we should start increasing the speed"; the Rochester Rd loop held 27 mph until the
# loop's very end, then the merge was short): the cruise takes ~1-2 s to answer, the camera curve slowdown still has
# what is left of the curve
RAMP_HWY_RELEASE = 120.0  # m (drives 61 / 64: 60 was late, by hand he speeds up with ~180 m of the loop ramp left)
# On-ramps from a street (owner, drive 56: the Crooks Rd loop onto M-59, a ~27 mph curve 40 m into the ramp, entered from
# 46 mph with a hard brake; the Mound Rd ramp, navigation announced "on ramp" 400 m out, entered at 40 for a 29 mph
# curve): before the turn onto a ramp that leads to a highway, its first curve is planned like a ramp curve, when
#   * navigation's next maneuver is that ramp (type in ONRAMP_NAV_TYPES, matched at its start within NAV_MATCH_*), or
#   * his turn signal is on toward it and its start is at most ONRAMP_SIGNAL_RANGE ahead (no navigation needed).
# (drive 69 t 686-698: navigation's "turn left" onto Dequindre Rd 130 m ahead matched an M-59 on-ramp starting on that
# road past the junction, and its 57 m loop held the set speed at 27 on the street for 11 s: a turn, a merge or a fork
# is no ramp; only a maneuver that is the ramp itself counts)
ONRAMP_NAV_TYPES = ("on ramp", "ramp")
ONRAMP_NAV_RANGE = 600.0      # m
ONRAMP_SIGNAL_RANGE = 150.0   # m
ONRAMP_SIDE_SIN = 0.34        # the ramp leaves at least ~20 deg toward a side (its first curve's entry from its start): that side
# By his signal the ramp must leave the road we are on: its start at most this far sideways of our line of travel (an
# ordinary turn's signal at a junction found a ramp starting on the crossing road, past the junction)
ONRAMP_SIGNAL_LATERAL = 25.0  # m
# The decision is kept this long after it was last made (drive 69: the planner's answer went on and off every 0.5 s,
# the set speed's raise was blocked and let go with it); while kept, its distance is counted from the ramp's start
ONRAMP_KEEP = 2.0             # s
# Round L (drive 6f t 625-646, S Rochester Rd to M-59 E: navigation's primary maneuver was "turn" / slight right at 369 m,
# its "on ramp" came second in allManeuvers at the same distance; the on-ramp ahead was found only from his signal at ~160
# m, he braked at -3 m/s^2). Navigation names the ramp also when one of its first ONRAMP_NAV_ALL maneuvers is "on ramp" /
# "ramp" within ONRAMP_NAV_ALL_TOL of the primary's distance (that maneuver's distance is matched at the ramp's start as
# before), or when the primary is a ONRAMP_NAV_PRIMARY maneuver toward the ramp's side (strictly) and the ramp's start is
# within ONRAMP_NAV_ALL_TOL of its distance and on our road (at most ONRAMP_SIGNAL_LATERAL sideways: drive 69's turn onto
# the crossing road whose on-ramp started 40 m along it stays out)
ONRAMP_NAV_ALL = 3
ONRAMP_NAV_ALL_TOL = 50.0     # m
ONRAMP_NAV_PRIMARY = ("turn", "end of road", "continue", "fork")
APEX_PASSED = 10.0        # m: a road curve stops governing once its apex is this far behind (slow in, speed up from
                          # the apex), when the camera curve slowdown is on to hold the speed for what is left of it
# Ramp loops (drive 53, the on-ramp loop onto M-59: 71 m, 243 deg, 386 m long): "ahead" is a straight-line bearing test,
# so inside a loop its exit point is beside or behind the car for most of it, and the curve was lost 1 s after its
# entry (status 0 for 9 s in the loop, the camera's Max Drop floor alone held the set speed 11 mph above the car); after
# ~250 deg its own entry point came back "ahead" (173 m, a curve that was behind us), which kept the set speed at 29 for
# 4 s more at the end of the ramp. A ramp curve whose entry the car has reached is therefore "the one we are in" until
# the car has driven its length (the distance driven: the planner's speed x time), and never ahead again on that ramp.
TILE_KEEP = 25            # squares kept in memory
RESCAN_SECONDS = 120.0    # how often the curve_map folder is checked for new files (curve_mapd builds them while parked)
MATCH_DIST = 20.0         # m: the car is on a road shape within this distance (GPS error + road width)
MATCH_HEADING = 35.0      # deg: and heading along it (either direction) within this
MATCH_STAY = 12.0         # m: the car stays on the road it is on until that road is this far away (or turns away)
LINK_CLASSES = (2, 4, 6, 8, 10)   # ramps and other link roads (highwayClass)
MATCH_CELL = 500          # grid cells of 1/500 deg (~220 m) for the road shape index
NAV_EXIT_TYPES = ("off ramp", "fork")
# The car side's street speed (speed_limit_follow rules 8 / 9): navigation's posted limit (Mapbox) as a second opinion.
# Drive 55: 91 messages with 50 / 70 / 45 / 25 mph, 9 with 5 mph (junk) and 0. Fresh this long only.
NAV_LIMIT_MIN = 15 * 0.44704   # m/s (24 km/h): under this it is no limit
NAV_FRESH = 3.0                # s
NAV_TURN_TYPES = ("turn", "end of road", "roundabout", "rotary", "exit roundabout", "exit rotary")
NAV_EXIT_RANGE = 1500.0   # m: exits matched from this far
NAV_MATCH_TOL = 0.25      # match tolerance as a share of the announced distance (at least NAV_MATCH_MIN)
NAV_MATCH_MIN = 150.0
HIGHWAY_CLASSES = (1, 3)
RAMP_TYPES = {"exit": 1, "entrance": 2, "interchange": 3, "link": 4, "two-way": 4}

# frogpilotPlan.mapCurveStatus (the car side acts on 1 and 2 only: 4 and 5 are "no curve" on purpose)
STATUS_NONE, STATUS_CURVE, STATUS_RAMP_CURVE, STATUS_NO_DATA = 0, 1, 2, 3
STATUS_PAUSED, STATUS_HELD = 4, 5   # paused: the GPS is not trusted; held: a curve found, but the road match is not trusted

# GPS trust (GpsTrust). The device's GPS can be off by 100 m or more for a whole drive while reporting a fix (drive
# 3f: the track had the road's exact shape, 90-180 m to one side): then nothing matches, or worse, a parallel road
# matches. Safe by default: when the match is doubtful the map does not slow early; the camera curve slowdown is
# the safety net.
# starpilotPlan.mapGpsTrust
TRUST_OK, TRUST_SHAKY, TRUST_LOW, TRUST_PAUSED_ACCURACY, TRUST_PAUSED_LOST = 0, 1, 2, 3, 4
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
PASS_TURN_SHARE = 0.5      # a pass that turned less than this share of the curve's turn did not drive the curve
PASS_TURN_MIN = 10.0       # deg: ... checked for curves that turn at least this much
# The pass is the car's own driving from PASS_PRE before the curve's start to PASS_POST past its end, placed by the
# distance the car travelled (not by the moments it counted as inside the curve). Drive 44, Dobry Drive (map 131 m,
# 64 m long, 20 deg): "inside" starts once the start is NEAR behind and ends NEAR before the end, on a GPS position
# that moves once a second (~18 m at 40 mph) and lags, so only 2 s right after the tightest part were measured: 6-8
# deg of the 21 deg the car turned, and every pass was dropped as "did not turn" (drives 3e to 4c, the speed never
# learned). The end is placed from the distance to the curve's exit (the smallest estimate: the freshest GPS fix),
# less the distance driven in PASS_GPS_LAG (drive 44: the car's own turn was centred 0.6-0.9 s before the map curve's
# middle as placed from the GPS, 4-17 m on 7 passes).
PASS_GPS_LAG = 0.7         # s
PASS_PRE = 25.0            # m before the curve's start ...
PASS_POST = 25.0           # m ... and after its end (what is left of the GPS and map error)
PASS_DT = 0.05             # s per planner cycle (the pass functions assume it too)
PASS_MAX_DT = 0.2          # s: a longer pause between planner cycles starts the pass over
# a pass counts for its radius only with this share of it at PASS_MIN_SPEED or more (review of the window: a 40 m bend at
# 7 m/s with a 12 m/s approach left only the straight road around it, "2000 m")
PASS_FAST_SHARE = 0.9
# A pass that ends in a stop after the curve (drive 6a: West Utica's second curve, capped at 37 and driven at 1-2 m/s^2,
# with a red light after it): the braking / slow samples past the curve's end are cut off and the pass ends
# there, so it still counts (clean, its speed the curve's), as any pass through the curve's end would
PASS_STOP_CUT = True
PASS_BUFFER = 2400         # planner cycles of driving kept for the pass window (120 s)
# frogpilotPlan.mapNavExit
NAV_NONE, NAV_EXIT_CURVES, NAV_EXIT_NO_CURVES, NAV_EXIT_NO_DATA = 0, 1, 2, 3

# ---- navigation exits watched past the split (the owner's exit design) ----
# The exit navigation matched is watched until its start is WATCH_END behind: the car side's plan needs to know, also
# while the GPS match is doubtful at the split (drive 53: trust 2 from the split until the link was matched 137 m past it),
# whether the car is on the exit's link or still on the highway. The map's own shapes are compared with the car: the
# nearest highway road shape and the nearest shape of the exit's link (and of the branches of a fork on it) within
# SHAPE_RANGE, by distance and by heading. starpilotPlan.mapExitShape: 2 when we are closer to the link AND our heading is
# closer to the link's bearing, 1 the same for the highway, 0 neither (no answer: the car side keeps counting the
# distance driven). One frame's answer; the car side wants it sustained.
EXIT_SHAPE_NONE, EXIT_SHAPE_HIGHWAY, EXIT_SHAPE_LINK = 0, 1, 2
# Safety review of dff3e3310: in the lane next to the exit past the split the car is closer to the link's shape anyway,
# so a 1-4 deg heading drift toward the opening or the GPS heading's noise (2-4 deg) gave "link" on the highway. "Link"
# now needs our heading at least SHAPE_LINK_GAP closer to the link's than to the highway's (drive 53: 1.9-2.8 deg once on
# the ramps, 0.3-0.9 deg in their first second), and no answer below SHAPE_MIN_SPEED (the GPS heading is noise in a jam).
SHAPE_LINK_GAP = 1.0       # deg
SHAPE_MIN_SPEED = 10.0     # m/s
SHAPE_RANGE = 40.0         # m
SHAPE_HEADING = 60.0       # deg: a shape turned more than this from our heading is not the road we are on
# Past the exit link's start, closer to the link's shape but heading along the highway (drives 54 / 55: a car going on in
# the right lane is 7-8 m from the highway's center line and 1-6 m from the link's first shape, heading 0.1-0.2 deg off
# the highway's and ~5 deg off the link's; no answer for 2.5-5.5 s while the tentative slowdown ran): "highway" when the
# highway's heading is SHAPE_HWY_HEADING_GAP closer, within SHAPE_HWY_HEADING_MAX of ours, and its shape within
# SHAPE_HWY_NEAR (half a wide carriageway). Taken ramps (drive 53 E2, 26-54 m on: 2.3-2.9 deg closer, the highway's
# 1.3-1.6 deg off ours; drive 53 E1 and drives 54 / 55's two: 1.5 deg or less) never give it; 2 deg gave E2 1 s of it.
SHAPE_HWY_HEADING_GAP = 3.5  # deg
SHAPE_HWY_HEADING_MAX = 1.0  # deg
SHAPE_HWY_NEAR = 12.0        # m
WATCH_END = 600.0          # m behind its start (or this much more than navigation's range ahead): forgotten
WATCH_KEEP = 3             # exits watched at most (a reroute, the next maneuver)
WATCH_NEAR = 600.0         # m: the shape check is for the watched exit whose start is nearest, within this
# Forks on an exit link (a 2-lane exit that splits into a left and a right branch): another link starting on the exit's
# own link, FORK_MIN past its start. Until the car is seen on one branch (the shapes, as above, between the branches,
# sustained FORK_CONFIRM past the fork) the exit's curve is the lower of both branches'; after, only the taken branch's.
FORK_NEAR = 12.0           # m from the exit's link shape
FORK_MIN = 30.0            # m past the exit's start (a link from the split itself is another exit)
FORK_CONFIRM = 1.0         # s
FORK_SEEN = 15.0           # m: the fork's branches are told apart once the fork is this close or behind
# starpilotPlan.mapExitFork: 0 none, 1 not decided yet, 2 decided (navigation's branch, or navigation did not say), 3
# decided: the other branch than navigation's
FORK_NONE, FORK_OPEN, FORK_NAV, FORK_OTHER = 0, 1, 2, 3


def near_shapes(grid, lat, lon, bearing, chains):
  """{chain id: (m, deg)} the nearest road shape of each of `chains` within SHAPE_RANGE whose direction is within
  SHAPE_HEADING of our heading (either way along it unless one-way) and that difference; "hwy": the same for any highway
  (motorway / trunk) shape. bearing None: any direction (the difference is then 0)."""
  out = {}
  if not grid or lat is None:
    return out
  cla, clo = int(math.floor(lat * MATCH_CELL)), int(math.floor(lon * MATCH_CELL))
  kx = math.radians(1.0) * EARTH_R * math.cos(math.radians(lat))
  ky = math.radians(1.0) * EARTH_R
  for dla in (-1, 0, 1):
    for dlo in (-1, 0, 1):
      for la1, lo1, la2, lo2, _wid, cid, one, cls in (seg for ri in grid for seg in ri.segments(cell_key(cla + dla, clo + dlo))):
        key = cid if cid in chains else ("hwy" if cls in HIGHWAY_CLASSES else None)
        if key is None:
          continue
        ax, ay = (lo1 - lon) * kx, (la1 - lat) * ky
        bx, by = (lo2 - lon) * kx, (la2 - lat) * ky
        dx, dy = bx - ax, by - ay
        ll = dx * dx + dy * dy
        if ll < 1e-6:
          continue
        t = max(0.0, min(1.0, -(ax * dx + ay * dy) / ll))
        d = math.hypot(ax + t * dx, ay + t * dy)
        if d > SHAPE_RANGE:
          continue
        diff = 0.0
        if bearing is not None:
          diff = abs((math.degrees(math.atan2(dx, dy)) - bearing + 180.0) % 360.0 - 180.0)
          if not one:
            diff = min(diff, 180.0 - diff)
          if diff > SHAPE_HEADING:
            continue
        if key not in out or d < out[key][0]:
          out[key] = (d, diff)
  return out


# Where the highway and the exit link go ahead (owner: the driving model's predicted path bends away from the highway as
# soon as the ramp separates, independent of the GPS): the heading of each road's own shape BEND_AHEAD m ahead along it,
# relative to the highway's heading at the car (not our heading: the GPS heading's error does not enter). For the link
# that is its divergence plus its bend, for the highway its own bend. The car side compares the model path's heading that
# far ahead relative to our lane's (map_curve_control.ModelPath). Drive 53: E1's link diverges ~5 deg from its start,
# E2's ~4-6 deg and bends 6-13 deg 80-140 m past it.
BEND_AHEAD = 80.0          # m
BEND_STEP = 10.0           # m walked along the shape per step
BEND_RANGE = 160.0         # m: shapes this far from the car are collected (one scan of the 3 x 3 cells)


def road_bends(grid, lat, lon, bearing, chains):
  """(highway, link) heading BEND_AHEAD m ahead along each relative to the highway's at the car, deg right positive;
  None for one not found."""
  if not grid or lat is None:
    return None, None
  cla, clo = int(math.floor(lat * MATCH_CELL)), int(math.floor(lon * MATCH_CELL))
  kx = math.radians(1.0) * EARTH_R * math.cos(math.radians(lat))
  ky = math.radians(1.0) * EARTH_R
  segs = {"hwy": [], "link": []}
  hx, hy = math.sin(math.radians(bearing)), math.cos(math.radians(bearing))
  for dla in (-1, 0, 1):
    for dlo in (-1, 0, 1):
      for la1, lo1, la2, lo2, _wid, cid, one, cls in (seg for ri in grid for seg in ri.segments(cell_key(cla + dla, clo + dlo))):
        key = "link" if cid in chains else ("hwy" if cls in HIGHWAY_CLASSES else None)
        if key is None:
          continue
        ax, ay, bx, by = (lo1 - lon) * kx, (la1 - lat) * ky, (lo2 - lon) * kx, (la2 - lat) * ky
        dx, dy = bx - ax, by - ay
        ll = dx * dx + dy * dy
        t = max(0.0, min(1.0, -(ax * dx + ay * dy) / ll)) if ll > 1e-6 else 0.0
        if math.hypot(ax + t * dx, ay + t * dy) > BEND_RANGE:
          continue
        if (bx - ax) * hx + (by - ay) * hy < 0:
          if one:
            continue              # the other direction of a one-way road (the other carriageway)
          ax, ay, bx, by = bx, by, ax, ay
        segs[key].append((ax, ay, bx, by))
  hw = _walk(segs["hwy"], bearing, SHAPE_RANGE)
  if hw is None:
    return None, None
  lk = _walk(segs["link"], bearing, BEND_RANGE)
  h0 = hw[0]
  rel = lambda h: (h - h0 + 180.0) % 360.0 - 180.0  # noqa: E731
  return rel(hw[1]), (rel(lk[1]) if lk is not None else None)


def _nearest_on(segs, px, py, heading, max_turn):
  best = None
  for ax, ay, bx, by in segs:
    dx, dy = bx - ax, by - ay
    ll = dx * dx + dy * dy
    if ll < 1e-6:
      continue
    h = math.degrees(math.atan2(dx, dy))
    if abs((h - heading + 180.0) % 360.0 - 180.0) > max_turn:
      continue
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / ll))
    qx, qy = ax + t * dx, ay + t * dy
    d = math.hypot(qx - px, qy - py)
    if best is None or d < best[0]:
      best = (d, qx, qy, h)
  return best


def _walk(segs, bearing, reach):
  """(heading at the shape's point nearest the car, heading BEND_AHEAD m on along it), unwrapped; None when the shape is
  farther than `reach` or ends before."""
  if not segs:
    return None
  hit = _nearest_on(segs, 0.0, 0.0, bearing, SHAPE_HEADING)
  if hit is None or hit[0] > reach:
    return None
  _, x, y, h = hit
  h0, s = h, 0.0
  while s < BEND_AHEAD - 1e-6:
    x, y = x + math.sin(math.radians(h)) * BEND_STEP, y + math.cos(math.radians(h)) * BEND_STEP
    s += BEND_STEP
    hit = _nearest_on(segs, x, y, h, 45.0)
    if hit is None or hit[0] > BEND_STEP:
      return None                 # the shape ends (or leaves the collected range) before BEND_AHEAD
    _, x, y, h_new = hit
    h = h + ((h_new - h + 180.0) % 360.0 - 180.0)
  return h0, h


def shape_verdict(a, b, gap=0.0, b_heading=None):
  """1 when shape a is closer AND heads closer (by `gap` deg at least) to our heading than shape b, -1 when b is closer
  and heads closer, 0 neither (or unknown). b_heading (deg, m, deg): also -1 when b heads closer by the first, is within
  the second and heads within the third of ours, whatever the distances (the exit watch past the link's start:
  SHAPE_HWY_HEADING_GAP, SHAPE_HWY_NEAR, SHAPE_HWY_HEADING_MAX)."""
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


def nav_side(modifier):
  """Side of a navigation maneuver from its modifier ("slight right", "left", ...): -1 left, 1 right, 0 unknown."""
  m = (modifier or "").lower()
  return 1 if "right" in m else (-1 if "left" in m else 0)


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


def curve_speed_limit(radius, advisory=0.0, lat_limit=PLAN_LAT):
  """Speed (m/s) a curve allows: its shape at `lat_limit`, and at most its advisory sign + ADVISORY_OVER."""
  v = math.sqrt(lat_limit * radius)
  return min(v, advisory + ADVISORY_OVER) if advisory > 0 else v


def plan_speed(radius, distance, advisory=0.0, reaction_speed=0.0, lat_limit=PLAN_LAT, decel=PLAN_DECEL, floor=0.0,
               lead_time=PLAN_LEAD_TIME):
  """Highest speed (m/s) from which the stock cruise (reacting for PLAN_REACTION_TIME at `reaction_speed`, then
  slowing at `decel`) reaches the curve speed (never below `floor`, the driver's max drop) `lead_time` before its
  entry (the car side's map_curve_speed; PLAN_ROAD_LEAD_TIME for road curves)."""
  v_curve = max(curve_speed_limit(radius, advisory, lat_limit), floor)
  room = distance - v_curve * lead_time - PLAN_REACTION_TIME * max(reaction_speed, v_curve)
  return math.sqrt(v_curve ** 2 + 2 * decel * max(room, 0.0))


def max_drop_floor(driver_speed, max_drop_mph=DEFAULT_MAX_DROP_MPH, metric=False):
  """The lowest speed (m/s) the car side takes a road curve to: the driver's speed less the Curve Max Drop in whole
  steps (CurveSlowdownLogic.max_drop_steps); 0 when there is no driver's speed."""
  if driver_speed <= 0:
    return 0.0
  steps = int(round(max_drop_mph * 1.609344)) if metric else int(round(max_drop_mph))
  return max(driver_speed - steps * ((1 / 3.6) if metric else MPH), 0.0)


def read_profile_decel(path=PROFILE_PATH, default=PROFILE_DECEL_DEFAULT):
  """The car profile's measured cruise decel (clamped to PROFILE_DECEL_RANGE), else the default. Kept for the logs
  and the tests that check both sides read the file alike; the plan itself uses PLAN_DECEL (see there). A few KB;
  never raises. Mirrors car_profile.read_profile_decel (this module stays dependency free)."""
  try:
    with open(path) as f:
      use = json.load(f).get("decel", {}).get("use")
    v = float(use)
  except (OSError, ValueError, TypeError, AttributeError):
    return default
  if v != v:
    return default
  return min(max(v, PROFILE_DECEL_RANGE[0]), PROFILE_DECEL_RANGE[1])


def leave_caller_priority():
  """Background threads here inherit starpilot_process's real-time priority and core (SCHED_FIFO 51 on core 5, the
  same as plannerd and radard, which cannot preempt it: drive 3e). Give this thread normal priority and the other
  cores (as starpilot/common/feature_events.py does; a copy so this module stays dependency free)."""
  try:
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  except (AttributeError, OSError, PermissionError):
    pass
  try:
    cores = os.sched_getaffinity(0)
    others = set(range(os.cpu_count() or 1)) - cores
    if others:
      os.sched_setaffinity(0, others)
  except (AttributeError, OSError):
    pass
  try:
    os.nice(10)
  except OSError:
    pass


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
    self.events = []                       # (what, info) for the planner's log

  @property
  def status(self):
    if self.paused:
      return TRUST_PAUSED_LOST if self.reason == "lost" else TRUST_PAUSED_ACCURACY
    if self.low:
      return TRUST_LOW
    return TRUST_SHAKY if self.shaky else TRUST_OK

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
      self.steady_t = 0.0
    elif matched:
      self.steady_t += dt
    self.shaky = self.paused or self.low or self.steady_t < STEADY_TIME - TIMER_EPS

  def _pause(self, reason, acc, rel):
    self.paused, self.reason, self.good_t, self.steady_t = True, reason, 0.0, 0.0
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
  from an entrance ramp the car joins the freeway it merges into, not the next ramp). Taking an exit is therefore seen once the ramp is about MATCH_STAY away from the main road (typically
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
          if d.get("version", 0) < 3:
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


MEMORY_PATH = "/data/media/0/curve_memory.json"
MEMORY_MATCH = 20.0        # m between a curve's apex and a remembered one to be the same curve (curve files rebuilt)
MEMORY_SAME_PASS = 60.0    # s: passes this close in time are one pass (a divided road is two curve records ~20 m apart)
# A curve is remembered per direction of travel (drive 50, West Utica, map 91 m, ~35 mph both ways: the left turn one
# way kept up with the steering, 2.1 m/s^2 desired; the right turn the other way maxed the steering for 3.5 s at
# 2.5 m/s^2 desired, "Turn Exceeds Steering Limit". One shared record capped the right turn and the next clean left
# turn lifted the cap straight back). The direction is the bearing from the curve's entry to its exit as driven: it
# flips by 180 deg for the other way and barely moves when the curve files are rebuilt (apex matching's reason).
MEMORY_DIR_MATCH = 90.0    # deg: a record counts for a direction of travel within this of its own
MEMORY_DIR_MIN_CHORD = 5.0  # m: entry and exit closer than this (a full loop) give no direction (the record is shared)
MEMORY_MIN_PASSES = 2      # clean passes before the measured radius is used
MEMORY_AGREE = 1.35        # ... and the last ones must agree within this factor
MEMORY_KEEP = 6            # passes kept per curve
MEMORY_RANGE = (0.7, 5.0)  # measured radius used within this factor of the map's (a bad pass cannot do much)
MEMORY_MAX_R = 2000.0      # m: a pass that never really turned counts as this
# Ramp curves (round L, drive 6f: the exit to S Rochester Rd's left curve, map 85 m -> 33.3 mph at RAMP_LAT; GPS heading /
# steering measured 113-132 m at its tightest, he drove it at 40-45 mph and 2.8-3.2 m/s^2): the same store and record
# (keyed by the apex and direction), passes measured as for road curves (openpilot steering only), "full" ones only, none
# with a stop in the curve (PASS_MIN_SPEED with the brake) or the turn signal. Planned at max(map, RAMP_MEMORY_SHARE x the
# tightest pass), at most RAMP_MEMORY_MAX x the map's, from RAMP_MEMORY_MIN_PASSES pass on; never below the map's radius,
# and no speed cap from memory on a ramp this round. Still at RAMP_LAT on that radius.
# Review of round L (rl/s5.py: one wrong 200 m pass on an 85 m curve planned it at +40 %, 3.6 m/s^2 on the true radius,
# and a saturated or hand-steered pass afterwards changed nothing): RAMP_MEMORY_MIN_PASSES clean passes whose last two
# agree within MEMORY_AGREE, at most RAMP_MEMORY_MAX (1.15: ~3.0 m/s^2 on the map's radius at RAMP_LAT); only passes tagged
# "ramp" in records tagged "ramp" with a direction (a direction unknown: the map's radius); and a pass with the steering at
# its limit (more than PASS_SAT_BLIP), his steering or his brake in the curve clears the record's ramp passes (back to the
# map's radius).
RAMP_MEMORY_SHARE = 0.9
RAMP_MEMORY_MAX = 1.15
RAMP_MEMORY_MIN_PASSES = 2
PASS_MIN_SPEED = 8.0       # m/s
PASS_MIN_TIME = 2.0        # s of samples in the curve
PASS_SATURATED = 0.98      # |steer command|: the car could not follow (it goes wide): the pass does not count ...
# ... unless at the limit only this long in all (drive 44, Dobry Drive: one frame at 0.987 entering the curve at 40 mph,
# the car followed the curve fine)
PASS_SAT_BLIP = 0.1        # s
PASS_FILTER = 1.0          # s moving average of the measured curvature
# Steering at its limit: the Torque Interceptor could not steer this curve at that speed (drive 3d: the 30 mph West
# Utica curve maxed the steering at 37 mph on every pass, FrogPilot's too). Such a pass caps the curve's speed.
SAT_MIN_TIME = 1.0         # s with the steering command at PASS_SATURATED or more for a pass to count as maxed out
SAT_LAT_SHARE = 0.9        # share of the pass with openpilot steering
# Saturated samples with the driver's hands on the wheel do not count: the command also saturates when the driver
# steers against openpilot's path (another line through the curve), which says nothing about the curve.
SAT_FACTOR = 0.9           # a maxed-out pass caps the curve at this share of the speed the steering maxed out at
CAP_FLOOR = 25 * MPH       # never below this (the stock cruise's lowest set speed) ...
CAP_RISE = 0.5 * MPH       # a clean pass at (or within CAP_NEAR of) the cap raises it this much ...
CAP_NEAR = 2.5 * MPH       # (true speed: the dash reads ~2 mph above it, and the car side keeps a margin above the cap)
CAP_JUMP = 5 * MPH         # ... a clean pass well above the cap raises it to that pass's speed, at most this much
# A cap holds the car at the cap, so its passes are never "well above" it and it rose CAP_RISE a drive (drive 69: West
# Utica's second curve capped at 37 mph, driven at 38 with 1.1-1.3 m/s^2 on a ~250 m radius, 48 needed no slowdown):
# a full clean pass at or near the cap that measured a radius whose PLAN_LAT speed is CAP_LIFT_MARGIN or more above the
# cap raises it CAP_JUMP, every such pass (review: never removed outright). A pass with the steering maxed out caps the
# curve again whatever was lifted (add_saturated_pass).
CAP_LIFT_MARGIN = 5 * MPH
# One trusted gentle pass: a map curve much sharper than the road really is (a kink where two map ways join) is
# corrected after a single clean pass, when that pass was full, fast enough and right through the curve's apex.
MEMORY_ONE_PASS = 1.6      # the newest pass measured at least this many times the map's radius ...
PASS_TRUST_MIN_SPEED = 10.0  # m/s ... at this median speed or more ...
PASS_APEX_NEAR = 15.0      # m ... the car came this close to the curve's apex (else: another road, the pass counts for nothing)
PASS_EXIT_NEAR = 30.0      # m ... and the pass ended at the curve's exit (not by losing the road match)

# Camera check of map curves (CurveVeto): the driving model's path ahead is compared with the map curve's radius.
VETO_FACTOR = 2.0          # the camera sees the curve at least this many times gentler than planned ...
VETO_CONFIRM = 20          # ... on this many frames in a row where it can see the curve (1 s at 20 Hz): ignored
REARM_FACTOR = 1.3         # an ignored curve governs again when the camera sees it within this factor ...
REARM_CONFIRM = 10         # ... on this many seeing frames in a row (0.5 s)
VETO_MAX_FLIPS = 2         # after this many take-backs a curve is never ignored again this drive
VETO_FORGET = 40           # planner frames a curve may be missing (road match flicker) before its check starts over
VETO_CHORD = 30.0          # m: heading change over +-this along the path (the curve builder's HALF_WINDOW)
VETO_PAD = 20.0            # m added around the curve's span
VETO_STEP = 5.0            # m between curvature samples
VETO_SEE_MAX = 110.0       # m: the camera judges a curve only when its apex + VETO_PAD is this close ...
VETO_PATH_MARGIN = 10.0    # ... and this far inside the end of the model's path
VETO_YSTD_MAX = 1.5        # m: the path's sideways uncertainty at the apex
VETO_MIN_SPEED = 8.0       # m/s (at low speed the model's path is short)


def curve_pass_radius(samples, dt=0.05, max_r=MEMORY_MAX_R):
  """Radius (m) a clean pass through a curve measured: the tightest PASS_FILTER moving average of the car's own path
  curvature, or None when the pass does not count. samples: (v m/s, curvature 1/m, lat active, driver steering,
  |steer command|) per planner cycle while in the curve."""
  if not samples or len(samples) * dt < PASS_MIN_TIME:
    return None
  if any((not lat) or driver for _, _, lat, driver, _ in samples):
    return None
  if sum(1 for _, _, _, _, cmd in samples if cmd >= PASS_SATURATED) * dt > PASS_SAT_BLIP + 1e-9:
    return None
  ks = [abs(k) for v, k, _, _, _ in samples if v >= PASS_MIN_SPEED]
  if len(ks) < PASS_FAST_SHARE * len(samples):
    return None     # slow through (part of) the curve: the fast part left may be only the road before and after it
  n = max(int(round(PASS_FILTER / dt)), 1)
  if len(ks) < n:
    return None
  best = max(sum(ks[j:j + n]) / n for j in range(len(ks) - n + 1))
  return max_r if best <= 1.0 / max_r else min(1.0 / best, max_r)


def curve_pass_saturation(samples, dt=0.05):
  """Speed (m/s) at which the steering first stayed at its limit through the curve (SAT_MIN_TIME in all), or None.
  samples as for curve_pass_radius."""
  if not samples or len(samples) * dt < PASS_MIN_TIME:
    return None
  if sum(1 for _, _, lat, _, _ in samples if lat) < SAT_LAT_SHARE * len(samples):
    return None
  sat = [lat and not driver and cmd >= PASS_SATURATED and v >= PASS_MIN_SPEED for v, _, lat, driver, cmd in samples]
  if sum(sat) * dt < SAT_MIN_TIME:
    return None
  # the speed at the start of the longest run at the limit (a short blip on entry does not set the speed)
  best_len, best_start, run_start = 0, 0, None
  for k, s in enumerate(sat + [False]):
    if s and run_start is None:
      run_start = k
    elif not s and run_start is not None:
      if k - run_start > best_len:
        best_len, best_start = k - run_start, run_start
      run_start = None
  return samples[best_start][0]


def same_curve(a, b):
  """The same map curve (curve files rebuilt by a rescan give new dicts for it)."""
  return a.get("s_a") == b.get("s_a") and a.get("apex") == b.get("apex") and a.get("len") == b.get("len")


def driven_turn(samples, dt=0.05):
  """Degrees the car itself turned during a pass (signed sum of curvature x distance, so lane wiggles cancel)."""
  return abs(math.degrees(sum(k * v * dt for v, k, _, _, _ in samples))) if samples else 0.0


def curve_pass_speed(samples):
  """Median speed (m/s) of a pass."""
  vs = sorted(v for v, _, _, _, _ in samples)
  return vs[len(vs) // 2] if vs else 0.0


def curve_direction(c, increasing):
  """Direction of travel through curve dict `c` for the curve memory: the bearing (deg) from its entry to its exit as
  driven, or None when unknown. The curve files hold one record per curve for both ways (a -> b and b -> a;
  `increasing` says which, as MapCurves._curves_ahead sees it from the road match), so the record itself says
  nothing about the way; the bearing does, and it is the same from a rebuilt file (the entry and exit move a few m,
  a reversed chain swaps a and b and `increasing` together)."""
  try:
    if increasing is None or "a" not in c or "b" not in c:
      return None
    entry, exit_ = (c["a"], c["b"]) if increasing else (c["b"], c["a"])
    if dist_m(entry[0], entry[1], exit_[0], exit_[1]) < MEMORY_DIR_MIN_CHORD:
      return None
    return round(bearing_to(entry[0], entry[1], exit_[0], exit_[1]), 1)
  except (KeyError, TypeError, ValueError, IndexError):
    return None


def same_direction(a, b):
  """Directions of travel a and b (deg) agree: within MEMORY_DIR_MATCH (the other way is 180 deg off)."""
  return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0) <= MEMORY_DIR_MATCH


class CurveMemory:
  """Per map curve (by its apex position, so it survives rebuilt curve files) and direction of travel (curve_direction;
  the other way through the same curve is another record), kept in MEMORY_PATH (a small JSON list):
    * the radius the car actually drove: used instead of the map's once MEMORY_MIN_PASSES clean passes agree;
    * a speed cap ("cap", m/s) after a pass where the steering maxed out: SAT_FACTOR of that speed, never below the
      curve's yellow sign or CAP_FLOOR. Each clean pass near the cap raises it by CAP_RISE (searching back up).
  Records written before directions were kept (no "dir") hold passes of both ways: such a record's radius stands in
  for a direction only until a pass that way is recorded (which starts a record of its own, without the old passes),
  its cap a little longer (speed_cap); it is never written to again and stays in the file. `direction` None (the offline tests and tools: unknown) reads
  and writes the record without a direction, as before."""

  def __init__(self, path=MEMORY_PATH):
    self.path = path
    self.entries = []
    try:
      with open(path) as f:
        data = json.load(f)
      self.entries = [e for e in data.get("curves", []) if "lat" in e and "lon" in e]
    except (OSError, ValueError, AttributeError):
      self.entries = []
    self.saving = False
    self.dirty = False

  def _find(self, lat, lon, direction=None):
    """The record for the curve at (lat, lon) driven in `direction` (deg), or None: the nearest record within
    MEMORY_MATCH whose direction agrees (same_direction); with none, the nearest record without a direction.
    direction None: a record without a direction, else the nearest with one (the offline tools' view)."""
    best, best_d = None, MEMORY_MATCH
    undirected, undirected_d = None, MEMORY_MATCH
    for e in self.entries:
      try:
        d = dist_m(lat, lon, e["lat"], e["lon"])
        e_dir = e.get("dir")
        if d > MEMORY_MATCH:
          continue
        if e_dir is None:
          if d <= undirected_d:
            undirected, undirected_d = e, d
        elif (direction is None or same_direction(e_dir, direction)) and d <= best_d:
          best, best_d = e, d
      except (KeyError, TypeError, ValueError):
        continue
    if direction is None:
      return undirected if undirected is not None else best
    return best if best is not None else undirected

  def radius(self, c, direction=None):
    """Learned radius for curve dict `c` driven in `direction` (curve_direction), or None: the middle of the last
    passes when MEMORY_MIN_PASSES or more agree; else the newest pass alone when it was a trusted gentle pass
    (one_pass_ok). A sharper newest pass therefore goes back to the map's radius by itself, and a later gentle pass
    earns it again."""
    try:
      e = self._find(*c["apex"], direction)
      if e is None:
        return None
      passes = [p for p in e.get("passes", []) if isinstance(p, list) and len(p) >= 2]
      lo, hi = MEMORY_RANGE
      if len(passes) >= MEMORY_MIN_PASSES:
        last = sorted(float(p[0]) for p in passes[-3:])
        if last[-1] <= last[0] * MEMORY_AGREE:
          return min(max(last[len(last) // 2], c["r"] * lo), c["r"] * hi)
      if passes and self.one_pass_ok(c, passes[-1]):
        return min(max(float(passes[-1][0]), c["r"] * lo), c["r"] * hi)
      return None
    except (KeyError, TypeError, IndexError, ValueError):
      return None

  def ramp_radius(self, c, direction=None):
    """The map radius (not scaled for RAMP_LAT) to plan ramp curve `c` with from its passes (RAMP_MEMORY_*), or None
    (also when the direction of travel is unknown)."""
    try:
      if direction is None:
        return None
      e = self._find(*c["apex"], direction)
      if e is None or not e.get("ramp") or e.get("dir") is None:
        return None
      passes = [float(p[0]) for p in e.get("passes", []) if isinstance(p, list) and len(p) >= 4 and
                "full" in (p[3] or []) and "ramp" in (p[3] or []) and 0 < float(p[0]) < MEMORY_MAX_R]
      if len(passes) < RAMP_MEMORY_MIN_PASSES:
        return None
      a, b = passes[-2], passes[-1]
      if max(a, b) > MEMORY_AGREE * min(a, b):
        return None
      r_map = float(c["r"])
      return min(max(r_map, RAMP_MEMORY_SHARE * min(a, b)), RAMP_MEMORY_MAX * r_map)
    except (KeyError, TypeError, IndexError, ValueError):
      return None

  def add_ramp_pass(self, c, r, direction, t=None, v=None, flags=None):
    """A clean pass through ramp curve `c` (tagged "ramp", the record too); nothing without a direction."""
    if direction is None:
      return
    self.add_pass(c, r, t=t, v=v, flags=sorted(set(flags or []) | {"ramp"}), direction=direction)
    e = self._find(*c["apex"], direction)
    if e is not None and e.get("dir") is not None:
      e["ramp"] = True

  def clear_ramp(self, c, direction=None):
    """A ramp pass that went wrong (the steering at its limit, his steering or brake): the ramp passes of the record
    driven in `direction` are dropped (every ramp record at the curve when it is unknown): the map's radius again."""
    try:
      hit = False
      for e in self.entries:
        if not e.get("ramp") or dist_m(c["apex"][0], c["apex"][1], e["lat"], e["lon"]) > MEMORY_MATCH:
          continue
        if direction is not None and e.get("dir") is not None and not same_direction(e["dir"], direction):
          continue
        e["passes"] = [p for p in e.get("passes", []) if not (isinstance(p, list) and len(p) >= 4 and "ramp" in (p[3] or []))]
        hit = True
      if hit:
        self.save()
    except (KeyError, TypeError, ValueError, IndexError):
      pass

  @staticmethod
  def one_pass_ok(c, p):
    """Pass p ([r, t, v, flags]) alone shows the curve is much gentler than the map says: a full pass through the
    apex, fast enough, MEMORY_ONE_PASS x the map's radius or more, and a real turn (MEMORY_MAX_R means it never
    turned: another road, not a gentle curve). Old 2-value passes never qualify."""
    try:
      if len(p) < 4 or p[2] is None or "full" not in (p[3] or []):
        return False
      r = float(p[0])
      return float(p[2]) >= PASS_TRUST_MIN_SPEED and MEMORY_ONE_PASS * float(c["r"]) <= r < MEMORY_MAX_R
    except (TypeError, ValueError, KeyError):
      return False

  def speed_cap(self, c, direction=None):
    """Speed cap (m/s) for curve dict `c` driven in `direction` from passes where the steering maxed out, or None.
    A cap only ever makes a curve slower, so a record without a direction keeps capping a direction (safe side) until
    that direction's own record has a cap of its own or MEMORY_MIN_PASSES clean passes; then it decides alone."""
    try:
      e = self._find(*c["apex"], direction)
      if e is None:
        return None
      cap = e.get("cap")
      if not cap and direction is not None and e.get("dir") is not None and \
         len([p for p in e.get("passes", []) if isinstance(p, list) and len(p) >= 2]) < MEMORY_MIN_PASSES:
        legacy = self._find(*c["apex"])          # the record without a direction, if any
        cap = None if legacy is None or legacy.get("dir") is not None else legacy.get("cap")
      return float(cap) if cap else None
    except (KeyError, TypeError, ValueError):
      return None

  def _entry(self, c, direction=None):
    """The record to write a pass through `c` in `direction` to: made when there is none, also when only a record
    without a direction stands in (its passes mix both ways: it is never written to again)."""
    e = self._find(*c["apex"], direction)
    if e is None or (direction is None) != (e.get("dir") is None):
      e = {"lat": round(c["apex"][0], 6), "lon": round(c["apex"][1], 6), "r_map": c.get("r"),
           "name": c.get("name", ""), "passes": []}
      if direction is not None:
        e["dir"] = round(float(direction), 1)
      self.entries.append(e)
    return e

  def add_saturated_pass(self, c, v, t=None, direction=None):
    """A pass through curve `c` in `direction` where the steering maxed out at speed v (m/s): cap the curve's speed."""
    t = time.time() if t is None else t   # noqa: TID251  wall time, as add_pass
    e = self._entry(c, direction)
    if abs(t - float(e.get("cap_t", -1e12))) < MEMORY_SAME_PASS:
      return                                  # the same pass again (the other carriageway's record)
    floor = max(CAP_FLOOR, float(c.get("adv", 0.0) or 0.0))
    old = float(e["cap"]) if e.get("cap") else float("inf")
    e["cap"] = round(max(floor, min(old, SAT_FACTOR * v)), 2)
    e["cap_t"] = round(t, 1)
    e["cap_hits"] = int(e.get("cap_hits", 0)) + 1
    e.pop("cap_lift", None)                   # (the lifts are counted again from here)
    self.save()

  def add_pass(self, c, r, t=None, v=None, flags=None, direction=None):
    """A clean pass through curve `c` in `direction` that measured radius r (m), at wall time t (s) and median speed
    v (m/s). flags: "full" (through the apex to the exit), "vetoed" (the camera check ignored the curve: audit only)."""
    t = time.time() if t is None else t   # noqa: TID251  wall time: passes are compared across drives and reboots
    e = self._entry(c, direction)
    if v is not None and e.get("cap") and v >= float(e["cap"]) - CAP_NEAR and \
       abs(t - float(e.get("cap_t", -1e12))) >= MEMORY_SAME_PASS:
      # it took the curve cleanly near the cap (try a bit faster), or well above it (up to that speed)
      cap = float(e["cap"])
      new_cap = max(cap + CAP_RISE, min(v, cap + CAP_JUMP))
      if "full" in (flags or []) and math.sqrt(PLAN_LAT * max(float(r), 0.0)) >= cap + CAP_LIFT_MARGIN:
        # (CAP_LIFT_MARGIN) the curve measured much gentler than the cap: up CAP_JUMP (counted for the record)
        e["cap_lift"] = int(e.get("cap_lift", 0)) + 1
        new_cap = cap + CAP_JUMP
      e["cap"] = round(new_cap, 2)
      e["cap_t"] = round(t, 1)
    passes = [p for p in e["passes"] if isinstance(p, list) and len(p) >= 2]
    if passes and abs(t - passes[-1][1]) < MEMORY_SAME_PASS:
      passes = passes[:-1]                    # the same pass again (the other carriageway's record): keep the newest
    new = [round(float(r), 1), round(t, 1), None if v is None else round(float(v), 2), sorted(set(flags or []))]
    e["passes"] = (passes + [new])[-MEMORY_KEEP:]
    self.save()

  def note_veto(self, c, direction=None):
    """The camera check ignored curve `c` driven in `direction` (counted for the record only, never read)."""
    e = self._entry(c, direction)
    e["veto_hits"] = int(e.get("veto_hits", 0)) + 1
    self.save()

  def save(self):
    """Write the file in the background; a pass added while writing is written right after."""
    self.dirty = True
    if self.saving:
      return
    self.saving = True

    def write():
      leave_caller_priority()
      try:
        while self.dirty:
          self.dirty = False
          # version 2: records carry "dir" (deg); older code never reads the version and matches by position alone
          data = {"version": 2, "curves": [dict(e) for e in self.entries]}
          tmp = self.path + ".tmp"
          with open(tmp, "w") as f:
            json.dump(data, f)
          os.replace(tmp, self.path)
      except OSError:
        pass
      finally:
        self.saving = False
    threading.Thread(target=write, daemon=True).start()


class CameraView:
  """What the driving model sees ahead in one frame (camera_view() builds it from modelV2, radarState, carState):
  per path point s (m along the path), psi (heading, rad, relative to the car) and ystd (m, sideways uncertainty);
  red (model confidence red), lane_change, blinker, lead_d (m to the lead, None without one), v_ego (m/s)."""

  def __init__(self, s, psi, ystd, red=False, lane_change=False, blinker=False, lead_d=None, v_ego=0.0):
    self.s, self.psi, self.ystd = list(s), list(psi), list(ystd)
    self.red, self.lane_change, self.blinker = bool(red), bool(lane_change), bool(blinker)
    self.lead_d, self.v_ego = lead_d, float(v_ego)

  @property
  def end(self):
    return self.s[-1] if self.s else 0.0

  def _at(self, values, x):
    s = self.s
    if x <= s[0]:
      return values[0]
    for i in range(1, len(s)):
      if x <= s[i]:
        w = (x - s[i - 1]) / max(s[i] - s[i - 1], 1e-6)
        return values[i - 1] + w * (values[i] - values[i - 1])
    return values[-1]

  def radius(self, lo, hi):
    """Tightest radius (m) of the path between lo and hi m along it: heading change over +-VETO_CHORD (shorter at
    the ends of the path, never under VETO_CHORD in all). inf when straight."""
    end = self.end
    hi = min(hi, end)
    worst = 0.0
    x = max(lo, 0.0)
    while x <= hi + 1e-6:
      a, b = max(x - VETO_CHORD, 0.0), min(x + VETO_CHORD, end)
      if b - a >= VETO_CHORD:
        worst = max(worst, abs(self._at(self.psi, b) - self._at(self.psi, a)) / (b - a))
      x += VETO_STEP
    return 1.0 / worst if worst > 1e-6 else float("inf")

  def blind(self, s_apex):
    """Why the camera cannot judge a curve whose apex is s_apex m ahead now (None: it can)."""
    if self.v_ego < VETO_MIN_SPEED:
      return "slow"
    if self.red:
      return "red"
    if self.lane_change or self.blinker:
      return "signal"
    if len(self.s) < 2 or s_apex + VETO_PAD > min(VETO_SEE_MAX, self.end - VETO_PATH_MARGIN):
      return "far"
    if self._at(self.ystd, s_apex) > VETO_YSTD_MAX:
      return "ystd"
    if self.lead_d is not None and self.lead_d < s_apex + VETO_PAD:
      return "lead"
    return None


def _raw(v):
  return int(getattr(v, "raw", v))


def camera_view(sm):
  """CameraView of this planner frame, or None when the model is not there (never raises)."""
  try:
    if not (sm.alive["modelV2"] and sm.valid["modelV2"]):
      return None
    md = sm["modelV2"]
    xs, ys = list(md.position.x), list(md.position.y)
    n = len(xs)
    if n < 2 or len(ys) != n:
      return None
    s = [0.0]
    for i in range(1, n):
      s.append(s[-1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]))
    psi = list(md.orientation.z)
    if len(psi) != n:
      psi = [0.0] + [math.atan2(ys[i] - ys[i - 1], xs[i] - xs[i - 1]) for i in range(1, n)]
    ystd = list(md.position.yStd)
    if len(ystd) != n:
      ystd = [float("inf")] * n            # no uncertainty from the model: never judge
    cs = sm["carState"]
    lead = sm["radarState"].leadOne
    lead_d = float(lead.dRel) if lead.status else None
    return CameraView(s, psi, ystd, red=_raw(md.confidence) == 0, lane_change=_raw(md.meta.laneChangeState) != 0,
                      blinker=bool(cs.leftBlinker or cs.rightBlinker), lead_d=lead_d, v_ego=float(cs.vEgo))
  except Exception:
    return None


class CurveVeto:
  """Camera check of map curves. A map curve the driving model sees as at least VETO_FACTOR x gentler, on
  VETO_CONFIRM frames in a row where it can see the curve's apex well (CameraView.blind), stops governing: the
  next curve governs, or none (the speed comes back as for any curve). While ignored it is still watched: seen
  sharp again (REARM_FACTOR) it governs again; after VETO_MAX_FLIPS take-backs it is never ignored again this drive.
  Never ignored: ramp curves, curves with a yellow sign, curves the curve memory knows (a cap or a learned radius:
  the car has driven them). Frames where the camera cannot judge leave the counters as they are.
  The car side only sees the curve gone; the camera curve slowdown keeps running as the safety net.
  Honest limit: the map slows early (~290 m before the curve at 45 mph), the camera judges at ~110 m, so this saves
  about the second half of a false slowdown; the curve memory then corrects the curve after one clean pass."""

  def __init__(self):
    self.reset_drive()

  def reset_drive(self):
    self.state = {}           # (chain, s_a) -> {"vetoed", "n", "flips", "r_cam", "seen"}
    self.given_up = set()     # keys never ignored again this drive
    self.frame = 0
    self.events = []          # for the planner's log (it empties the list)
    self.hidden = None        # (key, r_cam) of an ignored curve this frame, or None

  def tick(self):
    self.frame += 1
    self.hidden = None
    for key in [k for k, st in self.state.items() if self.frame - st["seen"] > VETO_FORGET]:
      del self.state[key]

  def is_vetoed(self, key):
    st = self.state.get(key)
    return bool(st and st["vetoed"])

  def flips(self, key):
    st = self.state.get(key)
    return st["flips"] if st else (VETO_MAX_FLIPS if key in self.given_up else 0)

  @staticmethod
  def vetoable(mc, c, shaky=False):
    """shaky (GpsTrust): the road match may be wrong, so a ramp, signed or remembered curve may be on another road:
    the camera may end the slowdown for any curve then."""
    if "apex" not in c:
      return False
    if shaky:
      return True
    if c.get("ramp") or float(c.get("adv", 0.0) or 0.0) > 0:
      return False
    if mc.memory is not None:
      d = mc.direction(c)
      if mc.memory.speed_cap(c, d) is not None or mc.memory.radius(c, d) is not None:
        return False
    return True

  def filter(self, mc, cid, cands, camera, shaky=False):
    """cands: [(curve, m to its entry)] -> the same without the ignored curves."""
    out = []
    for c, d in cands:
      key = (cid, c["s_a"])
      if key in self.given_up or not self.vetoable(mc, c, shaky):
        self.state.pop(key, None)
        out.append((c, d))
        continue
      st = self.state.get(key)
      if st is None:
        st = self.state[key] = {"vetoed": False, "n": 0, "flips": 0, "r_cam": 0.0}
      st["seen"] = self.frame
      self._check(mc, key, st, c, d, camera)
      if st["vetoed"]:
        if self.hidden is None:
          self.hidden = (key, st["r_cam"])
      else:
        out.append((c, d))
    return out

  def _check(self, mc, key, st, c, d_entry, camera):
    g = mc.geom.get(id(c))
    if camera is None or g is None:
      return
    d_apex, d_exit = g
    s_apex = max(d_apex, 0.0)               # behind the apex: what is left of the curve
    if camera.blind(s_apex) is not None:
      return
    r_eff = mc._radius(c)
    s_exit = d_exit if d_entry <= 0.0 else d_entry + float(c.get("len", 0.0))
    lo = max(d_entry - VETO_PAD, 0.0)
    hi = min(s_exit + VETO_PAD, VETO_SEE_MAX, camera.end - VETO_PATH_MARGIN)
    r_cam = camera.radius(lo, hi)
    st["r_cam"] = r_cam
    if not st["vetoed"]:
      st["n"] = st["n"] + 1 if r_cam >= VETO_FACTOR * r_eff else 0
      if st["n"] >= VETO_CONFIRM:
        st["vetoed"], st["n"] = True, 0
        self._event("veto", c, key, st, d_entry, d_apex, r_eff, camera)
    else:
      st["n"] = st["n"] + 1 if r_cam <= REARM_FACTOR * r_eff else 0
      if st["n"] >= REARM_CONFIRM:
        st["vetoed"], st["n"] = False, 0
        st["flips"] += 1
        self._event("rearm", c, key, st, d_entry, d_apex, r_eff, camera)
        if st["flips"] >= VETO_MAX_FLIPS:
          self.given_up.add(key)
          self._event("give_up", c, key, st, d_entry, d_apex, r_eff, camera)

  def _event(self, what, c, key, st, d_entry, d_apex, r_eff, camera):
    r_cam = st["r_cam"]
    self.events.append({
      "what": what, "curve": c, "name": c.get("name", "") or "", "chain": key[0], "s_a": key[1],
      "r_map": float(c.get("r", 0.0)), "r_eff": round(r_eff, 1), "turn": float(c.get("turn", 0.0) or 0.0),
      "len": float(c.get("len", 0.0) or 0.0), "d_entry": round(d_entry, 1), "d_apex": round(d_apex, 1),
      "r_cam": round(r_cam, 1) if r_cam != float("inf") else -1.0,
      "ystd_apex": round(camera._at(camera.ystd, max(d_apex, 0.0)), 2), "lead_d": camera.lead_d,
      "v_ego": round(camera.v_ego, 2), "frames": VETO_CONFIRM if what == "veto" else REARM_CONFIRM, "flips": st["flips"]})


class MapCurves:
  """The governing curve ahead, the ramp we are on, and the matched navigation exit."""

  def __init__(self, store=None, release_after_apex=False, memory=None, decel=PLAN_DECEL):
    self.store = store if store is not None else MapCurveStore()
    self.release_after_apex = release_after_apex
    self.memory = memory
    self.decel = decel           # the cruise's slowdown for the plan (PLAN_DECEL)
    self.floor = 0.0             # m/s: the driver's max drop floor this cycle (max_drop_floor; the car side plans with it too)
    self.veto = CurveVeto()      # camera check (acts only when update() gets a camera view)
    self.odo = 0.0               # m driven (update()'s gps speed x time)
    self.odo_ok = False          # update() has been given the speed (the planner always; offline callers may not)
    self.entered = {}            # ramp curve s_a -> odo at its entry, on the ramp chain `entered_chain`
    self.entered_chain = 0
    self.watches = []            # navigation exits watched past the split (_watch_add), newest last
    self.onramp_keep = None      # [s left, curve, ramp start (lat, lon), m from the start to the curve] (ONRAMP_KEEP)
    self.nav_all = []            # navigation's first maneuvers (ONRAMP_NAV_ALL)
    self.reset()

  def reset(self):
    self.status = STATUS_NONE
    self.curve = None            # governing curve dict
    self.distance = 0.0          # m to its entry (0 = in it)
    self.radius = 0.0
    self.advisory = 0.0
    self.on_ramp = False
    self.ramp = None
    self.onramp_ahead = False      # the governing curve is an on-ramp's first curve, before the turn onto it
    self.ramp_end_dist = 0.0
    self.nav_exit = NAV_NONE
    self.nav_exit_dist = 0.0
    self.nav_ramp = None
    self.nav_curve = None
    self.nav_curve_dist = 0.0
    self.nav_side = 0
    self.data_here = False
    self.chain_id = 0
    self.increasing = None       # driving the matched road chain towards higher s (curve memory: the direction of travel)
    self.inside = None           # the road curve we are driving through (also after its apex), for curve memory
    self.inside_dir = None       # ... and our direction of travel through it (curve_direction), kept with the pass
    self.inside_exit = float("inf")  # m to its exit
    self.next_entry = float("inf")   # m to the start of the nearest road curve ahead (curve memory: where a pass ends) ...
    self.next_curve = None           # ... and that curve
    self.geom = {}               # id(curve) -> (m to its apex, negative when behind; m to its exit) this frame
    self.veto_status = 0         # starpilotPlan.mapCurveVeto
    self.veto_radius = 0.0
    self.source = 0              # starpilotPlan.mapCurveSource
    self.paused = False          # GpsTrust paused: nothing planned from the map
    self.held = None             # (curve, distance) found but not acted on: the road match is not trusted
    self.trust_status = TRUST_OK # starpilotPlan.mapGpsTrust
    self.watch = None            # the watched exit the mapExit* fields are about this frame
    self.exit_shape = EXIT_SHAPE_NONE
    self.exit_link = (0.0, 0.0)  # (m, deg) to the exit link's nearest shape (0, 0 none)
    self.exit_hwy = (0.0, 0.0)   # ... the highway's
    self.exit_fork = FORK_NONE
    self.exit_curve = None       # the watched exit's next curve (forks decided) and the m to its entry
    self.exit_curve_dist = 0.0
    self.exit_start = 0.0        # m to the watched exit's link start (negative behind), 0 unknown
    self.exit_bends = (None, None)  # (highway, link) bends ahead, deg right positive (road_bends)

  # ---- the road ahead ----
  def _road(self, way_id, is_forward):
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

  def _curves_ahead(self, cid, increasing, s0, s1, lat, lon, bearing):
    out = []
    inside_exit = float("inf")
    for c in self.store.curves.get(cid, []):
      if self.odo_ok and c.get("ramp") and c["s_a"] in self.entered:
        left = c["len"] - (self.odo - self.entered[c["s_a"]])
        if 0.0 < left < inside_exit:
          # (RAMP_MEMORY_*) the ramp curve we are in, for its pass
          self.inside, inside_exit = c, left
          self.inside_exit, self.inside_dir = left, curve_direction(c, increasing)
        r_here = self.store.ramps.get(cid)          # (on_ramp is only known after this, each update)
        to_hwy = bool(r_here is not None and ((r_here.get("to") or {}).get("cls") in HIGHWAY_CLASSES))
        if left > (RAMP_HWY_RELEASE if to_hwy else 0.0):
          out.append((c, 0.0))    # in it by the distance driven since its entry (a loop's exit is not "ahead")
        continue                  # entered once: never ahead again on this ramp (a loop's own entry comes back ahead)
      entry, exit_ = (c["a"], c["b"]) if increasing else (c["b"], c["a"])
      s_entry, s_exit = (c["s_a"], c["s_b"]) if increasing else (c["s_b"], c["s_a"])
      if increasing and (s_exit < s0 or s_entry > s1 + LOOKAHEAD):
        continue
      if not increasing and (s_exit > s1 or s_entry < s0 - LOOKAHEAD):
        continue
      on_this_way = (s0 <= s_entry <= s1)
      ent_ahead, d_entry = ahead(lat, lon, bearing, *entry)
      ex_ahead, d_exit = ahead(lat, lon, bearing, *exit_)
      if "apex" in c:
        ap_ok, ap_d = ahead(lat, lon, bearing, *c["apex"])
        self.geom[id(c)] = (ap_d if ap_ok else -ap_d, d_exit)
      later_way = (s_entry > s1) if increasing else (s_entry < s0)
      if later_way or (on_this_way and ent_ahead):
        if d_entry <= LOOKAHEAD:
          out.append((c, d_entry))
          if self.odo_ok and c.get("ramp") and on_this_way and d_entry <= NEAR:
            self.entered[c["s_a"]] = self.odo + d_entry
          if not c.get("ramp") and d_entry < self.next_entry:
            self.next_entry, self.next_curve = d_entry, c
      elif ex_ahead and d_exit > NEAR and d_exit <= c["len"] + 60:
        if d_exit < inside_exit:
          # the curve we are in is the one we leave first (ramp curves too: RAMP_MEMORY_*) (the next one may already match up to 60 m before it)
          self.inside, inside_exit = c, d_exit
          self.inside_exit = d_exit
          self.inside_dir = curve_direction(c, increasing)
        if self.release_after_apex and not c.get("ramp") and "apex" in c:
          ap_ahead, d_apex = ahead(lat, lon, bearing, *c["apex"])
          if not ap_ahead and d_apex > APEX_PASSED:
            continue              # past the apex: the next curve (if any) governs, or none (the speed comes back)
        out.append((c, 0.0))      # in the curve
        if self.odo_ok and c.get("ramp"):
          self.entered[c["s_a"]] = self.odo
    return out

  def direction(self, c):
    """Our direction of travel through curve `c` on the matched road (curve_direction), for the curve memory: the same
    quantity MapCurvesPlanner records a pass under (inside_dir), so what was learned one way is planned that way."""
    return curve_direction(c, self.increasing)

  def _radius(self, c):
    """The curve's radius: what the car measured on clean passes (curve memory) if known, else the map's; made
    tighter when the memory caps the curve's speed (steering maxed out there), so that the curve's speed at PLAN_LAT
    is the cap. The car side plans at PLAN_LAT or less, so it never goes faster than the cap."""
    if c.get("ramp"):
      return self._ramp_r(c, self.direction(c))
    r = ramp_radius(c)
    if self.memory is not None:
      d = self.direction(c)
      learned = self.memory.radius(c, d)
      if learned is not None:
        r = learned
      cap = self.memory.speed_cap(c, d)
      if cap is not None:
        r = min(r, cap * cap / PLAN_LAT)
    return r

  def _ramp_r(self, c, direction=None):
    """A ramp curve's radius to plan with: the map's, or from its passes (RAMP_MEMORY_*), scaled for RAMP_LAT.
    direction None (unknown, or a curve on another road than the matched one: an on-ramp ahead): the map's."""
    learned = self.memory.ramp_radius(c, direction) if self.memory is not None else None
    if learned is None:
      return ramp_radius(c)
    return learned * RAMP_LAT / PLAN_LAT

  def _source(self, c):
    """Where the curve's radius comes from: 0 the map, 1 curve memory (driven passes), 2 a memory speed cap."""
    if self.memory is None:
      return 0
    if c.get("ramp"):
      return 1 if self.memory.ramp_radius(c, self.direction(c)) is not None else 0
    d = self.direction(c)
    learned = self.memory.radius(c, d)
    cap = self.memory.speed_cap(c, d)
    r = learned if learned is not None else c["r"]
    if cap is not None and cap * cap / PLAN_LAT < r:
      return 2
    return 1 if learned is not None else 0

  def update(self, lat, lon, bearing, way_id, is_forward, nav=None, camera=None, trust=None, gps=None, nav_all=None):
    """nav: (maneuver type, modifier, distance m) of the next maneuver, or None. camera: CameraView or None
    (no camera check). trust: GpsTrust (None: the position is trusted, as in the offline tests), fed here with
    gps = {"dt", "v_ego", "acc", "rel", "ambiguous"}. nav_all: navigation's first maneuvers [(type, modifier, m)]
    (allManeuvers, ONRAMP_NAV_ALL; the on-ramp ahead only)."""
    self.nav_all = list(nav_all or [])[:ONRAMP_NAV_ALL]
    self.reset()
    self.veto.tick()
    if gps is not None:
      self.odo += max(float(gps.get("dt", 0.05)), 0.0) * max(float(gps.get("v_ego", 0.0)), 0.0)
      self.odo_ok = True
    if lat is None:
      return
    self.store.update(lat, lon)
    self.data_here = self.store.covers(lat, lon)
    shaky = False
    if trust is not None:
      g = gps or {}
      trust.update(float(g.get("dt", 0.05)), float(g.get("v_ego", 0.0)), float(g.get("acc", 0.0)), int(g.get("rel", 0)),
                   self.data_here, bool(way_id), bool(g.get("ambiguous", False)))
      self.trust_status = trust.status
      if trust.paused:
        self.paused = True
        return                   # no curve, ramp or exit: all of them come from the position
      shaky = trust.shaky
    # the watched exit (also while the match is doubtful: the shapes themselves tell the link from the highway)
    self._watch_update(lat, lon, bearing, max(float((gps or {}).get("dt", 0.05)), 0.0),
                       float(gps["v_ego"]) if gps is not None and "v_ego" in gps else None)
    road = self._road(way_id, is_forward) if way_id else None
    if road is not None:
      cid, increasing, s0, s1 = road
      self.chain_id, self.increasing = cid, increasing
      if cid != self.entered_chain:
        self.entered, self.entered_chain = {}, cid      # another road: its ramp curves are new
      cands = self._curves_ahead(cid, increasing, s0, s1, lat, lon, bearing)
      if self.watches:
        cands = self._fork_filter(cid, cands)
      if cands:
        cands = self.veto.filter(self, cid, cands, camera, shaky)
      if cands:
        c, d = min(cands, key=lambda cd: plan_speed(self._radius(cd[0]), cd[1], cd[0].get("adv", 0.0), decel=self.decel,
                                                    floor=0.0 if cd[0].get("ramp") else self.floor,
                                                    lead_time=PLAN_RAMP_LEAD_TIME if cd[0].get("ramp") else PLAN_ROAD_LEAD_TIME))
        if trust is not None and trust.low:
          self.held = (c, d)     # another road this close may be the one we are on: do not slow early
          self.status = STATUS_HELD
        else:
          self.curve, self.distance, self.radius = c, d, self._radius(c)
          self.advisory = float(c.get("adv", 0.0) or 0.0)
          self.status = STATUS_RAMP_CURVE if c.get("ramp") else STATUS_CURVE
          self.source = self._source(c)
      if self.veto.hidden is not None:
        self.veto_status = 1
        r_cam = self.veto.hidden[1]
        self.veto_radius = r_cam if r_cam != float("inf") else MEMORY_MAX_R
      elif self.curve is not None and self.veto.flips((cid, self.curve["s_a"])) > 0:
        self.veto_status = 2
      ramp = self.store.ramps.get(cid)
      if ramp is not None and (ramp["travel"] == "both" or (ramp["travel"] == "ab") == increasing) and \
         not (trust is not None and trust.low):
        self.on_ramp, self.ramp = True, ramp
        end = ramp["end"] if ramp["travel"] != "both" or increasing else ramp["start"]
        self.ramp_end_dist = dist_m(lat, lon, *end)
    # (also when the street itself has no curve record: a straight street is no chain)
    if not self.on_ramp and (gps or {}).get("street") and not (trust is not None and trust.low) and self.data_here:
      self._onramp_ahead(lat, lon, bearing, nav, int((gps or {}).get("signal", 0)),
                         max(float((gps or {}).get("dt", 0.05)), 0.0))
    else:
      self.onramp_keep = None
    if nav is not None and not (trust is not None and trust.low):
      self._nav_exit(lat, lon, bearing, *nav)
    if nav is not None and self.on_ramp:
      self._nav_fork(lat, lon, bearing, *nav)

  def _onramp_ahead(self, lat, lon, bearing, nav, signal, dt=0.05):
    """On a street: the first curve of an on-ramp (to a highway) we are about to turn onto, by navigation or by the turn
    signal (ONRAMP_*), governs when it needs a lower speed than the curve found so far (status STATUS_RAMP_CURVE,
    onramp_ahead: the car side plans it as a ramp curve before the ramp is matched). A decision is kept ONRAMP_KEEP."""
    best = self._onramp_find(lat, lon, bearing, nav, signal)
    if best is not None:
      self.onramp_keep = [ONRAMP_KEEP, best[1], best[3], best[4]]
    elif self.onramp_keep is not None:
      keep = self.onramp_keep
      keep[0] -= dt
      ok, d = ahead(lat, lon, bearing, *keep[2])
      if keep[0] <= 0.0 or not ok:
        self.onramp_keep = None
        return
      c, s_c = keep[1], keep[3]
      best = (plan_speed(self._ramp_r(c), d + s_c, c.get("adv", 0.0), decel=self.decel, floor=0.0,
                         lead_time=PLAN_RAMP_LEAD_TIME), c, d + s_c)
    else:
      return
    v, c, dd = best[:3]
    if self.curve is not None:
      cur = plan_speed(self.radius, self.distance, self.advisory, decel=self.decel,
                       floor=0.0 if self.status == STATUS_RAMP_CURVE else self.floor,
                       lead_time=PLAN_RAMP_LEAD_TIME if self.status == STATUS_RAMP_CURVE else PLAN_ROAD_LEAD_TIME)
      if cur <= v:
        return
    self.curve, self.distance, self.radius = c, dd, self._ramp_r(c)
    self.advisory = float(c.get("adv", 0.0) or 0.0)
    self.status, self.source, self.onramp_ahead = STATUS_RAMP_CURVE, 0, True

  def _onramp_find(self, lat, lon, bearing, nav, signal):
    """_onramp_ahead's search: (plan speed, curve, m to it, the ramp's start, m from the start to the curve) or None."""
    nav_ok = nav is not None and nav[0] in ONRAMP_NAV_TYPES and 0 < nav[2] <= ONRAMP_NAV_RANGE
    # (ONRAMP_NAV_ALL) the ramp named among navigation's first maneuvers at the primary's distance: that one is matched
    nav_prim = None
    if not nav_ok and nav is not None and 0 < nav[2] <= ONRAMP_NAV_RANGE + ONRAMP_NAV_ALL_TOL:
      named = next((m for m in getattr(self, "nav_all", []) if m[0] in ONRAMP_NAV_TYPES and 0 < m[2] <= ONRAMP_NAV_RANGE and
                    abs(m[2] - nav[2]) <= ONRAMP_NAV_ALL_TOL), None)
      if named is not None:
        nav, nav_ok = (named[0], named[1], named[2]), True
      elif nav[0] in ONRAMP_NAV_PRIMARY and nav_side(nav[1]) != 0:
        nav_prim = nav
    if not nav_ok and nav_prim is None and not signal:
      return None
    best = None
    for r in self.store.ramps_all:
      if r["type"] == "exit" or r["travel"] == "both" or (r.get("to") or {}).get("cls") not in HIGHWAY_CLASSES:
        continue
      start = r["start"] if r["travel"] == "ab" else r["end"]
      ok, d = ahead(lat, lon, bearing, *start)
      if not ok or d > ONRAMP_NAV_RANGE:
        continue
      chain_len = self.store.chains.get(r["chain"], {}).get("len", 0.0)
      firsts = sorted(((c["s_a"] if r["travel"] == "ab" else chain_len - c["s_b"]), c)
                      for c in self.store.curves.get(r["chain"], []))
      if not firsts:
        continue
      s_first, c_first = firsts[0]
      entry = (c_first["a"] if r["travel"] == "ab" else c_first["b"]) if s_first > NEAR else (r["end"] if r["travel"] == "ab" else r["start"])
      lean = math.sin(math.radians(bearing_to(*start, *entry) - bearing))   # + the ramp leaves to our right
      side = 1 if lean > ONRAMP_SIDE_SIN else (-1 if lean < -ONRAMP_SIDE_SIN else 0)
      by_nav = nav_ok and abs(d - nav[2]) <= max(NAV_MATCH_MIN, NAV_MATCH_TOL * nav[2]) and \
        (nav_side(nav[1]) in (0, side) or side == 0)
      lateral = abs(d * math.sin(math.radians(bearing_to(lat, lon, *start) - bearing)))
      by_signal = signal != 0 and side == signal and d <= ONRAMP_SIGNAL_RANGE and lateral <= ONRAMP_SIGNAL_LATERAL
      by_prim = (nav_prim is not None and side != 0 and nav_side(nav_prim[1]) == side and
                 abs(d - nav_prim[2]) <= ONRAMP_NAV_ALL_TOL and lateral <= ONRAMP_SIGNAL_LATERAL)
      if not (by_nav or by_signal or by_prim):
        continue
      for s_c, c in firsts:
        if s_c < 0 or s_c > ONRAMP_NAV_RANGE:
          continue
        v = plan_speed(self._ramp_r(c), d + s_c, c.get("adv", 0.0), decel=self.decel, floor=0.0,
                       lead_time=PLAN_RAMP_LEAD_TIME)
        if best is None or v < best[0]:
          best = (v, c, d + s_c, start, s_c)
    return best

  def _nav_fork(self, lat, lon, bearing, mtype, modifier, distance):
    """On a watched exit's link: navigation's next maneuver at one of its forks says which branch it takes (logs, and
    mapExitFork 3 when the car took the other one)."""
    side = nav_side(modifier)
    w = self.watch
    if w is None or not side or mtype not in NAV_EXIT_TYPES + ("turn", "continue"):
      return
    for f in w["forks"]:
      ok, d = ahead(lat, lon, bearing, *f["at"])
      if f["res"] is None and ok and abs(d - distance) <= max(NAV_MATCH_MIN, NAV_MATCH_TOL * distance):
        f["nav"] = side

  def _nav_exit(self, lat, lon, bearing, mtype, modifier, distance):
    if mtype not in NAV_EXIT_TYPES or not (0 < distance <= NAV_EXIT_RANGE) or self.on_ramp:
      return
    self.nav_side = nav_side(modifier)
    if not self.data_here:
      self.nav_exit, self.nav_exit_dist = NAV_EXIT_NO_DATA, distance
      return
    tol = max(NAV_MATCH_MIN, NAV_MATCH_TOL * distance)
    best, best_err = None, tol
    for r in self.store.ramps_all:
      if r["type"] not in ("exit", "interchange") or r["travel"] == "both":
        continue
      ok, d = ahead(lat, lon, bearing, *r["start"])
      if not ok or abs(d - distance) > best_err:
        continue
      # the ramp must leave roughly in our direction (not an exit of the other carriageway)
      first_dir = bearing_to(*r["start"], *r["end"])
      if math.cos(math.radians(first_dir - bearing)) < -0.2:
        continue
      best, best_err = r, abs(d - distance)
    if best is None:
      self.nav_exit, self.nav_exit_dist = NAV_EXIT_NO_DATA, distance
      return
    self.nav_ramp, self.nav_exit_dist = best, distance
    w = self._watch_add(best)
    curves = self.store.curves.get(best["chain"], [])
    chain_len = self.store.chains.get(best["chain"], {}).get("len", 0.0)
    # (each with its m from the split along the exit; the branches of its forks count too, from the fork: until the car
    # is on one, the lower speed of both governs)
    curves = [(c, c["s_a"] if best["travel"] == "ab" else chain_len - c["s_b"]) for c in curves]
    for f in (w["forks"] if w["chain"] == best["chain"] else []):
      curves += [(c, f["from_start"] + s) for c, s in self._branch_curves(f)]
    if curves:
      c, s = min(curves, key=lambda cs: plan_speed(ramp_radius(cs[0]), distance + cs[1], cs[0].get("adv", 0.0), decel=self.decel))
      self.nav_curve, self.nav_curve_dist = c, distance + s
      self.nav_exit = NAV_EXIT_CURVES
    else:
      self.nav_exit = NAV_EXIT_NO_CURVES

  # ---- the watched navigation exit (past the split, forks) ----
  def _watch_add(self, r, side=0):
    """Watch the exit `r` navigation matched (its forks found once). A branch of a watched exit's fork (navigation's
    next maneuver on the link, before the road match is on it) is not a new exit: it says which branch navigation
    takes (side: the maneuver's side)."""
    for w in self.watches:
      if w["ramp"] is r or w["ramp"].get("chain") == r.get("chain"):
        return w
      for f in w["forks"]:
        if f["chain"] == r.get("chain"):
          f["nav"] = f["side"]
          return w
    w = {"ramp": r, "chain": r["chain"], "forks": self._forks(r), "entered": {}}
    self.watches = (self.watches + [w])[-WATCH_KEEP:]
    return w

  def _forks(self, r):
    """The links that start on exit `r`'s own link at least FORK_MIN past its start: its branches."""
    grid = self.store.grid
    out = []
    for o in self.store.ramps_all:
      if o is r or o.get("chain") == r.get("chain") or o["travel"] == "both":
        continue
      d0 = dist_m(*r["start"], *o["start"])
      if d0 < FORK_MIN or d0 > float(r.get("len", 0.0)) + FORK_NEAR:
        continue
      la, lo = o["start"]
      brg = bearing_to(la, lo, *o["end"])
      near = near_shapes(grid, la, lo, None, {r["chain"]})
      hit = near.get(r["chain"])
      if hit is None or hit[0] > FORK_NEAR:
        continue
      # which side the branch leaves to: its bearing toward its end against the exit link's toward its own end
      turn = (brg - bearing_to(la, lo, *r["end"]) + 540.0) % 360.0 - 180.0
      out.append({"ramp": o, "chain": o["chain"], "at": (la, lo), "from_start": d0, "side": 1 if turn > 0 else -1,
                  "res": None, "t": 0.0, "pending": None, "nav": 0})
    return out

  def _branch_curves(self, fork):
    """The fork's own branch's curves, each with its m from the branch's start along it."""
    o = fork["ramp"]
    curves = self.store.curves.get(o["chain"], [])
    chain_len = self.store.chains.get(o["chain"], {}).get("len", 0.0)
    return [(c, (c["s_a"] if o["travel"] == "ab" else chain_len - c["s_b"])) for c in curves]

  def _beyond(self, w, fork, c):
    """A curve of the exit's own link that is past the fork (its continuing branch's)."""
    entry = c["a"] if w["ramp"]["travel"] == "ab" else c["b"]
    return dist_m(*w["ramp"]["start"], *entry) > fork["from_start"] - 5.0

  def _exit_candidates(self, w, own):
    """The curves of the watched exit that may still come: (curve, its chain) of the exit's own link (`own`: those of
    its chain) and of the branches, by the forks decided so far (FORK_*): a branch not taken is never one of them."""
    out = []
    for c in own:
      if any(f["res"] == "side" and self._beyond(w, f, c) for f in w["forks"]):
        continue                  # the continuing branch of a fork that went the other way
      out.append(c)
    for f in w["forks"]:
      if f["res"] in (None, "side"):
        out += [c for c, _ in self._branch_curves(f)]
    return out

  def _fork_update(self, w, lat, lon, bearing, dt):
    """Decide the forks the car has reached: the shapes of the two branches past the fork (sustained FORK_CONFIRM)."""
    grid = self.store.grid
    for f in w["forks"]:
      if f["res"] is not None:
        continue
      ok, d = ahead(lat, lon, bearing, *f["at"])
      if ok and d > FORK_SEEN:
        f["t"], f["pending"] = 0.0, None
        continue
      if not ok and d > WATCH_END:
        continue
      near = near_shapes(grid, lat, lon, bearing, {f["chain"], w["chain"]})
      v = shape_verdict(near.get(f["chain"]), near.get(w["chain"]))
      pick = "side" if v > 0 else ("cont" if v < 0 else None)
      if pick is None:
        continue                  # (no answer this frame: the count goes on)
      f["t"] = f["t"] + dt if pick == f["pending"] else dt
      f["pending"] = pick
      if f["t"] >= FORK_CONFIRM - 1e-6:
        f["res"] = pick

  def _watch_update(self, lat, lon, bearing, dt, v=None):
    """The shape check and the exit's curve for the watched exit whose start is nearest (starpilotPlan.mapExit*)."""
    keep = []
    for w in self.watches:
      ok, d = ahead(lat, lon, bearing, *w["ramp"]["start"])
      if (not ok and d > WATCH_END + float(w["ramp"].get("len", 0.0))) or (ok and d > NAV_EXIT_RANGE + WATCH_END):
        continue
      keep.append((d, w))
    self.watches = [w for _, w in keep]
    if not keep:
      return
    d, w = min(keep, key=lambda dw: dw[0])
    if d > WATCH_NEAR + float(w["ramp"].get("len", 0.0)):
      return
    self.watch = w
    chains = {w["chain"]} | {f["chain"] for f in w["forks"]}
    near = near_shapes(self.store.grid, lat, lon, bearing, chains)
    links = [near[c] for c in chains if c in near]
    link = min(links, key=lambda dh: dh[0]) if links else None
    hwy = near.get("hwy")
    # where the map's link starts (the car side's split point: navigation's exit point may be tens of m off it)
    ok, d_start = ahead(lat, lon, bearing, *w["ramp"]["start"])
    # (past it, heading along the highway is "highway" even closer to the link's shape: SHAPE_HWY_HEADING_GAP)
    verdict = shape_verdict(link, hwy, SHAPE_LINK_GAP, None if ok else (SHAPE_HWY_HEADING_GAP, SHAPE_HWY_NEAR,
                                                                               SHAPE_HWY_HEADING_MAX))
    if v is not None and v < SHAPE_MIN_SPEED:
      verdict = 0
    self.exit_shape = EXIT_SHAPE_LINK if verdict > 0 else (EXIT_SHAPE_HIGHWAY if verdict < 0 else EXIT_SHAPE_NONE)
    self.exit_link = link or (0.0, 0.0)
    self.exit_hwy = hwy or (0.0, 0.0)
    self.exit_bends = road_bends(self.store.grid, lat, lon, bearing, chains)
    self.exit_start = (d_start if ok else -d_start) or 0.1
    self._fork_update(w, lat, lon, bearing, dt)
    if w["forks"]:
      self.exit_fork = FORK_OPEN if any(f["res"] is None for f in w["forks"]) else (
        FORK_OTHER if any(f["nav"] and ((f["nav"] == f["side"]) != (f["res"] == "side")) for f in w["forks"]) else FORK_NAV)
    # the exit's next curve from here: straight-line ahead (a curve we reached is "in it" for its length)
    best, best_d, best_speed = None, 0.0, float("inf")
    cands = self._exit_candidates(w, self.store.curves.get(w["chain"], []))
    travel_ab = {w["chain"]: w["ramp"]["travel"] == "ab", **{f["chain"]: f["ramp"]["travel"] == "ab" for f in w["forks"]}}
    for c in cands:
      key = (c.get("chain"), c["s_a"])
      if key in w["entered"]:
        if self.odo - w["entered"][key] < c["len"]:
          d_e = 0.0
        else:
          continue
      else:
        ab = travel_ab.get(c.get("chain"), True)
        entry, exit_ = (c["a"], c["b"]) if ab else (c["b"], c["a"])
        ok, d_e = ahead(lat, lon, bearing, *entry)
        if not ok:
          ok_x, d_x = ahead(lat, lon, bearing, *exit_)
          if not ok_x or d_x > c["len"] + 60:
            continue
          d_e = 0.0
        if d_e <= NEAR and self.odo_ok:
          w["entered"][key] = self.odo + d_e
      sp = plan_speed(ramp_radius(c), d_e, c.get("adv", 0.0), decel=self.decel)
      if sp < best_speed:
        best, best_d, best_speed = c, d_e, sp
    if best is not None:
      self.exit_curve, self.exit_curve_dist = best, best_d

  def _fork_filter(self, cid, cands):
    """On a watched exit's own link: its continuing branch's curves go once a fork went the other way, and the
    branches of a fork not decided yet (or taken) count as curves ahead (the lower speed governs until it is)."""
    w = next((w for w in self.watches if w["chain"] == cid and w["forks"]), None)
    if w is None:
      return cands
    keep = {id(c) for c in self._exit_candidates(w, [c for c, _ in cands])}
    out = [(c, d) for c, d in cands if id(c) in keep]
    ex = self.exit_curve if self.watch is w else None
    if ex is not None and ex.get("chain") != cid and all(c is not ex for c, _ in out):
      out.append((ex, self.exit_curve_dist))
    return out


class MapCurvesPlanner:
  """Planner-side wrapper: reads the setting, GPS position, map match and navigation, runs MapCurves and
  fills frogpilotPlan; also says when to show the notices (frogpilot_events)."""
  PARAM = "MazdaMapCurves"

  def __init__(self, params, params_memory):
    self.params = params
    self.params_memory = params_memory
    self.mode = 0
    self.counter = 0
    self.mc = None
    self.announced_curve = None
    self.announced_exit = None
    self.notice = None          # "curve" / "exit_data" / "exit_no_data" / "paused" for one planner cycle
    self.profile_path = PROFILE_PATH
    self.decel = PLAN_DECEL     # the plan's decel (the same constant on the car side)
    self.profile_decel = PROFILE_DECEL_DEFAULT   # what the car profile measured (logs only), read with the mode at the start of every drive
    self.max_drop_mph = DEFAULT_MAX_DROP_MPH   # the driver's Curve Max Drop, read with the mode (the car side reads it at start too)
    self.metric = False
    self.recorder = None        # camera check / GPS trust events (feature logs), started on first use
    self.recorder_failed = False
    self.trust = GpsTrust()
    self.pause_announced = False  # "Map curves paused" is shown once per drive (and when a pause ends a slowdown)
    self.prev_slowing = False
    self.held_logged = set()
    self.gps_acc, self.gps_rel = 0.0, 0

  def _read_mode(self):
    try:
      self.mode = int(self.params.get(self.PARAM, encoding="utf8") or 0)
    except (TypeError, ValueError):
      self.mode = 0
    self.profile_decel = read_profile_decel(self.profile_path)
    try:
      raw = self.params.get("MazdaCurveSlowdownMaxDrop", encoding="utf8")
      choice = 1 if raw is None or raw == "" else int(raw)    # as curve_slowdown.CurveSlowdown reads it
      self.max_drop_mph = MAX_DROP_CHOICES_MPH[choice] if 0 <= choice < len(MAX_DROP_CHOICES_MPH) else DEFAULT_MAX_DROP_MPH
    except (TypeError, ValueError, AttributeError):
      self.max_drop_mph = DEFAULT_MAX_DROP_MPH
    try:
      self.metric = bool(self.params.get_bool("IsMetric"))
    except Exception:
      self.metric = False
    if self.mc is not None:
      self.mc.decel = self.decel

  @staticmethod
  def _signal(sm):
    """His turn signal: 1 right, -1 left, 0 none (never raises)."""
    try:
      cs = sm["carState"]
      return 1 if cs.rightBlinker else (-1 if cs.leftBlinker else 0)
    except Exception:
      return 0

  def _floor(self, set_speed):
    return max_drop_floor(set_speed, self.max_drop_mph, self.metric)

  def update(self, sm, gps_position, v_ego, set_speed):
    self.notice = None
    # The car side reads the setting once when a drive starts, so read it here only then too (first call, after
    # a pause in calls, or while not started): the screen never says "Slowing" when the car would not act.
    import time
    now = self.now = time.monotonic()
    try:
      started = bool(sm["deviceState"].started)
    except Exception:
      started = True
    drive_start = self.counter == 0 or now - getattr(self, "last_call", now) > 5.0 or (started and not getattr(self, "prev_started", False))
    if drive_start or (not started and self.counter % 20 == 0):
      self._read_mode()
    if drive_start and self.mc is not None:
      self.mc.veto.reset_drive()  # a curve ignored or taken back last drive is checked afresh
    if drive_start:
      self.trust.reset_drive()
      self.pause_announced, self.prev_slowing = False, False
      self.held_logged = set()
    self.call_dt = now - getattr(self, "last_call", now - 0.05)   # unclamped: curve memory's distance travelled
    dt = min(max(self.call_dt, 0.01), 0.2)
    self.prev_started = started
    self.last_call = now
    self.counter += 1
    if self.mode == 0:
      if self.mc is not None:
        self.mc.reset()
      return
    if self.mc is None:
      # the speed comes back from a road curve's apex only with the camera curve slowdown on (it keeps the speed
      # for whatever is left of the curve, and the next curve keeps governing when it is close)
      self.mc = MapCurves(release_after_apex=self.params.get_bool("MazdaCurveSlowdown"), memory=CurveMemory(), decel=self.decel)
      self.pass_curve, self.pass_samples = None, []
      self.matcher = RoadMatcher(self.mc.store)
    elif started:
      # curve files built while parked (curve_mapd) or copied over: re-index at the start of a drive, then every
      # RESCAN_SECONDS (a few folder stats here; the file listing runs in the background)
      self.mc.store.rescan_if_changed(now, force=drive_start)
    if gps_position is None:
      self.mc.reset()
      return
    # The road is matched here on the curve files' road shapes (StarPilot's map program publishes no way match; the
    # old FrogPilot MapMatchedWay path read a key that never existed, so it never ran, and is gone).
    self.gps_acc = float(gps_position.get("accuracy", 0.0) or 0.0)
    self.gps_rel = int(gps_position.get("reliability", 0) or 0)
    acc, rel = (self.gps_acc, self.gps_rel) if USE_GPS_ACCURACY else (0.0, 0)
    way_id, is_forward = self.matcher.update(gps_position["latitude"], gps_position["longitude"], gps_position["bearing"], acc)
    self.road_class = self.matcher.road_class
    nav = None
    try:
      if sm.alive["navInstruction"] and sm.valid["navInstruction"]:
        ni = sm["navInstruction"]
        if ni.maneuverType and ni.maneuverDistance > 0:
          nav = (str(ni.maneuverType), str(ni.maneuverModifier), float(ni.maneuverDistance))
    except Exception:
      nav = None
    nav_all = []
    try:
      if nav is not None:
        nav_all = [(str(m.type), str(m.modifier), float(m.distance))
                   for m in list(sm["navInstruction"].allManeuvers)[:ONRAMP_NAV_ALL]]
    except Exception:
      nav_all = []
    try:
      if sm.alive["navInstruction"] and sm.valid["navInstruction"] and sm.updated["navInstruction"]:
        self._nav_street(sm["navInstruction"], now)
    except Exception:
      pass                         # (the street speed's second opinion only: never the exits' navigation)
    camera = camera_view(sm)
    was_paused = self.trust.paused
    floor = self._floor(set_speed)
    self.mc.floor = floor
    self.mc.update(gps_position["latitude"], gps_position["longitude"], gps_position["bearing"], way_id, is_forward, nav,
                   camera=camera, trust=self.trust,
                   gps={"dt": dt, "v_ego": v_ego, "acc": acc, "rel": rel, "ambiguous": self.matcher.ambiguous,
                        "signal": self._signal(sm),
                        "street": bool(self.road_class) and self.road_class not in HIGHWAY_CLASSES + LINK_CLASSES},
                   nav_all=nav_all)
    mc = self.mc
    if self.trust.paused and not was_paused and mc.data_here and (not self.pause_announced or self.prev_slowing):
      self.pause_announced = True
      self.notice = "paused"     # once per drive, and whenever a pause ends a slowdown (the speed comes back)
    self._feature_log(mc, v_ego, set_speed, now, gps_position)
    self._measure_pass(sm, mc.inside, gps_position["latitude"], gps_position["longitude"], dt=self.call_dt)
    road_floor = 0.0 if mc.status == STATUS_RAMP_CURVE else floor     # ramp curves are planned without the max drop floor
    road_lead = PLAN_RAMP_LEAD_TIME if mc.status == STATUS_RAMP_CURVE else PLAN_ROAD_LEAD_TIME
    # (PLAN_RAMP_LEAD_TIME) a ramp curve's reaction at the car's own speed
    react = max(v_ego, 0.0) if mc.status == STATUS_RAMP_CURVE else set_speed
    self.prev_slowing = mc.curve is not None and set_speed > 0 and \
      plan_speed(mc.radius, mc.distance, mc.advisory, react, decel=self.decel, floor=road_floor,
                 lead_time=road_lead) < set_speed - 2 * MPH
    # notices: once per curve that would need a slowdown, once per announced exit
    if mc.curve is not None and set_speed > 0:
      key = (mc.chain_id, mc.curve["s_a"])
      if key != self.announced_curve and \
         plan_speed(mc.radius, mc.distance, mc.advisory, react, decel=self.decel, floor=road_floor,
                    lead_time=road_lead) < set_speed - 2 * MPH and \
         curve_speed_limit(mc.radius, mc.advisory) < set_speed - 2 * MPH:
        self.announced_curve = key
        self.notice = "curve"
    if mc.nav_exit != NAV_NONE:
      key = (mc.nav_ramp or {}).get("chain", -1)
      if key != self.announced_exit:
        self.announced_exit = key
        self.notice = "exit_data" if mc.nav_exit in (NAV_EXIT_CURVES, NAV_EXIT_NO_CURVES) else "exit_no_data"
    elif not mc.on_ramp:
      self.announced_exit = None

  VETO_COLUMNS = ["t_wall", "vEgo_mph", "set_mph", "map_status", "map_dist", "map_r", "veto", "veto_r",
                  "gps_acc", "gps_rel", "gps_sats", "trust", "chain", "near_road", "other_road", "ambig"]

  def _feature_log(self, mc, v_ego, set_speed, now, gps_position=None):
    """Feature log of the camera check and the GPS trust gate: a row per frame, an event when a curve is ignored,
    taken back or given up on, when the map pauses or resumes, and once per held curve (never raises into the
    planner)."""
    try:
      events, mc.veto.events = mc.veto.events, []
      trust_events, self.trust.events = self.trust.events, []
      g = gps_position or {}
      m = getattr(self, "matcher", None)
      held = None
      if mc.held is not None:
        hc, hd = mc.held
        key = (mc.chain_id, hc["s_a"])
        if key not in self.held_logged:
          self.held_logged.add(key)
          held = {"name": hc.get("name", "") or "", "r": float(hc.get("r", 0.0)), "distance": round(hd, 1),
                  "chain": mc.chain_id, "other_road_m": round(m.other_d, 1) if m else 0.0, "trust": mc.trust_status,
                  "gps_acc": self.gps_acc, "gps_rel": self.gps_rel}
      if self.recorder is None and not self.recorder_failed:
        try:
          from openpilot.starpilot.common.feature_events import EventRecorder
          self.recorder = EventRecorder(self.VETO_COLUMNS, hz=20, params=self.params)
        except Exception:
          self.recorder_failed = True
      for ev in events:
        if ev["what"] == "veto" and mc.memory is not None:
          mc.memory.note_veto(ev["curve"], mc.direction(ev["curve"]))
      rec = self.recorder
      if rec is None:
        return
      rec.add_row(now, (f"{time.time():.2f}", v_ego / MPH, set_speed / MPH, mc.status if mc.curve is not None else 0,  # noqa: TID251
                        mc.distance, mc.radius, mc.veto_status, mc.veto_radius, self.gps_acc, self.gps_rel,
                        int(g.get("satellites", 0) or 0), mc.trust_status, mc.chain_id,
                        round(m.nearest_d, 1) if m else 0.0, round(m.other_d, 1) if m else 0.0, bool(m and m.ambiguous)))
      for ev in events:
        rec.record("mapCurveVeto", now, {**{k: v for k, v in ev.items() if k != "curve"}, "set_mph": set_speed / MPH})
      for what, info in trust_events:
        # position rounded to ~10 m: enough to find the place in the drive, it stays on the device
        rec.record("mapCurveGps", now, {"what": what, **info, "v_ego": round(v_ego, 2), "sats": int(g.get("satellites", 0) or 0),
                                        "confidence": int(g.get("confidence", 0) or 0), "chain": mc.chain_id,
                                        "near_road_m": round(m.nearest_d, 1) if m else 0.0,
                                        "lat": round(float(g.get("latitude", 0.0)), 4), "lon": round(float(g.get("longitude", 0.0)), 4)})
      if held is not None:
        rec.record("mapCurveHeld", now, held)
      for info in getattr(self, "pass_drops", []):
        rec.record("mapCurvePassDropped", now, info)
      self.pass_drops = []
      rec.tick(now)
    except Exception:
      pass

  def _measure_pass(self, sm, inside, lat=None, lon=None, dt=PASS_DT):
    """Curve memory: keep the car's own driving (with the distance travelled); once past the end of the road curve
    we were in, the stretch from PASS_PRE before its start to PASS_POST past its end is the pass, and a clean pass
    adds its measured radius (never raises into the planner). A pass counts for nothing when the turn signal was on
    in it (a lane change) or the car never came within PASS_APEX_NEAR of the apex (another road)."""
    try:
      counter = getattr(self, "counter", 0)
      if counter - getattr(self, "pass_frame", counter) > 1 or not hasattr(self, "pass_buf") or not 0.0 <= dt <= PASS_MAX_DT:
        # a gap in the calls (no GPS, a pause between drives, a stalled planner): the distance travelled is unknown
        self.pass_buf, self.pass_x, self.pass_prev_end = [], 0.0, -float("inf")
        self.pass_curve = None
      self.pass_frame = counter
      cs, cc = sm["carState"], sm["carControl"]
      v = max(float(cs.vEgo), 0.0)
      self.pass_x += v * dt
      x = self.pass_x
      # (PASS_STOP_CUT) a stop beginning: braking or slow (review of round k, S2: not openpilot off or his steering, which
      # in the curve's tail still spoil the pass as before)
      stopping = bool(getattr(cs, "brakePressed", False)) or v < PASS_MIN_SPEED
      self.pass_buf.append((x, (v, float(sm["controlsState"].curvature), bool(cc.latActive), bool(cs.steeringPressed),
                                abs(float(cc.actuators.torque))), bool(cs.leftBlinker or cs.rightBlinker), stopping))
      if len(self.pass_buf) > PASS_BUFFER + 200:
        del self.pass_buf[:200]
      if inside is not None and self.pass_curve is not None and inside is not self.pass_curve and \
         same_curve(inside, self.pass_curve):
        self.pass_curve = inside              # the same curve from rebuilt curve files (a rescan): carry on
      if inside is not None and inside is not self.pass_curve:
        if self.pass_curve is not None:
          self._end_pass(self._pass_end())    # in the next curve already: this pass ends where that one starts
        self.pass_curve, self.pass_x_exit, self.pass_x_next = inside, float("inf"), float("inf")
        self.pass_apex, self.pass_exit, self.pass_vetoed, self.pass_shaky = float("inf"), float("inf"), False, False
        self.pass_exits, self.pass_exit_seen = [], None
        # the direction of travel the pass is remembered under: as MapCurves plans this curve now (direction(c))
        self.pass_dir = getattr(self.mc, "inside_dir", None)
      c = self.pass_curve
      if c is None:
        return
      if inside is c:
        if self.mc.inside_exit != self.pass_exit_seen:
          # a new GPS fix (the distance only changes with one): its estimate of the exit; the middle of them all, so
          # one fix far off does not move the pass
          self.pass_exit_seen = self.mc.inside_exit
          self.pass_exits.append(x + self.mc.inside_exit - v * PASS_GPS_LAG)
          ex = sorted(self.pass_exits)
          self.pass_x_exit = ex[(len(ex) - 1) // 2]
        self.pass_exit = min(self.pass_exit, self.mc.inside_exit)
      nxt = getattr(self.mc, "next_curve", None)
      if nxt is not None and nxt is not c and not same_curve(nxt, c):
        # where the next curve starts: a back-to-back curve (an S-bend) must not end up in this pass (its opposite
        # turn cancels this one's, its tighter stretch reads as this curve's radius)
        self.pass_x_next = min(self.pass_x_next, x + self.mc.next_entry - v * PASS_GPS_LAG)
      if lat is not None and "apex" in c:
        self.pass_apex = min(self.pass_apex, dist_m(lat, lon, *c["apex"]))
      self.pass_vetoed |= self.mc.veto.is_vetoed((self.mc.chain_id, c["s_a"]))
      self.pass_shaky |= bool(self.trust.shaky)
      if not self._same(inside, c) and x >= self._pass_end():
        self._end_pass(self._pass_end())
      elif PASS_STOP_CUT and not self._same(inside, c) and self.pass_x_exit <= x and stopping:
        # through the curve's end, then braking / slowing for a stop (drive 6a, West Utica's second curve before a red
        # light): the pass ends here, clean, without the stop, and counts as full (the car is through the curve's end)
        self.pass_stop_cut = True
        try:
          self._end_pass(x)
        finally:
          self.pass_stop_cut = False
    except Exception:
      self.pass_curve, self.pass_buf = None, []

  @staticmethod
  def _same(a, b):
    return a is b or (a is not None and b is not None and same_curve(a, b))

  def _pass_end(self):
    """m travelled where the pending pass ends: PASS_POST past its curve's end, or where the next curve starts
    (never before this curve's own end)."""
    return min(self.pass_x_exit + PASS_POST, max(self.pass_x_next, self.pass_x_exit))

  def _end_pass(self, x_end):
    """The pass through self.pass_curve ends at x_end (m travelled): its samples, then _finish_pass."""
    c = self.pass_curve
    self.pass_curve = None
    x_start = max(self.pass_x_exit - float(c.get("len", 0.0)) - PASS_PRE, self.pass_prev_end)
    self.pass_prev_end = x_end              # the next pass never takes this one's stretch (an S-bend)
    if self.mc.memory is None or not (x_start < x_end < float("inf")):
      return
    if not self.pass_buf or self.pass_buf[0][0] > x_start + 1.0:
      return                                # the kept driving does not reach back to its start (a gap): not a pass
    # (PASS_STOP_CUT) past the curve's end, the stop's own samples are not the curve's
    window = [(s, b) for xx, s, b, st in self.pass_buf if x_start <= xx <= x_end and
              not (PASS_STOP_CUT and st and xx >= self.pass_x_exit)]
    self.pass_curve, self.pass_samples = c, [s for s, _ in window]
    self.pass_blinker = any(b for _, b in window)
    # (RAMP_MEMORY_*) a stop in the curve: braking below PASS_MIN_SPEED before its end
    self.pass_stop = any(st and s[0] < PASS_MIN_SPEED for xx, s, b, st in self.pass_buf
                         if x_start <= xx <= min(x_end, self.pass_x_exit))
    # ... and his brake (or steering) in it: the ramp memory is cleared (RAMP_MEMORY_*)
    self.pass_brake = any(st and s[0] >= PASS_MIN_SPEED for xx, s, b, st in self.pass_buf
                          if x_start <= xx <= min(x_end, self.pass_x_exit))
    try:
      self._finish_pass()
    finally:
      self.pass_curve, self.pass_samples = None, []

  def _finish_pass(self):
    c, samples = self.pass_curve, self.pass_samples
    if self.pass_blinker or self.pass_apex > PASS_APEX_NEAR:
      return
    direction = getattr(self, "pass_dir", None)
    if c.get("ramp"):
      # (RAMP_MEMORY_*) the steering at its limit, his steering or his brake in the curve: back to the map's radius
      sat = sum(1 for _, _, _, _, cmd in samples if cmd >= PASS_SATURATED) * PASS_DT > PASS_SAT_BLIP + 1e-9
      if sat or any(drv for _, _, _, drv, _ in samples) or getattr(self, "pass_brake", False) or \
         getattr(self, "pass_stop", False):
        self.mc.memory.clear_ramp(c, direction)
        return
    # the road match may have been wrong (GPS not trusted at some point in the pass), or the car did not turn
    # through this curve (a parallel road): learning from it could make a real curve look gentle
    turn_c = float(c.get("turn", 0.0) or 0.0)
    driven = driven_turn(samples)
    reason = "shaky" if getattr(self, "pass_shaky", False) else \
      ("turn" if turn_c >= PASS_TURN_MIN and driven < PASS_TURN_SHARE * turn_c else None)
    if reason is not None:
      self.pass_drops = getattr(self, "pass_drops", []) + [{"reason": reason, "name": c.get("name", "") or "",
                                                             "map_turn": turn_c, "driven_turn": round(driven, 1)}]
      return
    v_sat = curve_pass_saturation(samples)
    if v_sat is not None:
      self.mc.memory.add_saturated_pass(c, v_sat, direction=direction)
      return
    r = curve_pass_radius(samples)
    if r is not None:
      full = self.pass_exit <= PASS_EXIT_NEAR or getattr(self, "pass_stop_cut", False)
      flags = (["full"] if full else []) + (["vetoed"] if self.pass_vetoed else [])
      if c.get("ramp"):
        self.mc.memory.add_ramp_pass(c, r, direction, v=curve_pass_speed(samples), flags=flags)
      else:
        self.mc.memory.add_pass(c, r, v=curve_pass_speed(samples), flags=flags, direction=direction)

  def _nav_street(self, ni, now):
    """Navigation's limit and next maneuver for the car side's street speed (mapNavSpeedLimit, mapNavManeuver*)."""
    lim = float(ni.speedLimit or 0.0)
    self.nav_street = (lim if lim >= NAV_LIMIT_MIN else 0.0, float(ni.maneuverDistance or 0.0),
                       str(ni.maneuverType or "") in NAV_TURN_TYPES and str(ni.maneuverModifier or "") != "straight", now)

  def publish(self, plan):
    mc = self.mc
    plan.mapCurveMode = self.mode
    if mc is None or self.mode == 0:
      return
    ns = getattr(self, "nav_street", None)
    if ns is not None and getattr(self, "now", ns[3]) - ns[3] <= NAV_FRESH:
      plan.mapNavSpeedLimit, plan.mapNavManeuverDistance, plan.mapNavManeuverTurn = ns[0], ns[1], ns[2]
    if not mc.data_here:
      plan.mapCurveStatus = STATUS_NO_DATA
    elif mc.paused:
      plan.mapCurveStatus = STATUS_PAUSED
    elif mc.curve is not None:
      plan.mapCurveStatus = mc.status
    elif mc.held is not None:
      plan.mapCurveStatus = STATUS_HELD      # with the held curve's distance and radius, for the logs only
    else:
      plan.mapCurveStatus = STATUS_NONE
    if mc.curve is None and mc.held is not None:
      plan.mapCurveDistance = float(mc.held[1])
      plan.mapCurveRadius = float(mc._radius(mc.held[0]))
    else:
      plan.mapCurveDistance = float(mc.distance)
      plan.mapCurveRadius = float(mc.radius)
    plan.mapCurveAdvisory = float(mc.advisory)
    # the governing curve's speed as the car side takes it: a road curve never below the driver's Curve Max Drop floor
    # (drive 54, West Utica: the notice said 32 for a curve the car took to 35)
    if mc.curve is not None and mc.radius > 0:
      plan.mapCurveTarget = float(max(curve_speed_limit(mc.radius, mc.advisory),
                                      0.0 if mc.status == STATUS_RAMP_CURVE else mc.floor))
    plan.mapCurveName = (mc.curve or {}).get("name", "") or ""
    plan.mapOnRamp = bool(mc.on_ramp)
    plan.mapOnRampAhead = bool(getattr(mc, "onramp_ahead", False))
    r = mc.ramp if mc.on_ramp else mc.nav_ramp
    if r is not None:
      plan.mapRampType = RAMP_TYPES.get(r["type"], 4)
      to = r.get("to") or {}
      plan.mapRampToSpeed = float(to.get("speed", 0.0) or 0.0)
      plan.mapRampToHighway = to.get("cls") in HIGHWAY_CLASSES
      plan.mapRampEndDistance = float(mc.ramp_end_dist)
    plan.mapRoadClass = int(getattr(self, "road_class", 0))
    plan.mapCurveVeto = int(mc.veto_status)
    plan.mapCurveVetoRadius = float(mc.veto_radius)
    plan.mapCurveSource = int(mc.source)
    plan.mapGpsTrust = int(mc.trust_status)
    plan.mapGpsAccuracy = float(self.gps_acc)
    plan.mapNavExit = mc.nav_exit
    plan.mapNavExitDistance = float(mc.nav_exit_dist)
    plan.mapNavExitSide = int(mc.nav_side)
    if mc.nav_curve is not None:
      plan.mapNavCurveRadius = ramp_radius(mc.nav_curve)
      plan.mapNavCurveDistance = float(mc.nav_curve_dist)
      plan.mapNavCurveAdvisory = float(mc.nav_curve.get("adv", 0.0) or 0.0)
    plan.mapExitShape = int(mc.exit_shape)
    plan.mapExitLinkDist, plan.mapExitLinkHeading = (float(v) for v in mc.exit_link)
    plan.mapExitHighwayDist, plan.mapExitHighwayHeading = (float(v) for v in mc.exit_hwy)
    plan.mapExitFork = int(mc.exit_fork)
    plan.mapExitStart = float(mc.exit_start)
    hb, lb = mc.exit_bends
    plan.mapExitBendValid = hb is not None and lb is not None
    if plan.mapExitBendValid:
      plan.mapExitHighwayBend, plan.mapExitLinkBend = float(hb), float(lb)
    if mc.exit_curve is not None:
      plan.mapExitCurveRadius = ramp_radius(mc.exit_curve)
      plan.mapExitCurveDistance = float(mc.exit_curve_dist)
      plan.mapExitCurveAdvisory = float(mc.exit_curve.get("adv", 0.0) or 0.0)
