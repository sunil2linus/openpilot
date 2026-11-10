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
        exit side (LaneView). Map lane data (which lanes exit) is not in the curve files. On a short ramp (its curve
        out of reach from that speed) the split speed is lowered to the one that makes it (short_split_speed). On the
        camera alone (the edge lane is also the ordinary right lane) only from DECIDE_NEAR and no deeper than
        SPLIT_DECEL takes the speed from there; his signal toward the exit allows it from farther out.
      - At the split (the map's link start when the planner gives it, else navigation's exit point): confirmed on the
        ramp by the map's shapes (starpilotPlan.mapExitShape: our position closer to the link's shape and our heading
        closer to its by SHAPE_LINK_GAP, on consecutive answers, EXIT_CONFIRM_SHAPE_ONLY), or the road match on the link
        (or the planner's ramp) with GPS trusted for LINK_CONFIRM. The driving model's predicted path (ModelPath) is
        logged only for now (MODEL_DECIDES). Until the car is on the link for sure (LINK_SETTLE) a confirmation is
        overturned (fast restore) by the shapes or the trusted road match saying "highway", or his gas held; by a gas
        tap, the other signal or a lane change away only while
        the shapes' last answer is not "link". With exit lane evidence the slowdown starts at the split, tentative (TENT_*:
        bounded, undone by the missed exit test); without, the command holds until then. Then the full slowdown toward
        the exit's curve (the planner's exit curve, the lower of the branches of a fork not decided yet; else the
        announced curve carried on by the distance driven), firm (the cruise's strongest) when the room left needs it.
      - Missed (past the split the shapes say "highway", or the road match on
        the highway with GPS trusted HWY_MATCH_STAY): the plan ends and the speed comes back at once (SET+ holds). With
        no answer at all (a GPS pause) the set speed is held, never restored on a guess (EXIT_LOST_MAX: then back with
        taps when matched to a highway). The driver decides too: his gas, a signal to the other side,
        or a lane change away near the split end the exit slowing at once; a last-minute dive into the ramp is still
        confirmed.
      - "Exit curve - brake" (chime_count): before the split, in an exit lane, when even the above cannot reach the
        exit's curve (the cruise's most); also at the confirmation when the room left is too short.
    Without navigation there is no plan: ramps are handled once on them.
  * Ramp rules (ramp curves, fast restore, street cap) only start after the car has been on the ramp for
    RAMP_HOLD, and the street rules only after STREET_PROGRESS along the exit, so a short wrong ramp match
    (passing an exit in the right lane) changes nothing.
  * End of a ramp onto a highway: restore at once with SET+ holds (merge).
  * End of an exit onto a street with a known speed limit: within STREET_CAP_DISTANCE of the end, cap the
    set speed at the street's limit (plus the Street Speed setting's offset); once off the ramp, that lowered set speed
    becomes the driver's speed (never restored to the highway speed on a street). The street's limit unknown: no
    restore on the ramp above what its slowdown left, and STREET_FALLBACK near its end.
  * Exit to a street, completed (drive 55: GPS trust 2 on the whole ramp, so the planner never said "on a ramp" and
    none of the above ran; his highway 75 came back on the ramp and again with RES on the street): after a link entered
    from a highway (EXIT_LINK_MIN driven on it), once a street (not a highway, not a link) is matched EXIT_STREET_STEADY
    with GPS trust ok, the exit's last target is the street's steady limit plus the setting's offset (STREET_OFFSETS),
    or STREET_FALLBACK when the map has none: the set speed is taken there by our presses (SET- taps down, or a
    pending restore only up to it) and becomes his speed. Only under EXIT_STREET_MAX_V, once per exit; his gas or his
    SET buttons end it (what it reached is his). His RES later on the street (the stock cruise brings back his highway
    speed): speed_limit_follow rule 8 (the match on engage), carried out here (limit_lower).
  * The speed limit follower's street speed (speed_limit_follow rules 8 and 9: the match on engage, a lower limit):
    limit_lower, the speed to take the set speed down to, paced so the cruise slows at most ~0.5 m/s^2 (the set speed
    kept LOWER_GAP under the car's speed: the stock cruise slows ~0.19 m/s^2 per m/s of gap), never a hold; then his
    speed. A pending restore goes only up to it. Curves, stops and the car ahead still win (the lowest target); his gas
    held (GAS_OVERTURN_*) or his SET buttons end it (what it reached is his); cruise off ends it.

Pure logic (no openpilot imports) so it can be unit tested; the wrapper in curve_slowdown.py feeds it.
"""
from dataclasses import dataclass

try:
  from opendbc.car.mazda.curve_target import (MapCurveTarget, MapCurveFollower, map_curve_target, NO_CURVE, START_GAP,
                                              END_GAP, MAP_LATERAL_ACCEL, MAP_DECEL, MAP_FOLLOW_BUDGET,
                                              MAP_START_GAP, MAP_LEAD_TIME, MAP_ROAD_UNDERSHOOT, MAP_ROAD_LEAD_TIME,
                                              MAP_ROAD_ACCEL_TAU)
except ImportError:          # offline tests load this file directly
  from curve_target import (MapCurveTarget, MapCurveFollower, map_curve_target, NO_CURVE, START_GAP, END_GAP,
                            MAP_LATERAL_ACCEL, MAP_DECEL, MAP_FOLLOW_BUDGET, MAP_START_GAP, MAP_LEAD_TIME,
                            MAP_ROAD_UNDERSHOOT, MAP_ROAD_LEAD_TIME, MAP_ROAD_ACCEL_TAU)

MPH = 0.44704
KPH = 1 / 3.6
OFF, TEST, ON = 0, 1, 2
RAMP_FLOOR = 25 * MPH
RAMP_BUDGET = 60                # SET- presses for one ramp (70 -> 25 mph plus ignored presses)
DT_CTRL = 0.01
STREET_CAP_DISTANCE = 200.0     # m before the end of an exit onto a street
RAMP_HOLD = 1.0                 # s on the ramp before ramp rules apply (a short wrong ramp match does nothing)
STREET_PROGRESS = 50.0          # m driven along an exit (its end getting closer) before the street rules apply
# An exit onto a street whose limit the map does not know (mapRampToSpeed 0; outside review of e1be62454, sim_1c.py: after
# the ramp curve 53 SET+ back toward his highway 75, 51.8 mph near the ramp's end): no restore above the speed the ramp's
# slowdown left while on the ramp, and within STREET_CAP_DISTANCE of its end at most this (mph; km/h when metric). Owner
# (drives 54 / 55): 43 mph, "the sweet spot for 35 or 50 streets"; never gets the Street Speed offset
STREET_FALLBACK = (43, 70)
# Street Speed (MazdaStreetSpeedOffset 0..3): added to every target set from a posted street limit (not a highway's, not a
# link's): legal limit, +3, +5 (default), +8 mph; metric 0, +5, +8, +13 km/h (the same speeds rounded to whole km/h)
STREET_OFFSETS = ((0, 3, 5, 8), (0, 5, 8, 13))
STREET_OFFSET_DEFAULT = 2
STREET_LIMIT_MAX = (55, 90)     # mph / km/h: a posted limit above this is not a street's (the exit's street rules ignore it)
# Exit to a street, completed (see the module docstring)
EXIT_LINK_MIN = 50.0            # m driven on a link entered from a highway before a street counts as the exit's
EXIT_STREET_STEADY = 2.0        # s matched to a street (GPS trust ok) before the exit's street target applies
EXIT_STREET_LIMIT_STEADY = 2.0  # s the street's posted limit is reported unchanged before it is its target
EXIT_STREET_MAX_V = 55 * MPH    # m/s: only below this (a wrong street match on the highway is at highway speed)
EXIT_STREET_TIME = 60.0         # s after the street was matched: his RES may still be capped
# Speed limit follow's street speed (limit_lower): the set speed this far under the car's speed at most while it slows
# (the stock cruise ~0.19 m/s^2 per m/s of gap after ~1 s: ~0.46 m/s^2; drives 4f-53 measured): 45 -> 30 mph in ~14 s
LOWER_GAP = 2.4                 # m/s
# Set far above the car's speed (his RES at a red light brings back the highway's 75 at 12 mph): one SET- hold down to
# the target, but never below the car's speed plus this (it cannot brake the car), when this much above that
LOWER_HOLD_ABOVE = 5 * MPH      # m/s
LOWER_HOLD_MIN = 10 * MPH       # m/s
EXIT_STREET_DONE = 0.6 * MPH    # m/s: the set speed this close to the target: it is his speed
ADOPT_LANDED_FRAMES = 2         # the street speed reached: adopted this frame and the next (_street_end)
HWY_RESET = 1.0                 # s matched to a highway: the exit sequence starts over
STATUS_CURVE, STATUS_RAMP_CURVE = 1, 2
STATUS_PAUSED, STATUS_HELD = 4, 5   # the planner's: GPS not trusted / a curve found but the road match not trusted
# A ramp curve lost to status 4 / 5 (drive 69 t 529: in the on-ramp loop's entry GPS trust fell to 2, the loop's 30 mph
# curve became a held 212 m one, and with the camera curve off for his signal the restore held SET+ 31 -> 35 where the
# car was at ~3.4 m/s^2): while on a link, or for RAMP_STATUS_HOLD after a ramp curve last governed, status 4 / 5 keeps
# the ramp curve's last command (no SET+ above it; nothing lower than the set speed it found either). A ramp onto a
# highway lets go of it RAMP_STATUS_HWY_RELEASE before its end, as the planner does (map_curves.RAMP_HWY_RELEASE).
RAMP_STATUS_HOLD = 3.0           # s
RAMP_STATUS_HWY_RELEASE = 120.0  # m
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
# s of the shapes saying "link" on consecutive answers (review of dff3e3310: 0.5 s with "no answer" gaps not counting
# against it confirmed on the highway; drive 53: "link" held without a gap 10+ s once on both ramps)
EXIT_CONFIRM_TIME = 1.0
EXIT_STAY_TIME = 1.0            # s of the shapes saying "highway" past the exit point: we stayed on (missed exit)
# Planner without the shape check (an older build): past the exit point by this, on the highway with the match not
# doubtful, the exit is missed (twice drive 53's 34 m link match at 400). The last resort for everything:
# EXIT_PASS_MARGIN past the exit point and still not confirmed.
EXIT_STAY_MARGIN = 90.0         # m
EXIT_PASS_MARGIN = 250.0        # m
# ... and then (no fast restore: it may be the ramp) the set speed is held until the shapes or the road match tell, at most
# EXIT_LOST_MAX past the exit point; then his speed comes back (taps) when matched to a highway, else it becomes his speed
EXIT_LOST_MAX = 800.0           # m
# The road match on a link with GPS trust below TRUST_LOW: this long confirms the ramp (review: LINK_HOLD's 0.5 s took a
# 34 m wrong match), LINK_SETTLE for the car to be on it for sure (drive 53: the matches held 10+ s on both ramps)
LINK_CONFIRM = 2.0              # s
LINK_SETTLE = 3.0               # s
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
SPLIT_CAP_FLOOR = 10            # mph (km/h when metric): never kept more than this below the split speed
SPLIT_OFFSETS = (0, 5, 10)      # MazdaExitSplitSpeed 0/1/2: the highway's limit + this (mph, km/h when metric); 3: Off
SPLIT_OFF = 3
# The early brake chime: the cruise's strongest slowdown with the set speed held far below (drives 44-53: 0.9-1.0 m/s^2,
# never more) after its ~1 s dead time; the curve's speed reached MAP_LEAD_TIME before its entry.
CHIME_DECEL = 0.9               # m/s^2 (planned with, the short ramp's split speed)
CHIME_DEAD = 1.0                # s
# The chime itself only when even that cannot make it: the cruise's most (1.0 m/s^2) and the curve's speed by its entry
# (no MAP_LEAD_TIME), so the planned short ramp speed (0.9, the lead time) leaves a margin before the chime
CHIME_MAX = 1.0                 # m/s^2
# ... and the ramp is only confirmed (the full slowdown starts) a few s past the exit point: drive 53 (its GPS through the
# planner), the shapes said "link" from 3.4 s (E1) and 3.9 s (E2) after navigation's exit point (the car's heading turns
# toward the link's only once the exit lane leaves the highway), confirmed EXIT_CONFIRM_TIME later
CHIME_CONFIRM = 4.0             # s at the split speed
CHIME_AHEAD = 12.0              # s before the split (at least CHIME_AHEAD_MIN): the chime before the split comes from here
CHIME_AHEAD_MIN = 250.0         # m
DECIDE_NEAR = 300.0             # m before the exit point: his gas / a lane change away there is his decision
# Final review of 7d155ce18 (owner's safety priority): the camera's edge lane is also the ordinary right through lane, so
# on the camera alone the slowdown before the split (the setting's split speed, the short ramp's) starts only DECIDE_NEAR
# before it, and no deeper than SPLIT_DECEL takes the speed from there (cam_floor); only his signal toward the exit starts
# it earlier and deeper (s1_bstay.py: with the camera from 2 km, 47 mph at the split from his 75 / 80 while he stayed on,
# the slowdown from 1.2-1.7 km before, 27-66 s more than 20 mph under his speed)
# Owner (after the drive 53 replay of dff3e3310: E1 49 mph into the 32 mph curve, E2 44 into 31): the ramp slowdown starts
# at the split, not at the shape confirmation (4.8 s later on drive 53), when there was exit lane evidence (his signal
# toward the exit, the camera's edge lane) DECIDE_NEAR before it. Until confirmed it is tentative, and its cost when he
# stays on is bounded: the set speed never more than TENT_GAP below the speed (the cruise slows ~0.19 m/s^2 per m/s of
# gap, measured: ~TENT_DECEL), for TENT_MAX after the split at most; then the set speed goes back to the speed (no more
# slowing) until the shapes confirm the ramp (the full slowdown) or say "highway" (the missed exit test: restored at
# once). He loses at most ~TENT_DECEL x TENT_MAX (~1.3 m/s, 3 mph) for ~3-5 s (test_exit_plan; stay3.py; drive 53's
# split geometry put the "highway" answer for a car going on 4 / 5.5 / 7 m right of the highway's line 1.3 / 2.8 / 4.3 s
# past the exit point, stayreal.py).
TENT_GAP = 5 * MPH              # m/s
TENT_MAX = 3.0                  # s
TENT_LINK = 0.5                 # s of "link" without a gap (the shapes or the model path) that carry it on past TENT_MAX
# ... and so does no answer at all yet, up to TENT_MAX_QUIET (drive 53: the shapes' first answer past the split came 5-6 s
# on at E1 with SHAPE_LINK_GAP; a car going on in the lane next to the exit gets "highway" 1.3-4.3 s past it, stayreal.py)
TENT_MAX_QUIET = 6.0            # s
QUIET_FROM = 10.0               # m past the split (at the split itself the shapes may still say "highway")
TENT_DECEL = 0.45               # m/s^2: the most it gives (the chime)
# ... and whatever carries it on (final review: shapes saying "link" 34 m, nothing 6 m, over and over, ratcheted it down
# 23 mph under his speed while he stayed on): it stops slowing TENT_HARD_MAX after the split, or once the speed is
# TENT_DIP_MAX under the speed at the split (drive 53's replay, the shapes alone: the confirmation 7.8 s after E1's split,
# the speed 6.3 mph down by then; 6.2 s / 4.3 mph at E2; a cap on the set speed instead cost E1 4 mph at its curve)
TENT_HARD_MAX = 9.0             # s
TENT_DIP_MAX = 8 * MPH          # m/s
# ... and what it gave on drive 53's replay (the taps follow the speed down, the gap 3-5 mph), with the confirmation it
# waits for (drive 53's GPS through the planner, the shapes alone with SHAPE_LINK_GAP and EXIT_CONFIRM_SHAPE_ONLY: 7.3-7.6 s
# past E1's exit point, 5.7-6.2 s past E2's, the GPS fix's age included; the model path's agreement would shorten it, not
# verified): the short ramp's split speed is planned with these
TENT_PLAN_DECEL = 0.35          # m/s^2
TENT_PLAN_CONFIRM = 7.0         # s
# the firm slowdown after the confirmation as planned for the short ramp's split speed: it takes ~2 s to build up (the
# SET- hold, the cruise's answer) to its ~0.95-1.0 (drive 53 replay: 0.94 m/s^2 on average from the confirmation to E2's
# curve), so a little under CHIME_DECEL
SHORT_PLAN_DECEL = 0.8          # m/s^2
# Short ramps (owner): when the exit's curve cannot be reached from the split speed (the tentative slowdown to the
# confirmation CHIME_CONFIRM after the split, then the cruise's strongest), the split speed is lowered to the speed that
# makes it (exit_room_needed), only in an exit lane, never raising; a long ramp keeps the setting's split speed.
# A new posted limit on the highway must be steady this long before it is the highway's (not a flicker to the ramp's
# limit near the split)
LIMIT_STEADY = 3.0              # s
LIMIT_NEAR = 1000.0             # m before the split: no first limit, none lower than the highway's (a ramp's)
RESTORE_PULSE = 1.0             # s the fast restore is asked for after a plan ends by his decision or a missed exit
# The road match on a highway with GPS trusted this long past the split, EXIT_STAY_MARGIN or more past it and the shapes' last
# answer not "link", is staying on (final review: with no shape answer the speed was held and then adopted 20-50 mph low);
# so is, after a confirmation, the model path saying "highway" EXIT_STAY_TIME with the shapes' last answer not "link"
HWY_MATCH_STAY = 2.0            # s
# His gas past the split, before the car is on the ramp for sure, held this long or speeding the car up this much since
# he pressed it, is his decision even while the shapes' last answer is "link" (re-review of d4f848215: a false "link" for
# 1.5 s, then silence, ran the full ramp slowdown on the highway to ~30 mph, his gas ignored); a tap is not (drive 53's
# E1 / E2 timelines: a 3 m tap on the ramp kept the slowdown, s7_rampgas.py). (Not the speed over the set speed: the
# firm slowdown holds the set speed far below it on purpose.)
GAS_OVERTURN_TIME = 1.5         # s
GAS_OVERTURN_GAIN = 3 * MPH     # m/s

# ---- the camera's view of our lane (LaneView) ----
# modelV2 lane lines 0-3 (left-left, left, right, right-right; y positive to the right, MODEL_LEFT) and
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
# The model's y is positive to the RIGHT (final review: selfdrive/controls/lib/ldw.py, the left line close when its y >
# -1.08; radard.py matches a radar point's yRel to -lead.y; the device frame is x forward, y right, z down). LaneView still
# takes the side from our own two lines when it sees them (-1: y positive right), MODEL_LEFT until then.
MODEL_LEFT = -1.0


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
    self.left = MODEL_LEFT                # -1: the model's y is positive to the right (+1: to the left)
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
    stds = edge_stds if edge_stds and len(edge_stds) >= 2 else (EDGE_STD_MAX, EDGE_STD_MAX)   # (missing: not certain)
    gaps = []
    # which way is left in the model's y: from our two lines when seen (convention free), else MODEL_LEFT
    if probs[1] > LANE_OWN_PROB and probs[2] > LANE_OWN_PROB and abs(lines_y[1] - lines_y[2]) > 1.0:
      self.left = 1.0 if lines_y[1] > lines_y[2] else -1.0
    for s, own, beyond, e in ((-1, 1, 0, 0), (1, 2, 3, 1)):
      ref = lines_y[own] if probs[own] > LANE_OWN_PROB else (self.left if s < 0 else -self.left) * LANE_HALF
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
      jump = ((lines_y[1] - self.prev[0]) + (lines_y[2] - self.prev[1])) / 2 * self.left
      # (moving left the lines drift right, then at the crossing both lines become the ones left of them: both jump a
      # lane to the left; moving right the opposite)
      if jump > LANE_JUMP:
        self.change = -1
      elif jump < -LANE_JUMP:
        self.change = 1
    self.prev = (lines_y[1], lines_y[2]) if both else None


# ---- the driving model's predicted path (owner: a second, independent source for the ramp confirmation) ----
# The model path's heading MODEL_AHEAD m ahead relative to our lane's direction near the car (its lane lines 5-20 m ahead;
# without them the path's own heading 5-15 m ahead), right positive; the planner gives the highway's and the exit link's
# heading that far ahead along their shapes relative to the highway's here (mapExitHighwayBend / mapExitLinkBend; drive
# 53: the links ~5 deg off the highway from their start). When the two differ by MODEL_SEP or more, the model path that is
# MODEL_GAP closer to one of them says which road we follow. A drift in the lane does not move it (the path goes back to
# the lane's center ahead); a ramp does once its lane separates. Not verified on a drive: the qlogs have no modelV2 (the
# exit log records these values).
MODEL_AHEAD = 80.0              # m (map_curves.BEND_AHEAD)
MODEL_SEP = 3.0                 # deg
MODEL_GAP = 1.0                 # deg
MODEL_STALE = 0.5               # s: a model answer older than this is no answer (dropouts)
MODEL_CONFIRM = 0.5             # s of the model saying "link" (consecutive), with the shapes' EXIT_CONFIRM_TIME
MODEL_STAY_SHAPES = 0.5         # s of the shapes and the model path both saying "highway" past the exit point: stayed on
# Without a model answer (no modelV2, the roads not apart enough ahead) the shapes alone need this long
EXIT_CONFIRM_SHAPE_ONLY = 1.5   # s
# Outside review of e1be62454 (sim_1a.py): the model path is measured against our own lane ahead, the map's bends against
# the highway at the car, so once the car's lane is a straight ramp (drive 53: E1's first 270 m, E2's first 80 m) the
# model path reads "highway" on the ramp (E1 confirmed only 213 m past the split, 44 mph into its 32 mph curve; E2 never).
# Until the two are in one frame and checked on a drive with modelV2, the model path is logged only (mapExit: model_rel,
# model_verdict, the bends) and never changes a decision (the confirmation, the missed exit, the tentative slowdown).
MODEL_DECIDES = False


def _interp(x, xs, ys):
  if not xs or x < xs[0] or x > xs[-1]:
    return None
  for k in range(1, len(xs)):
    if xs[k] >= x:
      x0, x1 = xs[k - 1], xs[k]
      f = (x - x0) / (x1 - x0) if x1 > x0 else 0.0
      return ys[k - 1] + (ys[k] - ys[k - 1]) * f
  return ys[-1]


def model_path_heading(path_x, path_y, lines=(), left=MODEL_LEFT):
  """deg, right positive: the model path's heading MODEL_AHEAD m ahead relative to our lane's near the car. lines: [(x
  list, y list)] of our lane lines the model is sure of. None when the path is too short."""
  import math
  xs, ys = list(path_x), list(path_y)
  a, b = _interp(MODEL_AHEAD - 5.0, xs, ys), _interp(MODEL_AHEAD + 5.0, xs, ys)
  if a is None or b is None:
    return None
  h_far = math.degrees(math.atan2(b - a, 10.0))
  refs = []
  for lx, ly in lines:
    lx, ly = list(lx), list(ly)
    p, q = _interp(5.0, lx, ly), _interp(20.0, lx, ly)
    if p is not None and q is not None:
      refs.append(math.degrees(math.atan2(q - p, 15.0)))
  if not refs:
    p, q = _interp(5.0, xs, ys), _interp(15.0, xs, ys)
    if p is None or q is None:
      return None
    refs.append(math.degrees(math.atan2(q - p, 10.0)))
  return -(h_far - sum(refs) / len(refs)) * left


class ModelPath:
  """The model path's heading ahead (model_path_heading) as the wrapper last gave it, with its age."""

  def __init__(self):
    self.rel = None
    self.age = MODEL_STALE + 1.0

  def update(self, rel):
    self.rel, self.age = rel, 0.0

  def tick(self, dt):
    self.age += dt

  def verdict(self, f):
    """2 the link, 1 the highway, 0 no answer, None not available (no fresh model path, the roads not apart ahead)."""
    if self.rel is None or self.age > MODEL_STALE or not f.exit_bend_valid or \
       abs(f.exit_link_bend - f.exit_hwy_bend) < MODEL_SEP:
      return None
    dl, dh = abs(self.rel - f.exit_link_bend), abs(self.rel - f.exit_hwy_bend)
    return 2 if dl + MODEL_GAP <= dh else (1 if dh + MODEL_GAP <= dl else 0)


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
  exit_start: float = 0.0         # mapExitStart: m to the exit's link start on the map (negative behind), 0 unknown
  exit_hwy_bend: float = 0.0      # mapExitHighwayBend / mapExitLinkBend / mapExitBendValid (ModelPath)
  exit_link_bend: float = 0.0
  exit_bend_valid: bool = False
  onramp_ahead: bool = False      # mapOnRampAhead: the ramp curve is an on-ramp's first curve, before the turn onto it

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
                     int(g("mapExitFork", 0)), float(g("mapExitStart")), float(g("mapExitHighwayBend")),
                     float(g("mapExitLinkBend")), bool(g("mapExitBendValid", False)), bool(g("mapOnRampAhead", False)))


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
  # the governing target is the street speed (the exit's street, limit_lower): the controller may press from KEEP_SPEED
  # (SlowdownInputs.street_lower)
  street: bool = False
  # m/s: the street speed just reached and adopted (0 none): the wrapper also forgets a set speed above the anchor at it
  # (our last press landing in the adopt frame re-anchored one step under it: drive 54, t 745.2, 43 -> 42)
  adopt_target: float = 0.0


class ExitPlan:
  """The exit navigation's route takes, by where it is (m of the control's odometer)."""

  def __init__(self, odo, f):
    self.start_off = 0.0          # m the map's link start is past navigation's exit point (mapExitStart)
    self.exit_at = odo + f.nav_exit_distance
    self.side = f.nav_exit_side
    self.curve = (0.0, 0.0, 0.0)  # radius, m of the odometer at its entry, advisory
    self.limit = 0.0              # m/s, the highway's posted limit (last known while on the highway)
    self.confirmed = False
    self.declined = False         # his decision (gas, the other signal, a lane change away): no exit slowing before the ramp
    self.signalled = False        # he signalled toward the exit (kept: the signal may go off once in the exit lane)
    self.chimed = False           # the brake chime before the split
    self.slowing = False          # the slowdown before the split is on
    self.evidence = False         # exit lane evidence (signal toward it, the camera's edge lane) DECIDE_NEAR before it
    self.tentative = False        # the ramp slowdown started at the split, not confirmed yet
    self.tent_s = 0.0             # s of it
    self.tent_v0 = None           # m/s: the speed when it began (TENT_DIP_MAX)
    self.cam_floor = None         # m/s: on the camera alone, the lowest speed before the split (SPLIT_DECEL from DECIDE_NEAR)
    self.v_cap = 0.0              # m/s: the car was slower than the split speed when it began: the speed is kept at this
    self.hold = None              # m/s: the command held from the split to the confirmation
    self.link_s = self.hwy_s = 0.0  # s of the shapes saying link (consecutive) / highway
    self.last_shape = 0           # the shapes' last answer (0 none yet)
    self.answered = False         # the shapes answered (either way) past the split (QUIET_FROM)
    self.match_s = 0.0            # s matched to a link road (or the planner's ramp) with GPS trusted
    self.hwy_match_s = 0.0        # s matched to a highway with GPS trusted past the split
    self.gas_s = 0.0              # s of his gas without a break (GAS_OVERTURN_TIME)
    self.gas_v0 = 0.0             # m/s: the speed when he pressed it
    self.model_link_s = self.model_hwy_s = 0.0  # s of the model path saying link / highway (consecutive)
    self.model_seen = False       # the model path gave an answer
    self.settled = False          # on the ramp for sure (LINK_SETTLE): a confirmation is no longer overturned
    self.last_cmd = None          # m/s: the plan's last command (held when the split is not resolved, EXIT_LOST)
    self.grace = 0.0              # s navigation has not shown it (far from it)
    self.seen_shape = False       # the planner gives the shape check
    self.why = ""                 # how it ended (logs)
    self.update_nav(odo, f)

  def update_nav(self, odo, f):
    self.exit_at = odo + f.nav_exit_distance + self.start_off
    if f.nav_exit_side:
      self.side = f.nav_exit_side
    if f.nav_curve_radius > 0:
      self.curve = (f.nav_curve_radius, odo + f.nav_curve_distance, f.nav_curve_advisory)
    elif f.nav_exit == 2:
      self.curve = (0.0, 0.0, 0.0)                # (review: re-identified as an exit without curves: not the old one's)
    self.grace = 0.0

  def same(self, odo, f):
    d = f.nav_exit_distance
    return abs(odo + d + self.start_off - self.exit_at) <= max(NAV_SAME_MIN, NAV_SAME_SHARE * d)


HOLD_WHYS = ("exit hold", "exit lost", "exit wait")


def exit_room_needed(v, v_curve, confirm=CHIME_CONFIRM, tent=TENT_DECEL, decel=CHIME_DECEL, dead=CHIME_DEAD,
                     lead=MAP_LEAD_TIME, tent_time=TENT_MAX):
  """m from the split the cruise needs from v (m/s) to be at v_curve `lead` s before the curve's entry: its dead time,
  the tentative slowdown (tent, for tent_time), the speed then until the confirmation `confirm` s after the split has
  taken effect (its dead time), then the cruise's strongest (decel)."""
  if v <= v_curve:
    return v * dead
  t_t = min(tent_time, confirm)
  v1 = max(v - tent * t_t, v_curve)
  d = v * dead + ((v * v - v1 * v1) / (2 * tent) if tent > 0 else 0.0)
  if v1 > v_curve:
    d += v1 * (confirm - (v - v1) / tent if tent > 0 else confirm)
    d += (v1 * v1 - v_curve * v_curve) / (2 * decel)
  return d + v_curve * lead


def short_split_speed(room, v_curve, hi):
  """The highest split speed (m/s, at most hi) from which exit_room_needed (as planned: TENT_PLAN_*) fits `room` m
  (never below v_curve)."""
  def need(v):
    return exit_room_needed(v, v_curve, confirm=TENT_PLAN_CONFIRM, tent=TENT_PLAN_DECEL, decel=SHORT_PLAN_DECEL)
  if need(hi) <= room:
    return hi
  lo = v_curve
  for _ in range(30):
    mid = (lo + hi) / 2
    lo, hi = (mid, hi) if need(mid) <= room else (lo, mid)
  return lo


def reach_needed(v, v_curve, decel=CHIME_DECEL, dead=CHIME_DEAD, lead=MAP_LEAD_TIME):
  """m the cruise needs from v (m/s) to be at v_curve MAP_LEAD_TIME before a curve's entry at its strongest slowdown."""
  if v <= v_curve:
    return 0.0
  return v * dead + (v * v - v_curve * v_curve) / (2 * decel) + v_curve * lead


class MapCurveControl:
  def __init__(self, lat_accel, decel=MAP_DECEL, split_mode=0, metric=False, street_mode=STREET_OFFSET_DEFAULT):
    """decel: the cruise's planned slowdown, m/s^2 (MAP_DECEL; the planner plans road curves with the same constant),
    for road curves, ramp curves and navigation exits alike. split_mode: MazdaExitSplitSpeed (0 the highway's limit, 1
    +5, 2 +10, 3 Off); metric: the offsets are km/h. street_mode: MazdaStreetSpeedOffset (STREET_OFFSETS)."""
    lat_accel = min(lat_accel, MAP_LATERAL_ACCEL)
    self.lat_accel = lat_accel
    self.decel = decel
    self.split_mode = split_mode if split_mode in (0, 1, 2, SPLIT_OFF) else 0
    self.unit = KPH if metric else MPH
    m = 1 if metric else 0
    street_mode = street_mode if street_mode in range(len(STREET_OFFSETS[m])) else STREET_OFFSET_DEFAULT
    self.street_add = STREET_OFFSETS[m][street_mode] * self.unit      # m/s added to a posted street limit
    self.street_fallback = STREET_FALLBACK[m] * self.unit
    self.street_limit_max = STREET_LIMIT_MAX[m] * self.unit
    self.street_floor = RAMP_FLOOR   # (curve_slowdown.MIN_SET_SPEED: the controller never goes under it for a curve target)
    self.curve = MapCurveTarget(lat_accel, decel)
    self.follower = MapCurveFollower(firm=True)          # (firm only for a ramp curve, see update)
    self.road_follower = MapCurveFollower(undershoot=MAP_ROAD_UNDERSHOOT, lead_time=MAP_ROAD_LEAD_TIME, closed_loop=True)
    self.a_f = None                 # the car's acceleration, filtered (MAP_ROAD_ACCEL_TAU), for the road follower
    self.nav = MapCurveTarget(lat_accel, decel)
    self.nav_follower = MapCurveFollower(firm=True)
    self.split_follower = MapCurveFollower(max_decel=SPLIT_MAX_DECEL, undershoot=SPLIT_UNDERSHOOT, lead_time=0.0)
    self.split_active = False
    self.lane = LaneView()          # fed by the wrapper from modelV2
    self.model = ModelPath()        # the same (model_path_heading)
    self.model_v = None             # this frame's model verdict (logs)
    self.odo = 0.0                  # m driven (vEgo x dt; drive() through a planner gap)
    self.plan = None                # ExitPlan
    self.last_plan = None           # the plan that ended last (logs; its exit is never taken on again)
    self.done_at = None             # m of the odometer: the exit point of a plan that ended (no re-adoption)
    self.restore_frames = 0         # fast restore asked for this many frames more (RESTORE_PULSE)
    self.adopt_frames = 0           # the set speed becomes his for this many frames more (an exit never resolved)
    self.adopt_target = 0.0         # m/s: _street_end's target while its adopt lasts
    self.hwy_limit = (0.0, 0.0)     # the highway's steady posted limit (m/s) and the odometer where it became steady
    self.lim_new, self.lim_s = 0.0, 0.0
    self.chime_count = 0            # "Exit curve - brake" chimes so far (the wrapper raises the alert on a change)
    self.chime_why = ""
    self.on_street_exit = False
    self.street_last = None         # m/s: the last speed a slowdown asked for on this exit onto a street (STREET_DEFAULT)
    self.ramp_frames = 0            # consecutive updates on a ramp
    self.ramp_end_first = 0.0       # distance to the ramp's end when it was first matched
    self.link_frames = 0            # consecutive updates matched to a link road
    self.road_before = 0            # the last road class matched that is not a link (0 unknown): where a link was entered
    self.link_hwy = False           # the link we are on is connected to a highway (HIGHWAY_LINK_CLASSES)
    # the exit to a street, completed (_exit_street): the link from a highway, the street, its limit, the target
    self.xs_link = 0.0              # m driven on a link entered from a highway (-1: not on such a link)
    self.xs_exited = False          # EXIT_LINK_MIN of it driven: a street after it is the exit's
    self.xs_street_s = 0.0          # s matched to a street (trust ok) since
    self.xs_hwy_s = 0.0             # s matched to a highway (HWY_RESET)
    self.xs_since = None            # s since the street counted (None: no exit street open)
    self.xs_lim, self.xs_lim_s = 0.0, 0.0   # the street's posted limit as reported, and s unchanged
    self.xs_on = False              # the street target is being applied
    self.xs_done = False            # applied once on this street (his RES may still open it again)
    self.ll_on = False              # limit_lower (speed_limit_follow rules 8 / 9) being carried out
    self.ll_gas_s, self.ll_gas_v0 = 0.0, 0.0
    self.lower_event = None         # "done" / "cancel: ..." when it ended (the wrapper tells the follower)
    self.xs_target = NO_CURVE
    self.ramp_last_cmd = None       # m/s: the last ramp curve's command (RAMP_STATUS_HOLD)
    self.ramp_last_curve = NO_CURVE # m/s: ... and that curve's speed
    self.ramp_last_frames = 0       # updates left in which status 4 / 5 still keeps it (off a link)
    self.ramp_last_onramp = False   # ... that ramp curve was on (or before) a ramp onto a highway
    # on (or just before) a ramp onto a highway, as far as known this update: the wrapper keeps camera-only stops (the
    # stop-light detector, the model's planned stop) out there (CurveSlowdown.update's red_ok)
    self.onramp_now = False

  @property
  def exit_rem(self):
    """m to the planned exit point (negative past it), None without a plan."""
    return None if self.plan is None else self.plan.exit_at - self.odo

  def drive(self, step):
    """m driven (the car's own speed x dt) while the planner is missing (the wrapper calls update only while it is
    alive): the exit plan is counted on by it, so a planner gap does not stretch the slowdown past the exit point."""
    self.odo += step

  def _end(self, why, restore=False, adopt=False):
    """restore: the speed back at once (fast restore); adopt: the set speed now becomes his (nothing restored: the exit
    was never resolved, it may be the ramp)."""
    self.adopt_frames = 10 if adopt else 0
    p = self.plan
    if p is not None:
      p.why = why
      self.last_plan, self.done_at = p, p.exit_at
    self.plan = None
    self.split_active = False
    self.split_follower.reset()
    self.nav_follower.reset()
    self.nav.active = False
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
      # where the map's link starts is the split (navigation's exit point may be tens of m off it): taken while it is
      # ahead and agrees with navigation's within NAV_SAME_MIN x 2
      if (not p.confirmed and f.exit_start > 0 and abs(self.odo + f.exit_start - p.exit_at) <= 2 * NAV_SAME_MIN):
        p.start_off += self.odo + f.exit_start - p.exit_at
        p.exit_at = self.odo + f.exit_start
      # the highway's own limit (review: the first one seen was taken at once, and a ramp's 45 showing up before the
      # split became the split target): the steady one (_highway_limit) once it was steady LIMIT_NEAR or more before the
      # split; closer, never a lower one than the highway's known one (the ramp's), nor a first one
      lim, at = self.hwy_limit
      if lim > 0 and abs(lim - p.limit) > 0.1 and (at <= p.exit_at - LIMIT_NEAR or (p.limit > 0 and lim > p.limit)):
        p.limit = lim
    return p

  def _highway_limit(self, f, on_link):
    """The posted limit steady LIMIT_STEADY while matched to a highway with GPS trust ok: (m/s, the odometer where it
    became steady)."""
    if f.road_class in HIGHWAY_CLASSES and f.limit > 0 and not on_link and f.gps_trust < TRUST_LOW:
      self.lim_s = self.lim_s + DT_CTRL if abs(f.limit - self.lim_new) < 0.1 else 0.0
      self.lim_new = f.limit
      if self.lim_s >= LIMIT_STEADY - 1e-6 and abs(f.limit - self.hwy_limit[0]) > 0.1:
        self.hwy_limit = (f.limit, self.odo)
    else:
      self.lim_s = 0.0

  def _overturn(self, p, why):
    """A plan whose ramp was taken for sure turns out wrong (still on the highway, his decision): it ends, the speed comes
    back at once."""
    self._end(why, restore=True)

  def _exit(self, f, driver_speed, v_ego, on_link, signal, gas):
    """The plan's command: (speed, plan, curve speed, why, follower or None, firm). NO_CURVE when it asks for nothing."""
    p = self._nav_plan(f, on_link, signal)
    none = (NO_CURVE, NO_CURVE, NO_CURVE, "", None, False)
    if p is None:
      return none
    rem = p.exit_at - self.odo
    side = p.side
    # the shape check (the planner's): "link" only on consecutive answers (review: "no answer" frames between them kept
    # the count, a coin-flip heading gave "link" on the highway), "highway" past the exit point; the last answer given
    if f.exit_shape:
      p.seen_shape = True
      p.last_shape = f.exit_shape
      if rem < -QUIET_FROM:
        p.answered = True
    p.link_s = p.link_s + DT_CTRL if (f.exit_shape == 2 and rem <= EXIT_EARLY) else 0.0
    p.hwy_s = p.hwy_s + DT_CTRL if (f.exit_shape == 1 and rem < 0) else (0.0 if f.exit_shape == 2 else p.hwy_s)
    # the model path (ModelPath): "link" / "highway" on consecutive answers
    mv = self.model_v = self.model.verdict(f)
    p.model_link_s = p.model_link_s + DT_CTRL if (mv == 2 and rem <= EXIT_EARLY) else 0.0
    p.model_hwy_s = p.model_hwy_s + DT_CTRL if (mv == 1 and rem < 0) else 0.0
    if mv is not None:
      p.model_seen = True
    # (MODEL_DECIDES: the verdict and its counts the decisions below use; logged only when off)
    md = mv if MODEL_DECIDES else None
    m_hwy = p.model_hwy_s if MODEL_DECIDES else 0.0
    m_link = p.model_link_s if MODEL_DECIDES else 0.0
    # stayed on: the shapes say "highway" EXIT_STAY_TIME, or MODEL_STAY_SHAPES with the model path agreeing (the model
    # alone never: in a parallel exit lane before its gore the path follows the highway too)
    stayed = p.hwy_s >= EXIT_STAY_TIME - 1e-6 or (p.hwy_s >= MODEL_STAY_SHAPES - 1e-6 and m_hwy >= MODEL_STAY_SHAPES - 1e-6)
    shapes_link = (p.link_s >= EXIT_CONFIRM_TIME - 1e-6 and m_link >= MODEL_CONFIRM - 1e-6) if md is not None else \
      p.link_s >= EXIT_CONFIRM_SHAPE_ONLY - 1e-6
    # the road match on the link with GPS trusted (review: a 34 m wrong match confirmed after LINK_HOLD): LINK_CONFIRM;
    # the planner's ramp (mapOnRamp: the match on a ramp's chain, trust not low) counts the same, no sooner (final review:
    # its RAMP_HOLD confirmed and settled a 34 m wrong match with trust 0 at once, 78 -> 32 on the highway)
    p.match_s = p.match_s + DT_CTRL if ((on_link or f.on_ramp) and f.gps_trust < TRUST_LOW) else 0.0
    link_match = p.match_s >= LINK_CONFIRM - 1e-6
    # on the ramp for sure: matched to it, trusted, long enough; until then a confirmation can still be overturned
    if p.match_s >= LINK_SETTLE - 1e-6:
      p.settled = True
    # matched to the highway, trusted, past the split; with the shapes' last answer not "link": staying on (HWY_MATCH_STAY)
    hwy_match = f.road_class in HIGHWAY_CLASSES and not on_link and not f.on_ramp and f.gps_trust < TRUST_LOW
    p.hwy_match_s = p.hwy_match_s + DT_CTRL if (hwy_match and rem < 0) else 0.0
    not_link = p.last_shape != 2
    # his gas held (GAS_OVERTURN_*): his decision past the split whatever the shapes' last answer
    if gas and p.gas_s <= 0.0:
      p.gas_v0 = v_ego
    p.gas_s = p.gas_s + DT_CTRL if gas else 0.0
    gas_firm = gas and (p.gas_s >= GAS_OVERTURN_TIME - 1e-6 or v_ego > p.gas_v0 + GAS_OVERTURN_GAIN)
    stayed = stayed or (not_link and p.hwy_match_s >= HWY_MATCH_STAY - 1e-6 and rem < -EXIT_STAY_MARGIN)
    if not p.confirmed:
      if on_link and f.gps_trust < TRUST_LOW and p.match_s >= LINK_CONFIRM - 1e-6 and rem > EXIT_EARLY + NAV_SAME_MIN:
        self._end("an earlier exit")              # on a link well before this exit: its own ramp rules
        return none
      if rem <= EXIT_EARLY and (shapes_link or link_match):
        p.confirmed = True
        self.split_active = False
        self.split_follower.reset()
        r, at, adv = self._curve_now(f, p)
        if r > 0 and reach_needed(v_ego, map_curve_target(r, self.lat_accel, adv), decel=CHIME_MAX, lead=0.0) > at:
          self._chime("confirmed, too little room")
    near = rem <= DECIDE_NEAR
    if p.confirmed and not p.settled:
      # (review: after the confirmation nothing ended a false one) until the car is on the ramp for sure: positive highway
      # evidence (the shapes, the trusted match or the model path saying "highway"); his gas, the other signal or a lane
      # change away only while the shapes' last answer is not "link" (final review: on the real ramp a gas tap, a fork's
      # keep-left signal or a lane change on a 2-lane ramp restored the speed: E1 35-49 mph into its 32 mph curve)
      model_hwy = not_link and m_hwy >= EXIT_STAY_TIME - 1e-6
      why = ("still on the highway" if (stayed or model_hwy) else
             "gas" if gas_firm else
             "" if not not_link else
             "gas" if gas else
             "the other signal" if (signal and side and signal == -side) else
             "lane change away" if (side and self.lane.change == -side) else "")
      if why:
        self._overturn(p, "overturned: " + why)
        return none
    if p.confirmed:
      return self._ramp_command(f, p, driver_speed, v_ego, on_link)
    # his decision: the other signal, a lane change away from the exit side near it, his gas while we slow for it; past
    # the split only while the shapes' last answer is not "link" (final review: on the real ramp before the confirmation a
    # gas tap or a fork's keep-left signal took the tentative slowdown back, E2 44-46 mph into its 31 mph curve)
    decide = (near or p.slowing) if rem > 0 else not_link
    if signal:
      if side and signal == side and rem > 0:
        p.signalled, p.declined = True, False      # (a new signal toward it takes a decision back)
      elif side and signal == -side:
        p.signalled = False                        # (review: one the other way far out kept the old signal)
        if decide:
          # (re-review: far out it only ends the slowing; the plan takes new exit lane evidence again)
          p.declined = p.declined or not (rem > DECIDE_NEAR)
          self._end_slowing(p, restore=True)
      # (the exit's side unknown: a signal says nothing; the camera neither)
    if side and self.lane.change == -side:
      p.signalled = False                          # (final review: out of the exit lane, his old signal no longer counts)
      if decide and not p.declined:
        # while slowing it ends at any distance; farther than DECIDE_NEAR only the slowing (re-review of d4f848215: a
        # permanent decline there, then back into the exit lane, took E1 at 71 mph into its 32 mph curve)
        p.declined = not (rem > DECIDE_NEAR)
        self._end_slowing(p, restore=True)
    if gas and (decide or (rem <= 0 and gas_firm)) and not p.declined:
      p.declined = True
      self._end_slowing(p, restore=True)
    # missed: past the exit point the shapes say "highway" (fast restore); with no shape answer (an older planner, a GPS
    # pause) the plan only ends EXIT_PASS_MARGIN on, without a fast restore (review: that took 70 -> 80 on a ramp): the
    # set speed is held there (LOST) until the shapes or the road match say where we are
    if rem < 0 and stayed:
      self._end("missed", restore=True)
      return none
    if rem < -EXIT_PASS_MARGIN:
      if rem < -EXIT_LOST_MAX or p.last_cmd is None or p.declined:
        # (final review: adopted 20-50 mph under his speed) matched to a highway then (GPS trusted): his speed back
        # (the usual taps, no fast restore); else the held set speed becomes his
        # (re-review: GPS trusted and the shapes' last answer not "link": a doubtful match beside the ramp took 54 -> 75 on it)
        on_hwy = (f.road_class in HIGHWAY_CLASSES and not on_link and not f.on_ramp and f.gps_trust < TRUST_LOW and
                  p.last_shape != 2)
        self._end("not confirmed, on the highway" if on_hwy else "not confirmed",
                  adopt=p.last_cmd is not None and not p.declined and not on_hwy)
        return none
      return p.last_cmd, p.last_cmd, p.last_cmd, "exit lost", None, False
    if p.declined:
      return none
    cam = side != 0 and self.lane.edge[side]
    in_lane = p.signalled or cam
    if in_lane and 0 < rem <= DECIDE_NEAR:
      p.evidence = True
    # on the camera alone (no signal toward it): nothing before DECIDE_NEAR, and no lower than SPLIT_DECEL takes the
    # speed from where it first counts (cam_floor)
    cam_only = cam and not p.signalled
    if cam_only and 0 < rem <= DECIDE_NEAR and p.cam_floor is None:
      p.cam_floor = self._gentle(v_ego, rem)
    vs = self.split_speed(p)
    # slower than the split speed near it: kept from speeding up into the exit, never more than SPLIT_CAP_FLOOR under the
    # split speed (review: behind a truck at 45 until 290 m before, he stays on: 30 mph under the traffic), and only with
    # the highway's limit known
    if (rem > 0 and in_lane and vs > 0 and not p.v_cap and rem <= DECIDE_NEAR and v_ego + SPLIT_CAP_MARGIN < vs):
      p.v_cap = max(v_ego + SPLIT_CAP_MARGIN, vs - SPLIT_CAP_FLOOR * self.unit)
    if p.v_cap:
      vs = min(vs, p.v_cap)
    if rem > 0 and in_lane and p.curve[0] > 0 and self.split_mode != SPLIT_OFF:
      # a short ramp: the split speed that still makes its curve (never raising: only below what it would be)
      r, at, adv = p.curve
      v_c = map_curve_target(r, self.lat_accel, adv)
      base = vs if vs > 0 else driver_speed
      v_short = short_split_speed(at - p.exit_at, v_c, base)
      if v_short < base - 0.05:
        vs = v_short
    if cam_only and vs > 0:
      vs = max(vs, p.cam_floor) if p.cam_floor is not None else 0.0
    if rem > 0:
      # (on the camera alone from DECIDE_NEAR, as its slowdown: his signal may still come and allow a deeper one)
      # (drive 55: the camera's edge lane alone chimed 300 m before an exit he stayed past; the edge lane is also the
      # ordinary right lane) only his signal toward the exit chimes before the split; the confirmation's chime stays
      if p.signalled and not p.chimed and p.curve[0] > 0 and rem <= max(CHIME_AHEAD * v_ego, CHIME_AHEAD_MIN):
        # the speed at the split: the chosen one (slowed to it, or the cruise may still speed up to it; no lower than the
        # cruise can take it there, SPLIT_MAX_DECEL after its dead time), else the current one
        v_at = min(vs, driver_speed) if vs > 0 else v_ego
        v_at = max(v_at, (max(v_ego * v_ego - 2 * SPLIT_MAX_DECEL * max(rem - v_ego * CHIME_DEAD, 0.0), 0.0)) ** 0.5)
        if cam_only:
          v_at = max(v_at, p.cam_floor)
        r, at, adv = p.curve
        if exit_room_needed(v_at, map_curve_target(r, self.lat_accel, adv), decel=CHIME_MAX, lead=0.0) > at - p.exit_at:
          p.chimed = True
          self._chime("before the split")
      if p.evidence and not in_lane and not p.signalled and p.cam_floor is not None:
        # the camera alone, its edge lane gone before the split (drive 55, exit 1: the exit lane opened beside him, he
        # went on in the through lane; held 8 s, then the tentative slowdown past the split, 55 mph in 67 mph traffic):
        # the slowing ends (the usual restore, no fast one) and no tentative slowdown past the split on that evidence
        self._end_slowing(p)
        p.evidence = False
        return none
      if p.slowing and not in_lane:
        # the exit lane answer lost (review: a restore, then SET- again, churned 80 -> 72 -> 79 -> 76): the speed is held
        return self._cmd(p, p.hold, p.hold, p.hold, "exit hold", None)
      if vs <= 0 or not in_lane or driver_speed <= vs + START_GAP:
        return none
      plan = (vs * vs + 2 * SPLIT_DECEL * max(rem - v_ego * 1.5, 0.0)) ** 0.5
      if not self.split_active:
        self.split_active = plan <= driver_speed - MAP_START_GAP
      else:
        self.split_active = plan < driver_speed - END_GAP
      if not self.split_active:
        if p.slowing:
          return self._cmd(p, p.hold, p.hold, p.hold, "exit hold", None)
        return none
      p.slowing = True
      cmd = min(self.split_follower.update(v_ego, vs, rem, plan) if v_ego > 0 else plan, driver_speed)
      p.hold = cmd
      return self._cmd(p, cmd, plan, vs, "exit split", self.split_follower)
    # past the exit point, not confirmed yet: with exit lane evidence the ramp slowdown starts here, tentative (the set
    # speed at most TENT_GAP below the speed for TENT_MAX, longer only while the shapes' last answer is "link"; the
    # missed exit test above undoes it); else hold what the slowdown had
    if p.evidence:
      p.tentative = True
      p.tent_s += DT_CTRL
      if p.tent_v0 is None:
        p.tent_v0 = v_ego
      cmd, sp, v_c, why, fol, _ = self._ramp_command(f, p, driver_speed, v_ego, on_link)
      if cmd < NO_CURVE:
        # (no shape answer at all since QUIET_FROM past the split: an answer either way is followed as such); carried on
        # past TENT_MAX only while the model path does not say "highway", never past TENT_HARD_MAX nor TENT_DIP_MAX
        model_ok = m_hwy <= 0.0
        quiet = p.tent_s <= TENT_MAX_QUIET and not p.answered and model_ok
        more = model_ok and (p.link_s >= TENT_LINK - 1e-6 or m_link >= TENT_LINK - 1e-6)
        if (p.tent_s <= TENT_MAX or quiet or more) and p.tent_s <= TENT_HARD_MAX and v_ego > p.tent_v0 - TENT_DIP_MAX:
          cmd = max(cmd, v_ego - TENT_GAP)
          if p.hold is not None and p.slowing and p.tent_s <= TENT_MAX:
            cmd = min(cmd, p.hold)
        else:
          # no more slowing until it is confirmed: the set speed back to the speed (SET+ taps), never above his; on a
          # ramp matched as such (drive 54, exit 2: a SET+ 55 -> 57 on the real ramp just before its curve) none
          cmd = min(max(cmd, v_ego), driver_speed)
          if (on_link or f.on_ramp) and p.last_cmd is not None:
            cmd = min(cmd, p.last_cmd)
          return self._cmd(p, cmd, cmd, cmd, "exit wait", None)
        return self._cmd(p, cmd, sp, v_c, "exit tentative", fol)
    if p.hold is not None and p.slowing:
      return self._cmd(p, p.hold, p.hold, p.hold, "exit hold", None)
    return none

  @staticmethod
  def _gentle(v, rem):
    """m/s: the speed SPLIT_DECEL takes v to in rem m (after the cruise's ~1.5 s, as the split plan has it)."""
    return max(v * v - 2 * SPLIT_DECEL * max(rem - v * 1.5, 0.0), 0.0) ** 0.5

  def _cmd(self, p, cmd, plan, v_c, why, fol, firm=False):
    p.last_cmd = cmd
    return cmd, plan, v_c, why, fol, firm

  def _end_slowing(self, p, restore=False):
    """The slowdown before the split ends by his decision: the speed comes back at once."""
    if p.slowing and restore:
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
    if (ramp_curve and f.exit_fork in (0, 2, 3)) or carried_past:
      self._end("on the ramp")                    # the ramp's own curves from here
      return none
    if r <= 0 and (on_link or p.confirmed):
      # an exit without curves (drive 55, exit 2: it ended here on the link match before any confirmation, and the
      # restore took 64 -> 75 on the ramp): what the slowdown left is held until the ramp is confirmed, then it is his
      # speed (the street's rules from there); the missed exit test above still restores at once
      if p.confirmed and on_link:
        self._end("on the ramp, no curve", adopt=True)
        return none
      if p.confirmed and p.exit_at - self.odo < -EXIT_PASS_MARGIN:
        # (review of merge-f: confirmed by the shapes, the link never matched: held for good, no caps nor raises) far
        # enough past the split: what it reached is his (the shapes saying "highway" overturn it before, at once)
        self._end("confirmed, no link match", adopt=True)
        return none
      held = p.last_cmd if p.last_cmd is not None else p.hold
      return (held, held, held, "exit hold", None, False) if held is not None else none
    a, sp = self.nav.update(r, max(d, 0.0), driver_speed, adv)
    if not a:
      self.nav_follower.reset()
      return none
    v_c = map_curve_target(r, self.lat_accel, adv)
    if v_ego <= 0:
      return sp, sp, v_c, "nav exit", None, False
    cmd = self.nav_follower.update(v_ego, v_c, max(d, 0.0), sp)
    return cmd, sp, v_c, "nav exit", self.nav_follower, self.nav_follower.firm

  def update(self, f: MapFields, driver_speed, signal, v_ego=0.0, max_drop=0.0, gas=False, a_ego=None,
             set_speed=None, driver_set=False, limit_lower=0.0, limit_lower_gas=True):
    """signal: turn signal side, -1 left, 1 right, 0 none. v_ego: the true speed (m/s) for the followers (0: plan
    only). max_drop: the driver's Curve Max Drop in m/s (0: unknown, no floor in the plan). gas: the driver is on the
    gas (his decision near an exit). a_ego: the car's acceleration (m/s^2; None: the road follower runs open loop).
    set_speed: the cruise's set speed (m/s; None: the driver's speed), driver_set: he is on SET-/SET+ (the exit's
    street target: _exit_street). limit_lower: speed_limit_follow's street speed (m/s, 0 none: _limit_lower);
    limit_lower_gas: his gas held ends it (a lower limit's, rule 9; not rule 8's match)."""
    out = MapOutput()
    if a_ego is None:
      self.a_f = None
    else:
      self.a_f = a_ego if self.a_f is None else self.a_f + DT_CTRL / (MAP_ROAD_ACCEL_TAU + DT_CTRL) * (a_ego - self.a_f)
    self.odo += max(v_ego, 0.0) * DT_CTRL
    self.model.tick(DT_CTRL)
    if self.adopt_frames > 0:
      self.adopt_frames -= 1
      if self.adopt_frames == 0:
        self.adopt_target = 0.0
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
    self._exit_street_track(f, v_ego, on_link)
    if f.road_class and f.road_class not in LINK_CLASSES and not f.on_ramp:
      self.road_before, self.link_hwy = f.road_class, False
    elif (f.road_class in LINK_CLASSES or f.on_ramp) and (f.ramp_to_highway or f.road_class in HIGHWAY_LINK_CLASSES or
                                                          self.road_before in HIGHWAY_CLASSES):
      self.link_hwy = True
    if f.mode == OFF or driver_speed <= 0:
      if self.ll_on:
        self.ll_on, self.lower_event = False, "cancel: cruise off"
      if self.xs_on:
        # (review of merge-f: his brake mid-way, then RES brought back what it had reached, 60 for 50, and nothing took
        # it on: his highway 75 stayed the restore's target on the street) it goes on after his RES, no margin
        self.xs_on, self.xs_done = False, False
      self.curve.active = self.nav.active = False
      self.follower.reset()
      self.road_follower.reset()
      self.nav_follower.reset()
      if self.plan is not None:
        self._end("off")
      self.restore_frames = 0
      self.on_street_exit = f.on_ramp and self.on_street_exit
      self.ramp_last_cmd, self.ramp_last_frames = None, 0
      self.onramp_now = False
      return out
    out.link = on_link and self.link_hwy
    # only 1 and 2 are curves to act on: 4 (paused, GPS not trusted) and 5 (held, road match not trusted) are "no
    # curve" on purpose, so the speed comes back as for any curve that ends (a navigation exit plan is the exception)
    # (an on-ramp's first curve before the turn onto it, by navigation or his signal: as a ramp curve, mapOnRampAhead)
    onramp = f.status == STATUS_RAMP_CURVE and f.onramp_ahead and not f.on_ramp and f.road_class not in HIGHWAY_CLASSES
    radius = f.radius if f.status == STATUS_CURVE or (f.status == STATUS_RAMP_CURVE and (ramp_held or onramp)) else 0.0
    road_curve = f.status == STATUS_CURVE and not f.on_ramp
    # a road curve is never planned below the driver's Curve Max Drop (the controller would not go there anyway, and
    # planning for a lower speed only starts the slowdown earlier than that speed needs)
    floor = max(driver_speed - max_drop, 0.0) if (road_curve and max_drop > 0) else 0.0
    a, sp = self.curve.update(radius, f.distance, driver_speed, f.advisory, floor=floor,
                              lead_time=MAP_ROAD_LEAD_TIME if road_curve else MAP_LEAD_TIME)
    ramp_curve = a and f.status == STATUS_RAMP_CURVE and (f.on_ramp or onramp)
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
      speed = fol.update(v_ego, out.curve_speed, f.distance, sp, a_ego=self.a_f if fol is self.road_follower else None)
      track, settled, firm = True, fol.settled, fol.firm
      low = fol.low
    else:
      self.follower.reset()
      self.road_follower.reset()
      low = NO_CURVE
    ramp_hold = False
    # a ramp onto a highway: so mapped, an on-ramp's curve before the turn onto it, or a highway's link entered from a
    # street (not from a highway: an exit's link may end at a street's light, which he stops at)
    hwy_link = (f.on_ramp and f.ramp_to_highway) or onramp or \
      (f.road_class in HIGHWAY_LINK_CLASSES and self.road_before not in (0,) + HIGHWAY_CLASSES)
    if ramp_curve and track:
      self.ramp_last_cmd, self.ramp_last_curve = speed, out.curve_speed
      self.ramp_last_frames = int(RAMP_STATUS_HOLD / DT_CTRL)
      self.ramp_last_onramp = hwy_link
    elif self.ramp_last_cmd is not None:
      self.ramp_last_frames = max(self.ramp_last_frames - 1, 0)
      hwy_end = f.on_ramp and f.ramp_to_highway and 0.0 < f.ramp_end_distance <= RAMP_STATUS_HWY_RELEASE
      # (also a ramp curve matched again whose ramp is not held yet, RAMP_HOLD: drive 69 t 530.8-531.8)
      lost = f.status in (STATUS_PAUSED, STATUS_HELD) or (f.status == STATUS_RAMP_CURVE and radius <= 0.0)
      if not a and lost and (on_link or self.ramp_last_frames > 0) and not hwy_end:
        # (RAMP_STATUS_HOLD) its last command, or the set speed when that is above it: no raise, no further SET-
        ramp_hold = True
        speed = max(self.ramp_last_cmd, set_speed if set_speed is not None else 0.0)
        why, track, settled, firm = "ramp hold", True, False, False
        out.curve_speed, low = self.ramp_last_curve, speed
      elif not lost or not (on_link or self.ramp_last_frames > 0) or hwy_end:
        self.ramp_last_cmd, self.ramp_last_frames = None, 0   # the curve really ended (or the hold ran out)
    self.onramp_now = hwy_link or (ramp_hold and self.ramp_last_onramp)
    self._highway_limit(f, on_link)
    cmd, nplan, ncurve, nwhy, nfol, nfirm = self._exit(f, driver_speed, v_ego, on_link, int(signal), gas)
    na = cmd < NO_CURVE
    if na and cmd < speed:
      speed, why = cmd, nwhy
      # (a held command: tracked both ways, at its speed: the controller may take the set speed up to it)
      track = nfol is not None or nwhy in HOLD_WHYS
      settled = nfol.settled if nfol is not None else nwhy in HOLD_WHYS
      firm = nfirm
      out.plan, out.curve_speed = nplan, ncurve
      low = nfol.low if nfol is not None else cmd
    exit_to_street = f.on_ramp and ramp_held and f.ramp_type == RAMP_EXIT and not f.ramp_to_highway
    progress = self.ramp_end_first - f.ramp_end_distance >= STREET_PROGRESS
    street_exit = exit_to_street and progress and f.ramp_to_speed > 0
    capped = False
    street_to = f.ramp_to_speed + self.street_add       # (the Street Speed setting's offset)
    if street_exit and f.ramp_end_distance <= STREET_CAP_DISTANCE and street_to < driver_speed - START_GAP:
      if street_to < speed:
        speed, why, track, settled, firm = street_to, "street cap", False, False, False
      capped = True
    if exit_to_street and (a or na):
      self.street_last = speed
    elif not f.on_ramp:
      self.street_last = None
    if exit_to_street and progress and f.ramp_to_speed <= 0 and not (a or na):
      # the street's limit unknown: the speed the ramp's slowdown left (never restored on the ramp), and STREET_DEFAULT
      # near its end
      cap = NO_CURVE if self.street_last is None else self.street_last
      if f.ramp_end_distance <= STREET_CAP_DISTANCE:
        cap = min(cap, self.street_fallback)
      if cap < driver_speed - START_GAP:
        speed, why, track, settled, firm = cap, "street cap", False, False, False
        capped = True
    xs = self._exit_street(f, driver_speed, v_ego, set_speed, gas, driver_set)
    street = False
    if xs < NO_CURVE:
      xs_cmd, xs_firm = self._street_cmd(xs, driver_speed if set_speed is None else set_speed, v_ego)
      if xs_cmd < speed:
        speed, why, track, settled, firm = xs_cmd, "exit street", True, xs_cmd <= xs + 0.01, xs_firm
        capped, low, street = True, xs_cmd, True
    ll, ll_raise, ll_firm = self._limit_lower(limit_lower, driver_speed, v_ego, set_speed, gas, driver_set,
                                              gas_cancels=limit_lower_gas)
    if ll < NO_CURVE and ll < speed:
      speed, why, track, settled, firm = ll, "limit", True, ll_raise, ll_firm
      capped, low, street = True, ll, True
    out.active = a or na or capped or ramp_hold
    out.speed = speed
    out.track = track
    out.settled = track and settled
    out.firm = firm and track
    if ramp_curve or na or capped or ramp_hold:
      # ramps and exits may go past the Max Drop, to the ramp floor; a followed one's undershoot stays as for a road
      # curve (MAP_UNDERSHOOT below the curve's speed, FIRM_UNDERSHOOT when firm), never under the ramp floor
      out.floor = max(RAMP_FLOOR, low) if track else RAMP_FLOOR
      out.budget = RAMP_BUDGET
    elif track:
      # the follower may ask up to MAP_ROAD_UNDERSHOOT below the curve's speed (so at most that far below the max drop
      # floor, which the curve's speed never goes under): that is the floor the controller gets, and nothing lower
      out.floor, out.budget = out.curve_speed - MAP_ROAD_UNDERSHOOT, MAP_FOLLOW_BUDGET
    out.fast_restore = (f.on_ramp and ramp_held and f.ramp_to_highway and not out.active) or \
      (self.restore_frames > 0 and not out.active)
    out.adopt = (self.on_street_exit and not f.on_ramp and f.road_class not in HIGHWAY_CLASSES) or self.adopt_frames > 0
    # keep the lowered speed after an exit onto a street: known when the road after it is matched as not a freeway
    # (own road matching), otherwise only after driving along the exit
    self.on_street_exit = (exit_to_street and (progress or f.road_class != 0)) or (self.on_street_exit and f.on_ramp)
    out.why = why
    out.street = street
    out.adopt_target = self.adopt_target if self.adopt_frames > 0 else 0.0
    out.chime = self.chime_count != getattr(self, "_chimes_out", 0)
    self._chimes_out = self.chime_count
    return out

  def _exit_street_track(self, f, v_ego, on_link):
    """The exit to a street (the road classes, also while cruise is off): a link entered from a highway, then a street
    matched EXIT_STREET_STEADY with GPS trust ok; the street's limit and how long it has been reported unchanged."""
    dt = DT_CTRL
    if f.road_class in HIGHWAY_CLASSES and not f.on_ramp:
      self.xs_hwy_s += dt
      if self.xs_hwy_s >= HWY_RESET - 1e-6:
        self.xs_link, self.xs_exited, self.xs_street_s, self.xs_since = 0.0, False, 0.0, None
        self.xs_on, self.xs_done = False, False
      return
    self.xs_hwy_s = 0.0
    if f.road_class in LINK_CLASSES or f.on_ramp:
      if self.road_before in HIGHWAY_CLASSES and self.xs_since is None:
        self.xs_link += max(v_ego, 0.0) * dt
        self.xs_exited = self.xs_exited or self.xs_link >= EXIT_LINK_MIN
      self.xs_street_s = 0.0
      return
    if self.xs_since is not None:
      self.xs_since += dt
      if self.xs_since > EXIT_STREET_TIME and not self.xs_on and self.xs_done:
        # (review of merge-f: never while it is taking the set speed down, nor before it could: a queue counted the
        # 60 s while the taps waited for 25 mph, and the restore went back to his highway 75)
        self.xs_since, self.xs_exited, self.xs_on = None, False, False
    if not f.road_class:
      return                       # (unknown: nothing counts, nothing resets)
    self.xs_lim_s = self.xs_lim_s + dt if abs(f.limit - self.xs_lim) < 0.1 else 0.0
    self.xs_lim = f.limit
    if not self.xs_exited or self.xs_since is not None:
      return
    # (review of merge-f: a wrong street match beside a congested highway at trust 1) GPS trust 0 only
    self.xs_street_s = self.xs_street_s + dt if f.gps_trust == 0 else 0.0
    if self.xs_street_s >= EXIT_STREET_STEADY - 1e-6:
      self.xs_since, self.xs_done = 0.0, False      # the exit's street: its target applies (_exit_street)
      if self.plan is not None:
        self._end("on a street")

  def street_target(self):
    """m/s: the exit's street target now (the street's steady limit plus the setting's offset, else STREET_FALLBACK),
    NO_CURVE while a limit is reported but not steady yet, or not a street's."""
    if self.xs_lim <= 0:
      # (review of merge-f: a one-frame dropout of the street's limit took the target 50 -> 43 -> 50) "no limit" itself
      # steady EXIT_STREET_LIMIT_STEADY first
      return self.street_fallback if self.xs_lim_s >= EXIT_STREET_LIMIT_STEADY - 1e-6 else NO_CURVE
    if self.xs_lim_s < EXIT_STREET_LIMIT_STEADY - 1e-6 or self.xs_lim > self.street_limit_max + 0.1:
      return NO_CURVE
    return self.xs_lim + self.street_add

  def _exit_street(self, f, driver_speed, v_ego, set_speed, gas, driver_set):
    """The exit's last target on the street (m/s, NO_CURVE: none): the set speed is taken to it (down by SET- taps, or a
    pending restore only up to it), then it becomes his speed (adopt)."""
    if self.xs_since is None:
      self.xs_on = False
      return NO_CURVE
    set_v = driver_speed if set_speed is None else set_speed
    tgt = self.street_target()
    if not self.xs_on:
      if tgt >= NO_CURVE:
        return NO_CURVE                            # (its limit not steady yet: wait for it)
      if self.xs_done or v_ego >= EXIT_STREET_MAX_V:
        return NO_CURVE
      self.xs_done = True
      if set_v > tgt + EXIT_STREET_DONE or driver_speed > tgt + EXIT_STREET_DONE:
        self.xs_on, self.xs_target = True, tgt
      else:
        return NO_CURVE                            # (his speed already at or under it: his)
    if tgt < NO_CURVE:
      self.xs_target = tgt                         # (its limit became known meanwhile)
    # (second review: his gas at a launch ended it and adopted his highway 75 on the street) his gas does not end it
    # (nothing is pressed while he is on it); his SET buttons do. Adopted only where it governs: the set speed at it
    ended = self._street_end(self.xs_target, set_v, driver_speed, driver_set)
    if ended:
      self.xs_on = False
      return NO_CURVE
    return self.xs_target

  def _street_end(self, target, set_v, driver_speed, driver_set, gas_why=""):
    """'' while the street speed goes on, else why it ended; adopted (his speed) only when the set speed is at it
    (second review: a stop's or a curve's lowered set speed was made his 19 on his gas), for ADOPT_LANDED_FRAMES only:
    this frame and the next (our press landing this frame is counted by the controller in it, after the adopt)."""
    at = abs(set_v - target) <= EXIT_STREET_DONE or set_v <= self.street_floor + EXIT_STREET_DONE and target <= self.street_floor
    why = ("cancel: his SET" if driver_set else gas_why if gas_why else "done" if at else
           "his speed under it" if (set_v < target and driver_speed <= target + EXIT_STREET_DONE) else "")
    if why and (at or why == "cancel: his SET" or (gas_why and set_v >= target - EXIT_STREET_DONE)):
      self.adopt_frames = max(self.adopt_frames, ADOPT_LANDED_FRAMES)
      self.adopt_target = set_v if at else 0.0
    return why

  def _street_cmd(self, target, set_v, v_ego):
    """(command, firm) toward a street speed: paced (LOWER_GAP under the car's speed), or one SET- hold toward
    max(target, speed + LOWER_HOLD_ABOVE) when the set speed is LOWER_HOLD_MIN above that."""
    above = max(target, v_ego + LOWER_HOLD_ABOVE)
    if set_v - above >= LOWER_HOLD_MIN:
      return above, True
    return max(target, v_ego - LOWER_GAP), False

  def _limit_lower(self, target, driver_speed, v_ego, set_speed, gas, driver_set, gas_cancels=True):
    """(m/s command, raise ok, firm) for speed_limit_follow's street speed (rules 8 / 9), NO_CURVE: none. Paced
    (_street_cmd); done (his speed) at it; his SET buttons end it, his gas held only for a lower limit (gas_cancels:
    rule 9; rule 8's match waits his gas out). Never under the controller's floor (street_floor)."""
    if target <= 0:
      self.ll_on = False
      return NO_CURVE, False, False
    target = max(target, self.street_floor)
    set_v = driver_speed if set_speed is None else set_speed
    if not self.ll_on:
      self.ll_on, self.ll_gas_s = True, 0.0
    if gas and self.ll_gas_s <= 0.0:
      self.ll_gas_v0 = v_ego
    self.ll_gas_s = self.ll_gas_s + DT_CTRL if gas else 0.0
    gas_why = "cancel: his gas" if (gas_cancels and gas and (self.ll_gas_s >= GAS_OVERTURN_TIME - 1e-6 or
                                                             v_ego > self.ll_gas_v0 + GAS_OVERTURN_GAIN)) else ""
    why = self._street_end(target, set_v, driver_speed, driver_set, gas_why)
    if why:
      self.ll_on, self.lower_event = False, why
      return NO_CURVE, False, False
    cmd, firm = self._street_cmd(target, set_v, v_ego)
    return cmd, cmd <= target + 0.01, firm

  def state(self):
    """The exit plan's state for the logs."""
    p = self.plan
    if p is None:
      return ""
    if p.confirmed:
      return "confirmed"
    if p.declined:
      return "declined"
    if (p.exit_at - self.odo) <= 0:
      return "tentative" if p.tentative else "split hold"
    return "slowing" if p.slowing else "planned"


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
