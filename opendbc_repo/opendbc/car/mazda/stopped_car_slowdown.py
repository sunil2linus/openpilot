"""
Stopped car ahead, or a stop ahead with a much slower car: lower the stock cruise set speed early (Mazda Gen1,
stock MRCC).

The stock radar cruise often brakes late, or not at all, for a car that is already stopped.
When openpilot's vision lead is a stopped car in our lane, this asks the shared set speed
controller (curve_slowdown.CurveSlowdownLogic) to lower the set speed towards a target that
lets a gentle cruise slowdown reach the floor speed about 15 m behind the lead, so the car
approaches slower and the MRCC radar has time to react. It never stops the car: the MRCC or
the driver does the final braking.

Floor mode (drive 00000043, Nov 4): the car slows only ~0.5-0.9 m/s^2 while the set speed is 3-15 mph below its speed,
but ~1.6 m/s^2 once the gap is 18 mph or more, so tapping down along the distance target was too slow (the driver
braked at 28-40 mph three times). Once a stop is confirmed and the car ahead is within FLOOR_DISTANCE, the target
goes straight to the floor (SET- holds), never below where the car ahead will be in FLOOR_LEAD_HORIZON s.

Two ways in:
  * "stopped car": a confident vision lead in our lane that is stopped and that we are closing in on.
  * "stop with car": the driving model wants to slow down (red light / stop ahead: its desired acceleration is at
    most MODEL_SLOW_ACCEL) and there is a car ahead that is stopped or much slower than us (STOP_CAR_*). The model
    alone (no car) only warns (stop_warning.py); with a car following at our speed the model slows all the time just
    to keep the gap, so the car ahead must be much slower. Checked on drives 31/33/35/36: this fired 17 times, every
    time the car really came to a stop within 12 s, 0.5-4 s before the driver braked; never while just following.
    It is released when traffic moves again (the model stops slowing and the car ahead is no longer much slower).
  * "stop ahead": the planned stop alone (the red light warning's check), the floor hold at once (WARN_*); without the
    stop-light detector (starpilotPlan.redLight) backing it, 1 mph taps until it is confirmed (WARN_GUARD_*).
  * "red light (head start)": the stop-light detector alone, on for 2 s: 1 mph taps, at most 5 mph, until one of the
    above takes over or the light goes (RED_*). "slow car (unsure)": a low-confidence slower car, the same taps (UNSURE_*).
    Both from 15 mph; with the cruise still accelerating (set speed well above ours) the set speed first comes down to
    our speed, and below 25 mph the 5 mph are counted from there (CATCH_UP_*; from 25 mph never more than 5 below the
    set speed). Below 25 mph the unsure car also needs the car ahead not accelerating (UNSURE_LOW_*).

This module holds the triggers (conservative, to avoid phantom slowdowns), the target and the
urgency; it has no openpilot dependencies so it can be unit tested. Settings
(MazdaStoppedCarSlowdown): 0 off, 1 test (no presses, logs the would-be timeline), 2 on.
"""
import math
from dataclasses import dataclass

DT_CTRL = 0.01
MS_PER_MPH = 0.44704

OFF, TEST, ON = 0, 1, 2

MIN_PROB = 0.85
MAX_LATERAL = 1.0           # m: |yRel| of the lead
STOPPED_SPEED = 2.0         # m/s: lead slower than this counts as stopped
MIN_CLOSING = 3.0           # m/s
HOLD_TIME = 0.5             # s the condition must hold before the slowdown starts
START_SPEED = 20 * MS_PER_MPH
CRUISE_SETTLE = 0.3         # s
RELEASE_LEAD_SPEED = 4.0    # m/s: the lead moved away (vision), held for RELEASE_MOVING_TIME
RELEASE_MOVING_TIME = 0.5   # s
RELEASE_LOST_TIME = 2.0     # s without a stopped vision lead in our lane before the reason clears
LOST_MIN_PROB = 0.5         # a vision lead below this no longer counts as "still there"
EGO_STOPPED_SPEED = 1.0     # m/s: we have stopped behind the lead: the driver's set speed can come back (once moving)
# Stock radar (CRZ_CTRL): RADAR_HAS_LEAD and RADAR_LEAD_RELATIVE_DISTANCE, a bucket 1 = FARTHEST .. 5 = NEAREST
# (0 = no lead). It is logged, and its pick-up times are kept, but it never keeps the reason active and never
# releases it: in traffic it almost always reports some lead, and a dropped stationary target must not raise the
# set speed while we are still approaching.
# The floor is the car's own minimum set speed, 30 km/h (18.6 mph, the dash shows 19) (owner, Nov 6: "when we drop the
# speed by holding the button it's only going down to 20, let's do one more 1 mph step down to 19, every mph
# matters"): the long press stops at 20 and one SET- tap takes it to the minimum (curve_slowdown.STOP_MIN_SET_SPEED).
FLOOR_SPEED = 30 / 3.6
TARGET_DECEL = 1.0          # m/s^2 the stock cruise slows at once the set speed is lowered
TARGET_MARGIN = 15.0        # m behind the lead where the floor speed should be reached
FLOOR_MODE_ENABLED = True
FLOOR_DISTANCE = 130.0      # m: a confirmed stop this close: target = floor (SET- holds), not the distance target
FLOOR_LEAD_HORIZON = 2.0    # s: ... but not below the car ahead's speed this far ahead (a slower car that keeps moving)
# drive 45: the guard made two holds with a pause; the owner wants one hold to 20 mph (the radar keeps the gap if the
# car ahead keeps moving, and traffic moving again restores)
FLOOR_LEAD_GUARD = False
# A slower car that keeps moving (drive 4d, tag 4, t 2267-2276): a car 100-125 m ahead slowed from ~42 to a steady
# ~31 mph; the model's plan dipped with it (a "planned stop", latched) and an early start went to the floor: one hold
# 45 -> 20, the car ahead never stopped, and the driver pressed the gas and brought the set speed back himself. Now
# while the car ahead is clearly moving (MOVING_LEAD_SPEED or more) and not braking hard (aLeadK filtered over
# MOVING_LEAD_TAU above MOVING_LEAD_DECEL), the target is its speed less MOVING_LEAD_MARGIN; once it slows below that
# speed or brakes harder, it is the floor for the rest of the episode (a stop: one hold, drive 45). Stops seen so far
# (drive 4d t 547, 619, 838, 1809) braked at -0.9..-1.7; tag 4 at -0.1..-0.3 once steady. The target only goes down
# during an episode, so a hold is never aborted by it.
MOVING_LEAD_GUARD = True
MOVING_LEAD_SPEED = 20 * MS_PER_MPH
MOVING_LEAD_MARGIN = 5 * MS_PER_MPH
MOVING_LEAD_DECEL = -0.8     # m/s^2 (radarState aLeadK, filtered): braking harder than this is a stop
MOVING_LEAD_TAU = 1.0        # s
# ... and when we are at that moving car's speed (within MOVING_MATCH_CLOSING for MOVING_MATCH_TIME, the car not
# braking), the episode releases and the speed comes back with the hold (owner: the car ahead keeps moving, bring it
# back fast; the stock radar keeps the gap once it picks the car up, the set speed only caps it)
MOVING_MATCH_CLOSING = 2.0   # m/s
MOVING_MATCH_TIME = 2.0      # s
# for the release the car must be steady (tag 4 once steady: -0.1..-0.3), and no stop planned (a queue car coasting
# to a light at -0.5 is not "moving on")
MOVING_MATCH_DECEL = -0.3    # m/s^2
# The driver on the gas during an episode (drive 4d, tags 3/4): they are overriding the slowdown; after
# GAS_RELEASE_TIME it is released (a "clear" release: fast restore when no slower car is in our lane).
GAS_RELEASE_TIME = 1.0       # s
# ... and that car does not start it again (until it stops or slows OVERRIDE_SLOWER below its speed at the release,
# another car is the lead, or the gas has been off OVERRIDE_REARM); nothing starts on the gas
OVERRIDE_SLOWER = 3.0        # m/s
OVERRIDE_REARM = 5.0         # s
# The car ahead pulling away after a stop (drive 4d, t 2017-2022): the restore may go at any speed once no car is
# ahead or the car ahead is at least PULL_AWAY faster than us (not while creeping in a queue at its speed, nor while
# still rolling up to a stopped car)
PULL_AWAY = 1.0              # m/s
PULL_AWAY_TIME = 0.5         # s it must hold (a one-frame vision dropout is not "no car ahead")
# The car ahead pulling away (owner, Nov 6, drive 4f tag 6, t 796-803: we slowed behind a car at a light, the radar
# followed it; it crept at 2-5 mph from 798.5, then pulled away at 0.9-1.5 m/s^2 from 800.8, at our speed; our
# restore only landed at 802.5, after the radar match release's 2 s): a car ahead at about our speed or faster that
# accelerates at PULL_AWAY_ACCEL for PULL_AWAY_ACCEL_TIME, no stop planned, releases at once and the set speed goes
# back up with holds. The bigger the gap from our speed to the set speed, the harder the MRCC accelerates (drives
# 4c/4d/4f, no car ahead: median 0.33 m/s^2 at a 0-5 mph gap, 0.75 at 10-15, 1.06 at 20-25).
PULL_AWAY_ACCEL_ENABLED = True
PULL_AWAY_ACCEL = 0.8        # m/s^2 (radarState leadOne.aLeadK)
PULL_AWAY_ACCEL_TIME = 0.5   # s (tag 6: from 800.8 the release at ~801.3, the restore ~1.2 s sooner)
# the car ahead at least our speed less PULL_AWAY_SLOWER, the radar following it; without the radar it must be
# PULL_AWAY faster than us (review: an untracked car that may stop again)
PULL_AWAY_SLOWER = 0.5       # m/s
PULL_AWAY_MIN_SPEED = 2.0    # m/s: ... and moving
PULL_AWAY_SAME_CAR = 1.0     # s the same car (no jump of PULL_AWAY_JUMP in its distance, review: a cut-in) ...
PULL_AWAY_JUMP = 4.0         # m
# Also needed (review): the camera sure of it (MIN_PROB), the model not slowing (its desired acceleration above
# GO_ACCEL; tag 6: -0.09 at 800.6, +0.31 at 801.1) and no stop planned.
PULLING_AWAY = "car ahead pulling away"
# ... and that car does not start it again (until it stops, or another car is the lead); nothing starts on the gas
# The car ahead's speed (drive 4b, T1): the vision lead's vLead jumped 18 / 28 / 42 mph frame to frame while the car
# ahead was really doing ~40, and one slow frame started a "stop with car" (45 -> 30). It is filtered: the median over
# VLEAD_WINDOW, the highest value while fewer than VLEAD_MIN_FILL s are in the window (a short window can never look
# slow); a distance jump of LEAD_JUMP m is a new car and starts the window again.
VLEAD_WINDOW = 0.5          # s
VLEAD_MIN_FILL = 0.3        # s
LEAD_JUMP = 20.0            # m
# The stock radar following the car ahead (drive 4b, T5-T7; owner's rules): when the radar was seen tracking the car
# while it was still moving, the MRCC brakes for it itself (drive 44: radar tracked -> the MRCC stopped by itself;
# not tracked / flickering -> the driver braked at -4.6 m/s^2). Then we stop lowering and leave the set speed
# alone (no SET-, no SET+) while it handles the car ahead: we have matched its speed, or the car is braking
# (aEgo <= RADAR_BRAKING_ACCEL), or no hard braking is needed yet (required < RADAR_YIELD_DECEL). Once the MRCC has
# matched the car (within RADAR_FOLLOW_CLOSING for RADAR_RESTORE_TIME), the episode releases and the speed goes back
# up with SET+ holds; the radar keeps the gap, the set speed only caps it. A stopped car the radar reports without
# having tracked it moving is not trusted (the dangerous case). A radar dropout of more than RADAR_DROPOUT (drive 4b:
# the radar dropped a car at 42 m for 1 s) ends the follow at once (lowering resumes, floor hold) and, until the
# episode ends, the radar must then be on RADAR_REACQUIRE_TIME continuously to count again.
RADAR_FOLLOW_ENABLED = True
RADAR_TRACK_TIME = 1.0      # s of RADAR_HAS_LEAD (gaps up to RADAR_DROPOUT) before the radar counts as tracking
RADAR_DROPOUT = 0.3         # s
RADAR_REACQUIRE_TIME = 2.0  # s, after a dropout ended a follow
RADAR_FOLLOW_CLOSING = 3.0  # m/s: we are at the car's speed (within this)
RADAR_BRAKING_ACCEL = -0.8  # m/s^2: the MRCC is braking (more than a lowered set speed alone gives, ~0.9 at most)
RADAR_YIELD_DECEL = 1.0     # m/s^2: required deceleration below this: no urgency yet, leave it to the MRCC
# The match release also needs the car ahead not braking harder than RADAR_MATCH_LEAD_DECEL and the model no longer
# slowing / planning a stop: a car stopping at a light is followed down to the stop (owner: "wait for our car to come
# to a stop"), then the ego-stopped restore.
RADAR_RESTORE_TIME = 2.0    # s at the car's speed with the radar following before the release
RADAR_MATCH_LEAD_DECEL = 1.0  # m/s^2
URGENT_TTC = 5.0            # s
URGENT_DECEL = 1.5          # m/s^2: closing^2 / (2 (dRel - 15))

# Stop ahead with a much slower car (red light + a car stopped or slowing for it)
MODEL_SLOW_ACCEL = -0.4     # m/s^2: the model's desired acceleration at most this = it wants to slow down
STOP_CAR_MIN_PROB = 0.4     # the car ahead is often first seen at ~100 m with a low probability
STOP_CAR_MAX_LATERAL = 2.0  # m: |yRel| (a queue far ahead is not centred yet)
STOP_CAR_MAX_DISTANCE = 150.0
STOP_CAR_SLOWER = 5.0       # m/s (11 mph): the car ahead is at least this much slower than us ...
STOP_CAR_RATIO = 0.5        # ... or at most this fraction of our speed
GO_ACCEL = 0.0              # m/s^2: the model no longer slows down (traffic moving / green)
GO_SLOWER = 2.0             # m/s: the car ahead is no longer much slower than us
GO_TIME = 1.0               # s both must hold before the stop with a car is released
# ... or, without the model's go (drive 6c t 762-784: the light green, the floor 19 held 11.5 s while the car went 46 ->
# 22.5 mph behind a car at 25-34 mph 65-125 m ahead; the model's desired acceleration stayed <= 0 as we slowed; drive 6d t
# 222-231): with the floor committed (the moving car's release, MOVING_MATCH_*, is off then) no planned stop and the
# stop-light detector off for GO_MOVING_CLEAR, the car ahead moving (filtered, at least GO_MOVING_LEAD, not braking:
# MOVING_MATCH_DECEL) and at most GO_MOVING_CLOSING slower than us, for GO_TIME
GO_MOVING_ENABLED = True
GO_MOVING_CLEAR = 1.0       # s
GO_MOVING_LEAD = 5.0        # m/s
GO_MOVING_CLOSING = 2.5     # m/s
TRAFFIC_MOVING = "traffic moving again (model no longer slowing, car ahead not much slower)"
CAR_AHEAD_MOVING = "car ahead moving again (no stop planned, no red light)"
# The stop with car gets the stop ahead's red light protection (drive 6a t 584.9: released "traffic moving again" with
# the light on, the plan blinking off under it): while the light is on (STOP_AHEAD_RED_HOLD / STOP_AHEAD_RED_HOLD_MAX)
# no go / moving car / pulling away release, and such a release with the light on keeps the lowered set speed until it
# has gone (RED_RELEASE_HOLD_MAX, as a stop ahead's)
STOP_CAR_RED_HOLD = True
STOPPED_CAR, STOP_WITH_CAR = "stopped car", "stop with car"

# Red light first (drive 2026-11-03 21:12): the stop-ahead warning fired 5 s before a hard brake, but the queue car
# ahead was only 25-37 % "a car" to the model at 110-130 m (the radar saw it at 11 m), below STOP_CAR_MIN_PROB.
# Once the model has planned a stop (the same check as the planner's red light warning, stop_warning.plan_slowdown),
# a low-confidence stopped or much slower car near our lane starts the slowdown (taps only until it is confirmed).
EARLY_START_ENABLED = True
PLAN_STOP_RATIO = 0.70      # planned minimum speed <= this share of ours ... (stop_warning.STOP_SLOWDOWN)
PLAN_STOP_HORIZON = 8.0     # ... within this many seconds of the plan (STOP_HORIZON)
PLAN_STOP_POINTS = 2        # ... on this many consecutive plan points (STOP_HOLD_POINTS)
PLAN_STOP_DECEL = 0.9       # m/s^2 implied to get there (STOP_DECEL)
PLAN_STOP_BUFFER = 2.0      # m (STOP_BUFFER)
LATCH_DEBOUNCE = 0.5        # s of the planned stop before the latch sets ...
LATCH_DROPOUT = 0.25        # ... gaps up to this long allowed
LATCH_HOLD = 4.0            # s the latch stays after the plan last showed the stop (~80 m at 45 mph)
STOP_CAR_EARLY_MIN_PROB = 0.2   # the car ahead while latched (11 such cases on 2026-11-03: the driver braked in 10)
EARLY_HOLD = 0.5            # s (cumulative) of the early condition before the start ...
EARLY_DROPOUT = 0.3         # ... the lead may blink out this long (it is intermittent at 110 m)
EARLY_CONFIRM_PROB = 0.4    # an early start is confirmed (holds, fast pace) once the car is this sure ...
EARLY_CONFIRM_TIME = 0.3    # ... for this long, or the stop is urgent
RELEASE_LOST_TIME_LATCHED = 5.0  # s without the car before "lead lost" while the plan still shows a stop

# A car the camera is not sure of yet (owner, Nov 5: "when we see the yellow triangle let's start slowing down 1 mph at
# a time until the confidence score reaches 0.85, this way we are not slowing down too much but have a head start").
# The UI draws the lead triangle for any vision lead (radarState.leadOne.status), at 0.25-0.40 confidence at ~120 m;
# the stopped car start needs MIN_PROB (0.85), which came 1.2-5.8 s later (drives 4c / 4d: 4c t 584, the confidence
# stayed 0.25-0.32 all the way, our slowdown 2.4 s and the driver's brake 3.5 s after the triangle). Drives 4c, 4d and
# 44: 19 such starts, 15 followed by a stop or the driver braking within 12 s, 4 slower moving cars the MRCC followed.
# SET- taps only (1 mph each, the relaxed pace), at most UNSURE_MAX_DROP below our speed at the start; the stopped car
# / stop with car / stop ahead starts take the episode over as soon as they hold (confidence, planned stop).
UNSURE_ENABLED = True
UNSURE_LATERAL = 1.0         # m: |lead offset from our path|
UNSURE_MAX_DISTANCE = 130.0  # m
UNSURE_SLOWER = 4.5          # m/s (10 mph): the car is at least this much slower than us (filtered)
# drive 52 (t 188-191, night, a queue at a light): a slow car in our lane at 77-87 m, 9-16 mph, 55-76 % a car, while we
# drove 23.5-24.4 mph and the cruise was still accelerating (set 35): the old 25 mph minimum kept the unsure car out
UNSURE_MIN_SPEED = 15 * MS_PER_MPH
UNSURE_HOLD = 0.5            # s (cumulative, EARLY_DROPOUT gaps) before the taps start
UNSURE_MAX_DROP = 5 * MS_PER_MPH
# out of our path or no longer much slower (and not stopping) this long: released, the speed back; not seen:
# RELEASE_LOST_TIME, the speed not back at once (a far low-confidence lead blinks)
UNSURE_LOST_TIME = 1.0       # s
UNSURE_SURE_TIME = 1.0       # s sure (MIN_PROB) without a stop start taking over: a slower moving car, the MRCC's
UNSURE_REARM = 3.0           # s after an unsure release before another
# Below CATCH_UP_LOW_SPEED the unsure car also needs the car ahead not accelerating, for its start and while it runs
# (review a2: pulling away from a light at 16 mph under a 45 set, a low-confidence car 40-60 m ahead accelerating
# from 2-6 mph: the first drop took the set 45 -> 19, the car ~13 mph slower 10 s later than without it). Drive 52's
# queue car (t 187.9-193.9, why the minimum is 15 mph): aLeadK -0.19..-0.60 throughout. Once the camera is sure of the
# car, the pull-away release (PULL_AWAY_ACCEL_*) ends such an episode too.
UNSURE_LOW_MAX_ACCEL = 0.3   # m/s^2: radarState aLeadK, filtered over UNSURE_LOW_ACCEL_TAU from the car's first frame
UNSURE_LOW_ACCEL_TAU = 0.5   # s
SLOW_UNSURE = "slow car (unsure)"
UNSURE_GONE = "unsure car gone"
UNSURE_NOT_STOP = "car confirmed, not a stop"

# Restore after a release where the car may still be ahead (drive 4c, tag 3 / t 778): cruise re-engaged 2 s after the
# driver braked at a red light; the only car ahead was 79 m away at 36 mph against our 27, yet the set speed came back
# 35 -> 45 by 0.5 s taps (the "cruise off / brake" release restores gently). Now such a release (CLEAR_RELEASE_REASONS)
# restores with the usual SET+ hold rule whenever no slower car is in our lane (clear_ahead, checked every frame); a
# "lead lost (vision)" release stays gentle: there "no lead" is the dropout itself.
CLEAR_RELEASE_REASONS = frozenset({"cruise off / brake", "openpilot disengaged", "lead left the lane", "lead leaving the lane",
                                   "driver on the gas"})
CLEAR_LATERAL = MAX_LATERAL         # m: |lead offset from our path| within this is "in our lane" for the restore check
CLEAR_MIN_PROB = STOP_CAR_MIN_PROB  # a car sure enough to start a stop with car keeps the restore gentle
CLEAR_SLOWER = MIN_CLOSING          # m/s: a car at least this much slower than us (or stopped) keeps the restore gentle

# A car ahead moving out of our lane (drive 4c, tag 2, t 736-740): a white SUV slowing for a red light, centred
# (yRel -0.15..-0.34 m at 65-100 m, prob 0.87-0.96) when the stop with car started and held 45 -> 35, then drifting
# into a right-turn lane: -0.82 (738), -0.95 (739), -1.68 (740); the driver braked at 740 while we still held, since
# "still there" allows |yRel| <= 2.0 m and the lost release takes 2-5 s. Now the episode is released once the car's
# offset from the model path is LEAVE_Y_MIN or more, has grown LEAVE_DRIFT from its smallest value in this episode,
# and the raw yRel agrees (so a curve, which bends the path too, does not count), for LEAVE_TIME. A lead switch
# (LEAD_JUMP) restarts the baseline. The release is a "clear" one: the SET- hold stops at once and the restore uses
# SET+ holds unless another slower car is in our lane.
LEAVE_ENABLED = True
LEAVE_Y_MIN = 0.8           # m: tag 2's first clearly-off reading was 0.82; centred readings were 0.15-0.34
LEAVE_DRIFT = 0.4           # m grown since the episode's smallest |offset| (0.82 - 0.34 = 0.48 at the worst baseline)
LEAVE_TIME = 0.5            # s
LEAVE_TAU = 0.2             # s: low-pass on the path offset
LEAVE_MAX_DISTANCE = 120.0  # m: beyond this the lead's lateral position is too noisy to call (tag 2: 65-100 m)
LEAVE_MIN_PROB = LOST_MIN_PROB
# a car released as leaving does not start a slowdown again until its offset is below LEAVE_Y_MIN - LEAVE_BACK (it
# came back) or another car is the lead
LEAVE_BACK = 0.5            # m

# Red light, no car seen yet (drive 4c, tag 4, 45 mph): the planner's STOP AHEAD warning fired at t 1654.0, the
# camera's first lead (prob 0.40, 113 m, 27 mph) at 1654.8, our first SET- tap at 1655.2 (early start: taps until
# confirmed), the hold only at 1656.2, 20 mph at 1658.8; the driver braked at 1659.2 (-4.4 m/s^2). The owner wants
# the lowering to start on the warning itself, with the long press at once. Evidence for the warning: 6 of 6 this
# drive and 17 of 17 on earlier drives were followed by the driver braking within 0.8-5 s; starpilotPlan.redLight
# (FrogPilot's flag) fired 8 times at speed with no stop and is not used. The warning's gates are replicated here
# (stop_warning.py: STOP_DEBOUNCE / STOP_DROPOUT / STOP_MIN_SPEED / NOT_SLOWING_ACCEL / ACCEL_FILTER_TAU /
# LANE_CHANGE_HOLD / REARM_TIME and lead_is_relevant) rather than read from starpilotOnroadEvents: that event only
# exists while StopWarningRedLight is On, is a one-frame pulse re-published at 1 Hz, and the plan rule itself is
# already replicated (plan_stop).
WARNING_START_ENABLED = True
WARN_DEBOUNCE = 1.0            # s the warning condition must hold for the warning to show (STOP_DEBOUNCE) ...
# ... but the lowering starts sooner (owner, Nov 5: "first start reducing the speed and then display the warning,
# every second lost is valuable"; drive 4d: the warning and our start were 14 ms apart). Drive 44 replayed (27 min, 8
# times the plan said stop): a 0.5 s start adds one start that never became a warning, and the driver braked for
# that one 2.1 s later.
WARN_START_DEBOUNCE = 0.5      # s
# A start before the warning would have shown is released (and the set speed restored) when the planned stop goes
# away before WARN_DEBOUNCE (the plan dipped for a moment): a dip that never became a warning costs ~0.5 s of SET-,
# not a hold until the stop "clears" (review: 1.6-4.6 s, 5-15 mph).
STOP_AHEAD_UNCONFIRMED = "stop ahead gone before the warning"
WARN_DROPOUT = 0.25            # s (STOP_DROPOUT)
WARN_MIN_SPEED = 25 * MS_PER_MPH   # (STOP_MIN_SPEED)
WARN_NOT_SLOWING_ACCEL = -0.5  # m/s^2 on the filtered aEgo: the MRCC / driver is not already slowing (NOT_SLOWING_ACCEL)
WARN_ACCEL_TAU = 0.5           # s (ACCEL_FILTER_TAU)
WARN_LANE_CHANGE_HOLD = 2.0    # s (LANE_CHANGE_HOLD)
WARN_REARM = 3.0               # s the condition must be gone before another start on the warning (REARM_TIME)
WARN_FLOOR = True              # a warning-only start aims at the floor (20 mph) with a SET- hold at once (owner)
STOP_AHEAD = "stop ahead"      # kind: the planned stop alone, no car seen yet
# ... and while one of our own tap episodes (the unsure car, the red light head start) slows the car, "not slowing" is
# judged against OWN_SLOWING_ACCEL instead (stop_warning.OWN_SLOWDOWN_ACCEL): a lowered set speed slows the car
# ~0.5-1.0 m/s^2, which must not keep the stop ahead start from taking the episode over
WARN_OWN_SLOWING_ACCEL = -2.0  # m/s^2

# Red light first, from the stop-light detector (starpilotPlan.redLight: the model predicts a stop within a few seconds).
# Drive 50 (t 307-322, 45 mph, a stopped car in the queue): redLight came on at 307.5, our stop ahead start only at 315.0
# (45 -> 20 within 2.5 s), the stopped car was first seen at 317.0 (118 m, 29 % a car), sure (94 %) at 321.5 / 54 m; the
# driver braked at 322.5 at 32.4 mph (~4 m/s^2). With the set speed at the floor and no radar lock the stock cruise slows
# only ~0.8-1.0 m/s^2, so every second of head start counts (a cruise model: 1 mph steps from 2 s after redLight ~29.8
# mph at the brake instead of 32.6). Drives 4f and 50: in all 7 genuine STOP AHEAD warnings redLight came on 3.4-12 s
# before the warning and stayed on ~4-7 s; redLight alone also has 1-9 s episodes at speed with no stop (drive 4f:
# 154-160 s at 37 mph, 397-402 s at 50 mph, several 1-3 s blips), so on its own it only earns a head start: once on for
# RED_ON_TIME (gaps up to RED_START_DROPOUT), SET- taps 1 mph every RED_STEP_TIME, at most RED_MAX_DROP below
# max(our speed, the set speed), silent (no alert; the stop event log has the kind). The stopped car / stop with car /
# stop ahead starts take the episode over as soon as they hold; redLight off for RED_DROPOUT with nothing else
# releases it and the speed comes back (at once unless a slower car is in our lane). After any release the light must
# go off before another head start (the driver on the gas: their override).
RED_START_ENABLED = True
RED_ON_TIME = 2.0            # s
RED_START_DROPOUT = 0.25     # s (the flag is a 20 Hz signal; qlogs show it at ~2 Hz)
RED_DROPOUT = 1.0            # s off before the head start is released
RED_STEP_TIME = 1.0          # s between the 1 mph steps (the false episodes above cost 2-5 mph)
RED_MAX_DROP = 5 * MS_PER_MPH
# drive 52 (t 182.3-185.8): redLight on 3.5 s at 20-24 mph, set 35, then the queue car ahead and a hard brake at 196:
# the old minimum (WARN_MIN_SPEED, 25 mph) kept the head start out
RED_MIN_SPEED = 15 * MS_PER_MPH
# ... but below WARN_MIN_SPEED only for a light that came on RED_AFTER_STOP or more after we were last stopped. Drives 4c,
# 4d, 4f, 50, 52 replayed: the two other head starts the lower minimum adds were both for a light that came on while
# pulling away from a stop, no stop after it (drive 4d t 1432-1438, on 7.8 s after the stop, 17-24 mph, set 55; drive 50
# t 226-234, on 1.7 s after the stop, the cruise resumed at 19.8 mph, set 43); with the first drop to our speed each
# would have held the car near 20 mph for 2-5 s. Drive 52: the light came on 17.8 s after its last stop. (An onset is
# the light on after RED_DROPOUT off; a light already on when we stopped counts as coming on when we roll again: review
# a8, the same light on through the stop and while pulling away gave a SET- hold 45 -> 25 at 15 mph; drive 4f t 1122.6-
# 1131, redLight on from before the stop up to 15 mph, cruise off.)
RED_LOW_SPEED = WARN_MIN_SPEED
RED_AFTER_STOP = 12.0        # s
RED_LIGHT = "red light (head start)"
# No head start while a curve slowdown is on (drive 69 t 760, a 38 mph map curve: the detector took the tail lights ahead
# for a red light, 38 -> 35 for 30 s); the stop ahead warning start itself is unchanged
RED_IN_CURVE_OFF = True
RED_GONE = "red light gone"
# When the light goes (drive 52: it went at 187.8 s, the queue car was in our lane at 188.0, 55-76 % a car), a slower
# car in our lane of any confidence the unsure car would take counts as "slower car ahead": no restore at once, and while
# it is there after the release no SET+ of ours (a hold in progress stops), so the restore never goes past our speed
# into it (review a1: a car seen 1 s after the release found our SET+ hold going 27 -> 35 at 29 mph, the unsure car's
# first SET- 1.5 s later, HOLD_SETTLE). The controller gets it as stop_slower_ahead (slower_after_red).

# The cruise still accelerating when a tap episode starts (drive 52, t 188-196): set 35 at 23.5-24.4 mph with a slow car
# ahead; the unsure cap max(speed, set) - 5 = 30 let the car go on accelerating, 24 -> 29 mph by 193.0 (our stop ahead
# start at 191.0 came while it still was, and the cruise answers ~1 s late), and with no radar lock the lowered set speed
# then slowed it only 0.3-0.7 m/s^2: the driver braked at 25.7 mph, 25 m behind the queue car (up to 4.3 m/s^2). When
# the set speed is more than CATCH_UP_GAP above our speed at a red light head start or an unsure car start, the first
# action brings the set speed down to our speed (target = our speed at the start, urgent: a SET- hold or fast taps, never
# below the floor), then the usual 1 mph steps, the cap (RED_MAX_DROP / UNSURE_MAX_DROP, 5 mph) measured from our speed
# at the start instead of max(speed, set). Cruising steady the set speed is within -0.8..+0.3 mph of vEgo (drives 4c,
# 4d, 4f, 50, 52; 10-90 %): there the behaviour is unchanged. The first drop is done once the set speed is within 1 mph
# of that speed, or after CATCH_UP_TIME (presses lost, or Test mode: then the steps go on from there).
CATCH_UP_ENABLED = True
CATCH_UP_GAP = 2 * MS_PER_MPH
CATCH_UP_TIME = 3.0          # s
# ... at CATCH_UP_LOW_SPEED or more the cap stays the old one, max(speed, set) - 5 mph, and the first drop stops there
# when that is above our speed: a false alarm at speed costs 5 mph at most, as before (drive 4f t 156: a false redLight
# for 15 s, no stop, 38.8-39.8 mph under a 45 set after a curve; the cap from our speed cost 45 -> 34, 11 mph, ~10 s to
# come back; the old head start 45 -> 40, 1 mph a second)
CATCH_UP_LOW_SPEED = 25 * MS_PER_MPH

# False stop ahead guard (drive 50, t 107-114, 42 mph: a green light with a car turning across; the plan said stop, the
# set speed went 45 -> 19 and the car lost ~12 mph before the restore; redLight had been on for only ~0.5 s and was off
# ~2.0-2.5 s before our start). A stop ahead start with redLight neither on nor on within WARN_RED_RECENT, and no slower
# car confirmed (STOP_CAR_MIN_PROB), taps 1 mph at a time, at most UNSURE_MAX_DROP (the unsure car's cap), instead of
# the floor hold; the warning itself shows as before (stop_warning.py). It becomes the full drop when redLight comes on,
# a car confirms the stop (the early start's EARLY_CONFIRM_* rule) or the plan has shown the stop WARN_GUARD_PLAN_HOLD
# continuously (LATCH_DROPOUT gaps). All 7 genuine warnings (drives 4f, 50) had redLight on at our start: unchanged.
WARN_GUARD_ENABLED = True
# A stop ahead (no car) whose planned stop is gone this long, with no slower car in our lane (any confidence the unsure
# car takes), is released at once (restore_now), without waiting for the model to want to speed up (GO_TIME) or the latch
# to run out (LATCH_HOLD). Drive 54, t 758.65: the floor hold for a slowdown the model planned ~115 m ahead (the queue was
# 266 m ahead), the plan gone at 761.8 (for good at 762.5), released only at 764.66 with the car at 35 mph and still
# slowing. The genuine stops ahead of drive 54 (t 210, 373, 1245) had no plan gap over 0.07 s. A new start on the
# warning still needs its condition gone WARN_REARM first (no down-up-down for the same stop).
STOP_AHEAD_GONE_ENABLED = True
STOP_AHEAD_GONE_TIME = 1.0   # s
STOP_AHEAD_CLEARED = "stop ahead cleared (no stop planned)"
# ... but not while the stop-light detector is still on (drive 69, t 125-134, 244-248, 1193-1200, 1388: the planned stop
# blinked off for 0.3-1 s under a red light, the episode was released and its restore took the set speed back up, the
# light and the plan still on; the driver braked each time): a stop ahead (no car) is released as cleared / gone before the
# warning only once redLight has been off RED_DROPOUT, or the plan has been gone STOP_AHEAD_RED_HOLD with the light on (a
# false red light holds the slowdown that long at most). The driver's gas or brake still release it at once.
STOP_AHEAD_RED_HOLD = 6.0    # s
# After such a release the stop starts again at once (no WARN_REARM wait) while the light is (back) on and the planned
# stop latched again, for REARM_ON_RED_TIME after the release.
REARM_ON_RED_TIME = 10.0     # s
# Limits (review of the merged fix): the light on alone holds a stop ahead at most STOP_AHEAD_RED_HOLD_MAX from the first
# planned-stop dropout under it (a red arrow while going straight, the cross street's red: the plan blinking at least
# every 6 s would otherwise hold the set speed down until his gas / brake), and never once the model says go (GO_TIME).
# The restart at once happens once per release chain: not again until he braked / we stopped / the gas, or
# REARM_CHAIN_TIME after it (a gappy detector would cycle down / up / down).
STOP_AHEAD_RED_HOLD_MAX = 8.0   # s
# (second review) ... counted as time WITH THE PLAN ABSENT under the light, summed over the episode: a real red light
# approached for 10+ s with short plan blinks (0.4 s, then 1.2 s eight seconds later) is never near it, while a red arrow
# for another lane (the plan off about half the time) reaches it in ~16 s. A gap of STOP_AHEAD_RED_HOLD continuous
# seconds still ends it sooner.
# A stop ahead released while the light is still on (by those limits) does not restore at once: the lowered set speed is
# kept (hold_restore, no SET+) until the light has been off RED_DROPOUT, the driver's gas, or the model says go (GO_TIME),
# at most RED_RELEASE_HOLD_MAX after the release (a stuck redLight flag). Real red light, the camera losing the planned
# stop near the line at 20-30 mph: no release for 6 s of no plan, and after it the set speed stays where it is, never up.
RED_RELEASE_HOLD_MAX = 20.0  # s
REARM_CHAIN_TIME = 30.0      # s
WARN_RED_RECENT = 1.5        # s (3 s would have let the drive 50 false stop through; redLight gaps seen: <= 1 s)
WARN_GUARD_PLAN_HOLD = 4.0   # s
# A far stop ahead (drive 6a t 580: the plan's stop 116 m ahead at 50.5 mph, the real light > 250 m away and green; set
# 50 -> 20 at once, the car down to 22 mph). The plan's stop distance is the distance to its slowest point in the 8 s
# horizon (its needed deceleration is always >= 1.4), so a camera-only stop ahead (no slower car) is staged on the
# extrapolated stop distance d_stop = v^2 / (2 plan decel) + 2: below FAR_STOP_DISTANCE the floor at once (as before);
# otherwise for FAR_STOP_STAGE the set speed target is the speed from which FAR_STOP_DECEL stops in d_stop (whole mph
# up, never below the floor), and at its end the floor only when the plan showed the stop on FAR_STOP_PLAN_SHARE of the
# frames from FAR_STOP_WINDOW_START on; otherwise the episode ends (restore). Starting at FAR_STOP_RED_SPEED or more the
# stop-light detector alone does not take or hold the floor (6a: the plan on 3 % of the frames, the light on for 4 s
# more); below it the red light rules are as before. Never a restore while the light is on, at any speed (review of
# round k: 69@1193 and 6c 768 restored toward real reds): such a release keeps the lowered set speed (RED_RELEASE_HOLD_MAX,
# his gas) and re-arms on the light (REARM_ON_RED_TIME). Real reds: the floor 1.3-1.9 s later (architect's replay), 0-3
# mph more at his brake.
FAR_STOP_ENABLED = True
FAR_STOP_DISTANCE = 80.0     # m
FAR_STOP_DECEL = 0.8         # m/s^2 (never more: the cruise's own slowdown at a low set speed)
FAR_STOP_MIN_PLAN_DECEL = 0.1  # m/s^2: guard
FAR_STOP_STAGE = 2.5         # s
FAR_STOP_WINDOW_START = 0.5  # s
FAR_STOP_PLAN_SHARE = 0.5
# (second review, V2: the stage's cost on real reds) while the light is on and the stage began at FAR_STOP_RED_SPEED or
# more: its target from the plan's own stop distance (plan_stop_d, nearer than d_stop), the floor at its end from
# FAR_STOP_RED_PLAN_SHARE of the plan; without the light as above
FAR_STOP_RED_PLAN_D = True
FAR_STOP_RED_PLAN_SHARE = 0.25
FAR_STOP_RED_SPEED = 45 * MS_PER_MPH
FAR_STOP_UNCONFIRMED = "far stop ahead not confirmed by the plan"
TAP_KINDS = frozenset({SLOW_UNSURE, RED_LIGHT})   # our tap-only episodes (no lead target, no holds)
# FrogPilot's lead relevance (stop_warning.lead_is_relevant), duplicated so this module stays dependency free
REL_STOP_DISTANCE, REL_LEAD_STOP_MARGIN, REL_MAX_T_FOLLOW, REL_PLANNER_TIME, REL_CRUISING_SPEED = 6.0, 4.0, 3.0, 10.0, 5.0

# Near-miss candidate (logged when the trigger did not fire)
NEAR_MISS_LEAD_SPEED = 3.0
NEAR_MISS_DISTANCE = 120.0
NEAR_MISS_EGO_SPEED = 15 * MS_PER_MPH
NEAR_MISS_HOLD = 0.3


def _frames(seconds):
  return int(round(seconds / DT_CTRL))


@dataclass
class StoppedCarInputs:
  enabled: bool            # openpilot engaged
  cruise_enabled: bool     # stock cruise engaged
  brake_pressed: bool
  v_ego: float             # m/s
  lead_alive: bool         # radarState is being received
  lead_status: bool
  lead_prob: float
  lead_d: float
  lead_v: float
  lead_y: float
  lead_a: float = 0.0               # radarState leadOne.aLeadK (m/s^2), for the floor mode's lead guard
  radar_has_lead: bool = False      # stock MRCC radar (CRZ_CTRL.RADAR_HAS_LEAD): debounced here, drives the radar follow
  radar_lead_distance: int = 0      # its distance bucket (CRZ_CTRL.RADAR_LEAD_RELATIVE_DISTANCE: 1 farthest .. 5 nearest), logged only
  model_alive: bool = False         # modelV2 is being received
  model_accel: float = 0.0          # modelV2.action.desiredAcceleration: < 0 the model wants to slow down
  plan_stop: bool = False           # the model plans a stop ahead (plan_stop(), the red light warning's check)
  plan_ratio: float = 1.0           # slowest planned speed in the horizon / ours (logs)
  plan_stop_d: float = 0.0          # m to where the stop is planned (logs)
  a_ego: float = 0.0                # m/s^2 (CS.out.aEgo): the MRCC braking for the car ahead
  lead_y_path: float | None = None  # m: the lead's offset from the model path (yRel sign, lead_path_offset); None = lead_y
  model_length: float = 0.0         # m: modelV2.position.x[-1] (the planner's model_length), for the lead relevance
  lane_change: bool = False         # modelV2.meta.laneChangeState starting / finishing
  gas_pressed: bool = False
  set_speed: float = 0.0            # m/s: the cruise set speed (0 unknown), the unsure car's cap is below it
  red_light: bool = False           # starpilotPlan.redLight (the stop-light detector), False when the plan is not alive
  curve_active: bool = False        # a curve slowdown target (camera or a followed map curve) is on: no red light head start


def _interp(x, xp, fp):
  if not xp:
    return 0.0
  if x <= xp[0]:
    return fp[0]
  for k in range(1, len(xp)):
    if x <= xp[k]:
      x0, x1, y0, y1 = xp[k - 1], xp[k], fp[k - 1], fp[k]
      return y0 if x1 == x0 else y0 + (y1 - y0) * (x - x0) / (x1 - x0)
  return fp[-1]


def plan_stop(model_t, model_v, model_x, v_ego):
  """(stop planned, slowest planned speed / ours, m to it): the model's plan drops to PLAN_STOP_RATIO of our speed or
  less within PLAN_STOP_HORIZON on PLAN_STOP_POINTS points in a row, implying PLAN_STOP_DECEL or more (the planner's
  red light warning, stop_warning.StopWarningLogic.plan_slowdown / stop_ahead, without its lead and alert gates)."""
  if not model_t or len(model_t) != len(model_v) or v_ego <= 0.5:
    return False, 1.0, 0.0
  horizon = [v for t, v in zip(model_t, model_v, strict=True) if t <= PLAN_STOP_HORIZON]
  ratio = min(horizon) / v_ego if horizon else 1.0
  slow = PLAN_STOP_RATIO * v_ego
  t_min, v_min = None, None
  for k, (t, v) in enumerate(zip(model_t, model_v, strict=True)):
    if t > PLAN_STOP_HORIZON:
      break
    held = model_v[k:k + PLAN_STOP_POINTS]
    if len(held) == PLAN_STOP_POINTS and all(x <= slow for x in held) and (v_min is None or v < v_min):
      t_min, v_min = t, v
  if t_min is None:
    return False, ratio, 0.0
  d = _interp(t_min, model_t, model_x) if len(model_x) == len(model_t) else (v_ego + v_min) * t_min / 2
  required = (v_ego ** 2 - max(v_min, 0.0) ** 2) / (2 * max(d - PLAN_STOP_BUFFER, 1.0))
  return required >= PLAN_STOP_DECEL and v_ego >= START_SPEED, ratio, d


def stopped_car_target(d_rel, floor=FLOOR_SPEED, decel=TARGET_DECEL, margin=TARGET_MARGIN):
  """Set speed (true speed, m/s) from which a `decel` slowdown reaches `floor` `margin` m before the lead."""
  return math.sqrt(floor ** 2 + 2 * decel * max(d_rel - margin, 0.0))


def lead_path_offset(lead_d, lead_y, model_x, model_y):
  """lead_y (radarState yRel = -lead y, so right positive) plus the model path's lateral position at the lead's
  distance (model y is left positive), so a car in our lane on a curve reads ~0. Returns lead_y without a path."""
  xs, ys = list(model_x), list(model_y)
  if len(xs) < 2 or len(xs) != len(ys):
    return lead_y
  return lead_y + _interp(lead_d, xs, ys)


def lead_relevant(lead_status, lead_d, lead_v, lead_a, standstill, v_ego, model_length):
  """stop_warning.lead_is_relevant (FrogPilot's is_lead_relevant); model_length <= 0 (unknown) skips its path term."""
  following_speed = max(v_ego, REL_CRUISING_SPEED)
  lead_speed = lead_v + min(lead_a, 0.0) * REL_PLANNER_TIME / 2
  in_path = model_length <= 0 or lead_d < (0.0 if standstill else model_length) + REL_STOP_DISTANCE + REL_LEAD_STOP_MARGIN
  return bool(lead_status and in_path and
              lead_d < REL_STOP_DISTANCE + following_speed * REL_MAX_T_FOLLOW +
              max(following_speed - lead_speed, 0.0) * REL_PLANNER_TIME)


def urgency(v_ego, lead_d, lead_v):
  """(required deceleration to stop 15 m behind the lead, time to collision)."""
  closing = v_ego - lead_v
  required = closing ** 2 / (2 * max(lead_d - TARGET_MARGIN, 1.0)) if closing > 0 else 0.0
  ttc = lead_d / closing if closing > 0.1 else float("inf")
  return required, ttc


class StoppedCarTrigger:
  """Decides when a stopped car ahead should lower the set speed, and until when."""

  def __init__(self):
    self.frame = 0
    self.active = False
    self.hold_frames = 0
    self.lost_frames = 0
    self.out_of_lane_frames = 0
    self.cruise_on_frames = 0
    self.target = 0.0
    self.urgent = False
    self.floor_mode = False          # this episode: a confirmed stop within FLOOR_DISTANCE, straight to the floor
    self.required = 0.0
    self.ttc = float("inf")
    self.start_frame = -1
    self.release_reason = None
    self.restore_now = False         # set for one frame when the lead has clearly gone (moved away, or we stopped
                                     # behind it): restore at once, SET+ holds allowed. Lost / left-lane releases
                                     # use the normal restore instead, since the car may still be ahead.
    self.moving_frames = 0
    self.go_frames = 0
    self.moving_on_frames = 0        # the car ahead moving again, no stop planned, no red light (GO_MOVING_*)
    self.kind = None                 # STOPPED_CAR or STOP_WITH_CAR while active (or last active)
    self.prev_radar_has_lead = False
    self.radar_rises = []            # frames at which the stock radar picked up a lead (for the logs)
    # red light first: the stop-ahead latch and the early start
    self.latched = False
    self.latch_frames = 0            # debounce of the planned stop
    self.latch_gap = 0
    self.latch_set_frame = -1
    self.last_plan_stop = -10_000
    self.early = False               # the active episode started early (low-confidence car while latched)
    self.confirmed = False           # ... and has since been confirmed (prob / urgency): holds and fast pace allowed
    self.confirm_frames = 0
    self.confirm_frame = -1
    self.early_frames = 0
    self.early_gap = 0
    self.unsure_frames = 0           # the unsure car condition (UNSURE_*)
    self.unsure_gap = 0
    self.unsure_off_frames = 0       # in an unsure episode: the car gone / out of path / not slower
    self.unsure_sure_frames = 0
    self.unsure_target = 0.0
    self.unsure_release_frame = -10_000
    self.unsure_a_f = 0.0            # the car ahead's aLeadK, filtered from its first frame (UNSURE_LOW_*)
    self.unsure_low = False          # this unsure episode began below CATCH_UP_LOW_SPEED
    self.hold_restore = False        # latched and not active: keep the lowered set speed (no SET+) for now
    # the car ahead's speed, filtered (VLEAD_*)
    self.lead_v_hist = []            # [(frame, vLead)]
    self.lead_v_f = 0.0
    self.prev_lead_d = None
    # the stock radar (RADAR_*)
    self.radar_frames = 0            # frames the radar has reported a lead (gaps up to RADAR_DROPOUT)
    self.radar_gap = 0
    self.radar_stable = False        # ... long enough to count as tracking
    self.radar_follow = False        # tracking a car it was seen tracking while that car was moving
    self.radar_flicker = False       # a follow ended by a dropout this episode: RADAR_REACQUIRE_TIME to count again
    self.radar_hold = False          # active, and the MRCC handles the car ahead: no SET- / SET+ from us
    self.radar_match_frames = 0      # following at the car's speed
    # restore after a brake / lane exit release (CLEAR_*)
    self.clear_release = False       # the last release was one where the car ahead may be gone (CLEAR_RELEASE_REASONS)
    self.clear_ahead = False         # ... and no slower car is in our lane now: the restore may use SET+ holds
    self.pulling_away = False        # not active, and no car ahead or it is pulling away: restores at any speed
    self.pull_frames = 0
    # the car ahead leaving our lane (LEAVE_*)
    self.y_f = None                  # filtered offset from the path
    self.y_base = None               # smallest |y_f| this episode / since the last lead switch
    self.leave_frames = 0
    self.leaving = False
    self.left_car = False            # the current lead was released as leaving our lane: it does not start one again
    # the stop ahead warning (WARN_*)
    self.a_f = None
    self.since_lane_change = _frames(WARN_LANE_CHANGE_HOLD)
    self.warn_frames = 0
    self.warn_gap = 0
    self.warn_off_frames = _frames(WARN_REARM)
    self.warn_cond = False
    self.warning = False             # the warning condition has held WARN_START_DEBOUNCE (we start on it)
    self.warn_latched = False        # an episode started on the warning: the condition must be gone WARN_REARM first
    self.warn_start = False          # this episode started on the warning alone (logs)
    self.warn_pending = False        # ... before the warning shows: released if the planned stop goes away first
    self.unsure_from = False         # this episode began as an unsure car (taps) and a stop start took it over
    self.pull_accel_frames = 0       # the car ahead accelerating away (PULL_AWAY_ACCEL_*)
    self.pull_lead_since = 0         # frame the current car ahead was first seen (no distance jump since)
    self.warn_plan_frames = 0
    self.warn_plan_gap = 0
    self.gas_frames = 0
    self.overridden = False          # the driver went on the gas for the current lead: it does not start one again
    self.overridden_v = 0.0          # its speed at the release
    self.gas_off_frames = 0
    self.moving_target = None        # MOVING_LEAD_GUARD: the lowest moving-car target this episode, None once at floor
    self.lead_a_f = 0.0
    self.moving_match_frames = 0
    self.floor_committed = False
    # the red light head start (RED_*)
    self.red_frames = 0              # frames redLight has been on (gaps up to RED_START_DROPOUT)
    self.red_gap = _frames(RED_DROPOUT) + 1   # (off so far: a light on at the first frame is an onset)
    self.last_red_frame = -10_000
    self.red_latched = False         # an episode was released while the light was on: it must go off before a head start
    self.rearm_on_red = False        # released as a stop ahead cleared / gone: a red light and the planned stop restart it
    self.release_frame = -10_000     # (REARM_ON_RED_TIME)
    self.rearm_used_frame = None     # the restart at once was used (once per release chain, REARM_CHAIN_TIME)
    self.red_absent_frames = 0       # frames with the plan absent under the light this episode (STOP_AHEAD_RED_HOLD_MAX)
    self.red_release_frame = None    # a stop ahead released with the light still on: no restore yet (RED_RELEASE_HOLD_MAX)
    self.go_free_frames = 0          # the model says go (as go_frames), counted while not active
    self.rolling_frames = 0          # frames since we were last stopped (RED_AFTER_STOP) ...
    self.red_onset_rolling = 0       # ... when the light last came on
    self.red_base = 0.0              # the head start's reference speed (max(ours, set), or ours: CATCH_UP_*) and its cap
    self.red_cap = 0.0
    self.slower_after_red = False    # not active since a red light gone release, a slower car in our lane (RED_GONE)
    # a tap episode that began with the cruise still accelerating (CATCH_UP_*)
    self.catch_up = False            # the first drop (to our speed at the start) is under way
    self.caught_up = False           # this episode began with it (logs)
    self.tap_base = 0.0              # the tap episode's reference speed
    self.catch_frame = -1            # frame the first drop began
    self.step_frame = -1             # frame the 1 mph steps' clock starts (the start, or the end of the first drop)
    # the false stop ahead guard (WARN_GUARD_*)
    self.warn_unsure = False         # this stop ahead episode is not confirmed yet: taps only (the unsure cap)
    self.warn_guarded = False        # ... it started that way (logs)
    # the far stop ahead's stage (FAR_STOP_*)
    self.far_stage = False           # this stop ahead is in its first FAR_STOP_STAGE on the distance target
    self.far_frame = -1
    self.far_fast = False            # ... it started at FAR_STOP_RED_SPEED or more
    self.far_target = 0.0
    self.far_plan = self.far_window = 0
    self.far_staged = False          # this episode began staged (logs)

  def _update_red_light(self, i):
    self.rolling_frames = 0 if i.v_ego < EGO_STOPPED_SPEED else self.rolling_frames + 1
    if i.red_light and self.red_gap > _frames(RED_DROPOUT):
      self.red_onset_rolling = self.rolling_frames
    # a light on before / through our stop counts as coming on when we start rolling again (review a8)
    self.red_onset_rolling = min(self.red_onset_rolling, self.rolling_frames)
    if i.red_light:
      self.red_frames += 1
      self.red_gap = 0
      self.last_red_frame = self.frame
    else:
      self.red_gap += 1
      if self.red_gap > _frames(RED_START_DROPOUT):
        self.red_frames = 0
      if self.red_gap > _frames(RED_DROPOUT):
        self.red_latched = False

  def _red_start(self, i, start_gates):
    """The stop-light detector alone, long enough: the head start (nothing else active)."""
    return (RED_START_ENABLED and not self.red_latched and self.red_frames >= _frames(RED_ON_TIME) and start_gates and
            i.v_ego >= RED_MIN_SPEED and (i.v_ego >= RED_LOW_SPEED or self.red_onset_rolling >= _frames(RED_AFTER_STOP)) and
            self.since_lane_change >= _frames(WARN_LANE_CHANGE_HOLD) and
            self.a_f is not None and self.a_f > WARN_NOT_SLOWING_ACCEL and   # (not when the car is already slowing)
            not (RED_IN_CURVE_OFF and i.curve_active))

  def _filter_lead_v(self, i):
    if i.lead_alive and i.lead_status:
      if self.prev_lead_d is None or abs(i.lead_d - self.prev_lead_d) > PULL_AWAY_JUMP:
        self.pull_lead_since = self.frame    # a car (re)appeared or another one: not "the same car" yet
        self.pull_accel_frames = 0
      if self.prev_lead_d is not None and abs(i.lead_d - self.prev_lead_d) > LEAD_JUMP:
        self.lead_v_hist = []        # another car: start again
        self.radar_follow = False
        self.y_f = self.y_base = None
        self.leave_frames = 0
        self.left_car = False
        self.overridden = False
        self.lead_a_f = i.lead_a
      if self.prev_lead_d is None or abs(i.lead_d - self.prev_lead_d) > LEAD_JUMP:
        self.unsure_a_f = i.lead_a           # (UNSURE_LOW_*) a car (re)appeared or another one: its own from here
      self.prev_lead_d = i.lead_d
      self.lead_v_hist.append((self.frame, i.lead_v))
      alpha = DT_CTRL / (MOVING_LEAD_TAU + DT_CTRL)
      self.lead_a_f += alpha * (i.lead_a - self.lead_a_f)
      self.unsure_a_f += DT_CTRL / (UNSURE_LOW_ACCEL_TAU + DT_CTRL) * (i.lead_a - self.unsure_a_f)
    else:
      self.prev_lead_d = None
    self.lead_v_hist = [(f, v) for f, v in self.lead_v_hist if self.frame - f < _frames(VLEAD_WINDOW)]
    if not self.lead_v_hist:
      self.lead_v_f = i.lead_v
      return
    values = sorted(v for _, v in self.lead_v_hist)
    if self.frame - self.lead_v_hist[0][0] < _frames(VLEAD_MIN_FILL):
      self.lead_v_f = values[-1]
    else:
      n = len(values)
      self.lead_v_f = values[n // 2] if n % 2 else (values[n // 2 - 1] + values[n // 2]) / 2

  def _update_radar(self, i):
    if i.radar_has_lead:
      self.radar_frames += 1
      self.radar_gap = 0
    else:
      self.radar_gap += 1
      if self.radar_gap > _frames(RADAR_DROPOUT):
        self.radar_frames = 0
    need = RADAR_REACQUIRE_TIME if self.radar_flicker else RADAR_TRACK_TIME
    self.radar_stable = self.radar_frames >= _frames(need)
    vision = i.lead_alive and i.lead_status and i.lead_prob >= STOP_CAR_EARLY_MIN_PROB
    if self.radar_follow and not self.radar_stable:
      self.radar_follow = False
      self.radar_flicker = self.active   # during an episode: the radar must prove itself for longer
    elif self.radar_stable and vision and self.lead_v_f > STOPPED_SPEED:
      self.radar_follow = True
    stopping = i.plan_stop or self._model_slowing(i) or i.lead_a < -RADAR_MATCH_LEAD_DECEL
    matched = (self.active and self.radar_follow and i.lead_status and self.lead_v_f > STOPPED_SPEED and
               i.v_ego - self.lead_v_f <= RADAR_FOLLOW_CLOSING and not stopping)
    self.radar_match_frames = self.radar_match_frames + 1 if matched else 0

  @staticmethod
  def _y_path(i):
    return i.lead_y if i.lead_y_path is None else i.lead_y_path

  def _update_leaving(self, i):
    present = i.lead_alive and i.lead_status
    self.gas_off_frames = 0 if i.gas_pressed else self.gas_off_frames + 1
    if (not present or self.lead_v_f <= STOPPED_SPEED or self.lead_v_f <= self.overridden_v - OVERRIDE_SLOWER or
        self.gas_off_frames >= _frames(OVERRIDE_REARM)):
      self.overridden = False        # gone, that car is stopping / slowing a lot now, or the driver let go a while ago
    if not present:
      self.left_car = False
    if present:
      y = self._y_path(i)
      alpha = DT_CTRL / (LEAVE_TAU + DT_CTRL)
      self.y_f = y if self.y_f is None else self.y_f + alpha * (y - self.y_f)
      if self.left_car and abs(self.y_f) < LEAVE_Y_MIN - LEAVE_BACK:
        self.left_car = False        # it came back into our lane
      if self.active:
        self.y_base = abs(self.y_f) if self.y_base is None else min(self.y_base, abs(self.y_f))
    self.leaving = bool(LEAVE_ENABLED and self.active and present and self.y_f is not None and self.y_base is not None and
                        i.lead_prob >= LEAVE_MIN_PROB and 0 < i.lead_d <= LEAVE_MAX_DISTANCE and
                        abs(self.y_f) >= LEAVE_Y_MIN and abs(i.lead_y) >= LEAVE_Y_MIN and
                        abs(self.y_f) - self.y_base >= LEAVE_DRIFT)
    self.leave_frames = self.leave_frames + 1 if self.leaving else 0

  def _update_warning(self, i):
    alpha = DT_CTRL / (WARN_ACCEL_TAU + DT_CTRL)
    self.a_f = i.a_ego if self.a_f is None else self.a_f + alpha * (i.a_ego - self.a_f)
    self.since_lane_change = 0 if i.lane_change else self.since_lane_change + 1
    relevant = lead_relevant(i.lead_alive and i.lead_status, i.lead_d, i.lead_v, i.lead_a, i.v_ego < EGO_STOPPED_SPEED,
                             i.v_ego, i.model_length)
    # our own taps (unsure car, red light head start) slow the car: not "already slowing" (as the planner's warning)
    not_slowing = self.a_f > (WARN_OWN_SLOWING_ACCEL if (self.active and self.kind in TAP_KINDS) else WARN_NOT_SLOWING_ACCEL)
    self.warn_cond = bool(i.plan_stop and i.cruise_enabled and not i.brake_pressed and not_slowing and
                          i.v_ego >= WARN_MIN_SPEED and not relevant and
                          self.since_lane_change >= _frames(WARN_LANE_CHANGE_HOLD))
    if self.warn_cond:
      self.warn_frames += 1
      self.warn_gap = 0
      self.warn_off_frames = 0
    else:
      self.warn_gap += 1
      self.warn_off_frames += 1
      if self.warn_gap > _frames(WARN_DROPOUT):
        self.warn_frames = 0
      if self.warn_off_frames >= _frames(WARN_REARM):
        self.warn_latched = False
    self.warning = self.warn_cond and self.warn_frames >= _frames(WARN_START_DEBOUNCE)

  def _slower_car_ahead(self, i):
    v = self.lead_v_f
    return (i.lead_alive and i.lead_status and i.lead_prob >= CLEAR_MIN_PROB and abs(self._y_path(i)) <= CLEAR_LATERAL and
            0 < i.lead_d <= STOP_CAR_MAX_DISTANCE and (v <= STOPPED_SPEED or i.v_ego - v >= CLEAR_SLOWER))

  def _lead_ok(self, i):
    v = self.lead_v_f
    return (not self.left_car and i.lead_alive and i.lead_status and i.lead_prob >= MIN_PROB and abs(i.lead_y) <= MAX_LATERAL and
            v <= STOPPED_SPEED and i.v_ego - v >= MIN_CLOSING and i.lead_d > 0)

  @staticmethod
  def _model_slowing(i):
    return i.model_alive and i.model_accel <= MODEL_SLOW_ACCEL

  def _slow_car(self, i, max_lateral=STOP_CAR_MAX_LATERAL, min_prob=STOP_CAR_MIN_PROB):
    """A car ahead (near our lane) that is stopped or much slower than us (its filtered speed)."""
    v = self.lead_v_f
    return (not self.left_car and not self.overridden and i.lead_alive and i.lead_status and i.lead_prob >= min_prob and
            abs(i.lead_y) <= max_lateral and
            0 < i.lead_d <= STOP_CAR_MAX_DISTANCE and
            (i.v_ego - v >= STOP_CAR_SLOWER or v <= STOP_CAR_RATIO * i.v_ego))

  def _stop_with_car(self, i):
    return self._model_slowing(i) and self._slow_car(i)

  def _early_stop_with_car(self, i):
    return (EARLY_START_ENABLED and self.latched and self._model_slowing(i) and
            self._slow_car(i, min_prob=STOP_CAR_EARLY_MIN_PROB))

  def _update_latch(self, i):
    if i.plan_stop:
      self.latch_frames += 1
      self.latch_gap = 0
      self.last_plan_stop = self.frame
    else:
      self.latch_gap += 1
      if self.latch_gap > _frames(LATCH_DROPOUT):
        self.latch_frames = 0
    if not self.latched and self.latch_frames >= _frames(LATCH_DEBOUNCE):
      self.latched = True
      self.latch_set_frame = self.frame
    if self.latched and (self.frame - self.last_plan_stop > _frames(LATCH_HOLD) or i.v_ego < EGO_STOPPED_SPEED):
      self.latched = False
      self.latch_frames = 0

  @property
  def latch_age(self):
    """s since the stop-ahead latch set (0 when not latched)."""
    return (self.frame - self.latch_set_frame) * DT_CTRL if self.latched else 0.0

  def blockers(self, i):
    """Why the trigger is not (yet) active, for the near-miss log."""
    out = []
    if not i.enabled:
      out.append("openpilot not engaged")
    if not i.cruise_enabled:
      out.append("cruise off")
    elif self.cruise_on_frames < _frames(CRUISE_SETTLE):
      out.append("cruise engaged < 0.3 s")
    if i.brake_pressed:
      out.append("brake pressed")
    if not i.lead_alive:
      out.append("no radarState")
    elif not i.lead_status:
      out.append("no lead")
    else:
      if i.lead_prob < MIN_PROB:
        out.append(f"prob {i.lead_prob:.2f} < {MIN_PROB}")
      if abs(i.lead_y) > MAX_LATERAL:
        out.append(f"yRel {i.lead_y:.1f} > {MAX_LATERAL}")
      if self.lead_v_f > STOPPED_SPEED:
        out.append(f"vLead {self.lead_v_f:.1f} > {STOPPED_SPEED} m/s")
      if i.v_ego - self.lead_v_f < MIN_CLOSING:
        out.append(f"closing {i.v_ego - self.lead_v_f:.1f} < {MIN_CLOSING} m/s")
    if i.v_ego < START_SPEED and not self.active:
      out.append(f"speed {i.v_ego / MS_PER_MPH:.1f} < {START_SPEED / MS_PER_MPH:.0f} mph")
    if not i.model_alive:
      out.append("stop with car: no modelV2")
    elif i.model_accel > MODEL_SLOW_ACCEL:
      out.append(f"stop with car: model accel {i.model_accel:.2f} > {MODEL_SLOW_ACCEL}")
    if i.lead_status and STOP_CAR_EARLY_MIN_PROB <= i.lead_prob < STOP_CAR_MIN_PROB and not self.latched:
      out.append("stop ahead not latched (a low-confidence car only counts after a planned stop)")
    if i.plan_stop and not self.active:
      if self.since_lane_change < _frames(WARN_LANE_CHANGE_HOLD):
        out.append("stop ahead: lane change")
      if self.a_f is not None and self.a_f <= WARN_NOT_SLOWING_ACCEL:
        out.append(f"stop ahead: already slowing aEgo {self.a_f:.2f}")
      if lead_relevant(i.lead_alive and i.lead_status, i.lead_d, i.lead_v, i.lead_a, i.v_ego < EGO_STOPPED_SPEED,
                       i.v_ego, i.model_length):
        out.append("stop ahead: lead relevant")
      if self.warn_latched:
        out.append("stop ahead: already started for this stop")
      elif self.warn_cond and not self.warning:
        out.append(f"stop ahead: warning held {self.warn_frames * DT_CTRL:.2f} s < {WARN_START_DEBOUNCE}")
    if i.red_light and not self.active:
      if self.red_latched:
        out.append("red light: an episode was released while it was on")
      elif self.red_frames < _frames(RED_ON_TIME):
        out.append(f"red light: on {self.red_frames * DT_CTRL:.2f} s < {RED_ON_TIME}")
      elif i.v_ego < RED_LOW_SPEED and self.red_onset_rolling < _frames(RED_AFTER_STOP):
        out.append(f"red light: came on {self.red_onset_rolling * DT_CTRL:.1f} s after the last stop (< {RED_AFTER_STOP:.0f} s)")
    if not out and not self.active:
      out.append(f"held {self.hold_frames * DT_CTRL:.2f} s < {HOLD_TIME}")
    return out

  def update(self, i: StoppedCarInputs):
    """Returns True while the set speed should be held down for a stopped car; sets target/urgent."""
    self.frame += 1
    self.restore_now = False
    self.cruise_on_frames = self.cruise_on_frames + 1 if i.cruise_enabled else 0
    if i.radar_has_lead and not self.prev_radar_has_lead:
      self.radar_rises.append(self.frame)
    self.prev_radar_has_lead = i.radar_has_lead
    self._filter_lead_v(i)
    self._update_latch(i)
    self._update_red_light(i)
    self._update_warning(i)
    self._update_radar(i)
    self._update_leaving(i)
    self.gas_frames = self.gas_frames + 1 if (i.gas_pressed and self.active) else 0
    lead_ok = self._lead_ok(i)
    stop_with_car = self._stop_with_car(i)
    gates = (i.enabled and i.cruise_enabled and not i.brake_pressed and
             self.cruise_on_frames >= _frames(CRUISE_SETTLE))
    start_gates = gates and not i.gas_pressed
    self.required, self.ttc = urgency(i.v_ego, i.lead_d, self.lead_v_f) if (i.lead_status and i.lead_alive) else (0.0, float("inf"))
    self.urgent = self.ttc <= URGENT_TTC or self.required >= URGENT_DECEL

    if not self.active or self.kind in TAP_KINDS:
      unsure_on = self.active and self.kind == SLOW_UNSURE
      red_on = self.active and self.kind == RED_LIGHT
      self.hold_frames = self.hold_frames + 1 if ((lead_ok or stop_with_car) and start_gates and i.v_ego >= START_SPEED) else 0
      if self._early_stop_with_car(i) and start_gates and i.v_ego >= START_SPEED:
        self.early_frames += 1
        self.early_gap = 0
      else:
        self.early_gap += 1
        if self.early_gap > _frames(EARLY_DROPOUT) or not self.latched:
          self.early_frames = 0
      early_start = self.early_frames >= _frames(EARLY_HOLD)
      warn_start = (WARNING_START_ENABLED and self.warning and not self.warn_latched and start_gates and
                    i.v_ego >= WARN_MIN_SPEED)
      unsure = self._unsure_car(i) and start_gates
      if unsure:
        self.unsure_frames += 1
        self.unsure_gap = 0
      else:
        self.unsure_gap += 1
        if self.unsure_gap > _frames(EARLY_DROPOUT):
          self.unsure_frames = 0
      # the stop again at once after a stop ahead released as cleared / gone: the light (back) on, the plan latched again
      red_rearm = (self.rearm_on_red and self.frame - self.release_frame <= _frames(REARM_ON_RED_TIME) and
                   self.red_gap <= _frames(RED_START_DROPOUT) and self.latched and i.plan_stop and start_gates and
                   i.v_ego >= WARN_MIN_SPEED)
      if red_rearm:
        warn_start = True
        self.rearm_used_frame = self.frame
      if self.hold_frames >= _frames(HOLD_TIME) or early_start or warn_start:
        self._start(i, lead_ok, early_start, warn_start)
      elif unsure_on:
        self._update_unsure(i, gates)
      elif red_on:
        self._update_red(i, gates)
      elif (UNSURE_ENABLED and self.unsure_frames >= _frames(UNSURE_HOLD) and
            self.frame - self.unsure_release_frame >= _frames(UNSURE_REARM)):
        self._start_unsure(i)
      elif self._red_start(i, start_gates):
        self._start_red(i)
    elif self.gas_frames >= _frames(GAS_RELEASE_TIME):
      self._release("driver on the gas", False)
    elif self.kind == STOP_AHEAD and not self._slow_car(i, min_prob=STOP_CAR_EARLY_MIN_PROB):
      # the planned stop alone, no slower car seen yet (a car at our speed does not end it: the plan still shows the
      # stop, the MRCC keeps the gap to that car)
      going = i.model_alive and i.model_accel > GO_ACCEL and not i.plan_stop
      self.go_frames = self.go_frames + 1 if going else 0
      # the light still on (STOP_AHEAD_RED_HOLD): no release as cleared / gone before the warning
      red_on = self.red_gap < _frames(RED_DROPOUT)
      if not red_on:
        self.red_absent_frames = 0
      elif not i.plan_stop:
        self.red_absent_frames += 1
      red_holds = (red_on and self.frame - self.last_plan_stop < _frames(STOP_AHEAD_RED_HOLD) and
                   self.red_absent_frames < _frames(STOP_AHEAD_RED_HOLD_MAX) and self.go_frames < _frames(GO_TIME) and
                   not (self.far_stage and self.far_fast))
      still_planned = self._warn_still_planned(i) if self.warn_pending else True
      if not gates:
        self._release("cruise off / brake" if not (i.cruise_enabled and not i.brake_pressed) else "openpilot disengaged", False)
      elif i.v_ego < EGO_STOPPED_SPEED:
        self._release("ego stopped behind the lead")
      elif self.warn_pending and not still_planned and not red_holds:
        self._release(STOP_AHEAD_UNCONFIRMED)
      elif not red_holds and (self.go_frames >= _frames(GO_TIME) or not self.latched or
                              (STOP_AHEAD_GONE_ENABLED and self.frame - self.last_plan_stop >= _frames(STOP_AHEAD_GONE_TIME) and
                               not self._slower_in_lane(i))):
        self._release(STOP_AHEAD_CLEARED)
      elif self.far_stage:
        self._update_far_stage(i)
    elif self.kind in (STOP_WITH_CAR, STOP_AHEAD) or (stop_with_car and i.lead_v > RELEASE_LEAD_SPEED):
      if self.kind == STOP_AHEAD:
        # the car for the planned stop appeared: from here on a stop with a car (confirmed / floor mode stay)
        self.lost_frames = self.go_frames = self.moving_on_frames = 0
        self.far_stage = False
      # a stop with a much slower car (or a "stopped" car that is rolling slowly towards the stop): the car ahead
      # may be moving, so "moved away" is not a release; traffic moving again is
      self.kind = STOP_WITH_CAR
      self.moving_frames = 0
      min_prob = STOP_CAR_EARLY_MIN_PROB if self.early else STOP_CAR_MIN_PROB
      vision = i.lead_alive and i.lead_status and i.lead_prob >= min_prob
      still_there = vision and abs(i.lead_y) <= STOP_CAR_MAX_LATERAL
      self.lost_frames = 0 if still_there else self.lost_frames + 1
      # traffic moving again also needs the model to no longer plan a stop (21:11:57: released, set back to 45,
      # then the hard stop 15 s later)
      going = (i.model_alive and i.model_accel > GO_ACCEL and not i.plan_stop and
               (not still_there or i.v_ego - i.lead_v < GO_SLOWER))
      # (once the floor is committed: before that the moving car's own release, MOVING_MATCH_*, is the one)
      moving_on = (GO_MOVING_ENABLED and self.floor_committed and still_there and self.lead_v_f >= GO_MOVING_LEAD and
                   self.lead_a_f >= MOVING_MATCH_DECEL and i.v_ego - self.lead_v_f <= GO_MOVING_CLOSING and
                   self.frame - self.last_plan_stop >= _frames(GO_MOVING_CLEAR) and self.red_gap >= _frames(GO_MOVING_CLEAR))
      self.go_frames = self.go_frames + 1 if going else 0
      self.moving_on_frames = self.moving_on_frames + 1 if moving_on else 0
      # the light on: no go / moving car / pulling away release (STOP_CAR_RED_HOLD, the stop ahead's red_holds)
      red_on = self.red_gap < _frames(RED_DROPOUT)
      if not red_on:
        self.red_absent_frames = 0
      elif not i.plan_stop:
        self.red_absent_frames += 1
      red_holds = (STOP_CAR_RED_HOLD and red_on and self.frame - self.last_plan_stop < _frames(STOP_AHEAD_RED_HOLD) and
                   self.red_absent_frames < _frames(STOP_AHEAD_RED_HOLD_MAX))
      matched = (MOVING_LEAD_GUARD and still_there and not self.floor_committed and not i.plan_stop and
                 self.lead_v_f >= MOVING_LEAD_SPEED and self.lead_a_f >= MOVING_MATCH_DECEL and
                 i.v_ego - self.lead_v_f <= MOVING_MATCH_CLOSING)
      self.moving_match_frames = self.moving_match_frames + 1 if matched else 0
      self.pull_accel_frames = self.pull_accel_frames + 1 if (still_there and self._pulling_away_accel(i)) else 0
      lost_time = RELEASE_LOST_TIME_LATCHED if self.latched else RELEASE_LOST_TIME
      if not gates:
        self._release("cruise off / brake" if not (i.cruise_enabled and not i.brake_pressed) else "openpilot disengaged", False)
      elif i.v_ego < EGO_STOPPED_SPEED:
        self._release("ego stopped behind the lead")
      elif self.pull_accel_frames >= _frames(PULL_AWAY_ACCEL_TIME) and not red_holds:
        self._release(PULLING_AWAY)
      elif self.radar_match_frames >= _frames(RADAR_RESTORE_TIME):
        self._release("radar following the car ahead at our speed")
      elif self.leave_frames >= _frames(LEAVE_TIME):
        self._release("lead leaving the lane", False)
      elif self.moving_match_frames >= _frames(MOVING_MATCH_TIME) and not red_holds:
        self._release("at the moving car's speed")
      elif self.go_frames >= _frames(GO_TIME) and not red_holds:
        self._release(TRAFFIC_MOVING)
      elif self.moving_on_frames >= _frames(GO_TIME):
        self._release(CAR_AHEAD_MOVING)
      elif self.lost_frames >= _frames(lost_time):
        self._release("lead lost (vision)", False)
    else:
      vision = i.lead_alive and i.lead_status and i.lead_prob >= LOST_MIN_PROB
      self.moving_frames = self.moving_frames + 1 if (vision and i.lead_v > RELEASE_LEAD_SPEED) else 0
      # "still there" = a vision lead in our lane that is not driving away; the stock radar never counts. In our lane
      # = its offset from the model path (lead_path_offset), so a stopped car ahead in a curve stays ours (drive 4c)
      y_lane = self._y_path(i)
      still_there = vision and abs(y_lane) <= MAX_LATERAL and i.lead_v <= RELEASE_LEAD_SPEED
      out = vision and abs(y_lane) > MAX_LATERAL
      self.lost_frames = 0 if still_there else self.lost_frames + 1
      self.out_of_lane_frames = self.out_of_lane_frames + 1 if out else 0
      in_lane = vision and abs(y_lane) <= MAX_LATERAL
      self.pull_accel_frames = self.pull_accel_frames + 1 if (in_lane and self._pulling_away_accel(i)) else 0
      if not gates:
        self._release("cruise off / brake" if not (i.cruise_enabled and not i.brake_pressed) else "openpilot disengaged", False)
      elif self.pull_accel_frames >= _frames(PULL_AWAY_ACCEL_TIME):
        self._release(PULLING_AWAY)
      elif self.moving_frames >= _frames(RELEASE_MOVING_TIME):
        self._release("lead moved away (vision)")
      elif i.v_ego < EGO_STOPPED_SPEED:
        self._release("ego stopped behind the lead")
      elif self.radar_match_frames >= _frames(RADAR_RESTORE_TIME):
        self._release("radar following the car ahead at our speed")
      elif self.leave_frames >= _frames(LEAVE_TIME):
        self._release("lead leaving the lane", False)
      elif self.out_of_lane_frames >= _frames(RELEASE_LOST_TIME):
        # a "clear" release: the restore uses SET+ holds only while no slower car is in our lane (clear_ahead)
        self._release("lead left the lane", False)
      elif self.lost_frames >= _frames(RELEASE_LOST_TIME):
        self._release("lead lost (vision)", False)
    if self.active and self.early and not self.confirmed and self.kind == STOP_WITH_CAR:
      # (the early start's car; a guarded stop ahead becomes a stop with car once a slower car is seen, not before, and
      # that car only confirms it while the model wants to slow down now: drive 50 t 109.7, the turning car was 89 %
      # a car and 10 mph slower, the model's desired acceleration -0.05)
      sure = (i.lead_alive and i.lead_status and i.lead_prob >= EARLY_CONFIRM_PROB and
              (not self.warn_unsure or self._model_slowing(i)))
      self.confirm_frames = self.confirm_frames + 1 if sure else 0
      if self.confirm_frames >= _frames(EARLY_CONFIRM_TIME) or self.urgent:
        self.confirmed = True
        self.confirm_frame = self.frame
    if self.active and self.warn_unsure:
      # the guarded stop ahead: the stop-light detector agreeing or the plan holding the stop long enough confirms it
      # (a car confirming it: the early start rule above)
      if not self.confirmed and (i.red_light or self.latch_frames >= _frames(WARN_GUARD_PLAN_HOLD)):
        self.confirmed = True
        self.confirm_frame = self.frame
      if self.confirmed:
        self.warn_unsure = False
        self.floor_mode = self.floor_mode or (self.kind == STOP_AHEAD and WARN_FLOOR)
    if self.active and i.lead_status and self.kind not in TAP_KINDS:
      self.target = stopped_car_target(i.lead_d)
      self.floor_mode = self.floor_mode or (FLOOR_MODE_ENABLED and self.confirmed and 0 < i.lead_d <= FLOOR_DISTANCE)
      if self.floor_mode:
        lead_soon = i.lead_v + min(i.lead_a, 0.0) * FLOOR_LEAD_HORIZON if FLOOR_LEAD_GUARD else 0.0
        floor = max(FLOOR_SPEED, lead_soon)
        moving = (MOVING_LEAD_GUARD and not self.floor_committed and self.kind == STOP_WITH_CAR and
                  self.lead_v_f >= MOVING_LEAD_SPEED and self.lead_a_f > MOVING_LEAD_DECEL)
        if moving:
          t = max(floor, self.lead_v_f - MOVING_LEAD_MARGIN)
          self.moving_target = t if self.moving_target is None else min(self.moving_target, t)
          floor = self.moving_target
        elif MOVING_LEAD_GUARD:
          self.floor_committed = True          # it slowed, brakes or a stop is planned: the floor from now on
        self.target = min(self.target, floor)
    if self.active and self.kind == STOP_AHEAD:
      if self.warn_unsure:
        self.target = self.unsure_target     # the guarded stop ahead: 1 mph taps, the unsure car's cap
      else:
        self.target = FLOOR_SPEED if WARN_FLOOR else stopped_car_target(max(i.plan_stop_d, 0.0))
        if self.far_stage:
          self.target = self.far_target  # (FAR_STOP_*) the distance target first
    if self.active and self.kind == STOP_WITH_CAR and self.warn_unsure:
      self.target = max(self.target, self.unsure_target)   # ... its car, until it confirms the stop: the same cap
    if self.active and self.early and not self.confirmed and self.lost_frames > _frames(EARLY_DROPOUT):
      self.target = 100.0            # not confirmed and the car is not seen now: hold the speed, do not lower further
    if self.active and self.kind == STOP_WITH_CAR:
      # a stop with a much slower car: slow down at once (holds allowed); an early start only taps until confirmed
      self.urgent = self.confirmed
    if self.active and self.floor_mode:
      self.urgent = True             # fast pace and holds, both kinds
    if self.active and self.kind == STOP_AHEAD and self.warn_unsure:
      self.urgent = False
    if self.active and self.kind in TAP_KINDS:
      self._update_catch_up(i)
    if self.active and self.kind == SLOW_UNSURE:
      # 1 mph taps, at most UNSURE_MAX_DROP; none while the car is not seen (as an unconfirmed early start); first the
      # drop to our speed when the cruise was still accelerating (CATCH_UP_*)
      seen = self.lost_frames <= _frames(EARLY_DROPOUT)
      self.target = (self.tap_base if self.catch_up else self.unsure_target) if seen else 100.0
      self.urgent = self.catch_up and seen
    if self.active and self.kind == RED_LIGHT:
      # the head start: (the drop to our speed first, CATCH_UP_*,) one 1 mph step at once and one more every
      # RED_STEP_TIME, down to the cap
      steps = (self.frame - self.step_frame) // _frames(RED_STEP_TIME) + 1
      self.target = self.red_base if self.catch_up else max(self.red_base - steps * MS_PER_MPH, self.red_cap)
      self.urgent = self.catch_up
    closing_f = i.v_ego - self.lead_v_f
    handles = (RADAR_FOLLOW_ENABLED and self.radar_follow and i.lead_status and
               (closing_f <= RADAR_FOLLOW_CLOSING or i.a_ego <= RADAR_BRAKING_ACCEL or self.required < RADAR_YIELD_DECEL))
    self.radar_hold = self.active and handles
    if self.radar_hold:
      self.target, self.urgent = 100.0, False    # the MRCC handles the car ahead: no SET- (and no SET+ while active)
    # a stop ahead released with the light still on: no restore until it has gone, his gas, the model's go or the limit
    going_free = i.model_alive and i.model_accel > GO_ACCEL and not i.plan_stop
    self.go_free_frames = self.go_free_frames + 1 if (going_free and not self.active) else 0
    if self.red_release_frame is not None and (
        self.active or self.red_gap >= _frames(RED_DROPOUT) or i.gas_pressed or
        self.go_free_frames >= _frames(GO_TIME) or self.frame - self.red_release_frame > _frames(RED_RELEASE_HOLD_MAX)):
      # the light gone or the model's go: the restore at once (as the release would have); his gas: his; the limit: the
      # normal (gentle) restore
      self.restore_now = self.restore_now or (not self.active and (self.red_gap >= _frames(RED_DROPOUT) or
                                                                 self.go_free_frames >= _frames(GO_TIME)))
      self.red_release_frame = None
    self.hold_restore = EARLY_START_ENABLED and (self.latched or self.red_release_frame is not None) and not self.active
    self.clear_ahead = bool(not self.active and self.clear_release and not self._slower_car_ahead(i))
    seen = i.lead_alive and i.lead_status and i.lead_prob >= CLEAR_MIN_PROB and abs(self._y_path(i)) <= CLEAR_LATERAL
    pull = not self.active and (not seen or self.lead_v_f >= i.v_ego + PULL_AWAY or self._pulling_away_accel(i))
    self.pull_frames = self.pull_frames + 1 if pull else 0
    self.pulling_away = self.pull_frames >= _frames(PULL_AWAY_TIME)
    self.slower_after_red = bool(not self.active and self.release_reason == RED_GONE and self._slower_in_lane(i))
    return self.active

  def _start(self, i, lead_ok, early_start, warn_start):
    """A stopped car / stop with car / stop ahead episode starts (or takes over an unsure car episode)."""
    took_over = self.active and self.kind in TAP_KINDS
    from_red = took_over and self.kind == RED_LIGHT
    self.rearm_on_red = False
    self.red_absent_frames = 0
    self.red_release_frame = None
    self.catch_up = False                      # (a tap episode's first drop: the new kind's own target from here)
    self.caught_up = self.caught_up and took_over
    self.active = True
    self.early = self.hold_frames < _frames(HOLD_TIME)
    self.warn_start = not (self.hold_frames >= _frames(HOLD_TIME) or early_start)
    self.kind = STOPPED_CAR if (lead_ok and not self.early) else (STOP_AHEAD if self.warn_start else STOP_WITH_CAR)
    # the false stop guard: a stop ahead start the stop-light detector does not back (redLight not on within
    # WARN_RED_RECENT) and no slower car confirmed: taps only until it is confirmed (WARN_GUARD_*)
    self.warn_unsure = bool(WARN_GUARD_ENABLED and self.warn_start and
                            self.frame - self.last_red_frame > _frames(WARN_RED_RECENT) and not self._slow_car(i))
    self.warn_guarded = self.warn_unsure
    if self.warn_unsure:
      # the cap: the taps so far stay (an unsure car / head start taken over), otherwise the unsure car's
      if from_red:
        self.unsure_target = self.red_cap
      elif not took_over:
        self.unsure_target = max(max(i.v_ego, i.set_speed) - UNSURE_MAX_DROP, FLOOR_SPEED)
    self.confirmed = not self.early or (self.warn_start and not self.warn_unsure)   # the warning start: holds and the fast pace at once
    self.confirm_frames = 0
    self.confirm_frame = self.frame if self.confirmed else -1
    self.floor_mode = self.warn_start and WARN_FLOOR and not self.warn_unsure
    # a far camera-only stop ahead: the distance target first (FAR_STOP_*)
    d_stop = self._d_stop(i)
    self.far_stage = self.far_staged = bool(FAR_STOP_ENABLED and self.floor_mode and self.kind == STOP_AHEAD and
                                            d_stop >= FAR_STOP_DISTANCE)
    self.far_frame = self.frame
    self.far_fast = i.v_ego >= FAR_STOP_RED_SPEED
    # (second review, V2) under the light at FAR_STOP_RED_SPEED or more: the plan's own stop distance, not the extrapolated
    red_d = i.plan_stop_d if (FAR_STOP_RED_PLAN_D and i.red_light and self.far_fast and i.plan_stop_d > 0) else d_stop
    self.far_target = self._far_target(red_d) if self.far_stage else 0.0
    self.far_plan = self.far_window = 0
    if self.warn_start:
      self.warn_latched = True
      self.warn_pending = self.warn_frames < _frames(WARN_DEBOUNCE)
      self.warn_plan_frames, self.warn_plan_gap = self.warn_frames, 0
    else:
      self.warn_pending = False
    self.y_base = abs(self.y_f) if (self.y_f is not None and i.lead_status) else None
    self.leave_frames = 0
    self.clear_release = False
    self.moving_target = None
    self.floor_committed = False
    self.moving_match_frames = 0
    self.gas_frames = 0
    if not took_over:
      self.start_frame = self.frame            # (an unsure car start: the episode began with its taps)
    self.unsure_from = took_over
    self.lost_frames = self.out_of_lane_frames = self.go_frames = self.moving_on_frames = 0
    self.early_frames = self.early_gap = 0
    self.unsure_frames = self.unsure_gap = 0
    self.pull_accel_frames = 0
    self.release_reason = None

  @staticmethod
  def _d_stop(i):
    """The planned stop's distance extrapolated from the plan's deceleration to its slowest point (FAR_STOP_*): v^2 / (2
    decel) + 2; 0 (near) when unknown."""
    v, d = i.v_ego, i.plan_stop_d
    if d <= 0 or v <= 0.5:
      return 0.0
    v_min = max(i.plan_ratio, 0.0) * v
    decel = max((v * v - v_min * v_min) / (2 * max(d - PLAN_STOP_BUFFER, 1.0)), FAR_STOP_MIN_PLAN_DECEL)
    return v * v / (2 * decel) + PLAN_STOP_BUFFER

  @staticmethod
  def _far_target(d_stop):
    v = math.sqrt(2 * FAR_STOP_DECEL * max(d_stop, 0.0))
    return max(FLOOR_SPEED, math.ceil(v / MS_PER_MPH - 1e-6) * MS_PER_MPH)

  def _update_far_stage(self, i):
    """The far stop ahead's stage (FAR_STOP_*): its distance target, then the floor or the end at FAR_STOP_STAGE."""
    age = self.frame - self.far_frame
    if i.plan_stop:
      d_stop = self._d_stop(i)
      if d_stop < FAR_STOP_DISTANCE:
        self.far_stage = False         # close now: the floor at once
        return
      red_on = self.red_gap < _frames(RED_DROPOUT)
      red_d = i.plan_stop_d if (FAR_STOP_RED_PLAN_D and red_on and self.far_fast and i.plan_stop_d > 0) else d_stop
      self.far_target = min(self.far_target, self._far_target(red_d))
    if age >= _frames(FAR_STOP_WINDOW_START):
      self.far_window += 1
      self.far_plan += bool(i.plan_stop)
    if age >= _frames(FAR_STOP_STAGE):
      red_on = self.red_gap < _frames(RED_DROPOUT)
      share = FAR_STOP_RED_PLAN_SHARE if (self.far_fast and red_on) else FAR_STOP_PLAN_SHARE
      unconfirmed = self.far_plan < share * max(self.far_window, 1)
      # (below FAR_STOP_RED_SPEED with the light on: the floor, the red light rules as before)
      if unconfirmed and (self.far_fast or self.red_gap >= _frames(RED_DROPOUT)):
        self._release(FAR_STOP_UNCONFIRMED)
      self.far_stage = False

  def _unsure_car(self, i, any_prob=False):
    """The triangle: a car the camera shows (any confidence) in our path, much slower than us (UNSURE_*). any_prob: a
    car the camera is sure of counts too (the red light gone restore, RED_GONE)."""
    return (UNSURE_ENABLED and not self.left_car and not self.overridden and not self.radar_follow and
            i.lead_alive and i.lead_status and 0 < i.lead_d <= UNSURE_MAX_DISTANCE and
            abs(self._y_path(i)) <= UNSURE_LATERAL and i.v_ego - self.lead_v_f >= UNSURE_SLOWER and
            self.lead_v_f > -STOPPED_SPEED and          # not an oncoming car (review)
            i.v_ego >= UNSURE_MIN_SPEED and (any_prob or i.lead_prob < MIN_PROB) and
            (i.v_ego >= CATCH_UP_LOW_SPEED or (self.unsure_a_f <= UNSURE_LOW_MAX_ACCEL and not self._pulling_away_accel(i))))

  def _slower_in_lane(self, i):
    """A slower car in our lane for the red light gone restore: the clear-ahead check's, or one the unsure car takes."""
    return self._slower_car_ahead(i) or self._unsure_car(i, any_prob=True)

  def _start_red(self, i):
    """The red light head start: 1 mph steps while only the stop-light detector says stop (RED_*)."""
    self.active = True
    self.kind = RED_LIGHT
    self.early, self.confirmed, self.warn_start, self.warn_pending = False, False, False, False
    self.warn_unsure = self.warn_guarded = False
    self.confirm_frames, self.confirm_frame = 0, -1
    self.floor_mode = False
    self.unsure_from = False
    self.red_base, self.red_cap = self._tap_reference(i, RED_MAX_DROP)
    self.y_base = abs(self.y_f) if (self.y_f is not None and i.lead_status) else None
    self.leave_frames = 0
    self.clear_release = False
    self.moving_target = None
    self.floor_committed = False
    self.moving_match_frames = 0
    self.gas_frames = 0
    self.start_frame = self.frame
    self.lost_frames = self.out_of_lane_frames = self.go_frames = 0
    self.release_reason = None

  def _tap_reference(self, i, max_drop):
    """A red light head start / unsure car start: (its reference speed, its cap). The set speed more than CATCH_UP_GAP
    above ours (the cruise still accelerating): the first action is the drop to our speed and the cap is max_drop below
    it; from CATCH_UP_LOW_SPEED the cap stays max(ours, set) - max_drop and the first drop stops there if that is above
    our speed. Otherwise max(ours, set), the cap max_drop below it."""
    self.catch_up = self.caught_up = bool(CATCH_UP_ENABLED and i.set_speed > i.v_ego + CATCH_UP_GAP)
    self.catch_frame = self.step_frame = self.frame
    ref = max(i.v_ego, i.set_speed)
    cap = ref - max_drop
    if self.catch_up and i.v_ego < CATCH_UP_LOW_SPEED:
      ref = i.v_ego
      cap = ref - max_drop
    elif self.catch_up:
      ref = max(i.v_ego, cap)
    self.tap_base = ref
    return ref, max(cap, FLOOR_SPEED)

  def _update_catch_up(self, i):
    """The first drop is done once the set speed is within 1 mph of the reference speed, or after CATCH_UP_TIME; the
    1 mph steps' clock starts then."""
    if self.catch_up and (i.set_speed <= self.tap_base + MS_PER_MPH + 1e-3 or
                          self.frame - self.catch_frame >= _frames(CATCH_UP_TIME)):
      self.catch_up = False
      self.step_frame = self.frame

  def _update_red(self, i, gates):
    """A red light head start that no stop start has taken over (yet)."""
    if self.gas_frames >= _frames(GAS_RELEASE_TIME):
      self._release("driver on the gas", False)
    elif not gates:
      self._release("cruise off / brake" if not (i.cruise_enabled and not i.brake_pressed) else "openpilot disengaged", False)
    elif i.v_ego < EGO_STOPPED_SPEED:
      self._release("ego stopped behind the lead")
    elif self.red_gap > _frames(RED_DROPOUT):
      # the light is gone (green, or the detector changed its mind): the speed back at once unless a slower car (any
      # confidence the unsure car takes) is in our lane
      self._release(RED_GONE, not self._slower_in_lane(i))

  def _start_unsure(self, i):
    self.active = True
    self.kind = SLOW_UNSURE
    self.early, self.confirmed, self.warn_start, self.warn_pending = True, False, False, False
    self.warn_unsure = self.warn_guarded = False
    self.confirm_frames, self.confirm_frame = 0, -1
    self.floor_mode = False
    self.unsure_from = False
    # below the set speed (review: v_ego 30 mph under a 45 set gave 13 taps), never below our speed less the cap; with
    # the cruise still accelerating, the drop to our speed first and the cap below our speed (CATCH_UP_*)
    _, self.unsure_target = self._tap_reference(i, UNSURE_MAX_DROP)
    self.unsure_low = i.v_ego < CATCH_UP_LOW_SPEED
    self.pull_accel_frames = 0
    self.unsure_off_frames = self.unsure_sure_frames = 0
    self.y_base = abs(self.y_f) if (self.y_f is not None and i.lead_status) else None
    self.leave_frames = 0
    self.clear_release = False
    self.moving_target = None
    self.floor_committed = False
    self.moving_match_frames = 0
    self.gas_frames = 0
    self.start_frame = self.frame
    self.lost_frames = self.out_of_lane_frames = self.go_frames = 0
    self.release_reason = None

  def _update_unsure(self, i, gates):
    """An unsure car episode that no stop start has taken over (yet)."""
    seen = i.lead_alive and i.lead_status and 0 < i.lead_d <= UNSURE_MAX_DISTANCE
    self.lost_frames = 0 if seen else self.lost_frames + 1
    stopping = i.plan_stop or self._model_slowing(i) or i.lead_a <= -RADAR_MATCH_LEAD_DECEL
    away = seen and (abs(self._y_path(i)) > UNSURE_LATERAL + 0.5 or
                     (i.v_ego - self.lead_v_f < GO_SLOWER and not stopping) or
                     (self.unsure_low and self.unsure_a_f > UNSURE_LOW_MAX_ACCEL and not stopping))
    self.unsure_off_frames = self.unsure_off_frames + 1 if away else 0
    self.pull_accel_frames = self.pull_accel_frames + 1 if (seen and self._pulling_away_accel(i)) else 0
    # sure and moving: a slower moving car (the MRCC follows it); sure and stopped: keep the taps' target until a
    # stop start takes over (review: a stopped car on a curve, raw yRel 1.4, was released and the speed restored)
    sure = seen and i.lead_prob >= MIN_PROB and self.lead_v_f > STOPPED_SPEED
    self.unsure_sure_frames = self.unsure_sure_frames + 1 if sure else 0
    if self.gas_frames >= _frames(GAS_RELEASE_TIME):
      self._release("driver on the gas", False)
    elif not gates:
      self._release("cruise off / brake" if not (i.cruise_enabled and not i.brake_pressed) else "openpilot disengaged", False)
    elif self.unsure_low and self.pull_accel_frames >= _frames(PULL_AWAY_ACCEL_TIME):
      self._release(PULLING_AWAY)
    elif self.radar_match_frames >= _frames(RADAR_RESTORE_TIME):
      self._release("radar following the car ahead at our speed")
    elif self.unsure_off_frames >= _frames(UNSURE_LOST_TIME):
      self._release(UNSURE_GONE)                 # a false alarm (it left our path / sped up): the speed back at once
    elif self.lost_frames >= _frames(RELEASE_LOST_TIME):
      self._release(UNSURE_GONE, False)          # not seen: a far low-confidence lead blinks; no SET+ hold at once
    elif self.unsure_sure_frames >= _frames(UNSURE_SURE_TIME):
      self._release(UNSURE_NOT_STOP, False)      # a slower moving car: the MRCC follows it

  def _pulling_away_accel(self, i):
    """The car ahead accelerating away from about our speed, no stop planned (PULL_AWAY_ACCEL_*)."""
    faster = self.lead_v_f >= (i.v_ego - PULL_AWAY_SLOWER if self.radar_follow else i.v_ego + PULL_AWAY)
    return (PULL_AWAY_ACCEL_ENABLED and i.lead_alive and i.lead_status and i.lead_prob >= MIN_PROB and
            not i.plan_stop and i.model_alive and i.model_accel > GO_ACCEL and
            self.frame - self.pull_lead_since >= _frames(PULL_AWAY_SAME_CAR) and
            i.lead_a >= PULL_AWAY_ACCEL and self.lead_v_f >= PULL_AWAY_MIN_SPEED and faster)

  def _warn_still_planned(self, i):
    """A start before the warning: the planned stop (the model's plan alone: our own slowdown makes the "not
    slowing" gate false) must last until the warning would show, with the warning's dropout."""
    if i.plan_stop:
      self.warn_plan_frames += 1
      self.warn_plan_gap = 0
    else:
      self.warn_plan_gap += 1
    if self.warn_plan_frames >= _frames(WARN_DEBOUNCE):
      self.warn_pending = False
      return True
    return self.warn_plan_gap <= _frames(WARN_DROPOUT)

  def _release(self, reason, restore_now=True):
    go_kind = STOP_CAR_RED_HOLD and self.kind == STOP_WITH_CAR and reason in (TRAFFIC_MOVING, "at the moving car's speed",
                                                                            PULLING_AWAY, CAR_AHEAD_MOVING)
    # (review of round k, B1) a far stop's releases never restore while the light is on, whatever the speed: the light
    # alone only no longer takes / holds the floor at FAR_STOP_RED_SPEED or more (red_holds, _update_far_stage)
    if (reason in (STOP_AHEAD_CLEARED, STOP_AHEAD_UNCONFIRMED, FAR_STOP_UNCONFIRMED) or go_kind) and \
       self.red_gap < _frames(RED_DROPOUT):
      restore_now = False              # (RED_RELEASE_HOLD_MAX) the light still on: the lowered set speed stays for now
      self.red_release_frame = self.frame
    self.active = False
    self.far_stage = False
    self.hold_frames = 0
    self.moving_frames = 0
    self.go_frames = self.moving_on_frames = 0
    self.release_reason = reason
    self.restore_now = restore_now
    self.clear_release = reason in CLEAR_RELEASE_REASONS
    self.left_car = reason == "lead leaving the lane"
    self.overridden = reason == "driver on the gas"
    self.warn_pending = False
    self.warn_unsure = False
    self.catch_up = False
    self.red_latched = self.red_gap <= _frames(RED_DROPOUT)   # the light must go off before a red light head start
    self.release_frame = self.frame
    if reason in ("cruise off / brake", "openpilot disengaged", "ego stopped behind the lead", "driver on the gas") or (
        self.rearm_used_frame is not None and self.frame - self.rearm_used_frame > _frames(REARM_CHAIN_TIME)):
      self.rearm_used_frame = None     # a new chain: the restart at once is available again
    self.rearm_on_red = reason in (STOP_AHEAD_CLEARED, STOP_AHEAD_UNCONFIRMED) and self.rearm_used_frame is None
    if restore_now and self.frame - self.last_plan_stop > _frames(LATCH_DROPOUT):
      # a release that restores at once with no stop planned now: the latch goes with it, so the restore is not held
      # back as "a stop planned ahead" (hold_restore, which the controller's restore_now respects); a plan that comes
      # back latches again after LATCH_DEBOUNCE. With the planned stop still showing the latch stays and holds it.
      self.latched = False
      self.latch_frames = 0
    if reason != RED_GONE:
      # no unsure car start for UNSURE_REARM after any release but the head start's: when the light goes, a slower car
      # seen next is the queue the light was for (drive 52: the light went at 187.8 s, the car in our lane was seen at
      # 188.0; the rearm would have kept it out until 190.8, while the restore gave the cruise back its 35 mph)
      self.unsure_release_frame = self.frame
    self.pull_accel_frames = 0
    self.overridden_v = self.lead_v_f
    self.leave_frames = 0
    self.radar_flicker = False
    self.radar_match_frames = 0
    self.radar_hold = False

  def near_miss_candidate(self, i):
    stopped = (i.lead_alive and i.lead_status and i.lead_v <= NEAR_MISS_LEAD_SPEED and
               0 < i.lead_d <= NEAR_MISS_DISTANCE and i.v_ego >= NEAR_MISS_EGO_SPEED)
    return stopped or (self._model_slowing(i) and self._slow_car(i, max_lateral=3.0, min_prob=0.2) and
                       i.v_ego >= NEAR_MISS_EGO_SPEED)
