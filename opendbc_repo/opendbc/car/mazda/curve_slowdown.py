"""
Mazda Gen1 curve slowdown using the stock cruise SET-/SET+ buttons.

On Gen1 Mazdas without a Radar Interceptor, openpilot cannot command acceleration;
the stock MRCC always controls speed. This helper lowers the stock cruise set speed a
little before curves by sending the same SET- button message the steering wheel sends,
then restores it with SET+ after the curve. The stock cruise still does all braking.

The decision logic (CurveSlowdownLogic) has no openpilot dependencies so it can be unit
tested. The CAN timing follows sunnypilot's Mazda ICBM implementation (a message at
least 0.2 s apart, counter offsets [1, 1, 0, skip]).

Design rules (each enforced by the logic itself, not by timing assumptions):
  * One unconfirmed press at a time. After a press nothing else is sent until the set speed
    moves one step in that direction or PENDING_TIMEOUT passes. The car ignores roughly one
    press in four (seen on the road), so every press stays "in flight" for LATE_WINDOW after
    that: a late landing is still ours, and the next press is only sent if the limits below
    hold even if every press in flight still lands. A lost or slow press can only make the
    system do less, never more, and we never send in the opposite direction while a press
    is in flight.
  * The driver's own SET+/SET- taps are read from the stock steering wheel switch message, so a
    tap in the same direction as our presses is not mistaken for one of ours landing. Any other
    set speed change that is not exactly our pending step is the driver too: we stop
    and leave the set speed where the driver put it. A change that is not a whole number
    of steps is also the driver's (the Mazda minimum set speed is 30 km/h = 18.6 mph, so the
    first tap above it is a 0.4 mph step) unless it arrives while our own press is in
    flight, which means the dash steps in another unit: then we switch off for the drive.
  * Never press SET+ if that could go above the driver's set speed (baseline).
    Never press SET- below max(driver's speed - max drop, MIN_SET_SPEED), and never below the
    curve target plus a margin (so FrogPilot's curve flag stays on through the curve).
  * The driver's speed is never re-derived from a lowered set speed. We keep `offset`, the
    number of steps we are currently below the driver's speed, for the whole drive; the
    driver's speed is always set_speed + offset * step. Only a driver change resets it.
  * Transient conditions (gas, a short planner gap, speed dipping, cruise or openpilot
    briefly off) only pause pressing. If cruise is re-engaged at the speed we left, we
    restore the driver's speed after the curve. After PAUSE_ABANDON seconds paused we
    forget and leave the set speed alone.
  * Button traffic is bounded: a SET- budget per curve (restoring is never cut short),
    and a back-off when the car ignores presses (give up after NO_EFFECT_TRIPS back-offs).
"""
import math
from dataclasses import dataclass, field

DT_CTRL = 0.01                     # car controller runs at 100 Hz
MS_PER_MPH = 0.44704
MS_PER_KPH = 1 / 3.6

BUTTON_GAP_FRAMES = 20             # never send messages closer than 0.2 s
COUNTER_OFFSETS = [1, 1, 0, None]  # sunnypilot Mazda ICBM pattern

MAX_DROP_CHOICES_MPH = (5, 10, 15)    # "MazdaCurveSlowdownMaxDrop" param index 0/1/2
DEFAULT_MAX_DROP_MPH = 10
MIN_SET_SPEED = 25.0 * MS_PER_MPH     # never set cruise below 25 mph
START_SPEED = 25.0 * MS_PER_MPH       # only start a slowdown at 25 mph or more (true speed)
KEEP_SPEED = 12.0 * MS_PER_MPH        # keep managing (incl. restoring) down to 12 mph (a 20 mph set speed reads ~18 true)
RESTORE_MOVING_SPEED = 5.0            # m/s: after stopping behind a car, the driver's speed comes back once we are
                                      # clearly rolling again (not while creeping in a queue) ...
STANDSTILL_RESTORE_DELAY = 1.0        # s: ... or once we have been stopped this long. Checked on the owner's car
                                      # (2018 CX-5): SET+ at a stop behind a car only raises the set speed, the
                                      # MRCC keeps holding behind the car ahead.
CURVE_MARGIN = 0.3                    # m/s: stay this far above the curve target
CRUISE_SETTLE = 0.3                   # s: cruise must be engaged this long before we press
RESTORE_DELAY = 0.5                   # s of FrogPilot saying the curve no longer limits us before raising
FLICKER_RESTORE_DELAY = 2.0           # used instead while the curve flag has just flickered back on
FLICKER_WINDOW = 3.0                  # s: flag coming back on this soon after going off = flicker
MIN_DWELL = 1.0                       # s at least between the last SET- and a SET+
# PENDING_TIMEOUT: the feature logs of drives 33-3d (806 taps): the car answered in 0.10 s typically, 0.22 s for 99 %,
# so an ignored tap (about 1 in 4) now costs 0.3 s instead of 0.5 s; later answers are still ours (LATE_WINDOW).
PENDING_TIMEOUT = 0.3                 # s to wait for a press to show on the set speed
LATE_WINDOW = 3.0                     # s after the timeout during which a late change is still ours (car speed unknown)
LATE_FACTOR = 4                       # once the car's answer time is known, a press is lost after this many times it
MIN_IN_FLIGHT_WINDOW = 1.0            # s: a press is never forgotten sooner than this
DRIVER_RECENT = 1.5                   # s after the driver's last SET+/SET- edge (or while held): changes are theirs
MAX_IN_FLIGHT = 3                     # presses that may be unaccounted for before we stop sending
# Lost taps (drive 54, t 1244.6-1252): the red light head start's fast taps lost two (the car ignored them); every newer
# landing is booked to the oldest press, so two presses stayed "in flight" for the whole stop and the confirmed floor
# hold (which needed nothing in flight) never started: 49 -> 33 by 1 mph taps in 6.5 s instead of one hold to 19. A hold
# (stop, firm curve or restore) may now start while only our own taps in its direction past PENDING_TIMEOUT are in
# flight: they stay in flight (the limits still assume they land, a late landing is booked as before, the hold's bound
# counts them) and only no longer keep the hold back. A confirmed floor stop that would hold but for a fresh press in
# flight waits for it up to STOP_HOLD_WAIT instead of tapping meanwhile. (Moving such taps to `expired` once a newer
# one lands was tried: with answers slower than PENDING_TIMEOUT the late one then lands uncounted and the randomized
# limit tests go 1 step past the driver's speed / max drop.)
STOP_HOLD_WAIT = 0.3                  # s
NO_EFFECT_LIMIT = 6                   # presses in a row with no effect before backing off (1 in 4 ignored is normal)
NO_EFFECT_BACKOFF = 10.0              # s to wait after that before trying again
NO_EFFECT_TRIPS = 3                   # back-offs before giving up (speed left alone)
PAUSE_ABANDON = 30.0                  # s paused before we forget the driver's speed
# A driver SET tap before we lowered anything for the current curve / stopped car (drive 44: SET- 28 s and SET+ 18 s
# before a 30 mph curve the map had already announced) only stands us down this long, not for the whole curve.
DRIVER_REARM = 8.0                    # s without a driver SET button, nothing of ours lowered or in flight
DRIVER_QUIET = 1.0                    # s: none of our frames while the driver is on a SET button, nor this soon after
EXTRA_LOWER_PRESSES = 6               # SET- budget per curve = max drop steps + this (covers ignored presses)

# Stopped car ahead (shares this controller with the curve slowdown)
# floor for a stopped car: the car's own 30 km/h minimum (18.6 mph, the dash shows 19); the long press stops at 20
# and one SET- tap lands on the minimum (owner, Nov 6: "one more 1 mph step down to 19, every mph matters")
STOP_MIN_SET_SPEED = 30.0 / 3.6
MAX_STOP_PRESSES = 30                    # SET- budget (taps + holds) per stopped car
RELAXED_GAP_FRAMES = 50                  # 0.5 s between taps when the stopped car is not urgent
# Long press. Measured from the owner's own holds in the logs (5 holds, routes 31/32): on button-down
# the set speed drops 1 mph at once, ~0.5 s later it jumps to the next multiple of 5 mph, then 5 mph
# every 0.6 s while the button is held. What happens after release (how many more 5 mph steps can
# still land) is not visible in the logs, so it is assumed to be one quantum until measured, and every
# downward change during a hold and HOLD_SETTLE after it is ours.
LONG_PRESS_ENABLED = True
HOLD_FRAME_GAP = 10                      # frames between the hold's SET frames (stock 10 Hz cadence)
# Phase-locked holds (drive 44): the wheel keeps sending "not pressed" frames at 10 Hz during our hold, so the car
# mostly read our holds as quick taps (1 mph every 0.1-0.3 s). The one hold it took as a long press (2:05,
# 34 -> 33 -> 30 -> 25) had our frames ~5 ms after each wheel frame; failed ones were 15-80 ms after. So a hold frame
# now goes out on the cycle a new wheel frame is first seen (CTR = wheel + 1), one per wheel frame. Experimental: every
# hold logs its delays and whether it landed as a long press ("quantum") or as taps.
HOLD_PHASE_LOCK = True
HOLD_COUNTER_OFFSET = 0                  # create_button_cmd adds 1: CTR = wheel CTR + 1
HOLD_FALLBACK_FRAMES = 12                # no new wheel frame this long after our last hold frame: send anyway
# Drive 45: phase-locked holds landed as real long presses (45 -> 40 -> 35, 34 -> 30 -> 25) with nothing landing after
# the release, but each stopped at 35 / 25 because the release kept the old margin (two quanta) and the target
# followed the car ahead's speed, so there were two holds with a 1.5 s pause. With phase lock the margins come from
# phase-locked holds only: down 0 quanta (a stop hold aims at 20 mph; one quantum too many is clamped by the car at
# its 30 km/h minimum and handled), up 2 (never above the driver's speed) until a phase-locked hold of either
# direction has measured the car, then the worst seen, at least 1.
# Drives 44/45: nothing landed after any phase-locked release, so upwards the default is now 1 quantum (a restore hold
# stops 5 mph below the driver's speed, the last 5 are taps: the owner's request after drive 4b); a hold that measures
# more raises it (worst seen + 1), and a 1-quantum overrun at most lands on the driver's speed, never above.
LOCK_OVERRUN_DOWN_DEFAULT = 0
LOCK_OVERRUN_UP_DEFAULT = 1
# Owner's request (drive 45): after a stop, while the stock radar is following a car ahead, bring the speed back with
# SET+ holds (5 mph steps) instead of slow taps: the radar keeps the gap to that car, the set speed only caps it.
# radar_lead is debounced by the stopped car trigger (radar_stable: on for 1 s, gaps up to 0.3 s).
RADAR_LEAD_FAST_RESTORE = True
HOLD_MAX_SECONDS = 6.0                   # a hold is released after this at the latest
HOLD_NO_EFFECT_SECONDS = 1.0             # a hold with no set speed change by then is released (the car ignores it)
HOLD_QUANTUM_MPH = 5                     # the car's long-press step
HOLD_SETTLE = 1.5                        # s after the last frame or landing during which changes are still the hold's
HOLD_OVERRUN_DEFAULT = 1                 # quanta that may still land after a SET- release, until measured
# HOLD_UP_OVERRUN_DEFAULT 2 = what a measured clean hold gives: every hold in the feature logs of drives 33-3d (9 holds)
# had nothing land after its release. With 3, the restore from a stop at 22 mph released its hold at 26 and tapped
# the last 19 mph (drive 3c).
HOLD_UP_OVERRUN_DEFAULT = 2              # assumed for SET+ holds until a hold has measured the car (then worst seen + 1)
HOLD_MISS_LIMIT = 2                      # holds with no effect before holds are off for the drive
HOLD_GOALS_MPH = (15, 10)                # stopped car, car not measured yet: first hold 15 mph down, the second 10 more
# SET- holds per stopped car; once the car's holds are measured, a hold aims straight at the floor (the release rule
# keeps the overrun margin), then taps
MAX_STOP_HOLDS = 2
CAR_MIN_SET_SPEED = 30.0 * MS_PER_KPH    # the car's own minimum set speed (18.6 mph): SET- below it lands short
# Restores (curve over, stopped car clearly gone, traffic moving again, the radar following the car ahead, a ramp):
# one SET+ hold up to a multiple of 5 at least _up_overrun() quanta below the driver's speed (with the default of one:
# 5 mph below), released when the next landing would pass that, then 1 mph taps; so the hold can never take the set
# speed past the driver's speed. A hold is only used when it gains at least RESTORE_HOLD_MIN_GAIN quanta over taps.
# Gentle (0.5 s taps, no holds) stays for the releases where the car may still be ahead: lead lost (vision), and lead
# left / leaving the lane, cruise off / brake or disengaged while a slower car is still in our lane (drive 4c: with
# nothing slower in our lane those restore with the hold, stop_clear_ahead), unless the radar follows a car.
RESTORE_HOLD_MIN_GAIN = 1
# Speed limit follow (speed_limit_follow.py, drives 4c, 52): a higher posted limit becomes the driver's speed (the
# anchor); the restore rule above then brings the set speed up: 5 mph or less by taps, more by one hold to 5 below, then
# taps. A lower one only stops a pending restore there (never a SET-), and while it is not steady yet it only holds our
# SET+ below it (limit_ceiling). The same raise takes the highway's limit on a ramp (drive 53: the merge, timed by the
# follower) and the street's after an exit.
LIMIT_DRIVER_QUIET = 3.0                 # s after the driver's last SET edge before a higher limit is adopted
# Fast restores at any speed (drive 4d, t 2017-2022 and 1825-1829): after a stop the restore waited until we were
# back at KEEP_SPEED (12 mph) while the car ahead was already driving off; the radar keeps the gap and the set speed
# only caps it (owner). A restore that may use holds (the car ahead clearly gone, nothing slower in our lane, or the
# radar following a car) now goes at any speed; SET- still needs START_SPEED / KEEP_SPEED.
FAST_RESTORE_ANY_SPEED = True
# SET+ on the gas (drive 4d, tag 3, t 1943-1954): after a green light the driver drove 10 s on the gas with the set
# speed at 20 (our restore waits while the gas is pressed). On this car a SET+ press while the car is faster than the
# set speed sets the current speed (tags 3/4: 20 -> 39 and 20 -> 38 at once, then the long press steps). So:
#  * any SET+ tap of ours that may jump (the car JUMP_STEPS or more faster than the set speed; both drive 4d jumps
#    landed on round(vEgo)) is booked as one jump of up to the driver's speed (it used to read as "more steps than
#    our presses" and switch everything off for the drive), and is only sent while where it may land (plus
#    GAS_JUMP_MARGIN on the gas: the car keeps accelerating until it lands) stays at or below the driver's speed;
#    no holds then;
#  * on the gas, once nothing slower is in our lane, SET+ taps (no holds, never SET-) restore too.
# (SET- while the car is faster than the set speed lands as a normal step: every floor hold and curve tap does that.)
GAS_JUMP_ENABLED = True
JUMP_STEPS = 2                           # the car this much faster than the set speed: a SET+ jumps
JUMP_UNCERT = 0                          # steps: the jump may land this much above round(vEgo) (drive 4d: 0)
GAS_JUMP_MARGIN = 1                      # steps, on the gas: landing + this <= the driver's speed
# ... but only on the gas (drive 4f, tag 3, t 185.5-188.4: off the gas, 8 SET+ taps of ours 12-14 mph slower than
# the car each landed +1 mph; booked as possible jumps they were taps only, no hold: 20 -> 45 took 8.8 s). Every jump
# on drives 4c, 4d and 4f (7) had the gas pressed within 0.5 s before it landed; none without. Off the gas a SET+ is
# a normal step, holds allowed. Still never a SET+ that could land above the driver's speed if it did jump, and a
# landing on the current speed from a SET+ of ours off the gas is still booked as ours (JUMP_OFF_GAS_CLAIM).
GAS_JUMP_RECENT = 1.0                    # s after the gas was last pressed during which a SET+ may still jump
# Never speed up inside a curve (drive 53, the on-ramp loop onto M-59, t 1019-1030): behind a car the set speed sat at
# 39-41 while the car went 27-28 mph into a 71 m loop; the cruise followed the car ahead up to 32 mph in the loop
# ("Turn Exceeds Steering Limit" twice). While the camera's curve target governs on a link road (map curves On: the
# map's road class for map_curve_control.LINK_HOLD), the target is also never above the current speed plus half a
# step, so the set speed ends at most a step above it; the ramp floor still holds. On this car the true speed holds at
# or a little above the dash set speed (drive 53: set 29 -> 29.4-29.5 mph, set 75 -> 75.0-75.9), so this does not
# chase the speed down. On a normal road the camera curve slowdown is as before (no extra SET- behind a slower car in a
# curve). A followed map curve (curve_track) has its own rule: never above the curve's speed, back up to it once the
# car is nearly there.
CURVE_NO_ACCEL = True
# The curve's restore on the gas (drive 53, t 1035-1060): after the on-ramp loop the set speed stayed 29 with the
# driver's 50 while he drove onto M-59 on the gas (27 -> 49 mph); the curve's "over" timer is frozen on the gas, so the
# restore only came at 1060.9, 11 s after the loop, and when he lifted at 1056 the car slowed toward 29 (-0.8 m/s^2).
# Our own curve target does not depend on the pedal, so once it has been off for RESTORE_DELAY (counted on the gas
# too; FLICKER_RESTORE_DELAY when the camera's target was the last one on, as it flickers) the restore goes on the gas
# as well: SET+ jumps to the current speed (never one that could land above the driver's speed, see GAS_JUMP_*), then
# taps.
CURVE_GAS_RESTORE = True
# A firm map curve (a ramp curve or an exit with too little room for the usual follow, curve_target.FIRM_*): the set
# speed has to go ~13 mph under the speed at once for the cruise's strongest slowdown (~1.0 m/s^2); taps take ~0.3 s a
# mph (one unconfirmed at a time), a SET- hold ~0.12 s (1 mph, then 5 mph every 0.6 s). So a firm curve that wants at
# least CURVE_HOLD_MIN_QUANTA below the set speed takes one SET- hold toward it (bounded by the target with the hold's
# overrun margin, as a stop hold), then taps; at most CURVE_HOLDS per curve.
CURVE_HOLD_ENABLED = True
CURVE_HOLD_MIN_QUANTA = 2
CURVE_HOLDS = 1
# A restore asked for at once (restore_pending: a stop ahead that cleared, a red light gone) while a followed map curve's
# target is on: the restore hold (RESTORE_HOLD_MIN_GAIN) runs up to the curve's target with its usual margin below it,
# then taps (never above the curve's target, see up_ok); before, inside any curve a restore only tapped (drive 54, t
# 764.7-768: 19 -> 26 in 3.3 s in a 42 mph curve, the car slowing 35 -> 30, 20 s before the real stop).
RESTORE_IN_CURVE_HOLD = True
# A lower posted limit's cap (rule 3) while a curve target is on: the driver's speed is where the curve's Max Drop floor
# counts from, so bringing it down there could lower the floor and cause a SET- for a lower limit
# (speed_limit_follow tests: a cap during a curve waits for it to end). It is applied in the curve too once the set
# speed is already at or below the lowest the curve could then ask for (the new driver's speed less the Max Drop and
# the road follower's undershoot, curve_target.MAP_ROAD_UNDERSHOOT): then it cannot cause a SET-. Drive 54, t 742-769:
# a 70 from the highway capped at 45 on the street never applied through a followed map curve, and the pending cap kept
# the in-curve restore hold (RESTORE_IN_CURVE_HOLD) out.
CAP_CURVE_MARGIN = 3 * MS_PER_MPH        # m/s (curve_target.MAP_ROAD_UNDERSHOOT)
SET_MINUS = "SET_MINUS"
SET_PLUS = "SET_PLUS"


def _frames(seconds):
  return int(round(seconds / DT_CTRL))


@dataclass
class SlowdownInputs:
  enabled: bool            # openpilot engaged (CC.enabled)
  cruise_enabled: bool     # stock MRCC engaged
  set_speed: float         # stock cruise set speed, m/s (dash units)
  v_ego: float             # true speed, m/s
  gas_pressed: bool
  brake_pressed: bool
  standstill: bool
  plan_alive: bool         # starpilotPlan is being received
  csc_active: bool         # FrogPilot Curve Speed Controller wants to slow down
  csc_speed: float         # Curve Speed Controller target, true speed m/s
  other_button_active: bool  # openpilot is sending CANCEL/RESUME this frame
  driver_button: object = None  # SET_MINUS / SET_PLUS while the driver holds that stock button, else None
  stop_active: bool = False     # stopped car ahead: lower the set speed towards stop_target
  stop_target: float = 0.0      # m/s, true speed; clipped to [STOP_MIN_SET_SPEED, driver's speed] here
  stop_urgent: bool = False     # fast pace for the stopped car
  stop_restore_now: bool = False  # the stopped car reason just cleared (lead moving, ego stopped, lead lost): restore at once
  curve_floor: float = 0.0        # m/s: when > 0 the curve may go this low (a ramp, a followed map curve's undershoot), past the max drop
  curve_budget: int = 0           # SET- presses allowed for this curve when more than the usual budget (a ramp)
  # csc_speed is a map curve follower's set speed command (map_curve_control.py): no margin on it, and when it comes
  # back up during the curve (the car is nearly at the curve's speed) the set speed is raised to it with SET+ taps at
  # once, not after the curve, so the car does not end up below the curve's speed (or the camera's lowest target in
  # that curve, margin included, when that is lower: map_curve_control.CurveMix)
  curve_track: bool = False
  curve_raise: bool = False       # (with curve_track) the follower has settled at the curve's speed: the raise may go
  # the camera's curve target governs on a link (map_curve_control.CurveMix.no_accel): inside the curve the set speed is
  # never left more than a step above the current speed, so the cruise does not speed up in it (CURVE_NO_ACCEL)
  curve_no_accel: bool = False
  # (with curve_track) the followed curve is firm (map_curve_control.MapOutput.firm): a SET- hold may take the set speed
  # down to its command (CURVE_HOLD_*)
  curve_hold: bool = False
  # (with curve_track) the governing target is the street speed (map_curve_control MapOutput.street: the exit's street,
  # speed_limit_follow rules 8 / 9): presses from KEEP_SPEED, not START_SPEED (his RES at a red light: second review)
  street_lower: bool = False
  fast_restore: bool = False      # restore at once with SET+ holds (a ramp merging onto a highway)
  hold_restore: bool = False      # a stop is planned ahead (stopped car trigger latched): keep the lowered speed for now
  btn_counter: int = -1           # the wheel's CRZ_BTNS counter (-1 unknown: holds are timed by HOLD_FRAME_GAP)
  radar_lead: bool = False        # the stock radar follows a lead (CRZ_CTRL.RADAR_HAS_LEAD, debounced: trigger.radar_stable)
  # the posted limit rose to this (m/s) with the driver's speed at the old one or up to 5 above, or a first limit
  # (speed_limit_follow.py rules 1, 2): make it the driver's speed
  limit_raise: float = 0.0
  # the posted limit fell to this (m/s) with the driver's speed at the old one (speed_limit_follow.py rule 3): a pending
  # restore stops here, or at the set speed if that is higher (never a SET- for it)
  limit_cap: float = 0.0
  # the posted limit went back down soon after our raise (speed_limit_follow.py rule 4): undo our raise (m/s: the
  # driver's speed before it, > 0 while offered)
  limit_undo: float = 0.0
  # a lower posted limit reported but not steady yet, the driver's speed in the band of the one before (m/s, 0 none,
  # speed_limit_follow.SpeedLimitFollower.ceiling_speed): no SET+ of ours above it while it is reported, never a SET-
  limit_ceiling: float = 0.0
  # stopped car trigger: released by brake / disengage / lane exit and no slower car is in our lane: the restore may
  # use SET+ holds (stopped_car_slowdown CLEAR_*)
  stop_clear_ahead: bool = False
  # stopped car trigger: no car ahead in our lane, or it is pulling away from us (restores at any speed)
  stop_pulling_away: bool = False
  # stopped car trigger: after a red light gone release a slower car (any confidence) is in our lane: no SET+ hold, no
  # SET+ at all until it is gone or the stop slowdown takes it on (stopped_car_slowdown RED_GONE, slower_after_red)
  stop_slower_ahead: bool = False


@dataclass
class Press:
  """One of our button actions on its way to the set speed."""
  direction: str
  frame: int
  reserve: int = 1         # steps this action may move the set speed (worst case)
  remaining: int = 1       # steps not yet seen on the set speed
  landed: int = 0
  hold: bool = False
  frames_sent: int = 0
  release_frame: int = -1  # hold: frame of the last SET- frame sent
  landings: list = field(default_factory=list)   # hold: [(frame, steps)]
  jump: bool = False       # a SET+ on the gas: the set speed jumps to the current speed (up to `reserve` steps, once)


class CurveSlowdownLogic:
  def __init__(self, is_metric: bool, max_drop_mph: float = DEFAULT_MAX_DROP_MPH):
    self.step = MS_PER_KPH if is_metric else MS_PER_MPH
    self.half_step = self.step / 2
    self.max_drop_steps = int(round(max_drop_mph * 1.609344)) if is_metric else int(round(max_drop_mph))
    self.max_lower_presses = self.max_drop_steps + EXTRA_LOWER_PRESSES
    self.frame = 0
    self.last_send_frame = -10_000
    self.button_frame = 0
    self.last_set_speed = None       # last set speed seen while cruise was engaged
    self.cruise_on_frames = 0
    self.curve_free_frames = _frames(RESTORE_DELAY)
    self.curve_free_gas = _frames(RESTORE_DELAY)       # the same, counted on the gas too (CURVE_GAS_RESTORE)
    self.curve_gas_camera = False                      # the last curve target on was the camera's (not a followed map curve)
    self.reason_free_frames = _frames(RESTORE_DELAY)   # frames with neither the curve nor a stopped car active

    # The driver's speed: the set speed they had before our presses moved it (None: we are not managing it).
    # A restore only brings the actual set speed back to it, never above (drive 2026-11-03: the driver brought the
    # speed back himself and the old step count added its steps on top). `offset` is derived from it.
    self.anchor = None
    self.prev_set_speed = None       # the set speed before the last observed change
    self.driver_events = []          # driver set speed changes while we managed it (for the per-event log)
    self.set_while_off = False       # the driver pressed SET- with cruise off: cruise engages at a new speed
    self.pending = None              # the newest Press, until it lands or times out
    self.inflight = []               # [Press] not yet (fully) seen on the set speed, oldest first
    self.latencies = []              # frames from our press to the set speed change, learned on the road
    self.expired = []                # [Press] given up on, still claimable for a while
    self.driver_taps = []            # [(direction, frame)] of the driver's own recent taps, not yet seen on the set speed
    self.prev_driver_button = None
    self.last_driver_edge = {SET_MINUS: -10_000, SET_PLUS: -10_000}
    self.last_gas_frame = -10_000    # last frame the gas was pressed (GAS_JUMP_RECENT)
    self.blocked_frame = None        # frame the current block started (DRIVER_REARM)
    self.rearms = 0                  # blocks lifted by DRIVER_REARM (logs)
    self.no_effect = 0
    self.no_effect_trips = 0
    self.backoff_until = 0
    self.lower_presses = 0           # SET- presses in the current curve
    self.stop_presses = 0            # SET- presses (taps + holds) for the current stopped car
    self.last_minus_frame = -10_000
    self.restoring = False
    self.blocked = False             # driver changed the speed: no lowering until every reason is over
    self.pause_frames = 0
    self.cruise_gap = False          # cruise was off since the last observed set speed
    self.flicker = False             # curve flag came back on shortly after going off (this episode)
    self.prev_csc_active = False
    self.disabled = False            # set speed steps don't match our unit: off for this drive

    # Long presses (stopped car): the staircase above is what the car did for the driver; whether it
    # answers our frames the same way is checked on the road, so every hold is logged with what landed
    # and when, any landing size is handled, and holds stop after HOLD_MISS_LIMIT holds with no effect.
    self.hold = None                 # the Press of the hold being sent, while frames are still going out
    self.prev_btn_counter = -1
    self.lock_overrun = {SET_MINUS: LOCK_OVERRUN_DOWN_DEFAULT, SET_PLUS: LOCK_OVERRUN_UP_DEFAULT}
    self.lock_measured = False       # a phase-locked hold has measured what lands after a release
    self.wheel_frame = -10_000       # frame a new wheel CRZ_BTNS frame was last seen
    self.new_wheel = False
    self.phase_lock = False
    self.holds_ok = True             # allowed in the current episode
    self.hold_misses = 0
    self.hold_overrun = HOLD_OVERRUN_DEFAULT   # quanta seen to land after a release (max), starts conservative
    self.hold_measured = False
    self.hold_log = []               # [dict] one per hold, for the per-event log
    self.holds_this_episode = 0      # SET- holds for the current stopped car (HOLD_GOALS_MPH)
    self.curve_holds = 0             # SET- holds for the current curve (CURVE_HOLDS)
    self.hold_kind = None            # "stop" / "curve" / "restore" / "restore_curve": what the hold in progress is for
    self.hold_bound = None           # set speed the current hold must not cross
    self.hold_wait_frame = None      # frame a confirmed floor stop began waiting for a press in flight (STOP_HOLD_WAIT)
    self.hold_margin = False         # the current hold is a restore hold (keeps a quantum of margin, see RESTORE_HOLD_*)
    self.stop_episode = False        # we lowered for a stopped car: the restore may use SET+ holds
    self.restore_pending = False     # the stopped car reason cleared for good: restore at once, even at standstill
    self.restore_in_curve = False    # restore_pending came while a curve target was on (capped by it, see update)
    self.standstill_frames = 0
    self.last_action = None          # "tap" / "hold" / "restore" of the last frame sent (for logs)
    self.limit_adopted = None        # (driver's speed before, new limit) on the frame a higher limit was adopted
    self.limit_cap = None            # a lower limit (m/s): no SET+ of ours above it until the driver's speed comes down
    self.limit_capped = None         # (driver's speed before, the cap) on the frame a lower limit capped it
    # our last raise for a posted limit (speed_limit_follow.py rule 4): {"frame", "before" (m/s), "new", "touched": the
    # driver changed his speed, braked or cruise was off since}
    self.limit_raised = None
    self.undo_cap = None             # rule 4 without the undo: like limit_cap at the driver's old speed, kept until applied
    self.limit_undone = None         # (driver's speed before, back to, "undo" / "cap") on the frame rule 4 was dealt with
    self.limit_ceiling = None        # this frame's provisional ceiling (SlowdownInputs.limit_ceiling) below the anchor
    # What the car profile (car_profile.py) remembers across drives: every hold whose landing after the release was
    # measured, ours (_finish_hold) and the driver's own (a held SET-/SET+ on the stock switch, watched below, so
    # Test mode learns the car too), is counted here with its quanta after the release.
    self.holds_learned = 0
    self.last_hold_learned = None    # (quanta after release, "ours" / "driver")
    self.driver_hold = None          # the driver's hold being watched: dict(dir, down, release, landed, after, last)

  @property
  def offset(self):
    """Steps the actual set speed is below the driver's speed (negative: above), 0 when not managing."""
    if self.anchor is None or self.last_set_speed is None:
      return 0
    return int(round(self.anchor / self.step)) - int(round(self.last_set_speed / self.step))

  @property
  def managing(self):
    return self.anchor is not None or bool(self.inflight)

  @property
  def in_flight(self):
    """Directions of presses sent but not yet seen on the set speed, oldest first."""
    return [p.direction for p in self.inflight]

  @property
  def holds_allowed(self):
    return LONG_PRESS_ENABLED and self.holds_ok and self.hold_misses < HOLD_MISS_LIMIT

  def _matching_press(self, direction):
    """Which press a one-step change in `direction` most likely is: ("driver", k) for driver_taps[k]
    when the driver tapped that button recently (their tap first: being wrong leaves one of ours to
    expire and ends one step below the driver's speed, the safe side), else ("ours", k) for the
    oldest of our presses in flight (FIFO: the count in flight is what the limits depend on, and
    keeping the newest only makes us wait longer), else None."""
    for k, (d, _) in enumerate(self.driver_taps):
      if d == direction:
        return ("driver", k)
    for k, p in enumerate(self.inflight):
      if p.direction == direction:
        return ("ours", k)
    return None

  def _lost_taps(self, direction, before):
    """Our taps in `direction` in flight, sent PENDING_TIMEOUT or more before frame `before`, not landed (ignored)."""
    return [p for p in self.inflight if p.direction == direction and not p.hold and not p.jump and p.remaining > 0 and
            before - p.frame >= _frames(PENDING_TIMEOUT)]

  def _only_lost_in_flight(self, direction):
    """Nothing in flight but lost taps of ours in `direction`: they do not keep a hold from starting (STOP_HOLD_WAIT)."""
    return bool(self.inflight) and len(self._lost_taps(direction, self.frame)) == len(self.inflight)

  @property
  def latency(self):
    """Typical frames from our press to the set speed change (median of what we have seen), or None."""
    if not self.latencies:
      return None
    return sorted(self.latencies)[len(self.latencies) // 2]

  def _in_flight_window(self):
    """Frames after a press during which its landing is still recognised as ours."""
    latency = self.latency
    if latency is None:
      return _frames(PENDING_TIMEOUT + LATE_WINDOW)
    return max(_frames(MIN_IN_FLIGHT_WINDOW), int(LATE_FACTOR * latency))

  def _cap(self):
    """The highest a SET+ of ours may take the set speed while a lower limit's cap (rule 3, or rule 4's) is pending."""
    caps = [c for c in (self.limit_cap, self.undo_cap) if c is not None]
    return min(caps) if caps else None

  def _capped_anchor(self):
    """The driver's speed with the pending cap applied: the cap, or the set speed if higher, never above it now."""
    return min(self.anchor, max(self._cap(), self.last_set_speed))

  def _cap_fits_curve(self):
    """Inside a curve the cap may apply only when the set speed is already at or below the lowest the curve could ask
    for from the capped driver's speed (its Max Drop floor less the road undershoot): then it cannot cause a SET-
    (CAP_CURVE_MARGIN)."""
    return self.last_set_speed <= self._capped_anchor() - self.max_drop_steps * self.step - CAP_CURVE_MARGIN + 1e-3

  def _ceiling(self):
    """The highest a SET+ of ours may take the set speed now: a pending cap, or a lower limit not steady yet."""
    caps = [c for c in (self._cap(), self.limit_ceiling) if c is not None]
    return min(caps) if caps else None

  def _forget(self):
    """Leave the set speed where it is and stop managing it (it becomes the driver's speed)."""
    self.anchor = None
    self.limit_cap = None
    self.undo_cap = None
    self.restoring = False
    self.no_effect = 0
    self.no_effect_trips = 0
    self.pause_frames = 0

  def _disable(self):
    self._forget()
    self.limit_raised = None
    self.pending = None
    self.hold = None
    self.inflight = []
    self.expired = []
    self.disabled = True

  def _apply_ours(self, direction, n=1):
    """One of our presses landed. The set speed itself says where we are (offset); a landing with nothing
    anchored (a press given up on landing late, or ours landing after the driver took over) anchors at the speed
    before it, so it is put right."""
    if self.anchor is None and self.prev_set_speed is not None:
      self.anchor = self.prev_set_speed
    self.no_effect = 0
    self.no_effect_trips = 0

  def _driver_change(self, direction, steps):
    """The driver changed the set speed while we may be managing it. SET+: they are taking back our lowering;
    the anchor stays (the rest is restored later) until they reach it, then there is nothing to restore. SET-:
    their new choice: we stop managing and leave it (a missed restore is the safe side)."""
    before = self.anchor
    self.restoring = False
    self.blocked = True
    if direction == SET_MINUS:
      self._forget()
      action = "adopted_lower"
    elif self.anchor is not None and self.last_set_speed >= self.anchor - self.half_step:
      ours_down = any(p.direction == SET_MINUS for p in self.inflight)
      self.anchor = self.last_set_speed if ours_down else None
      action = "reached_anchor"
    else:
      action = "kept" if self.anchor is not None else "none"
    if before is not None and len(self.driver_events) < 20:     # drained by the per-event log every frame
      self.driver_events.append({"dir": direction, "steps": steps, "set_before": self.prev_set_speed,
                                 "set_after": self.last_set_speed, "anchor_before": before, "anchor_after": self.anchor,
                                 "inflight": len(self.inflight), "action": action})

  def _quantum_steps(self):
    return int(round(HOLD_QUANTUM_MPH * 1.609344)) if self.step < MS_PER_MPH else HOLD_QUANTUM_MPH

  def _up_overrun(self):
    """Quanta assumed to land after a SET+ release: conservative until a hold has measured the car."""
    if self.phase_lock:
      return self.lock_overrun[SET_PLUS]
    return (self.hold_overrun + 1) if self.hold_measured else HOLD_UP_OVERRUN_DEFAULT

  def _down_overrun(self):
    """Quanta assumed to land after a SET- release."""
    return self.lock_overrun[SET_MINUS] if self.phase_lock else self.hold_overrun

  def _hold_reserve(self):
    """Steps a hold may still take after we stop sending (we release right after a quantum lands, so
    the next one is the overrun), plus the tap the button-down gives."""
    return self._down_overrun() * self._quantum_steps() + 1

  def _finish_hold(self, p):
    """A hold is resolved: learn how it landed."""
    if not p.hold:
      return
    q = self._quantum_steps()
    after = sum(n for f, n in p.landings if f > p.release_frame + HOLD_FRAME_GAP + _frames(0.2))
    quanta_after = -(-after // q)          # round up: a partial quantum after release counts as one
    locked = any(e.get("phase_lock") for e in self.hold_log if e["frame"] == p.frame)
    clamped = p.direction == SET_MINUS and self.last_set_speed is not None and \
      abs(self.last_set_speed - CAR_MIN_SET_SPEED) < self.half_step
    if locked and p.landed > q and clamped:
      # it ran onto the car's minimum: at least one quantum landed after the release, but how many is cut short,
      # so it measures nothing; upwards be wider than that (3 quanta) until a clean hold has measured the car
      self.lock_overrun[SET_MINUS] = max(self.lock_overrun[SET_MINUS], 1)
      if not self.lock_measured:
        self.lock_overrun[SET_PLUS] = max(self.lock_overrun[SET_PLUS], LOCK_OVERRUN_UP_DEFAULT + 2)
    elif locked and p.landed > q:
      # the car steps the same way in both directions: one measurement sets both margins
      self.lock_overrun[SET_MINUS] = max(self.lock_overrun[SET_MINUS], quanta_after)
      # upwards one more than seen, like _up_overrun for unlocked holds: never above the driver's speed
      up = quanta_after + 1 if not self.lock_measured else max(self.lock_overrun[SET_PLUS], quanta_after + 1)
      self.lock_overrun[SET_PLUS] = up
      self.lock_measured = True
    if p.landed > q:
      # A real staircase was seen: the overrun after a release is learned as the worst seen, never
      # below one quantum (the release always comes just after a landing, so at most the next one
      # should still land; more than that means the car keeps stepping, which we then assume).
      self.hold_overrun = max(1, self.hold_overrun if self.hold_measured else 0, quanta_after)
      self.hold_measured = True
      self.holds_learned += 1
      self.last_hold_learned = (quanta_after, "ours")
    elif p.landed == 0:
      self.hold_misses += 1
      self.holds_ok = False
    for entry in self.hold_log:
      if entry["frame"] == p.frame:
        entry.update(pattern="quantum" if any(n >= 2 for _, n in p.landings) else ("taps" if p.landed else "none"))
        entry.update(landed=p.landed, landed_after_release=after, landings=list(p.landings),
                     release_frame=p.release_frame, done=True)
    if p in self.inflight:
      self.inflight.remove(p)
    if p in self.expired:
      self.expired.remove(p)

  # ---- the driver's own holds: the car's landing after a release is the same whoever held the button ----
  def _watch_driver_hold_landing(self, direction, steps):
    dh = self.driver_hold
    if dh is None or dh["dir"] != direction or abs(steps - round(steps)) > 0.25:
      return
    n = int(round(steps))
    dh["landed"] += n
    if dh["release"] is not None and self.frame > dh["release"] + HOLD_FRAME_GAP + _frames(0.2):
      dh["after"] += n
    dh["last"] = self.frame

  def _watch_driver_hold(self, button):
    """Called every frame with the stock switch's SET-/SET+ state (None when not pressed)."""
    dh = self.driver_hold
    if button in (SET_MINUS, SET_PLUS):
      if dh is None or dh["release"] is not None:
        if dh is not None:
          self._finish_driver_hold()     # a new press settles the previous one
        self.driver_hold = {"dir": button, "down": self.frame, "release": None, "landed": 0, "after": 0, "last": self.frame}
      elif dh["dir"] != button:
        self.driver_hold = None
      return
    if dh is None:
      return
    if dh["release"] is None:
      dh["release"] = self.frame
    elif self.frame - max(dh["release"], dh["last"]) > _frames(HOLD_SETTLE):
      self._finish_driver_hold()

  def _finish_driver_hold(self):
    dh, self.driver_hold = self.driver_hold, None
    if dh is None or dh["release"] is None:
      return
    q = self._quantum_steps()
    if dh["landed"] > q:
      # a real staircase (a hold, not a tap): learned exactly like one of ours (see _finish_hold)
      quanta_after = -(-dh["after"] // q)
      self.hold_overrun = max(1, self.hold_overrun if self.hold_measured else 0, quanta_after)
      self.hold_measured = True
      self.holds_learned += 1
      self.last_hold_learned = (quanta_after, "driver")

  def _consume_ours(self, direction, n):
    """Account n steps in `direction` to our presses in flight, oldest first. A hold takes
    everything that comes its way (its reserve is a bound, not a count)."""
    while n > 0:
      p = next((pr for pr in self.inflight if pr.direction == direction and (pr.remaining > 0 or pr.hold)), None)
      if p is None:
        break
      take = n if p.hold else min(n, p.remaining)
      p.remaining = 0 if p.jump else max(p.remaining - take, 0)
      p.landed += take
      p.landings.append((self.frame, take))
      n -= take
      self._apply_ours(direction, take)
      if p.remaining == 0 and not p.hold:
        self.inflight.remove(p)
    self.pending = None

  def _observe_set_speed(self, i):
    """Attribute set speed changes to our press or to the driver."""
    if self.last_set_speed is None:
      self.last_set_speed = i.set_speed
      return
    delta = i.set_speed - self.last_set_speed
    after_gap = self.cruise_gap
    self.cruise_gap = False
    if abs(delta) < self.half_step:
      return
    self.prev_set_speed = self.last_set_speed
    self.last_set_speed = i.set_speed
    direction = SET_PLUS if delta > 0 else SET_MINUS
    steps = abs(delta) / self.step
    self._watch_driver_hold_landing(direction, steps)
    match = self._matching_press(direction)
    ours_k = match[1] if match is not None and match[0] == "ours" else None
    driver_k = match[1] if match is not None and match[0] == "driver" else None
    driver_recent = (driver_k is not None or self.prev_driver_button == direction or
                     self.frame - self.last_driver_edge[direction] <= _frames(DRIVER_RECENT))
    jp = next((p for p in self.inflight + self.expired if p.jump and p.direction == direction), None)
    # (review: after a cruise gap the driver's SET at the current speed is theirs, as is anything past their speed)
    jump_ok = (not after_gap and self.anchor is not None and i.set_speed <= self.anchor + self.half_step)
    if not jump_ok:
      jp = None
    if jp is None and jump_ok and direction == SET_PLUS and driver_k is None and not driver_recent and steps > 1.25 and \
       abs(i.set_speed / self.step - round(i.v_ego / self.step)) <= 1.0:
      # a SET+ of ours off the gas that jumped to the current speed after all (not seen on drives 4c-4f): ours, not
      # "more steps than our presses" (which switches everything off for the drive)
      jp = next((p for p in self.inflight + self.expired if p.direction == SET_PLUS and not p.hold), None)
      if jp is not None:
        jp.reserve = max(jp.reserve, int(math.ceil(steps - 0.25)))
    if jp is not None and direction == SET_PLUS and driver_k is None and steps <= jp.reserve + 0.25:
      # our SET+ jump (to the current speed), on time or late, a whole number of steps or not (from the car's 30 km/h
      # minimum): all of it is ours, up to the driver's speed
      for lst in (self.inflight, self.expired):
        if jp in lst:
          lst.remove(jp)
      jp.landed, jp.remaining = int(math.ceil(steps - 0.25)), 0
      if self.pending is jp:
        self.pending = None
      self._apply_ours(direction, jp.landed)
      return
    n_ours = sum(p.remaining for p in self.inflight if p.direction == direction)
    hold_open = any(p.hold for p in self.inflight if p.direction == direction)
    expired_k = next((k for k, p in enumerate(self.expired) if p.direction == direction), None)
    if abs(steps - round(steps)) > 0.25:
      if (ours_k is not None and hold_open and direction == SET_MINUS and
          abs(i.set_speed - CAR_MIN_SET_SPEED) < self.half_step):
        # Our SET- hold ran onto the car's own minimum (30 km/h): a short last step, still ours. The overrun was
        # larger than assumed: a phase-locked hold keeps one more quantum of margin from now on; otherwise no more
        # holds this drive. Never a disable.
        if self.phase_lock:
          self.lock_overrun[SET_MINUS] = max(self.lock_overrun[SET_MINUS], 1)
        else:
          self.hold_misses = HOLD_MISS_LIMIT
          self.holds_ok = False
        self._consume_ours(direction, int(math.ceil(steps)))
        return
      if (ours_k is not None and not hold_open and direction == SET_MINUS and steps < 1.75 and
          driver_k is None and not driver_recent and
          abs(i.set_speed - CAR_MIN_SET_SPEED) < self.half_step):
        # our SET- tap from the whole step above onto the car's 30 km/h minimum: one press, one (short) step
        self._consume_ours(direction, 1)
        return
      if (ours_k is not None and direction == SET_PLUS and self.prev_set_speed is not None and
          driver_k is None and not driver_recent and
          abs(self.prev_set_speed - CAR_MIN_SET_SPEED) < self.half_step):
        # our first SET+ from the car's 30 km/h minimum lands on the next whole step: one press, one (short) step
        self._consume_ours(direction, 1)
        return
      if ours_k is not None and not driver_recent and not after_gap and steps < 2.5:
        # Our press landed as a fraction of a step: the car's set speed steps don't match our
        # unit (e.g. km/h setting with an mph dash). We cannot tell our presses from the
        # driver's: switch off for this drive.
        self._disable()
      else:
        # Not ours and not a whole step: the driver (e.g. the first tap above the car's
        # 30 km/h minimum set speed, which is 18.6 mph on an mph dash).
        self._forget()
        self.blocked = True
      return
    n = int(round(steps))
    one_step = n == 1
    if one_step and driver_k is not None:
      # The driver tapped this button just before: their step, even if ours is in flight too.
      # No more lowering for this curve (_driver_change says what happens to the restore).
      del self.driver_taps[driver_k]
      self._driver_change(direction, 1)
    elif ours_k is not None and (n <= n_ours or hold_open):
      # Ours (one press, several presses landing between two samples, or a hold stepping several
      # at once). Any landing also confirms that the car is listening, so the next press may go out.
      if len(self.inflight) == 1 and not after_gap and not self.inflight[0].hold:
        # Unambiguous landing with cruise on throughout: learn how fast this car answers.
        self.latencies = (self.latencies + [self.frame - self.inflight[0].frame])[-9:]
      if hold_open and n > n_ours:
        if self.hold is not None and self.hold.direction == direction:
          # Still holding: the car steps for as long as we hold (a SET+ restore hold may run more quanta than the
          # reserve, which is sized for a release). Grow the reserve with what landed.
          self.hold.remaining += n - n_ours
          self.hold.reserve += n - n_ours
        else:
          # More than the hold had reserved: the car kept stepping after release. Still ours (the
          # driver's speed reference must not move), but no more holds this drive.
          self.hold_misses = HOLD_MISS_LIMIT
          self.holds_ok = False
      self._consume_ours(direction, n)
    elif one_step and expired_k is not None and not driver_recent and not after_gap:
      # A press we had given up on landed late. Counting it as ours keeps the driver's speed
      # reference right; if we already restored, the next pass corrects the extra step.
      p = self.expired[expired_k]
      p.remaining -= 1
      p.landed += 1
      if p.remaining <= 0:
        del self.expired[expired_k]
        self._finish_hold(p)
      self._apply_ours(direction)
    elif after_gap:
      # Cruise was re-engaged at a different speed: the driver chose a new speed.
      self._forget()
      self.blocked = True
    elif one_step:
      # A single unexplained step is a driver tap (or the driver's tap matched our pending
      # press first and this is our own press landing). No more lowering for this curve.
      self._driver_change(direction, 1)
    elif driver_recent:
      # The driver held the button: several steps of theirs.
      self.driver_taps = [(d, f) for d, f in self.driver_taps if d != direction]
      self._driver_change(direction, n)
    elif ours_k is not None:
      # More steps than we have presses in flight, in our direction: the car may step more than
      # once per press. We can't keep the limits reliably, so switch off for this drive.
      self._disable()
    else:
      # Several steps at once (e.g. holding the button): the driver set a new speed.
      self._forget()
      self.blocked = True

  def _start_hold(self, button):
    """The hold's bookkeeping, before (phase lock: possibly a few frames before) its first frame goes out."""
    reserve = self._hold_reserve() + self._quantum_steps()   # first quantum + what may follow a release
    p = Press(button, self.frame, reserve=reserve, remaining=reserve, hold=True)
    self.hold = p
    self.pending = p
    self.inflight.append(p)
    self.hold_log.append({"frame": self.frame, "direction": button, "frames_sent": 0, "landed": 0, "done": False,
                          "overrun_assumed": self.hold_overrun, "bound_mph": (self.hold_bound or 0) / MS_PER_MPH,
                          "phase_lock": self.phase_lock, "delays": [], "fallbacks": 0, "pattern": ""})

  def _send(self, button, hold=False):
    """Book-keeping for a frame going out now; returns the counter offset or None (skip frame)."""
    if hold and self.phase_lock:
      offset = HOLD_COUNTER_OFFSET
    else:
      offset = COUNTER_OFFSETS[self.button_frame % len(COUNTER_OFFSETS)]
      self.button_frame += 1
    if offset is None:
      return None
    self.last_send_frame = self.frame
    if hold:
      if self.hold is None:
        self._start_hold(button)
      self.hold.frames_sent += 1
      self.hold_log[-1]["delays"].append(self.frame - self.wheel_frame)
      self.hold.release_frame = self.frame
      self.hold_log[-1]["frames_sent"] = self.hold.frames_sent
    else:
      p = Press(button, self.frame)
      p.frames_sent = 1
      self.pending = p
      self.inflight.append(p)
    return offset

  def update(self, i: SlowdownInputs):
    """Returns (button, counter_offset) to send this frame, or (None, None)."""
    self.frame += 1
    self.last_action = None
    self.limit_adopted = None
    self.limit_capped = None
    self.limit_undone = None
    self.new_wheel = i.btn_counter >= 0 and i.btn_counter != self.prev_btn_counter
    if self.new_wheel:
      self.wheel_frame = self.frame
    self.prev_btn_counter = i.btn_counter
    self.phase_lock = HOLD_PHASE_LOCK and i.btn_counter >= 0
    if self.disabled:
      return None, None
    cruise_ok = i.cruise_enabled and i.set_speed > 0
    self.cruise_on_frames = self.cruise_on_frames + 1 if cruise_ok else 0
    if not cruise_ok:
      if self.last_set_speed is not None:
        self.cruise_gap = True
      if i.driver_button == SET_MINUS:
        self.set_while_off = True     # SET with cruise off engages at the current speed: the driver's new speed
      # Freeze the timers of a press in flight while cruise is off, so it is still
      # recognised as ours when cruise comes back (the set speed is not observed meanwhile).
      for p in self.inflight + self.expired:
        p.frame += 1
      self.hold = None     # a hold is not continued across a cruise cancel
    elif self.set_while_off:
      self.set_while_off = False
      if self.anchor is not None:
        self._forget()
        self.blocked = True

    # The driver's own SET+/SET- taps (rising edge). Our own frames are never read back here:
    # pandad reports transmitted frames on bus + 128 and the bus-0 parser ignores those.
    if i.driver_button in (SET_MINUS, SET_PLUS):
      if i.driver_button != self.prev_driver_button:
        self.driver_taps.append((i.driver_button, self.frame))
        self.blocked = True
        self.hold = None   # the driver is on the buttons: do not keep holding
      self.last_driver_edge[i.driver_button] = self.frame     # kept fresh while held
    self.prev_driver_button = i.driver_button
    if i.gas_pressed:
      self.last_gas_frame = self.frame
    self.driver_taps = [(d, f) for d, f in self.driver_taps if self.frame - f <= self._in_flight_window()]
    self._watch_driver_hold(i.driver_button if cruise_ok else None)

    if cruise_ok:
      self._observe_set_speed(i)
    # (every driver change of the set speed and every driver SET edge sets `blocked`, which our raise needed clear)
    if self.limit_raised is not None and (self.blocked or i.brake_pressed or not cruise_ok):
      self.limit_raised["touched"] = True

    # A press that has not shown up yet: count it; it stays in flight for a late arrival.
    if self.pending is not None and not self.pending.hold and self.frame - self.pending.frame > _frames(PENDING_TIMEOUT):
      self.pending = None
      self.no_effect += 1
    window = self._in_flight_window()
    for p in [p for p in self.inflight if not p.hold and self.frame - p.frame > window]:
      self.inflight.remove(p)
      self.expired.append(p)
    for p in [p for p in self.expired if self.frame - p.frame > _frames(PENDING_TIMEOUT + LATE_WINDOW)]:
      self.expired.remove(p)
    # A hold is resolved HOLD_SETTLE after its last frame (and no landing for that long).
    for p in [p for p in self.inflight if p.hold and p is not self.hold]:
      last = max([p.release_frame] + [f for f, _ in p.landings])
      if self.frame - last > _frames(HOLD_SETTLE):
        if p.landed == 0:
          self.no_effect += 1
        self._finish_hold(p)
        if self.pending is p:
          self.pending = None
    if self.no_effect >= NO_EFFECT_LIMIT:
      self.no_effect = 0
      self.no_effect_trips += 1
      self.backoff_until = self.frame + _frames(NO_EFFECT_BACKOFF)
      if self.no_effect_trips >= NO_EFFECT_TRIPS:
        self._forget()   # the car is not responding: leave the set speed alone

    # Curve tracking. Frozen while the planner is missing or the driver is on the gas
    # (FrogPilot's curve flag is not meaningful then).
    if i.plan_alive and not i.gas_pressed:
      if i.csc_active and not self.prev_csc_active and self.offset != 0 and \
         0 < self.curve_free_frames <= _frames(FLICKER_WINDOW):
        self.flicker = True
      self.prev_csc_active = i.csc_active
      self.curve_free_frames = 0 if i.csc_active else self.curve_free_frames + 1
      self.reason_free_frames = 0 if (i.csc_active or i.stop_active or i.hold_restore) else self.reason_free_frames + 1
    if i.plan_alive:
      self.curve_free_gas = 0 if i.csc_active else self.curve_free_gas + 1
      if i.csc_active:
        self.curve_gas_camera = not i.curve_track
    # The posted limit back down soon after our raise (speed_limit_follow.py rule 4, review: a raise for a wrong road's
    # limit, 60 on a 45 road, was never taken back): if the driver has not touched his speed since our raise and no stop
    # / curve slowdown owns the set speed, his speed goes back to what it was before and the restore below lowers the set
    # speed back to it (SET- taps, undoing our own presses). Otherwise his old speed only caps a pending restore, as
    # rule 3 does.
    if i.limit_undo > 0 and self.limit_raised is not None:
      raised, self.limit_raised = self.limit_raised, None
      # (his speed before our raise; after an exit the follower asks for the street's lower limit, above it: rule 6)
      before = max(raised["before"], i.limit_undo)
      owned = (i.csc_active or i.stop_active or i.hold_restore or self.stop_episode or self.restore_pending or
               self.lower_presses > 0 or self.stop_presses > 0)
      if not raised["touched"] and not owned:
        self.limit_undone = (self.anchor if self.anchor is not None else self.last_set_speed, before, "undo")
        self.anchor = before
        self.undo_cap = None
      elif self.anchor is not None and before < self.anchor - self.half_step:
        self.limit_undone = (self.anchor, before, "cap")
        self.undo_cap = before
    # A lower posted limit (speed_limit_follow.py rule 3; drive 52, tag 450: after a stop our restore took the set back
    # up to 50 on a new 45 road). From now on no SET+ of ours above it; once none of our presses can still land (and no
    # curve / stopped car is on: their limits count from the driver's speed) the driver's speed comes down to it, or
    # to the set speed if that is higher. Never a SET- for a lower limit. If the lower limit goes first (the cross road
    # matched while stopped at a light, drives 4c / 4d) the restore goes on to the driver's speed.
    if i.limit_cap > 0:
      if (self.anchor is not None and i.limit_cap < self.anchor - self.half_step and
          (self.limit_cap is None or abs(i.limit_cap - self.limit_cap) > self.half_step)):
        self.limit_cap = i.limit_cap
        self.limit_capped = (self.anchor, i.limit_cap)
    else:
      self.limit_cap = None
    if self._cap() is not None:
      if self.anchor is None:
        self.limit_cap = self.undo_cap = None
      elif (cruise_ok and not self.inflight and self.pending is None and self.hold is None and
            not any(p.direction == SET_PLUS for p in self.expired) and not i.stop_active and
            (not i.csc_active or self._cap_fits_curve())):
        # a pending restore stops at the cap (or the set speed, if higher); a set speed above the driver's speed (our
        # own overshoot, a SET- correcting it) keeps the driver's speed: a cap only ever limits SET+
        self.anchor = self._capped_anchor()
        self.limit_cap = self.undo_cap = None
    if self.offset == 0 and not self.inflight:
      self.flicker = False
      self.anchor = None             # back at the driver's speed: nothing left to restore
    # (provisional: only while it is below the driver's speed; it never lowers the anchor, rule 3's cap does that)
    self.limit_ceiling = (i.limit_ceiling if (i.limit_ceiling > 0 and self.anchor is not None and
                                              i.limit_ceiling < self.anchor - self.half_step) else None)
    restore_delay = FLICKER_RESTORE_DELAY if self.flicker else RESTORE_DELAY
    curve_over = self.curve_free_frames >= _frames(restore_delay)
    all_over = self.reason_free_frames >= _frames(restore_delay)
    if curve_over:
      self.lower_presses = 0
      self.curve_holds = 0
    if not i.stop_active:
      self.stop_presses = 0
    if all_over:
      self.blocked = False
      self.holds_ok = self.hold_misses < HOLD_MISS_LIMIT
    if not self.blocked:
      self.blocked_frame = None
    elif self.blocked_frame is None:
      self.blocked_frame = self.frame
    if (self.blocked and self.lower_presses == 0 and self.stop_presses == 0 and self.offset == 0 and not self.inflight and
        self.hold is None and i.driver_button not in (SET_MINUS, SET_PLUS) and
        self.frame - max(self.blocked_frame, *self.last_driver_edge.values()) >= _frames(DRIVER_REARM)):
      self.blocked = False          # the driver adjusted before we did anything: their speed is the new reference
      self.blocked_frame = None
      self.rearms += 1

    if i.stop_restore_now and self.offset != 0:
      self.restore_pending = True
    # (not while a planned stop holds the speed: review, a merge raise 45 -> 70 went up by SET+ holds with a stopped car
    # latched ahead)
    if i.fast_restore and self.offset > 0 and not i.csc_active and not i.stop_active and not i.hold_restore:
      self.restore_pending = True
    if self.offset == 0 and not self.inflight or i.stop_active:
      self.restore_pending = False
    # A restore asked for at once (a red light gone, a guarded stop ahead cleared) while a curve target is on only goes
    # up to the curve's target (below); the curve keeps governing, and when it ends its own restore (RESTORE_DELAY, the
    # flicker delay) brings the rest. Review sB/sG/sI: restore_pending ignored the curve's target and its 1 s SET-
    # settle, so the set speed went 35 <-> 37 every 0.2 s through a 35 mph curve (~50 presses), and once the curve's
    # SET- budget was spent the restore went on to 44-45 inside it.
    if self.restore_pending and i.csc_active:
      self.restore_in_curve = True
    elif self.restore_in_curve and not i.csc_active:
      self.restore_pending = False
    if not self.restore_pending:
      self.restore_in_curve = False
    speed_ok = i.v_ego >= (KEEP_SPEED if self.managing else START_SPEED) or (i.stop_active and i.v_ego >= KEEP_SPEED) or (
      i.street_lower and i.csc_active and i.curve_track and not i.stop_active and not i.hold_restore and
      i.v_ego >= KEEP_SPEED)
    self.standstill_frames = self.standstill_frames + 1 if i.standstill else 0
    # At a standstill only the restore after stopping behind the car is sent (SET+ there only raises the set
    # speed; the MRCC keeps holding behind the car ahead), once we have been stopped STANDSTILL_RESTORE_DELAY.
    # While creeping in a queue nothing is sent until we are clearly rolling again.
    stopped_restore = self.restore_pending and self.standstill_frames >= _frames(STANDSTILL_RESTORE_DELAY)
    fast_any = FAST_RESTORE_ANY_SPEED and self.offset > 0 and i.stop_pulling_away and (
      self.restore_pending or (i.stop_clear_ahead and not i.stop_active) or
      (RADAR_LEAD_FAST_RESTORE and i.radar_lead and self.stop_episode and not i.stop_active))
    restore_only = not (speed_ok or stopped_restore) and fast_any    # rolling slowly: SET+ only
    can_act = (i.enabled and cruise_ok and i.plan_alive and not i.brake_pressed and
               (not i.standstill or stopped_restore) and
               (speed_ok or stopped_restore or (self.restore_pending and i.v_ego > RESTORE_MOVING_SPEED) or fast_any) and
               self.cruise_on_frames >= _frames(CRUISE_SETTLE))
    if not can_act:
      self.hold = None
      # Time with cruise off does not count: on resume we compare the set speed instead.
      if self.offset != 0 and cruise_ok:
        self.pause_frames += 1
        if self.pause_frames >= _frames(PAUSE_ABANDON):
          self._forget()
      return None, None
    self.pause_frames = 0

    # Decide as if every press in flight still lands: the limits then hold whether it does or not.
    flying = sum(p.remaining if p.direction == SET_MINUS else -p.remaining for p in self.inflight)
    set_speed = i.set_speed - flying * self.step
    offset = self.offset + flying
    driver_speed = set_speed + offset * self.step
    targets = []
    if i.csc_active:
      if i.curve_floor > 0:
        floor = max(i.curve_floor, MIN_SET_SPEED)
      else:
        floor = max(driver_speed - self.max_drop_steps * self.step, MIN_SET_SPEED)
      curve_t = i.csc_speed + (0.0 if i.curve_track else CURVE_MARGIN)
      if CURVE_NO_ACCEL and i.curve_no_accel and not i.curve_track:
        curve_t = min(curve_t, i.v_ego + self.half_step)    # never a set speed that speeds the car up in the curve
      targets.append(("curve", max(curve_t, floor)))
    if i.stop_active:
      targets.append(("stop", max(min(i.stop_target, driver_speed), STOP_MIN_SET_SPEED)))
    reason, target = min(targets, key=lambda rt: rt[1]) if targets else (None, None)

    # A hold in progress: keep sending its frames at the stock cadence, nothing else. Stop at once
    # when its reason is gone, the driver touches a button or a pedal, it has run long enough, or
    # the next quantum plus the quanta that may still land after the release would cross its bound.
    if self.hold is not None:
      q = self._quantum_steps()
      d = self.hold.direction
      # the quantum still to come while holding plus the overrun after the release must fit the bound
      # (a restore hold keeps a quantum of margin below the driver's speed instead of the assumed overrun)
      if d == SET_PLUS and self.hold_margin:
        # a restore hold keeps its margin in the bound: release once the next landing (the next multiple of the
        # quantum above the set speed) would pass it
        next_land = (int(round(i.set_speed / self.step)) // q + 1) * q * self.step
        crosses = next_land > self.hold_bound + self.half_step
      else:
        overrun = (1 + (self._down_overrun() if d == SET_MINUS else self._up_overrun())) * q * self.step
        crosses = ((i.set_speed - overrun < self.hold_bound - self.half_step) if d == SET_MINUS else
                   (i.set_speed + overrun > self.hold_bound + self.half_step))
      # a SET- hold also stops when the target rises above its bound (the radar took over the car ahead; a firm curve's
      # follower going back up to the curve's speed)
      kind = "curve" if self.hold_kind == "curve" else "stop"
      target_rose = d == SET_MINUS and reason == kind and target > self.hold_bound + self.half_step
      # (a restore hold inside a followed map curve, RESTORE_IN_CURVE_HOLD: only the curve may be on, its target still at
      # or above the hold's bound)
      curve_ok = (self.hold_kind == "restore_curve" and reason == "curve" and i.curve_track and
                  target >= self.hold_bound - self.half_step)
      reason_gone = (reason != kind or target_rose or (kind == "curve" and not i.curve_hold)) if d == SET_MINUS else \
                    ((reason is not None and not curve_ok) or offset <= 0 or self._ceiling() is not None or
                     (i.stop_slower_ahead and self.stop_episode))
      no_effect = self.hold.landed == 0 and self.frame - self.hold.frame >= _frames(HOLD_NO_EFFECT_SECONDS)
      if (reason_gone or i.gas_pressed or i.other_button_active or no_effect or
          self.frame - self.hold.frame >= _frames(HOLD_MAX_SECONDS) or crosses):
        if target_rose and kind == "stop":
          self.holds_this_episode = max(0, self.holds_this_episode - 1)   # the floor hold stays available
        if not crosses and self.hold.frames_sent:
          # stopped between two landings (its reason went, a pedal, ...): the quantum the car was about to step can
          # still land right after our last frame (drive 4c sim: 15 ms later). That one is this hold's, not an
          # overrun (which would switch holds off for the drive); it stays within the bound (floor / driver's speed).
          self.hold.remaining += q
          self.hold.reserve += q
          if self.hold_log:
            self.hold_log[-1]["aborted"] = True
        self.hold = None
        return None, None
      if self.phase_lock:
        ref = self.last_send_frame if self.hold.frames_sent else self.hold.frame
        due = self.new_wheel                    # one frame per wheel frame, right after it
        if (not due and self.frame - ref >= HOLD_FALLBACK_FRAMES and
            self.frame - self.wheel_frame >= HOLD_FALLBACK_FRAMES):
          due = True                            # the wheel's frames stopped coming: keep the hold alive anyway
          self.hold_log[-1]["fallbacks"] += 1
      else:
        due = self.frame - self.last_send_frame >= HOLD_FRAME_GAP
      if due:
        off = self._send(d, hold=True)
        if off is not None:
          self.last_action = "hold"
          return d, off
      return None, None

    # Nothing while the driver is on a SET button or just was: their change is attributed cleanly first.
    if i.driver_button in (SET_MINUS, SET_PLUS) or \
       self.frame - max(self.last_driver_edge.values()) < _frames(DRIVER_QUIET):
      return None, None

    # One unconfirmed press at a time, a bounded number in flight, nothing while a hold settles;
    # also wait during a back-off.
    if (self.pending is not None or len(self.inflight) >= MAX_IN_FLIGHT or self.frame < self.backoff_until or
        any(p.hold for p in self.inflight)):
      return None, None

    # A higher posted limit (the driver's speed was the old one): it becomes the driver's speed, never more; the
    # restore below brings the set speed up. Not while a reason is active, a planned stop holds the speed
    # (hold_restore), the driver stands us down or is adjusting.
    if (i.limit_raise > 0 and reason is None and not self.blocked and not self.inflight and self.hold is None and
        not i.hold_restore and self.last_set_speed is not None and i.limit_raise > driver_speed + self.half_step and
        self.frame - max(self.last_driver_edge.values()) >= _frames(LIMIT_DRIVER_QUIET)):
      self.limit_adopted = (self.anchor if self.anchor is not None else i.set_speed, i.limit_raise)
      self.limit_raised = {"frame": self.frame, "before": self.limit_adopted[0], "new": i.limit_raise, "touched": False}
      self.anchor = i.limit_raise
      self.limit_cap = self.undo_cap = None
      offset = self.offset
      driver_speed = set_speed + offset * self.step

    want_lower = False
    room = 0
    if reason is not None and not self.blocked:
      # How many more steps down this wants (re-lowering while restoring needs 2 steps).
      room = int((set_speed - target + 1e-3) // self.step)
      budget_ok = (self.lower_presses < max(self.max_lower_presses, i.curve_budget)) if reason == "curve" else \
                  (self.stop_presses < MAX_STOP_PRESSES)
      want_lower = budget_ok and room >= (2 if self.restoring else 1) and not restore_only

    # on the gas the stop reasons' "over" timer is frozen (reason_free_frames): a clear stop restore still may go
    # a curve's restore on the gas (CURVE_GAS_RESTORE): the curve target off for RESTORE_DELAY, on the gas too, and
    # nothing of a stop in it. A camera curve waits FLICKER_RESTORE_DELAY (review: its target flickers off and on, and
    # the flicker is not tracked on the gas: a SET+ jump mid-curve, then SET- again); a followed map curve does not
    gas_delay = FLICKER_RESTORE_DELAY if self.curve_gas_camera else RESTORE_DELAY
    curve_gas = (CURVE_GAS_RESTORE and self.curve_free_gas >= _frames(gas_delay) and not self.stop_episode and
                 not i.hold_restore and self.stop_presses == 0)
    gas_restore = (GAS_JUMP_ENABLED and i.gas_pressed and not i.csc_active and not i.stop_active and
                   (self.restore_pending or i.stop_clear_ahead or curve_gas))
    button = None
    raise_in_curve = False
    dwell_ok = self.frame - self.last_minus_frame >= _frames(MIN_DWELL)
    # While a curve target is on, a SET+ (a raise, or any restore) only goes to the highest step at or below the
    # lowest target, never above it (review sE: rounding up gave 37 for a 36.3 mph curve), whatever the SET- budget
    # left; and it waits out the curve's 1 s SET- settle like the curve's own restore does.
    up_ok = not i.csc_active or (target is not None and set_speed + self.step <= target + 1e-3)
    # the highest a SET+ of ours may take the set speed: the driver's speed, a lower limit's cap still pending, or a lower
    # limit reported but not steady yet (speed_limit_follow.py, drive 53 tag 1324.5)
    ceiling = self._ceiling()
    top = driver_speed if ceiling is None else min(driver_speed, ceiling)
    if ceiling is not None:
      up_ok = up_ok and set_speed + self.step <= top + 1e-3
    # no SET+ of ours while a slower car is in our lane after a red light gone release (the queue the light was for): the
    # stop slowdown takes it on 0.5 s later, and a SET+ still in flight then holds its first SET- back (drive 52 closed
    # loop: a tap 25 -> 26 at 188.36, at our speed, put the first SET- 0.5 s later)
    if i.stop_slower_ahead and self.stop_episode:
      up_ok = False
    if want_lower:
      self.restoring = False
      button = SET_MINUS
    elif (i.curve_track and i.curve_raise and reason == "curve" and not self.blocked and offset > 0 and up_ok and
          not restore_only and dwell_ok and not i.stop_active and not i.hold_restore):
      # A followed map curve's command came back up (the car is nearly at the curve's speed, see
      # curve_target.MapCurveFollower): raise the set speed to it now, with taps, while the curve is still on. Up
      # only to the lowest target (a stopped car ahead that wants less is the reason then, and this does not run);
      # `restoring` makes a re-lowering wait for a whole extra step, as after any restore (no ping-pong). Never during
      # a stop episode or while a planned stop holds the speed: the stop reasons that publish target 100 ("hold, do
      # not lower further": an unconfirmed early start's lost car, an unsure car not seen, the radar handling the car)
      # leave the curve as the lowest target, and review sH raised 31 -> 34 while closing on a slower car.
      self.restoring = True
      button = SET_PLUS
      raise_in_curve = True
    elif offset != 0 and (all_over or self.restore_pending or gas_restore) and (offset < 0 or up_ok) and \
         (dwell_ok or ((self.restore_pending or gas_restore) and not i.csc_active)):
      self.restoring = True
      button = SET_PLUS if offset > 0 else SET_MINUS
    elif offset == 0:
      self.restoring = False
      self.stop_episode = False
      self.holds_this_episode = 0

    # a SET+ while the car is faster than the set speed jumps to the current speed (see GAS_JUMP_*)
    v_hi = int(round(i.v_ego / self.step)) + JUMP_UNCERT
    faster = button == SET_PLUS and v_hi >= int(round(i.set_speed / self.step)) + JUMP_STEPS
    may_jump = faster and (i.gas_pressed or self.frame - self.last_gas_frame <= _frames(GAS_JUMP_RECENT))
    jump = False
    if faster and v_hi + (GAS_JUMP_MARGIN if may_jump else 0) > int(round(top / self.step)):
      return None, None                # if it did jump it could land above the driver's speed
    if may_jump and i.csc_active:
      # a jump lands on the current speed, which in a curve may be above its target (review sF: a raise 0.1 s after
      # the gas went up 31 -> 37 in a 35 mph curve): wait out the jump window, then taps
      return None, None
    if may_jump:
      if self.inflight:
        return None, None              # (or on top of a press in flight)
      jump = True
    gas_ok = GAS_JUMP_ENABLED and button == SET_PLUS and gas_restore and offset > 0
    if button is None or (i.gas_pressed and not gas_ok) or i.other_button_active:
      return None, None
    if any(p.direction != button for p in self.inflight):
      # Never reverse direction while a press may still land.
      return None, None
    urgent = want_lower and reason == "stop" and i.stop_urgent
    # Relaxed pacing for a stopped car that is not urgent, and for the gentle restore after a lost /
    # left-lane release (the car may still be ahead: a re-trigger should win over the taps).
    radar_restore = RADAR_LEAD_FAST_RESTORE and i.radar_lead and not want_lower and self.stop_episode
    fast_stop = self.restore_pending or radar_restore or i.stop_clear_ahead
    gentle = (want_lower and reason == "stop" and not urgent) or \
             (not want_lower and self.stop_episode and not fast_stop)
    gap = RELAXED_GAP_FRAMES if gentle else BUTTON_GAP_FRAMES
    if self.frame - self.last_send_frame <= gap:
      return None, None

    # Long presses. Lowering for a confirmed stopped car: the first hold aims HOLD_GOALS_MPH[0] down,
    # the second HOLD_GOALS_MPH[1] more, bounded by max(target, 20 mph floor); then taps. Restoring
    # after a stopped car has clearly gone: SET+ holds bounded by the driver's speed, then taps. A hold only starts
    # when its first quantum plus what may still land after the release stays inside the bound.
    q = self._quantum_steps()
    worst = (q + self._hold_reserve()) * self.step      # tap + first quantum + pending quantum + measured overrun
    hold = False
    self.hold_margin = False
    flight_ok = not self.inflight or self._only_lost_in_flight(button)
    # a restore asked for at once (a stop ahead that cleared, a red light gone) inside a followed map curve: a SET+ hold
    # up to the curve's target, never above it (RESTORE_IN_CURVE_HOLD; drive 54 t 764.7-768: 19 -> 26 by taps in a 42
    # mph curve while the car slowed 35 -> 30)
    in_curve = (RESTORE_IN_CURVE_HOLD and i.csc_active and i.curve_track and reason == "curve" and target is not None and
                self.restore_pending and not raise_in_curve)
    # a confirmed floor stop that would hold now but for a fresh press in flight waits for it (STOP_HOLD_WAIT)
    if (want_lower and reason == "stop" and i.stop_urgent and self.holds_allowed and not flight_ok and not jump and
        not i.gas_pressed and self.holds_this_episode < MAX_STOP_HOLDS and
        set_speed - worst >= max(target, STOP_MIN_SET_SPEED) - self.half_step):
      if self.hold_wait_frame is None:
        self.hold_wait_frame = self.frame
      if self.frame - self.hold_wait_frame < _frames(STOP_HOLD_WAIT):
        return None, None
    else:
      self.hold_wait_frame = None
    if self.holds_allowed and flight_ok and not jump and not i.gas_pressed:
      # stop holds only when urgent (confirmed and close: floor mode); an unconfirmed early start or a far stopped car
      # taps (this used to follow from the wide overrun margin, now it is explicit)
      if want_lower and reason == "stop" and i.stop_urgent and self.holds_this_episode < MAX_STOP_HOLDS:
        hard = max(target, STOP_MIN_SET_SPEED)
        hold = set_speed - worst >= hard - self.half_step
        if hold:
          if self.hold_measured or self.phase_lock:
            self.hold_bound = hard       # straight to the floor / target; the release keeps the overrun margin
          else:
            k = min(self.holds_this_episode, len(HOLD_GOALS_MPH) - 1)
            goal = HOLD_GOALS_MPH[k] * (1.609344 if self.step < MS_PER_MPH else 1.0) * self.step
            self.hold_bound = max(hard, set_speed - goal - self.step)   # the goal (plus the button-down tap) shapes the release
          self.holds_this_episode += 1
          self.hold_kind = "stop"
      elif (CURVE_HOLD_ENABLED and want_lower and reason == "curve" and i.curve_track and i.curve_hold and
            self.curve_holds < CURVE_HOLDS and set_speed - target >= CURVE_HOLD_MIN_QUANTA * q * self.step - 1e-3):
        # a firm map curve: one hold toward its command; the release keeps the overrun margin above it, as a stop hold's
        hard = max(target, MIN_SET_SPEED)
        hold = set_speed - worst >= hard - self.half_step
        if hold:
          self.hold_bound = hard
          self.curve_holds += 1
          self.hold_kind = "curve"
      elif (button == SET_PLUS and not want_lower and (all_over or self.restore_pending) and
            (not i.csc_active or in_curve) and
            (not self.stop_episode or fast_stop or i.fast_restore) and self._ceiling() is None and
            not (i.stop_slower_ahead and self.stop_episode)):
        # one rule for every restore hold (RESTORE_HOLD_MIN_GAIN): a stop episode only when its car has clearly
        # gone (restore_now) or the radar follows a car; a lost / left-lane release taps so a re-trigger wins (and
        # taps while a lower limit's cap or ceiling is pending, or a slower car is in our lane after a red light gone: they
        # release a hold)
        # (inside a followed map curve: bounded by the curve's target as well, never above it: RESTORE_IN_CURVE_HOLD)
        driver_steps = int(round(top / self.step))
        if in_curve:
          driver_steps = min(driver_steps, int(math.floor(target / self.step + 1e-3)))
        set_steps = int(round(set_speed / self.step))
        margin = self._up_overrun() * q                      # what may still land after the release
        bound_steps = ((driver_steps - margin) // q) * q     # a multiple of the quantum, that far below
        first = ((set_steps + 1 + q) // q) * q               # where the hold's first jump lands (after its tap)
        hold = first <= bound_steps and bound_steps - set_steps >= RESTORE_HOLD_MIN_GAIN * q
        # the car faster than the set speed: if the hold's first frame did jump to the current speed (not seen off
        # the gas), it must still land at or below the hold's bound (review: coasting at the driver's speed, a jump
        # to 45 and the quantum after the release: 50)
        hold = hold and (not faster or v_hi <= bound_steps)
        if hold:
          self.hold_bound = bound_steps * self.step
          self.hold_margin = True
          self.hold_kind = "restore_curve" if in_curve else "restore"
    if want_lower and reason == "stop":
      self.stop_episode = True
    if button == SET_MINUS and self.anchor is None:
      self.anchor = i.set_speed      # the driver's speed, before our first press moves it
    if hold:
      self.hold_wait_frame = None
    if hold and self.phase_lock and not self.new_wheel:
      self._start_hold(button)        # its first frame goes out with the next wheel frame
      off = None
    else:
      off = self._send(button, hold=hold)
      if off is None:
        return None, None
      if jump and self.pending is not None and not self.pending.hold:
        p = self.pending                # lands as one jump up to the current speed, never past the driver's speed
        p.jump = True
        p.reserve = p.remaining = max(1, int(round(top / self.step)) - int(round(i.set_speed / self.step)))
    if button == SET_MINUS and want_lower:
      if reason == "curve":
        self.lower_presses += 1
      else:
        self.stop_presses += 1
      self.last_minus_frame = self.frame
    if off is None:
      return None, None
    self.last_action = "hold" if hold else ("tap" if want_lower else ("jump" if jump else ("raise" if raise_in_curve else "restore")))
    return button, off


class CurveSlowdown:
  """Card-side wrapper: reads starpilotPlan and radarState, runs the stopped car trigger and the
  shared set speed controller, builds the CAN messages and writes the per-event logs."""
  # csc_active / csc_mph: our own curve target (curve_target.py); fp_csc_mph: FrogPilot's, for comparison
  EVENT_COLUMNS = ["t_wall", "vEgo_mph", "aEgo", "brake", "gas", "cruise", "op", "set_mph", "csc_active", "csc_mph",
                   "road_curv", "fp_csc_active", "fp_csc_mph",
                   "lead_status", "lead_prob", "lead_d", "lead_v", "lead_y", "radar_has_lead", "radar_dist_bucket",
                   "stop_active", "stop_target_mph", "stop_urgent", "stop_required", "stop_ttc", "stop_floor", "offset", "in_flight",
                   "hold", "press", "driver_btn", "blocked", "restoring",
                   "map_mode", "map_status", "map_dist", "map_r", "map_active", "map_mph", "map_why", "on_ramp",
                   "ramp_type", "ramp_to_mph", "ramp_to_hwy", "ramp_end", "nav_exit", "nav_dist", "nav_r", "blinker", "exit_state", "map_adv_mph",
                   "map_veto", "gps_trust", "gps_acc", "decel_learn", "answer_frames",
                   "anchor_mph", "plan_stop", "plan_ratio", "stop_latched", "stop_early",
                   "lead_v_f", "radar_stable", "radar_follow", "radar_hold",
                   "lead_y_path", "leaving", "clear_ahead", "warn_cond", "warn_s", "warn_start", "stop_kind",
                   "slc_mph", "slc_src", "limit_stable_mph", "limit_offer_mph", "limit_raise_mph",
                   # navigation exits (map_curve_control.ExitPlan, LaneView; the planner's shape check): see EXIT_LOG_*
                   "exit_rem", "exit_shape", "shape_link_m", "shape_hwy_m", "shape_link_deg", "shape_hwy_deg", "exit_fork",
                   "exit_curve_r", "exit_curve_d", "lane_probs", "edge_gap_l", "edge_gap_r", "lane_edge", "lane_change",
                   "model_rel", "model_verdict", "bend_hwy", "bend_link"]
  # The exit log (owner's request): one "mapExit" event per navigation exit, from EXIT_LOG_BEFORE before its exit point
  # to EXIT_LOG_AFTER past it (or the plan's end), with the rows above, so a drive can show what the camera's lane check,
  # the shape check and the confirmation did. No GPS position in it.
  EXIT_LOG_BEFORE = 600.0         # m
  EXIT_LOG_AFTER = 300.0          # m
  EXIT_CHIME_HOLD = 1.0           # s the brake alert flag stays up (the planner reads it at 20 Hz)

  def __init__(self, CP, params, curve_enabled=True, stop_mode=0, learning=False, map_mode=0, limit_mode=0, follow_mode=0):
    import cereal.messaging as messaging
    from opendbc.car.mazda.values import MazdaSafetyFlags
    from opendbc.car.mazda.curve_target import CurveTarget, lateral_limit
    try:
      raw = params.get("MazdaCurveSlowdownMaxDrop", encoding="utf8")
      choice = 1 if raw is None or raw == "" else int(raw)   # typed params return 0 for "5 mph"
    except (TypeError, ValueError):
      choice = 1
    max_drop = MAX_DROP_CHOICES_MPH[choice] if 0 <= choice < len(MAX_DROP_CHOICES_MPH) else DEFAULT_MAX_DROP_MPH
    self.logic = CurveSlowdownLogic(params.get_bool("IsMetric"), max_drop)
    self.limit_mode = limit_mode
    self.limit_follow = None
    # Street Speed (MazdaStreetSpeedOffset: legal limit, +3, +5, +8 mph; read at the start of a drive as the others)
    from opendbc.car.mazda.speed_limit_follow import STREET_OFFSET_DEFAULT
    try:
      raw = params.get("MazdaStreetSpeedOffset", encoding="utf8")
      street_mode = STREET_OFFSET_DEFAULT if raw is None or raw == "" else int(raw)
    except (TypeError, ValueError, AttributeError):
      street_mode = STREET_OFFSET_DEFAULT  # (an older params build without the key: the default, +5)
    self.street_mode = street_mode
    # Slow for Lower Limits (MazdaSlowLowerLimit, default on: speed_limit_follow rule 9)
    try:
      raw = params.get("MazdaSlowLowerLimit", encoding="utf8")
      slow_lower = True if raw is None or raw == "" else (raw if isinstance(raw, bool) else bool(int(raw)))
    except (TypeError, ValueError, AttributeError):
      slow_lower = True
    self.slow_lower = slow_lower
    if limit_mode:
      from opendbc.car.mazda.speed_limit_follow import SpeedLimitFollower
      self.limit_follow = SpeedLimitFollower(params.get_bool("IsMetric"), street_mode=street_mode, slow_lower=slow_lower)
    self.limit_want = 0.0
    self.limit_cap_logged = None     # the follower's cap (rule 3) last logged
    self.limit_ceiling_logged = None # the follower's provisional ceiling last logged
    self.curve_enabled = curve_enabled
    self.stop_mode = stop_mode
    self.has_ti = bool(CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR)
    self.lat_limit = lateral_limit(self.has_ti, float(getattr(CP, "maxLateralAccel", 0.0) or 0.0))
    self.curve_target = CurveTarget(self.lat_limit)
    # The car profile: what earlier drives learned about this car (answer time, hold overrun, cruise decel), see
    # car_profile.py. Loaded here (a few KB), applied to the logic, saved in the background as it learns. The learned
    # cruise decel is still measured and logged, but map curves no longer plan with it: it measures the pace of the
    # small-step staircase (0.15-0.40 m/s^2), and the follower (curve_target.MapCurveFollower) commands the set speed
    # below the plan so the cruise slows at MAP_DECEL instead; planning with the learned pace would only start the
    # slowdown earlier again (the planner uses the same constant, see map_curves.PLAN_DECEL).
    from opendbc.car.mazda.curve_target import MAP_DECEL
    self.profile = None
    self.decel_learner = None
    self.decel_used = MAP_DECEL
    self.profile_holds_seen = 0
    self.prev_cruise = False
    try:
      self.profile = self._load_profile(CP, params)
    except Exception:
      self.profile = None
    self.map_mode = map_mode
    self.map_control = None
    self.map_out = None
    self.map_event = None
    self.exit_event = None
    self.exit_brake_frames = 0       # "Exit curve - brake": frames the flag stays up (starpilotCarState.mazdaExitCurveBrake)
    self.lane_frame = 0              # frame the camera's lanes were last read
    if map_mode:
      from opendbc.car.mazda.map_curve_control import MapCurveControl
      try:
        raw = params.get("MazdaExitSplitSpeed", encoding="utf8")
        split_mode = 0 if raw is None or raw == "" else int(raw)
      except (TypeError, ValueError, AttributeError):
        split_mode = 0               # (an older params build without the key: the default, the highway's limit)
      self.map_control = MapCurveControl(self.lat_limit, self.decel_used, split_mode=split_mode,
                                         metric=bool(params.get_bool("IsMetric")), street_mode=street_mode)
    from opendbc.car.mazda.map_curve_control import CurveMix
    self.curve_mix = CurveMix(CURVE_MARGIN)
    self.learner = None
    services = ['starpilotPlan', 'radarState', 'modelV2']
    if follow_mode:
      services.append('drivingModelData')   # the model follower's fallback for the model's accel (model_follow.py)
    if limit_mode:
      # the next posted limit's distance (speed_limit_follow.py rule 5, the merge onto a highway: starpilotPlan only has
      # the value, slcNextSpeedLimit); two floats at 20 Hz
      services.append('mapdOut')
    if learning:
      try:
        from opendbc.car.mazda.curve_learner import CurveLearner
        self.learner = CurveLearner(self.lat_limit, self._load_learning_state())
        services.append('livePose')
      except Exception:
        self.learner = None
    self.sm = messaging.SubMaster(services)
    from opendbc.car.mazda.stopped_car_slowdown import StoppedCarTrigger
    self.trigger = StoppedCarTrigger()
    self.frame = 0
    self.driver_other_frame = -10_000  # the last frame his CANCEL / RES was down (update)
    self.recorder = None
    self.curve_event = None
    self.stop_event = None
    self.stop_presses_at_start = 0
    self.near_miss = None
    try:
      from openpilot.starpilot.common.feature_events import EventRecorder, NearMissTracker
      self.recorder = EventRecorder(self.EVENT_COLUMNS, hz=20, params=params)
      self.near_miss = NearMissTracker(self.recorder, "stoppedCarSlowdown")
    except Exception:
      self.recorder = None
    self.follow = None
    if follow_mode:
      try:
        self._init_model_follow(params)
      except Exception:
        self.follow = None

  def _alive(self, service):
    return bool(self.sm.alive[service] and self.sm.recv_frame[service] > 0)

  def _road_name(self):
    """mapdOut's road name ('' unknown): speed_limit_follow rule 7, a turn onto another road."""
    try:
      return str(self.sm['mapdOut'].roadName or "") if self._alive('mapdOut') else ""
    except (KeyError, AttributeError):
      return ""

  def _next_limit(self, plan, plan_alive):
    """The next posted limit ahead (m/s, 0 none) and the m to it (-1 unknown): mapdOut's, else starpilotPlan's value
    without a distance."""
    try:
      if self._alive('mapdOut'):
        m = self.sm['mapdOut']
        limit, dist = float(m.nextSpeedLimit or 0.0), float(m.nextSpeedLimitDistance or 0.0)
        if limit > 0:
          return limit, (dist if dist > 0 else -1.0)
    except (KeyError, AttributeError):
      pass                           # (not subscribed: tests, an older build)
    return (float(getattr(plan, "slcNextSpeedLimit", 0.0) or 0.0) if plan_alive else 0.0), -1.0

  # ---- car profile (never raises into the car controller: called inside try/except) ----
  def _load_profile(self, CP, params):
    from opendbc.car.mazda.car_profile import CarProfile, DecelLearner
    profile = CarProfile(fingerprint=str(CP.carFingerprint), ti=self.has_ti)
    try:
      if params.get_bool("MazdaResetCarProfile"):
        profile.delete()
        params.put_bool("MazdaResetCarProfile", False)
    except Exception:
      pass
    lg = self.logic
    lg.latencies = profile.latencies                     # the timeout logic is tuned from the first press
    lg.hold_overrun, lg.hold_measured = profile.hold_start(HOLD_OVERRUN_DEFAULT)
    profile.note_drive()
    self.decel_learner = DecelLearner(DT_CTRL)
    return profile

  def _learn_profile(self, i, si, lead_status, lead_d, t):
    """Feed the profile what this frame taught: new answer times, a hold's landing after its release (ours or the
    driver's), a clean cruise slowdown's decel; save in the background now and then."""
    p, lg = self.profile, self.logic
    if lg.latencies != p.data["latencies"]:
      p.set_latencies(lg.latencies)
    while self.profile_holds_seen < lg.holds_learned:
      self.profile_holds_seen += 1
      after, who = lg.last_hold_learned or (lg.hold_overrun, "ours")
      p.add_hold(after, driver=(who == "driver"))
      if self.recorder is not None:
        self.recorder.record("carProfileHold", t, {"after_release_quanta": after, "by": who, "hold_overrun_now": lg.hold_overrun,
                                                   "profile_holds": p.holds, "profile_worst": p.data["hold"]["worst_after_release"],
                                                   "profile": p.summary()}, signals=False)
    from opendbc.car.mazda.car_profile import DecelInputs
    d = self.decel_learner.update(DecelInputs(v_ego=i.v_ego, set_speed=i.set_speed, cruise_enabled=i.cruise_enabled,
                                              brake_pressed=i.brake_pressed, gas_pressed=i.gas_pressed, lead_status=lead_status,
                                              lead_d=lead_d, csc_active=i.csc_active))
    if d is not None:
      use = p.add_decel_episode(d)
      if self.recorder is not None:
        self.recorder.record("carProfileDecel", t, {"decel_measured": d, "decel_episodes": p.decel_episodes, "decel_use_next_drive": use,
                                                    "decel_used_this_drive": self.decel_used, "profile": p.summary()}, min_gap=30.0,
                             signals=False)
    cruise_off = self.prev_cruise and not i.cruise_enabled
    self.prev_cruise = i.cruise_enabled
    p.save_if_due(t, force=cruise_off)

  @staticmethod
  def _hold_text(h):
    landings = [(round((f - h['frame']) * DT_CTRL, 2), n) for f, n in h.get('landings', [])]
    return (f"frames_sent {h.get('frames_sent')} landed {h.get('landed')} after_release {h.get('landed_after_release', 0)} " +
            f"landings {landings} pattern {h.get('pattern', '')} phase_lock {h.get('phase_lock')} " +
            f"delays {h.get('delays', [])[:12]} fallbacks {h.get('fallbacks', 0)}")

  def _profile_summary(self):
    """The learned values in use, for every event's summary (so a drive can be audited)."""
    lg = self.logic
    out = {"answer_frames": lg.latency, "hold_overrun": lg.hold_overrun, "hold_measured": lg.hold_measured,
           "decel_used": self.decel_used}
    if self.profile is not None:
      out["profile"] = self.profile.summary()
      out["profile_loaded"] = self.profile.loaded
      if self.profile.discarded:
        out["profile_discarded"] = self.profile.discarded
    return out

  def update(self, CC, CS, packer, CP, other_button_active):
    from opendbc.car.mazda import mazdacan
    from opendbc.car.mazda.values import Buttons
    from opendbc.car.mazda.stopped_car_slowdown import StoppedCarInputs, TEST, ON

    self.frame += 1
    # his CANCEL / RES on the wheel (outside review of e1be62454: never read; during one of our holds, up to 6 s, our
    # frame right after the wheel's could hide his CANCEL from the PCM): our holds and presses stop at once and no frame
    # of ours goes out while he holds it, nor DRIVER_QUIET after (as for his SET buttons); openpilot's own CANCEL /
    # RESUME (carcontroller) are untouched
    if getattr(CS, "crz_cancel", False) or getattr(CS, "crz_resume", False):
      self.driver_other_frame = self.frame
    if self.frame - self.driver_other_frame < _frames(DRIVER_QUIET):
      other_button_active = True
    self.sm.update(0)
    plan = self.sm['starpilotPlan']
    lead = self.sm['radarState'].leadOne
    cruise_enabled = bool(CS.out.cruiseState.enabled)
    v_ego = float(CS.out.vEgo)

    # Stopped car ahead (Test: trigger and target are computed and logged, nothing is pressed)
    from opendbc.car.mazda.stopped_car_slowdown import plan_stop
    p_stop, p_ratio, p_d = False, 1.0, 0.0
    path_x, path_y, model_length, lane_change = [], [], 0.0, False
    vel_t, vel_x = [], []                          # the model's velocity plan (also the model follower's)
    if self._alive('modelV2'):
      try:
        m = self.sm['modelV2']
        path_x, path_y = list(m.position.x), list(m.position.y)
        model_length = float(path_x[-1]) if path_x else 0.0
        vel_t, vel_x = list(m.velocity.t), list(m.velocity.x)
        p_stop, p_ratio, p_d = plan_stop(vel_t, vel_x, path_x, v_ego)
      except Exception:
        p_stop, p_ratio, p_d = False, 1.0, 0.0     # a malformed plan only loses the red-light latch
        vel_t, vel_x = [], []
      try:
        lane_change = str(self.sm['modelV2'].meta.laneChangeState) in ("laneChangeStarting", "laneChangeFinishing")
      except Exception:
        lane_change = False
    from opendbc.car.mazda.stopped_car_slowdown import lead_path_offset
    lead_y_path = (lead_path_offset(float(lead.dRel), float(lead.yRel), path_x, path_y)
                   if (lead.status and path_x) else float(lead.yRel))
    stop_inputs = StoppedCarInputs(
      enabled=bool(CC.enabled), cruise_enabled=cruise_enabled, brake_pressed=bool(CS.out.brakePressed), v_ego=v_ego,
      lead_alive=self._alive('radarState'), lead_status=bool(lead.status), lead_prob=float(lead.modelProb),
      lead_d=float(lead.dRel), lead_v=float(lead.vLead), lead_y=float(lead.yRel), lead_a=float(getattr(lead, "aLeadK", 0.0)),
      radar_has_lead=bool(getattr(CS, "radar_has_lead", False)), radar_lead_distance=int(getattr(CS, "radar_lead_distance", 0)),
      model_alive=self._alive('modelV2'), model_accel=float(self.sm['modelV2'].action.desiredAcceleration),
      plan_stop=p_stop, plan_ratio=p_ratio, plan_stop_d=p_d, a_ego=float(CS.out.aEgo), lead_y_path=lead_y_path,
      model_length=model_length, lane_change=lane_change, gas_pressed=bool(CS.out.gasPressed),
      set_speed=float(CS.out.cruiseState.speed) if cruise_enabled else 0.0,
      # the stop-light detector (the red light head start / the false stop guard); False while the plan is not alive
      red_light=bool(getattr(plan, "redLight", False)) if self._alive('starpilotPlan') else False)
    stop_active = self.trigger.update(stop_inputs) if self.stop_mode in (TEST, ON) else False

    # Our own curve target from the road curvature (see curve_target.py), against the driver's own speed
    lg = self.logic
    plan_alive = self._alive('starpilotPlan')
    blinker = bool(CS.out.leftBlinker or CS.out.rightBlinker)
    self._blinker = blinker
    set_speed = float(CS.out.cruiseState.speed)
    driver_speed = set_speed + lg.offset * lg.step if (cruise_enabled and set_speed > 0) else 0.0
    # FrogPilot's road curvature, computed from the model here (StarPilot's own roadCurvature is a different quantity)
    if not self._alive('modelV2'):
      self._road_curv = 0.0
    elif self.sm.updated['modelV2'] or not hasattr(self, "_road_curv"):
      from opendbc.car.mazda.curve_target import model_road_curvature
      m = self.sm['modelV2']
      self._road_curv = model_road_curvature(list(m.velocity.x), list(m.orientationRate.z), list(m.position.x),
                                             list(m.position.y), v_ego, self.lat_limit)
    plan_alive = plan_alive and self._alive('modelV2')
    curve_active, curve_speed = self.curve_target.update(self._road_curv, driver_speed, plan_alive, blinker)
    curve_active = curve_active and self.curve_enabled

    # The camera's view of our lane for navigation exits (map_curve_control.LaneView), read when the model updates
    if self.map_control is not None:
      try:
        self._lanes()
      except Exception:
        self.map_control.lane.update(DT_CTRL, False)
    # Map curves (Test: computed and logged only; On: combined with the camera curve target)
    curve_floor, curve_budget, fast_restore, curve_track, curve_raise = 0.0, 0, False, False, False
    curve_hold = False
    curve_street = False                   # the street speed governs (MapOutput.street): presses from KEEP_SPEED
    curve_no_accel = False                 # only on a link, which only the map knows (CurveMix.no_accel, CURVE_NO_ACCEL)
    self.map_out = None
    if self.map_control is not None and plan_alive:
      from opendbc.car.mazda.map_curve_control import MapFields, ON as MAP_ON
      signal = 1 if CS.out.rightBlinker else (-1 if CS.out.leftBlinker else 0)
      mo = self.map_out = self.map_control.update(MapFields.from_plan(plan), driver_speed, signal, v_ego=v_ego,
                                                  a_ego=float(CS.out.aEgo),
                                                  max_drop=lg.max_drop_steps * lg.step, gas=bool(CS.out.gasPressed),
                                                  set_speed=set_speed if (cruise_enabled and set_speed > 0) else None,
                                                  driver_set=bool(CS.crz_set_minus or CS.crz_set_plus),
                                                  limit_lower=self._limit_lower(plan),
                                                  limit_lower_gas=bool(self.limit_follow is not None and
                                                                       self.limit_follow.lower is not None and
                                                                       self.limit_follow.lower.kind == "lower"))
      if self.map_control.lower_event is not None:
        # the street speed (speed_limit_follow rules 8 / 9) carried out or ended by him: the follower knows
        if self.limit_follow is not None:
          self.limit_follow.lower_finished(self.map_control.lower_event,
                                           (set_speed / self.limit_follow.step) if set_speed > 0 else None)
        self.map_control.lower_event = None
      if mo.adopt and lg.offset > 0 and self.map_mode == MAP_ON:
        lg._forget()        # off an exit onto a street: the lowered set speed is now the driver's speed
      if self.map_mode == MAP_ON and plan.mapCurveMode == MAP_ON:
        # a followed road curve: the camera's target counts at the lowest it has been in it, so the follower's step
        # back up never follows the camera's target up (CurveMix)
        curve_active, curve_speed, curve_floor, curve_budget, curve_track, curve_raise = self.curve_mix.update(
          curve_active, curve_speed, mo, drop_floor=max(driver_speed - lg.max_drop_steps * lg.step, MIN_SET_SPEED))
        curve_no_accel = self.curve_mix.no_accel
        curve_hold = self.curve_mix.firm
        curve_street = bool(mo.street and curve_track and abs(curve_speed - mo.speed) < 1e-3)
        fast_restore = mo.fast_restore
      if mo.chime:
        # "Exit curve - brake" (On only: Test logs it); the planner raises the alert from starpilotCarState
        self.exit_brake_frames = int(self.EXIT_CHIME_HOLD / DT_CTRL) if self.map_mode == MAP_ON else 0
        if self.recorder is not None:
          try:
            import time
            self.recorder.record("mapExitBrake", time.monotonic(), {
              "mode": "on" if self.map_mode == MAP_ON else "test", "why": self.map_control.chime_why,
              "vEgo_mph": v_ego / MS_PER_MPH, "exit_m": self.map_control.exit_rem or 0.0}, min_gap=0.0, signals=False)
          except Exception:
            pass
    elif self.map_control is not None:
      # a planner gap: a carried exit is still counted on by the car's own speed (review: counted only while the
      # planner was alive, a gap stretched the slowdown past the exit point)
      self.map_control.drive(max(v_ego, 0.0) * DT_CTRL)

    # Posted limit (speed_limit_follow.py; Test: logged only). The driver's speed is the anchor itself, else the set
    # speed: set_speed + offset above mixes this frame's set speed with last frame's offset, a one-step blip on every
    # frame one of our presses lands, which the follower would take for the driver changing his speed.
    from opendbc.car.mazda.speed_limit_follow import ON as LIMIT_ON
    self.limit_want = 0.0
    if self.limit_follow is not None:
      from opendbc.car.mazda.speed_limit_follow import LimitInputs
      limit_driver = (lg.anchor if lg.anchor is not None else set_speed) if (cruise_enabled and set_speed > 0) else 0.0
      next_limit, next_dist = self._next_limit(plan, plan_alive)
      li = LimitInputs(limit=float(getattr(plan, "slcSpeedLimit", 0.0) or 0.0) if plan_alive else 0.0,
                       source=str(getattr(plan, "slcSpeedLimitSource", "") or ""),
                       gps_trust=int(getattr(plan, "mapGpsTrust", 0) or 0), driver_speed=limit_driver,
                       cruise_enabled=cruise_enabled,
                       road_class=int(getattr(plan, "mapRoadClass", 0) or 0) if plan_alive else 0,
                       ramp_to_highway=bool(plan_alive and getattr(plan, "mapOnRamp", False) and
                                            getattr(plan, "mapRampToHighway", False)),
                       next_limit=next_limit, next_distance=next_dist, v_ego=v_ego,
                       driver_set=bool(CS.crz_set_minus or CS.crz_set_plus), road_name=self._road_name(),
                       nav_limit=float(getattr(plan, "mapNavSpeedLimit", 0.0) or 0.0) if plan_alive else 0.0,
                       nav_dist=float(getattr(plan, "mapNavManeuverDistance", 0.0) or 0.0) if plan_alive else 0.0,
                       nav_turn=bool(plan_alive and getattr(plan, "mapNavManeuverTurn", False)),
                       turned=self._heading_turned(CS, CP, v_ego),
                       pending=bool(cruise_enabled and lg.anchor is not None and set_speed < lg.anchor - lg.half_step))
      # rules 8 / 9 only act with Speed Limit Follow and Map Curves both On (second review: Map Curves in Test left the
      # slowdown never carried out, never finished, its alert on, rules 8 / 9 blocked)
      self.limit_follow.street_active = self._street_active(plan)
      self.limit_want = self.limit_follow.update(li)
      if not self.limit_follow.street_active and self.limit_follow.lower is not None:
        self.limit_follow.lower_finished("test: logged only")      # (Test: nothing carried out)
    limit_cap = self.limit_follow.cap_speed if self.limit_follow is not None else 0.0
    limit_undo = self.limit_follow.undo_speed if self.limit_follow is not None else 0.0
    limit_ceiling = self.limit_follow.ceiling_speed if self.limit_follow is not None else 0.0

    inputs = SlowdownInputs(
      enabled=bool(CC.enabled),
      cruise_enabled=cruise_enabled,
      set_speed=float(CS.out.cruiseState.speed),
      v_ego=v_ego,
      gas_pressed=bool(CS.out.gasPressed),
      brake_pressed=bool(CS.out.brakePressed),
      standstill=bool(CS.out.standstill),
      plan_alive=plan_alive,
      csc_active=curve_active,
      csc_speed=min(float(curve_speed), 100.0),
      curve_floor=curve_floor,
      curve_budget=curve_budget,
      curve_track=curve_track,
      curve_raise=curve_raise,
      curve_no_accel=curve_no_accel,
      curve_hold=curve_hold,
      street_lower=curve_street,
      fast_restore=fast_restore,
      other_button_active=other_button_active,
      driver_button=SET_MINUS if CS.crz_set_minus else (SET_PLUS if CS.crz_set_plus else None),
      stop_active=stop_active and self.stop_mode == ON,
      stop_target=float(self.trigger.target),
      stop_urgent=bool(self.trigger.urgent),
      stop_restore_now=bool(self.trigger.restore_now) and self.stop_mode == ON,
      hold_restore=bool(self.trigger.hold_restore) and self.stop_mode == ON,
      btn_counter=int(getattr(CS, "crz_btns_counter", -1)),
      radar_lead=bool(self.trigger.radar_stable),
      stop_clear_ahead=bool(self.trigger.clear_ahead) and self.stop_mode == ON,
      stop_pulling_away=bool(self.trigger.pulling_away) and self.stop_mode == ON,
      stop_slower_ahead=bool(self.trigger.slower_after_red) and self.stop_mode == ON,
      limit_raise=self.limit_want if self.limit_mode == LIMIT_ON else 0.0,
      limit_cap=limit_cap if self.limit_mode == LIMIT_ON else 0.0,
      limit_undo=limit_undo if self.limit_mode == LIMIT_ON else 0.0,
      limit_ceiling=limit_ceiling if self.limit_mode == LIMIT_ON else 0.0,
    )
    button, offset = self.logic.update(inputs)
    if self.exit_brake_frames > 0:
      self.exit_brake_frames -= 1
    if self.limit_follow is not None:
      try:
        self._limit_bookkeeping(inputs)
      except Exception:
        pass
    if self.profile is not None:
      try:
        import time
        self._learn_profile(inputs, stop_inputs, bool(lead.status) and self._alive('radarState'), float(lead.dRel), time.monotonic())
      except Exception:
        self.profile = None   # never let the profile affect anything else
    try:
      self._log(inputs, stop_inputs, stop_active, button, float(CS.out.aEgo), plan)
    except Exception:
      pass
    if self.learner is not None:
      try:
        self._learn(CC, CS, v_ego, cruise_enabled, blinker)
      except Exception:
        self.learner = None   # never let the learner affect anything else
    if self.follow is not None and self.frame % self.FOLLOW_EVERY == 0:
      # Test only: works out and logs the set speed the model would want; its result never reaches `button`
      try:
        self._model_follow(CS, stop_inputs, cruise_enabled, v_ego, vel_t, vel_x, curve_active)
      except Exception as e:
        self._model_follow_failed(e)
    if button is None:
      return []
    btn = Buttons.SET_MINUS if button == SET_MINUS else Buttons.SET_PLUS
    return [mazdacan.create_button_cmd(packer, CP, CS.crz_btns_counter + offset, btn)]

  # ---- model follower (TEST ONLY, model_follow.py: logs the set speed it would command, never presses) ----
  FOLLOW_EVERY = 5                # frames: 20 Hz
  FOLLOW_ROW_EVERY = 10           # frames: a signals row at 10 Hz
  FOLLOW_SUMMARY_EVERY = 300.0    # s of cruise between two drive summary lines (also one at every cruise off)
  FOLLOW_SUMMARY_CSV = "model_follow_drives.csv"
  FOLLOW_SUMMARY_MAX_BYTES = 1_000_000
  # the episode's signals.csv (no GPS); speeds mph, the model's accel m/s^2
  FOLLOW_COLUMNS = ["vEgo_mph", "aEgo", "cruise", "set_mph", "driver_mph", "follow_mph", "target_mph", "reason", "model_a",
                    "model_a_f", "req_decel", "v_look_mph", "src", "lead", "lead_d", "lead_v_mph", "lead_prob", "red_light",
                    "features", "brake", "gas", "driver_btn", "taps"]
  # one line per write; a drive's last line has its totals (drive_start_wall tells the drives apart)
  FOLLOW_SUMMARY_COLUMNS = ["drive_start_wall", "t_wall", "route", "mode", "metric", "cruise_on_s", "follow_below_s", "lowers",
                            "changes", "would_taps", "would_actions", "episodes", "episodes_written", "episodes_skipped",
                            "follower_lower_eps", "actual_lower_eps", "src_modelV2_s", "src_drivingModelData_s", "no_model_s",
                            "plan_used_s", "ticks", "call_avg_us", "call_max_us", "error"]

  def _init_model_follow(self, params):
    import time
    from opendbc.car.mazda.model_follow import ModelFollower, EpisodeTracker, DT
    self.follow = ModelFollower(params.get_bool("IsMetric"), dt=DT)
    self.follow_eps = EpisodeTracker()
    self.follow_event = None
    self.follow_params = params
    self.follow_metric = bool(params.get_bool("IsMetric"))
    self.follow_start_wall = time.time()
    self.follow_error = ""
    self.follow_stats = {"cruise_on_s": 0.0, "v2": 0.0, "dmd": 0.0, "none": 0.0, "plan": 0.0, "ticks": 0, "us_sum": 0.0,
                         "us_max": 0.0}
    self.follow_prev_cruise = False
    self.follow_since_write = 0.0
    self.follow_rec = None
    try:
      from openpilot.starpilot.common.feature_events import EventRecorder
      self.follow_rec = EventRecorder(self.FOLLOW_COLUMNS, hz=10, params=params)
    except Exception:
      self.follow_rec = None

  def _follow_features(self, curve_active):
    """Which of our features are on now, e.g. "stop:stopped car+curve+ours"."""
    lg = self.logic
    parts = []
    if self.trigger.active and self.stop_mode:
      parts.append(f"stop:{self.trigger.kind}")
    if curve_active:
      parts.append("curve")
    if self.map_out is not None and self.map_out.active:
      parts.append("map")
    if self.limit_want > 0 or (self.limit_follow is not None and self.limit_follow.cap is not None):
      parts.append("limit")
    if lg.offset != 0 or lg.inflight:
      parts.append("ours")
    return "+".join(parts)

  def _model_follow(self, CS, si, cruise_enabled, v_ego, vel_t, vel_x, curve_active):
    import time
    c0 = time.perf_counter()
    lg = self.logic
    st = self.follow_stats
    t = time.monotonic()
    dt = self.FOLLOW_EVERY * DT_CTRL
    set_speed = float(CS.out.cruiseState.speed) if cruise_enabled else 0.0
    driver = (lg.anchor if lg.anchor is not None else set_speed) if set_speed > 0 else 0.0
    src, accel, plan_t, plan_v = "", None, None, None
    if si.model_alive:
      src, accel = "v2", si.model_accel
      if len(vel_t) >= 2:
        plan_t, plan_v = vel_t, vel_x
    else:
      try:
        if self.sm.alive['drivingModelData'] and self.sm.recv_frame['drivingModelData'] > 0:
          src, accel = "dmd", float(self.sm['drivingModelData'].action.desiredAcceleration)
      except Exception:
        src, accel = "", None    # not subscribed (or no such service): no model, the follower is off
    f = self.follow
    cmd = f.update(v_ego, driver, cruise_enabled, accel, plan_t, plan_v, radar_lead=bool(getattr(CS, "radar_has_lead", False)))
    if cruise_enabled:
      st["cruise_on_s"] += dt
      self.follow_since_write += dt
      st["v2" if src == "v2" else ("dmd" if src == "dmd" else "none")] += dt
      if f.used_plan:
        st["plan"] += dt
    brake, gas = bool(CS.out.brakePressed), bool(CS.out.gasPressed)
    driver_btn = bool(getattr(CS, "crz_set_minus", False) or getattr(CS, "crz_set_plus", False))
    features = self._follow_features(curve_active)
    model_a = accel if accel is not None else 0.0
    rec = self.follow_rec
    # the episodes run on the car's own clock (frames); the event log on the monotonic clock of its rows
    acts = self.follow_eps.update(self.frame * DT_CTRL, cruise_enabled, v_ego, si.a_ego, cmd, set_speed, driver, model_a, f.reason, f.presses,
                                  f.press_actions, si.lead_status, si.lead_d, si.lead_v, si.lead_prob, si.red_light, features,
                                  brake, gas, driver_btn)
    if rec is not None:
      if self.frame % self.FOLLOW_ROW_EVERY == 0 or f.presses:
        rec.add_row(t, (v_ego / MS_PER_MPH, si.a_ego, cruise_enabled, set_speed / MS_PER_MPH, driver / MS_PER_MPH, cmd / MS_PER_MPH,
                        f.target / MS_PER_MPH, f.reason, model_a, f.a_f if f.a_f is not None else 0.0, f.req, f.v_look / MS_PER_MPH,
                        src, si.lead_status, si.lead_d, si.lead_v / MS_PER_MPH, si.lead_prob, si.red_light, features, brake, gas,
                        driver_btn, f.presses))
      for kind, summary in acts or ():
        if kind == "open":
          summary.update({"src": src, "plan_used": f.used_plan, "metric": self.follow_metric})
          self.follow_event = rec.open("modelFollow", t, summary)
        elif self.follow_event is not None:
          rec.close(self.follow_event, t, summary)
          self.follow_event = None
      rec.tick(t)
    # the drive summary line: at every cruise off, and every FOLLOW_SUMMARY_EVERY s of cruise
    if (self.follow_prev_cruise and not cruise_enabled) or self.follow_since_write >= self.FOLLOW_SUMMARY_EVERY:
      self.follow_since_write = 0.0
      self._write_follow_summary()
    self.follow_prev_cruise = cruise_enabled
    us = (time.perf_counter() - c0) * 1e6
    st["ticks"] += 1
    st["us_sum"] += us
    if us > st["us_max"]:
      st["us_max"] = us

  def _model_follow_failed(self, e):
    """Any exception in the follower: off for the rest of the drive, said once (carlog, an event, the summary line)."""
    import time
    self.follow = None
    try:
      self.follow_error = f"{type(e).__name__}: {e}"[:200].replace(",", ";").replace("\n", " ")
      from opendbc.car.carlog import carlog
      carlog.exception("mazda model follower failed, off for this drive")
    except Exception:
      pass
    try:
      if self.recorder is not None:
        self.recorder.record("modelFollowError", time.monotonic(), {"error": self.follow_error}, signals=False)
      self._write_follow_summary()
    except Exception:
      pass

  def _follow_summary_row(self):
    import time
    st, eps = self.follow_stats, self.follow_eps
    f = self.follow
    ticks = max(st["ticks"], 1)
    return {"drive_start_wall": f"{self.follow_start_wall:.0f}", "t_wall": f"{time.time():.0f}", "mode": "test",
            "metric": int(self.follow_metric), "cruise_on_s": round(st["cruise_on_s"], 1),
            "follow_below_s": round(f.below_time, 1) if f else "", "lowers": f.lowers if f else "",
            "changes": f.changes if f else "", "would_taps": f.taps if f else "", "would_actions": f.actions if f else "",
            "episodes": eps.episodes, "episodes_written": eps.written, "episodes_skipped": eps.skipped,
            "follower_lower_eps": eps.follower_lower, "actual_lower_eps": eps.actual_lower,
            "src_modelV2_s": round(st["v2"], 1), "src_drivingModelData_s": round(st["dmd"], 1), "no_model_s": round(st["none"], 1),
            "plan_used_s": round(st["plan"], 1), "ticks": st["ticks"], "call_avg_us": round(st["us_sum"] / ticks, 1),
            "call_max_us": round(st["us_max"], 1), "error": self.follow_error}

  def _write_follow_summary(self):
    import threading
    row = self._follow_summary_row()
    threading.Thread(target=self._write_follow_line, args=(row,), daemon=True).start()

  @staticmethod
  def _leave_priority():
    """A writer thread inherits card's SCHED_FIFO and core (outside review of e1be62454; drives 3b / 44: "System
    Lagging" from file writes at card's priority): normal priority elsewhere first (car_profile.leave_caller_priority)."""
    try:
      from opendbc.car.mazda.car_profile import leave_caller_priority
      leave_caller_priority()
    except Exception:
      pass

  def _write_follow_line(self, row):
    import os
    self._leave_priority()
    try:
      try:
        row["route"] = str(self.follow_params.get("CurrentRoute", encoding="utf8") or "")
      except Exception:
        row["route"] = ""
      root = self._learning_dir()
      os.makedirs(root, exist_ok=True)
      path = os.path.join(root, self.FOLLOW_SUMMARY_CSV)
      if os.path.exists(path) and os.path.getsize(path) > self.FOLLOW_SUMMARY_MAX_BYTES:
        os.replace(path, path + ".1")
      new = not os.path.exists(path)
      with open(path, "a") as fh:
        if new:
          fh.write(",".join(self.FOLLOW_SUMMARY_COLUMNS) + "\n")
        fh.write(",".join(str(row.get(k, "")).replace(",", ";") for k in self.FOLLOW_SUMMARY_COLUMNS) + "\n")
    except Exception:
      pass

  # ---- curve learner (log only; called inside try/except) ----
  LEARNING_CSV = "curve_learning.csv"
  LEARNING_STATE = "curve_learning_state.json"
  LEARNING_CSV_MAX_BYTES = 2_000_000
  LEARNING_CSV_COLUMNS = ["t_wall", "duration_s", "lat_p90", "lat_peak", "speed_at_peak_mph", "mean_speed_mph",
                          "engaged_share", "cruise_off_share", "gas_s", "blinker", "our_slowdown", "driver_help_lat",
                          "saturated_lat", "too_fast_lat", "capability_before", "preference_before", "used",
                          "capability", "preference", "preference_confidence", "suggested", "lat_limit_used", "ti"]

  @classmethod
  def _learning_dir(cls):
    from openpilot.starpilot.common.feature_events import EVENTS_ROOT
    return EVENTS_ROOT

  def _load_learning_state(self):
    import json
    import os
    try:
      with open(os.path.join(self._learning_dir(), self.LEARNING_STATE)) as f:
        return json.load(f)
    except Exception:
      return None

  def _learn(self, CC, CS, v_ego, cruise_enabled, blinker):
    from opendbc.car.mazda.curve_learner import LearnerInputs
    lat = None
    if self._alive('livePose'):
      pose = self.sm['livePose']
      if pose.inputsOK:
        # device-frame yaw rate (the device faces forward; its small mount pitch changes this by < 1 %)
        lat = float(pose.angularVelocityDevice.z) * v_ego
    rec = self.learner.update(LearnerInputs(
      v_ego=v_ego, lat_accel=lat, lat_active=bool(CC.latActive), steering_pressed=bool(CS.out.steeringPressed),
      steer_cmd=abs(float(CC.actuators.torque)), cruise_enabled=cruise_enabled, gas_pressed=bool(CS.out.gasPressed),
      brake_pressed=bool(CS.out.brakePressed), blinker=blinker, our_offset=self.logic.offset,
      driver_set_minus=bool(CS.crz_set_minus)))
    if rec is not None:
      import threading
      rec.update(lat_limit_used=self.lat_limit, ti=self.has_ti)
      threading.Thread(target=self._write_learning, args=(rec, self.learner.state()), daemon=True).start()

  def _write_learning(self, rec, state):
    import json
    import os
    import time
    self._leave_priority()
    try:
      root = self._learning_dir()
      os.makedirs(root, exist_ok=True)
      path = os.path.join(root, self.LEARNING_CSV)
      if os.path.exists(path) and os.path.getsize(path) > self.LEARNING_CSV_MAX_BYTES:
        os.replace(path, path + ".1")
      new = not os.path.exists(path)
      rec = dict(rec, t_wall=f"{time.time():.1f}")
      with open(path, "a") as f:
        if new:
          f.write(",".join(self.LEARNING_CSV_COLUMNS) + "\n")
        f.write(",".join(str(rec.get(k, "")).replace(",", ";") for k in self.LEARNING_CSV_COLUMNS) + "\n")
      tmp = os.path.join(root, self.LEARNING_STATE + ".tmp")
      with open(tmp, "w") as f:
        json.dump(state, f, indent=1)
      os.replace(tmp, os.path.join(root, self.LEARNING_STATE))
    except Exception:
      pass

  # ---- map curve logs ----
  def _heading_turned(self, CS, CP, v_ego):
    """speed_limit_follow rule 7: the car's heading changed TURN_HEADING within TURN_HEADING_TIME and _DIST (its yaw
    from the speed and the steering angle: v tan(angle / steer ratio) / wheelbase)."""
    import math
    from opendbc.car.mazda.speed_limit_follow import HeadingWatch
    if getattr(self, "heading", None) is None:
      self.heading = HeadingWatch()
    ratio = float(getattr(CP, "steerRatio", 0.0) or 15.5)
    wheelbase = float(getattr(CP, "wheelbase", 0.0) or 2.7)
    angle = float(getattr(CS.out, "steeringAngleDeg", 0.0) or 0.0)
    rate = max(v_ego, 0.0) * math.tan(math.radians(angle) / ratio) / wheelbase
    self.heading.update(DT_CTRL, math.degrees(rate), v_ego)
    return self.heading.turned()

  def _street_active(self, plan):
    """speed_limit_follow rules 8 / 9 act (Speed Limit Follow and Map Curves On, the planner's map curves On)."""
    from opendbc.car.mazda.speed_limit_follow import ON as LIMIT_ON
    from opendbc.car.mazda.map_curve_control import ON as MAP_ON
    return (self.limit_follow is not None and self.limit_mode == LIMIT_ON and self.map_control is not None and
            self.map_mode == MAP_ON and int(getattr(plan, "mapCurveMode", 0) or 0) == MAP_ON)

  def _limit_lower(self, plan):
    """m/s: speed_limit_follow's street speed for the map control to carry out (_street_active only), 0 none."""
    if not self._street_active(plan):
      return 0.0
    return self.limit_follow.lower_speed

  @property
  def limit_slow_limit(self):
    """starpilotCarState.mazdaLimitSlowLimit: the lower posted limit we are slowing for (rule 9, On), m/s; 0 none."""
    lf = self.limit_follow
    if lf is None or lf.lower is None or lf.lower.kind != "lower" or self.map_control is None or not self.map_control.ll_on:
      return 0.0
    return float(lf.lower.new or 0) * lf.step

  @property
  def exit_brake_alert(self):
    """starpilotCarState.mazdaExitCurveBrake: up for EXIT_CHIME_HOLD after each "Exit curve - brake" chime (On)."""
    return self.exit_brake_frames > 0

  def _lanes(self):
    """Feed the camera's lane lines and road edges near the car to the exit lane check (LaneView)."""
    from opendbc.car.mazda.map_curve_control import LANE_NEAR_X
    lane = self.map_control.lane
    if not self._alive('modelV2'):
      lane.update(DT_CTRL, False)
      self.lane_frame = self.frame
      return
    if not self.sm.updated['modelV2']:
      return
    dt = max(self.frame - self.lane_frame, 1) * DT_CTRL
    self.lane_frame = self.frame
    m = self.sm['modelV2']

    def near(line):
      xs, ys = list(line.x), list(line.y)
      pts = [y for x, y in zip(xs, ys, strict=False) if x <= LANE_NEAR_X]
      return sum(pts) / len(pts) if pts else (ys[0] if ys else 0.0)
    lines, edges = list(m.laneLines), list(m.roadEdges)
    probs = list(m.laneLineProbs)
    if len(lines) < 4 or len(edges) < 2:
      lane.update(dt, False)
    else:
      lane.update(dt, True, probs, [near(x) for x in lines], [near(e) for e in edges], list(m.roadEdgeStds))
    # the model path's heading ahead against our lane's (map_curve_control.ModelPath): a few points of each
    from opendbc.car.mazda.map_curve_control import model_path_heading
    ours = [(lines[k].x, lines[k].y) for k in (1, 2) if len(lines) >= 4 and len(probs) >= 4 and probs[k] > 0.5]
    rel = model_path_heading(m.position.x, m.position.y, ours, lane.left)
    if rel is not None:
      self.map_control.model.update(rel)

  def _exit_row(self, plan):
    mc = self.map_control
    if mc is None or plan is None:
      return (0.0, 0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, "", 0.0, 0.0, 0, 0, 0.0, -1, 0.0, 0.0)
    ln = mc.lane
    rem = mc.exit_rem
    edge = (2 if ln.edge[-1] and ln.edge[1] else (1 if ln.edge[1] else (-1 if ln.edge[-1] else 0)))
    g = lambda k: float(getattr(plan, k, 0.0) or 0.0)  # noqa: E731
    return (round(rem, 1) if rem is not None else 0.0, int(getattr(plan, "mapExitShape", 0) or 0), g("mapExitLinkDist"),
            g("mapExitHighwayDist"), g("mapExitLinkHeading"), g("mapExitHighwayHeading"), int(getattr(plan, "mapExitFork", 0) or 0),
            g("mapExitCurveRadius"), g("mapExitCurveDistance"), "/".join(f"{p:.2f}" for p in ln.probs),
            round(ln.gaps[0], 2), round(ln.gaps[1], 2), edge, ln.change,
            round(mc.model.rel, 2) if mc.model.rel is not None and mc.model.age <= 0.5 else 0.0,
            -1 if mc.model_v is None else mc.model_v, g("mapExitHighwayBend"), g("mapExitLinkBend"))

  def _exit_episode(self, t, i, plan):
    """One "mapExit" event per navigation exit (EXIT_LOG_*): what the lane check, the shape check and the confirmation
    did around the split."""
    mc = self.map_control
    if self.recorder is None or mc is None:
      return
    p, rem = mc.plan, mc.exit_rem
    if self.exit_event is None and p is not None and rem is not None and -self.EXIT_LOG_AFTER < rem <= self.EXIT_LOG_BEFORE:
      self.exit_event = self.recorder.open("mapExit", t, {
        "mode": "test" if self.map_mode == 1 else "on", "exit_m": round(rem, 1), "side": p.side,
        "limit_mph": p.limit / MS_PER_MPH, "split_mph": mc.split_speed(p) / MS_PER_MPH, "split_setting": mc.split_mode,
        "curve_r": p.curve[0], "curve_after_split_m": round(p.curve[1] - p.exit_at, 1) if p.curve[0] > 0 else 0.0,
        "set_mph": i.set_speed / MS_PER_MPH, "vEgo_mph": i.v_ego / MS_PER_MPH, "signalled": p.signalled,
        "lane_edge": bool(p.side and mc.lane.edge.get(p.side, False))})
      self.exit_track = {"plan": p, "edge_s": 0.0, "sig_s": 0.0, "v_split": None, "rem_confirm": None, "chimes": mc.chime_count,
                         "min_v": i.v_ego, "brake": False, "model_link_m": None, "model_hwy_m": None, "shape_link_m": None,
                         "model_seen": False, "tentative_m": None}
    if self.exit_event is None:
      return
    tr = self.exit_track
    q = tr["plan"]
    tr["min_v"] = min(tr["min_v"], i.v_ego)
    tr["brake"] |= i.brake_pressed
    here = q.exit_at - mc.odo
    if q.side and mc.lane.edge.get(q.side, False) and here > 0:
      tr["edge_s"] += DT_CTRL
    if q.signalled and here > 0:
      tr["sig_s"] += DT_CTRL
    if tr["v_split"] is None and here <= 0:
      tr["v_split"] = i.v_ego / MS_PER_MPH
    if tr["rem_confirm"] is None and q.confirmed:
      tr["rem_confirm"] = round(here, 1)
    # where the model path first said link / highway, the shapes link, the tentative slowdown began (m from the exit point)
    tr["model_seen"] |= mc.model_v is not None
    if tr["model_link_m"] is None and mc.model_v == 2:
      tr["model_link_m"] = round(-here, 1)
    if tr["model_hwy_m"] is None and mc.model_v == 1 and here < 0:
      tr["model_hwy_m"] = round(-here, 1)
    if tr["shape_link_m"] is None and int(getattr(plan, "mapExitShape", 0) or 0) == 2:
      tr["shape_link_m"] = round(-here, 1)
    if tr["tentative_m"] is None and q.tentative:
      tr["tentative_m"] = round(-here, 1)
    if mc.plan is not q or here < -self.EXIT_LOG_AFTER:
      self.recorder.close(self.exit_event, t, {
        "result": "confirmed" if q.confirmed else ("declined" if q.declined else (q.why or "past the log window")),
        "ended": q.why, "declined": q.declined, "confirmed_m_from_exit_point": tr["rem_confirm"],
        "v_at_split_mph": tr["v_split"], "min_speed_mph": tr["min_v"] / MS_PER_MPH, "driver_braked": tr["brake"],
        "edge_lane_s": round(tr["edge_s"], 1), "signalled_s": round(tr["sig_s"], 1),
        "chimes": mc.chime_count - tr["chimes"], "chime_why": mc.chime_why if mc.chime_count > tr["chimes"] else "",
        "model_seen": tr["model_seen"], "model_link_m_past": tr["model_link_m"], "model_highway_m_past": tr["model_hwy_m"],
        "shape_link_m_past": tr["shape_link_m"], "tentative_m_past": tr["tentative_m"], "settled": q.settled,
        "set_mph_after": i.set_speed / MS_PER_MPH})
      self.exit_event = None

  def _map_row(self, plan):
    mo = self.map_out
    if plan is None or self.map_control is None:
      return (0,) * 21
    return (int(plan.mapCurveMode), int(plan.mapCurveStatus), float(plan.mapCurveDistance), float(plan.mapCurveRadius),
            bool(mo.active) if mo else False, (mo.speed / MS_PER_MPH if mo and mo.active else 0.0), (mo.why if mo else ""),
            bool(plan.mapOnRamp), int(plan.mapRampType), float(plan.mapRampToSpeed) / MS_PER_MPH, bool(plan.mapRampToHighway),
            float(plan.mapRampEndDistance), int(plan.mapNavExit), float(plan.mapNavExitDistance), float(plan.mapNavCurveRadius),
            bool(getattr(self, "_blinker", False)), self.map_control.state(),
            float(getattr(plan, "mapCurveAdvisory", 0.0)) / MS_PER_MPH, int(getattr(plan, "mapCurveVeto", 0)),
            int(getattr(plan, "mapGpsTrust", 0)), float(getattr(plan, "mapGpsAccuracy", 0.0)))

  def _map_episode(self, t, i, plan):
    """One event per map curve / ramp action (Test or On): from the map reason starting until it is over
    and the set speed is back (On) or the reason is gone (Test)."""
    mo = self.map_out
    if self.recorder is None or mo is None or plan is None:
      return
    lg = self.logic
    if self.map_event is None and mo.active:
      self.map_event = self.recorder.open("mapCurve", t, {
        "mode": "test" if self.map_mode == 1 else "on", "why": mo.why, "name": plan.mapCurveName,
        "radius_m": float(plan.mapCurveRadius), "distance_m": float(plan.mapCurveDistance),
        "advisory_mph": float(getattr(plan, "mapCurveAdvisory", 0.0)) / MS_PER_MPH,
        "target_mph": mo.speed / MS_PER_MPH, "set_mph": i.set_speed / MS_PER_MPH, "vEgo_mph": i.v_ego / MS_PER_MPH,
        "plan_mph": (mo.plan / MS_PER_MPH if mo.plan < 100.0 else 0.0),
        "curve_speed_mph": (mo.curve_speed / MS_PER_MPH if mo.curve_speed < 100.0 else 0.0), "track": bool(mo.track),
        "max_drop_steps": lg.max_drop_steps,
        "lat_limit": self.lat_limit, "on_ramp": bool(plan.mapOnRamp), "ramp_type": int(plan.mapRampType),
        "ramp_to_mph": float(plan.mapRampToSpeed) / MS_PER_MPH, "ramp_to_highway": bool(plan.mapRampToHighway),
        "nav_exit": int(plan.mapNavExit), "nav_exit_m": float(plan.mapNavExitDistance),
        "source": int(getattr(plan, "mapCurveSource", 0)), "gps_trust": int(getattr(plan, "mapGpsTrust", 0)),
        "gps_acc_m": float(getattr(plan, "mapGpsAccuracy", 0.0)), **self._profile_summary()})
      self.map_min_speed = i.v_ego
      self.map_vetoed, self.map_veto_radius = False, 0.0
      self.map_min_set = i.set_speed
      self.map_brake = False
    if self.map_event is not None:
      self.map_min_speed = min(self.map_min_speed, i.v_ego)
      self.map_min_set = min(self.map_min_set, i.set_speed) if i.set_speed > 0 else self.map_min_set
      self.map_brake |= i.brake_pressed
      if int(getattr(plan, "mapCurveVeto", 0)) == 1:
        self.map_vetoed, self.map_veto_radius = True, float(getattr(plan, "mapCurveVetoRadius", 0.0))
      done = not mo.active and (self.map_mode == 1 or (lg.offset == 0 and not lg.inflight))
      if done:
        self.recorder.close(self.map_event, t, {"min_speed_mph": self.map_min_speed / MS_PER_MPH,
                                                "min_set_mph": self.map_min_set / MS_PER_MPH, "driver_braked": self.map_brake,
                                                "set_mph_after": i.set_speed / MS_PER_MPH, "last_why": mo.why,
                                                "vetoed": self.map_vetoed, "veto_radius_m": self.map_veto_radius})
        self.map_event = None

  # ---- per-event logs (never raises into the car controller: called inside try/except) ----
  def _limit_bookkeeping(self, i):
    """Close the follower's offer once taken (On) or noted (Test), and log it; the same for a lower limit's cap."""
    lg, lf = self.logic, self.limit_follow
    unit = lf.step / MS_PER_MPH
    adopted = lg.limit_adopted
    if adopted is None and self.limit_mode == 1 and self.limit_want > 0:
      adopted = (i.set_speed + lg.offset * lg.step, self.limit_want)
    if adopted is not None:
      o = lf.offer
      lf.mark_adopted()
      if self.recorder is not None:
        import time
        gap_steps = int(round((adopted[1] - i.set_speed) / lg.step))
        self.recorder.record("speedLimitAdopt", time.monotonic(), {
          "mode": "on" if self.limit_mode == 2 else "test",
          "old_mph": (o.old * unit) if o and o.old is not None else None, "kind": o.kind if o else "",
          "new_mph": adopted[1] / MS_PER_MPH, "source": o.source if o else "", "set_mph": i.set_speed / MS_PER_MPH,
          "driver_mph_before": adopted[0] / MS_PER_MPH, "vEgo_mph": i.v_ego / MS_PER_MPH, "gap_steps": gap_steps,
          "how": "taps" if gap_steps <= lg._quantum_steps() else "hold to 5 below, then taps",
          "offer_age_s": round((lf.frame - o.frame) * DT_CTRL, 2) if o else None, "clear_ahead": i.stop_clear_ahead,
          "stop_episode": lg.stop_episode, "limit_mph": (o.new * unit) if o else None,
          # (map lag: mapd's next limit and its distance when the offer was made)
          "next_limit_mph": lf.last_next[0] / MS_PER_MPH, "next_dist_m": round(lf.last_next[1], 1)},
          min_gap=0.0, signals=False)
    c = lf.cap
    if c is not None and c.driver is None:
      c = None                       # (cruise off when the lower limit came: the follower decides when it is back)
    if c is not None and c is not self.limit_cap_logged:
      # a lower limit (rule 3), its first frame: On, the controller took it if a restore was pending above it; Test: the
      # same check here (the set speed counted with our presses still in flight, which may still land)
      self.limit_cap_logged = c
      capped = lg.limit_capped
      if self.limit_mode == 1 and lg.anchor is not None and c.new * lf.step < lg.anchor - lg.half_step:
        capped = (lg.anchor, c.new * lf.step)
      if self.recorder is not None:
        import time
        flying = sum(p.remaining if p.direction == SET_PLUS else -p.remaining for p in lg.inflight)
        set_after = i.set_speed + flying * lg.step
        if capped is not None:
          self.recorder.record("speedLimitCap", time.monotonic(), {
            "mode": "on" if self.limit_mode == 2 else "test", "old_mph": c.old * unit, "new_mph": c.new * unit,
            "source": c.source, "driver_mph_before": capped[0] / MS_PER_MPH, "set_mph": i.set_speed / MS_PER_MPH,
            "set_mph_with_in_flight": set_after / MS_PER_MPH,
            "restore_to_mph": max(capped[1], set_after) / MS_PER_MPH, "vEgo_mph": i.v_ego / MS_PER_MPH,
            "cruise": i.cruise_enabled, "in_flight": len(lg.inflight), "stop_episode": lg.stop_episode},
            min_gap=0.0, signals=False)
        else:
          why = "lower limit: no restore pending, the set speed stays (never SET- for a lower limit)"
          lf.events.append(("speedLimitNotFollowed", {"old": c.old, "new": c.new, "source": c.source, "reason": why}))
    if c is not None and (self.limit_mode != 2 or lg.limit_cap is None):
      lf.mark_capped()               # nothing (more) to cap: nothing pending, the restore stopped there, or Test
    ce = lf.ceiling
    if ce is None:
      self.limit_ceiling_logged = None
    elif ce != self.limit_ceiling_logged and lg.anchor is not None and ce * lf.step < lg.anchor - lg.half_step:
      # a lower limit not steady yet holds a pending restore back (On; Test: would)
      self.limit_ceiling_logged = ce
      if self.recorder is not None:
        import time
        self.recorder.record("speedLimitHold", time.monotonic(), {
          "mode": "on" if self.limit_mode == 2 else "test", "limit_mph": ce * unit, "steady_mph": (lf.stable or 0) * unit,
          "driver_mph": lg.anchor / MS_PER_MPH, "set_mph": i.set_speed / MS_PER_MPH, "vEgo_mph": i.v_ego / MS_PER_MPH,
          "cruise": i.cruise_enabled}, min_gap=10.0, signals=False)
    u = lf.undo
    if u is not None:
      # rule 4: On, the controller dealt with it this frame (or had nothing of ours to take back); Test: logged only
      done = lg.limit_undone if self.limit_mode == 2 else None
      if self.recorder is not None:
        import time
        how = (done[2] if done is not None else
               ("test" if self.limit_mode != 2 else "nothing pending (the driver's own change, a brake or a slowdown)"))
        self.recorder.record("speedLimitUndo", time.monotonic(), {
          "mode": "on" if self.limit_mode == 2 else "test", "raised_to_mph": u.old * unit, "back_to_mph": u.new * unit,
          "source": u.source, "how": how, "driver_mph_before": (done[0] / MS_PER_MPH) if done is not None else None,
          "set_mph": i.set_speed / MS_PER_MPH, "vEgo_mph": i.v_ego / MS_PER_MPH, "cruise": i.cruise_enabled,
          "in_flight": len(lg.inflight), "stop_episode": lg.stop_episode}, min_gap=0.0, signals=False)
      lf.mark_undone()
    if lf.events:
      if self.recorder is not None:
        import time
        for kind, data in lf.events[-3:]:
          own = kind in ("speedLimitMatch", "speedLimitSlow")      # (rules 8 / 9: start, done, cancel, undo each)
          self.recorder.record(kind, time.monotonic(), {"old_mph": data["old"] * unit if data["old"] is not None else None,
                                                        "new_mph": data["new"] * unit, "source": data["source"],
                                                        "reason": data["reason"],
                                                        "driver_mph": (i.set_speed + lg.offset * lg.step) / MS_PER_MPH,
                                                        **({"mode": "on" if self.limit_mode == 2 else "test",
                                                            "vEgo_mph": i.v_ego / MS_PER_MPH, **data.get("log", {})}
                                                           if own else {})},
                               min_gap=0.0 if own else 10.0, signals=False)
      lf.events = []

  def _log(self, i, si, stop_active, button, a_ego, plan=None):
    if self.recorder is None:
      return
    import time
    t = time.monotonic()
    lg = self.logic
    if button is not None or self.frame % 5 == 0:
      self.recorder.add_row(t, (
        f"{time.time():.2f}", i.v_ego / MS_PER_MPH, a_ego, i.brake_pressed, i.gas_pressed, i.cruise_enabled, i.enabled,
        i.set_speed / MS_PER_MPH, i.csc_active, i.csc_speed / MS_PER_MPH,
        float(getattr(self, "_road_curv", 0.0)), bool(plan.cscControllingSpeed) if plan is not None else False,
        float(plan.cscSpeed) / MS_PER_MPH if plan is not None else 0.0,
        si.lead_status, si.lead_prob, si.lead_d, si.lead_v, si.lead_y, si.radar_has_lead, si.radar_lead_distance,
        stop_active, self.trigger.target / MS_PER_MPH,
        self.trigger.urgent, self.trigger.required, min(self.trigger.ttc, 99.0), self.trigger.floor_mode, lg.offset, len(lg.inflight),
        lg.hold is not None, {SET_MINUS: "M", SET_PLUS: "P"}.get(button, ""), i.driver_button or "", lg.blocked, lg.restoring,
        *self._map_row(plan), bool(self.decel_learner.active) if self.decel_learner else False, lg.latency or 0,
        (lg.anchor or 0.0) / MS_PER_MPH, si.plan_stop, si.plan_ratio, self.trigger.latched,
        self.trigger.early and self.trigger.active, self.trigger.lead_v_f, self.trigger.radar_stable,
        self.trigger.radar_follow, self.trigger.radar_hold,
        si.lead_y_path if si.lead_y_path is not None else si.lead_y, self.trigger.leaving, self.trigger.clear_ahead,
        self.trigger.warn_cond, round(self.trigger.warn_frames * DT_CTRL, 2), self.trigger.warn_start and self.trigger.active,
        self.trigger.kind if self.trigger.active else "",
        float(getattr(plan, "slcSpeedLimit", 0.0) or 0.0) / MS_PER_MPH if plan is not None else 0.0,
        str(getattr(plan, "slcSpeedLimitSource", "") or "") if plan is not None else "",
        self.limit_follow.stable_speed / MS_PER_MPH if self.limit_follow else 0.0,
        self.limit_follow.offer_speed / MS_PER_MPH if self.limit_follow else 0.0, self.limit_want / MS_PER_MPH,
        *self._exit_row(plan)))
    if lg.driver_events:
      # driver set speed changes while we managed it (what happened to the restore): one summary-only event per
      # frame (drive 44: five 400-row events in a few seconds helped make card lag); the episode events have the rows
      evs = [{k: (round(v / MS_PER_MPH, 2) if isinstance(v, float) and k != "steps" else v) for k, v in ev.items()}
             for ev in lg.driver_events]
      summary = dict(evs[-1])
      summary.update({"count": len(evs), "events": evs})
      self.recorder.record("driverSetChange", t, summary, min_gap=0.0, signals=False)
    lg.driver_events = []
    self._map_episode(t, i, plan)
    self._exit_episode(t, i, plan)
    # curve slowdown episode: from the first SET- for a curve to the restore being complete
    if self.curve_event is None and button == SET_MINUS and lg.last_action in ("tap", "hold") and not stop_active and i.csc_active:
      self.curve_event = self.recorder.open("curveSlowdown", t, {
        "mode": "on", "set_mph_before": i.set_speed / MS_PER_MPH, "vEgo_mph": i.v_ego / MS_PER_MPH,
        "csc_mph": i.csc_speed / MS_PER_MPH, "max_drop_steps": lg.max_drop_steps,
        "anchor_mph": (lg.anchor or i.set_speed) / MS_PER_MPH, **self._profile_summary()})
      self.curve_presses = 0
    if self.curve_event is not None:
      if button is not None:
        self.curve_presses += 1
      if lg.offset == 0 and not lg.inflight and not i.csc_active and not stop_active:
        self.recorder.close(self.curve_event, t, {"presses": self.curve_presses, "set_mph_after": i.set_speed / MS_PER_MPH,
                                                   "disabled": lg.disabled, "no_effect_trips": lg.no_effect_trips})
        self.curve_event = None
    # stopped car event: from the trigger going active until the reason has cleared and the restore is done
    if self.stop_event is None and stop_active:
      self.stop_event = self.recorder.open("stoppedCarSlowdown", t, {
        "mode": "test" if self.stop_mode == 1 else "on", "kind": self.trigger.kind, "model_accel": si.model_accel,
        "set_mph_before": i.set_speed / MS_PER_MPH,
        "vEgo_mph": i.v_ego / MS_PER_MPH, "lead_d": si.lead_d, "lead_v": si.lead_v, "lead_prob": si.lead_prob,
        "lead_y": si.lead_y, "target_mph": self.trigger.target / MS_PER_MPH, "required_decel": self.trigger.required,
        "ttc": min(self.trigger.ttc, 99.0), "urgent": self.trigger.urgent, "early": self.trigger.early,
        "latched": self.trigger.latched, "latch_age_s": round(self.trigger.latch_age, 2), "plan_ratio_at_start": si.plan_ratio,
        "plan_stop_d": si.plan_stop_d, "anchor_mph": ((lg.anchor or i.set_speed) / MS_PER_MPH),
        # the stop-light detector: on now, and for how long (the red light head start's reason); a guarded stop ahead
        "red_light": si.red_light, "red_on_s": round(self.trigger.red_frames * DT_CTRL, 2),
        "warn_guarded": self.trigger.warn_guarded, **self._profile_summary()})
      self.stop_presses_at_start = lg.stop_presses
      self.stop_brake = False
      self.stop_holds_at_start = len(lg.hold_log)
      self.stop_min_set = i.set_speed
      self.radar_hold_frames = 0
    if self.stop_event is not None:
      self.stop_min_set = min(self.stop_min_set, i.set_speed)
      self.radar_hold_frames = getattr(self, "radar_hold_frames", 0) + int(self.trigger.radar_hold)
      self.stop_brake = getattr(self, "stop_brake", False) or i.brake_pressed
      done = not stop_active and lg.offset == 0 and not lg.inflight
      if done or (not stop_active and self.stop_mode == 1):
        holds = lg.hold_log[self.stop_holds_at_start:]
        self.recorder.close(self.stop_event, t, {
          "release_reason": self.trigger.release_reason, "kind": self.trigger.kind,
          "set_mph_min": self.stop_min_set / MS_PER_MPH,
          "set_mph_after": i.set_speed / MS_PER_MPH, "holds": len(holds),
          "holds_detail": "; ".join(self._hold_text(h) for h in holds),
          "hold_overrun_learned": lg.hold_overrun, "hold_measured": lg.hold_measured, "holds_learned_all_drives": self.profile.holds if self.profile else 0,
          "holds_allowed": lg.holds_allowed, "disabled": lg.disabled,
          "radar_lead_rises_s": [round((f - self.trigger.start_frame) * DT_CTRL, 2) for f in self.trigger.radar_rises
                                 if f >= self.trigger.start_frame - 1500],
          "driver_braked": self.stop_brake, "early": self.trigger.early, "floor_mode": self.trigger.floor_mode,
          "prob_rise_s": (round((self.trigger.confirm_frame - self.trigger.start_frame) * DT_CTRL, 2)
                          if self.trigger.early and self.trigger.confirm_frame >= 0 else None),
          "anchor_mph_end": (lg.anchor or 0.0) / MS_PER_MPH,
          "radar_hold_s": round(getattr(self, "radar_hold_frames", 0) * DT_CTRL, 2),
          "warn_start": self.trigger.warn_start, "clear_release": self.trigger.clear_release,
          "unsure_from": bool(getattr(self.trigger, "unsure_from", False)),
          "warn_guarded": self.trigger.warn_guarded, "confirmed": self.trigger.confirmed})
        self.stop_event = None
    if self.near_miss is not None and self.stop_mode in (1, 2):
      cand = self.trigger.near_miss_candidate(si)
      self.near_miss.update(t, cand, stop_active, self.trigger.blockers(si) if cand else (),
                            {"lead_d": si.lead_d, "lead_v": si.lead_v, "lead_prob": si.lead_prob, "lead_y": si.lead_y,
                             "vEgo_mph": i.v_ego / MS_PER_MPH, "model_accel": si.model_accel,
                             "latched": self.trigger.latched, "plan_ratio": si.plan_ratio})
    self.recorder.tick(t)
