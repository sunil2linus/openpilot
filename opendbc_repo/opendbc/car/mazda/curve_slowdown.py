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
# Speed limit follow (speed_limit_follow.py, drive 4c): a higher posted limit becomes the driver's speed (the anchor);
# the restore rule above then brings the set speed up: 5 mph or less by taps, more by one hold to 5 below, then taps.
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
  curve_floor: float = 0.0        # m/s: when > 0 the curve may go this low (a ramp), past the max drop
  curve_budget: int = 0           # SET- presses allowed for this curve when more than the usual budget (a ramp)
  fast_restore: bool = False      # restore at once with SET+ holds (a ramp merging onto a highway)
  hold_restore: bool = False      # a stop is planned ahead (stopped car trigger latched): keep the lowered speed for now
  btn_counter: int = -1           # the wheel's CRZ_BTNS counter (-1 unknown: holds are timed by HOLD_FRAME_GAP)
  radar_lead: bool = False        # the stock radar follows a lead (CRZ_CTRL.RADAR_HAS_LEAD, debounced: trigger.radar_stable)
  # the posted limit rose to this (m/s) and the driver's speed matched the old one (speed_limit_follow.py): make it
  # the driver's speed
  limit_raise: float = 0.0
  # stopped car trigger: released by brake / disengage / lane exit and no slower car is in our lane: the restore may
  # use SET+ holds (stopped_car_slowdown CLEAR_*)
  stop_clear_ahead: bool = False
  # stopped car trigger: no car ahead in our lane, or it is pulling away from us (restores at any speed)
  stop_pulling_away: bool = False


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
    self.hold_bound = None           # set speed the current hold must not cross
    self.hold_margin = False         # the current hold is a restore hold (keeps a quantum of margin, see RESTORE_HOLD_*)
    self.stop_episode = False        # we lowered for a stopped car: the restore may use SET+ holds
    self.restore_pending = False     # the stopped car reason cleared for good: restore at once, even at standstill
    self.standstill_frames = 0
    self.last_action = None          # "tap" / "hold" / "restore" of the last frame sent (for logs)
    self.limit_adopted = None        # (driver's speed before, new limit) on the frame a higher limit was adopted
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

  def _forget(self):
    """Leave the set speed where it is and stop managing it (it becomes the driver's speed)."""
    self.anchor = None
    self.restoring = False
    self.no_effect = 0
    self.no_effect_trips = 0
    self.pause_frames = 0

  def _disable(self):
    self._forget()
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
    if self.offset == 0 and not self.inflight:
      self.flicker = False
      self.anchor = None             # back at the driver's speed: nothing left to restore
    restore_delay = FLICKER_RESTORE_DELAY if self.flicker else RESTORE_DELAY
    curve_over = self.curve_free_frames >= _frames(restore_delay)
    all_over = self.reason_free_frames >= _frames(restore_delay)
    if curve_over:
      self.lower_presses = 0
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
    if i.fast_restore and self.offset > 0 and not i.csc_active and not i.stop_active:
      self.restore_pending = True
    if self.offset == 0 and not self.inflight or i.stop_active:
      self.restore_pending = False
    speed_ok = i.v_ego >= (KEEP_SPEED if self.managing else START_SPEED) or (i.stop_active and i.v_ego >= KEEP_SPEED)
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
      targets.append(("curve", max(i.csc_speed + CURVE_MARGIN, floor)))
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
      # a SET- hold also stops when the target rises above its bound (the radar took over the car ahead)
      target_rose = d == SET_MINUS and reason == "stop" and target > self.hold_bound + self.half_step
      reason_gone = (reason != "stop" or target_rose) if d == SET_MINUS else (reason is not None or offset <= 0)
      no_effect = self.hold.landed == 0 and self.frame - self.hold.frame >= _frames(HOLD_NO_EFFECT_SECONDS)
      if (reason_gone or i.gas_pressed or i.other_button_active or no_effect or
          self.frame - self.hold.frame >= _frames(HOLD_MAX_SECONDS) or crosses):
        if target_rose:
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
    # restore below brings the set speed up. Not while a reason is active, the driver stands us down or is adjusting.
    if (i.limit_raise > 0 and reason is None and not self.blocked and not self.inflight and self.hold is None and
        self.last_set_speed is not None and i.limit_raise > driver_speed + self.half_step and
        self.frame - max(self.last_driver_edge.values()) >= _frames(LIMIT_DRIVER_QUIET)):
      self.limit_adopted = (self.anchor if self.anchor is not None else i.set_speed, i.limit_raise)
      self.anchor = i.limit_raise
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
    gas_restore = (GAS_JUMP_ENABLED and i.gas_pressed and not i.csc_active and not i.stop_active and
                   (self.restore_pending or i.stop_clear_ahead))
    button = None
    if want_lower:
      self.restoring = False
      button = SET_MINUS
    elif offset != 0 and (all_over or self.restore_pending or gas_restore) and \
         (self.frame - self.last_minus_frame >= _frames(MIN_DWELL) or self.restore_pending or gas_restore):
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
    if faster and v_hi + (GAS_JUMP_MARGIN if may_jump else 0) > int(round(driver_speed / self.step)):
      return None, None                # if it did jump it could land above the driver's speed
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
    if self.holds_allowed and not self.inflight and not jump and not i.gas_pressed:
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
      elif (button == SET_PLUS and not want_lower and not i.csc_active and (all_over or self.restore_pending) and
            (not self.stop_episode or fast_stop or i.fast_restore)):
        # one rule for every restore hold (RESTORE_HOLD_MIN_GAIN): a stop episode only when its car has clearly
        # gone (restore_now) or the radar follows a car; a lost / left-lane release taps so a re-trigger wins
        driver_steps = int(round(driver_speed / self.step))
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
    if want_lower and reason == "stop":
      self.stop_episode = True
    if button == SET_MINUS and self.anchor is None:
      self.anchor = i.set_speed      # the driver's speed, before our first press moves it
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
        p.reserve = p.remaining = max(1, int(round(driver_speed / self.step)) - int(round(i.set_speed / self.step)))
    if button == SET_MINUS and want_lower:
      if reason == "curve":
        self.lower_presses += 1
      else:
        self.stop_presses += 1
      self.last_minus_frame = self.frame
    if off is None:
      return None, None
    self.last_action = "hold" if hold else ("tap" if want_lower else ("jump" if jump else "restore"))
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
                   "ramp_type", "ramp_to_mph", "ramp_to_hwy", "ramp_end", "nav_exit", "nav_dist", "nav_r", "blinker", "exit_intent", "map_adv_mph",
                   "map_veto", "gps_trust", "gps_acc", "decel_learn", "answer_frames",
                   "anchor_mph", "plan_stop", "plan_ratio", "stop_latched", "stop_early",
                   "lead_v_f", "radar_stable", "radar_follow", "radar_hold",
                   "lead_y_path", "leaving", "clear_ahead", "warn_cond", "warn_s", "warn_start", "stop_kind",
                   "slc_mph", "slc_src", "limit_stable_mph", "limit_offer_mph", "limit_raise_mph"]

  def __init__(self, CP, params, curve_enabled=True, stop_mode=0, learning=False, map_mode=0, limit_mode=0):
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
    if limit_mode:
      from opendbc.car.mazda.speed_limit_follow import SpeedLimitFollower
      self.limit_follow = SpeedLimitFollower(params.get_bool("IsMetric"))
    self.limit_want = 0.0
    self.curve_enabled = curve_enabled
    self.stop_mode = stop_mode
    self.has_ti = bool(CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR)
    self.lat_limit = lateral_limit(self.has_ti, float(getattr(CP, "maxLateralAccel", 0.0) or 0.0))
    self.curve_target = CurveTarget(self.lat_limit)
    # The car profile: what earlier drives learned about this car (answer time, hold overrun, cruise decel), see
    # car_profile.py. Loaded here (a few KB), applied to the logic, saved in the background as it learns.
    from opendbc.car.mazda.curve_target import MAP_DECEL
    self.profile = None
    self.decel_learner = None
    self.decel_used = MAP_DECEL
    self.profile_holds_seen = 0
    self.prev_cruise = False
    try:
      self.profile = self._load_profile(CP, params)
      if self.profile is not None:
        self.decel_used = self.profile.decel(MAP_DECEL)
    except Exception:
      self.profile = None
    self.map_mode = map_mode
    self.map_control = None
    self.map_out = None
    self.map_event = None
    if map_mode:
      from opendbc.car.mazda.map_curve_control import MapCurveControl
      self.map_control = MapCurveControl(self.lat_limit, self.decel_used)
    self.learner = None
    services = ['starpilotPlan', 'radarState', 'modelV2']
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

  def _alive(self, service):
    return bool(self.sm.alive[service] and self.sm.recv_frame[service] > 0)

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
    self.sm.update(0)
    plan = self.sm['starpilotPlan']
    lead = self.sm['radarState'].leadOne
    cruise_enabled = bool(CS.out.cruiseState.enabled)
    v_ego = float(CS.out.vEgo)

    # Stopped car ahead (Test: trigger and target are computed and logged, nothing is pressed)
    from opendbc.car.mazda.stopped_car_slowdown import plan_stop
    p_stop, p_ratio, p_d = False, 1.0, 0.0
    path_x, path_y, model_length, lane_change = [], [], 0.0, False
    if self._alive('modelV2'):
      try:
        m = self.sm['modelV2']
        path_x, path_y = list(m.position.x), list(m.position.y)
        model_length = float(path_x[-1]) if path_x else 0.0
        p_stop, p_ratio, p_d = plan_stop(list(m.velocity.t), list(m.velocity.x), path_x, v_ego)
      except Exception:
        p_stop, p_ratio, p_d = False, 1.0, 0.0     # a malformed plan only loses the red-light latch
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
      set_speed=float(CS.out.cruiseState.speed) if cruise_enabled else 0.0)
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

    # Map curves (Test: computed and logged only; On: combined with the camera curve target)
    curve_floor, curve_budget, fast_restore = 0.0, 0, False
    self.map_out = None
    if self.map_control is not None and plan_alive:
      from opendbc.car.mazda.map_curve_control import MapFields, ON as MAP_ON
      signal = 1 if CS.out.rightBlinker else (-1 if CS.out.leftBlinker else 0)
      mo = self.map_out = self.map_control.update(MapFields.from_plan(plan), driver_speed, signal)
      if mo.adopt and lg.offset > 0 and self.map_mode == MAP_ON:
        lg._forget()        # off an exit onto a street: the lowered set speed is now the driver's speed
      if self.map_mode == MAP_ON and plan.mapCurveMode == MAP_ON:
        if mo.active and (not curve_active or mo.speed < curve_speed):
          curve_speed = mo.speed
          curve_floor, curve_budget = mo.floor, mo.budget
        curve_active = curve_active or mo.active
        fast_restore = mo.fast_restore

    # Posted limit rising (speed_limit_follow.py; Test: logged only)
    from opendbc.car.mazda.speed_limit_follow import ON as LIMIT_ON
    self.limit_want = 0.0
    if self.limit_follow is not None:
      from opendbc.car.mazda.speed_limit_follow import LimitInputs
      li = LimitInputs(limit=float(getattr(plan, "slcSpeedLimit", 0.0) or 0.0) if plan_alive else 0.0,
                       source=str(getattr(plan, "slcSpeedLimitSource", "") or ""),
                       gps_trust=int(getattr(plan, "mapGpsTrust", 0) or 0), driver_speed=driver_speed,
                       cruise_enabled=cruise_enabled)
      self.limit_want = self.limit_follow.update(li)

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
      limit_raise=self.limit_want if self.limit_mode == LIMIT_ON else 0.0,
    )
    button, offset = self.logic.update(inputs)
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
    if button is None:
      return []
    btn = Buttons.SET_MINUS if button == SET_MINUS else Buttons.SET_PLUS
    return [mazdacan.create_button_cmd(packer, CP, CS.crz_btns_counter + offset, btn)]

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
  def _map_row(self, plan):
    mo = self.map_out
    if plan is None or self.map_control is None:
      return (0,) * 21
    return (int(plan.mapCurveMode), int(plan.mapCurveStatus), float(plan.mapCurveDistance), float(plan.mapCurveRadius),
            bool(mo.active) if mo else False, (mo.speed / MS_PER_MPH if mo and mo.active else 0.0), (mo.why if mo else ""),
            bool(plan.mapOnRamp), int(plan.mapRampType), float(plan.mapRampToSpeed) / MS_PER_MPH, bool(plan.mapRampToHighway),
            float(plan.mapRampEndDistance), int(plan.mapNavExit), float(plan.mapNavExitDistance), float(plan.mapNavCurveRadius),
            bool(getattr(self, "_blinker", False)), bool(self.map_control.exit_intent),
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
    """Close the follower's offer once taken (On) or noted (Test), and log it."""
    lg, lf = self.logic, self.limit_follow
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
          "mode": "on" if self.limit_mode == 2 else "test", "old_mph": (o.old * lf.step / MS_PER_MPH) if o else None,
          "new_mph": adopted[1] / MS_PER_MPH, "source": o.source if o else "", "set_mph": i.set_speed / MS_PER_MPH,
          "driver_mph_before": adopted[0] / MS_PER_MPH, "vEgo_mph": i.v_ego / MS_PER_MPH, "gap_steps": gap_steps,
          "how": "taps" if gap_steps <= lg._quantum_steps() else "hold to 5 below, then taps",
          "offer_age_s": round((lf.frame - o.frame) * DT_CTRL, 2) if o else None, "clear_ahead": i.stop_clear_ahead,
          "stop_episode": lg.stop_episode}, min_gap=0.0, signals=False)
    if lf.events:
      if self.recorder is not None:
        import time
        for kind, data in lf.events[-3:]:
          unit = lf.step / MS_PER_MPH
          self.recorder.record(kind, time.monotonic(), {"old_mph": data["old"] * unit, "new_mph": data["new"] * unit,
                                                        "source": data["source"], "reason": data["reason"],
                                                        "driver_mph": (i.set_speed + lg.offset * lg.step) / MS_PER_MPH},
                               min_gap=10.0, signals=False)
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
        self.limit_follow.offer_speed / MS_PER_MPH if self.limit_follow else 0.0, self.limit_want / MS_PER_MPH))
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
        "plan_stop_d": si.plan_stop_d, "anchor_mph": ((lg.anchor or i.set_speed) / MS_PER_MPH), **self._profile_summary()})
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
          "unsure_from": bool(getattr(self.trigger, "unsure_from", False))})
        self.stop_event = None
    if self.near_miss is not None and self.stop_mode in (1, 2):
      cand = self.trigger.near_miss_candidate(si)
      self.near_miss.update(t, cand, stop_active, self.trigger.blockers(si) if cand else (),
                            {"lead_d": si.lead_d, "lead_v": si.lead_v, "lead_prob": si.lead_prob, "lead_y": si.lead_y,
                             "vEgo_mph": i.v_ego / MS_PER_MPH, "model_accel": si.model_accel,
                             "latched": self.trigger.latched, "plan_ratio": si.plan_ratio})
    self.recorder.tick(t)
