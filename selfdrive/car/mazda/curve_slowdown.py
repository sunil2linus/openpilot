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
RESTORE_MOVING_SPEED = 2.0            # m/s: after a stopped car, the driver's speed comes back once we move again
CURVE_MARGIN = 0.3                    # m/s: stay this far above the curve target
CRUISE_SETTLE = 0.3                   # s: cruise must be engaged this long before we press
RESTORE_DELAY = 0.5                   # s of FrogPilot saying the curve no longer limits us before raising
FLICKER_RESTORE_DELAY = 2.0           # used instead while the curve flag has just flickered back on
FLICKER_WINDOW = 3.0                  # s: flag coming back on this soon after going off = flicker
MIN_DWELL = 1.0                       # s at least between the last SET- and a SET+
PENDING_TIMEOUT = 0.5                 # s to wait for a press to show on the set speed (the car answers in < 0.2 s)
LATE_WINDOW = 3.0                     # s after the timeout during which a late change is still ours (car speed unknown)
LATE_FACTOR = 4                       # once the car's answer time is known, a press is lost after this many times it
MIN_IN_FLIGHT_WINDOW = 1.0            # s: a press is never forgotten sooner than this
DRIVER_RECENT = 1.5                   # s after the driver's last SET+/SET- edge (or while held): changes are theirs
MAX_IN_FLIGHT = 3                     # presses that may be unaccounted for before we stop sending
NO_EFFECT_LIMIT = 6                   # presses in a row with no effect before backing off (1 in 4 ignored is normal)
NO_EFFECT_BACKOFF = 10.0              # s to wait after that before trying again
NO_EFFECT_TRIPS = 3                   # back-offs before giving up (speed left alone)
PAUSE_ABANDON = 30.0                  # s paused before we forget the driver's speed
EXTRA_LOWER_PRESSES = 6               # SET- budget per curve = max drop steps + this (covers ignored presses)

# Stopped car ahead (shares this controller with the curve slowdown)
STOP_MIN_SET_SPEED = 20.0 * MS_PER_MPH   # floor for a stopped car: one step above the car's own 30 km/h minimum
MAX_STOP_PRESSES = 30                    # SET- budget (taps + holds) per stopped car
RELAXED_GAP_FRAMES = 50                  # 0.5 s between taps when the stopped car is not urgent
# Long press. Measured from the owner's own holds in the logs (5 holds, routes 31/32): on button-down
# the set speed drops 1 mph at once, ~0.5 s later it jumps to the next multiple of 5 mph, then 5 mph
# every 0.6 s while the button is held. What happens after release (how many more 5 mph steps can
# still land) is not visible in the logs, so it is assumed to be one quantum until measured, and every
# downward change during a hold and HOLD_SETTLE after it is ours.
LONG_PRESS_ENABLED = True
HOLD_FRAME_GAP = 10                      # frames between the hold's SET frames (stock 10 Hz cadence)
HOLD_MAX_SECONDS = 4.0                   # a hold is released after this at the latest
HOLD_NO_EFFECT_SECONDS = 1.0             # a hold with no set speed change by then is released (the car ignores it)
HOLD_QUANTUM_MPH = 5                     # the car's long-press step
HOLD_SETTLE = 1.5                        # s after the last frame or landing during which changes are still the hold's
HOLD_OVERRUN_DEFAULT = 1                 # quanta that may still land after a SET- release, until measured
HOLD_UP_OVERRUN_DEFAULT = 3              # assumed for SET+ holds until a hold has measured the car (then worst seen + 1)
HOLD_MISS_LIMIT = 2                      # holds with no effect before holds are off for the drive
HOLD_GOALS_MPH = (15, 10)                # stopped car: first hold aims 15 mph down, the second 10 more, then taps
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
  plan_alive: bool         # frogpilotPlan is being received
  csc_active: bool         # FrogPilot Curve Speed Controller wants to slow down
  csc_speed: float         # Curve Speed Controller target, true speed m/s
  other_button_active: bool  # openpilot is sending CANCEL/RESUME this frame
  driver_button: object = None  # SET_MINUS / SET_PLUS while the driver holds that stock button, else None
  stop_active: bool = False     # stopped car ahead: lower the set speed towards stop_target
  stop_target: float = 0.0      # m/s, true speed; clipped to [STOP_MIN_SET_SPEED, driver's speed] here
  stop_urgent: bool = False     # fast pace for the stopped car
  stop_restore_now: bool = False  # the stopped car reason just cleared (lead moving, ego stopped, lead lost): restore at once


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

    self.offset = 0                  # steps we are below the driver's speed (negative: above)
    self.pending = None              # the newest Press, until it lands or times out
    self.inflight = []               # [Press] not yet (fully) seen on the set speed, oldest first
    self.latencies = []              # frames from our press to the set speed change, learned on the road
    self.expired = []                # [Press] given up on, still claimable for a while
    self.driver_taps = []            # [(direction, frame)] of the driver's own recent taps, not yet seen on the set speed
    self.prev_driver_button = None
    self.last_driver_edge = {SET_MINUS: -10_000, SET_PLUS: -10_000}
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
    self.holds_ok = True             # allowed in the current episode
    self.hold_misses = 0
    self.hold_overrun = HOLD_OVERRUN_DEFAULT   # quanta seen to land after a release (max), starts conservative
    self.hold_measured = False
    self.hold_log = []               # [dict] one per hold, for the per-event log
    self.holds_this_episode = 0      # SET- holds for the current stopped car (HOLD_GOALS_MPH)
    self.hold_bound = None           # set speed the current hold must not cross
    self.stop_episode = False        # we lowered for a stopped car: the restore may use SET+ holds
    self.restore_pending = False     # the stopped car reason cleared for good: restore at once, even at standstill
    self.last_action = None          # "tap" / "hold" / "restore" of the last frame sent (for logs)

  @property
  def managing(self):
    return self.offset != 0 or bool(self.inflight)

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
    """Leave the set speed where it is and stop managing it."""
    self.offset = 0
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
    self.offset += n if direction == SET_MINUS else -n
    self.no_effect = 0
    self.no_effect_trips = 0

  def _quantum_steps(self):
    return int(round(HOLD_QUANTUM_MPH * 1.609344)) if self.step < MS_PER_MPH else HOLD_QUANTUM_MPH

  def _up_overrun(self):
    """Quanta assumed to land after a SET+ release: conservative until a hold has measured the car."""
    return (self.hold_overrun + 1) if self.hold_measured else HOLD_UP_OVERRUN_DEFAULT

  def _hold_reserve(self):
    """Steps a hold may still take after we stop sending (we release right after a quantum lands, so
    the next one is the overrun), plus the tap the button-down gives."""
    return self.hold_overrun * self._quantum_steps() + 1

  def _finish_hold(self, p):
    """A hold is resolved: learn how it landed."""
    if not p.hold:
      return
    q = self._quantum_steps()
    after = sum(n for f, n in p.landings if f > p.release_frame + HOLD_FRAME_GAP + _frames(0.2))
    quanta_after = -(-after // q)          # round up: a partial quantum after release counts as one
    if p.landed > q:
      # A real staircase was seen: the overrun after a release is learned as the worst seen, never
      # below one quantum (the release always comes just after a landing, so at most the next one
      # should still land; more than that means the car keeps stepping, which we then assume).
      self.hold_overrun = max(1, self.hold_overrun if self.hold_measured else 0, quanta_after)
      self.hold_measured = True
    elif p.landed == 0:
      self.hold_misses += 1
      self.holds_ok = False
    for entry in self.hold_log:
      if entry["frame"] == p.frame:
        entry.update(landed=p.landed, landed_after_release=after, landings=list(p.landings),
                     release_frame=p.release_frame, done=True)
    if p in self.inflight:
      self.inflight.remove(p)
    if p in self.expired:
      self.expired.remove(p)

  def _consume_ours(self, direction, n):
    """Account n steps in `direction` to our presses in flight, oldest first. A hold takes
    everything that comes its way (its reserve is a bound, not a count)."""
    while n > 0:
      p = next((pr for pr in self.inflight if pr.direction == direction and (pr.remaining > 0 or pr.hold)), None)
      if p is None:
        break
      take = n if p.hold else min(n, p.remaining)
      p.remaining = max(p.remaining - take, 0)
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
    self.last_set_speed = i.set_speed
    direction = SET_PLUS if delta > 0 else SET_MINUS
    steps = abs(delta) / self.step
    match = self._matching_press(direction)
    ours_k = match[1] if match is not None and match[0] == "ours" else None
    driver_k = match[1] if match is not None and match[0] == "driver" else None
    driver_recent = (driver_k is not None or self.prev_driver_button == direction or
                     self.frame - self.last_driver_edge[direction] <= _frames(DRIVER_RECENT))
    n_ours = sum(p.remaining for p in self.inflight if p.direction == direction)
    hold_open = any(p.hold for p in self.inflight if p.direction == direction)
    expired_k = next((k for k, p in enumerate(self.expired) if p.direction == direction), None)
    if abs(steps - round(steps)) > 0.25:
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
      # Their speed moves by this step: keep `offset`, so driver_speed = set_speed + offset * step
      # shifts with it. No more lowering for this curve.
      del self.driver_taps[driver_k]
      self.restoring = False
      self.blocked = True
    elif ours_k is not None and (n <= n_ours or hold_open):
      # Ours (one press, several presses landing between two samples, or a hold stepping several
      # at once). Any landing also confirms that the car is listening, so the next press may go out.
      if len(self.inflight) == 1 and not after_gap and not self.inflight[0].hold:
        # Unambiguous landing with cruise on throughout: learn how fast this car answers.
        self.latencies = (self.latencies + [self.frame - self.inflight[0].frame])[-9:]
      if hold_open and n > n_ours:
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
      # press first and this is our own press landing). Either way the driver's speed moves by
      # this step: keep `offset`, so driver_speed = set_speed + offset * step shifts with it.
      # No more lowering for this curve.
      self.restoring = False
      self.blocked = True
    elif driver_recent:
      # The driver held the button: they set a new speed.
      self.driver_taps = [(d, f) for d, f in self.driver_taps if d != direction]
      self._forget()
      self.blocked = True
    elif ours_k is not None:
      # More steps than we have presses in flight, in our direction: the car may step more than
      # once per press. We can't keep the limits reliably, so switch off for this drive.
      self._disable()
    else:
      # Several steps at once (e.g. holding the button): the driver set a new speed.
      self._forget()
      self.blocked = True

  def _send(self, button, hold=False):
    """Book-keeping for a frame going out now; returns the counter offset or None (skip frame)."""
    offset = COUNTER_OFFSETS[self.button_frame % len(COUNTER_OFFSETS)]
    self.button_frame += 1
    if offset is None:
      return None
    self.last_send_frame = self.frame
    if hold:
      if self.hold is None:
        reserve = self._hold_reserve() + self._quantum_steps()   # first quantum + what may follow a release
        p = Press(button, self.frame, reserve=reserve, remaining=reserve, hold=True)
        self.hold = p
        self.pending = p
        self.inflight.append(p)
        self.hold_log.append({"frame": self.frame, "direction": button, "frames_sent": 0, "landed": 0, "done": False,
                              "overrun_assumed": self.hold_overrun, "bound_mph": (self.hold_bound or 0) / MS_PER_MPH})
      self.hold.frames_sent += 1
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
    if self.disabled:
      return None, None
    cruise_ok = i.cruise_enabled and i.set_speed > 0
    self.cruise_on_frames = self.cruise_on_frames + 1 if cruise_ok else 0
    if not cruise_ok:
      if self.last_set_speed is not None:
        self.cruise_gap = True
      # Freeze the timers of a press in flight while cruise is off, so it is still
      # recognised as ours when cruise comes back (the set speed is not observed meanwhile).
      for p in self.inflight + self.expired:
        p.frame += 1
      self.hold = None     # a hold is not continued across a cruise cancel

    # The driver's own SET+/SET- taps (rising edge). Our own frames are never read back here:
    # pandad reports transmitted frames on bus + 128 and the bus-0 parser ignores those.
    if i.driver_button in (SET_MINUS, SET_PLUS):
      if i.driver_button != self.prev_driver_button:
        self.driver_taps.append((i.driver_button, self.frame))
        self.blocked = True
        self.hold = None   # the driver is on the buttons: do not keep holding
      self.last_driver_edge[i.driver_button] = self.frame     # kept fresh while held
    self.prev_driver_button = i.driver_button
    self.driver_taps = [(d, f) for d, f in self.driver_taps if self.frame - f <= self._in_flight_window()]

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
      self.reason_free_frames = 0 if (i.csc_active or i.stop_active) else self.reason_free_frames + 1
    if self.offset == 0 and not self.inflight:
      self.flicker = False
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

    if i.stop_restore_now and self.offset != 0:
      self.restore_pending = True
    if self.offset == 0 and not self.inflight or i.stop_active:
      self.restore_pending = False
    speed_ok = i.v_ego >= (KEEP_SPEED if self.managing else START_SPEED) or (i.stop_active and i.v_ego >= KEEP_SPEED)
    # Never press at standstill (SET at standstill is unverified on the car); after a stopped car the driver's
    # speed comes back as soon as we are moving again.
    can_act = (i.enabled and cruise_ok and i.plan_alive and not i.brake_pressed and not i.standstill and
               (speed_ok or (self.restore_pending and i.v_ego > RESTORE_MOVING_SPEED)) and
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
      overrun = (1 + (self.hold_overrun if d == SET_MINUS else self._up_overrun())) * q * self.step
      crosses = ((i.set_speed - overrun < self.hold_bound - self.half_step) if d == SET_MINUS else
                 (i.set_speed + overrun > self.hold_bound + self.half_step))
      reason_gone = (reason != "stop") if d == SET_MINUS else (reason is not None or offset <= 0)
      no_effect = self.hold.landed == 0 and self.frame - self.hold.frame >= _frames(HOLD_NO_EFFECT_SECONDS)
      if (reason_gone or i.gas_pressed or i.other_button_active or no_effect or
          self.frame - self.hold.frame >= _frames(HOLD_MAX_SECONDS) or crosses):
        self.hold = None
        return None, None
      if self.frame - self.last_send_frame >= HOLD_FRAME_GAP:
        off = self._send(d, hold=True)
        if off is not None:
          self.last_action = "hold"
          return d, off
      return None, None

    # One unconfirmed press at a time, a bounded number in flight, nothing while a hold settles;
    # also wait during a back-off.
    if (self.pending is not None or len(self.inflight) >= MAX_IN_FLIGHT or self.frame < self.backoff_until or
        any(p.hold for p in self.inflight)):
      return None, None

    want_lower = False
    room = 0
    if reason is not None and not self.blocked:
      # How many more steps down this wants (re-lowering while restoring needs 2 steps).
      room = int((set_speed - target + 1e-3) // self.step)
      budget_ok = (self.lower_presses < self.max_lower_presses) if reason == "curve" else (self.stop_presses < MAX_STOP_PRESSES)
      want_lower = budget_ok and room >= (2 if self.restoring else 1)

    button = None
    if want_lower:
      self.restoring = False
      button = SET_MINUS
    elif offset != 0 and (all_over or self.restore_pending) and \
         (self.frame - self.last_minus_frame >= _frames(MIN_DWELL) or self.restore_pending):
      self.restoring = True
      button = SET_PLUS if offset > 0 else SET_MINUS
    elif offset == 0:
      self.restoring = False
      self.stop_episode = False
      self.holds_this_episode = 0

    if button is None or i.gas_pressed or i.other_button_active:
      return None, None
    if any(p.direction != button for p in self.inflight):
      # Never reverse direction while a press may still land.
      return None, None
    urgent = want_lower and reason == "stop" and i.stop_urgent
    gap = RELAXED_GAP_FRAMES if (want_lower and reason == "stop" and not urgent) else BUTTON_GAP_FRAMES
    if self.frame - self.last_send_frame <= gap:
      return None, None

    # Long presses. Lowering for a confirmed stopped car: the first hold aims HOLD_GOALS_MPH[0] down,
    # the second HOLD_GOALS_MPH[1] more, bounded by max(target, 20 mph floor); then taps. Restoring
    # after a stopped car: SET+ holds bounded by the driver's speed, then taps. A hold only starts
    # when its first quantum plus what may still land after the release stays inside the bound.
    q = self._quantum_steps()
    worst = (q + self._hold_reserve()) * self.step      # tap + first quantum + pending quantum + measured overrun
    hold = False
    if self.holds_allowed and not self.inflight:
      if want_lower and reason == "stop" and self.holds_this_episode < len(HOLD_GOALS_MPH):
        goal = HOLD_GOALS_MPH[self.holds_this_episode] * (1.609344 if self.step < MS_PER_MPH else 1.0) * self.step
        hard = max(target, STOP_MIN_SET_SPEED)
        hold = set_speed - worst >= hard - self.half_step
        if hold:
          self.hold_bound = max(hard, set_speed - goal - self.step)   # the goal (plus the button-down tap) shapes the release
          self.holds_this_episode += 1
      elif button == SET_PLUS and self.stop_episode and not want_lower:
        # upwards the bound is the driver's speed itself: a tap, the first quantum and the quanta that may
        # follow the release (3 until a hold has measured the car, then the worst seen plus one) must fit
        worst_up = (q + self._up_overrun() * q + 1) * self.step
        hold = set_speed + worst_up <= driver_speed + self.half_step
        if hold:
          self.hold_bound = driver_speed
    if want_lower and reason == "stop":
      self.stop_episode = True
    off = self._send(button, hold=hold)
    if off is None:
      return None, None
    if button == SET_MINUS and want_lower:
      if reason == "curve":
        self.lower_presses += 1
      else:
        self.stop_presses += 1
      self.last_minus_frame = self.frame
    self.last_action = "hold" if hold else ("tap" if want_lower else "restore")
    return button, off


class CurveSlowdown:
  """Card-side wrapper: reads frogpilotPlan and radarState, runs the stopped car trigger and the
  shared set speed controller, builds the CAN messages and writes the per-event logs."""
  EVENT_COLUMNS = ["t_wall", "vEgo_mph", "aEgo", "brake", "gas", "cruise", "op", "set_mph", "csc_active", "csc_mph",
                   "lead_status", "lead_prob", "lead_d", "lead_v", "lead_y", "radar_has_lead", "radar_dist_bucket",
                   "stop_active", "stop_target_mph", "stop_urgent", "stop_required", "stop_ttc", "offset", "in_flight",
                   "hold", "press", "driver_btn", "blocked", "restoring"]

  def __init__(self, CP, params, curve_enabled=True, stop_mode=0):
    import cereal.messaging as messaging
    try:
      choice = int(params.get("MazdaCurveSlowdownMaxDrop", encoding="utf8") or 1)
    except (TypeError, ValueError):
      choice = 1
    max_drop = MAX_DROP_CHOICES_MPH[choice] if 0 <= choice < len(MAX_DROP_CHOICES_MPH) else DEFAULT_MAX_DROP_MPH
    self.logic = CurveSlowdownLogic(params.get_bool("IsMetric"), max_drop)
    self.curve_enabled = curve_enabled
    self.stop_mode = stop_mode
    self.sm = messaging.SubMaster(['frogpilotPlan', 'radarState'])
    from openpilot.selfdrive.car.mazda.stopped_car_slowdown import StoppedCarTrigger
    self.trigger = StoppedCarTrigger()
    self.frame = 0
    self.recorder = None
    self.curve_event = None
    self.stop_event = None
    self.stop_presses_at_start = 0
    self.near_miss = None
    try:
      from openpilot.frogpilot.common.feature_events import EventRecorder, NearMissTracker
      self.recorder = EventRecorder(self.EVENT_COLUMNS, hz=20, params=params)
      self.near_miss = NearMissTracker(self.recorder, "stoppedCarSlowdown")
    except Exception:
      self.recorder = None

  def _alive(self, service):
    return bool(self.sm.alive[service] and self.sm.recv_frame[service] > 0)

  def update(self, CC, CS, packer, CP, other_button_active):
    from openpilot.selfdrive.car.mazda import mazdacan
    from openpilot.selfdrive.car.mazda.values import Buttons
    from openpilot.selfdrive.car.mazda.stopped_car_slowdown import StoppedCarInputs, TEST, ON

    self.frame += 1
    self.sm.update(0)
    plan = self.sm['frogpilotPlan']
    lead = self.sm['radarState'].leadOne
    cruise_enabled = bool(CS.out.cruiseState.enabled)
    v_ego = float(CS.out.vEgo)

    # Stopped car ahead (Test: trigger and target are computed and logged, nothing is pressed)
    stop_inputs = StoppedCarInputs(
      enabled=bool(CC.enabled), cruise_enabled=cruise_enabled, brake_pressed=bool(CS.out.brakePressed), v_ego=v_ego,
      lead_alive=self._alive('radarState'), lead_status=bool(lead.status), lead_prob=float(lead.modelProb),
      lead_d=float(lead.dRel), lead_v=float(lead.vLead), lead_y=float(lead.yRel),
      radar_has_lead=bool(getattr(CS, "radar_has_lead", False)), radar_lead_distance=int(getattr(CS, "radar_lead_distance", 0)))
    stop_active = self.trigger.update(stop_inputs) if self.stop_mode in (TEST, ON) else False

    inputs = SlowdownInputs(
      enabled=bool(CC.enabled),
      cruise_enabled=cruise_enabled,
      set_speed=float(CS.out.cruiseState.speed),
      v_ego=v_ego,
      gas_pressed=bool(CS.out.gasPressed),
      brake_pressed=bool(CS.out.brakePressed),
      standstill=bool(CS.out.standstill),
      plan_alive=self._alive('frogpilotPlan'),
      csc_active=bool(plan.cscActive) and self.curve_enabled,
      csc_speed=float(plan.cscSpeed),
      other_button_active=other_button_active,
      driver_button=SET_MINUS if CS.crz_set_minus else (SET_PLUS if CS.crz_set_plus else None),
      stop_active=stop_active and self.stop_mode == ON,
      stop_target=float(self.trigger.target),
      stop_urgent=bool(self.trigger.urgent),
      stop_restore_now=bool(self.trigger.restore_now) and self.stop_mode == ON,
    )
    button, offset = self.logic.update(inputs)
    try:
      self._log(inputs, stop_inputs, stop_active, button, float(CS.out.aEgo))
    except Exception:
      pass
    if button is None:
      return []
    btn = Buttons.SET_MINUS if button == SET_MINUS else Buttons.SET_PLUS
    return [mazdacan.create_button_cmd(packer, CP, CS.crz_btns_counter + offset, btn)]

  # ---- per-event logs (never raises into the car controller: called inside try/except) ----
  def _log(self, i, si, stop_active, button, a_ego):
    if self.recorder is None:
      return
    import time
    t = time.monotonic()
    lg = self.logic
    if button is not None or self.frame % 5 == 0:
      self.recorder.add_row(t, (
        f"{time.time():.2f}", i.v_ego / MS_PER_MPH, a_ego, i.brake_pressed, i.gas_pressed, i.cruise_enabled, i.enabled,
        i.set_speed / MS_PER_MPH, i.csc_active, i.csc_speed / MS_PER_MPH,
        si.lead_status, si.lead_prob, si.lead_d, si.lead_v, si.lead_y, si.radar_has_lead, si.radar_lead_distance,
        stop_active, self.trigger.target / MS_PER_MPH,
        self.trigger.urgent, self.trigger.required, min(self.trigger.ttc, 99.0), lg.offset, len(lg.inflight),
        lg.hold is not None, {SET_MINUS: "M", SET_PLUS: "P"}.get(button, ""), i.driver_button or "", lg.blocked, lg.restoring))
    # curve slowdown episode: from the first SET- for a curve to the restore being complete
    if self.curve_event is None and button == SET_MINUS and lg.last_action in ("tap", "hold") and not stop_active and i.csc_active:
      self.curve_event = self.recorder.open("curveSlowdown", t, {
        "mode": "on", "set_mph_before": i.set_speed / MS_PER_MPH, "vEgo_mph": i.v_ego / MS_PER_MPH,
        "csc_mph": i.csc_speed / MS_PER_MPH, "max_drop_steps": lg.max_drop_steps})
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
        "mode": "test" if self.stop_mode == 1 else "on", "set_mph_before": i.set_speed / MS_PER_MPH,
        "vEgo_mph": i.v_ego / MS_PER_MPH, "lead_d": si.lead_d, "lead_v": si.lead_v, "lead_prob": si.lead_prob,
        "lead_y": si.lead_y, "target_mph": self.trigger.target / MS_PER_MPH, "required_decel": self.trigger.required,
        "ttc": min(self.trigger.ttc, 99.0), "urgent": self.trigger.urgent})
      self.stop_presses_at_start = lg.stop_presses
      self.stop_holds_at_start = len(lg.hold_log)
      self.stop_min_set = i.set_speed
    if self.stop_event is not None:
      self.stop_min_set = min(self.stop_min_set, i.set_speed)
      done = not stop_active and lg.offset == 0 and not lg.inflight
      if done or (not stop_active and self.stop_mode == 1):
        holds = lg.hold_log[self.stop_holds_at_start:]
        self.recorder.close(self.stop_event, t, {
          "release_reason": self.trigger.release_reason, "set_mph_min": self.stop_min_set / MS_PER_MPH,
          "set_mph_after": i.set_speed / MS_PER_MPH, "holds": len(holds),
          "holds_detail": "; ".join(f"frames_sent {h.get('frames_sent')} landed {h.get('landed')} after_release {h.get('landed_after_release', 0)} "
                                    f"landings {[(round((f - h['frame']) * DT_CTRL, 2), n) for f, n in h.get('landings', [])]}"
                                    for h in holds),
          "hold_overrun_learned": lg.hold_overrun, "holds_allowed": lg.holds_allowed, "disabled": lg.disabled,
          "radar_lead_rises_s": [round((f - self.trigger.start_frame) * DT_CTRL, 2) for f in self.trigger.radar_rises
                                 if f >= self.trigger.start_frame - 1500]})
        self.stop_event = None
    if self.near_miss is not None and self.stop_mode in (1, 2):
      cand = self.trigger.near_miss_candidate(si)
      self.near_miss.update(t, cand, stop_active, self.trigger.blockers(si) if cand else (),
                            {"lead_d": si.lead_d, "lead_v": si.lead_v, "lead_prob": si.lead_prob, "lead_y": si.lead_y,
                             "vEgo_mph": i.v_ego / MS_PER_MPH})
    self.recorder.tick(t)
