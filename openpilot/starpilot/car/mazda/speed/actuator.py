"""
The ONE MRCC button actuator: moves the stock cruise set speed toward the planner's command with the steering-wheel
cruise buttons (CRZ_BTNS SET+/SET-), and tells the planner which set speed changes were the driver's.

It never decides WHY: the planner (planner.py) gives a command (a set speed in dash units, or None) and the actuator
only presses toward it. Rules kept from the old fork's controller (curve_slowdown.py, drives 31-6f):

  * Taps: one frame per press, at least BUTTON_GAP apart, counter pattern COUNTER_OFFSETS (sunnypilot ICBM). One
    unconfirmed press at a time; a press stays "in flight" for its landing window (from the learned answer time) and
    the next one only goes if the set speed would still be on the right side of the command if every press in
    flight still lands. Never a press in the opposite direction while one is in flight.
  * Holds (5 mph steps): frames phase-locked to the wheel's own CRZ_BTNS frames (sent right after a new wheel frame,
    CTR = wheel + 1), released when the next quantum plus the measured overrun could pass the command. What lands
    after a release is measured (car_profile.py keeps the worst). A hold that changes nothing in 1 s is a miss;
    two misses switch holds off for HOLD_MISS_FORGET.
  * His buttons win: any driver SET+/SET-/RES/CANCEL, brake, gas, cruise off or openpilot's own CANCEL/RESUME
    pause pressing (and cut a hold). A set speed change near his button edge, or that is not explained by our presses,
    is HIS: reported to the planner as a driver change. Nothing is pressed while he is on a button or DRIVER_QUIET
    after it.
  * The press that engages is the engagement: SET+, SET- or RES from standby. The first frame with the cruise on
    reports "engaged" (another set speed than before, or the first) or "resumed" (the same). A set speed change within
    ENGAGE_SETTLE after that, with his button edge within DRIVER_RECENT before the cruise came on and no new one
    since, is the car still showing the speed that press set: reported as "engaged" too (changed), not his change.
  * Whole dash steps: the set speed is compared as round(set / step) (the dash reads 48.989 for 49).
  * The car JUMP_STEPS or more faster than the set: no SET+ (it sets the car's current speed); no SET- either while
    that is from his gas (the same jump; after our own lowering a SET- steps down as usual). A set change the other
    way right after a press of ours (in flight, or just expired) with none of his buttons is that press landing as a
    jump: ours, never his.
  * A press of ours that lands as a fraction of a step means the dash unit is not ours: off for the drive.
  * Ignored presses: NO_EFFECT_LIMIT in a row back off NO_EFFECT_BACKOFF; NO_EFFECT_TRIPS back-offs: off for the drive.
  * The car's maximum set speed is 90 mph / 145 km/h; it is only learned lower when the set speed reaches it, or
    after SET_MAX_MISSES spaced, unanswered single SET+ presses at the same set speed at or above SET_MAX_LEARN.
  * Below the car's own minimum set speed (30 km/h = 18.6 mph) nothing is pressed down.
  * (Stage 3) The one press with the cruise OFF: the gas auto-resume's RES (auto_resume.py decides when; the panda's
    window lets it through). Only with the cruise off, no openpilot CANCEL/RESUME frame and none of his buttons or the
    brake this frame. It is booked as his RES: the RES edge (so DRIVER_QUIET follows it, as after his RES) and the
    "driver_button" RES event the planner and the stop slowdown read; the cruise coming back is the usual "resumed".
Pure logic, no openpilot imports: unit tested offline (tests/test_actuator.py).
"""
from dataclasses import dataclass, field

DT = 0.01                     # card runs at 100 Hz
MPH = 0.44704
KPH = 1 / 3.6

SET_PLUS = "SET_PLUS"
SET_MINUS = "SET_MINUS"
RESUME = "RES"                # (Stage 3) only the gas auto-resume's RES with the cruise off

BUTTON_GAP = 20               # frames (0.2 s) between two of our frames, taps
COUNTER_OFFSETS = (1, 1, 0, None)   # tap frames: CTR = wheel CTR + 1 + offset; None = skip that slot
PENDING_TIMEOUT = 0.3         # s: an unconfirmed tap blocks the next one this long
LATE_WINDOW = 3.0             # s after the timeout during which a late landing is still ours (answer time unknown)
LATE_FACTOR = 4               # once the answer time is known: in flight for this many times it ...
MIN_IN_FLIGHT = 1.0           # s ... and never less than this
MAX_IN_FLIGHT = 3             # presses unaccounted for at most
LATENCY_KEEP = 9
DRIVER_RECENT = 1.5           # s after his button edge: changes in that direction are his
DRIVER_QUIET = 1.0            # s after his last button edge before we press again
CRUISE_SETTLE = 0.3           # s of cruise engaged before the first press
ENGAGE_SETTLE = 0.5           # s after the cruise came on: a set change without a new button edge is the engagement's
GAS_SETTLE = 0.5              # s after the gas is released before a press (a SET on the gas jumps to the current speed)
NO_EFFECT_LIMIT = 6
NO_EFFECT_BACKOFF = 10.0      # s
NO_EFFECT_TRIPS = 3

HOLD_ENABLED = True
HOLD_QUANTUM_MPH = 5          # the long-press step (km/h dash: 10, assumed until seen on a metric car)
HOLD_QUANTUM_KPH = 10
HOLD_MIN_QUANTA = 2           # a hold is used only when the command is at least this many quanta away
HOLD_FALLBACK_FRAMES = 12     # no new wheel frame this long after our last hold frame: send anyway
HOLD_NO_EFFECT = 1.0          # s: a hold with no landing by then is released and counted as a miss
HOLD_MAX = 6.0                # s
HOLD_SETTLE = 1.5             # s after a hold's last frame during which landings are still the hold's
HOLD_MISS_LIMIT = 2
HOLD_MISS_FORGET = 120.0      # s
HOLD_OVERRUN_DOWN = 0         # quanta that may land after a phase-locked SET- release (drives 44/45: none)
HOLD_OVERRUN_UP = 1           # ... after a SET+ release (kept at 1: a restore hold stops a quantum short)

JUMP_STEPS = 2                # the car this many steps faster than the set speed: a SET+ sets the current speed; so
                              # does a SET- while the car is above the set from HIS GAS (r3 rt 435: our SET- at 32 mph,
                              # set 28, his gas just released, made it 31). Above the set because WE lowered it, a SET-
                              # steps down as usual (every slowdown; drives 44/45).
CAR_MIN_SET = 30 * KPH        # the car's own minimum set speed (18.6 mph)
CAR_MAX_SET = (90 * MPH, 145 * KPH)
SET_MAX_LEARN = (88 * MPH, 142 * KPH)
SET_MAX_MISSES = 2
SET_MAX_MISS_GAP = 0.6        # s between two misses that count
SET_MAX_NEAR = (3 * MPH, 5 * KPH)


def frames(seconds):
  return int(round(seconds / DT))


@dataclass
class ActInputs:
  cruise_enabled: bool
  set_speed: float            # m/s, the stock set speed as the car reports it (dash units)
  v_ego: float = 0.0
  gas: bool = False
  brake: bool = False
  standstill: bool = False
  set_plus: bool = False      # the driver's buttons as the wheel's CRZ_BTNS shows them
  set_minus: bool = False
  resume: bool = False
  cancel: bool = False
  wheel_counter: int = -1     # CRZ_BTNS CTR of the wheel's last frame (-1 unknown)
  command: float | None = None   # m/s (dash units) the planner wants the set speed at, None: leave it
  allow_lower: bool = True
  other_button: bool = False  # openpilot sends CANCEL/RESUME this frame
  dry_run: bool = False       # test mode: decide, never press (nothing is booked as ours)
  auto_resume: bool = False   # (Stage 3) the gas auto-resume wants its RES frame now (cruise off)


@dataclass
class Press:
  direction: str
  frame: int
  remaining: int = 1
  landed: int = 0
  hold: bool = False
  release_frame: int = -1
  frames_sent: int = 0
  alone: bool = False
  after_release: int = 0      # hold: steps that landed after its release (HOLD_SETTLE)


@dataclass
class ActOutput:
  button: str | None = None   # SET_PLUS / SET_MINUS (or RESUME, Stage 3) to send this frame
  counter: int = 0            # the value for mazdacan.create_button_cmd's `counter` (it sends counter + 1)
  would_press: str | None = None   # dry run: what would have been pressed
  events: list = field(default_factory=list)


class Actuator:
  def __init__(self, is_metric=False, latencies=(), overrun_up=HOLD_OVERRUN_UP, overrun_down=HOLD_OVERRUN_DOWN):
    self.metric = bool(is_metric)
    self.step = KPH if self.metric else MPH
    self.quantum = HOLD_QUANTUM_KPH if self.metric else HOLD_QUANTUM_MPH
    self.frame = 0
    self.latencies = [int(x) for x in latencies if int(x) > 0][-LATENCY_KEEP:]
    self.overrun = {SET_PLUS: max(HOLD_OVERRUN_UP, int(overrun_up)), SET_MINUS: max(HOLD_OVERRUN_DOWN, int(overrun_down))}
    self.inflight: list[Press] = []
    self.expired: list[Press] = []
    self.hold: Press | None = None        # the hold whose frames are still going out
    self.settling: Press | None = None    # a released hold, HOLD_SETTLE
    self.holds_learned = []               # [(quanta after release, driver)] for the car profile
    self.hold_misses = 0
    self.hold_miss_frame = -10**9
    self.last_set = None                  # last set speed seen with cruise on
    self.cruise_gap = True                # cruise was off since last_set
    self.cruise_frames = 0
    self.last_send = -10**9
    self.tap_slot = 0
    self.prev_counter = -1
    self.wheel_frame = -10**9
    self.new_wheel = False
    self.edges = {SET_PLUS: -10**9, SET_MINUS: -10**9, "RES": -10**9, "CANCEL": -10**9}
    self.prev_buttons = (False, False, False, False)
    self.rise = -10**9                    # frame of his last button edge (a press starting)
    self.engage_frame = -10**9            # frame the cruise last came on
    self.gas_frame = -10**9
    self.gas_above = False                # the car JUMP_STEPS+ above the set since his gas (no SET-: it would jump)
    self.no_effect = 0
    self.trips = 0
    self.backoff_until = -1
    self.disabled = None                  # why, when off for the drive
    self.set_max = None                   # learned lower maximum (m/s)
    self.max_miss = None                  # (set m/s, misses, frame of the last)

  # ---------------- helpers ----------------
  def steps(self, speed):
    return int(round(speed / self.step))

  @property
  def latency(self):
    return sorted(self.latencies)[len(self.latencies) // 2] if self.latencies else None

  def window(self):
    lat = self.latency
    if lat is None:
      return frames(PENDING_TIMEOUT + LATE_WINDOW)
    return max(frames(MIN_IN_FLIGHT), LATE_FACTOR * lat)

  def top(self):
    top = CAR_MAX_SET[1] if self.metric else CAR_MAX_SET[0]
    return min(top, self.set_max) if self.set_max is not None else top

  @property
  def holds_allowed(self):
    if self.hold_misses and self.frame - self.hold_miss_frame > frames(HOLD_MISS_FORGET):
      self.hold_misses = 0
    return HOLD_ENABLED and self.hold_misses < HOLD_MISS_LIMIT

  def driver_recent(self, direction=None, seconds=DRIVER_RECENT):
    keys = (direction,) if direction else tuple(self.edges)
    return any(self.frame - self.edges[k] <= frames(seconds) for k in keys)

  def _event(self, out, kind, **kw):
    out.events.append({"kind": kind, "frame": self.frame, **kw})

  def _forget_presses(self):
    self.expired.extend(p for p in self.inflight if not p.hold)
    self.inflight = []
    self.hold = None
    self.settling = None

  # ---------------- what landed ----------------
  def _observe(self, i, out):
    if not i.cruise_enabled:
      if self.inflight or self.hold:
        self._forget_presses()
      self.cruise_gap = True
      self.cruise_frames = 0
      return
    self.cruise_frames += 1
    if self.last_set is None or self.cruise_gap:
      before, self.last_set = self.last_set, i.set_speed
      after_gap = self.cruise_gap
      self.cruise_gap = False
      self.engage_frame = self.frame
      if before is None or self.steps(before) != self.steps(i.set_speed):
        self._event(out, "engaged", before=before, after=i.set_speed, changed=before is not None)
      elif after_gap:
        self._event(out, "resumed", after=i.set_speed)
      return
    delta = i.set_speed - self.last_set
    if abs(delta) < 0.5 * self.step:
      return
    before, self.last_set = self.last_set, i.set_speed
    if (self.frame - self.engage_frame <= frames(ENGAGE_SETTLE) and
        self.engage_frame - frames(DRIVER_RECENT) <= self.rise <= self.engage_frame and
        not self.inflight and self.hold is None and self.settling is None):
      # the car still showing the speed his engaging press set (his press just before the cruise came on, no new
      # one since): the engagement
      self._event(out, "engaged", before=before, after=i.set_speed, changed=True, settle=True)
      return
    direction = SET_PLUS if delta > 0 else SET_MINUS
    raw_steps = abs(delta) / self.step
    n = int(round(raw_steps))
    ours = [p for p in self.inflight if p.direction == direction]
    held = (i.set_plus and direction == SET_PLUS) or (i.set_minus and direction == SET_MINUS)
    his = held or self.driver_recent(direction) or self.driver_recent("RES", 0.5) or self.driver_recent("CANCEL", 0.5)
    hold_open = any(p.hold for p in ours) or (self.settling is not None and self.settling.direction == direction)
    if not his and not ours and not hold_open:
      # a change the other way while a press of ours is in flight (or just expired) and none of his: the stock cruise
      # took the car's speed (JUMP_STEPS: SET- / SET+ with the car away from the set). That press's doing: OURS,
      # never his (r3 rt 435: booked his, it blocked the raise for a new limit).
      other = next((p for p in self.inflight if p.direction != direction and not p.hold), None)
      if other is not None:
        self.inflight.remove(other)
      else:
        other = next((p for p in self.expired if p.direction != direction), None)
        if other is not None:
          self.expired.remove(other)
      if other is not None:
        self.no_effect = 0
        self._event(out, "ours", direction=direction, steps=n if abs(raw_steps - n) <= 0.25 else round(raw_steps, 2),
                    jump=True, pressed=other.direction)
        return
    at_min = abs(i.set_speed - CAR_MIN_SET) < 0.5 * self.step or abs(before - CAR_MIN_SET) < 0.5 * self.step
    if abs(raw_steps - n) > 0.25:
      if at_min and (ours or hold_open) and not his:
        n = max(1, int(raw_steps + 0.75))       # a short step onto / off the car's 30 km/h minimum: still ours
      elif (ours or hold_open) and not his and raw_steps < 2.5:
        self.disabled = "set speed steps do not match the dash unit"
        self._event(out, "disabled", why=self.disabled)
        return
      else:
        self._event(out, "driver", direction=direction, before=before, after=i.set_speed, why="fraction")
        return
    if his and not hold_open:
      self._event(out, "driver", direction=direction, before=before, after=i.set_speed, why="button")
      return
    if ours or hold_open:
      self._consume(direction, n, out)
      return
    late = next((p for p in self.expired if p.direction == direction), None)
    if late is not None and n == 1:
      self.expired.remove(late)
      self.no_effect = 0
      self._event(out, "ours", direction=direction, steps=1, late=True)
      return
    self._event(out, "driver", direction=direction, before=before, after=i.set_speed, why="unexplained")

  def _consume(self, direction, n, out):
    taken = 0
    for p in list(self.inflight):
      if p.direction != direction or taken >= n:
        continue
      if p.hold:
        take = n - taken                   # a hold takes whatever comes its way
        if p.release_frame >= 0:
          p.after_release += take          # measured: what lands after a release
      else:
        take = min(n - taken, p.remaining)
        if len(self.inflight) == 1 and take and p.landed == 0 and p.remaining == 1:
          self.latencies = (self.latencies + [self.frame - p.frame])[-LATENCY_KEEP:]
        p.remaining -= take
      p.landed += take
      taken += take
      if not p.hold and p.remaining <= 0 and p in self.inflight:
        self.inflight.remove(p)
    self.no_effect = 0
    self.trips = 0 if taken else self.trips
    if taken < n:
      # more steps than our presses could make: we cannot keep the bounds. The set speed is his from here.
      self._event(out, "driver", direction=direction, before=None, after=self.last_set, why="more steps than ours")
      self._forget_presses()
      return
    self._event(out, "ours", direction=direction, steps=n)
    if self.max_miss is not None and direction == SET_PLUS:
      self.max_miss = None

  # ---------------- in flight bookkeeping ----------------
  def _expire(self, i, out):
    win = self.window()
    for p in list(self.inflight):
      if p.hold:
        continue
      if self.frame - p.frame >= win:
        self.inflight.remove(p)
        if p.landed == 0:
          self.expired.append(p)
          self._missed(p, i, out)
    self.expired = [p for p in self.expired if self.frame - p.frame < 2 * win]
    s = self.settling
    if s is not None and self.frame - max(s.release_frame, s.frame) > frames(HOLD_SETTLE):
      q_after = int(round(s.after_release / self.quantum)) if s.after_release >= self.quantum else 0
      if s.landed > 0 and s.frames_sent > 0:
        self.holds_learned.append((q_after, False))
        if q_after > self.overrun[s.direction]:
          self.overrun[s.direction] = q_after
          self._event(out, "hold_overrun", direction=s.direction, quanta=q_after)
      self.settling = None
      if s in self.inflight:
        self.inflight.remove(s)

  def _missed(self, p, i, out):
    top = self.top()
    if (p.direction == SET_PLUS and p.alone and self.last_set is not None and
        self.last_set >= (SET_MAX_LEARN[1] if self.metric else SET_MAX_LEARN[0]) - 1e-3):
      s, n, f = self.max_miss if self.max_miss is not None else (self.last_set, 0, -10**9)
      if abs(s - self.last_set) < 0.5 * self.step and self.frame - f >= frames(SET_MAX_MISS_GAP):
        n += 1
        self.max_miss = (s, n, self.frame)
      elif abs(s - self.last_set) >= 0.5 * self.step:
        self.max_miss = (self.last_set, 1, self.frame)
      if self.max_miss[1] >= SET_MAX_MISSES and self.last_set < top:
        self.set_max = self.last_set
        self._event(out, "set_max", speed=self.last_set)
      return
    self.no_effect += 1
    if self.no_effect >= NO_EFFECT_LIMIT:
      self.no_effect = 0
      self.trips += 1
      self.backoff_until = self.frame + frames(NO_EFFECT_BACKOFF)
      self._event(out, "backoff", trips=self.trips)
      if self.trips >= NO_EFFECT_TRIPS:
        self.disabled = "the car ignores our presses"
        self._event(out, "disabled", why=self.disabled)

  # ---------------- the hold ----------------
  def _hold_bound_ok(self, cur, cmd, direction):
    """True while the next quantum plus the overrun can still land without passing the command."""
    q = self.quantum
    margin = (1 + self.overrun[direction]) * q
    return cur - margin >= cmd if direction == SET_MINUS else cur + margin <= cmd

  def _stop_hold(self, out, why, miss=False):
    h = self.hold
    self.hold = None
    if h is None:
      return
    h.release_frame = self.frame
    if miss:
      self.hold_misses += 1
      self.hold_miss_frame = self.frame
      self._event(out, "hold_miss", why=why)
    self.settling = h
    self._event(out, "hold_released", why=why, landed=h.landed, frames=h.frames_sent)

  def _hold_step(self, i, out, cur, cmd):
    h = self.hold
    d = h.direction
    pause = self._pause_reason(i)
    if pause:
      self._stop_hold(out, pause)
      return None
    if cmd is None or (d == SET_MINUS and (cur <= cmd or not i.allow_lower)) or (d == SET_PLUS and cur >= cmd):
      self._stop_hold(out, "command reached")
      return None
    if h.landed == 0 and self.frame - h.frame >= frames(HOLD_NO_EFFECT):
      self._stop_hold(out, "no effect", miss=True)
      return None
    if self.frame - h.frame >= frames(HOLD_MAX) or not self._hold_bound_ok(self.steps(i.set_speed), cmd, d):
      self._stop_hold(out, "bound")
      return None
    due = self.new_wheel
    ref = self.last_send if h.frames_sent else h.frame
    if not due and self.frame - ref >= HOLD_FALLBACK_FRAMES and self.frame - self.wheel_frame >= HOLD_FALLBACK_FRAMES:
      due = True
    if not due:
      return None
    h.frames_sent += 1
    self.last_send = self.frame
    return d, max(i.wheel_counter, 0)

  # ---------------- the decision ----------------
  def _pause_reason(self, i):
    if not i.cruise_enabled:
      return "cruise off"
    if i.brake:
      return "brake"
    if i.gas or self.frame - self.gas_frame < frames(GAS_SETTLE):
      return "gas"
    if i.other_button:
      return "openpilot button"
    if i.set_plus or i.set_minus or i.resume or i.cancel:
      return "driver button"
    if self.driver_recent(seconds=DRIVER_QUIET):
      return "driver recent"
    if i.standstill:
      return "standstill"
    return None

  def _auto_resume(self, i, out):
    """(Stage 3) the gas auto-resume's RES frame, the one press with the cruise off (see the module notes)."""
    if i.cruise_enabled or i.other_button or i.brake or i.set_plus or i.set_minus or i.resume or i.cancel or i.dry_run:
      self._event(out, "auto_res_refused")
      return out
    self.edges["RES"] = self.frame
    self._event(out, "driver_button", button="RES", auto=True)
    out.button, out.counter = RESUME, max(i.wheel_counter, 0)
    return out

  def update(self, i: ActInputs) -> ActOutput:
    out = ActOutput()
    self.frame += 1
    self.new_wheel = i.wheel_counter >= 0 and i.wheel_counter != self.prev_counter
    if self.new_wheel:
      self.wheel_frame = self.frame
    self.prev_counter = i.wheel_counter
    buttons = (i.set_plus, i.set_minus, i.resume, i.cancel)
    for name, now, before in zip((SET_PLUS, SET_MINUS, "RES", "CANCEL"), buttons, self.prev_buttons, strict=True):
      if now:
        self.edges[name] = self.frame      # held counts as recent all along
      if now and not before:
        self.rise = self.frame
        self._event(out, "driver_button", button=name)
    self.prev_buttons = buttons
    if i.gas:
      self.gas_frame = self.frame
    near = i.set_speed > 0 and self.steps(i.v_ego) < self.steps(i.set_speed) + JUMP_STEPS
    if not i.cruise_enabled or near:
      self.gas_above = False
    elif i.gas:
      self.gas_above = True              # his gas has the car above the set (until it is back near it)
    self._observe(i, out)
    if self.hold is not None and (i.set_plus or i.set_minus or i.resume or i.cancel or i.brake or not i.cruise_enabled):
      self._stop_hold(out, "driver")
    self._expire(i, out)
    if i.auto_resume:
      return self._auto_resume(i, out)
    if self.disabled:
      return out
    if self.last_set is None or not i.cruise_enabled:
      return out
    if any(e["kind"] in ("driver", "engaged", "disabled") for e in out.events):
      return out                           # the planner hears about his change first (next frame)
    cur = self.steps(i.set_speed)
    cmd = None if i.command is None else self.steps(min(i.command, self.top()))
    if cmd is not None and cmd < self.steps(CAR_MIN_SET):
      cmd = self.steps(CAR_MIN_SET)
    if self.hold is not None:
      r = self._hold_step(i, out, cur, cmd)
      if r is None:
        return out
      out.button, out.counter = r
      return out
    if cmd is None or self._pause_reason(i) or self.cruise_frames < frames(CRUISE_SETTLE):
      return out
    if self.frame < self.backoff_until or self.frame - self.last_send < BUTTON_GAP:
      return out
    if self.settling is not None:
      return out
    projected = cur + sum(p.remaining for p in self.inflight if p.direction == SET_PLUS) \
                    - sum(p.remaining for p in self.inflight if p.direction == SET_MINUS)
    if cmd < projected and i.allow_lower:
      if self.gas_above:
        return out                         # above the set from his gas: a SET- would jump UP to the car's speed. The
                                           # cruise brings it back to the set; the SET- goes once it is near
      direction = SET_MINUS
    elif cmd > projected and cur < self.steps(self.top()) and self.steps(i.v_ego) < cur + JUMP_STEPS:
      direction = SET_PLUS                 # (not while the car is faster than the set: a SET+ would jump to its speed)
    else:
      return out
    if any(p.direction != direction for p in self.inflight):
      return out                           # never against a press in flight
    if len(self.inflight) >= MAX_IN_FLIGHT:
      return out
    near = SET_MAX_NEAR[1] if self.metric else SET_MAX_NEAR[0]
    if direction == SET_PLUS and self.inflight and i.set_speed >= self.top() - near - 1e-3:
      return out                           # near the top: one SET+ at a time (a miss there may be the car's max)
    pending = [p for p in self.inflight if p.landed == 0 and self.frame - p.frame < frames(PENDING_TIMEOUT)]
    if pending:
      return out
    if i.dry_run:
      out.would_press = direction
      return out
    gap_quanta = abs(cmd - cur) / self.quantum
    if (not self.inflight and self.holds_allowed and gap_quanta >= HOLD_MIN_QUANTA and
        self._hold_bound_ok(cur, cmd, direction) and i.wheel_counter >= 0):
      self.hold = Press(direction, self.frame, remaining=10**6, hold=True)
      self.inflight.append(self.hold)
      self._event(out, "hold_start", direction=direction, cur=cur, cmd=cmd)
      if not self.new_wheel:
        return out                         # its first frame goes with the next wheel frame
      self.hold.frames_sent = 1
      self.last_send = self.frame
      out.button, out.counter = direction, i.wheel_counter
      return out
    offset = COUNTER_OFFSETS[self.tap_slot % len(COUNTER_OFFSETS)]
    self.tap_slot += 1
    if offset is None:
      return out
    p = Press(direction, self.frame, alone=not self.inflight)
    p.frames_sent = 1
    self.inflight.append(p)
    self.last_send = self.frame
    out.button = direction
    out.counter = (max(i.wheel_counter, 0) + offset) % 16
    self._event(out, "tap", direction=direction, cur=cur, cmd=cmd)
    return out
