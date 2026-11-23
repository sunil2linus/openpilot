"""A small model of the Gen1 MRCC set speed and its buttons, for the actuator tests (not a drive simulation).

  * The wheel sends CRZ_BTNS at 10 Hz with its counter; our frames are separate messages.
  * A tap (one frame) moves the set speed one dash step after `latency` frames; `ignore` drops every n-th tap.
  * A long press: our frames every wheel frame (within `lock_ms` after it) -> one step at once, after 0.5 s the next
    multiple of 5, then 5 more every 0.6 s while the frames keep coming; `overrun` more quanta land after release.
  * Set speed limits: the car's minimum 30 km/h (18.64 mph on an mph dash), maximum `max_mph`.
  * `jump`: the car is above the set from his gas: a tap landing with the car JUMP_STEPS or more faster than the set
    (SET+ or SET-) sets the car's current speed (whole dash steps, rounded down: r3 rt 435, 31.7 mph -> 31).
"""
from openpilot.starpilot.car.mazda.speed import actuator as A

MPH = A.MPH


class Mrcc:
  def __init__(self, set_mph=45, latency=12, ignore=0, overrun=0, max_mph=90, lock_ms=40, holds_work=True,
               metric_dash=False, v_mph=None, jump=False):
    self.jump = jump
    self.metric_dash = metric_dash
    self.unit = A.KPH if metric_dash else MPH
    self.set = set_mph * self.unit          # m/s as the car reports it
    self.latency, self.ignore, self.overrun = latency, ignore, overrun
    self.max = max_mph * self.unit
    self.lock_ms, self.holds_work = lock_ms, holds_work
    self.frame = 0
    self.counter = 0
    self.pending = []                       # (frame due, steps)
    self.taps = 0
    self.hold_frames = []                   # frames of our phase-locked hold frames
    self.hold_on = False
    self.hold_start = None
    self.next_q = None
    self.overrun_left = 0
    self.sent = []                          # (frame, button, counter)
    self.v = (v_mph if v_mph is not None else set_mph - 2) * MPH
    self.track = v_mph is None             # the car follows its set speed (a fixed v_mph stays)

  @property
  def set_units(self):
    return self.set / self.unit

  def _move(self, steps):
    new = self.set + steps * self.unit
    if steps < 0 and new < A.CAR_MIN_SET - 1e-6:
      new = A.CAR_MIN_SET
    elif steps > 0 and self.set < A.CAR_MIN_SET + 0.5 * self.unit and self.set_units % 1:
      new = round(self.set_units + 0.5) * self.unit   # off the 18.64 minimum: the next whole step
    self.set = min(self.max, new)

  def receive(self, button, counter):
    self.sent.append((self.frame, button, counter, self.counter))
    sign = 1 if button == A.SET_PLUS else -1
    since_wheel = (self.frame % 10) * 10    # ms after the last wheel frame
    locked = self.holds_work and since_wheel <= self.lock_ms and counter == self.counter
    if locked and self.hold_frames and self.frame - self.hold_frames[-1] <= 15:
      # a second phase-locked frame right after the first (CTR = wheel + 1): a long press
      if not self.hold_on:
        self.hold_on, self.hold_start, self.hold_sign = True, self.frame, sign
        self.next_q = self.frame + 40
      self.hold_frames.append(self.frame)
      return
    if locked:
      self.hold_frames.append(self.frame)   # the first frame steps one at once, like a tap
    self.taps += 1
    if self.ignore and self.taps % self.ignore == 0:
      return
    self.pending.append((self.frame + self.latency, sign))

  def tick(self):
    self.frame += 1
    if self.track:
      want = self.set - (0 if self.metric_dash else 2 * MPH)
      self.v += max(-0.5 * 0.01, min(0.5 * 0.01, want - self.v))
    if self.frame % 10 == 0:
      self.counter = (self.counter + 1) % 16
    if self.hold_on:
      if self.frame - self.hold_frames[-1] > 15:      # frames stopped: released
        self.hold_on = False
        self.overrun_left = self.overrun
        self.next_q = self.frame + 30 if self.overrun else None
      elif self.next_q is not None and self.frame >= self.next_q:
        self._to_quantum(self.hold_sign)
        self.next_q = self.frame + 60
    elif self.overrun_left and self.next_q is not None and self.frame >= self.next_q:
      self._to_quantum(self.hold_sign)
      self.overrun_left -= 1
      self.next_q = self.frame + 60 if self.overrun_left else None
    for due, steps in list(self.pending):
      if self.frame >= due:
        self.pending.remove((due, steps))
        if self.jump and int(self.v / self.unit) >= round(self.set_units) + A.JUMP_STEPS:
          self.set = int(self.v / self.unit) * self.unit     # the jump: the car's current speed when it lands
        else:
          self._move(steps)

  def _to_quantum(self, sign):
    q = 10 if self.metric_dash else 5
    u = round(self.set_units)
    target = (u // q) * q if sign < 0 else (u // q + 1) * q
    if sign < 0 and target == u:
      target -= q
    self._move(target - u)


def run(act, car, frames, command=None, cruise=True, every=None, **kw):
  """Run `frames` frames. command: a number (mph/kph dash units), a function of the frame, or None.
  kw: extra ActInputs fields (constant) ; every(frame, inputs_dict) may change them per frame."""
  outs = []
  for _ in range(frames):
    car.tick()
    cmd = command(car.frame) if callable(command) else command
    fields = {"cruise_enabled": cruise, "set_speed": car.set, "v_ego": car.v, "wheel_counter": car.counter,
                  "command": None if cmd is None else cmd * car.unit}
    fields.update(kw)
    if every is not None:
      every(car.frame, fields)
    out = act.update(A.ActInputs(**fields))
    if out.button is not None:
      car.receive(out.button, out.counter)
    outs.append(out)
  return outs


def events(outs, kind=None):
  return [e for o in outs for e in o.events if kind is None or e["kind"] == kind]
