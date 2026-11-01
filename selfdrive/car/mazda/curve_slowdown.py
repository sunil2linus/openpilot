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
  * One press at a time. After a press nothing else is sent until the set speed moves
    one step in that direction, or a long timeout passes (and then a late arrival is
    still recognised before anything else is sent). A lost or slow press can only make
    the system do less, never more.
  * Any set speed change that is not exactly our pending step is the driver: we stop
    and leave the set speed where the driver put it.
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
from dataclasses import dataclass

DT_CTRL = 0.01                     # car controller runs at 100 Hz
MS_PER_MPH = 0.44704
MS_PER_KPH = 1 / 3.6

BUTTON_GAP_FRAMES = 20             # never send messages closer than 0.2 s
COUNTER_OFFSETS = [1, 1, 0, None]  # sunnypilot Mazda ICBM pattern

MAX_DROP_CHOICES_MPH = (5, 10, 15)    # "MazdaCurveSlowdownMaxDrop" param index 0/1/2
DEFAULT_MAX_DROP_MPH = 10
MIN_SET_SPEED = 25.0 * MS_PER_MPH     # never set cruise below 25 mph
START_SPEED = 25.0 * MS_PER_MPH       # only start a slowdown at 25 mph or more (true speed)
KEEP_SPEED = 18.0 * MS_PER_MPH        # keep managing (incl. restoring) down to 18 mph
CURVE_MARGIN = 0.3                    # m/s: stay this far above the curve target
CRUISE_SETTLE = 0.3                   # s: cruise must be engaged this long before we press
RESTORE_DELAY = 0.5                   # s of FrogPilot saying the curve no longer limits us before raising
FLICKER_RESTORE_DELAY = 2.0           # used instead while the curve flag has just flickered back on
FLICKER_WINDOW = 3.0                  # s: flag coming back on this soon after going off = flicker
MIN_DWELL = 1.0                       # s at least between the last SET- and a SET+
PENDING_TIMEOUT = 1.5                 # s to wait for a press to show on the set speed
LATE_WINDOW = 2.0                     # s after the timeout during which a late change is still ours
NO_EFFECT_LIMIT = 3                   # presses in a row with no effect before backing off
NO_EFFECT_BACKOFF = 10.0              # s to wait after that before trying again
NO_EFFECT_TRIPS = 3                   # back-offs before giving up (speed left alone)
PAUSE_ABANDON = 30.0                  # s paused before we forget the driver's speed
EXTRA_LOWER_PRESSES = 4               # SET- budget per curve = max drop steps + this

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

    self.offset = 0                  # steps we are below the driver's speed (negative: above)
    self.pending = None              # (direction, frame) of a press not yet seen on the set speed
    self.late = None                 # (direction, frame) of a press that timed out
    self.no_effect = 0
    self.no_effect_trips = 0
    self.backoff_until = 0
    self.lower_presses = 0           # SET- presses in the current curve
    self.last_minus_frame = -10_000
    self.restoring = False
    self.blocked = False             # driver changed the speed: no lowering until the curve is over
    self.pause_frames = 0
    self.cruise_gap = False          # cruise was off since the last observed set speed
    self.flicker = False             # curve flag came back on shortly after going off (this episode)
    self.prev_csc_active = False
    self.disabled = False            # set speed steps don't match our unit: off for this drive

  @property
  def managing(self):
    return self.offset != 0 or self.pending is not None or self.late is not None

  def _forget(self):
    """Leave the set speed where it is and stop managing it."""
    self.offset = 0
    self.restoring = False
    self.no_effect = 0
    self.no_effect_trips = 0
    self.pause_frames = 0

  def _apply_ours(self, direction):
    self.offset += 1 if direction == SET_MINUS else -1
    self.no_effect = 0
    self.no_effect_trips = 0

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
    if abs(steps - round(steps)) > 0.25:
      # The car's set speed steps don't match our unit (e.g. km/h setting with an mph dash).
      # We cannot tell our presses from the driver's: switch off for this drive.
      self._forget()
      self.disabled = True
      return
    one_step = round(steps) == 1
    if one_step and self.pending is not None and self.pending[0] == direction:
      self.pending = None
      self._apply_ours(direction)
    elif (one_step and self.late is not None and self.late[0] == direction and
          self.frame - self.late[1] <= _frames(PENDING_TIMEOUT + LATE_WINDOW)):
      self.late = None
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
    elif ((self.pending is not None and self.pending[0] == direction) or
          (self.late is not None and self.late[0] == direction)):
      # Several steps in the direction of our own press: the car may step more than once per
      # press (or the driver held the button at the same moment). We can't keep the limits
      # reliably, so switch off for this drive and leave the speed alone.
      self._forget()
      self.pending = None
      self.late = None
      self.disabled = True
    else:
      # Several steps at once (e.g. holding the button): the driver set a new speed.
      self._forget()
      self.blocked = True

  def update(self, i: SlowdownInputs):
    """Returns (button, counter_offset) to send this frame, or (None, None)."""
    self.frame += 1
    if self.disabled:
      return None, None
    cruise_ok = i.cruise_enabled and i.set_speed > 0
    self.cruise_on_frames = self.cruise_on_frames + 1 if cruise_ok else 0
    if not cruise_ok:
      if self.last_set_speed is not None:
        self.cruise_gap = True
      # Freeze the timers of a press in flight while cruise is off, so it is still
      # recognised as ours when cruise comes back (the set speed is not observed meanwhile).
      if self.pending is not None:
        self.pending = (self.pending[0], self.pending[1] + 1)
      if self.late is not None:
        self.late = (self.late[0], self.late[1] + 1)

    if cruise_ok:
      self._observe_set_speed(i)

    # A press that never showed up: count it, but remember it for a late arrival.
    if self.pending is not None and self.frame - self.pending[1] > _frames(PENDING_TIMEOUT):
      self.late = self.pending
      self.pending = None
      self.no_effect += 1
    if self.late is not None and self.frame - self.late[1] > _frames(PENDING_TIMEOUT + LATE_WINDOW):
      self.late = None
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
    if self.offset == 0 and self.pending is None and self.late is None:
      self.flicker = False
    restore_delay = FLICKER_RESTORE_DELAY if self.flicker else RESTORE_DELAY
    curve_over = self.curve_free_frames >= _frames(restore_delay)
    if curve_over:
      self.lower_presses = 0
      self.blocked = False

    speed_ok = i.v_ego >= (KEEP_SPEED if self.managing else START_SPEED)
    can_act = (i.enabled and cruise_ok and i.plan_alive and not i.brake_pressed and not i.standstill and
               speed_ok and self.cruise_on_frames >= _frames(CRUISE_SETTLE))
    if not can_act:
      # Time with cruise off does not count: on resume we compare the set speed instead.
      if self.offset != 0 and cruise_ok:
        self.pause_frames += 1
        if self.pause_frames >= _frames(PAUSE_ABANDON):
          self._forget()
      return None, None
    self.pause_frames = 0

    # One press at a time; also wait during a back-off.
    if self.pending is not None or self.late is not None or self.frame < self.backoff_until:
      return None, None

    set_speed = i.set_speed
    driver_speed = set_speed + self.offset * self.step
    floor = max(driver_speed - self.max_drop_steps * self.step, MIN_SET_SPEED)
    target = max(i.csc_speed + CURVE_MARGIN, floor)

    # How many more steps down this curve wants (re-lowering while restoring needs 2 steps).
    room = int((set_speed - target + 1e-3) // self.step)
    want_lower = (i.csc_active and not self.blocked and self.lower_presses < self.max_lower_presses and
                  room >= (2 if self.restoring else 1))

    button = None
    if want_lower:
      self.restoring = False
      button = SET_MINUS
    elif self.offset != 0 and curve_over and self.frame - self.last_minus_frame >= _frames(MIN_DWELL):
      self.restoring = True
      button = SET_PLUS if self.offset > 0 else SET_MINUS
    elif self.offset == 0:
      self.restoring = False

    if button is None or i.gas_pressed or i.other_button_active:
      return None, None
    if self.frame - self.last_send_frame <= BUTTON_GAP_FRAMES:
      return None, None

    offset = COUNTER_OFFSETS[self.button_frame % len(COUNTER_OFFSETS)]
    self.button_frame += 1
    if offset is None:
      return None, None

    self.last_send_frame = self.frame
    self.pending = (button, self.frame)
    if button == SET_MINUS and want_lower:
      self.lower_presses += 1
      self.last_minus_frame = self.frame
    return button, offset


class CurveSlowdown:
  """Thin wrapper that reads frogpilotPlan and builds the CAN message."""
  def __init__(self, CP, params):
    import cereal.messaging as messaging
    try:
      choice = int(params.get("MazdaCurveSlowdownMaxDrop", encoding="utf8") or 1)
    except (TypeError, ValueError):
      choice = 1
    max_drop = MAX_DROP_CHOICES_MPH[choice] if 0 <= choice < len(MAX_DROP_CHOICES_MPH) else DEFAULT_MAX_DROP_MPH
    self.logic = CurveSlowdownLogic(params.get_bool("IsMetric"), max_drop)
    self.sm = messaging.SubMaster(['frogpilotPlan'])

  def update(self, CC, CS, packer, CP, other_button_active):
    from openpilot.selfdrive.car.mazda import mazdacan
    from openpilot.selfdrive.car.mazda.values import Buttons

    self.sm.update(0)
    plan = self.sm['frogpilotPlan']
    inputs = SlowdownInputs(
      enabled=bool(CC.enabled),
      cruise_enabled=bool(CS.out.cruiseState.enabled),
      set_speed=float(CS.out.cruiseState.speed),
      v_ego=float(CS.out.vEgo),
      gas_pressed=bool(CS.out.gasPressed),
      brake_pressed=bool(CS.out.brakePressed),
      standstill=bool(CS.out.standstill),
      plan_alive=bool(self.sm.alive['frogpilotPlan'] and self.sm.recv_frame['frogpilotPlan'] > 0),
      csc_active=bool(plan.cscActive),
      csc_speed=float(plan.cscSpeed),
      other_button_active=other_button_active,
    )
    button, offset = self.logic.update(inputs)
    if button is None:
      return []
    btn = Buttons.SET_MINUS if button == SET_MINUS else Buttons.SET_PLUS
    return [mazdacan.create_button_cmd(packer, CP, CS.crz_btns_counter + offset, btn)]
