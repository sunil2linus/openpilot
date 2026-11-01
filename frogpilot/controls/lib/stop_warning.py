"""
Stop warnings for cars where openpilot cannot brake (e.g. Gen1 Mazda on stock radar cruise).

Two independent warnings, each with its own setting (0 = off, 1 = test, 2 = on):
  * Stop ahead ("red light"): the driving model plans to slow down a lot soon (to at most
    STOP_SLOWDOWN of the current speed within STOP_HORIZON, the same rule FrogPilot's own
    stop light detection uses), the planned deceleration is significant, there is no relevant
    lead car, and the car is not slowing down. Note: this is the model's *plan* (red light,
    stop sign, sharp turn...), not a traffic light colour detector. At 45-50 mph the model's
    plan does not reach a full stop inside its horizon before the driver brakes (seen on the
    road), so a full-stop criterion would never fire; the slowdown fraction does.
  * Stopped car ahead: the lead car is stopped or nearly stopped, we are closing in fast,
    and the car is not slowing down.

Test mode never makes a sound: it only appends a line to LOG_PATH each time the warning would
have fired, so false alarms (e.g. at green lights) can be reviewed before turning sound on.

Rules for both warnings:
  * Urgent only: above a minimum speed, the required deceleration is significant, and the
    driver/stock cruise is not already slowing (brake not pressed, aEgo above -0.5 m/s^2).
  * The condition must hold for a short time (debounce) before firing.
  * At most one warning per event: after firing, the condition must be gone for REARM_TIME
    before it can fire again, and never more often than MIN_GAP for the same warning.
  * Only while the stock cruise is engaged (the situation where the car may not stop by itself).

StopWarningLogic has no openpilot dependencies so it can be unit tested.
"""
import datetime
import json
import os
from dataclasses import dataclass, field

DT = 0.05                       # planner runs at 20 Hz
MS_PER_MPH = 0.44704

OFF, TEST, ON = 0, 1, 2

# Stop ahead (model plans to slow down a lot / stop)
STOP_SLOWDOWN = 0.70            # planned minimum speed <= this fraction of the current speed (FrogPilot's SLOWDOWN_PERCENTAGE)
STOP_HORIZON = 8.0              # s of the plan that count (FrogPilot's STOP_TIME)
STOP_MIN_SPEED = 25 * MS_PER_MPH
STOP_DECEL = 0.9                # m/s^2: deceleration the plan implies to reach that speed where it is planned
STOP_BUFFER = 2.0               # m subtracted from the planned distance
STOP_DEBOUNCE = 1.0             # s
STOP_DROPOUT = 0.25             # s: the plan may miss the condition this long without restarting the debounce
STOP_HOLD_POINTS = 2            # the slowdown must hold for this many consecutive plan points (single dips are noise)

# Stopped car ahead
LEAD_MIN_SPEED = 20 * MS_PER_MPH
LEAD_STOPPED_SPEED = 2.0        # m/s: lead slower than this counts as stopped
LEAD_MIN_PROB = 0.8
LEAD_MAX_LATERAL = 1.2          # m: lead must be in our lane (|yRel|)
LEAD_MIN_CLOSING = 4.0          # m/s
LEAD_MAX_TTC = 4.0              # s
LEAD_DECEL = 1.5                # m/s^2: required deceleration to stop behind it
LEAD_BUFFER = 5.0               # m
LEAD_DEBOUNCE = 0.5             # s

NOT_SLOWING_ACCEL = -0.5        # m/s^2: slower than this means the car is already braking
ACCEL_FILTER_TAU = 0.5          # s: low-pass on aEgo so noise does not reset the debounce
REARM_TIME = 3.0                # s the condition must be gone before the same warning can fire again
MIN_GAP = 10.0                  # s between two warnings of the same kind

STOP_AHEAD = "stopAhead"
STOPPED_LEAD = "stoppedLead"

LOG_PATH = "/data/media/0/stop_warnings.csv"
LOG_MAX_BYTES = 1_000_000
LOG_HEADER = "time,kind,mode,speed_mph,a_ego,distance_m,required_decel,time_s,lead_speed_mph,lead_prob,lat,lon\n"


def _frames(seconds):
  return int(round(seconds / DT))


def _interp(x, xp, fp):
  """np.interp for short lists without numpy (keeps this module dependency free)."""
  if not xp:
    return 0.0
  if x <= xp[0]:
    return fp[0]
  for k in range(1, len(xp)):
    if x <= xp[k]:
      x0, x1 = xp[k - 1], xp[k]
      y0, y1 = fp[k - 1], fp[k]
      return y0 if x1 == x0 else y0 + (y1 - y0) * (x - x0) / (x1 - x0)
  return fp[-1]


@dataclass
class StopWarningInputs:
  cruise_enabled: bool
  v_ego: float
  a_ego: float
  brake_pressed: bool
  standstill: bool
  model_t: list = field(default_factory=list)   # modelV2.velocity.t
  model_v: list = field(default_factory=list)   # modelV2.velocity.x
  model_x: list = field(default_factory=list)   # modelV2.position.x (same time points)
  lead_status: bool = False
  lead_d: float = 0.0
  lead_v: float = 0.0
  lead_prob: float = 0.0
  lead_y: float = 0.0
  lead_relevant: bool = False


@dataclass
class Detection:
  kind: str
  distance: float
  required_decel: float
  time: float
  lead_speed: float = 0.0
  lead_prob: float = 0.0


class _Gate:
  """Debounce (tolerating short dropouts) + one-per-event latch + minimum gap for one warning kind."""
  def __init__(self, debounce, dropout=0.0):
    self.debounce_frames = _frames(debounce)
    self.dropout_frames = _frames(dropout)
    self.true_frames = 0
    self.false_frames = _frames(REARM_TIME)
    self.latched = False
    self.last_fire = -10_000

  def update(self, condition, frame):
    if condition:
      self.true_frames += 1
      self.false_frames = 0
    else:
      self.false_frames += 1
      if self.false_frames > self.dropout_frames:
        self.true_frames = 0
      if self.false_frames >= _frames(REARM_TIME):
        self.latched = False
    if (condition and not self.latched and self.true_frames >= self.debounce_frames and
        frame - self.last_fire >= _frames(MIN_GAP)):
      self.latched = True
      self.last_fire = frame
      return True
    return False


class StopWarningLogic:
  def __init__(self):
    self.frame = 0
    self.gates = {STOP_AHEAD: _Gate(STOP_DEBOUNCE, STOP_DROPOUT), STOPPED_LEAD: _Gate(LEAD_DEBOUNCE)}
    self.a_filtered = None

  def not_slowing(self, i):
    alpha = DT / (ACCEL_FILTER_TAU + DT)
    self.a_filtered = i.a_ego if self.a_filtered is None else self.a_filtered + alpha * (i.a_ego - self.a_filtered)
    return not i.brake_pressed and self.a_filtered > NOT_SLOWING_ACCEL

  def stop_ahead(self, i):
    if i.lead_relevant or i.v_ego < STOP_MIN_SPEED or not i.model_t or len(i.model_t) != len(i.model_v):
      return None
    # Slowest point of the plan within the horizon (FrogPilot's stop light rule: <= 70 % of our
    # speed), counting only points where the next point is slow too (a single dip is model noise).
    slow = STOP_SLOWDOWN * i.v_ego
    t_min, v_min = None, None
    for k, (t, v) in enumerate(zip(i.model_t, i.model_v)):
      if t > STOP_HORIZON:
        break
      held = i.model_v[k:k + STOP_HOLD_POINTS]
      if len(held) == STOP_HOLD_POINTS and all(x <= slow for x in held) and (v_min is None or v < v_min):
        t_min, v_min = t, v
    if t_min is None:
      return None
    xs = i.model_x if len(i.model_x) == len(i.model_t) else []
    d_min = _interp(t_min, i.model_t, xs) if xs else (i.v_ego + v_min) * t_min / 2
    required = (i.v_ego ** 2 - max(v_min, 0.0) ** 2) / (2 * max(d_min - STOP_BUFFER, 1.0))
    if required < STOP_DECEL:
      return None
    return Detection(STOP_AHEAD, d_min, required, t_min)

  def stopped_lead(self, i):
    if not i.lead_status or i.lead_prob < LEAD_MIN_PROB or i.v_ego < LEAD_MIN_SPEED:
      return None
    if i.lead_v > LEAD_STOPPED_SPEED or i.lead_d <= 0 or abs(i.lead_y) > LEAD_MAX_LATERAL:
      return None
    closing = i.v_ego - i.lead_v
    if closing < LEAD_MIN_CLOSING:
      return None
    ttc = i.lead_d / closing
    required = closing ** 2 / (2 * max(i.lead_d - LEAD_BUFFER, 1.0))
    if ttc > LEAD_MAX_TTC or required < LEAD_DECEL:
      return None
    return Detection(STOPPED_LEAD, i.lead_d, required, ttc, i.lead_v, i.lead_prob)

  def update(self, i: StopWarningInputs, modes: dict):
    """Returns a list of (Detection, mode) that fired this frame (mode TEST or ON)."""
    self.frame += 1
    not_slowing = self.not_slowing(i)   # always update the filter
    active = i.cruise_enabled and not i.standstill and not_slowing
    fired = []
    for kind, detect in ((STOP_AHEAD, self.stop_ahead), (STOPPED_LEAD, self.stopped_lead)):
      mode = modes.get(kind, OFF)
      det = detect(i) if (active and mode != OFF) else None
      if self.gates[kind].update(det is not None, self.frame) and det is not None:
        fired.append((det, mode))
    return fired


def format_log_line(det, mode, v_ego, a_ego, gps=None, now=None):
  now = now or datetime.datetime.now()
  lat = f"{gps['latitude']:.6f}" if gps else ""
  lon = f"{gps['longitude']:.6f}" if gps else ""
  return (f"{now.isoformat(timespec='seconds')},{det.kind},{'test' if mode == TEST else 'on'},"
          f"{v_ego / MS_PER_MPH:.1f},{a_ego:.2f},{det.distance:.1f},{det.required_decel:.2f},{det.time:.1f},"
          f"{det.lead_speed / MS_PER_MPH:.1f},{det.lead_prob:.2f},{lat},{lon}\n")


def append_log(line, path=LOG_PATH):
  try:
    if os.path.exists(path) and os.path.getsize(path) > LOG_MAX_BYTES:
      os.replace(path, path + ".old")
    new = not os.path.exists(path)
    with open(path, "a") as f:
      if new:
        f.write(LOG_HEADER)
      f.write(line)
  except OSError:
    pass


class StopWarning:
  """Planner-side wrapper: reads settings, builds inputs from SubMaster data, logs, returns events to add."""
  PARAM_KEYS = {STOP_AHEAD: "StopWarningRedLight", STOPPED_LEAD: "StopWarningStoppedCar"}

  def __init__(self, params, params_memory):
    self.params = params
    self.params_memory = params_memory
    self.logic = StopWarningLogic()
    self.modes = {STOP_AHEAD: OFF, STOPPED_LEAD: OFF}
    self.read_counter = 0

  def _read_modes(self):
    for kind, key in self.PARAM_KEYS.items():
      try:
        value = int(self.params.get(key, encoding="utf8") or 0)
      except (TypeError, ValueError):
        value = OFF
      self.modes[kind] = value if value in (OFF, TEST, ON) else OFF

  def update(self, sm, frogpilot_planner):
    """Returns the list of warning kinds to alert on now (only those in ON mode)."""
    if self.read_counter % _frames(1.0) == 0:
      self._read_modes()
    self.read_counter += 1
    if self.modes[STOP_AHEAD] == OFF and self.modes[STOPPED_LEAD] == OFF:
      return []

    cs = sm["carState"]
    model = sm["modelV2"]
    lead = frogpilot_planner.lead_one
    inputs = StopWarningInputs(
      cruise_enabled=bool(cs.cruiseState.enabled),
      v_ego=float(cs.vEgo),
      a_ego=float(cs.aEgo),
      brake_pressed=bool(cs.brakePressed),
      standstill=bool(cs.standstill),
      model_t=list(model.velocity.t),
      model_v=list(model.velocity.x),
      model_x=list(model.position.x),
      lead_status=bool(lead.status),
      lead_d=float(lead.dRel),
      lead_v=float(lead.vLead),
      lead_prob=float(lead.modelProb),
      lead_y=float(lead.yRel),
      lead_relevant=bool(frogpilot_planner.lead_relevant),
    )
    alerts = []
    for det, mode in self.logic.update(inputs, self.modes):
      gps = None
      try:
        gps = json.loads(self.params_memory.get("LastGPSPosition") or "null")
      except (TypeError, ValueError):
        gps = None
      append_log(format_log_line(det, mode, inputs.v_ego, inputs.a_ego, gps))
      if mode == ON:
        alerts.append(det.kind)
    return alerts
