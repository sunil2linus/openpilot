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
LANE_CHANGE_HOLD = 2.0          # s after a lane change with no stop ahead warning: the model's plan dips while the
                                # car changes lanes (drive 3b: 2 of 6 warnings right after a lane change began)

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

NOT_SLOWING_ACCEL = -0.5        # m/s^2: slower than this means the car is already braking ...
# ... but not when it is our own stop slowdown (Mazda: the set speed lowered for this stop, starpilotCarState.
# mazdaStopSlowdownActive) that slows the car: it starts before the warning (stopped_car_slowdown.WARN_START_DEBOUNCE)
# and the warning must still show after it (owner: "first start reducing the speed and then display the warning").
# A lowered set speed slows the car ~0.9-1.6 m/s^2; harder than OWN_SLOWDOWN_ACCEL is the radar cruise braking for a
# car it tracks (up to ~3.8) or the driver: already handled.
OWN_SLOWDOWN_ACCEL = -2.0       # m/s^2
ACCEL_FILTER_TAU = 0.5          # s: low-pass on aEgo so noise does not reset the debounce
REARM_TIME = 3.0                # s the condition must be gone before the same warning can fire again
MIN_GAP = 10.0                  # s between two warnings of the same kind

STOP_AHEAD = "stopAhead"
STOPPED_LEAD = "stoppedLead"

LOG_PATH = "/data/media/0/stop_warnings.csv"
LOG_MAX_BYTES = 1_000_000
LOG_HEADER = ("time,kind,mode,speed_mph,a_ego,distance_m,required_decel,time_s,lead_speed_mph,lead_prob,"
              "lead_status,lead_d_m,lead_y_m,lead_relevant,lat,lon\n")


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
  lane_change: bool = False     # modelV2.meta.laneChangeState is laneChangeStarting / laneChangeFinishing
  own_slowdown: bool = False    # starpilotCarState.mazdaStopSlowdownActive


@dataclass
class Detection:
  kind: str
  distance: float
  required_decel: float
  time: float
  lead_speed: float = 0.0
  lead_prob: float = 0.0
  # The lead the planner saw at the time, for both kinds (lets lead-following false alarms be told apart)
  lead_status: bool = False
  lead_d: float = 0.0
  lead_y: float = 0.0
  lead_relevant: bool = False


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


# FrogPilot's lead relevance (frogpilot_planner.is_lead_relevant), for planners without it (StarPilot)
_STOP_DISTANCE = 6.0
_LEAD_STOP_MARGIN = 4.0
_MAX_T_FOLLOW = 3.0
_PLANNER_TIME = 10.0
_CRUISING_SPEED = 5.0


def lead_is_relevant(lead_status, lead_d, lead_v, lead_a, standstill, v_ego, model_length):
  following_speed = max(v_ego, _CRUISING_SPEED)
  lead_speed = lead_v + min(lead_a, 0.0) * _PLANNER_TIME / 2
  return bool(lead_status and
              lead_d < (0.0 if standstill else model_length) + _STOP_DISTANCE + _LEAD_STOP_MARGIN and
              lead_d < _STOP_DISTANCE + following_speed * _MAX_T_FOLLOW + max(following_speed - lead_speed, 0.0) * _PLANNER_TIME)


class StopWarningLogic:
  def __init__(self):
    self.frame = 0
    self.gates = {STOP_AHEAD: _Gate(STOP_DEBOUNCE, STOP_DROPOUT), STOPPED_LEAD: _Gate(LEAD_DEBOUNCE)}
    self.a_filtered = None
    self.since_lane_change = _frames(LANE_CHANGE_HOLD)

  def not_slowing(self, i):
    alpha = DT / (ACCEL_FILTER_TAU + DT)
    self.a_filtered = i.a_ego if self.a_filtered is None else self.a_filtered + alpha * (i.a_ego - self.a_filtered)
    return not i.brake_pressed and self.a_filtered > (OWN_SLOWDOWN_ACCEL if i.own_slowdown else NOT_SLOWING_ACCEL)

  @staticmethod
  def plan_slowdown(i, fraction=STOP_SLOWDOWN):
    """Slowest point of the plan within the horizon that is <= fraction of our speed for
    STOP_HOLD_POINTS consecutive points (a single dip is model noise): (t, v, distance, implied
    deceleration to get there) or None."""
    if not i.model_t or len(i.model_t) != len(i.model_v):
      return None
    slow = fraction * i.v_ego
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
    return t_min, v_min, d_min, required

  def stop_ahead(self, i):
    if i.lead_relevant or i.v_ego < STOP_MIN_SPEED or self.since_lane_change < _frames(LANE_CHANGE_HOLD):
      return None
    plan = self.plan_slowdown(i)
    if plan is None or plan[3] < STOP_DECEL:
      return None
    t_min, v_min, d_min, required = plan
    return Detection(STOP_AHEAD, d_min, required, t_min)

  @staticmethod
  def lead_metrics(i):
    """(closing speed, time to collision, required deceleration) for the current lead."""
    closing = i.v_ego - i.lead_v
    ttc = i.lead_d / closing if closing > 0.1 else float("inf")
    required = closing ** 2 / (2 * max(i.lead_d - LEAD_BUFFER, 1.0)) if closing > 0 else 0.0
    return closing, ttc, required

  def stopped_lead(self, i):
    if not i.lead_status or i.lead_prob < LEAD_MIN_PROB or i.v_ego < LEAD_MIN_SPEED:
      return None
    if i.lead_v > LEAD_STOPPED_SPEED or i.lead_d <= 0 or abs(i.lead_y) > LEAD_MAX_LATERAL:
      return None
    closing, ttc, required = self.lead_metrics(i)
    if closing < LEAD_MIN_CLOSING:
      return None
    if ttc > LEAD_MAX_TTC or required < LEAD_DECEL:
      return None
    return Detection(STOPPED_LEAD, i.lead_d, required, ttc, i.lead_v, i.lead_prob)

  # ---- diagnostics for the per-event logs (what blocked a candidate) ----
  def common_blockers(self, i, mode):
    out = []
    if mode == OFF:
      out.append("setting off")
    if not i.cruise_enabled:
      out.append("cruise off")
    if i.standstill:
      out.append("standstill")
    if i.brake_pressed:
      out.append("brake pressed")
    limit = OWN_SLOWDOWN_ACCEL if i.own_slowdown else NOT_SLOWING_ACCEL
    if self.a_filtered is not None and self.a_filtered <= limit:
      out.append(f"already slowing aEgo {self.a_filtered:.2f} <= {limit}")
    return out

  def stop_ahead_blockers(self, i, mode):
    out = self.common_blockers(i, mode)
    if i.lead_relevant:
      out.append(f"lead relevant d {i.lead_d:.0f} m vLead {i.lead_v / MS_PER_MPH:.0f} mph")
    if i.v_ego < STOP_MIN_SPEED:
      out.append(f"speed {i.v_ego / MS_PER_MPH:.1f} < {STOP_MIN_SPEED / MS_PER_MPH:.0f} mph")
    if self.since_lane_change < _frames(LANE_CHANGE_HOLD):
      out.append("lane change")
    plan = self.plan_slowdown(i)
    if plan is None:
      loose = self.plan_slowdown(i, 1.0)
      ratio = loose[1] / max(i.v_ego, 0.1) if loose else 1.0
      out.append(f"plan min {ratio:.2f} > {STOP_SLOWDOWN:.2f} of speed (2 points)")
    elif plan[3] < STOP_DECEL:
      out.append(f"implied decel {plan[3]:.2f} < {STOP_DECEL}")
    if not out:
      gate = self.gates[STOP_AHEAD]
      out.append("debounce" if gate.true_frames < gate.debounce_frames else "latched or min gap")
    return out

  def stopped_lead_blockers(self, i, mode):
    out = self.common_blockers(i, mode)
    if not i.lead_status:
      out.append("no lead")
    else:
      if i.lead_prob < LEAD_MIN_PROB:
        out.append(f"prob {i.lead_prob:.2f} < {LEAD_MIN_PROB}")
      if abs(i.lead_y) > LEAD_MAX_LATERAL:
        out.append(f"yRel {i.lead_y:.1f} > {LEAD_MAX_LATERAL}")
      if i.lead_v > LEAD_STOPPED_SPEED:
        out.append(f"vLead {i.lead_v:.1f} > {LEAD_STOPPED_SPEED} m/s")
      closing, ttc, required = self.lead_metrics(i)
      if closing < LEAD_MIN_CLOSING:
        out.append(f"closing {closing:.1f} < {LEAD_MIN_CLOSING} m/s")
      if ttc > LEAD_MAX_TTC:
        out.append(f"ttc {ttc:.1f} > {LEAD_MAX_TTC} s")
      if required < LEAD_DECEL:
        out.append(f"required decel {required:.2f} < {LEAD_DECEL}")
    if i.v_ego < LEAD_MIN_SPEED:
      out.append(f"speed {i.v_ego / MS_PER_MPH:.1f} < {LEAD_MIN_SPEED / MS_PER_MPH:.0f} mph")
    if not out:
      gate = self.gates[STOPPED_LEAD]
      out.append("debounce" if gate.true_frames < gate.debounce_frames else "latched or min gap")
    return out

  def update(self, i: StopWarningInputs, modes: dict):
    """Returns a list of (Detection, mode) that fired this frame (mode TEST or ON)."""
    self.frame += 1
    self.since_lane_change = 0 if i.lane_change else self.since_lane_change + 1
    not_slowing = self.not_slowing(i)   # always update the filter
    active = i.cruise_enabled and not i.standstill and not_slowing
    fired = []
    for kind, detect in ((STOP_AHEAD, self.stop_ahead), (STOPPED_LEAD, self.stopped_lead)):
      mode = modes.get(kind, OFF)
      det = detect(i) if (active and mode != OFF) else None
      if self.gates[kind].update(det is not None, self.frame) and det is not None:
        det.lead_status, det.lead_d, det.lead_y, det.lead_relevant = i.lead_status, i.lead_d, i.lead_y, i.lead_relevant
        if kind == STOP_AHEAD:
          det.lead_speed, det.lead_prob = i.lead_v, i.lead_prob
        fired.append((det, mode))
    return fired


# ---- data for a later red light slowdown: every plan dip and every unannounced driver stop ----
DIP_RATIO = 0.85                # the plan's slowest point <= this fraction of our speed (looser than STOP_SLOWDOWN)
DIP_MIN_TIME = 1.0              # s the dip must hold (STOP_DROPOUT gaps tolerated) to count
DIP_WATCH = 8.0                 # s after the dip starts: a brake / cruise cancel in this time = the driver stopped
DIP_END = 2.0                   # s without the dip before an episode ends
DIP_MAX = 60.0                  # s: an episode is closed after this at the latest
BRAKE_LOOKBACK = 10.0           # s: a driver brake with no dip in this time before it is a "no dip" stop
DIP_MIN_SPEED = 25 * MS_PER_MPH

PLAN_DIP = "stopAhead_planDip"
NO_DIP_BRAKE = "stopAhead_driverBrakeNoDip"


class PlanDipTracker:
  """Logs every time the model's plan starts to slow down (to DIP_RATIO of our speed) with cruise on, at 25 mph or
  more and no relevant lead, and what followed: did the warning fire, did the driver brake or cancel cruise (and how
  long after the dip started). A dip the driver does not brake for is the green light / hesitation class; the time
  from the dip to the driver's brake is how early a red light slowdown could act. It also logs each driver brake
  (or cruise cancel) from 25 mph with cruise on and no relevant lead that had no dip in the BRAKE_LOOKBACK before it:
  the stops the model did not see coming (or brakes for something else; check the video).

  update() returns actions for the event recorder: ("open", kind, summary), ("close", kind, extra),
  ("point", kind, summary)."""

  def __init__(self):
    self.frame = 0
    self.true_frames = 0
    self.false_frames = _frames(DIP_END)
    self.ep = None
    self.last_dip_frame = -10_000
    self.prev_cruise = False

  def update(self, i, warning_fired, plan=None):
    """plan: StopWarningLogic.plan_slowdown(i, DIP_RATIO) (computed by the caller to share it), or None."""
    self.frame += 1
    out = []
    cond = (i.cruise_enabled and not i.standstill and i.v_ego >= DIP_MIN_SPEED and not i.lead_relevant and
            plan is not None)
    if cond:
      self.true_frames += 1
      self.false_frames = 0
      self.last_dip_frame = self.frame
    else:
      self.false_frames += 1
      if self.false_frames > _frames(STOP_DROPOUT) and self.ep is None:
        self.true_frames = 0
    driver_action = self.prev_cruise and (i.brake_pressed or not i.cruise_enabled)
    driver_action_speed_ok = i.v_ego >= DIP_MIN_SPEED and not i.lead_relevant
    self.prev_cruise = i.cruise_enabled and not i.brake_pressed

    if self.ep is None and cond and self.true_frames >= _frames(DIP_MIN_TIME):
      start = self.frame - self.true_frames + 1
      t_min, v_min, d_min, required = plan
      self.ep = {"start": start, "min_ratio": v_min / max(i.v_ego, 0.1), "max_decel": required, "fired": None,
                 "brake": None, "speed": i.v_ego}
      out.append(("open", PLAN_DIP, {
        "speed_mph": i.v_ego / MS_PER_MPH, "plan_ratio": v_min / max(i.v_ego, 0.1), "plan_decel": required,
        "distance_m": d_min, "time_s": t_min, "dip_ratio_threshold": DIP_RATIO,
        "held_before_open_s": round(self.true_frames * DT, 2)}))
    ep = self.ep
    if ep is not None:
      if cond:
        ep["min_ratio"] = min(ep["min_ratio"], plan[1] / max(i.v_ego, 0.1))
        ep["max_decel"] = max(ep["max_decel"], plan[3])
      if warning_fired and ep["fired"] is None:
        ep["fired"] = (self.frame - ep["start"]) * DT
      if driver_action and ep["brake"] is None and (self.frame - ep["start"]) * DT <= DIP_WATCH + DIP_END:
        ep["brake"] = (self.frame - ep["start"]) * DT
      age = (self.frame - ep["start"]) * DT
      if (self.false_frames >= _frames(DIP_END) and (age >= DIP_WATCH or ep["brake"] is not None)) or age >= DIP_MAX:
        braked = ep["brake"] is not None and ep["brake"] <= DIP_WATCH
        out.append(("close", PLAN_DIP, {
          "outcome": "driver braked or cancelled cruise" if braked else
                     "no brake within 8 s (possible green light or model hesitation)",
          "driver_brake_after_dip_s": "" if ep["brake"] is None else round(ep["brake"], 2),
          "warning_fired_after_dip_s": "" if ep["fired"] is None else round(ep["fired"], 2),
          "min_plan_ratio": round(ep["min_ratio"], 3), "max_plan_decel": round(ep["max_decel"], 2),
          "dip_duration_s": round(age, 2)}))
        self.ep = None
        self.true_frames = 0
    elif driver_action and driver_action_speed_ok and self.frame - self.last_dip_frame > _frames(BRAKE_LOOKBACK):
      out.append(("point", NO_DIP_BRAKE, {
        "speed_mph": i.v_ego / MS_PER_MPH, "brake": i.brake_pressed, "cruise": i.cruise_enabled,
        "seconds_since_last_dip": "none in the drive" if self.last_dip_frame < 0 else
                                  round((self.frame - self.last_dip_frame) * DT, 1),
        "note": "driver stopped/slowed with no plan dip before it: a missed red light, or braking for something else"}))
    return out


def format_log_line(det, mode, v_ego, a_ego, gps=None, now=None):
  now = now or datetime.datetime.now()
  lat = f"{gps['latitude']:.6f}" if gps else ""
  lon = f"{gps['longitude']:.6f}" if gps else ""
  return (f"{now.isoformat(timespec='seconds')},{det.kind},{'test' if mode == TEST else 'on'},"
          f"{v_ego / MS_PER_MPH:.1f},{a_ego:.2f},{det.distance:.1f},{det.required_decel:.2f},{det.time:.1f},"
          f"{det.lead_speed / MS_PER_MPH:.1f},{det.lead_prob:.2f},{int(det.lead_status)},{det.lead_d:.1f},{det.lead_y:.1f},"
          f"{int(det.lead_relevant)},{lat},{lon}\n")


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
  EVENT_COLUMNS = ["t_wall", "vEgo_mph", "aEgo", "aEgo_filtered", "brake", "gas", "cruise", "set_mph",
                   "lead_status", "lead_prob", "lead_d", "lead_v", "lead_y", "lead_relevant",
                   "plan_min_mps", "plan_ratio", "plan_decel", "csc_active", "csc_mph",
                   "curve_offset", "curve_in_flight", "stop_ahead_cond", "stopped_lead_cond", "own_slowdown"]
  NEAR_MISS_STOP_AHEAD_RATIO = 0.80
  NEAR_MISS_STOP_AHEAD_SPEED = 20 * MS_PER_MPH
  NEAR_MISS_LEAD_SPEED = 3.0
  NEAR_MISS_LEAD_DISTANCE = 120.0
  NEAR_MISS_EGO_SPEED = 15 * MS_PER_MPH

  def __init__(self, params, params_memory):
    self.params = params
    self.params_memory = params_memory
    self.logic = StopWarningLogic()
    self.modes = {STOP_AHEAD: OFF, STOPPED_LEAD: OFF}
    self.read_counter = 0
    self.recorder = None
    self.near_miss = {}
    self.dips = PlanDipTracker()
    self.dip_event = None
    try:
      from openpilot.starpilot.common.feature_events import EventRecorder, NearMissTracker
      self.recorder = EventRecorder(self.EVENT_COLUMNS, hz=20, params=params)
      self.near_miss = {STOP_AHEAD: NearMissTracker(self.recorder, STOP_AHEAD),
                        STOPPED_LEAD: NearMissTracker(self.recorder, STOPPED_LEAD)}
    except Exception:
      self.recorder = None

  def _read_modes(self):
    for kind, key in self.PARAM_KEYS.items():
      try:
        value = int(self.params.get(key, encoding="utf8") or 0)
      except (TypeError, ValueError):
        value = OFF
      self.modes[kind] = value if value in (OFF, TEST, ON) else OFF

  def update(self, sm, planner):
    """Returns the list of warning kinds to alert on now (only those in ON mode)."""
    if self.read_counter % _frames(1.0) == 0:
      self._read_modes()
    self.read_counter += 1
    if self.modes[STOP_AHEAD] == OFF and self.modes[STOPPED_LEAD] == OFF:
      return []

    cs = sm["carState"]
    model = sm["modelV2"]
    lead = planner.lead_one
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
      lead_relevant=self._lead_relevant(planner, lead, cs),
      lane_change=self._lane_change(model),
      own_slowdown=self._own_slowdown(sm),
    )
    alerts = []
    fired = self.logic.update(inputs, self.modes)
    for det, mode in fired:
      gps = None
      try:
        gps = json.loads(self.params_memory.get("LastGPSPosition") or "null")
      except (TypeError, ValueError):
        gps = None
      append_log(format_log_line(det, mode, inputs.v_ego, inputs.a_ego, gps))
      if mode == ON:
        alerts.append(det.kind)
    try:
      self._log(sm, planner, inputs, fired)
    except Exception:
      pass
    return alerts

  @staticmethod
  def _own_slowdown(sm):
    try:
      return bool(sm["starpilotCarState"].mazdaStopSlowdownActive)
    except Exception:
      return False

  @staticmethod
  def _lane_change(model):
    try:
      return str(model.meta.laneChangeState) in ("laneChangeStarting", "laneChangeFinishing")
    except Exception:
      return False

  @staticmethod
  def _lead_relevant(planner, lead, cs):
    if hasattr(planner, "lead_relevant"):          # FrogPilot
      return bool(planner.lead_relevant)
    model_length = float(getattr(planner, "model_length", 0.0) or 0.0)
    return lead_is_relevant(bool(lead.status), float(lead.dRel), float(lead.vLead), float(getattr(lead, "aLeadK", 0.0)),
                            bool(cs.standstill), float(cs.vEgo), model_length)

  # ---- per-event logs (never raises into the planner: called inside try/except) ----
  def _log(self, sm, planner, i, fired):
    if self.recorder is None:
      return
    import time
    t = time.monotonic()
    logic = self.logic
    plan = logic.plan_slowdown(i, 1.0)
    plan_min = plan[1] if plan else float("nan")
    ratio = plan_min / max(i.v_ego, 0.1) if plan else float("nan")
    decel = plan[3] if plan else float("nan")
    try:
      vc = planner.starpilot_vcruise
      csc_active = bool(vc.csc_controlling_speed)
      csc_speed = float(vc.csc_target) / MS_PER_MPH
    except Exception:
      csc_active, csc_speed = False, 0.0
    try:
      fpcs = sm["starpilotCarState"]
      curve_offset, curve_in_flight = int(fpcs.curveSlowdownOffset), int(fpcs.curveSlowdownInFlight)
    except Exception:
      curve_offset, curve_in_flight = 0, 0
    sa = logic.stop_ahead(i) is not None
    sl = logic.stopped_lead(i) is not None
    self.recorder.add_row(t, (
      f"{time.time():.2f}", i.v_ego / MS_PER_MPH, i.a_ego, logic.a_filtered if logic.a_filtered is not None else 0.0,
      i.brake_pressed, False, i.cruise_enabled, float(sm["carState"].cruiseState.speed) / MS_PER_MPH,
      i.lead_status, i.lead_prob, i.lead_d, i.lead_v, i.lead_y, i.lead_relevant,
      plan_min, ratio, decel, csc_active, csc_speed, curve_offset, curve_in_flight, sa, sl, i.own_slowdown))
    fired_kinds = {det.kind for det, _ in fired}
    for det, mode in fired:
      self.recorder.record(det.kind, t, {
        "mode": "test" if mode == TEST else "on", "speed_mph": i.v_ego / MS_PER_MPH, "a_ego": i.a_ego,
        "distance_m": det.distance, "required_decel": det.required_decel, "time_s": det.time,
        "lead_status": det.lead_status, "lead_d": det.lead_d, "lead_v": det.lead_speed, "lead_prob": det.lead_prob,
        "lead_y": det.lead_y, "lead_relevant": det.lead_relevant, "plan_min_mps": plan_min, "plan_ratio": ratio,
        "own_slowdown": i.own_slowdown, "gps": self._gps()})
    # near-misses: a candidate was seen but the warning did not fire
    if self.modes[STOP_AHEAD] != OFF:
      cand = (not i.standstill and i.v_ego >= self.NEAR_MISS_STOP_AHEAD_SPEED and plan is not None and
              ratio <= self.NEAR_MISS_STOP_AHEAD_RATIO)
      self.near_miss[STOP_AHEAD].update(t, cand, STOP_AHEAD in fired_kinds,
                                        logic.stop_ahead_blockers(i, self.modes[STOP_AHEAD]) if cand else (),
                                        {"speed_mph": i.v_ego / MS_PER_MPH, "plan_ratio": ratio, "plan_decel": decel,
                                         "lead_d": i.lead_d, "lead_v": i.lead_v, "lead_relevant": i.lead_relevant})
    if self.modes[STOPPED_LEAD] != OFF:
      cand = (i.lead_status and i.lead_v <= self.NEAR_MISS_LEAD_SPEED and 0 < i.lead_d <= self.NEAR_MISS_LEAD_DISTANCE and
              i.v_ego >= self.NEAR_MISS_EGO_SPEED)
      closing, ttc, required = logic.lead_metrics(i)
      self.near_miss[STOPPED_LEAD].update(t, cand, STOPPED_LEAD in fired_kinds,
                                          logic.stopped_lead_blockers(i, self.modes[STOPPED_LEAD]) if cand else (),
                                          {"speed_mph": i.v_ego / MS_PER_MPH, "lead_d": i.lead_d, "lead_v": i.lead_v,
                                           "lead_prob": i.lead_prob, "lead_y": i.lead_y, "ttc": min(ttc, 99.0),
                                           "required_decel": required})
    # plan dips and unannounced driver stops (data for a red light slowdown)
    if self.modes[STOP_AHEAD] != OFF:
      dip_plan = logic.plan_slowdown(i, DIP_RATIO)
      for action, kind, data in self.dips.update(i, STOP_AHEAD in fired_kinds, dip_plan):
        if action == "open":
          data = dict(data, gps=self._gps())
          self.dip_event = self.recorder.open(kind, t, data, min_gap=0)
        elif action == "close":
          self.recorder.close(self.dip_event, t, data)
          self.dip_event = None
        else:
          self.recorder.record(kind, t, dict(data, gps=self._gps()), min_gap=5.0)
    self.recorder.tick(t)

  def _gps(self):
    try:
      gps = json.loads(self.params_memory.get("LastGPSPosition") or "null")
      return f"{gps['latitude']:.6f},{gps['longitude']:.6f}" if gps else ""
    except Exception:
      return ""
