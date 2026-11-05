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
FLOOR_SPEED = 20 * MS_PER_MPH
TARGET_DECEL = 1.0          # m/s^2 the stock cruise slows at once the set speed is lowered
TARGET_MARGIN = 15.0        # m behind the lead where the floor speed should be reached
FLOOR_MODE_ENABLED = True
FLOOR_DISTANCE = 130.0      # m: a confirmed stop this close: target = floor (SET- holds), not the distance target
FLOOR_LEAD_HORIZON = 2.0    # s: ... but not below the car ahead's speed this far ahead (a slower car that keeps moving)
# drive 45: the guard made two holds with a pause; the owner wants one hold to 20 mph (the radar keeps the gap if the
# car ahead keeps moving, and traffic moving again restores)
FLOOR_LEAD_GUARD = False
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
  radar_has_lead: bool = False      # stock MRCC radar (CRZ_CTRL.RADAR_HAS_LEAD), logged only
  radar_lead_distance: int = 0      # its distance bucket (CRZ_CTRL.RADAR_LEAD_RELATIVE_DISTANCE: 1 farthest .. 5 nearest), logged only
  model_alive: bool = False         # modelV2 is being received
  model_accel: float = 0.0          # modelV2.action.desiredAcceleration: < 0 the model wants to slow down
  plan_stop: bool = False           # the model plans a stop ahead (plan_stop(), the red light warning's check)
  plan_ratio: float = 1.0           # slowest planned speed in the horizon / ours (logs)
  plan_stop_d: float = 0.0          # m to where the stop is planned (logs)


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
    self.hold_restore = False        # latched and not active: keep the lowered set speed (no SET+) for now

  def _lead_ok(self, i):
    return (i.lead_alive and i.lead_status and i.lead_prob >= MIN_PROB and abs(i.lead_y) <= MAX_LATERAL and
            i.lead_v <= STOPPED_SPEED and i.v_ego - i.lead_v >= MIN_CLOSING and i.lead_d > 0)

  @staticmethod
  def _model_slowing(i):
    return i.model_alive and i.model_accel <= MODEL_SLOW_ACCEL

  @staticmethod
  def _slow_car(i, max_lateral=STOP_CAR_MAX_LATERAL, min_prob=STOP_CAR_MIN_PROB):
    """A car ahead (near our lane) that is stopped or much slower than us."""
    return (i.lead_alive and i.lead_status and i.lead_prob >= min_prob and abs(i.lead_y) <= max_lateral and
            0 < i.lead_d <= STOP_CAR_MAX_DISTANCE and
            (i.v_ego - i.lead_v >= STOP_CAR_SLOWER or i.lead_v <= STOP_CAR_RATIO * i.v_ego))

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
      if i.lead_v > STOPPED_SPEED:
        out.append(f"vLead {i.lead_v:.1f} > {STOPPED_SPEED} m/s")
      if i.v_ego - i.lead_v < MIN_CLOSING:
        out.append(f"closing {i.v_ego - i.lead_v:.1f} < {MIN_CLOSING} m/s")
    if i.v_ego < START_SPEED and not self.active:
      out.append(f"speed {i.v_ego / MS_PER_MPH:.1f} < {START_SPEED / MS_PER_MPH:.0f} mph")
    if not i.model_alive:
      out.append("stop with car: no modelV2")
    elif i.model_accel > MODEL_SLOW_ACCEL:
      out.append(f"stop with car: model accel {i.model_accel:.2f} > {MODEL_SLOW_ACCEL}")
    if i.lead_status and STOP_CAR_EARLY_MIN_PROB <= i.lead_prob < STOP_CAR_MIN_PROB and not self.latched:
      out.append("stop ahead not latched (a low-confidence car only counts after a planned stop)")
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
    self._update_latch(i)
    lead_ok = self._lead_ok(i)
    stop_with_car = self._stop_with_car(i)
    gates = (i.enabled and i.cruise_enabled and not i.brake_pressed and
             self.cruise_on_frames >= _frames(CRUISE_SETTLE))
    self.required, self.ttc = urgency(i.v_ego, i.lead_d, i.lead_v) if (i.lead_status and i.lead_alive) else (0.0, float("inf"))
    self.urgent = self.ttc <= URGENT_TTC or self.required >= URGENT_DECEL

    if not self.active:
      self.hold_frames = self.hold_frames + 1 if ((lead_ok or stop_with_car) and gates and i.v_ego >= START_SPEED) else 0
      if self._early_stop_with_car(i) and gates and i.v_ego >= START_SPEED:
        self.early_frames += 1
        self.early_gap = 0
      else:
        self.early_gap += 1
        if self.early_gap > _frames(EARLY_DROPOUT) or not self.latched:
          self.early_frames = 0
      early_start = self.early_frames >= _frames(EARLY_HOLD)
      if self.hold_frames >= _frames(HOLD_TIME) or early_start:
        self.active = True
        self.early = self.hold_frames < _frames(HOLD_TIME)
        self.kind = STOPPED_CAR if (lead_ok and not self.early) else STOP_WITH_CAR
        self.confirmed = not self.early
        self.confirm_frames = 0
        self.confirm_frame = self.frame if not self.early else -1
        self.floor_mode = False
        self.start_frame = self.frame
        self.lost_frames = self.out_of_lane_frames = self.go_frames = 0
        self.early_frames = self.early_gap = 0
        self.release_reason = None
    elif self.kind == STOP_WITH_CAR or (stop_with_car and i.lead_v > RELEASE_LEAD_SPEED):
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
      self.go_frames = self.go_frames + 1 if going else 0
      lost_time = RELEASE_LOST_TIME_LATCHED if self.latched else RELEASE_LOST_TIME
      if not gates:
        self._release("cruise off / brake" if not (i.cruise_enabled and not i.brake_pressed) else "openpilot disengaged", False)
      elif i.v_ego < EGO_STOPPED_SPEED:
        self._release("ego stopped behind the lead")
      elif self.go_frames >= _frames(GO_TIME):
        self._release("traffic moving again (model no longer slowing, car ahead not much slower)", False)
      elif self.lost_frames >= _frames(lost_time):
        self._release("lead lost (vision)", False)
    else:
      vision = i.lead_alive and i.lead_status and i.lead_prob >= LOST_MIN_PROB
      self.moving_frames = self.moving_frames + 1 if (vision and i.lead_v > RELEASE_LEAD_SPEED) else 0
      # "still there" = a vision lead in our lane that is not driving away; the stock radar never counts
      still_there = vision and abs(i.lead_y) <= MAX_LATERAL and i.lead_v <= RELEASE_LEAD_SPEED
      out = vision and abs(i.lead_y) > MAX_LATERAL
      self.lost_frames = 0 if still_there else self.lost_frames + 1
      self.out_of_lane_frames = self.out_of_lane_frames + 1 if out else 0
      if not gates:
        self._release("cruise off / brake" if not (i.cruise_enabled and not i.brake_pressed) else "openpilot disengaged", False)
      elif self.moving_frames >= _frames(RELEASE_MOVING_TIME):
        self._release("lead moved away (vision)")
      elif i.v_ego < EGO_STOPPED_SPEED:
        self._release("ego stopped behind the lead")
      elif self.out_of_lane_frames >= _frames(RELEASE_LOST_TIME):
        # the car may still be ahead: the gentle restore (taps, normal pacing) so a re-trigger wins
        self._release("lead left the lane", False)
      elif self.lost_frames >= _frames(RELEASE_LOST_TIME):
        self._release("lead lost (vision)", False)
    if self.active and self.early and not self.confirmed:
      sure = i.lead_alive and i.lead_status and i.lead_prob >= EARLY_CONFIRM_PROB
      self.confirm_frames = self.confirm_frames + 1 if sure else 0
      if self.confirm_frames >= _frames(EARLY_CONFIRM_TIME) or self.urgent:
        self.confirmed = True
        self.confirm_frame = self.frame
    if self.active and i.lead_status:
      self.target = stopped_car_target(i.lead_d)
      self.floor_mode = self.floor_mode or (FLOOR_MODE_ENABLED and self.confirmed and 0 < i.lead_d <= FLOOR_DISTANCE)
      if self.floor_mode:
        lead_soon = i.lead_v + min(i.lead_a, 0.0) * FLOOR_LEAD_HORIZON if FLOOR_LEAD_GUARD else 0.0
        self.target = min(self.target, max(FLOOR_SPEED, lead_soon))
    if self.active and self.early and not self.confirmed and self.lost_frames > _frames(EARLY_DROPOUT):
      self.target = 100.0            # not confirmed and the car is not seen now: hold the speed, do not lower further
    if self.active and self.kind == STOP_WITH_CAR:
      # a stop with a much slower car: slow down at once (holds allowed); an early start only taps until confirmed
      self.urgent = self.confirmed
    if self.active and self.floor_mode:
      self.urgent = True             # fast pace and holds, both kinds
    self.hold_restore = EARLY_START_ENABLED and self.latched and not self.active
    return self.active

  def _release(self, reason, restore_now=True):
    self.active = False
    self.hold_frames = 0
    self.moving_frames = 0
    self.go_frames = 0
    self.release_reason = reason
    self.restore_now = restore_now

  def near_miss_candidate(self, i):
    stopped = (i.lead_alive and i.lead_status and i.lead_v <= NEAR_MISS_LEAD_SPEED and
               0 < i.lead_d <= NEAR_MISS_DISTANCE and i.v_ego >= NEAR_MISS_EGO_SPEED)
    return stopped or (self._model_slowing(i) and self._slow_car(i, max_lateral=3.0, min_prob=0.2) and
                       i.v_ego >= NEAR_MISS_EGO_SPEED)
