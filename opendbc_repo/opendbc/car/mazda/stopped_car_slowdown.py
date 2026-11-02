"""
Stopped car ahead, or a stop ahead with a much slower car: lower the stock cruise set speed early (Mazda Gen1,
stock MRCC).

The stock radar cruise often brakes late, or not at all, for a car that is already stopped.
When openpilot's vision lead is a stopped car in our lane, this asks the shared set speed
controller (curve_slowdown.CurveSlowdownLogic) to lower the set speed towards a target that
lets a gentle cruise slowdown reach the floor speed about 15 m behind the lead, so the car
approaches slower and the MRCC radar has time to react. It never stops the car: the MRCC or
the driver does the final braking.

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
  radar_has_lead: bool = False      # stock MRCC radar (CRZ_CTRL.RADAR_HAS_LEAD), logged only
  radar_lead_distance: int = 0      # its distance bucket (CRZ_CTRL.RADAR_LEAD_RELATIVE_DISTANCE: 1 farthest .. 5 nearest), logged only
  model_alive: bool = False         # modelV2 is being received
  model_accel: float = 0.0          # modelV2.action.desiredAcceleration: < 0 the model wants to slow down


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
    lead_ok = self._lead_ok(i)
    stop_with_car = self._stop_with_car(i)
    gates = (i.enabled and i.cruise_enabled and not i.brake_pressed and
             self.cruise_on_frames >= _frames(CRUISE_SETTLE))
    self.required, self.ttc = urgency(i.v_ego, i.lead_d, i.lead_v) if (i.lead_status and i.lead_alive) else (0.0, float("inf"))
    self.urgent = self.ttc <= URGENT_TTC or self.required >= URGENT_DECEL

    if not self.active:
      self.hold_frames = self.hold_frames + 1 if ((lead_ok or stop_with_car) and gates and i.v_ego >= START_SPEED) else 0
      if self.hold_frames >= _frames(HOLD_TIME):
        self.active = True
        self.kind = STOPPED_CAR if lead_ok else STOP_WITH_CAR
        self.start_frame = self.frame
        self.lost_frames = self.out_of_lane_frames = self.go_frames = 0
        self.release_reason = None
    elif self.kind == STOP_WITH_CAR or (stop_with_car and i.lead_v > RELEASE_LEAD_SPEED):
      # a stop with a much slower car (or a "stopped" car that is rolling slowly towards the stop): the car ahead
      # may be moving, so "moved away" is not a release; traffic moving again is
      self.kind = STOP_WITH_CAR
      self.moving_frames = 0
      vision = i.lead_alive and i.lead_status and i.lead_prob >= STOP_CAR_MIN_PROB
      still_there = vision and abs(i.lead_y) <= STOP_CAR_MAX_LATERAL
      self.lost_frames = 0 if still_there else self.lost_frames + 1
      going = (i.model_alive and i.model_accel > GO_ACCEL and
               (not still_there or i.v_ego - i.lead_v < GO_SLOWER))
      self.go_frames = self.go_frames + 1 if going else 0
      if not gates:
        self._release("cruise off / brake" if not (i.cruise_enabled and not i.brake_pressed) else "openpilot disengaged", False)
      elif i.v_ego < EGO_STOPPED_SPEED:
        self._release("ego stopped behind the lead")
      elif self.go_frames >= _frames(GO_TIME):
        self._release("traffic moving again (model no longer slowing, car ahead not much slower)", False)
      elif self.lost_frames >= _frames(RELEASE_LOST_TIME):
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
    if self.active and i.lead_status:
      self.target = stopped_car_target(i.lead_d)
    if self.active and self.kind == STOP_WITH_CAR:
      self.urgent = True             # a stop with a much slower car: slow down at once (holds allowed)
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
