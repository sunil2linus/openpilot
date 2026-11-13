"""
Mazda Gen1 stock MRCC: the gas brings the cruise back after a brake (setting MazdaBrakeAutoResume, default off).

The owner's design (round n):
  A. Arm: the cruise was on for ARM_CRUISE_ON, then the driver's BRAKE turned it off (on -> off with the brake down, not
     his CANCEL, not the main switch, not a CANCEL of ours or one openpilot asked for), the main switch still on and a
     set speed stored. Then his gas (held GAS_HOLD, brake released) brings it back. Any speed, after a stop or a
     slowdown. No time limit: standing still never counts.
  B. Map guard (no timer): only on a mapped public road, the planner's road class known (highway, link, primary ..
     residential: PUBLIC_ROAD_CLASSES) with GPS trust MAX_GPS_TRUST. Unknown, other classes or a worse trust: no. One
     allowance: the map was good when he braked and is lost only AUTO_RESUME_MAP_GAP of moving time since it was last
     good, with the heading changed less than MAP_GAP_TURN since then: the brake's road still counts.
  C. Wheel calm: nothing is sent while turning (|lateral accel| from the wheel angle and the speed >= MERGE_TURN_LAT, or
     the wheel winding: curve_slowdown.MergeTurnGate's formula and constants); calm CALM_TIME first. The gas may be down
     during the turn; the press goes out when the wheel straightens, the gas pressed then (the panda only lets it
     through with the gas pressed now; within GAS_RECENT of his last gas we wait for it), the brake not.
  D. Disarm (until the cruise comes on again): his CANCEL / SET / RES on the wheel, the main switch off, not in D, a
     door or his seatbelt, the car's "not allowed" state (CRZ_CTRL byte 4, NOT_ALLOWED_TIME), the stored set speed gone,
     AUTO_RESUME_MAX_DIST driven since the brake. The map guard false at his gas press: that press is not used, a later
     one on a public road may be.
  E. Send: RES, up to BURST_FRAMES frames phase-locked to the wheel's CRZ_BTNS counter (CTR = wheel + 1, as
     curve_slowdown's holds), at least FRAME_GAP apart; the cruise must be on VERIFY_TIME after the last frame; one
     retry; then give up until the cruise comes on by other means. Never while curve_slowdown has a press going and the
     other way round (busy). openpilot refusing the engagement (its CANCEL): give up, no RES -> CANCEL loop.
  F. Speed jump: a stored set speed more than AUTO_RESUME_MAX_JUMP above the current speed: one SET- frame instead (from
     standby SET engages at the current speed), the street speed / merge / restore rules raise it from there.

Conservative: anything unknown means no frame. AutoResumeLogic is pure (no openpilot imports) and unit tested
(tests/test_auto_resume.py); AutoResume is the card-side wrapper (carcontroller.py).
"""
import math
from dataclasses import dataclass

from opendbc.car.mazda.curve_slowdown import (DT_CTRL, HOLD_FALLBACK_FRAMES, MERGE_TURN_LAT, MERGE_TURN_RATE, MS_PER_KPH,
                                              MS_PER_MPH, steer_lateral_accel)

ARM_CRUISE_ON = 2.0              # s the cruise was on before his brake
BRAKE_RECENT = 0.2               # s: the cruise falls 20-40 ms after the brake (13 logged brake cancels)
CANCEL_QUIET = 0.5               # s before the fall: no CANCEL frame of ours, no openpilot cancel without the brake, no wheel CANCEL
GAS_HOLD = 0.3                   # s his gas held (brake released) to count as a press
GAS_RECENT = 3.0                 # s after his gas was last down a press waiting for the wheel may still go
CALM_TIME = 0.5                  # s of calm wheel before a frame
AUTO_RESUME_MAP_GAP = 5.0        # s of moving time the map may be lost after a good brake-time road
MAP_GAP_TURN = 30.0              # deg: ... with the heading changed less than this since the map was last good
AUTO_RESUME_MAX_DIST = 800.0     # m driven since the brake without the cruise back: disarm
AUTO_RESUME_MAX_JUMP = 10 * MS_PER_MPH        # stored set speed this far above the current speed: SET- instead of RES
AUTO_RESUME_MAX_JUMP_KPH = 16 * MS_PER_KPH
PUBLIC_ROAD_CLASSES = tuple(range(1, 13))    # highwayClass 1 motorway .. 10 tertiary_link, 11 unclassified, 12 residential
MAX_GPS_TRUST = 0                # starpilotPlan.mapGpsTrust: 0 ok (as the street speed's rule 8)
NOT_ALLOWED_TIME = 0.5           # s of CRZ_CTRL byte 4 non-zero before it disarms
BURST_FRAMES = {"RES": 4, "SET_MINUS": 1}    # frames per attempt (one SET- only: a second would lower the new set speed)
FRAME_GAP = 10                   # frames: >= 0.1 s between two of our frames
VERIFY_TIME = 0.6                # s after the last frame for the cruise to be on
ATTEMPTS = 2                     # the first and one retry
MAX_FRAMES = 8                   # the panda's budget per brake cancel (mazda.h MAZDA_AR_MAX_FRAMES)
REFUSE_WINDOW = 2.0              # s after our engagement: openpilot's cancel (or the cruise off again) means it refused
REFUSE_CANCEL_FRAMES = 3         # frames of openpilot's cancel request in that window
OWN_ENGAGE_TIME = 2.0            # s the controller (curve_slowdown) is told the engagement was ours
BUSY_AFTER = 1.0                 # s after our last frame curve_slowdown still waits
MOVING_SPEED = 0.3               # m/s

OFF, ARMED, SENDING, DONE, GAVE_UP = 0, 1, 2, 3, 4   # starpilotCarState.mazdaAutoResume
RES, SET_MINUS = "RES", "SET_MINUS"


def _frames(seconds):
  return int(round(seconds / DT_CTRL))


@dataclass
class AutoResumeInputs:
  cruise_enabled: bool
  cruise_available: bool          # the main switch (CRZ_CTRL.CRZ_AVAILABLE)
  set_speed: float                # the stored set speed, m/s (dash units; kept while in standby)
  v_ego: float                    # m/s
  gas_pressed: bool
  brake_pressed: bool
  steering_angle: float = 0.0     # deg
  steering_rate: float = 0.0      # deg/s
  yaw_rate: float = 0.0           # deg/s (from the speed and the wheel angle)
  wheelbase: float = 2.7
  gear_drive: bool = True
  door_open: bool = False
  seatbelt_unlatched: bool = False
  wheel_cancel: bool = False      # the driver's own wheel buttons (bus 0; our frames are never read back)
  wheel_resume: bool = False
  wheel_set: bool = False         # SET+ or SET-
  set_allowed: bool = False       # CRZ_INFO.ACC_SET_ALLOWED
  not_allowed: bool = False       # CRZ_CTRL byte 4 non-zero
  op_cancel: bool = False         # openpilot asks for CANCEL (CC.cruiseControl.cancel)
  cancel_sent: bool = False       # a CANCEL frame went out this frame (carcontroller)
  other_button_active: bool = False   # openpilot's RESUME (stop and go) this frame
  plan_alive: bool = False        # starpilotPlan
  road_class: int = 0             # starpilotPlan.mapRoadClass
  gps_trust: int = 4              # starpilotPlan.mapGpsTrust
  engageable: bool = False        # selfdriveState.engageable (False when not received): no openpilot no-entry
  slowdown_busy: bool = False     # curve_slowdown has a press going (pending, a hold, a frame in the last BUSY_AFTER)
  slowdown_minus_inflight: bool = False  # ... a SET- of it not yet seen on the set speed (would claim our SET- landing)
  btn_counter: int = -1           # the wheel's CRZ_BTNS counter
  is_metric: bool = False


class AutoResumeLogic:
  def __init__(self, is_metric: bool = False):
    self.jump = AUTO_RESUME_MAX_JUMP_KPH if is_metric else AUTO_RESUME_MAX_JUMP
    self.frame = 0
    self.state = OFF
    self.events = []                 # (kind, data) for the event recorder, drained by the wrapper
    self.prev_enabled = False
    self.prev_brake = False
    self.on_frames = 0
    self.last_brake_edge = -10_000
    self.last_cancel_sent = -10_000
    self.last_op_cancel = -10_000
    self.last_wheel_cancel = -10_000
    self.prev_counter = -1
    self.wheel_frame = -10_000
    self.new_wheel = False
    self.yaw = 0.0
    self.calm_frames = 0
    self.lat = 0.0
    self.gas_frames = 0
    self.last_gas = -10_000
    self.own_engage_frame = -10_000
    self.refuse_frames = 0
    self._reset_episode()

  def _reset_episode(self):
    self.brake_map_ok = False
    self.map_gap = 0.0               # s of moving time since the map was last good
    self.map_yaw = 0.0               # heading then
    self.dist = 0.0
    self.press = None                # frame of a gas press waiting to go
    self.press_spent = False         # this gas press was used (sent, refused): a new one needs the gas released first
    self.attempt = 0
    self.button = None
    self.burst_sent = 0
    self.total_sent = 0
    self.last_frame = -10_000
    self.not_allowed_frames = 0
    self.wait_why = ""
    self.armed_frame = -10_000

  # ---- helpers ----
  @staticmethod
  def map_ok(i):
    return bool(i.plan_alive and int(i.road_class) in PUBLIC_ROAD_CLASSES and int(i.gps_trust) <= MAX_GPS_TRUST)

  def map_allowed(self, i):
    if self.map_ok(i):
      return True
    return self.brake_map_ok and self.map_gap <= AUTO_RESUME_MAP_GAP and abs(self.yaw - self.map_yaw) < MAP_GAP_TURN

  @property
  def busy(self):
    """curve_slowdown must not press: our frames are going or just went, or the cruise just came on by us."""
    return (self.state == SENDING or self.frame - self.last_frame < _frames(BUSY_AFTER) or
            self.frame - self.own_engage_frame < _frames(OWN_ENGAGE_TIME))

  @property
  def own_engage(self):
    """The cruise came on by our RES / SET- within OWN_ENGAGE_TIME (not his press, not an unexplained change)."""
    return self.frame - self.own_engage_frame < _frames(OWN_ENGAGE_TIME)

  def _event(self, kind, **data):
    if len(self.events) < 20:
      self.events.append((kind, data))

  def _disarm(self, why, state=OFF):
    if self.state in (ARMED, SENDING):
      self._event("autoResumeRefused", reason=why, disarmed=True, attempt=self.attempt, frames=self.total_sent,
                  dist_m=round(self.dist, 1))
    self.state = state
    self._reset_episode()

  def _drop_press(self, why):
    if self.press is not None:
      self._event("autoResumeRefused", reason=why, disarmed=False, attempt=self.attempt, frames=self.total_sent,
                  dist_m=round(self.dist, 1))
    self.press = None

  # ---- per frame ----
  def update(self, i: AutoResumeInputs):
    """Returns the button to send this frame (RES / SET_MINUS) or None. The frame carries CTR = wheel + 1."""
    self.frame += 1
    f = self.frame
    v = max(float(i.v_ego), 0.0)
    moving = v > MOVING_SPEED
    self.yaw += float(i.yaw_rate) * DT_CTRL
    self.new_wheel = i.btn_counter >= 0 and i.btn_counter != self.prev_counter
    if self.new_wheel:
      self.wheel_frame = f
    self.prev_counter = i.btn_counter
    if i.brake_pressed and not self.prev_brake:
      self.last_brake_edge = f
    self.prev_brake = i.brake_pressed
    if i.cancel_sent:
      self.last_cancel_sent = f
    if i.op_cancel and not i.brake_pressed:
      self.last_op_cancel = f
    if i.wheel_cancel:
      self.last_wheel_cancel = f
    if i.gas_pressed:
      self.last_gas = f
    self.gas_frames = self.gas_frames + 1 if (i.gas_pressed and not i.brake_pressed) else 0
    if not i.gas_pressed:
      self.press_spent = False
    # wheel calm (curve_slowdown.MergeTurnGate's formula and constants, CALM_TIME)
    self.lat = steer_lateral_accel(v, float(i.steering_angle), float(i.wheelbase) or 2.7)
    winding = abs(i.steering_rate) > MERGE_TURN_RATE and i.steering_angle * i.steering_rate > 0
    self.calm_frames = self.calm_frames + 1 if (abs(self.lat) < MERGE_TURN_LAT and not winding) else 0

    rose = i.cruise_enabled and not self.prev_enabled
    fell = self.prev_enabled and not i.cruise_enabled
    on_before = self.on_frames
    self.on_frames = self.on_frames + 1 if i.cruise_enabled else 0
    self.prev_enabled = i.cruise_enabled
    button = None

    # openpilot refusing an engagement of ours: its CANCEL, or the cruise off again soon after
    if self.state == DONE and f - self.own_engage_frame <= _frames(REFUSE_WINDOW):
      self.refuse_frames = self.refuse_frames + 1 if (i.op_cancel and i.cruise_enabled) else 0
      if self.refuse_frames >= REFUSE_CANCEL_FRAMES or i.cancel_sent or fell:
        self._event("autoResumeRefused", reason="openpilot cancel" if not fell or i.cancel_sent else "cruise off again",
                    disarmed=True, attempt=self.attempt, frames=self.total_sent, dist_m=0.0)
        self.state = GAVE_UP
        self._reset_episode()
        return None

    if rose:
      if self.state == SENDING and self.button is not None and self.total_sent > 0:
        self.state = DONE
        self.own_engage_frame = f
        self.refuse_frames = 0
        self._event("autoResumeSent", button=self.button, attempt=self.attempt, frames=self.total_sent,
                    set_mph=i.set_speed / MS_PER_MPH, vEgo_mph=v / MS_PER_MPH, dist_m=round(self.dist, 1),
                    ok=True, wait_s=round((f - self.armed_frame) * DT_CTRL, 1))
        return None
      # on by him (his RES / SET) or by openpilot: nothing of ours
      self.state = OFF
      self._reset_episode()
      return None

    if fell:
      self._arm_or_not(i, on_before, v)
      return None

    if self.state not in (ARMED, SENDING):
      return None

    # ---- armed: cruise off, waiting for his gas ----
    self.dist += v * DT_CTRL
    if self.map_ok(i):
      self.map_gap, self.map_yaw = 0.0, self.yaw
    elif moving:
      self.map_gap += DT_CTRL
    self.not_allowed_frames = self.not_allowed_frames + 1 if i.not_allowed else 0
    why = None
    if i.wheel_cancel or i.wheel_resume or i.wheel_set:
      why = "his button"
    elif not i.cruise_available:
      why = "main off"
    elif not i.gear_drive:
      why = "not in D"
    elif i.door_open or i.seatbelt_unlatched:
      why = "door / seatbelt"
    elif self.not_allowed_frames >= _frames(NOT_ALLOWED_TIME):
      why = "car not allowed"
    elif i.set_speed <= 0.0:
      why = "no set speed"
    elif self.dist > AUTO_RESUME_MAX_DIST:
      why = "distance"
    if why is not None:
      self._disarm(why)
      return None

    if self.state == ARMED:
      if self.press is None and not self.press_spent and self.gas_frames >= _frames(GAS_HOLD):
        self.press_spent = True
        if not self.map_allowed(i):
          self._event("autoResumeRefused", reason="map", disarmed=False, attempt=self.attempt, frames=self.total_sent,
                      dist_m=round(self.dist, 1), road_class=int(i.road_class), gps_trust=int(i.gps_trust),
                      plan_alive=bool(i.plan_alive))
        else:
          self.press = f
      if self.press is not None:
        if i.brake_pressed:
          self._drop_press("brake")
        elif f - self.last_gas > _frames(GAS_RECENT):
          self._drop_press("gas released")
        elif not self.map_allowed(i):
          self._drop_press("map")
        else:
          self.wait_why = self._send_blocker(i)
          if self.wait_why == "":
            self.button = SET_MINUS if i.set_speed > v + self.jump else RES
            if self.button == SET_MINUS and i.slowdown_minus_inflight:
              self.wait_why = "slowdown SET- in flight"
            else:
              self.state = SENDING
              self.attempt += 1
              self.burst_sent = 0
    if self.state == SENDING:
      button = self._sending(i, f)
    return button

  def _send_blocker(self, i):
    """Why no frame may go this frame ("" none)."""
    if self.calm_frames < _frames(CALM_TIME):
      return "turning"
    if not i.gas_pressed:
      return "gas"
    if i.brake_pressed:
      return "brake"
    if not i.set_allowed:
      return "car not ready"
    if not i.engageable:
      return "openpilot no entry"
    if i.op_cancel or i.cancel_sent or i.other_button_active:
      return "openpilot button"
    if i.slowdown_busy:
      return "slowdown busy"
    if self.total_sent >= MAX_FRAMES:
      return "budget"
    return ""

  def _sending(self, i, f):
    if i.brake_pressed:
      # he brakes again: this attempt ends, the next gas press may try again (attempts are not given back)
      self.state = ARMED
      self.press = None
      if self.attempt >= ATTEMPTS:
        self._give_up("brake during the last attempt")
      return None
    n = BURST_FRAMES[self.button]
    if self.burst_sent < n:
      if f - self.last_gas > _frames(GAS_RECENT):
        self.state = ARMED
        self._drop_press("gas released")
        if self.attempt >= ATTEMPTS:
          self._give_up("gas released during the last attempt")
        return None
      if self._send_blocker(i) != "":
        return None
      due = self.new_wheel or (f - self.wheel_frame >= HOLD_FALLBACK_FRAMES and f - self.last_frame >= HOLD_FALLBACK_FRAMES)
      if not due or f - self.last_frame < FRAME_GAP:
        return None
      self.burst_sent += 1
      self.total_sent += 1
      self.last_frame = f
      return self.button
    # the burst is out: the cruise must be on within VERIFY_TIME (the rise is handled in update)
    if f - self.last_frame > _frames(VERIFY_TIME):
      if self.attempt < ATTEMPTS and self.total_sent + BURST_FRAMES[self.button] <= MAX_FRAMES:
        self._event("autoResumeRefused", reason="no response, retry", disarmed=False, attempt=self.attempt,
                    frames=self.total_sent, dist_m=round(self.dist, 1))
        self.attempt += 1
        self.burst_sent = 0
        # the speed may have changed: RES or SET- again from the same rule
        self.button = SET_MINUS if i.set_speed > max(float(i.v_ego), 0.0) + self.jump else RES
        if self.button == SET_MINUS and i.slowdown_minus_inflight:
          self._give_up("slowdown SET- in flight")
      else:
        self._give_up("no response")
    return None

  def _give_up(self, why):
    self._event("autoResumeRefused", reason=why, disarmed=True, attempt=self.attempt, frames=self.total_sent,
                dist_m=round(self.dist, 1))
    self.state = GAVE_UP
    self._reset_episode()

  def _arm_or_not(self, i, on_before, v):
    f = self.frame
    if self.state == GAVE_UP:
      return
    by_brake = i.brake_pressed or f - self.last_brake_edge <= _frames(BRAKE_RECENT)
    if not by_brake:
      self.state = OFF
      self._reset_episode()
      return
    why = None
    if on_before < _frames(ARM_CRUISE_ON):
      why = "cruise on too short"
    elif f - self.last_cancel_sent <= _frames(CANCEL_QUIET) or f - self.last_op_cancel <= _frames(CANCEL_QUIET):
      why = "a cancel of openpilot's"
    elif f - self.last_wheel_cancel <= _frames(CANCEL_QUIET):
      why = "his CANCEL"
    elif not i.cruise_available:
      why = "main off"
    elif i.set_speed <= 0.0:
      why = "no set speed"
    self._reset_episode()
    if why is not None:
      self.state = OFF
      self._event("autoResumeRefused", reason=why, disarmed=True, attempt=0, frames=0, dist_m=0.0)
      return
    self.state = ARMED
    self.armed_frame = f
    self.brake_map_ok = self.map_ok(i)
    self.map_yaw = self.yaw
    self.press_spent = i.gas_pressed     # (a gas already down at the brake is not his press)
    self._event("autoResumeArmed", set_mph=i.set_speed / MS_PER_MPH, vEgo_mph=v / MS_PER_MPH,
                map_ok=self.brake_map_ok, road_class=int(i.road_class), gps_trust=int(i.gps_trust))


def enabled(CP, params):
  """The wrapper runs only with the setting on and the panda's flag set (interface.py: Gen1 stock MRCC, which also needs
  no openpilot longitudinal); otherwise nothing of this file ever runs."""
  from opendbc.car.mazda.values import MazdaSafetyFlags
  flags = int(CP.flags)
  if not (flags & MazdaSafetyFlags.GEN1 and flags & MazdaSafetyFlags.AUTO_RESUME):
    return False
  if flags & (MazdaSafetyFlags.RADAR_INTERCEPTOR | MazdaSafetyFlags.NO_MRCC) or CP.openpilotLongitudinalControl:
    return False
  try:
    return bool(params.get_bool("MazdaBrakeAutoResume"))
  except Exception:
    return False


class AutoResume:
  """Card-side wrapper: reads starpilotPlan (the map guard) and selfdriveState (openpilot's no-entry), feeds the logic,
  builds the CAN frame and writes the feature events (autoResumeArmed / autoResumeSent / autoResumeRefused)."""
  COLUMNS = ["vEgo_mph", "gas", "brake", "cruise", "set_mph", "state", "lat", "calm", "road_class", "gps_trust",
             "map_gap", "dist", "set_allowed", "not_allowed", "engageable", "wait", "sent"]

  def __init__(self, CP, params):
    import cereal.messaging as messaging
    self.sm = messaging.SubMaster(['starpilotPlan', 'selfdriveState'])
    try:
      self.is_metric = bool(params.get_bool("IsMetric"))
    except Exception:
      self.is_metric = False
    self.logic = AutoResumeLogic(self.is_metric)
    self.recorder = None
    try:
      from openpilot.starpilot.common.feature_events import EventRecorder
      self.recorder = EventRecorder(self.COLUMNS, hz=10, params=params)
    except Exception:
      self.recorder = None
    self.ratio = float(getattr(CP, "steerRatio", 0.0) or 15.5)
    self.wheelbase = float(getattr(CP, "wheelbase", 0.0) or 2.7)
    self.frame = 0

  def _alive(self, service):
    return bool(self.sm.alive[service] and self.sm.recv_frame[service] > 0)

  @staticmethod
  def _trust(plan):
    t = getattr(plan, "mapGpsTrust", None)
    try:
      return int(t) if t is not None else 4
    except (TypeError, ValueError):
      return 4

  @property
  def state(self):
    return self.logic.state

  @property
  def busy(self):
    return self.logic.busy

  @staticmethod
  def slowdown_state(slowdown):
    """(busy, a SET- in flight) of curve_slowdown's controller: a press of its going, or its frame within BUSY_AFTER."""
    lg = getattr(slowdown, "logic", None) if slowdown is not None else None
    if lg is None:
      return False, False
    busy = bool(getattr(lg, "pending", None) is not None or getattr(lg, "hold", None) is not None or
                lg.frame - getattr(lg, "last_send_frame", -10_000) < _frames(BUSY_AFTER))
    minus = any(getattr(p, "direction", None) == "SET_MINUS" for p in list(getattr(lg, "inflight", [])))
    return busy, minus

  def update(self, CC, CS, packer, CP, slowdown=None, cancel_sent=False):
    from opendbc.car import structs
    from opendbc.car.mazda import mazdacan
    from opendbc.car.mazda.values import Buttons
    self.frame += 1
    self.sm.update(0)
    out = CS.out
    plan_alive = self._alive('starpilotPlan')
    plan = self.sm['starpilotPlan']
    engageable = bool(self._alive('selfdriveState') and getattr(self.sm['selfdriveState'], "engageable", False))
    v = float(out.vEgo)
    angle = float(getattr(out, "steeringAngleDeg", 0.0) or 0.0)
    yaw_rate = math.degrees(max(v, 0.0) * math.tan(math.radians(angle) / self.ratio) / self.wheelbase)
    busy, minus = self.slowdown_state(slowdown)
    gear = getattr(out, "gearShifter", None)
    i = AutoResumeInputs(
      cruise_enabled=bool(out.cruiseState.enabled), cruise_available=bool(out.cruiseState.available),
      set_speed=float(out.cruiseState.speed), v_ego=v, gas_pressed=bool(out.gasPressed), brake_pressed=bool(out.brakePressed),
      steering_angle=angle, steering_rate=float(getattr(out, "steeringRateDeg", 0.0) or 0.0), yaw_rate=yaw_rate,
      wheelbase=self.wheelbase, gear_drive=gear == structs.CarState.GearShifter.drive,
      door_open=bool(getattr(out, "doorOpen", False)), seatbelt_unlatched=bool(getattr(out, "seatbeltUnlatched", False)),
      wheel_cancel=bool(getattr(CS, "crz_cancel", False)), wheel_resume=bool(getattr(CS, "crz_resume", False)),
      wheel_set=bool(getattr(CS, "crz_set_plus", False) or getattr(CS, "crz_set_minus", False)),
      set_allowed=bool(getattr(CS, "crz_set_allowed", False)), not_allowed=bool(getattr(CS, "crz_not_allowed", False)),
      op_cancel=bool(CC.cruiseControl.cancel), cancel_sent=bool(cancel_sent), other_button_active=bool(CC.cruiseControl.resume),
      plan_alive=plan_alive, road_class=int(getattr(plan, "mapRoadClass", 0) or 0) if plan_alive else 0,
      gps_trust=self._trust(plan) if plan_alive else 4, engageable=engageable,
      slowdown_busy=busy, slowdown_minus_inflight=minus, btn_counter=int(getattr(CS, "crz_btns_counter", -1)),
      is_metric=self.is_metric)
    button = self.logic.update(i)
    self._record(i, button)
    if button is None:
      return []
    btn = Buttons.RESUME if button == RES else Buttons.SET_MINUS
    return [mazdacan.create_button_cmd(packer, CP, CS.crz_btns_counter, btn)]

  def _record(self, i, button):
    if self.recorder is None:
      self.logic.events.clear()
      return
    try:
      import time
      t = time.monotonic()
      lg = self.logic
      if self.frame % 10 == 0 or button is not None:
        self.recorder.add_row(t, (round(i.v_ego / MS_PER_MPH, 2), int(i.gas_pressed), int(i.brake_pressed),
                                  int(i.cruise_enabled), round(i.set_speed / MS_PER_MPH, 1), lg.state, round(lg.lat, 2),
                                  lg.calm_frames, i.road_class, i.gps_trust, round(lg.map_gap, 1), round(lg.dist, 1),
                                  int(i.set_allowed), int(i.not_allowed), int(i.engageable), lg.wait_why,
                                  button or ""))
      for kind, data in lg.events:
        self.recorder.record(kind, t, data, min_gap=0.0)
      lg.events.clear()
      self.recorder.tick(t)
    except Exception:
      self.logic.events.clear()
