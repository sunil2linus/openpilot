"""
Gas auto-resume (Stage 3, setting `auto_resume`, off by default): after his BRAKE turned the stock cruise off, his gas
presses RES for him, so the cruise comes back at his set speed. Port of the owner's previous fork
(starpilot-mazda a3f2d2e, opendbc/car/mazda/auto_resume.py, "MazdaBrakeAutoResume"); the rules are the same:

  A. Arm: the cruise was on for ARM_CRUISE_ON, then the driver's BRAKE turned it off (on -> off with the brake down, not
     his CANCEL, not the main switch, not a CANCEL of ours or one openpilot asked for), the main switch still on and a
     set speed stored. Then his gas (held GAS_HOLD, brake released) brings it back. Any speed, after a stop or a
     slowdown. No time limit: standing still never counts.
  B. Map guard (no timer): only on a mapped public road (road class 1 motorway .. 12 residential) with GPS trust 0, AT
     THE SEND, and the road unchanged for MAP_STABLE_TIME before it. 7.0: from the map view (mazda_mapviewd, the same
     file the curve features read): "cls" for the class, "trust" for the GPS trust and the matched road "chain" in place
     of mapd's road name; the view counts while it is alive (view_age.py: up to VIEW_ALIVE after its GPS message, so
     a 1 Hz GPS works), and the road must also be the same in ROAD_VIEWS views in a row (known and stable, as the old
     fork's name hold meant). Unknown, other classes or a worse trust: no.
  C. Wheel calm: nothing is sent while turning (|lateral accel| from the wheel angle and the speed >= MERGE_TURN_LAT, the
     wheel winding, or |wheel angle| >= AUTO_RESUME_MAX_ANGLE); calm CALM_TIME first; AUTO_RESUME_BIG_TURN_WAIT after
     the wheel was over AUTO_RESUME_BIG_TURN. The gas may be down during the turn; the press goes out when the wheel
     straightens, the gas pressed then (the panda only lets it through with the gas pressed now; within GAS_RECENT of
     his last gas we wait for it), the brake not.
  G. Creep guard: under AUTO_RESUME_CREEP_SPEED only behind a moving car ahead (radard's lead: within LEAD_NEAR, faster
     than LEAD_MOVING and not more than LEAD_SLOWER slower than us); otherwise wait (armed).
  D. Disarm (until the cruise comes on again): his CANCEL / SET / RES on the wheel, the main switch off, not in D, a
     door or his seatbelt, the car's "not allowed" state (CRZ_CTRL byte 4, NOT_ALLOWED_TIME), the stored set speed gone,
     AUTO_RESUME_MAX_DIST driven or AUTO_RESUME_MAX_MOVING moving since the brake.
  E. Send: RES, up to BURST_FRAMES frames phase-locked to the wheel's CRZ_BTNS counter (right after a new wheel frame;
     the frame carries CTR = wheel + 1), at least FRAME_GAP apart; the cruise must be on VERIFY_TIME after the last
     frame; one retry; then give up until the cruise comes on by other means. Never while the speed control's actuator
     has a press going (busy). openpilot refusing the engagement (its CANCEL): give up, no RES -> CANCEL loop.
  F. Always RES, never SET-.

The panda (opendbc/safety/modes/mazda.h, safety param bit 256) only lets a RES through while controls are not allowed
inside its own window (the cruise's falling edge with the brake and the main switch on; 8 frames; 180 s of moving
time); the car side stays inside it (MAX_FRAMES, AUTO_RESUME_MAX_MOVING).

In 7.0 the frame goes out through the speed control's one button actuator (actuator.py, its RES path for the cruise
off); the planner and the stop slowdown then see it as his RES ("resumed", RES quiet). AutoResumeLogic is pure (no
openpilot imports; tests/test_auto_resume.py); inputs_from() builds its inputs from the caller's frame dict and ctx.
"""
import math
from dataclasses import dataclass

DT_CTRL = 0.01                   # card runs at 100 Hz
MS_PER_MPH = 0.44704
DRIVER_QUIET = 1.0               # s (actuator.DRIVER_QUIET: after his RES nothing of ours is pressed this long)
HOLD_FALLBACK_FRAMES = 12        # (actuator.HOLD_FALLBACK_FRAMES) no new wheel frame this long: send anyway
MERGE_TURN_LAT = 1.0             # m/s^2 (the old fork's curve_slowdown.MergeTurnGate)
MERGE_TURN_RATE = 15.0           # deg/s with |angle| growing
MERGE_TURN_RATIO = 18.9
MERGE_TURN_SLIP = 0.001          # s^2/m^2

ARM_CRUISE_ON = 2.0              # s the cruise was on before his brake
# the cruise falls 20-40 ms after the brake edge, with the brake still down (16 logged brake cancels): the brake must be
# down on the frame it falls, as the panda requires (mazda.h: brake_pressed at the CRZ_CTRL falling edge)
CANCEL_QUIET = 0.5               # s before the fall: no CANCEL frame of ours, no openpilot cancel without the brake, no wheel CANCEL
GAS_HOLD = 0.3                   # s his gas held (brake released) to count as a press
GAS_RECENT = 3.0                 # s after his gas was last down a press waiting for the wheel may still go
CALM_TIME = 0.5                  # s of calm wheel before a frame
AUTO_RESUME_MAX_ANGLE = 45.0     # deg: |wheel angle| under this for CALM_TIME too
AUTO_RESUME_BIG_TURN = 90.0      # deg: after a big turn of the wheel wait AUTO_RESUME_BIG_TURN_WAIT, armed
AUTO_RESUME_BIG_TURN_WAIT = 2.0  # s since |wheel angle| was last above AUTO_RESUME_BIG_TURN
MAP_STABLE_TIME = 1.0            # s the road class and road unchanged before a send
MAP_BLIP = 1.5                   # s: class 0 for less than this, then the same class and road again, is no road change
ROAD_NAME_HOLD = 5.0             # s (the old fork's mapdOut name hold; the 7.0 view's road is alive or there is no map)
ROAD_VIEWS = 2                   # views in a row with the same road class and road before a send (a 1 Hz GPS: 1 s)
ROAD_NAME_FRESH = 4.0            # s
AUTO_RESUME_CREEP_SPEED = 10 * MS_PER_MPH    # m/s (4.5): slower than this only behind a moving lead
LEAD_NEAR = 50.0                 # m
LEAD_MOVING = 3 * MS_PER_MPH     # m/s (1.34)
LEAD_SLOWER = 1.0                # m/s: the lead not slower than us by more than this
AUTO_RESUME_MAX_DIST = 800.0     # m driven since the brake without the cruise back: disarm
# the panda closes its window after 180 s of moving time (mazda.h MAZDA_AR_MAX_MOVING_US): disarm a little before, so
# no frame of ours is dropped there (a crawl in traffic: 800 m at 10 mph is ~180 s)
AUTO_RESUME_MAX_MOVING = 170.0   # s
PUBLIC_ROAD_CLASSES = tuple(range(1, 13))    # highwayClass 1 motorway .. 10 tertiary_link, 11 unclassified, 12 residential
MAX_GPS_TRUST = 0                # the map view's GpsTrust status: 0 ok
NOT_ALLOWED_TIME = 0.5           # s of CRZ_CTRL byte 4 non-zero before it disarms
BURST_FRAMES = 4                 # RES frames per attempt
FRAME_GAP = 10                   # frames: >= 0.1 s between two of our frames
VERIFY_TIME = 0.6                # s after the last frame for the cruise to be on
ATTEMPTS = 2                     # the first and one retry
MAX_FRAMES = 8                   # the panda's budget per brake cancel (mazda.h MAZDA_AR_MAX_FRAMES)
REFUSE_WINDOW = 2.0              # s after our engagement: openpilot's cancel (or the cruise off again) means it refused
REFUSE_CANCEL_FRAMES = 3         # frames of openpilot's cancel request in that window
OWN_ENGAGE_TIME = DRIVER_QUIET   # s after our engagement the actuator still waits (busy): as after his RES on the wheel
BUSY_AFTER = 1.0                 # s after our last frame the actuator still waits
MOVING_SPEED = 0.3               # m/s

OFF, ARMED, SENDING, DONE, GAVE_UP = 0, 1, 2, 3, 4
STATE_NAMES = {OFF: "off", ARMED: "armed", SENDING: "sending", DONE: "done", GAVE_UP: "gave_up"}
RES = "RES"


def _frames(seconds):
  return int(round(seconds / DT_CTRL))


def steer_lateral_accel(v, angle_deg, wheelbase, ratio=MERGE_TURN_RATIO, slip=MERGE_TURN_SLIP):
  """m/s^2 from the speed and the steering wheel angle (a bicycle model with an understeer term)."""
  return v * v * math.radians(angle_deg) / (ratio * wheelbase * (1.0 + slip * v * v))


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
  yaw_rate: float = 0.0           # deg/s (logged only)
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
  cancel_sent: bool = False       # a CANCEL frame goes out this frame (carcontroller)
  other_button_active: bool = False   # openpilot's RESUME (stop and go) this frame
  plan_alive: bool = False        # the map view is alive (view_age.py)
  road_class: int = 0             # the view's road class
  road_name: str = ""             # the view's matched road ("" none)
  road_name_age: float = 0.0      # s since that road was received
  road_name_last: str = ""        # the last road received, however old (compared with the road at his brake)
  road_views: int = ROAD_VIEWS    # views in a row with this road class and road
  lead_status: bool = False       # radarState.leadOne
  lead_d: float = 0.0             # m
  lead_v: float = 0.0             # m/s
  can_valid: bool = False         # CarState.canValid
  gps_trust: int = 4              # the view's GpsTrust status
  engageable: bool = False        # selfdriveState.engageable (False when not received): no openpilot no-entry
  slowdown_busy: bool = False     # the actuator has a press going (a hold, a frame in the last BUSY_AFTER)
  btn_counter: int = -1           # the wheel's CRZ_BTNS counter
  is_metric: bool = False


class AutoResumeLogic:
  def __init__(self, is_metric: bool = False):
    self.frame = 0
    self.state = OFF
    self.events = []                 # (kind, data) for the event log, drained by the caller
    self.prev_enabled = False
    self.on_frames = 0
    self.last_cancel_sent = -10_000
    self.last_op_cancel = -10_000
    self.last_wheel_cancel = -10_000
    self.prev_counter = -1
    self.wheel_frame = -10_000
    self.new_wheel = False
    self.calm_frames = 0
    self.big_turn_frame = -10_000    # frame |wheel angle| was last above AUTO_RESUME_BIG_TURN
    self.road = None                 # (class, road) last frame
    self.road_frame = -10_000        # frame it last changed
    self.blip_frame = None           # (MAP_BLIP) the first frame of a class 0 blip on a known road
    self.good_frames = 0             # consecutive frames with a public road class
    self.brake_name = ""             # the road when his brake turned the cruise off
    self.last_wheel_btn = -10_000    # frame his RES / SET was last down
    self.lat = 0.0
    self.gas_frames = 0
    self.last_gas = -10_000
    self.own_engage_frame = -10_000
    self.refuse_frames = 0
    self._prev_frame_state = None
    self._reset_episode()

  def _reset_episode(self):
    self.dist = 0.0
    self.moving_time = 0.0
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
    return self.map_ok(i)

  def road_stable(self):
    return self.frame - self.road_frame >= _frames(MAP_STABLE_TIME)

  @staticmethod
  def creep_ok(i):
    """Under AUTO_RESUME_CREEP_SPEED only behind a moving car ahead."""
    v = max(float(i.v_ego), 0.0)
    if v >= AUTO_RESUME_CREEP_SPEED:
      return True
    return bool(i.lead_status and 0.0 < i.lead_d <= LEAD_NEAR and i.lead_v > LEAD_MOVING and i.lead_v >= v - LEAD_SLOWER)

  @property
  def busy(self):
    """The actuator must not press: our frames are going or just went, or the cruise just came on by us."""
    return (self.state == SENDING or self.frame - self.last_frame < _frames(BUSY_AFTER) or
            self.frame - (self.own_engage_frame - 1) < _frames(OWN_ENGAGE_TIME))

  @property
  def own_engaged(self):
    """The cruise came on by our RES within OWN_ENGAGE_TIME (the frame it rose included)."""
    return self.state == DONE and self.frame - self.own_engage_frame < _frames(OWN_ENGAGE_TIME)

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

  def unsend(self):
    """The frame update() just returned did not go out (the actuator refused it): take it back from the counts."""
    if self._prev_frame_state is not None and self.state == SENDING:
      self.burst_sent, self.total_sent, self.last_frame = self._prev_frame_state
    self._prev_frame_state = None

  # ---- per frame ----
  def update(self, i: AutoResumeInputs):
    """Returns the button to send this frame (RES) or None. The frame carries CTR = wheel + 1."""
    self.frame += 1
    self._prev_frame_state = None
    f = self.frame
    v = max(float(i.v_ego), 0.0)
    moving = v > MOVING_SPEED
    road = (int(i.road_class) if i.plan_alive else 0, str(i.road_name or ""))
    self.good_frames = self.good_frames + 1 if (i.plan_alive and int(i.road_class) in PUBLIC_ROAD_CLASSES) else 0
    if road[0] == 0 and self.road is not None and self.road[0] != 0:
      # (MAP_BLIP) class 0 on a known road: the road is kept for MAP_BLIP (the send still needs the map good)
      if self.blip_frame is None:
        self.blip_frame = f
      if f - self.blip_frame >= _frames(MAP_BLIP):
        self.road, self.road_frame, self.blip_frame = road, self.blip_frame, None
    else:
      if road != self.road:
        self.road, self.road_frame = road, f
      self.blip_frame = None
    if i.wheel_resume or i.wheel_set:
      self.last_wheel_btn = f
    self.new_wheel = i.btn_counter >= 0 and i.btn_counter != self.prev_counter
    if self.new_wheel:
      self.wheel_frame = f
    self.prev_counter = i.btn_counter
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
    # wheel calm
    self.lat = steer_lateral_accel(v, float(i.steering_angle), float(i.wheelbase) or 2.7)
    winding = abs(i.steering_rate) > MERGE_TURN_RATE and i.steering_angle * i.steering_rate > 0
    straight = abs(float(i.steering_angle)) < AUTO_RESUME_MAX_ANGLE
    if abs(float(i.steering_angle)) > AUTO_RESUME_BIG_TURN:
      self.big_turn_frame = f
    self.calm_frames = self.calm_frames + 1 if (abs(self.lat) < MERGE_TURN_LAT and not winding and straight) else 0

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
      # (his own RES / SET on the wheel just before: his engagement, not ours)
      if (self.state == SENDING and self.button is not None and self.total_sent > 0 and
          f - self.last_wheel_btn > _frames(CANCEL_QUIET)):
        self.state = DONE
        self.own_engage_frame = f
        self.refuse_frames = 0
        self._event("autoResumeSent", button=self.button, attempt=self.attempt, frames=self.total_sent,
                    set_mph=round(i.set_speed / MS_PER_MPH, 1), vEgo_mph=round(v / MS_PER_MPH, 1),
                    dist_m=round(self.dist, 1), ok=True, wait_s=round((f - self.armed_frame) * DT_CTRL, 1))
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
    if moving:
      self.moving_time += DT_CTRL
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
    elif self.moving_time > AUTO_RESUME_MAX_MOVING:
      why = "moving time"
    if why is not None:
      self._disarm(why)
      return None

    if self.state == ARMED:
      if self.press is None and not self.press_spent and self.gas_frames >= _frames(GAS_HOLD):
        self.press_spent = True
        self.press = f
        if not self.map_allowed(i):
          # the map is a wait while his gas is held, not a spent press; the send itself still needs the map good
          self._event("autoResumeRefused", reason="map (waiting)", disarmed=False, attempt=self.attempt,
                      frames=self.total_sent, dist_m=round(self.dist, 1), road_class=int(i.road_class),
                      gps_trust=int(i.gps_trust), plan_alive=bool(i.plan_alive))
      if self.press is not None:
        if i.brake_pressed:
          self._drop_press("brake")
        elif f - self.last_gas > _frames(GAS_RECENT):
          self._drop_press("map" if not self.map_ok(i) else "gas released")
        else:
          self.wait_why = self._send_blocker(i)
          if self.wait_why == "":
            self.button = RES
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
    if (self.frame - self.big_turn_frame) * DT_CTRL < AUTO_RESUME_BIG_TURN_WAIT:
      return "after a big turn"
    if not i.gas_pressed:
      return "gas"
    if not self.map_ok(i):
      return "map"
    if not self.road_stable() or self.good_frames < _frames(MAP_STABLE_TIME) or int(i.road_views) < ROAD_VIEWS:
      return "road just changed"
    last = str(i.road_name_last or i.road_name or "")
    if not (i.road_name_age <= ROAD_NAME_FRESH or last == self.brake_name):
      return "road name not fresh"
    if not self.creep_ok(i):
      return "slow, no moving lead"
    if not i.can_valid:
      return "can invalid"
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
    n = BURST_FRAMES
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
      self._prev_frame_state = (self.burst_sent, self.total_sent, self.last_frame)
      self.burst_sent += 1
      self.total_sent += 1
      self.last_frame = f
      return self.button
    # the burst is out: the cruise must be on within VERIFY_TIME (the rise is handled in update)
    if f - self.last_frame > _frames(VERIFY_TIME):
      if self.attempt < ATTEMPTS and self.total_sent + BURST_FRAMES <= MAX_FRAMES:
        self._event("autoResumeRefused", reason="no response, retry", disarmed=False, attempt=self.attempt,
                    frames=self.total_sent, dist_m=round(self.dist, 1))
        self.attempt += 1
        self.burst_sent = 0
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
    if not i.brake_pressed:
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
    self.brake_name = str(i.road_name_last or i.road_name or "")   # the last road received at his brake
    self.press_spent = i.gas_pressed     # (a gas already down at the brake is not his press)
    self._event("autoResumeArmed", set_mph=round(i.set_speed / MS_PER_MPH, 1), vEgo_mph=round(v / MS_PER_MPH, 1),
                map_ok=self.map_ok(i), road_class=int(i.road_class), gps_trust=int(i.gps_trust))


class ViewRoad:
  """The map guard's road from the map view file (mazda_mapviewd). update() -> (alive, class, road, age, last road, trust):
  alive = the view is alive (view_age.py) with a GPS position; road = the matched road ("chain", "" unmatched), the
  stand-in for the old fork's mapd road name; age = s since a matched road was last seen; trust = the view's GpsTrust
  status (4 unknown). `views`: how many views in a row had this road class and road (0 without a live view)."""
  def __init__(self):
    self.last = ""
    self.last_t = None
    self.clock = None
    self.key = None
    self.views = 0

  def _age(self, now):
    return 1e9 if self.last_t is None else max(now - self.last_t, 0.0)

  def update(self, view, now):
    from openpilot.starpilot.car.mazda.speed.view_age import GONE, ViewClock   # (AutoResumeLogic itself stays pure)
    if self.clock is None:
      self.clock = ViewClock()
    if self.clock.update(view, now) == GONE:
      self.key, self.views = None, 0
      return False, 0, "", self._age(now), self.last, 4
    matched = bool(view.get("matched"))
    road = str(int(view.get("chain", 0) or 0)) if matched else ""
    if road:
      self.last, self.last_t = road, now
    cls = int(view.get("cls", 0) or 0) if matched else 0
    if (cls, road) != self.key:
      self.key, self.views = (cls, road), 1
    elif self.clock.new:
      self.views += 1
    trust = int(view["trust"]) if view.get("trust") is not None else 4
    return True, cls, road, self._age(now), self.last, trust


def inputs_from(d, ctx, road: ViewRoad, *, wheelbase=2.7, steer_ratio=15.5, slowdown_busy=False, is_metric=False):
  """AutoResumeInputs from the caller's frame dict `d` (caller.SpeedControl.inputs) and the contributors' ctx."""
  now = float(ctx.get("now", 0.0))
  alive, cls, name, age, last, trust = road.update(ctx.get("view"), now)
  lead = ctx.get("lead") or None
  if lead is not None and now - float(lead.get("t", now)) > 0.5:
    lead = None
  v = float(d.get("v_ego", 0.0))
  angle = float(d.get("angle", 0.0) or 0.0)
  yaw_rate = math.degrees(max(v, 0.0) * math.tan(math.radians(angle) / steer_ratio) / wheelbase) if wheelbase else 0.0
  return AutoResumeInputs(
    cruise_enabled=bool(d["cruise_enabled"]), cruise_available=bool(d["main_on"]), set_speed=float(d["set_speed"]),
    v_ego=v, gas_pressed=bool(d["gas"]), brake_pressed=bool(d["brake"]), steering_angle=angle,
    steering_rate=float(d.get("steer_rate", 0.0) or 0.0), yaw_rate=yaw_rate, wheelbase=float(wheelbase) or 2.7,
    gear_drive=bool(d.get("gear_drive", False)), door_open=bool(d.get("door_open", False)),
    seatbelt_unlatched=bool(d.get("seatbelt_unlatched", False)), wheel_cancel=bool(d["cancel"]),
    wheel_resume=bool(d["resume"]), wheel_set=bool(d["set_plus"] or d["set_minus"]),
    set_allowed=bool(d.get("set_allowed", False)), not_allowed=bool(d.get("not_allowed", False)),
    op_cancel=bool(d.get("op_cancel", False)), cancel_sent=bool(d.get("cancel_sent", False)),
    other_button_active=bool(d.get("op_resume", False)), plan_alive=alive, road_class=cls if alive else 0,
    road_name=name, road_name_age=age, road_name_last=last, road_views=road.views if alive else 0,
    lead_status=bool(lead is not None and lead.get("status")), lead_d=float(lead["d"]) if lead is not None else 0.0,
    lead_v=float(lead["v"]) if lead is not None else 0.0, can_valid=bool(d.get("can_valid", False)),
    gps_trust=trust if alive else 4, engageable=bool(ctx.get("engageable", False)), slowdown_busy=bool(slowdown_busy),
    btn_counter=int(d.get("wheel_counter", -1)), is_metric=bool(is_metric))
