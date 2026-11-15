"""
Camera-led curve handling (round Q; owner: "map speed only as reference, live camera decides; start safe; +1 taps inside
the curve when live data proves margin; press-and-hold toward the curve end; the steering-straightening check starts too
late"). Setting MazdaCameraCurveRaise (default on). Pure logic, no openpilot imports; the wrapper in curve_slowdown.py
feeds it every frame and acts on its answer.

The live data:
  * measured lateral acceleration now: from the wheel angle and the speed (curve_slowdown.steer_lateral_accel:
    v^2 rad(angle) / (18.9 x 2.7 x (1 + 0.001 v^2)); carState.yawRate is always 0 on this car).
  * the model's predicted lateral acceleration ahead (modelV2: the curvature of its plan, orientationRate.z / velocity.x
    at each point, against position.x): at a speed v the peak |curvature| x v^2 over the points v x t0 .. v x t1 m
    ahead. Offline (drives 6f-75, drivingModelData's path, 2 Hz in the qlogs): reliable to ~2 s ahead (under-predicts
    by more than 0.5 m/s^2 in 11 % of frames at 2 s, 24 % at 3 s); the measured lateral now and the model's peak ahead
    together under-predicted by more than 0.5 in ~1 %.

Rules (all need the camera's lane lines (probability >= CAM_LANE_PROB), the speed >= CAM_MIN_SPEED, a fresh model):
  (a) The approach is unchanged: the target is the lower of the map's and the camera's, as before.
  (b) Phantom release: past a map curve's start with the measured |lateral| under CAM_PHANTOM_LAT and the model's peak
      over 0.5-2 s at the current set speed under it too, for CAM_PHANTOM_TIME: no more SET- for that map curve; the
      normal restore applies (drive 75 t ~700, Rochester Rd construction: 45 -> 35 mph on a straight road).
  (c) In-curve +1 tap (at most every CAM_TAP_DT) when all: the larger of the measured lateral scaled to the speed + 1 mph
      and the model's peak over 0.5-3 s at that speed under CAM_LAT_LKAS (CAM_LAT_TI with the stock LKAS blocked or the
      camera's fault: the TI alone); the set at most round(speed) + 1; not tightening (the model's peak over 0.5-2 s under
      the measured x CAM_TIGHTEN_GAIN + CAM_TIGHTEN_ADD); a steering reserve (the median |torque command| over
      CAM_TORQUE_TIME under CAM_TORQUE_RESERVE); the set under the street / limit / merge target (`cap`) and under the plan
      speed of any map curve ahead more than CAM_MAP_AHEAD_MIN away (`map_ahead`).
  (d) The curve ending: the model's peak over 2-3 s under CAM_END_LAT and over 0.5-2 s under the lateral limit, a curve
      actually driven (|lateral| >= CAM_IN_CURVE_LAT within CAM_IN_CURVE_TIME): the camera's own target is let go and the
      set speed goes back with SET+ holds (the merge boost on an on-ramp; else the restore to the anchor), never above the
      next map curve's plan (drive 56: a turn from a light, a short straight, then a loop 3-5 s ahead beyond the model's
      horizon: without the map ahead it ran into the loop too fast). On a link it is one more "ok" beside MergeTurnGate.
  (e) Never: a slower / closing car ahead, a red light / a stop / a planned stop, his steering, a lane change signal (not
      on a ramp), the camera's fault (no live data to trust), the curvature ahead rising, his SET- / CANCEL / brake
      (the controller's blocked rules), the Speed Limit / Map Curves Test modes (never a press there).
"""
import math
from collections import deque
from dataclasses import dataclass, field

DT_CTRL = 0.01
MS_PER_MPH = 0.44704

CAM_TAP_DT = 0.5                # s between two in-curve +1 taps
CAM_LAT_LKAS = 1.7              # m/s^2: the lateral a +1 may predict at most (stock LKAS + TI: the working limit ~2.0)
# (C1: the owner's 20 Hz rlogs, the steering command saturated in 47 % of frames at 2.0-2.3 m/s^2; the steer-angle lateral
# reads 0.1-0.2 low in curves against livePose) the measured lateral is taken CAM_MEAS_MARGIN higher in every comparison
CAM_MEAS_MARGIN = 0.15          # m/s^2
CAM_LAT_TI = 1.4                # m/s^2: ... with the TI alone (LKAS blocked, the camera's fault)
CAM_END_LAT = 1.0               # m/s^2: the model's peak over 2-3 s under this: the curve ends
CAM_PHANTOM_LAT = 1.0           # m/s^2
CAM_PHANTOM_TIME = 0.5          # s
CAM_IN_CURVE_LAT = 1.2          # m/s^2: a curve was driven: |lateral| this high ...
CAM_IN_CURVE_TIME = 2.0         # s ... within this
CAM_LANE_PROB = 0.3
CAM_MIN_SPEED = 25 * MS_PER_MPH  # m/s
CAM_TORQUE_RESERVE = 0.70       # median |torque command| over CAM_TORQUE_TIME under this (C3; was 0.85)
CAM_TORQUE_SAT = 0.99           # (C3) |torque| at or above this ...
CAM_TORQUE_SAT_SHARE = 0.10     # ... in at most this share of the last CAM_TORQUE_TIME
CAM_TORQUE_TIME = 1.0           # s
CAM_TIGHTEN_GAIN = 1.1
CAM_TIGHTEN_ADD = 0.2           # m/s^2
CAM_MAP_AHEAD_MIN = 10.0        # m: a map curve ahead farther than this caps every raise at its plan
CAM_MODEL_STALE = 0.3           # s: a model frame older than this is no model
# (replay of drive 74 t 410-423, exit 46's ramp curve, and t 1107-1117, the turnaround that was too fast: the 2 Hz model
# path under-read the curve tightening, a tap at its entry and the end's SET+ in the hairpin took the lateral to 2.3) no
# tap while the measured lateral is still rising more than CAM_RISE over the last CAM_RISE_TIME, and no end while the
# measured lateral is at the limit or the steering has no reserve
CAM_RISE = 0.3                  # m/s^2
CAM_RISE_TIME = 1.0             # s
# ... and a curve driven over CAM_OVER (too fast already: the turnaround's 3.6 m/s^2) gets neither taps nor the end's SET+
# for the rest of that curve slowdown: the normal restore only
# Review of round Q, B1 (closed loop: R130, 60 m straight, R90 -> curve 2 entered at 37.8 mph / 3.28 m/s^2 with the set
# back up): the end is checked again every frame while it lasts and cancelled at once (for the rest of that curve
# slowdown) when the model's 0.5-2 s peak reaches the limit, its 2-3 s peak CAM_END_LAT + CAM_END_HYST, the lateral rises
# (CAM_RISE), the steering reserve is gone, or the incoming curve target falls CAM_END_DROP under where it was when the end
# began. Without map curve data beyond the model's horizon (not on a ramp) the end's raise goes at most CAM_END_STEP over
# the speed it began at; the rest by the normal restore.
CAM_END_HYST = 0.3              # m/s^2
CAM_END_DROP = 1 * MS_PER_MPH   # m/s
CAM_END_STEP = 10 * MS_PER_MPH  # m/s
# Re-review of round Q, R1: the end may not start, and is cancelled, while any incoming target (the camera's, the map's,
# a map curve ahead's plan) is under the car's speed or the set + one step (the camera saw curve 2 at 30 mph with the car at
# 55: the end ran SET+ to his speed). CAM_END_STEP bounds every end, ramps included; curve_on going off ends it as a cancel.
# R2: the phantom release only with the map curve's apex known (starpilotPlan.mapCurveApexDistance): ahead within
# CAM_APEX_SEE m and CAM_APEX_TIME s with the model straight through it, or passed with the lateral under CAM_PHANTOM_LAT all
# the way; never on a link / ramp.
CAM_APEX_SEE = 110.0            # m (map_curves.VETO_SEE_MAX)
CAM_APEX_TIME = 2.0             # s
CAM_APEX_PAD = 20.0             # m past the apex the model must be straight too
CAM_END_CONFIRM = 0.5           # s the end's conditions hold before it acts (one model frame of "straight" is not an end)
# (C2) 2.5 on the margin-corrected lateral; the latch ends CAM_OVER_HOLD after the lateral is back under the limit (a loop's
# entry peaked ~2.8, its gentler middle may still get taps: drive 75, taps geometrically fine from ~1152)
CAM_OVER = 2.5                  # m/s^2
CAM_OVER_HOLD = 4.0             # s
CAM_PATH_SHORT = 0.8            # the model's path must reach this share of the farthest distance asked for

NO_CURVE = 1000.0


def model_peak(xs, kappas, v, t0, t1):
  """m/s^2: the peak |curvature| x v^2 of the model's plan between v x t0 and v x t1 m ahead (xs: m ahead of each point,
  kappas: 1/m). None when the path does not reach CAM_PATH_SHORT of v x t1 (no answer)."""
  if not xs or not kappas or v <= 0:
    return None
  d0, d1 = v * t0, v * t1
  if xs[-1] < CAM_PATH_SHORT * d1:
    return None
  best = 0.0
  for x, k in zip(xs, kappas, strict=False):
    if d0 <= x <= d1:
      if not (math.isfinite(k) and math.isfinite(x)):
        return None               # (review of round Q, M2) a broken model point: no answer, not "straight"
      best = max(best, abs(k))
  return best * v * v


def model_kappas(pos_x, rate_z, vel_x):
  """1/m at each point of modelV2's plan: orientationRate.z / velocity.x (the plan's own speed, at least 1 m/s)."""
  n = min(len(pos_x), len(rate_z), len(vel_x))
  ks = [rate_z[k] / max(vel_x[k], 1.0) if math.isfinite(vel_x[k]) else float("nan") for k in range(n)]
  if not all(math.isfinite(k) for k in ks) or not all(math.isfinite(x) for x in pos_x[:n]):
    return [], []                 # (M2) the model invalid this frame
  return list(pos_x[:n]), ks


@dataclass
class CamInputs:
  v_ego: float                    # m/s
  set_speed: float                # m/s (the cruise's)
  step: float = MS_PER_MPH        # m/s per tap
  lat: float = 0.0                # m/s^2, measured now (signed)
  model_x: list = field(default_factory=list)      # m ahead of each model point (None / empty: no fresh model)
  model_k: list = field(default_factory=list)      # 1/m at each
  lane_prob: float = 0.0          # the lower of our two lane lines' probabilities
  torque: float = 0.0             # |the lateral torque command| (-1..1)
  ti_only: bool = False           # the stock LKAS blocked (or the camera's fault): CAM_LAT_TI
  cam_fault: bool = False         # the camera reports a fault
  curve_on: bool = False          # a curve target (camera or map) holds the set speed down now
  map_in_curve: bool = False      # past the governing map curve's start (its distance 0)
  map_curve_id: tuple = ()        # that map curve (status, radius) for the phantom release
  map_ahead: float = NO_CURVE     # m/s: the plan speed of the next map curve more than CAM_MAP_AHEAD_MIN away
  cap: float = NO_CURVE           # m/s: the street / limit / merge target (every raise stays under it)
  lead_slow: bool = False         # a slower / closing car ahead
  stop: bool = False              # a red light, a stop episode, the model's planned stop
  steer_override: bool = False    # his hands on the wheel (carState.steeringPressed)
  blinker: bool = False
  on_ramp: bool = False           # a link road (a signal there is not a lane change)
  blocked: bool = False           # his SET- / CANCEL / brake (the controller's rules)
  test_mode: bool = False         # a Test mode governs: never press
  target: float = NO_CURVE        # m/s: the incoming curve target (camera or map) this frame (B1)
  map_covered: bool = False       # map curve data covers the road beyond the model's horizon (B1)
  apex_d: object = None           # m to the governing map curve's apex (negative: passed), None unknown (R2 / R3)
  cam_target: float = NO_CURVE    # m/s: the camera curve target alone (its plan sees past the 3 s window: R1)


@dataclass
class CamOutput:
  tap: bool = False               # one +1 tap now (goal: tap_goal)
  tap_goal: float = 0.0           # m/s
  ending: bool = False            # the curve ends (rule d): the camera's target let go, SET+ holds
  phantom: bool = False           # the governing map curve is released (rule b)
  keep: bool = False              # what earlier taps reached may stay (the live data still shows the margin at this speed)
  end_goal: float = NO_CURVE      # m/s: while ending, the most the end's raise may take the set to (CAM_END_STEP)
  end_cancel: bool = False        # the end was cancelled this frame (the wrapper drops its restore at once)
  why: str = ""                   # what held the tap back (logs)
  events: list = field(default_factory=list)       # (kind, data) for the event recorder


class CameraCurve:
  def __init__(self):
    self.frame = 0
    self.lat_hist = deque(maxlen=int(CAM_IN_CURVE_TIME / DT_CTRL))
    self.trq_hist = deque(maxlen=int(CAM_TORQUE_TIME / DT_CTRL))
    self.last_tap = -10_000
    self.ending = False
    self.phantom_id = None
    self.phantom_frames = 0
    self.prev_set = None
    self.over = False                # (CAM_OVER) this curve was driven too fast: no taps, no end
    self.over_clear = 0              # (C2) frames back under the limit since
    self.end_frames = 0              # (CAM_END_CONFIRM)
    self.end_done = False            # (B1) an end was cancelled in this curve slowdown: no new one until it is over
    self.end_v0 = 0.0                # m/s: the speed when the end began
    self.end_t0 = NO_CURVE           # m/s: the incoming curve target then

  def _never(self, i):
    if i.test_mode:
      return "test mode"
    if i.cam_fault:
      return "camera fault"
    if i.blocked:
      return "his buttons / brake"
    if i.steer_override:
      return "his steering"
    if i.lead_slow:
      return "car ahead"
    if i.stop:
      return "stop"
    if i.blinker and not i.on_ramp:
      return "signal"
    return ""

  def update(self, i: CamInputs) -> CamOutput:
    self.frame += 1
    out = CamOutput()
    lat_raw = abs(float(i.lat))
    lat = lat_raw + CAM_MEAS_MARGIN      # (C1; the tightening test keeps the raw one: there the margin is the unsafe side)
    self.lat_hist.append(lat)
    self.trq_hist.append(abs(float(i.torque)))
    lim = CAM_LAT_TI if i.ti_only else CAM_LAT_LKAS
    v = max(float(i.v_ego), 0.0)
    model = bool(i.model_x) and bool(i.model_k)
    guards = model and i.lane_prob >= CAM_LANE_PROB and v >= CAM_MIN_SPEED
    never = self._never(i)
    if not i.curve_on:
      if self.ending:
        self._cancel(out, "curve off", lat)     # (R1 c) as a cancel: no restore of ours left behind
      self.ending, self.over, self.end_done = False, False, False
    elif lat >= CAM_OVER:
      self.over, self.over_clear = True, 0
    elif self.over:
      self.over_clear = self.over_clear + 1 if lat < lim else 0
      if self.over_clear >= int(CAM_OVER_HOLD / DT_CTRL):
        self.over = False                     # (C2) CAM_OVER_HOLD back under the limit
    # (b) the phantom release: kept for that map curve until it changes / goes
    if self.phantom_id is not None and self.phantom_id != i.map_curve_id:
      self.phantom_id = None
    vq = max(i.set_speed, v)
    if i.map_curve_id != getattr(self, "count_id", None):
      self.count_id, self.phantom_frames, self.ph_lat = i.map_curve_id, 0, 0.0   # (another map curve: its own count)
    if i.map_in_curve:
      self.ph_lat = max(self.ph_lat, lat)
    pk_set = None
    if model and i.apex_d is not None and 0.0 <= i.apex_d <= min(CAM_APEX_SEE, CAM_APEX_TIME * vq) and vq > 0:
      pk_set = model_peak(i.model_x, i.model_k, vq, 0.0, (i.apex_d + CAM_APEX_PAD) / vq)   # (R2) straight through the apex
    passed = i.apex_d is not None and i.apex_d < 0.0 and self.ph_lat < CAM_PHANTOM_LAT
    straight = (guards and i.map_in_curve and not i.on_ramp and lat < CAM_PHANTOM_LAT and not i.cam_fault and
                ((pk_set is not None and pk_set < CAM_PHANTOM_LAT) or passed))
    self.phantom_frames = self.phantom_frames + 1 if straight else 0
    if (self.phantom_id is None and i.map_curve_id and self.phantom_frames >= int(CAM_PHANTOM_TIME / DT_CTRL)):
      self.phantom_id = i.map_curve_id
      out.events.append(("camPhantomRelease", {"lat": round(lat, 2), "model_peak_0p5_2s": round(pk_set, 2),
                                               "vEgo_mph": round(v / MS_PER_MPH, 1), "set_mph": round(i.set_speed / MS_PER_MPH, 1),
                                               "map_curve": str(i.map_curve_id), "lane_prob": round(i.lane_prob, 2)}))
    out.phantom = self.phantom_id is not None
    if self.over and not never:
      never = "too fast in this curve"
    if not guards or never or not i.curve_on:
      out.why = never or ("no model / lanes / speed" if not guards else "no curve")
      if self.ending and i.curve_on:
        self._cancel(out, never or "no model / lanes / speed", lat)
      self.ending = False
      return self._done(out, i)
    in_curve = max(self.lat_hist) >= CAM_IN_CURVE_LAT if self.lat_hist else False
    pk_now = model_peak(i.model_x, i.model_k, v, 0.5, 2.0)
    tightening = pk_now is None or pk_now >= lat_raw * CAM_TIGHTEN_GAIN + CAM_TIGHTEN_ADD
    pk_cur = model_peak(i.model_x, i.model_k, v, 0.5, 3.0)
    out.keep = pk_cur is not None and pk_cur < lim and lat < lim and not tightening
    # (d) the curve ending
    pk_end = model_peak(i.model_x, i.model_k, v, 2.0, 3.0)
    trq = sorted(self.trq_hist)[len(self.trq_hist) // 2] if self.trq_hist else 1.0
    sat = (sum(1 for q in self.trq_hist if q >= CAM_TORQUE_SAT) / len(self.trq_hist)) if self.trq_hist else 1.0
    no_reserve = trq >= CAM_TORQUE_RESERVE or sat > CAM_TORQUE_SAT_SHARE      # (C3)
    n_rise = int(CAM_RISE_TIME / DT_CTRL)
    rising = len(self.lat_hist) > 1 and lat - self.lat_hist[max(len(self.lat_hist) - 1 - n_rise, 0)] > CAM_RISE
    low_target = i.target < max(v, i.set_speed + i.step) - 1e-3 or i.map_ahead < v     # (R1 a)
    if self.ending:
      # (B1) checked again every frame
      why_c = ("target under the speed" if low_target else
               "model peak" if pk_now is None or pk_now >= lim else
               "curve ahead" if pk_end is None or pk_end >= CAM_END_LAT + CAM_END_HYST else
               "lateral rising" if rising or lat >= lim else
               "steering reserve" if no_reserve else
               "target dropping" if i.target < min(self.end_t0, i.set_speed) - CAM_END_DROP else "")
      if why_c:
        self._cancel(out, why_c, lat)
        self.ending = False
    end_ok = (not self.ending and not self.end_done and not low_target and in_curve and pk_end is not None and
              pk_end < CAM_END_LAT and
              pk_now is not None and pk_now < lim and lat < lim and not no_reserve and not rising)
    self.end_frames = self.end_frames + 1 if end_ok else 0
    if end_ok and self.end_frames >= int(CAM_END_CONFIRM / DT_CTRL):
      self.ending = True
      self.end_v0, self.end_t0 = v, i.target
      out.events.append(("camCurveEnd", {"lat": round(lat, 2), "model_peak_2_3s": round(pk_end, 2),
                                         "model_peak_0p5_2s": round(pk_now, 2), "vEgo_mph": round(v / MS_PER_MPH, 1),
                                         "set_mph": round(i.set_speed / MS_PER_MPH, 1), "lane_prob": round(i.lane_prob, 2),
                                         "map_ahead_mph": round(min(i.map_ahead, NO_CURVE) / MS_PER_MPH, 1),
                                         "ti_only": i.ti_only}))
    if self.ending:
      out.ending = True
      out.end_goal = min(self.end_v0 + CAM_END_STEP, i.map_ahead)   # (R1 b) every end, ramps included
      return self._done(out, i)
    # (c) the in-curve +1 tap
    v1 = v + MS_PER_MPH
    pk1 = model_peak(i.model_x, i.model_k, v1, 0.5, 3.0)
    pred = None if pk1 is None or v <= 0 else max(lat * (v1 / v) ** 2, pk1)
    goal = i.set_speed + i.step
    set_steps = int(round(i.set_speed / i.step))
    why = ("not in the curve" if not (i.map_in_curve or lat >= CAM_PHANTOM_LAT) else
           "no prediction" if pred is None else
           "lateral" if pred >= lim else
           "set above the speed" if set_steps + 1 > int(round(v / i.step)) + 1 else
           "tightening" if tightening or rising else
           "steering reserve" if no_reserve else
           "cap" if goal > i.cap + 1e-3 else
           "map curve ahead" if goal > i.map_ahead + 1e-3 else
           "camera curve ahead" if goal > i.cam_target + 1e-3 else
           "wait" if self.frame - self.last_tap < int(CAM_TAP_DT / DT_CTRL) else "")
    out.why = why
    if not why:
      self.last_tap = self.frame
      out.tap, out.tap_goal = True, goal
      out.events.append(("camCurveTap", {"lat": round(lat, 2), "pred_lat": round(pred, 2), "lim": lim,
                                         "model_peak_0p5_2s": round(pk_now, 2), "torque_med": round(trq, 2),
                                         "vEgo_mph": round(v / MS_PER_MPH, 1), "set_mph": round(i.set_speed / MS_PER_MPH, 1),
                                         "goal_mph": round(goal / MS_PER_MPH, 1), "lane_prob": round(i.lane_prob, 2),
                                         "cap_mph": round(min(i.cap, NO_CURVE) / MS_PER_MPH, 1),
                                         "map_ahead_mph": round(min(i.map_ahead, NO_CURVE) / MS_PER_MPH, 1)}))
    return self._done(out, i)

  def _cancel(self, out, why, lat):
    self.end_done = True
    out.end_cancel = True
    out.events.append(("camCurveEndCancel", {"why": why, "lat": round(lat, 2)}))

  def _done(self, out, i):
    self.prev_set = i.set_speed
    return out
