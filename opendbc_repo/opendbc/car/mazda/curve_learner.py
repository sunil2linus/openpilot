"""
Curve learner for the Mazda curve slowdown (LOG ONLY for now: what it learns is written to a file and the
per-curve log, it does not change the curve target yet).

It splits every curve driven at 25 mph or more into one record and learns two numbers (m/s^2 of lateral
acceleration):

  * capability: how much curve openpilot steers on this car without help. Starts at the curve target's
    limit (3.0 with a Torque Interceptor, the car's steering limit without one). Rises slowly towards the
    curves openpilot took cleanly (engaged, no driver torque, steering not saturated); drops at once to 90 %
    of the lateral acceleration where the driver had to help or the steering saturated, when that happened
    near the limit (a small nudge on a gentle curve says nothing about the limit).
  * preference: how fast the driver likes to take curves. Learned only from curves where the driver chose
    the speed: cruise off (manual speed), or the gas pressed in the curve. A brake or SET- tap in a curve
    with cruise on is a "too fast" sample (90 % of the lateral acceleration at that moment). Curves where our
    slowdown had lowered the set speed are never used for the preference (our numbers, not the driver's).

  suggested = min(capability, preference), the number a later version would plan curves at.

Bounds: everything stays within [LAT_MIN, LAT_MAX]. No openpilot dependencies, so it can be unit tested.
"""
from dataclasses import dataclass, field

DT_CTRL = 0.01
MS_PER_MPH = 0.44704

LAT_MIN = 1.0                 # m/s^2: never learn below / above these
LAT_MAX = 3.5
CURVE_START = 1.0             # |lateral accel| that starts a curve
CURVE_END = 0.6               # ... and below which it is over (for CURVE_END_TIME)
CURVE_END_TIME = 0.5          # s
MIN_CURVE_TIME = 0.5          # s above CURVE_START for a curve to count
MIN_SPEED = 25.0 * MS_PER_MPH
ENGAGED_SHARE = 0.9           # share of the curve with openpilot steering for a capability sample
CRUISE_OFF_SHARE = 0.9        # share of the curve with cruise off for a "manual speed" preference sample
GAS_TIME = 0.3                # s of gas in the curve for a preference sample with cruise on
SATURATED_STEER = 0.95        # |actuators.steer| at or above this is the steering limit
SATURATED_TIME = 0.3          # s
NEAR_LIMIT = 0.75             # a help / saturation counts for the capability only at >= this share of it
CAP_RISE = 0.1                # share of the gap closed by one clean curve above the capability
HELP_FACTOR = 0.9
TOO_FAST_FACTOR = 0.9
PREF_SAMPLES = 50             # preference = mean of the last PREF_SAMPLES samples, blended from the start
                              # value by n / PREF_SAMPLES
PERCENTILE = 90               # a curve's lateral acceleration (FrogPilot's learner uses the same)


def _percentile(values, pct):
  s = sorted(values)
  if not s:
    return 0.0
  k = (len(s) - 1) * pct / 100.0
  lo = int(k)
  hi = min(lo + 1, len(s) - 1)
  return s[lo] + (s[hi] - s[lo]) * (k - lo)


def _clip(x):
  return min(max(x, LAT_MIN), LAT_MAX)


@dataclass
class LearnerInputs:
  v_ego: float              # m/s
  lat_accel: float          # m/s^2, measured (yaw rate * speed); NaN/None when not valid
  lat_active: bool          # openpilot is steering
  steering_pressed: bool    # driver torque on the wheel
  steer_cmd: float          # |actuators.steer|, 0..1
  cruise_enabled: bool
  gas_pressed: bool
  brake_pressed: bool
  blinker: bool
  our_offset: int = 0       # steps our slowdown had the set speed lowered
  driver_set_minus: bool = False


@dataclass
class Curve:
  start_frame: int
  samples: list = field(default_factory=list)      # |lat| while in the curve
  frames: int = 0
  above: int = 0                                    # frames above CURVE_START
  engaged: int = 0
  cruise_off: int = 0
  gas: int = 0
  speed_sum: float = 0.0
  peak: float = 0.0
  peak_speed: float = 0.0
  blinker: bool = False
  slowed: bool = False                              # our slowdown was active in this curve
  help_lat: float = None                            # |lat| when the driver first had to steer (openpilot engaged)
  sat_frames: int = 0
  sat_lat: float = None                             # |lat| when the steering first stayed saturated
  too_fast_lat: float = None                        # |lat| at the first brake / SET- with cruise on
  low_frames: int = 0
  was_slow: bool = False                            # dropped below MIN_SPEED (turn at an intersection)


class CurveLearner:
  def __init__(self, start_lat, state=None):
    self.start_lat = _clip(start_lat)
    self.capability = self.start_lat
    self.pref_samples = []
    self.counts = {"curves": 0, "capability_up": 0, "capability_down": 0, "preference": 0, "too_fast": 0, "ignored": 0}
    if state:
      self.load(state)
    self.curve = None
    self.frame = 0

  # ---- persistence ----
  def load(self, state):
    try:
      self.capability = _clip(float(state.get("capability", self.capability)))
      self.pref_samples = [_clip(float(x)) for x in state.get("preference_samples", [])][-PREF_SAMPLES:]
      for k in self.counts:
        self.counts[k] = int(state.get("counts", {}).get(k, 0))
    except (TypeError, ValueError, AttributeError):
      pass

  def state(self):
    return {"version": 1, "start_lat": self.start_lat, "capability": round(self.capability, 3),
            "preference": round(self.preference, 3), "preference_confidence": round(self.confidence, 2),
            "suggested": round(self.suggested, 3), "preference_samples": [round(x, 3) for x in self.pref_samples],
            "counts": dict(self.counts)}

  @property
  def confidence(self):
    return min(len(self.pref_samples) / PREF_SAMPLES, 1.0)

  @property
  def preference(self):
    if not self.pref_samples:
      return self.start_lat
    mean = sum(self.pref_samples) / len(self.pref_samples)
    return _clip(self.start_lat + self.confidence * (mean - self.start_lat))

  @property
  def suggested(self):
    return min(self.capability, self.preference)

  # ---- per frame ----
  def update(self, i: LearnerInputs):
    """Returns a finished curve record (dict) when a curve ends, else None."""
    self.frame += 1
    lat = i.lat_accel
    valid = lat is not None and lat == lat
    a = abs(lat) if valid else 0.0
    c = self.curve
    if c is None:
      if valid and a >= CURVE_START and i.v_ego >= MIN_SPEED:
        c = self.curve = Curve(self.frame)
      else:
        return None
    c.frames += 1
    c.samples.append(a)
    c.speed_sum += i.v_ego
    if a >= CURVE_START:
      c.above += 1
    if a > c.peak:
      c.peak, c.peak_speed = a, i.v_ego
    c.engaged += i.lat_active
    c.cruise_off += not i.cruise_enabled
    c.gas += i.gas_pressed
    c.blinker |= i.blinker
    c.slowed |= i.our_offset > 0
    c.was_slow |= i.v_ego < MIN_SPEED
    if i.lat_active and i.steering_pressed and c.help_lat is None:
      c.help_lat = a
    c.sat_frames = c.sat_frames + 1 if (i.lat_active and i.steer_cmd >= SATURATED_STEER) else 0
    if c.sat_frames >= int(SATURATED_TIME / DT_CTRL) and c.sat_lat is None:
      c.sat_lat = a
    if i.cruise_enabled and (i.brake_pressed or i.driver_set_minus) and c.too_fast_lat is None:
      c.too_fast_lat = a
    c.low_frames = c.low_frames + 1 if a < CURVE_END else 0
    if c.low_frames >= int(CURVE_END_TIME / DT_CTRL) or not valid:
      self.curve = None
      return self._finish(c)
    return None

  def _finish(self, c):
    n = max(c.frames - c.low_frames, 1)
    samples = c.samples[:n]
    lat = _percentile(samples, PERCENTILE)
    rec = {"duration_s": round(n * DT_CTRL, 2), "lat_p90": round(lat, 2), "lat_peak": round(c.peak, 2),
           "speed_at_peak_mph": round(c.peak_speed / MS_PER_MPH, 1), "mean_speed_mph": round(c.speed_sum / max(c.frames, 1) / MS_PER_MPH, 1),
           "engaged_share": round(c.engaged / max(c.frames, 1), 2), "cruise_off_share": round(c.cruise_off / max(c.frames, 1), 2),
           "gas_s": round(c.gas * DT_CTRL, 2), "blinker": c.blinker, "our_slowdown": c.slowed,
           "driver_help_lat": None if c.help_lat is None else round(c.help_lat, 2),
           "saturated_lat": None if c.sat_lat is None else round(c.sat_lat, 2),
           "too_fast_lat": None if c.too_fast_lat is None else round(c.too_fast_lat, 2),
           "capability_before": round(self.capability, 3), "preference_before": round(self.preference, 3)}
    used = []
    ignored = None
    if c.above * DT_CTRL < MIN_CURVE_TIME:
      ignored = "too short"
    elif c.blinker:
      ignored = "turn signal"
    elif c.was_slow:
      ignored = "below 25 mph (turn)"
    if ignored is None:
      # capability: how much openpilot steers without help
      if c.engaged / max(c.frames, 1) >= ENGAGED_SHARE:
        problem = min(x for x in (c.help_lat, c.sat_lat, LAT_MAX * 2) if x is not None)
        if problem < LAT_MAX * 2 and problem >= NEAR_LIMIT * self.capability:
          new = _clip(min(self.capability, HELP_FACTOR * problem))
          if new < self.capability:
            self.capability = new
            self.counts["capability_down"] += 1
            used.append("capability down")
        elif c.help_lat is None and c.sat_lat is None and lat > self.capability:
          self.capability = _clip(self.capability + CAP_RISE * (lat - self.capability))
          self.counts["capability_up"] += 1
          used.append("capability up")
      # preference: only curves where the driver chose the speed, never ones our slowdown changed
      if not c.slowed:
        manual = c.cruise_off / max(c.frames, 1) >= CRUISE_OFF_SHARE
        if c.too_fast_lat is not None and c.too_fast_lat >= CURVE_START:
          self._add_pref(TOO_FAST_FACTOR * c.too_fast_lat)
          self.counts["too_fast"] += 1
          used.append("preference (too fast)")
        elif manual or c.gas * DT_CTRL >= GAS_TIME:
          self._add_pref(lat)
          self.counts["preference"] += 1
          used.append("preference (manual speed)" if manual else "preference (gas)")
    if not used:
      self.counts["ignored"] += 1
    self.counts["curves"] += 1
    rec.update(used=", ".join(used) or ("ignored: " + ignored if ignored else "nothing to learn"),
               capability=round(self.capability, 3), preference=round(self.preference, 3),
               preference_confidence=round(self.confidence, 2), suggested=round(self.suggested, 3))
    return rec

  def _add_pref(self, x):
    self.pref_samples = (self.pref_samples + [_clip(x)])[-PREF_SAMPLES:]
