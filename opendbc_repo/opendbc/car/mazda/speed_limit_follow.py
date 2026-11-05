"""
Posted speed limit rising (Mazda Gen1, stock MRCC): when the road's limit goes up and the driver's set speed was the
old road's limit, the new limit becomes the driver's speed, and the set speed controller (curve_slowdown.
CurveSlowdownLogic) brings the set speed up to it with its restore rule: a gap of 5 mph or less by 1 mph taps, more by
one SET+ hold to a multiple of 5 at least one quantum below the new limit, then taps (owner, drive 4c: "if the
difference is not too much like 5 mph, increment 1 mph per click, but if we are merging into a highway increase in
press and hold mode").

Drive 4c, tag 3: a 45 mph road, a stop at a red light (set lowered 45 -> 35 for a stopped car, the driver's speed 45),
a turn onto a 50 mph road: starpilotPlan.slcSpeedLimit / slcMapSpeedLimit 45 -> 50 at t 776, cruise re-engaged at
t 778, our restore brought the set speed back to the old 45; the driver pressed SET+ to 50 himself at t 796.

Raise only: a lower limit never lowers the set speed here (a possible later option). A speed the driver chose himself
(not the old limit) is left alone. The limit comes from StarPilot's speed limit fields in starpilotPlan, which on this
car (no openpilot longitudinal) are only filled while Show Speed Limits is on (display-only mode: the raw map /
dashboard limit). Setting MazdaSpeedLimitFollow: 0 off, 1 test (logs what it would do), 2 on.

No openpilot dependencies, so it can be unit tested.
"""
from dataclasses import dataclass

DT_CTRL = 0.01
MS_PER_MPH = 0.44704
MS_PER_KPH = 1 / 3.6

OFF, TEST, ON = 0, 1, 2

LIMIT_STABLE_TIME = 3.0      # s the new value must be reported unchanged (map matching flaps between roads at junctions)
LIMIT_OLD_MAX_AGE = 30.0     # s: the old limit counts as "what the driver matched" only this long after it was last seen
LIMIT_OFFER_TIME = 30.0      # s an offer (old -> new) stays open (cruise may be off meanwhile: drive 4c, t 776-778)
# slcSpeedLimitSource values that are a posted limit; never "None" (with the Speed Limit Controller active that can be
# the set speed itself), "Previous Limit" (a remembered value) or "Vision" (the sign reader, not checked on this car)
LIMIT_SOURCES = ("Map Data", "Dashboard", "Mapbox")
LIMIT_MAP_SOURCES = ("Map Data", "Mapbox")
LIMIT_GPS_TRUST_MAX = 1      # starpilotPlan.mapGpsTrust (0 ok, 1 shaky, 2+ low / paused) for a map limit
LIMIT_MAX_JUMP_MPH = 30      # 45 -> 75 (a street onto a freeway) is the largest plausible rise
LIMIT_MAX_MPH = 80           # never set the cruise above this
LIMIT_MULTIPLE_MPH = 5       # US limits are multiples of 5 mph ...
LIMIT_MULTIPLE_KPH = 10      # ... metric ones of 10 km/h
LIMIT_WHOLE_TOL = 0.15       # steps: the limit must be a whole multiple within this (a km/h limit on an mph dash is not)


def _frames(seconds):
  return int(round(seconds / DT_CTRL))


@dataclass
class LimitInputs:
  limit: float              # m/s, starpilotPlan.slcSpeedLimit (0 unknown)
  source: str               # starpilotPlan.slcSpeedLimitSource
  gps_trust: int            # starpilotPlan.mapGpsTrust
  driver_speed: float       # m/s: the controller's driver speed (anchor, else the set speed); 0 when cruise is off
  cruise_enabled: bool


@dataclass
class Offer:
  """One (old -> new) raise the controller may take."""
  old: float
  new: float
  source: str
  frame: int


class SpeedLimitFollower:
  def __init__(self, is_metric):
    self.step = MS_PER_KPH if is_metric else MS_PER_MPH
    self.multiple = LIMIT_MULTIPLE_KPH if is_metric else LIMIT_MULTIPLE_MPH
    self.frame = 0
    self.stable = None             # the limit (in steps) reported unchanged for LIMIT_STABLE_TIME
    self.stable_last_seen = -10_000
    self.candidate = None
    self.candidate_frames = 0
    self.offer = None
    self.events = []               # (kind, dict) for the per-event log, drained by the caller

  def _steps(self, v):
    return v / self.step

  def _plausible(self, old, new, source, gps_trust):
    """'' or why a rise old -> new (steps) is not taken."""
    unit_mph = self.step / MS_PER_MPH
    if (new - old) * unit_mph > LIMIT_MAX_JUMP_MPH + 1e-6:
      return f"jump {new - old} > {LIMIT_MAX_JUMP_MPH} mph"
    if new * unit_mph > LIMIT_MAX_MPH + 1e-6:
      return f"limit {new} > {LIMIT_MAX_MPH} mph"
    if new % self.multiple or old % self.multiple:
      return f"not a multiple of {self.multiple}"
    if source in LIMIT_MAP_SOURCES and gps_trust > LIMIT_GPS_TRUST_MAX:
      return f"GPS trust {gps_trust}"
    return ""

  def update(self, i: LimitInputs):
    """m/s to raise the driver's speed to this frame, else 0.0."""
    self.frame += 1
    reported = None
    if i.limit >= 1.0 and i.source in LIMIT_SOURCES:
      x = self._steps(i.limit)
      if abs(x - round(x)) <= LIMIT_WHOLE_TOL:
        reported = int(round(x))
    if reported is not None and reported == self.stable:
      self.stable_last_seen = self.frame
    if reported == self.candidate:
      self.candidate_frames += 1
    else:
      self.candidate, self.candidate_frames = reported, 0
    if self.candidate is not None and self.candidate != self.stable and self.candidate_frames >= _frames(LIMIT_STABLE_TIME):
      old, old_seen, new = self.stable, self.stable_last_seen, self.candidate
      self.stable, self.stable_last_seen = new, self.frame
      if old is not None and new > old:
        why = "old limit not seen recently" if self.frame - old_seen > _frames(LIMIT_OLD_MAX_AGE) else \
              self._plausible(old, new, i.source, i.gps_trust)
        if why:
          self.offer = None
          self.events.append(("speedLimitNotFollowed", {"old": old, "new": new, "source": i.source, "reason": why}))
        else:
          self.offer = Offer(old, new, i.source, self.frame)
      else:
        self.offer = None            # a lower (or first) limit: nothing to raise; an open offer is void
    o = self.offer
    if o is None:
      return 0.0
    if self.frame - o.frame > _frames(LIMIT_OFFER_TIME):
      self.offer = None
      self.events.append(("speedLimitNotFollowed", {"old": o.old, "new": o.new, "source": o.source,
                                                    "reason": "cruise not back at the old limit in time"}))
      return 0.0
    if not i.cruise_enabled or i.driver_speed <= 0:
      return 0.0                     # cruise off: wait (drive 4c: the limit changed 2 s before cruise came back)
    d = self._steps(i.driver_speed)
    if abs(d - o.old) < 0.5:
      return o.new * self.step
    self.offer = None
    if abs(d - o.new) >= 0.5:        # the driver chose another speed: theirs
      self.events.append(("speedLimitNotFollowed", {"old": o.old, "new": o.new, "source": o.source,
                                                    "reason": f"driver's speed {round(d)} is not the old limit"}))
    return 0.0

  def mark_adopted(self):
    """The controller took the offer (or Test mode logged it): close it."""
    self.offer = None

  @property
  def stable_speed(self):
    return (self.stable or 0) * self.step

  @property
  def offer_speed(self):
    return self.offer.new * self.step if self.offer is not None else 0.0
