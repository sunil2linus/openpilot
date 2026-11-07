"""
Posted speed limit follow (Mazda Gen1, stock MRCC). openpilot does not control gas or brake on this car: the set speed
controller (curve_slowdown.CurveSlowdownLogic) changes the stock cruise's set speed with SET-/SET+ presses. The posted
limit only moves the driver's speed (the controller's anchor, the speed its restores go back to):

1. A higher limit: when the limit rises and the driver's speed is at the old limit or up to one limit multiple above
   it (5 mph, 10 km/h), the new limit becomes the driver's speed. Always the new limit itself, never the limit plus
   the driver's offset (drive 52: at 50 on a 45 road, he chose 55 on the 55 road). The controller brings the set speed
   up with its restore rule: a gap of 5 mph or less by 1 mph taps, more by one SET+ hold to a multiple of 5 at least
   one quantum below the new limit, then taps (owner, drive 4c: "if the difference is not too much like 5 mph,
   increment 1 mph per click, but if we are merging into a highway increase in press and hold mode").
2. A first limit: when no posted limit was reported for LIMIT_FIRST_GAP (or none since the drive started) and the old
   limit was not seen for LIMIT_OLD_MAX_AGE, a new limit up to LIMIT_FIRST_MAX_MPH (km/h: _KPH) above the driver's
   speed becomes the driver's speed; only if the driver's speed was already that value when the new limit appeared,
   and cruise was on from then until the limit was steady.
3. A lower limit: when the driver's speed is in the band of rule 1 for the old limit, a restore still pending (the set
   speed below the driver's speed after a stopped car, the red light head start or a curve) stops at the new limit:
   the driver's speed comes down to the new limit, or to the set speed if that is already higher. Never a SET- for a
   lower limit: a set speed at or above the new limit stays where it is, and with no restore pending nothing happens.
   If the lower limit goes again before the restore got there (the cross road's limit matched while stopped at a
   light: drives 4c, 4d), the cap goes with it; once taken, a rise back is rule 1 from the lower limit.
4. Undo (map glitches): within LIMIT_UNDO_TIME of a raise the controller took, a lower limit at or below the old one
   (after a first limit: any lower one) takes the raise back: the driver's speed goes back to what it was, and if the
   driver has not touched his speed since, the controller lowers the set speed back to it (undoing our own presses:
   the only SET- the posted limit can cause). Otherwise (his own change, brake, cancel, a stop or curve slowdown on)
   it only works like rule 3's cap at his old speed.

Raises (rules 1, 2) are harder than caps: the new limit must be steady for LIMIT_RAISE_STABLE_TIME (caps and the undo:
LIMIT_STABLE_TIME), the driver's speed must be known (cruise on) at that moment, and a first limit or a rise of more
than LIMIT_BIG_RAISE_MPH (km/h: _KPH) needs GPS trust 0 on every frame of that steady time.

Drive 4c, tag 3: a 45 mph road, a stop at a red light (set lowered 45 -> 35 for a stopped car, the driver's speed 45),
a turn onto a 50 mph road: starpilotPlan.slcSpeedLimit / slcMapSpeedLimit 45 -> 50 at t 776, cruise re-engaged at
t 778, our restore brought the set speed back to the old 45; the driver pressed SET+ to 50 himself at t 796.

Drive 52 (all three tagged by the owner; slcSpeedLimit from Map Data, GPS trust ok). He drives at the limit or a bit
over: 35 on an unmapped street, 45 then 50 on a 45 road, 55 on a 55 road.
 * Tag 318 (rule 1): at 50 on a 45 road (he set 50 himself at t 230) the limit went 45 -> 55 at t 310.3. Not followed
   then (his speed was not exactly the old limit); he raised 50 -> 55 by hand at t 321.
 * Tag 211 (rule 2): no limit since the start (slc 0 from t 5.7 to 187.3, residential streets, set 35 chosen on the
   unmapped street), a turn onto a 45 road: slc 45 at t 187.3. A stop slowdown ran t 191-206 (cruise off t 196-200.2 by
   his brake, our restore back to 35 by t 206). Not followed (a first limit); he held SET+ 35 -> 45 himself at t 212.8.
 * Tag 450 (rule 3): on a 50 road at 50 our red light head start lowered the set 50 -> 45, he braked to a stop (cruise
   off t 403-442) and meanwhile the limit fell 50 -> 45 (t 440.3). He resumed at t 442.4 (stored set 45) and our restore
   took the set back UP to 50, his old speed, on the 45 road (t 444-446); he pressed SET- back to 45 at t 448.

The driver's own choice wins: a speed outside rule 1's band, a speed he set after a first limit appeared, or any change
of his while a raise is offered leaves the set speed alone. An offer that is not taken within LIMIT_OFFER_TIME ends.
The limit comes from StarPilot's speed limit fields in starpilotPlan, which on this car (no openpilot longitudinal) are
only filled while Show Speed Limits is on (display-only mode: the raw map / dashboard limit). Setting
MazdaSpeedLimitFollow: 0 off, 1 test (logs what it would do), 2 on.

No openpilot dependencies, so it can be unit tested.
"""
from dataclasses import dataclass

DT_CTRL = 0.01
MS_PER_MPH = 0.44704
MS_PER_KPH = 1 / 3.6

OFF, TEST, ON = 0, 1, 2

LIMIT_STABLE_TIME = 3.0      # s the new value must be reported unchanged (map matching flaps between roads at junctions)
# A higher limit (rules 1, 2) must be reported unchanged this long before it is offered (review of drive 52's rules, the
# measured car: a wrong road matched for 3.5 s, 60 on a 45 road, took the set speed to 59-60 and nothing took it back).
# Drive 52's tags 211 and 318 stayed 123 s and 30 s. Lower limits (rule 3, the undo) keep LIMIT_STABLE_TIME.
LIMIT_RAISE_STABLE_TIME = 5.0  # s
LIMIT_OLD_MAX_AGE = 30.0     # s: the previous limit counts as the old one only this long after it was last seen
# Drive 4d (tag 2, t 740-808): 45 mph road, a red light, a right turn onto a 55 mph road; while waiting at the light the
# map matched the cross road for 28 s (45 -> 40 at t 759, then 40 -> 55 at t 787). The driver's speed (45) was not the
# 40 just before the 55, so nothing was offered. The old limit is now the last limit the driver's speed matched (seen
# within LIMIT_MATCH_MAX_AGE, a long red light included), whatever came in between. "Matched" is rule 1's band; when
# the driver's speed is in the band of two limits, the higher one (the one he is closest to) is kept.
LIMIT_MATCH_MAX_AGE = 90.0   # s
LIMIT_OFFER_TIME = 30.0      # s an offer (old -> new) stays open (cruise may be off meanwhile: drive 4c, t 776-778)
# slcSpeedLimitSource values that are a posted limit; never "None" (with the Speed Limit Controller active that can be
# the set speed itself), "Previous Limit" (a remembered value) or "Vision" (the sign reader, not checked on this car)
LIMIT_SOURCES = ("Map Data", "Dashboard", "Mapbox")
LIMIT_MAP_SOURCES = ("Map Data", "Mapbox")
LIMIT_GPS_TRUST_MAX = 1      # starpilotPlan.mapGpsTrust (0 ok, 1 shaky, 2+ low / paused) for a map limit
# A first limit (rule 2) and a rise of more than LIMIT_BIG_RAISE_MPH / _KPH need trust 0 on every frame of the steady
# time that offers it (drive 52: 0 throughout for tags 211, 318 and 450; drives 4f t 86.6 and 50 t 84.3, first limits
# with trust 1, were not followed by the old rules either)
LIMIT_FIRM_TRUST_MAX = 0
LIMIT_BIG_RAISE_MPH = 10     # (old -> new limit) 45 -> 55 is not big, 45 -> 60 is
LIMIT_BIG_RAISE_KPH = 16
LIMIT_MAX_JUMP_MPH = 30      # 45 -> 75 (a street onto a freeway) is the largest plausible change, up or down
LIMIT_MAX_MPH = 80           # never set the cruise above this
LIMIT_MULTIPLE_MPH = 5       # US limits are multiples of 5 mph ...
LIMIT_MULTIPLE_KPH = 10      # ... metric ones of 10 km/h
LIMIT_WHOLE_TOL = 0.15       # steps: the limit must be a whole multiple within this (a km/h limit on an mph dash is not)
# Rule 1's band (drive 52, tag 318): the driver's speed counts as "at the old limit" from the old limit up to this many
# limit multiples above it (5 mph, 10 km/h); he drove 50 on a 45 road and chose 55 when it became a 55 road.
LIMIT_BAND = 1
# Rule 2 (drive 52, tag 211): 182 s with no limit (residential streets), then a 45 road; his speed 35 (set on the
# unmapped street), he chose 45 by hand 25 s later. A first limit needs this long with no posted limit reported (a
# junction's short flap to no limit is a normal change: the old limit was seen just before) ...
LIMIT_FIRST_GAP = 10.0       # s
LIMIT_FIRST_MAX_MPH = 15     # ... and is followed only this far above the driver's speed (35 -> 45 on drive 52) ...
LIMIT_FIRST_MAX_KPH = 25     # ... (metric)
# Rule 4 (review: a raise from a map glitch was never taken back): a lower limit whose first frame comes this soon after
# the controller took our raise. Drive 52: after tag 318's raise (45 -> 55 at t 310.3) the next lower limit (35) came
# 39.5 s after the limit rose; the 45 road after tag 211 stayed 123 s.
LIMIT_UNDO_TIME = 30.0       # s


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
  """One change of the driver's speed the controller may take: "raise" (rule 1), "first" (rule 2, old None), "cap"
  (rule 3, a lower limit: open while that limit stays and the driver's speed is unchanged) or "undo" (rule 4: old is
  the limit we raised to, new the driver's speed before our raise)."""
  old: object               # steps, None for a first limit
  new: int
  source: str
  frame: int
  kind: str = "raise"
  driver: object = None     # steps: the driver's speed it was offered for (a cap: None while cruise is off)


class SpeedLimitFollower:
  def __init__(self, is_metric):
    self.step = MS_PER_KPH if is_metric else MS_PER_MPH
    self.multiple = LIMIT_MULTIPLE_KPH if is_metric else LIMIT_MULTIPLE_MPH
    self.first_max = LIMIT_FIRST_MAX_KPH if is_metric else LIMIT_FIRST_MAX_MPH
    self.big_raise = LIMIT_BIG_RAISE_KPH if is_metric else LIMIT_BIG_RAISE_MPH
    self.frame = 0
    self.stable = None             # the limit (in steps) reported unchanged for LIMIT_STABLE_TIME
    self.stable_last_seen = -10_000
    self.candidate = None
    self.candidate_frames = 0
    self.candidate_trust = 0       # the highest GPS trust value (map sources) while the candidate has been reported
    self.seen = {}                 # {limit (steps): last frame reported}, for rule 2's quiet time
    self.matched = None            # the last stable limit the driver's speed was at (steps, rule 1's band) ...
    self.matched_seen = -10_000    # ... and when
    self.driver = None             # the driver's speed (steps) last seen with cruise on ...
    self.driver_since = -10_000    # ... and the frame it last changed
    self.cruise_off_frame = 0      # the last frame cruise was off (or the driver's speed unknown)
    self.rise = None               # (old, new, first frame): a higher limit waiting for LIMIT_RAISE_STABLE_TIME
    self.offer = None              # an open raise (rule 1 / 2)
    self.cap = None                # rule 3: open while the lower limit stays and the driver's speed is unchanged
    self.raised = None             # rule 4: the last raise the controller took (an Offer, frame = when it took it)
    self.undo = None               # rule 4: open until the controller has dealt with it
    self.events = []               # (kind, dict) for the per-event log, drained by the caller

  def _steps(self, v):
    return v / self.step

  def _in_band(self, d, old):
    """The driver's speed d (steps) is at the old limit or up to LIMIT_BAND multiples above it."""
    return -0.5 < d - old < LIMIT_BAND * self.multiple + 0.5

  def _plausible(self, old, new, source, gps_trust, firm=False):
    """'' or why a change old -> new (steps; old None for a first limit) is not taken. firm: a first limit or a big
    rise, GPS trust LIMIT_FIRM_TRUST_MAX."""
    unit_mph = self.step / MS_PER_MPH
    if old is not None and abs(new - old) * unit_mph > LIMIT_MAX_JUMP_MPH + 1e-6:
      return f"jump {abs(new - old)} > {LIMIT_MAX_JUMP_MPH} mph"
    if new * unit_mph > LIMIT_MAX_MPH + 1e-6:
      return f"limit {new} > {LIMIT_MAX_MPH} mph"
    if new % self.multiple or (old is not None and old % self.multiple):
      return f"not a multiple of {self.multiple}"
    if source in LIMIT_MAP_SOURCES and gps_trust > (LIMIT_FIRM_TRUST_MAX if firm else LIMIT_GPS_TRUST_MAX):
      return f"GPS trust {gps_trust}" + (" (a first limit or a big rise needs 0)" if firm else "")
    return ""

  def _not_followed(self, old, new, source, reason):
    self.events.append(("speedLimitNotFollowed", {"old": old, "new": new, "source": source, "reason": reason}))

  def update(self, i: LimitInputs):
    """m/s to raise the driver's speed to this frame, else 0.0. A lower limit's cap (rule 3) is in `cap`."""
    self.frame += 1
    reported = None
    if i.limit >= 1.0 and i.source in LIMIT_SOURCES:
      x = self._steps(i.limit)
      if abs(x - round(x)) <= LIMIT_WHOLE_TOL:
        reported = int(round(x))
    d = self._steps(i.driver_speed) if (i.cruise_enabled and i.driver_speed > 0) else None
    if d is None:
      self.cruise_off_frame = self.frame
    if d is not None and (self.driver is None or abs(d - self.driver) >= 0.5):
      self.driver, self.driver_since = d, self.frame
    c = self.cap
    if c is not None and d is not None:
      if c.driver is None:           # cruise was off when the lower limit came: his speed now, the first frame back
        if self._in_band(d, c.old):
          c.driver = d
        else:
          self.cap = None
      elif abs(d - c.driver) >= 0.5:
        self.cap = None              # the driver's speed came down (the cap taken) or the driver changed it
    if self.undo is not None and self.frame - self.undo.frame > _frames(LIMIT_OFFER_TIME):
      self.undo = None
    if reported is not None and reported == self.stable:
      self.stable_last_seen = self.frame
      m = self.matched
      keep = (m is not None and m > self.stable and d is not None and self._in_band(d, m) and
              self.frame - self.matched_seen <= _frames(LIMIT_MATCH_MAX_AGE))
      if d is not None and self._in_band(d, self.stable) and not keep:
        self.matched, self.matched_seen = self.stable, self.frame
    trust = i.gps_trust if i.source in LIMIT_MAP_SOURCES else 0
    if reported is not None:
      self.seen[reported] = self.frame
    if reported == self.candidate:
      self.candidate_frames += 1
      self.candidate_trust = max(self.candidate_trust, trust)
    else:
      self.candidate, self.candidate_frames, self.candidate_trust = reported, 0, trust
    if self.candidate is not None and self.candidate != self.stable and self.candidate_frames >= _frames(LIMIT_STABLE_TIME):
      self._new_limit(i, d)
    if self.rise is not None:
      self._steady_rise(i, d)
    return self._offered(d)

  def _new_limit(self, i, d):
    new, start = self.candidate, self.frame - self.candidate_frames    # start: the new limit's first frame
    old, old_seen = self.stable, self.stable_last_seen
    if (self.matched is not None and self.matched != old and self.matched < new and
        self.frame - self.matched_seen <= _frames(LIMIT_MATCH_MAX_AGE)):
      old, old_seen = self.matched, self.frame        # the limit the driver was at, before a short detour (4d)
    self.stable, self.stable_last_seen = new, self.frame
    # a new limit: an open offer is void, and so is an open cap (drives 4c t 614-655, 1704-1739 and 4d t 759-787: the
    # cross road's 40 for 30-40 s while stopped at a light, then the 45 again; the restore then goes on to 45)
    if self.rise is not None:
      r_old, r_new, _ = self.rise
      self._not_followed(r_old, r_new, i.source, f"not steady for {LIMIT_RAISE_STABLE_TIME:.0f} s")
    self.offer, self.cap, self.rise, self.undo = None, None, None, None
    stale = old is None or self.frame - old_seen > _frames(LIMIT_OLD_MAX_AGE)
    quiet = all(start - f >= _frames(LIMIT_FIRST_GAP) for v, f in self.seen.items() if v != new)
    r = self.raised
    if (r is not None and start - r.frame <= _frames(LIMIT_UNDO_TIME) and new < r.new and
        (r.old is None or new <= r.old)):
      # rule 4: back at (or below) the limit we raised from, soon after our raise (a first limit had none: any lower
      # one). The controller decides how (rule 4 in the module docstring); no rule 3 cap from our own raised speed. No
      # GPS trust check: it only ever goes back to the driver's own speed.
      self.raised = None
      self.undo = Offer(r.new, r.driver, i.source, self.frame, "undo", r.new)
      return
    if stale and quiet:
      if d is None or self.cruise_off_frame >= start or new - round(d) <= 0:
        self._first_limit(new, start, d, i)            # cruise off since it appeared, or nothing to raise: now
      else:
        self.rise = (None, new, start)                 # rule 2, decided once steady (_steady_rise)
    elif old is None or new == old:
      return                                           # a flap at the start of the drive: nothing known to compare
    elif stale:
      if new > old:
        self._not_followed(old, new, i.source, "old limit not seen recently")
    elif new > old:
      if d is not None and abs(d - new) < 0.5:
        return                                         # already at the new limit
      self.rise = (old, new, start)                    # rule 1, decided once steady (_steady_rise)
    elif d is None or self._in_band(d, old):
      why = self._plausible(old, new, i.source, i.gps_trust)
      if why:
        self._not_followed(old, new, i.source, f"lower limit: {why}")
      else:
        # (cruise off: decided with the driver's speed on the first frame it is back; until then nothing to cap)
        self.cap = Offer(old, new, i.source, self.frame, "cap", d)
    # (a lower limit with the driver's speed outside the band: his own speed, nothing to do)

  def _steady_rise(self, i, d):
    """A higher limit once reported unchanged for LIMIT_RAISE_STABLE_TIME: rule 1 or 2 with the driver's speed now."""
    old, new, start = self.rise
    if self.frame - start > _frames(LIMIT_STABLE_TIME + LIMIT_OFFER_TIME):
      self.rise = None
      self._not_followed(old, new, i.source, f"not steady for {LIMIT_RAISE_STABLE_TIME:.0f} s")
      return
    if self.candidate != new or self.candidate_frames < _frames(LIMIT_RAISE_STABLE_TIME):
      return                                           # (a short flap restarts the count)
    self.rise = None
    if old is None:
      self._first_limit(new, start, d, i)
      return
    if d is not None and abs(d - new) < 0.5:
      return                                           # already at the new limit
    why = self._plausible(old, new, i.source, self.candidate_trust, firm=new - old > self.big_raise)
    if not why and d is None:
      why = "cruise off when the higher limit became steady"
    if not why and not self._in_band(d, old):
      why = f"driver's speed {round(d)} is not the old limit or up to {LIMIT_BAND * self.multiple} above"
    if why:
      self._not_followed(old, new, i.source, why)
    else:
      self.offer = Offer(old, new, i.source, self.frame, "raise", d)

  def _first_limit(self, new, start, d, i):
    """Rule 2: a first limit after none (or none for a long time)."""
    if d is not None and new - round(d) <= 0:
      return                                           # at or below the driver's speed: nothing to raise
    why = self._plausible(None, new, i.source, self.candidate_trust, firm=True)
    if not why:
      if d is None or self.cruise_off_frame >= start:
        # (review: a first limit that came with cruise off was offered for his speed from before, and RES, which is
        # not a SET edge for LIMIT_DRIVER_QUIET, took it 0.3 s after re-engaging)
        why = "first limit: cruise was not on from when the limit appeared until it was steady"
      elif self.driver_since > start:
        why = f"first limit: driver's speed {round(d)} was set after the limit appeared"
      elif new - round(d) > self.first_max:
        why = f"first limit {new} is more than {self.first_max} above the driver's speed {round(d)}"
    if why:
      self._not_followed(None, new, i.source, why)
    else:
      self.offer = Offer(None, new, i.source, self.frame, "first", d)

  def _offered(self, d):
    o = self.offer
    if o is None:
      return 0.0
    if self.frame - o.frame > _frames(LIMIT_OFFER_TIME):
      self.offer = None
      self._not_followed(o.old, o.new, o.source, "not taken in time (cruise off, a slowdown or the driver on the buttons)")
      return 0.0
    if d is None:
      return 0.0                     # cruise off since it was offered: wait (drive 52: a stop slowdown, his brake)
    if abs(d - o.new) < 0.5:
      self.offer = None              # already there (the driver's own SET+: drive 52, t 321)
      return 0.0
    if abs(d - o.driver) < 0.5:
      return o.new * self.step
    self.offer = None                # the driver chose another speed: theirs
    self._not_followed(o.old, o.new, o.source, f"driver changed his speed to {round(d)}")
    return 0.0

  def mark_adopted(self):
    """The controller took the offer (or Test mode logged it): close it, and remember it for rule 4."""
    o = self.offer
    if o is not None:
      self.raised = Offer(o.old, o.new, o.source, self.frame, o.kind, o.driver)
    self.offer = None

  def mark_capped(self):
    """Nothing to cap (no restore pending when the lower limit came), or Test mode logged it: close it."""
    self.cap = None

  def mark_undone(self):
    """The controller dealt with the undo (or Test mode logged it): close it."""
    self.undo = None

  @property
  def stable_speed(self):
    return (self.stable or 0) * self.step

  @property
  def offer_speed(self):
    return self.offer.new * self.step if self.offer is not None else 0.0

  @property
  def cap_speed(self):
    return self.cap.new * self.step if (self.cap is not None and self.cap.driver is not None) else 0.0

  @property
  def undo_speed(self):
    """Rule 4: the driver's speed before our raise (m/s) while an undo is open, else 0."""
    return self.undo.new * self.step if self.undo is not None else 0.0
