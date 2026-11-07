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
   it only works like rule 3's cap at his old speed. Only with GPS trust 0 on every frame of the lower limit's steady
   time; never for a merge (rule 5) once the highway is matched.
5. Merge onto a highway: on a link that leads to a highway (starpilotPlan.mapRampToHighway) from a street, the next
   posted limit ahead (mapdOut.nextSpeedLimit: the highway's) becomes the driver's speed when it is above his speed and
   his speed is a street's (at most MERGE_SURFACE_MAX_MPH, km/h: _KPH). Always the limit itself (drive 53: he chose 70
   on the 70 road, 75 later himself). Only a highway limit (a whole multiple, MERGE_LIMIT_MIN_MPH .. LIMIT_MAX_MPH, at
   most LIMIT_MAX_JUMP_MPH above his speed), the next limit and his speed unchanged for MERGE_STABLE_TIME with GPS
   trust 0 and cruise on throughout (a lower trust after that is fine: near the merge the highway runs next to the
   ramp), once per link. Timed for the merge: offered when the distance left to where the
   limit starts (mapdOut.nextSpeedLimitDistance, counted down with the speed between its updates) is what the stock
   cruise needs to get within MERGE_NEAR_MPH of it (MERGE_ACCEL) plus MERGE_LEAD_TIME, or at once when less (unknown
   too). The controller takes it once no curve or stopped car needs a lower speed (the lowest target wins) and stays
   open while on the link and LIMIT_OFFER_TIME after it. His SET press, cancel or brake on the link ends it (after it
   was taken his buttons work as on any restore); so does any SET press of his, cancel or brake since the link began,
   or a planned stop ahead (the controller waits). Rule 4 takes it back if the next limit is his old one or lower
   within LIMIT_UNDO_TIME before the highway is matched, or if the car gets onto a road that is not a highway for
   LIMIT_STABLE_TIME (another link matched).
6. After an exit: the first limit after a link from a highway (until EXIT_TIME after the car got onto the street) is
   the street's. A limit below the driver's speed caps a pending restore at it (any band: his highway speed is not a
   street's). A limit above his speed, with no SET press of his since the link began (our slowdown left the set speed
   there), becomes his speed once steady 5 s off the link, as a first limit (GPS trust 0, cruise on since it appeared
   until it is taken), any gap (drive 53: the floor 19 to 50). A lower limit within LIMIT_UNDO_TIME of that raise
   (a wrong first one) is rule 4's undo, down to the lower limit (never below his speed before our raise).

A lower limit reported but not steady yet (LIMIT_STABLE_TIME), with the driver's speed in rule 1's band of the one
before (the one he matched, else the steady one): no SET+ of ours above it while it is reported (provisional, at once,
also on the first frame cruise is back), never a SET-. If it goes again (a junction's flap) the restore goes on; if it
stays, rule 3 decides. Drive 53, tag 1324.5: the 45 -> 40 came 0.1 s before the resume (set 39, his 50); the restore
went to 50 by t 1323, and with only rule 3's cap 3 s later it would have stopped at 47-48 on the 40 road.

Raises (rules 1, 2, 6) are harder than caps: the new limit must be steady for LIMIT_RAISE_STABLE_TIME (caps and the undo:
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

Drive 53 (night, highway with navigation; owner tags 310, 457, 1059.7, 1324.5, 1355.9):
 * Ramp 1 (rule 5): set 45 from the 45 road, onto the link at 18 mph (t 298.2), no limit on it (t 302.2-333.2), mapd's
   next limit 70 at 614 m (t 302.2); nothing happened and he held SET+ 45 -> 70 himself at t 320.7 (mapd: 378 m at
   t 319.2, 305 m at t 321.7), 64 mph at the merge (t 335). Replayed: offered at t 320.6, 338 m before (sim: 65 mph).
 * Ramp 2 (rule 5): a loop's curve held the set at 29 (his 50) from t 1021 to the merge (next limit 70 at 412 m,
   t 1020); after it he was on the gas, his SET+ jumped to 48 at t 1060.9, then he held it to 55 and later 70 (his
   speed for the restore was still 50). Offered at t 1026, taken when the curve ends (sim: 29 -> 70 by holds).
 * Tag 457 (rule 6): the exit's ramp curve and a stop left the set at the floor 19 (his 75); off the ramp the exit's
   street rule (map_curve_control adopt) made the 19 his speed (t 443.2); the street's 50 came at t 448.2 (none for
   43 s): not followed (rule 2: 31 mph), he held SET+ himself at t 457. Replayed: 19 -> 50 offered at t 453.2.
 * Tags 1324.5, 1355.9 (the provisional ceiling): see above; then the 40 road, our curve slowdown, his brake, the resume
   at t 1351.1 and the 45 again at t 1354.2: 40 on the 40 road, 45 (rule 1) on the 45 road.

The driver's own choice wins: a speed outside rule 1's band, a speed he set after a first limit appeared, or any change
of his while a raise is offered leaves the set speed alone. An offer that is not taken within LIMIT_OFFER_TIME ends.
The limit comes from StarPilot's speed limit fields in starpilotPlan, which on this car (no openpilot longitudinal) are
only filled while Show Speed Limits is on (display-only mode: the raw map / dashboard limit); the road class and the
ramp from our curve map matcher (starpilotPlan.map*), the next limit and its distance from mapdOut. Setting
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

# starpilotPlan.mapRoadClass (our own curve map matcher, map_curves.py; 0 unknown): highways and the link roads (ramps)
HIGHWAY_CLASSES = (1, 3)              # motorway, trunk (map_curves.HIGHWAY_CLASSES)
LINK_CLASSES = (2, 4, 6, 8, 10)       # map_curves.LINK_CLASSES
# Rule 5, merge (drive 53, tags 310 and 1059.7: "not speeding up after entering the ramp to merge into highway"). On a
# link that leads to a highway (mapRampToHighway) the next posted limit ahead (mapdOut.nextSpeedLimit, the highway's 70)
# becomes the driver's speed: he held SET+ 45 -> 70 himself 305 m before the merge (ramp 1), gassed and held it up
# 29 -> 70 after the merge (ramp 2). Only a highway limit (a whole multiple, 50 .. LIMIT_MAX_MPH mph) and only from a
# surface road speed (MERGE_SURFACE_MAX_*: the street he came from), up to LIMIT_MAX_JUMP_MPH higher.
MERGE_LIMIT_MIN_MPH = 50
MERGE_LIMIT_MIN_KPH = 80
MERGE_SURFACE_MAX_MPH = 55
MERGE_SURFACE_MAX_KPH = 90
# The next limit and the driver's speed reported unchanged this long, GPS trust 0 and cruise on throughout (ramp 1: trust
# 0 from the link's start to 28 s in, 2 near the merge, where the highway is next to the ramp; ramp 2 the same)
MERGE_STABLE_TIME = LIMIT_RAISE_STABLE_TIME
# When: the stock cruise's speed-up with the set speed above the speed (drives 4c-53, median m/s^2 by the gap in mph:
# up to 5 below, 5-10, ..., 20 and more), so the cruise is MERGE_NEAR_MPH below the highway's limit where it starts
# (mapdOut.nextSpeedLimitDistance). 45 -> 65 mph: 11.9 s, 300 m; with MERGE_LEAD_TIME for our presses (a SET+ hold to 5
# below, then taps) the start for ramp 1 is 340 m before (he started his own hold at 320.7 s: mapd 378 m at 319.2 s,
# 305 m at 321.7 s).
MERGE_ACCEL = ((5, 0.33), (10, 0.51), (15, 0.75), (20, 0.91), (1000, 1.06))
MERGE_NEAR_MPH = 5
MERGE_LEAD_TIME = 2.0        # s
# A merge raise the controller took is taken back (as rule 4) when the car gets onto a road that is not a highway (the
# map matched another link) for LIMIT_STABLE_TIME before a highway was matched, within this
MERGE_WATCH_TIME = 60.0      # s
# Rule 6, after an exit (drive 53, tag 457): a stop on the exit ramp left the set speed at the floor (19, the driver's
# speed 75 = the highway's; the exit's street rule then made the 19 his speed off the ramp) and the street's 50 came 4 s
# after the ramp ended (none for 43 s before): rule 2 refuses 31 mph. The first limit after a link from a highway is the
# street's: it caps a pending restore at it (no band), or, below it with no SET press of the driver's since the link
# began, raises the driver's speed to it (as a first limit: steady 5 s with GPS trust 0, cruise on since it appeared,
# off the link). Open from the link's start until EXIT_TIME after the car got onto the street.
EXIT_TIME = 60.0             # s


def _frames(seconds):
  return int(round(seconds / DT_CTRL))


@dataclass
class LimitInputs:
  limit: float              # m/s, starpilotPlan.slcSpeedLimit (0 unknown)
  source: str               # starpilotPlan.slcSpeedLimitSource
  gps_trust: int            # starpilotPlan.mapGpsTrust
  driver_speed: float       # m/s: the controller's driver speed (anchor, else the set speed); 0 when cruise is off
  cruise_enabled: bool
  road_class: int = 0       # starpilotPlan.mapRoadClass (0 unknown)
  ramp_to_highway: bool = False  # starpilotPlan.mapOnRamp and mapRampToHighway: this link leads to a highway
  next_limit: float = 0.0   # m/s, the next posted limit ahead (mapdOut.nextSpeedLimit, else slcNextSpeedLimit; 0 none)
  next_distance: float = -1.0  # m to it (mapdOut.nextSpeedLimitDistance; < 0 unknown)
  v_ego: float = 0.0        # m/s, the true speed
  driver_set: bool = False  # the driver is on the SET-/SET+ switch this frame (cruise on or off)


@dataclass
class Offer:
  """One change of the driver's speed the controller may take: "raise" (rule 1), "first" (rule 2, old None), "cap"
  (rule 3, a lower limit: open while that limit stays and the driver's speed is unchanged), "undo" (rule 4: old is
  the limit we raised to, new the driver's speed before our raise), "merge" (rule 5: old the street's limit, else his
  speed) or "exit" (rule 6, old None)."""
  old: object               # steps, None for a first limit
  new: int
  source: str
  frame: int
  kind: str = "raise"
  driver: object = None     # steps: the driver's speed it was offered for (a cap: None while cruise is off)
  # a cap that came with cruise off: kept only if his speed is in rule 1's band of `old` (False, after an exit: kept if
  # his speed is above the new limit)
  band: bool = True


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
    self.cruise_drop_frame = -10_000  # the last frame cruise went off (his cancel or brake) after being on
    self.rise = None               # (old, new, first frame): a higher limit waiting for LIMIT_RAISE_STABLE_TIME
    self.offer = None              # an open raise (rule 1 / 2)
    self.cap = None                # rule 3: open while the lower limit stays and the driver's speed is unchanged
    self.raised = None             # rule 4: the last raise the controller took (an Offer, frame = when it took it)
    self.undo = None               # rule 4: open until the controller has dealt with it
    self.events = []               # (kind, dict) for the per-event log, drained by the caller
    self.ceiling = None            # a lower limit reported but not steady yet: no SET+ of ours above it (steps)
    is_metric = self.step < MS_PER_MPH
    self.merge_min = MERGE_LIMIT_MIN_KPH if is_metric else MERGE_LIMIT_MIN_MPH
    self.surface_max = MERGE_SURFACE_MAX_KPH if is_metric else MERGE_SURFACE_MAX_MPH
    self.road = 0                  # the last known road class (mapRoadClass; 0 frames are skipped) ...
    self.road_since = 0            # ... since this frame
    self.link_start = None         # the frame the link the car is on began (None: not on a link)
    self.link_from = 0             # the road class before it
    self.link_last = -10_000       # the last frame on a link
    self.link_to_hwy = False       # this link leads to a highway (mapRampToHighway seen on it)
    self.merge_done = False        # rule 5 decided for this link
    self.merge_window = None       # (next limit, driver's speed, first frame): steady, trust 0, cruise on
    self.merge_watch = None        # a merge raise the controller took, until a highway is matched
    self.next_raw = None           # the next limit's distance as last reported (m) ...
    self.next_dist = None          # ... and counted down with the speed since
    self.exit_start = None         # rule 6: the frame a link from a highway began (None: no exit open)
    self.exit_street = None        # the frame the car first got onto a street after it
    self.driver_set_frame = -10_000  # the last frame the driver was on SET-/SET+

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
      if self.cruise_off_frame != self.frame - 1:
        self.cruise_drop_frame = self.frame
      self.cruise_off_frame = self.frame
    if d is not None and (self.driver is None or abs(d - self.driver) >= 0.5):
      self.driver, self.driver_since = d, self.frame
    if i.driver_set:
      self.driver_set_frame = self.frame
    self._road(i)
    c = self.cap
    if c is not None and d is not None:
      if c.driver is None:           # cruise was off when the lower limit came: his speed now, the first frame back
        if (self._in_band(d, c.old) if c.band else d > c.new + 0.5):
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
    self._provisional(reported, d, i)
    self._merge(i, d)
    return self._offered(d)

  # ---- road classes, the provisional ceiling, rules 5 (merge) and 6 (after an exit) ----

  def _road(self, i):
    """Road class bookkeeping (frames with class 0, unknown, are skipped: drive 53 t 443.2, link -> 0 -> street)."""
    if i.next_limit > 0 and i.next_distance >= 0:
      if self.next_raw is None or abs(i.next_distance - self.next_raw) > 0.5:
        self.next_raw = self.next_dist = i.next_distance      # (mapd's distance moves in steps of 1-2 s)
      else:
        self.next_dist = max(0.0, self.next_dist - max(i.v_ego, 0.0) * DT_CTRL)
    else:
      self.next_raw = self.next_dist = None
    c = i.road_class
    if (self.exit_start is not None and self.exit_street is not None and
        self.frame - self.exit_street > _frames(EXIT_TIME) and not (self.rise is not None and self.rise[0] == "exit")):
      self.exit_start = None         # no limit for EXIT_TIME on the street: nothing of the exit's any more
    w = self.merge_watch
    if w is not None and self.frame - w.frame > _frames(MERGE_WATCH_TIME):
      self.merge_watch = None
    if not c:
      return
    if c != self.road:
      if c in LINK_CLASSES and self.road not in LINK_CLASSES:
        self.link_start, self.link_from, self.link_to_hwy, self.merge_done, self.merge_window = (
          self.frame, self.road, False, False, None)
        if self.road in HIGHWAY_CLASSES:
          self.exit_start, self.exit_street = self.frame, None
      self.road, self.road_since = c, self.frame
    if c in LINK_CLASSES:
      self.link_last = self.frame
      self.link_to_hwy = self.link_to_hwy or bool(i.ramp_to_highway)
      return
    self.link_start = None
    if c in HIGHWAY_CLASSES:
      self.exit_start = self.merge_watch = None    # back on a highway (an interchange), or the merge got there
      if self.raised is not None and self.raised.kind == "merge":
        self.raised = None           # on the highway the merge is done: never undone (rule 4) from here
      return
    if self.exit_start is not None and self.exit_street is None:
      self.exit_street = self.frame
    r = self.merge_watch
    if r is not None and self.frame - self.road_since >= _frames(LIMIT_STABLE_TIME):
      # rule 5 taken back: after our merge raise the car is on a road that is not a highway (as rule 4's undo)
      self.merge_watch = None
      if self.raised is r:
        self.raised = None
        self.undo = Offer(r.new, r.driver, r.source, self.frame, "undo", r.new)

  def _provisional(self, reported, d, i):
    """A lower limit reported now, not steady yet, with the driver's speed in rule 1's band of the limit before (the
    one he matched, else the steady one): no SET+ of ours above it until it goes again (a junction's flap: the restore
    only pauses) or rule 3 decides once it is steady. Drive 53, tag 1324.5: the 45 -> 40 came 0.1 s before cruise was
    back (set 39, his 50); the restore went to 50 by 1323, rule 3's cap 3 s later would have stopped it at 47."""
    self.ceiling = None
    if reported is None or d is None or reported == self.stable:
      return
    m = self.matched
    if m is not None and self.frame - self.matched_seen <= _frames(LIMIT_MATCH_MAX_AGE) and self._in_band(d, m):
      base = m
    elif self.stable is not None and self._in_band(d, self.stable):
      base = self.stable
    else:
      return
    if reported < base and reported < d - 0.5 and not self._plausible(base, reported, i.source, i.gps_trust):
      self.ceiling = reported

  def _merge_need(self, v, new):
    """m the cruise needs from v (m/s) to MERGE_NEAR_MPH below the limit `new` (steps), set at it (MERGE_ACCEL), plus
    MERGE_LEAD_TIME at v for our presses."""
    target = new * self.step - MERGE_NEAR_MPH * MS_PER_MPH
    v = max(v, 0.0)
    dist = v * MERGE_LEAD_TIME
    dv = 0.5 * MS_PER_MPH
    while v < target:
      gap = (new * self.step - v) / MS_PER_MPH
      a = next(acc for top, acc in MERGE_ACCEL if gap < top)
      step = min(dv, target - v)
      dist += (v + step / 2) * step / a
      v += step
    return dist

  def _merge(self, i, d):
    """Rule 5: on a link to a highway, the highway's limit ahead becomes the driver's speed, timed for the merge."""
    o = self.offer
    if o is not None and o.kind == "merge":
      if self.road not in LINK_CLASSES + HIGHWAY_CLASSES and self.frame - self.road_since >= _frames(LIMIT_STABLE_TIME):
        self.offer = None
        self._not_followed(o.old, o.new, o.source, "merge: off the link onto a road that is not a highway")
      return
    if self.link_start is None or self.merge_done or not self.link_to_hwy or self.link_from in HIGHWAY_CLASSES:
      self.merge_window = None
      return
    n = None
    if i.next_limit >= 1.0:
      x = self._steps(i.next_limit)
      if abs(x - round(x)) <= LIMIT_WHOLE_TOL:
        n = int(round(x))
    if self.driver_set_frame >= self.link_start or self.cruise_drop_frame >= self.link_start:
      # the driver sets his own speed, cancels or brakes at any time since the link began (as rules 2 and 6): his
      # (review: only once the window was open; engaged by SET at 40 on the link, or a brake and RES there, -> 70).
      # Cruise already off when the link began and resumed (RES) on it is not his choice of speed: drive 53, ramp 1
      # (tag 310), cruise off from t 290.0, RES at 298.5, 0.3 s after the link was matched, at his street's 45
      self.merge_done = True
      self._not_followed(None, self.merge_window[0] if self.merge_window is not None else n, "Map Data",
                         "merge: the driver set his speed on the link" if self.driver_set_frame >= self.link_start else
                         "merge: cruise off on the link")
      return
    w = self.merge_window
    ready = w is not None and self.frame - w[2] >= _frames(MERGE_STABLE_TIME)
    if n is None or d is None:
      if not ready:
        self.merge_window = None     # (once steady, a drop of the next limit only waits: slcNextSpeedLimit flickers)
      return
    if w is None or w[0] != n or abs(w[1] - d) >= 0.5 or (not ready and i.gps_trust > LIMIT_FIRM_TRUST_MAX):
      # trust 0 is needed for the steady time only: near the merge the highway runs next to the ramp and the trust
      # drops (drive 53: 2 from 200 m before both merges)
      self.merge_window = (n, d, self.frame) if i.gps_trust <= LIMIT_FIRM_TRUST_MAX else None
      return
    if not ready:
      return
    unit_mph = self.step / MS_PER_MPH
    why = ""
    if d >= n - 0.5:
      why = f"driver's speed {round(d)} is not below the highway's {n}"
    elif n % self.multiple or n * unit_mph > LIMIT_MAX_MPH + 1e-6 or n < self.merge_min:
      why = f"next limit {n} is not a highway limit"
    elif d > self.surface_max + 0.5:
      why = f"driver's speed {round(d)} is not a street's (more than {self.surface_max})"
    elif (n - d) * unit_mph > LIMIT_MAX_JUMP_MPH + 0.5:
      why = f"jump {round(n - d)} > {LIMIT_MAX_JUMP_MPH} mph"
    if why:
      self.merge_done = True
      self._not_followed(None, n, "Map Data", "merge: " + why)
      return
    if self.next_dist is not None and self.next_dist > self._merge_need(i.v_ego, n):
      return                         # not yet: the cruise reaches the highway's speed near the merge from here
    self.merge_done = True
    old = self.stable if (self.stable is not None and self.frame - self.stable_last_seen <= _frames(LIMIT_OLD_MAX_AGE)) else None
    self.offer = Offer(old if old is not None and old < n else round(d), n, "Map Data", self.frame, "merge", d)

  def _new_limit(self, i, d):
    new, start = self.candidate, self.frame - self.candidate_frames    # start: the new limit's first frame
    old, old_seen = self.stable, self.stable_last_seen
    if (self.matched is not None and self.matched != old and self.matched < new and
        self.frame - self.matched_seen <= _frames(LIMIT_MATCH_MAX_AGE)):
      old, old_seen = self.matched, self.frame        # the limit the driver was at, before a short detour (4d)
    self.stable, self.stable_last_seen = new, self.frame
    o = self.offer
    if o is not None and o.kind == "merge" and new >= o.driver - 0.5:
      return                         # the highway's own limit (or the link's, not below his speed): the merge stands
    # a new limit: an open offer is void, and so is an open cap (drives 4c t 614-655, 1704-1739 and 4d t 759-787: the
    # cross road's 40 for 30-40 s while stopped at a light, then the 45 again; the restore then goes on to 45)
    if self.rise is not None:
      r_old, r_new, _ = self.rise
      self._not_followed(None if r_old == "exit" else r_old, r_new, i.source, f"not steady for {LIMIT_RAISE_STABLE_TIME:.0f} s")
    self.offer, self.cap, self.rise, self.undo = None, None, None, None
    stale = old is None or self.frame - old_seen > _frames(LIMIT_OLD_MAX_AGE)
    quiet = all(start - f >= _frames(LIMIT_FIRST_GAP) for v, f in self.seen.items() if v != new)
    r = self.raised
    if (r is not None and start - r.frame <= _frames(LIMIT_UNDO_TIME) and new < r.new and
        (r.old is None or new <= r.old)):
      # rule 4: back at (or below) the limit we raised from, soon after our raise (a first limit or an exit had none:
      # any lower one). The controller decides how (rule 4 in the module docstring); no rule 3 cap from our own raised
      # speed. GPS trust 0 on every frame of its steady time (review: a street's 45 for 3 s with trust 2 a few s after
      # a merge took the set 70 -> 45 on the highway, 25 SET-).
      if self.candidate_trust > LIMIT_FIRM_TRUST_MAX:
        self._not_followed(r.new, new, i.source, f"undo: GPS trust {self.candidate_trust} (an undo needs 0)")
        return
      self.raised = self.merge_watch = None
      # after an exit (rule 6) his speed goes to the lower limit, the street's (review: a wrong first 65 raised the
      # floor 19 to 65, then the street's 35 only capped the restore at 50), never below his speed before our raise
      back = max(new, r.driver) if r.kind == "exit" else r.driver
      self.undo = Offer(r.new, back, i.source, self.frame, "undo", r.new)
      return
    if self.exit_start is not None and start >= self.exit_start:
      self._exit_limit(new, old, d, i, start)   # rule 6: the first limit after a link from a highway
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

  def _exit_limit(self, new, old, d, i, start):
    """Rule 6: the street's limit after an exit. Above the driver's speed (or cruise off: decided when it is back) a
    cap of a pending restore, any band; below it a raise, decided once steady (_steady_rise)."""
    hw = old if (old is not None and old > new) else None
    why = self._plausible(None, new, i.source, i.gps_trust)
    if why:
      self.exit_start = None
      self._not_followed(hw, new, i.source, f"after the exit: {why}")
      return
    if d is None or d > new + 0.5:
      self.cap = Offer(hw, new, i.source, self.frame, "cap", d, band=False)
    if d is None or d < new - 0.5:
      self.rise = ("exit", new, start)
    else:
      self.exit_start = None

  def _exit_raise(self, new, start, d, i):
    exit_start, self.exit_start = self.exit_start, None
    if d is not None and d >= new - 0.5:
      return                                           # already there (or above: a cap's)
    why = self._plausible(None, new, i.source, self.candidate_trust, firm=True)
    if not why:
      if d is None or self.cruise_off_frame >= start:
        why = "cruise was not on from when the limit appeared until it was steady"
      elif exit_start is None or self.driver_set_frame >= exit_start:
        why = f"the driver set his speed ({round(d)}) himself since the exit"
    if why:
      self._not_followed(None, new, i.source, f"after the exit: {why}")
    else:
      self.offer = Offer(None, new, i.source, self.frame, "exit", d)

  def _steady_rise(self, i, d):
    """A higher limit once reported unchanged for LIMIT_RAISE_STABLE_TIME: rule 1 or 2 with the driver's speed now."""
    old, new, start = self.rise
    if self.frame - start > _frames(LIMIT_STABLE_TIME + LIMIT_OFFER_TIME):
      self.rise = None
      if old == "exit":
        self.exit_start = None
        old = None
      self._not_followed(old, new, i.source, f"not steady for {LIMIT_RAISE_STABLE_TIME:.0f} s")
      return
    if self.candidate != new or self.candidate_frames < _frames(LIMIT_RAISE_STABLE_TIME):
      return                                           # (a short flap restarts the count)
    if old == "exit" and self.road in LINK_CLASSES:
      return                                           # still on the exit (mapd's lookahead): once off it
    self.rise = None
    if old == "exit":
      self._exit_raise(new, start, d, i)
      return
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
    # a merge stays open while on the link and LIMIT_OFFER_TIME after it (drive 53, ramp 2: a loop's curve held the set
    # speed for 31 s, to the merge)
    since = max(o.frame, self.link_last) if o.kind == "merge" else o.frame
    if self.frame - since > _frames(LIMIT_OFFER_TIME):
      self.offer = None
      self._not_followed(o.old, o.new, o.source, "not taken in time (cruise off, a slowdown or the driver on the buttons)")
      return 0.0
    if d is None:
      if o.kind in ("merge", "exit"):
        # his cancel or brake on the ramp: his; after an exit cruise must stay on from when the limit appeared until it
        # is taken (as rule 2's check up to the offer; review: an exit offer survived cruise off, taken right after RES)
        self.offer = None
        self._not_followed(o.old, o.new, o.source, "merge: cruise off" if o.kind == "merge" else "after the exit: cruise off")
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
    if o is not None and not (o.kind == "merge" and self.road in HIGHWAY_CLASSES):
      # (a merge taken on the highway is never undone; rule 6's undo goes to the lower limit, not back to the floor)
      self.raised = Offer(o.old, o.new, o.source, self.frame, o.kind, o.driver)
      if o.kind == "merge":
        self.merge_watch = self.raised
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
  def ceiling_speed(self):
    """A lower limit reported but not steady yet (m/s): no SET+ of ours above it, never a SET- (0: none)."""
    return self.ceiling * self.step if self.ceiling is not None else 0.0

  @property
  def undo_speed(self):
    """Rule 4: the driver's speed before our raise (m/s) while an undo is open, else 0."""
    return self.undo.new * self.step if self.undo is not None else 0.0
