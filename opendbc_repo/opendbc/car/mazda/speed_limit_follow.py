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

7. After a turn onto another road (mapdOut's road name changes and stays TURN_NAME_STEADY, the car's heading changed
   TURN_HEADING within TURN_HEADING_TIME and TURN_HEADING_DIST (HeadingWatch) at some time since the new name appeared,
   not on a highway or a link):
   the new road's first steady limit is handled as rule 6's: a restore never goes back above it (any band), and below it
   (with no SET press of his since the turn, cruise on since it appeared, GPS trust 0, at most LIMIT_FIRST_MAX above his
   speed) it becomes his speed. Never a SET- (it never slows him early); open TURN_TIME. The same limit as the old road's
   counts as the new road's once the name has changed.

8. Match on engage (owner after drives 54 / 55: "if we know the road speed, irrespective of whether I canceled the
   cruise to come to a stop at a red signal, if I set the cruise we should either reduce or increase the speed to the
   road's speed limit"): cruise turned on (SET or RES) on a street, the road class steady ENGAGE_ROAD_STEADY with GPS
   trust 0: the steady limit plus the Street Speed offset becomes his speed (up: an offer the controller raises to, down:
   `lower`, the map control's paced slowdown, then his; his gas does not end it). No map limit: navigation's alone (up
   to NAV_ALONE_MAX) only down, to the higher of it plus the offset and STREET_FALLBACK; nothing known: only down, to
   STREET_FALLBACK; the fallback only with the car at most FALLBACK_MAX_SPEED. His SET within ENGAGE_WINDOW first:
   his. This also covers his RES after an exit (map_curve_control's street rule while cruise is on). Rules 8 and 9 act only with Map Curves On (street_active).
   No target under STREET_FLOOR (the controller's 25 mph curve floor: a 20 mph / 30 km/h zone ends at 25 / 41).
9. Slow for a lower limit (setting MazdaSlowLowerLimit, default on): on a street a lower limit steady LOWER_STEADY
   (LOWER_BIG_STEADY for a drop of more than LOWER_BIG_DROP) with GPS trust 0 on every frame, the road name unchanged
   since it appeared (or a turn onto a new road), his speed above it plus the offset: `lower` (the map control's paced
   slowdown, ~0.5 m/s^2 at most, then his speed). His gas held or his SET buttons end it (his speed what it reached); the
   limit back up within LOWER_UNDO_TIME: his speed before it comes back (an "unlower" offer; a pending restore's anchor
   only up to the new limit plus the offset). Once per steady limit (a later drop to it again slows again).
   Navigation's limit (LimitInputs.nav_limit, Mapbox; under NAV_LIMIT_MIN junk) is a second opinion: the two agree
   (NAV_AGREE): LOWER_NAV_STEADY / LOWER_NAV_BIG_STEADY; they disagree: never past either one (the engage match only
   into [lower, higher] + offset, the slowdown only down to the higher one + offset). A lower limit that appears with
   navigation's next turn within NAV_TURN_NEAR is the cross road's: not slowed for (rule 7 after the turn). Navigation's
   next maneuver farther than NAV_STRAIGHT: a road name change at a junction must stay TURN_NAME_STEADY_STRAIGHT.

Street Speed (MazdaStreetSpeedOffset, STREET_OFFSETS): on a street (the matched road class known and not a highway nor a
link; after an exit or a turn always) every target set from a posted limit is the limit plus the setting's offset: rules
1, 2, 3, 6, 7, the provisional ceiling, rule 6's undo. Rule 1's band on a street is the old limit up to the larger of
LIMIT_BAND multiples and the offset (at 53 on a 45 road with +8 he is "at" the 45 road's speed). Highways and links (the
merge, rule 5) never take it; with the road class unknown (Map Curves off) neither.

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
# ... but a one-band rise (rule 1, at most LIMIT_BIG_RAISE_MPH / _KPH) with GPS trust 0 on every frame of it only this
# long (owner, drives 54 / 55: the raise came 5-8 s into the new road; the 3.5 s wrong match above had trust 1-2). First
# limits, big rises, trust above 0 and the undo keep theirs.
LIMIT_RISE_QUICK_TIME = 2.5  # s
# ... not again this long after a rule 4 undo (a 45 / 55 flap every 4 s: one raise and its undo, then LIMIT_RAISE_STABLE_TIME)
LIMIT_QUICK_AFTER_UNDO = 60.0  # s
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
# (owner, drive 54: the raise came 382 m before the merge, he was on the gas from 456; now as soon as the window is ready,
# 2 s: drive 54's would have come at 448.3 at 30 mph, 554 m before the highway's limit)
MERGE_STABLE_TIME = 2.0      # s
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
# Rule 7, after a turn onto another road: mapdOut's road name (not empty) changed and stayed this long ...
TURN_NAME_STEADY = 2.0       # s
TURN_EARLY = 2.0             # s: a limit whose first frame came this long before the name changed is the new road's
TURN_TIME = EXIT_TIME        # s the turn's window stays open on the new road
# Street Speed (MazdaStreetSpeedOffset 0..3, map_curve_control.STREET_OFFSETS): legal limit, +3, +5 (default), +8 mph;
# metric 0, +5, +8, +13 km/h
STREET_OFFSETS = ((0, 3, 5, 8), (0, 5, 8, 13))
STREET_OFFSET_DEFAULT = 2
STREET_FALLBACK = (43, 70)   # mph / km/h: no limit known (map_curve_control.STREET_FALLBACK)
# Rule 8, match on engage
ENGAGE_ROAD_STEADY = 2.0     # s the road class known (a street) and GPS trust 0 before it decides
# rules 8 / 9 under way end off a street only once the road class has said so this long (third review: one frame of a
# link class near a ramp, drive 53 t 443, ended the match and the restore took his stored 75 back)
LOWER_NOT_STREET_TIME = 2.0  # s
ENGAGE_WINDOW = 15.0         # s after cruise came on it may still decide (the road / trust becoming steady)
ENGAGE_SET_QUIET = 0.5       # s: his SET press that engaged the cruise is not "his SET after it"
# the limit itself steady this long with GPS trust 0 on every frame of it (drive 55, t 627: his RES at a junction while the
# cross road's 35 had been reported 3 s; it went after 4 s)
ENGAGE_LIMIT_STEADY = 5.0    # s
# Rule 9, slow for a lower limit (MazdaSlowLowerLimit)
LOWER_STEADY = 5.0           # s steady, GPS trust 0 on every frame (a junction's cross road: drive 55 had a 4 s 35)
LOWER_BIG_DROP = (20, 32)    # mph / km/h
LOWER_BIG_STEADY = 8.0       # s
LOWER_NAV_STEADY = 2.5       # s with navigation's limit agreeing
LOWER_NAV_BIG_STEADY = 5.0   # s
LOWER_UNDO_TIME = 30.0       # s: the limit back up this soon after it slowed him: his speed before comes back
NAV_LIMIT_MIN = (15, 24)     # mph / km/h: navigation's limit below this is no limit (drive 55: 9 messages of 5 mph)
NAV_AGREE = (1, 2)           # mph / km/h
# Navigation's limit is its route step's: drive 55 kept the highway's 70 along the whole Dobry Drive step (a 45 street,
# t 552-622), which as a "disagreeing" opinion kept his RES at 75 there. On a street a navigation limit above this is
# a highway's: no limit.
NAV_STREET_MAX = (55, 90)    # mph / km/h
# ... but with no map limit at all, navigation's up to this counts as the limit (second review: a 65 road with no map
# limit, SET at 70 -> the 43 fallback): it may cap him at it plus the offset, never raise him on it alone
NAV_ALONE_MAX = (70, 112)    # mph / km/h
# Rule 8's fallback only with the car at most this fast (second review: no speed guard; as map_curve_control's
# EXIT_STREET_MAX_V)
FALLBACK_MAX_SPEED = (50, 80)  # mph / km/h
# The lowest street speed: the controller never takes a curve target under 25 mph (curve_slowdown.MIN_SET_SPEED); a
# 20 mph / 30 km/h zone ends here (second review: a lower target never finished)
STREET_FLOOR = (25, 41)      # mph / km/h (41: 25 mph rounded up)
NAV_TURN_NEAR = 80.0         # m: navigation's next turn this close when a lower limit appears: the cross road's
NAV_STRAIGHT = 300.0         # m: navigation's next maneuver farther: we go straight through this junction
TURN_NAME_STEADY_STRAIGHT = 5.0  # s a road name change must last then
# (review of merge-f: road names change along the same road, and the highway's name became the street's after the exit)
# a name change is a turn (rule 7) only with the car's heading changed this much within this long
# (second review: 30 deg in 10 s is a 380 m radius sweep at 45 mph) 45 deg within 10 s and 150 m (a 190 m radius
# sweep at most); drive 55's Mound Road right turn: ~90 deg in ~40 m
TURN_HEADING = 45.0          # deg
TURN_HEADING_TIME = 10.0     # s
TURN_HEADING_DIST = 150.0    # m
NAV_TURN_TYPES = ("turn", "end of road", "roundabout", "rotary", "exit roundabout", "exit rotary")


def _frames(seconds):
  return int(round(seconds / DT_CTRL))


class HeadingWatch:
  """The car's heading change within the last TURN_HEADING_TIME and TURN_HEADING_DIST from its yaw (the wrapper: speed
  and steering angle): turned() for rule 7 (a turn, not a road name changing along the same road)."""
  SAMPLE = 0.5                   # s between the kept samples

  def __init__(self):
    self.yaw = 0.0                 # deg, integrated
    self.hist = []                 # (s, m, yaw) samples
    self.t = 0.0
    self.odo = 0.0                 # m

  def update(self, dt, yaw_rate_deg, v=None):
    """v: the car's speed (m/s; None: the distance bound off)."""
    self.t += dt
    self.odo += max(v, 0.0) * dt if v is not None else 0.0
    self.yaw += yaw_rate_deg * dt
    if not self.hist or self.t - self.hist[-1][0] >= self.SAMPLE:
      self.hist.append((self.t, self.odo, self.yaw))
    self.hist = [h for h in self.hist if self.t - h[0] <= TURN_HEADING_TIME and self.odo - h[1] <= TURN_HEADING_DIST]

  def turned(self):
    return any(abs(self.yaw - y) >= TURN_HEADING for _, _, y in self.hist)


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
  road_name: str = ""       # mapdOut.roadName (rule 7: a turn onto another road)
  nav_limit: float = 0.0    # m/s, navigation's posted limit (starpilotPlan.mapNavSpeedLimit: fresh, 0 none)
  nav_dist: float = 0.0     # m to navigation's next maneuver (0 none)
  nav_turn: bool = False    # that maneuver is a turn
  turned: bool = False      # the car's heading changed TURN_HEADING within TURN_HEADING_TIME (the wrapper: the yaw)
  # a restore of the controller's is pending (the set speed under its anchor): driver_speed is not the set speed he
  # chose (rule 9's undo never brings such a speed back above the limit)
  pending: bool = False


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
  tgt: object = None        # steps: the driver's speed it asks for (the limit plus the Street Speed offset), None: new
  own: bool = True          # rule 9: `driver` was his own set speed (False: a pending restore's, not his choice)

  @property
  def target(self):
    return self.new if self.tgt is None else self.tgt


class SpeedLimitFollower:
  def __init__(self, is_metric, street_mode=STREET_OFFSET_DEFAULT, slow_lower=True):
    self.step = MS_PER_KPH if is_metric else MS_PER_MPH
    m = 1 if is_metric else 0
    self.slow_lower = bool(slow_lower)
    self.fallback = STREET_FALLBACK[m]
    self.big_drop = LOWER_BIG_DROP[m]
    self.nav_min, self.nav_agree, self.nav_street_max = NAV_LIMIT_MIN[m], NAV_AGREE[m], NAV_STREET_MAX[m]
    self.nav_alone_max, self.fallback_max, self.street_floor = NAV_ALONE_MAX[m], FALLBACK_MAX_SPEED[m], STREET_FLOOR[m]
    self.street_active = True      # rules 8 / 9 act (the wrapper: Speed Limit Follow and Map Curves On)
    offs = STREET_OFFSETS[1 if is_metric else 0]
    self.street_add = offs[street_mode] if street_mode in range(len(offs)) else offs[STREET_OFFSET_DEFAULT]  # steps
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
    self.exit_kind = "exit"        # rule 6's window: "exit" (after a link from a highway) or "turn" (rule 7)
    self.exit_retry = None         # rule 6 / 7: the first limit refused only for the GPS trust (steps): tried again
    self.name, self.name_cand, self.name_frames = "", "", 0   # mapdOut's road name: steady, candidate, frames
    self.turn_check = False        # rule 7: the steady limit as the new road's once (the same value as the old road's)
    self.last_next = (0.0, -1.0)   # the next limit (m/s) and its distance (m) when the last offer was made (logs)
    self.undo_frame = -1_000_000   # the last rule 4 undo (LIMIT_QUICK_AFTER_UNDO)
    # rules 8 / 9: the engage match, the slowdown for a lower limit (`lower`: an Offer the map control carries out)
    self.lower = None              # Offer: kind "match" or "lower", tgt the speed (steps), driver his speed before
    self.lowered = None            # the last finished slowdown for a lower limit (LOWER_UNDO_TIME)
    self.lower_skip = None         # the limit a slowdown ended for by his decision: not again for it
    self.engage_frame = None       # cruise came on (rule 8 not decided yet)
    self.trust0_frames = 0         # consecutive frames with GPS trust 0
    self.unknown_frames = 0        # consecutive frames with no posted limit reported
    self.cand_name = ""            # the road name when the candidate limit first appeared ...
    self.cand_turn = False         # ... and navigation's turn was within NAV_TURN_NEAR then
    self.turn_frame = -1_000_000   # rule 7's last turn (the name changed)
    self.turned_frame = -1_000_000  # the last frame the heading watch said "turned"
    self.nav = None                # navigation's limit (steps, None: none usable) this frame
    self.stable_before = None      # the steady limit before the current one

  def _steps(self, v):
    return v / self.step

  def _street(self):
    """On a street: the matched road class known, not a highway, not a link."""
    return bool(self.road) and self.road not in HIGHWAY_CLASSES + LINK_CLASSES

  def _add(self, kind=""):
    """steps the Street Speed setting adds to a posted limit now (after an exit or a turn: always)."""
    return self.street_add if (kind in ("exit", "turn") or self._street()) else 0

  def _in_band(self, d, old):
    """The driver's speed d (steps) is at the old limit or up to LIMIT_BAND multiples above it (on a street: or the
    Street Speed offset above it, if more)."""
    return -0.5 < d - old < max(LIMIT_BAND * self.multiple, self._add()) + 0.5

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
    if i.turned:
      self.turned_frame = self.frame
    self._turn(i)
    c = self.cap
    if c is not None and d is not None:
      if c.driver is None:           # cruise was off when the lower limit came: his speed now, the first frame back
        if (self._in_band(d, c.old) if c.band else d > c.target + 0.5):
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
      self.cand_name = self.name
      self.cand_turn = bool(i.nav_turn and 0 < i.nav_dist <= NAV_TURN_NEAR)
    if self.candidate is not None and self.candidate != self.stable and self.candidate_frames >= _frames(self._stable_time()):
      self._new_limit(i, d)
    if self.rise is not None:
      self._steady_rise(i, d)
    if (self.exit_start is not None and reported is not None and reported == self.stable and
        (self.turn_check or self.exit_retry == reported) and self.rise is None and
        not self._plausible(None, reported, i.source, i.gps_trust)):
      # rule 7: the new road's limit the same as the old road's; rule 6 / 7: one refused only for the GPS trust, again
      # (review of merge-f: the retry kept the old trust 2 in candidate_trust and never raised) with a fresh steady time
      if self.exit_retry == reported or self.candidate_trust > LIMIT_FIRM_TRUST_MAX:
        self.candidate_frames, self.candidate_trust = 0, trust
      self.turn_check, self.exit_retry = False, None
      self._exit_limit(reported, None, d, i, self.frame)
    self._provisional(reported, d, i)
    self._merge(i, d)
    self._street_speed(i, d, reported)
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
      self.turn_check, self.exit_retry = False, None
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
          self.exit_start, self.exit_street, self.exit_kind, self.exit_retry = self.frame, None, "exit", None
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

  def _stable_time(self):
    """s a new limit must be reported unchanged before it is steady: LIMIT_STABLE_TIME, a one-band rise with GPS trust 0
    (rule 1, F) LIMIT_RISE_QUICK_TIME."""
    c, st = self.candidate, self.stable
    if st is not None and c is not None and self.exit_start is None and st < c and self._rise_time(st, c) < LIMIT_STABLE_TIME:
      return LIMIT_RISE_QUICK_TIME
    return LIMIT_STABLE_TIME

  def _turn(self, i):
    """Rule 7: mapdOut's road name changed (not empty) and stayed TURN_NAME_STEADY on a street: rule 6's window opens
    for the new road (unless an exit's is open)."""
    n = (i.road_name or "").strip()
    if not n or n == self.name:
      self.name_cand, self.name_frames = "", 0
      return
    if n != self.name_cand:
      self.name_cand, self.name_frames = n, 0
    self.name_frames += 1
    straight = i.nav_dist > NAV_STRAIGHT and not (i.nav_turn and i.nav_dist <= NAV_STRAIGHT)
    if self.name_frames < _frames(TURN_NAME_STEADY_STRAIGHT if straight else TURN_NAME_STEADY):
      return                         # (navigation says straight on through here: a short name change is a flap)
    old, self.name, self.name_cand = self.name, n, ""
    # the heading turned at any time since the new name appeared (mapd's name comes seconds after the turn: drive 55,
    # Mound Road, 45 deg at 624.8, the name steady at 635.6 with the 10 s window just past)
    turned = self.frame - self.turned_frame < self.name_frames      # (name_frames counts its first frame)
    if old and turned:
      self.turn_frame = self.frame
    if old and turned and self._street() and (self.exit_start is None or self.exit_kind == "turn"):
      self.exit_start = self.frame - self.name_frames - _frames(TURN_EARLY)
      self.exit_street, self.exit_kind, self.exit_retry, self.turn_check = self.frame, "turn", None, True
      if self.rise is not None and self.rise[0] == "exit":
        self.rise = None             # (the old road's pending raise: the new road decides)
    self.name_frames = 0

  # ---- rules 8 (match on engage) and 9 (slow for a lower limit) ----

  def _log(self, kind, why, limit, target, d, sources=""):
    self.events.append((kind, {"old": None, "new": limit if limit is not None else 0, "source": sources or "Map Data",
                               "reason": why, "log": {"target_mph": target * self.step / MS_PER_MPH,
                                                      "driver_before_mph": (d or 0) * self.step / MS_PER_MPH,
                                                      "sources": sources}}))

  def _nav_limit(self, i):
    """Navigation's limit (steps) when usable (fresh: the planner's; not junk under NAV_LIMIT_MIN), else None."""
    if i.nav_limit <= 0:
      return None
    x = self._steps(i.nav_limit)
    top = self.nav_street_max if i.limit >= 1.0 else self.nav_alone_max
    if self._street() and x > top + 0.5:
      return None                      # (its route step's highway limit on a street: drive 55, Dobry Drive)
    return int(round(x)) if x >= self.nav_min - 0.5 else None

  def _street_speed(self, i, d, reported):
    """Rules 8 and 9 (no limit follow on a highway, a link or an unknown road)."""
    self.nav = self._nav_limit(i)
    self.trust0_frames = self.trust0_frames + 1 if i.gps_trust <= LIMIT_FIRM_TRUST_MAX else 0
    self.unknown_frames = self.unknown_frames + 1 if reported is None else 0
    if d is None:
      self.engage_frame = None
      if self.lower is not None:
        self._log("speedLimitSlow" if self.lower.kind == "lower" else "speedLimitMatch", "cancel: cruise off",
                  self.lower.new, self.lower.target, None)
        self.lower = None
      return
    if self.cruise_off_frame == self.frame - 1:
      self.engage_frame = self.frame
    lw = self.lower
    if lw is not None:
      kind = "speedLimitSlow" if lw.kind == "lower" else "speedLimitMatch"
      if not self._street() and self.frame - self.road_since >= _frames(LOWER_NOT_STREET_TIME):
        self.lower = None              # (a later RES re-opens rule 8, as after cruise off)
        self._log(kind, "cancel: not on a street", lw.new, lw.target, d)
      elif lw.kind == "lower" and reported != lw.new and reported is not None and reported > lw.new:
        self.lower = None              # the limit back up before he got there: his speed (the anchor) comes back
        self._log("speedLimitSlow", "undo: the limit back up before it got there", reported, lw.driver, d)
      elif not self.street_active and i.driver_set:
        self.lower_finished("cancel: his SET")       # (paused: the map control does not see it)
      return
    if not self.street_active:
      self.engage_frame = None
      return
    if self.engage_frame is not None:
      self._engage(i, d, reported)
    elif self.slow_lower:
      self._slow_lower(i, d, reported)

  def _engage(self, i, d, reported):
    """Rule 8: cruise came on; decided once on a street with the road and GPS trust 0 steady ENGAGE_ROAD_STEADY."""
    if self.frame - self.engage_frame > _frames(ENGAGE_WINDOW) or (
        self.driver_set_frame > self.engage_frame + _frames(ENGAGE_SET_QUIET)):
      self.engage_frame = None         # (too late, or his own SET since: his)
      return
    steady = _frames(ENGAGE_ROAD_STEADY)
    if not self._street() or self.frame - self.road_since < steady or self.trust0_frames < steady:
      return
    nav = self.nav
    if (reported is not None and reported == self.stable and reported == self.candidate and
        self.candidate_frames >= _frames(ENGAGE_LIMIT_STEADY) and self.candidate_trust <= LIMIT_FIRM_TRUST_MAX):
      lo = hi = reported
      sources = "map"
      if nav is not None:
        if abs(nav - reported) <= self.nav_agree:
          sources = "map+nav"
        else:
          lo, hi = min(nav, reported), max(nav, reported)
          sources = f"map {reported}, nav {nav}: between"
      add = self._add()
      if d > hi + add + 0.5:
        tgt, limit = hi + add, hi
      elif d < lo + add - 0.5:
        tgt, limit = lo + add, lo
      else:
        self.engage_frame = None
        return
    elif reported is None and self.unknown_frames >= steady:
      # no map limit: only down, to the fallback; navigation's alone (NAV_LIMIT_MIN up to NAV_ALONE_MAX) only keeps him
      # at or above it, at its limit plus the offset (third review: drive 54, t 1155-1225, its 25 lingered ~70 s over
      # three roads, a primary road at 31 mph: never under the fallback on it). The fallback only at a street's speed:
      # the car at most FALLBACK_MAX_SPEED (third review: was his set speed; RES 75 at 20 mph kept 75)
      tgt, limit = self.fallback, None
      sources = "none: fallback"
      if nav is not None:
        tgt, limit, sources = max(nav + self._add(), self.fallback), nav, "nav only"
      if tgt <= self.fallback + 0.5 and i.v_ego > (self.fallback_max + 0.5) * self.step:
        self.engage_frame = None
        return
      if d <= tgt + 0.5:
        self.engage_frame = None
        return
    else:
      return                           # (a limit not steady yet, or not with trust 0: wait for it)
    self.engage_frame = None
    tgt = max(tgt, self.street_floor)
    if abs(tgt - d) < 0.5:
      return
    if tgt > d:
      self.offer = Offer(None, limit, i.source or "Map Data", self.frame, "match", d, tgt=tgt)
    else:
      self.lower = Offer(None, limit, i.source or "Map Data", self.frame, "match", d, tgt=tgt)
    self._log("speedLimitMatch", "start: cruise on, " + ("up" if tgt > d else "down"), limit, tgt, d, sources)

  def _slow_lower(self, i, d, reported):
    """Rule 9: a lower limit steady LOWER_STEADY (navigation agreeing: LOWER_NAV_STEADY) with GPS trust 0 throughout."""
    c = self.candidate
    if (c is None or reported != c or c == self.lower_skip or not self._street() or self.candidate_trust > 0 or
        self.cand_turn or self.name_cand):
      return
    if self.cand_name != self.name and self.turn_frame < self.frame - self.candidate_frames - _frames(TURN_EARLY):
      return                           # (the name changed since it appeared and not by a turn: wait)
    old = self.stable_before if self.stable == c else self.stable
    if old is None or c >= old:
      return
    nav = self.nav
    agree = nav is not None and abs(nav - c) <= self.nav_agree
    big = old - c > self.big_drop
    need = (LOWER_NAV_BIG_STEADY if big else LOWER_NAV_STEADY) if agree else (LOWER_BIG_STEADY if big else LOWER_STEADY)
    if self.candidate_frames < _frames(need):
      return
    limit, sources = c, ("map+nav" if agree else "map")
    if nav is not None and not agree:
      limit, sources = max(nav, c), f"map {c}, nav {nav}: the higher"
    tgt = max(limit + self._add(), self.street_floor)
    self.lower_skip = c                # (once for this limit: until the steady limit changes)
    if d <= tgt + 0.5:
      return
    self.lower = Offer(old, limit, i.source or "Map Data", self.frame, "lower", d, tgt=tgt)
    self.lower.own = not i.pending     # (his own speed, not a pending restore's: the undo may bring it back as it was)
    self.cap = None
    self._log("speedLimitSlow", f"start: {old} -> {c}", limit, tgt, d, sources)

  def lower_finished(self, why, d_now=None):
    """The map control finished `lower` (why: "done", or "cancel: ..." by his gas / buttons / cruise off): his speed is
    what it reached; a finished slowdown for a lower limit may still be undone (LOWER_UNDO_TIME)."""
    lw = self.lower
    if lw is None:
      return
    self.lower = None
    if why == "done" and lw.kind == "lower":
      self.lowered = Offer(lw.old, lw.new, lw.source, self.frame, "lower", lw.driver, tgt=lw.tgt, own=lw.own)
    self._log("speedLimitSlow" if lw.kind == "lower" else "speedLimitMatch", why, lw.new, lw.target,
              d_now if d_now is not None else lw.driver)

  @property
  def paused(self):
    """Rules 8 / 9's slowdown waits (third review: the planner's map curve mode flipped mid-way): kept, nothing
    pressed (the wrapper holds the restore), carried on when they act again."""
    return self.lower is not None and not self.street_active

  @property
  def lower_speed(self):
    """m/s: rules 8 / 9's speed for the map control's paced slowdown (0: none)."""
    return self.lower.target * self.step if self.lower is not None else 0.0

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
    add = self._add()
    if reported < base and reported + add < d - 0.5 and not self._plausible(base, reported, i.source, i.gps_trust):
      self.ceiling = reported + add

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
    # (owner, drive 54: timed for the merge with _merge_need it came 382 m before it, "we can do it way before that"):
    # offered as soon as the window is ready
    self.merge_done = True
    self.last_next = (i.next_limit, self.next_dist if self.next_dist is not None else -1.0)
    old = self.stable if (self.stable is not None and self.frame - self.stable_last_seen <= _frames(LIMIT_OLD_MAX_AGE)) else None
    self.offer = Offer(old if old is not None and old < n else round(d), n, "Map Data", self.frame, "merge", d)

  def _new_limit(self, i, d):
    new, start = self.candidate, self.frame - self.candidate_frames    # start: the new limit's first frame
    if new != self.lower_skip:
      self.lower_skip = None           # (second review: never cleared, a later lower limit in the drive was ignored)
    lw = self.lowered
    if (lw is not None and self.frame - lw.frame <= _frames(LOWER_UNDO_TIME) and new > lw.new and d is not None and
        abs(d - lw.target) < 0.5 and self.driver_set_frame < lw.frame):
      # rule 9's undo: the limit back up soon after it slowed him (a wrong lower limit): his speed before comes back
      self.lowered = None
      self.stable, self.stable_last_seen = new, self.frame
      # (second review: a pending restore's 70 / 75 came back on the street) his own speed as it was; a pending
      # restore's at most the new limit plus the offset
      back = lw.driver if lw.own else min(lw.driver, max(new + self._add(), self.street_floor))
      if back > d + 0.5:
        self.offer = Offer(lw.new, new, i.source, self.frame, "unlower", d, tgt=back)
      self.cap, self.rise = None, None
      self._log("speedLimitSlow", "undo: the limit back up", new, back, d)
      return
    old, old_seen = self.stable, self.stable_last_seen
    self.stable_before = self.stable
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
      self.undo_frame = self.frame
      # after an exit (rule 6) his speed goes to the lower limit, the street's (review: a wrong first 65 raised the
      # floor 19 to 65, then the street's 35 only capped the restore at 50), never below his speed before our raise
      back = max(new + self._add(r.kind), r.driver) if r.kind in ("exit", "turn") else r.driver
      self.undo = Offer(r.new, back, i.source, self.frame, "undo", r.new)
      return
    if self.exit_start is not None and start >= self.exit_start:
      self.turn_check = False
      self._exit_limit(new, old, d, i, start)   # rule 6 / 7: the first limit after a link from a highway, or a turn
      return
    if stale and quiet:
      if d is None or self.cruise_off_frame >= start or new + self._add() - round(d) <= 0:
        self._first_limit(new, start, d, i)            # cruise off since it appeared, or nothing to raise: now
      else:
        self.rise = (None, new, start)                 # rule 2, decided once steady (_steady_rise)
    elif old is None or new == old:
      return                                           # a flap at the start of the drive: nothing known to compare
    elif stale:
      if new > old:
        self._not_followed(old, new, i.source, "old limit not seen recently")
    elif new > old:
      if d is not None and abs(d - new - self._add()) < 0.5:
        return                                         # already at the new limit
      self.rise = (old, new, start)                    # rule 1, decided once steady (_steady_rise)
    elif d is None or self._in_band(d, old):
      why = self._plausible(old, new, i.source, i.gps_trust)
      if why:
        self._not_followed(old, new, i.source, f"lower limit: {why}")
      else:
        # (cruise off: decided with the driver's speed on the first frame it is back; until then nothing to cap)
        self.cap = Offer(old, new, i.source, self.frame, "cap", d, tgt=new + self._add())
    # (a lower limit with the driver's speed outside the band: his own speed, nothing to do)

  def _exit_limit(self, new, old, d, i, start):
    """Rule 6 / 7: the street's limit after an exit or a turn (its target: plus the Street Speed offset). Above the
    driver's speed (or cruise off: decided when it is back) a cap of a pending restore, any band; below it a raise,
    decided once steady (_steady_rise). Refused only for the GPS trust: tried again while it stays (exit_retry)."""
    hw = old if (old is not None and old > new) else None
    tgt = new + self._add(self.exit_kind)
    why = self._plausible(None, new, i.source, i.gps_trust)
    if why:
      if why.startswith("GPS trust"):
        self.exit_retry = new        # (drive 55: the new road's 50 steady just as the trust went 2 for 6 s)
      else:
        self.exit_start = None
      self._not_followed(hw, new, i.source, f"after the {self.exit_kind}: {why}")
      return
    self.exit_retry = None
    if d is None or d > tgt + 0.5:
      self.cap = Offer(hw, new, i.source, self.frame, "cap", d, band=False, tgt=tgt)
    if d is None or d < tgt - 0.5:
      self.rise = ("exit", new, start)
    else:
      self.exit_start = None

  def _exit_raise(self, new, start, d, i):
    exit_start, self.exit_start = self.exit_start, None
    kind = self.exit_kind
    tgt = new + self._add(kind)
    if d is not None and d >= tgt - 0.5:
      return                                           # already there (or above: a cap's)
    why = self._plausible(None, new, i.source, self.candidate_trust, firm=True)
    if not why:
      if d is None or self.cruise_off_frame >= start:
        why = "cruise was not on from when the limit appeared until it was steady"
      elif exit_start is None or self.driver_set_frame >= exit_start:
        why = f"the driver set his speed ({round(d)}) himself since the {kind}"
      elif kind == "turn" and new - round(d) > self.first_max:
        why = f"limit {new} is more than {self.first_max} above the driver's speed {round(d)}"
    if why:
      self._not_followed(None, new, i.source, f"after the {kind}: {why}")
    else:
      self.offer = Offer(None, new, i.source, self.frame, kind, d, tgt=tgt)
      self.last_next = (i.next_limit, self.next_dist if self.next_dist is not None else -1.0)

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
    if self.candidate != new or self.candidate_frames < _frames(self._rise_time(old, new)):
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
    add = self._add()
    if d is not None and abs(d - new - add) < 0.5:
      return                                           # already at the new limit
    why = self._plausible(old, new, i.source, self.candidate_trust, firm=new - old > self.big_raise)
    if not why and d is None:
      why = "cruise off when the higher limit became steady"
    if not why and not self._in_band(d, old):
      why = f"driver's speed {round(d)} is not the old limit or up to {LIMIT_BAND * self.multiple} above"
    if why:
      self._not_followed(old, new, i.source, why)
    else:
      self.offer = Offer(old, new, i.source, self.frame, "raise", d, tgt=new + add)
      self.last_next = (i.next_limit, self.next_dist if self.next_dist is not None else -1.0)

  def _rise_time(self, old, new):
    """s a higher limit must be steady before it is offered: LIMIT_RISE_QUICK_TIME for a one-band rise (rule 1) with
    GPS trust 0 on every frame of it, else LIMIT_RAISE_STABLE_TIME."""
    quick = (old not in (None, "exit") and 0 < new - old <= self.big_raise and self.candidate_trust <= LIMIT_FIRM_TRUST_MAX and
             self.frame - self.undo_frame > _frames(LIMIT_QUICK_AFTER_UNDO))
    return LIMIT_RISE_QUICK_TIME if quick else LIMIT_RAISE_STABLE_TIME

  def _first_limit(self, new, start, d, i):
    """Rule 2: a first limit after none (or none for a long time)."""
    add = self._add()
    if d is not None and new + add - round(d) <= 0:
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
      self.offer = Offer(None, new, i.source, self.frame, "first", d, tgt=new + add)
      self.last_next = (i.next_limit, self.next_dist if self.next_dist is not None else -1.0)

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
    if abs(d - o.target) < 0.5:
      self.offer = None              # already there (the driver's own SET+: drive 52, t 321)
      return 0.0
    if abs(d - o.driver) < 0.5:
      return o.target * self.step
    self.offer = None                # the driver chose another speed: theirs
    self._not_followed(o.old, o.new, o.source, f"driver changed his speed to {round(d)}")
    return 0.0

  def mark_adopted(self):
    """The controller took the offer (or Test mode logged it): close it, and remember it for rule 4."""
    o = self.offer
    if o is not None and not (o.kind == "merge" and self.road in HIGHWAY_CLASSES):
      # (a merge taken on the highway is never undone; rule 6's undo goes to the lower limit, not back to the floor)
      self.raised = Offer(o.old, o.new, o.source, self.frame, o.kind, o.driver, tgt=o.tgt)
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
    return self.offer.target * self.step if self.offer is not None else 0.0

  @property
  def cap_speed(self):
    return self.cap.target * self.step if (self.cap is not None and self.cap.driver is not None) else 0.0

  @property
  def ceiling_speed(self):
    """A lower limit reported but not steady yet (m/s): no SET+ of ours above it, never a SET- (0: none)."""
    return self.ceiling * self.step if self.ceiling is not None else 0.0

  @property
  def undo_speed(self):
    """Rule 4: the driver's speed before our raise (m/s) while an undo is open, else 0."""
    return self.undo.new * self.step if self.undo is not None else 0.0
