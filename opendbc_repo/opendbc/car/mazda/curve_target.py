"""
Curve target for the Mazda set speed slowdown.

FrogPilot's Curve Speed Controller plans curves at min(profile, CP.maxLateralAccel) on torque-controlled
cars, which is 1.09 m/s^2 on a CX-5 whatever profile is chosen, and it also adds the estimated road roll
(a steady 2 deg reading, probably the device mount, made curves of one direction look ~40 % sharper). With a
Torque Interceptor openpilot can steer far more than that (route 31: 3.4 m/s^2 at 45 mph through a 30 mph
curve with no help), so the slowdown computes its own target from the road curvature FrogPilot reads off
the model path (starpilotPlan.roadCurvature, the point ahead that needs the most slowing down):

  target = sqrt(lateral limit / |curvature|)

  * lateral limit: TI_LATERAL_ACCEL with a Torque Interceptor; without one the car's own steering limit
    (CP.maxLateralAccel, the same cap FrogPilot uses), so cars without a TI keep today's behaviour.
  * no roll term (symmetric for left and right curves; a level road reads ~0 anyway).
  * a curve must be seen for CONFIRM_TIME: the target used is the least demanding of the last second, so a
    brief spike in the model's curvature does nothing.
  * it only acts when the target is at least START_GAP below the driver's own set speed, and stops once
    the curve needs less than END_GAP (hysteresis).
  * nothing while a turn signal is on (turns at intersections, lane changes), like FrogPilot's curve flag.

No openpilot dependencies, so it can be unit tested.
"""
import math
from collections import deque

DT_CTRL = 0.01
MS_PER_MPH = 0.44704

TI_LATERAL_ACCEL = 3.0          # m/s^2 with a Torque Interceptor
DEFAULT_LATERAL_ACCEL = 2.0     # m/s^2 when the car reports no maxLateralAccel
MIN_LATERAL_ACCEL = 0.8
MAX_LATERAL_ACCEL = 3.0         # openpilot's MAX_LATERAL_ACCEL_NO_ROLL
CONFIRM_TIME = 1.0              # s a curve must be seen before it counts
START_GAP = 2.0 * MS_PER_MPH    # m/s below the driver's speed before we act
END_GAP = 1.0 * MS_PER_MPH      # m/s: the curve is over once it needs less than this
MIN_CURVATURE = 1e-5            # 1/m (100 km radius)
NO_CURVE = 1000.0               # m/s: "no limit"


def lateral_limit(has_ti, max_lateral_accel):
  """Lateral acceleration the curve target is planned at."""
  if has_ti:
    return TI_LATERAL_ACCEL
  base = max_lateral_accel if max_lateral_accel and max_lateral_accel > 0 else DEFAULT_LATERAL_ACCEL
  return min(max(base, MIN_LATERAL_ACCEL), MAX_LATERAL_ACCEL)


MINIMUM_PLANNED_SPEED = 2.0     # m/s: plan points slower than this are stop approaches, not curves
CRUISING_SPEED = 5.0            # m/s


def model_road_curvature(vel, rate, pos_x, pos_y, v_ego, lat_accel):
  """Road curvature (1/m) of the model plan point that needs the most slowing down from v_ego, the way
  FrogPilot computes frogpilotPlan.roadCurvature (frogpilot_utilities.calculate_road_curvature, without its roll
  term). StarPilot's starpilotPlan.roadCurvature is a different quantity (peak planned lateral acceleration over
  the current speed squared), so the slowdown computes this itself from modelV2 to keep the FrogPilot tuning."""
  n = min(len(vel), len(rate), len(pos_x), len(pos_y))
  if n == 0:
    return 0.0
  t_div = max(v_ego, CRUISING_SPEED)
  dist = 0.0
  best_req, best_req_k = 0.0, None
  min_speed, min_speed_k = None, 0.0
  for i in range(n):
    if i > 0:
      dist += math.hypot(pos_x[i] - pos_x[i - 1], pos_y[i] - pos_y[i - 1])
    k = rate[i] / max(vel[i], 1.0) if vel[i] >= MINIMUM_PLANNED_SPEED else 0.0
    cs = max(math.sqrt(lat_accel / max(abs(k), 1e-6)), CRUISING_SPEED)
    req = (v_ego - cs) / max(dist / t_div, 1.0)
    if req > best_req:
      best_req, best_req_k = req, k
    if min_speed is None or cs < min_speed:
      min_speed, min_speed_k = cs, k
  return float(best_req_k if best_req_k is not None else min_speed_k)


def curve_speed(curvature, lat_accel):
  """Speed (m/s) at which a curve of `curvature` (1/m) gives `lat_accel` (m/s^2)."""
  k = abs(curvature)
  if not math.isfinite(k) or k < MIN_CURVATURE:
    return NO_CURVE
  return math.sqrt(lat_accel / k)


class CurveTarget:
  def __init__(self, lat_accel, confirm_time=CONFIRM_TIME):
    self.lat_accel = lat_accel
    # (round P, curve_slowdown.TI_ONLY_*) with a Torque Interceptor: every curve while ti_only (the stock LKAS blocked, the
    # camera's fault) is planned at ti_lat (only the TI steers there); None: off
    self.ti_lat = None
    self.ti_only = False
    self.window = deque(maxlen=max(1, int(round(confirm_time / DT_CTRL))))
    self.active = False
    self.speed = NO_CURVE          # the least demanding target of the last CONFIRM_TIME (m/s, true speed)
    self.raw = NO_CURVE            # this frame's target

  def reset(self):
    self.window.clear()
    self.active = False
    self.speed = self.raw = NO_CURVE

  def update(self, curvature, driver_speed, plan_alive=True, blinker=False):
    """curvature: starpilotPlan.roadCurvature; driver_speed: the driver's own set speed (m/s).
    Returns (active, target speed m/s)."""
    if not plan_alive:
      self.reset()
      return False, NO_CURVE
    self.raw = NO_CURVE if blinker else curve_speed(curvature, self.lat_accel)
    if (self.ti_lat is not None and self.ti_lat < self.lat_accel and self.raw < NO_CURVE and
        self.ti_only):
      self.raw = curve_speed(curvature, self.ti_lat)
    self.window.append(self.raw)
    self.speed = max(self.window)
    if not self.active:
      self.active = len(self.window) == self.window.maxlen and self.speed <= driver_speed - START_GAP
    else:
      self.active = self.speed < driver_speed - END_GAP
    return self.active, self.speed

  @property
  def low(self):
    """The most demanding target of the last CONFIRM_TIME (m/s): a single straight frame does not hide a curve."""
    return min(self.window) if self.window else NO_CURVE


# ---------------- map curves (from starpilotPlan, see starpilot/controls/lib/map_curves.py) ----------------

MAP_LATERAL_ACCEL = 2.2       # m/s^2: map curves are planned at most at this (route 35/36: the 30 mph curve on West
                              # Utica at 45 mph = 3.2 m/s^2 with the steering at 100 % for 3.5-4.5 s, out of the lane once)
# Planned slowdown for a road curve: where the slowdown starts. Drive 38 (Dobry Drive, 45 -> 39 mph in 1 mph taps)
# showed the stock cruise follows small set speed steps at only ~0.28 m/s^2, so the plan was made gentle (0.25) and
# the staircase long. Drive 50 (West Utica, 45 mph, the 91 m curve held at 35 by the Curve Max Drop) showed the
# other side of that: the slowdown began 380-395 m (17-21 s) before the curve and the car crawled down 1 mph at a
# time, "slowing way before the curve" (owner). Measured on drives 44/45/4c/4d/4f/50 (cruise on, no lead, no pedals,
# over 27 mph), the cruise slows in proportion to how far the set speed is below the true speed, about 0.19 m/s^2
# per m/s of gap (2-4 mph: 0.27, 6-8: 0.66, 10-12: 0.84, 14-16: 0.95 m/s^2), about a second after the press. So the
# plan is now firm (MAP_DECEL) and the set speed is commanded below the plan (MapCurveFollower, below) so the cruise
# delivers that decel: the slowdown starts about half as far out and the car is still at the curve's speed
# MAP_LEAD_TIME before the entry.
# Ramp curves and navigation exits plan with it too since drive 53 (their 0.5 m/s^2 staircase left the set speed only
# 5-9 mph under the speed: the car slowed 0.4-0.8 m/s^2 and entered an 85 m exit curve at 38-39 mph for 31).
MAP_DECEL = 0.45              # m/s^2: what a 5-6 mph set speed gap gives on this cruise (6-8 mph: 0.66 m/s^2 measured)
MAP_LEAD_TIME = 1.0           # s: be at the curve's speed this long before its entry (slow before the curve, not in it)
# Round L: ramp curves (on a ramp, or an on-ramp's first curve ahead) 0.5 s, the planner's map_curves.PLAN_RAMP_LEAD_TIME
# (navigation exits keep MAP_LEAD_TIME: their split / chime planning is built on it)
MAP_RAMP_LEAD_TIME = 0.5      # s
MAP_REACTION_TIME = 1.5       # s: the stock cruise starts slowing about this long after a SET- press
ADVISORY_OVER = 10 * MS_PER_MPH  # m/s: never more than this over a yellow advisory sign the map has for the curve
# map curves start at the first 1 mph step (the map does not flicker, and a later start would need a bigger, quicker
# drop to catch up)
MAP_START_GAP = 1.0 * MS_PER_MPH  # m/s
# The follower (road curves, ramp curves and navigation exits; the camera curve slowdown keeps its staircase):
#  * MAP_FOLLOW_GAIN: the measured 0.19 m/s^2 per m/s of gap, taken a little lower so the gap asked for errs on the
#    firm side (a bigger gap slows more; the undershoot bounds below keep it in check).
#  * MAP_FOLLOW_DEAD_TIME: the cruise answers a set speed change about a second later; the room counted on for the
#    required decel starts after it.
#  * MAP_UNDERSHOOT: the set speed is never asked more than this below the curve's speed, so the car cannot run away
#    below it, and the lowest set speed asked for is at most this far under the driver's Curve Max Drop floor
#    (the curve's speed is never planned below that floor).
#  * MAP_FOLLOW_MAX_DECEL: the gap asked for never means more than this. Until the cruise answers (the dead time) the
#    car has not slowed while the room has shrunk, so a pure feedback would escalate; for a short 5 mph drop it ran the
#    gap to the undershoot bound (10 mph, ~0.8 m/s^2 on the measured cruise) where 0.5 was needed. Capped, the measured
#    cruise peaks at ~0.65; a car that would need more arrives a little fast and the camera slowdown has the rest.
#  * MAP_SETTLE: this close to the curve's speed the set speed goes back up to it (SET+ taps): the taps take ~1 s and
#    the cruise answers them a second later, during which it slows ~2 mph more at 0.45 m/s^2 (the simulated measured
#    cruise enters West Utica at 35.1 for 35 with 2.0; with 1.5 it dipped to 34.0 first).
MAP_FOLLOW_GAIN = 0.17        # (m/s^2) per m/s of set speed gap
MAP_FOLLOW_DEAD_TIME = 1.0    # s
MAP_UNDERSHOOT = 5 * MS_PER_MPH   # m/s
MAP_SETTLE = 2.0 * MS_PER_MPH     # m/s
MAP_FOLLOW_MAX_DECEL = 0.6    # m/s^2: a 7.9 mph gap at MAP_FOLLOW_GAIN
MAP_SETTLE_HYST = 1.0 * MS_PER_MPH  # m/s: once settled, only this much above the settle speed asks for a gap again (no churn)
MAP_MIN_ROOM = 5.0            # m: the required decel is never computed over less than this (a curve we are nearly in)
MAP_FOLLOW_BUDGET = 40        # SET- presses for one followed curve (max drop + undershoot + ignored presses, also in km/h)
# Road curves (not ramps or exits, which keep the values above): drive 54, West Utica Road (the 30 mph curve, 35 by the
# memory cap / Max Drop) at 45.4 mph: the follow began 190-204 m out as planned, but the set speed went 3 under the curve
# (the command 5 under), the car slowed up to 0.82 m/s^2 (the cruise there gave ~0.55 m/s^2 at a 1-2 m/s gap, about
# twice the model above) and was at the curve's speed 43-60 m (3-4 s) before the entry; owner: "we could slow down a
# little later, it brakes a little too much". So for road curves: the undershoot is 3 mph, the curve's speed is reached
# MAP_ROAD_LEAD_TIME before the entry (car side and planner), and the follower asks for no lower set speed while the car
# already slows at least as hard as the room left needs (aEgo filtered over MAP_ROAD_ACCEL_TAU, MapCurveFollower
# closed_loop): a cruise that answers harder than the model is not pushed further.
MAP_ROAD_UNDERSHOOT = 1 * MS_PER_MPH           # m/s (drives 56 / 59: was 3 mph, the set went 45 -> 32 -> 35 for a 35 curve)
MAP_ROAD_LEAD_TIME = 0.5      # s
MAP_ROAD_ACCEL_TAU = 0.5      # s
# Firm follow (ramp curves and navigation exits only, owner's exit design): when the room left needs more than
# MAP_FOLLOW_MAX_DECEL, the set speed is taken well below so the stock cruise slows at its strongest. Measured: the
# cruise slows ~0.19 m/s^2 per m/s of gap, capped at ~1.0 m/s^2 (drives 44-53: a set speed held far below gave 0.9-1.0,
# never more), so a gap of ~13 mph at MAP_FOLLOW_GAIN already gives all it has. The command keeps that gap as the car
# slows, at most FIRM_UNDERSHOOT below the curve's speed (the ramp floor still holds), and steps back up to the curve's
# speed FIRM_SETTLE_TIME of the current slowdown early (the taps up take ~1-2 s and the cruise answers a second later).
FIRM_MAX_DECEL = 1.0          # m/s^2
FIRM_UNDERSHOOT = 8 * MS_PER_MPH  # m/s
# ... for a curve taken under FIRM_LOW_CURVE only FIRM_UNDERSHOOT_LOW (drive 69: the dash showed 26 for a 33 mph exit
# curve the car took at 33; at that gap the cruise already slows ~0.9 m/s^2 of its ~1.0)
FIRM_UNDERSHOOT_LOW = 6 * MS_PER_MPH  # m/s
FIRM_LOW_CURVE = 40 * MS_PER_MPH      # m/s


def firm_undershoot(v_curve):
  """The firm follow's undershoot (m/s) for a curve taken at v_curve (m/s)."""
  return FIRM_UNDERSHOOT_LOW if (v_curve or 0.0) < FIRM_LOW_CURVE else FIRM_UNDERSHOOT
FIRM_SETTLE_TIME = 1.6        # s


def cruise_decel(gap):
  """The stock cruise's slowdown (m/s^2) with the set speed `gap` m/s below the true speed (measured, see above)."""
  return min(max(gap, 0.0) * 0.19, FIRM_MAX_DECEL)


def map_curve_speed(radius, distance, lat_accel, advisory=0.0, reaction_speed=0.0, decel=MAP_DECEL,
                    lead_time=MAP_LEAD_TIME, reaction_time=MAP_REACTION_TIME, floor=0.0):
  """Set speed allowed now (m/s, true speed) for a map curve of `radius` m starting `distance` m ahead
  (`advisory`: its yellow sign speed in m/s, 0 if the map has none; `reaction_speed`: the speed driven while the
  cruise reacts, the driver's set speed; `floor`: the lowest speed the set speed may be taken to, the driver's Curve
  Max Drop: the plan aims for that, not lower). The cruise reacts, then slows at `decel` and reaches the curve's
  speed `lead_time` before the entry."""
  if radius <= 0:
    return NO_CURVE
  v_curve = map_curve_target(radius, lat_accel, advisory, floor)
  room = distance - v_curve * lead_time - reaction_time * max(reaction_speed, v_curve)
  return math.sqrt(v_curve ** 2 + 2 * decel * max(room, 0.0))


def map_curve_target(radius, lat_accel, advisory=0.0, floor=0.0):
  """The speed (m/s, true) a map curve is taken at: its shape at lat_accel, at most the advisory sign + ADVISORY_OVER,
  and never below `floor` (the driver's Curve Max Drop, 0 for none)."""
  v_curve = math.sqrt(lat_accel * radius)
  if advisory > 0:
    v_curve = min(v_curve, advisory + ADVISORY_OVER)
  return max(v_curve, floor)


def map_follow_speed(v_ego, v_curve, distance, lead_time=MAP_LEAD_TIME, dead_time=MAP_FOLLOW_DEAD_TIME,
                     gain=MAP_FOLLOW_GAIN, undershoot=MAP_UNDERSHOOT, settle=MAP_SETTLE, max_decel=MAP_FOLLOW_MAX_DECEL):
  """The set speed (m/s, true) to ask the cruise for now so that it is at `v_curve` `lead_time` before the curve's
  entry `distance` m ahead: the decel needed over the room left (after the cruise's dead time), at most `max_decel`,
  turned into the set speed gap that makes the cruise slow that hard, bounded below by `v_curve` - `undershoot`.
  Within `settle` of the curve's speed the answer is the curve's speed itself."""
  if v_ego <= v_curve + settle:
    return v_curve
  room = max(distance - v_curve * lead_time - v_ego * dead_time, MAP_MIN_ROOM)
  required = min((v_ego ** 2 - v_curve ** 2) / (2 * room), max_decel)
  return max(v_ego - required / gain, v_curve - undershoot)


class MapCurveTarget:
  """Like CurveTarget, without the 1 s confirmation and starting at the first 1 mph step (the map does not flicker)."""

  def __init__(self, lat_accel, decel=MAP_DECEL):
    self.lat_accel = lat_accel
    self.decel = decel
    self.active = False
    self.speed = NO_CURVE

  def update(self, radius, distance, driver_speed, advisory=0.0, decel=None, floor=0.0, lead_time=MAP_LEAD_TIME,
             reaction_speed=None):
    """reaction_speed: the speed the cruise's reaction time is counted at (None: the driver's speed)."""
    react = driver_speed if reaction_speed is None else reaction_speed
    self.speed = map_curve_speed(radius, distance, self.lat_accel, advisory, reaction_speed=react,
                                 decel=self.decel if decel is None else decel, floor=floor, lead_time=lead_time)
    if not self.active:
      self.active = radius > 0 and self.speed <= driver_speed - MAP_START_GAP
    else:
      self.active = radius > 0 and self.speed < driver_speed - END_GAP
    return self.active, self.speed


class MapCurveFollower:
  """The set speed to command for the curve (road, ramp or exit) MapCurveTarget made active: map_follow_speed, never
  above the plan, and only ever going down until the car is nearly at the curve's speed (then up to it), so the buttons
  are not churned by small changes of the required decel. Forgotten when the curve changes or the slowdown ends.

  max_decel / undershoot / lead_time: the follow's own bounds (the gentle slowdown before an exit's split uses smaller
  ones). firm: may turn firm (FIRM_*) when the room left needs more than max_decel (ramp curves, exits). closed_loop:
  no lower command while the car already slows at least as hard as the room left needs (road curves, MAP_ROAD_*)."""

  def __init__(self, max_decel=MAP_FOLLOW_MAX_DECEL, undershoot=MAP_UNDERSHOOT, lead_time=MAP_LEAD_TIME, firm=False,
               closed_loop=False):
    self.max_decel, self.undershoot, self.lead_time, self.can_firm = max_decel, undershoot, lead_time, firm
    self.closed_loop = closed_loop
    self.cmd = None
    self.v_curve = None
    self.settled = False          # the car is (nearly) at the curve's speed: the command is the curve's speed
    self.firm = False             # this curve needs the cruise's strongest slowdown (FIRM_*)

  def reset(self):
    self.cmd = self.v_curve = None
    self.settled = self.firm = False

  @property
  def low(self):
    """The lowest the command may go (m/s): the curve's speed less the undershoot in use."""
    return (self.v_curve or 0.0) - (firm_undershoot(self.v_curve) if self.firm else self.undershoot)

  def update(self, v_ego, v_curve, distance, plan_speed, a_ego=None):
    """a_ego: the car's filtered acceleration (m/s^2) for closed_loop (None: not used)."""
    if self.v_curve is None or abs(v_curve - self.v_curve) > 0.05:
      self.cmd, self.settled, self.firm = None, False, False   # a new curve (or its speed changed)
    self.v_curve = v_curve
    settle = MAP_SETTLE
    if self.firm and self.cmd is not None:
      # the car keeps slowing at the current gap's rate while the taps go up and the cruise answers
      settle = max(MAP_SETTLE, cruise_decel(v_ego - self.cmd) * FIRM_SETTLE_TIME)
    if v_ego <= v_curve + settle:
      self.settled = True
    elif v_ego > v_curve + settle + MAP_SETTLE_HYST:
      self.settled = False        # clearly too fast again (a downhill, a slow landing): ask for a gap again
    if self.settled:
      self.cmd = v_curve
      return self.cmd
    room = max(distance - v_curve * self.lead_time - v_ego * MAP_FOLLOW_DEAD_TIME, MAP_MIN_ROOM)
    if self.can_firm and not self.firm and (v_ego ** 2 - v_curve ** 2) / (2 * room) > self.max_decel:
      self.firm, self.cmd = True, None
    if self.firm:
      want = map_follow_speed(v_ego, v_curve, distance, lead_time=self.lead_time, max_decel=FIRM_MAX_DECEL,
                              undershoot=firm_undershoot(v_curve), settle=0.0)
    else:
      want = map_follow_speed(v_ego, v_curve, distance, lead_time=self.lead_time, max_decel=self.max_decel,
                              undershoot=self.undershoot)
    want = min(want, plan_speed)  # the plan is never below the curve's speed
    if (self.closed_loop and a_ego is not None and self.cmd is not None and want < self.cmd and
        -a_ego >= (v_ego ** 2 - v_curve ** 2) / (2 * room)):
      want = self.cmd             # already slowing as hard as the room left needs: no further SET-
    if self.cmd is None or want < self.cmd:
      self.cmd = want             # down with the car only (small rises of the command do not move the buttons)
    return self.cmd
