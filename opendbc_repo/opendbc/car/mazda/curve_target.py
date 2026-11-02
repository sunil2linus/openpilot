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
    self.window.append(self.raw)
    self.speed = max(self.window)
    if not self.active:
      self.active = len(self.window) == self.window.maxlen and self.speed <= driver_speed - START_GAP
    else:
      self.active = self.speed < driver_speed - END_GAP
    return self.active, self.speed


# ---------------- map curves (from starpilotPlan, see starpilot/controls/lib/map_curves.py) ----------------

MAP_LATERAL_ACCEL = 2.2       # m/s^2: map curves are planned at most at this (route 35/36: the 30 mph curve on West
                              # Utica at 45 mph = 3.2 m/s^2 with the steering at 100 % for 3.5-4.5 s, out of the lane once)
MAP_DECEL = 0.5               # m/s^2: planned slowdown, a little gentler than the stock cruise's own (0.6-0.7 measured), so
                              # the set speed comes down 1 mph about every second and the car follows it smoothly
MAP_LEAD_TIME = 1.0           # s: be at the curve's speed this long before its entry (slow before the curve, not in it)
MAP_REACTION_TIME = 1.5       # s: the stock cruise starts slowing about this long after a SET- press
ADVISORY_OVER = 10 * MS_PER_MPH  # m/s: never more than this over a yellow advisory sign the map has for the curve
MAP_START_GAP = 1.0 * MS_PER_MPH  # m/s: map curves start at the first 1 mph step (the map does not flicker, and a later
                                  # start would need a bigger, quicker drop to catch up)


def map_curve_speed(radius, distance, lat_accel, advisory=0.0, reaction_speed=0.0, decel=MAP_DECEL,
                    lead_time=MAP_LEAD_TIME, reaction_time=MAP_REACTION_TIME):
  """Set speed allowed now (m/s, true speed) for a map curve of `radius` m starting `distance` m ahead
  (`advisory`: its yellow sign speed in m/s, 0 if the map has none; `reaction_speed`: the speed driven while the
  cruise reacts, the driver's set speed). The cruise reacts, then slows at `decel` and reaches the curve's speed
  `lead_time` before the entry."""
  if radius <= 0:
    return NO_CURVE
  v_curve = math.sqrt(lat_accel * radius)
  if advisory > 0:
    v_curve = min(v_curve, advisory + ADVISORY_OVER)
  room = distance - v_curve * lead_time - reaction_time * max(reaction_speed, v_curve)
  return math.sqrt(v_curve ** 2 + 2 * decel * max(room, 0.0))


class MapCurveTarget:
  """Like CurveTarget, without the 1 s confirmation and starting at the first 1 mph step (the map does not flicker)."""

  def __init__(self, lat_accel):
    self.lat_accel = lat_accel
    self.active = False
    self.speed = NO_CURVE

  def update(self, radius, distance, driver_speed, advisory=0.0):
    self.speed = map_curve_speed(radius, distance, self.lat_accel, advisory, reaction_speed=driver_speed)
    if not self.active:
      self.active = radius > 0 and self.speed <= driver_speed - MAP_START_GAP
    else:
      self.active = radius > 0 and self.speed < driver_speed - END_GAP
    return self.active, self.speed
