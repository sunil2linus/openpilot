"""
Curve target for the Mazda set speed slowdown.

FrogPilot's Curve Speed Controller plans curves at min(profile, CP.maxLateralAccel) on torque-controlled
cars, which is 1.09 m/s^2 on a CX-5 whatever profile is chosen, and it also adds the estimated road roll
(a steady 2 deg reading, probably the device mount, made curves of one direction look ~40 % sharper). With a
Torque Interceptor openpilot can steer far more than that (route 31: 3.4 m/s^2 at 45 mph through a 30 mph
curve with no help), so the slowdown computes its own target from the road curvature FrogPilot reads off
the model path (frogpilotPlan.roadCurvature, the point ahead that needs the most slowing down):

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
    """curvature: frogpilotPlan.roadCurvature; driver_speed: the driver's own set speed (m/s).
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
