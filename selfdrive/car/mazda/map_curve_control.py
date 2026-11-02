"""
Map curves on the Mazda set speed controller: turns the map curve fields of frogpilotPlan (published by
frogpilot/controls/lib/map_curves.py) into the inputs of CurveSlowdownLogic.

  * Curve ahead (or the one we are in): the set speed allowed now so that a gentle cruise slowdown
    (MAP_DECEL) reaches the curve's speed (this car's lateral limit) MAP_MARGIN before its entry. Same
    2 mph start gap / 1 mph end gap as the camera curve target.
  * Ramps (we are on an exit / entrance / interchange ramp and the curve is on it): may go past the
    driver's Max Drop, down to 25 mph, with a bigger SET- budget.
  * Navigation exit before the split: only with a turn signal on and within NAV_ARM_DISTANCE of the exit,
    the exit's tightest curve is planned the same way (never on navigation alone).
  * End of a ramp onto a highway: restore at once with SET+ holds (merge).
  * End of an exit onto a street with a known speed limit: within STREET_CAP_DISTANCE of the end, cap the
    set speed at the street's limit; once off the ramp, that lowered set speed becomes the driver's speed
    (never restored to the highway speed on a street).

Pure logic (no openpilot imports) so it can be unit tested; the wrapper in curve_slowdown.py feeds it.
"""
from dataclasses import dataclass

try:
  from openpilot.selfdrive.car.mazda.curve_target import MapCurveTarget, NO_CURVE, START_GAP
except ImportError:          # offline tests load this file directly
  from curve_target import MapCurveTarget, NO_CURVE, START_GAP

MPH = 0.44704
OFF, TEST, ON = 0, 1, 2
RAMP_FLOOR = 25 * MPH
RAMP_BUDGET = 60                # SET- presses for one ramp (70 -> 25 mph plus ignored presses)
NAV_ARM_DISTANCE = 600.0        # m before the exit
STREET_CAP_DISTANCE = 200.0     # m before the end of an exit onto a street
STATUS_CURVE, STATUS_RAMP_CURVE = 1, 2
RAMP_EXIT = 1


@dataclass
class MapFields:
  mode: int = OFF
  status: int = 0
  distance: float = 0.0
  radius: float = 0.0
  on_ramp: bool = False
  ramp_type: int = 0
  ramp_to_speed: float = 0.0
  ramp_to_highway: bool = False
  ramp_end_distance: float = 0.0
  nav_exit: int = 0
  nav_exit_distance: float = 0.0
  nav_curve_radius: float = 0.0
  nav_curve_distance: float = 0.0

  @staticmethod
  def from_plan(plan):
    return MapFields(int(plan.mapCurveMode), int(plan.mapCurveStatus), float(plan.mapCurveDistance),
                     float(plan.mapCurveRadius), bool(plan.mapOnRamp), int(plan.mapRampType),
                     float(plan.mapRampToSpeed), bool(plan.mapRampToHighway), float(plan.mapRampEndDistance),
                     int(plan.mapNavExit), float(plan.mapNavExitDistance), float(plan.mapNavCurveRadius),
                     float(plan.mapNavCurveDistance))


@dataclass
class MapOutput:
  active: bool = False          # a map reason wants the set speed lowered
  speed: float = NO_CURVE       # m/s, true speed
  floor: float = 0.0            # > 0: a ramp, may go this low past the max drop
  budget: int = 0
  fast_restore: bool = False
  adopt: bool = False           # off an exit onto a street: keep the lowered set speed as the driver's speed
  why: str = ""                 # curve / ramp / nav exit / street cap (for the logs)


class MapCurveControl:
  def __init__(self, lat_accel):
    self.curve = MapCurveTarget(lat_accel)
    self.nav = MapCurveTarget(lat_accel)
    self.on_street_exit = False

  def update(self, f: MapFields, driver_speed, blinker):
    out = MapOutput()
    if f.mode == OFF or driver_speed <= 0:
      self.curve.active = self.nav.active = False
      self.on_street_exit = f.on_ramp and self.on_street_exit
      return out
    radius = f.radius if f.status in (STATUS_CURVE, STATUS_RAMP_CURVE) else 0.0
    a, sp = self.curve.update(radius, f.distance, driver_speed)
    ramp_curve = a and f.status == STATUS_RAMP_CURVE and f.on_ramp
    armed = f.nav_exit == 1 and f.nav_curve_radius > 0 and blinker and not f.on_ramp and f.nav_exit_distance <= NAV_ARM_DISTANCE
    na, nsp = self.nav.update(f.nav_curve_radius if armed else 0.0, f.nav_curve_distance, driver_speed)
    speed, why = NO_CURVE, ""
    if a:
      speed, why = sp, ("ramp" if ramp_curve else "curve")
    if na and nsp < speed:
      speed, why = nsp, "nav exit"
    street_exit = f.on_ramp and f.ramp_type == RAMP_EXIT and not f.ramp_to_highway and f.ramp_to_speed > 0
    capped = False
    if street_exit and f.ramp_end_distance <= STREET_CAP_DISTANCE and f.ramp_to_speed < driver_speed - START_GAP:
      if f.ramp_to_speed < speed:
        speed, why = f.ramp_to_speed, "street cap"
      capped = True
    out.active = a or na or capped
    out.speed = speed
    if ramp_curve or na or capped:
      out.floor, out.budget = RAMP_FLOOR, RAMP_BUDGET
    out.fast_restore = f.on_ramp and f.ramp_to_highway and not out.active
    out.adopt = self.on_street_exit and not f.on_ramp
    self.on_street_exit = street_exit or (self.on_street_exit and f.on_ramp)
    out.why = why
    return out
