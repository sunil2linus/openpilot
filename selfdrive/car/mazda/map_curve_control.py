"""
Map curves on the Mazda set speed controller: turns the map curve fields of frogpilotPlan (published by
frogpilot/controls/lib/map_curves.py) into the inputs of CurveSlowdownLogic.

  * Curve ahead (or the one we are in): the set speed allowed now so that a gentle cruise slowdown
    (MAP_DECEL) reaches the curve's speed (this car's lateral limit) MAP_MARGIN before its entry. Same
    2 mph start gap / 1 mph end gap as the camera curve target.
  * Ramps (we are on an exit / entrance / interchange ramp and the curve is on it): may go past the
    driver's Max Drop, down to 25 mph, with a bigger SET- budget.
  * Navigation exit before the split: never on navigation alone. Once the driver signals toward the exit's side
    (any side if navigation does not say) while navigation announces it (up to 1500 m ahead), the exit is
    remembered: the signal may go off again after moving into the exit lane. The exit's tightest curve is then
    planned the same way (so the slowdown starts only as late as that curve allows). Forgotten when the car is
    on the ramp (the ramp's own curves take over), when navigation no longer shows that exit for NAV_GRACE
    (passed it, rerouted), or when the driver signals the other way.
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
NAV_GRACE = 3.0                 # s the exit plan is kept while navigation drops it (map match catching up at the split)
DT_CTRL = 0.01
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
  nav_exit_side: int = 0          # -1 left, 1 right, 0 unknown
  advisory: float = 0.0           # m/s, yellow sign speed of the curve (0 none)
  nav_curve_advisory: float = 0.0

  @staticmethod
  def from_plan(plan):
    return MapFields(int(plan.mapCurveMode), int(plan.mapCurveStatus), float(plan.mapCurveDistance),
                     float(plan.mapCurveRadius), bool(plan.mapOnRamp), int(plan.mapRampType),
                     float(plan.mapRampToSpeed), bool(plan.mapRampToHighway), float(plan.mapRampEndDistance),
                     int(plan.mapNavExit), float(plan.mapNavExitDistance), float(plan.mapNavCurveRadius),
                     float(plan.mapNavCurveDistance), int(getattr(plan, "mapNavExitSide", 0)),
                     float(getattr(plan, "mapCurveAdvisory", 0.0)), float(getattr(plan, "mapNavCurveAdvisory", 0.0)))


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
    self.exit_intent = False      # the driver signalled for the announced exit
    self.nav_grace = 0
    self.last_nav = (0.0, 0.0, 0.0)  # radius, distance, advisory of the remembered exit curve

  def _exit_intent(self, f, signal):
    """signal: -1 left, 1 right, 0 none. Returns (radius, distance, advisory) of the exit curve to plan, or zeros."""
    nav_ok = f.nav_exit == 1 and f.nav_curve_radius > 0 and not f.on_ramp
    if signal != 0 and f.nav_exit_side != 0 and signal == -f.nav_exit_side:
      self.exit_intent = False    # signalling the other way: not this exit
    elif nav_ok and signal != 0:
      self.exit_intent = True
    if not self.exit_intent:
      return 0.0, 0.0, 0.0
    if nav_ok:
      self.nav_grace = 0
      self.last_nav = (f.nav_curve_radius, f.nav_curve_distance, f.nav_curve_advisory)
      return self.last_nav
    self.nav_grace += 1
    if f.on_ramp or self.nav_grace > NAV_GRACE / DT_CTRL:
      self.exit_intent = False
      return 0.0, 0.0, 0.0
    return self.last_nav          # brief drop-out: keep the last plan (a ceiling, the ramp curve is lower)

  def update(self, f: MapFields, driver_speed, signal):
    """signal: turn signal side, -1 left, 1 right, 0 none."""
    out = MapOutput()
    if f.mode == OFF or driver_speed <= 0:
      self.curve.active = self.nav.active = False
      self.exit_intent = False
      self.on_street_exit = f.on_ramp and self.on_street_exit
      return out
    radius = f.radius if f.status in (STATUS_CURVE, STATUS_RAMP_CURVE) else 0.0
    a, sp = self.curve.update(radius, f.distance, driver_speed, f.advisory)
    ramp_curve = a and f.status == STATUS_RAMP_CURVE and f.on_ramp
    nav_r, nav_d, nav_adv = self._exit_intent(f, int(signal))
    na, nsp = self.nav.update(nav_r, nav_d, driver_speed, nav_adv)
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
