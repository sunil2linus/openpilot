"""Stage 3 safety param bits of a Gen1 Mazda, set once at car start (the panda's safety param is fixed for the drive).

Both come from the Mazda speed control setting (MazdaSpeedControlConfig, openpilot/starpilot/car/mazda/speed/config.py),
read by openpilot/starpilot/vehicle_preferences.py and passed in here as two booleans, after the Torque Interceptor's
own prepare() (torque_interceptor.py). Both are off by default; with both off the CarParams are not touched.

* AUTO_RESUME (256): the panda's gas auto-resume window (mazda.h). Stock radar cruise only (pcmCruise, no openpilot
  longitudinal). The car side (speed/auto_resume.py) runs only when this bit is in the safety param.
* TI_STEER_700 (512): the steering limit test, only with the TI bit set. The panda limit and carcontroller's STEER_MAX
  go from 600 to 700 (values.py CarControllerParams). The torque controller works in lateral acceleration and sends
  lat / latAccelFactor x STEER_MAX, so latAccelFactor is scaled by 700/600 and friction by 600/700 (friction is applied
  as friction x latAccelFactor): the torque for any lateral acceleration below the old ceiling, and the friction
  torque, stay as with 600; only the ceiling moves (the owner's previous fork, round U10).

These bits are never put in CP.flags (the dashboard, AOL and TI checks compare CP.flags exactly).
"""
from opendbc.car import structs
from opendbc.car.mazda.values import CarControllerParams, MazdaFlags, MazdaSafetyFlags

STAGE3_BITS = int(MazdaSafetyFlags.AUTO_RESUME | MazdaSafetyFlags.TI_STEER_700)
STEER_700_SCALE = CarControllerParams.TI_STEER_MAX_TEST / CarControllerParams.TI_STEER_MAX   # 700 / 600


def _mazda_gen1(cp) -> bool:
  try:
    return bool(cp.brand == "mazda" and int(cp.flags) & int(MazdaFlags.GEN1) and not cp.notCar and
                len(cp.safetyConfigs) == 1 and cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.mazda and
                cp.pcmCruise and not cp.openpilotLongitudinalControl)
  except (AttributeError, TypeError, ValueError, IndexError):
    return False


def safety_param(cp) -> int:
  try:
    return int(cp.safetyConfigs[0].safetyParam) if len(cp.safetyConfigs) else 0
  except (AttributeError, TypeError, ValueError, IndexError):
    return 0


def auto_resume(cp) -> bool:
  return bool(_mazda_gen1(cp) and safety_param(cp) & int(MazdaSafetyFlags.AUTO_RESUME))


def ti_steer_700(cp) -> bool:
  p = safety_param(cp)
  return bool(_mazda_gen1(cp) and p & int(MazdaSafetyFlags.TORQUE_INTERCEPTOR) and p & int(MazdaSafetyFlags.TI_STEER_700))


def prepare(cp, *, auto_resume_requested: bool = False, steer_700_requested: bool = False):
  """Set the requested bits where they apply. Idempotent; a no-op when nothing is requested."""
  if not (auto_resume_requested or steer_700_requested) or not _mazda_gen1(cp):
    return cp
  param = safety_param(cp)
  if auto_resume_requested and not param & int(MazdaSafetyFlags.AUTO_RESUME):
    cp.safetyConfigs[0].safetyParam = param | int(MazdaSafetyFlags.AUTO_RESUME)
  param = safety_param(cp)
  if (steer_700_requested and param & int(MazdaSafetyFlags.TORQUE_INTERCEPTOR) and
      int(cp.flags) & int(MazdaFlags.TORQUE_INTERCEPTOR) and not param & int(MazdaSafetyFlags.TI_STEER_700)):
    cp.safetyConfigs[0].safetyParam = param | int(MazdaSafetyFlags.TI_STEER_700)
    torque = cp.lateralTuning.torque
    torque.latAccelFactor = float(torque.latAccelFactor) * STEER_700_SCALE
    torque.friction = float(torque.friction) / STEER_700_SCALE
  return cp
