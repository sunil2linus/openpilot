"""Gen1 Mazda + MoreTorque Torque Interceptor (TI).

The TI sits between the EPS and the car. openpilot sends it the same steering command as the stock LKAS message
(CAM_LKAS2, 0x249, on bus 1), and it reports its state and its own driver torque sensor in TI_FEEDBACK (0x24A,
bus 1). The TI keeps steering below the EPS's 45 km/h LKAS cut-off, so a TI car has no minimum steer speed.

The TI is not detected from CAN: it is selected by the TorqueInterceptorEnabled setting (off by default) and
applied here before the car interface is created, like the other StarPilot startup preferences. Without the
setting the CarParams stay exactly as stock StarPilot builds them.
"""
from opendbc.car import structs
from opendbc.car.mazda.values import CAR, MazdaFlags, MazdaSafetyFlags

# All Gen1 platforms (the only Mazda generation in this port) can carry a Gen1 TI.
TI_CARS = frozenset(CAR)


def supported(cp) -> bool:
  try:
    return bool(cp.brand == "mazda" and cp.carFingerprint in TI_CARS and int(cp.flags) == int(MazdaFlags.GEN1) and
                not cp.notCar and len(cp.safetyConfigs) == 1 and
                cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.mazda and
                int(cp.safetyConfigs[0].safetyParam) == 0 and
                not cp.openpilotLongitudinalControl and cp.pcmCruise)
  except (AttributeError, TypeError, ValueError, IndexError):
    return False


def installed(cp) -> bool:
  return bool(cp.brand == "mazda" and cp.flags & MazdaFlags.TORQUE_INTERCEPTOR)


def prepare(cp, requested: bool):
  """Mark a Gen1 Mazda as TI-equipped when the owner selected it. Idempotent; a no-op when not requested."""
  if requested and supported(cp):
    cp.flags |= int(MazdaFlags.TORQUE_INTERCEPTOR)
    cp.safetyConfigs[0].safetyParam |= int(MazdaSafetyFlags.TORQUE_INTERCEPTOR)
    # The TI steers below the stock LKAS speed (the c4aa1083e low-speed dropout fix), so no minimum steer speed.
    # steerAtStandstill stays False like mazda-frogpilot-dev: no TI torque while stopped.
    cp.minSteerSpeed = 0.0
    # Stock 2017-21 Gen1 platforms are dashcam-only because the EPS drops LKAS below 45 km/h and locks out;
    # the TI removes that limit, so a TI car is controllable (as on the owner's previous fork).
    cp.dashcamOnly = False
  return cp
