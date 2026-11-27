from opendbc.car import structs
from opendbc.car.mazda import ti_aol
from opendbc.car.mazda.values import CAR, MazdaFlags

CARS = frozenset((CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021))


def qualified(cp, *, marked_only=False):
  return (cp.brand == "mazda" and cp.carFingerprint in CARS and int(cp.flags) == int(MazdaFlags.GEN1) and
          not cp.passive and not cp.notCar and not cp.dashcamOnly and
          cp.transmissionType != structs.CarParams.TransmissionType.manual and
          cp.fingerprintSource != structs.CarParams.FingerprintSource.fixed and
          cp.pcmCruise and not cp.openpilotLongitudinalControl and len(cp.safetyConfigs) == 1 and
          cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.mazda and
          cp.safetyConfigs[0].safetyParam == 0 and cp.alternativeExperience in ((32,) if marked_only else (0, 32)))


def aol_qualified(cp, *, marked_only=False):
  """Either Mazda AOL profile: stock (above) or the Torque Interceptor one (ti_aol.py)."""
  return qualified(cp, marked_only=marked_only) or ti_aol.qualified(cp, marked_only=marked_only)


# Pauses steering but keeps the driver's latch. For a TI car: gear, steerFaultTemporary (the TI not running while the
# EPS blocks LKAS) and standstill (steerAtStandstill is False) apply as on the stock cars; minSteerSpeed is 0.
def temporary_restriction(cp, state):
  return bool(state is not None and aol_qualified(cp, marked_only=True) and state.canValid and not state.canTimeout and
              not state.steerFaultPermanent and (state.gearShifter != structs.CarState.GearShifter.drive or
                                                 state.steerFaultTemporary or state.standstill or (cp.minSteerSpeed > 0 and state.vEgo < cp.minSteerSpeed)))


def native_observation(native, *, latched, panda_ready, restricted, now_ns, pending_since_ns):
  lost = not panda_ready or native is None
  if not latched or lost or restricted:
    return 0, lost, False
  if native.requestedLateral and native.lateralAllowed:
    return 0, False, False
  pending_since_ns = pending_since_ns or now_ns
  return pending_since_ns, False, now_ns - pending_since_ns >= 200_000_000
