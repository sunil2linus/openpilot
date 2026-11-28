from opendbc.car import structs
from opendbc.car.mazda import ti_aol
from opendbc.car.mazda.stock_aol import aol_qualified, qualified, temporary_restriction
from openpilot.starpilot.aol.policy import AolVehiclePolicy
from openpilot.starpilot.car.mazda.stock_intent import MazdaStockCardIntent


# Two profiles, the same intent and runtime: stock (2022 CX-5 / 2021 CX-9, safety param 0) and the Torque Interceptor
# one (opendbc/car/mazda/ti_aol.py: the TI_AOL safety param bit is added at car start, like alternative experience 32).
def policy_for(cp):
  admitted = aol_qualified(cp)
  ti = ti_aol.qualified(cp)
  return AolVehiclePolicy(intent_supported=admitted, settings_supported=admitted, runtime_supported=admitted,
                          normal_runtime_supported=admitted, ordinary_axis_ack_required=aol_qualified(cp, marked_only=True),
                          explicit_latch=admitted, alternative_experience_addition=32 if admitted else 0,
                          safety_param_addition=ti_aol.ADDITION if ti else 0)


def native_profile_supported(model, param):
  return model == int(structs.CarParams.SafetyModel.mazda) and (param == 0 or ti_aol.native_param(param))


def native_accepts_cp(cp, model, param):
  # Each profile accepts only its own native params (stock 0, TI the TI AOL params).
  if qualified(cp, marked_only=True):
    return model == int(structs.CarParams.SafetyModel.mazda) and param == 0
  return bool(ti_aol.qualified(cp, marked_only=True) and model == int(structs.CarParams.SafetyModel.mazda) and
              ti_aol.native_param(param))


def native_latch_rejected(cp, native):
  return aol_qualified(cp, marked_only=True) and native.requestedLateral and not native.lateralAllowed


def ordinary_axis_request_allowed(cp, state):
  return aol_qualified(cp, marked_only=True) and state.canValid and not state.canTimeout and not state.steerFaultPermanent


def retain_on_native_denial(cp, native, state):
  return temporary_restriction(cp, state)


def create_intent(cp, settings):
  return MazdaStockCardIntent(cp, settings)


def physical_cancel_lateral_allowed(cp, cs, intent, native, *, settings, previous,
                                    source_ns, now_ns, axis_session_id) -> bool:
  """Let the mapped intent owner apply Cancel while retaining a granted LAT axis."""
  from openpilot.starpilot.aol.intent import ButtonType
  from openpilot.starpilot.aol.runtime import (AXIS_MAX_AGE_NS, INTENT_MAX_AGE_NS, SAFETY_MAX_AGE_NS,
                                               native_inventory_matches_cp)
  try:
    return bool(qualified(cp, marked_only=True) and settings is not None and settings.enabled and
      previous is not None and previous.lateral_active and previous.native_acknowledged and
      intent is not None and intent.settingsQualified and intent.allowedLatch and intent.lateralArmed and
      not intent.pauseLateral and intent.producerSessionId and intent.carStateLogMonoTime == source_ns and
      0 < source_ns <= intent.observedMonoTime <= now_ns <= intent.validUntilMonoTime and
      now_ns - source_ns <= AXIS_MAX_AGE_NS and now_ns - intent.observedMonoTime <= INTENT_MAX_AGE_NS and
      intent.validUntilMonoTime - intent.observedMonoTime <= SAFETY_MAX_AGE_NS and
      cs.canValid and not cs.canTimeout and cs.cruiseState.available and not cs.steerFaultTemporary and
      not cs.steerFaultPermanent and not cs.accFaulted and not cs.doorOpen and
      any(event.type == ButtonType.cancel for event in cs.buttonEvents) and
      native is not None and native.compatible and int(native.protocolVersion) == 1 and
      axis_session_id and native.axisSessionId == axis_session_id and native.requestedLateral and native.lateralAllowed and
      native_accepts_cp(cp, native.safetyModel, native.safetyParam) and native.pandaInventory and
      native_inventory_matches_cp(native, cp) and
      0 < native.observedMonoTime <= now_ns <= native.validUntilMonoTime and
      now_ns - native.observedMonoTime <= SAFETY_MAX_AGE_NS and
      native.validUntilMonoTime - native.observedMonoTime <= SAFETY_MAX_AGE_NS)
  except (AttributeError, KeyError, IndexError, TypeError, ValueError, OverflowError):
    return False


def ti_expected_stopped_lateral_denial(cp, cs, native, *, sm, axis_session_id, now_ns):
  """A stop with the Torque Interceptor profile: the panda withdraws lateral at a standstill (vehicle_moving); that
  is an expected pause, not lost permission. Same rule as Hyundai's angle stop (vEgoRaw <= 0.3 m/s, where openpilot
  does not steer anyway). Intent only: the panda still decides every command."""
  import math
  from openpilot.starpilot.aol.transport_pause import _configured_transport_native
  try:
    return bool(cp.brand == 'mazda' and ti_aol.qualified(cp, marked_only=True) and
      cs.canValid and not cs.canTimeout and not cs.steerFaultTemporary and not cs.steerFaultPermanent and
      not cs.accFaulted and cs.gearShifter == structs.CarState.GearShifter.drive and
      math.isfinite(cs.vEgoRaw) and 0.0 <= cs.vEgoRaw <= 0.3 and
      native is not None and native.requestedLateral and not native.lateralAllowed and native.pandaInventory and
      _configured_transport_native(sm, cp, axis_session_id, now_ns, native, allow_restricted_lateral=True) == native)
  except (AttributeError, KeyError, IndexError, TypeError, ValueError, OverflowError):
    return False
