"""Always On Lateral for a Gen1 Mazda with a MoreTorque Torque Interceptor (TI).

7.0's stock Mazda AOL (stock_aol.py) admits only the 2022 CX-5 / 2021 CX-9 with safety param 0. A TI car always has
the TI bit in its safety param, so it was excluded on purpose (mazda.h turns all TX off for AOL with any other param).
This profile is the same panda policy (opendbc/safety/modes/mazda_stock_aol.h) with three TI rules, selected by the
panda safety param bit MazdaSafetyFlags.TI_AOL (mazda.h MAZDA_PARAM_TI_AOL):

* the EPS's LKAS_BLOCK is not a source (below ~45 km/h the EPS blocks the stock path, the TI steers);
* a camera fault (CAM_LKAS ERR_BIT_1 from the camera) blocks and clears only the AOL token (steering with the cruise
  off); with the cruise engaged it changes nothing, as without AOL;
* openpilot always sends CAM_LKAS / CAM_LKAS2 and the HUD (the TI TX table), nothing from the camera is forwarded.

The bit is added at car start by card (AolVehiclePolicy.safety_param_addition, with alternative experience 32) only when
the Always On Lateral setting is on and the car qualifies here; so it takes effect at the next drive.

Unmarked (before card adds AOL): alternative experience 0, safety param TI (+ optional Stage 3 bits).
Marked (after): alternative experience 32, safety param TI | TI_AOL (+ the same optional bits). Nothing else qualifies.
"""
from opendbc.car import structs
from opendbc.car.mazda.torque_interceptor import TI_CARS
from opendbc.car.mazda.values import MazdaFlags, MazdaSafetyFlags

TI_FLAGS = int(MazdaFlags.GEN1 | MazdaFlags.TORQUE_INTERCEPTOR)
OPTIONAL_BITS = int(MazdaSafetyFlags.AUTO_RESUME | MazdaSafetyFlags.TI_STEER_700)
UNMARKED_BASE = int(MazdaSafetyFlags.TORQUE_INTERCEPTOR)
MARKED_BASE = int(MazdaSafetyFlags.TORQUE_INTERCEPTOR | MazdaSafetyFlags.TI_AOL)
ADDITION = int(MazdaSafetyFlags.TI_AOL)


def param_qualified(param: int, alternative_experience: int, *, marked_only: bool = False) -> bool:
  base = int(param) & ~OPTIONAL_BITS
  if int(alternative_experience) == 32:
    return base == MARKED_BASE
  return not marked_only and int(alternative_experience) == 0 and base == UNMARKED_BASE


def native_param(param: int) -> bool:
  """The safety params the panda runs this profile with (mazda.h mazda_ti_aol_param), alternative experience 32."""
  return 0 <= int(param) <= 0xFFFF and (int(param) & ~OPTIONAL_BITS) == MARKED_BASE


def qualified(cp, *, marked_only=False):
  # The fingerprint may be fixed (the owner's car is chosen in settings): every TI_CARS platform is Gen1 with the same
  # DBC and the same panda safety, and the panda reads every AOL condition from CAN itself.
  try:
    return bool(cp.brand == "mazda" and cp.carFingerprint in TI_CARS and int(cp.flags) == TI_FLAGS and
                not cp.passive and not cp.notCar and not cp.dashcamOnly and
                cp.transmissionType != structs.CarParams.TransmissionType.manual and
                cp.pcmCruise and not cp.openpilotLongitudinalControl and len(cp.safetyConfigs) == 1 and
                cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.mazda and
                param_qualified(int(cp.safetyConfigs[0].safetyParam), int(cp.alternativeExperience), marked_only=marked_only))
  except (AttributeError, TypeError, ValueError, IndexError, OverflowError):
    return False
