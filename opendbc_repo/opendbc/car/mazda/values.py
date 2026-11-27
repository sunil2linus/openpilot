from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


# Steer torque limits

class CarControllerParams:
  STEER_MAX = 800                # theoretical max_steer 2047
  STEER_DELTA_UP = 10             # torque increase per refresh
  STEER_DELTA_DOWN = 25           # torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 1     # weight driver torque
  STEER_DRIVER_FACTOR = 1         # from dbc
  STEER_STEP = 1  # 100 Hz

  # Gen1 + MoreTorque Torque Interceptor (TI): the same command is also sent to the TI (CAM_LKAS2), so it uses the
  # proven mazda-frogpilot-dev limit of 600. The panda safety (mazda.h, MAZDA_TI_STEERING_LIMITS) uses the same limit.
  TI_STEER_MAX = 600
  TI_STEER_DRIVER_MULTIPLIER = 40  # weight driver torque (the TI's own sensor, see carstate)
  # Steering limit test (Stage 3, off by default): 700 with the TI and MazdaSafetyFlags.TI_STEER_700 in the safety param
  # (speed_flags.py sets it at car start; mazda.h MAZDA_TI_STEERING_LIMITS_700 uses the same limit).
  TI_STEER_MAX_TEST = 700

  def __init__(self, CP):
    # Without a TI the class values above are used unchanged (stock StarPilot).
    if CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      self.STEER_MAX = self.TI_STEER_MAX
      self.STEER_DRIVER_MULTIPLIER = self.TI_STEER_DRIVER_MULTIPLIER
      try:
        param = int(CP.safetyConfigs[0].safetyParam)
      except (AttributeError, IndexError, TypeError, ValueError):
        param = 0
      if param & MazdaSafetyFlags.TORQUE_INTERCEPTOR and param & MazdaSafetyFlags.TI_STEER_700:
        self.STEER_MAX = self.TI_STEER_MAX_TEST


@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.COMMA_POWER_REQUIRED])
  requires_comma_power: bool = True


class Footnote(Enum):
  COMMA_POWER_REQUIRED = CarFootnote(
    "Mazda vehicles <b>require</b> comma power installation.",
    Column.MAKE, setup_note=True)


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


class MazdaFlags(IntFlag):
  # Static flags
  # Gen 1 hardware: same CAN messages and same camera
  GEN1 = 1

  # Dynamic flags
  # Gen1 with a MoreTorque Torque Interceptor installed (set only by the TorqueInterceptorEnabled setting,
  # see torque_interceptor.py). Same bit as MazdaSafetyFlags.TORQUE_INTERCEPTOR.
  TORQUE_INTERCEPTOR = 8


class MazdaSafetyFlags(IntFlag):
  # panda safety param bits (opendbc/safety/modes/mazda.h MAZDA_PARAM_*, tests/test_safety_flags.py checks they match).
  # 0 is stock StarPilot. These bits are only in the safety param, never in CP.flags.
  TORQUE_INTERCEPTOR = 8
  # Stage 3, both off by default and set at car start from the Mazda speed control setting (speed_flags.py):
  AUTO_RESUME = 256     # the gas auto-resume window: RES alone after a brake cancel (mazda.h)
  TI_STEER_700 = 512    # steering limit test: TI limit 700 instead of 600 (ignored without TORQUE_INTERCEPTOR)
  # Always On Lateral with the TI (ti_aol.py): set at car start, only with TORQUE_INTERCEPTOR and alternative
  # experience 32, when the Always On Lateral setting is on. Ignored by the panda in any other combination.
  TI_AOL = 1024


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mazda_2017'})
  flags: int = MazdaFlags.GEN1


class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.5)
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=3.1, steerRatio=17.6)
  )
  MAZDA_3 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2017-18")],
    MazdaCarSpecs(mass=2875 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=14.0)
  )
  MAZDA_6 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 6 2017-20")],
    MazdaCarSpecs(mass=3443 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=15.5)
  )
  MAZDA_CX9_2021 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2021-23", video="https://youtu.be/dA3duO4a0O4")],
    MAZDA_CX9.specs
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-25")],
    MAZDA_CX5.specs,
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph
  TI_STEER_THRESHOLD = 6  # steeringPressed threshold on the TI's driver torque sensor


class TI_STATE:
  # TI_FEEDBACK.STATE
  DISCOVER = 0
  OFF = 1
  DRIVER_OVER = 2
  RUN = 3


class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4


FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"[A-Z0-9-]{11,16}\x00{8,13}",
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
