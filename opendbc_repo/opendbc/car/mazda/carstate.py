from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mazda.values import CAR, DBC, LKAS_LIMITS, MazdaFlags, TI_STATE
from opendbc.car.mazda.stock_aol import qualified

ButtonType = structs.CarState.ButtonEvent.Type

DASHBOARD_CARS = frozenset(CAR)
# Gen1 alone (stock) or Gen1 with a Torque Interceptor: both have the same camera.
DASHBOARD_FLAGS = (int(MazdaFlags.GEN1), int(MazdaFlags.GEN1 | MazdaFlags.TORQUE_INTERCEPTOR))


class DashboardCameraParser(CANParser):
  def __init__(self, dbc_name):
    super().__init__(dbc_name, [], 2)
    self.dashboard_parser = CANParser(dbc_name, [("CAM_TRAFFIC_SIGNS", float('nan'))], 2)

  def update(self, strings, sendcan=False):
    packets = strings
    if packets and not isinstance(packets[0], list | tuple):
      packets = [packets]
    result = super().update(packets, sendcan)
    admitted = [(stamp, [(address, data, bus) for address, data, bus in frames
                         if address == 0x35F and bus == 2 and len(data) == 8]) for stamp, frames in packets]
    self.dashboard_parser.update(admitted, sendcan)
    return result


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]

    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.lkas_allowed_speed = False

    self.distance_button = 0
    self.accel_button = 0
    self.decel_button = 0
    self.cancel_button = 0
    self.dashboard_stop_sign = None

    # Torque Interceptor (TI) state, from TI_FEEDBACK on bus 1
    self.torque_interceptor = bool(CP.flags & MazdaFlags.TORQUE_INTERCEPTOR)
    self.ti_version = 1
    self.ti_state = TI_STATE.RUN
    self.ti_violation = 0
    self.ti_error = 0
    self.ti_ramp_down = False
    self.ti_lkas_allowed = False

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FL"],
      cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"],
      cp.vl["WHEEL_SPEEDS"]["RR"],
    )

    # Match panda speed reading
    speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    ret.standstill = speed_kph <= .1

    can_gear = int(cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.genericToggle = bool(cp.vl["BLINK_INFO"]["HIGH_BEAMS"])
    ret.leftBlindspot = cp.vl["BSM"]["LEFT_BS_STATUS"] != 0
    ret.rightBlindspot = cp.vl["BSM"]["RIGHT_BS_STATUS"] != 0
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    if self.torque_interceptor:
      # Driver torque comes from the TI's own sensor. The panda safety (mazda.h) uses this same
      # signal (0x24A on bus 1) for its driver-torque limit, so the carcontroller limits use it too.
      cp_body = can_parsers[Bus.body]
      ti = cp_body.vl["TI_FEEDBACK"]
      ret.steeringTorque = ti["TI_TORQUE_SENSOR"]
      ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.TI_STEER_THRESHOLD

      self.ti_version = ti["VERSION_NUMBER"]
      self.ti_state = ti["STATE"]  # DISCOVER = 0, OFF = 1, DRIVER_OVER = 2, RUN = 3
      self.ti_violation = ti["VIOL"]  # 0 = no violation
      self.ti_error = ti["ERROR"]  # 0 = no error
      if self.ti_version > 1:
        self.ti_ramp_down = ti["RAMP_DOWN"] == 1
      self.ti_lkas_allowed = not self.ti_ramp_down and self.ti_state == TI_STATE.RUN
    else:
      ret.steeringTorque = cp.vl["STEER_TORQUE"]["STEER_TORQUE_SENSOR"]
      ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD

    ret.steeringTorqueEps = cp.vl["STEER_TORQUE"]["STEER_TORQUE_MOTOR"]
    ret.steeringRateDeg = cp.vl["STEER_RATE"]["STEER_ANGLE_RATE"]

    ret.brakePressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1

    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["DRIVER_SEATBELT"] == 0
    ret.doorOpen = any([cp.vl["DOORS"]["FL"], cp.vl["DOORS"]["FR"],
                        cp.vl["DOORS"]["BL"], cp.vl["DOORS"]["BR"]])

    # TODO: this should be from 0 - 1.
    ret.gasPressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0

    # Either due to low speed or hands off
    lkas_blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] == 1

    if self.CP.minSteerSpeed > 0:
      # LKAS is enabled at 52kph going up and disabled at 45kph going down
      # wait for LKAS_BLOCK signal to clear when going up since it lags behind the speed sometimes
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED and not lkas_blocked:
        self.lkas_allowed_speed = True
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    # TODO: the signal used for available seems to be the adaptive cruise signal, instead of the main on
    #       it should be used for carState.cruiseState.nonAdaptive instead
    ret.cruiseState.available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
    ret.cruiseState.enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1
    ret.cruiseState.standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS

    # stock lkas should be on
    # TODO: is this needed?
    # With a TI the stock LKAS state does not matter (mazda-frogpilot-dev ignores it and the TI steers down to
    # 0 mph; LANE_LINES == 0 was seen below ~28 mph), so "Invalid LKAS setting" is only raised without a TI.
    ret.invalidLkasSetting = cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0 and not self.torque_interceptor

    if not self.torque_interceptor:
      if ret.cruiseState.enabled:
        if not self.lkas_allowed_speed and self.acc_active_last:
          self.low_speed_alert = True
        else:
          self.low_speed_alert = False
      ret.lowSpeedAlert = self.low_speed_alert

    # Check if LKAS is disabled due to lack of driver torque when all other states indicate
    # it should be enabled (steer lockout). Don't warn until we actually get lkas active
    # and lose it again, i.e, after initial lkas activation.
    # With a TI, the EPS blocking stock LKAS (LKAS_BLOCK) is not a fault while the TI is running.
    ret.steerFaultTemporary = self.lkas_allowed_speed and lkas_blocked and not self.ti_lkas_allowed

    self.acc_active_last = ret.cruiseState.enabled

    self.crz_btns_counter = cp.vl["CRZ_BTNS"]["CTR"]

    # camera signals
    self.cam_lkas = cp_cam.vl["CAM_LKAS"]
    self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]
    # As on the owner's previous TI fork (mazda-frogpilot-dev): with a TI the camera's LKAS error bit is not a
    # permanent steering fault, the TI keeps steering.
    ret.steerFaultPermanent = cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1 and not self.torque_interceptor

    # cruise control button events: distance, inc, and dec
    prev_cancel_button = self.cancel_button
    self.cancel_button = cp.vl["CRZ_BTNS"]["CAN_OFF"]
    prev_distance_button = self.distance_button
    prev_accel_button = self.accel_button
    prev_decel_button = self.decel_button
    self.distance_button = cp.vl["CRZ_BTNS"]["DISTANCE_LESS"]
    self.accel_button = cp.vl["CRZ_BTNS"]["RES"]
    self.decel_button = cp.vl["CRZ_BTNS"]["SET_M"]

    ret.buttonEvents = [
      *(create_button_events(self.cancel_button, prev_cancel_button, {1: ButtonType.cancel}) if qualified(self.CP, marked_only=True) else []),
      *create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.accel_button, prev_accel_button, {1: ButtonType.accelCruise}),
      *create_button_events(self.decel_button, prev_decel_button, {1: ButtonType.decelCruise}),
    ]

    self.dashboard_stop_sign = None
    if isinstance(cp_cam, DashboardCameraParser):
      dashboard = cp_cam.dashboard_parser
      stamp = dashboard.ts_nanos["CAM_TRAFFIC_SIGNS"]["STOP_SIGN"]
      if stamp > 0:
        self.dashboard_stop_sign = (dashboard.vl["CAM_TRAFFIC_SIGNS"]["STOP_SIGN"] == 9, stamp)

    return ret

  @staticmethod
  def get_can_parsers(CP):
    parsers = {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: (DashboardCameraParser(DBC[CP.carFingerprint][Bus.pt])
                if CP.brand == 'mazda' and CP.carFingerprint in DASHBOARD_CARS and
                int(CP.flags) in DASHBOARD_FLAGS and not CP.notCar
                else CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2)),
    }
    if CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      # TI_FEEDBACK (0x24A) from the Torque Interceptor on bus 1 (the panda routes bus 1 to the OBD-II CAN pins)
      parsers[Bus.body] = CANParser(DBC[CP.carFingerprint][Bus.pt], [("TI_FEEDBACK", 50)], 1)
    return parsers
