from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.carlog import carlog
from opendbc.car.mazda.values import CarControllerParams, Buttons, MazdaSafetyFlags
from openpilot.common.realtime import DT_CTRL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.ccp = CarControllerParams(CP)
    self.hold_timer_frame = 0
    self.hold_delay_frame = 0
    self.resume_timer_frame = 0
    self.acc_filter = FirstOrderFilter(0.0, .1, DT_CTRL, initialized=False)
    self.filtered_acc_last = 0
    self.long_active_last = False
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")

    # Optional: lower the stock cruise set speed for curves, map curves and stopped cars ahead with
    # SET-/SET+ button presses. Gen1 with stock MRCC only (no Radar Interceptor, no openpilot
    # longitudinal). All default off.
    self.curve_slowdown = None
    if CP.flags & MazdaSafetyFlags.GEN1 and not CP.flags & (MazdaSafetyFlags.RADAR_INTERCEPTOR | MazdaSafetyFlags.NO_MRCC) and \
       not CP.openpilotLongitudinalControl:
      try:
        from opendbc.car.mazda.curve_slowdown import CurveSlowdown

        def mode(key):
          try:
            v = int(self.params.get(key, encoding="utf8") or 0)
          except (TypeError, ValueError):
            v = 0
          return v if v in (0, 1, 2) else 0

        curve_enabled = self.params.get_bool("MazdaCurveSlowdown")
        stop_mode = mode("MazdaStoppedCarSlowdown")
        learning = self.params.get_bool("MazdaCurveLearning")
        map_mode = mode("MazdaMapCurves")
        # needs StarPilot's Show Speed Limits so starpilotPlan carries the posted limit (speed_limit_follow.py)
        limit_mode = mode("MazdaSpeedLimitFollow")
        # the model follower: Test (log only) unless turned off; it never presses (model_follow.py)
        from opendbc.car.mazda.model_follow import mode_from_params
        follow_mode = mode_from_params(self.params)
        if curve_enabled or stop_mode or learning or map_mode or limit_mode or follow_mode:
          self.curve_slowdown = CurveSlowdown(CP, self.params, curve_enabled=curve_enabled, stop_mode=stop_mode,
                                              learning=learning, map_mode=map_mode, limit_mode=limit_mode,
                                              follow_mode=follow_mode)
      except Exception:
        carlog.exception("mazda set speed slowdown init failed, disabled")
        self.curve_slowdown = None

    # Optional: the gas brings the cruise back after his brake (auto_resume.py; MazdaBrakeAutoResume, default off; the
    # panda allows its RES only with the flag the car interface sets for it).
    self.auto_resume = None
    try:
      from opendbc.car.mazda import auto_resume
      if auto_resume.enabled(CP, self.params):
        self.auto_resume = auto_resume.AutoResume(CP, self.params)
    except Exception:
      carlog.exception("mazda auto resume init failed, disabled")
      self.auto_resume = None


  def update(self, CC, CS, now_nanos, starpilot_toggles):
    can_sends = []

    apply_torque = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * self.ccp.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, self.ccp)

    self.apply_torque_last = apply_torque

    if self.CP.flags & MazdaSafetyFlags.GEN1:
      cancel_sent = False
      if CC.cruiseControl.cancel:
        # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
        # a race condition with the stock system, where the second cancel from openpilot
        # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
        # read 3 messages and most likely sync state before we attempt cancel.
        self.brake_counter = self.brake_counter + 1
        if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
          # Cancel Stock ACC if it's enabled while OP is disengaged
          # Send at a rate of 10hz until we sync with stock ACC state
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
          cancel_sent = True
      else:
        self.brake_counter = 0
        if CC.cruiseControl.resume and self.frame % 5 == 0:
          # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
          # Send Resume button when planner wants car to move
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

      if self.auto_resume is not None:
        try:
          can_sends.extend(self.auto_resume.update(CC, CS, self.packer, self.CP, self.curve_slowdown, cancel_sent))
        except Exception:
          carlog.exception("mazda auto resume failed, disabling")
          self.auto_resume = None

      if self.curve_slowdown is not None:
        try:
          # (auto resume) never both sending: ours going or just gone holds the slowdown's presses back
          ours = self.auto_resume is not None and self.auto_resume.busy
          other_button_active = bool(CC.cruiseControl.cancel or CC.cruiseControl.resume or ours)
          can_sends.extend(self.curve_slowdown.update(CC, CS, self.packer, self.CP, other_button_active))
        except Exception:
          # Never let this optional feature affect steering: turn it off for the rest of the drive.
          carlog.exception("mazda curve slowdown failed, disabling")
          self.curve_slowdown = None

      # send HUD alerts
      if self.frame % 50 == 0:
        ldw = CC.hudControl.visualAlert == VisualAlert.ldw
        steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
        # TODO: find a way to silence audible warnings so we can add more hud alerts
        steer_required = steer_required and CS.lkas_allowed_speed
        if not self.CP.flags & MazdaSafetyFlags.NO_FSC:
          can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

      if self.CP.openpilotLongitudinalControl:
        hold = False
        if CS.out.standstill:
          hold = (self.frame - self.hold_timer_frame) < 600
        else:
          self.hold_timer_frame = self.frame

          stock_acc = CS.crz_info["ACCEL_CMD"]
          op_acc = CC.actuators.accel * 1150
          op_acc = max(-1000, min(op_acc, 1000))

          if self.params.get_bool("BlendedACC"):
            if CC.longActive:
              if not self.long_active_last:
                self.acc_filter.initialized = False
              target_acc = op_acc if self.params_memory.get_int("CEStatus") != 0 else stock_acc
              acc_output = self.acc_filter.update(target_acc)
            else:
              acc_output = stock_acc
          else:
            acc_output = op_acc

          acc_output = max(-1000, min(acc_output, 1000))
          CS.crz_info["ACCEL_CMD"] = acc_output

        if self.frame % 2 == 0:
          can_sends.extend(mazdacan.create_radar_command(self.packer, self.frame, CC.longActive, CS, hold))

    elif self.CP.flags & MazdaSafetyFlags.GEN2:
      if self.CP.openpilotLongitudinalControl:
        stock_acc = CS.acc["ACCEL_CMD"]
        op_acc = (CC.actuators.accel * 200) + 2000

        if self.params.get_bool("BlendedACC"):
          if CC.longActive:
            if not self.long_active_last:
              self.acc_filter.initialized = False
            target_acc = op_acc if self.params_memory.get_int("CEStatus") != 0 else stock_acc
            acc_output = self.acc_filter.update(target_acc)
          else:
            acc_output = stock_acc
        else:
          acc_output = op_acc if CC.longActive else stock_acc

        CS.acc["ACCEL_CMD"] = acc_output

      resume = False
      hold = False
      if self.frame % 2 == 0: # send ACC command at 50hz
        """
        Without this hold/resum logic, the car will only stop momentarily.
        It will then start creeping forward again. This logic allows the car to
        apply the electric brake to hold the car. The hold delay also fixes a
        bug with the stock ACC where it sometimes will apply the brakes too early
        when coming to a stop.
        """
        if CS.out.standstill: # if we're stopped
          if not ((self.frame - self.hold_delay_frame) < 50): # and we have been stopped for more than hold_delay duration. This prevents a hard brake if we aren't fully stopped.
            if ((CC.cruiseControl.resume and CC.actuators.longControlState != LongCtrlState.stopping) or
                CC.cruiseControl.override or CS.out.gasPressed or
                (CC.actuators.longControlState == LongCtrlState.starting) or CS.acc["RESUME"]): # if we are resuming or overriding, we want to release the brake
              self.resume_timer_frame = self.frame # reset the resume timer so its active
            else: # otherwise we're holding
              hold = (self.frame - self.hold_timer_frame) < 600 # hold for 6s. This allows the electric brake to hold the car.

        else: # if we're moving
          self.hold_timer_frame = self.frame # reset the hold timer so its active when we stop
          self.hold_delay_frame = self.frame # reset the hold delay

        resume = (self.frame - self.resume_timer_frame) < 50 # stay on for 0.5s to release the brake. This allows the car to move.
        can_sends.append(mazdacan.create_acc_cmd(self.packer, CS.acc, hold, resume))


    # send steering command
    # Gen1 + Torque Interceptor: like mazda-frogpilot-dev, the TI (CAM_LKAS2, bus 1) gets the very same
    # command as CAM_LKAS on bus 0. StarPilot-dev only sent a non-zero TI command while the TI already
    # reported RUN (ti_lkas_allowed); below the stock LKAS speed (~28 mph) the stock EPS ignores CAM_LKAS,
    # so the TI is the only actuator there, and that gating left it with zero commands and a
    # steerFaultTemporary ("steering temporarily unavailable"). The panda checks both messages
    # against the same limits (opendbc/safety/modes/mazda.h).
    can_sends.extend(mazdacan.create_steering_control(
      self.packer, self.CP, self.frame, apply_torque, CS.cam_lkas,
      apply_torque if self.CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR else None))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / self.ccp.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.long_active_last = CC.longActive
    self.frame += 1
    return new_actuators, can_sends