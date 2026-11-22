#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/mazda_stock_aol.h"

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U
// Gen1 MoreTorque Torque Interceptor (TI), on bus 1 (the panda routes bus 1 to the OBD-II CAN pins, see main.c)
#define MAZDA_TI_LKAS       0x249U  // DBC: CAM_LKAS2, the TI steering command
#define MAZDA_TI_FEEDBACK   0x24AU  // DBC: TI_FEEDBACK, TI state and the TI's driver torque sensor

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_AUX  1
#define MAZDA_CAM  2

// safety param flags (opendbc/car/mazda/values.py: MazdaSafetyFlags). 0 is stock StarPilot.
#define MAZDA_PARAM_TORQUE_INTERCEPTOR 8U
#define MAZDA_PARAM_AUTO_RESUME 256U      // gas auto-resume window (Mazda speed control "auto_resume", off by default)
#define MAZDA_PARAM_TI_STEER_700 512U     // steering limit test 700, only together with the TI flag (off by default)

static bool mazda_torque_interceptor = false;
static bool mazda_auto_resume = false;
static bool mazda_ti_steer_700 = false;

// Gas auto-resume (stock MRCC, openpilot/starpilot/car/mazda/speed/auto_resume.py; port of the owner's previous fork).
// While controls are not allowed only CANCEL may be sent, except in this narrow window: after the driver's brake turned
// the cruise off (the cruise's falling edge with the brake pressed and the main switch on), with the main switch still
// on, the brake released and the gas pressed now, RES (0x04) alone may be sent, at most MAZDA_AR_MAX_FRAMES frames per
// brake cancel and only within MAZDA_AR_MAX_MOVING_US of moving time after it. The window closes when the cruise comes
// on or the main switch goes off; the frame budget is refilled only by a new cruise on -> brake cancel cycle (never by a
// new brake press). SET+ / SET- are never allowed. Without the flag nothing changes.
#define MAZDA_AR_MAX_FRAMES 8U
#define MAZDA_AR_MAX_MOVING_US 180000000U   // 180 s, counted only while the car moves
#define MAZDA_BTN_RES 0x04U                 // CRZ_BTNS byte 0: RES (bit 2) alone
static bool mazda_ar_window = false;        // the last cruise falling edge was a brake cancel, window still open
static bool mazda_ar_main_on = false;       // CRZ_CTRL CRZ_AVAILABLE (bit 17), read only with the flag
static bool mazda_ar_cruise_prev = false;
static bool mazda_ar_moving_prev = false;
static uint32_t mazda_ar_moving_us = 0U;    // moving time since that falling edge
static uint32_t mazda_ar_ts_last = 0U;
static uint32_t mazda_ar_frames = 0U;       // RES frames allowed in this window

static void mazda_auto_resume_cruise(bool cruise_engaged, bool main_on) {
  mazda_ar_main_on = main_on;
  if (cruise_engaged) {
    mazda_ar_window = false;
  } else if (mazda_ar_cruise_prev) {
    // falling edge: a window only for a brake cancel with the main switch on; a fresh budget and moving time
    mazda_ar_window = brake_pressed && main_on;
    mazda_ar_frames = 0U;
    mazda_ar_moving_us = 0U;
    mazda_ar_moving_prev = vehicle_moving;
    mazda_ar_ts_last = microsecond_timer_get();
  } else {
  }
  if (!main_on) {
    mazda_ar_window = false;
  }
  mazda_ar_cruise_prev = cruise_engaged;
}

static void mazda_auto_resume_time(void) {
  // count the time since the brake cancel while the car moves (standing still never counts)
  uint32_t ts = microsecond_timer_get();
  if (mazda_ar_window && mazda_ar_moving_prev) {
    uint32_t elapsed = safety_get_ts_elapsed(ts, mazda_ar_ts_last);
    if (elapsed > (MAZDA_AR_MAX_MOVING_US - mazda_ar_moving_us)) {
      mazda_ar_moving_us = MAZDA_AR_MAX_MOVING_US + 1U;
      mazda_ar_window = false;
    } else {
      mazda_ar_moving_us += elapsed;
    }
  }
  mazda_ar_ts_last = ts;
  mazda_ar_moving_prev = vehicle_moving;
}

static bool mazda_auto_resume_allowed(uint8_t button) {
  bool allowed = false;
  // not with the RX checks failing: gas / brake / cruise could be stale
  if (mazda_auto_resume && mazda_ar_window && (button == MAZDA_BTN_RES) && mazda_ar_main_on && !brake_pressed &&
      gas_pressed && !safety_rx_checks_invalid && (mazda_ar_moving_us <= MAZDA_AR_MAX_MOVING_US) &&
      (mazda_ar_frames < MAZDA_AR_MAX_FRAMES)) {
    mazda_ar_frames += 1U;
    allowed = true;
  }
  return allowed;
}

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  mazda_aol_rx(msg);
  if (msg_matches(msg, MAZDA_ENGINE_DATA, MAZDA_MAIN)) {
    // sample speed: scale by 0.01 to get kph
    int speed = (msg->data[2] << 8) | msg->data[3];
    vehicle_moving = speed > 10;  // moving when speed > 0.1 kph
    if (mazda_auto_resume) {
      mazda_auto_resume_time();
    }
  }

  // With a TI the driver torque comes from the TI's own sensor, not from the EPS
  if (msg_matches(msg, MAZDA_STEER_TORQUE, MAZDA_MAIN) && !mazda_torque_interceptor) {
    int torque_driver_new = msg->data[0] - 127U;
    // update array of samples
    update_sample(&torque_driver, torque_driver_new);
  }

  if (msg_matches(msg, MAZDA_TI_FEEDBACK, MAZDA_AUX) && mazda_torque_interceptor) {
    int torque_driver_new = msg->data[0] - 127;
    update_sample(&torque_driver, torque_driver_new);
  }

  // enter controls on rising edge of ACC, exit controls on ACC off
  if (msg_matches(msg, MAZDA_CRZ_CTRL, MAZDA_MAIN)) {
    bool cruise_engaged = msg->data[0] & 0x8U;
    const bool was_allowed = controls_allowed;
    pcm_cruise_check(cruise_engaged);
    mazda_aol_engagement(msg, cruise_engaged, !was_allowed && controls_allowed);
    if (mazda_auto_resume) {
      mazda_auto_resume_cruise(cruise_engaged, GET_BIT(msg, 17U));
    }
  }

  if (msg_matches(msg, MAZDA_ENGINE_DATA, MAZDA_MAIN)) {
    gas_pressed = (msg->data[4] | (msg->data[5] & 0xF0U)) != 0U;
  }

  if (msg_matches(msg, MAZDA_PEDALS, MAZDA_MAIN)) {
    brake_pressed = (msg->data[0] & 0x10U);
  }
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    .max_torque = 800,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // Gen1 + Torque Interceptor: carcontroller STEER_MAX is 600 (mazda-frogpilot-dev value) and the same
  // command goes to CAM_LKAS (bus 0) and to the TI as CAM_LKAS2 (bus 1), so both are limited to 600.
  const TorqueSteeringLimits MAZDA_TI_STEERING_LIMITS = {
    .max_torque = 600,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // Steering limit test (Mazda speed control setting "ti_steer_700", off by default; only with the TI flag, see
  // mazda_init): carcontroller STEER_MAX 700. Everything but max_torque is the same as MAZDA_TI_STEERING_LIMITS.
  const TorqueSteeringLimits MAZDA_TI_STEERING_LIMITS_700 = {
    .max_torque = 700,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  const TorqueSteeringLimits ti_limits = mazda_ti_steer_700 ? MAZDA_TI_STEERING_LIMITS_700 : MAZDA_TI_STEERING_LIMITS;
  const TorqueSteeringLimits limits = mazda_torque_interceptor ? ti_limits : MAZDA_STEERING_LIMITS;

  bool tx = true;
  // Check if msg is sent on the main BUS
  // steer cmd checks
  if (msg_matches(msg, MAZDA_LKAS, MAZDA_MAIN)) {
    int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

    if (steer_torque_cmd_checks(desired_torque, -1, limits)) {
      tx = false;
    }
  }

  // TI command (CAM_LKAS2): same 12-bit, offset 2048 encoding as CAM_LKAS. carcontroller sends the same value right
  // after CAM_LKAS in every frame, so the TI command must repeat the last CAM_LKAS value the checks above accepted
  // (desired_torque_last; 0 after a violation or without controls), and it is bounded by max_torque on its own. It is
  // not checked as a second torque command: that moved the one shared rate limiter twice per frame, so the two
  // messages taking turns could step twice max_rate_up per frame (Stage 3 review). It is only on the TX list with a TI.
  if (msg_matches(msg, MAZDA_TI_LKAS, MAZDA_AUX)) {
    int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;
    const bool torque_allowed = lateral_controls_allowed() || (desired_torque == 0);
    const bool over_max = safety_max_limit_check(desired_torque, limits.max_torque, -limits.max_torque);

    if ((desired_torque != desired_torque_last) || !torque_allowed || over_max) {
      tx = false;
    }
  }

  // cruise buttons check
  if (msg_matches(msg, MAZDA_CRZ_BTNS, MAZDA_MAIN)) {
    // allow resume spamming while controls allowed, but
    // only allow cancel while controls not allowed (and the auto-resume window's RES, see above)
    bool cancel_cmd = (msg->data[0] == 0x1U);
    if (!controls_allowed && !cancel_cmd) {
      // (MISRA 13.5: the check with a side effect, the window's frame budget, is not an operand of &&)
      tx = mazda_auto_resume_allowed(msg->data[0]);
    }
  }

  mazda_aol_tx_commit(msg, tx && !relay_malfunction);
  return tx;
}

static safety_config mazda_init(uint16_t param) {
  static CanMsg MAZDA_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
  };

  static const CanMsg MAZDA_TI_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_TI_LKAS, 1, 8, .check_relay = false},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
  };

  static const RxCheck mazda_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  // With a TI its feedback (driver torque) is required as well
  static const RxCheck mazda_ti_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_TI_FEEDBACK,  1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  mazda_torque_interceptor = GET_FLAG(param, MAZDA_PARAM_TORQUE_INTERCEPTOR);
  mazda_auto_resume = GET_FLAG(param, MAZDA_PARAM_AUTO_RESUME);
  mazda_ti_steer_700 = mazda_torque_interceptor && GET_FLAG(param, MAZDA_PARAM_TI_STEER_700);
  mazda_ar_window = false;
  mazda_ar_main_on = false;
  mazda_ar_cruise_prev = false;
  mazda_ar_moving_prev = false;
  mazda_ar_moving_us = 0U;
  mazda_ar_ts_last = 0U;
  mazda_ar_frames = 0U;

  mazda_aol_reset();
  mazda_aol_enabled = (param == 0U) && ((unsigned int)alternative_experience == 32U);
  MAZDA_TX_MSGS[0].check_relay = !mazda_aol_enabled;
  MAZDA_TX_MSGS[2].check_relay = !mazda_aol_enabled;
  if (mazda_aol_enabled) {
    static const AolSafetyPolicy policy = {
      .reset = mazda_aol_reset,
      .host_request = mazda_aol_request,
      .request_mask = mazda_aol_request_mask,
      .permission_mask = mazda_aol_permission,
      .rx_invalid = mazda_aol_clear,
    };
    aol_policy = &policy;
  }
  safety_config ret = BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
  if (mazda_torque_interceptor) {
    ret = BUILD_SAFETY_CFG(mazda_ti_rx_checks, MAZDA_TI_TX_MSGS);
  }
  if (((unsigned int)alternative_experience == 32U) && (param != 0U)) {
    ret.tx_msgs = NULL;
    ret.tx_msgs_len = 0;
  }
  return ret;
}

// Like mazda-frogpilot-dev: with a TI never pass its command (0x249) between the car and the camera. The command
// is also seen on the main bus, and forwarded to the camera it turned the camera's LKAS off (CAM_LANEINFO.LANE_LINES = 0)
// and the EPS then ignored CAM_LKAS (LKAS_BLOCK), so only the TI steered (owner's fork, commit f913837).
static bool mazda_fwd_hook(int bus_num, int addr) {
  return (mazda_torque_interceptor && (addr == (int)MAZDA_TI_LKAS)) || mazda_aol_fwd(bus_num, addr);
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
  .optional_rx = mazda_aol_optional_rx,
  .fwd = mazda_fwd_hook,
};
