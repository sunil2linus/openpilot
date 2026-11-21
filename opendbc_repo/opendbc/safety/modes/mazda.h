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

static bool mazda_torque_interceptor = false;

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  mazda_aol_rx(msg);
  if (msg_matches(msg, MAZDA_ENGINE_DATA, MAZDA_MAIN)) {
    // sample speed: scale by 0.01 to get kph
    int speed = (msg->data[2] << 8) | msg->data[3];
    vehicle_moving = speed > 10;  // moving when speed > 0.1 kph
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

  const TorqueSteeringLimits limits = mazda_torque_interceptor ? MAZDA_TI_STEERING_LIMITS : MAZDA_STEERING_LIMITS;

  bool tx = true;
  // Check if msg is sent on the main BUS
  // steer cmd checks
  if (msg_matches(msg, MAZDA_LKAS, MAZDA_MAIN)) {
    int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

    if (steer_torque_cmd_checks(desired_torque, -1, limits)) {
      tx = false;
    }
  }

  // TI command (CAM_LKAS2): same 12-bit, offset 2048 encoding as CAM_LKAS and the same value in the same
  // frame, so checking it with the same limits passes a valid command and it cannot bypass the torque checks.
  // It is only on the TX list with a TI.
  if (msg_matches(msg, MAZDA_TI_LKAS, MAZDA_AUX)) {
    int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

    if (steer_torque_cmd_checks(desired_torque, -1, limits)) {
      tx = false;
    }
  }

  // cruise buttons check
  if (msg_matches(msg, MAZDA_CRZ_BTNS, MAZDA_MAIN)) {
    // allow resume spamming while controls allowed, but
    // only allow cancel while controls not allowed
    bool cancel_cmd = (msg->data[0] == 0x1U);
    if (!controls_allowed && !cancel_cmd) {
      tx = false;
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
