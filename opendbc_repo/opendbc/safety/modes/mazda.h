#pragma once

#include "opendbc/safety/declarations.h"

// CAN msgs we care about
/********** GEN1 msgs **********/
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U
#define TI_STEER_TORQUE     0x24AU
// Radar
#define MAZDA_CRZ_INFO      0x21BU
#define MAZDA_RADAR_361     0x361U
#define MAZDA_RADAR_362     0x362U
#define MAZDA_RADAR_363     0x363U
#define MAZDA_RADAR_364     0x364U
#define MAZDA_RADAR_365     0x365U
#define MAZDA_RADAR_366     0x366U
#define MAZDA_RADAR_499     0x499U
/************* GEN2 msgs *************/
#define MAZDA_2019_BRAKE          0x43FU  // main bus
#define MAZDA_2019_GAS            0x202U  // camera bus DBC: ENGINE_DATA
#define MAZDA_2019_CRUISE         0x44AU  // main bus. DBC: CRUISE_STATE
#define MAZDA_2019_SPEED          0x217U  // camera bus. DBC: SPEED
#define MAZDA_2019_STEER_TORQUE   0x24BU  // aux bus. DBC: TI_FEEDBACK
#define MAZDA_2019_CRZ_BTNS       0x9DU   // rx on main tx on camera. DBC: CRZ_BTNS
#define MAZDA_2019_ACC            0x220U  // main bus. DBC: ACC
#define MAZDA_TI_LKAS             0x249U
/************* GEN3 msgs *************/
#define MAZDA_2023_SPEED          0x215  // main bus. DBC: WHEEL_SPEEDS
#define MAZDA_2023_BRAKE          0x9F  // main bus. DBC: BRAKE_PEDAL
#define MAZDA_2023_ACC            0x21e  // main bus. DBC: ACC

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_AUX  1
#define MAZDA_CAM  2

// param flag masks
const int FLAG_GEN1 = 1;
const int FLAG_GEN2 = 2;
const int FLAG_GEN3 = 4;
const int FLAG_TORQUE_INTERCEPTOR = 8;
const int FLAG_RADAR_INTERCEPTOR = 16;
const int FLAG_NO_FSC = 32;
const int FLAG_NO_MRCC = 64;
const int FLAG_AUTO_RESUME = 256;   // MazdaSafetyFlags.AUTO_RESUME (setting MazdaBrakeAutoResume)

bool gen1 = false;
bool gen2 = false;
bool gen3 = false;
bool torque_interceptor = false;
bool radar_interceptor = false;
bool no_fsc = false;
bool no_mrcc = false;
bool auto_resume = false;

// Gen1 stock MRCC, gas auto-resume (opendbc/car/mazda/auto_resume.py, setting MazdaBrakeAutoResume, default off).
// While controls are not allowed only CANCEL may be sent, except in this narrow window: after the driver's brake
// turned the cruise off (the cruise's falling edge with the brake pressed), with the main switch still on, the brake
// released and the gas pressed now, RES (0x04) or SET- (0x20, it engages at the current speed) may be sent, at most
// MAZDA_AR_MAX_FRAMES frames per brake cancel and only within MAZDA_AR_MAX_MOVING_US of moving time after it. The
// window closes when the cruise comes on or the main switch goes off; the frame budget is refilled only by a new
// cruise on -> brake cancel cycle (never by a new brake press). SET+ is never allowed. Without the flag nothing changes.
#define MAZDA_AR_MAX_FRAMES 8U
#define MAZDA_AR_MAX_MOVING_US 180000000U   // 180 s, counted only while the car moves
#define MAZDA_BTN_RES 0x04U                 // CRZ_BTNS byte 0: RES (bit 2) alone
#define MAZDA_BTN_SET_MINUS 0x20U           // CRZ_BTNS byte 0: SET_M (bit 5) alone
static bool mazda_ar_window = false;        // the last cruise falling edge was a brake cancel, window still open
static bool mazda_ar_cruise_prev = false;
static bool mazda_ar_moving_prev = false;
static uint32_t mazda_ar_moving_us = 0U;    // moving time since that falling edge
static uint32_t mazda_ar_ts_last = 0U;
static uint32_t mazda_ar_frames = 0U;       // button frames allowed in this window

static void mazda_auto_resume_cruise(bool cruise_engaged) {
  if (cruise_engaged) {
    mazda_ar_window = false;
  } else if (mazda_ar_cruise_prev) {
    // falling edge: a window only for a brake cancel with the main switch on; a fresh budget and moving time
    mazda_ar_window = auto_resume && brake_pressed && acc_main_on;
    mazda_ar_frames = 0U;
    mazda_ar_moving_us = 0U;
    mazda_ar_moving_prev = vehicle_moving;
    mazda_ar_ts_last = microsecond_timer_get();
  } else {
  }
  if (!acc_main_on) {
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
  bool button_ok = (button == MAZDA_BTN_RES) || (button == MAZDA_BTN_SET_MINUS);
  if (auto_resume && gen1 && !no_mrcc && !radar_interceptor && mazda_ar_window && button_ok && acc_main_on &&
      !brake_pressed && gas_pressed && (mazda_ar_moving_us <= MAZDA_AR_MAX_MOVING_US) &&
      (mazda_ar_frames < MAZDA_AR_MAX_FRAMES)) {
    mazda_ar_frames += 1U;
    allowed = true;
  }
  return allowed;
}


// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if ((int)msg->bus == MAZDA_MAIN) {
    if (gen1) {
      if (msg->addr == MAZDA_ENGINE_DATA) {
        // sample speed: scale by 0.01 to get kph
        int speed = (msg->data[2] << 8) | msg->data[3];
        vehicle_moving = speed > 10; // moving when speed > 0.1 kph
        mazda_auto_resume_time();
      }

      if (msg->addr == MAZDA_STEER_TORQUE && !torque_interceptor) {
        int torque_driver_new = msg->data[0] - 127U;
        // update array of samples
        update_sample(&torque_driver, torque_driver_new);
      }

      // enter controls on rising edge of ACC, exit controls on ACC off
      if (msg->addr == MAZDA_CRZ_CTRL && !no_mrcc && !radar_interceptor) {
        acc_main_on = GET_BIT(msg, 17U);
        bool cruise_engaged = msg->data[0] & 0x8U;
        pcm_cruise_check(cruise_engaged);
        mazda_auto_resume_cruise(cruise_engaged);
      }

      if (msg->addr == MAZDA_ENGINE_DATA) {
        gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
      }

      if (msg->addr == MAZDA_PEDALS) {
        brake_pressed = (msg->data[0] & 0x10U);
        if (radar_interceptor || no_mrcc) {
          // acc_main_on = GET_BIT(to_push, 17U); TODO
          bool cruise_engaged = msg->data[0] & 0x8U;
          pcm_cruise_check(cruise_engaged);
        }
      }
    }

    if (gen2) {
      if (msg->addr == MAZDA_2019_BRAKE) {
        brake_pressed = (msg->data[5] & 0x4U);
      }

      if (msg->addr == MAZDA_2019_CRUISE) {
        acc_main_on = true;
        bool cruise_engaged = msg->data[0] & 0x20U;
        bool pre_enable = msg->data[0] & 0x40U;
        pcm_cruise_check((cruise_engaged || pre_enable));
      }
    }

    if (gen3) {
      if (msg->addr == MAZDA_2023_BRAKE) {
        brake_pressed = (msg->data[7] & 0x8U);
      }
      if (msg->addr == MAZDA_2023_SPEED) {
        int speed = ( (msg->data[0] << 8) | (msg->data[1]) ) - 10000; // Front Left Wheel Speed
        vehicle_moving = speed != 0;
      }
    }
  }

  if ((msg->bus == MAZDA_AUX)) {

    if (msg->addr == TI_STEER_TORQUE && gen1 && torque_interceptor) {
      int torque_driver_new = msg->data[0] - 127;
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    // Not for Gen1: there bus 1 is the Torque Interceptor / OBD bus and engagement comes from
    // CRZ_CTRL (or PEDALS) on the main bus, like mazda-frogpilot-dev.
    if ((msg->addr == MAZDA_CRZ_CTRL) && !gen1) {
      bool cruise_engaged = msg->data[0] & 0x8U;
      pcm_cruise_check(cruise_engaged);

      acc_main_on = GET_BIT(msg, 17U);
    }

    if (msg->addr == MAZDA_2019_STEER_TORQUE && (gen2 || gen3)) {
      update_sample(&torque_driver, (int16_t)(msg->data[0] << 8 | msg->data[1]));
    }

    if (msg->addr == MAZDA_2019_CRUISE && gen3) {
      uint8_t state = msg->data[0] & 0x70U;
      bool cruise_engaged = (0x30U == state);
      bool cruise_override = (0x40U == state);
      //bool cruise_disable = (state == 0x10U);
      acc_main_on = true;
      pcm_cruise_check(cruise_engaged || cruise_override);
    }
  }

  if ((msg->bus == MAZDA_CAM)) {
    if (gen2) {

      if (msg->addr == MAZDA_2019_GAS) {
        gas_pressed = (msg->data[4] || ((msg->data[5] & 0xC0U)));
      }

      if (msg->addr == MAZDA_2019_SPEED) {
        // sample speed: scale by 0.01 to get kph
        int speed = (msg->data[4] << 8) | (msg->data[5]);
        vehicle_moving = (speed > 10);  // moving when speed > 0.1 kph
      }
    }
    if (gen3) {
      if (msg->addr == MAZDA_2019_GAS) {
        gas_pressed = (msg->data[4] || (msg->data[5] & 0xC0U));
      }
    }
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

  const TorqueSteeringLimits MAZDA_2019_STEERING_LIMITS = {
    .max_torque = 8000,
    .max_rate_up = 45,
    .max_rate_down = 80,
    .max_rt_delta = 1688,  // (45*100hz*250000/1000000)*1.5
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 1400,
    .type = TorqueDriverLimited,
  };

  bool tx = true;
  const TorqueSteeringLimits gen1_limits = torque_interceptor ? MAZDA_TI_STEERING_LIMITS : MAZDA_STEERING_LIMITS;
  // Check if msg is sent on the main BUS
  if (msg->bus == (unsigned char)MAZDA_MAIN) {
    // steer cmd checks
    if (msg->addr == MAZDA_LKAS) {
      int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

      if (steer_torque_cmd_checks(desired_torque, -1, gen1_limits)) {
        tx = false;
      }
    }

    // cruise buttons check
    if (msg->addr == MAZDA_CRZ_BTNS) {
      // allow resume spamming while controls allowed, but
      // only allow cancel while controls not allowed (and the auto-resume window's RES / SET-, see above)
      bool cancel_cmd = (msg->data[0] == 0x1U);
      if (!controls_allowed && !cancel_cmd && !mazda_auto_resume_allowed(msg->data[0])) {
        tx = false;
      }
    }
  }
  // Gen1 Torque Interceptor command (DBC CAM_LKAS2): LKAS_REQUEST uses the same 12-bit, offset 2048
  // encoding as CAM_LKAS. It carries the same value as CAM_LKAS in the same frame, so checking it with
  // the same limits is a no-op for a valid command, and it can no longer bypass the torque checks
  // (mazda-frogpilot-dev only had the TX allow list for it).
  if (gen1 && torque_interceptor && (msg->bus == MAZDA_AUX) && (msg->addr == MAZDA_TI_LKAS)) {
    int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;
    if (steer_torque_cmd_checks(desired_torque, -1, gen1_limits)) {
      tx = false;
    }
  }

  if ((gen2 || gen3) && (msg->bus == MAZDA_AUX) && (msg->addr == MAZDA_TI_LKAS)) {
    int desired_torque = (int16_t)((msg->data[0] << 8) | msg->data[1]);  // signal is signed
    if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_2019_STEERING_LIMITS)) {
      tx = false;
    }
  }

  return tx;
}

static safety_config mazda_init(uint16_t param) {
  static const CanMsg MAZDA_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true}};
  static const CanMsg MAZDA_TI_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_TI_LKAS, 1, 8, .check_relay = false}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true}};
  static const CanMsg MAZDA_RI_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
                                    {MAZDA_CRZ_CTRL, 0, 8, .check_relay = true}, {MAZDA_CRZ_INFO, 0, 8, .check_relay = true}, {MAZDA_RADAR_361, 0, 8, .check_relay = true}, {MAZDA_RADAR_362, 0, 8, .check_relay = true},
                                    {MAZDA_RADAR_363, 0, 8, .check_relay = true}, {MAZDA_RADAR_364, 0, 8, .check_relay = true}, {MAZDA_RADAR_365, 0, 8, .check_relay = true}, {MAZDA_RADAR_366, 0, 8, .check_relay = true},
                                    {MAZDA_RADAR_499, 0, 8, .check_relay = true}};
  static const CanMsg MAZDA_TI_RI_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_TI_LKAS, 1, 8, .check_relay = false}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
                                  {MAZDA_CRZ_CTRL, 0, 8, .check_relay = true}, {MAZDA_CRZ_INFO, 0, 8, .check_relay = true}, {MAZDA_RADAR_361, 0, 8, .check_relay = true}, {MAZDA_RADAR_362, 0, 8, .check_relay = true},
                                  {MAZDA_RADAR_363, 0, 8, .check_relay = true}, {MAZDA_RADAR_364, 0, 8, .check_relay = true}, {MAZDA_RADAR_365, 0, 8, .check_relay = true}, {MAZDA_RADAR_366, 0, 8, .check_relay = true},
                                  {MAZDA_RADAR_499, 0, 8, .check_relay = true}};

  static const CanMsg MAZDA_2019_TX_MSGS[] = {{MAZDA_TI_LKAS, 1, 8, .check_relay = false}, {MAZDA_2019_ACC, 2, 8, .check_relay = true}};

  // Gen1 RX checks. CRZ_CTRL (0x21c) on the main bus is the engagement source with stock MRCC, so it
  // is only checked then. With a Radar Interceptor or NoMRCC engagement comes from PEDALS (0x165)
  // and CRZ_CTRL is not used by the safety, so it is not required (mazda-frogpilot-dev also dropped
  // it in TI mode). With a Torque Interceptor the TI feedback (0x24A, bus 1, driver torque) is required.
  static RxCheck mazda_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };
  static RxCheck mazda_no_mrcc_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };
  static RxCheck mazda_ti_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{TI_STEER_TORQUE,    1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };
  static RxCheck mazda_ti_no_mrcc_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{TI_STEER_TORQUE,    1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };
  static RxCheck mazda_2019_rx_checks[] = {
    {.msg = {{MAZDA_2019_BRAKE,         0, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_GAS,           2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_CRUISE,        0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_SPEED,         2, 8, 30U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_STEER_TORQUE,  1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  static RxCheck mazda_2023_rx_checks[] = {
    {.msg = {{MAZDA_2023_BRAKE,         0, 8, 5U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_GAS,           2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_CRUISE,        1, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2023_SPEED,         2, 8, 30U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_STEER_TORQUE,  1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
};

  // Check flags
  gen1 = GET_FLAG(param, FLAG_GEN1);
  gen2 = GET_FLAG(param, FLAG_GEN2);
  gen3 = GET_FLAG(param, FLAG_GEN3);
  radar_interceptor = GET_FLAG(param, FLAG_RADAR_INTERCEPTOR);
  torque_interceptor = GET_FLAG(param, FLAG_TORQUE_INTERCEPTOR);
  no_fsc = GET_FLAG(param, FLAG_NO_FSC);
  no_mrcc = GET_FLAG(param, FLAG_NO_MRCC);
  auto_resume = GET_FLAG(param, FLAG_AUTO_RESUME);
  mazda_ar_window = false;
  mazda_ar_cruise_prev = false;
  mazda_ar_moving_prev = false;
  mazda_ar_moving_us = 0U;
  mazda_ar_ts_last = 0U;
  mazda_ar_frames = 0U;

  safety_config ret = BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);

  if (gen1) {
    const bool stock_mrcc = !no_mrcc && !radar_interceptor;
    if (torque_interceptor) {
      if (stock_mrcc) {
        SET_RX_CHECKS(mazda_ti_rx_checks, ret);
      } else {
        SET_RX_CHECKS(mazda_ti_no_mrcc_rx_checks, ret);
      }
    } else {
      if (stock_mrcc) {
        SET_RX_CHECKS(mazda_rx_checks, ret);
      } else {
        SET_RX_CHECKS(mazda_no_mrcc_rx_checks, ret);
      }
    }

    if (radar_interceptor && torque_interceptor) {
      SET_TX_MSGS(MAZDA_TI_RI_TX_MSGS, ret);
    } else if (radar_interceptor) {
      SET_TX_MSGS(MAZDA_RI_TX_MSGS, ret);
    } else if (torque_interceptor) {
      SET_TX_MSGS(MAZDA_TI_TX_MSGS, ret);
    } else {
      SET_TX_MSGS(MAZDA_TX_MSGS, ret);
    }
  }

  if (gen2) {
    ret = BUILD_SAFETY_CFG(mazda_2019_rx_checks, MAZDA_2019_TX_MSGS);
  }
  if (gen3) {
    ret = BUILD_SAFETY_CFG(mazda_2023_rx_checks, MAZDA_2019_TX_MSGS);
  }

  return ret;
}

// Like mazda-frogpilot-dev: never pass the Torque Interceptor command (0x249) between the car and the camera.
// With a TI the command is also seen on the main bus, and the generic forwarding passed it to the camera at
// 100 Hz. The camera then turned its LKAS off (CAM_LANEINFO.LANE_LINES = 0, the dash "LKAS off" light that the
// LKAS button cannot clear) and the EPS ignored CAM_LKAS (STEER_RATE.LKAS_BLOCK = 1, LKAS_EFFECTIVE = 0), so only
// the TI steered: weaker steering in curves than on mazda-frogpilot-dev (drives 36-38).
static bool mazda_fwd_hook(int bus_num, int addr) {
  SAFETY_UNUSED(bus_num);
  return addr == MAZDA_TI_LKAS;
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
  .fwd = mazda_fwd_hook,
};