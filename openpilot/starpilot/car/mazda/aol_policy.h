#pragma once

#include "selfdrive/pandad/aol_protocol.h"

// The panda runs Mazda AOL with two safety params (opendbc/safety/modes/mazda.h, both with alternative experience 32):
// stock AOL with 0, and the Torque Interceptor AOL profile with TORQUE_INTERCEPTOR (8) | TI_AOL (1024), optionally with
// the Stage 3 bits AUTO_RESUME (256) / TI_STEER_700 (512) and nothing else (mazda_ti_aol_param; Python:
// opendbc/car/mazda/ti_aol.py native_param).
inline bool mazda_aol_param(uint16_t param) {
  const uint16_t optional = 256U | 512U;
  return (param == 0U) || ((param & static_cast<uint16_t>(~optional)) == (8U | 1024U));
}

inline constexpr AolSafetyProfile MAZDA_AOL_PROFILE{13U, mazda_aol_param, true};
