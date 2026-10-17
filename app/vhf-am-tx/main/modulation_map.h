#ifndef AM_TRANSMITTER_MAIN_MODULATION_MAP_H_
#define AM_TRANSMITTER_MAIN_MODULATION_MAP_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// x -> PWM duty mapping (spec 6.2) and the hard duty ceiling.
//
// Kept free of any IDF dependency so the host tests can exercise exactly the
// code the ISR runs. Both functions run in the 25kHz tick.

#include <esp_attr.h>

#include <cstdint>

#include "hardware_config.h"

namespace transmitter_system {
namespace modulation_map {

/// Clamps a raw duty register value to the hard ceiling. Every path that
/// writes a duty without going through XToDuty() has to pass through this.
inline uint32_t IRAM_ATTR ClampDuty(uint32_t duty) {
  return duty > hardware_config::kDutyLimit ? hardware_config::kDutyLimit
                                              : duty;
}

/// Converts the modulation input x to a duty register value.
///
///   duty = kModKCal * (1 + x) * 256
///
/// First order error feedback dither (spec 6.4) carries the rounding error
/// into the next sample, which moves the quantisation noise out of the audio
/// band. Where the result is clamped the error is dropped instead: carrying
/// it through a clamp would release one large correction the moment the clamp
/// lets go.
inline uint32_t IRAM_ATTR XToDuty(float x, float* dither_state) {
  float target = hardware_config::kModKCal * (1.0f + x) *
                 static_cast<float>(hardware_config::kPwmFullScale);
  if (dither_state) {
    target += *dither_state;
  }

  const int32_t reg = static_cast<int32_t>(target + 0.5f);

  if (reg < 0) {
    if (dither_state) *dither_state = 0.0f;
    return 0;
  }
  if (reg > static_cast<int32_t>(hardware_config::kDutyLimit)) {
    if (dither_state) *dither_state = 0.0f;
    return hardware_config::kDutyLimit;
  }

  if (dither_state) {
    // |error| <= 0.5 for any value that reaches here; the bound is only a
    // guard against something upstream feeding a NaN or a huge x.
    float err = target - static_cast<float>(reg);
    if (err > 1.0f) err = 1.0f;
    if (err < -1.0f) err = -1.0f;
    *dither_state = err;
  }
  return static_cast<uint32_t>(reg);
}

}  // namespace modulation_map
}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_MODULATION_MAP_H_
