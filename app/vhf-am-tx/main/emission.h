#ifndef AM_TRANSMITTER_MAIN_EMISSION_H_
#define AM_TRANSMITTER_MAIN_EMISSION_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Class of emission (spec 5.3)

#include <cstdint>

namespace transmitter_system {

enum class Emission : uint8_t {
  kA2a = 0,  // keyed tone on a steady carrier (ARDF)
  kA1a = 1,  // keyed carrier (CW mode, D:DOT)
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_EMISSION_H_
