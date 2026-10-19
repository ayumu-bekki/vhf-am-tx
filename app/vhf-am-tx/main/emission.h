#ifndef AM_TRANSMITTER_MAIN_EMISSION_H_
#define AM_TRANSMITTER_MAIN_EMISSION_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Class of emission (spec 5.3 / 9.2)

#include <cstdint>

namespace transmitter_system {

enum class Emission : uint8_t {
  kA2a = 0,  // keyed tone on a steady carrier
  kA1a = 1,  // keyed carrier
  kCount,
};

inline Emission SanitizeEmission(uint8_t raw) {
  return raw < static_cast<uint8_t>(Emission::kCount)
             ? static_cast<Emission>(raw)
             : Emission::kA2a;
}

/// SW2 in the ARDF / CW modes: A2A <-> A1A. Ignored while a run is going
/// (spec 9.4.2). Returns true when the emission changed.
inline bool TryToggleEmission(Emission* emission, bool running) {
  if (running) {
    return false;
  }
  *emission = (*emission == Emission::kA2a) ? Emission::kA1a : Emission::kA2a;
  return true;
}

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_EMISSION_H_
