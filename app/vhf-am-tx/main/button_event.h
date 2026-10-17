#ifndef AM_TRANSMITTER_MAIN_BUTTON_EVENT_H_
#define AM_TRANSMITTER_MAIN_BUTTON_EVENT_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp

#include <cstdint>

namespace transmitter_system {

/// Buttons are named by role, not by the silkscreen number (spec 1.1), so
/// renumbering the switches on the schematic does not reach into the code.
/// What FN+/FN- actually do depends on the mode (spec 9.2).
struct ButtonEvent {
  enum class Button : uint8_t {
    kMode = 0,     // SW0 - cycles through modes
    kFnPlus = 1,   // SW1 - also the voice PTT and the ARDF run toggle
    kFnMinus = 2,  // SW2
  };

  enum class Type : uint8_t {
    kPress = 0,
    kRelease = 1,
    /// Auto-repeat tick while held. Only FN+/FN- generate these; SW_MODE, the
    /// PTT and the ARDF run toggle are always single shot (spec 9.7).
    kRepeat = 2,
  };

  Button button;
  Type type;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_BUTTON_EVENT_H_
