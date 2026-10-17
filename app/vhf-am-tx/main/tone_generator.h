#ifndef AM_TRANSMITTER_MAIN_TONE_GENERATOR_H_
#define AM_TRANSMITTER_MAIN_TONE_GENERATOR_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// A2A tone generation (spec 5)

#include <esp_attr.h>

#include <cstdint>

#include "hardware_config.h"

namespace transmitter_system {

/// 700Hz sine with a raised-cosine keying envelope.
///
/// Runs entirely inside the 25kHz ISR. A phase accumulator drives a 256 entry
/// table; no interpolation, because 256 points already put THD below -48dB
/// (spec 5.1).
///
/// The envelope is what keeps the signal clean: keying a square edge splatters
/// across tens of kHz, so every element ramps over 5ms (spec 5.2 / 10.3).
class ToneGenerator {
 public:
  /// Keying style (spec 5.3).
  enum class KeyingMode : uint8_t {
    /// A2A proper: the tone amplitude is keyed and the carrier stays up, so a
    /// backwave cannot exist. This is the default.
    kToneKeying = 0,
    /// Conventional CW feel: x is pulled to -1 (duty 0) on key up.
    kCarrierKeying = 1,
  };

  ToneGenerator();

  /// Builds the sine and envelope tables. Call once before the ISR starts.
  void Initialize();

  /// Key down / key up. The envelope handles the transition; calling this
  /// repeatedly with the same value is harmless.
  void IRAM_ATTR SetKey(bool key_down) { key_down_ = key_down; }

  bool IsKeyDown() const { return key_down_; }

  /// True once the envelope has fully decayed after key up. The CW keyer uses
  /// this to know when an element has really finished sounding.
  bool IRAM_ATTR IsSilent() const { return !key_down_ && envelope_index_ == 0; }

  void SetKeyingMode(KeyingMode mode) { keying_mode_ = mode; }
  KeyingMode GetKeyingMode() const { return keying_mode_; }

  /// Output amplitude of the tone, in units of x. Defaults to kXGainA2a
  /// (0.912 = 80% modulation / 0.877 M2 loss at 700Hz).
  void SetAmplitude(float amplitude) { amplitude_ = amplitude; }

  /// Resets phase and envelope. Call when transmission stops so the next key
  /// down starts from a zero crossing.
  void Reset();

  /// Produces one sample of x. Called exactly once per 25kHz tick.
  float IRAM_ATTR Next();

 private:
  // Tables are built at startup rather than being constexpr so they land in
  // .data (internal RAM) and stay reachable with the flash cache disabled.
  int16_t sine_table_[hardware_config::kSineTableSize];
  float envelope_table_[hardware_config::kEnvelopeSamples + 1];

  uint32_t phase_;
  uint32_t envelope_index_;
  bool key_down_;
  float amplitude_;
  KeyingMode keying_mode_;
  bool initialized_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_TONE_GENERATOR_H_
