// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// A2A tone generation (spec 5)

#include "tone_generator.h"

#include <cmath>

namespace transmitter_system {

namespace {
constexpr float kPi = 3.14159265358979323846f;
// Sine table is stored as int16_t; this is the full-scale value.
constexpr float kSineScale = 32767.0f;
constexpr float kSineInvScale = 1.0f / 32767.0f;
}  // namespace

ToneGenerator::ToneGenerator()
    : sine_table_{},
      envelope_table_{},
      phase_(0),
      envelope_index_(0),
      key_down_(false),
      amplitude_(hardware_config::kXGainA2a),
      keying_mode_(KeyingMode::kToneKeying),
      initialized_(false) {}

void ToneGenerator::Initialize() {
  if (initialized_) {
    return;
  }

  for (uint32_t i = 0; i < hardware_config::kSineTableSize; ++i) {
    const float angle =
        2.0f * kPi * static_cast<float>(i) /
        static_cast<float>(hardware_config::kSineTableSize);
    sine_table_[i] = static_cast<int16_t>(std::lround(std::sin(angle) * kSineScale));
  }

  // env(t) = 0.5 * (1 - cos(pi * t / 5ms)), spec 5.2.
  // Index 0 is fully off and index kEnvelopeSamples is fully on, so the table
  // holds one extra entry.
  for (uint32_t i = 0; i <= hardware_config::kEnvelopeSamples; ++i) {
    const float t =
        static_cast<float>(i) /
        static_cast<float>(hardware_config::kEnvelopeSamples);
    envelope_table_[i] = 0.5f * (1.0f - std::cos(kPi * t));
  }

  initialized_ = true;
}

void ToneGenerator::Reset() {
  phase_ = 0;
  envelope_index_ = 0;
  key_down_ = false;
}

float ToneGenerator::Next() {
  // Envelope follows the key with a 5ms raised cosine in both directions.
  if (key_down_) {
    if (envelope_index_ < hardware_config::kEnvelopeSamples) {
      ++envelope_index_;
    }
  } else {
    if (envelope_index_ > 0) {
      --envelope_index_;
    }
  }

  const float envelope = envelope_table_[envelope_index_];

  if (keying_mode_ == KeyingMode::kCarrierKeying) {
    // Pull x down to -1 (duty 0) on key up, ramping through the same
    // envelope so the edges stay soft.
    // x = -1 at envelope 0, x = tone at envelope 1.
    const int16_t sample =
        sine_table_[phase_ >> (32 - 8)];  // top 8 bits index the 256 table
    phase_ += hardware_config::kTonePhaseIncrement;
    const float tone = static_cast<float>(sample) * kSineInvScale * amplitude_;
    return -1.0f + envelope * (tone + 1.0f);
  }

  // Tone keying (default): the carrier stays put at x = 0 and only the tone
  // amplitude is keyed, so there is no backwave by construction.
  if (envelope_index_ == 0) {
    // Fully silent. Keep the phase parked at zero so the next key down starts
    // from a zero crossing instead of wherever the accumulator happened to be.
    phase_ = 0;
    return 0.0f;
  }

  const int16_t sample = sine_table_[phase_ >> (32 - 8)];
  phase_ += hardware_config::kTonePhaseIncrement;
  return static_cast<float>(sample) * kSineInvScale * amplitude_ * envelope;
}

}  // namespace transmitter_system

// EOF
