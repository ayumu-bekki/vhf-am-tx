#ifndef AM_TRANSMITTER_MAIN_VOICE_DSP_H_
#define AM_TRANSMITTER_MAIN_VOICE_DSP_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// A3E voice processing chain (spec 4)

#include <esp_attr.h>

#include <atomic>
#include <cstdint>

#include "biquad.h"
#include "hardware_config.h"

namespace transmitter_system {

/// Microphone path: DC removal -> HPF -> EQ -> compressor -> limiter -> LPF.
///
/// The order is fixed (spec 4). Two constraints pin it down:
///   - the limiter must sit after the EQ, because the EQ adds up to +8.6dB at
///     3kHz and something has to catch that;
///   - the LPF must sit after the EQ too. The EQ inverts M2, which keeps
///     rising with frequency - it reaches about +24dB at 10kHz relative to
///     1kHz - so without the low pass behind it the ADC noise floor and any
///     aliases get amplified straight into the occupied bandwidth.
class VoiceDsp {
 public:
  VoiceDsp();

  /// Installs the filter coefficients. Call once before the ISR starts.
  void Initialize();

  /// Resets all filter state. Call when transmission starts so a stale tail
  /// from the previous over does not leak into the new one.
  void Reset();

  /// Processes one sample. `sample` is the bias-corrected ADC reading
  /// normalised to roughly -1..+1 full scale.
  /// Returns the modulation input x, already limited to the A3E window.
  float IRAM_ATTR Process(float sample);

  /// Makeup gain applied before the limiter (spec 4.3). Set from the UI.
  void SetMakeupGain(float gain) {
    makeup_gain_.store(gain, std::memory_order_relaxed);
  }
  float GetMakeupGain() const {
    return makeup_gain_.load(std::memory_order_relaxed);
  }

  /// Fraction of the last second's samples that hit the limiter, 0..1.
  ///
  /// This is the only real handle the operator has on whether the speech
  /// level is right, so it goes on the display (spec 4.2). Aim for 1-5%.
  float GetLimiterActivity() const;

  /// Peak absolute value of x seen since the last call, for level display.
  float ConsumePeak();

 private:
  // Filters. EQ is a biquad plus a first order section (spec 4.1).
  Biquad hpf_;
  Biquad eq_biquad_;
  FirstOrder eq_first_;
  Biquad lpf_;

  // Compressor state (spec 4.3): feed-forward, peak detecting.
  float envelope_;
  float attack_coeff_;
  float release_coeff_;

  std::atomic<float> makeup_gain_;

  // Limiter activity over a sliding one second window.
  uint32_t limit_hits_;
  uint32_t limit_samples_;
  std::atomic<float> limiter_activity_;

  std::atomic<float> peak_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_VOICE_DSP_H_
