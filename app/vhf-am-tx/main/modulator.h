#ifndef AM_TRANSMITTER_MAIN_MODULATOR_H_
#define AM_TRANSMITTER_MAIN_MODULATOR_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// PWM output and the 25kHz master tick (spec 6, 7)

#include <driver/gptimer.h>
#include <driver/ledc.h>
#include <esp_attr.h>

#include <atomic>
#include <cstdint>

#include "gptimer.h"
#include "hardware_config.h"

namespace transmitter_system {

/// Who is allowed to write the modulation input x on each tick.
///
/// Switching modes is exactly this: a change of source. The PWM side does not
/// care where x came from (spec 2.1).
enum class ModulationSource : uint8_t {
  /// Nobody writes x; it is pinned at 0 (carrier level) or the duty is forced
  /// to 0 entirely, depending on whether the carrier is meant to be up.
  kSilent = 0,
  /// A2A: the CW keyer drives the tone generator.
  kCwKeyer,
  /// A3E: the microphone chain.
  kVoice,
  /// Test mode: a fixed duty register value, bypassing the x mapping.
  kFixedDuty,
  /// Test mode: continuous tone, no keying.
  kContinuousTone,
};

/// Statistics gathered inside the ISR, for the 20us budget check of spec 7.1.
///
/// Measured on hardware: 4us idle, 8us for A2A, 26us for A3E. The A3E figure
/// is over the 20us guideline but inside the 40us period, and was accepted
/// as-is (spec 7.4). Watch `max_us` here if voice quality ever looks wrong.
struct IsrStats {
  uint32_t max_us;  // worst case since boot
  uint32_t avg_us;  // average over the last ~40ms
};

/// Owns the LEDC channel and the 25kHz master tick.
///
/// Everything that happens per sample happens here: fetch x from whichever
/// source is active, map it to a duty register value, write it out. The tick
/// is the master clock of the whole system; the ADC free-runs and is coupled
/// through a ring buffer instead of driving anything (spec 7.1).
class Modulator {
 public:
  /// Callback that produces one sample of x. Runs in ISR context, so it and
  /// everything it touches must be in IRAM.
  using SampleCallback = float (*)(void* context);

  Modulator();
  ~Modulator();

  /// Sets up LEDC with duty 0 and only then attaches the GPIO.
  ///
  /// Order matters: configuring the pin first would put an undefined level on
  /// the modulator input for an instant (spec 8.4).
  bool Initialize();

  /// Starts the 25kHz tick. Nothing is emitted until the source is set to
  /// something other than kSilent and the carrier is enabled.
  bool Start();
  void Stop();

  /// Installs the per-sample callbacks. Call before Start().
  void SetCwKeyerCallback(SampleCallback cb, void* context) {
    cw_callback_ = cb;
    cw_context_ = context;
  }
  void SetVoiceCallback(SampleCallback cb, void* context) {
    voice_callback_ = cb;
    voice_context_ = context;
  }
  void SetToneCallback(SampleCallback cb, void* context) {
    tone_callback_ = cb;
    tone_context_ = context;
  }

  void SetSource(ModulationSource source) {
    source_.store(source, std::memory_order_release);
  }

  /// Fixed duty for the D:DUTY calibration mode. Clamped to kDutyLimit.
  void SetFixedDuty(uint32_t duty);

  /// Carrier level for the silent source (D:CARR): 1.0 is the full carrier
  /// duty, 0.5 is half of it, 1.8 the D:CARR peak. Goes through the same ramp
  /// and the same duty ceiling as everything else. Clamped to 0..2.
  void SetCarrierLevel(float level);

  /// Gates the output. When false the duty is forced to 0 regardless of x,
  /// which is how key-up and transmit-stop are expressed (spec 6.3).
  void SetOutputEnabled(bool enabled) {
    output_enabled_.store(enabled, std::memory_order_release);
  }
  bool IsOutputEnabled() const {
    return output_enabled_.load(std::memory_order_acquire);
  }

  /// Output stage factor s (spec 6.5). Written before a transmission starts
  /// and only read by the ISR while it runs.
  void SetPowerScale(float scale) {
    power_scale_.store(scale, std::memory_order_relaxed);
  }

  /// Jumps the start/stop ramp to its end value (full) without a ramp. A1A
  /// uses this: x stays at -1 while key up, so the drain is at 0V and there is
  /// nothing to ramp.
  void SnapRampFull();

  /// Ramp control for the start and stop sequences (spec 8.2 / 8.3).
  /// The ramp runs inside the ISR so its timing is exact.
  void StartRampUp();
  void StartRampDown();
  bool IsRampActive() const {
    return ramp_active_.load(std::memory_order_acquire);
  }

  /// Monotonic tick counter, used by the watchdog to notice a stalled ISR
  /// (spec 8.6).
  uint32_t GetTickCount() const {
    return tick_count_.load(std::memory_order_relaxed);
  }

  IsrStats GetStats() const;

 private:
  /// Builds the raised-cosine ramp table. Called from Initialize().
  void BuildRampTable();

  static bool IRAM_ATTR TimerCallback(gptimer_handle_t timer,
                                      const gptimer_alarm_event_data_t* data,
                                      void* user_data);
  void IRAM_ATTR Tick();

 private:
  GPTimer timer_;
  bool ledc_ready_;

  SampleCallback cw_callback_;
  void* cw_context_;
  SampleCallback voice_callback_;
  void* voice_context_;
  SampleCallback tone_callback_;
  void* tone_context_;

  std::atomic<ModulationSource> source_;
  std::atomic<uint32_t> fixed_duty_;
  std::atomic<float> carrier_level_;
  std::atomic<float> power_scale_{1.0f};
  std::atomic<bool> output_enabled_;

  // Ramp state (ISR only, except the request flags)
  std::atomic<bool> ramp_active_;
  std::atomic<int8_t> ramp_request_;  // +1 up, -1 down, 2 snap to full, 0 none
  int8_t ramp_direction_;
  uint32_t ramp_index_;
  float ramp_scale_;

  /// Raised-cosine ramp shape, precomputed.
  ///
  /// Evaluating cosf() in the ISR would reach into flash for the libm code on
  /// every sample of every transition, which is exactly the cache miss the
  /// IRAM requirement exists to avoid. The table lives in .data (internal
  /// RAM) and covers index 0..kTxRampSamples inclusive.
  float ramp_table_[hardware_config::kTxRampSamples + 1];

  // Error feedback dither state (spec 6.4), ISR only.
  float dither_;

  std::atomic<uint32_t> tick_count_;

  // ISR timing statistics
  std::atomic<uint32_t> stat_max_cycles_;
  std::atomic<uint32_t> stat_avg_cycles_;
  uint32_t stat_accum_cycles_;
  uint32_t stat_accum_count_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_MODULATOR_H_
