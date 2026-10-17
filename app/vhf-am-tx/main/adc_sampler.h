#ifndef AM_TRANSMITTER_MAIN_ADC_SAMPLER_H_
#define AM_TRANSMITTER_MAIN_ADC_SAMPLER_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// ADC input: microphone and battery (spec 3)

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_continuous.h>
#include <esp_attr.h>

#include <atomic>
#include <cstdint>

#include "hardware_config.h"

namespace transmitter_system {

/// Free-running two channel ADC.
///
/// Both the microphone (CH1) and the battery (CH0) share ADC1 in continuous
/// DMA mode at 50kS/s total, giving 25kS/s each so the microphone lines up
/// with the master tick. `adc_oneshot_read()` is unusable here: it blocks and
/// is not ISR safe (spec 3.1).
///
/// The ADC is deliberately *not* the clock. It free-runs and hands samples to
/// the 25kHz tick through a small ring buffer, so a jittery DMA completion
/// cannot drag the modulation timing with it (spec 7.1).
class AdcSampler {
 public:
  AdcSampler();
  ~AdcSampler();

  bool Initialize();
  bool Start();
  void Stop();

  /// Latest microphone sample, bias removed and normalised to about -1..+1.
  /// Returns 0 when the ring is empty, which is the safe value: it maps to
  /// the carrier level rather than to silence-with-a-click.
  float IRAM_ATTR ReadMicrophone();

  /// Converts the latest battery average to volts. Call from a task (the
  /// main loop) at least every 100ms.
  ///
  /// The conversion lives here rather than in the DMA callback because
  /// adc_cali_raw_to_voltage() is in flash: the callback is registered
  /// IRAM-safe and keeps running while NVS has the flash cache disabled, and
  /// calling into flash from there dies with a cache access error.
  void UpdateBattery();

  /// Battery voltage in volts, updated at 10Hz.
  float GetBatteryVoltage() const {
    return battery_voltage_.load(std::memory_order_relaxed);
  }

  /// One second moving average of the battery voltage.
  ///
  /// The inhibit decision uses this, never the instantaneous reading: peaks
  /// draw 1.05A and the pack sags 0.3-0.5V, so judging on the raw value would
  /// cut transmission on every key down (spec 3.6).
  float GetBatteryAverageVoltage() const {
    return battery_average_.load(std::memory_order_relaxed);
  }

  /// Measured DC bias of the microphone input, in raw ADC counts.
  /// The nominal 1.65V is never assumed; it is measured at startup and then
  /// tracked slowly (spec 3.4).
  float GetMicrophoneBias() const {
    return mic_bias_.load(std::memory_order_relaxed);
  }

  /// True while the input is clipping (spec 3.3).
  bool IsClipping() const { return clipping_.load(std::memory_order_relaxed); }

  uint32_t GetRingOverruns() const {
    return ring_overrun_.load(std::memory_order_relaxed);
  }
  uint32_t GetRingUnderruns() const {
    return ring_underrun_.load(std::memory_order_relaxed);
  }

  bool IsReady() const { return handle_ != nullptr; }

 private:
  static bool IRAM_ATTR ConversionDoneCallback(
      adc_continuous_handle_t handle, const adc_continuous_evt_data_t* edata,
      void* user_data);
  void IRAM_ATTR HandleConversion(const adc_continuous_evt_data_t* edata);
  void IRAM_ATTR PushMicrophoneSample(uint32_t raw);
  void IRAM_ATTR AccumulateBatterySample(uint32_t raw);

 private:
  adc_continuous_handle_t handle_;
  adc_cali_handle_t cali_handle_;
  bool calibrated_;

  // Microphone ring buffer, written by the DMA callback and read by the tick.
  volatile int16_t mic_ring_[hardware_config::kMicRingSize];
  std::atomic<uint32_t> mic_write_index_;
  std::atomic<uint32_t> mic_read_index_;

  // Bias tracking. Startup average first, then a slow follower.
  std::atomic<float> mic_bias_;
  uint32_t bias_init_count_;
  uint64_t bias_init_sum_;
  bool bias_initialised_;

  // Clip detection
  uint32_t clip_run_;
  std::atomic<bool> clipping_;

  // Battery accumulation in the callback, 2500 samples per 10Hz update. The
  // mean raw code is handed to UpdateBattery() through battery_raw_mean_;
  // battery_seq_ counts the hand-overs.
  uint32_t battery_count_;
  uint64_t battery_sum_;
  std::atomic<uint32_t> battery_raw_mean_;
  std::atomic<uint32_t> battery_seq_;

  // Task side (UpdateBattery() only).
  uint32_t battery_seq_seen_;
  std::atomic<float> battery_voltage_;
  std::atomic<float> battery_average_;
  float battery_history_[hardware_config::kBatInhibitAvgCount];
  uint32_t battery_history_index_;
  bool battery_history_filled_;

  std::atomic<uint32_t> ring_overrun_;
  std::atomic<uint32_t> ring_underrun_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_ADC_SAMPLER_H_
