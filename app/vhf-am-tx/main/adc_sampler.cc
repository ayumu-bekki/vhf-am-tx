// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// ADC input: microphone and battery (spec 3)

#include "adc_sampler.h"

#include <esp_adc/adc_cali_scheme.h>

#include <cstring>

#include "logger.h"

namespace transmitter_system {

namespace {

constexpr adc_unit_t kAdcUnit = ADC_UNIT_1;
constexpr adc_channel_t kMicChannel = ADC_CHANNEL_1;      // GPIO1 (D1)
constexpr adc_channel_t kBatteryChannel = ADC_CHANNEL_0;  // GPIO0 (D0)

// Attenuation. The microphone sits on a 1.65V bias with roughly +-0.5V of
// signal, i.e. 1.15-2.15V; 6dB (about 1.75V full scale) would clip the top,
// so 12dB it is (spec 3.1).
constexpr adc_atten_t kAdcAtten = ADC_ATTEN_DB_12;
constexpr adc_bitwidth_t kAdcBitwidth = ADC_BITWIDTH_12;

// One conversion frame. Larger frames mean fewer interrupts but more latency
// between the ADC and the tick; 64 conversions is about 1.3ms at 50kS/s.
constexpr uint32_t kConvFrameSize = 64 * SOC_ADC_DIGI_RESULT_BYTES;
constexpr uint32_t kMaxStoreBufSize = kConvFrameSize * 4;

// Full scale in millivolts for the fallback conversion when calibration is
// unavailable. 12dB attenuation is roughly 0-3.1V on this part; the exact
// figure is what adc_cali exists to supply, so this is only a rough backstop.
constexpr int kFallbackFullScaleMv = 3100;
constexpr int kAdcMaxRaw = 4095;

// Normalisation for the microphone: the usable swing either side of the bias
// is about half the range, so scale by that to land x in roughly -1..+1.
constexpr float kMicNormalise = 1.0f / 2048.0f;

}  // namespace

AdcSampler::AdcSampler()
    : handle_(nullptr),
      cali_handle_(nullptr),
      calibrated_(false),
      mic_ring_{},
      mic_write_index_(0),
      mic_read_index_(0),
      mic_bias_(2048.0f),
      bias_init_count_(0),
      bias_init_sum_(0),
      bias_initialised_(false),
      clip_run_(0),
      clipping_(false),
      battery_count_(0),
      battery_sum_(0),
      battery_raw_mean_(0),
      battery_seq_(0),
      battery_seq_seen_(0),
      battery_voltage_(0.0f),
      battery_average_(0.0f),
      battery_history_{},
      battery_history_index_(0),
      battery_history_filled_(false),
      ring_overrun_(0),
      ring_underrun_(0) {}

AdcSampler::~AdcSampler() {
  Stop();
  if (handle_) {
    adc_continuous_deinit(handle_);
    handle_ = nullptr;
  }
  if (cali_handle_) {
    adc_cali_delete_scheme_curve_fitting(cali_handle_);
    cali_handle_ = nullptr;
  }
}

bool AdcSampler::Initialize() {
  adc_continuous_handle_cfg_t handle_config = {};
  handle_config.max_store_buf_size = kMaxStoreBufSize;
  handle_config.conv_frame_size = kConvFrameSize;
  handle_config.flags.flush_pool = 1;

  esp_err_t ret = adc_continuous_new_handle(&handle_config, &handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "adc_continuous_new_handle failed: %s",
             esp_err_to_name(ret));
    return false;
  }

  // Two channels scanned in turn (spec 3.5). The callback must dispatch on
  // the channel field rather than assuming an order: if a frame boundary
  // slips, microphone and battery would silently swap.
  adc_digi_pattern_config_t pattern[2] = {};
  pattern[0].atten = kAdcAtten;
  pattern[0].channel = kMicChannel;
  pattern[0].unit = kAdcUnit;
  pattern[0].bit_width = kAdcBitwidth;
  pattern[1].atten = kAdcAtten;
  pattern[1].channel = kBatteryChannel;
  pattern[1].unit = kAdcUnit;
  pattern[1].bit_width = kAdcBitwidth;

  adc_continuous_config_t config = {};
  config.pattern_num = 2;
  config.adc_pattern = pattern;
  config.sample_freq_hz = hardware_config::kAdcSampleFreqHz;
  config.conv_mode = ADC_CONV_SINGLE_UNIT_1;

  ret = adc_continuous_config(handle_, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "adc_continuous_config failed (%u Hz): %s",
             hardware_config::kAdcSampleFreqHz, esp_err_to_name(ret));
    return false;
  }

  adc_continuous_evt_cbs_t callbacks = {};
  callbacks.on_conv_done = &AdcSampler::ConversionDoneCallback;
  ret = adc_continuous_register_event_callbacks(handle_, &callbacks, this);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "adc callback registration failed: %s",
             esp_err_to_name(ret));
    return false;
  }

  // Calibration. The raw code is not linear, so converting to volts without
  // this would put the battery reading out by a long way (spec 3.1).
  adc_cali_curve_fitting_config_t cali_config = {};
  cali_config.unit_id = kAdcUnit;
  cali_config.atten = kAdcAtten;
  cali_config.bitwidth = kAdcBitwidth;

  if (adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle_) ==
      ESP_OK) {
    calibrated_ = true;
  } else {
    calibrated_ = false;
    ESP_LOGW(kTag, "ADC calibration unavailable; battery reading is approximate");
  }

  ESP_LOGI(kTag, "ADC ready: %u S/s total (%u per channel), cal=%d",
           hardware_config::kAdcSampleFreqHz,
           hardware_config::kAdcPerChannelHz, calibrated_ ? 1 : 0);
  return true;
}

bool AdcSampler::Start() {
  if (!handle_) {
    return false;
  }
  esp_err_t ret = adc_continuous_start(handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "adc_continuous_start failed: %s", esp_err_to_name(ret));
    return false;
  }
  return true;
}

void AdcSampler::Stop() {
  if (handle_) {
    adc_continuous_stop(handle_);
  }
}

bool AdcSampler::ConversionDoneCallback(
    adc_continuous_handle_t handle, const adc_continuous_evt_data_t* edata,
    void* user_data) {
  static_cast<AdcSampler*>(user_data)->HandleConversion(edata);
  return false;
}

void AdcSampler::HandleConversion(const adc_continuous_evt_data_t* edata) {
  if (!edata || !edata->conv_frame_buffer) {
    return;
  }

  const uint32_t count = edata->size / SOC_ADC_DIGI_RESULT_BYTES;
  const adc_digi_output_data_t* data =
      reinterpret_cast<const adc_digi_output_data_t*>(edata->conv_frame_buffer);

  for (uint32_t i = 0; i < count; ++i) {
    const uint32_t channel = data[i].type2.channel;
    const uint32_t raw = data[i].type2.data;

    // Dispatch on the channel, never on position in the frame.
    if (channel == kMicChannel) {
      PushMicrophoneSample(raw);
    } else if (channel == kBatteryChannel) {
      AccumulateBatterySample(raw);
    }
  }
}

void AdcSampler::PushMicrophoneSample(uint32_t raw) {
  // Clip detection on the raw code (spec 3.3). Note this only catches the ADC
  // rails; the microphone module saturates at about +-0.5V, which is still
  // inside the ADC range, so the limiter activity figure has to be watched
  // alongside this.
  if (raw < hardware_config::kAdcClipLow ||
      raw > hardware_config::kAdcClipHigh) {
    if (++clip_run_ >= hardware_config::kAdcClipRunLength) {
      clipping_.store(true, std::memory_order_relaxed);
    }
  } else {
    clip_run_ = 0;
    clipping_.store(false, std::memory_order_relaxed);
  }

  // Bias: average the first 200ms, then track very slowly (spec 3.4). The
  // nominal 1.65V is never assumed - divider tolerance and ADC offset move it
  // by tens of millivolts, and it drifts with temperature.
  float bias = mic_bias_.load(std::memory_order_relaxed);

  if (!bias_initialised_) {
    bias_init_sum_ += raw;
    if (++bias_init_count_ >= hardware_config::kAdcBiasInitSamples) {
      bias = static_cast<float>(bias_init_sum_) /
             static_cast<float>(bias_init_count_);
      mic_bias_.store(bias, std::memory_order_relaxed);
      bias_initialised_ = true;
    }
    return;  // do not emit audio until the bias is known
  }

  // fc = 1Hz one pole. Three decades below the audio band, so it tracks drift
  // without touching the modulation.
  bias += hardware_config::kAdcBiasAlpha * (static_cast<float>(raw) - bias);
  mic_bias_.store(bias, std::memory_order_relaxed);

  const float centred = static_cast<float>(raw) - bias;

  const uint32_t write = mic_write_index_.load(std::memory_order_relaxed);
  const uint32_t read = mic_read_index_.load(std::memory_order_acquire);
  const uint32_t next = (write + 1) % hardware_config::kMicRingSize;

  if (next == read) {
    // The tick is not draining fast enough. Drop the sample rather than
    // overwrite one it has not read; either way it is recorded.
    ring_overrun_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  mic_ring_[write] = static_cast<int16_t>(centred);
  mic_write_index_.store(next, std::memory_order_release);
}

void AdcSampler::AccumulateBatterySample(uint32_t raw) {
  battery_sum_ += raw;
  if (++battery_count_ < hardware_config::kBatAvgSamples) {
    return;
  }

  battery_raw_mean_.store(static_cast<uint32_t>(battery_sum_ / battery_count_),
                          std::memory_order_relaxed);
  battery_seq_.fetch_add(1, std::memory_order_release);
  battery_sum_ = 0;
  battery_count_ = 0;
}

void AdcSampler::UpdateBattery() {
  const uint32_t seq = battery_seq_.load(std::memory_order_acquire);
  if (seq == battery_seq_seen_) {
    return;
  }
  battery_seq_seen_ = seq;
  const uint32_t mean = battery_raw_mean_.load(std::memory_order_relaxed);

  int millivolts = 0;
  if (!calibrated_ ||
      adc_cali_raw_to_voltage(cali_handle_, static_cast<int>(mean),
                              &millivolts) != ESP_OK) {
    millivolts = static_cast<int>(mean) * kFallbackFullScaleMv / kAdcMaxRaw;
  }

  const float volts =
      (static_cast<float>(millivolts) / 1000.0f) * hardware_config::kBatDivRatio;

  battery_voltage_.store(volts, std::memory_order_relaxed);

  // One second moving average over the 10Hz samples. This, not the
  // instantaneous value, is what gates transmission.
  battery_history_[battery_history_index_] = volts;
  battery_history_index_ =
      (battery_history_index_ + 1) % hardware_config::kBatInhibitAvgCount;
  if (battery_history_index_ == 0) {
    battery_history_filled_ = true;
  }

  const uint32_t samples = battery_history_filled_
                               ? hardware_config::kBatInhibitAvgCount
                               : battery_history_index_;
  float sum = 0.0f;
  for (uint32_t i = 0; i < samples; ++i) {
    sum += battery_history_[i];
  }
  battery_average_.store(sum / static_cast<float>(samples),
                         std::memory_order_relaxed);
}

float AdcSampler::ReadMicrophone() {
  const uint32_t read = mic_read_index_.load(std::memory_order_relaxed);
  const uint32_t write = mic_write_index_.load(std::memory_order_acquire);

  if (read == write) {
    // Nothing queued. Returning 0 puts x at the carrier level, which is the
    // benign failure: silence with no discontinuity.
    ring_underrun_.fetch_add(1, std::memory_order_relaxed);
    return 0.0f;
  }

  const int16_t sample = mic_ring_[read];
  mic_read_index_.store((read + 1) % hardware_config::kMicRingSize,
                        std::memory_order_release);

  return static_cast<float>(sample) * kMicNormalise;
}

}  // namespace transmitter_system

// EOF
