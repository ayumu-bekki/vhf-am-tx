// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// PWM output and the 25kHz master tick (spec 6, 7)

#include "modulator.h"

#include <esp_cpu.h>
#include <esp_private/esp_clk.h>
#include <hal/ledc_ll.h>
#include <soc/ledc_struct.h>

#include <cmath>

#include "logger.h"
#include "modulation_map.h"

namespace transmitter_system {

namespace {

constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;  // C6 has no high speed
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kLedcChannel = LEDC_CHANNEL_0;

constexpr float kPi = 3.14159265358979323846f;

// CPU frequency in MHz, for converting cycle counts to microseconds.
// Read once at init rather than per tick.
uint32_t g_cpu_mhz = 160;

// Statistics are averaged over this many ticks (about 40ms at 25kHz).
constexpr uint32_t kStatWindow = 1024;

/// Writes the duty register through the low level layer.
///
/// ledc_set_duty() + ledc_update_duty() do argument validation and take a
/// mutex on every call, which is far more than a 40us budget can absorb. The
/// LL functions are what those eventually call; they are static inline, so
/// this costs no more than the raw register writes while staying correct if
/// the register layout ever moves. This ISR is the only writer of the channel
/// once the tick is running.
inline void IRAM_ATTR WriteDutyFast(uint32_t duty) {
  ledc_ll_set_duty_int_part(&LEDC, kLedcMode, kLedcChannel, duty);
  ledc_ll_set_duty_start(&LEDC, kLedcMode, kLedcChannel);
  ledc_ll_ls_channel_update(&LEDC, kLedcMode, kLedcChannel);
}

}  // namespace

Modulator::Modulator()
    : timer_(),
      ledc_ready_(false),
      cw_callback_(nullptr),
      cw_context_(nullptr),
      voice_callback_(nullptr),
      voice_context_(nullptr),
      tone_callback_(nullptr),
      tone_context_(nullptr),
      source_(ModulationSource::kSilent),
      fixed_duty_(0),
      carrier_level_(1.0f),
      output_enabled_(false),
      ramp_active_(false),
      ramp_request_(0),
      ramp_direction_(0),
      ramp_index_(0),
      ramp_scale_(0.0f),
      ramp_table_{},
      dither_(0.0f),
      tick_count_(0),
      stat_max_cycles_(0),
      stat_avg_cycles_(0),
      stat_accum_cycles_(0),
      stat_accum_count_(0) {}

Modulator::~Modulator() { Stop(); }

void Modulator::BuildRampTable() {
  // env(t) = 0.5 * (1 - cos(pi * t)), the same raised cosine the keying
  // envelope uses (spec 8.2 / 8.3).
  for (uint32_t i = 0; i <= hardware_config::kTxRampSamples; ++i) {
    const float t = static_cast<float>(i) /
                    static_cast<float>(hardware_config::kTxRampSamples);
    ramp_table_[i] = 0.5f * (1.0f - std::cos(kPi * t));
  }
}

bool Modulator::Initialize() {
  g_cpu_mhz = esp_clk_cpu_freq() / 1000000;

  BuildRampTable();

  // Timer first, with duty resolution 8 bits at 250kHz.
  // 80MHz / 250kHz = 320 steps, so 8 bits fits with a fractional divider of
  // 1.25. 9 bits would need 80MHz / (250kHz * 512) = 0.625 < 1 and cannot work
  // (spec 6.1).
  ledc_timer_config_t timer_config = {};
  timer_config.speed_mode = kLedcMode;
  timer_config.duty_resolution =
      static_cast<ledc_timer_bit_t>(hardware_config::kPwmBits);
  timer_config.timer_num = kLedcTimer;
  timer_config.freq_hz = hardware_config::kPwmHz;
  timer_config.clk_cfg = LEDC_USE_PLL_DIV_CLK;  // PLL_F80M on ESP32-C6

  esp_err_t ret = ledc_timer_config(&timer_config);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "ledc_timer_config failed (%u Hz / %u bit): %s",
             hardware_config::kPwmHz, hardware_config::kPwmBits,
             esp_err_to_name(ret));
    return false;
  }

  // Channel with duty 0. The GPIO is attached as part of this call, and the
  // duty is already 0, so the pin never carries an undefined level
  // (spec 8.4).
  ledc_channel_config_t channel_config = {};
  channel_config.gpio_num = hardware_config::kPwmPin;
  channel_config.speed_mode = kLedcMode;
  channel_config.channel = kLedcChannel;
  channel_config.timer_sel = kLedcTimer;
  channel_config.duty = 0;
  channel_config.hpoint = 0;
  channel_config.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
  // intr_type is deprecated in v6.0; the driver handles interrupts itself.

  ret = ledc_channel_config(&channel_config);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "ledc_channel_config failed: %s", esp_err_to_name(ret));
    return false;
  }

  ledc_set_duty(kLedcMode, kLedcChannel, 0);
  ledc_update_duty(kLedcMode, kLedcChannel);

  ledc_ready_ = true;
  ESP_LOGI(kTag, "LEDC ready: %u Hz, %u bit, GPIO%d, CPU %u MHz",
           hardware_config::kPwmHz, hardware_config::kPwmBits,
           hardware_config::kPwmPin, g_cpu_mhz);
  // One line that tells a build for the old modulator (R8 20k: gain 3.00,
  // carrier 93) from this one at a glance.
  ESP_LOGI(kTag, "MOD_GAIN=%.2f K=%.5f carrier=%u hard_max=%u",
           static_cast<double>(hardware_config::kModGain),
           static_cast<double>(hardware_config::kModKCal),
           static_cast<unsigned>(hardware_config::kDutyCarrier),
           static_cast<unsigned>(hardware_config::kDutyLimit));
  return true;
}

bool Modulator::Start() {
  if (!ledc_ready_) {
    ESP_LOGE(kTag, "Modulator::Start before Initialize");
    return false;
  }
  if (timer_.IsValid()) {
    return true;
  }

  // Priority 3 and an unshared interrupt: the only jitter source left on the
  // duty update is ISR latency variation, and spec 7.2 puts the target at
  // 0.3us. Sharing the line would let another handler run first.
  esp_err_t ret =
      timer_.Create(hardware_config::kTickTimerResolutionHz,
                    &Modulator::TimerCallback, this, 3, /*shared=*/false);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "Failed to create master tick timer: %s",
             esp_err_to_name(ret));
    return false;
  }

  ret = timer_.Start(hardware_config::kTickPeriodUs);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "Failed to start master tick: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(kTag, "Master tick running at %u Hz (%u us)",
           hardware_config::kTickHz, hardware_config::kTickPeriodUs);
  return true;
}

void Modulator::Stop() {
  if (timer_.IsValid()) {
    timer_.Stop();
    timer_.Destroy();
  }
  if (ledc_ready_) {
    ledc_set_duty(kLedcMode, kLedcChannel, 0);
    ledc_update_duty(kLedcMode, kLedcChannel);
  }
}

void Modulator::SetFixedDuty(uint32_t duty) {
  fixed_duty_.store(modulation_map::ClampDuty(duty),
                    std::memory_order_relaxed);
}

void Modulator::SetCarrierLevel(float level) {
  if (!(level > 0.0f)) level = 0.0f;  // also catches NaN
  if (level > 2.0f) level = 2.0f;
  carrier_level_.store(level, std::memory_order_relaxed);
}

void Modulator::StartRampUp() {
  ramp_request_.store(1, std::memory_order_release);
  ramp_active_.store(true, std::memory_order_release);
}

void Modulator::StartRampDown() {
  ramp_request_.store(-1, std::memory_order_release);
  ramp_active_.store(true, std::memory_order_release);
}

void Modulator::SnapRampFull() {
  ramp_request_.store(2, std::memory_order_release);
}

bool Modulator::TimerCallback(gptimer_handle_t timer,
                              const gptimer_alarm_event_data_t* data,
                              void* user_data) {
  static_cast<Modulator*>(user_data)->Tick();
  return false;  // no task woken
}

void Modulator::Tick() {
  const uint32_t cycles_start = esp_cpu_get_cycle_count();

  tick_count_.fetch_add(1, std::memory_order_relaxed);

  // --- Ramp handling (spec 8.2 / 8.3) -------------------------------------
  const int8_t request = ramp_request_.exchange(0, std::memory_order_acq_rel);
  if (request == 2) {
    ramp_direction_ = 0;
    ramp_scale_ = 1.0f;
    ramp_active_.store(false, std::memory_order_release);
  } else if (request != 0) {
    ramp_direction_ = request;
    ramp_index_ = 0;
  }

  if (ramp_direction_ != 0) {
    ++ramp_index_;
    if (ramp_index_ >= hardware_config::kTxRampSamples) {
      ramp_scale_ = (ramp_direction_ > 0) ? 1.0f : 0.0f;
      ramp_direction_ = 0;
      ramp_active_.store(false, std::memory_order_release);
    } else {
      // Table lookup, not cosf(): libm lives in flash, and reaching into it
      // from the ISR is exactly the cache miss IRAM placement exists to
      // prevent.
      const float shaped = ramp_table_[ramp_index_];
      ramp_scale_ = (ramp_direction_ > 0) ? shaped : (1.0f - shaped);
    }
  }

  // --- Produce the duty ----------------------------------------------------
  uint32_t duty = 0;
  const ModulationSource source = source_.load(std::memory_order_acquire);

  if (source == ModulationSource::kFixedDuty) {
    // Calibration mode: write the register value straight through, still
    // subject to the hard ceiling. The carrier is off in this mode, so this
    // only exercises the modulator board.
    duty = fixed_duty_.load(std::memory_order_relaxed);
  } else if (!output_enabled_.load(std::memory_order_acquire) &&
             ramp_scale_ <= 0.0f) {
    // Deliberate zero: key up or transmit stopped. Kept separate from the
    // limiter so the limiter's lower bound cannot get in its way (spec 6.3).
    duty = 0;
    dither_ = 0.0f;
  } else {
    float x = 0.0f;

    switch (source) {
      case ModulationSource::kCwKeyer:
        if (cw_callback_) {
          x = cw_callback_(cw_context_);
        }
        break;
      case ModulationSource::kContinuousTone:
        if (tone_callback_) {
          x = tone_callback_(tone_context_);
        }
        break;
      case ModulationSource::kVoice:
        if (voice_callback_) {
          x = voice_callback_(voice_context_);
        }
        break;
      case ModulationSource::kSilent:
        // Unmodulated carrier. (1 + x) is the envelope, so a level of 0.5
        // gives half the carrier duty.
        x = carrier_level_.load(std::memory_order_relaxed) - 1.0f;
        break;
      case ModulationSource::kFixedDuty:
        break;  // handled above
    }

    // Absolute clamp, independent of the per-mode soft limits.
    if (x > hardware_config::kXAbsMax) {
      x = hardware_config::kXAbsMax;
    } else if (x < hardware_config::kXAbsMin) {
      x = hardware_config::kXAbsMin;
    }

    // Apply the start/stop ramp. Scaling (1 + x) keeps the shape of the
    // modulation while the envelope moves between 0 and the carrier level.
    if (ramp_scale_ < 1.0f) {
      x = ramp_scale_ * (1.0f + x) - 1.0f;
    }
    duty = modulation_map::XToDuty(
        x, &dither_, power_scale_.load(std::memory_order_relaxed));
  }

  // Whatever got here, the ceiling applies. XToDuty() and SetFixedDuty()
  // already clamp; this is the last check before the register.
  WriteDutyFast(modulation_map::ClampDuty(duty));

  // --- Statistics ----------------------------------------------------------
  const uint32_t elapsed = esp_cpu_get_cycle_count() - cycles_start;

  uint32_t previous_max = stat_max_cycles_.load(std::memory_order_relaxed);
  if (elapsed > previous_max) {
    stat_max_cycles_.store(elapsed, std::memory_order_relaxed);
  }

  stat_accum_cycles_ += elapsed;
  if (++stat_accum_count_ >= kStatWindow) {
    stat_avg_cycles_.store(stat_accum_cycles_ / stat_accum_count_,
                           std::memory_order_relaxed);
    stat_accum_cycles_ = 0;
    stat_accum_count_ = 0;
  }
}

IsrStats Modulator::GetStats() const {
  IsrStats stats = {};
  const uint32_t mhz = g_cpu_mhz > 0 ? g_cpu_mhz : 1;
  stats.max_us = stat_max_cycles_.load(std::memory_order_relaxed) / mhz;
  stats.avg_us = stat_avg_cycles_.load(std::memory_order_relaxed) / mhz;
  return stats;
}

}  // namespace transmitter_system

// EOF
