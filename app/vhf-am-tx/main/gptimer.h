#ifndef AM_TRANSMITTER_MAIN_GPTIMER_H_
#define AM_TRANSMITTER_MAIN_GPTIMER_H_
// ESP32 144MHz AM Transmitter
// (C)2024 bekki.jp

// Include ----------------------
#include <driver/gptimer.h>

namespace transmitter_system {

/// GPTimer wrapper.
///
/// Two users with different needs share this: the button poller is happy with
/// whatever the driver hands out, while the 25kHz master tick wants a high
/// priority, non-shared interrupt so its latency jitter stays inside the
/// budget of spec 7.2. Hence the priority argument.
class GPTimer {
 public:
  GPTimer() : gptimer_(nullptr) {}

  ~GPTimer() { Destroy(); }

  /// @param resolution  counter resolution in Hz
  /// @param function    alarm callback
  /// @param user_data   passed through to the callback
  /// @param intr_priority  0 lets the driver choose (1-3); pass 3 for the
  ///                       master tick. Higher is better for jitter.
  /// @param shared      allow the interrupt line to be shared. The master
  ///                    tick must not share, or another peripheral's handler
  ///                    lands in front of it.
  esp_err_t Create(const uint32_t resolution, gptimer_alarm_cb_t function,
                   void* const user_data, const int intr_priority = 0,
                   const bool shared = true) {
    gptimer_config_t timer_config = {};
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = resolution;
    timer_config.intr_priority = intr_priority;
    timer_config.flags.intr_shared = shared ? 1u : 0u;

    esp_err_t ret = gptimer_new_timer(&timer_config, &gptimer_);
    if (ret != ESP_OK) {
      gptimer_ = nullptr;
      return ret;
    }

    gptimer_event_callbacks_t gptimer_callback = {};
    gptimer_callback.on_alarm = function;
    ret = gptimer_register_event_callbacks(gptimer_, &gptimer_callback,
                                           user_data);
    if (ret != ESP_OK) {
      Destroy();
      return ret;
    }

    ret = gptimer_enable(gptimer_);
    if (ret != ESP_OK) {
      Destroy();
      return ret;
    }
    return ESP_OK;
  }

  void Destroy() {
    if (gptimer_) {
      gptimer_disable(gptimer_);
      gptimer_del_timer(gptimer_);
      gptimer_ = nullptr;
    }
  }

  esp_err_t Start(const uint64_t wait_count) const {
    if (!gptimer_) {
      return ESP_ERR_INVALID_STATE;
    }

    gptimer_alarm_config_t alarm_config = {};
    alarm_config.alarm_count = wait_count;
    alarm_config.reload_count = 0ull;
    alarm_config.flags.auto_reload_on_alarm = 1u;

    esp_err_t ret = gptimer_set_alarm_action(gptimer_, &alarm_config);
    if (ret != ESP_OK) {
      return ret;
    }
    return gptimer_start(gptimer_);
  }

  esp_err_t Stop() const {
    if (!gptimer_) {
      return ESP_ERR_INVALID_STATE;
    }
    return gptimer_stop(gptimer_);
  }

  bool IsValid() const { return gptimer_ != nullptr; }

 private:
  mutable gptimer_handle_t gptimer_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_GPTIMER_H_
