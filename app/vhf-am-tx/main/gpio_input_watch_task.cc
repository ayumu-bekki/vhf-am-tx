// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Button input watch task (spec 9.7)

#include "gpio_input_watch_task.h"

#include <freertos/task.h>

#include "hardware_config.h"
#include "logger.h"

namespace transmitter_system {

namespace {

// Debounce and auto-repeat staging, all in polling intervals.
constexpr uint32_t kDebounceTicks =
    hardware_config::kBtnDebounceMs / hardware_config::kButtonPollIntervalMs;
constexpr uint32_t kRepeatDelayTicks =
    hardware_config::kBtnRepeatDelayMs / hardware_config::kButtonPollIntervalMs;
constexpr uint32_t kRepeatSlowPeriodTicks =
    1000u / (hardware_config::kBtnRepeatSlowHz *
             hardware_config::kButtonPollIntervalMs);
constexpr uint32_t kRepeatFastAfterTicks =
    hardware_config::kBtnRepeatFastAfterMs /
    hardware_config::kButtonPollIntervalMs;
constexpr uint32_t kRepeatFastPeriodTicks =
    1000u / (hardware_config::kBtnRepeatFastHz *
             hardware_config::kButtonPollIntervalMs);

static_assert(pdMS_TO_TICKS(hardware_config::kButtonPollIntervalMs) > 0,
              "button poll interval is shorter than the FreeRTOS tick");

}  // namespace

void GpioInputWatchTask::Button::Poll() {
  // Buttons pull to ground, so a low level means pressed.
  const bool down = (gpio_get_level(gpio_) == 0);

  switch (state_) {
    case State::kIdle:
      if (down) {
        state_ = State::kDebouncing;
        counter_ = 1;
      }
      break;

    case State::kDebouncing:
      if (!down) {
        // Bounce, not a real press.
        state_ = State::kIdle;
        counter_ = 0;
        break;
      }
      if (++counter_ >= kDebounceTicks) {
        state_ = State::kPressed;
        counter_ = 0;
        hold_ticks_ = 0;
        Emit(ButtonEvent::Type::kPress);
      }
      break;

    case State::kPressed:
      if (!down) {
        Release();
        break;
      }
      ++hold_ticks_;
      if (allow_repeat_ && ++counter_ >= kRepeatDelayTicks) {
        state_ = State::kRepeatSlow;
        counter_ = 0;
        Emit(ButtonEvent::Type::kRepeat);
      }
      break;

    case State::kRepeatSlow:
      if (!down) {
        Release();
        break;
      }
      ++hold_ticks_;
      if (++counter_ >= kRepeatSlowPeriodTicks) {
        counter_ = 0;
        Emit(ButtonEvent::Type::kRepeat);
      }
      if (hold_ticks_ >= kRepeatFastAfterTicks) {
        state_ = State::kRepeatFast;
        counter_ = 0;
      }
      break;

    case State::kRepeatFast:
      if (!down) {
        Release();
        break;
      }
      if (++counter_ >= kRepeatFastPeriodTicks) {
        counter_ = 0;
        Emit(ButtonEvent::Type::kRepeat);
      }
      break;
  }
}

GpioInputWatchTask::GpioInputWatchTask()
    : Task(kTaskName, kPriority, kCoreId, kStackDepth), last_wake_(0) {}

void GpioInputWatchTask::AddButton(const Button& button) {
  // External pull-ups are not fitted, so the internal ones must be on
  // (spec 1.1).
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = 1ull << button.GetGpio();
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;  // polled, not interrupt driven
  gpio_config(&io_conf);

  buttons_.push_back(button);
}

void GpioInputWatchTask::Initialize() {
  ESP_LOGI(kTag, "Start GpioInputWatchTask");
  last_wake_ = xTaskGetTickCount();
}

void GpioInputWatchTask::Update() {
  vTaskDelayUntil(&last_wake_,
                  pdMS_TO_TICKS(hardware_config::kButtonPollIntervalMs));
  for (auto& button : buttons_) {
    button.Poll();
  }
}

}  // namespace transmitter_system
