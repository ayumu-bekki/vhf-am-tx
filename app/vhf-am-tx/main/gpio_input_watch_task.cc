// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Button input watch task (spec 9.7)

#include "gpio_input_watch_task.h"

namespace transmitter_system {

namespace {

// Debounce is specified in milliseconds; convert to polling intervals.
constexpr uint32_t kDebounceTicks = hardware_config::kBtnDebounceCount;

// Auto-repeat staging, all in polling intervals.
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

constexpr int kQueueTimeoutMs = 1000;

}  // namespace

void GpioInputWatchTask::GpioInfo::Check() {
  // Buttons pull to ground, so a low level means pressed.
  const bool down = (gpio_get_level(gpio_no_) == 0);

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
        state_ = State::kIdle;
        counter_ = 0;
        Emit(ButtonEvent::Type::kRelease);
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
        state_ = State::kIdle;
        counter_ = 0;
        Emit(ButtonEvent::Type::kRelease);
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
        state_ = State::kIdle;
        counter_ = 0;
        Emit(ButtonEvent::Type::kRelease);
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
    : Task(std::string(kTaskName), kPriority, kCoreId),
      message_queue_(),
      gptimer_(),
      gpio_list_() {
  if (!message_queue_.Create(hardware_config::kGpioMessageQueueSize)) {
    ESP_LOGE(kTag, "Creating button poll queue failed");
  }

  gptimer_.Create(hardware_config::kGpioTimerResolutionHz,
                  &GpioInputWatchTask::TimerCallback, &message_queue_);
}

GpioInputWatchTask::~GpioInputWatchTask() {
  gptimer_.Destroy();
  message_queue_.Destroy();
}

void GpioInputWatchTask::Initialize() {
  ESP_LOGI(kTag, "Start GpioInputWatchTask");
}

void GpioInputWatchTask::Update() {
  gptimer_.Start(hardware_config::kButtonPollIntervalMs * 1000);

  int event_type = 0;
  while (true) {
    if (message_queue_.ReceiveWait(&event_type, kQueueTimeoutMs)) {
      if (event_type == 1) {
        for (auto&& gpio_info : gpio_list_) {
          gpio_info.Check();
        }
      }
    }
  }
}

void GpioInputWatchTask::AddMonitor(GpioInfo gpio_info, GpioPullUpDown pull) {
  // External pull-ups are not fitted, so the internal ones must be on
  // (spec 1.1).
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = 1ull << gpio_info.GetGpioNo();
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = (pull == GpioPullUpDown::kPullUpResistorEnable)
                           ? GPIO_PULLUP_ENABLE
                           : GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = (pull == GpioPullUpDown::kPullDownResistorEnable)
                             ? GPIO_PULLDOWN_ENABLE
                             : GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;  // polled, not interrupt driven

  gpio_config(&io_conf);

  gpio_list_.emplace_back(std::move(gpio_info));
}

bool GpioInputWatchTask::TimerCallback(
    gptimer_handle_t timer, const gptimer_alarm_event_data_t* event_data,
    void* message_queue) {
  MessageQueue<int>* const queue =
      static_cast<MessageQueue<int>*>(message_queue);
  return queue->SendFromISR(1);
}

}  // namespace transmitter_system

// EOF
