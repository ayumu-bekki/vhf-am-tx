#ifndef AM_TRANSMITTER_MAIN_GPIO_INPUT_WATCH_TASK_H_
#define AM_TRANSMITTER_MAIN_GPIO_INPUT_WATCH_TASK_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Button input watch task (spec 9.7)

#include <driver/gpio.h>
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>

#include <vector>

#include "button_event.h"
#include "gptimer.h"
#include "hardware_config.h"
#include "logger.h"
#include "message_queue.h"
#include "task.h"

namespace transmitter_system {

/// Polls the buttons on a timer and turns them into events.
///
/// Polling rather than edge interrupts: the rates involved are tiny next to
/// the 25kHz modulation tick, and polling keeps debouncing trivial and
/// predictable. It also keeps button handling off the interrupt path entirely,
/// which matters because the master tick wants the interrupt controller to
/// itself (spec 7.2).
class GpioInputWatchTask final : public Task {
 public:
  static constexpr std::string_view kTaskName = "GpioInputWatchTask";
  static constexpr int32_t kPriority = Task::PRIORITY_NORMAL;
  static constexpr int32_t kCoreId = PRO_CPU_NUM;

  enum class GpioPullUpDown {
    kNone,
    kPullUpResistorEnable,
    kPullDownResistorEnable,
  };

  /// One monitored button.
  ///
  /// Auto-repeat ramps in two stages (spec 9.7): nothing for 600ms, then 8
  /// repeats per second, then 25 per second after a further 2 seconds. Editing
  /// a callsign prefix means walking 29 options, so without this the operator
  /// would be pressing the button all day.
  class GpioInfo {
   public:
    enum class State : uint8_t {
      kIdle,
      kDebouncing,
      kPressed,
      kRepeatSlow,
      kRepeatFast,
    };

    GpioInfo(gpio_num_t gpio_no, ButtonEvent::Button button_id,
             MessageQueue<ButtonEvent>* event_queue, bool allow_repeat)
        : gpio_no_(gpio_no),
          button_id_(button_id),
          event_queue_(event_queue),
          allow_repeat_(allow_repeat),
          state_(State::kIdle),
          counter_(0),
          hold_ticks_(0) {}

    gpio_num_t GetGpioNo() const { return gpio_no_; }

    /// Called once per polling interval.
    void Check();

   private:
    void Emit(ButtonEvent::Type type) {
      if (event_queue_) {
        event_queue_->Send({button_id_, type});
      }
    }

    gpio_num_t gpio_no_;
    ButtonEvent::Button button_id_;
    MessageQueue<ButtonEvent>* event_queue_;
    bool allow_repeat_;

    State state_;
    uint32_t counter_;     // counts polling intervals within the current state
    uint32_t hold_ticks_;  // total intervals held down, for the fast stage
  };

 public:
  GpioInputWatchTask();
  ~GpioInputWatchTask();

  void Initialize() override;
  void Update() override;

  void AddMonitor(GpioInfo gpio_info,
                  GpioPullUpDown pull = GpioPullUpDown::kPullUpResistorEnable);

 private:
  // Runs in ISR context; GPTIMER_ISR_CACHE_SAFE requires it to be in IRAM.
  static bool IRAM_ATTR TimerCallback(
      gptimer_handle_t timer, const gptimer_alarm_event_data_t* event_data,
      void* message_queue);

 private:
  MessageQueue<int> message_queue_;
  GPTimer gptimer_;
  std::vector<GpioInfo> gpio_list_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_GPIO_INPUT_WATCH_TASK_H_
