#ifndef AM_TRANSMITTER_MAIN_GPIO_INPUT_WATCH_TASK_H_
#define AM_TRANSMITTER_MAIN_GPIO_INPUT_WATCH_TASK_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Button input watch task (spec 9.7)

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>

#include <vector>

#include "button_event.h"
#include "message_queue.h"
#include "task.h"

namespace transmitter_system {

/// Polls the buttons every kButtonPollIntervalMs and turns them into events.
///
/// Polling rather than edge interrupts: the rates involved are tiny next to
/// the 25kHz modulation tick, and polling keeps debouncing trivial and
/// predictable. The period comes from vTaskDelayUntil() on the 1kHz FreeRTOS
/// tick, so button handling never touches the interrupt controller, which
/// the master tick wants to itself (spec 7.2).
class GpioInputWatchTask final : public Task {
 public:
  static constexpr std::string_view kTaskName = "GpioInputWatchTask";
  static constexpr UBaseType_t kPriority = Task::kPriorityNormal;
  static constexpr BaseType_t kCoreId = PRO_CPU_NUM;
  static constexpr uint32_t kStackDepth = 4096;

  /// One monitored button. Buttons pull the pin to ground; the internal
  /// pull-up holds it high otherwise (spec 1.1).
  ///
  /// Auto-repeat ramps in two stages (spec 9.7): nothing for 600ms, then 8
  /// repeats per second, then 25 per second after a further 2 seconds. Editing
  /// a callsign prefix means walking 29 options, so without this the operator
  /// would be pressing the button all day.
  class Button {
   public:
    Button(gpio_num_t gpio, ButtonEvent::Button id,
           MessageQueue<ButtonEvent>* event_queue, bool allow_repeat)
        : gpio_(gpio),
          id_(id),
          event_queue_(event_queue),
          allow_repeat_(allow_repeat) {}

    gpio_num_t GetGpio() const { return gpio_; }

    /// Called once per polling interval.
    void Poll();

   private:
    enum class State : uint8_t {
      kIdle,
      kDebouncing,
      kPressed,
      kRepeatSlow,
      kRepeatFast,
    };

    void Emit(ButtonEvent::Type type) {
      if (event_queue_) {
        event_queue_->Send({id_, type});
      }
    }
    void Release() {
      state_ = State::kIdle;
      counter_ = 0;
      Emit(ButtonEvent::Type::kRelease);
    }

    gpio_num_t gpio_;
    ButtonEvent::Button id_;
    MessageQueue<ButtonEvent>* event_queue_;
    bool allow_repeat_;

    State state_ = State::kIdle;
    uint32_t counter_ = 0;     // polling intervals within the current state
    uint32_t hold_ticks_ = 0;  // total intervals held down, for the fast stage
  };

  GpioInputWatchTask();

  /// Configures the pin and adds it to the poll list. Call before Start().
  void AddButton(const Button& button);

 protected:
  void Initialize() override;
  void Update() override;

 private:
  std::vector<Button> buttons_;
  TickType_t last_wake_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_GPIO_INPUT_WATCH_TASK_H_
