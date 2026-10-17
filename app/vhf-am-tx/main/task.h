#ifndef AM_TRANSMITTER_MAIN_TASK_H_
#define AM_TRANSMITTER_MAIN_TASK_H_
// ESP32 144MHz AM Transmitter
// (C)2024 bekki.jp

// Include ----------------------
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <string_view>

namespace transmitter_system {

/// Thin wrapper around a FreeRTOS task.
///
/// Start() creates the task, which calls Initialize() once and then Update()
/// repeatedly until Stop(). Update() is one iteration of the task's loop and
/// is expected to block (queue wait, delay) so it does not spin.
class Task {
 public:
  static constexpr uint32_t kDefaultStackDepth = 8192;

  /// Task priorities, relative to the top of the FreeRTOS range.
  static constexpr UBaseType_t kPriorityTop = configMAX_PRIORITIES - 1;
  static constexpr UBaseType_t kPriorityHigh = kPriorityTop - 3;
  static constexpr UBaseType_t kPriorityNormal = kPriorityTop - 4;
  static constexpr UBaseType_t kPriorityLow = 0;

  Task(std::string_view name, UBaseType_t priority, BaseType_t core_id,
       uint32_t stack_depth = kDefaultStackDepth);
  virtual ~Task();

  Task(const Task&) = delete;
  Task& operator=(const Task&) = delete;

  /// Creates the FreeRTOS task. Calling it again is a no-op.
  bool Start();

  /// Asks the loop to end after the current Update().
  void Stop();

 protected:
  /// Called once on the new task before the first Update().
  virtual void Initialize() {}

  /// One iteration of the task loop.
  virtual void Update() = 0;

 private:
  enum class Status : uint8_t { kReady, kRunning, kStopped };

  static void Entry(void* param);
  void Run();

  std::atomic<Status> status_;
  std::string name_;
  UBaseType_t priority_;
  BaseType_t core_id_;
  uint32_t stack_depth_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_TASK_H_
