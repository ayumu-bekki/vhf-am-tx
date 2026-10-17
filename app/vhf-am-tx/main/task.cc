// ESP32 144MHz AM Transmitter
// (C)2024 bekki.jp

// Include ----------------------
#include "task.h"

#include "logger.h"

namespace transmitter_system {

Task::Task(std::string_view name, UBaseType_t priority, BaseType_t core_id,
           uint32_t stack_depth)
    : status_(Status::kReady),
      name_(name),
      priority_(priority),
      core_id_(core_id),
      stack_depth_(stack_depth) {}

Task::~Task() { Stop(); }

bool Task::Start() {
  Status expected = Status::kReady;
  if (!status_.compare_exchange_strong(expected, Status::kRunning)) {
    return expected == Status::kRunning;
  }
  const BaseType_t ok =
      xTaskCreatePinnedToCore(&Task::Entry, name_.c_str(), stack_depth_, this,
                              priority_, nullptr, core_id_);
  if (ok != pdPASS) {
    ESP_LOGE(kTag, "Failed to create task %s", name_.c_str());
    status_.store(Status::kReady);
    return false;
  }
  return true;
}

void Task::Stop() {
  Status expected = Status::kRunning;
  status_.compare_exchange_strong(expected, Status::kStopped);
}

void Task::Run() {
  Initialize();
  while (status_.load() == Status::kRunning) {
    Update();
  }
}

void Task::Entry(void* param) {
  static_cast<Task*>(param)->Run();
  vTaskDelete(nullptr);
}

}  // namespace transmitter_system
