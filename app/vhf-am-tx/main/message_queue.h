#ifndef AM_TRANSMITTER_MAIN_MESSAGE_QUEUE_H_
#define AM_TRANSMITTER_MAIN_MESSAGE_QUEUE_H_
// ESP32 144MHz AM Transmitter
// (C)2024 bekki.jp

// Include ----------------------
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <cstdint>

namespace transmitter_system {

/// Fixed-size FreeRTOS queue of trivially copyable T.
template <typename T>
class MessageQueue {
 public:
  MessageQueue() = default;
  ~MessageQueue() { Destroy(); }

  MessageQueue(const MessageQueue&) = delete;
  MessageQueue& operator=(const MessageQueue&) = delete;

  bool Create(const UBaseType_t length) {
    if (!queue_) {
      queue_ = xQueueCreate(length, sizeof(T));
    }
    return queue_ != nullptr;
  }

  void Destroy() {
    if (queue_) {
      vQueueDelete(queue_);
      queue_ = nullptr;
    }
  }

  /// Waits up to `timeout_ms` for an item. False on timeout.
  bool ReceiveWait(T* const out, const uint32_t timeout_ms) {
    return queue_ &&
           xQueueReceive(queue_, out, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
  }

  /// Non-blocking send. False when the queue is full (the item is dropped).
  bool Send(const T& data) {
    return queue_ && xQueueSend(queue_, &data, 0) == pdTRUE;
  }

 private:
  QueueHandle_t queue_ = nullptr;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_MESSAGE_QUEUE_H_
