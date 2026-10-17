#ifndef TEST_STUB_FREERTOS_TASK_H_
#define TEST_STUB_FREERTOS_TASK_H_
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "FreeRTOS.h"

struct StubTask {
  std::mutex m;
  std::condition_variable cv;
  uint32_t notified = 0;
  std::thread th;
};
typedef StubTask* TaskHandle_t;
typedef void (*TaskFunction_t)(void*);

inline thread_local StubTask* t_current_task = nullptr;

inline BaseType_t xTaskCreate(TaskFunction_t fn, const char*, uint32_t, void* arg,
                              uint32_t, TaskHandle_t* out) {
  auto* t = new StubTask();
  *out = t;
  t->th = std::thread([t, fn, arg] {
    t_current_task = t;
    fn(arg);
  });
  t->th.detach();
  return pdPASS;
}
inline void vTaskDelete(TaskHandle_t) {}
inline void vTaskDelay(TickType_t ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
inline BaseType_t xTaskNotifyGive(TaskHandle_t t) {
  std::lock_guard<std::mutex> l(t->m);
  t->notified++;
  t->cv.notify_one();
  return pdTRUE;
}
inline uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ms) {
  StubTask* t = t_current_task;
  std::unique_lock<std::mutex> l(t->m);
  t->cv.wait_for(l, std::chrono::milliseconds(ms), [t] { return t->notified > 0; });
  const uint32_t n = t->notified;
  t->notified = clear ? 0 : (n ? n - 1 : 0);
  return n;
}
#endif
