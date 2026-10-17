#ifndef TEST_STUB_FREERTOS_H_
#define TEST_STUB_FREERTOS_H_
// Host stand-in: just enough FreeRTOS for TransmitterSettings' writer task.
// Tasks are std::threads and notifications are a condition variable.
#include <cstdint>
typedef int BaseType_t;
typedef uint32_t TickType_t;
#define pdPASS 1
#define pdTRUE 1
#define pdMS_TO_TICKS(ms) (ms)
#define tskIDLE_PRIORITY 0
#endif
