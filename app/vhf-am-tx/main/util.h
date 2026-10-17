#ifndef AM_TRANSMITTER_MAIN_UTIL_H_
#define AM_TRANSMITTER_MAIN_UTIL_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp

// Include ----------------------
#include <esp_timer.h>

#include <cstdint>

namespace transmitter_system {
namespace util {

/// Blocks the calling task for at least `sleep_milliseconds` (rounded to the
/// FreeRTOS tick).
void SleepMillisecond(const uint32_t sleep_milliseconds);

/// Milliseconds since boot (esp_timer). Inline so the host tests, which
/// fake esp_timer_get_time(), do not need util.cc.
inline int64_t NowMs() { return esp_timer_get_time() / 1000; }

/// Initialize NVS flash (call once at startup)
bool InitializeNvs();

}  // namespace util
}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_UTIL_H_
