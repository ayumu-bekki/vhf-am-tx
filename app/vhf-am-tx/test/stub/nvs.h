#ifndef TEST_STUB_NVS_H_
#define TEST_STUB_NVS_H_
// Host stand-in for NVS: an in-memory map whose commit is deliberately slow,
// like a flash write, and which counts how many commits happened.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "esp_err.h"

typedef uint32_t nvs_handle_t;
enum nvs_open_mode_t { NVS_READONLY, NVS_READWRITE };

struct StubNvs {
  std::mutex m;
  std::map<std::string, std::string> kv;
  std::atomic<int> commits{0};
  std::atomic<int> commit_ms{50};
};
inline StubNvs g_nvs;

inline esp_err_t nvs_open(const char*, nvs_open_mode_t, nvs_handle_t* h) {
  *h = 1;
  return ESP_OK;
}
inline void nvs_close(nvs_handle_t) {}
template <typename T>
inline esp_err_t nvs_set_int(const char* k, T v) {
  std::lock_guard<std::mutex> l(g_nvs.m);
  g_nvs.kv[k] = std::to_string((unsigned long long)v);
  return ESP_OK;
}
inline esp_err_t nvs_set_u8(nvs_handle_t, const char* k, uint8_t v) { return nvs_set_int(k, v); }
inline esp_err_t nvs_set_u32(nvs_handle_t, const char* k, uint32_t v) { return nvs_set_int(k, v); }
inline esp_err_t nvs_set_str(nvs_handle_t, const char* k, const char* v) {
  std::lock_guard<std::mutex> l(g_nvs.m);
  g_nvs.kv[k] = v;
  return ESP_OK;
}
inline esp_err_t nvs_commit(nvs_handle_t) {
  std::this_thread::sleep_for(std::chrono::milliseconds(g_nvs.commit_ms.load()));
  g_nvs.commits++;
  return ESP_OK;
}
inline esp_err_t nvs_erase_key(nvs_handle_t, const char* k) {
  std::lock_guard<std::mutex> l(g_nvs.m);
  return g_nvs.kv.erase(k) ? ESP_OK : ESP_ERR_NOT_FOUND;
}
inline esp_err_t nvs_get_u8(nvs_handle_t, const char*, uint8_t*) { return ESP_ERR_NOT_FOUND; }
inline esp_err_t nvs_get_u32(nvs_handle_t, const char*, uint32_t*) { return ESP_ERR_NOT_FOUND; }
inline esp_err_t nvs_get_str(nvs_handle_t, const char*, char*, size_t*) { return ESP_ERR_NOT_FOUND; }
#endif
