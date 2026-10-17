// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp

// Include ----------------------
#include "util.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>

#include <sstream>

#include "logger.h"

namespace transmitter_system {
namespace util {

/// Sleep
void SleepMillisecond(const uint32_t sleep_milliseconds) {
  TickType_t last_wake_time = xTaskGetTickCount();
  vTaskDelayUntil(&last_wake_time, sleep_milliseconds / portTICK_PERIOD_MS);
}

std::vector<std::string> SplitString(const std::string& str, const char delim) {
  std::vector<std::string> elements;
  std::stringstream ss(str);
  std::string item;
  while (getline(ss, item, delim)) {
    if (!item.empty()) {
      elements.push_back(item);
    }
  }
  return elements;
}

bool InitializeNvs() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(kTag, "NVS needs erase, erasing...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "NVS init failed: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(kTag, "NVS initialized");
  return true;
}

}  // namespace util
}  // namespace transmitter_system
