// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp

// Include ----------------------
#include "util.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>

#include "logger.h"

namespace transmitter_system {
namespace util {

void SleepMillisecond(const uint32_t sleep_milliseconds) {
  vTaskDelay(pdMS_TO_TICKS(sleep_milliseconds));
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
