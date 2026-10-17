// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Persistent settings (spec 9.6)

#include "transmitter_settings.h"

#include <esp_timer.h>

#include <cstring>

#include "logger.h"

namespace transmitter_system {

namespace {
int64_t NowMs() { return esp_timer_get_time() / 1000; }
}  // namespace

TransmitterSettings::TransmitterSettings()
    : nvs_handle_(0),
      nvs_handle_open_(false),
      dirty_(false),
      last_change_ms_(0),
      force_(false),
      stop_writer_(false),
      writer_exited_(false),
      writer_task_(nullptr),
      frequency_hz_(hardware_config::kFreqDefaultHz),
      callsign_{},
      ardf_id_(static_cast<morse::ArdfId>(hardware_config::kArdfIdDefault)),
      cw_wpm_(hardware_config::kCwWpmDefault) {
  std::strncpy(callsign_, hardware_config::kCallDefault, sizeof(callsign_) - 1);
  callsign_[sizeof(callsign_) - 1] = '\0';
}

TransmitterSettings::~TransmitterSettings() {
  if (writer_task_) {
    stop_writer_.store(true);
    xTaskNotifyGive(writer_task_);
    for (int i = 0; i < 50 && !writer_exited_.load(); ++i) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    writer_task_ = nullptr;
  }
  // Do not lose a pending change on shutdown.
  if (dirty_) {
    Commit();
  }
  CloseNvsHandle();
}

bool TransmitterSettings::OpenNvsHandle() {
  if (nvs_handle_open_) {
    return true;
  }
  esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &nvs_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Failed to open NVS: %s", esp_err_to_name(err));
    return false;
  }
  nvs_handle_open_ = true;
  return true;
}

void TransmitterSettings::CloseNvsHandle() {
  if (nvs_handle_open_) {
    nvs_close(nvs_handle_);
    nvs_handle_open_ = false;
  }
}

bool TransmitterSettings::Load() {
  nvs_handle_t read_handle;
  esp_err_t err = nvs_open(kNvsNamespace, NVS_READONLY, &read_handle);
  if (err != ESP_OK) {
    ESP_LOGI(kTag, "No saved settings, using defaults");
    OpenNvsHandle();
    return false;
  }

  uint32_t freq = frequency_hz_;
  uint8_t ardf_id = static_cast<uint8_t>(ardf_id_);
  uint8_t wpm = cw_wpm_;

  nvs_get_u32(read_handle, kKeyFrequency, &freq);
  nvs_get_u8(read_handle, kKeyArdfId, &ardf_id);
  nvs_get_u8(read_handle, kKeyCwWpm, &wpm);

  char call[sizeof(callsign_)] = {};
  size_t call_len = sizeof(call);
  if (nvs_get_str(read_handle, kKeyCallsign, call, &call_len) == ESP_OK) {
    std::strncpy(callsign_, call, sizeof(callsign_) - 1);
    callsign_[sizeof(callsign_) - 1] = '\0';
  }

  nvs_close(read_handle);

  // Validate everything that comes back. A corrupted or out-of-range value
  // must not be able to put the transmitter somewhere unsafe.
  frequency_hz_ = (freq >= hardware_config::kFreqMinHz &&
                   freq <= hardware_config::kFreqMaxHz)
                      ? freq
                      : hardware_config::kFreqDefaultHz;

  ardf_id_ = (ardf_id < static_cast<uint8_t>(morse::ArdfId::kCount))
                 ? static_cast<morse::ArdfId>(ardf_id)
                 : static_cast<morse::ArdfId>(hardware_config::kArdfIdDefault);

  cw_wpm_ = (wpm >= hardware_config::kCwWpmMin &&
             wpm <= hardware_config::kCwWpmMax)
                ? wpm
                : hardware_config::kCwWpmDefault;

  ESP_LOGI(kTag,
           "Settings loaded: freq=%u call=%s ardf=%u wpm=%u", frequency_hz_, callsign_, static_cast<uint8_t>(ardf_id_), cw_wpm_);

  if (OpenNvsHandle()) {
    // Drop the carrier duty an older build stored. Read back, a value
    // calibrated for the old modulator (93) would put 3.84V on the drain.
    // This runs before the ADC and the tick start, so the flash write cannot
    // collide with an ISR. After the first run the key is gone and this does
    // nothing.
    bool erased = false;
    if (nvs_erase_key(nvs_handle_, kKeyLegacyDutyCarrier) == ESP_OK) {
      ESP_LOGW(kTag, "Erased stale stored carrier duty");
      erased = true;
    }
    // The operating mode is no longer stored; the unit always starts in VOICE.
    if (nvs_erase_key(nvs_handle_, kKeyLegacyMode) == ESP_OK) {
      erased = true;
    }
    if (erased) {
      nvs_commit(nvs_handle_);
    }
  }
  dirty_ = false;
  return true;
}

void TransmitterSettings::MarkDirty() {
  // Callers hold mutex_.
  last_change_ms_.store(NowMs());
  dirty_.store(true);
}

bool TransmitterSettings::StartWriter() {
  if (writer_task_) {
    return true;
  }
  stop_writer_.store(false);
  writer_exited_.store(false);
  // Below every other task: a commit may take as long as it likes, as long as
  // it never gets in the way of the buttons, the display or the keyer.
  const BaseType_t ok = xTaskCreate(&TransmitterSettings::WriterTask,
                                    "settings_writer", 4096, this,
                                    tskIDLE_PRIORITY + 1, &writer_task_);
  if (ok != pdPASS) {
    writer_task_ = nullptr;
    ESP_LOGE(kTag, "Failed to start settings writer; saving synchronously");
    return false;
  }
  return true;
}

void TransmitterSettings::WriterTask(void* arg) {
  static_cast<TransmitterSettings*>(arg)->WriterLoop();
  vTaskDelete(nullptr);
}

void TransmitterSettings::WriterLoop() {
  while (!stop_writer_.load()) {
    // Woken early by a forced commit, otherwise polls every 100ms.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

    if (!dirty_.load()) {
      force_.store(false);
      continue;
    }
    const bool force = force_.exchange(false);
    if (!force && (NowMs() - last_change_ms_.load()) <
                      hardware_config::kNvsWriteDelayMs) {
      continue;
    }
    Commit();
  }
  writer_exited_.store(true);
}

void TransmitterSettings::Tick(bool force) {
  if (!dirty_.load()) {
    return;
  }
  if (writer_task_) {
    if (force) {
      force_.store(true);
      xTaskNotifyGive(writer_task_);
    }
    return;
  }
  if (!force && (NowMs() - last_change_ms_.load()) <
                    hardware_config::kNvsWriteDelayMs) {
    return;
  }
  Commit();
}

bool TransmitterSettings::Commit() {
  if (!OpenNvsHandle()) {
    return false;
  }

  // Copy under the lock, write without it: the flash work is the slow part
  // and must not hold up the setters. A change that lands during the write
  // sets dirty_ again and goes out with the next commit.
  uint32_t frequency_hz;
  char callsign[sizeof(callsign_)];
  uint8_t ardf_id;
  uint8_t cw_wpm;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    frequency_hz = frequency_hz_;
    std::memcpy(callsign, callsign_, sizeof(callsign));
    ardf_id = static_cast<uint8_t>(ardf_id_);
    cw_wpm = cw_wpm_;
    dirty_.store(false);
  }

  esp_err_t err = ESP_OK;
  if (err == ESP_OK) err = nvs_set_u32(nvs_handle_, kKeyFrequency, frequency_hz);
  if (err == ESP_OK) err = nvs_set_str(nvs_handle_, kKeyCallsign, callsign);
  if (err == ESP_OK) err = nvs_set_u8(nvs_handle_, kKeyArdfId, ardf_id);
  if (err == ESP_OK) err = nvs_set_u8(nvs_handle_, kKeyCwWpm, cw_wpm);
  if (err == ESP_OK) err = nvs_commit(nvs_handle_);

  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Failed to save settings: %s", esp_err_to_name(err));
    dirty_.store(true);  // try again later
    return false;
  }

  ESP_LOGD(kTag, "Settings saved");
  return true;
}

void TransmitterSettings::SetFrequencyHz(uint32_t hz) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (hz < hardware_config::kFreqMinHz) {
    hz = hardware_config::kFreqMinHz;
  }
  if (hz > hardware_config::kFreqMaxHz) {
    hz = hardware_config::kFreqMaxHz;
  }
  if (frequency_hz_ == hz) {
    return;
  }
  frequency_hz_ = hz;
  MarkDirty();
}

void TransmitterSettings::SetCallsign(const char* callsign) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!callsign) {
    return;
  }
  if (std::strncmp(callsign_, callsign, sizeof(callsign_) - 1) == 0) {
    return;
  }
  std::strncpy(callsign_, callsign, sizeof(callsign_) - 1);
  callsign_[sizeof(callsign_) - 1] = '\0';
  MarkDirty();
}

void TransmitterSettings::SetArdfId(morse::ArdfId id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (static_cast<uint8_t>(id) >= static_cast<uint8_t>(morse::ArdfId::kCount)) {
    return;
  }
  if (ardf_id_ == id) {
    return;
  }
  ardf_id_ = id;
  MarkDirty();
}

void TransmitterSettings::SetCwWpm(uint8_t wpm) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (wpm < hardware_config::kCwWpmMin) {
    wpm = hardware_config::kCwWpmMin;
  }
  if (wpm > hardware_config::kCwWpmMax) {
    wpm = hardware_config::kCwWpmMax;
  }
  if (cw_wpm_ == wpm) {
    return;
  }
  cw_wpm_ = wpm;
  MarkDirty();
}

}  // namespace transmitter_system

// EOF
