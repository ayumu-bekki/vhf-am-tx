#ifndef AM_TRANSMITTER_MAIN_TRANSMITTER_SETTINGS_H_
#define AM_TRANSMITTER_MAIN_TRANSMITTER_SETTINGS_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Persistent settings (spec 9.6)

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs.h>

#include <atomic>
#include <cstdint>
#include <mutex>

#include "emission.h"
#include "hardware_config.h"
#include "morse.h"

namespace transmitter_system {

/// NVS-backed settings.
///
/// Writes are deferred: a change marks the set dirty and the actual commit
/// happens once nothing has changed for 3 seconds, or when the mode is left.
/// Committing on every button press would burn flash endurance for no reason,
/// and auto-repeat can generate 25 changes a second.
///
/// Once StartWriter() has run, commits happen on a low priority background
/// task. A flash erase/write takes tens of milliseconds or more, and doing it
/// on the UI task made the buttons feel laggy. Nothing is written unless a
/// value actually changed.
class TransmitterSettings final {
 public:
  TransmitterSettings();
  ~TransmitterSettings();

  /// Reads everything back, falling back to the defaults in
  /// hardware_config.h for keys that are missing (first boot).
  bool Load();

  /// Starts the background writer. Call once, after Load(). Until then (host
  /// tests, early boot) Tick() commits synchronously.
  bool StartWriter();

  /// With the writer running: `force` asks it to commit right away, without
  /// the hold-off; otherwise this does nothing, the writer polls on its own.
  /// Without the writer: commits if dirty and the hold-off has elapsed, or
  /// at once when `force` is set, on the calling thread.
  void Tick(bool force = false);

  bool IsDirty() const { return dirty_.load(); }

  /// False when NVS could not be opened, i.e. nothing will be saved.
  bool IsStorageAvailable() const { return nvs_handle_open_; }

  // --- accessors; setters mark dirty and restart the hold-off --------------

  uint32_t GetFrequencyHz() const { return frequency_hz_; }
  void SetFrequencyHz(uint32_t hz);

  /// Callsign: kCallLength (6) characters of A-Z / 0-9 (spec 9.4.4), or an
  /// empty string while none has been saved. SetCallsign() rejects anything
  /// that is not a complete callsign.
  const char* GetCallsign() const { return callsign_; }
  bool HasCallsign() const { return callsign_[0] != '\0'; }
  void SetCallsign(const char* callsign);
  static bool IsValidCallsign(const char* callsign);

  morse::ArdfId GetArdfId() const { return ardf_id_; }
  void SetArdfId(morse::ArdfId id);

  uint8_t GetCwWpm() const { return cw_wpm_; }
  void SetCwWpm(uint8_t wpm);

  /// Output stage, 0..kPowerCount-1 (spec 9.6, key "power").
  uint8_t GetPower() const { return power_; }
  void SetPower(uint8_t power);

  /// Class of emission for the ARDF and CW modes (key "emission").
  Emission GetEmission() const { return emission_; }
  void SetEmission(Emission emission);

 private:
  bool OpenNvsHandle();
  void CloseNvsHandle();
  bool Commit();
  void MarkDirty();

  static void WriterTask(void* arg);
  void WriterLoop();

 private:
  nvs_handle_t nvs_handle_;
  bool nvs_handle_open_;

  // Setters (UI task) and the writer task meet here. The mutex covers the
  // value fields below; the flags are atomics so Tick() can look at them
  // without taking it.
  std::mutex mutex_;
  std::atomic<bool> dirty_;
  std::atomic<int64_t> last_change_ms_;
  std::atomic<bool> force_;
  std::atomic<bool> stop_writer_;
  std::atomic<bool> writer_exited_;
  TaskHandle_t writer_task_;

  uint32_t frequency_hz_;
  char callsign_[hardware_config::kCallLength + 1];
  morse::ArdfId ardf_id_;
  uint8_t cw_wpm_;
  uint8_t power_;
  Emission emission_;

  static constexpr const char* kNvsNamespace = "amtx";
  static constexpr const char* kKeyLegacyMode = "mode";
  static constexpr const char* kKeyFrequency = "freq_hz";
  static constexpr const char* kKeyCallsign = "call";
  static constexpr const char* kKeyArdfId = "ardf_id";
  static constexpr const char* kKeyCwWpm = "cw_wpm";
  static constexpr const char* kKeyPower = "power";
  static constexpr const char* kKeyEmission = "emission";
  // Written by firmware that stored the calibrated carrier duty. The duty is
  // fixed by the modulator circuit (hardware_config.h) and never persisted
  // now; a stale value read back from an older board would put the wrong
  // voltage on the drain. Erased at start-up.
  static constexpr const char* kKeyLegacyDutyCarrier = "duty_car";
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_TRANSMITTER_SETTINGS_H_
