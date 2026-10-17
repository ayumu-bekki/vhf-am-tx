#ifndef AM_TRANSMITTER_MAIN_UI_CONTROLLER_TASK_H_
#define AM_TRANSMITTER_MAIN_UI_CONTROLLER_TASK_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Modes, buttons and display (spec 9)

#include <cstdint>
#include <memory>
#include <string_view>

#include "ardf_scheduler.h"
#include "button_event.h"
#include "message_queue.h"
#include "modulator.h"
#include "morse.h"
#include "si5351a.h"
#include "st7032.h"
#include "task.h"
#include "transmitter_settings.h"
#include "tx_controller.h"
#include "voice_dsp.h"

namespace transmitter_system {

/// Operating modes (spec 9.1), in the order SW_MODE cycles through.
///
/// The three test modes are appended to the same list. They are needed in the
/// field - the duty calibration of spec 11 is done against an oscilloscope,
/// and the serial console is unavailable because the power rails are split at
/// the header - so they have to be reachable from the buttons alone.
enum class UiMode : uint8_t {
  kVoice = 0,    // A3E, FN+ is the PTT
  kArdf,         // A2A, FN+ toggles the run
  kFrequency,    // +-20kHz
  kCallsign,     // 6 character editor
  kIdent,        // MOE..MO
  kWpm,          // CW speed, 5..40
  kTestDuty,     // fixed duty, carrier off - measure to set kModKCal
  kTestCarrier,  // carrier only, x = 0
  kTestTone,     // continuous A2A tone
  kTestBurst,    // tone for 5s at the start of every minute, silent otherwise
  kCount,
};

/// Owns the user interface: reads button events, drives the mode machine and
/// paints the 16x2 LCD.
class UiControllerTask final : public Task {
 public:
  static constexpr std::string_view kTaskName = "UiControllerTask";
  static constexpr int32_t kPriority = Task::PRIORITY_NORMAL;
  static constexpr int32_t kCoreId = PRO_CPU_NUM;

  UiControllerTask(ST7032* lcd, TransmitterSettings* settings,
                   TxController* tx, ArdfScheduler* ardf, Modulator* modulator,
                   Si5351A* si5351a, VoiceDsp* voice);

  void Initialize() override;
  void Update() override;

  MessageQueue<ButtonEvent>* GetButtonEventQueue() { return &event_queue_; }

  /// Latest battery reading, pushed in by the battery monitor.
  void SetBatteryVoltage(float volts) { battery_voltage_ = volts; }


  UiMode GetMode() const { return mode_; }

 private:
  void DispatchEvent(const ButtonEvent& event);
  void OnModePress();
  void OnFnPlusPress();
  void OnFnPlusRelease();
  void OnFnMinusPress();
  void OnFnPlusRepeat();
  void OnFnMinusRepeat();

  void EnterMode(UiMode mode);
  void LeaveMode(UiMode mode);
  bool IsModeSwitchAllowed() const;

  void AdjustFrequency(int32_t steps);
  void ApplyPendingFrequency();

  void CycleCallsignCursor();
  void IncrementCallsignChar();

  void AdjustIdent(int direction);
  void AdjustWpm(int direction);
  float CarrierLevelValue() const;
  void StopTestOutput();
  void UpdateBurst();
  bool SplashActive() const;
  void DrawSplash();

  void UpdateDisplay();
  void FormatLine1(char* out, size_t size);
  void FormatLine2(char* out, size_t size);

 private:
  MessageQueue<ButtonEvent> event_queue_;

  ST7032* lcd_;
  TransmitterSettings* settings_;
  TxController* tx_;
  ArdfScheduler* ardf_;
  Modulator* modulator_;
  Si5351A* si5351a_;
  VoiceDsp* voice_;

  UiMode mode_;

  // Frequency edits are batched: writing the Si5351A on every press would
  // choke the I2C bus, so the write happens 300ms after the last change
  // (spec 9.4.3).
  uint32_t pending_frequency_hz_;
  int64_t frequency_change_ms_;
  bool frequency_pending_;

  // Callsign editor state (spec 9.4.4)
  char callsign_[hardware_config::kCallLength + 1];
  uint8_t callsign_cursor_;

  // Test mode state
  uint32_t test_duty_;
  bool test_output_on_;
  // D:CARR level: 0 = L (half carrier), 1 = H (full carrier), 2 = P (peak,
  // 1.8x carrier, imitates a modulation crest). P switches itself off.
  uint8_t carrier_level_;
  int64_t carrier_on_ms_;

  // D:BRST state
  int64_t burst_start_ms_;
  int64_t burst_last_cycle_;
  bool burst_tx_active_;
  bool test_duty_active_;  // D:DUTY output is live (after the first +/-)

  float battery_voltage_;

  int64_t splash_start_ms_;
  int64_t splash_until_ms_;  // 0 when there is nothing to show

  // The idle screen alternates between the mode line and the battery line
  // every 5 seconds; edit modes hold their content instead (spec 9.5).
  int64_t display_alternate_ms_;
  bool display_show_battery_;
  int64_t last_display_ms_;
};

using UiControllerTaskUniquePtr = std::unique_ptr<UiControllerTask>;

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_UI_CONTROLLER_TASK_H_
