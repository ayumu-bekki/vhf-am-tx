#ifndef AM_TRANSMITTER_MAIN_UI_CONTROLLER_TASK_H_
#define AM_TRANSMITTER_MAIN_UI_CONTROLLER_TASK_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Modes, buttons and display (spec 9)

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "ardf_scheduler.h"
#include "button_event.h"
#include "cw_keyer.h"
#include "hardware_config.h"
#include "message_queue.h"
#include "modulator.h"
#include "si5351a.h"
#include "st7032.h"
#include "task.h"
#include "transmitter_settings.h"
#include "tx_controller.h"

namespace transmitter_system {

/// Operating modes (spec 9.1), in the order SW_MODE cycles through.
///
/// The test modes are appended to the same list and only reachable when
/// CONFIG_AM_TX_DEBUG_MODES is set. They are needed in the field - the duty
/// calibration of spec 11 is done against an oscilloscope, and the serial
/// console is unavailable because the power rails are split at the header -
/// so they have to be reachable from the buttons alone.
enum class UiMode : uint8_t {
  kVoice = 0,    // A3E, FN+ is the PTT
  kCw,           // A1A hand key: on the air while SW1 is held
  kArdf,         // A2A, FN+ toggles the run
  kFrequency,    // +-20kHz
  kPower,        // output stage 1.5W / 500mW / 100mW
  kCallsign,     // 6 character editor
  kIdent,        // MOE..MO
  kWpm,          // CW speed, 5..40
  kTestDuty,     // fixed duty, carrier off - measure to set kModKCal
  kTestCarrier,  // carrier only, x = 0
  kTestTone,     // continuous A2A tone
  kTestDot,      // continuous A1A dits, CLK0 switched as in the CW mode
  kTestBurst,    // tone for 5s at the start of every minute, silent otherwise
  kCount,
};

/// Owns the user interface: reads button events, drives the mode machine and
/// paints the 16x2 LCD.
class UiControllerTask final : public Task {
 public:
  static constexpr std::string_view kTaskName = "UiControllerTask";
  static constexpr UBaseType_t kPriority = Task::kPriorityNormal;
  static constexpr BaseType_t kCoreId = PRO_CPU_NUM;

  UiControllerTask(ST7032& lcd, TransmitterSettings& settings,
                   TxController& tx, ArdfScheduler& ardf, Modulator& modulator,
                   Si5351A& si5351a, CwKeyer& keyer);

  MessageQueue<ButtonEvent>* GetButtonEventQueue() { return &event_queue_; }

  /// Latest battery reading, pushed in by the main loop.
  void SetBatteryVoltage(float volts) {
    battery_voltage_.store(volts, std::memory_order_relaxed);
  }

 protected:
  void Initialize() override;
  void Update() override;

 private:
  // D:CARR output level, cycled with FN-.
  enum class CarrierLevel : uint8_t {
    kLow = 0,   // half carrier
    kHigh,      // full carrier
    kPeak,      // 1.8x carrier, imitates a modulation crest; times out
    kStage,     // carrier of the output stage chosen in the PWR mode
    kCount,
  };

  void DispatchEvent(const ButtonEvent& event);
  void OnModePress();
  void OnFnPlusPress();
  void OnFnPlusRelease();
  void OnFnMinusPress();
  void OnFnRepeat(ButtonEvent::Button button);

  void EnterMode(UiMode mode);
  void LeaveMode(UiMode mode);
  bool IsModeSwitchAllowed() const;

  void AdjustFrequency(int32_t steps);
  void ApplyPendingFrequency(bool force);

  void CycleCallsignCursor();
  void IncrementCallsignChar();

  void AdjustPower(int direction);
  void ApplyTxSettings();
  void UpdateArdfStopHold();
  void UpdateDot();
  static bool IsModeEnabled(UiMode mode);
  static UiMode NextMode(UiMode mode);

  void AdjustIdent(int direction);
  void AdjustWpm(int direction);
  void AdjustTestDuty(int direction);
  float CarrierLevelValue() const;
  void StopTestOutput();
  void UpdateBurst();
  void UpdateCarrierPeakTimeout();
  bool SplashActive() const;

  void UpdateDisplay();
  void FormatLine1(char* out, size_t size) const;
  void FormatLine2(char* out, size_t size) const;

 private:
  MessageQueue<ButtonEvent> event_queue_;

  ST7032& lcd_;
  TransmitterSettings& settings_;
  TxController& tx_;
  ArdfScheduler& ardf_;
  Modulator& modulator_;
  Si5351A& si5351a_;
  CwKeyer& keyer_;

  UiMode mode_;

  // Frequency edits are batched: writing the Si5351A on every press would
  // choke the I2C bus, so the write happens 300ms after the last change
  // (spec 9.4.3), or at once when the mode is left.
  uint32_t pending_frequency_hz_;
  int64_t frequency_change_ms_;
  bool frequency_pending_;

  // Callsign editor state (spec 9.4.4)
  char callsign_[hardware_config::kCallLength + 1];
  uint8_t callsign_cursor_;

  // ARDF: when FN+ went down while a run was in progress (0 = not counting).
  int64_t ardf_stop_press_ms_;

  // Test mode state
  uint32_t test_duty_;
  bool test_duty_active_;  // D:DUTY output is live (after the first +/-)
  bool test_output_on_;
  CarrierLevel carrier_level_;
  int64_t carrier_on_ms_;

  // D:BRST state
  int64_t burst_start_ms_;
  int64_t burst_last_cycle_;
  bool burst_tx_active_;

  std::atomic<float> battery_voltage_;

  int64_t splash_until_ms_;  // 0 when there is nothing to show
  int64_t last_display_ms_;

  // What is on the LCD right now, so unchanged rows are not rewritten. A row
  // whose write failed is marked stale and goes out again next refresh.
  char shown_[hardware_config::kLcdLines][hardware_config::kLcdColumns + 1];
  bool shown_valid_[hardware_config::kLcdLines];
  int parked_cursor_col_;  // -1 when the cursor is not parked
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_UI_CONTROLLER_TASK_H_
