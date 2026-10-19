// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Modes, buttons and display (spec 9)

#include "ui_controller_task.h"

#include <cstdio>
#include <cstring>

#include "logger.h"
#include "util.h"

namespace transmitter_system {

using util::NowMs;

namespace {

/// Callsign prefixes (spec 9.4.4): JA-JS, 7J-7N, 8J-8N. 29 in total.
const char* const kPrefixes[] = {
    "JA", "JB", "JC", "JD", "JE", "JF", "JG", "JH", "JI", "JJ",
    "JK", "JL", "JM", "JN", "JO", "JP", "JQ", "JR", "JS",
    "7J", "7K", "7L", "7M", "7N",
    "8J", "8K", "8L", "8M", "8N",
};
constexpr uint8_t kPrefixCount = sizeof(kPrefixes) / sizeof(kPrefixes[0]);

/// Cursor positions within the 6 character callsign.
/// P0 covers the two prefix characters, P1..P4 are one each.
constexpr uint8_t kCursorPositions = 5;
constexpr uint8_t kCursorColumn[kCursorPositions] = {0, 2, 3, 4, 5};
/// The editor line reads "CS:JL1HMT", so the LCD cursor sits 3 columns right
/// of the index into the callsign.
constexpr uint8_t kCallsignLabelCols = 3;
/// Shown in the editor for positions not entered yet while no callsign is
/// set. The first FN- on a position replaces it with the first valid value.
constexpr char kCallsignUnsetChar = '-';

/// Button queue poll timeout: bounds how late the timed work in Update()
/// (frequency write, display refresh, D:BRST) can run.
constexpr uint32_t kQueueTimeoutMs = 50;

/// Output stage on the PWR mode's top line ("H:1.5W" / "M:500mW" / "L:100mW")
/// and as the single letter at the right end of the top line in the other
/// modes.
const char* const kPowerLabel[hardware_config::kPowerCount] = {
    "H:1.5W", "M:500mW", "L:100mW"};
constexpr char kPowerLetter[hardware_config::kPowerCount] = {'H', 'M', 'L'};

/// D:DOT sends this over and over: 24 dits.
constexpr const char* kDotText = "EEEEEEEEEEEEEEEEEEEEEEEE";

/// Finds the prefix index for the first two characters, or -1 if unknown
/// (so the next press lands on the first prefix).
int FindPrefixIndex(const char* callsign) {
  for (uint8_t i = 0; i < kPrefixCount; ++i) {
    if (callsign[0] == kPrefixes[i][0] && callsign[1] == kPrefixes[i][1]) {
      return i;
    }
  }
  return -1;
}

bool IsTestMode(UiMode mode) {
  return mode == UiMode::kTestDuty || mode == UiMode::kTestCarrier ||
         mode == UiMode::kTestTone || mode == UiMode::kTestDot ||
         mode == UiMode::kTestBurst;
}

/// "M23A11" - ISR max and average in microseconds, with a '!' in front when
/// the max has broken the 20us budget of spec 7.1. The serial console is not
/// available in the field, so this is how the budget is checked on hardware.
void FormatIsrStats(const IsrStats& stats, char* out, size_t size) {
  const char flag = (stats.max_us > hardware_config::kIsrBudgetUs) ? '!' : ' ';
  std::snprintf(out, size, "%cM%2uA%2u", flag,
                static_cast<unsigned>(stats.max_us),
                static_cast<unsigned>(stats.avg_us));
}

}  // namespace

UiControllerTask::UiControllerTask(ST7032& lcd, TransmitterSettings& settings,
                                   TxController& tx, ArdfScheduler& ardf,
                                   Modulator& modulator, Si5351A& si5351a,
                                   CwKeyer& keyer)
    : Task(kTaskName, kPriority, kCoreId),
      lcd_(lcd),
      settings_(settings),
      tx_(tx),
      ardf_(ardf),
      modulator_(modulator),
      si5351a_(si5351a),
      keyer_(keyer),
      mode_(UiMode::kVoice),
      pending_frequency_hz_(hardware_config::kFreqDefaultHz),
      frequency_change_ms_(0),
      frequency_pending_(false),
      callsign_{},
      callsign_cursor_(0),
      ardf_stop_press_ms_(0),
      test_duty_(hardware_config::kDutyCarrier),
      test_duty_active_(false),
      test_output_on_(false),
      carrier_level_(CarrierLevel::kLow),
      carrier_on_ms_(0),
      burst_start_ms_(0),
      burst_last_cycle_(-1),
      burst_tx_active_(false),
      battery_voltage_(0.0f),
      splash_until_ms_(0),
      last_display_ms_(0),
      shown_{},
      shown_valid_{},
      parked_cursor_col_(-1) {
  event_queue_.Create(hardware_config::kButtonEventQueueSize);
}

void UiControllerTask::Initialize() {
  ESP_LOGI(kTag, "Start UiControllerTask");

  // The saved mode is not restored: the machine always comes up in VOICE,
  // idle (spec 9.6). Frequency, callsign, ID and WPM are still restored.
  mode_ = UiMode::kVoice;
  pending_frequency_hz_ = settings_.GetFrequencyHz();
  if (settings_.HasCallsign()) {
    std::memcpy(callsign_, settings_.GetCallsign(), sizeof(callsign_));
  } else {
    std::memset(callsign_, kCallsignUnsetChar, hardware_config::kCallLength);
    callsign_[hardware_config::kCallLength] = '\0';
  }

  // Program the synthesiser once at startup. The output stays disabled; the
  // transmit sequence decides when the carrier appears.
  si5351a_.SetFrequency(Si5351A::Clk::kClk0, pending_frequency_hz_);
  ApplyTxSettings();

  // Title screen for a moment; a button press dismisses it.
  splash_until_ms_ = NowMs() + hardware_config::kBootTitleMs;

  EnterMode(mode_);
  UpdateDisplay();
}

void UiControllerTask::Update() {
  ButtonEvent event = {};
  if (event_queue_.ReceiveWait(&event, kQueueTimeoutMs)) {
    DispatchEvent(event);
  }

  ApplyPendingFrequency(/*force=*/false);
  UpdateCarrierPeakTimeout();
  UpdateBurst();
  UpdateDot();
  UpdateArdfStopHold();
  settings_.Tick();

  const int64_t now = NowMs();
  if ((now - last_display_ms_) >=
      static_cast<int64_t>(hardware_config::kDisplayUpdateIntervalMs)) {
    last_display_ms_ = now;
    UpdateDisplay();
  }
}

// ---------------------------------------------------------------------------
// Button dispatch
// ---------------------------------------------------------------------------

void UiControllerTask::DispatchEvent(const ButtonEvent& event) {
  // While the start-up screen is up, the first press only dismisses
  // it; it must not also change the mode or start a transmission.
  if (SplashActive()) {
    if (event.type == ButtonEvent::Type::kPress) {
      splash_until_ms_ = 0;
      UpdateDisplay();
    }
    return;
  }

  switch (event.type) {
    case ButtonEvent::Type::kPress:
      switch (event.button) {
        case ButtonEvent::Button::kMode: OnModePress(); break;
        case ButtonEvent::Button::kFnPlus: OnFnPlusPress(); break;
        case ButtonEvent::Button::kFnMinus: OnFnMinusPress(); break;
      }
      break;

    case ButtonEvent::Type::kRelease:
      // Only the voice PTT cares about a release.
      if (event.button == ButtonEvent::Button::kFnPlus) {
        OnFnPlusRelease();
      }
      break;

    case ButtonEvent::Type::kRepeat:
      OnFnRepeat(event.button);
      break;
  }
}

/// Modes left out by menuconfig (ARDF, CW, test modes) are skipped by
/// SW_MODE. Without ARDF the ID and WPM modes go with it; the callsign editor
/// always stays.
bool UiControllerTask::IsModeEnabled(UiMode mode) {
  switch (mode) {
    case UiMode::kArdf:
    case UiMode::kIdent:
    case UiMode::kWpm:
      return hardware_config::kArdfModeEnabled;
    case UiMode::kCw:
      return hardware_config::kCwModeEnabled;
    case UiMode::kTestDuty:
    case UiMode::kTestCarrier:
    case UiMode::kTestTone:
    case UiMode::kTestDot:
    case UiMode::kTestBurst:
      return hardware_config::kDebugModesEnabled;
    default:
      return true;
  }
}

UiMode UiControllerTask::NextMode(UiMode mode) {
  uint8_t next = static_cast<uint8_t>(mode);
  for (uint8_t i = 0; i < static_cast<uint8_t>(UiMode::kCount); ++i) {
    next = (next + 1) % static_cast<uint8_t>(UiMode::kCount);
    if (IsModeEnabled(static_cast<UiMode>(next))) {
      return static_cast<UiMode>(next);
    }
  }
  return UiMode::kVoice;
}

bool UiControllerTask::IsModeSwitchAllowed() const {
  // Transmitting, or an ARDF run in progress even during a silent slot
  // (spec 9.3). The run flag is what matters, not whether the carrier
  // happens to be up right now.
  if (tx_.IsTransmitting() || tx_.IsArdfRunning()) {
    return false;
  }
  // Test modes that are actively driving the output count as transmitting.
  return !(IsTestMode(mode_) && test_output_on_);
}

void UiControllerTask::OnModePress() {
  if (!IsModeSwitchAllowed()) {
    ESP_LOGD(kTag, "Mode switch ignored while transmitting");
    return;
  }

  LeaveMode(mode_);

  mode_ = NextMode(mode_);

  EnterMode(mode_);

  // Leaving an edit mode saves what was edited; the mode itself is not stored.
  settings_.Tick(/*force=*/true);

  UpdateDisplay();
}

void UiControllerTask::OnFnPlusPress() {
  switch (mode_) {
    case UiMode::kVoice:
      // PTT: transmit only while held (spec 9.4.1).
      if (tx_.RequestStart()) {
        tx_.SetKeyed(true);
      }
      break;

    case UiMode::kArdf:
      if (ardf_.IsRunning()) {
        // Stopping takes a 1s hold; UpdateArdfStopHold() does it.
        ardf_stop_press_ms_ = NowMs();
      } else {
        ardf_.Start();
      }
      break;

    case UiMode::kCw:
      // Hand key: on the air for as long as SW1 is held, like the voice PTT.
      if (tx_.RequestStart()) {
        tx_.SetKeyed(true);
      }
      break;

    case UiMode::kFrequency:
      AdjustFrequency(+1);
      break;

    case UiMode::kPower:
      AdjustPower(-1);  // up: towards 1.5W (index 0)
      break;

    case UiMode::kCallsign:
      CycleCallsignCursor();
      break;

    case UiMode::kIdent:
      AdjustIdent(+1);
      break;

    case UiMode::kWpm:
      AdjustWpm(+1);
      break;

    case UiMode::kTestDuty:
      AdjustTestDuty(+1);
      break;

    case UiMode::kTestCarrier:
    case UiMode::kTestTone:
    case UiMode::kTestDot:
      // Toggle the carrier through the normal start/stop sequence, so the
      // ordering rules of spec 8.2 / 8.3 apply here too.
      if (test_output_on_) {
        StopTestOutput();
        break;
      }
      tx_.SetKeyedSource(mode_ == UiMode::kTestTone
                             ? ModulationSource::kContinuousTone
                         : mode_ == UiMode::kTestDot
                             ? ModulationSource::kCwKeyer
                             : ModulationSource::kSilent);
      ApplyTxSettings();
      if (tx_.RequestStart()) {
        test_output_on_ = true;
        carrier_on_ms_ = NowMs();
        if (mode_ == UiMode::kTestTone || mode_ == UiMode::kTestDot) {
          tx_.SetKeyed(true);
        }
      }
      break;

    case UiMode::kTestBurst:
      if (test_output_on_) {
        StopTestOutput();
      } else {
        burst_start_ms_ = NowMs();
        burst_last_cycle_ = -1;
        test_output_on_ = true;  // UpdateBurst() starts the first burst
        UpdateBurst();
      }
      break;

    case UiMode::kCount:
      break;
  }

  UpdateDisplay();
}

void UiControllerTask::OnFnPlusRelease() {
  ardf_stop_press_ms_ = 0;  // released before the hold time: nothing happens
  if (mode_ == UiMode::kVoice || mode_ == UiMode::kCw) {
    tx_.SetKeyed(false);
    tx_.RequestStop();
    UpdateDisplay();
  }
}

void UiControllerTask::OnFnMinusPress() {
  switch (mode_) {
    case UiMode::kCw:
      ToggleEmission();  // SW2: A2A <-> A1A, ignored while running
      break;

    case UiMode::kFrequency:
      AdjustFrequency(-1);
      break;

    case UiMode::kPower:
      AdjustPower(+1);  // down: towards 100mW
      break;

    case UiMode::kCallsign:
      IncrementCallsignChar();
      break;

    case UiMode::kIdent:
      AdjustIdent(-1);
      break;

    case UiMode::kWpm:
      AdjustWpm(-1);
      break;

    case UiMode::kTestDuty:
      AdjustTestDuty(-1);
      break;

    case UiMode::kTestCarrier:
      // The level is only changeable while the carrier is off, which
      // includes the ramp-down after OFF.
      if (!test_output_on_ && !tx_.IsTransmitting()) {
        carrier_level_ = static_cast<CarrierLevel>(
            (static_cast<uint8_t>(carrier_level_) + 1) %
            static_cast<uint8_t>(CarrierLevel::kCount));  // L -> H -> P -> S -> L
        modulator_.SetCarrierLevel(CarrierLevelValue());
        ApplyTxSettings();
      }
      break;

    default:
      break;  // unused in the other modes
  }

  UpdateDisplay();
}

void UiControllerTask::OnFnRepeat(ButtonEvent::Button button) {
  // Auto-repeat only applies to the value-adjusting modes; the PTT and the
  // ARDF run toggle are single shot (spec 9.7).
  switch (mode_) {
    case UiMode::kFrequency:
    case UiMode::kPower:
    case UiMode::kCallsign:
    case UiMode::kIdent:
    case UiMode::kWpm:
    case UiMode::kTestDuty:
      if (button == ButtonEvent::Button::kFnPlus) {
        OnFnPlusPress();
      } else if (button == ButtonEvent::Button::kFnMinus) {
        OnFnMinusPress();
      }
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Mode entry / exit
// ---------------------------------------------------------------------------

void UiControllerTask::EnterMode(UiMode mode) {
  switch (mode) {
    case UiMode::kVoice:
      tx_.SetKeyedSource(ModulationSource::kVoice);
      break;

    case UiMode::kArdf:
      tx_.SetKeyedSource(ModulationSource::kCwKeyer);
      break;

    case UiMode::kCw:
      tx_.SetKeyedSource(ModulationSource::kCwKeyer);
      break;

    case UiMode::kCallsign:
      callsign_cursor_ = 0;
      lcd_.SetCursorVisible(true, true);
      break;

    case UiMode::kTestDuty:
      // Calibration mode: the carrier stays off and a fixed duty goes to the
      // modulator so its output can be trimmed to 3.6V (spec 12 / 11).
      // Starts OFF (duty 0); the first +/- press switches it ON at the
      // current value without changing it.
      test_output_on_ = false;
      test_duty_active_ = false;
      modulator_.SetFixedDuty(0);
      modulator_.SetSource(ModulationSource::kFixedDuty);
      break;

    case UiMode::kTestCarrier:
      // Always starts at L (half carrier), the safe side.
      carrier_level_ = CarrierLevel::kLow;
      modulator_.SetCarrierLevel(CarrierLevelValue());
      test_output_on_ = false;
      break;

    case UiMode::kTestTone:
    case UiMode::kTestDot:
    case UiMode::kTestBurst:
      test_output_on_ = false;
      burst_tx_active_ = false;
      break;

    default:
      break;
  }
  ApplyTxSettings();
}

void UiControllerTask::LeaveMode(UiMode mode) {
  switch (mode) {
    case UiMode::kFrequency:
      // Write a retune that is still inside its hold-off now. Applied later
      // it could land while another mode is transmitting.
      ApplyPendingFrequency(/*force=*/true);
      break;

    case UiMode::kCallsign:
      lcd_.SetCursorVisible(false, false);
      // Only a complete callsign is saved. One still showing '-' positions
      // stays in the editor for next time and nothing is stored.
      if (TransmitterSettings::IsValidCallsign(callsign_)) {
        settings_.SetCallsign(callsign_);
        ardf_.SetCallsign(callsign_);
      } else {
        ESP_LOGW(kTag, "Callsign %s incomplete: not saved", callsign_);
      }
      break;

    case UiMode::kTestDuty:
      // Nothing is stored. The carrier duty comes from the circuit constants
      // (hardware_config.h); a measurement made here goes into kModKCal and a
      // rebuild.
      ESP_LOGI(kTag, "D:DUTY left at %u (carrier is %u)",
               static_cast<unsigned>(test_duty_),
               static_cast<unsigned>(hardware_config::kDutyCarrier));
      modulator_.SetFixedDuty(0);
      modulator_.SetSource(ModulationSource::kSilent);
      break;

    case UiMode::kTestCarrier:
    case UiMode::kTestTone:
    case UiMode::kTestDot:
    case UiMode::kTestBurst:
      if (mode == UiMode::kTestDot) {
        keyer_.Stop();
      }
      if (tx_.IsTransmitting()) {
        tx_.RequestStop();
      }
      test_output_on_ = false;
      burst_tx_active_ = false;
      modulator_.SetCarrierLevel(1.0f);
      break;

    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Edit modes
// ---------------------------------------------------------------------------

void UiControllerTask::AdjustFrequency(int32_t steps) {
  int64_t next = static_cast<int64_t>(pending_frequency_hz_) +
                 static_cast<int64_t>(steps) * hardware_config::kFreqStepHz;

  // Clamp rather than wrap, so a long press cannot walk off the end of the
  // band and round onto something out of plan (spec 9.4.3).
  if (next < hardware_config::kFreqMinHz) {
    next = hardware_config::kFreqMinHz;
  }
  if (next > hardware_config::kFreqMaxHz) {
    next = hardware_config::kFreqMaxHz;
  }

  const uint32_t clamped = static_cast<uint32_t>(next);
  if (clamped == pending_frequency_hz_) {
    return;
  }

  pending_frequency_hz_ = clamped;
  frequency_change_ms_ = NowMs();
  frequency_pending_ = true;
}

void UiControllerTask::ApplyPendingFrequency(bool force) {
  if (!frequency_pending_) {
    return;
  }
  if (!force && (NowMs() - frequency_change_ms_) <
                    static_cast<int64_t>(hardware_config::kFreqWriteDelayMs)) {
    return;
  }
  // Retuning disables the output with no ramp-down, which is exactly what
  // spec 8.3 forbids while the drain is driven. Frequency changes are only
  // possible in FREQ mode, which cannot transmit, and leaving it flushes
  // them, so this is a guard rather than a path that is expected to run.
  if (tx_.IsTransmitting()) {
    ESP_LOGW(kTag, "Retune deferred: transmitting");
    return;
  }

  frequency_pending_ = false;
  si5351a_.SetFrequency(Si5351A::Clk::kClk0, pending_frequency_hz_);
  settings_.SetFrequencyHz(pending_frequency_hz_);
}

void UiControllerTask::CycleCallsignCursor() {
  callsign_cursor_ = (callsign_cursor_ + 1) % kCursorPositions;
}

void UiControllerTask::IncrementCallsignChar() {
  switch (callsign_cursor_) {
    case 0: {
      // Prefix: 29 options, both characters move together.
      const int index = (FindPrefixIndex(callsign_) + 1) % kPrefixCount;
      callsign_[0] = kPrefixes[index][0];
      callsign_[1] = kPrefixes[index][1];
      break;
    }
    case 1:
      // Region digit 0-9
      callsign_[2] = (callsign_[2] >= '0' && callsign_[2] < '9')
                         ? static_cast<char>(callsign_[2] + 1)
                         : '0';
      break;
    default: {
      // Suffix letters A-Z, three of them, fixed length (spec 9.4.4).
      const uint8_t pos = kCursorColumn[callsign_cursor_];
      callsign_[pos] = (callsign_[pos] >= 'A' && callsign_[pos] < 'Z')
                           ? static_cast<char>(callsign_[pos] + 1)
                           : 'A';
      break;
    }
  }
}

void UiControllerTask::AdjustPower(int direction) {
  // Clamp rather than wrap: a long press stops at 1.5W / 100mW (spec 9.2).
  const int next = static_cast<int>(settings_.GetPower()) + direction;
  if (next < 0 || next >= hardware_config::kPowerCount) {
    return;
  }
  settings_.SetPower(static_cast<uint8_t>(next));
  ApplyTxSettings();
}

void UiControllerTask::ToggleEmission() {
  Emission emission = settings_.GetEmission();
  if (TryToggleEmission(&emission, tx_.IsTransmitting())) {
    settings_.SetEmission(emission);
    ApplyTxSettings();
  }
}

/// Hands the stage and the emission of the current mode to the TX controller,
/// which latches them when the next transmission starts. ARDF and everything
/// but the CW mode are A2A; D:DOT is A1A; the D:CARR levels other than S run
/// at 1.5W whatever the stage is (calibration reference).
void UiControllerTask::ApplyTxSettings() {
  Emission emission = Emission::kA2a;
  if (mode_ == UiMode::kCw) {
    emission = settings_.GetEmission();
  } else if (mode_ == UiMode::kTestDot) {
    emission = Emission::kA1a;
  }
  tx_.SetEmission(emission);
  tx_.SetManualKeying(mode_ == UiMode::kCw);

  const bool fixed_full =
      mode_ == UiMode::kTestCarrier && carrier_level_ != CarrierLevel::kStage;
  tx_.SetPowerIndex(fixed_full ? 0 : settings_.GetPower());
}

void UiControllerTask::AdjustIdent(int direction) {
  const int count = static_cast<int>(morse::ArdfId::kCount);
  const int index =
      (static_cast<int>(settings_.GetArdfId()) + count + direction) % count;

  const morse::ArdfId id = static_cast<morse::ArdfId>(index);
  settings_.SetArdfId(id);
  ardf_.SetArdfId(id);
}

void UiControllerTask::AdjustWpm(int direction) {
  // Clamp rather than wrap: a long press should stop at the end of the range,
  // not jump from 40 back to 5. SetCwWpm() clamps.
  const int wpm = static_cast<int>(settings_.GetCwWpm()) + direction;
  if (wpm < hardware_config::kCwWpmMin || wpm > hardware_config::kCwWpmMax) {
    return;
  }
  settings_.SetCwWpm(static_cast<uint8_t>(wpm));
  ardf_.SetWpm(static_cast<uint8_t>(wpm));
}

// ---------------------------------------------------------------------------
// Test modes
// ---------------------------------------------------------------------------

void UiControllerTask::AdjustTestDuty(int direction) {
  // The first press only switches the output on at the current value.
  if (test_duty_active_) {
    if (direction > 0 && test_duty_ < hardware_config::kDutyLimit) {
      ++test_duty_;
    } else if (direction < 0 && test_duty_ > 0) {
      --test_duty_;
    }
  }
  test_duty_active_ = true;
  modulator_.SetFixedDuty(test_duty_);
}

float UiControllerTask::CarrierLevelValue() const {
  switch (carrier_level_) {
    case CarrierLevel::kHigh:
    case CarrierLevel::kStage: return 1.0f;
    case CarrierLevel::kPeak: return hardware_config::kCarrierPeakFactor;
    default: return 0.5f;
  }
}

void UiControllerTask::StopTestOutput() {
  if (mode_ == UiMode::kTestDot) {
    keyer_.Stop();
  }
  tx_.SetKeyed(false);
  tx_.RequestStop();
  test_output_on_ = false;
  burst_tx_active_ = false;
}

// P is a heavy load on the final stage: it ends itself after a few seconds,
// through the normal ramp-down.
void UiControllerTask::UpdateCarrierPeakTimeout() {
  if (mode_ == UiMode::kTestCarrier && test_output_on_ &&
      carrier_level_ == CarrierLevel::kPeak &&
      (NowMs() - carrier_on_ms_) >= hardware_config::kCarrierPeakMaxMs) {
    StopTestOutput();
    UpdateDisplay();
  }
}

// ARDF: FN+ held for kArdfStopHoldMs while a run is in progress ends it.
void UiControllerTask::UpdateArdfStopHold() {
  if (ardf_stop_press_ms_ == 0) {
    return;
  }
  if (mode_ != UiMode::kArdf || !ardf_.IsRunning()) {
    ardf_stop_press_ms_ = 0;
    return;
  }
  if ((NowMs() - ardf_stop_press_ms_) >=
      static_cast<int64_t>(hardware_config::kArdfStopHoldMs)) {
    ardf_stop_press_ms_ = 0;
    ardf_.Stop();
    UpdateDisplay();
  }
}

// D:DOT: keep the keyer fed with dits while the carrier is up.
void UiControllerTask::UpdateDot() {
  if (mode_ != UiMode::kTestDot || !test_output_on_) {
    return;
  }
  const TxState state = tx_.GetState();
  if ((state == TxState::kTxReady || state == TxState::kTxKeyed) &&
      !keyer_.IsBusy()) {
    keyer_.Send(kDotText);
  }
}

// D:BRST: tone for kBurstOnMs at the start of every kBurstPeriodMs, nothing
// (carrier included) for the rest. Each burst goes through the normal
// start/stop sequence. A refused start (battery) is tried once per cycle.
void UiControllerTask::UpdateBurst() {
  if (!test_output_on_ || mode_ != UiMode::kTestBurst) {
    return;
  }
  const int64_t elapsed = NowMs() - burst_start_ms_;
  const int64_t cycle = elapsed / hardware_config::kBurstPeriodMs;
  const bool in_window =
      (elapsed % hardware_config::kBurstPeriodMs) < hardware_config::kBurstOnMs;

  if (in_window && !burst_tx_active_ && cycle != burst_last_cycle_) {
    burst_last_cycle_ = cycle;
    tx_.SetKeyedSource(ModulationSource::kContinuousTone);
    if (tx_.RequestStart()) {
      tx_.SetKeyed(true);
      burst_tx_active_ = true;
    }
  } else if (!in_window && burst_tx_active_) {
    tx_.SetKeyed(false);
    tx_.RequestStop();
    burst_tx_active_ = false;
  }
}

// ---------------------------------------------------------------------------
// Display (spec 9.5)
// ---------------------------------------------------------------------------

void UiControllerTask::FormatLine1(char* out, size_t size) const {
  if (tx_.GetState() == TxState::kFault) {
    std::snprintf(out, size, "FAULT");
    return;
  }

  // The battery inhibit pins the top line (spec 9.5). Shown on the same
  // one second average that gates transmission, so it cannot flicker on
  // every key down.
  if (tx_.IsInhibited()) {
    std::snprintf(out, size, "LOW BATT");
    return;
  }

  const char* const on_off = test_output_on_ ? "ON " : "OFF";

  switch (mode_) {
    case UiMode::kVoice:
      std::snprintf(out, size, "VOICE%s", tx_.IsTransmitting() ? " TX" : "");
      break;

    case UiMode::kCw: {
      const char* const emission =
          settings_.GetEmission() == Emission::kA1a ? "A1A" : "A2A";
      if (tx_.IsTransmitting()) {
        std::snprintf(out, size, "CW %s TX", emission);
      } else {
        std::snprintf(out, size, "CW %s", emission);
      }
      break;
    }

    case UiMode::kArdf: {
      const char* id_text = morse::GetArdfIdText(settings_.GetArdfId());
      if (!ardf_.HasCallsign()) {
        // FN+ is refused until a callsign is set (spec 10.5).
        std::snprintf(out, size, "ARDF NO CALL");
      } else if (!ardf_.IsRunning()) {
        std::snprintf(out, size, "ARDF %-3s", id_text);
      } else if (tx_.IsTransmitting()) {
        std::snprintf(out, size, "ARDF %-3s TX", id_text);
      } else {
        std::snprintf(out, size, "ARDF %-3s %3us", id_text,
                      static_cast<unsigned>(ardf_.GetSecondsToNextSlot()));
      }
      break;
    }

    case UiMode::kFrequency:
      std::snprintf(out, size, "FREQ");
      break;

    case UiMode::kPower:
      std::snprintf(out, size, "%s", kPowerLabel[settings_.GetPower()]);
      break;

    case UiMode::kCallsign:
      std::snprintf(out, size, "CS:%s", callsign_);
      break;

    case UiMode::kIdent:
      std::snprintf(out, size, "ARDF ID:%s",
                    morse::GetArdfIdText(settings_.GetArdfId()));
      break;

    case UiMode::kWpm:
      std::snprintf(out, size, "ARDF WPM:%u",
                    static_cast<unsigned>(settings_.GetCwWpm()));
      break;

    case UiMode::kTestDuty:
      std::snprintf(out, size, "D:DUTY %3u %s",
                    static_cast<unsigned>(test_duty_),
                    test_duty_active_ ? "ON " : "OFF");
      break;

    case UiMode::kTestCarrier:
      std::snprintf(out, size, "D:CARR %c %s",
                    "LHPS"[static_cast<uint8_t>(carrier_level_)], on_off);
      break;

    case UiMode::kTestTone:
      std::snprintf(out, size, "D:TONE %s", on_off);
      break;

    case UiMode::kTestDot:
      std::snprintf(out, size, "D:DOT %s", on_off);
      break;

    case UiMode::kTestBurst: {
      const int64_t phase_s =
          test_output_on_
              ? ((NowMs() - burst_start_ms_) % hardware_config::kBurstPeriodMs) /
                    1000
              : 0;
      std::snprintf(out, size, "D:BRST %s%2ds %s", on_off,
                    static_cast<int>(phase_s), burst_tx_active_ ? "TX" : "--");
      break;
    }

    case UiMode::kCount:
      std::snprintf(out, size, "?");
      break;
  }
}

void UiControllerTask::FormatLine2(char* out, size_t size) const {
  // The modes that run the expensive part of the modulation path show the
  // ISR statistics instead of the frequency: D:TONE / D:BRST always, VOICE
  // while keyed (the A3E chain can only be measured while it runs, which is
  // exactly while the PTT is held).
  const bool show_isr_stats =
      mode_ == UiMode::kTestTone || mode_ == UiMode::kTestDot ||
      mode_ == UiMode::kTestBurst ||
      (mode_ == UiMode::kVoice && tx_.GetState() == TxState::kTxKeyed);
  if (show_isr_stats) {
    FormatIsrStats(modulator_.GetStats(), out, size);
    return;
  }

  // Status character right after the frequency (spec 9.5).
  char status = ' ';
  const float battery = battery_voltage_.load(std::memory_order_relaxed);
  if (tx_.GetState() == TxState::kFault) {
    status = 'X';
  } else if (tx_.IsTransmitting()) {
    status = '*';
  } else if (battery > 0.0f && battery < hardware_config::kBatWarnV) {
    status = '!';
  }

  // "145.70MHz*": two decimals are enough (the step is 20kHz).
  const unsigned mhz = pending_frequency_hz_ / 1000000;
  const unsigned khz10 = (pending_frequency_hz_ % 1000000) / 10000;
  std::snprintf(out, size, "%3u.%02uMHz%c", mhz, khz10, status);
}

bool UiControllerTask::SplashActive() const {
  return splash_until_ms_ != 0 && NowMs() < splash_until_ms_;
}

void UiControllerTask::UpdateDisplay() {
  if (!lcd_.IsReady()) {
    return;
  }

  constexpr int kCols = hardware_config::kLcdColumns;
  char line1[kCols + 1] = {};
  char line2[kCols + 1] = {};
  const bool splash = SplashActive();

  if (splash) {
    std::snprintf(line1, sizeof(line1), "%s", hardware_config::kBootTitle);
    // Blank when no callsign has been saved.
    std::snprintf(line2, sizeof(line2), "%s", settings_.GetCallsign());
  } else {
    FormatLine1(line1, sizeof(line1));
    FormatLine2(line2, sizeof(line2));

    // Output stage letter at the right end of the top line, in the modes that
    // transmit with it (VOICE, CW, ARDF). Not on FAULT.
    if ((mode_ == UiMode::kVoice || mode_ == UiMode::kCw ||
         mode_ == UiMode::kArdf) &&
        tx_.GetState() != TxState::kFault) {
      const size_t len = std::strlen(line1);
      if (len < kCols - 1) {
        for (size_t i = len; i < kCols - 1; ++i) line1[i] = ' ';
        line1[kCols - 1] = kPowerLetter[settings_.GetPower()];
        line1[kCols] = '\0';
      }
    }

    // Battery voltage in the bottom right corner ("12.8V"). It is laid over
    // whatever the second line holds, ISR statistics included.
    const float battery = battery_voltage_.load(std::memory_order_relaxed);
    if (battery > 0.0f) {
      constexpr size_t kVoltCols = 5;
      char volt[kVoltCols + 1];
      std::snprintf(volt, sizeof(volt), "%4.1fV", static_cast<double>(battery));
      const size_t start = kCols - kVoltCols;
      for (size_t i = std::strlen(line2); i < start; ++i) line2[i] = ' ';
      std::memcpy(&line2[start], volt, kVoltCols);
      line2[kCols] = '\0';
    }
  }

  // Each row goes out as one I2C transaction, and only when it changed:
  // the bus is shared with the Si5351A, and most refreshes change nothing.
  const char* const lines[hardware_config::kLcdLines] = {line1, line2};
  bool wrote = false;
  for (uint8_t row = 0; row < hardware_config::kLcdLines; ++row) {
    char padded[kCols + 1];
    std::snprintf(padded, sizeof(padded), "%-*.*s", kCols, kCols, lines[row]);
    if (shown_valid_[row] && std::strcmp(padded, shown_[row]) == 0) {
      continue;
    }
    shown_valid_[row] = lcd_.WriteAt(0, row, padded, kCols);
    std::memcpy(shown_[row], padded, sizeof(padded));
    wrote = true;
  }

  // Park the hardware cursor on the character being edited so the operator
  // can see which position FN- will change. Writing a row moves the address,
  // so it is re-parked after any write.
  const int cursor_col =
      (mode_ == UiMode::kCallsign && !splash)
          ? kCallsignLabelCols + kCursorColumn[callsign_cursor_]
          : -1;
  if (cursor_col >= 0 && (wrote || cursor_col != parked_cursor_col_)) {
    lcd_.SetCursor(static_cast<uint8_t>(cursor_col), 0);
  }
  parked_cursor_col_ = cursor_col;
}

}  // namespace transmitter_system

// EOF
