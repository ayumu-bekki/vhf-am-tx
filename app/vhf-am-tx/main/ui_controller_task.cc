// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Modes, buttons and display (spec 9)

#include "ui_controller_task.h"

#include <esp_timer.h>

#include <cstdio>
#include <cstring>

#include "logger.h"
#include "util.h"

namespace transmitter_system {

namespace {

int64_t NowMs() { return esp_timer_get_time() / 1000; }

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

/// Finds the prefix index for the first two characters, or 0 if unknown.
uint8_t FindPrefixIndex(const char* callsign) {
  for (uint8_t i = 0; i < kPrefixCount; ++i) {
    if (callsign[0] == kPrefixes[i][0] && callsign[1] == kPrefixes[i][1]) {
      return i;
    }
  }
  return 0;
}

const char* ModeShortName(UiMode mode) {
  switch (mode) {
    case UiMode::kVoice: return "VOICE";
    case UiMode::kArdf: return "ARDF";
    case UiMode::kFrequency: return "FREQ";
    case UiMode::kCallsign: return "CALL";
    case UiMode::kIdent: return "ID";
    case UiMode::kWpm: return "WPM";
    case UiMode::kTestDuty: return "D:DUTY";
    case UiMode::kTestCarrier: return "D:CARR";
    case UiMode::kTestTone: return "D:TONE";
    case UiMode::kTestBurst: return "D:BRST";
    case UiMode::kCount: break;
  }
  return "?";
}

bool IsTestMode(UiMode mode) {
  return mode == UiMode::kTestDuty || mode == UiMode::kTestCarrier ||
         mode == UiMode::kTestTone || mode == UiMode::kTestBurst;
}

constexpr int kQueueTimeoutMs = 50;


}  // namespace

UiControllerTask::UiControllerTask(ST7032* lcd, TransmitterSettings* settings,
                                   TxController* tx, ArdfScheduler* ardf,
                                   Modulator* modulator, Si5351A* si5351a,
                                   VoiceDsp* voice)
    : Task(std::string(kTaskName), kPriority, kCoreId),
      event_queue_(),
      lcd_(lcd),
      settings_(settings),
      tx_(tx),
      ardf_(ardf),
      modulator_(modulator),
      si5351a_(si5351a),
      voice_(voice),
      mode_(UiMode::kVoice),
      pending_frequency_hz_(hardware_config::kFreqDefaultHz),
      frequency_change_ms_(0),
      frequency_pending_(false),
      callsign_{},
      callsign_cursor_(0),
      test_duty_(hardware_config::kDutyCarrier),
      test_output_on_(false),
      carrier_level_(0),
      carrier_on_ms_(0),
      burst_start_ms_(0),
      burst_last_cycle_(-1),
      burst_tx_active_(false),
      test_duty_active_(false),
      battery_voltage_(0.0f),
      splash_start_ms_(0),
      splash_until_ms_(0),
      display_alternate_ms_(0),
      display_show_battery_(false),
      last_display_ms_(0) {
  event_queue_.Create(hardware_config::kButtonEventQueueSize);
}

void UiControllerTask::Initialize() {
  ESP_LOGI(kTag, "Start UiControllerTask");

  if (settings_) {
    // The saved mode is not restored: the machine always comes up in VOICE,
    // idle (spec 9.6). Frequency, callsign, ID and WPM are still restored.
    mode_ = UiMode::kVoice;

    pending_frequency_hz_ = settings_->GetFrequencyHz();
    std::strncpy(callsign_, settings_->GetCallsign(), sizeof(callsign_) - 1);
    callsign_[sizeof(callsign_) - 1] = '\0';
    if (ardf_) {
      ardf_->SetArdfId(settings_->GetArdfId());
      ardf_->SetCallsign(callsign_);
    }
  }

  // Program the synthesiser once at startup. The output stays disabled; the
  // transmit sequence decides when the carrier appears.
  if (si5351a_) {
    si5351a_->SetFrequency(Si5351A::Clk::kClk0, pending_frequency_hz_);
  }

  // Title screen for a moment; a button press dismisses it.
  splash_start_ms_ = NowMs();
  splash_until_ms_ = splash_start_ms_ + hardware_config::kBootTitleMs;

  EnterMode(mode_);
  UpdateDisplay();
}

void UiControllerTask::Update() {
  ButtonEvent event = {};

  while (true) {
    if (event_queue_.ReceiveWait(&event, kQueueTimeoutMs)) {
      DispatchEvent(event);
    }

    ApplyPendingFrequency();

    // P is a heavy load on the final stage: it ends itself after a few
    // seconds, through the normal ramp-down.
    if (mode_ == UiMode::kTestCarrier && test_output_on_ &&
        carrier_level_ == 2 &&
        (NowMs() - carrier_on_ms_) >= hardware_config::kCarrierPeakMaxMs) {
      StopTestOutput();
      UpdateDisplay();
    }

    UpdateBurst();

    if (settings_) {
      settings_->Tick();
    }

    const int64_t now = NowMs();
    if ((now - display_alternate_ms_) >=
        static_cast<int64_t>(hardware_config::kDisplayAlternateMs)) {
      display_alternate_ms_ = now;
      display_show_battery_ = !display_show_battery_;
    }

    if ((now - last_display_ms_) >=
        static_cast<int64_t>(hardware_config::kDisplayUpdateIntervalMs)) {
      last_display_ms_ = now;
      UpdateDisplay();
    }
  }
}

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

  switch (event.button) {
    case ButtonEvent::Button::kMode:
      if (event.type == ButtonEvent::Type::kPress) {
        OnModePress();
      }
      break;

    case ButtonEvent::Button::kFnPlus:
      if (event.type == ButtonEvent::Type::kPress) {
        OnFnPlusPress();
      } else if (event.type == ButtonEvent::Type::kRelease) {
        OnFnPlusRelease();
      } else if (event.type == ButtonEvent::Type::kRepeat) {
        OnFnPlusRepeat();
      }
      break;

    case ButtonEvent::Button::kFnMinus:
      if (event.type == ButtonEvent::Type::kPress) {
        OnFnMinusPress();
      } else if (event.type == ButtonEvent::Type::kRepeat) {
        OnFnMinusRepeat();
      }
      break;
  }
}

bool UiControllerTask::IsModeSwitchAllowed() const {
  if (!tx_) {
    return true;
  }
  // Transmitting, or an ARDF run in progress even during a silent slot
  // (spec 9.3). The run flag is what matters, not whether the carrier
  // happens to be up right now.
  if (tx_->IsTransmitting() || tx_->IsArdfRunning()) {
    return false;
  }
  // Test modes that are actively driving the output count as transmitting.
  if (IsTestMode(mode_) && test_output_on_) {
    return false;
  }
  return true;
}

void UiControllerTask::OnModePress() {
  if (!IsModeSwitchAllowed()) {
    ESP_LOGD(kTag, "Mode switch ignored while transmitting");
    return;
  }

  LeaveMode(mode_);

  uint8_t next =
      (static_cast<uint8_t>(mode_) + 1) % static_cast<uint8_t>(UiMode::kCount);
  // The test modes are a debug feature: without the menuconfig option the
  // cycle wraps straight back to the first mode.
  if (!hardware_config::kDebugModesEnabled &&
      next >= static_cast<uint8_t>(UiMode::kTestDuty)) {
    next = 0;
  }
  mode_ = static_cast<UiMode>(next);

  EnterMode(mode_);

  if (settings_) {
    // Leaving an edit mode saves what was edited; the mode itself is not stored.
    settings_->Tick(/*force=*/true);
  }

  UpdateDisplay();
}

void UiControllerTask::EnterMode(UiMode mode) {
  switch (mode) {
    case UiMode::kVoice:
      if (tx_) {
        tx_->SetKeyedSource(ModulationSource::kVoice);
      }
      break;

    case UiMode::kArdf:
      if (tx_) {
        tx_->SetKeyedSource(ModulationSource::kCwKeyer);
      }
      break;

    case UiMode::kCallsign:
      callsign_cursor_ = 0;
      if (lcd_) {
        lcd_->SetCursorVisible(true, true);
      }
      break;

    case UiMode::kTestDuty:
      // Calibration mode: the carrier stays off and a fixed duty goes to the
      // modulator so its output can be trimmed to 3.6V (spec 12 / 11).
      // Starts OFF (duty 0); the first +/- press switches it ON at the
      // current value without changing it.
      test_output_on_ = false;
      test_duty_active_ = false;
      if (modulator_) {
        modulator_->SetFixedDuty(0);
        modulator_->SetSource(ModulationSource::kFixedDuty);
      }
      break;

    case UiMode::kTestCarrier:
      // Always starts at L (half carrier), the safe side.
      carrier_level_ = 0;
      if (modulator_) {
        modulator_->SetCarrierLevel(0.5f);
      }
      test_output_on_ = false;
      break;

    case UiMode::kTestTone:
    case UiMode::kTestBurst:
      test_output_on_ = false;
      burst_tx_active_ = false;
      break;

    default:
      break;
  }
}

float UiControllerTask::CarrierLevelValue() const {
  switch (carrier_level_) {
    case 1: return 1.0f;
    case 2: return hardware_config::kCarrierPeakFactor;
    default: return 0.5f;
  }
}

void UiControllerTask::StopTestOutput() {
  if (tx_) {
    tx_->SetKeyed(false);
    tx_->RequestStop();
  }
  test_output_on_ = false;
  burst_tx_active_ = false;
}

// D:BRST: tone for kBurstOnMs at the start of every kBurstPeriodMs, nothing
// (carrier included) for the rest. Each burst goes through the normal
// start/stop sequence. A refused start (battery) is tried once per cycle.
void UiControllerTask::UpdateBurst() {
  if (!tx_ || !test_output_on_ || mode_ != UiMode::kTestBurst) {
    return;
  }
  const int64_t elapsed = NowMs() - burst_start_ms_;
  const int64_t cycle = elapsed / hardware_config::kBurstPeriodMs;
  const bool in_window =
      (elapsed % hardware_config::kBurstPeriodMs) < hardware_config::kBurstOnMs;

  if (in_window && !burst_tx_active_ && cycle != burst_last_cycle_) {
    burst_last_cycle_ = cycle;
    tx_->SetKeyedSource(ModulationSource::kContinuousTone);
    if (tx_->RequestStart()) {
      tx_->SetKeyed(true);
      burst_tx_active_ = true;
    }
  } else if (!in_window && burst_tx_active_) {
    tx_->SetKeyed(false);
    tx_->RequestStop();
    burst_tx_active_ = false;
  }
}

void UiControllerTask::LeaveMode(UiMode mode) {
  switch (mode) {
    case UiMode::kCallsign:
      if (lcd_) {
        lcd_->SetCursorVisible(false, false);
      }
      if (settings_) {
        settings_->SetCallsign(callsign_);
      }
      if (ardf_) {
        ardf_->SetCallsign(callsign_);
      }
      break;

    case UiMode::kTestDuty:
      // Nothing is stored. The carrier duty comes from the circuit constants
      // (hardware_config.h); a measurement made here goes into kModKCal and a
      // rebuild.
      ESP_LOGI(kTag, "D:DUTY left at %u (carrier is %u)", test_duty_,
               static_cast<unsigned>(hardware_config::kDutyCarrier));
      if (modulator_) {
        modulator_->SetFixedDuty(0);
        modulator_->SetSource(ModulationSource::kSilent);
      }
      break;

    case UiMode::kTestCarrier:
    case UiMode::kTestTone:
    case UiMode::kTestBurst:
      if (tx_ && tx_->IsTransmitting()) {
        tx_->RequestStop();
      }
      test_output_on_ = false;
      burst_tx_active_ = false;
      if (modulator_) {
        modulator_->SetCarrierLevel(1.0f);
      }
      break;

    default:
      break;
  }
}

void UiControllerTask::OnFnPlusPress() {
  switch (mode_) {
    case UiMode::kVoice:
      // PTT: transmit only while held (spec 9.4.1).
      if (tx_ && tx_->RequestStart()) {
        tx_->SetKeyed(true);
      }
      break;

    case UiMode::kArdf:
      if (ardf_) {
        if (ardf_->IsRunning()) {
          ardf_->Stop();
        } else {
          ardf_->Start();
        }
      }
      break;

    case UiMode::kFrequency:
      AdjustFrequency(+1);
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
      if (test_duty_active_ && test_duty_ < hardware_config::kDutyLimit) {
        ++test_duty_;
      }
      test_duty_active_ = true;
      if (modulator_) {
        modulator_->SetFixedDuty(test_duty_);
      }
      break;

    case UiMode::kTestCarrier:
    case UiMode::kTestTone:
      // Toggle the carrier through the normal start/stop sequence, so the
      // ordering rules of spec 8.2 / 8.3 apply here too.
      if (!tx_) {
        break;
      }
      if (test_output_on_) {
        StopTestOutput();
      } else {
        tx_->SetKeyedSource(mode_ == UiMode::kTestTone
                                ? ModulationSource::kContinuousTone
                                : ModulationSource::kSilent);
        if (tx_->RequestStart()) {
          test_output_on_ = true;
          carrier_on_ms_ = NowMs();
          if (mode_ == UiMode::kTestTone) {
            tx_->SetKeyed(true);
          }
        }
      }
      break;

    case UiMode::kTestBurst:
      if (!tx_) {
        break;
      }
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
  // Only the voice PTT cares about the release.
  if (mode_ == UiMode::kVoice && tx_) {
    tx_->SetKeyed(false);
    tx_->RequestStop();
    UpdateDisplay();
  }
}

void UiControllerTask::OnFnMinusPress() {
  switch (mode_) {
    case UiMode::kFrequency:
      AdjustFrequency(-1);
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

    case UiMode::kTestCarrier:
      // L/H is only changeable while the carrier is off, which includes the
      // ramp-down after OFF.
      if (!test_output_on_ && !(tx_ && tx_->IsTransmitting())) {
        carrier_level_ = (carrier_level_ + 1) % 3;  // L -> H -> P -> L
        if (modulator_) {
          modulator_->SetCarrierLevel(CarrierLevelValue());
        }
      }
      break;

    case UiMode::kTestDuty:
      if (test_duty_active_ && test_duty_ > 0) {
        --test_duty_;
      }
      test_duty_active_ = true;
      if (modulator_) {
        modulator_->SetFixedDuty(test_duty_);
      }
      break;

    default:
      break;  // unused in the other modes
  }

  UpdateDisplay();
}

void UiControllerTask::OnFnPlusRepeat() {
  // Auto-repeat only applies to the value-adjusting modes; the PTT and the
  // ARDF run toggle are single shot (spec 9.7).
  switch (mode_) {
    case UiMode::kFrequency:
    case UiMode::kCallsign:
    case UiMode::kIdent:
    case UiMode::kWpm:
    case UiMode::kTestDuty:
      OnFnPlusPress();
      break;
    default:
      break;
  }
}

void UiControllerTask::OnFnMinusRepeat() {
  switch (mode_) {
    case UiMode::kFrequency:
    case UiMode::kCallsign:
    case UiMode::kIdent:
    case UiMode::kWpm:
    case UiMode::kTestDuty:
      OnFnMinusPress();
      break;
    default:
      break;
  }
}

void UiControllerTask::AdjustFrequency(int32_t steps) {
  const int64_t delta =
      static_cast<int64_t>(steps) * hardware_config::kFreqStepHz;
  int64_t next = static_cast<int64_t>(pending_frequency_hz_) + delta;

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

void UiControllerTask::ApplyPendingFrequency() {
  if (!frequency_pending_) {
    return;
  }
  if ((NowMs() - frequency_change_ms_) <
      static_cast<int64_t>(hardware_config::kFreqWriteDelayMs)) {
    return;
  }

  frequency_pending_ = false;

  if (si5351a_) {
    si5351a_->SetFrequency(Si5351A::Clk::kClk0, pending_frequency_hz_);
    // Retuning drops the carrier; bring it back if we were transmitting.
    if (tx_ && tx_->IsTransmitting()) {
      si5351a_->CarrierEnable();
    }
  }
  if (settings_) {
    settings_->SetFrequencyHz(pending_frequency_hz_);
  }
}

void UiControllerTask::CycleCallsignCursor() {
  callsign_cursor_ = (callsign_cursor_ + 1) % kCursorPositions;
}

void UiControllerTask::IncrementCallsignChar() {
  switch (callsign_cursor_) {
    case 0: {
      // Prefix: 29 options, both characters move together.
      const uint8_t index = (FindPrefixIndex(callsign_) + 1) % kPrefixCount;
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

void UiControllerTask::AdjustIdent(int direction) {
  if (!settings_) {
    return;
  }

  const uint8_t count = static_cast<uint8_t>(morse::ArdfId::kCount);
  uint8_t index = static_cast<uint8_t>(settings_->GetArdfId());
  index = static_cast<uint8_t>((index + count + direction) % count);

  const morse::ArdfId id = static_cast<morse::ArdfId>(index);
  settings_->SetArdfId(id);
  if (ardf_) {
    ardf_->SetArdfId(id);
  }
}

void UiControllerTask::AdjustWpm(int direction) {
  if (!settings_) {
    return;
  }

  // Clamp rather than wrap: a long press should stop at the end of the range,
  // not jump from 40 back to 5.
  int wpm = static_cast<int>(settings_->GetCwWpm()) + direction;
  if (wpm < hardware_config::kCwWpmMin) {
    wpm = hardware_config::kCwWpmMin;
  }
  if (wpm > hardware_config::kCwWpmMax) {
    wpm = hardware_config::kCwWpmMax;
  }

  settings_->SetCwWpm(static_cast<uint8_t>(wpm));
  if (ardf_) {
    ardf_->SetWpm(static_cast<uint8_t>(wpm));
  }
}

void UiControllerTask::FormatLine1(char* out, size_t size) {
  const TxState state = tx_ ? tx_->GetState() : TxState::kIdle;

  if (state == TxState::kFault) {
    std::snprintf(out, size, "FAULT   ");
    return;
  }

  // Battery below the inhibit threshold pins the top line (spec 9.5).
  if (hardware_config::kBatInhibitEnabled && battery_voltage_ > 0.0f &&
      battery_voltage_ < hardware_config::kBatInhibitV) {
    std::snprintf(out, size, "LOW BATT");
    return;
  }

  switch (mode_) {
    case UiMode::kVoice:
      if (tx_ && tx_->IsTransmitting()) {
        std::snprintf(out, size, "VOICE TX");
      } else {
        std::snprintf(out, size, "VOICE   ");
      }
      break;

    case UiMode::kArdf: {
      const char* id_text =
          morse::GetArdfIdText(settings_ ? settings_->GetArdfId()
                                         : morse::ArdfId::kMoe);
      if (ardf_ && ardf_->IsRunning()) {
        if (tx_ && tx_->IsTransmitting()) {
          std::snprintf(out, size, "ARDF %-3s TX", id_text);
        } else {
          std::snprintf(out, size, "ARDF %-3s %3us", id_text,
                        ardf_->GetSecondsToNextSlot());
        }
      } else {
        std::snprintf(out, size, "ARDF %-3s", id_text);
      }
      break;
    }

    case UiMode::kFrequency:
      std::snprintf(out, size, "FREQ");
      break;

    case UiMode::kCallsign:
      std::snprintf(out, size, "CS:%s", callsign_);
      break;

    case UiMode::kIdent:
      std::snprintf(out, size, "ARDF ID:%s",
                    morse::GetArdfIdText(
                        settings_ ? settings_->GetArdfId() : morse::ArdfId::kMoe));
      break;

    case UiMode::kWpm:
      std::snprintf(out, size, "ARDF WPM:%u",
                    settings_ ? settings_->GetCwWpm()
                              : hardware_config::kCwWpmDefault);
      break;

    case UiMode::kTestDuty:
      std::snprintf(out, size, "D:DUTY %3u %s", test_duty_,
                    test_duty_active_ ? "ON " : "OFF");
      break;

    case UiMode::kTestCarrier:
      std::snprintf(out, size, "D:CARR %c %s", "LHP"[carrier_level_],
                    test_output_on_ ? "ON " : "OFF");
      break;

    case UiMode::kTestTone:
      std::snprintf(out, size, "D:TONE %s", test_output_on_ ? "ON " : "OFF");
      break;

    case UiMode::kTestBurst: {
      const int64_t phase =
          test_output_on_
              ? ((NowMs() - burst_start_ms_) % hardware_config::kBurstPeriodMs) /
                    1000
              : 0;
      std::snprintf(out, size, "D:BRST %s%2ds %s", test_output_on_ ? "ON " : "OFF",
                    static_cast<int>(phase), burst_tx_active_ ? "TX" : "--");
      break;
    }

    case UiMode::kCount:
      std::snprintf(out, size, "%-8s", ModeShortName(mode_));
      break;
  }
}

void UiControllerTask::FormatLine2(char* out, size_t size) {
  // The modes that run the modulation path put the ISR statistics here
  // instead of the frequency: the 20us budget of spec 7.1 has to be checkable
  // on the bench, and the serial console is not available. D:DUTY and D:CARR
  // are trivial for the ISR, so they keep the frequency.
  if ((mode_ == UiMode::kTestTone || mode_ == UiMode::kTestBurst) &&
      modulator_) {
    const IsrStats stats = modulator_->GetStats();
    // "M23 A11" - ISR max and average in microseconds, with a '!' in place of
    // the leading space when the max has broken the 20us budget of spec 7.1.
    // Static analysis of the soft-float call counts puts the A3E path right
    // around 20us, so this is the number that settles it on real hardware.
    const char flag =
        (stats.max_us > hardware_config::kIsrBudgetUs) ? '!' : ' ';
    std::snprintf(out, size, "%cM%2uA%2u", flag, stats.max_us, stats.avg_us);
    return;
  }

  // Voice mode swaps the frequency for the ISR statistics while transmitting.
  // The A3E chain is the expensive path - static analysis of its soft-float
  // call count lands right on the 20us budget - and it can only be measured
  // while it is actually running, which is exactly while the PTT is held.
  if (mode_ == UiMode::kVoice && tx_ && tx_->GetState() == TxState::kTxKeyed &&
      modulator_) {
    const IsrStats stats = modulator_->GetStats();
    const char flag =
        (stats.max_us > hardware_config::kIsrBudgetUs) ? '!' : ' ';
    std::snprintf(out, size, "%cM%2uA%2u", flag, stats.max_us, stats.avg_us);
    return;
  }

  // Status character in column 8 (spec 9.5).
  char status = ' ';
  const TxState state = tx_ ? tx_->GetState() : TxState::kIdle;
  if (state == TxState::kFault) {
    status = 'X';
  } else if (tx_ && tx_->IsTransmitting()) {
    status = '*';
  } else if (battery_voltage_ > 0.0f &&
             battery_voltage_ < hardware_config::kBatWarnV) {
    status = '!';
  }

  // "145.70MHz*": two decimals are enough (the step is 20kHz). The status
  // character sits right after the frequency; the battery voltage owns the
  // bottom right corner (see UpdateDisplay()).
  const uint32_t mhz = pending_frequency_hz_ / 1000000;
  const uint32_t khz10 = (pending_frequency_hz_ % 1000000) / 10000;
  std::snprintf(out, size, "%3u.%02uMHz%c", mhz, khz10, status);
}

bool UiControllerTask::SplashActive() const {
  return splash_until_ms_ != 0 && NowMs() < splash_until_ms_;
}

void UiControllerTask::DrawSplash() {
  lcd_->SetCursor(0, 0);
  lcd_->Printf("%-*.*s", hardware_config::kLcdColumns,
               hardware_config::kLcdColumns, hardware_config::kBootTitle);
  lcd_->SetCursor(0, 1);
  lcd_->Printf("%-*.*s", hardware_config::kLcdColumns,
               hardware_config::kLcdColumns, callsign_);
}

void UiControllerTask::UpdateDisplay() {
  if (!lcd_ || !lcd_->IsReady()) {
    return;
  }

  if (SplashActive()) {
    DrawSplash();
    return;
  }

  char line1[hardware_config::kLcdColumns + 1] = {};
  char line2[hardware_config::kLcdColumns + 1] = {};

  FormatLine1(line1, sizeof(line1));
  FormatLine2(line2, sizeof(line2));

  // Battery voltage, always in the bottom right corner ("12.8V"). It is laid
  // over whatever the second line holds, ISR statistics included.
  if (battery_voltage_ > 0.0f) {
    constexpr size_t kVoltCols = 5;
    char volt[kVoltCols + 1];
    std::snprintf(volt, sizeof(volt), "%4.1fV",
                  static_cast<double>(battery_voltage_));
    const size_t start = hardware_config::kLcdColumns - kVoltCols;
    for (size_t i = std::strlen(line2); i < start; ++i) line2[i] = ' ';
    std::memcpy(&line2[start], volt, kVoltCols);
    line2[hardware_config::kLcdColumns] = '\0';
  }

  lcd_->SetCursor(0, 0);
  lcd_->Printf("%-*.*s", hardware_config::kLcdColumns,
               hardware_config::kLcdColumns, line1);
  lcd_->SetCursor(0, 1);
  lcd_->Printf("%-*.*s", hardware_config::kLcdColumns,
               hardware_config::kLcdColumns, line2);

  // Park the hardware cursor on the character being edited so the operator
  // can see which position FN- will change.
  if (mode_ == UiMode::kCallsign) {
    lcd_->SetCursor(kCallsignLabelCols + kCursorColumn[callsign_cursor_], 0);
  }
}

}  // namespace transmitter_system

// EOF
