// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// ARDF transmit schedule (spec 10.4)

#include "ardf_scheduler.h"

#include <esp_timer.h>

#include <cstring>

#include "hardware_config.h"
#include "logger.h"

namespace transmitter_system {

namespace {
int64_t NowUs() { return esp_timer_get_time(); }

constexpr int64_t kUsPerSecond = 1000000;
constexpr int64_t kCycleUs =
    static_cast<int64_t>(hardware_config::kArdfCycleS) * kUsPerSecond;
constexpr int64_t kSlotUs =
    static_cast<int64_t>(hardware_config::kArdfSlotS) * kUsPerSecond;

// Margin reserved at the end of a slot for the stop sequence: the 5ms ramp
// plus the 10ms settle, rounded up generously.
constexpr int64_t kStopMarginMs = 50;

// Send `text` followed by a word gap. The keyer only waits for the envelope
// to decay after a string, so back to back Send() calls (callsign then ID, or
// the ID repeating) would otherwise run together: "MOI" "MOI" sounds like
// "MOIMOI", with the I and the next M fused into "..--".
void SendWithWordGap(CwKeyer* keyer, const char* text) {
  char buf[CwKeyer::kMaxTextLength + 1] = {};
  std::strncpy(buf, text, CwKeyer::kMaxTextLength - 1);
  std::strncat(buf, " ", sizeof(buf) - std::strlen(buf) - 1);
  keyer->Send(buf);
}
}  // namespace

ArdfScheduler::ArdfScheduler()
    : tx_(nullptr),
      keyer_(nullptr),
      ardf_id_(static_cast<morse::ArdfId>(hardware_config::kArdfIdDefault)),
      callsign_{},
      running_(false),
      phase_(Phase::kWaiting),
      slot_done_(false),
      cycle_start_us_(0),
      last_call_us_(0) {}

void ArdfScheduler::SetCallsign(const char* callsign) {
  if (!callsign) {
    return;
  }
  std::strncpy(callsign_, callsign, sizeof(callsign_) - 1);
  callsign_[sizeof(callsign_) - 1] = '\0';
}

uint32_t ArdfScheduler::UnitsToMs(uint32_t units) const {
  const uint8_t wpm = keyer_ ? keyer_->GetWpm() : hardware_config::kCwWpmDefault;
  return units * morse::UnitMs(wpm);
}

bool ArdfScheduler::Start() {
  if (running_) {
    return true;
  }
  if (!HasCallsign()) {
    ESP_LOGW(kTag, "ARDF start refused: no callsign set");
    return false;
  }
  running_ = true;
  cycle_start_us_ = NowUs();
  last_call_us_ = 0;
  phase_ = Phase::kWaiting;
  slot_done_ = false;

  if (tx_) {
    tx_->SetArdfRunning(true);
    tx_->SetKeyedSource(ModulationSource::kCwKeyer);
  }

  ESP_LOGI(kTag, "ARDF run started (%s, slot %u)",
           morse::GetArdfIdText(ardf_id_), morse::GetArdfSlot(ardf_id_));
  return true;
}

void ArdfScheduler::Stop() {
  if (!running_) {
    return;
  }
  running_ = false;
  phase_ = Phase::kWaiting;

  if (keyer_) {
    keyer_->Stop();
  }
  if (tx_) {
    tx_->SetKeyed(false);
    tx_->SetArdfRunning(false);
    if (tx_->IsTransmitting()) {
      tx_->RequestStop();
    }
  }

  ESP_LOGI(kTag, "ARDF run stopped");
}

int64_t ArdfScheduler::CycleElapsedUs() const {
  const int64_t elapsed = NowUs() - cycle_start_us_;
  return elapsed % kCycleUs;
}

bool ArdfScheduler::IsInOwnSlot() const {
  if (morse::IsContinuous(ardf_id_)) {
    return true;  // MO transmits without interruption
  }

  const uint8_t slot = morse::GetArdfSlot(ardf_id_);
  if (slot == 0) {
    return false;
  }

  const int64_t elapsed = CycleElapsedUs();
  const int64_t slot_start = static_cast<int64_t>(slot - 1) * kSlotUs;
  return elapsed >= slot_start && elapsed < (slot_start + kSlotUs);
}

uint32_t ArdfScheduler::GetSecondsToNextSlot() const {
  if (!running_ || morse::IsContinuous(ardf_id_)) {
    return 0;
  }
  if (IsInOwnSlot()) {
    return 0;
  }

  const uint8_t slot = morse::GetArdfSlot(ardf_id_);
  if (slot == 0) {
    return 0;
  }

  const int64_t elapsed = CycleElapsedUs();
  const int64_t slot_start = static_cast<int64_t>(slot - 1) * kSlotUs;

  int64_t remaining = slot_start - elapsed;
  if (remaining < 0) {
    remaining += kCycleUs;  // slot already passed this cycle
  }
  return static_cast<uint32_t>(remaining / kUsPerSecond);
}

bool ArdfScheduler::IdRepetitionFits() const {
  if (morse::IsContinuous(ardf_id_)) {
    return true;
  }

  const uint8_t slot = morse::GetArdfSlot(ardf_id_);
  if (slot == 0) {
    return false;
  }

  const int64_t elapsed = CycleElapsedUs();
  const int64_t slot_end = static_cast<int64_t>(slot) * kSlotUs;
  const int64_t remaining_ms = (slot_end - elapsed) / 1000;

  // One more repetition needs the signal itself, a word gap after it, and
  // enough left over for the stop sequence.
  const int64_t needed_ms =
      UnitsToMs(morse::CountUnits(morse::GetArdfIdText(ardf_id_)) +
                morse::kWordGapUnits) +
      kStopMarginMs;

  return remaining_ms > needed_ms;
}

void ArdfScheduler::Tick() {
  if (!running_ || !tx_ || !keyer_) {
    return;
  }

  if (tx_->GetState() == TxState::kFault) {
    running_ = false;
    return;
  }

  const bool in_slot = IsInOwnSlot();

  switch (phase_) {
    case Phase::kWaiting:
      if (!in_slot) {
        slot_done_ = false;  // armed again for the next cycle
        break;
      }
      if (slot_done_) {
        break;  // already sent this slot's share; wait for the slot to end
      }
      // Slot opened. Between slots the carrier is down entirely: ARDF assumes
      // exactly one station is on the air at a time, so leaving a carrier
      // running would jam the others (spec 10.4).
      if (!tx_->RequestStart()) {
        break;  // inhibited; try again next tick
      }
      phase_ = Phase::kStarting;
      break;

    case Phase::kStarting:
      if (!in_slot) {
        // The slot closed before the carrier came up (a long inhibit, say).
        phase_ = Phase::kStopping;
        break;
      }
      if (tx_->GetState() == TxState::kIdle) {
        // The controller dropped the start, e.g. the battery inhibit tripped
        // between the request and its Tick(). Ask again, or go back to
        // waiting if it is refused outright.
        if (!tx_->RequestStart()) {
          phase_ = Phase::kWaiting;
        }
        break;
      }
      if (tx_->GetState() != TxState::kTxReady) {
        break;
      }
      // Callsign first, every slot (spec 10.5). The ARDF identifier is a
      // competition marker, not a station identification, so the callsign is
      // required separately.
      SendWithWordGap(keyer_, callsign_);
      tx_->SetKeyed(true);
      last_call_us_ = NowUs();
      phase_ = Phase::kSendingCall;
      break;

    case Phase::kSendingCall:
      if (keyer_->IsBusy()) {
        break;
      }
      if (!morse::IsContinuous(ardf_id_) && !in_slot) {
        phase_ = Phase::kStopping;
        break;
      }
      SendWithWordGap(keyer_, morse::GetArdfIdText(ardf_id_));
      phase_ = Phase::kSendingId;
      break;

    case Phase::kSendingId: {
      if (keyer_->IsBusy()) {
        break;
      }

      if (morse::IsContinuous(ardf_id_)) {
        // MO runs until stopped, re-identifying every 10 minutes. The
        // insertion happens between repetitions, never mid-character.
        const int64_t since_call_us = NowUs() - last_call_us_;
        if (since_call_us >=
            static_cast<int64_t>(hardware_config::kMoCallIntervalS) *
                kUsPerSecond) {
          SendWithWordGap(keyer_, callsign_);
          last_call_us_ = NowUs();
          phase_ = Phase::kSendingCall;
          break;
        }
        SendWithWordGap(keyer_, morse::GetArdfIdText(ardf_id_));
        break;
      }

      if (in_slot && IdRepetitionFits()) {
        SendWithWordGap(keyer_, morse::GetArdfIdText(ardf_id_));
        break;
      }

      phase_ = Phase::kStopping;
      break;
    }

    case Phase::kStopping:
      slot_done_ = true;
      tx_->SetKeyed(false);
      keyer_->Stop();
      if (tx_->IsTransmitting()) {
        tx_->RequestStop();
      }
      if (tx_->GetState() == TxState::kIdle) {
        phase_ = Phase::kWaiting;
      }
      break;
  }
}

}  // namespace transmitter_system

// EOF
