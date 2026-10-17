// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Transmit sequencing and state machine (spec 8)

#include "tx_controller.h"

#include "hardware_config.h"
#include "logger.h"
#include "util.h"

namespace transmitter_system {

using util::NowMs;

namespace {

[[maybe_unused]] const char* StateName(TxState state) {
  switch (state) {
    case TxState::kIdle: return "IDLE";
    case TxState::kTxStarting: return "TX_STARTING";
    case TxState::kTxReady: return "TX_READY";
    case TxState::kTxKeyed: return "TX_KEYED";
    case TxState::kTxStopping: return "TX_STOPPING";
    case TxState::kFault: return "FAULT";
  }
  return "?";
}
}  // namespace

TxController::TxController()
    : modulator_(nullptr),
      si5351a_(nullptr),
      state_(TxState::kIdle),
      ardf_running_(false),
      inhibited_(false),
      start_request_(false),
      stop_request_(false),
      keyed_request_(false),
      keyed_source_(ModulationSource::kCwKeyer),
      phase_start_ms_(0) {}

void TxController::TransitionTo(TxState state) {
  state_.store(state, std::memory_order_release);
  phase_start_ms_ = NowMs();
  ESP_LOGD(kTag, "TX state -> %s", StateName(state));
}

bool TxController::RequestStart() {
  const TxState state = GetState();
  if (state == TxState::kFault) {
    return false;
  }
  if (inhibited_.load(std::memory_order_acquire)) {
    ESP_LOGW(kTag, "Transmit start refused: battery below inhibit threshold");
    return false;
  }
  if (state == TxState::kTxStarting || state == TxState::kTxReady ||
      state == TxState::kTxKeyed) {
    // Already up. Leaving the flag set here would restart the carrier
    // unasked the next time the controller returns to kIdle.
    return true;
  }
  start_request_.store(true, std::memory_order_release);
  return true;
}

void TxController::RequestStop() {
  start_request_.store(false, std::memory_order_release);
  stop_request_.store(true, std::memory_order_release);
}

void TxController::SetKeyed(bool keyed) {
  keyed_request_.store(keyed, std::memory_order_release);
}

void TxController::SetInhibited(bool inhibited) {
  const bool was = inhibited_.exchange(inhibited, std::memory_order_acq_rel);
  if (inhibited && !was) {
    ESP_LOGW(kTag, "Battery low: inhibiting transmission");
    // Stop an over in progress rather than waiting for the operator.
    if (IsTransmitting()) {
      RequestStop();
    }
  }
}

void TxController::EnterFault(const char* reason) {
  if (GetState() == TxState::kFault) {
    return;
  }

  ESP_LOGE(kTag, "FAULT: %s", reason ? reason : "unknown");

  // Duty first, then the carrier - the same ordering as a normal stop, just
  // without the ramp, because a fault means we no longer trust the timing.
  if (modulator_) {
    modulator_->SetOutputEnabled(false);
    modulator_->SetSource(ModulationSource::kSilent);
  }
  if (si5351a_) {
    si5351a_->CarrierDisable();
  }

  ardf_running_.store(false, std::memory_order_release);
  TransitionTo(TxState::kFault);
}

void TxController::Tick() {
  if (!modulator_ || !si5351a_) {
    return;
  }

  const TxState state = GetState();

  if (state == TxState::kFault) {
    return;  // only a restart clears this
  }

  // A stop request wins over everything else, wherever we are.
  if (stop_request_.load(std::memory_order_acquire) &&
      state != TxState::kIdle && state != TxState::kTxStopping) {
    stop_request_.store(false, std::memory_order_release);
    keyed_request_.store(false, std::memory_order_release);

    // Step 1: ramp the duty down over 5ms. The carrier stays up throughout.
    modulator_->SetSource(ModulationSource::kSilent);
    modulator_->StartRampDown();
    TransitionTo(TxState::kTxStopping);
    return;
  }

  switch (state) {
    case TxState::kIdle:
      if (start_request_.exchange(false, std::memory_order_acq_rel)) {
        if (inhibited_.load(std::memory_order_acquire)) {
          break;
        }
        // Step 1: bring the carrier up and wait for lock. The duty is still
        // 0, so the drain sits at 0V and nothing radiates while we wait.
        modulator_->SetOutputEnabled(false);
        modulator_->SetSource(ModulationSource::kSilent);

        if (!si5351a_->CarrierEnable()) {
          EnterFault("carrier enable failed");
          break;
        }
        TransitionTo(TxState::kTxStarting);
      }
      break;

    case TxState::kTxStarting: {
      // CarrierEnable already waited for the PLL. Step 3: ramp the duty from
      // 0 up to the carrier level over 5ms.
      if (!modulator_->IsOutputEnabled()) {
        modulator_->SetOutputEnabled(true);
        modulator_->StartRampUp();
        break;
      }
      if (!modulator_->IsRampActive()) {
        TransitionTo(TxState::kTxReady);
      }
      break;
    }

    case TxState::kTxReady:
      if (keyed_request_.load(std::memory_order_acquire)) {
        modulator_->SetSource(keyed_source_.load(std::memory_order_acquire));
        TransitionTo(TxState::kTxKeyed);
      }
      break;

    case TxState::kTxKeyed:
      if (!keyed_request_.load(std::memory_order_acquire)) {
        // Back to carrier only. For A2A this is the normal resting state
        // between characters; the carrier is never keyed.
        modulator_->SetSource(ModulationSource::kSilent);
        TransitionTo(TxState::kTxReady);
      }
      break;

    case TxState::kTxStopping: {
      if (modulator_->IsRampActive()) {
        break;  // still ramping down
      }

      // Step 2: wait for the drain to reach 0 before touching the carrier.
      if ((NowMs() - phase_start_ms_) <
          static_cast<int64_t>(hardware_config::kTxStopSettleMs)) {
        break;
      }

      // Step 3: only now is it safe to stop CLK0.
      modulator_->SetOutputEnabled(false);
      si5351a_->CarrierDisable();
      // Callers such as the ARDF scheduler keep asking for a stop for as long
      // as IsTransmitting() is true, which includes this state. Tick() does not
      // consume a request here, so one is still pending now. Left alone, it
      // would abort the next start the moment it came up.
      stop_request_.store(false, std::memory_order_release);
      TransitionTo(TxState::kIdle);
      break;
    }

    case TxState::kFault:
      break;
  }
}

}  // namespace transmitter_system

// EOF
