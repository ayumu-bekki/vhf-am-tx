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
      tone_(nullptr),
      keyer_(nullptr),
      state_(TxState::kIdle),
      ardf_running_(false),
      inhibited_(false),
      start_request_(false),
      stop_request_(false),
      keyed_request_(false),
      keyed_source_(ModulationSource::kCwKeyer),
      power_index_(hardware_config::kPowerDefault),
      emission_(Emission::kA2a),
      manual_keying_(false),
      clk_lead_start_ms_(0),
      a1a_active_(false),
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
    if (keyer_) {
      keyer_->SetManualKey(false);  // 5ms envelope fall, then the ramp-down
    }
    clk_lead_start_ms_ = 0;

    // Step 1: ramp the duty down over 5ms. The carrier stays up throughout.
    // A1A keeps the keyer as the source: the silent source would be x = 0,
    // the carrier level, which is not where an A1A key-up sits.
    modulator_->SetSource(a1a_active_ ? ModulationSource::kCwKeyer
                                      : ModulationSource::kSilent);
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

        // Everything that depends on the stage or the emission is decided
        // here and not touched again until the next start: the ISR only reads
        // it (spec 6.5).
        modulator_->SetPowerScale(hardware_config::PowerFactor(
            power_index_.load(std::memory_order_acquire)));
        a1a_active_ = emission_.load(std::memory_order_acquire) ==
                          Emission::kA1a &&
                      keyed_source_.load(std::memory_order_acquire) ==
                          ModulationSource::kCwKeyer;
        if (tone_) {
          tone_->SetKeyingMode(a1a_active_
                                   ? ToneGenerator::KeyingMode::kA1a
                                   : ToneGenerator::KeyingMode::kToneKeying);
        }

        if (!si5351a_->CarrierEnable()) {
          EnterFault("carrier enable failed");
          break;
        }
        if (a1a_active_) {
          // The carrier stays off between elements. CLK0 is enabled only to
          // get the PLL locked; the keyer switches it from here on.
          si5351a_->CarrierOutput(false);
        }
        TransitionTo(TxState::kTxStarting);
      }
      break;

    case TxState::kTxStarting: {
      // CarrierEnable already waited for the PLL. Step 3: ramp the duty from
      // 0 up to the carrier level over 5ms.
      if (a1a_active_) {
        // x = -1 while key up, so the drain is already at 0V: no ramp, no
        // carrier. The keyer raises it with the first element.
        modulator_->SetSource(ModulationSource::kCwKeyer);
        modulator_->SetOutputEnabled(true);
        modulator_->SnapRampFull();
        TransitionTo(TxState::kTxReady);
        break;
      }
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
        const bool manual = manual_keying_.load(std::memory_order_acquire) &&
                            keyed_source_.load(std::memory_order_acquire) ==
                                ModulationSource::kCwKeyer;
        if (manual && a1a_active_) {
          // A1A: CLK0 up first, then the key follows 1ms later.
          if (clk_lead_start_ms_ == 0) {
            si5351a_->CarrierOutput(true);
            clk_lead_start_ms_ = NowMs();
            break;
          }
          if ((NowMs() - clk_lead_start_ms_) <
              static_cast<int64_t>(hardware_config::kA1aClkLeadMs)) {
            break;
          }
        }
        modulator_->SetSource(keyed_source_.load(std::memory_order_acquire));
        if (manual && keyer_) {
          keyer_->SetManualKey(true);
        }
        TransitionTo(TxState::kTxKeyed);
      }
      break;

    case TxState::kTxKeyed:
      if (!keyed_request_.load(std::memory_order_acquire)) {
        // Back to carrier only. For A2A this is the normal resting state
        // between characters; the carrier is never keyed.
        if (keyer_) {
          keyer_->SetManualKey(false);
        }
        if (clk_lead_start_ms_ != 0 && si5351a_) {
          si5351a_->CarrierOutput(false);  // A1A manual key released
        }
        clk_lead_start_ms_ = 0;
        modulator_->SetSource(a1a_active_ ? ModulationSource::kCwKeyer
                                          : ModulationSource::kSilent);
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
