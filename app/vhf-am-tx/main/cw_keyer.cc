// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// CW keyer (spec 10.3)

#include "cw_keyer.h"

#include <cstring>

namespace transmitter_system {

CwKeyer::CwKeyer()
    : tone_(nullptr),
      text_{},
      pending_text_{},
      pending_load_(false),
      busy_(false),
      stop_request_(false),
      char_index_(0),
      pattern_{0, 0},
      element_index_(0),
      phase_(Phase::kIdle),
      phase_samples_remaining_(0),
      wpm_(hardware_config::kCwWpmDefault),
      samples_per_unit_(0) {
  SetWpm(hardware_config::kCwWpmDefault);
}

void CwKeyer::SetWpm(uint8_t wpm) {
  if (wpm < hardware_config::kCwWpmMin) {
    wpm = hardware_config::kCwWpmMin;
  }
  if (wpm > hardware_config::kCwWpmMax) {
    wpm = hardware_config::kCwWpmMax;
  }
  wpm_ = wpm;

  // unit = 1200 / WPM milliseconds, in 25kHz samples.
  // Computed as a single expression so the rounding happens once.
  samples_per_unit_ =
      (1200u * hardware_config::kTickHz) / (static_cast<uint32_t>(wpm) * 1000u);
}

uint32_t CwKeyer::SamplesForUnits(uint32_t units) const {
  return units * samples_per_unit_;
}

void CwKeyer::Send(const char* text) {
  if (!text) {
    return;
  }

  // Stage the text; the ISR picks it up on its next tick. Copying straight
  // into text_ would race with the ISR reading it mid-character. The flag is
  // dropped first so the ISR cannot pick up a half-written staging buffer
  // left pending by an earlier Send().
  pending_load_.store(false, std::memory_order_release);
  size_t i = 0;
  for (; i < kMaxTextLength && text[i] != '\0'; ++i) {
    pending_text_[i] = text[i];
  }
  pending_text_[i] = '\0';

  busy_.store(true, std::memory_order_release);
  pending_load_.store(true, std::memory_order_release);
}

void CwKeyer::Stop() {
  // A Send() that has not been picked up yet is cancelled too. A Send() after
  // this call still goes out: Tick() handles the stop before the load.
  pending_load_.store(false, std::memory_order_release);
  stop_request_.store(true, std::memory_order_release);
}

float CwKeyer::Tick() {
  if (!tone_) {
    return 0.0f;
  }

  if (stop_request_.exchange(false, std::memory_order_acq_rel)) {
    ResetSequence();
    text_[0] = '\0';
    tone_->SetKey(false);
    if (!pending_load_.load(std::memory_order_acquire)) {
      busy_.store(false, std::memory_order_release);
      return tone_->Next();
    }
  }

  if (pending_load_.exchange(false, std::memory_order_acq_rel)) {
    std::memcpy(text_, pending_text_, sizeof(text_));
    ResetSequence();
    busy_.store(true, std::memory_order_release);
  }

  if (phase_samples_remaining_ > 0) {
    --phase_samples_remaining_;
  }

  if (phase_samples_remaining_ == 0) {
    switch (phase_) {
      case Phase::kIdle:
        // Nothing scheduled: try to start the next element of the string.
        StartNextElement();
        break;

      case Phase::kElement:
        // Element finished sounding; release the key and insert the gap that
        // follows it.
        tone_->SetKey(false);
        if (element_index_ < pattern_.length) {
          phase_ = Phase::kIntraGap;
          phase_samples_remaining_ =
              SamplesForUnits(morse::kIntraCharGapUnits);
        } else {
          AdvanceCharacter();
        }
        break;

      case Phase::kIntraGap:
      case Phase::kInterGap:
      case Phase::kWordGap:
        StartNextElement();
        break;

      case Phase::kTailGap:
        // The envelope has had time to decay; the string is done.
        phase_ = Phase::kIdle;
        busy_.store(false, std::memory_order_release);
        break;
    }
  }

  return tone_->Next();
}

void CwKeyer::ResetSequence() {
  phase_ = Phase::kIdle;
  phase_samples_remaining_ = 0;
  char_index_ = 0;
  element_index_ = 0;
  pattern_ = {0, 0};
}

void CwKeyer::StartNextElement() {
  // Still inside a character?
  if (element_index_ < pattern_.length) {
    const bool is_dah = (pattern_.bits >> (7 - element_index_)) & 0x01;
    ++element_index_;
    tone_->SetKey(true);
    phase_ = Phase::kElement;
    phase_samples_remaining_ =
        SamplesForUnits(is_dah ? morse::kDahUnits : morse::kDitUnits);
    return;
  }

  // Need the next character.
  while (text_[char_index_] != '\0') {
    const char c = text_[char_index_];
    ++char_index_;

    if (c == ' ') {
      // Explicit word gap. The inter-character gap already emitted after the
      // previous character counts towards it, so only the remainder is added.
      phase_ = Phase::kWordGap;
      phase_samples_remaining_ =
          SamplesForUnits(morse::kWordGapUnits - morse::kInterCharGapUnits);
      return;
    }

    const morse::Pattern pattern = morse::Lookup(c);
    if (pattern.length == 0) {
      continue;  // unsupported character, skip it
    }

    pattern_ = pattern;
    element_index_ = 0;
    StartNextElement();
    return;
  }

  // String exhausted. Hold for the envelope to finish decaying before
  // declaring the keyer idle, so a caller that chains transmissions does not
  // cut the tail off the last element.
  phase_ = Phase::kTailGap;
  phase_samples_remaining_ = hardware_config::kEnvelopeSamples;
}

void CwKeyer::AdvanceCharacter() {
  pattern_ = {0, 0};
  element_index_ = 0;

  // If more characters follow, separate them with an inter-character gap.
  if (text_[char_index_] != '\0') {
    phase_ = Phase::kInterGap;
    phase_samples_remaining_ = SamplesForUnits(morse::kInterCharGapUnits);
  } else {
    phase_ = Phase::kTailGap;
    phase_samples_remaining_ = hardware_config::kEnvelopeSamples;
  }
}

}  // namespace transmitter_system

// EOF
