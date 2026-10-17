// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// A3E voice processing chain (spec 4)

#include "voice_dsp.h"

#include <cmath>

namespace transmitter_system {

namespace {

// ---------------------------------------------------------------------------
// Filter coefficients
//
// All of these come from a bilinear transform at fs = 25kHz, normalised so
// a0 = 1. They are hard-coded rather than computed at boot because the
// derivation needs transcendental functions and the values never change.
//
// EQ (spec 4.1) inverts the analog path:
//
//   H_eq(s) = (1 + 3sRC + (sRC)^2) / (1 + s/wp)^2   x   (1 + sRC) / (1 + s/wp)
//             \___ inverse of M2 ____/                  \_ inverse of AA _/
//
//   RC = 47us (1kohm x 47nF, shared by M2 and the anti-alias filter)
//   wp = 2*pi*10000
//
// The spec suggests putting the poles at "well above 3kHz, e.g. 8kHz". 8kHz
// leaves a 1.10dB error at 3kHz, which misses the +-1dB requirement; 10kHz
// brings the worst case to 0.52dB. Error against the spec 4.1 table:
//
//   300Hz +0.10 / 500 +0.08 / 700 +0.05 / 1k 0.00
//   1.5k  -0.11 / 2k   -0.23 / 2.5k -0.37 / 3k -0.52   [dB]
//
// Judge the EQ on its own, not through the band-limiting filters: at the band
// edges the HPF and LPF skirts dominate, which is their job, not EQ error.
// ---------------------------------------------------------------------------

// EQ: second order section
constexpr float kEqBiquadB0 = 4.20877923f;
constexpr float kEqBiquadB1 = -2.80481917f;
constexpr float kEqBiquadB2 = -0.16357569f;
constexpr float kEqBiquadA1 = 0.22745090f;
constexpr float kEqBiquadA2 = 0.01293348f;

// EQ: first order section
constexpr float kEqFirstB0 = 1.86549013f;
constexpr float kEqFirstB1 = -0.75176468f;
constexpr float kEqFirstA1 = 0.11372545f;

// The EQ sections together have gain at 1kHz (+2.3436dB, linear 1.309719);
// normalise so the chain is unity there and the compressor threshold below
// means what it says.
constexpr float kEqNormalise = 0.763522f;  // 1 / 1.309719

// 300Hz high pass, 2nd order Butterworth (spec 4.4).
//
// Doubles up with the 318Hz analog HPF on purpose. The stated reason in the
// spec is residual DC, which the bias tracker (spec 3.4, fc = 1Hz) already
// removes; what this actually earns is low frequency rejection between about
// 10Hz and 300Hz - 31dB more at 50Hz, 19dB more at 100Hz. That is wind noise
// and handling rumble, which would otherwise eat limiter headroom outdoors.
//
// Deliberately kept even though dropping it would save about 4us of the 26us
// ISR time (see spec 7.4): the measured 26us still meets the 40us deadline,
// so there is no reason to trade audio quality for headroom that is not
// currently needed.
constexpr float kHpfB0 = 0.94808079f;
constexpr float kHpfB1 = -1.89616157f;
constexpr float kHpfB2 = 0.94808079f;
constexpr float kHpfA1 = -1.89346415f;
constexpr float kHpfA2 = 0.89885899f;

// 3kHz low pass, 2nd order Butterworth (spec 4.4), after the EQ.
constexpr float kLpfB0 = 0.09131490f;
constexpr float kLpfB1 = 0.18262980f;
constexpr float kLpfB2 = 0.09131490f;
constexpr float kLpfA1 = -0.98240579f;
constexpr float kLpfA2 = 0.34766539f;

// ---------------------------------------------------------------------------
// Compressor (spec 4.3), feed-forward with peak detection.
// ---------------------------------------------------------------------------

// Threshold at -12dB of full scale
constexpr float kCompThreshold = 0.25119f;  // 10^(-12/20)
constexpr float kCompRatio = 4.0f;
// Reciprocal, so the compressor multiplies instead of dividing: __divsf3 is
// about twice the cost of __mulsf3 and the ISR budget is tight (spec 7.4).
constexpr float kCompRatioRecip = 1.0f / kCompRatio;
constexpr float kCompAttackMs = 5.0f;
constexpr float kCompReleaseMs = 200.0f;

// Default makeup gain. The operator trims this from the UI; speech peaks only
// use a few percent of the ADC range (spec 3.2), so some makeup is expected.
constexpr float kDefaultMakeupGain = 1.0f;

// Limiter activity is averaged over one second.
constexpr uint32_t kActivityWindowSamples = hardware_config::kTickHz;
constexpr float kActivityWindowRecip =
    1.0f / static_cast<float>(kActivityWindowSamples);

/// One-pole smoothing coefficient for a given time constant.
float TimeConstantCoeff(float ms) {
  return std::exp(-1.0f / (ms * 0.001f * hardware_config::kTickHz));
}

}  // namespace

VoiceDsp::VoiceDsp()
    : hpf_(),
      eq_biquad_(),
      eq_first_(),
      lpf_(),
      envelope_(0.0f),
      attack_coeff_(0.0f),
      release_coeff_(0.0f),
      makeup_gain_(kDefaultMakeupGain),
      limit_hits_(0),
      limit_samples_(0),
      limiter_activity_(0.0f),
      peak_(0.0f) {}

void VoiceDsp::Initialize() {
  hpf_.SetCoefficients(kHpfB0, kHpfB1, kHpfB2, kHpfA1, kHpfA2);
  eq_biquad_.SetCoefficients(kEqBiquadB0, kEqBiquadB1, kEqBiquadB2, kEqBiquadA1,
                             kEqBiquadA2);
  eq_first_.SetCoefficients(kEqFirstB0, kEqFirstB1, kEqFirstA1);
  lpf_.SetCoefficients(kLpfB0, kLpfB1, kLpfB2, kLpfA1, kLpfA2);

  attack_coeff_ = TimeConstantCoeff(kCompAttackMs);
  release_coeff_ = TimeConstantCoeff(kCompReleaseMs);

  Reset();
}

void VoiceDsp::Reset() {
  hpf_.Reset();
  eq_biquad_.Reset();
  eq_first_.Reset();
  lpf_.Reset();
  envelope_ = 0.0f;
  limit_hits_ = 0;
  limit_samples_ = 0;
  limiter_activity_.store(0.0f, std::memory_order_relaxed);
  peak_.store(0.0f, std::memory_order_relaxed);
}

float VoiceDsp::Process(float sample) {
  // 1. Band limit the low end. The analog side already rolls off at 318Hz;
  //    this makes sure nothing DC-ish survives into the EQ.
  float x = hpf_.Process(sample);

  // 2. EQ - compensate the anti-alias filter and M2 (spec 4.1).
  x = eq_biquad_.Process(x);
  x = eq_first_.Process(x);
  x *= kEqNormalise;

  // 3. Compressor (spec 4.3). Peak detector with fast attack, slow release;
  //    a short release would pump the background noise up between syllables.
  const float level = std::fabs(x);
  if (level > envelope_) {
    envelope_ = attack_coeff_ * envelope_ + (1.0f - attack_coeff_) * level;
  } else {
    envelope_ = release_coeff_ * envelope_ + (1.0f - release_coeff_) * level;
  }

  float gain = makeup_gain_.load(std::memory_order_relaxed);

  if (envelope_ > kCompThreshold) {
    // Above threshold, let through threshold + excess/ratio.
    //
    // The division by the envelope is unavoidable (it is not a constant), but
    // folding the makeup gain into the same multiply saves one __mulsf3, and
    // the ratio becomes a reciprocal multiply.
    const float excess = envelope_ - kCompThreshold;
    const float target = kCompThreshold + excess * kCompRatioRecip;
    gain *= target / envelope_;
  }

  // 4. Compressor gain and makeup applied together.
  x *= gain;

  // 5. Low pass after the EQ (spec 4.4). This has to come before the limiter
  //    so the limiter sees the waveform that actually reaches the modulator.
  x = lpf_.Process(x);

  // 6. Asymmetric limiter (spec 4.2 / ADR-017). The negative side stops short
  //    of the knee voltage so the trough never collapses, while the positive
  //    side uses the headroom on the 8V rail. Hard clipping is fine here.
  bool limited = false;
  if (x > hardware_config::kXMaxA3e) {
    x = hardware_config::kXMaxA3e;
    limited = true;
  } else if (x < hardware_config::kXMinA3e) {
    x = hardware_config::kXMinA3e;
    limited = true;
  }

  // Limiter activity over a one second window - the operator's only cue that
  // the speech level is sane.
  if (limited) {
    ++limit_hits_;
  }
  if (++limit_samples_ >= kActivityWindowSamples) {
    limiter_activity_.store(
        static_cast<float>(limit_hits_) * kActivityWindowRecip,
        std::memory_order_relaxed);
    limit_hits_ = 0;
    limit_samples_ = 0;
  }

  // Track the peak for the level display.
  const float magnitude = std::fabs(x);
  const float previous_peak = peak_.load(std::memory_order_relaxed);
  if (magnitude > previous_peak) {
    peak_.store(magnitude, std::memory_order_relaxed);
  }

  return x;
}

float VoiceDsp::GetLimiterActivity() const {
  return limiter_activity_.load(std::memory_order_relaxed);
}

float VoiceDsp::ConsumePeak() {
  return peak_.exchange(0.0f, std::memory_order_relaxed);
}

}  // namespace transmitter_system

// EOF
