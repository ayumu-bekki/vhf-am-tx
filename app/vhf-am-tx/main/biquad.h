#ifndef AM_TRANSMITTER_MAIN_BIQUAD_H_
#define AM_TRANSMITTER_MAIN_BIQUAD_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Direct Form II Transposed biquad / first order sections
//
// These run inside the 25kHz ISR, so everything lives in the header and is
// marked IRAM_ATTR: a flash cache miss mid-tick costs tens of microseconds,
// which alone would blow the 20us budget of spec 7.1.
//
// ESP32-C6 has no FPU (rv32imac / ilp32), so every float operation here is a
// soft-float library call. Measure before adding stages - see the ISR timing
// statistics surfaced in the test modes.

#include <esp_attr.h>

#include <cstdint>

namespace transmitter_system {

/// Collapses subnormals to zero.
///
/// ESP32-C6 has no FPU, so every float operation is a libgcc call, and those
/// take a markedly slower path for subnormal operands. When the input goes
/// quiet the filter states decay smoothly into that range and stay there:
/// measured on hardware, the A3E chain cost 29.8us of DSP on decaying silence
/// against 16.9us on real signal - enough on its own to blow the 20us ISR
/// budget of spec 7.1 while nobody is even talking.
///
/// The threshold sits far below anything audible (one ADC LSB is around 5e-4
/// in these units), so this is inaudible and only removes the pathological
/// timing.
inline float IRAM_ATTR FlushDenormal(const float v) {
  return (v > -1e-20f && v < 1e-20f) ? 0.0f : v;
}

/// Second order section, Direct Form II Transposed.
///
///   y[n] = b0*x[n] + s1
///   s1   = b1*x[n] - a1*y[n] + s2
///   s2   = b2*x[n] - a2*y[n]
///
/// Transposed form is used rather than Direct Form I because it needs two
/// state variables instead of four and behaves better numerically at the low
/// end, which matters for the 300Hz high pass at fs = 25kHz.
class Biquad {
 public:
  Biquad() : b0_(1.0f), b1_(0.0f), b2_(0.0f), a1_(0.0f), a2_(0.0f) { Reset(); }

  /// Coefficients must already be normalised so that a0 == 1.
  void SetCoefficients(float b0, float b1, float b2, float a1, float a2) {
    b0_ = b0;
    b1_ = b1;
    b2_ = b2;
    a1_ = a1;
    a2_ = a2;
  }

  void Reset() {
    s1_ = 0.0f;
    s2_ = 0.0f;
  }

  inline float IRAM_ATTR Process(const float in) {
    const float out = b0_ * in + s1_;
    s1_ = FlushDenormal(b1_ * in - a1_ * out + s2_);
    s2_ = FlushDenormal(b2_ * in - a2_ * out);
    return out;
  }

 private:
  float b0_, b1_, b2_, a1_, a2_;
  float s1_, s2_;
};

/// First order section, same transposed structure.
///
///   y[n] = b0*x[n] + s1
///   s1   = b1*x[n] - a1*y[n]
class FirstOrder {
 public:
  FirstOrder() : b0_(1.0f), b1_(0.0f), a1_(0.0f) { Reset(); }

  void SetCoefficients(float b0, float b1, float a1) {
    b0_ = b0;
    b1_ = b1;
    a1_ = a1;
  }

  void Reset() { s1_ = 0.0f; }

  inline float IRAM_ATTR Process(const float in) {
    const float out = b0_ * in + s1_;
    s1_ = FlushDenormal(b1_ * in - a1_ * out);
    return out;
  }

 private:
  float b0_, b1_, a1_;
  float s1_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_BIQUAD_H_
