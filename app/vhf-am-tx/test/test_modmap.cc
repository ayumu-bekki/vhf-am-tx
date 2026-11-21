// Host test: x -> duty mapping and the hard ceiling (R6 22k, ADR-024).
//
// The mapping and the ceiling are the last thing between a firmware bug and
// the 8V rail, so they are checked on the host, with the exact functions the
// ISR runs, before anything goes near hardware.
#include <cmath>
#include <cstdio>

#include "hardware_config.h"
#include "modulation_map.h"

using namespace transmitter_system;
using hardware_config::kDutyHardMax;
using hardware_config::kDutyLimit;
using modulation_map::ClampDuty;
using modulation_map::XToDuty;

static int g_fail = 0;
static void Check(bool ok, const char* what) {
  printf("  %-66s %s\n", what, ok ? "OK" : "MISMATCH");
  if (!ok) ++g_fail;
}

int main() {
  printf("--- constants (T1-T3) ---\n");
  Check(std::fabs(hardware_config::kModGain - 3.2f) < 1e-6f, "T1 MOD_GAIN == 3.2");
  Check(std::lround(hardware_config::kModKNom * 256.0f) == 87, "T2 round(K_nom * 256) == 87");
  Check(hardware_config::kDutyCarrier == 86, "kDutyCarrier == 86 (calibrated)");
  const uint32_t derived = static_cast<uint32_t>(std::floor(
      hardware_config::kVDrainHardLimit * 256.0f /
      (hardware_config::kVddPwm * hardware_config::kModGain)));
  Check(derived == 187 && kDutyHardMax == 187, "T3 floor(7.75*256/(3.3*gain)) == 187 == kDutyHardMax");
  Check(static_cast<float>(kDutyHardMax) / 256.0f * hardware_config::kVddPwm *
                hardware_config::kModGain <= 7.85f,
        "hard max stays under the 7.85V rail limit");
  // The old 20k value must not satisfy the derivation (guards a wrong R6).
  const float old_gain = 1.0f + 20.0e3f / 10.0e3f;
  const uint32_t old_hard = static_cast<uint32_t>(std::floor(7.75f * 256.0f / (3.3f * old_gain)));
  Check(old_hard == 200 && old_hard != kDutyHardMax, "20k would give 200: the constants really are for 22k");

  Check(kDutyLimit == 184 && kDutyLimit <= kDutyHardMax,
        "cal: kDutyLimit == 184 and <= 187");
  printf("\n--- mapping (T4-T8, dither off) ---\n");
  struct { float x; uint32_t want; const char* name; } pts[] = {
      {0.0f, 86, "T4 x=0"},        {-1.0f, 0, "T5 x=-1"},
      {-0.85f, 13, "T6 x=-0.85"},  {0.912f, 164, "T7 x=+0.912"},
      {1.10f, 180, "T8 x=+1.10"},  {1.14f, 183, "x=+1.14"},
      {1.5f, 184, "x=+1.5 clamped"},
  };
  for (auto& p : pts) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%s -> %u", p.name, p.want);
    Check(XToDuty(p.x, nullptr) == p.want, buf);
  }

  printf("\n--- sweep (T9) ---\n");
  {
    bool in_range = true, monotonic = true;
    uint32_t prev = 0;
    for (int i = -2000; i <= 3000; ++i) {
      const uint32_t d = XToDuty(i * 0.001f, nullptr);
      if (d > kDutyLimit) in_range = false;
      if (d < prev) monotonic = false;
      prev = d;
    }
    Check(in_range, "T9 x in [-2, +3]: every duty within 0..184");
    Check(monotonic, "T9 non-decreasing in x");
  }

  printf("\n--- dither (T10, T11) ---\n");
  {
    float dither = 0.0f;
    double sum = 0;
    bool only_85_86 = true;
    for (int i = 0; i < 10000; ++i) {
      const uint32_t d = XToDuty(0.0f, &dither);
      sum += d;
      if (d != 85 && d != 86) only_85_86 = false;
    }
    const double mean = sum / 10000.0;
    char buf[80];
    snprintf(buf, sizeof(buf), "T10 mean %.3f (want %.2f +-0.01)", mean,
             hardware_config::kModKCal * 256.0f);
    Check(std::fabs(mean - hardware_config::kModKCal * 256.0f) < 0.01, buf);
    Check(only_85_86, "T10 only 85 and 86 are produced");
  }
  {
    float dither = 0.0f;
    bool pinned = true;
    for (int i = 0; i < 100; ++i) {
      if (XToDuty(3.0f, &dither) != kDutyLimit) pinned = false;
    }
    const uint32_t after = XToDuty(0.0f, &dither);
    Check(pinned, "T11 x=+3 is held at 184");
    Check(after == 85 || after == 86, "T11 first sample after the clamp is 85/86");
    float d2 = 0.0f;
    for (int i = 0; i < 100; ++i) XToDuty(-3.0f, &d2);
    const uint32_t after2 = XToDuty(0.0f, &d2);
    Check(after2 == 85 || after2 == 86, "T11 same after a low clamp");
  }

  printf("\n--- raw duty ceiling (T12, T14) ---\n");
  {
    // What SetFixedDuty() and the final write feed through ClampDuty().
    struct { uint32_t in, want; } c[] = {{0, 0},     {86, 86},   {184, 184},
                                         {185, 184}, {187, 184}, {200, 184},
                                         {255, 184}, {100000, 184}};
    bool ok = true;
    for (auto& e : c) {
      if (ClampDuty(e.in) != e.want) ok = false;
    }
    Check(ok, "T12 0/86/184 pass; 185/187/200/255/100000 -> 184");
    uint32_t peak = 0;
    for (uint32_t d = 0; d <= 400; ++d) {
      const uint32_t v = ClampDuty(d);
      if (v > peak) peak = v;
    }
    Check(peak == 184, "T14 a 0..400 duty sweep never exceeds 184");
  }

  printf("\n--- output stages (P1-P4) ---\n");
  {
    using hardware_config::CarrierDuty;
    using hardware_config::PowerFactor;
    Check(CarrierDuty(0) == 86 && CarrierDuty(1) == 50 && CarrierDuty(2) == 22,
          "P1 carrier duty 86 / 50 / 22 (1.5W / 500mW / 100mW)");
    Check(XToDuty(0.0f, nullptr, PowerFactor(1)) == 50 &&
              XToDuty(0.0f, nullptr, PowerFactor(2)) == 22,
          "P1 XToDuty(0, s) agrees with CarrierDuty");

    bool in_range = true, monotonic = true;
    for (uint8_t i = 0; i < hardware_config::kPowerCount; ++i) {
      uint32_t prev = 0;
      for (int k = -2000; k <= 3000; ++k) {
        const uint32_t d = XToDuty(k * 0.001f, nullptr, PowerFactor(i));
        if (d > kDutyLimit) in_range = false;
        if (d < prev) monotonic = false;
        prev = d;
      }
    }
    Check(in_range, "P2 every stage, x in [-2, +3]: duty within 0..kDutyLimit");
    Check(monotonic, "P2 every stage: non-decreasing in x");

    const uint32_t lo = XToDuty(-0.8f, nullptr, PowerFactor(2));
    const uint32_t hi = XToDuty(0.8f, nullptr, PowerFactor(2));
    char buf[80];
    snprintf(buf, sizeof(buf), "P3 100mW x=-0.8/+0.8 -> %u/%u (want ~4/~40)", lo, hi);
    Check(lo == 4 && hi == 40, buf);

    bool capped = true;
    for (int k = -2000; k <= 3000; ++k) {
      if (XToDuty(k * 0.001f, nullptr, 1.1f) > kDutyLimit) capped = false;
    }
    Check(capped, "P4 trim 1.1 on the 1.5W stage never exceeds kDutyLimit");
  }

  printf("\n===== %s (%d failure%s) =====\n", g_fail ? "FAILURES" : "ALL PASS",
         g_fail, g_fail == 1 ? "" : "s");
  return g_fail != 0;
}
