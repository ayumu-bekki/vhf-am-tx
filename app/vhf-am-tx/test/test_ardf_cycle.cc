// Host test: the ARDF run must transmit again in the next cycle.
//
// A real run lasts many five minute cycles, but the first slot is the only one
// anybody watches on the bench. This drives scheduler, TX controller and keyer
// together for several cycles with simulated time and counts how many times
// the carrier actually comes up.
#include <cstdio>
#include <string>
#include <vector>

#include "ardf_scheduler.h"
#include "cw_keyer.h"
#include "modulator.h"
#include "morse.h"
#include "si5351a.h"
#include "tone_generator.h"
#include "tx_controller.h"

using namespace transmitter_system;
int64_t g_fake_time_us = 0;
extern std::vector<std::string> g_trace;

int main() {
  int failures = 0;
  printf("--- MOE over 3 cycles: carrier must come up once per cycle ---\n");

  Modulator mod;
  Si5351A syn;
  TxController tx;
  tx.SetModulator(&mod);
  tx.SetSi5351A(&syn);
  CwKeyer keyer;
  ToneGenerator tone;
  tone.Initialize();
  keyer.SetToneGenerator(&tone);
  keyer.SetWpm(8);
  ArdfScheduler sch;
  sch.SetTxController(&tx);
  sch.SetCwKeyer(&keyer);
  sch.SetArdfId(morse::ArdfId::kMoe);
  sch.SetCallsign("JL1HMT");

  g_trace.clear();
  g_fake_time_us = 0;
  sch.Start();

  int overrun_ticks = 0;

  // 1ms steps; the keyer runs at 25kHz, so 25 samples per step.
  for (int ms = 0; ms < 3 * 300 * 1000 - 1000; ++ms) {
    g_fake_time_us += 1000;
    for (int i = 0; i < 25; ++i) keyer.Tick();
    // The real main loop runs tx_.Tick() then ardf_.Tick() every 100ms.
    if (ms % 100 == 0) {
      tx.Tick();
      sch.Tick();
      // Slot 1 is the first 60s of every 300s cycle. Note when the carrier is
      // up outside it: that would be on the air over another station's slot.
      const double in_cycle = (g_fake_time_us % 300000000) / 1e6;
      if (tx.IsTransmitting() && in_cycle >= 60.0) overrun_ticks++;
    }
  }

  int carrier_on = 0;
  for (auto& e : g_trace) {
    if (e == "carrier:ON") ++carrier_on;
  }
  printf("  carrier came up %d time(s) (expect 3)\n", carrier_on);
  if (carrier_on != 3) {
    printf("  MISMATCH\n");
    ++failures;
  } else {
    printf("  OK\n");
  }

  printf("  ticks with the carrier up outside slot 1: %d (expect 0)\n",
         overrun_ticks);
  if (overrun_ticks != 0) {
    printf("  MISMATCH\n");
    ++failures;
  } else {
    printf("  OK\n");
  }

  printf("\n===== %s (%d failure%s) =====\n", failures ? "FAILURES" : "ALL PASS",
         failures, failures == 1 ? "" : "s");
  return failures != 0;
}
