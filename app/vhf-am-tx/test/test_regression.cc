// Host test: regressions for defects found in review. Each block failed on the
// code before its fix.
#include <cstdio>
#include <string>
#include <vector>

#include "ardf_scheduler.h"
#include "cw_keyer.h"
#include "modulator.h"
#include "si5351a.h"
#include "tone_generator.h"
#include "tx_controller.h"

using namespace transmitter_system;
int64_t g_fake_time_us = 0;
extern std::vector<std::string> g_trace;

int fails = 0;
static void Check(bool ok, const char* what) {
  printf("  %-58s %s\n", what, ok ? "OK" : "*** FAIL ***");
  if (!ok) ++fails;
}

static void Advance(TxController& tx, int ms) {
  for (int i = 0; i < ms; ++i) {
    g_fake_time_us += 1000;
    tx.Tick();
  }
}

static int CountTrace(const char* event) {
  int n = 0;
  for (auto& e : g_trace) n += (e == event);
  return n;
}

int main() {
  printf("--- keyer: Send() right after Stop() is not lost ---\n");
  {
    ToneGenerator tone;
    tone.Initialize();
    CwKeyer k;
    k.SetToneGenerator(&tone);
    k.Send("MOE");
    for (int i = 0; i < 5000; ++i) k.Tick();
    k.Stop();
    k.Send("T");  // before the ISR has seen the stop
    k.Tick();
    Check(k.IsBusy(), "keyer busy with the new string");
    bool keyed = false;
    for (int i = 0; i < 25000 && k.IsBusy(); ++i) {
      k.Tick();
      keyed |= tone.IsKeyDown();
    }
    Check(keyed, "new string actually keys the tone");
  }

  printf("\n--- keyer: Stop() cancels a Send() not yet picked up ---\n");
  {
    ToneGenerator tone;
    tone.Initialize();
    CwKeyer k;
    k.SetToneGenerator(&tone);
    k.Send("MOE");
    k.Stop();
    k.Tick();
    Check(!k.IsBusy(), "idle after Send(); Stop()");
  }

  printf("\n--- tx: RequestStart() while up leaves nothing queued ---\n");
  {
    Modulator mod;
    Si5351A syn;
    TxController tx;
    tx.SetModulator(&mod);
    tx.SetSi5351A(&syn);
    g_trace.clear();
    tx.RequestStart();
    Advance(tx, 100);
    Check(tx.GetState() == TxState::kTxReady, "carrier up");
    tx.RequestStart();  // e.g. a second caller while already transmitting
    tx.RequestStop();
    Advance(tx, 200);
    Advance(tx, 200);
    Check(tx.GetState() == TxState::kIdle, "stays idle after the stop");
    Check(CountTrace("carrier:ON") == 1, "carrier came up exactly once");
  }

  printf("\n--- tx: RequestStop() cancels a queued start ---\n");
  {
    Modulator mod;
    Si5351A syn;
    TxController tx;
    tx.SetModulator(&mod);
    tx.SetSi5351A(&syn);
    g_trace.clear();
    tx.RequestStart();
    tx.RequestStop();  // PTT tapped faster than one controller tick
    Advance(tx, 200);
    Check(tx.GetState() == TxState::kIdle, "idle");
    Check(CountTrace("carrier:ON") == 0, "carrier never came up");
  }

  printf("\n--- ardf: start dropped by the battery inhibit is retried ---\n");
  {
    Modulator mod;
    Si5351A syn;
    TxController tx;
    tx.SetModulator(&mod);
    tx.SetSi5351A(&syn);
    ToneGenerator tone;
    tone.Initialize();
    CwKeyer k;
    k.SetToneGenerator(&tone);
    ArdfScheduler sch;
    sch.SetTxController(&tx);
    sch.SetCwKeyer(&k);
    sch.SetArdfId(morse::ArdfId::kMoe);  // slot 1: 0:00-1:00
    sch.SetCallsign("JL1HMT");

    g_fake_time_us = 0;
    sch.Start();
    // Main loop order: controller, then scheduler. The scheduler asks for a
    // start; the inhibit trips before the controller acts on it.
    tx.Tick();
    sch.Tick();
    tx.SetInhibited(true);
    bool transmitted = false;
    for (int ms = 0; ms < 30000; ++ms) {
      if (ms == 1000) tx.SetInhibited(false);  // battery recovers
      g_fake_time_us += 1000;
      tx.Tick();
      sch.Tick();
      for (int i = 0; i < 25; ++i) k.Tick();
      transmitted |= tx.IsTransmitting();
    }
    Check(transmitted, "transmits once the inhibit clears, same slot");
    sch.Stop();
  }

  printf("\n--- ardf: no callsign, no run (spec 10.5) ---\n");
  {
    Modulator mod;
    Si5351A syn;
    TxController tx;
    tx.SetModulator(&mod);
    tx.SetSi5351A(&syn);
    ArdfScheduler sch;
    sch.SetTxController(&tx);
    Check(!sch.HasCallsign(), "unset by default");
    Check(!sch.Start(), "Start() refused");
    Check(!sch.IsRunning() && !tx.IsArdfRunning(), "not running");
    sch.SetCallsign("JL1HMT");
    Check(sch.Start(), "starts once a callsign is set");
    sch.Stop();
  }

  printf("\n===== %s (%d failures) =====\n", fails ? "FAILURES" : "ALL PASS",
         fails);
  return fails ? 1 : 0;
}
