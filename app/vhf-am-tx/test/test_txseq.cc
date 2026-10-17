// Host test: transmit start/stop sequencing (spec 8.2 / 8.3).
//
// The stop order is the safety-critical part: killing the carrier before the
// drive has ramped down dumps 1.5W instantly and splatters the sidebands, and
// leaves the modulation node floating at 3.6V because the source follower
// cannot sink current. This test exists to make that regression loud.
#include <cstdio>
#include <vector>
#include <string>
#include "tx_controller.h"
#include "modulator.h"
#include "si5351a.h"

using namespace transmitter_system;
int64_t g_fake_time_us = 0;
extern std::vector<std::string> g_trace;

static void Advance(TxController& tx, int ms) {
    for (int i = 0; i < ms; ++i) { g_fake_time_us += 1000; tx.Tick(); }
}
static const char* Name(TxState s) {
    switch (s) { case TxState::kIdle:return "IDLE"; case TxState::kTxStarting:return "TX_STARTING";
        case TxState::kTxReady:return "TX_READY"; case TxState::kTxKeyed:return "TX_KEYED";
        case TxState::kTxStopping:return "TX_STOPPING"; case TxState::kFault:return "FAULT"; }
    return "?";
}

int main() {
    int failures = 0;

    // --- 1. Start sequence -------------------------------------------------
    printf("--- start sequence (spec 8.2) ---\n");
    {
        Modulator mod; Si5351A syn; TxController tx;
        tx.SetModulator(&mod); tx.SetSi5351A(&syn);
        g_trace.clear(); g_fake_time_us = 0;

        printf("  initial state: %s\n", Name(tx.GetState()));
        tx.RequestStart();
        Advance(tx, 100);
        printf("  after start:   %s\n", Name(tx.GetState()));
        printf("  trace: ");
        for (auto& e : g_trace) printf("%s ", e.c_str());
        printf("\n");

        // Carrier must come up BEFORE the duty ramps (duty stays 0 while the
        // PLL settles, so nothing radiates during the wait).
        int ci = -1, di = -1;
        for (size_t i = 0; i < g_trace.size(); ++i) {
            if (g_trace[i] == "carrier:ON" && ci < 0) ci = i;
            if (g_trace[i] == "duty:ramp_up" && di < 0) di = i;
        }
        const bool ok = ci >= 0 && di >= 0 && ci < di;
        printf("  carrier before duty ramp: %s\n", ok ? "OK" : "MISMATCH");
        if (!ok) ++failures;
        if (tx.GetState() != TxState::kTxReady) {
            printf("  expected TX_READY, got %s  MISMATCH\n", Name(tx.GetState())); ++failures;
        }
    }

    // --- 2. Stop sequence: ORDER IS SAFETY CRITICAL -----------------------
    printf("\n--- stop sequence (spec 8.3) ---\n");
    {
        Modulator mod; Si5351A syn; TxController tx;
        tx.SetModulator(&mod); tx.SetSi5351A(&syn);
        g_fake_time_us = 0;
        tx.RequestStart(); Advance(tx, 100);
        g_trace.clear();
        tx.RequestStop(); Advance(tx, 200);

        printf("  after stop:    %s\n", Name(tx.GetState()));
        printf("  trace: ");
        for (auto& e : g_trace) printf("%s ", e.c_str());
        printf("\n");

        int di = -1, ci = -1;
        for (size_t i = 0; i < g_trace.size(); ++i) {
            if (g_trace[i] == "duty:ramp_down" && di < 0) di = i;
            if (g_trace[i] == "carrier:OFF" && ci < 0) ci = i;
        }
        const bool ok = di >= 0 && ci >= 0 && di < ci;
        printf("  duty ramps down BEFORE carrier off: %s\n", ok ? "OK" : "*** WRONG ORDER ***");
        if (!ok) ++failures;
        if (tx.GetState() != TxState::kIdle) {
            printf("  expected IDLE, got %s  MISMATCH\n", Name(tx.GetState())); ++failures;
        }
    }

    // --- 3. Settle delay before the carrier is cut ------------------------
    printf("\n--- settle delay (spec 8.3 step 2, 10ms) ---\n");
    {
        Modulator mod; Si5351A syn; TxController tx;
        tx.SetModulator(&mod); tx.SetSi5351A(&syn);
        g_fake_time_us = 0;
        tx.RequestStart(); Advance(tx, 100);
        tx.RequestStop();
        // one Tick to enter TX_STOPPING, then check the carrier is still up
        Advance(tx, 1);
        const bool entered_stopping = tx.GetState() == TxState::kTxStopping;
        Advance(tx, 5);
        const bool still_stopping = tx.GetState() == TxState::kTxStopping;
        Advance(tx, 20);
        const bool done = tx.GetState() == TxState::kIdle;
        printf("  at  +1ms: %s\n", entered_stopping?"TX_STOPPING OK":"not stopping MISMATCH");
        printf("  at  +6ms: %s\n", still_stopping?"TX_STOPPING OK":"left early MISMATCH");
        printf("  at +26ms: %s\n", done?"IDLE OK":"still stopping MISMATCH");
        if (!entered_stopping || !still_stopping || !done) ++failures;
    }

    // --- 4. Battery inhibit blocks a start --------------------------------
    printf("\n--- battery inhibit (spec 3.6) ---\n");
    {
        Modulator mod; Si5351A syn; TxController tx;
        tx.SetModulator(&mod); tx.SetSi5351A(&syn);
        g_fake_time_us = 0;
        tx.SetInhibited(true);
        const bool refused = !tx.RequestStart();
        Advance(tx, 50);
        printf("  RequestStart refused: %s\n", refused?"OK":"MISMATCH");
        printf("  state stays IDLE:     %s\n", tx.GetState()==TxState::kIdle?"OK":"MISMATCH");
        if (!refused || tx.GetState()!=TxState::kIdle) ++failures;

        // And it stops an over already in progress
        tx.SetInhibited(false);
        tx.RequestStart(); Advance(tx, 100);
        const bool tx_up = tx.IsTransmitting();
        tx.SetInhibited(true); Advance(tx, 200);
        printf("  running over stopped: %s\n",
               (tx_up && tx.GetState()==TxState::kIdle)?"OK":"MISMATCH");
        if (!(tx_up && tx.GetState()==TxState::kIdle)) ++failures;
    }

    // --- 5. FAULT is terminal and safe ------------------------------------
    printf("\n--- FAULT (spec 8.6) ---\n");
    {
        Modulator mod; Si5351A syn; TxController tx;
        tx.SetModulator(&mod); tx.SetSi5351A(&syn);
        g_fake_time_us = 0;
        tx.RequestStart(); Advance(tx, 100);
        g_trace.clear();
        tx.EnterFault("test");
        printf("  trace on fault: ");
        for (auto& e : g_trace) printf("%s ", e.c_str());
        printf("\n");
        const bool carrier_off = std::find(g_trace.begin(), g_trace.end(),
                                           std::string("carrier:OFF")) != g_trace.end();
        printf("  carrier cut:    %s\n", carrier_off?"OK":"MISMATCH");
        if (!carrier_off) ++failures;

        // Must not be recoverable
        tx.RequestStart(); Advance(tx, 200);
        const bool stuck = tx.GetState() == TxState::kFault;
        printf("  stays in FAULT: %s\n", stuck?"OK":"MISMATCH (recovered!)");
        if (!stuck) ++failures;
    }

    printf("\n===== %s (%d failure%s) =====\n",
           failures?"FAILURES":"ALL PASS", failures, failures==1?"":"s");
    return failures != 0;
}
