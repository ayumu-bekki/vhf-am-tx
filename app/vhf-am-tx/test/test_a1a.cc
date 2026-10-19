// Host test: A1A keying (spec 5.3 / 10.3) and the CW mode hand key.
//
// The keyer is driven one 25kHz tick at a time, like the ISR does. The CLK0
// requests it raises are logged with the tick they came on, so the 1ms lead
// and the 10ms tail can be checked to the tick.
#include <cmath>
#include <cstdio>
#include <vector>

#include "cw_keyer.h"
#include "emission.h"
#include "modulator.h"
#include "si5351a.h"
#include "tone_generator.h"
#include "tx_controller.h"

using namespace transmitter_system;

int64_t g_fake_time_us = 0;
extern std::vector<std::string> g_trace;

static int g_fail = 0;
static void Check(bool ok, const char* what) {
  printf("  %-66s %s\n", what, ok ? "OK" : "MISMATCH");
  if (!ok) ++g_fail;
}

struct Req { uint32_t tick; bool on; };
static uint32_t g_tick = 0;
static std::vector<Req> g_reqs;
static void OnClk(void*, bool on) { g_reqs.push_back({g_tick, on}); }

struct Run {
  std::vector<float> x;
  std::vector<uint32_t> down_start;  // tick where the key went down
  std::vector<uint32_t> silent_from; // tick where the envelope reached zero
  std::vector<uint32_t> down_len;    // key-down length in ticks
};

static Run Drive(uint8_t wpm, const char* text, uint32_t ticks) {
  ToneGenerator tone;
  tone.Initialize();
  tone.SetKeyingMode(ToneGenerator::KeyingMode::kA1a);
  CwKeyer keyer;
  keyer.SetToneGenerator(&tone);
  keyer.SetWpm(wpm);
  keyer.SetClkRequestCallback(&OnClk, nullptr);
  g_reqs.clear();
  keyer.Send(text);

  Run r;
  bool prev_down = false, prev_silent = true;
  uint32_t down_at = 0;
  for (g_tick = 0; g_tick < ticks; ++g_tick) {
    r.x.push_back(keyer.Tick());
    const bool down = tone.IsKeyDown();
    if (down && !prev_down) { r.down_start.push_back(g_tick); down_at = g_tick; }
    if (!down && prev_down) r.down_len.push_back(g_tick - down_at);
    const bool silent = tone.IsSilent();
    if (silent && !prev_silent) r.silent_from.push_back(g_tick);
    prev_down = down;
    prev_silent = silent;
  }
  return r;
}

int main() {
  printf("--- A1: A1A keyer, one dit (15 WPM) ---\n");
  {
    const uint32_t unit = 1200u * 25000u / (15u * 1000u);  // 2000 samples
    Run r = Drive(15, "E", 25000);
    Check(r.down_start.size() == 1, "one key-down");
    Check(r.down_len.size() == 1 && r.down_len[0] == unit,
          "key-down lasts exactly one unit (same as A2A)");
    const uint32_t s = r.down_start[0];
    Check(r.x[0] == -1.0f && r.x[s - 1] == -1.0f, "x = -1 before the key goes down");
    Check(std::fabs(r.x[s + 124]) < 1e-6f, "x reaches 0 after the 5ms (125 tick) edge");
    Check(std::fabs(r.x[s + 124 + 100]) < 1e-6f, "x holds 0 while the key is down");
    const uint32_t up = s + unit;
    Check(std::fabs(r.x[up - 1]) < 1e-6f, "x still 0 on the last key-down tick");
    Check(r.x[up + 124] == -1.0f, "x back at -1 after the 5ms (125 tick) edge");
    float lo = 0, hi = -1;
    for (float v : r.x) { if (v < lo) lo = v; if (v > hi) hi = v; }
    Check(lo >= -1.0f && hi <= 1e-6f, "x stays within -1..0");
    bool mono_up = true;
    for (uint32_t i = s; i < s + 125; ++i) if (r.x[i + 1] < r.x[i]) mono_up = false;
    Check(mono_up, "rising edge is monotonic (raised cosine)");
  }

  for (uint8_t wpm : {15, 40}) {
    printf("\n--- A2: CLK0 requests at %u WPM (\"EEE T\") ---\n", wpm);
    Run r = Drive(wpm, "EEE T", 25000 * 4);
    char buf[96];
    bool lead_ok = true, tail_ok = true, alt_ok = true, in_gap = true;
    size_t on_i = 0, off_i = 0;
    for (size_t i = 0; i < g_reqs.size(); ++i) {
      if (i > 0 && g_reqs[i].on == g_reqs[i - 1].on) alt_ok = false;
      if (g_reqs[i].on) {
        // Next key-down after this request.
        uint32_t next = 0;
        for (uint32_t d : r.down_start) if (d >= g_reqs[i].tick) { next = d; break; }
        const int lead = (int)next - (int)g_reqs[i].tick;
        if (lead < 24 || lead > 26) lead_ok = false;
        ++on_i;
      } else {
        // Latest envelope end before this request.
        uint32_t last = 0;
        for (uint32_t s : r.silent_from) if (s <= g_reqs[i].tick) last = s;
        const int tail = (int)g_reqs[i].tick - (int)last;
        if (tail < 249 || tail > 251) tail_ok = false;
        ++off_i;
      }
    }
    // Every OFF must come before the next ON and every request sit in a gap.
    for (size_t i = 0; i + 1 < g_reqs.size(); ++i) {
      if (g_reqs[i].tick >= g_reqs[i + 1].tick) in_gap = false;
    }
    snprintf(buf, sizeof(buf), "%zu ON / %zu OFF requests, alternating", on_i, off_i);
    Check(alt_ok && on_i == off_i && on_i >= 4, buf);
    Check(lead_ok, "ON comes 1ms (25 ticks) before the key-down edge starts");
    Check(tail_ok, "OFF comes 10ms (250 ticks) after the key-up edge ends");
    Check(in_gap, "ON and OFF never overlap or reorder (fits the shortest gap)");
    Check(!g_reqs.empty() && !g_reqs.back().on, "CLK0 is off at the end");
  }

  printf("\n--- A2: A2A keeper never asks for CLK0 ---\n");
  {
    ToneGenerator tone;
    tone.Initialize();
    CwKeyer keyer;
    keyer.SetToneGenerator(&tone);
    keyer.SetClkRequestCallback(&OnClk, nullptr);
    g_reqs.clear();
    keyer.Send("EE");
    for (g_tick = 0; g_tick < 25000; ++g_tick) keyer.Tick();
    Check(g_reqs.empty() && !keyer.WantsClkOutput(), "no requests in A2A");
  }

  printf("\n--- A3: A1A slot start (spec 8.2) ---\n");
  {
    Modulator mod; Si5351A syn; ToneGenerator tone; TxController tx;
    tx.SetModulator(&mod); tx.SetSi5351A(&syn); tx.SetToneGenerator(&tone);
    tx.SetEmission(Emission::kA1a);
    tx.SetKeyedSource(ModulationSource::kCwKeyer);
    g_trace.clear(); g_fake_time_us = 0;
    tx.RequestStart();
    for (int i = 0; i < 100; ++i) { g_fake_time_us += 1000; tx.Tick(); }
    bool ramp = false, snap = false, off_after_on = false, saw_on = false;
    for (auto& e : g_trace) {
      if (e == "duty:ramp_up") ramp = true;
      if (e == "duty:snap_full") snap = true;
      if (e == "carrier:ON") saw_on = true;
      if (e == "carrier_out:OFF" && saw_on) off_after_on = true;
    }
    Check(tx.GetState() == TxState::kTxReady, "reaches TX_READY");
    Check(!ramp && snap, "no duty ramp: x stays -1");
    Check(off_after_on, "carrier_output(false) right after the PLL lock");
    Check(tone.GetKeyingMode() == ToneGenerator::KeyingMode::kA1a, "keyer in A1A mode");

    // A2A control: the old path is untouched.
    Modulator mod2; Si5351A syn2; ToneGenerator tone2; TxController tx2;
    tx2.SetModulator(&mod2); tx2.SetSi5351A(&syn2); tx2.SetToneGenerator(&tone2);
    tx2.SetEmission(Emission::kA2a);
    g_trace.clear();
    tx2.RequestStart();
    for (int i = 0; i < 100; ++i) { g_fake_time_us += 1000; tx2.Tick(); }
    bool ramp2 = false, out2 = false;
    for (auto& e : g_trace) {
      if (e == "duty:ramp_up") ramp2 = true;
      if (e.rfind("carrier_out", 0) == 0) out2 = true;
    }
    Check(ramp2 && !out2, "A2A still ramps up and never switches CLK0");

    // Only the CW keyer source is A1A: a carrier-only test stays a carrier.
    Modulator mod3; Si5351A syn3; ToneGenerator tone3; TxController tx3;
    tx3.SetModulator(&mod3); tx3.SetSi5351A(&syn3); tx3.SetToneGenerator(&tone3);
    tx3.SetEmission(Emission::kA1a);
    tx3.SetKeyedSource(ModulationSource::kSilent);
    g_trace.clear();
    tx3.RequestStart();
    for (int i = 0; i < 100; ++i) { g_fake_time_us += 1000; tx3.Tick(); }
    bool ramp3 = false;
    for (auto& e : g_trace) if (e == "duty:ramp_up") ramp3 = true;
    Check(ramp3, "carrier-only source ignores the emission");
  }

  printf("\n--- A5: CW mode hand key, A1A (SW1 held) ---\n");
  {
    ToneGenerator tone;
    tone.Initialize();
    tone.SetKeyingMode(ToneGenerator::KeyingMode::kA1a);
    CwKeyer keyer;
    keyer.SetToneGenerator(&tone);
    keyer.SetClkRequestCallback(&OnClk, nullptr);
    g_reqs.clear();
    std::vector<float> x;
    for (int i = 0; i < 100; ++i) x.push_back(keyer.Tick());
    keyer.SetManualKey(true);
    for (int i = 0; i < 1000; ++i) x.push_back(keyer.Tick());
    keyer.SetManualKey(false);
    for (int i = 0; i < 300; ++i) x.push_back(keyer.Tick());
    Check(x[99] == -1.0f, "key up: x = -1");
    Check(std::fabs(x[100 + 124]) < 1e-6f && std::fabs(x[1099]) < 1e-6f,
          "held: x rises to 0 in 5ms and stays at 0 for as long as it is held");
    Check(x[1100 + 124] == -1.0f, "released: x falls back to -1 in 5ms");
    Check(g_reqs.empty(), "the keyer itself does not switch CLK0 for the hand key");
  }

  printf("\n--- A5: CW mode hand key, sequencing ---\n");
  for (int a1a = 1; a1a >= 1; --a1a) {
    Modulator mod; Si5351A syn; ToneGenerator tone; CwKeyer keyer; TxController tx;
    tone.Initialize();
    keyer.SetToneGenerator(&tone);
    tx.SetModulator(&mod); tx.SetSi5351A(&syn); tx.SetToneGenerator(&tone);
    tx.SetCwKeyer(&keyer);
    tx.SetEmission(a1a ? Emission::kA1a : Emission::kA2a);
    tx.SetKeyedSource(ModulationSource::kCwKeyer);
    tx.SetManualKeying(true);
    g_trace.clear(); g_fake_time_us = 1000000;
    tx.RequestStart();
    tx.SetKeyed(true);
    int keyed_at = -1, clk_on_at = -1;
    for (int i = 0; i < 200; ++i) {
      g_fake_time_us += 1000;
      tx.Tick();
      if (clk_on_at < 0) for (auto& e : g_trace) if (e == "carrier_out:ON") clk_on_at = i;
      if (keyed_at < 0 && tx.GetState() == TxState::kTxKeyed) keyed_at = i;
    }
    printf("  %s: keyed at %d ms, CLK0 on at %d ms\n", a1a ? "A1A" : "A2A", keyed_at, clk_on_at);
    Check(tx.GetState() == TxState::kTxKeyed, "SW1 held: TX_KEYED");
    if (a1a) {
      Check(clk_on_at >= 0 && keyed_at - clk_on_at >= (int)hardware_config::kA1aClkLeadMs,
            "A1A: CLK0 switched on at least 1ms before the key goes down");
    } else {
      Check(clk_on_at < 0, "A2A: CLK0 is never switched");
    }
    keyer.Tick();
    tx.SetKeyed(false);
    tx.RequestStop();
    for (int i = 0; i < 100; ++i) { g_fake_time_us += 1000; tx.Tick(); }
    Check(tx.GetState() == TxState::kIdle, "SW1 released: stopped (ramp down, settle, carrier off)");
    Check(g_trace.back() == "carrier:OFF", "CLK0 stopped last");
  }

  printf("\n--- A5: CW mode hand key, A1A (SW1 held) ---\n");
  {
    ToneGenerator tone;
    tone.Initialize();
    tone.SetKeyingMode(ToneGenerator::KeyingMode::kA1a);
    CwKeyer keyer;
    keyer.SetToneGenerator(&tone);
    keyer.SetClkRequestCallback(&OnClk, nullptr);
    g_reqs.clear();
    std::vector<float> x;
    for (int i = 0; i < 100; ++i) x.push_back(keyer.Tick());
    keyer.SetManualKey(true);
    for (int i = 0; i < 1000; ++i) x.push_back(keyer.Tick());
    keyer.SetManualKey(false);
    for (int i = 0; i < 300; ++i) x.push_back(keyer.Tick());
    Check(x[99] == -1.0f, "key up: x = -1");
    Check(std::fabs(x[100 + 124]) < 1e-6f && std::fabs(x[1099]) < 1e-6f,
          "held: x rises to 0 in 5ms and stays at 0 for as long as it is held");
    Check(x[1100 + 124] == -1.0f, "released: x falls back to -1 in 5ms");
    Check(g_reqs.empty(), "the keyer itself does not switch CLK0 for the hand key");
  }

  printf("\n--- A5: CW mode hand key, sequencing ---\n");
  for (int a1a = 1; a1a >= 1; --a1a) {
    Modulator mod; Si5351A syn; ToneGenerator tone; CwKeyer keyer; TxController tx;
    tone.Initialize();
    keyer.SetToneGenerator(&tone);
    tx.SetModulator(&mod); tx.SetSi5351A(&syn); tx.SetToneGenerator(&tone);
    tx.SetCwKeyer(&keyer);
    tx.SetEmission(a1a ? Emission::kA1a : Emission::kA2a);
    tx.SetKeyedSource(ModulationSource::kCwKeyer);
    tx.SetManualKeying(true);
    g_trace.clear(); g_fake_time_us = 1000000;
    tx.RequestStart();
    tx.SetKeyed(true);
    int keyed_at = -1, clk_on_at = -1;
    for (int i = 0; i < 200; ++i) {
      g_fake_time_us += 1000;
      tx.Tick();
      if (clk_on_at < 0) for (auto& e : g_trace) if (e == "carrier_out:ON") clk_on_at = i;
      if (keyed_at < 0 && tx.GetState() == TxState::kTxKeyed) keyed_at = i;
    }
    printf("  %s: keyed at %d ms, CLK0 on at %d ms\n", a1a ? "A1A" : "A2A", keyed_at, clk_on_at);
    Check(tx.GetState() == TxState::kTxKeyed, "SW1 held: TX_KEYED");
    if (a1a) {
      Check(clk_on_at >= 0 && keyed_at - clk_on_at >= (int)hardware_config::kA1aClkLeadMs,
            "A1A: CLK0 switched on at least 1ms before the key goes down");
    } else {
      Check(clk_on_at < 0, "A2A: CLK0 is never switched");
    }
    tx.SetKeyed(false);
    tx.RequestStop();
    for (int i = 0; i < 100; ++i) { g_fake_time_us += 1000; tx.Tick(); }
    Check(tx.GetState() == TxState::kIdle, "SW1 released: stopped (ramp down, settle, carrier off)");
    Check(g_trace.back() == "carrier:OFF", "CLK0 stopped last");
  }

  printf("\n===== %s (%d failure%s) =====\n", g_fail ? "FAILURES" : "ALL PASS",
         g_fail, g_fail == 1 ? "" : "s");
  return g_fail != 0;
}
