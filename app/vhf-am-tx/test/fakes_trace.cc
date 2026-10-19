// Same fakes, but every hardware action is appended to a trace so the ordering
// rules of spec 8.2 / 8.3 can be asserted.
#include "modulator.h"
#include "si5351a.h"
#include <vector>
#include <string>

std::vector<std::string> g_trace;

namespace transmitter_system {

Modulator::Modulator()
    : timer_(), ledc_ready_(false),
      cw_callback_(nullptr), cw_context_(nullptr),
      voice_callback_(nullptr), voice_context_(nullptr),
      tone_callback_(nullptr), tone_context_(nullptr),
      source_(ModulationSource::kSilent),
      fixed_duty_(0), output_enabled_(false),
      ramp_active_(false), ramp_request_(0), ramp_direction_(0),
      ramp_index_(0), ramp_scale_(0.0f), ramp_table_{}, dither_(0.0f),
      tick_count_(0), stat_max_cycles_(0), stat_avg_cycles_(0),
      stat_accum_cycles_(0), stat_accum_count_(0) {}
Modulator::~Modulator() {}

void Modulator::StartRampUp()   { g_trace.push_back("duty:ramp_up");   ramp_active_.store(false); }
void Modulator::StartRampDown() { g_trace.push_back("duty:ramp_down"); ramp_active_.store(false); }

void Modulator::SnapRampFull()  { g_trace.push_back("duty:snap_full"); ramp_active_.store(false); }

Si5351A::Si5351A()
    : dev_handle_(nullptr), clk_output_status_(0),
      carrier_enabled_(false), frequency_set_(true),
      current_freq_hz_(144920000), carrier_clk_(Clk::kClk0) {}
Si5351A::~Si5351A() {}

bool Si5351A::CarrierOutput(bool on) { g_trace.push_back(on ? "carrier_out:ON" : "carrier_out:OFF"); return true; }
bool Si5351A::CarrierEnable()  { g_trace.push_back("carrier:ON");  carrier_enabled_=true;  return true; }
bool Si5351A::CarrierDisable() { g_trace.push_back("carrier:OFF"); carrier_enabled_=false; return true; }

}  // namespace transmitter_system
