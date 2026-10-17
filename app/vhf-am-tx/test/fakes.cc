// Stand-ins for the hardware-touching methods, so TxController's sequencing
// can be exercised. The ramp completes instantly and the carrier always
// succeeds, which is the "everything works" case we want to walk through.
#include "modulator.h"
#include "si5351a.h"

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
void Modulator::StartRampUp()  { ramp_active_.store(false); }
void Modulator::StartRampDown(){ ramp_active_.store(false); }

bool Si5351A::CarrierEnable()  { carrier_enabled_ = true;  return true; }
bool Si5351A::CarrierDisable() { carrier_enabled_ = false; return true; }

}  // namespace transmitter_system
