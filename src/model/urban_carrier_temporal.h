#ifndef GNSS_SIM_SRC_MODEL_URBAN_CARRIER_TEMPORAL_H_
#define GNSS_SIM_SRC_MODEL_URBAN_CARRIER_TEMPORAL_H_

#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_time.h"
#include "model/urban_signal_epoch.h"

#include <string>

namespace gnss_sim {

struct UrbanCarrierTemporalState {
    SignalId signal_id;
    int glonass_fcn;
    SimTime previous_time;
    double previous_wrapped_phase_rad;
    double unwrapped_phase_rad;
    double previous_carrier_range_bias_m;
    bool configured;
    bool continuity_active;
    bool ever_had_lock;
};

struct UrbanCarrierTemporalResult {
    double wavelength_m;
    double wrapped_phase_rad;
    double unwrapped_phase_rad;
    double carrier_range_bias_m;
    double environmental_range_rate_mps;
    bool tracking_lock_valid;
    bool carrier_adr_valid;
    bool phase_continuity_valid;
    bool environmental_range_rate_valid;
    bool cycle_slip_event;
};

void reset_urban_carrier_temporal_state(UrbanCarrierTemporalState* state);

// Consume the accepted #140 receiver-domain composite phasor. With the project
// exp(+jwt)/exp(-jkL) convention, equivalent environmental carrier range is
// -phase_unwrapped*lambda/(2*pi). Range rate is only formed between consecutive
// samples in one uninterrupted tracking segment.
bool update_urban_carrier_temporal_state(const SignalDefinition& signal, int glonass_fcn, const SimTime& current_time,
                                         const UrbanSignalEpochResult& epoch, UrbanCarrierTemporalState* state,
                                         UrbanCarrierTemporalResult* result, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_URBAN_CARRIER_TEMPORAL_H_
