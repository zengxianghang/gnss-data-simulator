#ifndef GNSS_SIM_SRC_MODEL_CARRIER_TRACKING_RUNTIME_H_
#define GNSS_SIM_SRC_MODEL_CARRIER_TRACKING_RUNTIME_H_

#include "core/deterministic_rng.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_types.h"
#include "model/carrier_tracking.h"
#include "model/measurement_model.h"

#include <cstdint>
#include <string>

namespace gnss_sim {

struct CarrierTrackingRuntimeState {
    CarrierTrackingState tracking;
    DeterministicRng rng;
    SimTime last_update_time;
    std::uint64_t ambiguity_key;
    std::uint64_t phase_segment_id;
    std::int64_t adr_cycle_offset_cycles;
    bool initialized;
    bool time_initialized;
};

struct CarrierTrackingRuntimeResult {
    CarrierTrackingResult tracking;
    std::uint64_t phase_segment_id;
    std::int64_t adr_cycle_offset_cycles;
    bool cycle_slip_event;
};

CarrierTrackingConfig carrier_tracking_core_config(const CarrierTrackingReceiverConfig& config);

void initialize_carrier_tracking_runtime_state(std::uint64_t simulator_seed, int satellite_number, SignalId signal_id,
                                               CarrierTrackingRuntimeState* state);

// Hard-reset tracking/continuity state while intentionally preserving the
// independent per-signal RNG stream. No legacy/global RNG is consumed here.
void reset_carrier_tracking_runtime_state(CarrierTrackingRuntimeState* state);

// Advance one carrier tracker from its previous runtime timestamp to
// current_time. The interval is subdivided into steps no larger than the
// configured coherent integration time so output rates below 50 Hz cannot skip
// persistence/pull-in/validity timing boundaries.
bool update_carrier_tracking_runtime(const CarrierTrackingReceiverConfig& config, const SimTime& current_time,
                                     bool code_tracking, double effective_cn0_dbhz, double wavelength_m,
                                     CarrierTrackingRuntimeState* state, CarrierTrackingRuntimeResult* result,
                                     std::string* error_message);

// Apply only the receiver carrier-tracking layer. The observation passed here
// is already the clean/open-sky or urban-physical observation, including any
// environmental_range_rate_mps term. Sign convention:
//   D_measured = D_physical + tracking_error_hz
//   range_rate_measured = range_rate_physical - wavelength * tracking_error_hz
bool apply_carrier_tracking_runtime_result(const CarrierTrackingRuntimeResult& carrier,
                                           MeasurementObservation* observation, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_CARRIER_TRACKING_RUNTIME_H_
