#ifndef GNSS_SIM_SRC_MODEL_MEASUREMENT_MODEL_H_
#define GNSS_SIM_SRC_MODEL_MEASUREMENT_MODEL_H_

#include "gnss/rtklib_adapter.h"
#include "gnss/satellite_engine.h"
#include "gnss/signal_definitions.h"
#include "model/atmosphere_model.h"
#include "model/signal_tracking.h"

#include <cstdint>
#include <string>

namespace gnss_sim {

enum class BroadcastCodeBiasStatus {
    kApplied,
    kNoCorrection,
    kUnavailableForMessageFamily,
};

struct CarrierAmbiguityState {
    SignalId signal_id;
    int satellite_number;
    SimTime tracking_start_time;
    std::int64_t ambiguity_cycles;
    bool initialized;
};

struct MeasurementObservation {
    SignalId signal_id;
    int satellite_number;
    int glonass_fcn;
    double wavelength_m;
    double geometric_range_m;
    double range_rate_mps;
    double satellite_clock_bias_m;
    double satellite_clock_drift_mps;
    RtklibBroadcastMessageFamily broadcast_message_family;
    double tgd_sec[4];
    double isc_sec[6];
    double glonass_dtaun_sec;
    double code_bias_m;
    double ionosphere_code_delay_m;
    double troposphere_delay_m;
    double pseudorange_m;
    double doppler_hz;
    double adr_cycles;
    double cn0_dbhz;
    std::int64_t lock_time_ns;
    std::int64_t ambiguity_cycles;
    BroadcastCodeBiasStatus code_bias_status;
    bool observation_available;
    bool pseudorange_valid;
    bool doppler_valid;
    bool adr_valid;
};

void reset_carrier_ambiguity_state(CarrierAmbiguityState* state);

bool compute_broadcast_code_bias_m(const SignalDefinition& signal, const RtklibBroadcastBiasData& bias_data,
                                   double* code_bias_m, BroadcastCodeBiasStatus* status, std::string* error_message);

bool generate_zero_noise_measurement(const RtklibNavStore* nav_store, const SatelliteGeometry& geometry,
                                     const ReceiverTruth& receiver, const SignalTracker& tracker,
                                     const AtmosphereCorrection& atmosphere, CarrierAmbiguityState* ambiguity_state,
                                     MeasurementObservation* observation, std::string* error_message);

bool generate_zero_noise_measurement_with_explicit_code_bias(
    const SatelliteGeometry& geometry, const ReceiverTruth& receiver, const SignalTracker& tracker,
    const AtmosphereCorrection& atmosphere, double code_bias_m, CarrierAmbiguityState* ambiguity_state,
    MeasurementObservation* observation, std::string* error_message);

const char* broadcast_code_bias_status_name(BroadcastCodeBiasStatus status);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_MEASUREMENT_MODEL_H_
