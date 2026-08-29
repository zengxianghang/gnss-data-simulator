#ifndef GNSS_SIM_SRC_MODEL_MEASUREMENT_ERROR_MODEL_H_
#define GNSS_SIM_SRC_MODEL_MEASUREMENT_ERROR_MODEL_H_

#include "core/deterministic_rng.h"
#include "gnss_sim/sim_config.h"
#include "model/measurement_model.h"
#include "model/signal_tracking.h"

#include <cstdint>
#include <string>

namespace gnss_sim {

enum class MeasurementErrorPhase {
    kStable,
    kTtffHot,
    kTtffWarm,
    kTtffCold,
    kReaReacquisition,
    kReaFadeOut,
};

struct MeasurementErrorContext {
    MeasurementErrorPhase phase;
    double rea_fade_progress;
};

struct MeasurementErrorSigmas {
    double psr_sigma_m;
    double doppler_sigma_mps;
    double adr_sigma_m;
    double cn0_sigma_dbhz;
    double cn0_drop_db;
};

struct MeasurementErrorState {
    SignalId signal_id;
    int satellite_number;
    SimTime tracking_start_time;
    std::int64_t last_lock_time_ns;
    DeterministicRng rng;
    double psr_correlated_error_m;
    double spare_gaussian;
    bool have_spare_gaussian;
    bool initialized;
};

void reset_measurement_error_state(MeasurementErrorState* state);

bool compute_measurement_error_sigmas(const MeasurementErrorConfig& config, const MeasurementErrorContext& context,
                                      std::int64_t lock_time_ns, MeasurementErrorSigmas* sigmas,
                                      std::string* error_message);

bool apply_measurement_error(const MeasurementErrorConfig& config, std::uint64_t run_seed,
                             const MeasurementErrorContext& context, const SignalTracker& tracker,
                             const MeasurementObservation& zero_noise_observation, MeasurementErrorState* state,
                             MeasurementObservation* reported_observation, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_MEASUREMENT_ERROR_MODEL_H_
