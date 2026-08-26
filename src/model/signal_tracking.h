#ifndef GNSS_SIM_SRC_MODEL_SIGNAL_TRACKING_H_
#define GNSS_SIM_SRC_MODEL_SIGNAL_TRACKING_H_

#include "core/deterministic_rng.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_types.h"
#include "model/cn0_model.h"

#include <cstdint>
#include <string>

namespace gnss_sim {

enum class SignalTrackingPhase {
    kSignalOff,
    kSearching,
    kAcquiring,
    kTracking,
};

enum class AcquisitionContext {
    kHot,
    kWarm,
    kCold,
    kReacquisition,
};

struct DelayDistribution {
    std::int64_t minimum_ns;
    std::int64_t p50_ns;
    std::int64_t p95_ns;
    std::int64_t maximum_ns;
};

struct SignalTrackingModelConfig {
    DelayDistribution hot_common_startup;
    DelayDistribution warm_common_startup;
    DelayDistribution warm_search_uncertainty;
    DelayDistribution hot_signal_acquisition[4];
    DelayDistribution reacquisition[4];
    std::int64_t psr_valid_delay_ns;
    std::int64_t doppler_valid_delay_ns;
    std::int64_t adr_valid_delay_ns;
};

struct ReceiverStartupTiming {
    StartupMode startup_mode;
    std::int64_t common_startup_delay_ns;
    std::int64_t search_uncertainty_delay_ns;
    std::int64_t total_search_ready_delay_ns;
};

struct SignalTracker {
    SignalId signal_id;
    SignalTrackingPhase phase;
    AcquisitionContext acquisition_context;
    SimTime state_since;
    SimTime search_ready_time;
    SimTime acquisition_complete_time;
    SimTime tracking_start_time;
    SimTime psr_valid_time;
    SimTime doppler_valid_time;
    SimTime adr_valid_time;
    double cn0_dbhz;
    std::int64_t lock_time_ns;
    bool scheduled;
    bool psr_valid;
    bool doppler_valid;
    bool adr_valid;
    bool observation_available;
};

SignalTrackingModelConfig default_signal_tracking_model_config();
bool validate_signal_tracking_model_config(const SignalTrackingModelConfig& config, std::string* error_message);

bool sample_receiver_startup_timing(StartupMode startup_mode, const SignalTrackingModelConfig& config,
                                    DeterministicRng* rng, ReceiverStartupTiming* timing);

void reset_signal_tracker(SignalTracker* tracker, SignalId signal_id, const SimTime& reset_time);
bool schedule_signal_acquisition(SignalTracker* tracker, AcquisitionContext context, const SimTime& signal_on_time,
                                 const SimTime& search_ready_time, double elevation_deg, const Cn0Model& cn0_model,
                                 const SignalTrackingModelConfig& config, DeterministicRng* rng,
                                 std::string* error_message);
bool update_signal_tracker(SignalTracker* tracker, const SimTime& current_time, bool signal_available,
                           double elevation_deg, const Cn0Model& cn0_model, std::string* error_message);

const char* signal_tracking_phase_name(SignalTrackingPhase phase);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_SIGNAL_TRACKING_H_
