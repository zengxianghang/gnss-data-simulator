#ifndef GNSS_SIM_SRC_MODEL_SIGNAL_TRACKING_H_
#define GNSS_SIM_SRC_MODEL_SIGNAL_TRACKING_H_

#include "core/deterministic_rng.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_types.h"
#include "model/cn0_model.h"
#include "model/code_tracking_dll.h"

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

enum class UrbanSignalState {
    kLos,
    kLosMultipath,
    kNlosTracked,
    kBlocked,
};

enum class SignalTrackingLossReason {
    kNone,
    kSignalUnavailable,
    kLowCn0,
    kNoStableDllRoot,
    kAbruptDllRootSwitch,
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

    // V1 receiver-model engineering assumptions. These are deterministic and
    // configurable, but are not vendor-specific measurements or PVT tuning
    // targets.
    double minimum_tracking_cn0_dbhz;
    double acquisition_cn0_threshold_dbhz;
    std::int64_t acquisition_cn0_persistence_ns;
    std::int64_t tracking_loss_cn0_persistence_ns;
    double dll_root_jump_threshold_chips;
    double los_multipath_cn0_delta_db;
    double los_multipath_code_bias_chips;
};

struct ReceiverStartupTiming {
    StartupMode startup_mode;
    std::int64_t common_startup_delay_ns;
    std::int64_t search_uncertainty_delay_ns;
    std::int64_t total_search_ready_delay_ns;
};

struct UrbanSignalTrackingInput {
    bool signal_available;
    bool direct_line_of_sight;
    double open_cn0_dbhz;
    double effective_cn0_dbhz;
    const CodeTrackingDllRoot* dll_roots;
    int dll_root_count;
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

    UrbanSignalState urban_state;
    SignalTrackingLossReason loss_reason;
    SimTime above_acquisition_threshold_since;
    SimTime below_tracking_threshold_since;
    bool above_acquisition_threshold_active;
    bool below_tracking_threshold_active;
    bool has_tracking_lock;
    bool previous_dll_root_valid;
    bool current_dll_root_valid;
    double previous_dll_code_phase_sec;
    double previous_dll_code_phase_chips;
    double current_dll_code_phase_sec;
    double current_dll_code_phase_chips;
    double current_dll_prompt_power;
    bool reacquisition_pending;
    bool reacquisition_event;
    bool carrier_continuity_valid;
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

// Urban-aware deterministic receiver update. The existing sampled acquisition
// schedule remains authoritative; this overlay adds effective-CN0 eligibility,
// tracking-loss persistence, DLL-root continuity, and the four urban states.
bool update_urban_signal_tracker(SignalTracker* tracker, const SimTime& current_time,
                                 const UrbanSignalTrackingInput& input, const SignalTrackingModelConfig& config,
                                 std::string* error_message);

const char* signal_tracking_phase_name(SignalTrackingPhase phase);
const char* urban_signal_state_name(UrbanSignalState state);
const char* signal_tracking_loss_reason_name(SignalTrackingLossReason reason);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_SIGNAL_TRACKING_H_
