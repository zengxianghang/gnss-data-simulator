#ifndef GNSS_SIM_SRC_MODEL_URBAN_SIGNAL_EPOCH_H_
#define GNSS_SIM_SRC_MODEL_URBAN_SIGNAL_EPOCH_H_

#include "model/code_tracking_dll.h"
#include "model/signal_tracking.h"
#include "model/urban_received_power.h"

#include <complex>
#include <cstdint>
#include <string>

namespace gnss_sim {

struct UrbanSignalEpochResult {
    UrbanReceivedPathSet received_paths;
    CodeTrackingDllRoot dll_roots[kMaxCodeTrackingDllRoots];
    int dll_root_count;
    CodeTrackingDllRootSearchStatus root_search_status;
    CodeTrackingDllSelectionMode selection_mode;
    int preselected_root_index;
    CodeTrackingDllRoot preselected_root;
    UrbanEffectiveCn0 effective_cn0;
    double code_bias_m;
    std::complex<double> tracked_composite_correlation;
    UrbanSignalState urban_state;
    SignalTrackingPhase tracking_phase;
    SignalTrackingLossReason loss_reason;
    double effective_cn0_dbhz;
    std::int64_t lock_time_ns;
    bool stable_root_available;
    bool selected_root_valid;
    bool tracked_composite_correlation_valid;
    bool observation_available;
    bool psr_valid;
    bool doppler_valid;
    bool adr_valid;
    bool reacquisition_event;
    bool carrier_continuity_valid;
};

// Core single-epoch orchestration over one already-built #120 path set. This
// function does not schedule acquisition delays; the caller must use the
// existing SignalTracker scheduling API before invoking it when acquisition is
// required. The same roots and effective CN0 are then consumed by #121.
bool update_urban_signal_epoch_from_paths(const SignalDefinition& signal, const SimTime& current_time,
                                          const UrbanReceivedPathSet& received_paths,
                                          const CodeTrackingDllConfig& dll_config,
                                          const SignalTrackingModelConfig& tracking_config, SignalTracker* tracker,
                                          UrbanSignalEpochResult* result, std::string* error_message);

// Production #140 entry: build the common #120 propagation path set from the
// already-converged authentic-NAV SatelliteGeometry, then run the core
// DLL/CN0/tracking orchestration above. Simulator runtime wiring remains #143.
bool compute_urban_signal_epoch(const Cn0Model& cn0_model, const UrbanSceneGeometryConfig& scene_config,
                                const UrbanRfConfig& rf_config, const CodeTrackingDllConfig& dll_config,
                                const SignalTrackingModelConfig& tracking_config, const SignalDefinition& signal,
                                int glonass_fcn, const SimTime& current_time, const ReceiverTruth& receiver,
                                const SatelliteGeometry& satellite_geometry, SignalTracker* tracker,
                                UrbanSignalEpochResult* result, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_URBAN_SIGNAL_EPOCH_H_
