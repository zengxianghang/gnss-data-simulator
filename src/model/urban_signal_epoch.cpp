#include "model/urban_signal_epoch.h"

#include <cmath>
#include <limits>

namespace gnss_sim {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool has_stable_root(const CodeTrackingDllRoot* roots, int root_count) {
    if (roots == nullptr || root_count <= 0) {
        return false;
    }
    for (int index = 0; index < root_count; ++index) {
        if (roots[index].stable) {
            return true;
        }
    }
    return false;
}

void set_no_trackable_root_cn0(double open_cn0_dbhz, UrbanEffectiveCn0* effective_cn0) {
    *effective_cn0 = {};
    effective_cn0->open_cn0_dbhz = open_cn0_dbhz;
    effective_cn0->composite_correlation = {0.0, 0.0};
    effective_cn0->composite_power_ratio = 0.0;
    effective_cn0->carrier_to_noise_density_hz = 0.0;
    effective_cn0->effective_cn0_dbhz = -std::numeric_limits<double>::infinity();
    effective_cn0->finite_effective_cn0 = false;
}

bool same_selected_root(const SignalTracker& tracker, const CodeTrackingDllRoot& root) {
    return tracker.current_dll_root_valid && tracker.current_dll_code_phase_sec == root.code_phase_sec &&
           tracker.current_dll_code_phase_chips == root.code_phase_chips &&
           tracker.current_dll_prompt_power == root.prompt_power;
}

} // namespace

bool update_urban_signal_epoch_from_paths(const SignalDefinition& signal, const SimTime& current_time,
                                          const UrbanReceivedPathSet& received_paths,
                                          const CodeTrackingDllConfig& dll_config,
                                          const SignalTrackingModelConfig& tracking_config, SignalTracker* tracker,
                                          UrbanSignalEpochResult* result, std::string* error_message) {
    if (tracker == nullptr || result == nullptr || tracker->signal_id != signal.signal_id ||
        received_paths.path_count <= 0 || received_paths.path_count > kMaxUrbanReceivedPaths ||
        !std::isfinite(received_paths.open_cn0_dbhz) || !validate_code_tracking_dll_config(dll_config, nullptr) ||
        !validate_signal_tracking_model_config(tracking_config, nullptr)) {
        set_error(error_message, "urban signal epoch request is invalid");
        return false;
    }

    UrbanSignalEpochResult output{};
    output.received_paths = received_paths;
    output.preselected_root_index = -1;
    output.selection_mode = tracker->phase == SignalTrackingPhase::kTracking && tracker->has_tracking_lock
                                ? CodeTrackingDllSelectionMode::TRACKED
                                : CodeTrackingDllSelectionMode::ACQUISITION;

    if (!find_code_tracking_dll_roots_with_status(signal, received_paths.paths, received_paths.path_count, dll_config,
                                                  output.dll_roots, kMaxCodeTrackingDllRoots, &output.dll_root_count,
                                                  &output.root_search_status, error_message)) {
        return false;
    }

    output.stable_root_available = has_stable_root(output.dll_roots, output.dll_root_count);
    if (output.stable_root_available) {
        const double previous_code_phase_sec =
            output.selection_mode == CodeTrackingDllSelectionMode::TRACKED ? tracker->previous_dll_code_phase_sec : 0.0;
        if (!select_code_tracking_dll_root(output.dll_roots, output.dll_root_count, output.selection_mode,
                                           previous_code_phase_sec, &output.preselected_root_index, error_message) ||
            output.preselected_root_index < 0 || output.preselected_root_index >= output.dll_root_count ||
            !output.dll_roots[output.preselected_root_index].stable) {
            return false;
        }
        output.preselected_root = output.dll_roots[output.preselected_root_index];
        if (!compute_urban_effective_cn0(signal, received_paths, output.preselected_root.code_phase_sec,
                                         &output.effective_cn0, error_message)) {
            return false;
        }
    } else {
        set_no_trackable_root_cn0(received_paths.open_cn0_dbhz, &output.effective_cn0);
    }

    UrbanSignalTrackingInput tracking_input{};
    tracking_input.signal_available = true;
    tracking_input.direct_line_of_sight = received_paths.direct_geometry.line_of_sight;
    tracking_input.open_cn0_dbhz = received_paths.open_cn0_dbhz;
    tracking_input.effective_cn0_dbhz = output.effective_cn0.effective_cn0_dbhz;
    tracking_input.dll_roots = output.dll_roots;
    tracking_input.dll_root_count = output.dll_root_count;
    if (!update_urban_signal_tracker(tracker, current_time, tracking_input, tracking_config, error_message)) {
        return false;
    }

    if (tracker->phase == SignalTrackingPhase::kTracking && tracker->has_tracking_lock) {
        if (!output.stable_root_available || !same_selected_root(*tracker, output.preselected_root)) {
            set_error(error_message, "urban signal epoch root selection diverged from tracking state");
            return false;
        }
    }

    output.selected_root_valid = tracker->current_dll_root_valid;
    if (output.selected_root_valid) {
        output.code_bias_m = kSpeedOfLightMps * tracker->current_dll_code_phase_sec;
    }
    output.tracked_composite_correlation_valid =
        tracker->phase == SignalTrackingPhase::kTracking && tracker->has_tracking_lock && output.selected_root_valid;
    if (output.tracked_composite_correlation_valid) {
        output.tracked_composite_correlation = output.effective_cn0.composite_correlation;
    }
    output.urban_state = tracker->urban_state;
    output.tracking_phase = tracker->phase;
    output.loss_reason = tracker->loss_reason;
    output.effective_cn0_dbhz = output.effective_cn0.effective_cn0_dbhz;
    output.lock_time_ns = tracker->lock_time_ns;
    output.observation_available = tracker->observation_available;
    output.psr_valid = tracker->psr_valid;
    output.doppler_valid = tracker->doppler_valid;
    output.adr_valid = tracker->adr_valid;
    output.reacquisition_event = tracker->reacquisition_event;
    output.carrier_continuity_valid = tracker->carrier_continuity_valid;

    *result = output;
    return true;
}

bool compute_urban_signal_epoch(const Cn0Model& cn0_model, const UrbanSceneGeometryConfig& scene_config,
                                const UrbanRfConfig& rf_config, const CodeTrackingDllConfig& dll_config,
                                const SignalTrackingModelConfig& tracking_config, const SignalDefinition& signal,
                                int glonass_fcn, const SimTime& current_time, const ReceiverTruth& receiver,
                                const SatelliteGeometry& satellite_geometry, SignalTracker* tracker,
                                UrbanSignalEpochResult* result, std::string* error_message) {
    UrbanReceivedPathSet received_paths{};
    if (!compute_urban_received_path_set(cn0_model, scene_config, rf_config, signal, glonass_fcn, current_time,
                                         receiver, satellite_geometry, &received_paths, error_message)) {
        return false;
    }
    return update_urban_signal_epoch_from_paths(signal, current_time, received_paths, dll_config, tracking_config,
                                                tracker, result, error_message);
}

} // namespace gnss_sim
