#include "gnss_sim/sim_time.h"
#include "model/signal_tracking.h"

#include <cmath>
#include <cstdint>

namespace gnss_sim {
namespace {

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_time(const SimTime& time) {
    return time.gps_week >= 0 && time.tow_ns >= 0 && time.tow_ns < GPS_WEEK_NANOSECONDS;
}

bool valid_effective_cn0(double cn0_dbhz) {
    return std::isfinite(cn0_dbhz) || (std::isinf(cn0_dbhz) && cn0_dbhz < 0.0);
}

bool elapsed_at_least(const SimTime& current_time, const SimTime& start_time, std::int64_t required_ns, bool* reached) {
    if (reached == nullptr || required_ns < 0) {
        return false;
    }
    std::int64_t elapsed_ns = 0;
    if (!difference_time_ns(current_time, start_time, &elapsed_ns) || elapsed_ns < 0) {
        return false;
    }
    *reached = elapsed_ns >= required_ns;
    return true;
}

void clear_measurement_validity(SignalTracker* tracker) {
    tracker->lock_time_ns = 0;
    tracker->psr_valid = false;
    tracker->doppler_valid = false;
    tracker->adr_valid = false;
    tracker->observation_available = false;
    tracker->carrier_continuity_valid = false;
}

void clear_dll_continuity(SignalTracker* tracker) {
    tracker->previous_dll_root_valid = false;
    tracker->current_dll_root_valid = false;
    tracker->previous_dll_code_phase_sec = 0.0;
    tracker->previous_dll_code_phase_chips = 0.0;
    tracker->current_dll_code_phase_sec = 0.0;
    tracker->current_dll_code_phase_chips = 0.0;
    tracker->current_dll_prompt_power = 0.0;
}

bool select_root(const UrbanSignalTrackingInput& input, CodeTrackingDllSelectionMode mode,
                 double previous_code_phase_sec, CodeTrackingDllRoot* selected, bool* found,
                 std::string* error_message) {
    if (selected == nullptr || found == nullptr || input.dll_root_count < 0 ||
        (input.dll_root_count > 0 && input.dll_roots == nullptr)) {
        set_error(error_message, "urban DLL root input is invalid");
        return false;
    }
    *found = false;
    if (input.dll_root_count == 0) {
        return true;
    }

    bool has_stable_root = false;
    for (int index = 0; index < input.dll_root_count; ++index) {
        if (input.dll_roots[index].stable) {
            has_stable_root = true;
            break;
        }
    }
    if (!has_stable_root) {
        return true;
    }

    int selected_index = -1;
    if (!select_code_tracking_dll_root(input.dll_roots, input.dll_root_count, mode, previous_code_phase_sec,
                                       &selected_index, error_message) ||
        selected_index < 0 || selected_index >= input.dll_root_count || !input.dll_roots[selected_index].stable) {
        return false;
    }
    *selected = input.dll_roots[selected_index];
    *found = true;
    return true;
}

void update_direct_classification(SignalTracker* tracker, const UrbanSignalTrackingInput& input,
                                  const SignalTrackingModelConfig& config) {
    if (!input.direct_line_of_sight) {
        tracker->urban_state = tracker->phase == SignalTrackingPhase::kTracking && tracker->has_tracking_lock
                                   ? UrbanSignalState::kNlosTracked
                                   : UrbanSignalState::kBlocked;
        return;
    }

    const double cn0_delta_db = std::abs(input.effective_cn0_dbhz - input.open_cn0_dbhz);
    const double code_bias_chips =
        tracker->current_dll_root_valid ? std::abs(tracker->current_dll_code_phase_chips) : 0.0;
    if (cn0_delta_db >= config.los_multipath_cn0_delta_db || code_bias_chips >= config.los_multipath_code_bias_chips) {
        tracker->urban_state = UrbanSignalState::kLosMultipath;
    } else {
        tracker->urban_state = UrbanSignalState::kLos;
    }
}

void begin_tracking_loss(SignalTracker* tracker, const SimTime& current_time, SignalTrackingLossReason reason) {
    tracker->phase = SignalTrackingPhase::kSearching;
    tracker->acquisition_context = AcquisitionContext::kReacquisition;
    tracker->state_since = current_time;
    tracker->search_ready_time = current_time;
    tracker->scheduled = false;
    tracker->has_tracking_lock = false;
    tracker->reacquisition_pending = true;
    tracker->reacquisition_event = false;
    tracker->loss_reason = reason;
    tracker->above_acquisition_threshold_active = false;
    tracker->below_tracking_threshold_active = false;
    clear_measurement_validity(tracker);
    clear_dll_continuity(tracker);
}

bool enter_tracking(SignalTracker* tracker, const SimTime& current_time, const SignalTrackingModelConfig& config,
                    const CodeTrackingDllRoot& selected_root, std::string* error_message) {
    SimTime psr_valid_time{};
    SimTime doppler_valid_time{};
    SimTime adr_valid_time{};
    if (!add_time_ns(current_time, config.psr_valid_delay_ns, &psr_valid_time) ||
        !add_time_ns(current_time, config.doppler_valid_delay_ns, &doppler_valid_time) ||
        !add_time_ns(current_time, config.adr_valid_delay_ns, &adr_valid_time)) {
        set_error(error_message, "urban signal tracking validity time overflows simulation time");
        return false;
    }

    tracker->phase = SignalTrackingPhase::kTracking;
    tracker->state_since = current_time;
    tracker->tracking_start_time = current_time;
    tracker->psr_valid_time = psr_valid_time;
    tracker->doppler_valid_time = doppler_valid_time;
    tracker->adr_valid_time = adr_valid_time;
    tracker->has_tracking_lock = true;
    tracker->lock_time_ns = 0;
    tracker->psr_valid = false;
    tracker->doppler_valid = false;
    tracker->adr_valid = false;
    tracker->observation_available = false;
    tracker->carrier_continuity_valid = false;
    tracker->reacquisition_event = tracker->acquisition_context == AcquisitionContext::kReacquisition;
    tracker->reacquisition_pending = false;
    tracker->above_acquisition_threshold_active = false;
    tracker->below_tracking_threshold_active = false;
    tracker->current_dll_root_valid = true;
    tracker->previous_dll_root_valid = true;
    tracker->current_dll_code_phase_sec = selected_root.code_phase_sec;
    tracker->current_dll_code_phase_chips = selected_root.code_phase_chips;
    tracker->current_dll_prompt_power = selected_root.prompt_power;
    tracker->previous_dll_code_phase_sec = selected_root.code_phase_sec;
    tracker->previous_dll_code_phase_chips = selected_root.code_phase_chips;
    return true;
}

bool update_acquisition_qualification(SignalTracker* tracker, const SimTime& current_time,
                                      const UrbanSignalTrackingInput& input, const SignalTrackingModelConfig& config,
                                      bool* qualified, std::string* error_message) {
    if (qualified == nullptr) {
        set_error(error_message, "urban acquisition qualification output is null");
        return false;
    }
    *qualified = false;
    if (input.effective_cn0_dbhz < config.acquisition_cn0_threshold_dbhz) {
        tracker->above_acquisition_threshold_active = false;
        return true;
    }
    if (!tracker->above_acquisition_threshold_active) {
        tracker->above_acquisition_threshold_active = true;
        tracker->above_acquisition_threshold_since = current_time;
    }
    if (!elapsed_at_least(current_time, tracker->above_acquisition_threshold_since,
                          config.acquisition_cn0_persistence_ns, qualified)) {
        set_error(error_message, "cannot compute urban acquisition qualification duration");
        return false;
    }
    return true;
}

bool update_tracking_validity(SignalTracker* tracker, const SimTime& current_time, std::string* error_message) {
    std::int64_t lock_time_ns = 0;
    if (!difference_time_ns(current_time, tracker->tracking_start_time, &lock_time_ns) || lock_time_ns < 0) {
        set_error(error_message, "cannot compute urban signal lock time");
        return false;
    }
    tracker->lock_time_ns = lock_time_ns;
    tracker->psr_valid = compare_sim_time(current_time, tracker->psr_valid_time) >= 0;
    tracker->doppler_valid = compare_sim_time(current_time, tracker->doppler_valid_time) >= 0;
    tracker->adr_valid = compare_sim_time(current_time, tracker->adr_valid_time) >= 0;
    tracker->observation_available = tracker->psr_valid;
    tracker->carrier_continuity_valid = tracker->adr_valid;
    return true;
}

} // namespace

bool update_urban_signal_tracker(SignalTracker* tracker, const SimTime& current_time,
                                 const UrbanSignalTrackingInput& input, const SignalTrackingModelConfig& config,
                                 std::string* error_message) {
    if (tracker == nullptr || !valid_time(current_time) || !std::isfinite(input.open_cn0_dbhz) ||
        !valid_effective_cn0(input.effective_cn0_dbhz) || input.dll_root_count < 0 ||
        (input.dll_root_count > 0 && input.dll_roots == nullptr) ||
        find_signal_definition(tracker->signal_id) == nullptr ||
        !validate_signal_tracking_model_config(config, nullptr)) {
        set_error(error_message, "urban signal tracking update has invalid arguments");
        return false;
    }
    if (compare_sim_time(current_time, tracker->state_since) < 0) {
        set_error(error_message, "urban signal tracking update precedes tracker state time");
        return false;
    }

    tracker->reacquisition_event = false;
    tracker->cn0_dbhz = input.effective_cn0_dbhz;
    tracker->current_dll_root_valid = false;

    if (!input.signal_available) {
        const bool had_lock = tracker->phase == SignalTrackingPhase::kTracking && tracker->has_tracking_lock;
        tracker->phase = SignalTrackingPhase::kSignalOff;
        tracker->state_since = current_time;
        tracker->scheduled = false;
        tracker->has_tracking_lock = false;
        tracker->reacquisition_pending = false;
        tracker->above_acquisition_threshold_active = false;
        tracker->below_tracking_threshold_active = false;
        if (had_lock) {
            tracker->loss_reason = SignalTrackingLossReason::kSignalUnavailable;
        }
        clear_measurement_validity(tracker);
        clear_dll_continuity(tracker);
        update_direct_classification(tracker, input, config);
        return true;
    }

    if (!tracker->scheduled) {
        if (!tracker->reacquisition_pending) {
            set_error(error_message, "urban signal is available but acquisition has not been scheduled");
            return false;
        }
        tracker->phase = SignalTrackingPhase::kSearching;
        clear_measurement_validity(tracker);
        update_direct_classification(tracker, input, config);
        return true;
    }

    if (tracker->phase != SignalTrackingPhase::kTracking || !tracker->has_tracking_lock) {
        if (compare_sim_time(current_time, tracker->search_ready_time) < 0) {
            tracker->phase = SignalTrackingPhase::kSearching;
            tracker->above_acquisition_threshold_active = false;
            clear_measurement_validity(tracker);
            update_direct_classification(tracker, input, config);
            return true;
        }

        tracker->phase = SignalTrackingPhase::kAcquiring;
        bool qualified = false;
        if (!update_acquisition_qualification(tracker, current_time, input, config, &qualified, error_message)) {
            return false;
        }

        CodeTrackingDllRoot selected_root{};
        bool root_found = false;
        if (!select_root(input, CodeTrackingDllSelectionMode::ACQUISITION, 0.0, &selected_root, &root_found,
                         error_message)) {
            return false;
        }
        if (root_found) {
            tracker->current_dll_root_valid = true;
            tracker->current_dll_code_phase_sec = selected_root.code_phase_sec;
            tracker->current_dll_code_phase_chips = selected_root.code_phase_chips;
            tracker->current_dll_prompt_power = selected_root.prompt_power;
        }

        if (qualified && root_found && compare_sim_time(current_time, tracker->acquisition_complete_time) >= 0) {
            if (!enter_tracking(tracker, current_time, config, selected_root, error_message)) {
                return false;
            }
        } else {
            clear_measurement_validity(tracker);
        }
        update_direct_classification(tracker, input, config);
        return true;
    }

    CodeTrackingDllRoot selected_root{};
    bool root_found = false;
    if (!select_root(input, CodeTrackingDllSelectionMode::TRACKED, tracker->previous_dll_code_phase_sec, &selected_root,
                     &root_found, error_message)) {
        return false;
    }
    if (!root_found) {
        begin_tracking_loss(tracker, current_time, SignalTrackingLossReason::kNoStableDllRoot);
        update_direct_classification(tracker, input, config);
        return true;
    }

    tracker->current_dll_root_valid = true;
    tracker->current_dll_code_phase_sec = selected_root.code_phase_sec;
    tracker->current_dll_code_phase_chips = selected_root.code_phase_chips;
    tracker->current_dll_prompt_power = selected_root.prompt_power;

    if (tracker->previous_dll_root_valid &&
        std::abs(selected_root.code_phase_chips - tracker->previous_dll_code_phase_chips) >
            config.dll_root_jump_threshold_chips) {
        begin_tracking_loss(tracker, current_time, SignalTrackingLossReason::kAbruptDllRootSwitch);
        update_direct_classification(tracker, input, config);
        return true;
    }

    if (input.effective_cn0_dbhz < config.minimum_tracking_cn0_dbhz) {
        if (!tracker->below_tracking_threshold_active) {
            tracker->below_tracking_threshold_active = true;
            tracker->below_tracking_threshold_since = current_time;
        }
        bool loss_qualified = false;
        if (!elapsed_at_least(current_time, tracker->below_tracking_threshold_since,
                              config.tracking_loss_cn0_persistence_ns, &loss_qualified)) {
            set_error(error_message, "cannot compute urban tracking-loss qualification duration");
            return false;
        }
        if (loss_qualified) {
            begin_tracking_loss(tracker, current_time, SignalTrackingLossReason::kLowCn0);
            update_direct_classification(tracker, input, config);
            return true;
        }
    } else {
        tracker->below_tracking_threshold_active = false;
    }

    tracker->previous_dll_root_valid = true;
    tracker->previous_dll_code_phase_sec = selected_root.code_phase_sec;
    tracker->previous_dll_code_phase_chips = selected_root.code_phase_chips;
    if (!update_tracking_validity(tracker, current_time, error_message)) {
        return false;
    }
    update_direct_classification(tracker, input, config);
    return true;
}

const char* urban_signal_state_name(UrbanSignalState state) {
    switch (state) {
        case UrbanSignalState::kLos:
            return "LOS";
        case UrbanSignalState::kLosMultipath:
            return "LOS_MULTIPATH";
        case UrbanSignalState::kNlosTracked:
            return "NLOS_TRACKED";
        case UrbanSignalState::kBlocked:
            return "BLOCKED";
    }
    return "UNKNOWN";
}

const char* signal_tracking_loss_reason_name(SignalTrackingLossReason reason) {
    switch (reason) {
        case SignalTrackingLossReason::kNone:
            return "NONE";
        case SignalTrackingLossReason::kSignalUnavailable:
            return "SIGNAL_UNAVAILABLE";
        case SignalTrackingLossReason::kLowCn0:
            return "LOW_CN0";
        case SignalTrackingLossReason::kNoStableDllRoot:
            return "NO_STABLE_DLL_ROOT";
        case SignalTrackingLossReason::kAbruptDllRootSwitch:
            return "ABRUPT_DLL_ROOT_SWITCH";
    }
    return "UNKNOWN";
}

} // namespace gnss_sim
