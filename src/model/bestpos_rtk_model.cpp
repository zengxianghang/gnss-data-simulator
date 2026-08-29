#include "model/bestpos_rtk_model.h"

#include "gnss_sim/sim_time.h"

namespace gnss_sim {
namespace {

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool qualifying_position(const BestposRtkConfig& config, const PositionSolution& position) {
    return position.valid && position.status == ReceiverSolutionStatus::kSolComputed &&
           position.type == ReceiverSolutionType::kSingle && position.used_satellites >= config.min_used_satellites;
}

} // namespace

void reset_bestpos_rtk_state(BestposRtkState* state) {
    if (state != nullptr) {
        *state = {};
    }
}

bool update_bestpos_rtk_state(const BestposRtkConfig& config, const SimTime& epoch_time,
                              const PositionSolution& position, BestposRtkState* state, std::string* error_message) {
    if (state == nullptr || config.stable_duration_ns < 0 || config.min_used_satellites < 1) {
        set_error(error_message, "BESTPOS RTK state request has invalid arguments");
        return false;
    }
    if (!config.enabled || !qualifying_position(config, position)) {
        reset_bestpos_rtk_state(state);
        return true;
    }
    if (state->fixed) {
        return true;
    }
    if (!state->stability_active) {
        state->stability_active = true;
        state->stable_since = epoch_time;
        if (config.stable_duration_ns == 0) {
            state->fixed = true;
        }
        return true;
    }

    std::int64_t stable_ns = 0;
    if (!difference_time_ns(epoch_time, state->stable_since, &stable_ns) || stable_ns < 0) {
        set_error(error_message, "BESTPOS RTK stability time is not monotonic");
        return false;
    }
    if (stable_ns >= config.stable_duration_ns) {
        state->fixed = true;
    }
    return true;
}

} // namespace gnss_sim
