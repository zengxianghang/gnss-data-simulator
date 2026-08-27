#include "scenario/scenario_engine.h"

#include "gnss_sim/sim_time.h"

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

bool compute_state(const ScenarioEngine& engine, const SimTime& current_time, ScenarioEpochState* state) {
    std::int64_t elapsed_ns = 0;
    if (!difference_time_ns(current_time, engine.start_time, &elapsed_ns) || elapsed_ns < 0) {
        return false;
    }

    ScenarioEpochState result{};
    result.time = current_time;
    result.receiver_powered = true;
    result.signal_available = true;

    if (engine.config.scenario == ScenarioType::REA) {
        const std::int64_t period_ns = engine.config.rea.signal_on_ns + engine.config.rea.signal_off_ns;
        result.cycle_index = static_cast<std::uint64_t>(elapsed_ns / period_ns);
        result.phase_elapsed_ns = elapsed_ns % period_ns;
        result.signal_available = result.phase_elapsed_ns < engine.config.rea.signal_on_ns;
    } else if (engine.config.scenario == ScenarioType::TTFF) {
        const std::int64_t period_ns = engine.config.ttff.power_on_ns + engine.config.ttff.power_off_ns;
        result.cycle_index = static_cast<std::uint64_t>(elapsed_ns / period_ns);
        result.phase_elapsed_ns = elapsed_ns % period_ns;
        result.receiver_powered = result.phase_elapsed_ns < engine.config.ttff.power_on_ns;
        result.signal_available = result.receiver_powered;
    } else {
        result.cycle_index = 0U;
        result.phase_elapsed_ns = elapsed_ns;
    }

    *state = result;
    return true;
}

} // namespace

bool initialize_scenario_engine(const SimConfig& config, const SimTime& start_time, ScenarioEngine* engine,
                                std::string* error_message) {
    if (engine == nullptr || !valid_time(start_time)) {
        set_error(error_message, "scenario engine initialization has invalid arguments");
        return false;
    }
    if (!validate_sim_config(config, error_message)) {
        return false;
    }

    ScenarioEngine result{};
    result.config = config;
    result.start_time = start_time;
    result.last_time = start_time;
    result.initialized = true;
    *engine = result;
    return true;
}

bool update_scenario_engine(ScenarioEngine* engine, const SimTime& current_time, ScenarioEpochState* state,
                            std::string* error_message) {
    if (engine == nullptr || state == nullptr || !engine->initialized || !valid_time(current_time) ||
        compare_sim_time(current_time, engine->start_time) < 0 ||
        (engine->have_last_state && compare_sim_time(current_time, engine->last_time) < 0)) {
        set_error(error_message, "scenario engine update has invalid or non-monotonic time");
        return false;
    }

    ScenarioEpochState result{};
    if (!compute_state(*engine, current_time, &result)) {
        set_error(error_message, "cannot compute scenario state");
        return false;
    }

    if (!engine->have_last_state) {
        result.power_on_transition = result.receiver_powered;
        result.signal_on_transition = result.signal_available;
    } else {
        result.power_on_transition = !engine->last_receiver_powered && result.receiver_powered;
        result.power_off_transition = engine->last_receiver_powered && !result.receiver_powered;
        result.signal_on_transition = !engine->last_signal_available && result.signal_available;
        result.signal_off_transition = engine->last_signal_available && !result.signal_available;
    }

    engine->last_time = current_time;
    engine->last_receiver_powered = result.receiver_powered;
    engine->last_signal_available = result.signal_available;
    engine->have_last_state = true;
    *state = result;
    return true;
}

} // namespace gnss_sim
