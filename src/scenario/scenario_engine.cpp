#include "scenario/scenario_engine.h"

#include "gnss_sim/sim_time.h"

namespace gnss_sim {
namespace {

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_start_time(const SimTime& time) {
    return time.gps_week >= 0 && time.tow_ns >= 0 && time.tow_ns < GPS_WEEK_NANOSECONDS;
}

bool phase_state(const SimConfig& config, std::int64_t elapsed_ns, ScenarioEpochState* state) {
    if (elapsed_ns < 0 || state == nullptr) {
        return false;
    }

    state->cycle_index = 0U;
    state->startup_mode = config.scenario == ScenarioType::TTFF ? config.ttff.startup_mode : StartupMode::HOT;
    state->powered = true;
    state->signal_available = true;

    if (config.scenario == ScenarioType::KS) {
        return true;
    }

    if (config.scenario == ScenarioType::REA) {
        const std::int64_t cycle_ns = config.rea.signal_on_ns + config.rea.signal_off_ns;
        if (cycle_ns <= 0) {
            return false;
        }
        state->cycle_index = static_cast<std::uint64_t>(elapsed_ns / cycle_ns);
        const std::int64_t phase_ns = elapsed_ns % cycle_ns;
        state->signal_available = phase_ns < config.rea.signal_on_ns;
        return true;
    }

    const std::int64_t cycle_ns = config.ttff.power_on_ns + config.ttff.power_off_ns;
    if (cycle_ns <= 0) {
        return false;
    }
    state->cycle_index = static_cast<std::uint64_t>(elapsed_ns / cycle_ns);
    const std::int64_t phase_ns = elapsed_ns % cycle_ns;
    state->powered = phase_ns < config.ttff.power_on_ns;
    state->signal_available = state->powered;
    return true;
}

} // namespace

bool initialize_scenario_engine(const SimConfig& config, const SimTime& start_time, ScenarioEngine* engine,
                                std::string* error_message) {
    if (engine == nullptr || !valid_start_time(start_time) || !validate_sim_config(config, error_message)) {
        if (engine == nullptr || !valid_start_time(start_time)) {
            set_error(error_message, "scenario engine initialization has invalid arguments");
        }
        return false;
    }

    ScenarioEngine result{};
    result.config = config;
    result.start_time = start_time;
    *engine = result;
    return true;
}

bool update_scenario_engine(ScenarioEngine* engine, const SimTime& epoch_time, ScenarioEpochState* state,
                            std::string* error_message) {
    if (engine == nullptr || state == nullptr || !valid_start_time(epoch_time)) {
        set_error(error_message, "scenario update has invalid arguments");
        return false;
    }

    std::int64_t elapsed_ns = 0;
    if (!difference_time_ns(epoch_time, engine->start_time, &elapsed_ns) || elapsed_ns < 0 ||
        elapsed_ns >= engine->config.duration_ns) {
        set_error(error_message, "scenario epoch is outside the configured run interval");
        return false;
    }

    ScenarioEpochState current{};
    current.time = epoch_time;
    current.elapsed_ns = elapsed_ns;
    if (!phase_state(engine->config, elapsed_ns, &current)) {
        set_error(error_message, "scenario phase cannot be represented");
        return false;
    }

    if (!engine->initialized) {
        current.power_on_event = current.powered;
        current.signal_on_event = current.signal_available;
        current.startup_event = current.powered;
    } else {
        current.power_on_event = !engine->previous.powered && current.powered;
        current.power_off_event = engine->previous.powered && !current.powered;
        current.signal_on_event = !engine->previous.signal_available && current.signal_available;
        current.signal_off_event = engine->previous.signal_available && !current.signal_available;
        current.startup_event = current.power_on_event;
    }

    engine->previous = current;
    engine->initialized = true;
    *state = current;
    return true;
}

} // namespace gnss_sim
