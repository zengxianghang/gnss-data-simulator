#ifndef GNSS_SIM_SRC_SCENARIO_SCENARIO_ENGINE_H_
#define GNSS_SIM_SRC_SCENARIO_SCENARIO_ENGINE_H_

#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_types.h"

#include <cstdint>
#include <string>

namespace gnss_sim {

struct ScenarioEpochState {
    SimTime time;
    bool receiver_powered;
    bool signal_available;
    bool power_on_transition;
    bool power_off_transition;
    bool signal_on_transition;
    bool signal_off_transition;
    std::uint64_t cycle_index;
    std::int64_t phase_elapsed_ns;
};

struct ScenarioEngine {
    SimConfig config;
    SimTime start_time;
    SimTime last_time;
    bool initialized;
    bool have_last_state;
    bool last_receiver_powered;
    bool last_signal_available;
};

bool initialize_scenario_engine(const SimConfig& config, const SimTime& start_time, ScenarioEngine* engine,
                                std::string* error_message);
bool update_scenario_engine(ScenarioEngine* engine, const SimTime& current_time, ScenarioEpochState* state,
                            std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_SCENARIO_SCENARIO_ENGINE_H_
