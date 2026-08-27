#ifndef GNSS_SIM_SRC_SCENARIO_SCENARIO_ENGINE_H_
#define GNSS_SIM_SRC_SCENARIO_SCENARIO_ENGINE_H_

#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_types.h"

#include <cstdint>
#include <string>

namespace gnss_sim {

struct ScenarioEpochState {
    SimTime time;
    std::int64_t elapsed_ns;
    std::uint64_t cycle_index;
    StartupMode startup_mode;
    bool powered;
    bool signal_available;
    bool power_on_event;
    bool power_off_event;
    bool signal_on_event;
    bool signal_off_event;
    bool startup_event;
};

struct ScenarioEngine {
    SimConfig config;
    SimTime start_time;
    ScenarioEpochState previous;
    bool initialized;
};

bool initialize_scenario_engine(const SimConfig& config, const SimTime& start_time, ScenarioEngine* engine,
                                std::string* error_message);
bool update_scenario_engine(ScenarioEngine* engine, const SimTime& epoch_time, ScenarioEpochState* state,
                            std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_SCENARIO_SCENARIO_ENGINE_H_
