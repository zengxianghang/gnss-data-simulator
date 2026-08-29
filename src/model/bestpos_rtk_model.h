#ifndef GNSS_SIM_SRC_MODEL_BESTPOS_RTK_MODEL_H_
#define GNSS_SIM_SRC_MODEL_BESTPOS_RTK_MODEL_H_

#include "gnss_sim/sim_config.h"
#include "solution/solution_engine.h"

#include <string>

namespace gnss_sim {

struct BestposRtkState {
    bool stability_active;
    bool fixed;
    SimTime stable_since;
};

void reset_bestpos_rtk_state(BestposRtkState* state);
bool update_bestpos_rtk_state(const BestposRtkConfig& config, const SimTime& epoch_time,
                              const PositionSolution& position, BestposRtkState* state,
                              std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_BESTPOS_RTK_MODEL_H_
