#ifndef GNSS_SIM_SRC_MODEL_RECEIVER_TRUTH_H_
#define GNSS_SIM_SRC_MODEL_RECEIVER_TRUTH_H_

#include "gnss_sim/sim_config.h"

#include <string>

namespace gnss_sim {

struct ReceiverTruth {
    double latitude_deg;
    double longitude_deg;
    double height_m;
    double position_ecef_m[3];
    double velocity_ecef_mps[3];
};

bool make_static_receiver_truth(const ReceiverConfig& receiver_config, ReceiverTruth* truth,
                                std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_RECEIVER_TRUTH_H_
