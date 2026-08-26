#include "model/receiver_truth.h"

#include "gnss/rtklib_adapter.h"

namespace gnss_sim {

bool make_static_receiver_truth(const ReceiverConfig& receiver_config, ReceiverTruth* truth,
                                std::string* error_message) {
    if (truth == nullptr) {
        if (error_message != nullptr) {
            *error_message = "receiver truth output must not be null";
        }
        return false;
    }

    ReceiverTruth result{};
    result.latitude_deg = receiver_config.latitude_deg;
    result.longitude_deg = receiver_config.longitude_deg;
    result.height_m = receiver_config.height_m;
    if (!rtklib_llh_to_ecef(result.latitude_deg, result.longitude_deg, result.height_m, result.position_ecef_m)) {
        if (error_message != nullptr) {
            *error_message = "receiver LLH is invalid or cannot be converted to ECEF";
        }
        return false;
    }

    result.velocity_ecef_mps[0] = 0.0;
    result.velocity_ecef_mps[1] = 0.0;
    result.velocity_ecef_mps[2] = 0.0;
    *truth = result;
    return true;
}

} // namespace gnss_sim
