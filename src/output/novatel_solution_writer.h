#ifndef GNSS_SIM_SRC_OUTPUT_NOVATEL_SOLUTION_WRITER_H_
#define GNSS_SIM_SRC_OUTPUT_NOVATEL_SOLUTION_WRITER_H_

#include "model/receiver_truth.h"
#include "solution/solution_engine.h"

#include <string>

namespace gnss_sim {

bool format_novatel_psrposa(const SolutionEpoch& solution, int tracked_satellites, std::string* message,
                            std::string* error_message);
bool format_novatel_psrvela(const SolutionEpoch& solution, std::string* message, std::string* error_message);
bool format_novatel_bestposa(const SolutionEpoch& solution, int tracked_satellites, const ReceiverTruth& truth,
                             bool rtk_fixed, const BestposRtkConfig& rtk_config, std::string* message,
                             std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_OUTPUT_NOVATEL_SOLUTION_WRITER_H_
