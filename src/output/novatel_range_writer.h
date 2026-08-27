#ifndef GNSS_SIM_SRC_OUTPUT_NOVATEL_RANGE_WRITER_H_
#define GNSS_SIM_SRC_OUTPUT_NOVATEL_RANGE_WRITER_H_

#include "gnss_sim/sim_types.h"
#include "model/measurement_model.h"

#include <string>

namespace gnss_sim {

bool format_novatel_rangea(const SimTime& time, const MeasurementObservation* observations, int observation_count,
                           std::string* message, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_OUTPUT_NOVATEL_RANGE_WRITER_H_
