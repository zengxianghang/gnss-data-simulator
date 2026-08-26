#ifndef GNSS_SIM_SRC_MODEL_CN0_MODEL_H_
#define GNSS_SIM_SRC_MODEL_CN0_MODEL_H_

#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_types.h"

#include <cstdint>

namespace gnss_sim {

enum class Cn0ModelSource {
    kBuiltinFallback,
};

struct Cn0Model {
    Cn0ModelSource source;
    std::uint64_t seed;
};

Cn0Model make_builtin_cn0_model(std::uint64_t seed);
bool cn0_model_estimate_dbhz(const Cn0Model& model, SignalId signal_id, double elevation_deg, const SimTime& time,
                             double* cn0_dbhz);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_CN0_MODEL_H_
