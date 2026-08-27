#ifndef GNSS_SIM_SRC_MODEL_CN0_MODEL_H_
#define GNSS_SIM_SRC_MODEL_CN0_MODEL_H_

#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace gnss_sim {

enum class Cn0ModelSource {
    kBuiltinFallback,
    kCalibratedCsv,
};

struct Cn0CalibratedBin {
    SignalId signal_id{SignalId::kGpsL1Ca};
    double elevation_min_deg{};
    double elevation_max_deg{};
    double elevation_center_deg{};
    double p50_dbhz{};
    bool upper_edge_inclusive{};
    bool ready{};
};

struct Cn0ModelIdentity {
    std::string schema_version;
    std::string file_name;
    std::string hash;
    std::uint64_t size_bytes{};
};

struct Cn0Model {
    Cn0ModelSource source{Cn0ModelSource::kBuiltinFallback};
    std::uint64_t seed{};
    std::vector<Cn0CalibratedBin> calibrated_bins;
    Cn0ModelIdentity identity;
};

Cn0Model make_builtin_cn0_model(std::uint64_t seed);
bool load_cn0_model_csv(const char* file_path, std::uint64_t seed, Cn0Model* model, std::string* error_message);
bool cn0_model_estimate_dbhz(const Cn0Model& model, SignalId signal_id, double elevation_deg, const SimTime& time,
                             double* cn0_dbhz);
const char* cn0_model_source_name(Cn0ModelSource source);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_CN0_MODEL_H_
