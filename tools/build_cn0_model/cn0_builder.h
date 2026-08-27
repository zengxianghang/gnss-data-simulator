#ifndef GNSS_SIM_TOOLS_BUILD_CN0_MODEL_CN0_BUILDER_H_
#define GNSS_SIM_TOOLS_BUILD_CN0_MODEL_CN0_BUILDER_H_

#include "tools/build_cn0_model/cn0_statistics.h"

#include <cstdint>
#include <string>
#include <vector>

namespace gnss_sim::cn0_builder {

struct Cn0InputSource {
    std::string observation_path;
    std::string navigation_path;
};

struct Cn0FileIdentity {
    std::string file_name;
    std::uint64_t size_bytes{};
    std::uint64_t fnv1a64{};
};

struct Cn0SourceMetadata {
    Cn0FileIdentity observation_file;
    Cn0FileIdentity navigation_file;
    RinexObsProvenance provenance;
    RinexObsStreamSummary stream_summary;
    bool observation_interval_available{};
    double observation_interval_sec{};
    bool sample_time_available{};
    SimTime first_sample_time{};
    SimTime last_sample_time{};
};

struct Cn0BuildResult {
    Cn0AggregationSummary aggregation_summary;
    std::vector<Cn0BinStatistics> bins;
    std::vector<Cn0SourceMetadata> sources;
};

bool build_cn0_model(const std::vector<Cn0InputSource>& sources, const Cn0AggregationConfig& config,
                     Cn0BuildResult* result, std::string* error_message);

bool write_cn0_metadata_json(const std::string& output_path, const Cn0AggregationConfig& config,
                             const Cn0BuildResult& result, std::string* error_message);

} // namespace gnss_sim::cn0_builder

#endif // GNSS_SIM_TOOLS_BUILD_CN0_MODEL_CN0_BUILDER_H_
