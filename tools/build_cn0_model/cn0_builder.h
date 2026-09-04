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

enum class Cn0ReferenceStatus {
    kReady,
    kInsufficientSupport,
};

struct Cn0NormalizationConfig {
    double reference_elevation_min_deg{60.0};
    double reference_elevation_max_deg{90.0};
    std::uint64_t min_reference_samples{20};
    std::uint64_t min_sources_per_bin{1};
};

struct Cn0SignalReference {
    GnssConstellation constellation{GnssConstellation::kGps};
    SignalId signal_id{SignalId::kGpsL1Ca};
    std::string rinex_signal_code;
    Cn0ReferenceStatus status{Cn0ReferenceStatus::kInsufficientSupport};
    std::uint64_t count{};
    double p50_dbhz{};
};

struct Cn0SourceNormalizedBin {
    GnssConstellation constellation{GnssConstellation::kGps};
    SignalId signal_id{SignalId::kGpsL1Ca};
    std::string rinex_signal_code;
    double elevation_min_deg{};
    double elevation_max_deg{};
    bool includes_upper_edge{};
    Cn0BinStatus source_status{Cn0BinStatus::kEmpty};
    std::uint64_t sample_count{};
    bool reference_ready{};
    double delta_p50_db{};
};

struct Cn0NormalizedSourceResult {
    Cn0SourceMetadata metadata;
    std::vector<Cn0SignalReference> references;
    std::vector<Cn0SourceNormalizedBin> bins;
};

struct Cn0NormalizedBin {
    GnssConstellation constellation{GnssConstellation::kGps};
    SignalId signal_id{SignalId::kGpsL1Ca};
    std::string rinex_signal_code;
    double elevation_min_deg{};
    double elevation_max_deg{};
    bool includes_upper_edge{};
    Cn0BinStatus status{Cn0BinStatus::kEmpty};
    std::uint64_t contributing_source_count{};
    double delta_p50_db{};
};

struct Cn0NormalizedBuildResult {
    Cn0AggregationSummary aggregation_summary;
    std::vector<Cn0NormalizedSourceResult> sources;
    std::vector<Cn0NormalizedBin> bins;
};

bool build_cn0_model(const std::vector<Cn0InputSource>& sources, const Cn0AggregationConfig& config,
                     Cn0BuildResult* result, std::string* error_message);

bool build_normalized_cn0_model(const std::vector<Cn0InputSource>& sources, const Cn0AggregationConfig& config,
                                const Cn0NormalizationConfig& normalization, Cn0NormalizedBuildResult* result,
                                std::string* error_message);

bool aggregate_normalized_cn0_sources(const std::vector<Cn0NormalizedSourceResult>& sources,
                                      const Cn0NormalizationConfig& normalization, std::vector<Cn0NormalizedBin>* bins,
                                      std::string* error_message);

bool write_cn0_metadata_json(const std::string& output_path, const Cn0AggregationConfig& config,
                             const Cn0BuildResult& result, std::string* error_message);

bool write_normalized_cn0_model_csv(const std::string& output_path, const std::vector<Cn0NormalizedBin>& bins,
                                    std::string* error_message);

bool write_normalized_cn0_metadata_json(const std::string& output_path, const Cn0AggregationConfig& config,
                                        const Cn0NormalizationConfig& normalization,
                                        const Cn0NormalizedBuildResult& result, std::string* error_message);

const char* cn0_reference_status_name(Cn0ReferenceStatus status);

} // namespace gnss_sim::cn0_builder

#endif // GNSS_SIM_TOOLS_BUILD_CN0_MODEL_CN0_BUILDER_H_
