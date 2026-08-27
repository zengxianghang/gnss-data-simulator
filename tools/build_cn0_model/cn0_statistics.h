#ifndef GNSS_SIM_TOOLS_BUILD_CN0_MODEL_CN0_STATISTICS_H_
#define GNSS_SIM_TOOLS_BUILD_CN0_MODEL_CN0_STATISTICS_H_

#include "tools/build_cn0_model/rinex_obs_stream.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gnss_sim::cn0_builder {

struct Cn0AggregationConfig {
    double elevation_min_deg{0.0};
    double elevation_max_deg{90.0};
    double elevation_bin_width_deg{5.0};
    std::uint64_t min_samples_per_bin{20};
    std::uint64_t min_temporal_pairs{20};
    double temporal_gap_tolerance_sec{0.005};
};

enum class Cn0BinStatus {
    kEmpty,
    kSparse,
    kReady,
};

enum class Cn0Ar1Status {
    kAvailable,
    kInsufficientSupport,
    kZeroVariance,
};

struct Cn0BinStatistics {
    GnssConstellation constellation{GnssConstellation::kGps};
    SignalId signal_id{SignalId::kGpsL1Ca};
    std::string rinex_signal_code;
    double elevation_min_deg{};
    double elevation_max_deg{};
    bool includes_upper_edge{};
    Cn0BinStatus status{Cn0BinStatus::kEmpty};
    std::uint64_t count{};
    double p05_dbhz{};
    double p10_dbhz{};
    double p25_dbhz{};
    double p50_dbhz{};
    double p75_dbhz{};
    double p90_dbhz{};
    double p95_dbhz{};
    double mean_dbhz{};
    double stddev_dbhz{};
    double mad_dbhz{};
    std::uint64_t delta_count{};
    double delta_p50_dbhz{};
    double delta_p90_dbhz{};
    double delta_p99_dbhz{};
    Cn0Ar1Status ar1_status{Cn0Ar1Status::kInsufficientSupport};
    double ar1{};
};

struct Cn0AggregationSummary {
    std::uint64_t input_samples{};
    std::uint64_t accepted_samples{};
    std::uint64_t rejected_validity{};
    std::uint64_t rejected_nonfinite{};
    std::uint64_t rejected_cn0_grid{};
    std::uint64_t rejected_elevation_range{};
    std::uint64_t temporal_pairs{};
    std::uint64_t temporal_rejected_no_interval{};
    std::uint64_t temporal_rejected_gap{};
    std::uint64_t temporal_rejected_bin_change{};
    std::uint64_t sources{};
};

class Cn0StatisticsAccumulator {
  public:
    explicit Cn0StatisticsAccumulator(const Cn0AggregationConfig& config);
    ~Cn0StatisticsAccumulator();

    Cn0StatisticsAccumulator(const Cn0StatisticsAccumulator&) = delete;
    Cn0StatisticsAccumulator& operator=(const Cn0StatisticsAccumulator&) = delete;

    bool valid(std::string* error_message) const;
    bool begin_source(double observation_interval_sec, std::string* error_message);
    bool add_sample(const RinexCn0Sample& sample, std::string* error_message);
    void end_source();

    const Cn0AggregationConfig& config() const;
    const Cn0AggregationSummary& summary() const;
    std::vector<Cn0BinStatistics> finalize() const;
    std::size_t bounded_histogram_cells() const;

  private:
    struct Impl;
    Impl* impl_;
};

const char* cn0_bin_status_name(Cn0BinStatus status);
const char* cn0_ar1_status_name(Cn0Ar1Status status);

bool write_cn0_model_csv(const std::string& output_path, const Cn0AggregationConfig& config,
                         const Cn0AggregationSummary& summary, const std::vector<Cn0BinStatistics>& bins,
                         std::string* error_message);

} // namespace gnss_sim::cn0_builder

#endif // GNSS_SIM_TOOLS_BUILD_CN0_MODEL_CN0_STATISTICS_H_
