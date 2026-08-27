#include "tools/build_cn0_model/cn0_statistics.h"

#include "gnss/signal_definitions.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gnss_sim::cn0_builder {
namespace {

constexpr int kCn0QuarterBuckets = 256;
constexpr double kQuarterDbHz = 0.25;
constexpr double kConfigTolerance = 1e-9;

struct TemporalMoments {
    std::uint64_t count{};
    std::uint64_t sum_x{};
    std::uint64_t sum_y{};
    std::uint64_t sum_x2{};
    std::uint64_t sum_y2{};
    std::uint64_t sum_xy{};
};

struct BinCell {
    std::array<std::uint64_t, kCn0QuarterBuckets> cn0_hist{};
    std::array<std::uint64_t, kCn0QuarterBuckets> delta_hist{};
    TemporalMoments temporal{};
};

struct PreviousSample {
    SimTime time{};
    int elevation_bin{-1};
    int cn0_quarters{};
};

struct WeightedValue {
    double value{};
    std::uint64_t count{};
};

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool finite_config(const Cn0AggregationConfig& config) {
    return std::isfinite(config.elevation_min_deg) && std::isfinite(config.elevation_max_deg) &&
           std::isfinite(config.elevation_bin_width_deg) && std::isfinite(config.temporal_gap_tolerance_sec);
}

bool validate_config(const Cn0AggregationConfig& config, int* bin_count, std::string* error_message) {
    if (!finite_config(config) || config.elevation_bin_width_deg <= 0.0 ||
        config.elevation_max_deg <= config.elevation_min_deg || config.temporal_gap_tolerance_sec < 0.0) {
        set_error(error_message, "CN0 aggregation config has invalid numeric bounds");
        return false;
    }
    if (config.elevation_min_deg < -90.0 || config.elevation_max_deg > 90.0) {
        set_error(error_message, "CN0 elevation model bounds must remain within [-90, 90] degrees");
        return false;
    }
    const double exact_bins = (config.elevation_max_deg - config.elevation_min_deg) / config.elevation_bin_width_deg;
    const double rounded_bins = std::round(exact_bins);
    if (std::fabs(exact_bins - rounded_bins) > kConfigTolerance || rounded_bins < 1.0 || rounded_bins > 180.0) {
        set_error(error_message, "CN0 elevation span must be an integer number of bins");
        return false;
    }
    if (bin_count != nullptr) {
        *bin_count = static_cast<int>(rounded_bins);
    }
    return true;
}

int find_signal_index(const std::vector<const SignalDefinition*>& signals, SignalId signal_id) {
    for (std::size_t index = 0; index < signals.size(); ++index) {
        if (signals[index]->signal_id == signal_id) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

bool cn0_to_quarters(double cn0_dbhz, int* quarters) {
    if (quarters == nullptr || !std::isfinite(cn0_dbhz)) {
        return false;
    }
    const double scaled = cn0_dbhz / kQuarterDbHz;
    const double rounded = std::round(scaled);
    if (std::fabs(scaled - rounded) > 1e-8 || rounded < 0.0 || rounded >= kCn0QuarterBuckets) {
        return false;
    }
    *quarters = static_cast<int>(rounded);
    return true;
}

int elevation_bin_index(const Cn0AggregationConfig& config, int bin_count, double elevation_deg) {
    if (!std::isfinite(elevation_deg) || elevation_deg < config.elevation_min_deg - kConfigTolerance ||
        elevation_deg > config.elevation_max_deg + kConfigTolerance) {
        return -1;
    }
    if (std::fabs(elevation_deg - config.elevation_max_deg) <= kConfigTolerance) {
        return bin_count - 1;
    }
    if (elevation_deg < config.elevation_min_deg) {
        elevation_deg = config.elevation_min_deg;
    }
    const double relative = (elevation_deg - config.elevation_min_deg) / config.elevation_bin_width_deg;
    const int index = static_cast<int>(std::floor(relative + kConfigTolerance));
    return index >= 0 && index < bin_count ? index : -1;
}

std::uint64_t histogram_count(const std::array<std::uint64_t, kCn0QuarterBuckets>& histogram) {
    std::uint64_t count = 0;
    for (const std::uint64_t value : histogram) {
        count += value;
    }
    return count;
}

double histogram_order_stat(const std::array<std::uint64_t, kCn0QuarterBuckets>& histogram, std::uint64_t rank) {
    std::uint64_t cumulative = 0;
    for (int bucket = 0; bucket < kCn0QuarterBuckets; ++bucket) {
        cumulative += histogram[static_cast<std::size_t>(bucket)];
        if (rank < cumulative) {
            return static_cast<double>(bucket) * kQuarterDbHz;
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

double histogram_quantile(const std::array<std::uint64_t, kCn0QuarterBuckets>& histogram, double probability) {
    const std::uint64_t count = histogram_count(histogram);
    if (count == 0 || !std::isfinite(probability) || probability < 0.0 || probability > 1.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (count == 1) {
        return histogram_order_stat(histogram, 0);
    }
    const double h = static_cast<double>(count - 1) * probability;
    const std::uint64_t lower_rank = static_cast<std::uint64_t>(std::floor(h));
    const std::uint64_t upper_rank = static_cast<std::uint64_t>(std::ceil(h));
    const double lower = histogram_order_stat(histogram, lower_rank);
    const double upper = histogram_order_stat(histogram, upper_rank);
    return lower + (h - static_cast<double>(lower_rank)) * (upper - lower);
}

double weighted_order_stat(const std::vector<WeightedValue>& values, std::uint64_t rank) {
    std::uint64_t cumulative = 0;
    for (const WeightedValue& item : values) {
        cumulative += item.count;
        if (rank < cumulative) {
            return item.value;
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

double weighted_quantile(const std::vector<WeightedValue>& values, std::uint64_t count, double probability) {
    if (count == 0 || values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (count == 1) {
        return weighted_order_stat(values, 0);
    }
    const double h = static_cast<double>(count - 1) * probability;
    const std::uint64_t lower_rank = static_cast<std::uint64_t>(std::floor(h));
    const std::uint64_t upper_rank = static_cast<std::uint64_t>(std::ceil(h));
    const double lower = weighted_order_stat(values, lower_rank);
    const double upper = weighted_order_stat(values, upper_rank);
    return lower + (h - static_cast<double>(lower_rank)) * (upper - lower);
}

double histogram_mad(const std::array<std::uint64_t, kCn0QuarterBuckets>& histogram, double median) {
    const std::uint64_t count = histogram_count(histogram);
    if (count == 0 || !std::isfinite(median)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::vector<WeightedValue> deviations;
    deviations.reserve(kCn0QuarterBuckets);
    for (int bucket = 0; bucket < kCn0QuarterBuckets; ++bucket) {
        const std::uint64_t bucket_count = histogram[static_cast<std::size_t>(bucket)];
        if (bucket_count == 0) {
            continue;
        }
        deviations.push_back({std::fabs(static_cast<double>(bucket) * kQuarterDbHz - median), bucket_count});
    }
    std::sort(deviations.begin(), deviations.end(),
              [](const WeightedValue& left, const WeightedValue& right) { return left.value < right.value; });
    return weighted_quantile(deviations, count, 0.5);
}

void histogram_mean_stddev(const std::array<std::uint64_t, kCn0QuarterBuckets>& histogram, double* mean,
                           double* stddev) {
    const std::uint64_t count = histogram_count(histogram);
    if (mean == nullptr || stddev == nullptr || count == 0) {
        return;
    }
    double sum = 0.0;
    double sum_square = 0.0;
    for (int bucket = 0; bucket < kCn0QuarterBuckets; ++bucket) {
        const double value = static_cast<double>(bucket) * kQuarterDbHz;
        const double weight = static_cast<double>(histogram[static_cast<std::size_t>(bucket)]);
        sum += value * weight;
        sum_square += value * value * weight;
    }
    *mean = sum / static_cast<double>(count);
    const double variance = std::max(0.0, sum_square / static_cast<double>(count) - (*mean) * (*mean));
    *stddev = std::sqrt(variance);
}

std::uint64_t previous_key(int satellite_number, SignalId signal_id) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(satellite_number)) << 32U) |
           static_cast<std::uint32_t>(signal_id);
}

double sim_time_difference_sec(const SimTime& current, const SimTime& previous) {
    const double week_seconds = static_cast<double>(current.gps_week - previous.gps_week) * 604800.0;
    const double tow_seconds = static_cast<double>(current.tow_ns - previous.tow_ns) / 1000000000.0;
    return week_seconds + tow_seconds;
}

bool temporal_ar1(const TemporalMoments& moments, std::uint64_t minimum_pairs, double* ar1, Cn0Ar1Status* status) {
    if (ar1 == nullptr || status == nullptr) {
        return false;
    }
    if (moments.count < minimum_pairs || moments.count < 2) {
        *status = Cn0Ar1Status::kInsufficientSupport;
        *ar1 = std::numeric_limits<double>::quiet_NaN();
        return true;
    }
    const double n = static_cast<double>(moments.count);
    const double sum_x = static_cast<double>(moments.sum_x);
    const double sum_y = static_cast<double>(moments.sum_y);
    const double numerator = n * static_cast<double>(moments.sum_xy) - sum_x * sum_y;
    const double variance_x = n * static_cast<double>(moments.sum_x2) - sum_x * sum_x;
    const double variance_y = n * static_cast<double>(moments.sum_y2) - sum_y * sum_y;
    if (variance_x <= 0.0 || variance_y <= 0.0) {
        *status = Cn0Ar1Status::kZeroVariance;
        *ar1 = std::numeric_limits<double>::quiet_NaN();
        return true;
    }
    *status = Cn0Ar1Status::kAvailable;
    *ar1 = numerator / std::sqrt(variance_x * variance_y);
    return true;
}

void write_optional(std::ostream& output, bool available, double value) {
    if (available && std::isfinite(value)) {
        output << value;
    }
}

} // namespace

struct Cn0StatisticsAccumulator::Impl {
    explicit Impl(const Cn0AggregationConfig& input_config) : config(input_config) {
        config_valid = validate_config(config, &bin_count, &config_error);
        std::size_t signal_count = 0;
        const SignalDefinition* definitions = signal_definitions(&signal_count);
        signals.reserve(signal_count);
        for (std::size_t index = 0; index < signal_count; ++index) {
            signals.push_back(definitions + index);
        }
        if (config_valid) {
            cells.resize(signals.size() * static_cast<std::size_t>(bin_count));
        }
    }

    Cn0AggregationConfig config{};
    Cn0AggregationSummary summary{};
    bool config_valid{};
    std::string config_error;
    int bin_count{};
    std::vector<const SignalDefinition*> signals;
    std::vector<BinCell> cells;
    bool source_active{};
    double source_interval_sec{std::numeric_limits<double>::quiet_NaN()};
    std::unordered_map<std::uint64_t, PreviousSample> previous;
};

Cn0StatisticsAccumulator::Cn0StatisticsAccumulator(const Cn0AggregationConfig& config) : impl_(new Impl(config)) {}

Cn0StatisticsAccumulator::~Cn0StatisticsAccumulator() {
    delete impl_;
}

bool Cn0StatisticsAccumulator::valid(std::string* error_message) const {
    if (impl_ == nullptr || !impl_->config_valid) {
        set_error(error_message, impl_ != nullptr ? impl_->config_error : "CN0 accumulator is not initialized");
        return false;
    }
    return true;
}

bool Cn0StatisticsAccumulator::begin_source(double observation_interval_sec, std::string* error_message) {
    if (!valid(error_message)) {
        return false;
    }
    impl_->previous.clear();
    impl_->source_active = true;
    impl_->source_interval_sec = std::isfinite(observation_interval_sec) && observation_interval_sec > 0.0
                                     ? observation_interval_sec
                                     : std::numeric_limits<double>::quiet_NaN();
    ++impl_->summary.sources;
    return true;
}

bool Cn0StatisticsAccumulator::add_sample(const RinexCn0Sample& sample, std::string* error_message) {
    if (!valid(error_message)) {
        return false;
    }
    if (!impl_->source_active) {
        set_error(error_message, "begin_source() must be called before CN0 samples are aggregated");
        return false;
    }
    ++impl_->summary.input_samples;
    if (sample.validity != Cn0SampleValidity::kValidDbHz) {
        ++impl_->summary.rejected_validity;
        return true;
    }
    if (!std::isfinite(sample.cn0_dbhz) || !std::isfinite(sample.elevation_rad)) {
        ++impl_->summary.rejected_nonfinite;
        return true;
    }

    int cn0_quarters = 0;
    if (!cn0_to_quarters(sample.cn0_dbhz, &cn0_quarters)) {
        ++impl_->summary.rejected_cn0_grid;
        return true;
    }
    const double elevation_deg = sample.elevation_rad * 180.0 / 3.14159265358979323846;
    const int bin_index = elevation_bin_index(impl_->config, impl_->bin_count, elevation_deg);
    if (bin_index < 0) {
        ++impl_->summary.rejected_elevation_range;
        return true;
    }
    const int signal_index = find_signal_index(impl_->signals, sample.signal_id);
    if (signal_index < 0) {
        set_error(error_message, "CN0 sample uses a signal that is absent from the central signal-definition table");
        return false;
    }

    BinCell& cell = impl_->cells[static_cast<std::size_t>(signal_index * impl_->bin_count + bin_index)];
    ++cell.cn0_hist[static_cast<std::size_t>(cn0_quarters)];
    ++impl_->summary.accepted_samples;

    const std::uint64_t key = previous_key(sample.satellite_number, sample.signal_id);
    const auto previous_it = impl_->previous.find(key);
    if (previous_it != impl_->previous.end()) {
        const PreviousSample& previous = previous_it->second;
        if (!std::isfinite(impl_->source_interval_sec)) {
            ++impl_->summary.temporal_rejected_no_interval;
        } else {
            const double gap_sec = sim_time_difference_sec(sample.time, previous.time);
            if (std::fabs(gap_sec - impl_->source_interval_sec) > impl_->config.temporal_gap_tolerance_sec) {
                ++impl_->summary.temporal_rejected_gap;
            } else if (previous.elevation_bin != bin_index) {
                ++impl_->summary.temporal_rejected_bin_change;
            } else {
                const int delta_quarters = std::abs(cn0_quarters - previous.cn0_quarters);
                ++cell.delta_hist[static_cast<std::size_t>(delta_quarters)];
                ++cell.temporal.count;
                cell.temporal.sum_x += static_cast<std::uint64_t>(previous.cn0_quarters);
                cell.temporal.sum_y += static_cast<std::uint64_t>(cn0_quarters);
                cell.temporal.sum_x2 += static_cast<std::uint64_t>(previous.cn0_quarters * previous.cn0_quarters);
                cell.temporal.sum_y2 += static_cast<std::uint64_t>(cn0_quarters * cn0_quarters);
                cell.temporal.sum_xy += static_cast<std::uint64_t>(previous.cn0_quarters * cn0_quarters);
                ++impl_->summary.temporal_pairs;
            }
        }
    }
    impl_->previous[key] = {sample.time, bin_index, cn0_quarters};
    return true;
}

void Cn0StatisticsAccumulator::end_source() {
    if (impl_ == nullptr) {
        return;
    }
    impl_->previous.clear();
    impl_->source_active = false;
    impl_->source_interval_sec = std::numeric_limits<double>::quiet_NaN();
}

const Cn0AggregationConfig& Cn0StatisticsAccumulator::config() const {
    return impl_->config;
}

const Cn0AggregationSummary& Cn0StatisticsAccumulator::summary() const {
    return impl_->summary;
}

std::vector<Cn0BinStatistics> Cn0StatisticsAccumulator::finalize() const {
    std::vector<Cn0BinStatistics> output;
    if (impl_ == nullptr || !impl_->config_valid) {
        return output;
    }
    output.reserve(impl_->cells.size());
    for (std::size_t signal_index = 0; signal_index < impl_->signals.size(); ++signal_index) {
        const SignalDefinition& signal = *impl_->signals[signal_index];
        for (int bin_index = 0; bin_index < impl_->bin_count; ++bin_index) {
            const BinCell& cell = impl_->cells[signal_index * static_cast<std::size_t>(impl_->bin_count) +
                                               static_cast<std::size_t>(bin_index)];
            Cn0BinStatistics statistics{};
            statistics.constellation = signal.constellation;
            statistics.signal_id = signal.signal_id;
            statistics.rinex_signal_code = signal.rinex_signal_code;
            statistics.elevation_min_deg = impl_->config.elevation_min_deg +
                                           static_cast<double>(bin_index) * impl_->config.elevation_bin_width_deg;
            statistics.elevation_max_deg = statistics.elevation_min_deg + impl_->config.elevation_bin_width_deg;
            statistics.includes_upper_edge = bin_index == impl_->bin_count - 1;
            statistics.count = histogram_count(cell.cn0_hist);
            if (statistics.count == 0) {
                statistics.status = Cn0BinStatus::kEmpty;
            } else if (statistics.count < impl_->config.min_samples_per_bin) {
                statistics.status = Cn0BinStatus::kSparse;
            } else {
                statistics.status = Cn0BinStatus::kReady;
            }

            if (statistics.count > 0) {
                statistics.p05_dbhz = histogram_quantile(cell.cn0_hist, 0.05);
                statistics.p10_dbhz = histogram_quantile(cell.cn0_hist, 0.10);
                statistics.p25_dbhz = histogram_quantile(cell.cn0_hist, 0.25);
                statistics.p50_dbhz = histogram_quantile(cell.cn0_hist, 0.50);
                statistics.p75_dbhz = histogram_quantile(cell.cn0_hist, 0.75);
                statistics.p90_dbhz = histogram_quantile(cell.cn0_hist, 0.90);
                statistics.p95_dbhz = histogram_quantile(cell.cn0_hist, 0.95);
                histogram_mean_stddev(cell.cn0_hist, &statistics.mean_dbhz, &statistics.stddev_dbhz);
                statistics.mad_dbhz = histogram_mad(cell.cn0_hist, statistics.p50_dbhz);
            } else {
                const double unavailable = std::numeric_limits<double>::quiet_NaN();
                statistics.p05_dbhz = unavailable;
                statistics.p10_dbhz = unavailable;
                statistics.p25_dbhz = unavailable;
                statistics.p50_dbhz = unavailable;
                statistics.p75_dbhz = unavailable;
                statistics.p90_dbhz = unavailable;
                statistics.p95_dbhz = unavailable;
                statistics.mean_dbhz = unavailable;
                statistics.stddev_dbhz = unavailable;
                statistics.mad_dbhz = unavailable;
            }

            statistics.delta_count = histogram_count(cell.delta_hist);
            if (statistics.delta_count > 0) {
                statistics.delta_p50_dbhz = histogram_quantile(cell.delta_hist, 0.50);
                statistics.delta_p90_dbhz = histogram_quantile(cell.delta_hist, 0.90);
                statistics.delta_p99_dbhz = histogram_quantile(cell.delta_hist, 0.99);
            } else {
                const double unavailable = std::numeric_limits<double>::quiet_NaN();
                statistics.delta_p50_dbhz = unavailable;
                statistics.delta_p90_dbhz = unavailable;
                statistics.delta_p99_dbhz = unavailable;
            }
            temporal_ar1(cell.temporal, impl_->config.min_temporal_pairs, &statistics.ar1, &statistics.ar1_status);
            output.push_back(std::move(statistics));
        }
    }
    return output;
}

std::size_t Cn0StatisticsAccumulator::bounded_histogram_cells() const {
    if (impl_ == nullptr) {
        return 0;
    }
    return impl_->cells.size() * static_cast<std::size_t>(kCn0QuarterBuckets) * 2U;
}

const char* cn0_bin_status_name(Cn0BinStatus status) {
    switch (status) {
        case Cn0BinStatus::kEmpty:
            return "EMPTY";
        case Cn0BinStatus::kSparse:
            return "SPARSE";
        case Cn0BinStatus::kReady:
            return "READY";
    }
    return "UNKNOWN";
}

const char* cn0_ar1_status_name(Cn0Ar1Status status) {
    switch (status) {
        case Cn0Ar1Status::kAvailable:
            return "AVAILABLE";
        case Cn0Ar1Status::kInsufficientSupport:
            return "INSUFFICIENT_SUPPORT";
        case Cn0Ar1Status::kZeroVariance:
            return "ZERO_VARIANCE";
    }
    return "UNKNOWN";
}

bool write_cn0_model_csv(const std::string& output_path, const Cn0AggregationConfig& config,
                         const Cn0AggregationSummary& summary, const std::vector<Cn0BinStatistics>& bins,
                         std::string* error_message) {
    std::ofstream output(output_path, std::ios::binary);
    if (!output) {
        set_error(error_message, "cannot open CN0 model CSV for writing: " + output_path);
        return false;
    }
    output.imbue(std::locale::classic());
    output << std::fixed << std::setprecision(6);
    output
        << "schema_version,constellation,signal,elevation_min_deg,elevation_max_deg,upper_edge_inclusive,status,count,"
           "p05_dbhz,p10_dbhz,p25_dbhz,p50_dbhz,p75_dbhz,p90_dbhz,p95_dbhz,mean_dbhz,stddev_dbhz,mad_dbhz,"
           "delta_count,delta_p50_dbhz,delta_p90_dbhz,delta_p99_dbhz,ar1_status,ar1\n";
    for (const Cn0BinStatistics& bin : bins) {
        output << "gnss-cn0-model-v1," << constellation_name(bin.constellation) << ',' << bin.rinex_signal_code << ','
               << bin.elevation_min_deg << ',' << bin.elevation_max_deg << ',' << (bin.includes_upper_edge ? 1 : 0)
               << ',' << cn0_bin_status_name(bin.status) << ',' << bin.count << ',';
        const bool has_samples = bin.count > 0;
        write_optional(output, has_samples, bin.p05_dbhz);
        output << ',';
        write_optional(output, has_samples, bin.p10_dbhz);
        output << ',';
        write_optional(output, has_samples, bin.p25_dbhz);
        output << ',';
        write_optional(output, has_samples, bin.p50_dbhz);
        output << ',';
        write_optional(output, has_samples, bin.p75_dbhz);
        output << ',';
        write_optional(output, has_samples, bin.p90_dbhz);
        output << ',';
        write_optional(output, has_samples, bin.p95_dbhz);
        output << ',';
        write_optional(output, has_samples, bin.mean_dbhz);
        output << ',';
        write_optional(output, has_samples, bin.stddev_dbhz);
        output << ',';
        write_optional(output, has_samples, bin.mad_dbhz);
        output << ',' << bin.delta_count << ',';
        const bool has_delta = bin.delta_count > 0;
        write_optional(output, has_delta, bin.delta_p50_dbhz);
        output << ',';
        write_optional(output, has_delta, bin.delta_p90_dbhz);
        output << ',';
        write_optional(output, has_delta, bin.delta_p99_dbhz);
        output << ',' << cn0_ar1_status_name(bin.ar1_status) << ',';
        write_optional(output, bin.ar1_status == Cn0Ar1Status::kAvailable, bin.ar1);
        output << '\n';
    }
    if (!output) {
        set_error(error_message, "failed while writing CN0 model CSV: " + output_path);
        return false;
    }
    (void)config;
    (void)summary;
    return true;
}

} // namespace gnss_sim::cn0_builder
