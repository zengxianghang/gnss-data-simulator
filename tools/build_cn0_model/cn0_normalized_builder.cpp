#include "gnss/signal_definitions.h"
#include "tools/build_cn0_model/cn0_builder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

namespace gnss_sim::cn0_builder {
namespace {

constexpr int kCn0QuarterBuckets = 256;
constexpr double kQuarterDbHz = 0.25;
constexpr double kPi = 3.14159265358979323846;

struct ReferenceAccumulator {
    std::vector<const SignalDefinition*> signals;
    std::vector<std::array<std::uint64_t, kCn0QuarterBuckets>> histograms;
};

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_normalization_config(const Cn0AggregationConfig& aggregation, const Cn0NormalizationConfig& normalization,
                                std::string* error_message) {
    if (!std::isfinite(normalization.reference_elevation_min_deg) ||
        !std::isfinite(normalization.reference_elevation_max_deg) ||
        normalization.reference_elevation_max_deg <= normalization.reference_elevation_min_deg ||
        normalization.reference_elevation_min_deg < aggregation.elevation_min_deg ||
        normalization.reference_elevation_max_deg > aggregation.elevation_max_deg ||
        normalization.min_reference_samples == 0 || normalization.min_sources_per_bin == 0) {
        set_error(error_message, "normalized CN0 configuration has invalid reference/support bounds");
        return false;
    }
    return true;
}

ReferenceAccumulator make_reference_accumulator() {
    ReferenceAccumulator accumulator{};
    std::size_t signal_count = 0;
    const SignalDefinition* definitions = signal_definitions(&signal_count);
    accumulator.signals.reserve(signal_count);
    accumulator.histograms.resize(signal_count);
    for (std::size_t index = 0; index < signal_count; ++index) {
        accumulator.signals.push_back(definitions + index);
    }
    return accumulator;
}

int find_signal_index(const ReferenceAccumulator& accumulator, SignalId signal_id) {
    for (std::size_t index = 0; index < accumulator.signals.size(); ++index) {
        if (accumulator.signals[index]->signal_id == signal_id) {
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

bool add_reference_sample(ReferenceAccumulator* accumulator, const Cn0NormalizationConfig& normalization,
                          const RinexCn0Sample& sample, std::string* error_message) {
    if (accumulator == nullptr) {
        set_error(error_message, "normalized CN0 reference accumulator is null");
        return false;
    }
    if (sample.validity != Cn0SampleValidity::kValidDbHz || !std::isfinite(sample.cn0_dbhz) ||
        !std::isfinite(sample.elevation_rad)) {
        return true;
    }
    const double elevation_deg = sample.elevation_rad * 180.0 / kPi;
    if (elevation_deg < normalization.reference_elevation_min_deg ||
        elevation_deg > normalization.reference_elevation_max_deg) {
        return true;
    }
    int quarters = 0;
    if (!cn0_to_quarters(sample.cn0_dbhz, &quarters)) {
        return true;
    }
    const int signal_index = find_signal_index(*accumulator, sample.signal_id);
    if (signal_index < 0) {
        set_error(error_message, "normalized CN0 reference sample uses an unknown central signal");
        return false;
    }
    ++accumulator->histograms[static_cast<std::size_t>(signal_index)][static_cast<std::size_t>(quarters)];
    return true;
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

double histogram_median(const std::array<std::uint64_t, kCn0QuarterBuckets>& histogram) {
    const std::uint64_t count = histogram_count(histogram);
    if (count == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (count == 1) {
        return histogram_order_stat(histogram, 0);
    }
    const double rank = 0.5 * static_cast<double>(count - 1);
    const std::uint64_t lower_rank = static_cast<std::uint64_t>(std::floor(rank));
    const std::uint64_t upper_rank = static_cast<std::uint64_t>(std::ceil(rank));
    const double lower = histogram_order_stat(histogram, lower_rank);
    const double upper = histogram_order_stat(histogram, upper_rank);
    return lower + (rank - static_cast<double>(lower_rank)) * (upper - lower);
}

std::vector<Cn0SignalReference> finalize_references(const ReferenceAccumulator& accumulator,
                                                    const Cn0NormalizationConfig& normalization) {
    std::vector<Cn0SignalReference> references;
    references.reserve(accumulator.signals.size());
    for (std::size_t index = 0; index < accumulator.signals.size(); ++index) {
        const SignalDefinition& signal = *accumulator.signals[index];
        Cn0SignalReference reference{};
        reference.constellation = signal.constellation;
        reference.signal_id = signal.signal_id;
        reference.rinex_signal_code = signal.rinex_signal_code;
        reference.count = histogram_count(accumulator.histograms[index]);
        if (reference.count >= normalization.min_reference_samples) {
            reference.status = Cn0ReferenceStatus::kReady;
            reference.p50_dbhz = histogram_median(accumulator.histograms[index]);
        } else {
            reference.status = Cn0ReferenceStatus::kInsufficientSupport;
            reference.p50_dbhz = std::numeric_limits<double>::quiet_NaN();
        }
        references.push_back(std::move(reference));
    }
    return references;
}

const Cn0SignalReference* find_reference(const std::vector<Cn0SignalReference>& references, SignalId signal_id) {
    for (const Cn0SignalReference& reference : references) {
        if (reference.signal_id == signal_id) {
            return &reference;
        }
    }
    return nullptr;
}

void add_summary(const Cn0AggregationSummary& source, Cn0AggregationSummary* destination) {
    destination->input_samples += source.input_samples;
    destination->accepted_samples += source.accepted_samples;
    destination->rejected_validity += source.rejected_validity;
    destination->rejected_nonfinite += source.rejected_nonfinite;
    destination->rejected_cn0_grid += source.rejected_cn0_grid;
    destination->rejected_elevation_range += source.rejected_elevation_range;
    destination->temporal_pairs += source.temporal_pairs;
    destination->temporal_rejected_no_interval += source.temporal_rejected_no_interval;
    destination->temporal_rejected_gap += source.temporal_rejected_gap;
    destination->temporal_rejected_bin_change += source.temporal_rejected_bin_change;
    destination->sources += source.sources;
}

double source_median(std::vector<double> values) {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    if ((values.size() & 1U) != 0U) {
        return values[middle];
    }
    return 0.5 * (values[middle - 1U] + values[middle]);
}

bool same_bin_key(const Cn0SourceNormalizedBin& left, const Cn0SourceNormalizedBin& right) {
    return left.signal_id == right.signal_id && left.rinex_signal_code == right.rinex_signal_code &&
           left.elevation_min_deg == right.elevation_min_deg && left.elevation_max_deg == right.elevation_max_deg &&
           left.includes_upper_edge == right.includes_upper_edge;
}

std::string json_escape(const std::string& value) {
    std::ostringstream output;
    for (const unsigned char character : value) {
        if (character == '\"' || character == '\\') {
            output << '\\' << static_cast<char>(character);
        } else if (character == '\n') {
            output << "\\n";
        } else if (character == '\r') {
            output << "\\r";
        } else if (character == '\t') {
            output << "\\t";
        } else if (character < 0x20U) {
            output << "?";
        } else {
            output << static_cast<char>(character);
        }
    }
    return output.str();
}

std::string hex64(std::uint64_t value) {
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::hex << std::setfill('0') << std::setw(16) << value;
    return output.str();
}

} // namespace

const char* cn0_reference_status_name(Cn0ReferenceStatus status) {
    switch (status) {
        case Cn0ReferenceStatus::kReady:
            return "READY";
        case Cn0ReferenceStatus::kInsufficientSupport:
            return "INSUFFICIENT_SUPPORT";
    }
    return "UNKNOWN";
}

bool aggregate_normalized_cn0_sources(const std::vector<Cn0NormalizedSourceResult>& sources,
                                      const Cn0NormalizationConfig& normalization, std::vector<Cn0NormalizedBin>* bins,
                                      std::string* error_message) {
    if (bins == nullptr || sources.empty() || normalization.min_sources_per_bin == 0) {
        set_error(error_message, "normalized CN0 aggregation requires sources/output and positive source support");
        return false;
    }
    const std::size_t bin_count = sources.front().bins.size();
    if (bin_count == 0) {
        set_error(error_message, "normalized CN0 aggregation has no source bins");
        return false;
    }
    for (const Cn0NormalizedSourceResult& source : sources) {
        if (source.bins.size() != bin_count) {
            set_error(error_message, "normalized CN0 sources use inconsistent bin layouts");
            return false;
        }
    }

    std::vector<Cn0NormalizedBin> output;
    output.reserve(bin_count);
    for (std::size_t index = 0; index < bin_count; ++index) {
        const Cn0SourceNormalizedBin& prototype = sources.front().bins[index];
        std::vector<double> deltas;
        deltas.reserve(sources.size());
        for (const Cn0NormalizedSourceResult& source : sources) {
            const Cn0SourceNormalizedBin& bin = source.bins[index];
            if (!same_bin_key(prototype, bin)) {
                set_error(error_message, "normalized CN0 sources use inconsistent signal/elevation ordering");
                return false;
            }
            if (bin.reference_ready && bin.source_status == Cn0BinStatus::kReady && std::isfinite(bin.delta_p50_db)) {
                deltas.push_back(bin.delta_p50_db);
            }
        }

        Cn0NormalizedBin aggregate{};
        aggregate.constellation = prototype.constellation;
        aggregate.signal_id = prototype.signal_id;
        aggregate.rinex_signal_code = prototype.rinex_signal_code;
        aggregate.elevation_min_deg = prototype.elevation_min_deg;
        aggregate.elevation_max_deg = prototype.elevation_max_deg;
        aggregate.includes_upper_edge = prototype.includes_upper_edge;
        aggregate.contributing_source_count = static_cast<std::uint64_t>(deltas.size());
        if (deltas.empty()) {
            aggregate.status = Cn0BinStatus::kEmpty;
            aggregate.delta_p50_db = std::numeric_limits<double>::quiet_NaN();
        } else {
            aggregate.delta_p50_db = source_median(std::move(deltas));
            aggregate.status = aggregate.contributing_source_count < normalization.min_sources_per_bin
                                   ? Cn0BinStatus::kSparse
                                   : Cn0BinStatus::kReady;
        }
        output.push_back(std::move(aggregate));
    }

    *bins = std::move(output);
    return true;
}

bool build_normalized_cn0_model(const std::vector<Cn0InputSource>& sources, const Cn0AggregationConfig& config,
                                const Cn0NormalizationConfig& normalization, Cn0NormalizedBuildResult* result,
                                std::string* error_message) {
    if (result == nullptr || sources.empty() || !valid_normalization_config(config, normalization, error_message)) {
        if (result == nullptr || sources.empty()) {
            set_error(error_message, "normalized CN0 build requires sources and a result output");
        }
        return false;
    }

    Cn0NormalizedBuildResult build{};
    build.sources.reserve(sources.size());
    for (const Cn0InputSource& source : sources) {
        Cn0BuildResult absolute{};
        if (!build_cn0_model({source}, config, &absolute, error_message) || absolute.sources.size() != 1U) {
            return false;
        }

        ReferenceAccumulator reference_accumulator = make_reference_accumulator();
        RinexObsProvenance reference_provenance{};
        RinexObsStreamSummary reference_summary{};
        if (!stream_rinex_cn0_samples(
                source.observation_path, source.navigation_path,
                [&](const RinexCn0Sample& sample) {
                    return add_reference_sample(&reference_accumulator, normalization, sample, error_message);
                },
                &reference_provenance, &reference_summary, error_message)) {
            return false;
        }

        Cn0NormalizedSourceResult normalized_source{};
        normalized_source.metadata = std::move(absolute.sources.front());
        normalized_source.references = finalize_references(reference_accumulator, normalization);
        normalized_source.bins.reserve(absolute.bins.size());
        for (const Cn0BinStatistics& bin : absolute.bins) {
            Cn0SourceNormalizedBin normalized_bin{};
            normalized_bin.constellation = bin.constellation;
            normalized_bin.signal_id = bin.signal_id;
            normalized_bin.rinex_signal_code = bin.rinex_signal_code;
            normalized_bin.elevation_min_deg = bin.elevation_min_deg;
            normalized_bin.elevation_max_deg = bin.elevation_max_deg;
            normalized_bin.includes_upper_edge = bin.includes_upper_edge;
            normalized_bin.source_status = bin.status;
            normalized_bin.sample_count = bin.count;
            const Cn0SignalReference* reference = find_reference(normalized_source.references, bin.signal_id);
            normalized_bin.reference_ready = reference != nullptr && reference->status == Cn0ReferenceStatus::kReady &&
                                             std::isfinite(reference->p50_dbhz);
            if (normalized_bin.reference_ready && bin.count > 0 && std::isfinite(bin.p50_dbhz)) {
                normalized_bin.delta_p50_db = bin.p50_dbhz - reference->p50_dbhz;
            } else {
                normalized_bin.delta_p50_db = std::numeric_limits<double>::quiet_NaN();
            }
            normalized_source.bins.push_back(std::move(normalized_bin));
        }
        add_summary(absolute.aggregation_summary, &build.aggregation_summary);
        build.sources.push_back(std::move(normalized_source));
    }

    std::sort(build.sources.begin(), build.sources.end(),
              [](const Cn0NormalizedSourceResult& left, const Cn0NormalizedSourceResult& right) {
                  if (left.metadata.observation_file.file_name != right.metadata.observation_file.file_name) {
                      return left.metadata.observation_file.file_name < right.metadata.observation_file.file_name;
                  }
                  if (left.metadata.observation_file.fnv1a64 != right.metadata.observation_file.fnv1a64) {
                      return left.metadata.observation_file.fnv1a64 < right.metadata.observation_file.fnv1a64;
                  }
                  return left.metadata.navigation_file.fnv1a64 < right.metadata.navigation_file.fnv1a64;
              });

    if (!aggregate_normalized_cn0_sources(build.sources, normalization, &build.bins, error_message)) {
        return false;
    }
    *result = std::move(build);
    return true;
}

bool write_normalized_cn0_model_csv(const std::string& output_path, const std::vector<Cn0NormalizedBin>& bins,
                                    std::string* error_message) {
    std::ofstream output(output_path, std::ios::binary);
    if (!output) {
        set_error(error_message, "cannot open normalized CN0 model CSV for writing: " + output_path);
        return false;
    }
    output.imbue(std::locale::classic());
    output << std::fixed << std::setprecision(6);
    output << "schema_version,model_semantic,constellation,signal,elevation_min_deg,elevation_max_deg,"
              "upper_edge_inclusive,status,contributing_source_count,delta_p50_db\n";
    for (const Cn0NormalizedBin& bin : bins) {
        output << "gnss-cn0-model-v2,NORMALIZED_ELEVATION_SHAPE," << constellation_name(bin.constellation) << ','
               << bin.rinex_signal_code << ',' << bin.elevation_min_deg << ',' << bin.elevation_max_deg << ','
               << (bin.includes_upper_edge ? 1 : 0) << ',' << cn0_bin_status_name(bin.status) << ','
               << bin.contributing_source_count << ',';
        if (bin.contributing_source_count > 0 && std::isfinite(bin.delta_p50_db)) {
            output << bin.delta_p50_db;
        }
        output << '\n';
    }
    if (!output) {
        set_error(error_message, "failed while writing normalized CN0 model CSV: " + output_path);
        return false;
    }
    return true;
}

bool write_normalized_cn0_metadata_json(const std::string& output_path, const Cn0AggregationConfig& config,
                                        const Cn0NormalizationConfig& normalization,
                                        const Cn0NormalizedBuildResult& result, std::string* error_message) {
    std::ofstream output(output_path, std::ios::binary);
    if (!output) {
        set_error(error_message, "cannot open normalized CN0 metadata JSON for writing: " + output_path);
        return false;
    }
    output.imbue(std::locale::classic());
    output << std::fixed << std::setprecision(6);
    output << "{\n";
    output << "  \"schema_version\":\"gnss-cn0-metadata-v2\",\n";
    output << "  \"model_schema_version\":\"gnss-cn0-model-v2\",\n";
    output << "  \"model_semantic\":\"NORMALIZED_ELEVATION_SHAPE\",\n";
    output << "  \"normalization\":{\"reference_elevation_min_deg\":" << normalization.reference_elevation_min_deg
           << ",\"reference_elevation_max_deg\":" << normalization.reference_elevation_max_deg
           << ",\"reference_statistic\":\"P50_R7\","
           << "\"min_reference_samples\":" << normalization.min_reference_samples
           << ",\"cross_source_statistic\":\"P50_EQUAL_SOURCE_WEIGHT\",\"min_sources_per_bin\":"
           << normalization.min_sources_per_bin << "},\n";
    output << "  \"elevation_bins\":{\"min_deg\":" << config.elevation_min_deg
           << ",\"max_deg\":" << config.elevation_max_deg << ",\"width_deg\":" << config.elevation_bin_width_deg
           << "},\n";
    output << "  \"sources\":[\n";
    for (std::size_t source_index = 0; source_index < result.sources.size(); ++source_index) {
        const Cn0NormalizedSourceResult& source = result.sources[source_index];
        output << "    {\"observation_file\":\"" << json_escape(source.metadata.observation_file.file_name)
               << "\",\"observation_fnv1a64\":\"" << hex64(source.metadata.observation_file.fnv1a64)
               << "\",\"navigation_file\":\"" << json_escape(source.metadata.navigation_file.file_name)
               << "\",\"navigation_fnv1a64\":\"" << hex64(source.metadata.navigation_file.fnv1a64)
               << "\",\"station_name\":\"" << json_escape(source.metadata.provenance.station_name)
               << "\",\"receiver_type\":\"" << json_escape(source.metadata.provenance.receiver_type)
               << "\",\"antenna_type\":\"" << json_escape(source.metadata.provenance.antenna_type)
               << "\",\"references\":[";
        for (std::size_t reference_index = 0; reference_index < source.references.size(); ++reference_index) {
            const Cn0SignalReference& reference = source.references[reference_index];
            if (reference_index > 0) {
                output << ',';
            }
            output << "{\"signal\":\"" << reference.rinex_signal_code << "\",\"status\":\""
                   << cn0_reference_status_name(reference.status) << "\",\"count\":" << reference.count
                   << ",\"p50_dbhz\":";
            if (reference.status == Cn0ReferenceStatus::kReady && std::isfinite(reference.p50_dbhz)) {
                output << reference.p50_dbhz;
            } else {
                output << "null";
            }
            output << '}';
        }
        output << "],\"bins\":[";
        for (std::size_t bin_index = 0; bin_index < source.bins.size(); ++bin_index) {
            const Cn0SourceNormalizedBin& bin = source.bins[bin_index];
            if (bin_index > 0) {
                output << ',';
            }
            output << "{\"signal\":\"" << bin.rinex_signal_code << "\",\"elevation_min_deg\":" << bin.elevation_min_deg
                   << ",\"status\":\"" << cn0_bin_status_name(bin.source_status)
                   << "\",\"sample_count\":" << bin.sample_count
                   << ",\"reference_ready\":" << (bin.reference_ready ? "true" : "false") << ",\"delta_p50_db\":";
            if (bin.reference_ready && bin.sample_count > 0 && std::isfinite(bin.delta_p50_db)) {
                output << bin.delta_p50_db;
            } else {
                output << "null";
            }
            output << '}';
        }
        output << "]}" << (source_index + 1U == result.sources.size() ? "\n" : ",\n");
    }
    output << "  ],\n";
    output << "  \"aggregate_bins\":[";
    for (std::size_t index = 0; index < result.bins.size(); ++index) {
        const Cn0NormalizedBin& bin = result.bins[index];
        if (index > 0) {
            output << ',';
        }
        output << "{\"signal\":\"" << bin.rinex_signal_code << "\",\"elevation_min_deg\":" << bin.elevation_min_deg
               << ",\"status\":\"" << cn0_bin_status_name(bin.status)
               << "\",\"contributing_source_count\":" << bin.contributing_source_count << ",\"delta_p50_db\":";
        if (bin.contributing_source_count > 0 && std::isfinite(bin.delta_p50_db)) {
            output << bin.delta_p50_db;
        } else {
            output << "null";
        }
        output << '}';
    }
    output << "]\n}\n";
    if (!output) {
        set_error(error_message, "failed while writing normalized CN0 metadata JSON: " + output_path);
        return false;
    }
    return true;
}

} // namespace gnss_sim::cn0_builder
