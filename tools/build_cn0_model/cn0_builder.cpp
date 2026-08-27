#include "tools/build_cn0_model/cn0_builder.h"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <utility>

namespace gnss_sim::cn0_builder {
namespace {

constexpr std::uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

std::string trim(std::string value) {
    const std::size_t begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const std::size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

bool file_identity(const std::string& path, Cn0FileIdentity* identity, std::string* error_message) {
    if (identity == nullptr) {
        set_error(error_message, "CN0 source identity output must not be null");
        return false;
    }
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        set_error(error_message, "cannot open CN0 source for identity: " + path);
        return false;
    }
    std::uint64_t hash = kFnvOffsetBasis;
    std::uint64_t size = 0;
    char buffer[65536];
    while (input) {
        input.read(buffer, sizeof(buffer));
        const std::streamsize count = input.gcount();
        for (std::streamsize index = 0; index < count; ++index) {
            hash ^= static_cast<unsigned char>(buffer[index]);
            hash *= kFnvPrime;
        }
        size += static_cast<std::uint64_t>(count);
    }
    if (!input.eof()) {
        set_error(error_message, "failed while hashing CN0 source: " + path);
        return false;
    }
    identity->file_name = std::filesystem::path(path).filename().string();
    identity->size_bytes = size;
    identity->fnv1a64 = hash;
    return true;
}

bool scan_observation_interval(const std::string& path, double* interval_sec, bool* available,
                               std::string* error_message) {
    if (interval_sec == nullptr || available == nullptr) {
        set_error(error_message, "RINEX interval outputs must not be null");
        return false;
    }
    *available = false;
    *interval_sec = std::numeric_limits<double>::quiet_NaN();
    std::ifstream input(path);
    if (!input) {
        set_error(error_message, "cannot open RINEX observation file for interval inspection: " + path);
        return false;
    }
    std::string line;
    while (std::getline(input, line)) {
        const std::string label = line.size() > 60 ? trim(line.substr(60)) : std::string{};
        if (label.find("INTERVAL") != std::string::npos) {
            std::istringstream value_stream(line.substr(0, std::min<std::size_t>(10, line.size())));
            value_stream.imbue(std::locale::classic());
            double value = 0.0;
            value_stream >> value;
            if (!value_stream || !std::isfinite(value) || value <= 0.0) {
                set_error(error_message, "RINEX observation INTERVAL record is invalid: " + path);
                return false;
            }
            if (*available && std::fabs(*interval_sec - value) > 1e-12) {
                set_error(error_message, "RINEX observation header has conflicting INTERVAL records: " + path);
                return false;
            }
            *available = true;
            *interval_sec = value;
        }
        if (label.find("END OF HEADER") != std::string::npos) {
            return true;
        }
    }
    set_error(error_message, "RINEX observation header has no END OF HEADER record: " + path);
    return false;
}

std::string json_escape(const std::string& value) {
    std::ostringstream output;
    for (const unsigned char character : value) {
        switch (character) {
            case '\"':
                output << "\\\"";
                break;
            case '\\':
                output << "\\\\";
                break;
            case '\b':
                output << "\\b";
                break;
            case '\f':
                output << "\\f";
                break;
            case '\n':
                output << "\\n";
                break;
            case '\r':
                output << "\\r";
                break;
            case '\t':
                output << "\\t";
                break;
            default:
                if (character < 0x20U) {
                    output << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
                           << static_cast<unsigned int>(character) << std::dec << std::setfill(' ');
                } else {
                    output << static_cast<char>(character);
                }
                break;
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

void write_sim_time_json(std::ostream& output, const SimTime& time) {
    output << "{\"gps_week\":" << time.gps_week << ",\"tow_ns\":" << time.tow_ns << '}';
}

void write_string_array(std::ostream& output, const std::vector<std::string>& values) {
    output << '[';
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            output << ',';
        }
        output << '\"' << json_escape(values[index]) << '\"';
    }
    output << ']';
}

} // namespace

bool build_cn0_model(const std::vector<Cn0InputSource>& sources, const Cn0AggregationConfig& config,
                     Cn0BuildResult* result, std::string* error_message) {
    if (result == nullptr || sources.empty()) {
        set_error(error_message, "CN0 model build requires at least one source and a result output");
        return false;
    }
    Cn0StatisticsAccumulator accumulator(config);
    if (!accumulator.valid(error_message)) {
        return false;
    }

    Cn0BuildResult build{};
    build.sources.reserve(sources.size());
    for (const Cn0InputSource& source : sources) {
        if (source.observation_path.empty() || source.navigation_path.empty()) {
            set_error(error_message, "CN0 source OBS/NAV paths must not be empty");
            return false;
        }
        Cn0SourceMetadata metadata{};
        if (!file_identity(source.observation_path, &metadata.observation_file, error_message) ||
            !file_identity(source.navigation_path, &metadata.navigation_file, error_message) ||
            !scan_observation_interval(source.observation_path, &metadata.observation_interval_sec,
                                       &metadata.observation_interval_available, error_message)) {
            return false;
        }
        const double interval = metadata.observation_interval_available ? metadata.observation_interval_sec
                                                                        : std::numeric_limits<double>::quiet_NaN();
        if (!accumulator.begin_source(interval, error_message)) {
            return false;
        }

        const bool streamed = stream_rinex_cn0_samples(
            source.observation_path, source.navigation_path,
            [&](const RinexCn0Sample& sample) {
                if (!metadata.sample_time_available) {
                    metadata.first_sample_time = sample.time;
                    metadata.sample_time_available = true;
                }
                metadata.last_sample_time = sample.time;
                return accumulator.add_sample(sample, error_message);
            },
            &metadata.provenance, &metadata.stream_summary, error_message);
        accumulator.end_source();
        if (!streamed) {
            return false;
        }
        build.sources.push_back(std::move(metadata));
    }

    build.aggregation_summary = accumulator.summary();
    build.bins = accumulator.finalize();
    *result = std::move(build);
    return true;
}

bool write_cn0_metadata_json(const std::string& output_path, const Cn0AggregationConfig& config,
                             const Cn0BuildResult& result, std::string* error_message) {
    std::ofstream output(output_path, std::ios::binary);
    if (!output) {
        set_error(error_message, "cannot open CN0 metadata JSON for writing: " + output_path);
        return false;
    }
    output.imbue(std::locale::classic());
    output << std::fixed << std::setprecision(6);
    output << "{\n";
    output << "  \"schema_version\":\"gnss-cn0-metadata-v1\",\n";
    output << "  \"builder_version\":\"gnss-cn0-builder-v1\",\n";
    output << "  \"model_schema_version\":\"gnss-cn0-model-v1\",\n";
    output << "  \"statistics\":{\"quantile\":\"R7_linear_h=(n-1)*p\",\"stddev\":\"population\","
              "\"mad\":\"median_absolute_deviation_from_p50\",\"delta_cn0\":\"absolute_consecutive_difference\","
              "\"cn0_grid_dbhz\":0.250000},\n";
    output << "  \"elevation_bins\":{\"min_deg\":" << config.elevation_min_deg
           << ",\"max_deg\":" << config.elevation_max_deg << ",\"width_deg\":" << config.elevation_bin_width_deg
           << ",\"rule\":\"[lower,upper), final bin includes max\"},\n";
    output << "  \"filtering\":{\"min_samples_per_bin\":" << config.min_samples_per_bin
           << ",\"min_temporal_pairs\":" << config.min_temporal_pairs
           << ",\"temporal_gap_tolerance_sec\":" << config.temporal_gap_tolerance_sec
           << ",\"temporal_rule\":\"same source,satellite,signal,elevation bin and declared INTERVAL\"},\n";
    const Cn0AggregationSummary& summary = result.aggregation_summary;
    output << "  \"summary\":{\"input_samples\":" << summary.input_samples
           << ",\"accepted_samples\":" << summary.accepted_samples
           << ",\"rejected_validity\":" << summary.rejected_validity
           << ",\"rejected_nonfinite\":" << summary.rejected_nonfinite
           << ",\"rejected_cn0_grid\":" << summary.rejected_cn0_grid
           << ",\"rejected_elevation_range\":" << summary.rejected_elevation_range
           << ",\"temporal_pairs\":" << summary.temporal_pairs
           << ",\"temporal_rejected_no_interval\":" << summary.temporal_rejected_no_interval
           << ",\"temporal_rejected_gap\":" << summary.temporal_rejected_gap
           << ",\"temporal_rejected_bin_change\":" << summary.temporal_rejected_bin_change
           << ",\"sources\":" << summary.sources << "},\n";
    output << "  \"sources\":[\n";
    for (std::size_t index = 0; index < result.sources.size(); ++index) {
        const Cn0SourceMetadata& source = result.sources[index];
        output << "    {\"observation\":{\"file_name\":\"" << json_escape(source.observation_file.file_name)
               << "\",\"size_bytes\":" << source.observation_file.size_bytes << ",\"fnv1a64\":\""
               << hex64(source.observation_file.fnv1a64) << "\"},\"navigation\":{\"file_name\":\""
               << json_escape(source.navigation_file.file_name)
               << "\",\"size_bytes\":" << source.navigation_file.size_bytes << ",\"fnv1a64\":\""
               << hex64(source.navigation_file.fnv1a64) << "\"},\"rinex_version\":" << source.provenance.rinex_version
               << ",\"station_name\":\"" << json_escape(source.provenance.station_name) << "\",\"marker_number\":\""
               << json_escape(source.provenance.marker_number) << "\",\"receiver_type\":\""
               << json_escape(source.provenance.receiver_type) << "\",\"antenna_type\":\""
               << json_escape(source.provenance.antenna_type) << "\",\"time_system\":\""
               << json_escape(source.provenance.observation_time_system) << "\",\"signal_strength_unit\":\""
               << json_escape(source.provenance.signal_strength_unit) << "\",\"signal_strength_unit_status\":\""
               << signal_strength_unit_status_name(source.provenance.signal_strength_unit_status)
               << "\",\"station_ecef_m\":[" << source.provenance.station_ecef_m[0] << ','
               << source.provenance.station_ecef_m[1] << ',' << source.provenance.station_ecef_m[2]
               << "],\"observation_interval_sec\":";
        if (source.observation_interval_available) {
            output << source.observation_interval_sec;
        } else {
            output << "null";
        }
        output << ",\"first_sample_time\":";
        if (source.sample_time_available) {
            write_sim_time_json(output, source.first_sample_time);
        } else {
            output << "null";
        }
        output << ",\"last_sample_time\":";
        if (source.sample_time_available) {
            write_sim_time_json(output, source.last_sample_time);
        } else {
            output << "null";
        }
        output << ",\"stream_summary\":{\"epochs\":" << source.stream_summary.epochs
               << ",\"observation_records\":" << source.stream_summary.observation_records
               << ",\"emitted_samples\":" << source.stream_summary.emitted_samples
               << ",\"valid_dbhz_samples\":" << source.stream_summary.valid_dbhz_samples
               << ",\"ambiguous_unit_samples\":" << source.stream_summary.ambiguous_unit_samples
               << ",\"missing_signal_strength\":" << source.stream_summary.missing_signal_strength
               << ",\"unsupported_signal_observables\":" << source.stream_summary.unsupported_signal_observables
               << ",\"unmapped_snr_slots\":" << source.stream_summary.unmapped_snr_slots
               << ",\"geometry_failures\":" << source.stream_summary.geometry_failures
               << ",\"out_of_order_epochs\":" << source.stream_summary.out_of_order_epochs
               << ",\"peak_epoch_observations\":" << source.stream_summary.peak_epoch_observations
               << ",\"unsupported_observables\":";
        write_string_array(output, source.stream_summary.unsupported_observables);
        output << "}}" << (index + 1 == result.sources.size() ? "\n" : ",\n");
    }
    output << "  ]\n";
    output << "}\n";
    if (!output) {
        set_error(error_message, "failed while writing CN0 metadata JSON: " + output_path);
        return false;
    }
    return true;
}

} // namespace gnss_sim::cn0_builder
