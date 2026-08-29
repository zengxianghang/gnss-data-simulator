#include "transient_validator.h"

#include "gnss_sim/sim_time.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gnss_sim {
namespace {

struct ObservationKey {
    int gps_week;
    std::int64_t tow_ns;
    int satellite_number;
    int signal_id;

    bool operator==(const ObservationKey& other) const {
        return gps_week == other.gps_week && tow_ns == other.tow_ns && satellite_number == other.satellite_number &&
               signal_id == other.signal_id;
    }
};

struct ObservationKeyHash {
    std::size_t operator()(const ObservationKey& key) const {
        std::size_t value = static_cast<std::size_t>(key.gps_week);
        value ^= static_cast<std::size_t>(key.tow_ns) + 0x9e3779b9U + (value << 6U) + (value >> 2U);
        value ^= static_cast<std::size_t>(key.satellite_number) + 0x9e3779b9U + (value << 6U) + (value >> 2U);
        value ^= static_cast<std::size_t>(key.signal_id) + 0x9e3779b9U + (value << 6U) + (value >> 2U);
        return value;
    }
};

struct SignalKey {
    int satellite_number;
    int signal_id;

    bool operator==(const SignalKey& other) const {
        return satellite_number == other.satellite_number && signal_id == other.signal_id;
    }
};

struct SignalKeyHash {
    std::size_t operator()(const SignalKey& key) const {
        return (static_cast<std::size_t>(key.satellite_number) << 8U) ^ static_cast<std::size_t>(key.signal_id);
    }
};

struct TruthObservation {
    ObservationKey key;
    double wavelength_m;
    double pseudorange_m;
    double doppler_hz;
    double adr_cycles;
    double cn0_dbhz;
    bool pseudorange_valid;
    bool doppler_valid;
    bool adr_valid;
    std::int64_t ambiguity_cycles;
    std::int64_t ambiguity_epoch_tow_ns;
};

struct EventRecord {
    std::int64_t absolute_ns;
    std::string type;
};

struct ReaCycle {
    std::int64_t off_ns;
    std::int64_t on_ns;
    std::int64_t end_ns;
};

struct ReaSerializedTiming {
    std::int64_t first_psr_ns;
    std::int64_t first_doppler_ns;
    std::int64_t first_adr_ns;
    std::int64_t last_observation_before_off_ns;
    std::int64_t first_observation_after_on_ns;
};

struct MetricAccumulator {
    std::vector<double> values;
};

struct WindowAccumulator {
    MetricAccumulator pseudorange_m;
    MetricAccumulator doppler_mps;
    MetricAccumulator adr_m;
    MetricAccumulator cn0_dbhz;
};

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool parse_csv_line(const std::string& line, std::vector<std::string>* fields) {
    if (fields == nullptr) {
        return false;
    }
    fields->clear();
    std::string field;
    bool quoted = false;
    for (std::size_t index = 0; index < line.size(); ++index) {
        const char character = line[index];
        if (quoted) {
            if (character == '"') {
                if (index + 1U < line.size() && line[index + 1U] == '"') {
                    field.push_back('"');
                    ++index;
                } else {
                    quoted = false;
                }
            } else {
                field.push_back(character);
            }
            continue;
        }
        if (character == '"' && field.empty()) {
            quoted = true;
        } else if (character == ',') {
            fields->push_back(field);
            field.clear();
        } else if (character != '\r') {
            field.push_back(character);
        }
    }
    if (quoted) {
        return false;
    }
    fields->push_back(field);
    return true;
}

std::size_t column_index(const std::vector<std::string>& header, const char* name) {
    const auto iterator = std::find(header.begin(), header.end(), name);
    return iterator == header.end() ? header.size() : static_cast<std::size_t>(iterator - header.begin());
}

bool parse_int(const std::string& text, int* value) {
    if (value == nullptr || text.empty()) {
        return false;
    }
    std::istringstream stream(text);
    stream.imbue(std::locale::classic());
    int result = 0;
    stream >> result;
    if (!stream || stream.peek() != std::char_traits<char>::eof()) {
        return false;
    }
    *value = result;
    return true;
}

bool parse_int64(const std::string& text, std::int64_t* value) {
    if (value == nullptr || text.empty()) {
        return false;
    }
    std::istringstream stream(text);
    stream.imbue(std::locale::classic());
    std::int64_t result = 0;
    stream >> result;
    if (!stream || stream.peek() != std::char_traits<char>::eof()) {
        return false;
    }
    *value = result;
    return true;
}

bool parse_double(const std::string& text, double* value) {
    if (value == nullptr || text.empty()) {
        return false;
    }
    std::istringstream stream(text);
    stream.imbue(std::locale::classic());
    double result = 0.0;
    stream >> result;
    if (!stream || stream.peek() != std::char_traits<char>::eof() || !std::isfinite(result)) {
        return false;
    }
    *value = result;
    return true;
}

bool parse_bool01(const std::string& text, bool* value) {
    if (value == nullptr || (text != "0" && text != "1")) {
        return false;
    }
    *value = text == "1";
    return true;
}

bool absolute_time_ns(int gps_week, std::int64_t tow_ns, std::int64_t* absolute_ns) {
    if (absolute_ns == nullptr || gps_week < 0 || tow_ns < 0 || tow_ns >= GPS_WEEK_NANOSECONDS) {
        return false;
    }
    const long double value = static_cast<long double>(gps_week) * static_cast<long double>(GPS_WEEK_NANOSECONDS) +
                              static_cast<long double>(tow_ns);
    if (value > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
        return false;
    }
    *absolute_ns = static_cast<std::int64_t>(value);
    return true;
}

bool load_truth_observations(const char* path, std::vector<TruthObservation>* records,
                             std::unordered_map<ObservationKey, std::size_t, ObservationKeyHash>* index,
                             std::string* error_message) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        set_error(error_message, std::string("cannot open observation truth: ") + path);
        return false;
    }
    std::string line;
    if (!std::getline(input, line)) {
        set_error(error_message, "observation truth is empty");
        return false;
    }
    std::vector<std::string> header;
    if (!parse_csv_line(line, &header)) {
        set_error(error_message, "observation truth header is malformed");
        return false;
    }
    const std::size_t gps_week_column = column_index(header, "gps_week");
    const std::size_t tow_column = column_index(header, "tow_ns");
    const std::size_t satellite_column = column_index(header, "satellite_number");
    const std::size_t signal_column = column_index(header, "signal_id");
    const std::size_t wavelength_column = column_index(header, "wavelength_m");
    const std::size_t psr_valid_column = column_index(header, "pseudorange_valid");
    const std::size_t doppler_valid_column = column_index(header, "doppler_valid");
    const std::size_t adr_valid_column = column_index(header, "adr_valid");
    const std::size_t ambiguity_column = column_index(header, "ambiguity_cycles");
    const std::size_t ambiguity_epoch_column = column_index(header, "ambiguity_epoch_tow_ns");
    const std::size_t psr_column = column_index(header, "pseudorange_m");
    const std::size_t doppler_column = column_index(header, "doppler_hz");
    const std::size_t adr_column = column_index(header, "adr_cycles");
    const std::size_t cn0_column = column_index(header, "cn0_dbhz");
    const std::size_t required[] = {
        gps_week_column,  tow_column,           satellite_column, signal_column,    wavelength_column,
        psr_valid_column, doppler_valid_column, adr_valid_column, ambiguity_column, ambiguity_epoch_column,
        psr_column,       doppler_column,       adr_column,       cn0_column};
    for (std::size_t value : required) {
        if (value == header.size()) {
            set_error(error_message, "observation truth is missing a required column");
            return false;
        }
    }

    records->clear();
    index->clear();
    std::uint64_t line_number = 1;
    while (std::getline(input, line)) {
        ++line_number;
        std::vector<std::string> fields;
        if (!parse_csv_line(line, &fields) || fields.size() != header.size()) {
            set_error(error_message, "observation truth row is malformed at line " + std::to_string(line_number));
            return false;
        }
        TruthObservation record{};
        if (!parse_int(fields[gps_week_column], &record.key.gps_week) ||
            !parse_int64(fields[tow_column], &record.key.tow_ns) ||
            !parse_int(fields[satellite_column], &record.key.satellite_number) ||
            !parse_int(fields[signal_column], &record.key.signal_id) ||
            !parse_double(fields[wavelength_column], &record.wavelength_m) ||
            !parse_bool01(fields[psr_valid_column], &record.pseudorange_valid) ||
            !parse_bool01(fields[doppler_valid_column], &record.doppler_valid) ||
            !parse_bool01(fields[adr_valid_column], &record.adr_valid) ||
            !parse_int64(fields[ambiguity_column], &record.ambiguity_cycles) ||
            !parse_int64(fields[ambiguity_epoch_column], &record.ambiguity_epoch_tow_ns) ||
            !parse_double(fields[psr_column], &record.pseudorange_m) ||
            !parse_double(fields[doppler_column], &record.doppler_hz) ||
            !parse_double(fields[adr_column], &record.adr_cycles) ||
            !parse_double(fields[cn0_column], &record.cn0_dbhz) || record.wavelength_m <= 0.0) {
            set_error(error_message,
                      "observation truth numeric field is malformed at line " + std::to_string(line_number));
            return false;
        }
        const auto inserted = index->emplace(record.key, records->size());
        if (!inserted.second) {
            set_error(error_message, "observation truth contains a duplicate epoch/satellite/signal key");
            return false;
        }
        records->push_back(record);
    }
    if (records->empty()) {
        set_error(error_message, "observation truth contains no observations");
        return false;
    }
    return true;
}

bool load_events(const char* path, std::vector<EventRecord>* events, std::string* error_message) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        set_error(error_message, std::string("cannot open event truth: ") + path);
        return false;
    }
    std::string line;
    if (!std::getline(input, line)) {
        set_error(error_message, "event truth is empty");
        return false;
    }
    std::vector<std::string> header;
    if (!parse_csv_line(line, &header)) {
        set_error(error_message, "event truth header is malformed");
        return false;
    }
    const std::size_t gps_week_column = column_index(header, "gps_week");
    const std::size_t tow_column = column_index(header, "tow_ns");
    const std::size_t type_column = column_index(header, "event_type");
    if (gps_week_column == header.size() || tow_column == header.size() || type_column == header.size()) {
        set_error(error_message, "event truth is missing required columns");
        return false;
    }

    events->clear();
    while (std::getline(input, line)) {
        std::vector<std::string> fields;
        int gps_week = 0;
        std::int64_t tow_ns = 0;
        std::int64_t absolute_ns = 0;
        if (!parse_csv_line(line, &fields) || fields.size() != header.size() ||
            !parse_int(fields[gps_week_column], &gps_week) || !parse_int64(fields[tow_column], &tow_ns) ||
            !absolute_time_ns(gps_week, tow_ns, &absolute_ns)) {
            set_error(error_message, "event truth row is malformed");
            return false;
        }
        events->push_back({absolute_ns, fields[type_column]});
    }
    std::sort(events->begin(), events->end(),
              [](const EventRecord& lhs, const EventRecord& rhs) { return lhs.absolute_ns < rhs.absolute_ns; });
    return true;
}

std::vector<ReaCycle> build_rea_cycles(const std::vector<EventRecord>& events) {
    std::vector<ReaCycle> cycles;
    bool waiting_for_on = false;
    std::int64_t off_ns = 0;
    for (const EventRecord& event : events) {
        if (event.type == "SIGNAL_OFF") {
            if (!cycles.empty() && cycles.back().end_ns == std::numeric_limits<std::int64_t>::max()) {
                cycles.back().end_ns = event.absolute_ns;
            }
            waiting_for_on = true;
            off_ns = event.absolute_ns;
        } else if (event.type == "SIGNAL_ON" && waiting_for_on) {
            cycles.push_back({off_ns, event.absolute_ns, std::numeric_limits<std::int64_t>::max()});
            waiting_for_on = false;
        }
    }
    return cycles;
}

bool in_signal_off_interval(std::int64_t time_ns, const std::vector<ReaCycle>& cycles) {
    for (const ReaCycle& cycle : cycles) {
        if (time_ns >= cycle.off_ns && time_ns < cycle.on_ns) {
            return true;
        }
    }
    return false;
}

const ReaCycle* active_reacquisition_cycle(std::int64_t time_ns, const std::vector<ReaCycle>& cycles) {
    for (const ReaCycle& cycle : cycles) {
        if (time_ns >= cycle.on_ns && time_ns < cycle.end_ns) {
            return &cycle;
        }
    }
    return nullptr;
}

bool in_fade_window(std::int64_t time_ns, const std::vector<EventRecord>& events, double fade_duration_sec) {
    if (!(fade_duration_sec > 0.0) || !std::isfinite(fade_duration_sec)) {
        return false;
    }
    const long double fade_ns =
        static_cast<long double>(fade_duration_sec) * static_cast<long double>(NANOSECONDS_PER_SECOND);
    for (const EventRecord& event : events) {
        if (event.type != "SIGNAL_OFF" || event.absolute_ns < time_ns) {
            continue;
        }
        return static_cast<long double>(event.absolute_ns - time_ns) <= fade_ns;
    }
    return false;
}

void add_metric(MetricAccumulator* accumulator, double value) {
    if (std::isfinite(value)) {
        accumulator->values.push_back(value);
    }
}

void add_observation(WindowAccumulator* accumulator, const ParsedRangeObservation& parsed,
                     const TruthObservation& truth) {
    if (truth.pseudorange_valid && parsed.pseudorange_valid) {
        add_metric(&accumulator->pseudorange_m, parsed.pseudorange_m - truth.pseudorange_m);
    }
    if (truth.doppler_valid) {
        add_metric(&accumulator->doppler_mps, (parsed.doppler_hz - truth.doppler_hz) * truth.wavelength_m);
    }
    if (truth.adr_valid && parsed.adr_valid) {
        add_metric(&accumulator->adr_m, (parsed.adr_cycles - truth.adr_cycles) * truth.wavelength_m);
    }
    add_metric(&accumulator->cn0_dbhz, parsed.cn0_dbhz - truth.cn0_dbhz);
}

double quantile_sorted(const std::vector<double>& sorted, double probability) {
    if (sorted.empty()) {
        return 0.0;
    }
    const double position = probability * static_cast<double>(sorted.size() - 1U);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
    const double fraction = position - static_cast<double>(lower);
    return sorted[lower] + (sorted[upper] - sorted[lower]) * fraction;
}

ErrorMetricStatistics finalize_metric(const MetricAccumulator& accumulator) {
    ErrorMetricStatistics result{};
    result.sample_count = static_cast<std::uint64_t>(accumulator.values.size());
    if (accumulator.values.empty()) {
        return result;
    }
    long double sum = 0.0L;
    long double square_sum = 0.0L;
    std::vector<double> absolute;
    absolute.reserve(accumulator.values.size());
    for (double value : accumulator.values) {
        sum += value;
        square_sum += static_cast<long double>(value) * static_cast<long double>(value);
        absolute.push_back(std::fabs(value));
    }
    const long double count = static_cast<long double>(accumulator.values.size());
    result.mean = static_cast<double>(sum / count);
    result.rms = static_cast<double>(std::sqrt(square_sum / count));
    long double variance_sum = 0.0L;
    for (double value : accumulator.values) {
        const long double difference = static_cast<long double>(value) - static_cast<long double>(result.mean);
        variance_sum += difference * difference;
    }
    result.standard_deviation = static_cast<double>(std::sqrt(variance_sum / count));
    std::sort(absolute.begin(), absolute.end());
    result.p50_absolute = quantile_sorted(absolute, 0.50);
    result.p95_absolute = quantile_sorted(absolute, 0.95);
    result.max_absolute = absolute.back();
    return result;
}

ObservationWindowStatistics finalize_window(const WindowAccumulator& accumulator) {
    ObservationWindowStatistics result{};
    result.pseudorange_m = finalize_metric(accumulator.pseudorange_m);
    result.doppler_mps = finalize_metric(accumulator.doppler_mps);
    result.adr_m = finalize_metric(accumulator.adr_m);
    result.cn0_dbhz = finalize_metric(accumulator.cn0_dbhz);
    return result;
}

void update_rea_timing(const std::vector<TruthObservation>& truth, const std::vector<ReaCycle>& cycles,
                       const std::vector<ReaSerializedTiming>& serialized_timing, ReaTransientStatistics* statistics) {
    statistics->reacquisition_cycles = static_cast<std::uint64_t>(cycles.size());
    for (std::size_t cycle_index = 0; cycle_index < cycles.size(); ++cycle_index) {
        const ReaCycle& cycle = cycles[cycle_index];
        const ReaSerializedTiming& timing = serialized_timing[cycle_index];
        const auto update_delay = [cycle](std::int64_t first_ns, double* maximum) {
            if (first_ns == std::numeric_limits<std::int64_t>::max()) {
                return;
            }
            const double delay =
                static_cast<double>(first_ns - cycle.on_ns) / static_cast<double>(NANOSECONDS_PER_SECOND);
            *maximum = std::max(*maximum, delay);
        };
        update_delay(timing.first_psr_ns, &statistics->max_first_psr_delay_sec);
        update_delay(timing.first_doppler_ns, &statistics->max_first_doppler_delay_sec);
        update_delay(timing.first_adr_ns, &statistics->max_first_adr_delay_sec);
        if (timing.last_observation_before_off_ns != std::numeric_limits<std::int64_t>::min()) {
            const double gap = static_cast<double>(cycle.off_ns - timing.last_observation_before_off_ns) /
                               static_cast<double>(NANOSECONDS_PER_SECOND);
            statistics->max_last_observation_to_signal_off_sec =
                std::max(statistics->max_last_observation_to_signal_off_sec, gap);
        }
        if (timing.first_observation_after_on_ns != std::numeric_limits<std::int64_t>::max()) {
            const double delay = static_cast<double>(timing.first_observation_after_on_ns - cycle.on_ns) /
                                 static_cast<double>(NANOSECONDS_PER_SECOND);
            statistics->max_first_observation_after_signal_on_sec =
                std::max(statistics->max_first_observation_after_signal_on_sec, delay);
        }

        std::unordered_map<SignalKey, const TruthObservation*, SignalKeyHash> before;
        std::unordered_map<SignalKey, const TruthObservation*, SignalKeyHash> after;
        for (const TruthObservation& record : truth) {
            if (!record.adr_valid) {
                continue;
            }
            std::int64_t time_ns = 0;
            if (!absolute_time_ns(record.key.gps_week, record.key.tow_ns, &time_ns)) {
                continue;
            }
            const SignalKey key{record.key.satellite_number, record.key.signal_id};
            if (time_ns < cycle.off_ns) {
                const auto iterator = before.find(key);
                if (iterator == before.end() || iterator->second->key.gps_week < record.key.gps_week ||
                    (iterator->second->key.gps_week == record.key.gps_week &&
                     iterator->second->key.tow_ns < record.key.tow_ns)) {
                    before[key] = &record;
                }
            } else if (time_ns >= cycle.on_ns && time_ns < cycle.end_ns && after.find(key) == after.end()) {
                after[key] = &record;
            }
        }
        for (const auto& entry : before) {
            const auto iterator = after.find(entry.first);
            if (iterator == after.end()) {
                continue;
            }
            ++statistics->ambiguity_pairs_checked;
            const TruthObservation* lhs = entry.second;
            const TruthObservation* rhs = iterator->second;
            if (lhs->ambiguity_epoch_tow_ns != rhs->ambiguity_epoch_tow_ns &&
                lhs->ambiguity_cycles != rhs->ambiguity_cycles) {
                ++statistics->ambiguity_pairs_changed;
            }
        }
    }
}

bool is_rea_label(const char* label) {
    return label != nullptr && std::string(label).rfind("REA", 0) == 0;
}

std::string json_escape(const char* text) {
    std::ostringstream output;
    for (const unsigned char character : std::string(text != nullptr ? text : "")) {
        switch (character) {
            case '\\':
                output << "\\\\";
                break;
            case '"':
                output << "\\\"";
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
                output << static_cast<char>(character);
                break;
        }
    }
    return output.str();
}

void write_metric_json(std::ostream& output, const ErrorMetricStatistics& metric, int indent) {
    const std::string pad(static_cast<std::size_t>(indent), ' ');
    output << "{\n"
           << pad << "  \"sample_count\": " << metric.sample_count << ",\n"
           << pad << "  \"mean\": " << metric.mean << ",\n"
           << pad << "  \"rms\": " << metric.rms << ",\n"
           << pad << "  \"standard_deviation\": " << metric.standard_deviation << ",\n"
           << pad << "  \"p50_absolute\": " << metric.p50_absolute << ",\n"
           << pad << "  \"p95_absolute\": " << metric.p95_absolute << ",\n"
           << pad << "  \"max_absolute\": " << metric.max_absolute << '\n'
           << pad << '}';
}

void write_window_json(std::ostream& output, const ObservationWindowStatistics& window, int indent) {
    const std::string pad(static_cast<std::size_t>(indent), ' ');
    output << "{\n" << pad << "  \"pseudorange_m\": ";
    write_metric_json(output, window.pseudorange_m, indent + 2);
    output << ",\n" << pad << "  \"doppler_mps\": ";
    write_metric_json(output, window.doppler_mps, indent + 2);
    output << ",\n" << pad << "  \"adr_m\": ";
    write_metric_json(output, window.adr_m, indent + 2);
    output << ",\n" << pad << "  \"cn0_dbhz\": ";
    write_metric_json(output, window.cn0_dbhz, indent + 2);
    output << '\n' << pad << '}';
}

} // namespace

bool validate_transient_observations_files(const char* log_path, const char* observation_truth_path,
                                           const char* event_truth_path, const char* rinex_nav_path,
                                           const TransientValidationOptions& options,
                                           TransientValidationSummary* summary, std::string* error_message) {
    if (log_path == nullptr || observation_truth_path == nullptr || event_truth_path == nullptr ||
        rinex_nav_path == nullptr || options.scenario_label == nullptr || summary == nullptr ||
        !std::isfinite(options.fade_duration_sec) || options.fade_duration_sec < 0.0 ||
        !std::isfinite(options.solution_elevation_mask_deg) || options.solution_elevation_mask_deg < 0.0 ||
        options.solution_elevation_mask_deg > 90.0) {
        set_error(error_message, "transient validator request has invalid arguments");
        return false;
    }

    std::vector<TruthObservation> truth;
    std::unordered_map<ObservationKey, std::size_t, ObservationKeyHash> truth_index;
    std::vector<EventRecord> events;
    if (!load_truth_observations(observation_truth_path, &truth, &truth_index, error_message) ||
        !load_events(event_truth_path, &events, error_message)) {
        return false;
    }
    const std::vector<ReaCycle> rea_cycles = build_rea_cycles(events);
    const bool rea = is_rea_label(options.scenario_label);
    std::vector<ReaSerializedTiming> rea_serialized_timing(
        rea_cycles.size(), {std::numeric_limits<std::int64_t>::max(), std::numeric_limits<std::int64_t>::max(),
                            std::numeric_limits<std::int64_t>::max(), std::numeric_limits<std::int64_t>::min(),
                            std::numeric_limits<std::int64_t>::max()});

    std::ifstream input(log_path, std::ios::binary);
    if (!input) {
        set_error(error_message, std::string("cannot open serialized receiver log: ") + log_path);
        return false;
    }

    WindowAccumulator early;
    WindowAccumulator recovery;
    WindowAccumulator settled;
    WindowAccumulator fade;
    WindowAccumulator reacquisition_early;
    WindowAccumulator reacquisition_recovery;
    WindowAccumulator reacquisition_settled;
    TransientValidationSummary result{};
    result.first_valid = {-1.0, -1.0, -1.0, -1.0};
    std::int64_t first_rangea_ns = std::numeric_limits<std::int64_t>::max();
    std::int64_t first_psr_ns = std::numeric_limits<std::int64_t>::max();
    std::int64_t first_doppler_ns = std::numeric_limits<std::int64_t>::max();
    std::int64_t first_adr_ns = std::numeric_limits<std::int64_t>::max();
    std::int64_t first_cn0_ns = std::numeric_limits<std::int64_t>::max();
    std::string line;
    std::uint64_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.rfind("#RANGEA,", 0) != 0) {
            continue;
        }
        ParsedRangeEpoch epoch{};
        std::string parse_error;
        if (!parse_rangea_line_independent(line, &epoch, &parse_error)) {
            set_error(error_message, "RANGEA line " + std::to_string(line_number) + ": " + parse_error);
            return false;
        }
        ++result.range_epochs;
        result.parsed_observations += static_cast<std::uint64_t>(epoch.observations.size());

        SimTime time{};
        std::int64_t absolute_ns = 0;
        if (!sim_time_from_week_sow(epoch.gps_week, epoch.sow_sec, &time) ||
            !absolute_time_ns(time.gps_week, time.tow_ns, &absolute_ns)) {
            set_error(error_message, "serialized RANGEA epoch time cannot be normalized");
            return false;
        }
        first_rangea_ns = std::min(first_rangea_ns, absolute_ns);
        if (rea && !epoch.observations.empty()) {
            for (std::size_t cycle_index = 0; cycle_index < rea_cycles.size(); ++cycle_index) {
                const ReaCycle& cycle = rea_cycles[cycle_index];
                ReaSerializedTiming& timing = rea_serialized_timing[cycle_index];
                if (absolute_ns < cycle.off_ns) {
                    timing.last_observation_before_off_ns =
                        std::max(timing.last_observation_before_off_ns, absolute_ns);
                } else if (absolute_ns >= cycle.on_ns && absolute_ns < cycle.end_ns) {
                    timing.first_observation_after_on_ns = std::min(timing.first_observation_after_on_ns, absolute_ns);
                }
            }
        }
        if (rea && in_signal_off_interval(absolute_ns, rea_cycles)) {
            ++result.rea.signal_off_range_epochs;
            if (!epoch.observations.empty()) {
                ++result.rea.signal_off_nonzero_epochs;
            }
        }
        const ReaCycle* reacquisition_cycle = rea ? active_reacquisition_cycle(absolute_ns, rea_cycles) : nullptr;
        const bool fade_active = rea && in_fade_window(absolute_ns, events, options.fade_duration_sec);

        for (const ParsedRangeObservation& parsed : epoch.observations) {
            const ObservationKey key{time.gps_week, time.tow_ns, parsed.satellite_number,
                                     static_cast<int>(parsed.signal_id)};
            const auto iterator = truth_index.find(key);
            if (iterator == truth_index.end()) {
                ++result.unmatched_observations;
                continue;
            }
            ++result.matched_observations;
            const TruthObservation& source = truth[iterator->second];
            if (parsed.pseudorange_valid) {
                first_psr_ns = std::min(first_psr_ns, absolute_ns);
            }
            // RANGEA has no independent Doppler-valid bit; require a matched serialized record and
            // the zero-noise truth validity for that exact field.
            if (source.doppler_valid) {
                first_doppler_ns = std::min(first_doppler_ns, absolute_ns);
            }
            if (parsed.adr_valid) {
                first_adr_ns = std::min(first_adr_ns, absolute_ns);
            }
            first_cn0_ns = std::min(first_cn0_ns, absolute_ns);
            if (reacquisition_cycle != nullptr) {
                const std::size_t cycle_index = static_cast<std::size_t>(reacquisition_cycle - rea_cycles.data());
                ReaSerializedTiming& timing = rea_serialized_timing[cycle_index];
                if (parsed.pseudorange_valid) {
                    timing.first_psr_ns = std::min(timing.first_psr_ns, absolute_ns);
                }
                // RANGEA currently carries no independent Doppler-valid bit. A matched serialized
                // record is therefore gated by the zero-noise truth validity for that exact field.
                if (source.doppler_valid) {
                    timing.first_doppler_ns = std::min(timing.first_doppler_ns, absolute_ns);
                }
                if (parsed.adr_valid) {
                    timing.first_adr_ns = std::min(timing.first_adr_ns, absolute_ns);
                }
            }
            WindowAccumulator* lock_window = &settled;
            if (parsed.lock_time_sec < 1.0) {
                lock_window = &early;
            } else if (parsed.lock_time_sec < 3.0) {
                lock_window = &recovery;
            }
            add_observation(lock_window, parsed, source);
            if (fade_active) {
                add_observation(&fade, parsed, source);
            }
            if (reacquisition_cycle != nullptr) {
                WindowAccumulator* reacquisition_window = &reacquisition_settled;
                const double elapsed_sec = static_cast<double>(absolute_ns - reacquisition_cycle->on_ns) /
                                           static_cast<double>(NANOSECONDS_PER_SECOND);
                if (elapsed_sec < 1.0) {
                    reacquisition_window = &reacquisition_early;
                } else if (elapsed_sec < 3.0) {
                    reacquisition_window = &reacquisition_recovery;
                }
                add_observation(reacquisition_window, parsed, source);
            }
        }
    }
    if (result.range_epochs == 0U || result.matched_observations == 0U) {
        set_error(error_message, "transient validator found no matched serialized RANGEA observations");
        return false;
    }

    const auto delay_from_first_rangea = [first_rangea_ns](std::int64_t sample_ns) {
        if (first_rangea_ns == std::numeric_limits<std::int64_t>::max() ||
            sample_ns == std::numeric_limits<std::int64_t>::max()) {
            return -1.0;
        }
        return static_cast<double>(sample_ns - first_rangea_ns) / static_cast<double>(NANOSECONDS_PER_SECOND);
    };
    result.first_valid.pseudorange_delay_sec = delay_from_first_rangea(first_psr_ns);
    result.first_valid.doppler_delay_sec = delay_from_first_rangea(first_doppler_ns);
    result.first_valid.adr_delay_sec = delay_from_first_rangea(first_adr_ns);
    result.first_valid.cn0_delay_sec = delay_from_first_rangea(first_cn0_ns);

    result.early = finalize_window(early);
    result.recovery = finalize_window(recovery);
    result.settled = finalize_window(settled);
    result.fade = finalize_window(fade);
    result.reacquisition_early = finalize_window(reacquisition_early);
    result.reacquisition_recovery = finalize_window(reacquisition_recovery);
    result.reacquisition_settled = finalize_window(reacquisition_settled);
    if (rea) {
        update_rea_timing(truth, rea_cycles, rea_serialized_timing, &result.rea);
    }

    if (!validate_rangea_roundtrip_file(
            log_path, rinex_nav_path, options.truth_latitude_deg, options.truth_longitude_deg, options.truth_height_m,
            options.solution_elevation_mask_deg, options.broadcast_atmosphere, &result.positioning, error_message)) {
        return false;
    }
    *summary = result;
    return true;
}

bool write_transient_validation_json(const char* output_path, const TransientValidationOptions& options,
                                     const TransientValidationSummary& summary, std::string* error_message) {
    if (output_path == nullptr || output_path[0] == '\0' || options.scenario_label == nullptr) {
        set_error(error_message, "transient validation JSON request has invalid arguments");
        return false;
    }
    std::ofstream output(output_path, std::ios::binary | std::ios::trunc);
    if (!output) {
        set_error(error_message, std::string("cannot open transient validation JSON: ") + output_path);
        return false;
    }
    output.imbue(std::locale::classic());
    output << std::setprecision(17);
    output << "{\n"
           << "  \"schema_version\": 1,\n"
           << "  \"scenario\": \"" << json_escape(options.scenario_label) << "\",\n"
           << "  \"fade_duration_sec\": " << options.fade_duration_sec << ",\n"
           << "  \"range_epochs\": " << summary.range_epochs << ",\n"
           << "  \"parsed_observations\": " << summary.parsed_observations << ",\n"
           << "  \"matched_observations\": " << summary.matched_observations << ",\n"
           << "  \"unmatched_observations\": " << summary.unmatched_observations << ",\n"
           << "  \"first_valid\": {\n"
           << "    \"pseudorange_delay_sec\": " << summary.first_valid.pseudorange_delay_sec << ",\n"
           << "    \"doppler_delay_sec\": " << summary.first_valid.doppler_delay_sec << ",\n"
           << "    \"adr_delay_sec\": " << summary.first_valid.adr_delay_sec << ",\n"
           << "    \"cn0_delay_sec\": " << summary.first_valid.cn0_delay_sec << "\n"
           << "  },\n"
           << "  \"windows\": {\n"
           << "    \"early_0_1s\": ";
    write_window_json(output, summary.early, 4);
    output << ",\n    \"recovery_1_3s\": ";
    write_window_json(output, summary.recovery, 4);
    output << ",\n    \"settled_ge_3s\": ";
    write_window_json(output, summary.settled, 4);
    output << ",\n    \"rea_fade\": ";
    write_window_json(output, summary.fade, 4);
    output << ",\n    \"rea_reacquisition_early_0_1s\": ";
    write_window_json(output, summary.reacquisition_early, 4);
    output << ",\n    \"rea_reacquisition_recovery_1_3s\": ";
    write_window_json(output, summary.reacquisition_recovery, 4);
    output << ",\n    \"rea_reacquisition_settled_ge_3s\": ";
    write_window_json(output, summary.reacquisition_settled, 4);
    output << "\n  },\n"
           << "  \"rea\": {\n"
           << "    \"signal_off_range_epochs\": " << summary.rea.signal_off_range_epochs << ",\n"
           << "    \"signal_off_nonzero_epochs\": " << summary.rea.signal_off_nonzero_epochs << ",\n"
           << "    \"reacquisition_cycles\": " << summary.rea.reacquisition_cycles << ",\n"
           << "    \"max_first_psr_delay_sec\": " << summary.rea.max_first_psr_delay_sec << ",\n"
           << "    \"max_first_doppler_delay_sec\": " << summary.rea.max_first_doppler_delay_sec << ",\n"
           << "    \"max_first_adr_delay_sec\": " << summary.rea.max_first_adr_delay_sec << ",\n"
           << "    \"max_last_observation_to_signal_off_sec\": " << summary.rea.max_last_observation_to_signal_off_sec
           << ",\n"
           << "    \"max_first_observation_after_signal_on_sec\": "
           << summary.rea.max_first_observation_after_signal_on_sec << ",\n"
           << "    \"ambiguity_pairs_checked\": " << summary.rea.ambiguity_pairs_checked << ",\n"
           << "    \"ambiguity_pairs_changed\": " << summary.rea.ambiguity_pairs_changed << "\n"
           << "  },\n"
           << "  \"positioning\": {\n"
           << "    \"solution_elevation_mask_deg\": " << options.solution_elevation_mask_deg << ",\n"
           << "    \"valid_position_epochs\": " << summary.positioning.valid_position_epochs << ",\n"
           << "    \"selected_position_observations\": " << summary.positioning.selected_position_observations << ",\n"
           << "    \"max_position_error_m\": " << summary.positioning.max_position_error_m << ",\n"
           << "    \"max_error_gps_week\": " << summary.positioning.max_error_gps_week << ",\n"
           << "    \"max_error_sow_sec\": " << summary.positioning.max_error_sow_sec << ",\n"
           << "    \"final_position_error_m\": " << summary.positioning.final_position_error_m << ",\n"
           << "    \"final_position_gps_week\": " << summary.positioning.final_position_gps_week << ",\n"
           << "    \"final_position_sow_sec\": " << summary.positioning.final_position_sow_sec << "\n"
           << "  }\n"
           << "}\n";
    if (!output) {
        set_error(error_message, "failed to write transient validation JSON");
        return false;
    }
    return true;
}

} // namespace gnss_sim
