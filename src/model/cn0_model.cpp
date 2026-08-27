#include "model/cn0_model.h"

#include "gnss_sim/sim_time.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

namespace gnss_sim {
namespace {

constexpr std::int64_t kFastPeriodNs = 20LL * NANOSECONDS_PER_SECOND;
constexpr std::int64_t kSlowPeriodNs = 120LL * NANOSECONDS_PER_SECOND;
constexpr double kFastAmplitudeDb = 0.25;
constexpr double kSlowAmplitudeDb = 0.50;
constexpr double kBinToleranceDeg = 1.0e-9;
constexpr std::uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr const char* kModelSchemaVersion = "gnss-cn0-model-v1";
constexpr const char* kModelHeader =
    "schema_version,constellation,signal,elevation_min_deg,elevation_max_deg,upper_edge_inclusive,status,count,"
    "p05_dbhz,p10_dbhz,p25_dbhz,p50_dbhz,p75_dbhz,p90_dbhz,p95_dbhz,mean_dbhz,stddev_dbhz,mad_dbhz,"
    "delta_count,delta_p50_dbhz,delta_p90_dbhz,delta_p99_dbhz,ar1_status,ar1";

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_time(const SimTime& time) {
    return time.gps_week >= 0 && time.tow_ns >= 0 && time.tow_ns < GPS_WEEK_NANOSECONDS;
}

double signal_cn0_offset_db(SignalId signal_id) {
    switch (signal_id) {
        case SignalId::kGpsL1Ca:
            return 0.0;
        case SignalId::kGpsL1C:
            return 1.0;
        case SignalId::kGpsL2P:
            return -2.0;
        case SignalId::kGpsL2C:
            return -1.0;
        case SignalId::kGpsL5Q:
            return 0.5;
        case SignalId::kQzssL1Ca:
            return 0.5;
        case SignalId::kQzssL1C:
            return 1.0;
        case SignalId::kQzssL2C:
            return -0.5;
        case SignalId::kQzssL5Q:
            return 0.5;
        case SignalId::kGlonassG1:
            return -0.5;
        case SignalId::kGlonassG2:
            return -1.5;
        case SignalId::kGlonassG3:
            return -0.5;
        case SignalId::kGalileoE1:
            return 0.5;
        case SignalId::kGalileoE5A:
            return 1.0;
        case SignalId::kGalileoE5B:
            return 0.5;
        case SignalId::kGalileoE6:
            return -0.5;
        case SignalId::kBeidouB1I:
            return -0.5;
        case SignalId::kBeidouB3I:
            return -1.0;
        case SignalId::kBeidouB1C:
            return 0.5;
        case SignalId::kBeidouB2A:
            return 0.5;
        case SignalId::kBeidouB2B:
            return 0.0;
    }
    return 0.0;
}

double elevation_baseline_dbhz(double elevation_deg) {
    const double elevation = std::clamp(elevation_deg, 0.0, 90.0);
    if (elevation <= 5.0) {
        return 28.0 + 0.4 * elevation;
    }
    if (elevation <= 15.0) {
        return 30.0 + 0.35 * (elevation - 5.0);
    }
    if (elevation <= 30.0) {
        return 33.5 + 0.3 * (elevation - 15.0);
    }
    if (elevation <= 60.0) {
        return 38.0 + 0.2 * (elevation - 30.0);
    }
    return 44.0 + 0.1 * (elevation - 60.0);
}

double builtin_nominal_dbhz(SignalId signal_id, double elevation_deg) {
    return elevation_baseline_dbhz(elevation_deg) + signal_cn0_offset_db(signal_id);
}

std::int64_t positive_mod(std::int64_t value, std::int64_t modulus) {
    const std::int64_t remainder = value % modulus;
    return remainder < 0 ? remainder + modulus : remainder;
}

double triangle_wave(std::int64_t phase_ns, std::int64_t period_ns) {
    const double phase = static_cast<double>(positive_mod(phase_ns, period_ns)) / static_cast<double>(period_ns);
    if (phase < 0.5) {
        return -1.0 + 4.0 * phase;
    }
    return 3.0 - 4.0 * phase;
}

std::int64_t phase_offset_ns(std::uint64_t seed, SignalId signal_id, std::int64_t period_ns, std::uint64_t salt) {
    std::uint64_t value = seed ^ salt;
    value ^= static_cast<std::uint64_t>(static_cast<unsigned int>(signal_id) + 1U) * 0x9E3779B97F4A7C15ULL;
    value ^= value >> 30U;
    value *= 0xBF58476D1CE4E5B9ULL;
    value ^= value >> 27U;
    value *= 0x94D049BB133111EBULL;
    value ^= value >> 31U;
    return static_cast<std::int64_t>(value % static_cast<std::uint64_t>(period_ns));
}

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::size_t begin = 0;
    while (begin <= line.size()) {
        const std::size_t comma = line.find(',', begin);
        if (comma == std::string::npos) {
            fields.push_back(line.substr(begin));
            break;
        }
        fields.push_back(line.substr(begin, comma - begin));
        begin = comma + 1;
    }
    return fields;
}

bool parse_finite_double(const std::string& text, double* value) {
    if (value == nullptr || text.empty()) {
        return false;
    }
    std::istringstream input(text);
    input.imbue(std::locale::classic());
    input >> std::noskipws >> *value;
    return input && input.eof() && std::isfinite(*value);
}

bool parse_u64(const std::string& text, std::uint64_t* value) {
    if (value == nullptr || text.empty() || text[0] == '-') {
        return false;
    }
    std::istringstream input(text);
    input.imbue(std::locale::classic());
    input >> std::noskipws >> *value;
    return input && input.eof();
}

bool parse_binary_flag(const std::string& text, bool* value) {
    if (value == nullptr) {
        return false;
    }
    if (text == "0") {
        *value = false;
        return true;
    }
    if (text == "1") {
        *value = true;
        return true;
    }
    return false;
}

bool constellation_from_name(const std::string& name, GnssConstellation* constellation) {
    if (constellation == nullptr) {
        return false;
    }
    if (name == "GPS") {
        *constellation = GnssConstellation::kGps;
        return true;
    }
    if (name == "GLONASS") {
        *constellation = GnssConstellation::kGlonass;
        return true;
    }
    if (name == "GALILEO") {
        *constellation = GnssConstellation::kGalileo;
        return true;
    }
    if (name == "BEIDOU") {
        *constellation = GnssConstellation::kBeidou;
        return true;
    }
    if (name == "QZSS") {
        *constellation = GnssConstellation::kQzss;
        return true;
    }
    return false;
}

bool parse_optional_finite(const std::string& text, bool* available, double* value) {
    if (available == nullptr || value == nullptr) {
        return false;
    }
    if (text.empty()) {
        *available = false;
        *value = std::numeric_limits<double>::quiet_NaN();
        return true;
    }
    *available = parse_finite_double(text, value);
    return *available;
}

const Cn0CalibratedBin* previous_signal_bin(const std::vector<Cn0CalibratedBin>& bins, SignalId signal_id) {
    for (auto iterator = bins.rbegin(); iterator != bins.rend(); ++iterator) {
        if (iterator->signal_id == signal_id) {
            return &*iterator;
        }
    }
    return nullptr;
}

bool bins_touch(const Cn0CalibratedBin& lower, const Cn0CalibratedBin& upper) {
    return std::fabs(lower.elevation_max_deg - upper.elevation_min_deg) <= kBinToleranceDeg;
}

bool bin_contains(const Cn0CalibratedBin& bin, double elevation_deg) {
    if (elevation_deg < bin.elevation_min_deg - kBinToleranceDeg ||
        elevation_deg > bin.elevation_max_deg + kBinToleranceDeg) {
        return false;
    }
    if (std::fabs(elevation_deg - bin.elevation_max_deg) <= kBinToleranceDeg) {
        return bin.upper_edge_inclusive;
    }
    return elevation_deg >= bin.elevation_min_deg - kBinToleranceDeg && elevation_deg < bin.elevation_max_deg;
}

bool calibrated_nominal_dbhz(const Cn0Model& model, SignalId signal_id, double elevation_deg, double* nominal_dbhz) {
    if (nominal_dbhz == nullptr) {
        return false;
    }

    std::size_t current_index = model.calibrated_bins.size();
    for (std::size_t index = 0; index < model.calibrated_bins.size(); ++index) {
        const Cn0CalibratedBin& bin = model.calibrated_bins[index];
        if (bin.signal_id == signal_id && bin_contains(bin, elevation_deg)) {
            current_index = index;
            break;
        }
    }
    if (current_index == model.calibrated_bins.size() || !model.calibrated_bins[current_index].ready) {
        *nominal_dbhz = builtin_nominal_dbhz(signal_id, elevation_deg);
        return true;
    }

    const Cn0CalibratedBin& current = model.calibrated_bins[current_index];
    if (std::fabs(elevation_deg - current.elevation_center_deg) <= kBinToleranceDeg) {
        *nominal_dbhz = current.p50_dbhz;
        return true;
    }

    const Cn0CalibratedBin* neighbor = nullptr;
    if (elevation_deg < current.elevation_center_deg) {
        for (std::size_t index = current_index; index > 0; --index) {
            const Cn0CalibratedBin& candidate = model.calibrated_bins[index - 1];
            if (candidate.signal_id == signal_id) {
                neighbor = &candidate;
                break;
            }
        }
        if (neighbor != nullptr && neighbor->ready && bins_touch(*neighbor, current)) {
            const double span = current.elevation_center_deg - neighbor->elevation_center_deg;
            const double fraction = (elevation_deg - neighbor->elevation_center_deg) / span;
            *nominal_dbhz = neighbor->p50_dbhz + fraction * (current.p50_dbhz - neighbor->p50_dbhz);
            return true;
        }
    } else {
        for (std::size_t index = current_index + 1; index < model.calibrated_bins.size(); ++index) {
            const Cn0CalibratedBin& candidate = model.calibrated_bins[index];
            if (candidate.signal_id == signal_id) {
                neighbor = &candidate;
                break;
            }
        }
        if (neighbor != nullptr && neighbor->ready && bins_touch(current, *neighbor)) {
            const double span = neighbor->elevation_center_deg - current.elevation_center_deg;
            const double fraction = (elevation_deg - current.elevation_center_deg) / span;
            *nominal_dbhz = current.p50_dbhz + fraction * (neighbor->p50_dbhz - current.p50_dbhz);
            return true;
        }
    }

    *nominal_dbhz = current.p50_dbhz;
    return true;
}

bool file_identity(const char* file_path, Cn0ModelIdentity* identity, std::string* error_message) {
    if (file_path == nullptr || identity == nullptr) {
        set_error(error_message, "CN0 model identity request has invalid arguments");
        return false;
    }
    std::ifstream input(file_path, std::ios::binary);
    if (!input) {
        set_error(error_message, std::string("cannot open configured CN0 model: ") + file_path);
        return false;
    }
    std::uint64_t hash = kFnvOffsetBasis;
    std::uint64_t size = 0;
    char buffer[65536];
    while (input) {
        input.read(buffer, static_cast<std::streamsize>(sizeof(buffer)));
        const std::streamsize count = input.gcount();
        for (std::streamsize index = 0; index < count; ++index) {
            hash ^= static_cast<unsigned char>(buffer[index]);
            hash *= kFnvPrime;
        }
        size += static_cast<std::uint64_t>(count);
    }
    if (!input.eof()) {
        set_error(error_message, "failed while hashing configured CN0 model");
        return false;
    }

    std::ostringstream hash_stream;
    hash_stream.imbue(std::locale::classic());
    hash_stream << std::hex << std::nouppercase << std::setw(16) << std::setfill('0') << hash;
    identity->schema_version = kModelSchemaVersion;
    identity->file_name = std::filesystem::path(file_path).filename().generic_string();
    identity->hash = hash_stream.str();
    identity->size_bytes = size;
    return true;
}

bool validate_statistics_fields(const std::vector<std::string>& fields, std::uint64_t count, const std::string& status,
                                std::size_t line_number, double* p50_dbhz, std::string* error_message) {
    double values[10]{};
    for (int index = 0; index < 10; ++index) {
        const std::string& field = fields[static_cast<std::size_t>(8 + index)];
        if (count == 0) {
            if (!field.empty()) {
                set_error(error_message,
                          "CN0 model EMPTY row contains statistics at line " + std::to_string(line_number));
                return false;
            }
            continue;
        }
        if (!parse_finite_double(field, &values[index])) {
            set_error(error_message,
                      "CN0 model contains missing/non-finite statistics at line " + std::to_string(line_number));
            return false;
        }
    }
    if (status == "EMPTY" && count != 0) {
        set_error(error_message, "CN0 model EMPTY row has nonzero count at line " + std::to_string(line_number));
        return false;
    }
    if ((status == "SPARSE" || status == "READY") && count == 0) {
        set_error(error_message, "CN0 model non-empty row has zero count at line " + std::to_string(line_number));
        return false;
    }
    if (status != "EMPTY" && status != "SPARSE" && status != "READY") {
        set_error(error_message, "CN0 model has unsupported bin status at line " + std::to_string(line_number));
        return false;
    }
    if (count > 0) {
        if (!(values[0] <= values[1] && values[1] <= values[2] && values[2] <= values[3] && values[3] <= values[4] &&
              values[4] <= values[5] && values[5] <= values[6]) ||
            values[8] < 0.0 || values[9] < 0.0) {
            set_error(error_message,
                      "CN0 model statistics are internally inconsistent at line " + std::to_string(line_number));
            return false;
        }
        *p50_dbhz = values[3];
    } else {
        *p50_dbhz = std::numeric_limits<double>::quiet_NaN();
    }
    return true;
}

bool validate_temporal_fields(const std::vector<std::string>& fields, std::size_t line_number,
                              std::string* error_message) {
    std::uint64_t delta_count = 0;
    if (!parse_u64(fields[18], &delta_count)) {
        set_error(error_message, "CN0 model has invalid delta_count at line " + std::to_string(line_number));
        return false;
    }
    bool delta_available[3]{};
    double delta[3]{};
    for (int index = 0; index < 3; ++index) {
        if (!parse_optional_finite(fields[static_cast<std::size_t>(19 + index)], &delta_available[index],
                                   &delta[index])) {
            set_error(error_message,
                      "CN0 model has non-finite Delta-CN0 statistic at line " + std::to_string(line_number));
            return false;
        }
    }
    if (delta_available[0] != delta_available[1] || delta_available[1] != delta_available[2]) {
        set_error(error_message,
                  "CN0 model has partially populated Delta-CN0 statistics at line " + std::to_string(line_number));
        return false;
    }
    if (delta_available[0] && (delta_count == 0 || delta[0] < 0.0 || delta[0] > delta[1] || delta[1] > delta[2])) {
        set_error(error_message,
                  "CN0 model Delta-CN0 statistics are inconsistent at line " + std::to_string(line_number));
        return false;
    }

    const std::string& ar1_status = fields[22];
    bool ar1_available = false;
    double ar1 = 0.0;
    if (!parse_optional_finite(fields[23], &ar1_available, &ar1)) {
        set_error(error_message, "CN0 model has non-finite AR(1) value at line " + std::to_string(line_number));
        return false;
    }
    if (ar1_status == "AVAILABLE") {
        if (!ar1_available || ar1 < -1.0 - 1.0e-12 || ar1 > 1.0 + 1.0e-12) {
            set_error(error_message,
                      "CN0 model AVAILABLE AR(1) row has invalid value at line " + std::to_string(line_number));
            return false;
        }
    } else if (ar1_status == "INSUFFICIENT_SUPPORT" || ar1_status == "ZERO_VARIANCE") {
        if (ar1_available) {
            set_error(error_message, "CN0 model unavailable AR(1) row unexpectedly has a value at line " +
                                         std::to_string(line_number));
            return false;
        }
    } else {
        set_error(error_message, "CN0 model has unsupported AR(1) status at line " + std::to_string(line_number));
        return false;
    }
    return true;
}

} // namespace

Cn0Model make_builtin_cn0_model(std::uint64_t seed) {
    Cn0Model model{};
    model.source = Cn0ModelSource::kBuiltinFallback;
    model.seed = seed;
    model.identity.schema_version = "builtin-cn0-v1";
    return model;
}

bool load_cn0_model_csv(const char* file_path, std::uint64_t seed, Cn0Model* model, std::string* error_message) {
    if (file_path == nullptr || file_path[0] == '\0' || model == nullptr) {
        set_error(error_message, "configured CN0 model path/output must not be null or empty");
        return false;
    }

    Cn0Model loaded{};
    loaded.source = Cn0ModelSource::kCalibratedCsv;
    loaded.seed = seed;
    if (!file_identity(file_path, &loaded.identity, error_message)) {
        return false;
    }

    std::ifstream input(file_path, std::ios::binary);
    if (!input) {
        set_error(error_message, std::string("cannot open configured CN0 model: ") + file_path);
        return false;
    }
    input.imbue(std::locale::classic());
    std::string line;
    if (!std::getline(input, line)) {
        set_error(error_message, "configured CN0 model is empty");
        return false;
    }
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    if (line != kModelHeader) {
        set_error(error_message, "configured CN0 model header/schema does not match gnss-cn0-model-v1");
        return false;
    }

    std::size_t line_number = 1;
    while (std::getline(input, line)) {
        ++line_number;
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) {
            set_error(error_message,
                      "configured CN0 model contains an empty row at line " + std::to_string(line_number));
            return false;
        }
        if (line.find('"') != std::string::npos) {
            set_error(error_message,
                      "configured CN0 model uses unsupported quoted CSV at line " + std::to_string(line_number));
            return false;
        }
        const std::vector<std::string> fields = split_csv_line(line);
        if (fields.size() != 24U) {
            set_error(error_message,
                      "configured CN0 model row must contain 24 columns at line " + std::to_string(line_number));
            return false;
        }
        if (fields[0] != kModelSchemaVersion) {
            set_error(error_message, "configured CN0 model row has incompatible schema version at line " +
                                         std::to_string(line_number));
            return false;
        }

        GnssConstellation constellation{};
        if (!constellation_from_name(fields[1], &constellation)) {
            set_error(error_message,
                      "configured CN0 model has unsupported constellation at line " + std::to_string(line_number));
            return false;
        }
        const SignalDefinition* definition = find_signal_definition_by_rinex(constellation, fields[2].c_str());
        if (definition == nullptr) {
            set_error(error_message, "configured CN0 model signal is absent from central signal definitions at line " +
                                         std::to_string(line_number));
            return false;
        }

        Cn0CalibratedBin bin{};
        bin.signal_id = definition->signal_id;
        if (!parse_finite_double(fields[3], &bin.elevation_min_deg) ||
            !parse_finite_double(fields[4], &bin.elevation_max_deg) ||
            !parse_binary_flag(fields[5], &bin.upper_edge_inclusive) || bin.elevation_min_deg < 0.0 ||
            bin.elevation_max_deg > 90.0 || bin.elevation_max_deg <= bin.elevation_min_deg) {
            set_error(error_message,
                      "configured CN0 model has invalid elevation bin at line " + std::to_string(line_number));
            return false;
        }
        bin.elevation_center_deg = 0.5 * (bin.elevation_min_deg + bin.elevation_max_deg);

        std::uint64_t count = 0;
        if (!parse_u64(fields[7], &count) ||
            !validate_statistics_fields(fields, count, fields[6], line_number, &bin.p50_dbhz, error_message) ||
            !validate_temporal_fields(fields, line_number, error_message)) {
            return false;
        }
        bin.ready = fields[6] == "READY";

        const Cn0CalibratedBin* previous = previous_signal_bin(loaded.calibrated_bins, bin.signal_id);
        if (previous != nullptr) {
            if (bin.elevation_min_deg <= previous->elevation_min_deg + kBinToleranceDeg) {
                set_error(error_message, "configured CN0 model bins are duplicate/non-monotonic at line " +
                                             std::to_string(line_number));
                return false;
            }
            if (bin.elevation_min_deg < previous->elevation_max_deg - kBinToleranceDeg) {
                set_error(error_message, "configured CN0 model bins overlap at line " + std::to_string(line_number));
                return false;
            }
            if (previous->upper_edge_inclusive) {
                set_error(error_message, "configured CN0 model has a non-final inclusive upper edge at line " +
                                             std::to_string(line_number - 1));
                return false;
            }
        }
        loaded.calibrated_bins.push_back(bin);
    }
    if (!input.eof()) {
        set_error(error_message, "failed while reading configured CN0 model");
        return false;
    }
    if (loaded.calibrated_bins.empty()) {
        set_error(error_message, "configured CN0 model contains no data rows");
        return false;
    }

    *model = std::move(loaded);
    return true;
}

bool cn0_model_estimate_dbhz(const Cn0Model& model, SignalId signal_id, double elevation_deg, const SimTime& time,
                             double* cn0_dbhz) {
    if (cn0_dbhz == nullptr || !std::isfinite(elevation_deg) || !valid_time(time) ||
        find_signal_definition(signal_id) == nullptr) {
        return false;
    }

    double nominal_dbhz = 0.0;
    switch (model.source) {
        case Cn0ModelSource::kBuiltinFallback:
            nominal_dbhz = builtin_nominal_dbhz(signal_id, elevation_deg);
            break;
        case Cn0ModelSource::kCalibratedCsv:
            if (!calibrated_nominal_dbhz(model, signal_id, elevation_deg, &nominal_dbhz)) {
                return false;
            }
            break;
    }

    const std::int64_t absolute_time_ns = static_cast<std::int64_t>(time.gps_week) * GPS_WEEK_NANOSECONDS + time.tow_ns;
    const std::int64_t fast_phase = absolute_time_ns + phase_offset_ns(model.seed, signal_id, kFastPeriodNs, 0xA5A5U);
    const std::int64_t slow_phase = absolute_time_ns + phase_offset_ns(model.seed, signal_id, kSlowPeriodNs, 0x5A5AU);

    double value = nominal_dbhz;
    value += kFastAmplitudeDb * triangle_wave(fast_phase, kFastPeriodNs);
    value += kSlowAmplitudeDb * triangle_wave(slow_phase, kSlowPeriodNs);
    *cn0_dbhz = std::clamp(value, 20.0, 55.0);
    return true;
}

const char* cn0_model_source_name(Cn0ModelSource source) {
    switch (source) {
        case Cn0ModelSource::kBuiltinFallback:
            return "BUILTIN_FALLBACK";
        case Cn0ModelSource::kCalibratedCsv:
            return "CALIBRATED_CSV";
    }
    return "UNKNOWN";
}

} // namespace gnss_sim
