#include "rangea_roundtrip.h"

#include "gnss/rtklib_adapter.h"
#include "gnss/signal_definitions.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace gnss_sim {
namespace {

constexpr unsigned int kPseudorangeValidBit = 1U << 12U;
constexpr unsigned int kConstellationShift = 16U;
constexpr unsigned int kConstellationMask = 0x7U;
constexpr unsigned int kSignalTypeShift = 21U;
constexpr unsigned int kSignalTypeMask = 0x1FU;

struct ParsedRangeObservation {
    int satellite_number;
    const SignalDefinition* definition;
    double pseudorange_m;
    double doppler_hz;
    double cn0_dbhz;
    double lock_time_sec;
    unsigned int tracking_status;
    bool pseudorange_valid;
};

struct ParsedRangeEpoch {
    int gps_week;
    double sow_sec;
    std::vector<ParsedRangeObservation> observations;
};

struct SelectedObservation {
    int satellite_number;
    int priority;
    RtklibRawCodeObservation observation;
};

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

std::vector<std::string> split(const std::string& text, char delimiter) {
    std::vector<std::string> fields;
    std::istringstream stream(text);
    std::string field;
    while (std::getline(stream, field, delimiter)) {
        fields.push_back(field);
    }
    if (!text.empty() && text.back() == delimiter) {
        fields.emplace_back();
    }
    return fields;
}

bool parse_int(const std::string& text, int* value) {
    if (value == nullptr || text.empty()) {
        return false;
    }
    std::istringstream stream(text);
    stream.imbue(std::locale::classic());
    int parsed = 0;
    stream >> parsed;
    if (!stream || stream.peek() != std::char_traits<char>::eof()) {
        return false;
    }
    *value = parsed;
    return true;
}

bool parse_unsigned_hex(const std::string& text, unsigned int* value) {
    if (value == nullptr || text.empty()) {
        return false;
    }
    std::istringstream stream(text);
    stream.imbue(std::locale::classic());
    unsigned int parsed = 0;
    stream >> std::hex >> parsed;
    if (!stream || stream.peek() != std::char_traits<char>::eof()) {
        return false;
    }
    *value = parsed;
    return true;
}

bool parse_double(const std::string& text, double* value) {
    if (value == nullptr || text.empty()) {
        return false;
    }
    std::istringstream stream(text);
    stream.imbue(std::locale::classic());
    double parsed = 0.0;
    stream >> parsed;
    if (!stream || stream.peek() != std::char_traits<char>::eof() || !std::isfinite(parsed)) {
        return false;
    }
    *value = parsed;
    return true;
}

std::uint32_t crc32(const std::string& payload) {
    constexpr std::uint32_t kPolynomial = UINT32_C(0xEDB88320);
    std::uint32_t crc = 0U;
    for (unsigned char byte : payload) {
        crc ^= static_cast<std::uint32_t>(byte);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) != 0U ? (crc >> 1U) ^ kPolynomial : crc >> 1U;
        }
    }
    return crc;
}

bool constellation_from_status(unsigned int status, GnssConstellation* constellation, char* satellite_prefix) {
    if (constellation == nullptr || satellite_prefix == nullptr) {
        return false;
    }
    switch ((status >> kConstellationShift) & kConstellationMask) {
        case 0U:
            *constellation = GnssConstellation::kGps;
            *satellite_prefix = 'G';
            return true;
        case 1U:
            *constellation = GnssConstellation::kGlonass;
            *satellite_prefix = 'R';
            return true;
        case 3U:
            *constellation = GnssConstellation::kGalileo;
            *satellite_prefix = 'E';
            return true;
        case 4U:
            *constellation = GnssConstellation::kBeidou;
            *satellite_prefix = 'C';
            return true;
        case 5U:
            *constellation = GnssConstellation::kQzss;
            *satellite_prefix = 'J';
            return true;
        default:
            return false;
    }
}

bool range_prn_to_rinex_prn(GnssConstellation constellation, int range_prn, int* rinex_prn) {
    if (rinex_prn == nullptr) {
        return false;
    }
    int prn = range_prn;
    if (constellation == GnssConstellation::kGlonass) {
        prn -= 37;
    } else if (constellation == GnssConstellation::kQzss) {
        prn -= 192;
    }
    if (prn <= 0 || prn > 99) {
        return false;
    }
    *rinex_prn = prn;
    return true;
}

RtklibBroadcastMessageFamily rtklib_message_family(NavMessageFamily family) {
    switch (family) {
        case NavMessageFamily::kGpsLnav:
        case NavMessageFamily::kQzssLnav:
        case NavMessageFamily::kBeidouD1D2:
            return RtklibBroadcastMessageFamily::kLegacy;
        case NavMessageFamily::kGpsCnav:
        case NavMessageFamily::kQzssCnav:
            return RtklibBroadcastMessageFamily::kCnav;
        case NavMessageFamily::kGpsCnav2:
        case NavMessageFamily::kQzssCnav2:
            return RtklibBroadcastMessageFamily::kCnav2;
        case NavMessageFamily::kGlonassFdma:
            return RtklibBroadcastMessageFamily::kGlonassFdma;
        case NavMessageFamily::kGlonassL3Oc:
            return RtklibBroadcastMessageFamily::kGlonassL3Oc;
        case NavMessageFamily::kGalileoInav:
            return RtklibBroadcastMessageFamily::kGalileoInav;
        case NavMessageFamily::kGalileoFnav:
            return RtklibBroadcastMessageFamily::kGalileoFnav;
        case NavMessageFamily::kBeidouBcnav1:
            return RtklibBroadcastMessageFamily::kBeidouBcnav1;
        case NavMessageFamily::kBeidouBcnav2:
            return RtklibBroadcastMessageFamily::kBeidouBcnav2;
        case NavMessageFamily::kBeidouBcnav3:
            return RtklibBroadcastMessageFamily::kBeidouBcnav3;
        case NavMessageFamily::kGalileoCnav:
            return RtklibBroadcastMessageFamily::kUnknown;
    }
    return RtklibBroadcastMessageFamily::kUnknown;
}

bool parse_rangea_line(const std::string& raw_line, ParsedRangeEpoch* epoch, std::string* error_message) {
    if (epoch == nullptr) {
        set_error(error_message, "RANGEA parser output is null");
        return false;
    }

    std::string line = raw_line;
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    if (line.rfind("#RANGEA,", 0) != 0) {
        set_error(error_message, "RANGEA line does not begin with #RANGEA");
        return false;
    }

    const std::size_t star = line.rfind('*');
    if (star == std::string::npos || star <= 1U || star + 9U != line.size()) {
        set_error(error_message, "RANGEA line has malformed CRC framing");
        return false;
    }
    const std::string payload = line.substr(1, star - 1U);
    unsigned int expected_crc = 0U;
    if (!parse_unsigned_hex(line.substr(star + 1U), &expected_crc) || expected_crc != crc32(payload)) {
        set_error(error_message, "RANGEA CRC mismatch");
        return false;
    }

    const std::size_t semicolon = payload.find(';');
    if (semicolon == std::string::npos) {
        set_error(error_message, "RANGEA line is missing header/body separator");
        return false;
    }
    const std::vector<std::string> header = split(payload.substr(0, semicolon), ',');
    if (header.size() != 10U || header[0] != "RANGEA") {
        set_error(error_message, "RANGEA header field count or log name is invalid");
        return false;
    }

    ParsedRangeEpoch result{};
    if (!parse_int(header[5], &result.gps_week) || !parse_double(header[6], &result.sow_sec) || result.gps_week < 0 ||
        result.sow_sec < 0.0 || result.sow_sec >= 604800.0) {
        set_error(error_message, "RANGEA GPST header is invalid");
        return false;
    }

    const std::vector<std::string> body = split(payload.substr(semicolon + 1U), ',');
    int observation_count = 0;
    if (body.empty() || !parse_int(body[0], &observation_count) || observation_count < 0) {
        set_error(error_message, "RANGEA observation count is invalid");
        return false;
    }
    constexpr std::size_t kFieldsPerObservation = 10U;
    const std::size_t expected_fields = 1U + static_cast<std::size_t>(observation_count) * kFieldsPerObservation;
    if (body.size() != expected_fields) {
        set_error(error_message, "RANGEA observation count does not match serialized field count");
        return false;
    }

    result.observations.reserve(static_cast<std::size_t>(observation_count));
    for (int index = 0; index < observation_count; ++index) {
        const std::size_t base = 1U + static_cast<std::size_t>(index) * kFieldsPerObservation;
        int range_prn = 0;
        int glofreq = 0;
        double pseudorange_m = 0.0;
        double pseudorange_sigma_m = 0.0;
        double adr_cycles = 0.0;
        double adr_sigma_cycles = 0.0;
        double doppler_hz = 0.0;
        double cn0_dbhz = 0.0;
        double lock_time_sec = 0.0;
        unsigned int status = 0U;
        if (!parse_int(body[base], &range_prn) || !parse_int(body[base + 1U], &glofreq) ||
            !parse_double(body[base + 2U], &pseudorange_m) || !parse_double(body[base + 3U], &pseudorange_sigma_m) ||
            !parse_double(body[base + 4U], &adr_cycles) || !parse_double(body[base + 5U], &adr_sigma_cycles) ||
            !parse_double(body[base + 6U], &doppler_hz) || !parse_double(body[base + 7U], &cn0_dbhz) ||
            !parse_double(body[base + 8U], &lock_time_sec) || !parse_unsigned_hex(body[base + 9U], &status) ||
            pseudorange_sigma_m < 0.0 || adr_sigma_cycles < 0.0 || cn0_dbhz < 0.0 || lock_time_sec < 0.0) {
            set_error(error_message,
                      "RANGEA observation contains malformed numeric fields at index " + std::to_string(index));
            return false;
        }
        static_cast<void>(adr_cycles);

        GnssConstellation constellation{};
        char satellite_prefix = '\0';
        if (!constellation_from_status(status, &constellation, &satellite_prefix)) {
            set_error(error_message,
                      "RANGEA observation has unsupported constellation bits at index " + std::to_string(index));
            return false;
        }
        if ((constellation == GnssConstellation::kGlonass && (glofreq < 0 || glofreq > 13)) ||
            (constellation != GnssConstellation::kGlonass && glofreq != 0)) {
            set_error(error_message,
                      "RANGEA observation has invalid GLONASS frequency field at index " + std::to_string(index));
            return false;
        }

        int rinex_prn = 0;
        if (!range_prn_to_rinex_prn(constellation, range_prn, &rinex_prn)) {
            set_error(error_message, "RANGEA observation has invalid PRN at index " + std::to_string(index));
            return false;
        }
        char satellite_id[4] = {satellite_prefix, static_cast<char>('0' + rinex_prn / 10),
                                static_cast<char>('0' + rinex_prn % 10), '\0'};
        int satellite_number = 0;
        if (!rtklib_satellite_id_to_number(satellite_id, &satellite_number)) {
            set_error(error_message, std::string("RANGEA satellite mapping is unsupported: ") + satellite_id);
            return false;
        }

        const int signal_type = static_cast<int>((status >> kSignalTypeShift) & kSignalTypeMask);
        const SignalDefinition* definition = find_signal_definition_by_oem7(constellation, signal_type);
        if (definition == nullptr) {
            set_error(error_message, "RANGEA signal type has no canonical mapping at index " + std::to_string(index));
            return false;
        }

        const bool pseudorange_valid = (status & kPseudorangeValidBit) != 0U;
        if ((pseudorange_valid && pseudorange_m <= 0.0) || (!pseudorange_valid && pseudorange_m != 0.0)) {
            set_error(error_message, "RANGEA pseudorange validity bit disagrees with serialized value at index " +
                                         std::to_string(index));
            return false;
        }

        ParsedRangeObservation observation{};
        observation.satellite_number = satellite_number;
        observation.definition = definition;
        observation.pseudorange_m = pseudorange_m;
        observation.doppler_hz = doppler_hz;
        observation.cn0_dbhz = cn0_dbhz;
        observation.lock_time_sec = lock_time_sec;
        observation.tracking_status = status;
        observation.pseudorange_valid = pseudorange_valid;
        result.observations.push_back(observation);
    }

    *epoch = std::move(result);
    return true;
}

int find_selected_satellite(const std::vector<SelectedObservation>& selected, int satellite_number) {
    for (std::size_t index = 0; index < selected.size(); ++index) {
        if (selected[index].satellite_number == satellite_number) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

bool select_position_observations(const ParsedRangeEpoch& epoch, std::vector<RtklibRawCodeObservation>* observations,
                                  std::string* error_message) {
    if (observations == nullptr) {
        set_error(error_message, "RANGEA selected-observation output is null");
        return false;
    }
    std::vector<SelectedObservation> selected;
    selected.reserve(epoch.observations.size());

    for (const ParsedRangeObservation& source : epoch.observations) {
        if (!source.pseudorange_valid) {
            continue;
        }
        const int priority = signal_single_point_priority(source.definition->signal_id);
        if (priority < 0) {
            continue;
        }
        int observation_code = 0;
        int frequency_index = 0;
        if (!signal_rtklib_observation_code(*source.definition, &observation_code, &frequency_index)) {
            set_error(error_message,
                      std::string("cannot map RANGEA signal to RTKLIB code: ") + source.definition->name);
            return false;
        }
        static_cast<void>(frequency_index);
        const RtklibBroadcastMessageFamily family = rtklib_message_family(source.definition->nav_message_family);
        if (family == RtklibBroadcastMessageFamily::kUnknown) {
            continue;
        }

        RtklibRawCodeObservation candidate{};
        candidate.satellite_number = source.satellite_number;
        candidate.observation_code = observation_code;
        candidate.message_family = family;
        candidate.pseudorange_m = source.pseudorange_m;
        candidate.cn0_dbhz = source.cn0_dbhz;
        candidate.pseudorange_valid = true;

        const int existing = find_selected_satellite(selected, source.satellite_number);
        if (existing >= 0) {
            if (priority < selected[static_cast<std::size_t>(existing)].priority) {
                selected[static_cast<std::size_t>(existing)].priority = priority;
                selected[static_cast<std::size_t>(existing)].observation = candidate;
            }
            continue;
        }
        SelectedObservation entry{};
        entry.satellite_number = source.satellite_number;
        entry.priority = priority;
        entry.observation = candidate;
        selected.push_back(entry);
    }

    std::sort(selected.begin(), selected.end(), [](const SelectedObservation& lhs, const SelectedObservation& rhs) {
        return lhs.satellite_number < rhs.satellite_number;
    });
    observations->clear();
    observations->reserve(selected.size());
    for (const SelectedObservation& entry : selected) {
        observations->push_back(entry.observation);
    }
    return true;
}

} // namespace

bool validate_rangea_roundtrip_stream(std::istream* input, const char* rinex_nav_path, double truth_latitude_deg,
                                      double truth_longitude_deg, double truth_height_m, double elevation_mask_deg,
                                      bool broadcast_atmosphere, RangeaRoundtripSummary* summary,
                                      std::string* error_message) {
    if (input == nullptr || rinex_nav_path == nullptr || rinex_nav_path[0] == '\0' || summary == nullptr ||
        !std::isfinite(truth_latitude_deg) || !std::isfinite(truth_longitude_deg) || !std::isfinite(truth_height_m) ||
        !std::isfinite(elevation_mask_deg)) {
        set_error(error_message, "RANGEA round-trip request has invalid arguments");
        return false;
    }

    RtklibNavStore* nav = create_rtklib_nav_store();
    if (nav == nullptr) {
        set_error(error_message, "cannot allocate RINEX NAV store for RANGEA round-trip validation");
        return false;
    }
    if (!load_rinex_nav_file(nav, rinex_nav_path, error_message)) {
        destroy_rtklib_nav_store(nav);
        return false;
    }

    double truth_ecef_m[3]{};
    if (!rtklib_llh_to_ecef(truth_latitude_deg, truth_longitude_deg, truth_height_m, truth_ecef_m)) {
        destroy_rtklib_nav_store(nav);
        set_error(error_message, "cannot convert RANGEA round-trip truth position to ECEF");
        return false;
    }

    RangeaRoundtripSummary result{};
    std::string line;
    std::uint64_t line_number = 0;
    std::string last_position_diagnostic;
    while (std::getline(*input, line)) {
        ++line_number;
        if (line.rfind("#RANGEA,", 0) != 0) {
            continue;
        }

        ParsedRangeEpoch epoch{};
        std::string parse_error;
        if (!parse_rangea_line(line, &epoch, &parse_error)) {
            destroy_rtklib_nav_store(nav);
            set_error(error_message, "RANGEA line " + std::to_string(line_number) + ": " + parse_error);
            return false;
        }
        ++result.range_epochs;
        result.parsed_observations += static_cast<std::uint64_t>(epoch.observations.size());

        std::vector<RtklibRawCodeObservation> selected;
        if (!select_position_observations(epoch, &selected, error_message)) {
            destroy_rtklib_nav_store(nav);
            return false;
        }
        result.selected_position_observations += static_cast<std::uint64_t>(selected.size());
        if (selected.empty()) {
            continue;
        }

        RtklibPositionSolution solution{};
        std::string solve_error;
        if (!rtklib_solve_raw_single_position(nav, epoch.gps_week, epoch.sow_sec, selected.data(),
                                              static_cast<int>(selected.size()), elevation_mask_deg,
                                              broadcast_atmosphere, &solution, &solve_error)) {
            destroy_rtklib_nav_store(nav);
            set_error(error_message, "RTKLIB raw RANGEA solve failed at GPST " + std::to_string(epoch.gps_week) + "/" +
                                         std::to_string(epoch.sow_sec) + ": " + solve_error);
            return false;
        }
        if (!solution.valid) {
            last_position_diagnostic = solution.diagnostic;
            continue;
        }

        const double dx = solution.position_ecef_m[0] - truth_ecef_m[0];
        const double dy = solution.position_ecef_m[1] - truth_ecef_m[1];
        const double dz = solution.position_ecef_m[2] - truth_ecef_m[2];
        const double error_m = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (!std::isfinite(error_m)) {
            destroy_rtklib_nav_store(nav);
            set_error(error_message, "RTKLIB raw RANGEA solution produced non-finite position error");
            return false;
        }
        ++result.valid_position_epochs;
        if (error_m > result.max_position_error_m) {
            result.max_position_error_m = error_m;
            result.max_error_gps_week = epoch.gps_week;
            result.max_error_sow_sec = epoch.sow_sec;
        }
    }

    destroy_rtklib_nav_store(nav);
    if (input->bad()) {
        set_error(error_message, "I/O failure while streaming RANGEA log");
        return false;
    }
    if (result.range_epochs == 0U) {
        set_error(error_message, "RANGEA round-trip log contains no RANGEA epochs");
        return false;
    }
    if (result.valid_position_epochs == 0U) {
        std::string message = "RANGEA round-trip produced no valid RTKLIB position epoch";
        if (!last_position_diagnostic.empty()) {
            message += ": " + last_position_diagnostic;
        }
        set_error(error_message, message);
        return false;
    }

    *summary = result;
    return true;
}

bool validate_rangea_roundtrip_file(const char* log_path, const char* rinex_nav_path, double truth_latitude_deg,
                                    double truth_longitude_deg, double truth_height_m, double elevation_mask_deg,
                                    bool broadcast_atmosphere, RangeaRoundtripSummary* summary,
                                    std::string* error_message) {
    if (log_path == nullptr || log_path[0] == '\0') {
        set_error(error_message, "RANGEA round-trip log path is invalid");
        return false;
    }
    std::ifstream input(log_path, std::ios::binary);
    if (!input) {
        set_error(error_message, std::string("cannot open RANGEA round-trip log: ") + log_path);
        return false;
    }
    return validate_rangea_roundtrip_stream(&input, rinex_nav_path, truth_latitude_deg, truth_longitude_deg,
                                            truth_height_m, elevation_mask_deg, broadcast_atmosphere, summary,
                                            error_message);
}

} // namespace gnss_sim
