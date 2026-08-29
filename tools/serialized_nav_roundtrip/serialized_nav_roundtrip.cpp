#include "serialized_nav_roundtrip.h"

#include "gnss/nav_output_record.h"
#include "gnss/rtklib_adapter.h"
#include "rangea_roundtrip.h"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <locale>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace gnss_sim {
namespace {

struct ParsedNavLine {
    int output_gps_week;
    double output_sow_sec;
    NavOutputRecord record;
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

bool parse_hex(const std::string& text, std::uint32_t* value) {
    if (value == nullptr || text.empty()) {
        return false;
    }
    std::istringstream stream(text);
    stream.imbue(std::locale::classic());
    std::uint32_t parsed = 0U;
    stream >> std::hex >> parsed;
    if (!stream || stream.peek() != std::char_traits<char>::eof()) {
        return false;
    }
    *value = parsed;
    return true;
}

bool parse_bool(const std::string& text, bool* value) {
    if (value == nullptr) {
        return false;
    }
    if (text == "TRUE") {
        *value = true;
        return true;
    }
    if (text == "FALSE") {
        *value = false;
        return true;
    }
    return false;
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

bool is_supported_nav_name(const std::string& name) {
    return name == "GPSEPHEMA" || name == "QZSSEPHEMERISA" || name == "GALEPHEMERISA" || name == "BD2EPHEMA" ||
           name == "GLOEPHEMERISA" || name == "IONUTCA" || name == "BD2IONUTCA";
}

int nearest_week(int reference_week, double reference_sow_sec, double target_sow_sec) {
    int week = reference_week;
    if (target_sow_sec > reference_sow_sec + 302400.0) {
        --week;
    } else if (target_sow_sec < reference_sow_sec - 302400.0) {
        ++week;
    }
    return week;
}

bool parse_generic_kepler(const std::string& name, const std::vector<std::string>& body, ParsedNavLine* parsed,
                          std::string* error_message) {
    const bool qzss = name == "QZSSEPHEMERISA";
    const bool beidou = name == "BD2EPHEMA";
    const std::size_t expected = qzss ? 36U : (beidou ? 33U : 32U);
    if (body.size() != expected) {
        set_error(error_message, name + " has unexpected field count");
        return false;
    }

    KeplerianNavOutputData eph{};
    eph.system = qzss ? NavOutputSystem::kQzss : (beidou ? NavOutputSystem::kBeidou : NavOutputSystem::kGps);
    eph.message_family = RtklibBroadcastMessageFamily::kLegacy;
    int duplicate_iode = 0;
    int duplicate_week = 0;
    bool flag = false;
    if (!parse_int(body[0], &eph.prn) || !parse_double(body[1], &eph.transmit_sow_sec) ||
        !parse_int(body[2], &eph.svh) || !parse_int(body[3], &eph.iode) || !parse_int(body[4], &duplicate_iode) ||
        !parse_int(body[5], &eph.toe_week) || !parse_int(body[6], &duplicate_week) ||
        !parse_double(body[7], &eph.toe_sow_sec) || !parse_double(body[8], &eph.semi_major_axis_m) ||
        !parse_double(body[9], &eph.delta_mean_motion_radps) || !parse_double(body[10], &eph.mean_anomaly_rad) ||
        !parse_double(body[11], &eph.eccentricity) || !parse_double(body[12], &eph.argument_of_perigee_rad) ||
        !parse_double(body[13], &eph.cuc_rad) || !parse_double(body[14], &eph.cus_rad) ||
        !parse_double(body[15], &eph.crc_m) || !parse_double(body[16], &eph.crs_m) ||
        !parse_double(body[17], &eph.cic_rad) || !parse_double(body[18], &eph.cis_rad) ||
        !parse_double(body[19], &eph.inclination_rad) || !parse_double(body[20], &eph.inclination_dot_radps) ||
        !parse_double(body[21], &eph.omega0_rad) || !parse_double(body[22], &eph.omega_dot_radps) ||
        !parse_int(body[23], &eph.iodc) || !parse_double(body[24], &eph.toc_sow_sec) ||
        !parse_double(body[25], &eph.tgd_sec[0])) {
        set_error(error_message, name + " contains malformed orbital fields");
        return false;
    }
    std::size_t clock_index = 26U;
    if (beidou) {
        if (!parse_double(body[26], &eph.tgd_sec[1])) {
            set_error(error_message, name + " contains malformed secondary group delay");
            return false;
        }
        clock_index = 27U;
    }
    if (!parse_double(body[clock_index], &eph.clock_bias_sec) ||
        !parse_double(body[clock_index + 1U], &eph.clock_drift_sec_per_sec) ||
        !parse_double(body[clock_index + 2U], &eph.clock_drift_rate_sec_per_sec2) ||
        !parse_bool(body[clock_index + 3U], &flag) ||
        !parse_double(body[clock_index + 4U], &eph.corrected_mean_motion_radps) ||
        !parse_double(body[clock_index + 5U], &eph.sva)) {
        set_error(error_message, name + " contains malformed clock/status fields");
        return false;
    }
    if (duplicate_iode != eph.iode || duplicate_week != eph.toe_week || eph.prn <= 0 || eph.toe_week < 0 ||
        eph.toe_sow_sec < 0.0 || eph.toe_sow_sec >= 604800.0 || eph.transmit_sow_sec < 0.0 ||
        eph.transmit_sow_sec >= 604800.0 || eph.toc_sow_sec < 0.0 || eph.toc_sow_sec >= 604800.0 ||
        eph.semi_major_axis_m <= 0.0) {
        set_error(error_message, name + " contains inconsistent required metadata");
        return false;
    }
    eph.flag = flag ? 1 : 0;
    eph.transmit_week = nearest_week(eph.toe_week, eph.toe_sow_sec, eph.transmit_sow_sec);
    eph.toc_week = nearest_week(eph.toe_week, eph.toe_sow_sec, eph.toc_sow_sec);
    parsed->record.kind = RtklibNavRecordKind::kEphemeris;
    parsed->record.ephemeris = eph;
    return true;
}

bool parse_galileo(const std::vector<std::string>& body, ParsedNavLine* parsed, std::string* error_message) {
    if (body.size() != 38U) {
        set_error(error_message, "GALEPHEMERISA has unexpected field count");
        return false;
    }
    KeplerianNavOutputData eph{};
    eph.system = NavOutputSystem::kGalileo;
    bool fnav_received = false;
    bool inav_received = false;
    if (!parse_int(body[0], &eph.prn) || !parse_bool(body[1], &fnav_received) || !parse_bool(body[2], &inav_received) ||
        !parse_int(body[3], &eph.galileo_e1b_health) || !parse_int(body[4], &eph.galileo_e5a_health) ||
        !parse_int(body[5], &eph.galileo_e5b_health) || !parse_int(body[6], &eph.galileo_e1b_dvs) ||
        !parse_int(body[7], &eph.galileo_e5a_dvs) || !parse_int(body[8], &eph.galileo_e5b_dvs) ||
        !parse_double(body[9], &eph.sva) || !parse_int(body[10], &eph.svh) || !parse_int(body[11], &eph.iode) ||
        !parse_double(body[12], &eph.toe_sow_sec) || !parse_double(body[13], &eph.sqrt_semi_major_axis_sqrt_m) ||
        !parse_double(body[14], &eph.delta_mean_motion_radps) || !parse_double(body[15], &eph.mean_anomaly_rad) ||
        !parse_double(body[16], &eph.eccentricity) || !parse_double(body[17], &eph.argument_of_perigee_rad) ||
        !parse_double(body[18], &eph.cuc_rad) || !parse_double(body[19], &eph.cus_rad) ||
        !parse_double(body[20], &eph.crc_m) || !parse_double(body[21], &eph.crs_m) ||
        !parse_double(body[22], &eph.cic_rad) || !parse_double(body[23], &eph.cis_rad) ||
        !parse_double(body[24], &eph.inclination_rad) || !parse_double(body[25], &eph.inclination_dot_radps) ||
        !parse_double(body[26], &eph.omega0_rad) || !parse_double(body[27], &eph.omega_dot_radps) ||
        !parse_double(body[28], &eph.galileo_fnav_toc_sow_sec) || !parse_double(body[29], &eph.galileo_fnav_clock[0]) ||
        !parse_double(body[30], &eph.galileo_fnav_clock[1]) || !parse_double(body[31], &eph.galileo_fnav_clock[2]) ||
        !parse_double(body[32], &eph.galileo_inav_toc_sow_sec) || !parse_double(body[33], &eph.galileo_inav_clock[0]) ||
        !parse_double(body[34], &eph.galileo_inav_clock[1]) || !parse_double(body[35], &eph.galileo_inav_clock[2]) ||
        !parse_double(body[36], &eph.tgd_sec[0]) || !parse_double(body[37], &eph.tgd_sec[1])) {
        set_error(error_message, "GALEPHEMERISA contains malformed fields");
        return false;
    }
    if (!inav_received && !fnav_received) {
        set_error(error_message, "GALEPHEMERISA carries neither INAV nor FNAV clock data");
        return false;
    }
    eph.galileo_fnav_received = fnav_received;
    eph.galileo_inav_received = inav_received;
    eph.message_family =
        inav_received ? RtklibBroadcastMessageFamily::kGalileoInav : RtklibBroadcastMessageFamily::kGalileoFnav;
    const double toc_sow = inav_received ? eph.galileo_inav_toc_sow_sec : eph.galileo_fnav_toc_sow_sec;
    const double* clock = inav_received ? eph.galileo_inav_clock : eph.galileo_fnav_clock;
    eph.clock_bias_sec = clock[0];
    eph.clock_drift_sec_per_sec = clock[1];
    eph.clock_drift_rate_sec_per_sec2 = clock[2];
    eph.semi_major_axis_m = eph.sqrt_semi_major_axis_sqrt_m * eph.sqrt_semi_major_axis_sqrt_m;
    eph.iodc = eph.iode;
    eph.toe_week = nearest_week(parsed->output_gps_week, parsed->output_sow_sec, eph.toe_sow_sec);
    eph.toc_sow_sec = toc_sow;
    eph.toc_week = nearest_week(eph.toe_week, eph.toe_sow_sec, toc_sow);
    // GALEPHEMERISA does not expose the original RINEX transmission epoch. The
    // receiver log header is the serialized availability time and is therefore
    // the only non-invented timestamp available for reconstructed ttr.
    eph.transmit_week = parsed->output_gps_week;
    eph.transmit_sow_sec = parsed->output_sow_sec;
    parsed->record.kind = RtklibNavRecordKind::kEphemeris;
    parsed->record.ephemeris = eph;
    return true;
}

bool parse_glonass(const std::vector<std::string>& body, ParsedNavLine* parsed, std::string* error_message) {
    if (body.size() != 29U) {
        set_error(error_message, "GLOEPHEMERISA has unexpected field count");
        return false;
    }
    GlonassNavOutputData glo{};
    int mode = 0;
    int reserved = 0;
    int reserved_a = 0;
    int reserved_b = 0;
    int duplicate_flags = 0;
    std::int64_t toe_ms = 0;
    double toe_ms_double = 0.0;
    if (!parse_int(body[0], &glo.slot_offset) || !parse_int(body[1], &glo.frequency_offset) ||
        !parse_int(body[2], &mode) || !parse_int(body[3], &reserved) || !parse_int(body[4], &glo.toe_week) ||
        !parse_double(body[5], &toe_ms_double) || !parse_int(body[6], &glo.gps_glonass_time_offset_sec) ||
        !parse_int(body[7], &glo.calendar_day_number) || !parse_int(body[8], &reserved_a) ||
        !parse_int(body[9], &reserved_b) || !parse_int(body[10], &glo.iode) || !parse_int(body[11], &glo.svh) ||
        !parse_double(body[12], &glo.position_ecef_m[0]) || !parse_double(body[13], &glo.position_ecef_m[1]) ||
        !parse_double(body[14], &glo.position_ecef_m[2]) || !parse_double(body[15], &glo.velocity_ecef_mps[0]) ||
        !parse_double(body[16], &glo.velocity_ecef_mps[1]) || !parse_double(body[17], &glo.velocity_ecef_mps[2]) ||
        !parse_double(body[18], &glo.acceleration_ecef_mps2[0]) ||
        !parse_double(body[19], &glo.acceleration_ecef_mps2[1]) ||
        !parse_double(body[20], &glo.acceleration_ecef_mps2[2]) || !parse_double(body[21], &glo.clock_bias_sec) ||
        !parse_double(body[22], &glo.relative_frequency_bias) || !parse_double(body[23], &glo.differential_delay_sec) ||
        !parse_double(body[24], &glo.frame_time_glonass_day_sec) || !parse_int(body[25], &glo.flags) ||
        !parse_int(body[26], &glo.sva) || !parse_int(body[27], &glo.age_days) ||
        !parse_int(body[28], &duplicate_flags)) {
        set_error(error_message, "GLOEPHEMERISA contains malformed fields");
        return false;
    }
    toe_ms = static_cast<std::int64_t>(std::llround(toe_ms_double));
    if (mode != 1 || reserved != 0 || reserved_a != 0 || reserved_b != 0 || duplicate_flags != glo.flags ||
        glo.slot_offset <= 37 || glo.frequency_offset < 0 || glo.frequency_offset > 13 || glo.toe_week < 0 ||
        toe_ms < 0 || toe_ms >= 604800000LL) {
        set_error(error_message, "GLOEPHEMERISA contains inconsistent metadata");
        return false;
    }
    glo.prn = glo.slot_offset - 37;
    glo.frequency_channel = glo.frequency_offset - 7;
    glo.message_family = RtklibBroadcastMessageFamily::kGlonassFdma;
    glo.message_type = 0;
    glo.toe_sow_sec = static_cast<double>(toe_ms) / 1000.0;
    // The ASCII record exposes frame time in GLONASS day seconds but not its
    // GPS week. For positioning, RTKLIB selects GLONASS ephemeris by toe; use
    // the receiver availability header as the serialized frame-time anchor.
    glo.frame_week = parsed->output_gps_week;
    glo.frame_sow_sec = parsed->output_sow_sec;
    parsed->record.kind = RtklibNavRecordKind::kGlonassEphemeris;
    parsed->record.glonass = glo;
    return true;
}

bool parse_ionutc(const std::string& name, const std::vector<std::string>& body, ParsedNavLine* parsed,
                  std::string* error_message) {
    if (body.size() != 17U) {
        set_error(error_message, name + " has unexpected field count");
        return false;
    }
    IonosphereNavOutputData ion{};
    ion.system = name == "IONUTCA" ? NavOutputSystem::kGps : NavOutputSystem::kBeidou;
    ion.coefficient_count = 8;
    ion.legacy_metadata = true;
    for (int index = 0; index < 8; ++index) {
        if (!parse_double(body[static_cast<std::size_t>(index)], &ion.coefficients[index])) {
            set_error(error_message, name + " contains malformed ionosphere coefficients");
            return false;
        }
    }
    int utc_week = 0;
    int utc_tot = 0;
    int duplicate_week = 0;
    int reserved = 0;
    int leap = 0;
    int future_leap = 0;
    int reserved_tail = 0;
    if (!parse_int(body[8], &utc_week) || !parse_int(body[9], &utc_tot) || !parse_double(body[10], &ion.utc[0]) ||
        !parse_double(body[11], &ion.utc[1]) || !parse_int(body[12], &duplicate_week) ||
        !parse_int(body[13], &reserved) || !parse_int(body[14], &leap) || !parse_int(body[15], &future_leap) ||
        !parse_int(body[16], &reserved_tail) || duplicate_week != utc_week || reserved != 0 || reserved_tail != 0) {
        set_error(error_message, name + " contains malformed UTC metadata");
        return false;
    }
    ion.utc[2] = static_cast<double>(utc_tot);
    ion.utc[3] = static_cast<double>(utc_week);
    ion.leap_seconds = name == "BD2IONUTCA" ? leap + 14 : leap;
    if (future_leap != leap) {
        set_error(error_message, name + " leap-second fields are inconsistent");
        return false;
    }
    ion.transmit_week = parsed->output_gps_week;
    ion.transmit_sow_sec = parsed->output_sow_sec;
    parsed->record.kind = RtklibNavRecordKind::kIonosphere;
    parsed->record.ionosphere = ion;
    return true;
}

bool parse_nav_line(const std::string& raw_line, ParsedNavLine* parsed, bool* recognized, std::string* error_message) {
    if (parsed == nullptr || recognized == nullptr) {
        set_error(error_message, "serialized NAV parser request has invalid arguments");
        return false;
    }
    *recognized = false;
    std::string line = raw_line;
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    if (line.empty() || line[0] != '#') {
        return true;
    }
    const std::size_t comma = line.find(',');
    if (comma == std::string::npos) {
        return true;
    }
    const std::string name = line.substr(1, comma - 1U);
    if (!is_supported_nav_name(name)) {
        return true;
    }
    *recognized = true;
    const std::size_t star = line.rfind('*');
    if (star == std::string::npos || star <= 1U || star + 9U != line.size()) {
        set_error(error_message, name + " has malformed CRC framing");
        return false;
    }
    const std::string payload = line.substr(1, star - 1U);
    std::uint32_t expected_crc = 0U;
    if (!parse_hex(line.substr(star + 1U), &expected_crc) || expected_crc != crc32(payload)) {
        set_error(error_message, name + " CRC mismatch");
        return false;
    }
    const std::size_t semicolon = payload.find(';');
    if (semicolon == std::string::npos) {
        set_error(error_message, name + " is missing header/body separator");
        return false;
    }
    const std::vector<std::string> header = split(payload.substr(0, semicolon), ',');
    if (header.size() != 10U || header[0] != name || !parse_int(header[5], &parsed->output_gps_week) ||
        !parse_double(header[6], &parsed->output_sow_sec) || parsed->output_gps_week < 0 ||
        parsed->output_sow_sec < 0.0 || parsed->output_sow_sec >= 604800.0) {
        set_error(error_message, name + " has invalid receiver header");
        return false;
    }
    const std::vector<std::string> body = split(payload.substr(semicolon + 1U), ',');
    if (name == "GPSEPHEMA" || name == "QZSSEPHEMERISA" || name == "BD2EPHEMA") {
        return parse_generic_kepler(name, body, parsed, error_message);
    }
    if (name == "GALEPHEMERISA") {
        return parse_galileo(body, parsed, error_message);
    }
    if (name == "GLOEPHEMERISA") {
        return parse_glonass(body, parsed, error_message);
    }
    return parse_ionutc(name, body, parsed, error_message);
}

void count_nav_record(const NavOutputRecord& record, SerializedNavRoundtripSummary* summary) {
    ++summary->nav_records;
    if (record.kind == RtklibNavRecordKind::kIonosphere) {
        ++summary->ionosphere_records;
        return;
    }
    if (record.kind == RtklibNavRecordKind::kGlonassEphemeris) {
        ++summary->glonass_ephemeris_records;
        return;
    }
    switch (record.ephemeris.system) {
        case NavOutputSystem::kGps:
            ++summary->gps_ephemeris_records;
            break;
        case NavOutputSystem::kGalileo:
            ++summary->galileo_ephemeris_records;
            break;
        case NavOutputSystem::kBeidou:
            ++summary->beidou_ephemeris_records;
            break;
        case NavOutputSystem::kQzss:
            ++summary->qzss_ephemeris_records;
            break;
        default:
            break;
    }
}

} // namespace

bool validate_serialized_nav_roundtrip_stream(std::istream* input, double truth_latitude_deg,
                                              double truth_longitude_deg, double truth_height_m,
                                              double elevation_mask_deg, bool broadcast_atmosphere,
                                              SerializedNavRoundtripSummary* summary, std::string* error_message) {
    if (input == nullptr || summary == nullptr || !std::isfinite(truth_latitude_deg) ||
        !std::isfinite(truth_longitude_deg) || !std::isfinite(truth_height_m) || !std::isfinite(elevation_mask_deg)) {
        set_error(error_message, "serialized NAV round-trip request has invalid arguments");
        return false;
    }
    RtklibNavStore* nav = create_rtklib_nav_store();
    if (nav == nullptr) {
        set_error(error_message, "cannot allocate serialized receiver NAV store");
        return false;
    }
    double truth_ecef_m[3]{};
    if (!rtklib_llh_to_ecef(truth_latitude_deg, truth_longitude_deg, truth_height_m, truth_ecef_m)) {
        destroy_rtklib_nav_store(nav);
        set_error(error_message, "cannot convert serialized round-trip truth position to ECEF");
        return false;
    }

    SerializedNavRoundtripSummary result{};
    std::string line;
    std::uint64_t line_number = 0U;
    std::string last_position_diagnostic;
    while (std::getline(*input, line)) {
        ++line_number;
        ParsedNavLine parsed_nav{};
        bool recognized_nav = false;
        std::string parse_error;
        if (!parse_nav_line(line, &parsed_nav, &recognized_nav, &parse_error)) {
            destroy_rtklib_nav_store(nav);
            set_error(error_message, "serialized NAV line " + std::to_string(line_number) + ": " + parse_error);
            return false;
        }
        if (recognized_nav) {
            if (!rtklib_append_nav_output_record(nav, parsed_nav.record, error_message)) {
                destroy_rtklib_nav_store(nav);
                return false;
            }
            count_nav_record(parsed_nav.record, &result);
            continue;
        }
        if (line.rfind("#RANGEA,", 0) != 0) {
            continue;
        }

        ParsedRangeEpoch epoch{};
        if (!parse_rangea_line_independent(line, &epoch, &parse_error)) {
            destroy_rtklib_nav_store(nav);
            set_error(error_message, "serialized RANGEA line " + std::to_string(line_number) + ": " + parse_error);
            return false;
        }
        ++result.range_epochs;
        result.parsed_observations += static_cast<std::uint64_t>(epoch.observations.size());
        RtklibPositionSolution solution{};
        int selected_count = 0;
        std::string solve_error;
        if (!solve_parsed_rangea_epoch(epoch, nav, elevation_mask_deg, broadcast_atmosphere, &solution, &selected_count,
                                       &solve_error)) {
            destroy_rtklib_nav_store(nav);
            set_error(error_message, "serialized self-contained solve failed at GPST " +
                                         std::to_string(epoch.gps_week) + "/" + std::to_string(epoch.sow_sec) + ": " +
                                         solve_error);
            return false;
        }
        result.selected_position_observations += static_cast<std::uint64_t>(selected_count);
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
            set_error(error_message, "serialized self-contained solve produced non-finite position error");
            return false;
        }
        if (result.valid_position_epochs == 0U) {
            result.first_valid_position_gps_week = epoch.gps_week;
            result.first_valid_position_sow_sec = epoch.sow_sec;
        }
        ++result.valid_position_epochs;
        result.final_position_error_m = error_m;
        result.final_position_gps_week = epoch.gps_week;
        result.final_position_sow_sec = epoch.sow_sec;
        if (error_m > result.max_position_error_m) {
            result.max_position_error_m = error_m;
            result.max_error_gps_week = epoch.gps_week;
            result.max_error_sow_sec = epoch.sow_sec;
        }
    }

    destroy_rtklib_nav_store(nav);
    if (input->bad()) {
        set_error(error_message, "I/O failure while streaming serialized receiver log");
        return false;
    }
    if (result.nav_records == 0U) {
        set_error(error_message, "serialized receiver log contains no supported EPHA/IONA records");
        return false;
    }
    if (result.range_epochs == 0U) {
        set_error(error_message, "serialized receiver log contains no RANGEA epochs");
        return false;
    }
    if (result.valid_position_epochs == 0U) {
        std::string message = "serialized receiver log produced no valid RTKLIB position epoch";
        if (!last_position_diagnostic.empty()) {
            message += ": " + last_position_diagnostic;
        }
        set_error(error_message, message);
        return false;
    }
    *summary = result;
    return true;
}

bool validate_serialized_nav_roundtrip_file(const char* log_path, double truth_latitude_deg, double truth_longitude_deg,
                                            double truth_height_m, double elevation_mask_deg, bool broadcast_atmosphere,
                                            SerializedNavRoundtripSummary* summary, std::string* error_message) {
    if (log_path == nullptr || log_path[0] == '\0') {
        set_error(error_message, "serialized NAV round-trip log path is invalid");
        return false;
    }
    std::ifstream input(log_path, std::ios::binary);
    if (!input) {
        set_error(error_message, std::string("cannot open serialized receiver log: ") + log_path);
        return false;
    }
    return validate_serialized_nav_roundtrip_stream(&input, truth_latitude_deg, truth_longitude_deg, truth_height_m,
                                                    elevation_mask_deg, broadcast_atmosphere, summary, error_message);
}

} // namespace gnss_sim
