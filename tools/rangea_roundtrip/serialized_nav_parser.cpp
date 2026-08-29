#include "serialized_nav_parser.h"

#include "gnss/rtklib_adapter.h"

#include <cmath>
#include <cstdint>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

namespace gnss_sim {
namespace {

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
    unsigned int parsed = 0U;
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

bool recognized_log_name(const std::string& name) {
    return name == "GPSEPHEMA" || name == "GLOEPHEMERISA" || name == "GALEPHEMERISA" || name == "BD2EPHEMA" ||
           name == "QZSSEPHEMERISA" || name == "IONUTCA" || name == "BD2IONUTCA";
}

bool parse_frame(const std::string& raw_line, std::string* log_name, int* output_week, double* output_sow,
                 std::vector<std::string>* body, bool* recognized, std::string* error_message) {
    if (log_name == nullptr || output_week == nullptr || output_sow == nullptr || body == nullptr ||
        recognized == nullptr) {
        set_error(error_message, "serialized NAV parser received null output");
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
    if (comma == std::string::npos || comma <= 1U) {
        return true;
    }
    const std::string name = line.substr(1, comma - 1U);
    if (!recognized_log_name(name)) {
        return true;
    }
    *recognized = true;

    const std::size_t star = line.rfind('*');
    if (star == std::string::npos || star <= 1U || star + 9U != line.size()) {
        set_error(error_message, name + " has malformed CRC framing");
        return false;
    }
    const std::string payload = line.substr(1, star - 1U);
    unsigned int expected_crc = 0U;
    if (!parse_unsigned_hex(line.substr(star + 1U), &expected_crc) || expected_crc != crc32(payload)) {
        set_error(error_message, name + " CRC mismatch");
        return false;
    }
    const std::size_t semicolon = payload.find(';');
    if (semicolon == std::string::npos) {
        set_error(error_message, name + " is missing header/body separator");
        return false;
    }
    const std::vector<std::string> header = split(payload.substr(0, semicolon), ',');
    if (header.size() != 10U || header[0] != name || !parse_int(header[5], output_week) ||
        !parse_double(header[6], output_sow) || *output_week < 0 || *output_sow < 0.0 || *output_sow >= 604800.0) {
        set_error(error_message, name + " header is invalid");
        return false;
    }
    *log_name = name;
    *body = split(payload.substr(semicolon + 1U), ',');
    return true;
}

int week_near_sow(int reference_week, double reference_sow, double target_sow) {
    int week = reference_week;
    const double difference = target_sow - reference_sow;
    if (difference > 302400.0) {
        --week;
    } else if (difference < -302400.0) {
        ++week;
    }
    return week;
}

bool satellite_number(char prefix, int prn, int* result) {
    if (result == nullptr || prn <= 0) {
        return false;
    }
    if (prefix == 'J' && prn > 99) {
        const std::string numeric_id = std::to_string(prn);
        return rtklib_satellite_id_to_number(numeric_id.c_str(), result);
    }
    if (prn > 99) {
        return false;
    }
    char id[4] = {prefix, static_cast<char>('0' + prn / 10), static_cast<char>('0' + prn % 10), '\0'};
    return rtklib_satellite_id_to_number(id, result);
}

bool parse_generic_kepler(const std::string& name, int output_week, double output_sow,
                          const std::vector<std::string>& fields, NavOutputRecord* record, std::string* error_message) {
    NavOutputSystem system = NavOutputSystem::kUnknown;
    char prefix = '\0';
    std::size_t expected = 0U;
    bool beidou = false;
    if (name == "GPSEPHEMA") {
        system = NavOutputSystem::kGps;
        prefix = 'G';
        expected = 32U;
    } else if (name == "BD2EPHEMA") {
        system = NavOutputSystem::kBeidou;
        prefix = 'C';
        expected = 33U;
        beidou = true;
    } else if (name == "QZSSEPHEMERISA") {
        system = NavOutputSystem::kQzss;
        prefix = 'J';
        expected = 36U;
    } else {
        return false;
    }
    if (fields.size() != expected) {
        set_error(error_message, name + " body field count is invalid");
        return false;
    }

    KeplerianNavOutputData eph{};
    eph.system = system;
    eph.message_family = RtklibBroadcastMessageFamily::kLegacy;
    if (!parse_int(fields[0], &eph.prn) || !satellite_number(prefix, eph.prn, &eph.satellite_number) ||
        !parse_double(fields[1], &eph.transmit_sow_sec) || !parse_int(fields[2], &eph.svh) ||
        !parse_int(fields[3], &eph.iode) || !parse_int(fields[4], &eph.iodc) || !parse_int(fields[5], &eph.toe_week) ||
        !parse_int(fields[6], &eph.toc_week) || !parse_double(fields[7], &eph.toe_sow_sec) ||
        !parse_double(fields[8], &eph.semi_major_axis_m) || !parse_double(fields[9], &eph.delta_mean_motion_radps) ||
        !parse_double(fields[10], &eph.mean_anomaly_rad) || !parse_double(fields[11], &eph.eccentricity) ||
        !parse_double(fields[12], &eph.argument_of_perigee_rad) || !parse_double(fields[13], &eph.cuc_rad) ||
        !parse_double(fields[14], &eph.cus_rad) || !parse_double(fields[15], &eph.crc_m) ||
        !parse_double(fields[16], &eph.crs_m) || !parse_double(fields[17], &eph.cic_rad) ||
        !parse_double(fields[18], &eph.cis_rad) || !parse_double(fields[19], &eph.inclination_rad) ||
        !parse_double(fields[20], &eph.inclination_dot_radps) || !parse_double(fields[21], &eph.omega0_rad) ||
        !parse_double(fields[22], &eph.omega_dot_radps) || !parse_int(fields[23], &eph.iodc) ||
        !parse_double(fields[24], &eph.toc_sow_sec) || !parse_double(fields[25], &eph.tgd_sec[0])) {
        set_error(error_message, name + " contains malformed ephemeris fields");
        return false;
    }
    if (fields[3] != fields[4] || fields[5] != fields[6]) {
        set_error(error_message, name + " duplicate IODE/week fields disagree");
        return false;
    }

    std::size_t index = 26U;
    if (beidou && !parse_double(fields[index++], &eph.tgd_sec[1])) {
        set_error(error_message, name + " BDS TGD field is invalid");
        return false;
    }
    bool data_flag = false;
    if (!parse_double(fields[index++], &eph.clock_bias_sec) ||
        !parse_double(fields[index++], &eph.clock_drift_sec_per_sec) ||
        !parse_double(fields[index++], &eph.clock_drift_rate_sec_per_sec2) ||
        !parse_bool(fields[index++], &data_flag) || !parse_double(fields[index++], &eph.corrected_mean_motion_radps) ||
        !parse_double(fields[index++], &eph.sva)) {
        set_error(error_message, name + " clock/status fields are invalid");
        return false;
    }
    eph.flag = data_flag ? 1 : 0;
    // The ASCII body carries transmit SOW but not its week. Anchor the week to the log header
    // (the receiver delivery time), not Toe, so week-boundary delivery remains causal.
    eph.transmit_week = week_near_sow(output_week, output_sow, eph.transmit_sow_sec);
    eph.toc_week = week_near_sow(eph.toe_week, eph.toe_sow_sec, eph.toc_sow_sec);
    eph.sqrt_semi_major_axis_sqrt_m = std::sqrt(eph.semi_major_axis_m);
    eph.message_type = 0;
    eph.code = 0;
    eph.fit_hours = 0.0;

    if (system == NavOutputSystem::kQzss) {
        for (; index < fields.size(); ++index) {
            double reserved = 0.0;
            if (!parse_double(fields[index], &reserved) || reserved != 0.0) {
                set_error(error_message, name + " reserved QZSS field is invalid");
                return false;
            }
        }
    } else if (index != fields.size()) {
        set_error(error_message, name + " has unexpected trailing fields");
        return false;
    }
    if (eph.toe_week < 0 || eph.toc_week < 0 || eph.transmit_week < 0 || eph.toe_sow_sec < 0.0 ||
        eph.toe_sow_sec >= 604800.0 || eph.toc_sow_sec < 0.0 || eph.toc_sow_sec >= 604800.0 ||
        eph.transmit_sow_sec < 0.0 || eph.transmit_sow_sec >= 604800.0 || eph.semi_major_axis_m <= 0.0) {
        set_error(error_message, name + " ephemeris time/orbit fields are out of range");
        return false;
    }
    record->kind = RtklibNavRecordKind::kEphemeris;
    record->ephemeris = eph;
    return true;
}

bool parse_galileo(int output_week, double output_sow, const std::vector<std::string>& fields, NavOutputRecord* record,
                   std::string* error_message) {
    if (fields.size() != 38U) {
        set_error(error_message, "GALEPHEMERISA body field count is invalid");
        return false;
    }
    KeplerianNavOutputData eph{};
    eph.system = NavOutputSystem::kGalileo;
    bool fnav = false;
    bool inav = false;
    if (!parse_int(fields[0], &eph.prn) || !satellite_number('E', eph.prn, &eph.satellite_number) ||
        !parse_bool(fields[1], &fnav) || !parse_bool(fields[2], &inav) ||
        !parse_int(fields[3], &eph.galileo_e1b_health) || !parse_int(fields[4], &eph.galileo_e5a_health) ||
        !parse_int(fields[5], &eph.galileo_e5b_health) || !parse_int(fields[6], &eph.galileo_e1b_dvs) ||
        !parse_int(fields[7], &eph.galileo_e5a_dvs) || !parse_int(fields[8], &eph.galileo_e5b_dvs) ||
        !parse_double(fields[9], &eph.sva) || !parse_int(fields[10], &eph.svh) || !parse_int(fields[11], &eph.iode) ||
        !parse_double(fields[12], &eph.toe_sow_sec) || !parse_double(fields[13], &eph.sqrt_semi_major_axis_sqrt_m) ||
        !parse_double(fields[14], &eph.delta_mean_motion_radps) || !parse_double(fields[15], &eph.mean_anomaly_rad) ||
        !parse_double(fields[16], &eph.eccentricity) || !parse_double(fields[17], &eph.argument_of_perigee_rad) ||
        !parse_double(fields[18], &eph.cuc_rad) || !parse_double(fields[19], &eph.cus_rad) ||
        !parse_double(fields[20], &eph.crc_m) || !parse_double(fields[21], &eph.crs_m) ||
        !parse_double(fields[22], &eph.cic_rad) || !parse_double(fields[23], &eph.cis_rad) ||
        !parse_double(fields[24], &eph.inclination_rad) || !parse_double(fields[25], &eph.inclination_dot_radps) ||
        !parse_double(fields[26], &eph.omega0_rad) || !parse_double(fields[27], &eph.omega_dot_radps) ||
        !parse_double(fields[28], &eph.galileo_fnav_toc_sow_sec) ||
        !parse_double(fields[29], &eph.galileo_fnav_clock[0]) ||
        !parse_double(fields[30], &eph.galileo_fnav_clock[1]) ||
        !parse_double(fields[31], &eph.galileo_fnav_clock[2]) ||
        !parse_double(fields[32], &eph.galileo_inav_toc_sow_sec) ||
        !parse_double(fields[33], &eph.galileo_inav_clock[0]) ||
        !parse_double(fields[34], &eph.galileo_inav_clock[1]) ||
        !parse_double(fields[35], &eph.galileo_inav_clock[2]) || !parse_double(fields[36], &eph.tgd_sec[0]) ||
        !parse_double(fields[37], &eph.tgd_sec[1])) {
        set_error(error_message, "GALEPHEMERISA contains malformed fields");
        return false;
    }
    if (!inav && !fnav) {
        set_error(error_message, "GALEPHEMERISA contains no received navigation family");
        return false;
    }
    eph.galileo_fnav_received = fnav;
    eph.galileo_inav_received = inav;
    eph.toe_week = week_near_sow(output_week, output_sow, eph.toe_sow_sec);
    eph.transmit_week = output_week;
    eph.transmit_sow_sec = output_sow;
    eph.semi_major_axis_m = eph.sqrt_semi_major_axis_sqrt_m * eph.sqrt_semi_major_axis_sqrt_m;
    eph.iodc = eph.iode;
    eph.code = 0;
    eph.flag = 0;
    eph.fit_hours = 0.0;
    if (inav) {
        eph.message_family = RtklibBroadcastMessageFamily::kGalileoInav;
        eph.toc_sow_sec = eph.galileo_inav_toc_sow_sec;
        eph.clock_bias_sec = eph.galileo_inav_clock[0];
        eph.clock_drift_sec_per_sec = eph.galileo_inav_clock[1];
        eph.clock_drift_rate_sec_per_sec2 = eph.galileo_inav_clock[2];
    } else {
        eph.message_family = RtklibBroadcastMessageFamily::kGalileoFnav;
        eph.toc_sow_sec = eph.galileo_fnav_toc_sow_sec;
        eph.clock_bias_sec = eph.galileo_fnav_clock[0];
        eph.clock_drift_sec_per_sec = eph.galileo_fnav_clock[1];
        eph.clock_drift_rate_sec_per_sec2 = eph.galileo_fnav_clock[2];
    }
    eph.toc_week = week_near_sow(eph.toe_week, eph.toe_sow_sec, eph.toc_sow_sec);
    record->kind = RtklibNavRecordKind::kEphemeris;
    record->ephemeris = eph;
    return true;
}

bool normalize_week_sow(int* week, double* sow_sec) {
    if (week == nullptr || sow_sec == nullptr || *week < 0 || !std::isfinite(*sow_sec)) {
        return false;
    }
    while (*sow_sec < 0.0) {
        *sow_sec += 604800.0;
        --(*week);
    }
    while (*sow_sec >= 604800.0) {
        *sow_sec -= 604800.0;
        ++(*week);
    }
    return *week >= 0;
}

bool parse_glonass(int output_week, double output_sow, const std::vector<std::string>& fields, NavOutputRecord* record,
                   std::string* error_message) {
    if (fields.size() != 29U) {
        set_error(error_message, "GLOEPHEMERISA body field count is invalid");
        return false;
    }
    GlonassNavOutputData glo{};
    glo.message_family = RtklibBroadcastMessageFamily::kGlonassFdma;
    int constant_one = 0;
    int constant_zero_a = 0;
    int constant_zero_b = 0;
    int constant_zero_c = 0;
    int toe_milliseconds = 0;
    if (!parse_int(fields[0], &glo.slot_offset) || !parse_int(fields[1], &glo.frequency_offset) ||
        !parse_int(fields[2], &constant_one) || !parse_int(fields[3], &constant_zero_a) ||
        !parse_int(fields[4], &glo.toe_week) || !parse_int(fields[5], &toe_milliseconds) ||
        !parse_int(fields[6], &glo.gps_glonass_time_offset_sec) || !parse_int(fields[7], &glo.calendar_day_number) ||
        !parse_int(fields[8], &constant_zero_b) || !parse_int(fields[9], &constant_zero_c) ||
        !parse_int(fields[10], &glo.iode) || !parse_int(fields[11], &glo.svh) ||
        !parse_double(fields[12], &glo.position_ecef_m[0]) || !parse_double(fields[13], &glo.position_ecef_m[1]) ||
        !parse_double(fields[14], &glo.position_ecef_m[2]) || !parse_double(fields[15], &glo.velocity_ecef_mps[0]) ||
        !parse_double(fields[16], &glo.velocity_ecef_mps[1]) || !parse_double(fields[17], &glo.velocity_ecef_mps[2]) ||
        !parse_double(fields[18], &glo.acceleration_ecef_mps2[0]) ||
        !parse_double(fields[19], &glo.acceleration_ecef_mps2[1]) ||
        !parse_double(fields[20], &glo.acceleration_ecef_mps2[2]) || !parse_double(fields[21], &glo.clock_bias_sec) ||
        !parse_double(fields[22], &glo.relative_frequency_bias) ||
        !parse_double(fields[23], &glo.differential_delay_sec) ||
        !parse_double(fields[24], &glo.frame_time_glonass_day_sec) || !parse_int(fields[25], &glo.flags) ||
        !parse_int(fields[26], &glo.sva) || !parse_int(fields[27], &glo.age_days)) {
        set_error(error_message, "GLOEPHEMERISA contains malformed fields");
        return false;
    }
    int repeated_flags = 0;
    if (!parse_int(fields[28], &repeated_flags) || repeated_flags != glo.flags || constant_one != 1 ||
        constant_zero_a != 0 || constant_zero_b != 0 || constant_zero_c != 0) {
        set_error(error_message, "GLOEPHEMERISA fixed/repeated fields are inconsistent");
        return false;
    }
    glo.prn = glo.slot_offset - 37;
    glo.frequency_channel = glo.frequency_offset - 7;
    if (glo.prn <= 0 || glo.prn > 31 || !satellite_number('R', glo.prn, &glo.satellite_number) ||
        toe_milliseconds < 0) {
        set_error(error_message, "GLOEPHEMERISA slot/frequency/toe fields are invalid");
        return false;
    }
    glo.toe_sow_sec = static_cast<double>(toe_milliseconds) / 1000.0;
    const double frame_gpst_day_sec = glo.frame_time_glonass_day_sec - glo.gps_glonass_time_offset_sec;
    double normalized_day_sec = std::fmod(frame_gpst_day_sec, 86400.0);
    if (normalized_day_sec < 0.0) {
        normalized_day_sec += 86400.0;
    }
    const double toe_day_start = std::floor(glo.toe_sow_sec / 86400.0) * 86400.0;
    glo.frame_week = glo.toe_week;
    glo.frame_sow_sec = toe_day_start + normalized_day_sec;
    if (glo.frame_sow_sec - glo.toe_sow_sec > 43200.0) {
        glo.frame_sow_sec -= 86400.0;
    } else if (glo.toe_sow_sec - glo.frame_sow_sec > 43200.0) {
        glo.frame_sow_sec += 86400.0;
    }
    if (!normalize_week_sow(&glo.frame_week, &glo.frame_sow_sec)) {
        set_error(error_message, "GLOEPHEMERISA frame time cannot be normalized");
        return false;
    }
    glo.message_type = 0;
    static_cast<void>(output_week);
    static_cast<void>(output_sow);
    record->kind = RtklibNavRecordKind::kGlonassEphemeris;
    record->glonass = glo;
    return true;
}

bool parse_ionosphere(const std::string& name, int output_week, double output_sow,
                      const std::vector<std::string>& fields, NavOutputRecord* record, std::string* error_message) {
    if (fields.size() != 17U) {
        set_error(error_message, name + " body field count is invalid");
        return false;
    }
    IonosphereNavOutputData ion{};
    ion.system = name == "IONUTCA" ? NavOutputSystem::kGps : NavOutputSystem::kBeidou;
    ion.coefficient_count = 8;
    ion.legacy_metadata = true;
    ion.transmit_week = output_week;
    ion.transmit_sow_sec = output_sow;
    for (int index = 0; index < 8; ++index) {
        if (!parse_double(fields[static_cast<std::size_t>(index)], &ion.coefficients[index])) {
            set_error(error_message, name + " contains malformed ionosphere coefficients");
            return false;
        }
    }
    int utc_week = 0;
    double utc_tot = 0.0;
    int repeated_week = 0;
    int reserved_zero = 0;
    int leap_a = 0;
    int leap_b = 0;
    int reserved_final = 0;
    if (!parse_int(fields[8], &utc_week) || !parse_double(fields[9], &utc_tot) ||
        !parse_double(fields[10], &ion.utc[0]) || !parse_double(fields[11], &ion.utc[1]) ||
        !parse_int(fields[12], &repeated_week) || !parse_int(fields[13], &reserved_zero) ||
        !parse_int(fields[14], &leap_a) || !parse_int(fields[15], &leap_b) || !parse_int(fields[16], &reserved_final) ||
        repeated_week != utc_week || reserved_zero != 0 || reserved_final != 0 || leap_a != leap_b) {
        set_error(error_message, name + " UTC/leap fields are invalid");
        return false;
    }
    ion.utc[2] = utc_tot;
    ion.utc[3] = static_cast<double>(utc_week);
    ion.leap_seconds = leap_a;
    record->kind = RtklibNavRecordKind::kIonosphere;
    record->ionosphere = ion;
    return true;
}

} // namespace

bool parse_serialized_novatel_nav_line_independent(const std::string& raw_line, ParsedSerializedNavRecord* parsed,
                                                   bool* recognized, std::string* error_message) {
    if (parsed == nullptr || recognized == nullptr) {
        set_error(error_message, "serialized NAV parser request has invalid arguments");
        return false;
    }
    std::string log_name;
    int output_week = 0;
    double output_sow = 0.0;
    std::vector<std::string> body;
    if (!parse_frame(raw_line, &log_name, &output_week, &output_sow, &body, recognized, error_message)) {
        return false;
    }
    if (!*recognized) {
        return true;
    }

    NavOutputRecord record{};
    bool ok = false;
    if (log_name == "GPSEPHEMA" || log_name == "BD2EPHEMA" || log_name == "QZSSEPHEMERISA") {
        ok = parse_generic_kepler(log_name, output_week, output_sow, body, &record, error_message);
    } else if (log_name == "GALEPHEMERISA") {
        ok = parse_galileo(output_week, output_sow, body, &record, error_message);
    } else if (log_name == "GLOEPHEMERISA") {
        ok = parse_glonass(output_week, output_sow, body, &record, error_message);
    } else if (log_name == "IONUTCA" || log_name == "BD2IONUTCA") {
        ok = parse_ionosphere(log_name, output_week, output_sow, body, &record, error_message);
    }
    if (!ok) {
        return false;
    }
    parsed->output_gps_week = output_week;
    parsed->output_sow_sec = output_sow;
    parsed->record = record;
    return true;
}

} // namespace gnss_sim
