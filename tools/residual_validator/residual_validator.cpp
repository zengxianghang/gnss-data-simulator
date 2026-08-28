#include "residual_validator.h"

#include "gnss/signal_definitions.h"

extern "C" {
#include <rtklib.h>
#include <rtklib_residual_ext.h>
}

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace gnss_sim::residual_validator {
namespace {

struct GroupKey {
    int scope_rank = 0;
    int signal_id = -1;
    std::string family;
    int satellite_number = 0;

    bool operator<(const GroupKey& other) const {
        return std::tie(scope_rank, signal_id, family, satellite_number) <
               std::tie(other.scope_rank, other.signal_id, other.family, other.satellite_number);
    }
};

struct Accumulator {
    const SignalDefinition* definition = nullptr;
    std::string scope;
    std::string family;
    int satellite_number = 0;
    std::uint64_t rows = 0;
    std::uint64_t code_unavailable = 0;
    std::uint64_t diagnostic_code_rows = 0;
    std::uint64_t diagnostic_doppler_rows = 0;
    double code_sum_squares = 0.0;
    double doppler_sum_squares = 0.0;
    double code_max_abs_m = 0.0;
    int code_max_gps_week = -1;
    double code_max_sow_sec = 0.0;
    double code_max_elevation_deg = 0.0;
    double doppler_max_abs_mps = 0.0;
    int doppler_max_gps_week = -1;
    double doppler_max_sow_sec = 0.0;
    double doppler_max_elevation_deg = 0.0;
    std::vector<double> code_abs;
    std::vector<double> doppler_abs;
};

struct ResidualContext {
    int gps_week = -1;
    double sow_sec = 0.0;
    double elevation_deg = 0.0;
};

void set_error(std::string* error_message, const std::string& value) {
    if (error_message != nullptr) {
        *error_message = value;
    }
}

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> fields;
    std::istringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    if (!line.empty() && line.back() == ',') {
        fields.emplace_back();
    }
    return fields;
}

std::map<std::string, std::size_t> header_columns(const std::string& line) {
    const std::vector<std::string> fields = split_csv(line);
    std::map<std::string, std::size_t> columns;
    for (std::size_t index = 0; index < fields.size(); ++index) {
        columns.emplace(fields[index], index);
    }
    return columns;
}

std::string native_rtklib_path(const std::string& path) {
#ifdef _WIN32
    std::string result = path;
    std::replace(result.begin(), result.end(), '/', '\\');
    return result;
#else
    return path;
#endif
}

bool load_nav(const std::string& path, nav_t* nav, std::string* error_message) {
    if (nav == nullptr || path.empty()) {
        set_error(error_message, "residual validator requires a navigation file");
        return false;
    }
    obs_t unused_obs{};
    sta_t station{};
    const std::string native_path = native_rtklib_path(path);
    if (readrnx(native_path.c_str(), 1, "", &unused_obs, nav, &station) == 0) {
        freeobs(&unused_obs);
        set_error(error_message, "cannot read navigation file: " + path);
        return false;
    }
    freeobs(&unused_obs);
    uniqnav(nav);
    return true;
}

int required_rtklib_message_type(const SignalDefinition& definition) {
    switch (definition.nav_message_family) {
        case NavMessageFamily::kGpsLnav:
        case NavMessageFamily::kQzssLnav:
            return NAV_LNAV;
        case NavMessageFamily::kGpsCnav:
        case NavMessageFamily::kQzssCnav:
            return NAV_CNAV;
        case NavMessageFamily::kGpsCnav2:
        case NavMessageFamily::kQzssCnav2:
            return NAV_CNV2;
        case NavMessageFamily::kGlonassFdma:
            return NAV_FDMA;
        case NavMessageFamily::kGlonassL3Oc:
            return NAV_L3OC;
        case NavMessageFamily::kGalileoInav:
            return NAV_INAV;
        case NavMessageFamily::kGalileoFnav:
            return NAV_FNAV;
        case NavMessageFamily::kGalileoCnav:
            return 0;
        case NavMessageFamily::kBeidouD1D2:
            return NAV_D1 | NAV_D2 | NAV_D1D2;
        case NavMessageFamily::kBeidouBcnav1:
            return NAV_CNV1;
        case NavMessageFamily::kBeidouBcnav2:
            return NAV_CNV2;
        case NavMessageFamily::kBeidouBcnav3:
            return NAV_CNV3;
    }
    return 0;
}

bool modern_gps_signal(SignalId signal_id) {
    return signal_id == SignalId::kGpsL1C || signal_id == SignalId::kGpsL2C || signal_id == SignalId::kGpsL5Q;
}

bool external_e6_state(const SignalDefinition& definition, const std::string& family,
                       const std::string& code_bias_status) {
    return definition.signal_id == SignalId::kGalileoE6 && family == "UNKNOWN" && code_bias_status == "APPLIED";
}

prcopt_t make_residual_options(AtmosphereMode atmosphere_mode) {
    prcopt_t options = prcopt_default;
    options.mode = PMODE_SINGLE;
    options.nf = 1;
    options.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_CMP;
    options.elmin = 0.0;
    options.sateph = EPHOPT_BRDC;
    if (atmosphere_mode == AtmosphereMode::kBroadcast) {
        options.ionoopt = IONOOPT_BRDC;
        options.tropopt = TROPOPT_SAAS;
    } else {
        options.ionoopt = IONOOPT_OFF;
        options.tropopt = TROPOPT_OFF;
    }
    return options;
}

double percentile95(std::vector<double>* values) {
    if (values == nullptr || values->empty()) {
        return 0.0;
    }
    std::sort(values->begin(), values->end());
    if (values->size() == 1U) {
        return values->front();
    }
    constexpr double kProbability = 0.95;
    const double index = static_cast<double>(values->size() - 1U) * kProbability;
    const std::size_t lower = static_cast<std::size_t>(std::floor(index));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(index));
    const double fraction = index - static_cast<double>(lower);
    return (*values)[lower] + fraction * ((*values)[upper] - (*values)[lower]);
}

Accumulator& get_group(std::map<GroupKey, Accumulator>* groups, int scope_rank, const SignalDefinition* definition,
                       const std::string& family, int satellite_number) {
    GroupKey key{scope_rank, static_cast<int>(definition->signal_id), family, satellite_number};
    auto [iterator, inserted] = groups->try_emplace(key);
    if (inserted) {
        iterator->second.definition = definition;
        iterator->second.scope =
            scope_rank == 0 ? "signal" : (scope_rank == 1 ? "signal_family" : "signal_family_satellite");
        iterator->second.family = family;
        iterator->second.satellite_number = satellite_number;
    }
    return iterator->second;
}

void update_group(Accumulator* accumulator, const ResidualContext& context, bool code_unavailable, bool diagnostic_code,
                  const double* code_residual_m, bool diagnostic_doppler, const double* doppler_residual_mps) {
    ++accumulator->rows;
    if (code_unavailable) {
        ++accumulator->code_unavailable;
    }
    if (code_residual_m != nullptr) {
        const double absolute = std::fabs(*code_residual_m);
        if (accumulator->code_abs.empty() || absolute > accumulator->code_max_abs_m) {
            accumulator->code_max_abs_m = absolute;
            accumulator->code_max_gps_week = context.gps_week;
            accumulator->code_max_sow_sec = context.sow_sec;
            accumulator->code_max_elevation_deg = context.elevation_deg;
        }
        accumulator->code_abs.push_back(absolute);
        accumulator->code_sum_squares += (*code_residual_m) * (*code_residual_m);
        if (diagnostic_code) {
            ++accumulator->diagnostic_code_rows;
        }
    }
    if (doppler_residual_mps != nullptr) {
        const double absolute = std::fabs(*doppler_residual_mps);
        if (accumulator->doppler_abs.empty() || absolute > accumulator->doppler_max_abs_mps) {
            accumulator->doppler_max_abs_mps = absolute;
            accumulator->doppler_max_gps_week = context.gps_week;
            accumulator->doppler_max_sow_sec = context.sow_sec;
            accumulator->doppler_max_elevation_deg = context.elevation_deg;
        }
        accumulator->doppler_abs.push_back(absolute);
        accumulator->doppler_sum_squares += (*doppler_residual_mps) * (*doppler_residual_mps);
        if (diagnostic_doppler) {
            ++accumulator->diagnostic_doppler_rows;
        }
    }
}

void update_all_groups(std::map<GroupKey, Accumulator>* groups, const SignalDefinition* definition,
                       const std::string& family, int satellite_number, const ResidualContext& context,
                       bool code_unavailable, bool diagnostic_code, const double* code_residual_m,
                       bool diagnostic_doppler, const double* doppler_residual_mps) {
    update_group(&get_group(groups, 0, definition, "ALL", 0), context, code_unavailable, diagnostic_code,
                 code_residual_m, diagnostic_doppler, doppler_residual_mps);
    update_group(&get_group(groups, 1, definition, family, 0), context, code_unavailable, diagnostic_code,
                 code_residual_m, diagnostic_doppler, doppler_residual_mps);
    update_group(&get_group(groups, 2, definition, family, satellite_number), context, code_unavailable,
                 diagnostic_code, code_residual_m, diagnostic_doppler, doppler_residual_mps);
}

SummaryRow finalize_group(Accumulator* accumulator) {
    SummaryRow row{};
    row.scope = accumulator->scope;
    row.signal_id = static_cast<int>(accumulator->definition->signal_id);
    row.signal_name = accumulator->definition->name;
    row.rinex_code = accumulator->definition->rinex_signal_code;
    row.oem7_signal_type = accumulator->definition->novatel_oem7_signal_type;
    row.family = accumulator->family;
    row.satellite_number = accumulator->satellite_number;
    row.rows = accumulator->rows;
    row.code_residuals = accumulator->code_abs.size();
    row.code_unavailable = accumulator->code_unavailable;
    row.diagnostic_code_rows = accumulator->diagnostic_code_rows;
    row.doppler_residuals = accumulator->doppler_abs.size();
    row.diagnostic_doppler_rows = accumulator->diagnostic_doppler_rows;
    if (!accumulator->code_abs.empty()) {
        row.code_rms_m = std::sqrt(accumulator->code_sum_squares / static_cast<double>(accumulator->code_abs.size()));
        row.code_max_abs_m = accumulator->code_max_abs_m;
        row.code_max_gps_week = accumulator->code_max_gps_week;
        row.code_max_sow_sec = accumulator->code_max_sow_sec;
        row.code_max_elevation_deg = accumulator->code_max_elevation_deg;
        row.code_p95_abs_m = percentile95(&accumulator->code_abs);
    }
    if (!accumulator->doppler_abs.empty()) {
        row.doppler_rms_mps =
            std::sqrt(accumulator->doppler_sum_squares / static_cast<double>(accumulator->doppler_abs.size()));
        row.doppler_max_abs_mps = accumulator->doppler_max_abs_mps;
        row.doppler_max_gps_week = accumulator->doppler_max_gps_week;
        row.doppler_max_sow_sec = accumulator->doppler_max_sow_sec;
        row.doppler_max_elevation_deg = accumulator->doppler_max_elevation_deg;
        row.doppler_p95_abs_mps = percentile95(&accumulator->doppler_abs);
    }
    return row;
}

std::string row_context(std::uint64_t line_number, const SignalDefinition& definition, int satellite_number,
                        int gps_week, double sow_sec) {
    std::ostringstream stream;
    stream << "truth line " << line_number << " signal=" << definition.name << " sat=" << satellite_number
           << " week=" << gps_week << " sow=" << std::fixed << std::setprecision(3) << sow_sec;
    return stream.str();
}

} // namespace

bool validate_observation_truth(const ValidationOptions& options, ValidationReport* report,
                                std::string* error_message) {
    if (report == nullptr || options.nav_path.empty() || options.observation_truth_path.empty()) {
        set_error(error_message, "residual validator requires nav path, truth CSV path, and output report");
        return false;
    }
    *report = ValidationReport{};

    auto nav = std::make_unique<nav_t>();
    if (!load_nav(options.nav_path, nav.get(), error_message)) {
        return false;
    }
    const auto nav_cleanup = [&nav]() { freenav(nav.get(), 0xFF); };

    std::ifstream input(options.observation_truth_path);
    if (!input) {
        nav_cleanup();
        set_error(error_message, "cannot open observation truth CSV: " + options.observation_truth_path);
        return false;
    }

    std::string line;
    if (!std::getline(input, line)) {
        nav_cleanup();
        set_error(error_message, "observation truth CSV is empty: " + options.observation_truth_path);
        return false;
    }
    const std::map<std::string, std::size_t> columns = header_columns(line);
    const char* required_columns[] = {
        "gps_week",
        "sow_sec",
        "signal_id",
        "signal_name",
        "satellite_number",
        "wavelength_m",
        "receiver_x_m",
        "receiver_y_m",
        "receiver_z_m",
        "receiver_vx_mps",
        "receiver_vy_mps",
        "receiver_vz_mps",
        "elevation_deg",
        "cn0_dbhz",
        "broadcast_message_family",
        "code_bias_status",
        "pseudorange_valid",
        "doppler_valid",
        "pseudorange_m",
        "doppler_hz",
        "satellite_x_m",
        "satellite_y_m",
        "satellite_z_m",
        "satellite_vx_mps",
        "satellite_vy_mps",
        "satellite_vz_mps",
        "satellite_clock_bias_m",
        "satellite_clock_drift_mps",
        "code_bias_m",
    };
    for (const char* required : required_columns) {
        if (columns.count(required) != 1U) {
            nav_cleanup();
            set_error(error_message, std::string("observation truth CSV missing required column: ") + required);
            return false;
        }
    }

    const prcopt_t residual_options = make_residual_options(options.atmosphere_mode);
    std::map<GroupKey, Accumulator> groups;
    std::uint64_t line_number = 1;
    try {
        while (std::getline(input, line)) {
            ++line_number;
            if (line.empty()) {
                continue;
            }
            const std::vector<std::string> fields = split_csv(line);
            if (fields.size() != columns.size()) {
                throw std::runtime_error("column count does not match header");
            }

            const int signal_id_value = std::stoi(fields[columns.at("signal_id")]);
            const SignalDefinition* definition = find_signal_definition(static_cast<SignalId>(signal_id_value));
            if (definition == nullptr) {
                throw std::runtime_error("unknown signal_id=" + std::to_string(signal_id_value));
            }
            if (fields[columns.at("signal_name")] != definition->name) {
                throw std::runtime_error("signal_name does not match central signal definition");
            }

            int observation_code = 0;
            int frequency_index = 0;
            if (!signal_rtklib_observation_code(*definition, &observation_code, &frequency_index) ||
                observation_code <= 0) {
                throw std::runtime_error("cannot resolve RTKLIB observation code");
            }
            static_cast<void>(frequency_index);

            const int gps_week = std::stoi(fields[columns.at("gps_week")]);
            const double sow_sec = std::stod(fields[columns.at("sow_sec")]);
            const double elevation_deg = std::stod(fields[columns.at("elevation_deg")]);
            const int satellite_number = std::stoi(fields[columns.at("satellite_number")]);
            if (satellite_number <= 0 || satellite_number > 255) {
                throw std::runtime_error("invalid RTKLIB satellite number");
            }
            const std::string family_from_truth = fields[columns.at("broadcast_message_family")];
            const std::string code_bias_status = fields[columns.at("code_bias_status")];
            const bool use_external_state = external_e6_state(*definition, family_from_truth, code_bias_status);
            const std::string report_family = use_external_state ? "HAS_PRECISE" : family_from_truth;
            const bool pseudorange_valid = fields[columns.at("pseudorange_valid")] == "1";
            const bool doppler_valid = fields[columns.at("doppler_valid")] == "1";

            obsd_t observation{};
            observation.time = gpst2time(gps_week, sow_sec);
            observation.sat = static_cast<unsigned char>(satellite_number);
            observation.rcv = 1;
            observation.code[0] = static_cast<unsigned char>(observation_code);
            observation.SNR[0] = static_cast<unsigned char>(
                std::clamp(std::lround(std::stod(fields[columns.at("cn0_dbhz")]) * 4.0), 0L, 255L));
            // RTKLIB's Doppler residual reconstructs transmit time from the raw pseudorange.
            // Keep P[0] populated even when code validity is false; pseudorange_valid still
            // controls whether a code residual is evaluated.
            observation.P[0] = std::stod(fields[columns.at("pseudorange_m")]);
            if (doppler_valid) {
                observation.D[0] = static_cast<float>(std::stod(fields[columns.at("doppler_hz")]));
            }

            const double wavelength_m = std::stod(fields[columns.at("wavelength_m")]);
            const double receiver_position_m[3] = {std::stod(fields[columns.at("receiver_x_m")]),
                                                   std::stod(fields[columns.at("receiver_y_m")]),
                                                   std::stod(fields[columns.at("receiver_z_m")])};
            const double receiver_velocity_mps[3] = {std::stod(fields[columns.at("receiver_vx_mps")]),
                                                     std::stod(fields[columns.at("receiver_vy_mps")]),
                                                     std::stod(fields[columns.at("receiver_vz_mps")])};

            double satellite_state[6]{};
            double satellite_clock[2]{};
            double code_bias_m = 0.0;
            if (use_external_state) {
                satellite_state[0] = std::stod(fields[columns.at("satellite_x_m")]);
                satellite_state[1] = std::stod(fields[columns.at("satellite_y_m")]);
                satellite_state[2] = std::stod(fields[columns.at("satellite_z_m")]);
                satellite_state[3] = std::stod(fields[columns.at("satellite_vx_mps")]);
                satellite_state[4] = std::stod(fields[columns.at("satellite_vy_mps")]);
                satellite_state[5] = std::stod(fields[columns.at("satellite_vz_mps")]);
                satellite_clock[0] = std::stod(fields[columns.at("satellite_clock_bias_m")]) / CLIGHT;
                satellite_clock[1] = std::stod(fields[columns.at("satellite_clock_drift_mps")]) / CLIGHT;
                code_bias_m = std::stod(fields[columns.at("code_bias_m")]);
            }

            bool diagnostic_code = false;
            double code_residual_m = 0.0;
            const double* code_residual_ptr = nullptr;
            if (pseudorange_valid) {
                int status = 0;
                if (use_external_state) {
                    status = rtklib_rescode_state_ext(&observation, nav.get(), &residual_options, receiver_position_m,
                                                      0.0, 0.0, satellite_state, satellite_clock, 0, code_bias_m,
                                                      wavelength_m, &code_residual_m, nullptr);
                } else {
                    const int message_type = required_rtklib_message_type(*definition);
                    status =
                        rtklib_rescode_signal_ext(&observation, nav.get(), &residual_options, receiver_position_m, 0.0,
                                                  0.0, message_type, wavelength_m, &code_residual_m, nullptr, nullptr);
                    if (status == 0 && options.allow_diagnostic_health && modern_gps_signal(definition->signal_id) &&
                        code_bias_status == "APPLIED") {
                        status = rtklib_rescode_signal_diagnostic_ext(&observation, nav.get(), &residual_options,
                                                                      receiver_position_m, 0.0, 0.0, message_type,
                                                                      wavelength_m, &code_residual_m, nullptr, nullptr);
                        diagnostic_code = status == 1;
                    }
                }
                if (status != 1) {
                    nav_cleanup();
                    set_error(error_message,
                              row_context(line_number, *definition, satellite_number, gps_week, sow_sec) +
                                  ": RTKLIB code residual failed for a truth-valid pseudorange");
                    return false;
                }
                code_residual_ptr = &code_residual_m;
            }

            bool diagnostic_doppler = false;
            double doppler_residual_mps = 0.0;
            const double* doppler_residual_ptr = nullptr;
            if (doppler_valid) {
                int status = 0;
                if (use_external_state) {
                    status = rtklib_resdop_state_ext(&observation, &residual_options, receiver_position_m,
                                                     receiver_velocity_mps, 0.0, satellite_state, satellite_clock, 0,
                                                     wavelength_m, &doppler_residual_mps, nullptr);
                } else if (options.allow_diagnostic_health && modern_gps_signal(definition->signal_id)) {
                    status = rtklib_resdop_signal_diagnostic_ext(&observation, nav.get(), &residual_options,
                                                                 receiver_position_m, receiver_velocity_mps, 0.0, 0,
                                                                 wavelength_m, &doppler_residual_mps, nullptr);
                    diagnostic_doppler = status == 1;
                } else {
                    status = rtklib_resdop_signal_ext(&observation, nav.get(), &residual_options, receiver_position_m,
                                                      receiver_velocity_mps, 0.0, 0, wavelength_m,
                                                      &doppler_residual_mps, nullptr);
                }
                if (status != 1) {
                    nav_cleanup();
                    set_error(error_message,
                              row_context(line_number, *definition, satellite_number, gps_week, sow_sec) +
                                  ": RTKLIB Doppler residual failed for a truth-valid Doppler");
                    return false;
                }
                doppler_residual_ptr = &doppler_residual_mps;
            }

            const ResidualContext residual_context{gps_week, sow_sec, elevation_deg};
            update_all_groups(&groups, definition, report_family, satellite_number, residual_context,
                              !pseudorange_valid, diagnostic_code, code_residual_ptr, diagnostic_doppler,
                              doppler_residual_ptr);
            ++report->input_rows;
        }
    } catch (const std::exception& exception) {
        nav_cleanup();
        set_error(error_message, "cannot parse/evaluate " + options.observation_truth_path + " at line " +
                                     std::to_string(line_number) + ": " + exception.what());
        return false;
    }

    report->rows.reserve(groups.size());
    for (auto& [key, accumulator] : groups) {
        static_cast<void>(key);
        report->rows.push_back(finalize_group(&accumulator));
    }
    nav_cleanup();
    return true;
}

bool write_summary_csv(const ValidationReport& report, const std::string& output_path, std::string* error_message) {
    std::ofstream output(output_path, std::ios::trunc);
    if (!output) {
        set_error(error_message, "cannot create residual summary CSV: " + output_path);
        return false;
    }
    output << "scope,signal_id,signal_name,rinex_code,oem7_signal_type,family,satellite_number,rows,code_residuals,"
              "code_unavailable,diagnostic_code_rows,code_rms_m,code_p95_abs_m,code_max_abs_m,code_max_gps_week,"
              "code_max_sow_sec,code_max_elevation_deg,doppler_residuals,diagnostic_doppler_rows,doppler_rms_mps,"
              "doppler_p95_abs_mps,doppler_max_abs_mps,doppler_max_gps_week,doppler_max_sow_sec,"
              "doppler_max_elevation_deg\n";
    output << std::fixed << std::setprecision(12);
    for (const SummaryRow& row : report.rows) {
        output << row.scope << ',' << row.signal_id << ',' << row.signal_name << ',' << row.rinex_code << ','
               << row.oem7_signal_type << ',' << row.family << ',' << row.satellite_number << ',' << row.rows << ','
               << row.code_residuals << ',' << row.code_unavailable << ',' << row.diagnostic_code_rows << ','
               << row.code_rms_m << ',' << row.code_p95_abs_m << ',' << row.code_max_abs_m << ','
               << row.code_max_gps_week << ',' << row.code_max_sow_sec << ',' << row.code_max_elevation_deg << ','
               << row.doppler_residuals << ',' << row.diagnostic_doppler_rows << ',' << row.doppler_rms_mps << ','
               << row.doppler_p95_abs_mps << ',' << row.doppler_max_abs_mps << ',' << row.doppler_max_gps_week << ','
               << row.doppler_max_sow_sec << ',' << row.doppler_max_elevation_deg << '\n';
    }
    output.flush();
    if (!output) {
        set_error(error_message, "failed while writing residual summary CSV: " + output_path);
        return false;
    }
    return true;
}

const SummaryRow* find_signal_summary(const ValidationReport& report, int signal_id) {
    for (const SummaryRow& row : report.rows) {
        if (row.scope == "signal" && row.signal_id == signal_id) {
            return &row;
        }
    }
    return nullptr;
}

} // namespace gnss_sim::residual_validator
