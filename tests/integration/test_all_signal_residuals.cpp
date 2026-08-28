#include "gnss/rtklib_adapter.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

#include <gtest/gtest.h>

extern "C" {
#include <rtklib.h>
#include <rtklib_residual_ext.h>
#include <rtklib_signal_bias_ext.h>
}

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct SignalResidualStats {
    std::uint64_t rows = 0;
    std::uint64_t code_residuals = 0;
    std::uint64_t code_unavailable = 0;
    std::uint64_t doppler_residuals = 0;
    double max_abs_code_m = 0.0;
    double max_abs_doppler_mps = 0.0;
};

std::string brd4_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

std::string gps_cnv2_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/gps_cnv2_g04_2022278.rnx";
}

std::string jrc_has_sp3_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/jrc_has_2026001_e02.sp3";
}

std::string jrc_has_clock_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/jrc_has_2026001_e02.clk";
}

std::string jrc_has_bias_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/jrc_has_2026001_e02_c6c.bia";
}

double rinex4_field(const std::string& line, std::size_t offset) {
    return std::stod(line.substr(offset, 19));
}

void write_rinex4_field(std::ostream* output, double value) {
    *output << std::scientific << std::setprecision(12) << std::setw(19) << value;
}

void write_rinex4_four(std::ostream* output, double a, double b, double c, double d) {
    *output << "    ";
    write_rinex4_field(output, a);
    write_rinex4_field(output, b);
    write_rinex4_field(output, c);
    write_rinex4_field(output, d);
    *output << '\n';
}

bool write_g3_overlay_nav(const std::filesystem::path& directory, std::string* output_path) {
    if (output_path == nullptr) {
        return false;
    }
    std::ifstream source(brd4_nav_path(), std::ios::binary);
    if (!source) {
        return false;
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(source, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        lines.push_back(line);
    }

    const std::filesystem::path path = directory / "brd400dlr_plus_synthetic_glo_l3oc.rnx";
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) {
        return false;
    }
    for (const std::string& original : lines) {
        output << original << '\n';
    }

    // Test-only overlay: create an L3OC companion for every real FDMA record.
    // Orbit/clock values are copied from the source record; only the nonzero
    // ISC_L3OCp is synthetic. The checked-in BRD400DLR fixture is not modified
    // and real-source validation must never use this overlay.
    constexpr double kSyntheticIscL3OcpSec = 25.0e-9;
    for (std::size_t index = 0; index + 5 < lines.size(); ++index) {
        std::istringstream header(lines[index]);
        std::string marker;
        std::string kind;
        std::string satellite;
        std::string family;
        header >> marker >> kind >> satellite >> family;
        if (marker != ">" || kind != "EPH" || satellite.size() != 3U || satellite[0] != 'R' || family != "FDMA") {
            continue;
        }
        const std::string& clock = lines[index + 1];
        const std::string& orbit1 = lines[index + 2];
        const std::string& orbit2 = lines[index + 3];
        const std::string& orbit3 = lines[index + 4];
        if (clock.rfind(satellite, 0) != 0 || clock.size() < 80U || orbit1.size() < 80U || orbit2.size() < 80U ||
            orbit3.size() < 80U) {
            return false;
        }

        std::istringstream epoch_stream(clock.substr(4, 19));
        double epoch[6]{};
        if (!(epoch_stream >> epoch[0] >> epoch[1] >> epoch[2] >> epoch[3] >> epoch[4] >> epoch[5])) {
            return false;
        }
        const double t_tm = time2gpst(epoch2time(epoch), nullptr);

        output << "> EPH " << satellite << " L3OC\n";
        output << clock.substr(0, 23);
        write_rinex4_field(&output, rinex4_field(clock, 23));
        write_rinex4_field(&output, rinex4_field(clock, 42));
        write_rinex4_field(&output, 0.0);
        output << '\n';
        write_rinex4_four(&output, rinex4_field(orbit1, 4), rinex4_field(orbit1, 23), rinex4_field(orbit1, 42),
                          rinex4_field(orbit1, 61));
        write_rinex4_four(&output, rinex4_field(orbit2, 4), rinex4_field(orbit2, 23), rinex4_field(orbit2, 42), 0.0);
        write_rinex4_four(&output, rinex4_field(orbit3, 4), rinex4_field(orbit3, 23), rinex4_field(orbit3, 42),
                          kSyntheticIscL3OcpSec);
        for (int extra = 0; extra < 4; ++extra) {
            write_rinex4_four(&output, 0.0, 0.0, 0.0, 0.0);
        }
        write_rinex4_four(&output, 0.0, 0.0, 0.0, t_tm);
    }
    output.flush();
    if (!output) {
        return false;
    }
    *output_path = path.string();
    return true;
}

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> fields;
    std::istringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
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

bool load_nav(const std::string& path, nav_t* nav) {
    obs_t unused_obs{};
    sta_t station{};
    const std::string native_path = native_rtklib_path(path);
    if (readrnx(native_path.c_str(), 1, "", &unused_obs, nav, &station) == 0) {
        freeobs(&unused_obs);
        return false;
    }
    freeobs(&unused_obs);
    uniqnav(nav);
    return true;
}

void free_nav(nav_t* nav) {
    if (nav != nullptr) {
        freenav(nav, 0xFF);
    }
}

int required_rtklib_message_type(const gnss_sim::SignalDefinition& definition) {
    switch (definition.nav_message_family) {
        case gnss_sim::NavMessageFamily::kGpsLnav:
        case gnss_sim::NavMessageFamily::kQzssLnav:
            return NAV_LNAV;
        case gnss_sim::NavMessageFamily::kGpsCnav:
        case gnss_sim::NavMessageFamily::kQzssCnav:
            return NAV_CNAV;
        case gnss_sim::NavMessageFamily::kGpsCnav2:
        case gnss_sim::NavMessageFamily::kQzssCnav2:
            return NAV_CNV2;
        case gnss_sim::NavMessageFamily::kGlonassFdma:
            return NAV_FDMA;
        case gnss_sim::NavMessageFamily::kGlonassL3Oc:
            return NAV_L3OC;
        case gnss_sim::NavMessageFamily::kGalileoInav:
            return NAV_INAV;
        case gnss_sim::NavMessageFamily::kGalileoFnav:
            return NAV_FNAV;
        case gnss_sim::NavMessageFamily::kGalileoCnav:
            return 0;
        case gnss_sim::NavMessageFamily::kBeidouD1D2:
            return NAV_D1 | NAV_D2 | NAV_D1D2;
        case gnss_sim::NavMessageFamily::kBeidouBcnav1:
            return NAV_CNV1;
        case gnss_sim::NavMessageFamily::kBeidouBcnav2:
            return NAV_CNV2;
        case gnss_sim::NavMessageFamily::kBeidouBcnav3:
            return NAV_CNV3;
    }
    return 0;
}

TEST(V1Acceptance, EveryFrozenSignalRunsTruthStateCodeAndDopplerResidualChecks) {
    gnss_sim::SimConfig base_config = gnss_sim::default_sim_config();
    base_config.scenario = gnss_sim::ScenarioType::KS;
    base_config.atmosphere_mode = gnss_sim::AtmosphereMode::BROADCAST;
    base_config.elevation_mask_deg = 0.0;
    base_config.sampling_rate_hz = 1;
    base_config.duration_ns = 60LL * gnss_sim::NANOSECONDS_PER_SECOND;
    base_config.measurement_noise_enabled = false;
    base_config.multipath_enabled = false;
    base_config.receiver_clock_bias_m = 0.0;
    base_config.receiver_clock_drift_mps = 0.0;
    base_config.seed = 0x51U;

    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 436500.0, &start));

    const std::filesystem::path directory = "gnss_sim_all_signal_residuals";
    std::error_code filesystem_error;
    std::filesystem::remove_all(directory, filesystem_error);
    filesystem_error.clear();
    ASSERT_TRUE(std::filesystem::create_directories(directory, filesystem_error));
    ASSERT_FALSE(filesystem_error);

    std::string nav_text;
    ASSERT_TRUE(write_g3_overlay_nav(directory, &nav_text));
    auto nav = std::make_unique<nav_t>();
    ASSERT_TRUE(load_nav(nav_text, nav.get()));

    const gtime_t diagnostic_time = gpst2time(start.gps_week, gnss_sim::sim_time_sow_sec(start));
    const int g17 = satid2no("G17");
    ASSERT_GT(g17, 0);
    for (gnss_sim::SignalId signal_id : {gnss_sim::SignalId::kGpsL2C, gnss_sim::SignalId::kGpsL5Q}) {
        const gnss_sim::SignalDefinition* definition = gnss_sim::find_signal_definition(signal_id);
        ASSERT_NE(definition, nullptr);
        int observation_code = 0;
        int frequency_index = 0;
        ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*definition, &observation_code, &frequency_index));
        static_cast<void>(frequency_index);
        eph_t eph{};
        geph_t geph{};
        rtklib_signal_bias_info_ext_t eph_info{};
        rtklib_signal_bias_info_ext_t bias_info{};
        double bias_m = 0.0;
        const int eph_status =
            rtklib_signal_ephemeris_ext(diagnostic_time, g17, static_cast<unsigned char>(observation_code), NAV_CNAV,
                                        nav.get(), &eph, &geph, &eph_info);
        const int bias_status =
            rtklib_signal_code_bias_ext(diagnostic_time, g17, static_cast<unsigned char>(observation_code), NAV_CNAV,
                                        nav.get(), &bias_m, &bias_info);
        std::fprintf(
            stderr,
            "GPS_CNAV_DIRECT signal=%s sat=G17 eph_status=%d eph_type=%d bias_status=%d bias_type=%d bias_m=%.9f\n",
            definition->name, eph_status, eph_info.message_type, bias_status, bias_info.message_type, bias_m);
    }

    prcopt_t residual_options = prcopt_default;
    residual_options.mode = PMODE_SINGLE;
    residual_options.nf = 1;
    residual_options.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_CMP;
    residual_options.elmin = 0.0;
    residual_options.sateph = EPHOPT_BRDC;
    residual_options.ionoopt = IONOOPT_BRDC;
    residual_options.tropopt = TROPOPT_SAAS;

    std::map<std::string, SignalResidualStats> stats;
    std::set<std::string> seen_signals;
    std::set<std::string> printed_g17_rows;

    struct ResidualSite {
        const char* name;
        double latitude_deg;
        double longitude_deg;
        double start_sow_sec;
        int duration_sec;
    };
    const ResidualSite sites[] = {
        {"asia", 20.0, 120.0, 436500.0, 60},
        {"gps_cnav", 36.272115, -19.973734, 437100.0, 120},
        {"bds_bcnav12", -47.507042, -174.038033, 436500.0, 60},
    };

    for (const ResidualSite& site : sites) {
        gnss_sim::SimConfig config = base_config;
        config.receiver = {site.latitude_deg, site.longitude_deg, 100.0};
        config.duration_ns = static_cast<std::int64_t>(site.duration_sec) * gnss_sim::NANOSECONDS_PER_SECOND;
        gnss_sim::SimTime site_start{};
        ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(start.gps_week, site.start_sow_sec, &site_start));

        if (std::string(site.name) == "gps_cnav") {
            const gtime_t site_time = gpst2time(site_start.gps_week, gnss_sim::sim_time_sow_sec(site_start));
            double receiver_llh[3] = {site.latitude_deg * D2R, site.longitude_deg * D2R, 100.0};
            double receiver_ecef[3]{};
            pos2ecef(receiver_llh, receiver_ecef);
            double generic_rs[6]{};
            double generic_dts[2]{};
            double generic_var = 0.0;
            int generic_svh = 0;
            double generic_los[3]{};
            double generic_azel[2]{};
            const int generic_status = satpos(site_time, site_time, g17, EPHOPT_BRDC, nav.get(), generic_rs,
                                              generic_dts, &generic_var, &generic_svh);
            if (generic_status == 1 && geodist(generic_rs, receiver_ecef, generic_los) > 0.0) {
                satazel(receiver_llh, generic_los, generic_azel);
            }
            eph_t cnav_eph{};
            geph_t unused_geph{};
            rtklib_signal_bias_info_ext_t cnav_info{};
            int l2c_code = 0;
            int l2c_frequency_index = 0;
            const gnss_sim::SignalDefinition* l2c_definition =
                gnss_sim::find_signal_definition(gnss_sim::SignalId::kGpsL2C);
            ASSERT_NE(l2c_definition, nullptr);
            ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*l2c_definition, &l2c_code, &l2c_frequency_index));
            static_cast<void>(l2c_frequency_index);
            const int cnav_status =
                rtklib_signal_ephemeris_ext(site_time, g17, static_cast<unsigned char>(l2c_code), NAV_CNAV, nav.get(),
                                            &cnav_eph, &unused_geph, &cnav_info);
            double cnav_rs[3]{};
            double cnav_dts = 0.0;
            double cnav_var = 0.0;
            double cnav_los[3]{};
            double cnav_azel[2]{};
            if (cnav_status == 1) {
                eph2pos(site_time, &cnav_eph, cnav_rs, &cnav_dts, &cnav_var);
                if (geodist(cnav_rs, receiver_ecef, cnav_los) > 0.0) {
                    satazel(receiver_llh, cnav_los, cnav_azel);
                }
            }
            std::fprintf(stderr,
                         "GPS_CNAV_WINDOW sow=%.1f generic_status=%d generic_svh=%d generic_elev=%.6f cnav_status=%d "
                         "cnav_svh=%d cnav_elev=%.6f cnav_type=%d\n",
                         site.start_sow_sec, generic_status, generic_svh, generic_azel[1] * R2D, cnav_status,
                         cnav_eph.svh, cnav_azel[1] * R2D, cnav_info.message_type);

            for (int eph_index = 0; eph_index < nav->n; ++eph_index) {
                const eph_t& candidate = nav->eph[eph_index];
                if (candidate.hdr.sys != SYS_GPS || candidate.hdr.msg_type != NAV_CNAV)
                    continue;
                char candidate_id[16]{};
                satno2id(candidate.sat, candidate_id);
                double candidate_rs[3]{};
                double candidate_dts = 0.0;
                double candidate_var = 0.0;
                double candidate_pos[3]{};
                eph2pos(site_time, &candidate, candidate_rs, &candidate_dts, &candidate_var);
                ecef2pos(candidate_rs, candidate_pos);
                std::fprintf(stderr, "GPS_CNAV_CANDIDATE sat=%s svh=%d toe_age=%.1f sub_lat=%.6f sub_lon=%.6f\n",
                             candidate_id, candidate.svh, std::fabs(timediff(candidate.toe, site_time)),
                             candidate_pos[0] * R2D, candidate_pos[1] * R2D);
            }
        }

        const std::filesystem::path site_directory = directory / site.name;
        ASSERT_TRUE(std::filesystem::create_directories(site_directory, filesystem_error));
        ASSERT_FALSE(filesystem_error);
        const std::filesystem::path output_path = site_directory / "simulated.log";
        const std::string output_text = output_path.string();
        const gnss_sim::SimulatorRunOptions options{nav_text.c_str(), output_text.c_str(), site_start};
        gnss_sim::SimulatorRunSummary run_summary{};
        std::string error_message;
        ASSERT_TRUE(gnss_sim::run_simulator(config, options, &run_summary, &error_message))
            << "site=" << site.name << " " << error_message;

        std::ifstream input(site_directory / "observation_truth.csv");
        ASSERT_TRUE(input.good()) << site.name;
        std::string line;
        ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
        const std::map<std::string, std::size_t> column = header_columns(line);
        for (const char* required :
             {"gps_week",         "sow_sec",         "signal_id",         "signal_name",   "satellite_number",
              "wavelength_m",     "receiver_x_m",    "receiver_y_m",      "receiver_z_m",  "receiver_vx_mps",
              "receiver_vy_mps",  "receiver_vz_mps", "elevation_deg",     "cn0_dbhz",      "broadcast_message_family",
              "code_bias_status", "tracking_phase",  "pseudorange_valid", "doppler_valid", "pseudorange_m",
              "doppler_hz"}) {
            ASSERT_EQ(column.count(required), 1U) << "site=" << site.name << " column=" << required;
        }

        while (std::getline(input, line)) {
            if (line.empty())
                continue;
            const std::vector<std::string> fields = split_csv(line);
            ASSERT_EQ(fields.size(), column.size());

            const int signal_id = std::stoi(fields[column.at("signal_id")]);
            const gnss_sim::SignalDefinition* definition =
                gnss_sim::find_signal_definition(static_cast<gnss_sim::SignalId>(signal_id));
            ASSERT_NE(definition, nullptr);
            const std::string signal_name = fields[column.at("signal_name")];
            ASSERT_EQ(signal_name, definition->name);
            seen_signals.insert(signal_name);
            SignalResidualStats& signal_stats = stats[signal_name];
            ++signal_stats.rows;

            int observation_code = 0;
            int frequency_index = 0;
            ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*definition, &observation_code, &frequency_index));
            ASSERT_GT(observation_code, 0);
            static_cast<void>(frequency_index);

            obsd_t observation{};
            const int gps_week = std::stoi(fields[column.at("gps_week")]);
            const double sow_sec = std::stod(fields[column.at("sow_sec")]);
            observation.time = gpst2time(gps_week, sow_sec);
            observation.sat = static_cast<unsigned char>(std::stoi(fields[column.at("satellite_number")]));
            observation.rcv = 1;
            observation.code[0] = static_cast<unsigned char>(observation_code);
            observation.SNR[0] = static_cast<unsigned char>(
                std::clamp(std::lround(std::stod(fields[column.at("cn0_dbhz")]) * 4.0), 0L, 255L));
            observation.P[0] = std::stod(fields[column.at("pseudorange_m")]);
            observation.D[0] = static_cast<float>(std::stod(fields[column.at("doppler_hz")]));

            char satellite_id[16]{};
            satno2id(observation.sat, satellite_id);
            if (std::string(site.name) == "gps_cnav" && std::string(satellite_id) == "G17" &&
                (signal_name == "GPS L2C" || signal_name == "GPS L5Q") && printed_g17_rows.insert(signal_name).second) {
                std::fprintf(
                    stderr,
                    "GPS_CNAV_TRUTH signal=%s sat=G17 elev=%s family=%s bias_status=%s tracking=%s psr_valid=%s\n",
                    signal_name.c_str(), fields[column.at("elevation_deg")].c_str(),
                    fields[column.at("broadcast_message_family")].c_str(),
                    fields[column.at("code_bias_status")].c_str(), fields[column.at("tracking_phase")].c_str(),
                    fields[column.at("pseudorange_valid")].c_str());
            }

            const double wavelength_m = std::stod(fields[column.at("wavelength_m")]);
            const double receiver_position_m[3] = {std::stod(fields[column.at("receiver_x_m")]),
                                                   std::stod(fields[column.at("receiver_y_m")]),
                                                   std::stod(fields[column.at("receiver_z_m")])};
            const double receiver_velocity_mps[3] = {std::stod(fields[column.at("receiver_vx_mps")]),
                                                     std::stod(fields[column.at("receiver_vy_mps")]),
                                                     std::stod(fields[column.at("receiver_vz_mps")])};
            const int required_message_type = required_rtklib_message_type(*definition);

            rtklib_signal_bias_info_ext_t bias_info{};
            double code_residual_m = 0.0;
            int code_status =
                rtklib_rescode_signal_ext(&observation, nav.get(), &residual_options, receiver_position_m, 0.0, 0.0,
                                          required_message_type, wavelength_m, &code_residual_m, nullptr, &bias_info);
            const bool family_unavailable = fields[column.at("code_bias_status")] == "UNAVAILABLE_FOR_MESSAGE_FAMILY";
            const bool gps_l1c_developmental = signal_name == "GPS L1C";
            const bool gps_l5_preoperational = signal_name == "GPS L5Q";
            const bool pseudorange_valid = fields[column.at("pseudorange_valid")] == "1";
            const bool doppler_valid = fields[column.at("doppler_valid")] == "1";

            if (gps_l5_preoperational && !family_unavailable && pseudorange_valid) {
                // L5 CNAV is intentionally broadcast unhealthy while pre-operational.
                // Raw RANGE validity is independent of that navigation-health flag,
                // but strict RTKLIB residual use must still reject it.
                ASSERT_EQ(code_status, 0) << "strict code residual must preserve L5 broadcast-health exclusion";
                code_status = rtklib_rescode_signal_diagnostic_ext(&observation, nav.get(), &residual_options,
                                                                   receiver_position_m, 0.0, 0.0, required_message_type,
                                                                   wavelength_m, &code_residual_m, nullptr, &bias_info);
                ASSERT_EQ(code_status, 1) << "diagnostic L5Q code residual failed; site=" << site.name
                                          << " sat=" << static_cast<int>(observation.sat);
                ++signal_stats.code_residuals;
                signal_stats.max_abs_code_m = (std::max)(signal_stats.max_abs_code_m, std::fabs(code_residual_m));
            } else if (pseudorange_valid) {
                ASSERT_EQ(code_status, 1) << "site=" << site.name << " signal=" << signal_name
                                          << " sat=" << static_cast<int>(observation.sat);
                ++signal_stats.code_residuals;
                signal_stats.max_abs_code_m = (std::max)(signal_stats.max_abs_code_m, std::fabs(code_residual_m));
            } else {
                ++signal_stats.code_unavailable;
                if (family_unavailable) {
                    EXPECT_EQ(code_status, 0)
                        << "site=" << site.name
                        << " unavailable family must remain unavailable to RTKLIB; signal=" << signal_name;
                }
            }

            if (gps_l1c_developmental && doppler_valid) {
                // GPS L1C currently carries no CNAV-2 navigation data. Code
                // bias therefore remains unavailable. Doppler needs no
                // observable-specific code bias, so validate against a
                // same-satellite generic orbit/clock state. The generic EPH
                // health bit is not an L1C/CNAV-2 health observation, so use
                // the diagnostic API to ignore broadcast health only; navsys
                // and explicit exsats exclusions remain enforced by RTKLIB.
                double doppler_residual_mps = 0.0;
                const int doppler_status = rtklib_resdop_signal_diagnostic_ext(
                    &observation, nav.get(), &residual_options, receiver_position_m, receiver_velocity_mps, 0.0, 0,
                    wavelength_m, &doppler_residual_mps, nullptr);
                ASSERT_EQ(doppler_status, 1) << "generic diagnostic L1C Doppler residual failed; site=" << site.name
                                             << " sat=" << static_cast<int>(observation.sat);
                ++signal_stats.doppler_residuals;
                signal_stats.max_abs_doppler_mps =
                    (std::max)(signal_stats.max_abs_doppler_mps, std::fabs(doppler_residual_mps));
            } else if ((signal_name == "GPS L2C" || gps_l5_preoperational) && doppler_valid) {
                // GPS modern Doppler uses the same-satellite generic orbit/clock
                // state and does not consume a code bias.  The health attached
                // to whichever generic ephemeris wins selection is not a
                // reliable per-signal L2C/L5Q health observation, so ignore
                // broadcast health for this diagnostic residual only.
                double doppler_residual_mps = 0.0;
                const int doppler_status = rtklib_resdop_signal_diagnostic_ext(
                    &observation, nav.get(), &residual_options, receiver_position_m, receiver_velocity_mps, 0.0, 0,
                    wavelength_m, &doppler_residual_mps, nullptr);
                ASSERT_EQ(doppler_status, 1) << "diagnostic L5Q Doppler residual failed; site=" << site.name
                                             << " sat=" << static_cast<int>(observation.sat);
                ++signal_stats.doppler_residuals;
                signal_stats.max_abs_doppler_mps =
                    (std::max)(signal_stats.max_abs_doppler_mps, std::fabs(doppler_residual_mps));
            } else if (doppler_valid) {
                double doppler_residual_mps = 0.0;
                const int doppler_status = rtklib_resdop_signal_ext(&observation, nav.get(), &residual_options,
                                                                    receiver_position_m, receiver_velocity_mps, 0.0, 0,
                                                                    wavelength_m, &doppler_residual_mps, nullptr);
                ASSERT_EQ(doppler_status, 1) << "site=" << site.name << " signal=" << signal_name
                                             << " sat=" << static_cast<int>(observation.sat);
                ++signal_stats.doppler_residuals;
                signal_stats.max_abs_doppler_mps =
                    (std::max)(signal_stats.max_abs_doppler_mps, std::fabs(doppler_residual_mps));
            }
        }
    }

    // Real GPS CNAV-2 companion, provenance-fixed from Orekit commit
    // c5b14ff008eb02482ad4e2a2347c97ce4800969c, which records that the
    // G04 block was extracted from BRD400DLR_S_20222780000_01D_MN.rnx.
    // The broadcast health is 1, so strict positioning use must reject it;
    // diagnostic residual validation may ignore health while preserving the
    // real CNAV-2 orbit/clock and L1C ISC/code-bias model.
    {
        auto cnv2_nav = std::make_unique<nav_t>();
        ASSERT_TRUE(load_nav(gps_cnv2_nav_path(), cnv2_nav.get()));

        gnss_sim::SimConfig config = base_config;
        config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
        // RINEX4 CNV2 carries a distinct orbital toes field.  For this record
        // eph.toe resolves to 275400 s, while eph.toes is 241200 s and is the
        // value used by RTKLIB's Earth-rotation term.  The corresponding G04
        // sub-satellite longitude is about 106.84 E at this epoch.
        config.receiver = {7.04, 106.84, 100.0};
        config.duration_ns = 60LL * gnss_sim::NANOSECONDS_PER_SECOND;
        gnss_sim::SimTime cnv2_start{};
        ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2230, 275400.0, &cnv2_start));

        const std::filesystem::path cnv2_directory = directory / "gps_cnv2_real";
        ASSERT_TRUE(std::filesystem::create_directories(cnv2_directory, filesystem_error));
        ASSERT_FALSE(filesystem_error);
        const std::filesystem::path cnv2_output_path = cnv2_directory / "simulated.log";
        const std::string cnv2_output_text = cnv2_output_path.string();
        const std::string cnv2_nav_text = gps_cnv2_nav_path();
        const gnss_sim::SimulatorRunOptions cnv2_options{cnv2_nav_text.c_str(), cnv2_output_text.c_str(), cnv2_start};
        gnss_sim::SimulatorRunSummary cnv2_summary{};
        std::string cnv2_error_message;
        ASSERT_TRUE(gnss_sim::run_simulator(config, cnv2_options, &cnv2_summary, &cnv2_error_message))
            << cnv2_error_message;

        prcopt_t cnv2_residual_options = residual_options;
        cnv2_residual_options.ionoopt = IONOOPT_OFF;
        cnv2_residual_options.tropopt = TROPOPT_OFF;

        std::ifstream input(cnv2_directory / "observation_truth.csv");
        ASSERT_TRUE(input.good());
        std::string line;
        ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
        const std::map<std::string, std::size_t> column = header_columns(line);
        std::uint64_t l1c_diagnostic_rows = 0;
        while (std::getline(input, line)) {
            if (line.empty())
                continue;
            const std::vector<std::string> fields = split_csv(line);
            ASSERT_EQ(fields.size(), column.size());
            if (fields[column.at("signal_name")] != "GPS L1C")
                continue;

            seen_signals.insert("GPS L1C");
            SignalResidualStats& signal_stats = stats["GPS L1C"];
            ++signal_stats.rows;

            const gnss_sim::SignalDefinition* definition =
                gnss_sim::find_signal_definition(gnss_sim::SignalId::kGpsL1C);
            ASSERT_NE(definition, nullptr);
            int observation_code = 0;
            int frequency_index = 0;
            ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*definition, &observation_code, &frequency_index));
            static_cast<void>(frequency_index);

            ASSERT_EQ(fields[column.at("broadcast_message_family")], "CNAV2");
            ASSERT_EQ(fields[column.at("code_bias_status")], "APPLIED");
            if (fields[column.at("pseudorange_valid")] != "1") {
                ++signal_stats.code_unavailable;
                continue;
            }

            obsd_t observation{};
            observation.time =
                gpst2time(std::stoi(fields[column.at("gps_week")]), std::stod(fields[column.at("sow_sec")]));
            observation.sat = static_cast<unsigned char>(std::stoi(fields[column.at("satellite_number")]));
            observation.rcv = 1;
            observation.code[0] = static_cast<unsigned char>(observation_code);
            observation.SNR[0] = static_cast<unsigned char>(
                std::clamp(std::lround(std::stod(fields[column.at("cn0_dbhz")]) * 4.0), 0L, 255L));
            observation.P[0] = std::stod(fields[column.at("pseudorange_m")]);

            const double receiver_position_m[3] = {std::stod(fields[column.at("receiver_x_m")]),
                                                   std::stod(fields[column.at("receiver_y_m")]),
                                                   std::stod(fields[column.at("receiver_z_m")])};
            const double wavelength_m = std::stod(fields[column.at("wavelength_m")]);
            double code_residual_m = 0.0;
            rtklib_signal_bias_info_ext_t bias_info{};
            const int strict_status =
                rtklib_rescode_signal_ext(&observation, cnv2_nav.get(), &cnv2_residual_options, receiver_position_m,
                                          0.0, 0.0, NAV_CNV2, wavelength_m, &code_residual_m, nullptr, &bias_info);
            ASSERT_EQ(strict_status, 0) << "strict L1C residual must preserve CNV2 broadcast-health exclusion";

            const int diagnostic_status = rtklib_rescode_signal_diagnostic_ext(
                &observation, cnv2_nav.get(), &cnv2_residual_options, receiver_position_m, 0.0, 0.0, NAV_CNV2,
                wavelength_m, &code_residual_m, nullptr, &bias_info);
            ASSERT_EQ(diagnostic_status, 1)
                << "real G04 CNV2 diagnostic L1C code residual failed at sow=" << fields[column.at("sow_sec")];
            EXPECT_EQ(bias_info.message_type, NAV_CNV2);
            ++signal_stats.code_residuals;
            ++l1c_diagnostic_rows;
            signal_stats.max_abs_code_m = (std::max)(signal_stats.max_abs_code_m, std::fabs(code_residual_m));
        }
        ASSERT_GT(l1c_diagnostic_rows, 0U) << "real G04 CNV2 fixture must exercise GPS L1C code residual";
        std::fprintf(stderr,
                     "GPS_CNV2_L1C_COVERAGE diagnostic_rows=%llu max_abs_code=%.9f source=BRD400DLR_2022278_G04\n",
                     static_cast<unsigned long long>(l1c_diagnostic_rows), stats["GPS L1C"].max_abs_code_m);
        free_nav(cnv2_nav.get());
    }

    // Official JRC Galileo HAS E6 companion. The RINEX NAV path supplies only
    // the simulator's constellation/signal roster here; its 2025 ephemerides
    // are deliberately stale at this 2026 epoch. Galileo E6 must therefore be
    // generated from JRC HAS precise orbit/clock + C6C OSB, and the RTKLIB
    // explicit-state oracle validates the exact truth state written by the
    // simulator without reinterpreting HAS as INAV/FNAV broadcast data.
    {
        gnss_sim::SimConfig config = base_config;
        config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
        config.receiver = {-43.2162386, -15.4759141, 100.0};
        config.duration_ns = 60LL * gnss_sim::NANOSECONDS_PER_SECOND;
        gnss_sim::SimTime has_start{};
        ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2399, 346100.0, &has_start));

        const std::filesystem::path has_directory = directory / "galileo_e6_jrc_has";
        ASSERT_TRUE(std::filesystem::create_directories(has_directory, filesystem_error));
        ASSERT_FALSE(filesystem_error);
        const std::filesystem::path has_output_path = has_directory / "simulated.log";
        const std::string has_output_text = has_output_path.string();
        const std::string has_nav_text = brd4_nav_path();
        const std::string has_sp3_text = jrc_has_sp3_path();
        const std::string has_clock_text = jrc_has_clock_path();
        const std::string has_bias_text = jrc_has_bias_path();
        const gnss_sim::SimulatorRunOptions has_options{
            has_nav_text.c_str(),   has_output_text.c_str(), has_start, nullptr, has_sp3_text.c_str(),
            has_clock_text.c_str(), has_bias_text.c_str()};
        gnss_sim::SimulatorRunSummary has_summary{};
        std::string has_error_message;
        ASSERT_TRUE(gnss_sim::run_simulator(config, has_options, &has_summary, &has_error_message))
            << has_error_message;

        prcopt_t has_residual_options = residual_options;
        has_residual_options.ionoopt = IONOOPT_OFF;
        has_residual_options.tropopt = TROPOPT_OFF;

        std::ifstream input(has_directory / "observation_truth.csv");
        ASSERT_TRUE(input.good());
        std::string line;
        ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
        const std::map<std::string, std::size_t> column = header_columns(line);
        std::uint64_t has_e6_rows = 0;
        while (std::getline(input, line)) {
            if (line.empty())
                continue;
            const std::vector<std::string> fields = split_csv(line);
            ASSERT_EQ(fields.size(), column.size());
            if (fields[column.at("signal_name")] != "Galileo E6")
                continue;

            seen_signals.insert("Galileo E6");
            SignalResidualStats& signal_stats = stats["Galileo E6"];
            ++signal_stats.rows;
            ASSERT_EQ(fields[column.at("broadcast_message_family")], "UNKNOWN");
            ASSERT_EQ(fields[column.at("code_bias_status")], "APPLIED");
            ASSERT_EQ(fields[column.at("pseudorange_valid")], "1");
            ASSERT_EQ(fields[column.at("doppler_valid")], "1");

            const gnss_sim::SignalDefinition* definition =
                gnss_sim::find_signal_definition(gnss_sim::SignalId::kGalileoE6);
            ASSERT_NE(definition, nullptr);
            int observation_code = 0;
            int frequency_index = 0;
            ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*definition, &observation_code, &frequency_index));
            static_cast<void>(frequency_index);

            obsd_t observation{};
            observation.time =
                gpst2time(std::stoi(fields[column.at("gps_week")]), std::stod(fields[column.at("sow_sec")]));
            observation.sat = static_cast<unsigned char>(std::stoi(fields[column.at("satellite_number")]));
            observation.rcv = 1;
            observation.code[0] = static_cast<unsigned char>(observation_code);
            observation.SNR[0] = static_cast<unsigned char>(
                std::clamp(std::lround(std::stod(fields[column.at("cn0_dbhz")]) * 4.0), 0L, 255L));
            observation.P[0] = std::stod(fields[column.at("pseudorange_m")]);
            observation.D[0] = static_cast<float>(std::stod(fields[column.at("doppler_hz")]));

            char satellite_id[16]{};
            satno2id(observation.sat, satellite_id);
            ASSERT_STREQ(satellite_id, "E02");

            const double receiver_position_m[3] = {std::stod(fields[column.at("receiver_x_m")]),
                                                   std::stod(fields[column.at("receiver_y_m")]),
                                                   std::stod(fields[column.at("receiver_z_m")])};
            const double receiver_velocity_mps[3] = {std::stod(fields[column.at("receiver_vx_mps")]),
                                                     std::stod(fields[column.at("receiver_vy_mps")]),
                                                     std::stod(fields[column.at("receiver_vz_mps")])};
            const double satellite_state[6] = {
                std::stod(fields[column.at("satellite_x_m")]),    std::stod(fields[column.at("satellite_y_m")]),
                std::stod(fields[column.at("satellite_z_m")]),    std::stod(fields[column.at("satellite_vx_mps")]),
                std::stod(fields[column.at("satellite_vy_mps")]), std::stod(fields[column.at("satellite_vz_mps")])};
            const double satellite_clock[2] = {std::stod(fields[column.at("satellite_clock_bias_m")]) / CLIGHT,
                                               std::stod(fields[column.at("satellite_clock_drift_mps")]) / CLIGHT};
            const double wavelength_m = std::stod(fields[column.at("wavelength_m")]);
            const double code_bias_m = std::stod(fields[column.at("code_bias_m")]);

            double code_residual_m = 0.0;
            const int code_status = rtklib_rescode_state_ext(
                &observation, nav.get(), &has_residual_options, receiver_position_m, 0.0, 0.0, satellite_state,
                satellite_clock, 0, code_bias_m, wavelength_m, &code_residual_m, nullptr);
            ASSERT_EQ(code_status, 1) << "JRC HAS E6 code residual failed at sow=" << fields[column.at("sow_sec")];
            ++signal_stats.code_residuals;
            signal_stats.max_abs_code_m = (std::max)(signal_stats.max_abs_code_m, std::fabs(code_residual_m));

            double doppler_residual_mps = 0.0;
            const int doppler_status = rtklib_resdop_state_ext(
                &observation, &has_residual_options, receiver_position_m, receiver_velocity_mps, 0.0, satellite_state,
                satellite_clock, 0, wavelength_m, &doppler_residual_mps, nullptr);
            ASSERT_EQ(doppler_status, 1) << "JRC HAS E6 Doppler residual failed at sow="
                                         << fields[column.at("sow_sec")];
            ++signal_stats.doppler_residuals;
            signal_stats.max_abs_doppler_mps =
                (std::max)(signal_stats.max_abs_doppler_mps, std::fabs(doppler_residual_mps));
            ++has_e6_rows;
        }
        ASSERT_GT(has_e6_rows, 0U) << "official JRC HAS fixture must exercise Galileo E6 residuals";
        std::fprintf(
            stderr,
            "GALILEO_HAS_E6_COVERAGE rows=%llu max_abs_code=%.9f max_abs_doppler=%.9f source=JRC_HAS_2026001_E02\n",
            static_cast<unsigned long long>(has_e6_rows), stats["Galileo E6"].max_abs_code_m,
            stats["Galileo E6"].max_abs_doppler_mps);
    }

    std::size_t definition_count = 0;
    const gnss_sim::SignalDefinition* definitions = gnss_sim::signal_definitions(&definition_count);
    ASSERT_NE(definitions, nullptr);
    ASSERT_EQ(definition_count, 21U);
    ASSERT_EQ(seen_signals.size(), definition_count);

    std::size_t code_covered_signal_count = 0;
    for (std::size_t index = 0; index < definition_count; ++index) {
        const std::string signal_name = definitions[index].name;
        ASSERT_EQ(seen_signals.count(signal_name), 1U) << signal_name;
        const SignalResidualStats& signal_stats = stats.at(signal_name);
        EXPECT_GT(signal_stats.rows, 0U) << signal_name;
        EXPECT_GT(signal_stats.doppler_residuals, 0U) << "every V1 frequency must execute resdop: " << signal_name;
        EXPECT_LT(signal_stats.max_abs_doppler_mps, 0.002)
            << "Doppler residual exceeds the RANGEA 0.001-Hz serialization floor: " << signal_name;
        EXPECT_GT(signal_stats.code_residuals + signal_stats.code_unavailable, 0U)
            << "every V1 frequency must execute a code-residual/bias availability check: " << signal_name;
        if (signal_stats.code_residuals > 0U) {
            ++code_covered_signal_count;
            EXPECT_LT(signal_stats.max_abs_code_m, 0.02)
                << "code residual exceeds the RANGEA millimetre serialization floor: " << signal_name;
        }
        std::fprintf(stderr, "CODE_COVERAGE signal=%s residual_rows=%llu unavailable_rows=%llu max_abs_code=%.9f\n",
                     signal_name.c_str(), static_cast<unsigned long long>(signal_stats.code_residuals),
                     static_cast<unsigned long long>(signal_stats.code_unavailable), signal_stats.max_abs_code_m);
        if (signal_name == "Galileo E6") {
            EXPECT_GT(signal_stats.code_residuals, 0U)
                << "official JRC HAS products must exercise Galileo E6 code residuals";
            EXPECT_GT(signal_stats.code_unavailable, 0U)
                << "the compact BRD400 fixture must still expose missing HAS explicitly";
        } else if (signal_name == "GPS L1C") {
            EXPECT_GT(signal_stats.code_residuals, 0U)
                << "provenance-fixed real G04 CNV2 must exercise L1C diagnostic code residual";
            EXPECT_GT(signal_stats.code_unavailable, 0U)
                << "the 2025 compact BRD4 fixture must still record its missing CNV2 family explicitly";
        } else {
            EXPECT_GT(signal_stats.code_residuals, 0U)
                << "real compact coverage union must exercise every available non-E6 signal: " << signal_name;
        }
    }
    std::fprintf(stderr, "CODE_COVERAGE_UNION covered=%zu total=21\n", code_covered_signal_count);
    EXPECT_EQ(code_covered_signal_count, 21U)
        << "official JRC HAS E6 companion must complete all 21 V1 code-residual paths";

    free_nav(nav.get());
    std::filesystem::remove_all(directory, filesystem_error);
}

} // namespace
