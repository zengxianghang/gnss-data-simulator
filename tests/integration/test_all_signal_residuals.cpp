#include "gnss/rtklib_adapter.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

#include <gtest/gtest.h>

extern "C" {
#include <rtklib.h>
#include <rtklib_residual_ext.h>
}

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
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

int required_rtklib_message_type(const gnss_sim::SignalDefinition& definition, const std::string& family) {
    if (family == "LEGACY") {
        if (definition.constellation == gnss_sim::GnssConstellation::kGps ||
            definition.constellation == gnss_sim::GnssConstellation::kQzss)
            return NAV_LNAV;
        if (definition.constellation == gnss_sim::GnssConstellation::kBeidou)
            return NAV_D1 | NAV_D2 | NAV_D1D2;
    }
    if (family == "CNAV")
        return NAV_CNAV;
    if (family == "CNAV2")
        return NAV_CNV2;
    if (family == "GALILEO_INAV")
        return NAV_INAV;
    if (family == "GALILEO_FNAV")
        return NAV_FNAV;
    if (family == "BEIDOU_BCNAV1")
        return NAV_CNV1;
    if (family == "BEIDOU_BCNAV2")
        return NAV_CNV2;
    if (family == "BEIDOU_BCNAV3")
        return NAV_CNV3;
    if (family == "GLONASS_FDMA")
        return NAV_FDMA;
    return 0;
}

TEST(V1Acceptance, EveryFrozenSignalRunsTruthStateCodeAndDopplerResidualChecks) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::KS;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::BROADCAST;
    config.elevation_mask_deg = 0.0;
    config.sampling_rate_hz = 1;
    config.duration_ns = 60LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.receiver = {20.0, 120.0, 100.0};
    config.measurement_noise_enabled = false;
    config.multipath_enabled = false;
    config.receiver_clock_bias_m = 0.0;
    config.receiver_clock_drift_mps = 0.0;
    config.seed = 0x51U;

    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 436500.0, &start));

    const std::filesystem::path directory = "gnss_sim_all_signal_residuals";
    std::error_code filesystem_error;
    std::filesystem::remove_all(directory, filesystem_error);
    filesystem_error.clear();
    ASSERT_TRUE(std::filesystem::create_directories(directory, filesystem_error));
    ASSERT_FALSE(filesystem_error);

    const std::filesystem::path output_path = directory / "simulated.log";
    const std::string output_text = output_path.string();
    const std::string nav_text = brd4_nav_path();
    const gnss_sim::SimulatorRunOptions options{nav_text.c_str(), output_text.c_str(), start};
    gnss_sim::SimulatorRunSummary run_summary{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::run_simulator(config, options, &run_summary, &error_message)) << error_message;

    nav_t nav{};
    ASSERT_TRUE(load_nav(nav_text, &nav));

    std::ifstream input(directory / "observation_truth.csv");
    ASSERT_TRUE(input.good());
    std::string line;
    ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
    const std::map<std::string, std::size_t> column = header_columns(line);
    for (const char* required :
         {"gps_week", "sow_sec", "signal_id", "signal_name", "satellite_number", "wavelength_m", "receiver_x_m",
          "receiver_y_m", "receiver_z_m", "receiver_vx_mps", "receiver_vy_mps", "receiver_vz_mps", "cn0_dbhz",
          "broadcast_message_family", "pseudorange_valid", "doppler_valid", "pseudorange_m", "doppler_hz"}) {
        ASSERT_EQ(column.count(required), 1U) << required;
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
    std::set<std::string> code_unavailable_signals;

    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
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

        const double wavelength_m = std::stod(fields[column.at("wavelength_m")]);
        const double receiver_position_m[3] = {std::stod(fields[column.at("receiver_x_m")]),
                                               std::stod(fields[column.at("receiver_y_m")]),
                                               std::stod(fields[column.at("receiver_z_m")])};
        const double receiver_velocity_mps[3] = {std::stod(fields[column.at("receiver_vx_mps")]),
                                                 std::stod(fields[column.at("receiver_vy_mps")]),
                                                 std::stod(fields[column.at("receiver_vz_mps")])};

        const std::string message_family = fields[column.at("broadcast_message_family")];
        const int required_message_type = required_rtklib_message_type(*definition, message_family);

        rtklib_signal_bias_info_ext_t bias_info{};
        double code_residual_m = 0.0;
        const int code_status =
            rtklib_rescode_signal_ext(&observation, &nav, &residual_options, receiver_position_m, 0.0, 0.0,
                                      required_message_type, wavelength_m, &code_residual_m, nullptr, &bias_info);
        if (fields[column.at("pseudorange_valid")] == "1") {
            ASSERT_EQ(code_status, 1) << "signal=" << signal_name << " sat=" << static_cast<int>(observation.sat);
            ++signal_stats.code_residuals;
            signal_stats.max_abs_code_m = (std::max)(signal_stats.max_abs_code_m, std::fabs(code_residual_m));
        } else {
            EXPECT_EQ(code_status, 0) << "invalid code must have an explicitly unavailable RTKLIB bias path; signal="
                                      << signal_name;
            ++signal_stats.code_unavailable;
            code_unavailable_signals.insert(signal_name);
        }

        if (fields[column.at("doppler_valid")] == "1") {
            double doppler_residual_mps = 0.0;
            const int doppler_status =
                rtklib_resdop_signal_ext(&observation, &nav, &residual_options, receiver_position_m,
                                         receiver_velocity_mps, 0.0, wavelength_m, &doppler_residual_mps, nullptr);
            ASSERT_EQ(doppler_status, 1) << "signal=" << signal_name << " sat=" << static_cast<int>(observation.sat);
            ++signal_stats.doppler_residuals;
            signal_stats.max_abs_doppler_mps =
                (std::max)(signal_stats.max_abs_doppler_mps, std::fabs(doppler_residual_mps));
        }
    }

    std::size_t definition_count = 0;
    const gnss_sim::SignalDefinition* definitions = gnss_sim::signal_definitions(&definition_count);
    ASSERT_NE(definitions, nullptr);
    ASSERT_EQ(definition_count, 21U);
    ASSERT_EQ(seen_signals.size(), definition_count);

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
            EXPECT_LT(signal_stats.max_abs_code_m, 0.02)
                << "code residual exceeds the RANGEA millimetre serialization floor: " << signal_name;
        }
    }

    EXPECT_EQ(code_unavailable_signals, (std::set<std::string>{"GLONASS G3", "Galileo E6"}));

    free_nav(&nav);
    std::filesystem::remove_all(directory, filesystem_error);
}

} // namespace
