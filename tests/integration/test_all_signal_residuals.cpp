#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"
#include "residual_validator.h"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <gtest/gtest.h>
#include <map>
#include <string>

namespace {

struct SignalResidualUnion {
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

void configure_zero_noise_ks(gnss_sim::SimConfig* config, gnss_sim::AtmosphereMode atmosphere_mode, double latitude_deg,
                             double longitude_deg, int duration_sec) {
    ASSERT_NE(config, nullptr);
    *config = gnss_sim::default_sim_config();
    config->scenario = gnss_sim::ScenarioType::KS;
    config->atmosphere_mode = atmosphere_mode;
    config->receiver = {latitude_deg, longitude_deg, 100.0};
    config->elevation_mask_deg = 0.0;
    config->sampling_rate_hz = 1;
    config->duration_ns = static_cast<std::int64_t>(duration_sec) * gnss_sim::NANOSECONDS_PER_SECOND;
    config->measurement_noise_enabled = false;
    config->multipath_enabled = false;
    config->receiver_clock_bias_m = 0.0;
    config->receiver_clock_drift_mps = 0.0;
    config->seed = 0x51U;
}

bool run_and_validate(const std::filesystem::path& directory, const gnss_sim::SimConfig& config,
                      const gnss_sim::SimTime& start, const std::string& nav_path,
                      gnss_sim::residual_validator::AtmosphereMode atmosphere_mode,
                      gnss_sim::residual_validator::ValidationReport* report, std::string* error_message,
                      const char* has_sp3_path = nullptr, const char* has_clock_path = nullptr,
                      const char* has_bias_path = nullptr) {
    std::error_code filesystem_error;
    if (!std::filesystem::create_directories(directory, filesystem_error) || filesystem_error) {
        if (!std::filesystem::exists(directory)) {
            if (error_message != nullptr) {
                *error_message = "cannot create compact residual directory: " + directory.string();
            }
            return false;
        }
    }

    const std::string output_path = (directory / "simulated.log").string();
    gnss_sim::SimulatorRunOptions run_options{};
    run_options.rinex_nav_path = nav_path.c_str();
    run_options.output_log_path = output_path.c_str();
    run_options.start_time = start;
    run_options.galileo_has_sp3_path = has_sp3_path;
    run_options.galileo_has_clock_path = has_clock_path;
    run_options.galileo_has_bias_path = has_bias_path;

    gnss_sim::SimulatorRunSummary run_summary{};
    if (!gnss_sim::run_simulator(config, run_options, &run_summary, error_message)) {
        return false;
    }

    gnss_sim::residual_validator::ValidationOptions validation_options{};
    validation_options.nav_path = nav_path;
    validation_options.observation_truth_path = (directory / "observation_truth.csv").string();
    validation_options.atmosphere_mode = atmosphere_mode;
    validation_options.allow_diagnostic_health = true;
    return gnss_sim::residual_validator::validate_observation_truth(validation_options, report, error_message);
}

void merge_signal_report(const gnss_sim::residual_validator::ValidationReport& report,
                         std::map<int, SignalResidualUnion>* aggregate) {
    ASSERT_NE(aggregate, nullptr);
    for (const auto& row : report.rows) {
        if (row.scope != "signal") {
            continue;
        }
        SignalResidualUnion& result = (*aggregate)[row.signal_id];
        result.rows += row.rows;
        result.code_residuals += row.code_residuals;
        result.code_unavailable += row.code_unavailable;
        result.doppler_residuals += row.doppler_residuals;
        result.max_abs_code_m = (std::max)(result.max_abs_code_m, row.code_max_abs_m);
        result.max_abs_doppler_mps = (std::max)(result.max_abs_doppler_mps, row.doppler_max_abs_mps);
    }
}

TEST(V1Acceptance, EveryFrozenSignalRunsTruthStateCodeAndDopplerResidualChecks) {
    const std::filesystem::path directory = "gnss_sim_all_signal_residuals";
    std::error_code filesystem_error;
    std::filesystem::remove_all(directory, filesystem_error);
    filesystem_error.clear();
    ASSERT_TRUE(std::filesystem::create_directories(directory, filesystem_error));
    ASSERT_FALSE(filesystem_error);

    std::map<int, SignalResidualUnion> aggregate;
    std::string error_message;

    struct BroadcastSite {
        const char* name;
        double latitude_deg;
        double longitude_deg;
        double start_sow_sec;
        int duration_sec;
    };
    const BroadcastSite sites[] = {
        {"asia", 20.0, 120.0, 436500.0, 60},
        {"gps_cnav", 36.272115, -19.973734, 437100.0, 120},
        {"bds_bcnav12", -47.507042, -174.038033, 436500.0, 60},
    };

    for (const BroadcastSite& site : sites) {
        gnss_sim::SimConfig config{};
        configure_zero_noise_ks(&config, gnss_sim::AtmosphereMode::BROADCAST, site.latitude_deg, site.longitude_deg,
                                site.duration_sec);
        gnss_sim::SimTime start{};
        ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2347, site.start_sow_sec, &start));

        gnss_sim::residual_validator::ValidationReport report{};
        const std::string nav_path = brd4_nav_path();
        ASSERT_TRUE(run_and_validate(directory / site.name, config, start, nav_path,
                                     gnss_sim::residual_validator::AtmosphereMode::kBroadcast, &report, &error_message))
            << "site=" << site.name << " " << error_message;
        merge_signal_report(report, &aggregate);
    }

    // Real GPS CNAV-2 companion. The G04 block is provenance-fixed from
    // BRD400DLR_S_20222780000_01D_MN.rnx and supplies real L1C ISC/code-bias
    // coverage. Its broadcast health is intentionally handled by the shared
    // validator diagnostic path without changing the generated measurement.
    {
        gnss_sim::SimConfig config{};
        configure_zero_noise_ks(&config, gnss_sim::AtmosphereMode::NONE, 7.04, 106.84, 60);
        gnss_sim::SimTime start{};
        ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2230, 275400.0, &start));

        gnss_sim::residual_validator::ValidationReport report{};
        const std::string nav_path = gps_cnv2_nav_path();
        ASSERT_TRUE(run_and_validate(directory / "gps_cnv2_real", config, start, nav_path,
                                     gnss_sim::residual_validator::AtmosphereMode::kNone, &report, &error_message))
            << error_message;
        merge_signal_report(report, &aggregate);
    }

    // Official JRC Galileo HAS E6-C companion. The 2025 broadcast fixture is
    // deliberately not used as an E6 state/bias source at this 2026 epoch;
    // simulator and shared validator consume one coherent SP3+CLK+C6C OSB
    // truth state through the RTKLIB explicit-state residual APIs.
    {
        gnss_sim::SimConfig config{};
        configure_zero_noise_ks(&config, gnss_sim::AtmosphereMode::NONE, -43.2162386, -15.4759141, 60);
        gnss_sim::SimTime start{};
        ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2399, 346100.0, &start));

        const std::string sp3_path = jrc_has_sp3_path();
        const std::string clock_path = jrc_has_clock_path();
        const std::string bias_path = jrc_has_bias_path();
        const std::string nav_path = brd4_nav_path();
        gnss_sim::residual_validator::ValidationReport report{};
        ASSERT_TRUE(run_and_validate(directory / "galileo_e6_jrc_has", config, start, nav_path,
                                     gnss_sim::residual_validator::AtmosphereMode::kNone, &report, &error_message,
                                     sp3_path.c_str(), clock_path.c_str(), bias_path.c_str()))
            << error_message;
        merge_signal_report(report, &aggregate);
    }

    std::size_t definition_count = 0;
    const gnss_sim::SignalDefinition* definitions = gnss_sim::signal_definitions(&definition_count);
    ASSERT_NE(definitions, nullptr);
    ASSERT_EQ(definition_count, 21U);

    std::size_t code_covered_signal_count = 0;
    std::size_t doppler_covered_signal_count = 0;
    for (std::size_t index = 0; index < definition_count; ++index) {
        const gnss_sim::SignalDefinition& definition = definitions[index];
        const int signal_id = static_cast<int>(definition.signal_id);
        ASSERT_EQ(aggregate.count(signal_id), 1U) << definition.name;
        const SignalResidualUnion& stats = aggregate.at(signal_id);

        EXPECT_GT(stats.rows, 0U) << definition.name;
        EXPECT_GT(stats.doppler_residuals, 0U)
            << "every V1 frequency must execute shared Doppler residual: " << definition.name;
        if (stats.doppler_residuals > 0U) {
            ++doppler_covered_signal_count;
        }
        EXPECT_LT(stats.max_abs_doppler_mps, 0.002)
            << "Doppler residual exceeds the RANGEA 0.001-Hz serialization floor: " << definition.name;

        EXPECT_GT(stats.code_residuals + stats.code_unavailable, 0U)
            << "every V1 frequency must have shared code-residual/bias availability evidence: " << definition.name;
        if (stats.code_residuals > 0U) {
            ++code_covered_signal_count;
            EXPECT_LT(stats.max_abs_code_m, 0.02)
                << "code residual exceeds the RANGEA millimetre serialization floor: " << definition.name;
        }

        std::fprintf(stderr,
                     "SHARED_RESIDUAL_COVERAGE signal=%s code_rows=%llu unavailable_rows=%llu doppler_rows=%llu "
                     "max_abs_code=%.9f max_abs_doppler=%.9f\n",
                     definition.name, static_cast<unsigned long long>(stats.code_residuals),
                     static_cast<unsigned long long>(stats.code_unavailable),
                     static_cast<unsigned long long>(stats.doppler_residuals), stats.max_abs_code_m,
                     stats.max_abs_doppler_mps);

        if (definition.signal_id == gnss_sim::SignalId::kGpsL1C) {
            EXPECT_GT(stats.code_residuals, 0U) << "real G04 CNAV2 companion must exercise GPS L1C code residual";
            EXPECT_GT(stats.code_unavailable, 0U)
                << "2025 compact broadcast coverage must retain missing CNAV2 code-bias evidence";
        } else if (definition.signal_id == gnss_sim::SignalId::kGalileoE6) {
            EXPECT_STREQ(definition.rinex_signal_code, "6C");
            EXPECT_EQ(definition.novatel_oem7_signal_type, 7);
            EXPECT_GT(stats.code_residuals, 0U)
                << "official JRC HAS companion must exercise Galileo E6-C code residual";
            EXPECT_GT(stats.code_unavailable, 0U)
                << "2025 compact broadcast coverage must retain missing HAS code-bias evidence";
        } else if (definition.signal_id == gnss_sim::SignalId::kGlonassG3) {
            EXPECT_EQ(stats.code_residuals, 0U)
                << "G3 code residual must stay unavailable until an authoritative real L3OC NAV record is present";
            EXPECT_GT(stats.code_unavailable, 0U)
                << "G3 must expose missing real L3OC code-bias coverage instead of synthesizing ephemeris";
        } else {
            EXPECT_GT(stats.code_residuals, 0U)
                << "compact coverage union must exercise code residual: " << definition.name;
        }
    }

    std::fprintf(stderr, "SHARED_CODE_COVERAGE_UNION covered=%zu total=%zu\n", code_covered_signal_count,
                 definition_count);
    std::fprintf(stderr, "SHARED_DOPPLER_COVERAGE_UNION covered=%zu total=%zu\n", doppler_covered_signal_count,
                 definition_count);
    // The authoritative BRD400DLR samples checked for Issue #66 contain no
    // GLONASS L3OC EPH record. G3 therefore contributes explicit
    // code_unavailable evidence while all 21 frequencies retain Doppler
    // residual coverage. Never synthesize an L3OC record to make this 21/21.
    EXPECT_EQ(code_covered_signal_count + 1U, definition_count);
    EXPECT_EQ(doppler_covered_signal_count, definition_count);

    std::filesystem::remove_all(directory, filesystem_error);
}

} // namespace
