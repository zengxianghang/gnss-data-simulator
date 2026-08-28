#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"
#include "residual_validator.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <gtest/gtest.h>
#include <string>

namespace {

std::string data_path(const char* name) {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/" + name;
}

const gnss_sim::residual_validator::SummaryRow*
signal_summary(const gnss_sim::residual_validator::ValidationReport& report, gnss_sim::SignalId signal_id) {
    return gnss_sim::residual_validator::find_signal_summary(report, static_cast<int>(signal_id));
}

std::size_t signal_summary_count(const gnss_sim::residual_validator::ValidationReport& report) {
    std::size_t count = 0;
    for (const auto& row : report.rows) {
        if (row.scope == "signal") {
            ++count;
        }
    }
    return count;
}

const gnss_sim::residual_validator::SummaryRow*
family_summary(const gnss_sim::residual_validator::ValidationReport& report, gnss_sim::SignalId signal_id,
               const std::string& family) {
    const int expected_id = static_cast<int>(signal_id);
    for (const auto& row : report.rows) {
        if (row.scope == "signal_family" && row.signal_id == expected_id && row.family == family) {
            return &row;
        }
    }
    return nullptr;
}

void configure_zero_noise_ks(gnss_sim::SimConfig* config, gnss_sim::AtmosphereMode atmosphere_mode, double latitude_deg,
                             double longitude_deg) {
    ASSERT_NE(config, nullptr);
    *config = gnss_sim::default_sim_config();
    config->scenario = gnss_sim::ScenarioType::KS;
    config->atmosphere_mode = atmosphere_mode;
    config->receiver = {latitude_deg, longitude_deg, 100.0};
    config->elevation_mask_deg = 0.0;
    config->sampling_rate_hz = 1;
    config->duration_ns = 60LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config->measurement_noise_enabled = false;
    config->multipath_enabled = false;
    config->receiver_clock_bias_m = 0.0;
    config->receiver_clock_drift_mps = 0.0;
    config->seed = 0x51U;
}

TEST(ResidualValidatorIntegration, CompactBroadcastTruthUsesSharedRtklibEvaluator) {
    const std::filesystem::path directory = "gnss_sim_shared_residual_broadcast";
    std::error_code filesystem_error;
    std::filesystem::remove_all(directory, filesystem_error);
    filesystem_error.clear();
    ASSERT_TRUE(std::filesystem::create_directories(directory, filesystem_error));
    ASSERT_FALSE(filesystem_error);

    gnss_sim::SimConfig config{};
    configure_zero_noise_ks(&config, gnss_sim::AtmosphereMode::BROADCAST, 20.0, 120.0);

    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 436500.0, &start));

    const std::string nav_path = data_path("brd400dlr_rinex4_acceptance_nav.rnx");
    const std::string output_path = (directory / "simulated.log").string();
    gnss_sim::SimulatorRunOptions run_options{};
    run_options.rinex_nav_path = nav_path.c_str();
    run_options.output_log_path = output_path.c_str();
    run_options.start_time = start;

    gnss_sim::SimulatorRunSummary run_summary{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::run_simulator(config, run_options, &run_summary, &error_message)) << error_message;

    gnss_sim::residual_validator::ValidationOptions validation_options{};
    validation_options.nav_path = nav_path;
    validation_options.observation_truth_path = (directory / "observation_truth.csv").string();
    validation_options.atmosphere_mode = gnss_sim::residual_validator::AtmosphereMode::kBroadcast;

    gnss_sim::residual_validator::ValidationReport report{};
    ASSERT_TRUE(gnss_sim::residual_validator::validate_observation_truth(validation_options, &report, &error_message))
        << error_message;
    EXPECT_GT(report.input_rows, 0U);
    EXPECT_EQ(signal_summary_count(report), 21U);

    for (gnss_sim::SignalId signal_id :
         {gnss_sim::SignalId::kGpsL1Ca, gnss_sim::SignalId::kQzssL1Ca, gnss_sim::SignalId::kGlonassG1,
          gnss_sim::SignalId::kGalileoE1, gnss_sim::SignalId::kBeidouB1I}) {
        const auto* summary = signal_summary(report, signal_id);
        ASSERT_NE(summary, nullptr);
        EXPECT_GT(summary->code_residuals, 0U);
        EXPECT_GT(summary->doppler_residuals, 0U);
        EXPECT_LT(summary->code_max_abs_m, 0.02);
        EXPECT_LT(summary->doppler_max_abs_mps, 0.002);
    }

    const auto* e6 = signal_summary(report, gnss_sim::SignalId::kGalileoE6);
    ASSERT_NE(e6, nullptr);
    EXPECT_STREQ(e6->rinex_code.c_str(), "6C");
    EXPECT_EQ(e6->oem7_signal_type, 7);
    EXPECT_GT(e6->code_unavailable, 0U);

    std::filesystem::remove_all(directory, filesystem_error);
}

TEST(ResidualValidatorIntegration, GalileoHasE6UsesSharedExplicitStateEvaluator) {
    const std::filesystem::path directory = "gnss_sim_shared_residual_has_e6";
    std::error_code filesystem_error;
    std::filesystem::remove_all(directory, filesystem_error);
    filesystem_error.clear();
    ASSERT_TRUE(std::filesystem::create_directories(directory, filesystem_error));
    ASSERT_FALSE(filesystem_error);

    gnss_sim::SimConfig config{};
    configure_zero_noise_ks(&config, gnss_sim::AtmosphereMode::NONE, -43.2162386, -15.4759141);

    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2399, 346100.0, &start));

    const std::string nav_path = data_path("brd400dlr_rinex4_acceptance_nav.rnx");
    const std::string sp3_path = data_path("jrc_has_2026001_e02.sp3");
    const std::string clock_path = data_path("jrc_has_2026001_e02.clk");
    const std::string bias_path = data_path("jrc_has_2026001_e02_c6c.bia");
    const std::string output_path = (directory / "simulated.log").string();

    gnss_sim::SimulatorRunOptions run_options{};
    run_options.rinex_nav_path = nav_path.c_str();
    run_options.output_log_path = output_path.c_str();
    run_options.start_time = start;
    run_options.galileo_has_sp3_path = sp3_path.c_str();
    run_options.galileo_has_clock_path = clock_path.c_str();
    run_options.galileo_has_bias_path = bias_path.c_str();

    gnss_sim::SimulatorRunSummary run_summary{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::run_simulator(config, run_options, &run_summary, &error_message)) << error_message;

    gnss_sim::residual_validator::ValidationOptions validation_options{};
    validation_options.nav_path = nav_path;
    validation_options.observation_truth_path = (directory / "observation_truth.csv").string();
    validation_options.atmosphere_mode = gnss_sim::residual_validator::AtmosphereMode::kNone;

    gnss_sim::residual_validator::ValidationReport report{};
    ASSERT_TRUE(gnss_sim::residual_validator::validate_observation_truth(validation_options, &report, &error_message))
        << error_message;

    const auto* e6 = signal_summary(report, gnss_sim::SignalId::kGalileoE6);
    ASSERT_NE(e6, nullptr);
    EXPECT_EQ(e6->rinex_code, "6C");
    EXPECT_EQ(e6->oem7_signal_type, 7);
    EXPECT_GT(e6->code_residuals, 0U);
    EXPECT_GT(e6->doppler_residuals, 0U);
    EXPECT_LT(e6->code_max_abs_m, 0.02);
    EXPECT_LT(e6->doppler_max_abs_mps, 0.002);

    const auto* has_family = family_summary(report, gnss_sim::SignalId::kGalileoE6, "HAS_PRECISE");
    ASSERT_NE(has_family, nullptr);
    EXPECT_EQ(has_family->code_residuals, e6->code_residuals);
    EXPECT_EQ(has_family->doppler_residuals, e6->doppler_residuals);

    const std::string summary_path = (directory / "residual_summary.csv").string();
    ASSERT_TRUE(gnss_sim::residual_validator::write_summary_csv(report, summary_path, &error_message)) << error_message;
    EXPECT_TRUE(std::filesystem::exists(summary_path));

    std::filesystem::remove_all(directory, filesystem_error);
}

} // namespace
