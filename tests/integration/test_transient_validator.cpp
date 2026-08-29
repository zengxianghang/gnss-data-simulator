#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"
#include "transient_validator.h"

#include <filesystem>
#include <gtest/gtest.h>
#include <string>

namespace {

std::string brd4_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

gnss_sim::SimTime start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 436500.0, &time));
    return time;
}

bool run_case(const std::filesystem::path& directory, const gnss_sim::SimConfig& config,
              gnss_sim::TransientValidationSummary* validation, std::string* error_message) {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    error.clear();
    if (!std::filesystem::create_directories(directory, error) || error) {
        if (error_message != nullptr) {
            *error_message = "cannot create transient-validator integration directory";
        }
        return false;
    }

    const std::string nav = brd4_nav_path();
    const std::string log = (directory / "simulated.log").string();
    const gnss_sim::SimulatorRunOptions run_options{nav.c_str(), log.c_str(), start_time()};
    gnss_sim::SimulatorRunSummary run_summary{};
    if (!gnss_sim::run_simulator(config, run_options, &run_summary, error_message)) {
        return false;
    }

    const std::string truth = (directory / "observation_truth.csv").string();
    const std::string events = (directory / "event_truth.csv").string();
    const char* scenario_label = config.scenario == gnss_sim::ScenarioType::TTFF ? "TTFF_HOT" : "REA_FADE";
    const gnss_sim::TransientValidationOptions options{
        scenario_label,
        config.measurement_error.rea_fade.duration_sec,
        config.receiver.latitude_deg,
        config.receiver.longitude_deg,
        config.receiver.height_m,
        config.solution_elevation_mask_deg,
        config.atmosphere_mode == gnss_sim::AtmosphereMode::BROADCAST,
    };
    return gnss_sim::validate_transient_observations_files(log.c_str(), truth.c_str(), events.c_str(), nav.c_str(),
                                                           options, validation, error_message);
}

void cleanup(const std::filesystem::path& directory) {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
}

void expect_same_first_valid_timing(const gnss_sim::FirstValidTimingStatistics& lhs,
                                    const gnss_sim::FirstValidTimingStatistics& rhs) {
    EXPECT_DOUBLE_EQ(lhs.pseudorange_delay_sec, rhs.pseudorange_delay_sec);
    EXPECT_DOUBLE_EQ(lhs.doppler_delay_sec, rhs.doppler_delay_sec);
    EXPECT_DOUBLE_EQ(lhs.adr_delay_sec, rhs.adr_delay_sec);
    EXPECT_DOUBLE_EQ(lhs.cn0_delay_sec, rhs.cn0_delay_sec);
}

gnss_sim::SimConfig ttff_hot_config() {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::TTFF;
    config.ttff.startup_mode = gnss_sim::StartupMode::HOT;
    config.ttff.power_on_ns = 20LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.ttff.power_off_ns = 2LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.duration_ns = 12LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.sampling_rate_hz = 10;
    config.elevation_mask_deg = 3.0;
    config.solution_elevation_mask_deg = 5.0;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::BROADCAST;
    config.measurement_noise_enabled = true;
    config.seed = 7401U;
    return config;
}

gnss_sim::SimConfig rea_fade_config() {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::REA;
    config.rea.signal_on_ns = 5LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_off_ns = 2LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.duration_ns = 18LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.sampling_rate_hz = 10;
    config.elevation_mask_deg = 3.0;
    config.solution_elevation_mask_deg = 5.0;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::BROADCAST;
    config.measurement_noise_enabled = true;
    config.measurement_error.rea_fade.duration_sec = 0.25;
    config.seed = 7402U;
    return config;
}

TEST(TransientValidatorIntegration, RealWhuTtffHotShowsTransientDecayAndPositionsSerializedRangea) {
    const std::filesystem::path directory = "gnss_sim_transient_real_whu_ttff_hot";
    gnss_sim::TransientValidationSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run_case(directory, ttff_hot_config(), &summary, &error_message)) << error_message;

    EXPECT_GT(summary.range_epochs, 0U);
    EXPECT_GT(summary.matched_observations, 0U);
    EXPECT_EQ(summary.unmatched_observations, 0U);
    EXPECT_GE(summary.first_valid.pseudorange_delay_sec, 0.0);
    EXPECT_GE(summary.first_valid.doppler_delay_sec, 0.0);
    EXPECT_GE(summary.first_valid.adr_delay_sec, 0.0);
    EXPECT_GE(summary.first_valid.cn0_delay_sec, 0.0);
    EXPECT_LT(summary.first_valid.pseudorange_delay_sec, 8.0);
    EXPECT_LT(summary.first_valid.doppler_delay_sec, 8.0);
    EXPECT_LT(summary.first_valid.adr_delay_sec, 8.0);
    ASSERT_GT(summary.early.pseudorange_m.sample_count, 20U);
    ASSERT_GT(summary.recovery.pseudorange_m.sample_count, 20U);
    ASSERT_GT(summary.settled.pseudorange_m.sample_count, 20U);
    EXPECT_GT(summary.early.pseudorange_m.rms, summary.recovery.pseudorange_m.rms);
    EXPECT_GT(summary.recovery.pseudorange_m.rms, summary.settled.pseudorange_m.rms);
    ASSERT_GT(summary.early.doppler_mps.sample_count, 20U);
    ASSERT_GT(summary.recovery.doppler_mps.sample_count, 20U);
    ASSERT_GT(summary.settled.doppler_mps.sample_count, 20U);
    EXPECT_GT(summary.early.doppler_mps.rms, summary.recovery.doppler_mps.rms);
    EXPECT_GT(summary.recovery.doppler_mps.rms, summary.settled.doppler_mps.rms);
    ASSERT_GT(summary.early.cn0_dbhz.sample_count, 20U);
    ASSERT_GT(summary.recovery.cn0_dbhz.sample_count, 20U);
    ASSERT_GT(summary.settled.cn0_dbhz.sample_count, 20U);
    EXPECT_GT(summary.early.cn0_dbhz.rms, summary.recovery.cn0_dbhz.rms);
    EXPECT_GT(summary.recovery.cn0_dbhz.rms, summary.settled.cn0_dbhz.rms);
    EXPECT_LT(summary.settled.pseudorange_m.rms, 0.30);
    EXPECT_LT(summary.settled.doppler_mps.rms, 0.15);
    EXPECT_LT(summary.settled.adr_m.rms, 0.01);
    EXPECT_GT(summary.positioning.valid_position_epochs, 0U);
    EXPECT_LT(summary.positioning.final_position_error_m, 5.0);

    cleanup(directory);
}

TEST(TransientValidatorIntegration, RealWhuReaFadeHasEmptyOffWindowsFreshAmbiguitiesAndRecovery) {
    const std::filesystem::path directory = "gnss_sim_transient_real_whu_rea_fade";
    gnss_sim::TransientValidationSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run_case(directory, rea_fade_config(), &summary, &error_message)) << error_message;

    EXPECT_GT(summary.matched_observations, 0U);
    EXPECT_EQ(summary.unmatched_observations, 0U);
    EXPECT_GT(summary.rea.signal_off_range_epochs, 0U);
    EXPECT_EQ(summary.rea.signal_off_nonzero_epochs, 0U);
    EXPECT_GE(summary.rea.reacquisition_cycles, 2U);
    EXPECT_LT(summary.rea.max_first_psr_delay_sec, 3.5);
    EXPECT_LT(summary.rea.max_first_doppler_delay_sec, 3.5);
    EXPECT_LT(summary.rea.max_first_adr_delay_sec, 4.0);
    EXPECT_GT(summary.rea.max_last_observation_to_signal_off_sec, 0.0);
    EXPECT_LE(summary.rea.max_last_observation_to_signal_off_sec, 0.2);
    EXPECT_GE(summary.rea.max_first_observation_after_signal_on_sec, 0.0);
    EXPECT_LT(summary.rea.max_first_observation_after_signal_on_sec, 3.5);
    ASSERT_GT(summary.rea.ambiguity_pairs_checked, 0U);
    EXPECT_EQ(summary.rea.ambiguity_pairs_changed, summary.rea.ambiguity_pairs_checked);

    ASSERT_GT(summary.fade.cn0_dbhz.sample_count, 0U);
    EXPECT_LT(summary.fade.cn0_dbhz.mean, -0.10);
    ASSERT_GT(summary.reacquisition_early.pseudorange_m.sample_count, 0U);
    ASSERT_GT(summary.reacquisition_settled.pseudorange_m.sample_count, 0U);
    EXPECT_GT(summary.reacquisition_early.pseudorange_m.rms, summary.reacquisition_settled.pseudorange_m.rms);
    EXPECT_GT(summary.positioning.valid_position_epochs, 0U);
    EXPECT_LT(summary.positioning.final_position_error_m, 5.0);

    cleanup(directory);
}

TEST(TransientValidatorIntegration, MeasurementNoiseToggleDoesNotMoveAcquisitionOrReacquisitionTiming) {
    const std::filesystem::path ttff_enabled_directory = "gnss_sim_transient_toggle_ttff_enabled";
    const std::filesystem::path ttff_disabled_directory = "gnss_sim_transient_toggle_ttff_disabled";
    gnss_sim::SimConfig ttff_enabled_config = ttff_hot_config();
    gnss_sim::SimConfig ttff_disabled_config = ttff_enabled_config;
    ttff_disabled_config.measurement_noise_enabled = false;
    gnss_sim::TransientValidationSummary ttff_enabled{};
    gnss_sim::TransientValidationSummary ttff_disabled{};
    std::string error_message;
    ASSERT_TRUE(run_case(ttff_enabled_directory, ttff_enabled_config, &ttff_enabled, &error_message)) << error_message;
    error_message.clear();
    ASSERT_TRUE(run_case(ttff_disabled_directory, ttff_disabled_config, &ttff_disabled, &error_message))
        << error_message;

    EXPECT_EQ(ttff_enabled.range_epochs, ttff_disabled.range_epochs);
    expect_same_first_valid_timing(ttff_enabled.first_valid, ttff_disabled.first_valid);

    const std::filesystem::path rea_enabled_directory = "gnss_sim_transient_toggle_rea_enabled";
    const std::filesystem::path rea_disabled_directory = "gnss_sim_transient_toggle_rea_disabled";
    gnss_sim::SimConfig rea_enabled_config = rea_fade_config();
    gnss_sim::SimConfig rea_disabled_config = rea_enabled_config;
    rea_disabled_config.measurement_noise_enabled = false;
    gnss_sim::TransientValidationSummary rea_enabled{};
    gnss_sim::TransientValidationSummary rea_disabled{};
    error_message.clear();
    ASSERT_TRUE(run_case(rea_enabled_directory, rea_enabled_config, &rea_enabled, &error_message)) << error_message;
    error_message.clear();
    ASSERT_TRUE(run_case(rea_disabled_directory, rea_disabled_config, &rea_disabled, &error_message)) << error_message;

    EXPECT_EQ(rea_enabled.range_epochs, rea_disabled.range_epochs);
    EXPECT_EQ(rea_enabled.rea.signal_off_range_epochs, rea_disabled.rea.signal_off_range_epochs);
    EXPECT_EQ(rea_enabled.rea.reacquisition_cycles, rea_disabled.rea.reacquisition_cycles);
    expect_same_first_valid_timing(rea_enabled.first_valid, rea_disabled.first_valid);
    EXPECT_DOUBLE_EQ(rea_enabled.rea.max_first_psr_delay_sec, rea_disabled.rea.max_first_psr_delay_sec);
    EXPECT_DOUBLE_EQ(rea_enabled.rea.max_first_doppler_delay_sec, rea_disabled.rea.max_first_doppler_delay_sec);
    EXPECT_DOUBLE_EQ(rea_enabled.rea.max_first_adr_delay_sec, rea_disabled.rea.max_first_adr_delay_sec);
    EXPECT_DOUBLE_EQ(rea_enabled.rea.max_last_observation_to_signal_off_sec,
                     rea_disabled.rea.max_last_observation_to_signal_off_sec);
    EXPECT_DOUBLE_EQ(rea_enabled.rea.max_first_observation_after_signal_on_sec,
                     rea_disabled.rea.max_first_observation_after_signal_on_sec);

    cleanup(ttff_enabled_directory);
    cleanup(ttff_disabled_directory);
    cleanup(rea_enabled_directory);
    cleanup(rea_disabled_directory);
}

} // namespace
