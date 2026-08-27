#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <string>

namespace {

std::string nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/gps_loopback_nav.rnx";
}

gnss_sim::SimTime start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2253, 172900.0, &time));
    return time;
}

gnss_sim::SimConfig base_config(gnss_sim::ScenarioType scenario) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = scenario;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    config.elevation_mask_deg = 0.0;
    config.output_eph = true;
    config.output_ion = true;
    config.seed = 1U;
    return config;
}

std::string test_output_path(const char* name) {
    return std::string("gnss_sim_") + name + ".log";
}

bool run(const gnss_sim::SimConfig& config, const char* name, gnss_sim::SimulatorRunSummary* summary,
         std::string* error_message) {
    const std::string output_path = test_output_path(name);
    std::remove(output_path.c_str());
    const std::string input_path = nav_path();
    const gnss_sim::SimulatorRunOptions options{input_path.c_str(), output_path.c_str(), start_time()};
    const bool ok = gnss_sim::run_simulator(config, options, summary, error_message);
    if (!ok) {
        std::remove(output_path.c_str());
    }
    return ok;
}

bool file_contains(const std::string& path, const std::string& text) {
    std::ifstream input(path, std::ios::binary);
    const std::string contents((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    return contents.find(text) != std::string::npos;
}

TEST(StreamingSimulator, KsProducesOneLogSetPerEpoch) {
    gnss_sim::SimConfig config = base_config(gnss_sim::ScenarioType::KS);
    config.sampling_rate_hz = 5;
    config.duration_ns = 10LL * gnss_sim::NANOSECONDS_PER_SECOND;
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run(config, "ks", &summary, &error_message)) << error_message;
    EXPECT_EQ(summary.scheduled_epochs, 50U);
    EXPECT_EQ(summary.powered_epochs, 50U);
    EXPECT_EQ(summary.signal_on_epochs, 50U);
    EXPECT_EQ(summary.range_messages, 50U);
    EXPECT_EQ(summary.psrpos_messages, 50U);
    EXPECT_EQ(summary.psrvel_messages, 50U);
    EXPECT_EQ(summary.power_on_events, 1U);
    EXPECT_EQ(summary.power_off_events, 0U);
    EXPECT_GT(summary.max_observations_per_epoch, 0);
    EXPECT_GT(summary.valid_position_epochs, 0U);
    EXPECT_GT(summary.valid_velocity_epochs, 0U);
    std::remove(test_output_path("ks").c_str());
}

TEST(StreamingSimulator, ReaKeepsLogsRunningWithZeroRangeDuringSignalOff) {
    gnss_sim::SimConfig config = base_config(gnss_sim::ScenarioType::REA);
    config.sampling_rate_hz = 10;
    config.duration_ns = 6LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_on_ns = 2LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_off_ns = 1LL * gnss_sim::NANOSECONDS_PER_SECOND;
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run(config, "rea", &summary, &error_message)) << error_message;
    EXPECT_EQ(summary.scheduled_epochs, 60U);
    EXPECT_EQ(summary.powered_epochs, 60U);
    EXPECT_EQ(summary.signal_on_epochs, 40U);
    EXPECT_EQ(summary.signal_off_epochs, 20U);
    EXPECT_EQ(summary.range_messages, 60U);
    EXPECT_EQ(summary.psrpos_messages, 60U);
    EXPECT_EQ(summary.psrvel_messages, 60U);
    EXPECT_EQ(summary.power_off_events, 0U);
    EXPECT_EQ(summary.signal_off_events, 2U);
    EXPECT_EQ(summary.signal_on_events, 2U);
    EXPECT_TRUE(file_contains(test_output_path("rea"), ";0*"));
    EXPECT_TRUE(file_contains(test_output_path("rea"), "INSUFFICIENT_OBS,NONE"));
    std::remove(test_output_path("rea").c_str());
}

void expect_ttff_log_suppression(gnss_sim::StartupMode mode, const char* name) {
    gnss_sim::SimConfig config = base_config(gnss_sim::ScenarioType::TTFF);
    config.ttff.startup_mode = mode;
    config.sampling_rate_hz = 10;
    config.duration_ns = 8LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.ttff.power_on_ns = 3LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.ttff.power_off_ns = 1LL * gnss_sim::NANOSECONDS_PER_SECOND;
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run(config, name, &summary, &error_message)) << error_message;
    EXPECT_EQ(summary.scheduled_epochs, 80U);
    EXPECT_EQ(summary.powered_epochs, 60U);
    EXPECT_EQ(summary.range_messages, 60U);
    EXPECT_EQ(summary.psrpos_messages, 60U);
    EXPECT_EQ(summary.psrvel_messages, 60U);
    EXPECT_EQ(summary.power_on_events, 2U);
    EXPECT_EQ(summary.power_off_events, 2U);
    std::remove(test_output_path(name).c_str());
}

TEST(StreamingSimulator, HotTtffSuppressesAllReceiverLogsWhilePowerIsOff) {
    expect_ttff_log_suppression(gnss_sim::StartupMode::HOT, "ttff_hot");
}

TEST(StreamingSimulator, WarmTtffSuppressesAllReceiverLogsWhilePowerIsOff) {
    expect_ttff_log_suppression(gnss_sim::StartupMode::WARM, "ttff_warm");
}

TEST(StreamingSimulator, ColdTtffAcquiresEphemerisBeforeSolutionBecomesValid) {
    gnss_sim::SimConfig config = base_config(gnss_sim::ScenarioType::TTFF);
    config.ttff.startup_mode = gnss_sim::StartupMode::COLD;
    config.sampling_rate_hz = 1;
    config.duration_ns = 40LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.ttff.power_on_ns = 40LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.ttff.power_off_ns = 2LL * gnss_sim::NANOSECONDS_PER_SECOND;
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run(config, "ttff_cold", &summary, &error_message)) << error_message;
    EXPECT_EQ(summary.scheduled_epochs, 40U);
    EXPECT_EQ(summary.powered_epochs, 40U);
    EXPECT_GT(summary.nav_messages, 0U);
    EXPECT_GT(summary.valid_position_epochs, 0U);
    EXPECT_GT(summary.valid_velocity_epochs, 0U);
    EXPECT_TRUE(file_contains(test_output_path("ttff_cold"), "#GPSEPHEMA,"));
    std::remove(test_output_path("ttff_cold").c_str());
}

TEST(StreamingSimulator, FutureOnlyEphemerisDoesNotAbortEarlierEpochs) {
    gnss_sim::SimConfig config = base_config(gnss_sim::ScenarioType::KS);
    config.sampling_rate_hz = 1;
    config.duration_ns = 4LL * gnss_sim::NANOSECONDS_PER_SECOND;
    const std::string output_path = test_output_path("future_sat");
    std::remove(output_path.c_str());
    const std::string input_path = std::string(GNSS_SIM_TEST_DATA_DIR) + "/gps_loopback_nav_future_sat.rnx";
    const gnss_sim::SimulatorRunOptions options{input_path.c_str(), output_path.c_str(), start_time()};
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::run_simulator(config, options, &summary, &error_message)) << error_message;
    EXPECT_EQ(summary.scheduled_epochs, 4U);
    EXPECT_EQ(summary.range_messages, 4U);
    EXPECT_GT(summary.valid_position_epochs, 0U);
    std::remove(output_path.c_str());
}

TEST(StreamingSimulator, OneSecondDurationHasExactEpochCountAtEveryFrozenRate) {
    for (const int rate : {1, 5, 10, 20, 50}) {
        std::uint64_t count = 0;
        ASSERT_TRUE(gnss_sim::epoch_count_for_duration(gnss_sim::NANOSECONDS_PER_SECOND, rate, &count));
        EXPECT_EQ(count, static_cast<std::uint64_t>(rate));
    }
}

} // namespace
