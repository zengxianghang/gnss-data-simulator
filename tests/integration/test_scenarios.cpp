#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"
#include "output/device_marker.h"

#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <vector>

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

std::string read_file(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

bool file_contains(const std::string& path, const std::string& text) {
    return read_file(path).find(text) != std::string::npos;
}

std::vector<std::string> bestpos_states(const std::string& log) {
    std::vector<std::string> states;
    std::size_t position = 0;
    while ((position = log.find("#BESTPOSA,", position)) != std::string::npos) {
        const std::size_t semicolon = log.find(';', position);
        const std::size_t first_comma =
            semicolon == std::string::npos ? std::string::npos : log.find(',', semicolon + 1);
        const std::size_t second_comma =
            first_comma == std::string::npos ? std::string::npos : log.find(',', first_comma + 1);
        if (semicolon == std::string::npos || first_comma == std::string::npos || second_comma == std::string::npos) {
            break;
        }
        states.push_back(log.substr(semicolon + 1, second_comma - semicolon - 1));
        position = second_comma + 1;
    }
    return states;
}

bool has_fix_reset_refix(const std::vector<std::string>& states) {
    bool saw_fix = false;
    bool saw_reset = false;
    for (const std::string& state : states) {
        if (!saw_fix && state == "SOL_COMPUTED,NARROW_INT") {
            saw_fix = true;
        } else if (saw_fix && !saw_reset && state != "SOL_COMPUTED,NARROW_INT") {
            saw_reset = true;
        } else if (saw_fix && saw_reset && state == "SOL_COMPUTED,NARROW_INT") {
            return true;
        }
    }
    return false;
}

std::size_t occurrence_count(const std::string& text, const std::string& needle) {
    std::size_t count = 0;
    std::size_t position = 0;
    while ((position = text.find(needle, position)) != std::string::npos) {
        ++count;
        position += needle.size();
    }
    return count;
}

TEST(StreamingSimulator, KsProducesOneLogSetPerEpoch) {
    gnss_sim::SimConfig config = base_config(gnss_sim::ScenarioType::KS);
    config.sampling_rate_hz = 5;
    config.duration_ns = 10LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.bestpos_rtk.stable_duration_ns = 1LL * gnss_sim::NANOSECONDS_PER_SECOND;
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run(config, "ks", &summary, &error_message)) << error_message;
    EXPECT_EQ(summary.scheduled_epochs, 50U);
    EXPECT_EQ(summary.powered_epochs, 50U);
    EXPECT_EQ(summary.signal_on_epochs, 50U);
    EXPECT_EQ(summary.range_messages, 50U);
    EXPECT_EQ(summary.psrpos_messages, 50U);
    EXPECT_EQ(summary.psrvel_messages, 50U);
    EXPECT_EQ(summary.bestpos_messages, 50U);
    const std::string log = read_file(test_output_path("ks"));
    EXPECT_EQ(occurrence_count(log, "#BESTPOSA,"), 50U);
    EXPECT_GT(occurrence_count(log, ";SOL_COMPUTED,SINGLE,"), 0U);
    EXPECT_GT(occurrence_count(log, ";SOL_COMPUTED,NARROW_INT,"), 0U);
    EXPECT_EQ(occurrence_count(log, gnss_sim::simulator_device_marker()), 0U);
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
    EXPECT_EQ(summary.bestpos_messages, 60U);
    const std::string log = read_file(test_output_path("rea"));
    EXPECT_EQ(occurrence_count(log, "#BESTPOSA,"), 60U);
    EXPECT_EQ(occurrence_count(log, gnss_sim::simulator_device_marker()), 0U);
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
    EXPECT_EQ(summary.bestpos_messages, 60U);
    const std::string log = read_file(test_output_path(name));
    EXPECT_EQ(occurrence_count(log, "#BESTPOSA,"), 60U);
    EXPECT_EQ(summary.power_on_events, 2U);
    EXPECT_EQ(summary.power_off_events, 2U);
    const std::string& marker = gnss_sim::simulator_device_marker();
    EXPECT_EQ(occurrence_count(log, marker), summary.power_on_events);
    std::size_t marker_position = 0;
    for (std::uint64_t index = 0; index < summary.power_on_events; ++index) {
        marker_position = log.find(marker, marker_position);
        ASSERT_NE(marker_position, std::string::npos);
        const std::size_t next_record = marker_position + marker.size();
        ASSERT_LT(next_record, log.size());
        EXPECT_EQ(log[next_record], '#');
        marker_position = next_record;
    }
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

TEST(StreamingSimulator, ReaBestPosFixResetsOnSignalLossAndRefixesAfterStability) {
    gnss_sim::SimConfig config = base_config(gnss_sim::ScenarioType::REA);
    config.sampling_rate_hz = 10;
    config.duration_ns = 18LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_on_ns = 7LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_off_ns = 2LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.bestpos_rtk.stable_duration_ns = 1LL * gnss_sim::NANOSECONDS_PER_SECOND;
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run(config, "rea_bestpos_rtk", &summary, &error_message)) << error_message;
    const std::vector<std::string> states = bestpos_states(read_file(test_output_path("rea_bestpos_rtk")));
    EXPECT_TRUE(has_fix_reset_refix(states));
    std::remove(test_output_path("rea_bestpos_rtk").c_str());
}

TEST(StreamingSimulator, TtffBestPosFixResetsAcrossPowerCycleAndRefixes) {
    gnss_sim::SimConfig config = base_config(gnss_sim::ScenarioType::TTFF);
    config.ttff.startup_mode = gnss_sim::StartupMode::HOT;
    config.sampling_rate_hz = 10;
    config.duration_ns = 18LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.ttff.power_on_ns = 7LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.ttff.power_off_ns = 2LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.bestpos_rtk.stable_duration_ns = 1LL * gnss_sim::NANOSECONDS_PER_SECOND;
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run(config, "ttff_bestpos_rtk", &summary, &error_message)) << error_message;
    const std::vector<std::string> states = bestpos_states(read_file(test_output_path("ttff_bestpos_rtk")));
    EXPECT_TRUE(has_fix_reset_refix(states));
    std::remove(test_output_path("ttff_bestpos_rtk").c_str());
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
