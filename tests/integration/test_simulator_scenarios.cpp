#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

#include <gtest/gtest.h>
#include <string>

namespace {

std::string nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/gps_loopback_nav.rnx";
}

gnss_sim::SimTime start_time() {
    gnss_sim::SimTime result{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2253, 172900.0, &result));
    return result;
}

struct SinkStats {
    std::uint64_t zero_range_count;
    std::uint64_t invalid_position_count;
    std::uint64_t invalid_velocity_count;
    std::uint64_t nav_count;
};

bool count_line(gnss_sim::SimulationLogKind kind, const gnss_sim::SimTime&, const char* data, std::size_t size,
                void* user_data) {
    SinkStats* stats = static_cast<SinkStats*>(user_data);
    if (stats == nullptr || data == nullptr) {
        return false;
    }
    const std::string line(data, size);
    if (kind == gnss_sim::SimulationLogKind::kRange && line.find(";0*") != std::string::npos) {
        ++stats->zero_range_count;
    } else if (kind == gnss_sim::SimulationLogKind::kPosition &&
               line.find(";INSUFFICIENT_OBS,NONE,") != std::string::npos) {
        ++stats->invalid_position_count;
    } else if (kind == gnss_sim::SimulationLogKind::kVelocity &&
               line.find(";INSUFFICIENT_OBS,NONE,") != std::string::npos) {
        ++stats->invalid_velocity_count;
    } else if (kind == gnss_sim::SimulationLogKind::kNavigation) {
        ++stats->nav_count;
    }
    return true;
}

gnss_sim::SimConfig base_config() {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    config.elevation_mask_deg = 0.0;
    config.output_eph = false;
    config.output_ion = false;
    config.seed = 7U;
    return config;
}

bool run(const gnss_sim::SimConfig& config, SinkStats* sink_stats, gnss_sim::SimulationRunStats* run_stats,
         std::string* error_message) {
    const std::string path = nav_path();
    const gnss_sim::SimulationRequest request{config, start_time(), path.c_str()};
    gnss_sim::SimulationOutputSink sink{};
    if (sink_stats != nullptr) {
        sink.write_line = count_line;
        sink.user_data = sink_stats;
    }
    return gnss_sim::run_simulation(request, sink, run_stats, error_message);
}

TEST(SimulatorIntegration, SupportedRatesHaveExactHalfOpenEpochCounts) {
    for (const int rate : {1, 5, 10, 20, 50}) {
        gnss_sim::SimConfig config = base_config();
        config.scenario = gnss_sim::ScenarioType::KS;
        config.sampling_rate_hz = rate;
        config.duration_ns = gnss_sim::NANOSECONDS_PER_SECOND;

        gnss_sim::SimulationRunStats stats{};
        std::string error_message;
        ASSERT_TRUE(run(config, nullptr, &stats, &error_message)) << "rate=" << rate << " " << error_message;
        EXPECT_EQ(stats.total_epochs, static_cast<std::uint64_t>(rate));
        EXPECT_EQ(stats.powered_epochs, static_cast<std::uint64_t>(rate));
        EXPECT_EQ(stats.range_log_count, static_cast<std::uint64_t>(rate));
        EXPECT_EQ(stats.position_log_count, static_cast<std::uint64_t>(rate));
        EXPECT_EQ(stats.velocity_log_count, static_cast<std::uint64_t>(rate));
        EXPECT_EQ(stats.runtime_signal_count, 40U);
        EXPECT_LE(stats.maximum_observations_in_epoch, stats.runtime_signal_count);
    }
}

TEST(SimulatorIntegration, ReaKeepsLoggingWhileSignalIsOff) {
    gnss_sim::SimConfig config = base_config();
    config.scenario = gnss_sim::ScenarioType::REA;
    config.sampling_rate_hz = 10;
    config.duration_ns = 3LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_on_ns = gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_off_ns = gnss_sim::NANOSECONDS_PER_SECOND / 2;

    SinkStats sink_stats{};
    gnss_sim::SimulationRunStats stats{};
    std::string error_message;
    ASSERT_TRUE(run(config, &sink_stats, &stats, &error_message)) << error_message;
    EXPECT_EQ(stats.total_epochs, 30U);
    EXPECT_EQ(stats.powered_epochs, 30U);
    EXPECT_EQ(stats.range_log_count, 30U);
    EXPECT_EQ(stats.position_log_count, 30U);
    EXPECT_EQ(stats.velocity_log_count, 30U);
    EXPECT_EQ(stats.signal_off_event_count, 2U);
    EXPECT_EQ(stats.signal_on_event_count, 2U);
    EXPECT_EQ(stats.startup_event_count, 1U);
    EXPECT_EQ(sink_stats.zero_range_count, 10U);
    EXPECT_GE(sink_stats.invalid_position_count, 10U);
    EXPECT_GE(sink_stats.invalid_velocity_count, 10U);
}

TEST(SimulatorIntegration, TtffSuppressesLogsDuringPowerOffForEveryStartupMode) {
    for (const gnss_sim::StartupMode mode :
         {gnss_sim::StartupMode::HOT, gnss_sim::StartupMode::WARM, gnss_sim::StartupMode::COLD}) {
        gnss_sim::SimConfig config = base_config();
        config.scenario = gnss_sim::ScenarioType::TTFF;
        config.ttff.startup_mode = mode;
        config.ttff.power_on_ns = gnss_sim::NANOSECONDS_PER_SECOND;
        config.ttff.power_off_ns = gnss_sim::NANOSECONDS_PER_SECOND / 2;
        config.sampling_rate_hz = 10;
        config.duration_ns = 3LL * gnss_sim::NANOSECONDS_PER_SECOND;

        SinkStats sink_stats{};
        gnss_sim::SimulationRunStats stats{};
        std::string error_message;
        ASSERT_TRUE(run(config, &sink_stats, &stats, &error_message))
            << gnss_sim::startup_mode_name(mode) << " " << error_message;
        EXPECT_EQ(stats.total_epochs, 30U);
        EXPECT_EQ(stats.powered_epochs, 20U);
        EXPECT_EQ(stats.range_log_count, 20U);
        EXPECT_EQ(stats.position_log_count, 20U);
        EXPECT_EQ(stats.velocity_log_count, 20U);
        EXPECT_EQ(stats.power_off_event_count, 2U);
        EXPECT_EQ(stats.power_on_event_count, 2U);
        EXPECT_EQ(stats.startup_event_count, 2U);
        if (mode == gnss_sim::StartupMode::COLD) {
            EXPECT_EQ(sink_stats.invalid_position_count, stats.position_log_count);
        }
    }
}

TEST(SimulatorIntegration, ColdEphemerisAppearsOnlyAfterNavigationCollection) {
    gnss_sim::SimConfig config = base_config();
    config.scenario = gnss_sim::ScenarioType::TTFF;
    config.ttff.startup_mode = gnss_sim::StartupMode::COLD;
    config.ttff.power_on_ns = 40LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.ttff.power_off_ns = gnss_sim::NANOSECONDS_PER_SECOND;
    config.sampling_rate_hz = 1;
    config.duration_ns = 40LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.output_eph = true;

    SinkStats sink_stats{};
    gnss_sim::SimulationRunStats stats{};
    std::string error_message;
    ASSERT_TRUE(run(config, &sink_stats, &stats, &error_message)) << error_message;
    EXPECT_EQ(stats.total_epochs, 40U);
    EXPECT_EQ(stats.powered_epochs, 40U);
    EXPECT_EQ(stats.startup_event_count, 1U);
    EXPECT_GT(stats.navigation_update_count, 0U);
    EXPECT_GT(stats.navigation_log_count, 0U);
    EXPECT_EQ(sink_stats.nav_count, stats.navigation_log_count);
}

} // namespace
