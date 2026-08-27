#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "scenario/scenario_engine.h"

#include <gtest/gtest.h>
#include <string>

namespace {

gnss_sim::SimTime time_at(const gnss_sim::SimTime& start, double seconds) {
    gnss_sim::SimTime result{};
    EXPECT_TRUE(gnss_sim::add_time_ns(start, static_cast<std::int64_t>(seconds * gnss_sim::NANOSECONDS_PER_SECOND),
                                      &result));
    return result;
}

gnss_sim::ScenarioEpochState update(gnss_sim::ScenarioEngine* engine, const gnss_sim::SimTime& time) {
    gnss_sim::ScenarioEpochState state{};
    std::string error_message;
    EXPECT_TRUE(gnss_sim::update_scenario_engine(engine, time, &state, &error_message)) << error_message;
    return state;
}

TEST(ScenarioEngine, KsIsContinuouslyPoweredAndSignaled) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::KS;
    config.duration_ns = 2LL * gnss_sim::NANOSECONDS_PER_SECOND;
    const gnss_sim::SimTime start{2300, 100LL * gnss_sim::NANOSECONDS_PER_SECOND};
    gnss_sim::ScenarioEngine engine{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::initialize_scenario_engine(config, start, &engine, &error_message)) << error_message;

    const auto first = update(&engine, start);
    EXPECT_TRUE(first.powered);
    EXPECT_TRUE(first.signal_available);
    EXPECT_TRUE(first.power_on_event);
    EXPECT_TRUE(first.signal_on_event);
    EXPECT_TRUE(first.startup_event);

    const auto second = update(&engine, time_at(start, 1.0));
    EXPECT_TRUE(second.powered);
    EXPECT_TRUE(second.signal_available);
    EXPECT_FALSE(second.power_on_event);
    EXPECT_FALSE(second.signal_on_event);
}

TEST(ScenarioEngine, ReaBoundaryIsFirstEpochAtOrAfterSignalEvent) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::REA;
    config.duration_ns = 700LL * gnss_sim::NANOSECONDS_PER_SECOND;
    const gnss_sim::SimTime start{2300, 0};
    gnss_sim::ScenarioEngine engine{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::initialize_scenario_engine(config, start, &engine, &error_message)) << error_message;

    update(&engine, start);
    auto state = update(&engine, time_at(start, 299.9));
    EXPECT_TRUE(state.signal_available);
    state = update(&engine, time_at(start, 300.0));
    EXPECT_FALSE(state.signal_available);
    EXPECT_TRUE(state.signal_off_event);
    EXPECT_TRUE(state.powered);
    state = update(&engine, time_at(start, 309.9));
    EXPECT_FALSE(state.signal_available);
    state = update(&engine, time_at(start, 310.0));
    EXPECT_TRUE(state.signal_available);
    EXPECT_TRUE(state.signal_on_event);
    EXPECT_EQ(state.cycle_index, 1U);
}

TEST(ScenarioEngine, TtffPowerCyclesAndRestartsConfiguredMode) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::TTFF;
    config.ttff.startup_mode = gnss_sim::StartupMode::COLD;
    config.duration_ns = 700LL * gnss_sim::NANOSECONDS_PER_SECOND;
    const gnss_sim::SimTime start{2300, 0};
    gnss_sim::ScenarioEngine engine{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::initialize_scenario_engine(config, start, &engine, &error_message)) << error_message;

    auto state = update(&engine, start);
    EXPECT_TRUE(state.startup_event);
    EXPECT_EQ(state.startup_mode, gnss_sim::StartupMode::COLD);
    state = update(&engine, time_at(start, 300.0));
    EXPECT_FALSE(state.powered);
    EXPECT_FALSE(state.signal_available);
    EXPECT_TRUE(state.power_off_event);
    EXPECT_TRUE(state.signal_off_event);
    state = update(&engine, time_at(start, 330.0));
    EXPECT_TRUE(state.powered);
    EXPECT_TRUE(state.signal_available);
    EXPECT_TRUE(state.power_on_event);
    EXPECT_TRUE(state.signal_on_event);
    EXPECT_TRUE(state.startup_event);
    EXPECT_EQ(state.cycle_index, 1U);
}

TEST(ScenarioEngine, WorksAcrossGpsWeekBoundary) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::REA;
    config.rea.signal_on_ns = 2LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_off_ns = 1LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.duration_ns = 5LL * gnss_sim::NANOSECONDS_PER_SECOND;
    const gnss_sim::SimTime start{2300, gnss_sim::GPS_WEEK_NANOSECONDS - gnss_sim::NANOSECONDS_PER_SECOND};
    gnss_sim::ScenarioEngine engine{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::initialize_scenario_engine(config, start, &engine, &error_message)) << error_message;

    update(&engine, start);
    const auto state = update(&engine, time_at(start, 2.0));
    EXPECT_EQ(state.time.gps_week, 2301);
    EXPECT_FALSE(state.signal_available);
    EXPECT_TRUE(state.signal_off_event);
}

} // namespace
