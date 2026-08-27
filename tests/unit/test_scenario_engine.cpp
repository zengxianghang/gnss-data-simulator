#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "scenario/scenario_engine.h"

#include <gtest/gtest.h>

namespace {

gnss_sim::SimTime make_time(int week, double sow) {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(week, sow, &time));
    return time;
}

TEST(ScenarioEngine, KsRemainsPoweredAndSignalOn) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::KS;
    gnss_sim::ScenarioEngine engine{};
    std::string error;
    ASSERT_TRUE(gnss_sim::initialize_scenario_engine(config, make_time(2300, 604799.5), &engine, &error)) << error;

    gnss_sim::ScenarioEpochState state{};
    ASSERT_TRUE(gnss_sim::update_scenario_engine(&engine, make_time(2300, 604799.5), &state, &error)) << error;
    EXPECT_TRUE(state.receiver_powered);
    EXPECT_TRUE(state.signal_available);
    EXPECT_TRUE(state.power_on_transition);
    EXPECT_TRUE(state.signal_on_transition);

    ASSERT_TRUE(gnss_sim::update_scenario_engine(&engine, make_time(2301, 0.5), &state, &error)) << error;
    EXPECT_TRUE(state.receiver_powered);
    EXPECT_TRUE(state.signal_available);
    EXPECT_FALSE(state.power_on_transition);
    EXPECT_FALSE(state.signal_on_transition);
}

TEST(ScenarioEngine, ReaChangesSignalAtExactIntegerBoundaries) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::REA;
    config.rea.signal_on_ns = 300LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_off_ns = 10LL * gnss_sim::NANOSECONDS_PER_SECOND;
    const gnss_sim::SimTime start = make_time(2300, 1000.0);
    gnss_sim::ScenarioEngine engine{};
    std::string error;
    ASSERT_TRUE(gnss_sim::initialize_scenario_engine(config, start, &engine, &error)) << error;

    gnss_sim::ScenarioEpochState state{};
    ASSERT_TRUE(gnss_sim::update_scenario_engine(&engine, start, &state, &error));
    ASSERT_TRUE(gnss_sim::update_scenario_engine(&engine, make_time(2300, 1299.999), &state, &error));
    EXPECT_TRUE(state.signal_available);
    ASSERT_TRUE(gnss_sim::update_scenario_engine(&engine, make_time(2300, 1300.0), &state, &error));
    EXPECT_FALSE(state.signal_available);
    EXPECT_TRUE(state.signal_off_transition);
    EXPECT_TRUE(state.receiver_powered);
    ASSERT_TRUE(gnss_sim::update_scenario_engine(&engine, make_time(2300, 1310.0), &state, &error));
    EXPECT_TRUE(state.signal_available);
    EXPECT_TRUE(state.signal_on_transition);
    EXPECT_EQ(state.cycle_index, 1U);
}

TEST(ScenarioEngine, TtffPowerOffSuppressesSignalAndRestartsEachCycle) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::TTFF;
    const gnss_sim::SimTime start = make_time(2300, 2000.0);
    gnss_sim::ScenarioEngine engine{};
    std::string error;
    ASSERT_TRUE(gnss_sim::initialize_scenario_engine(config, start, &engine, &error)) << error;

    gnss_sim::ScenarioEpochState state{};
    ASSERT_TRUE(gnss_sim::update_scenario_engine(&engine, start, &state, &error));
    ASSERT_TRUE(gnss_sim::update_scenario_engine(&engine, make_time(2300, 2300.0), &state, &error));
    EXPECT_FALSE(state.receiver_powered);
    EXPECT_FALSE(state.signal_available);
    EXPECT_TRUE(state.power_off_transition);
    EXPECT_TRUE(state.signal_off_transition);
    ASSERT_TRUE(gnss_sim::update_scenario_engine(&engine, make_time(2300, 2330.0), &state, &error));
    EXPECT_TRUE(state.receiver_powered);
    EXPECT_TRUE(state.signal_available);
    EXPECT_TRUE(state.power_on_transition);
    EXPECT_TRUE(state.signal_on_transition);
    EXPECT_EQ(state.cycle_index, 1U);
}

TEST(ScenarioEngine, EventEffectiveAtFirstEpochAtOrAfterBoundary) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::REA;
    config.rea.signal_on_ns = 1000500000LL;
    config.rea.signal_off_ns = 1000000000LL;
    config.sampling_rate_hz = 10;
    const gnss_sim::SimTime start = make_time(2300, 1000.0);
    gnss_sim::ScenarioEngine engine{};
    std::string error;
    ASSERT_TRUE(gnss_sim::initialize_scenario_engine(config, start, &engine, &error));

    gnss_sim::ScenarioEpochState state{};
    ASSERT_TRUE(gnss_sim::update_scenario_engine(&engine, start, &state, &error));
    ASSERT_TRUE(gnss_sim::update_scenario_engine(&engine, make_time(2300, 1001.0), &state, &error));
    EXPECT_TRUE(state.signal_available);
    ASSERT_TRUE(gnss_sim::update_scenario_engine(&engine, make_time(2300, 1001.1), &state, &error));
    EXPECT_FALSE(state.signal_available);
    EXPECT_TRUE(state.signal_off_transition);
}

} // namespace
