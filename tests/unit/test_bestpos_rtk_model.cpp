#include "gnss_sim/sim_time.h"
#include "model/bestpos_rtk_model.h"

#include <gtest/gtest.h>
#include <string>

namespace {

gnss_sim::PositionSolution valid_position(int used_satellites = 8) {
    gnss_sim::PositionSolution position{};
    position.valid = true;
    position.status = gnss_sim::ReceiverSolutionStatus::kSolComputed;
    position.type = gnss_sim::ReceiverSolutionType::kSingle;
    position.used_satellites = used_satellites;
    return position;
}

gnss_sim::SimTime time_at(double sow_sec) {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2300, sow_sec, &time));
    return time;
}

TEST(BestposRtkModel, FixesOnlyAfterContinuousStableDuration) {
    gnss_sim::BestposRtkConfig config{true, 5LL * gnss_sim::NANOSECONDS_PER_SECOND, 6, 0.01, 0.02};
    gnss_sim::BestposRtkState state{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(100.0), valid_position(), &state, &error_message));
    EXPECT_TRUE(state.stability_active);
    EXPECT_FALSE(state.fixed);
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(104.999), valid_position(), &state, &error_message));
    EXPECT_FALSE(state.fixed);
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(105.0), valid_position(), &state, &error_message));
    EXPECT_TRUE(state.fixed);
}

TEST(BestposRtkModel, InvalidOrLowSatellitePositionResetsFix) {
    gnss_sim::BestposRtkConfig config{true, 0, 6, 0.01, 0.02};
    gnss_sim::BestposRtkState state{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(100.0), valid_position(), &state, &error_message));
    ASSERT_TRUE(state.fixed);
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(101.0), valid_position(5), &state, &error_message));
    EXPECT_FALSE(state.stability_active);
    EXPECT_FALSE(state.fixed);

    gnss_sim::PositionSolution invalid{};
    invalid.status = gnss_sim::ReceiverSolutionStatus::kInsufficientObs;
    invalid.type = gnss_sim::ReceiverSolutionType::kNone;
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(102.0), invalid, &state, &error_message));
    EXPECT_FALSE(state.fixed);
}

TEST(BestposRtkModel, DisabledModelNeverFixes) {
    gnss_sim::BestposRtkConfig config{false, 0, 4, 0.01, 0.02};
    gnss_sim::BestposRtkState state{true, true, time_at(90.0)};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(100.0), valid_position(), &state, &error_message));
    EXPECT_FALSE(state.stability_active);
    EXPECT_FALSE(state.fixed);
}

} // namespace
