#include "model/cn0_model.h"

#include "gnss_sim/sim_time.h"

#include <gtest/gtest.h>

#include <cmath>

namespace {

double estimate(gnss_sim::SignalId signal_id, double elevation_deg, double sow_sec, std::uint64_t seed = 1234U) {
    const gnss_sim::Cn0Model model = gnss_sim::make_builtin_cn0_model(seed);
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2300, sow_sec, &time));
    double cn0_dbhz = 0.0;
    EXPECT_TRUE(gnss_sim::cn0_model_estimate_dbhz(model, signal_id, elevation_deg, time, &cn0_dbhz));
    return cn0_dbhz;
}

TEST(Cn0Model, ElevationFallbackIsContinuousAndMonotonicAtPiecewiseBoundaries) {
    const double sow = 180000.0;
    const gnss_sim::SignalId signal = gnss_sim::SignalId::kGpsL1Ca;
    const double elevations[] = {0.0, 4.999, 5.0, 5.001, 14.999, 15.0, 15.001,
                                 29.999, 30.0, 30.001, 59.999, 60.0, 60.001, 90.0};
    double previous = estimate(signal, elevations[0], sow);
    for (std::size_t index = 1; index < sizeof(elevations) / sizeof(elevations[0]); ++index) {
        const double current = estimate(signal, elevations[index], sow);
        EXPECT_GE(current + 1.0e-12, previous);
        previous = current;
    }
    EXPECT_GT(estimate(signal, 90.0, sow), estimate(signal, 3.0, sow) + 10.0);
}

TEST(Cn0Model, SameSeedAndTimeAreExactlyRepeatable) {
    const double first = estimate(gnss_sim::SignalId::kGalileoE5A, 42.0, 200000.25, 99U);
    const double second = estimate(gnss_sim::SignalId::kGalileoE5A, 42.0, 200000.25, 99U);
    EXPECT_DOUBLE_EQ(first, second);

    const double later = estimate(gnss_sim::SignalId::kGalileoE5A, 42.0, 200001.25, 99U);
    EXPECT_NE(first, later);
    EXPECT_LT(std::abs(first - later), 1.0);
}

TEST(Cn0Model, SignalCalibrationOffsetsRemainVisible) {
    const double l1c = estimate(gnss_sim::SignalId::kGpsL1C, 45.0, 250000.0);
    const double l2p = estimate(gnss_sim::SignalId::kGpsL2P, 45.0, 250000.0);
    EXPECT_GT(l1c, l2p + 1.0);
}

TEST(Cn0Model, TemporalVariationIsContinuousAcrossGpsWeekBoundary) {
    const gnss_sim::Cn0Model model = gnss_sim::make_builtin_cn0_model(7U);
    gnss_sim::SimTime before{};
    gnss_sim::SimTime after{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 604799.9, &before));
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2301, 0.1, &after));
    double before_cn0 = 0.0;
    double after_cn0 = 0.0;
    ASSERT_TRUE(gnss_sim::cn0_model_estimate_dbhz(model, gnss_sim::SignalId::kBeidouB1C, 35.0, before, &before_cn0));
    ASSERT_TRUE(gnss_sim::cn0_model_estimate_dbhz(model, gnss_sim::SignalId::kBeidouB1C, 35.0, after, &after_cn0));
    EXPECT_LT(std::abs(after_cn0 - before_cn0), 0.2);
}

TEST(Cn0Model, InvalidArgumentsAreRejected) {
    const gnss_sim::Cn0Model model = gnss_sim::make_builtin_cn0_model(1U);
    gnss_sim::SimTime time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 1.0, &time));
    EXPECT_FALSE(gnss_sim::cn0_model_estimate_dbhz(model, gnss_sim::SignalId::kGpsL1Ca, NAN, time, nullptr));
}

} // namespace
