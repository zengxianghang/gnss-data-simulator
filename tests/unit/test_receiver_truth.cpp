#include "gnss/rtklib_adapter.h"
#include "gnss_sim/sim_config.h"
#include "model/receiver_truth.h"

#include <cmath>
#include <gtest/gtest.h>
#include <string>

namespace {

TEST(ReceiverTruth, DefaultStaticSiteRoundTripsThroughRtklib) {
    const gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    gnss_sim::ReceiverTruth truth{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::make_static_receiver_truth(config.receiver, &truth, &error_message)) << error_message;

    EXPECT_DOUBLE_EQ(truth.latitude_deg, 20.0);
    EXPECT_DOUBLE_EQ(truth.longitude_deg, 120.0);
    EXPECT_DOUBLE_EQ(truth.height_m, 100.0);
    EXPECT_DOUBLE_EQ(truth.velocity_ecef_mps[0], 0.0);
    EXPECT_DOUBLE_EQ(truth.velocity_ecef_mps[1], 0.0);
    EXPECT_DOUBLE_EQ(truth.velocity_ecef_mps[2], 0.0);

    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double height_m = 0.0;
    ASSERT_TRUE(gnss_sim::rtklib_ecef_to_llh(truth.position_ecef_m, &latitude_deg, &longitude_deg, &height_m));
    EXPECT_NEAR(latitude_deg, 20.0, 1.0e-10);
    EXPECT_NEAR(longitude_deg, 120.0, 1.0e-10);
    EXPECT_NEAR(height_m, 100.0, 1.0e-6);
}

TEST(ReceiverTruth, RejectsInvalidCoordinatesAndNullOutput) {
    gnss_sim::ReceiverConfig invalid{91.0, 120.0, 100.0};
    gnss_sim::ReceiverTruth truth{};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::make_static_receiver_truth(invalid, &truth, &error_message));
    EXPECT_FALSE(error_message.empty());

    const gnss_sim::ReceiverConfig valid{20.0, 120.0, 100.0};
    error_message.clear();
    EXPECT_FALSE(gnss_sim::make_static_receiver_truth(valid, nullptr, &error_message));
    EXPECT_FALSE(error_message.empty());
}

} // namespace
