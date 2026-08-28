#include "gnss/galileo_has_adapter.h"

#include <gtest/gtest.h>

extern "C" {
#include <rtklib.h>
}

#include <cmath>
#include <string>

namespace {

std::string test_data_path(const char* name) {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/" + name;
}

TEST(GalileoHasAdapter, LoadsOfficialJrcE6ProductsAndReturnsCoherentE02Correction) {
    gnss_sim::GalileoHasStore* store = gnss_sim::create_galileo_has_store();
    ASSERT_NE(store, nullptr);

    const std::string sp3_path = test_data_path("jrc_has_2026001_e02.sp3");
    const std::string clock_path = test_data_path("jrc_has_2026001_e02.clk");
    const std::string bias_path = test_data_path("jrc_has_2026001_e02_c6c.bia");
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_galileo_has_products(store, sp3_path.c_str(), clock_path.c_str(), bias_path.c_str(),
                                                    &error_message))
        << error_message;

    const int satellite_number = satid2no("E02");
    ASSERT_GT(satellite_number, 0);
    double epoch[6] = {2026.0, 1.0, 1.0, 0.0, 8.0, 20.0};
    const gtime_t time = epoch2time(epoch);
    int gps_week = 0;
    const double sow_sec = time2gpst(time, &gps_week);

    gnss_sim::GalileoHasE6Correction correction{};
    ASSERT_TRUE(
        gnss_sim::galileo_has_e6_correction(store, gps_week, sow_sec, satellite_number, &correction, &error_message))
        << error_message;

    double position_norm_sq = 0.0;
    double velocity_norm_sq = 0.0;
    for (int index = 0; index < 3; ++index) {
        EXPECT_TRUE(std::isfinite(correction.satellite_state.position_ecef_m[index]));
        EXPECT_TRUE(std::isfinite(correction.satellite_state.velocity_ecef_mps[index]));
        position_norm_sq +=
            correction.satellite_state.position_ecef_m[index] * correction.satellite_state.position_ecef_m[index];
        velocity_norm_sq +=
            correction.satellite_state.velocity_ecef_mps[index] * correction.satellite_state.velocity_ecef_mps[index];
    }
    EXPECT_GT(std::sqrt(position_norm_sq), 2.0e7);
    EXPECT_LT(std::sqrt(position_norm_sq), 3.5e7);
    EXPECT_GT(std::sqrt(velocity_norm_sq), 100.0);
    EXPECT_LT(std::sqrt(velocity_norm_sq), 1.0e4);
    EXPECT_TRUE(std::isfinite(correction.satellite_state.clock_bias_sec));
    EXPECT_TRUE(std::isfinite(correction.satellite_state.clock_drift_sec_per_sec));
    EXPECT_NEAR(correction.code_osb_m, -0.4, 1.0e-12);

    double outside_epoch[6] = {2026.0, 1.0, 1.0, 0.0, 13.0, 30.0};
    const gtime_t outside_time = epoch2time(outside_epoch);
    int outside_week = 0;
    const double outside_sow = time2gpst(outside_time, &outside_week);
    EXPECT_FALSE(gnss_sim::galileo_has_e6_correction(store, outside_week, outside_sow, satellite_number, &correction,
                                                     &error_message));
    EXPECT_NE(error_message.find("C6C OSB"), std::string::npos);

    gnss_sim::destroy_galileo_has_store(store);
}

} // namespace
