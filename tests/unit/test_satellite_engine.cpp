#include "gnss/satellite_engine.h"

#include "gnss/rtklib_adapter.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "model/receiver_truth.h"

#include <gtest/gtest.h>

extern "C" {
#include <rtklib.h>
}

#ifdef lock
#undef lock
#endif
#ifdef unlock
#undef unlock
#endif

#include <cmath>
#include <string>

namespace {

constexpr double kSpeedOfLightMps = 299792458.0;

std::string test_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/mixed_nav_2019.rnx";
}

std::string rtklib_reference_nav_path() {
    std::string path = test_nav_path();
#ifdef _WIN32
    for (char& character : path) {
        if (character == '/') {
            character = '\\';
        }
    }
#endif
    return path;
}

class SatelliteEngineTest : public ::testing::Test {
  protected:
    void SetUp() override {
        nav_store_ = gnss_sim::create_rtklib_nav_store();
        ASSERT_NE(nav_store_, nullptr);
        std::string error_message;
        ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav_store_, test_nav_path().c_str(), &error_message))
            << error_message;

        const gnss_sim::SimConfig config = gnss_sim::default_sim_config();
        ASSERT_TRUE(gnss_sim::make_static_receiver_truth(config.receiver, &receiver_, &error_message))
            << error_message;
    }

    void TearDown() override {
        gnss_sim::destroy_rtklib_nav_store(nav_store_);
    }

    gnss_sim::RtklibNavStore* nav_store_ = nullptr;
    gnss_sim::ReceiverTruth receiver_{};
};

TEST(SatelliteEngineTime, PropagationSubtractionCrossesGpsWeek) {
    const gnss_sim::SimTime receive_time{2300, 50000000LL};
    int transmit_week = 0;
    double transmit_sow_sec = 0.0;
    ASSERT_TRUE(gnss_sim::subtract_propagation_time(receive_time, 0.075, &transmit_week, &transmit_sow_sec));
    EXPECT_EQ(transmit_week, 2299);
    EXPECT_NEAR(transmit_sow_sec, 604799.975, 1.0e-12);
}

TEST(SatelliteEngineMask, ExactlyOnAndAroundThreeDegreesAreDeterministic) {
    const double three_deg_rad = 3.0 * D2R;
    EXPECT_TRUE(gnss_sim::elevation_passes_mask(three_deg_rad, 3.0));
    EXPECT_FALSE(gnss_sim::elevation_passes_mask(three_deg_rad - 1.0e-12, 3.0));
    EXPECT_TRUE(gnss_sim::elevation_passes_mask(three_deg_rad + 1.0e-12, 3.0));
}

TEST_F(SatelliteEngineTest, TransmitTimeConvergesForRepresentativeSatellitesInEveryConstellation) {
    struct Case {
        const char* satellite_id;
        int gps_week;
        double receive_sow_sec;
    };
    const Case cases[] = {
        {"G01", 2041, 176400.0}, {"E01", 2041, 176400.0}, {"C01", 2041, 176400.0},
        {"J01", 2041, 176400.0}, {"R26", 2041, 258300.0},
    };

    for (const Case& test_case : cases) {
        SCOPED_TRACE(test_case.satellite_id);
        int satellite_number = 0;
        ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number(test_case.satellite_id, &satellite_number));

        gnss_sim::SimTime receive_time{};
        ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(test_case.gps_week, test_case.receive_sow_sec, &receive_time));
        gnss_sim::SatelliteGeometry geometry{};
        std::string error_message;
        ASSERT_TRUE(gnss_sim::compute_satellite_geometry(nav_store_, receiver_, receive_time, satellite_number, 3.0,
                                                         &geometry, &error_message))
            << error_message;

        EXPECT_GE(geometry.iteration_count, 1);
        EXPECT_LE(geometry.iteration_count, 12);
        EXPECT_GT(geometry.propagation_time_sec, 0.04);
        EXPECT_LT(geometry.propagation_time_sec, 0.20);
        EXPECT_GT(geometry.geometric_range_m, 1.0e7);
        EXPECT_LT(geometry.geometric_range_m, 6.0e7);
        EXPECT_TRUE(std::isfinite(geometry.range_rate_mps));
        EXPECT_TRUE(std::isfinite(geometry.azimuth_rad));
        EXPECT_TRUE(std::isfinite(geometry.elevation_rad));
        EXPECT_EQ(geometry.visible, geometry.healthy && geometry.above_elevation_mask);

        const double receive_sow = gnss_sim::sim_time_sow_sec(receive_time);
        if (geometry.transmit_gps_week == receive_time.gps_week) {
            EXPECT_LT(geometry.transmit_sow_sec, receive_sow);
        }
    }
}

TEST_F(SatelliteEngineTest, GeometryMatchesDirectRtklibReferenceAtConvergedTransmitTime) {
    int satellite_number = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &satellite_number));
    gnss_sim::SimTime receive_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180000.0, &receive_time));

    gnss_sim::SatelliteGeometry geometry{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_satellite_geometry(nav_store_, receiver_, receive_time, satellite_number, 3.0,
                                                     &geometry, &error_message))
        << error_message;

    nav_t reference_nav{};
    obs_t reference_obs{};
    sta_t reference_station{};
    const std::string reference_path = rtklib_reference_nav_path();
    ASSERT_NE(readrnx(reference_path.c_str(), 1, "", &reference_obs, &reference_nav, &reference_station), 0);
    freeobs(&reference_obs);
    uniqnav(&reference_nav);

    const gtime_t transmit_time = gpst2time(geometry.transmit_gps_week, geometry.transmit_sow_sec);
    double reference_state[6]{};
    double reference_clock[2]{};
    double reference_variance_m2 = 0.0;
    int reference_health = 0;
    ASSERT_NE(satpos(transmit_time, transmit_time, satellite_number, EPHOPT_BRDC, &reference_nav, reference_state,
                     reference_clock, &reference_variance_m2, &reference_health),
              0);

    double reference_los[3]{};
    const double reference_range_m = geodist(reference_state, receiver_.position_ecef_m, reference_los);
    ASSERT_GT(reference_range_m, 0.0);
    double receiver_pos[3]{};
    double reference_azel[2]{};
    ecef2pos(receiver_.position_ecef_m, receiver_pos);
    satazel(receiver_pos, reference_los, reference_azel);

    double reference_range_rate_mps = 0.0;
    for (int index = 0; index < 3; ++index) {
        reference_range_rate_mps +=
            (reference_state[index + 3] - receiver_.velocity_ecef_mps[index]) * reference_los[index];
    }
    reference_range_rate_mps +=
        OMGE / kSpeedOfLightMps *
        (reference_state[4] * receiver_.position_ecef_m[0] +
         reference_state[1] * receiver_.velocity_ecef_mps[0] -
         reference_state[3] * receiver_.position_ecef_m[1] -
         reference_state[0] * receiver_.velocity_ecef_mps[1]);

    for (int index = 0; index < 3; ++index) {
        EXPECT_NEAR(geometry.satellite_state.position_ecef_m[index], reference_state[index], 1.0e-6);
        EXPECT_NEAR(geometry.satellite_state.velocity_ecef_mps[index], reference_state[index + 3], 1.0e-9);
        EXPECT_NEAR(geometry.line_of_sight_ecef[index], reference_los[index], 1.0e-14);
    }
    EXPECT_NEAR(geometry.satellite_state.clock_bias_sec, reference_clock[0], 1.0e-15);
    EXPECT_NEAR(geometry.satellite_state.clock_drift_sec_per_sec, reference_clock[1], 1.0e-18);
    EXPECT_EQ(geometry.satellite_state.health, reference_health);
    EXPECT_NEAR(geometry.geometric_range_m, reference_range_m, 1.0e-6);
    EXPECT_NEAR(geometry.azimuth_rad, reference_azel[0], 1.0e-14);
    EXPECT_NEAR(geometry.elevation_rad, reference_azel[1], 1.0e-14);
    EXPECT_NEAR(geometry.range_rate_mps, reference_range_rate_mps, 1.0e-9);
    EXPECT_NEAR(geometry.propagation_time_sec, geometry.geometric_range_m / kSpeedOfLightMps, 1.0e-15);

    freenav(&reference_nav, 0xFF);
}

TEST_F(SatelliteEngineTest, SatelliteStateIsEvaluatedAtTransmitRatherThanReceiveTime) {
    int satellite_number = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &satellite_number));
    gnss_sim::SimTime receive_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180000.0, &receive_time));

    gnss_sim::SatelliteGeometry geometry{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_satellite_geometry(nav_store_, receiver_, receive_time, satellite_number, 3.0,
                                                     &geometry, &error_message))
        << error_message;

    gnss_sim::RtklibSatelliteState receive_state{};
    ASSERT_TRUE(gnss_sim::get_rtklib_satellite_state(nav_store_, receive_time.gps_week,
                                                     gnss_sim::sim_time_sow_sec(receive_time), satellite_number,
                                                     &receive_state, &error_message));

    const double position_delta_m =
        std::hypot(std::hypot(geometry.satellite_state.position_ecef_m[0] - receive_state.position_ecef_m[0],
                              geometry.satellite_state.position_ecef_m[1] - receive_state.position_ecef_m[1]),
                   geometry.satellite_state.position_ecef_m[2] - receive_state.position_ecef_m[2]);
    EXPECT_GT(position_delta_m, 10.0);
}

} // namespace
