#include "gnss/rtklib_adapter.h"
#include "gnss/satellite_engine.h"
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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
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
        ASSERT_TRUE(gnss_sim::make_static_receiver_truth(config.receiver, &receiver_, &error_message)) << error_message;
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
    const gtime_t selection_time = gpst2time(receive_time.gps_week, gnss_sim::sim_time_sow_sec(receive_time));
    double reference_state[6]{};
    double reference_clock[2]{};
    double reference_variance_m2 = 0.0;
    int reference_health = 0;
    ASSERT_NE(satpos(transmit_time, selection_time, satellite_number, EPHOPT_BRDC, &reference_nav, reference_state,
                     reference_clock, &reference_variance_m2, &reference_health),
              0);

    double reference_los[3]{};
    const double reference_range_m = geodist(reference_state, receiver_.position_ecef_m, reference_los);
    ASSERT_GT(reference_range_m, 0.0);
    double receiver_pos[3]{};
    double reference_azel[2]{};
    ecef2pos(receiver_.position_ecef_m, receiver_pos);
    satazel(receiver_pos, reference_los, reference_azel);

    // Rate of geodist(r_s(t - rho/c), r_r(t)): satellite term A, receiver term B.
    double satellite_term_mps = 0.0;
    double receiver_term_mps = 0.0;
    for (int index = 0; index < 3; ++index) {
        satellite_term_mps += reference_state[index + 3] * reference_los[index];
        receiver_term_mps -= receiver_.velocity_ecef_mps[index] * reference_los[index];
    }
    satellite_term_mps +=
        OMGE / kSpeedOfLightMps *
        (reference_state[3] * receiver_.position_ecef_m[1] - reference_state[4] * receiver_.position_ecef_m[0]);
    receiver_term_mps +=
        OMGE / kSpeedOfLightMps *
        (reference_state[0] * receiver_.velocity_ecef_mps[1] - reference_state[1] * receiver_.velocity_ecef_mps[0]);
    const double reference_range_rate_mps =
        (satellite_term_mps + receiver_term_mps) / (1.0 + satellite_term_mps / kSpeedOfLightMps);

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

TEST_F(SatelliteEngineTest, RangeRateIsTheRateOfTheSimulatedRange) {
    // Issue #181 item 2: range_rate_mps must be d(geometric_range_m)/dt of the
    // simulated light-time range, not RTKLIB resdop()'s first-order model.
    gnss_sim::RtklibNavStore* brd4_nav = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(brd4_nav, nullptr);
    std::string load_error;
    const std::string brd4_nav_path = std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(brd4_nav, brd4_nav_path.c_str(), &load_error)) << load_error;

    gnss_sim::SimTime receive_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 437100.0, &receive_time));
    constexpr std::int64_t kHalfStepNs = 1000000;
    gnss_sim::SimTime before{};
    gnss_sim::SimTime after{};
    ASSERT_TRUE(gnss_sim::add_time_ns(receive_time, -kHalfStepNs, &before));
    ASSERT_TRUE(gnss_sim::add_time_ns(receive_time, kHalfStepNs, &after));

    int checked = 0;
    double max_first_order_error_mps = 0.0;
    for (int prn = 1; prn <= 32; ++prn) {
        char satellite_id[8]{};
        std::snprintf(satellite_id, sizeof(satellite_id), "G%02d", prn);
        int satellite_number = 0;
        ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number(satellite_id, &satellite_number));
        gnss_sim::SatelliteGeometry geometry{};
        gnss_sim::SatelliteGeometry geometry_before{};
        gnss_sim::SatelliteGeometry geometry_after{};
        std::string error_message;
        if (!gnss_sim::compute_satellite_geometry(brd4_nav, receiver_, receive_time, satellite_number, -90.0, &geometry,
                                                  &error_message)) {
            continue;
        }
        ASSERT_TRUE(gnss_sim::compute_satellite_geometry(brd4_nav, receiver_, before, satellite_number, -90.0,
                                                         &geometry_before, &error_message))
            << error_message;
        ASSERT_TRUE(gnss_sim::compute_satellite_geometry(brd4_nav, receiver_, after, satellite_number, -90.0,
                                                         &geometry_after, &error_message))
            << error_message;
        const double finite_difference_mps =
            (geometry_after.geometric_range_m - geometry_before.geometric_range_m) / (2.0e-9 * kHalfStepNs);

        // RTKLIB's broadcast velocity is a 1 ms forward difference, which alone
        // leaves up to ~0.3 mm/s; the first-order model is off by millimetres.
        EXPECT_NEAR(geometry.range_rate_mps, finite_difference_mps, 5.0e-4) << satellite_id;

        const double* vs = geometry.satellite_state.velocity_ecef_mps;
        double first_order_mps = 0.0;
        for (int index = 0; index < 3; ++index) {
            first_order_mps += vs[index] * geometry.line_of_sight_ecef[index];
        }
        first_order_mps +=
            OMGE / kSpeedOfLightMps * (vs[1] * receiver_.position_ecef_m[0] - vs[0] * receiver_.position_ecef_m[1]);
        max_first_order_error_mps =
            std::max(max_first_order_error_mps, std::fabs(first_order_mps - finite_difference_mps));
        ++checked;
    }
    gnss_sim::destroy_rtklib_nav_store(brd4_nav);
    EXPECT_GE(checked, 12);
    EXPECT_GT(max_first_order_error_mps, 2.0e-3);
}

TEST_F(SatelliteEngineTest, AdapterCanFixEphemerisSelectionAtReceiveEpoch) {
    int satellite_number = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &satellite_number));

    gnss_sim::RtklibSatelliteState state{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::get_rtklib_satellite_state_with_selection_time(nav_store_, 2041, 179999.925, 2041, 180000.0,
                                                                         satellite_number, &state, &error_message))
        << error_message;

    nav_t reference_nav{};
    obs_t reference_obs{};
    sta_t reference_station{};
    const std::string reference_path = rtklib_reference_nav_path();
    ASSERT_NE(readrnx(reference_path.c_str(), 1, "", &reference_obs, &reference_nav, &reference_station), 0);
    freeobs(&reference_obs);
    uniqnav(&reference_nav);

    const gtime_t state_time = gpst2time(2041, 179999.925);
    const gtime_t selection_time = gpst2time(2041, 180000.0);
    double reference_state[6]{};
    double reference_clock[2]{};
    double reference_variance_m2 = 0.0;
    int reference_health = 0;
    ASSERT_NE(satpos(state_time, selection_time, satellite_number, EPHOPT_BRDC, &reference_nav, reference_state,
                     reference_clock, &reference_variance_m2, &reference_health),
              0);

    for (int index = 0; index < 3; ++index) {
        EXPECT_NEAR(state.position_ecef_m[index], reference_state[index], 1.0e-6);
        EXPECT_NEAR(state.velocity_ecef_mps[index], reference_state[index + 3], 1.0e-9);
    }
    EXPECT_NEAR(state.clock_bias_sec, reference_clock[0], 1.0e-15);
    EXPECT_NEAR(state.clock_drift_sec_per_sec, reference_clock[1], 1.0e-18);
    EXPECT_EQ(state.health, reference_health);
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
