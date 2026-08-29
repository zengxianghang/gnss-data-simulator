#include "gnss/rtklib_adapter.h"
#include "gnss/satellite_engine.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "model/atmosphere_model.h"
#include "model/measurement_model.h"
#include "model/receiver_truth.h"
#include "model/signal_tracking.h"

#include <gtest/gtest.h>
#include <string>

namespace {

constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kRadiansToDegrees = 180.0 / 3.141592653589793238462643383279502884;

std::string brd4_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

struct NavGuard {
    gnss_sim::RtklibNavStore* store;
    ~NavGuard() {
        gnss_sim::destroy_rtklib_nav_store(store);
    }
};

gnss_sim::SignalTracker tracking_tracker(gnss_sim::SignalId signal_id, const gnss_sim::SimTime& tracking_start) {
    gnss_sim::SignalTracker tracker{};
    tracker.signal_id = signal_id;
    tracker.phase = gnss_sim::SignalTrackingPhase::kTracking;
    tracker.tracking_start_time = tracking_start;
    tracker.cn0_dbhz = 45.0;
    tracker.lock_time_ns = 10 * gnss_sim::NANOSECONDS_PER_SECOND;
    tracker.scheduled = true;
    tracker.psr_valid = true;
    tracker.doppler_valid = true;
    tracker.adr_valid = true;
    tracker.observation_available = true;
    return tracker;
}

TEST(ZeroNoiseMeasurement, RealWhuC35NearHorizonRecomputesAtmosphereFromFinalCodeState) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);

    std::string error_message;
    const std::string nav_path = brd4_nav_path();
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, nav_path.c_str(), &error_message)) << error_message;

    gnss_sim::ReceiverConfig receiver_config{};
    receiver_config.latitude_deg = 20.0;
    receiver_config.longitude_deg = 120.0;
    receiver_config.height_m = 100.0;
    gnss_sim::ReceiverTruth receiver{};
    ASSERT_TRUE(gnss_sim::make_static_receiver_truth(receiver_config, &receiver, &error_message)) << error_message;

    gnss_sim::SimTime receive_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 437021.0, &receive_time));

    int satellite_number = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("C35", &satellite_number));
    EXPECT_EQ(satellite_number, 144);

    gnss_sim::SatelliteGeometry generic_geometry{};
    ASSERT_TRUE(gnss_sim::compute_satellite_geometry(nav.store, receiver, receive_time, satellite_number, -90.0,
                                                     &generic_geometry, &error_message))
        << error_message;
    ASSERT_GT(generic_geometry.elevation_rad * kRadiansToDegrees, 0.0);
    ASSERT_LT(generic_geometry.elevation_rad * kRadiansToDegrees, 0.02);

    gnss_sim::AtmosphereCorrection supplied_atmosphere{};
    supplied_atmosphere.mode = gnss_sim::AtmosphereMode::BROADCAST;
    supplied_atmosphere.ionosphere_code_delay_m = 123.0;
    supplied_atmosphere.troposphere_delay_m = 456.0;

    const gnss_sim::SignalId signals[] = {gnss_sim::SignalId::kBeidouB1I, gnss_sim::SignalId::kBeidouB3I};
    for (gnss_sim::SignalId signal_id : signals) {
        const gnss_sim::SignalDefinition* signal = gnss_sim::find_signal_definition(signal_id);
        ASSERT_NE(signal, nullptr);

        int observation_code = 0;
        int frequency_index = 0;
        ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*signal, &observation_code, &frequency_index));
        static_cast<void>(frequency_index);

        gnss_sim::RtklibSatelliteState expected_state{};
        ASSERT_TRUE(gnss_sim::get_rtklib_signal_satellite_state(
            nav.store, generic_geometry.transmit_gps_week, generic_geometry.transmit_sow_sec, satellite_number,
            observation_code, gnss_sim::RtklibBroadcastMessageFamily::kLegacy, &expected_state, &error_message))
            << error_message;

        double expected_range_m = 0.0;
        double expected_line_of_sight[3]{};
        ASSERT_TRUE(gnss_sim::rtklib_geometric_distance(expected_state.position_ecef_m, receiver.position_ecef_m,
                                                        &expected_range_m, expected_line_of_sight));
        double expected_azimuth_rad = 0.0;
        double expected_elevation_rad = 0.0;
        ASSERT_TRUE(gnss_sim::rtklib_azimuth_elevation(receiver.position_ecef_m, expected_line_of_sight,
                                                       &expected_azimuth_rad, &expected_elevation_rad));
        ASSERT_GT(expected_elevation_rad * kRadiansToDegrees, 0.0);
        ASSERT_LT(expected_elevation_rad * kRadiansToDegrees, 0.02);

        gnss_sim::AtmosphereCorrection expected_atmosphere{};
        ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(
            gnss_sim::AtmosphereMode::BROADCAST, nav.store, receive_time, signal_id, 0, receiver.position_ecef_m,
            expected_azimuth_rad, expected_elevation_rad, &expected_atmosphere, &error_message))
            << error_message;

        gnss_sim::SignalTracker tracker = tracking_tracker(signal_id, receive_time);
        gnss_sim::CarrierAmbiguityState ambiguity{};
        gnss_sim::MeasurementObservation observation{};
        ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement(nav.store, generic_geometry, receiver, tracker,
                                                              supplied_atmosphere, &ambiguity, &observation,
                                                              &error_message))
            << error_message;

        EXPECT_TRUE(observation.pseudorange_valid);
        EXPECT_EQ(observation.broadcast_message_family, gnss_sim::RtklibBroadcastMessageFamily::kLegacy);
        EXPECT_NEAR(observation.geometric_range_m, expected_range_m, 1.0e-6);
        EXPECT_NEAR(observation.satellite_clock_bias_m, kSpeedOfLightMps * expected_state.clock_bias_sec, 1.0e-9);
        EXPECT_NEAR(observation.ionosphere_code_delay_m, expected_atmosphere.ionosphere_code_delay_m, 1.0e-9);
        EXPECT_NEAR(observation.troposphere_delay_m, expected_atmosphere.troposphere_delay_m, 1.0e-9);
        EXPECT_NE(observation.ionosphere_code_delay_m, supplied_atmosphere.ionosphere_code_delay_m);
        EXPECT_NE(observation.troposphere_delay_m, supplied_atmosphere.troposphere_delay_m);

        // Issue #63 changes the code geometry/atmosphere path only. Doppler
        // intentionally keeps the existing generic range-rate/clock-drift
        // convention validated by the independent residual tests.
        EXPECT_NEAR(observation.range_rate_mps, generic_geometry.range_rate_mps, 1.0e-12);
        EXPECT_NEAR(observation.satellite_clock_drift_mps,
                    kSpeedOfLightMps * generic_geometry.satellite_state.clock_drift_sec_per_sec, 1.0e-9);

        const double expected_pseudorange_m = expected_range_m - kSpeedOfLightMps * expected_state.clock_bias_sec +
                                              observation.code_bias_m + expected_atmosphere.ionosphere_code_delay_m +
                                              expected_atmosphere.troposphere_delay_m;
        EXPECT_NEAR(observation.pseudorange_m, expected_pseudorange_m, 1.0e-6);
    }
}

} // namespace
