#include "gnss/rtklib_adapter.h"
#include "gnss/satellite_engine.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "model/atmosphere_model.h"
#include "model/measurement_model.h"
#include "model/receiver_truth.h"
#include "model/signal_tracking.h"

#include <cmath>
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

TEST(ZeroNoiseMeasurement, NearHorizonRealWhuRecomputesAtmosphereFromFinalSignalGeometry) {
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

    constexpr int kBeidouSatelliteNumber = 144;
    char satellite_id[4]{};
    ASSERT_TRUE(gnss_sim::rtklib_satellite_number_to_id(kBeidouSatelliteNumber, satellite_id));
    ASSERT_EQ(satellite_id[0], 'C');

    gnss_sim::SatelliteGeometry generic_geometry{};
    ASSERT_TRUE(gnss_sim::compute_satellite_geometry(nav.store, receiver, receive_time, kBeidouSatelliteNumber, 0.0,
                                                     &generic_geometry, &error_message))
        << error_message;
    const double generic_elevation_deg = generic_geometry.elevation_rad * kRadiansToDegrees;
    ASSERT_GT(generic_elevation_deg, 0.0);
    ASSERT_LT(generic_elevation_deg, 0.02);

    const gnss_sim::SignalId signals[] = {gnss_sim::SignalId::kBeidouB1I, gnss_sim::SignalId::kBeidouB3I};
    for (gnss_sim::SignalId signal_id : signals) {
        gnss_sim::SatelliteGeometry geometry = generic_geometry;
        gnss_sim::AtmosphereCorrection generic_atmosphere{};
        ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(
            gnss_sim::AtmosphereMode::BROADCAST, nav.store, receive_time, signal_id, 0, receiver.position_ecef_m,
            geometry.azimuth_rad, geometry.elevation_rad, &generic_atmosphere, &error_message))
            << error_message;

        gnss_sim::SignalTracker tracker = tracking_tracker(signal_id, receive_time);
        gnss_sim::CarrierAmbiguityState ambiguity{};
        gnss_sim::MeasurementObservation observation{};
        ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement(nav.store, geometry, receiver, tracker,
                                                              generic_atmosphere, &ambiguity, &observation,
                                                              &error_message))
            << error_message;

        gnss_sim::AtmosphereCorrection expected_atmosphere{};
        ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(
            gnss_sim::AtmosphereMode::BROADCAST, nav.store, receive_time, signal_id, observation.glonass_fcn,
            receiver.position_ecef_m, geometry.azimuth_rad, geometry.elevation_rad, &expected_atmosphere,
            &error_message))
            << error_message;

        EXPECT_TRUE(observation.pseudorange_valid);
        EXPECT_LT(geometry.elevation_rad * kRadiansToDegrees, 0.02);
        EXPECT_NEAR(observation.geometric_range_m, geometry.geometric_range_m, 1.0e-9);
        EXPECT_NEAR(observation.range_rate_mps, geometry.range_rate_mps, 1.0e-12);
        EXPECT_NEAR(observation.satellite_clock_bias_m,
                    kSpeedOfLightMps * geometry.satellite_state.clock_bias_sec, 1.0e-9);
        EXPECT_NEAR(observation.satellite_clock_drift_mps,
                    kSpeedOfLightMps * geometry.satellite_state.clock_drift_sec_per_sec, 1.0e-9);
        EXPECT_NEAR(observation.ionosphere_code_delay_m, expected_atmosphere.ionosphere_code_delay_m, 1.0e-9);
        EXPECT_NEAR(observation.troposphere_delay_m, expected_atmosphere.troposphere_delay_m, 1.0e-9);

        // This real-WHU near-horizon epoch is intentionally selected because
        // the old generic-vs-signal-family LOS mismatch was strongly amplified
        // by the Saastamoinen mapping function. Keep evidence that the test
        // still exercises that regression instead of becoming a no-op.
        EXPECT_GT(std::fabs(expected_atmosphere.troposphere_delay_m - generic_atmosphere.troposphere_delay_m), 0.1);

        const double expected_pseudorange_m =
            geometry.geometric_range_m - kSpeedOfLightMps * geometry.satellite_state.clock_bias_sec +
            observation.code_bias_m + expected_atmosphere.ionosphere_code_delay_m +
            expected_atmosphere.troposphere_delay_m;
        EXPECT_NEAR(observation.pseudorange_m, expected_pseudorange_m, 1.0e-6);
    }
}

} // namespace
