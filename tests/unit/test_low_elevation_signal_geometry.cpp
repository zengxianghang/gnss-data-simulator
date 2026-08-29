#include "gnss/rtklib_adapter.h"
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
constexpr double kEarthRotationRateRadPerSec = 7.2921151467e-5;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kRadiansToDegrees = 180.0 / kPi;

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

bool range_and_angles(const gnss_sim::RtklibSatelliteState& state, const gnss_sim::ReceiverTruth& receiver,
                      double* range_m, double line_of_sight_ecef[3], double* azimuth_rad, double* elevation_rad) {
    return gnss_sim::rtklib_geometric_distance(state.position_ecef_m, receiver.position_ecef_m, range_m,
                                               line_of_sight_ecef) &&
           gnss_sim::rtklib_azimuth_elevation(receiver.position_ecef_m, line_of_sight_ecef, azimuth_rad, elevation_rad);
}

double static_receiver_range_rate_mps(const gnss_sim::RtklibSatelliteState& state,
                                      const gnss_sim::ReceiverTruth& receiver, const double line_of_sight_ecef[3]) {
    double rate_mps = 0.0;
    for (int index = 0; index < 3; ++index) {
        rate_mps += state.velocity_ecef_mps[index] * line_of_sight_ecef[index];
    }
    rate_mps += kEarthRotationRateRadPerSec / kSpeedOfLightMps *
                (state.velocity_ecef_mps[1] * receiver.position_ecef_m[0] -
                 state.velocity_ecef_mps[0] * receiver.position_ecef_m[1]);
    return rate_mps;
}

TEST(ZeroNoiseMeasurement, RealRinexFamilyMismatchUsesFinalCodeGeometryForAtmosphere) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);

    std::string error_message;
    const std::string nav_path = brd4_nav_path();
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, nav_path.c_str(), &error_message)) << error_message;

    constexpr int kGpsWeek = 2347;
    constexpr double kTransmitSowSec = 435600.0;
    constexpr double kReceiveSowSec = 435600.1;
    int satellite_number = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("C22", &satellite_number));

    const gnss_sim::SignalDefinition* modern_signal = gnss_sim::find_signal_definition(gnss_sim::SignalId::kBeidouB1C);
    const gnss_sim::SignalDefinition* legacy_signal = gnss_sim::find_signal_definition(gnss_sim::SignalId::kBeidouB1I);
    ASSERT_NE(modern_signal, nullptr);
    ASSERT_NE(legacy_signal, nullptr);

    int modern_observation_code = 0;
    int modern_frequency_index = 0;
    int legacy_observation_code = 0;
    int legacy_frequency_index = 0;
    ASSERT_TRUE(
        gnss_sim::signal_rtklib_observation_code(*modern_signal, &modern_observation_code, &modern_frequency_index));
    ASSERT_TRUE(
        gnss_sim::signal_rtklib_observation_code(*legacy_signal, &legacy_observation_code, &legacy_frequency_index));
    static_cast<void>(modern_frequency_index);
    static_cast<void>(legacy_frequency_index);

    gnss_sim::RtklibSatelliteState modern_state{};
    gnss_sim::RtklibSatelliteState legacy_state{};
    ASSERT_TRUE(gnss_sim::get_rtklib_signal_satellite_state(
        nav.store, kGpsWeek, kTransmitSowSec, satellite_number, modern_observation_code,
        gnss_sim::RtklibBroadcastMessageFamily::kBeidouBcnav1, &modern_state, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::get_rtklib_signal_satellite_state(
        nav.store, kGpsWeek, kTransmitSowSec, satellite_number, legacy_observation_code,
        gnss_sim::RtklibBroadcastMessageFamily::kLegacy, &legacy_state, &error_message))
        << error_message;

    gnss_sim::SimTime receive_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(kGpsWeek, kReceiveSowSec, &receive_time));

    gnss_sim::ReceiverTruth receiver{};
    double modern_range_m = 0.0;
    double modern_line_of_sight[3]{};
    double modern_azimuth_rad = 0.0;
    double modern_elevation_rad = 0.0;
    double legacy_range_m = 0.0;
    double legacy_line_of_sight[3]{};
    double legacy_azimuth_rad = 0.0;
    double legacy_elevation_rad = 0.0;
    gnss_sim::AtmosphereCorrection modern_atmosphere{};
    gnss_sim::AtmosphereCorrection legacy_atmosphere{};
    bool found_low_elevation_case = false;

    for (double latitude_deg = -60.0; latitude_deg <= 60.0 && !found_low_elevation_case; latitude_deg += 10.0) {
        for (double longitude_deg = -180.0; longitude_deg < 180.0; longitude_deg += 0.25) {
            gnss_sim::ReceiverConfig candidate_config{};
            candidate_config.latitude_deg = latitude_deg;
            candidate_config.longitude_deg = longitude_deg;
            candidate_config.height_m = 0.0;
            gnss_sim::ReceiverTruth candidate{};
            if (!gnss_sim::make_static_receiver_truth(candidate_config, &candidate, &error_message) ||
                !range_and_angles(modern_state, candidate, &modern_range_m, modern_line_of_sight, &modern_azimuth_rad,
                                  &modern_elevation_rad) ||
                !range_and_angles(legacy_state, candidate, &legacy_range_m, legacy_line_of_sight, &legacy_azimuth_rad,
                                  &legacy_elevation_rad)) {
                continue;
            }

            const double modern_elevation_deg = modern_elevation_rad * kRadiansToDegrees;
            const double legacy_elevation_deg = legacy_elevation_rad * kRadiansToDegrees;
            if (modern_elevation_deg <= 0.0 || modern_elevation_deg > 2.0 || legacy_elevation_deg <= 0.0 ||
                legacy_elevation_deg > 2.0) {
                continue;
            }
            if (!gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::BROADCAST, nav.store, receive_time,
                                                         gnss_sim::SignalId::kBeidouB1I, 0, candidate.position_ecef_m,
                                                         modern_azimuth_rad, modern_elevation_rad, &modern_atmosphere,
                                                         &error_message) ||
                !gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::BROADCAST, nav.store, receive_time,
                                                         gnss_sim::SignalId::kBeidouB1I, 0, candidate.position_ecef_m,
                                                         legacy_azimuth_rad, legacy_elevation_rad, &legacy_atmosphere,
                                                         &error_message)) {
                continue;
            }
            if (std::fabs(legacy_atmosphere.troposphere_delay_m - modern_atmosphere.troposphere_delay_m) <= 0.1) {
                continue;
            }

            receiver = candidate;
            found_low_elevation_case = true;
            break;
        }
    }
    ASSERT_TRUE(found_low_elevation_case)
        << "real C22 D1/CNV1 records did not produce a valid low-elevation family-mismatch case";

    gnss_sim::SatelliteGeometry generic_geometry{};
    generic_geometry.receive_time = receive_time;
    generic_geometry.transmit_gps_week = kGpsWeek;
    generic_geometry.transmit_sow_sec = kTransmitSowSec;
    generic_geometry.satellite_number = satellite_number;
    generic_geometry.satellite_state = modern_state;
    generic_geometry.geometric_range_m = modern_range_m;
    generic_geometry.range_rate_mps = static_receiver_range_rate_mps(modern_state, receiver, modern_line_of_sight);
    generic_geometry.propagation_time_sec = modern_range_m / kSpeedOfLightMps;
    generic_geometry.azimuth_rad = modern_azimuth_rad;
    generic_geometry.elevation_rad = modern_elevation_rad;
    generic_geometry.iteration_count = 1;
    generic_geometry.healthy = modern_state.health == 0;
    generic_geometry.above_elevation_mask = modern_elevation_rad > 0.0;
    generic_geometry.visible = generic_geometry.healthy && generic_geometry.above_elevation_mask;
    for (int index = 0; index < 3; ++index) {
        generic_geometry.line_of_sight_ecef[index] = modern_line_of_sight[index];
    }

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
            nav.store, kGpsWeek, kTransmitSowSec, satellite_number, observation_code,
            gnss_sim::RtklibBroadcastMessageFamily::kLegacy, &expected_state, &error_message))
            << error_message;
        double expected_range_m = 0.0;
        double expected_line_of_sight[3]{};
        double expected_azimuth_rad = 0.0;
        double expected_elevation_rad = 0.0;
        ASSERT_TRUE(range_and_angles(expected_state, receiver, &expected_range_m, expected_line_of_sight,
                                     &expected_azimuth_rad, &expected_elevation_rad));
        ASSERT_GT(expected_elevation_rad, 0.0);

        gnss_sim::AtmosphereCorrection generic_atmosphere{};
        gnss_sim::AtmosphereCorrection expected_atmosphere{};
        ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(
            gnss_sim::AtmosphereMode::BROADCAST, nav.store, receive_time, signal_id, 0, receiver.position_ecef_m,
            generic_geometry.azimuth_rad, generic_geometry.elevation_rad, &generic_atmosphere, &error_message))
            << error_message;
        ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(
            gnss_sim::AtmosphereMode::BROADCAST, nav.store, receive_time, signal_id, 0, receiver.position_ecef_m,
            expected_azimuth_rad, expected_elevation_rad, &expected_atmosphere, &error_message))
            << error_message;
        ASSERT_GT(std::fabs(expected_atmosphere.troposphere_delay_m - generic_atmosphere.troposphere_delay_m), 0.1);

        gnss_sim::SignalTracker tracker = tracking_tracker(signal_id, receive_time);
        gnss_sim::CarrierAmbiguityState ambiguity{};
        gnss_sim::MeasurementObservation observation{};
        ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement(nav.store, generic_geometry, receiver, tracker,
                                                              generic_atmosphere, &ambiguity, &observation,
                                                              &error_message))
            << error_message;

        EXPECT_TRUE(observation.pseudorange_valid);
        EXPECT_EQ(observation.broadcast_message_family, gnss_sim::RtklibBroadcastMessageFamily::kLegacy);
        EXPECT_NEAR(observation.geometric_range_m, expected_range_m, 1.0e-6);
        EXPECT_NEAR(observation.satellite_clock_bias_m, kSpeedOfLightMps * expected_state.clock_bias_sec, 1.0e-9);
        EXPECT_NEAR(observation.ionosphere_code_delay_m, expected_atmosphere.ionosphere_code_delay_m, 1.0e-9);
        EXPECT_NEAR(observation.troposphere_delay_m, expected_atmosphere.troposphere_delay_m, 1.0e-9);

        // Issue #63 is a code/atmosphere consistency fix. Doppler keeps the
        // existing generic broadcast range-rate and clock-drift convention.
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
