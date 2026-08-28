#include "gnss/rtklib_adapter.h"
#include "gnss/satellite_engine.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_time.h"
#include "model/atmosphere_model.h"
#include "model/measurement_model.h"
#include "model/receiver_truth.h"

#include <cmath>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kGpsL1Hz = 1575.42e6;
constexpr double kGpsL2Hz = 1227.60e6;
constexpr double kGalE1Hz = 1575.42e6;
constexpr double kGalE5aHz = 1176.45e6;
constexpr double kGalE5bHz = 1207.14e6;

std::string mixed_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/mixed_nav_2019.rnx";
}

struct NavGuard {
    gnss_sim::RtklibNavStore* store;
    ~NavGuard() {
        gnss_sim::destroy_rtklib_nav_store(store);
    }
};

const gnss_sim::SignalDefinition& signal(gnss_sim::SignalId signal_id) {
    const gnss_sim::SignalDefinition* definition = gnss_sim::find_signal_definition(signal_id);
    EXPECT_NE(definition, nullptr);
    return *definition;
}

bool make_test_receiver(gnss_sim::ReceiverTruth* receiver, std::string* error_message) {
    gnss_sim::ReceiverConfig config{};
    config.latitude_deg = 20.0;
    config.longitude_deg = 120.0;
    config.height_m = 100.0;
    return gnss_sim::make_static_receiver_truth(config, receiver, error_message);
}

gnss_sim::RtklibBroadcastBiasData bias(gnss_sim::RtklibBroadcastMessageFamily family) {
    gnss_sim::RtklibBroadcastBiasData value{};
    value.message_family = family;
    value.glonass_fcn = -5;
    value.tgd_sec[0] = 10.0e-9;
    value.tgd_sec[1] = -4.0e-9;
    value.isc_sec[0] = 1.0e-9;
    value.isc_sec[1] = 2.0e-9;
    value.isc_sec[3] = 4.0e-9;
    value.isc_sec[5] = 6.0e-9;
    value.glonass_dtaun_sec = 3.0e-9;
    value.glonass_isc_l3ocp_sec = 25.0e-9;
    return value;
}

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

TEST(BroadcastCodeBias, GpsLegacyAndModernFollowIcdClockCorrections) {
    double code_bias_m = 0.0;
    gnss_sim::BroadcastCodeBiasStatus status{};
    std::string error_message;

    auto legacy = bias(gnss_sim::RtklibBroadcastMessageFamily::kLegacy);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGpsL1Ca), legacy, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_EQ(status, gnss_sim::BroadcastCodeBiasStatus::kApplied);
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * 10.0e-9, 1.0e-12);

    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGpsL2P), legacy, &code_bias_m,
                                                        &status, &error_message));
    const double gamma12 = (kGpsL1Hz / kGpsL2Hz) * (kGpsL1Hz / kGpsL2Hz);
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * gamma12 * 10.0e-9, 1.0e-12);

    auto cnav = bias(gnss_sim::RtklibBroadcastMessageFamily::kCnav);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGpsL2C), cnav, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * 8.0e-9, 1.0e-12);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGpsL5Q), cnav, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * 6.0e-9, 1.0e-12);

    auto cnav2 = bias(gnss_sim::RtklibBroadcastMessageFamily::kCnav2);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGpsL1C), cnav2, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * 4.0e-9, 1.0e-12);
}

TEST(BroadcastCodeBias, GalileoUsesClockFamilySpecificBgd) {
    double code_bias_m = 0.0;
    gnss_sim::BroadcastCodeBiasStatus status{};
    std::string error_message;

    auto inav = bias(gnss_sim::RtklibBroadcastMessageFamily::kGalileoInav);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGalileoE1), inav, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * -4.0e-9, 1.0e-12);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGalileoE5B), inav, &code_bias_m,
                                                        &status, &error_message));
    const double gamma15b = (kGalE1Hz / kGalE5bHz) * (kGalE1Hz / kGalE5bHz);
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * gamma15b * -4.0e-9, 1.0e-12);

    auto fnav = bias(gnss_sim::RtklibBroadcastMessageFamily::kGalileoFnav);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGalileoE5A), fnav, &code_bias_m,
                                                        &status, &error_message));
    const double gamma15a = (kGalE1Hz / kGalE5aHz) * (kGalE1Hz / kGalE5aHz);
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * gamma15a * 10.0e-9, 1.0e-12);

    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGalileoE6), inav, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_EQ(status, gnss_sim::BroadcastCodeBiasStatus::kUnavailableForMessageFamily);
    EXPECT_DOUBLE_EQ(code_bias_m, 0.0);
}

TEST(BroadcastCodeBias, BeidouAndGlonassUseTheirBroadcastDelayDefinitions) {
    double code_bias_m = 0.0;
    gnss_sim::BroadcastCodeBiasStatus status{};
    std::string error_message;

    auto legacy = bias(gnss_sim::RtklibBroadcastMessageFamily::kLegacy);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kBeidouB1I), legacy, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * 10.0e-9, 1.0e-12);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kBeidouB3I), legacy, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_EQ(status, gnss_sim::BroadcastCodeBiasStatus::kNoCorrection);
    EXPECT_DOUBLE_EQ(code_bias_m, 0.0);

    auto bcnav1 = bias(gnss_sim::RtklibBroadcastMessageFamily::kBeidouBcnav1);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kBeidouB1C), bcnav1, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * 10.0e-9, 1.0e-12);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kBeidouB2A), bcnav1, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * -4.0e-9, 1.0e-12);

    auto bcnav3 = bias(gnss_sim::RtklibBroadcastMessageFamily::kBeidouBcnav3);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kBeidouB2B), bcnav3, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * 10.0e-9, 1.0e-12);

    auto glonass = bias(gnss_sim::RtklibBroadcastMessageFamily::kGlonassFdma);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGlonassG1), glonass, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_EQ(status, gnss_sim::BroadcastCodeBiasStatus::kNoCorrection);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGlonassG2), glonass, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * 3.0e-9, 1.0e-12);

    auto glonass_l3oc = bias(gnss_sim::RtklibBroadcastMessageFamily::kGlonassL3Oc);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGlonassG3), glonass_l3oc,
                                                        &code_bias_m, &status, &error_message));
    EXPECT_EQ(status, gnss_sim::BroadcastCodeBiasStatus::kApplied);
    EXPECT_NEAR(code_bias_m, -kSpeedOfLightMps * 25.0e-9, 1.0e-12);
}

TEST(RtklibBroadcastBiasAdapter, SelectsLegacyTgdGalileoFamilyAndGlonassFcn) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, mixed_nav_path().c_str(), &error_message)) << error_message;

    int gps_sat = 0;
    int gal_sat = 0;
    int glo_sat = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &gps_sat));
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("E01", &gal_sat));
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("R26", &glo_sat));

    gnss_sim::RtklibBroadcastBiasData gps{};
    ASSERT_TRUE(gnss_sim::rtklib_broadcast_bias_data(nav.store, 2041, 180000.0, gps_sat, &gps, &error_message));
    EXPECT_EQ(gps.message_family, gnss_sim::RtklibBroadcastMessageFamily::kLegacy);
    EXPECT_NEAR(gps.tgd_sec[0], 5.587935447693e-09, 1.0e-20);

    gnss_sim::RtklibBroadcastBiasData gal{};
    ASSERT_TRUE(gnss_sim::rtklib_broadcast_bias_data(nav.store, 2041, 172800.0, gal_sat, &gal, &error_message));
    EXPECT_EQ(gal.message_family, gnss_sim::RtklibBroadcastMessageFamily::kGalileoInav);
    EXPECT_NEAR(gal.tgd_sec[0], -4.656612873077e-09, 1.0e-20);
    EXPECT_NEAR(gal.tgd_sec[1], -5.355104804039e-09, 1.0e-20);

    gnss_sim::RtklibBroadcastBiasData glo{};
    ASSERT_TRUE(gnss_sim::rtklib_broadcast_bias_data(nav.store, 2041, 258300.0, glo_sat, &glo, &error_message));
    EXPECT_EQ(glo.message_family, gnss_sim::RtklibBroadcastMessageFamily::kGlonassFdma);
    EXPECT_EQ(glo.glonass_fcn, -5);
}

TEST(ZeroNoiseMeasurement, ComponentsMatchReferenceEquationsAndAtmosphereSigns) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, mixed_nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::ReceiverTruth receiver{};
    ASSERT_TRUE(make_test_receiver(&receiver, &error_message));
    gnss_sim::SimTime receive_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180100.0, &receive_time));
    int satellite_number = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &satellite_number));
    gnss_sim::SatelliteGeometry geometry{};
    ASSERT_TRUE(gnss_sim::compute_satellite_geometry(nav.store, receiver, receive_time, satellite_number, -90.0,
                                                     &geometry, &error_message))
        << error_message;

    gnss_sim::SimTime tracking_start{};
    ASSERT_TRUE(gnss_sim::add_time_ns(receive_time, -10 * gnss_sim::NANOSECONDS_PER_SECOND, &tracking_start));
    gnss_sim::SignalTracker tracker = tracking_tracker(gnss_sim::SignalId::kGpsL1Ca, tracking_start);
    gnss_sim::AtmosphereCorrection atmosphere{};
    atmosphere.mode = gnss_sim::AtmosphereMode::BROADCAST;
    atmosphere.ionosphere_code_delay_m = 6.25;
    atmosphere.troposphere_delay_m = 2.1;
    gnss_sim::CarrierAmbiguityState ambiguity{};
    gnss_sim::MeasurementObservation observation{};
    ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement(nav.store, geometry, receiver, tracker, atmosphere,
                                                          &ambiguity, &observation, &error_message))
        << error_message;

    const double expected_clock_m = kSpeedOfLightMps * geometry.satellite_state.clock_bias_sec;
    const double expected_clock_drift_mps = kSpeedOfLightMps * geometry.satellite_state.clock_drift_sec_per_sec;
    const double expected_pr = geometry.geometric_range_m - expected_clock_m + observation.code_bias_m + 6.25 + 2.1;
    const double expected_doppler = -(geometry.range_rate_mps - expected_clock_drift_mps) / observation.wavelength_m;
    const double expected_adr =
        (geometry.geometric_range_m - expected_clock_m - 6.25 + 2.1) / observation.wavelength_m +
        static_cast<double>(observation.ambiguity_cycles);
    EXPECT_NEAR(observation.pseudorange_m, expected_pr, 1.0e-9);
    EXPECT_NEAR(observation.doppler_hz, expected_doppler, 1.0e-12);
    EXPECT_NEAR(observation.adr_cycles, expected_adr, 1.0e-8);
    EXPECT_TRUE(observation.pseudorange_valid);
    EXPECT_TRUE(observation.doppler_valid);
    EXPECT_TRUE(observation.adr_valid);
}

TEST(ZeroNoiseMeasurement, BroadcastHealthDoesNotInvalidateTrackedRawMeasurement) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, mixed_nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::ReceiverTruth receiver{};
    ASSERT_TRUE(make_test_receiver(&receiver, &error_message));
    gnss_sim::SimTime receive_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180100.0, &receive_time));
    int satellite_number = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &satellite_number));
    gnss_sim::SatelliteGeometry geometry{};
    ASSERT_TRUE(gnss_sim::compute_satellite_geometry(nav.store, receiver, receive_time, satellite_number, -90.0,
                                                     &geometry, &error_message))
        << error_message;
    ASSERT_TRUE(geometry.above_elevation_mask);
    geometry.healthy = false;
    geometry.visible = false;

    gnss_sim::SimTime tracking_start{};
    ASSERT_TRUE(gnss_sim::add_time_ns(receive_time, -10 * gnss_sim::NANOSECONDS_PER_SECOND, &tracking_start));
    gnss_sim::SignalTracker tracker = tracking_tracker(gnss_sim::SignalId::kGpsL1Ca, tracking_start);
    gnss_sim::AtmosphereCorrection atmosphere{};
    atmosphere.mode = gnss_sim::AtmosphereMode::BROADCAST;
    atmosphere.ionosphere_code_delay_m = 0.0;
    atmosphere.troposphere_delay_m = 0.0;
    gnss_sim::CarrierAmbiguityState ambiguity{};
    gnss_sim::MeasurementObservation observation{};
    ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement(nav.store, geometry, receiver, tracker, atmosphere,
                                                          &ambiguity, &observation, &error_message))
        << error_message;

    EXPECT_TRUE(observation.observation_available);
    EXPECT_TRUE(observation.pseudorange_valid);
    EXPECT_TRUE(observation.doppler_valid);
    EXPECT_TRUE(observation.adr_valid);
}

TEST(ZeroNoiseMeasurement, MultiFrequencySharesGeometryClockAndDiffersByWavelengthAndBias) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, mixed_nav_path().c_str(), &error_message));
    gnss_sim::ReceiverTruth receiver{};
    ASSERT_TRUE(make_test_receiver(&receiver, &error_message));
    gnss_sim::SimTime receive_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180100.0, &receive_time));
    int satellite_number = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &satellite_number));
    gnss_sim::SatelliteGeometry geometry{};
    ASSERT_TRUE(gnss_sim::compute_satellite_geometry(nav.store, receiver, receive_time, satellite_number, -90.0,
                                                     &geometry, &error_message));
    gnss_sim::AtmosphereCorrection atmosphere{};
    atmosphere.mode = gnss_sim::AtmosphereMode::NONE;
    gnss_sim::CarrierAmbiguityState l1_ambiguity{};
    gnss_sim::CarrierAmbiguityState l2_ambiguity{};
    gnss_sim::MeasurementObservation l1{};
    gnss_sim::MeasurementObservation l2{};
    gnss_sim::SignalTracker l1_tracker = tracking_tracker(gnss_sim::SignalId::kGpsL1Ca, receive_time);
    gnss_sim::SignalTracker l2_tracker = tracking_tracker(gnss_sim::SignalId::kGpsL2P, receive_time);
    ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement(nav.store, geometry, receiver, l1_tracker, atmosphere,
                                                          &l1_ambiguity, &l1, &error_message));
    ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement(nav.store, geometry, receiver, l2_tracker, atmosphere,
                                                          &l2_ambiguity, &l2, &error_message));
    EXPECT_DOUBLE_EQ(l1.geometric_range_m, l2.geometric_range_m);
    EXPECT_DOUBLE_EQ(l1.range_rate_mps, l2.range_rate_mps);
    EXPECT_DOUBLE_EQ(l1.satellite_clock_bias_m, l2.satellite_clock_bias_m);
    EXPECT_DOUBLE_EQ(l1.satellite_clock_drift_mps, l2.satellite_clock_drift_mps);
    EXPECT_NE(l1.wavelength_m, l2.wavelength_m);
    EXPECT_NE(l1.code_bias_m, l2.code_bias_m);
    EXPECT_NEAR(l2.pseudorange_m - l1.pseudorange_m, l2.code_bias_m - l1.code_bias_m, 1.0e-9);
}

TEST(ZeroNoiseMeasurement, GlonassFdmaUsesSatelliteFcnForWavelengthAndDoppler) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, mixed_nav_path().c_str(), &error_message));
    gnss_sim::ReceiverTruth receiver{};
    ASSERT_TRUE(make_test_receiver(&receiver, &error_message));
    gnss_sim::SimTime receive_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 258300.0, &receive_time));
    int satellite_number = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("R26", &satellite_number));
    gnss_sim::SatelliteGeometry geometry{};
    ASSERT_TRUE(gnss_sim::compute_satellite_geometry(nav.store, receiver, receive_time, satellite_number, -90.0,
                                                     &geometry, &error_message));
    gnss_sim::SignalTracker tracker = tracking_tracker(gnss_sim::SignalId::kGlonassG1, receive_time);
    gnss_sim::AtmosphereCorrection atmosphere{};
    atmosphere.mode = gnss_sim::AtmosphereMode::NONE;
    gnss_sim::CarrierAmbiguityState ambiguity{};
    gnss_sim::MeasurementObservation observation{};
    ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement(nav.store, geometry, receiver, tracker, atmosphere,
                                                          &ambiguity, &observation, &error_message));
    EXPECT_EQ(observation.glonass_fcn, -5);
    double expected_wavelength_m = 0.0;
    ASSERT_TRUE(gnss_sim::signal_wavelength_m(signal(gnss_sim::SignalId::kGlonassG1), -5, &expected_wavelength_m));
    EXPECT_DOUBLE_EQ(observation.wavelength_m, expected_wavelength_m);
    EXPECT_NE(observation.wavelength_m, kSpeedOfLightMps / 1602.0e6);
    EXPECT_NEAR(observation.doppler_hz,
                -(geometry.range_rate_mps - observation.satellite_clock_drift_mps) / expected_wavelength_m, 1.0e-12);
}

TEST(ZeroNoiseMeasurement, AdrAmbiguityIsContinuousResetsOnSignalOffAndChangesAfterReacquisition) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, mixed_nav_path().c_str(), &error_message));
    gnss_sim::ReceiverTruth receiver{};
    ASSERT_TRUE(make_test_receiver(&receiver, &error_message));
    int satellite_number = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &satellite_number));
    gnss_sim::SimTime t0{};
    gnss_sim::SimTime t1{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180100.0, &t0));
    ASSERT_TRUE(gnss_sim::add_time_ns(t0, gnss_sim::NANOSECONDS_PER_SECOND, &t1));
    gnss_sim::SatelliteGeometry g0{};
    gnss_sim::SatelliteGeometry g1{};
    ASSERT_TRUE(
        gnss_sim::compute_satellite_geometry(nav.store, receiver, t0, satellite_number, -90.0, &g0, &error_message));
    ASSERT_TRUE(
        gnss_sim::compute_satellite_geometry(nav.store, receiver, t1, satellite_number, -90.0, &g1, &error_message));
    gnss_sim::AtmosphereCorrection atmosphere{};
    atmosphere.mode = gnss_sim::AtmosphereMode::NONE;
    gnss_sim::SignalTracker tracker = tracking_tracker(gnss_sim::SignalId::kGpsL1Ca, t0);
    gnss_sim::CarrierAmbiguityState ambiguity{};
    gnss_sim::MeasurementObservation o0{};
    gnss_sim::MeasurementObservation o1{};
    ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement(nav.store, g0, receiver, tracker, atmosphere, &ambiguity, &o0,
                                                          &error_message));
    ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement(nav.store, g1, receiver, tracker, atmosphere, &ambiguity, &o1,
                                                          &error_message));
    EXPECT_EQ(o0.ambiguity_cycles, o1.ambiguity_cycles);
    const double carrier0 = g0.geometric_range_m - o0.satellite_clock_bias_m;
    const double carrier1 = g1.geometric_range_m - o1.satellite_clock_bias_m;
    EXPECT_NEAR(o1.adr_cycles - o0.adr_cycles, (carrier1 - carrier0) / o0.wavelength_m, 1.0e-7);

    gnss_sim::SignalTracker signal_off = tracker;
    signal_off.phase = gnss_sim::SignalTrackingPhase::kSignalOff;
    signal_off.psr_valid = false;
    signal_off.doppler_valid = false;
    signal_off.adr_valid = false;
    signal_off.observation_available = false;
    gnss_sim::MeasurementObservation off_observation{};
    ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement(nav.store, g1, receiver, signal_off, atmosphere, &ambiguity,
                                                          &off_observation, &error_message));
    EXPECT_FALSE(ambiguity.initialized);
    EXPECT_FALSE(off_observation.adr_valid);

    tracker.tracking_start_time = t1;
    gnss_sim::MeasurementObservation reacquired{};
    ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement(nav.store, g1, receiver, tracker, atmosphere, &ambiguity,
                                                          &reacquired, &error_message));
    EXPECT_NE(reacquired.ambiguity_cycles, o1.ambiguity_cycles);
}

} // namespace
