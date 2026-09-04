#include "gnss/signal_definitions.h"
#include "model/measurement_model.h"
#include "model/urban_carrier_temporal.h"
#include "model/urban_signal_epoch.h"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <string>

namespace {

gnss_sim::MeasurementObservation clean_observation(double wavelength_m) {
    gnss_sim::MeasurementObservation result{};
    result.signal_id = gnss_sim::SignalId::kGpsL1Ca;
    result.satellite_number = 7;
    result.glonass_fcn = 0;
    result.wavelength_m = wavelength_m;
    result.geometric_range_m = 22000000.0;
    result.range_rate_mps = -512.25;
    result.satellite_clock_bias_m = 13.5;
    result.satellite_clock_drift_mps = -0.125;
    result.broadcast_message_family = gnss_sim::RtklibBroadcastMessageFamily::kLegacy;
    result.tgd_sec[0] = 5.0e-9;
    result.isc_sec[0] = 2.0e-9;
    result.glonass_dtaun_sec = 3.0e-9;
    result.code_bias_m = 1.75;
    result.ionosphere_code_delay_m = 6.25;
    result.troposphere_delay_m = 2.1;
    result.pseudorange_m = 22000010.6;
    result.doppler_hz = 2692.5;
    result.adr_cycles = 115000123.25;
    result.cn0_dbhz = 45.0;
    result.lock_time_ns = 5000000000LL;
    result.ambiguity_cycles = 345678;
    result.code_bias_status = gnss_sim::BroadcastCodeBiasStatus::kApplied;
    result.observation_available = true;
    result.pseudorange_valid = true;
    result.doppler_valid = true;
    result.adr_valid = true;
    return result;
}

gnss_sim::UrbanSignalEpochResult tracked_epoch(double code_bias_m, double effective_cn0_dbhz,
                                               std::int64_t lock_time_ns) {
    gnss_sim::UrbanSignalEpochResult result{};
    result.tracking_phase = gnss_sim::SignalTrackingPhase::kTracking;
    result.code_bias_m = code_bias_m;
    result.effective_cn0_dbhz = effective_cn0_dbhz;
    result.lock_time_ns = lock_time_ns;
    result.selected_root_valid = true;
    result.tracked_composite_correlation_valid = true;
    result.observation_available = true;
    result.psr_valid = true;
    result.doppler_valid = true;
    result.adr_valid = true;
    result.carrier_continuity_valid = true;
    return result;
}

gnss_sim::UrbanCarrierTemporalResult tracked_temporal(double wavelength_m, double carrier_range_bias_m,
                                                      double environmental_range_rate_mps) {
    gnss_sim::UrbanCarrierTemporalResult result{};
    result.wavelength_m = wavelength_m;
    result.carrier_range_bias_m = carrier_range_bias_m;
    result.environmental_range_rate_mps = environmental_range_rate_mps;
    result.tracking_lock_valid = true;
    result.carrier_adr_valid = true;
    result.phase_continuity_valid = true;
    result.environmental_range_rate_valid = true;
    return result;
}

TEST(UrbanMeasurementApplication, AppliesCodeCarrierRateAndCn0ExactlyOnce) {
    constexpr double kWavelengthM = 0.20;
    gnss_sim::MeasurementObservation observation = clean_observation(kWavelengthM);
    const gnss_sim::MeasurementObservation clean = observation;
    const gnss_sim::UrbanSignalEpochResult epoch = tracked_epoch(4.5, 34.25, 8200000000LL);
    const gnss_sim::UrbanCarrierTemporalResult temporal = tracked_temporal(kWavelengthM, 0.32, 1.4);

    std::string error_message;
    ASSERT_TRUE(gnss_sim::apply_urban_measurement_effects(epoch, temporal, &observation, &error_message))
        << error_message;

    EXPECT_NEAR(observation.pseudorange_m - clean.pseudorange_m, 4.5, 1.0e-12);
    EXPECT_NEAR(observation.range_rate_mps - clean.range_rate_mps, 1.4, 1.0e-12);
    EXPECT_NEAR(observation.doppler_hz - clean.doppler_hz, -1.4 / kWavelengthM, 1.0e-12);
    EXPECT_NEAR(observation.adr_cycles - clean.adr_cycles, 0.32 / kWavelengthM, 1.0e-9);
    EXPECT_DOUBLE_EQ(observation.cn0_dbhz, 34.25);
    EXPECT_EQ(observation.lock_time_ns, 8200000000LL);
    EXPECT_TRUE(observation.observation_available);
    EXPECT_TRUE(observation.pseudorange_valid);
    EXPECT_TRUE(observation.doppler_valid);
    EXPECT_TRUE(observation.adr_valid);
}

TEST(UrbanMeasurementApplication, PreservesBroadcastAtmosphereClockAndAmbiguityDiagnostics) {
    constexpr double kWavelengthM = 0.20;
    gnss_sim::MeasurementObservation observation = clean_observation(kWavelengthM);
    const gnss_sim::MeasurementObservation clean = observation;
    const gnss_sim::UrbanSignalEpochResult epoch = tracked_epoch(3.0, 38.0, 6000000000LL);
    const gnss_sim::UrbanCarrierTemporalResult temporal = tracked_temporal(kWavelengthM, -0.11, -0.75);

    std::string error_message;
    ASSERT_TRUE(gnss_sim::apply_urban_measurement_effects(epoch, temporal, &observation, &error_message))
        << error_message;

    EXPECT_DOUBLE_EQ(observation.geometric_range_m, clean.geometric_range_m);
    EXPECT_DOUBLE_EQ(observation.satellite_clock_bias_m, clean.satellite_clock_bias_m);
    EXPECT_DOUBLE_EQ(observation.satellite_clock_drift_mps, clean.satellite_clock_drift_mps);
    EXPECT_EQ(observation.broadcast_message_family, clean.broadcast_message_family);
    EXPECT_DOUBLE_EQ(observation.tgd_sec[0], clean.tgd_sec[0]);
    EXPECT_DOUBLE_EQ(observation.isc_sec[0], clean.isc_sec[0]);
    EXPECT_DOUBLE_EQ(observation.glonass_dtaun_sec, clean.glonass_dtaun_sec);
    EXPECT_DOUBLE_EQ(observation.code_bias_m, clean.code_bias_m);
    EXPECT_EQ(observation.code_bias_status, clean.code_bias_status);
    EXPECT_DOUBLE_EQ(observation.ionosphere_code_delay_m, clean.ionosphere_code_delay_m);
    EXPECT_DOUBLE_EQ(observation.troposphere_delay_m, clean.troposphere_delay_m);
    EXPECT_EQ(observation.ambiguity_cycles, clean.ambiguity_cycles);
}

TEST(UrbanMeasurementApplication, FreshLockWithoutPathRateDoesNotInventDopplerCorrection) {
    constexpr double kWavelengthM = 0.20;
    gnss_sim::MeasurementObservation observation = clean_observation(kWavelengthM);
    const gnss_sim::MeasurementObservation clean = observation;
    const gnss_sim::UrbanSignalEpochResult epoch = tracked_epoch(0.8, 31.0, 0);
    gnss_sim::UrbanCarrierTemporalResult temporal = tracked_temporal(kWavelengthM, 0.07, 0.0);
    temporal.environmental_range_rate_valid = false;

    std::string error_message;
    ASSERT_TRUE(gnss_sim::apply_urban_measurement_effects(epoch, temporal, &observation, &error_message))
        << error_message;

    EXPECT_DOUBLE_EQ(observation.range_rate_mps, clean.range_rate_mps);
    EXPECT_DOUBLE_EQ(observation.doppler_hz, clean.doppler_hz);
    EXPECT_FALSE(observation.doppler_valid);
    EXPECT_TRUE(observation.adr_valid);
    EXPECT_NEAR(observation.adr_cycles - clean.adr_cycles, 0.07 / kWavelengthM, 1.0e-9);
}

TEST(UrbanMeasurementApplication, BlockedStateCannotLeakValidRangeObservation) {
    constexpr double kWavelengthM = 0.20;
    gnss_sim::MeasurementObservation observation = clean_observation(kWavelengthM);
    const gnss_sim::MeasurementObservation clean = observation;
    gnss_sim::UrbanSignalEpochResult epoch{};
    epoch.tracking_phase = gnss_sim::SignalTrackingPhase::kSearching;
    epoch.effective_cn0_dbhz = -std::numeric_limits<double>::infinity();
    epoch.lock_time_ns = 0;
    epoch.observation_available = false;
    epoch.psr_valid = false;
    epoch.doppler_valid = false;
    epoch.adr_valid = false;

    gnss_sim::UrbanCarrierTemporalResult temporal{};
    temporal.wavelength_m = kWavelengthM;

    std::string error_message;
    ASSERT_TRUE(gnss_sim::apply_urban_measurement_effects(epoch, temporal, &observation, &error_message))
        << error_message;

    EXPECT_FALSE(observation.observation_available);
    EXPECT_FALSE(observation.pseudorange_valid);
    EXPECT_FALSE(observation.doppler_valid);
    EXPECT_FALSE(observation.adr_valid);
    EXPECT_TRUE(std::isinf(observation.cn0_dbhz));
    EXPECT_LT(observation.cn0_dbhz, 0.0);
    EXPECT_DOUBLE_EQ(observation.pseudorange_m, clean.pseudorange_m);
    EXPECT_DOUBLE_EQ(observation.range_rate_mps, clean.range_rate_mps);
    EXPECT_DOUBLE_EQ(observation.doppler_hz, clean.doppler_hz);
    EXPECT_DOUBLE_EQ(observation.adr_cycles, clean.adr_cycles);
}

TEST(UrbanMeasurementApplication, CleanGeometryValidityRemainsAnUpperBound) {
    constexpr double kWavelengthM = 0.20;
    gnss_sim::MeasurementObservation observation = clean_observation(kWavelengthM);
    observation.observation_available = false;
    observation.pseudorange_valid = false;
    observation.doppler_valid = false;
    observation.adr_valid = false;
    const gnss_sim::UrbanSignalEpochResult epoch = tracked_epoch(0.5, 40.0, 7000000000LL);
    const gnss_sim::UrbanCarrierTemporalResult temporal = tracked_temporal(kWavelengthM, 0.02, 0.3);

    std::string error_message;
    ASSERT_TRUE(gnss_sim::apply_urban_measurement_effects(epoch, temporal, &observation, &error_message))
        << error_message;

    EXPECT_FALSE(observation.observation_available);
    EXPECT_FALSE(observation.pseudorange_valid);
    EXPECT_FALSE(observation.doppler_valid);
    EXPECT_FALSE(observation.adr_valid);
}

TEST(UrbanMeasurementApplication, NeutralUrbanTermsLeaveCleanObservableValuesUnchanged) {
    constexpr double kWavelengthM = 0.20;
    gnss_sim::MeasurementObservation observation = clean_observation(kWavelengthM);
    const gnss_sim::MeasurementObservation clean = observation;
    const gnss_sim::UrbanSignalEpochResult epoch = tracked_epoch(0.0, clean.cn0_dbhz, clean.lock_time_ns);
    const gnss_sim::UrbanCarrierTemporalResult temporal = tracked_temporal(kWavelengthM, 0.0, 0.0);

    std::string error_message;
    ASSERT_TRUE(gnss_sim::apply_urban_measurement_effects(epoch, temporal, &observation, &error_message))
        << error_message;

    EXPECT_DOUBLE_EQ(observation.pseudorange_m, clean.pseudorange_m);
    EXPECT_DOUBLE_EQ(observation.range_rate_mps, clean.range_rate_mps);
    EXPECT_DOUBLE_EQ(observation.doppler_hz, clean.doppler_hz);
    EXPECT_DOUBLE_EQ(observation.adr_cycles, clean.adr_cycles);
    EXPECT_DOUBLE_EQ(observation.cn0_dbhz, clean.cn0_dbhz);
    EXPECT_EQ(observation.lock_time_ns, clean.lock_time_ns);
}

TEST(UrbanMeasurementApplication, SignalSpecificWavelengthControlsDopplerAndAdrMapping) {
    const gnss_sim::SignalDefinition* gps_l5 = gnss_sim::find_signal_definition(gnss_sim::SignalId::kGpsL5Q);
    ASSERT_NE(gps_l5, nullptr);
    double wavelength_m = 0.0;
    ASSERT_TRUE(gnss_sim::signal_wavelength_m(*gps_l5, 0, &wavelength_m));

    gnss_sim::MeasurementObservation observation = clean_observation(wavelength_m);
    observation.signal_id = gnss_sim::SignalId::kGpsL5Q;
    const gnss_sim::MeasurementObservation clean = observation;
    const gnss_sim::UrbanSignalEpochResult epoch = tracked_epoch(0.0, 42.0, 9000000000LL);
    const gnss_sim::UrbanCarrierTemporalResult temporal = tracked_temporal(wavelength_m, 0.25 * wavelength_m, 2.0);

    std::string error_message;
    ASSERT_TRUE(gnss_sim::apply_urban_measurement_effects(epoch, temporal, &observation, &error_message))
        << error_message;

    EXPECT_NEAR(observation.doppler_hz - clean.doppler_hz, -2.0 / wavelength_m, 1.0e-12);
    EXPECT_NEAR(observation.adr_cycles - clean.adr_cycles, 0.25, 1.0e-9);
}

TEST(UrbanMeasurementApplication, RejectsMismatchedTemporalEpochPairWithoutMutation) {
    constexpr double kWavelengthM = 0.20;
    gnss_sim::MeasurementObservation observation = clean_observation(kWavelengthM);
    const gnss_sim::MeasurementObservation clean = observation;
    const gnss_sim::UrbanSignalEpochResult epoch = tracked_epoch(1.0, 35.0, 5000000000LL);
    gnss_sim::UrbanCarrierTemporalResult temporal = tracked_temporal(kWavelengthM, 0.1, 0.4);
    temporal.tracking_lock_valid = false;

    std::string error_message;
    EXPECT_FALSE(gnss_sim::apply_urban_measurement_effects(epoch, temporal, &observation, &error_message));
    EXPECT_FALSE(error_message.empty());
    EXPECT_DOUBLE_EQ(observation.pseudorange_m, clean.pseudorange_m);
    EXPECT_DOUBLE_EQ(observation.doppler_hz, clean.doppler_hz);
    EXPECT_DOUBLE_EQ(observation.adr_cycles, clean.adr_cycles);
}

} // namespace
