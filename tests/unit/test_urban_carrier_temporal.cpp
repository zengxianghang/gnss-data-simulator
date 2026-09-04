#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "model/urban_carrier_temporal.h"

#include <cmath>
#include <complex>
#include <cstdint>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kSyntheticSatelliteDistanceM = 20000000.0;
constexpr std::int64_t kMilliseconds = 1000000LL;

const gnss_sim::SignalDefinition& signal(gnss_sim::SignalId signal_id) {
    const gnss_sim::SignalDefinition* definition = gnss_sim::find_signal_definition(signal_id);
    EXPECT_NE(definition, nullptr);
    return *definition;
}

gnss_sim::SimTime time_at(double sow_sec) {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2400, sow_sec, &time));
    return time;
}

std::complex<double> phasor_for_range_bias(double range_bias_m, double wavelength_m) {
    return std::polar(1.0, -kTwoPi * range_bias_m / wavelength_m);
}

gnss_sim::UrbanSignalEpochResult tracked_epoch(const std::complex<double>& phasor, std::int64_t lock_time_ns,
                                               bool reacquisition_event = false, double code_bias_m = 0.0) {
    gnss_sim::UrbanSignalEpochResult epoch{};
    epoch.tracking_phase = gnss_sim::SignalTrackingPhase::kTracking;
    epoch.selected_root_valid = true;
    epoch.tracked_composite_correlation_valid = true;
    epoch.tracked_composite_correlation = phasor;
    epoch.code_bias_m = code_bias_m;
    epoch.lock_time_ns = lock_time_ns;
    epoch.adr_valid = true;
    epoch.carrier_continuity_valid = true;
    epoch.reacquisition_event = reacquisition_event;
    return epoch;
}

gnss_sim::UrbanSignalEpochResult lost_epoch() {
    gnss_sim::UrbanSignalEpochResult epoch{};
    epoch.tracking_phase = gnss_sim::SignalTrackingPhase::kSearching;
    epoch.selected_root_valid = false;
    epoch.tracked_composite_correlation_valid = false;
    epoch.lock_time_ns = 0;
    return epoch;
}

gnss_sim::DelayDistribution fixed_delay(std::int64_t delay_ns) {
    return {delay_ns, delay_ns, delay_ns, delay_ns};
}

gnss_sim::SignalTrackingModelConfig immediate_tracking_config() {
    gnss_sim::SignalTrackingModelConfig config = gnss_sim::default_signal_tracking_model_config();
    config.hot_common_startup = fixed_delay(0);
    config.warm_common_startup = fixed_delay(0);
    config.warm_search_uncertainty = fixed_delay(0);
    for (int index = 0; index < 4; ++index) {
        config.hot_signal_acquisition[index] = fixed_delay(0);
        config.reacquisition[index] = fixed_delay(0);
    }
    config.acquisition_cn0_persistence_ns = 0;
    config.psr_valid_delay_ns = 0;
    config.doppler_valid_delay_ns = 0;
    config.adr_valid_delay_ns = 0;
    return config;
}

gnss_sim::Cn0Model normalized_model(gnss_sim::SignalId signal_id, double high_cn0_dbhz) {
    gnss_sim::Cn0Model model{};
    model.source = gnss_sim::Cn0ModelSource::kCalibratedCsv;
    model.semantic = gnss_sim::Cn0ModelSemantic::kNormalizedElevationShape;
    model.seed = 41;
    gnss_sim::Cn0CalibratedBin bin{};
    bin.signal_id = signal_id;
    bin.elevation_min_deg = 0.0;
    bin.elevation_max_deg = 90.0;
    bin.elevation_center_deg = 45.0;
    bin.delta_p50_db = -2.0;
    bin.support_count = 8;
    bin.upper_edge_inclusive = true;
    bin.ready = true;
    model.calibrated_bins.push_back(bin);
    model.high_baselines.push_back({signal_id, high_cn0_dbhz});
    return model;
}

void schedule_tracker(gnss_sim::SignalTracker* tracker, const gnss_sim::SignalDefinition& definition,
                      const gnss_sim::SimTime& time, double elevation_deg, const gnss_sim::Cn0Model& cn0_model,
                      const gnss_sim::SignalTrackingModelConfig& config) {
    gnss_sim::reset_signal_tracker(tracker, definition.signal_id, time);
    gnss_sim::DeterministicRng rng{};
    gnss_sim::seed_rng(&rng, 41U);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::schedule_signal_acquisition(tracker, gnss_sim::AcquisitionContext::kHot, time, time,
                                                      elevation_deg, cn0_model, config, &rng, &error_message))
        << error_message;
}

void make_synthetic_geometry(double azimuth_deg, double elevation_deg, gnss_sim::ReceiverTruth* receiver,
                             gnss_sim::SatelliteGeometry* geometry) {
    ASSERT_NE(receiver, nullptr);
    ASSERT_NE(geometry, nullptr);
    *receiver = {};
    *geometry = {};
    geometry->satellite_state.position_ecef_m[0] = kSyntheticSatelliteDistanceM;
    geometry->azimuth_rad = azimuth_deg * kDegreesToRadians;
    geometry->elevation_rad = elevation_deg * kDegreesToRadians;
    geometry->geometric_range_m = kSyntheticSatelliteDistanceM;
    geometry->healthy = true;
    geometry->above_elevation_mask = true;
    geometry->visible = true;
}

TEST(UrbanCarrierTemporal, SingleMovingPathRateMatchesAnalyticRangeChange) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    double wavelength_m = 0.0;
    ASSERT_TRUE(gnss_sim::signal_wavelength_m(gps_l1, 0, &wavelength_m));
    const double first_bias_m = 0.10 * wavelength_m;
    const double second_bias_m = 0.20 * wavelength_m;
    const double dt_sec = 0.1;

    gnss_sim::UrbanCarrierTemporalState state{};
    gnss_sim::UrbanCarrierTemporalResult first{};
    gnss_sim::UrbanCarrierTemporalResult second{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(
        gps_l1, 0, time_at(100000.0), tracked_epoch(phasor_for_range_bias(first_bias_m, wavelength_m), 1000 * kMilliseconds),
        &state, &first, &error_message))
        << error_message;
    EXPECT_TRUE(first.phase_continuity_valid);
    EXPECT_FALSE(first.environmental_range_rate_valid);
    EXPECT_NEAR(first.carrier_range_bias_m, first_bias_m, 1.0e-12);

    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(
        gps_l1, 0, time_at(100000.1),
        tracked_epoch(phasor_for_range_bias(second_bias_m, wavelength_m), 1100 * kMilliseconds), &state, &second,
        &error_message))
        << error_message;
    EXPECT_TRUE(second.environmental_range_rate_valid);
    EXPECT_NEAR(second.carrier_range_bias_m, second_bias_m, 1.0e-12);
    EXPECT_NEAR(second.environmental_range_rate_mps, (second_bias_m - first_bias_m) / dt_sec, 1.0e-10);
}

TEST(UrbanCarrierTemporal, WrappedPhaseCrossingPiRemainsContinuous) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    double wavelength_m = 0.0;
    ASSERT_TRUE(gnss_sim::signal_wavelength_m(gps_l1, 0, &wavelength_m));
    const double first_bias_m = 0.49 * wavelength_m;
    const double second_bias_m = 0.51 * wavelength_m;

    gnss_sim::UrbanCarrierTemporalState state{};
    gnss_sim::UrbanCarrierTemporalResult first{};
    gnss_sim::UrbanCarrierTemporalResult second{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(
        gps_l1, 0, time_at(110000.0), tracked_epoch(phasor_for_range_bias(first_bias_m, wavelength_m), 1000 * kMilliseconds),
        &state, &first, &error_message));
    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(
        gps_l1, 0, time_at(110000.1),
        tracked_epoch(phasor_for_range_bias(second_bias_m, wavelength_m), 1100 * kMilliseconds), &state, &second,
        &error_message));

    EXPECT_LT(first.wrapped_phase_rad, 0.0);
    EXPECT_GT(second.wrapped_phase_rad, 0.0);
    EXPECT_LT(second.unwrapped_phase_rad, -kPi);
    EXPECT_NEAR(second.carrier_range_bias_m, second_bias_m, 1.0e-12);
}

TEST(UrbanCarrierTemporal, StableCodeBiasDoesNotCreateDopplerBias) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    double wavelength_m = 0.0;
    ASSERT_TRUE(gnss_sim::signal_wavelength_m(gps_l1, 0, &wavelength_m));
    const std::complex<double> phasor = phasor_for_range_bias(0.12 * wavelength_m, wavelength_m);

    gnss_sim::UrbanCarrierTemporalState state{};
    gnss_sim::UrbanCarrierTemporalResult result{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(
        gps_l1, 0, time_at(120000.0), tracked_epoch(phasor, 1000 * kMilliseconds, false, 20.0), &state, &result,
        &error_message));
    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(
        gps_l1, 0, time_at(120000.2), tracked_epoch(phasor, 1200 * kMilliseconds, false, 45.0), &state, &result,
        &error_message));
    EXPECT_TRUE(result.environmental_range_rate_valid);
    EXPECT_NEAR(result.environmental_range_rate_mps, 0.0, 1.0e-14);
}

TEST(UrbanCarrierTemporal, LossAndReacquisitionStartNewContinuitySegment) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    double wavelength_m = 0.0;
    ASSERT_TRUE(gnss_sim::signal_wavelength_m(gps_l1, 0, &wavelength_m));

    gnss_sim::UrbanCarrierTemporalState state{};
    gnss_sim::UrbanCarrierTemporalResult result{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(
        gps_l1, 0, time_at(130000.0),
        tracked_epoch(phasor_for_range_bias(0.1 * wavelength_m, wavelength_m), 1000 * kMilliseconds), &state, &result,
        &error_message));

    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(gps_l1, 0, time_at(130000.1), lost_epoch(), &state,
                                                              &result, &error_message));
    EXPECT_TRUE(result.cycle_slip_event);
    EXPECT_FALSE(result.phase_continuity_valid);
    EXPECT_FALSE(result.environmental_range_rate_valid);

    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(
        gps_l1, 0, time_at(130000.2),
        tracked_epoch(phasor_for_range_bias(0.4 * wavelength_m, wavelength_m), 0, true), &state, &result,
        &error_message));
    EXPECT_TRUE(result.cycle_slip_event);
    EXPECT_TRUE(result.phase_continuity_valid);
    EXPECT_FALSE(result.environmental_range_rate_valid);
}

TEST(UrbanCarrierTemporal, FreshLockInsideSkippedGapIsNeverDifferentiated) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    double wavelength_m = 0.0;
    ASSERT_TRUE(gnss_sim::signal_wavelength_m(gps_l1, 0, &wavelength_m));

    gnss_sim::UrbanCarrierTemporalState state{};
    gnss_sim::UrbanCarrierTemporalResult result{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(
        gps_l1, 0, time_at(140000.0),
        tracked_epoch(phasor_for_range_bias(0.1 * wavelength_m, wavelength_m), 5000 * kMilliseconds), &state, &result,
        &error_message));
    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(
        gps_l1, 0, time_at(140001.0),
        tracked_epoch(phasor_for_range_bias(0.4 * wavelength_m, wavelength_m), 200 * kMilliseconds), &state, &result,
        &error_message));
    EXPECT_TRUE(result.cycle_slip_event);
    EXPECT_FALSE(result.environmental_range_rate_valid);
}

TEST(UrbanCarrierTemporal, DifferentCarrierWavelengthUsesCentralSignalMetadata) {
    const gnss_sim::SignalDefinition& gal_e5a = signal(gnss_sim::SignalId::kGalileoE5A);
    double wavelength_m = 0.0;
    ASSERT_TRUE(gnss_sim::signal_wavelength_m(gal_e5a, 0, &wavelength_m));
    const double first_bias_m = 0.08 * wavelength_m;
    const double second_bias_m = 0.18 * wavelength_m;

    gnss_sim::UrbanCarrierTemporalState state{};
    gnss_sim::UrbanCarrierTemporalResult result{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(
        gal_e5a, 0, time_at(150000.0), tracked_epoch(phasor_for_range_bias(first_bias_m, wavelength_m), 1000 * kMilliseconds),
        &state, &result, &error_message));
    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(
        gal_e5a, 0, time_at(150000.1),
        tracked_epoch(phasor_for_range_bias(second_bias_m, wavelength_m), 1100 * kMilliseconds), &state, &result,
        &error_message));
    EXPECT_NEAR(result.wavelength_m, wavelength_m, 1.0e-15);
    EXPECT_NEAR(result.carrier_range_bias_m, second_bias_m, 1.0e-12);
}

TEST(UrbanCarrierTemporal, ProductionRoofTransitionProducesTimeCorrelatedPathRateFromSameEpochResult) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig sim_config = gnss_sim::default_sim_config();
    const gnss_sim::SignalTrackingModelConfig tracking_config = immediate_tracking_config();
    const gnss_sim::Cn0Model cn0 = normalized_model(gps_l1.signal_id, 55.0);
    const gnss_sim::CodeTrackingDllConfig dll_config = gnss_sim::default_code_tracking_dll_config();
    double skyline_rad = 0.0;
    double wall_distance_m = 0.0;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_skyline_elevation(scene, 0.0, &skyline_rad, &wall_distance_m, &error_message));

    const double first_elevation_deg = skyline_rad / kDegreesToRadians + 0.001;
    const double second_elevation_deg = skyline_rad / kDegreesToRadians + 0.002;
    const gnss_sim::SimTime first_time = time_at(160000.0);
    const gnss_sim::SimTime second_time = time_at(160000.1);
    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry first_geometry{};
    gnss_sim::SatelliteGeometry second_geometry{};
    make_synthetic_geometry(0.0, first_elevation_deg, &receiver, &first_geometry);
    make_synthetic_geometry(0.0, second_elevation_deg, &receiver, &second_geometry);
    gnss_sim::SignalTracker tracker{};
    schedule_tracker(&tracker, gps_l1, first_time, first_elevation_deg, cn0, tracking_config);

    gnss_sim::UrbanSignalEpochResult first_epoch{};
    gnss_sim::UrbanSignalEpochResult second_epoch{};
    ASSERT_TRUE(gnss_sim::compute_urban_signal_epoch(cn0, scene, sim_config.urban_rf, dll_config, tracking_config,
                                                     gps_l1, 0, first_time, receiver, first_geometry, &tracker,
                                                     &first_epoch, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_signal_epoch(cn0, scene, sim_config.urban_rf, dll_config, tracking_config,
                                                     gps_l1, 0, second_time, receiver, second_geometry, &tracker,
                                                     &second_epoch, &error_message))
        << error_message;
    ASSERT_EQ(first_epoch.urban_state, gnss_sim::UrbanSignalState::kLosMultipath);
    ASSERT_EQ(second_epoch.urban_state, gnss_sim::UrbanSignalState::kLosMultipath);

    gnss_sim::UrbanCarrierTemporalState state{};
    gnss_sim::UrbanCarrierTemporalResult first_result{};
    gnss_sim::UrbanCarrierTemporalResult second_result{};
    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(gps_l1, 0, first_time, first_epoch, &state, &first_result,
                                                              &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(gps_l1, 0, second_time, second_epoch, &state,
                                                              &second_result, &error_message))
        << error_message;
    EXPECT_TRUE(second_result.environmental_range_rate_valid);
    EXPECT_TRUE(std::isfinite(second_result.environmental_range_rate_mps));
    EXPECT_GT(std::abs(second_result.environmental_range_rate_mps), 1.0e-12);
}

TEST(UrbanCarrierTemporal, IdenticalSequencesAreDeterministic) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    double wavelength_m = 0.0;
    ASSERT_TRUE(gnss_sim::signal_wavelength_m(gps_l1, 0, &wavelength_m));
    const double biases[] = {0.10 * wavelength_m, 0.12 * wavelength_m, 0.15 * wavelength_m};
    const double times[] = {170000.0, 170000.1, 170000.2};

    gnss_sim::UrbanCarrierTemporalState first_state{};
    gnss_sim::UrbanCarrierTemporalState second_state{};
    std::string first_error;
    std::string second_error;
    for (int index = 0; index < 3; ++index) {
        const gnss_sim::UrbanSignalEpochResult epoch =
            tracked_epoch(phasor_for_range_bias(biases[index], wavelength_m), (1000 + index * 100) * kMilliseconds);
        gnss_sim::UrbanCarrierTemporalResult first{};
        gnss_sim::UrbanCarrierTemporalResult second{};
        ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(gps_l1, 0, time_at(times[index]), epoch, &first_state,
                                                                  &first, &first_error));
        ASSERT_TRUE(gnss_sim::update_urban_carrier_temporal_state(gps_l1, 0, time_at(times[index]), epoch, &second_state,
                                                                  &second, &second_error));
        EXPECT_DOUBLE_EQ(first.wrapped_phase_rad, second.wrapped_phase_rad);
        EXPECT_DOUBLE_EQ(first.unwrapped_phase_rad, second.unwrapped_phase_rad);
        EXPECT_DOUBLE_EQ(first.carrier_range_bias_m, second.carrier_range_bias_m);
        EXPECT_DOUBLE_EQ(first.environmental_range_rate_mps, second.environmental_range_rate_mps);
        EXPECT_EQ(first.environmental_range_rate_valid, second.environmental_range_rate_valid);
        EXPECT_EQ(first.cycle_slip_event, second.cycle_slip_event);
    }
}

} // namespace
