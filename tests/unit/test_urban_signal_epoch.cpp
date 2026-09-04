#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "model/urban_signal_epoch.h"

#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kSyntheticSatelliteDistanceM = 20000000.0;
constexpr double kSpeedOfLightMps = 299792458.0;

const gnss_sim::SignalDefinition& signal(gnss_sim::SignalId signal_id) {
    const gnss_sim::SignalDefinition* definition = gnss_sim::find_signal_definition(signal_id);
    EXPECT_NE(definition, nullptr);
    return *definition;
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
    model.seed = 23;
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

gnss_sim::SimTime time_at(double sow_sec) {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2400, sow_sec, &time));
    return time;
}

void schedule_tracker(gnss_sim::SignalTracker* tracker, const gnss_sim::SignalDefinition& definition,
                      const gnss_sim::SimTime& time, double elevation_deg, const gnss_sim::Cn0Model& cn0_model,
                      const gnss_sim::SignalTrackingModelConfig& config, std::uint64_t seed = 1U) {
    ASSERT_NE(tracker, nullptr);
    gnss_sim::reset_signal_tracker(tracker, definition.signal_id, time);
    gnss_sim::DeterministicRng rng{};
    gnss_sim::seed_rng(&rng, seed);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::schedule_signal_acquisition(tracker, gnss_sim::AcquisitionContext::kHot, time, time,
                                                      elevation_deg, cn0_model, config, &rng, &error_message))
        << error_message;
}

gnss_sim::UrbanReceivedPathSet path_set(double open_cn0_dbhz, bool direct_line_of_sight,
                                        const gnss_sim::CodeTrackingDllPath* paths, int path_count) {
    gnss_sim::UrbanReceivedPathSet result{};
    result.open_cn0_dbhz = open_cn0_dbhz;
    result.direct_geometry.line_of_sight = direct_line_of_sight;
    result.path_count = path_count;
    for (int index = 0; index < path_count; ++index) {
        result.paths[index] = paths[index];
    }
    return result;
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

TEST(UrbanSignalEpoch, SingleDirectPathUnifiesRootCn0AndLosTracking) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllPath paths[] = {{0.0, {1.0, 0.0}}};
    const gnss_sim::UrbanReceivedPathSet received = path_set(45.0, true, paths, 1);
    const gnss_sim::SignalTrackingModelConfig tracking = immediate_tracking_config();
    const gnss_sim::Cn0Model cn0 = normalized_model(gps_l1.signal_id, 47.0);
    const gnss_sim::SimTime time = time_at(100000.0);
    gnss_sim::SignalTracker tracker{};
    schedule_tracker(&tracker, gps_l1, time, 60.0, cn0, tracking);

    gnss_sim::UrbanSignalEpochResult result{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_urban_signal_epoch_from_paths(
        gps_l1, time, received, gnss_sim::default_code_tracking_dll_config(), tracking, &tracker, &result,
        &error_message))
        << error_message;
    EXPECT_EQ(result.root_search_status, gnss_sim::CodeTrackingDllRootSearchStatus::kRootsFound);
    EXPECT_TRUE(result.stable_root_available);
    EXPECT_TRUE(result.selected_root_valid);
    EXPECT_EQ(result.tracking_phase, gnss_sim::SignalTrackingPhase::kTracking);
    EXPECT_EQ(result.urban_state, gnss_sim::UrbanSignalState::kLos);
    EXPECT_NEAR(result.preselected_root.code_phase_chips, 0.0, 1.0e-8);
    EXPECT_NEAR(result.code_bias_m, 0.0, 1.0e-8);
    EXPECT_NEAR(result.effective_cn0_dbhz, 45.0, 1.0e-12);
    EXPECT_TRUE(result.tracked_composite_correlation_valid);
    EXPECT_NEAR(result.tracked_composite_correlation.real(), 1.0, 1.0e-14);
    EXPECT_NEAR(result.tracked_composite_correlation.imag(), 0.0, 1.0e-14);
}

TEST(UrbanSignalEpoch, DllCodeBiasComesOnlyFromSelectedCompositeRoot) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const double chip_duration_sec = 1.0 / gps_l1.code_correlation.chip_rate_hz;
    const gnss_sim::CodeTrackingDllPath paths[] = {
        {0.0, {1.0, 0.0}},
        {0.3 * chip_duration_sec, {0.5, 0.0}},
    };
    const gnss_sim::UrbanReceivedPathSet received = path_set(45.0, true, paths, 2);
    const gnss_sim::SignalTrackingModelConfig tracking = immediate_tracking_config();
    const gnss_sim::Cn0Model cn0 = normalized_model(gps_l1.signal_id, 47.0);
    const gnss_sim::SimTime time = time_at(110000.0);
    gnss_sim::SignalTracker tracker{};
    schedule_tracker(&tracker, gps_l1, time, 60.0, cn0, tracking);

    gnss_sim::UrbanSignalEpochResult result{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_urban_signal_epoch_from_paths(
        gps_l1, time, received, gnss_sim::default_code_tracking_dll_config(), tracking, &tracker, &result,
        &error_message))
        << error_message;
    EXPECT_NEAR(result.preselected_root.code_phase_chips, 0.05, 1.0e-6);
    EXPECT_NEAR(result.code_bias_m, kSpeedOfLightMps * result.preselected_root.code_phase_sec, 1.0e-9);
    EXPECT_EQ(result.urban_state, gnss_sim::UrbanSignalState::kLosMultipath);
}

TEST(UrbanSignalEpoch, ExactCoherentNullBecomesBlockedWithoutFabricatedRootOrCn0Floor) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::SignalTrackingModelConfig tracking = immediate_tracking_config();
    const gnss_sim::Cn0Model cn0 = normalized_model(gps_l1.signal_id, 60.0);
    const gnss_sim::SimTime first_time = time_at(120000.0);
    gnss_sim::SignalTracker tracker{};
    schedule_tracker(&tracker, gps_l1, first_time, 20.0, cn0, tracking);

    const gnss_sim::CodeTrackingDllPath initial_path[] = {{0.0, {1.0, 0.0}}};
    const gnss_sim::UrbanReceivedPathSet initial = path_set(55.0, false, initial_path, 1);
    gnss_sim::UrbanSignalEpochResult acquired{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_urban_signal_epoch_from_paths(
        gps_l1, first_time, initial, gnss_sim::default_code_tracking_dll_config(), tracking, &tracker, &acquired,
        &error_message))
        << error_message;
    ASSERT_EQ(acquired.tracking_phase, gnss_sim::SignalTrackingPhase::kTracking);
    ASSERT_EQ(acquired.urban_state, gnss_sim::UrbanSignalState::kNlosTracked);

    const gnss_sim::CodeTrackingDllPath cancelled_paths[] = {
        {0.0, {1.0, 0.25}},
        {0.0, {-1.0, -0.25}},
    };
    const gnss_sim::UrbanReceivedPathSet cancelled = path_set(55.0, false, cancelled_paths, 2);
    const gnss_sim::SimTime second_time = time_at(120000.1);
    gnss_sim::UrbanSignalEpochResult lost{};
    ASSERT_TRUE(gnss_sim::update_urban_signal_epoch_from_paths(
        gps_l1, second_time, cancelled, gnss_sim::default_code_tracking_dll_config(), tracking, &tracker, &lost,
        &error_message))
        << error_message;
    EXPECT_EQ(lost.root_search_status, gnss_sim::CodeTrackingDllRootSearchStatus::kNoRoots);
    EXPECT_EQ(lost.dll_root_count, 0);
    EXPECT_FALSE(lost.stable_root_available);
    EXPECT_FALSE(lost.selected_root_valid);
    EXPECT_TRUE(std::isinf(lost.effective_cn0_dbhz));
    EXPECT_LT(lost.effective_cn0_dbhz, 0.0);
    EXPECT_EQ(lost.tracking_phase, gnss_sim::SignalTrackingPhase::kSearching);
    EXPECT_EQ(lost.loss_reason, gnss_sim::SignalTrackingLossReason::kNoStableDllRoot);
    EXPECT_EQ(lost.urban_state, gnss_sim::UrbanSignalState::kBlocked);
}

TEST(UrbanSignalEpoch, DifferentCodeRateUsesSignalSpecificChipMetadata) {
    const gnss_sim::SignalDefinition& gps_l5 = signal(gnss_sim::SignalId::kGpsL5Q);
    const double chip_duration_sec = 1.0 / gps_l5.code_correlation.chip_rate_hz;
    const gnss_sim::CodeTrackingDllPath paths[] = {
        {0.0, {1.0, 0.0}},
        {0.3 * chip_duration_sec, {0.5, 0.0}},
    };
    const gnss_sim::UrbanReceivedPathSet received = path_set(45.0, true, paths, 2);
    const gnss_sim::SignalTrackingModelConfig tracking = immediate_tracking_config();
    const gnss_sim::Cn0Model cn0 = normalized_model(gps_l5.signal_id, 47.0);
    const gnss_sim::SimTime time = time_at(130000.0);
    gnss_sim::SignalTracker tracker{};
    schedule_tracker(&tracker, gps_l5, time, 60.0, cn0, tracking);

    gnss_sim::UrbanSignalEpochResult result{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_urban_signal_epoch_from_paths(
        gps_l5, time, received, gnss_sim::default_code_tracking_dll_config(), tracking, &tracker, &result,
        &error_message))
        << error_message;
    EXPECT_NEAR(result.preselected_root.code_phase_chips, 0.05, 1.0e-6);
    EXPECT_NEAR(result.preselected_root.code_phase_sec, 0.05 / gps_l5.code_correlation.chip_rate_hz, 1.0e-15);
    EXPECT_NEAR(result.code_bias_m, kSpeedOfLightMps * result.preselected_root.code_phase_sec, 1.0e-9);
}

TEST(UrbanSignalEpoch, ProductionRoofTransitionCanReachLosMultipath) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig sim_config = gnss_sim::default_sim_config();
    const gnss_sim::SignalTrackingModelConfig tracking = immediate_tracking_config();
    const gnss_sim::Cn0Model cn0 = normalized_model(gps_l1.signal_id, 55.0);
    double skyline_rad = 0.0;
    double wall_distance_m = 0.0;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_skyline_elevation(scene, 0.0, &skyline_rad, &wall_distance_m, &error_message));

    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry geometry{};
    const double elevation_deg = skyline_rad / kDegreesToRadians + 0.001;
    make_synthetic_geometry(0.0, elevation_deg, &receiver, &geometry);
    const gnss_sim::SimTime time = time_at(140000.0);
    gnss_sim::SignalTracker tracker{};
    schedule_tracker(&tracker, gps_l1, time, elevation_deg, cn0, tracking);

    gnss_sim::UrbanSignalEpochResult result{};
    ASSERT_TRUE(gnss_sim::compute_urban_signal_epoch(
        cn0, scene, sim_config.urban_rf, gnss_sim::default_code_tracking_dll_config(), tracking, gps_l1, 0, time,
        receiver, geometry, &tracker, &result, &error_message))
        << error_message;
    EXPECT_TRUE(result.received_paths.direct_geometry.line_of_sight);
    EXPECT_EQ(result.tracking_phase, gnss_sim::SignalTrackingPhase::kTracking);
    EXPECT_EQ(result.urban_state, gnss_sim::UrbanSignalState::kLosMultipath);
    EXPECT_LT(result.effective_cn0_dbhz, result.received_paths.open_cn0_dbhz - 0.5);
}

TEST(UrbanSignalEpoch, IdenticalInputsProduceIdenticalUnifiedResults) {
    const gnss_sim::SignalDefinition& gal_e1 = signal(gnss_sim::SignalId::kGalileoE1);
    const gnss_sim::CodeTrackingDllPath paths[] = {{0.0, {0.8, 0.1}}};
    const gnss_sim::UrbanReceivedPathSet received = path_set(46.0, true, paths, 1);
    const gnss_sim::SignalTrackingModelConfig tracking = immediate_tracking_config();
    const gnss_sim::Cn0Model cn0 = normalized_model(gal_e1.signal_id, 48.0);
    const gnss_sim::SimTime time = time_at(150000.0);
    gnss_sim::SignalTracker first_tracker{};
    gnss_sim::SignalTracker second_tracker{};
    schedule_tracker(&first_tracker, gal_e1, time, 60.0, cn0, tracking, 77U);
    schedule_tracker(&second_tracker, gal_e1, time, 60.0, cn0, tracking, 77U);

    gnss_sim::UrbanSignalEpochResult first{};
    gnss_sim::UrbanSignalEpochResult second{};
    std::string first_error;
    std::string second_error;
    ASSERT_TRUE(gnss_sim::update_urban_signal_epoch_from_paths(
        gal_e1, time, received, gnss_sim::default_code_tracking_dll_config(), tracking, &first_tracker, &first,
        &first_error));
    ASSERT_TRUE(gnss_sim::update_urban_signal_epoch_from_paths(
        gal_e1, time, received, gnss_sim::default_code_tracking_dll_config(), tracking, &second_tracker, &second,
        &second_error));
    EXPECT_EQ(first.dll_root_count, second.dll_root_count);
    EXPECT_EQ(first.root_search_status, second.root_search_status);
    EXPECT_EQ(first.preselected_root_index, second.preselected_root_index);
    EXPECT_DOUBLE_EQ(first.preselected_root.code_phase_sec, second.preselected_root.code_phase_sec);
    EXPECT_DOUBLE_EQ(first.effective_cn0_dbhz, second.effective_cn0_dbhz);
    EXPECT_DOUBLE_EQ(first.code_bias_m, second.code_bias_m);
    EXPECT_EQ(first.tracked_composite_correlation, second.tracked_composite_correlation);
    EXPECT_EQ(first.urban_state, second.urban_state);
    EXPECT_EQ(first.tracking_phase, second.tracking_phase);
}

} // namespace
