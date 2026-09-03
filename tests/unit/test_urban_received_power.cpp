#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "model/urban_received_power.h"

#include <cmath>
#include <complex>
#include <gtest/gtest.h>
#include <limits>
#include <string>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kSyntheticSatelliteDistanceM = 20000000.0;

const gnss_sim::SignalDefinition& signal(gnss_sim::SignalId signal_id) {
    const gnss_sim::SignalDefinition* definition = gnss_sim::find_signal_definition(signal_id);
    EXPECT_NE(definition, nullptr);
    return *definition;
}

void make_synthetic_geometry(double azimuth_deg, double elevation_deg, gnss_sim::ReceiverTruth* receiver,
                             gnss_sim::SatelliteGeometry* geometry) {
    ASSERT_NE(receiver, nullptr);
    ASSERT_NE(geometry, nullptr);
    *receiver = {};
    *geometry = {};
    receiver->position_ecef_m[0] = 0.0;
    receiver->position_ecef_m[1] = 0.0;
    receiver->position_ecef_m[2] = 0.0;
    geometry->satellite_state.position_ecef_m[0] = kSyntheticSatelliteDistanceM;
    geometry->satellite_state.position_ecef_m[1] = 0.0;
    geometry->satellite_state.position_ecef_m[2] = 0.0;
    geometry->azimuth_rad = azimuth_deg * kDegreesToRadians;
    geometry->elevation_rad = elevation_deg * kDegreesToRadians;
    geometry->geometric_range_m = kSyntheticSatelliteDistanceM;
    geometry->healthy = true;
    geometry->above_elevation_mask = true;
    geometry->visible = true;
}

gnss_sim::Cn0Model normalized_model(gnss_sim::SignalId signal_id, double high_cn0_dbhz) {
    gnss_sim::Cn0Model model{};
    model.source = gnss_sim::Cn0ModelSource::kCalibratedCsv;
    model.semantic = gnss_sim::Cn0ModelSemantic::kNormalizedElevationShape;
    model.seed = 17;
    gnss_sim::Cn0CalibratedBin bin{};
    bin.signal_id = signal_id;
    bin.elevation_min_deg = 0.0;
    bin.elevation_max_deg = 90.0;
    bin.elevation_center_deg = 45.0;
    bin.delta_p50_db = -2.0;
    bin.support_count = 5;
    bin.upper_edge_inclusive = true;
    bin.ready = true;
    model.calibrated_bins.push_back(bin);
    model.high_baselines.push_back({signal_id, high_cn0_dbhz});
    return model;
}

gnss_sim::SimTime test_time() {
    gnss_sim::SimTime time{};
    time.gps_week = 2400;
    time.tow_ns = 100000000000LL;
    return time;
}

TEST(UrbanReceivedPower, ConstructiveAndDestructiveFieldsMapToPowerDomainCn0) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllPath constructive[] = {
        {0.0, {1.0, 0.0}},
        {0.0, {0.5, 0.0}},
    };
    const gnss_sim::CodeTrackingDllPath destructive[] = {
        {0.0, {1.0, 0.0}},
        {0.0, {-0.5, 0.0}},
    };
    gnss_sim::UrbanEffectiveCn0 high{};
    gnss_sim::UrbanEffectiveCn0 low{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_effective_cn0_from_paths(45.0, gps_l1, constructive, 2, 0.0, &high,
                                                           &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::compute_effective_cn0_from_paths(45.0, gps_l1, destructive, 2, 0.0, &low,
                                                           &error_message))
        << error_message;
    EXPECT_NEAR(high.composite_power_ratio, 2.25, 1.0e-14);
    EXPECT_NEAR(high.effective_cn0_dbhz, 45.0 + 10.0 * std::log10(2.25), 1.0e-12);
    EXPECT_NEAR(low.composite_power_ratio, 0.25, 1.0e-14);
    EXPECT_NEAR(low.effective_cn0_dbhz, 45.0 + 10.0 * std::log10(0.25), 1.0e-12);
}

TEST(UrbanReceivedPower, CodeDelayDecorrelatesASeparatedPathAtDirectPrompt) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const double chip_duration_s = 1.0 / gps_l1.code_correlation.chip_rate_hz;
    const gnss_sim::CodeTrackingDllPath paths[] = {
        {0.0, {1.0, 0.0}},
        {chip_duration_s, {0.75, 0.0}},
    };
    gnss_sim::UrbanEffectiveCn0 effective{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_effective_cn0_from_paths(43.0, gps_l1, paths, 2, 0.0, &effective,
                                                           &error_message))
        << error_message;
    EXPECT_NEAR(effective.composite_correlation.real(), 1.0, 1.0e-14);
    EXPECT_NEAR(effective.composite_correlation.imag(), 0.0, 1.0e-14);
    EXPECT_NEAR(effective.effective_cn0_dbhz, 43.0, 1.0e-12);
}

TEST(UrbanReceivedPower, ExactCancellationHasNoArbitraryCn0Floor) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllPath paths[] = {
        {0.0, {1.0, 0.0}},
        {0.0, {-1.0, 0.0}},
    };
    gnss_sim::UrbanEffectiveCn0 effective{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_effective_cn0_from_paths(45.0, gps_l1, paths, 2, 0.0, &effective,
                                                           &error_message))
        << error_message;
    EXPECT_DOUBLE_EQ(effective.composite_power_ratio, 0.0);
    EXPECT_DOUBLE_EQ(effective.carrier_to_noise_density_hz, 0.0);
    EXPECT_FALSE(effective.finite_effective_cn0);
    EXPECT_TRUE(std::isinf(effective.effective_cn0_dbhz));
    EXPECT_LT(effective.effective_cn0_dbhz, 0.0);
}

TEST(UrbanReceivedPower, RooftopGrazingAppliesDiffractionTransferExactlyOnce) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    double skyline_rad = 0.0;
    double wall_distance_m = 0.0;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_skyline_elevation(scene, 0.0, &skyline_rad, &wall_distance_m, &error_message));

    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry geometry{};
    make_synthetic_geometry(0.0, skyline_rad / kDegreesToRadians, &receiver, &geometry);
    gnss_sim::UrbanReceivedPathSet paths{};
    ASSERT_TRUE(gnss_sim::compute_urban_received_path_set(normalized_model(gps_l1.signal_id, 47.0), scene,
                                                         config.urban_rf, gps_l1, 0, test_time(), receiver, geometry,
                                                         &paths, &error_message))
        << error_message;
    ASSERT_EQ(paths.diffraction_status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    ASSERT_EQ(paths.reflections.path_count, 0);
    ASSERT_EQ(paths.path_count, 1);
    EXPECT_NEAR(paths.diffraction.fresnel_v, 0.0, 1.0e-8);
    EXPECT_NEAR(paths.direct_voltage.real(), 0.5, 1.0e-10);
    EXPECT_NEAR(paths.direct_voltage.imag(), 0.0, 1.0e-10);
    EXPECT_NEAR(paths.paths[0].code_delay_sec, 0.0, 1.0e-18);

    gnss_sim::UrbanEffectiveCn0 effective{};
    ASSERT_TRUE(gnss_sim::compute_urban_effective_cn0(gps_l1, paths, 0.0, &effective, &error_message))
        << error_message;
    EXPECT_NEAR(effective.effective_cn0_dbhz, paths.open_cn0_dbhz - 6.020599913279624, 1.0e-9);
}

TEST(UrbanReceivedPower, FrozenBlockedGeometryRetainsIndependentReflectionAsSecondPath) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry geometry{};
    make_synthetic_geometry(180.0, 20.0, &receiver, &geometry);

    gnss_sim::UrbanReceivedPathSet paths{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_received_path_set(normalized_model(gps_l1.signal_id, 46.0), scene,
                                                         config.urban_rf, gps_l1, 0, test_time(), receiver, geometry,
                                                         &paths, &error_message))
        << error_message;
    ASSERT_EQ(paths.diffraction_status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    ASSERT_EQ(paths.reflections.path_count, 1);
    ASSERT_EQ(paths.path_count, 2);
    EXPECT_FALSE(paths.direct_geometry.line_of_sight);
    EXPECT_EQ(paths.paths[0].complex_voltage, paths.diffraction.fresnel_coefficient);
    EXPECT_NEAR(paths.paths[0].code_delay_sec, paths.diffraction.excess_delay_sec, 1.0e-18);
    EXPECT_GT(std::abs(paths.paths[1].complex_voltage), 0.0);
    EXPECT_TRUE(std::isfinite(paths.paths[1].complex_voltage.real()));
    EXPECT_TRUE(std::isfinite(paths.paths[1].complex_voltage.imag()));
    EXPECT_NEAR(paths.paths[1].code_delay_sec, paths.reflections.paths[0].excess_delay_sec, 1.0e-18);
}

TEST(UrbanReceivedPower, ChangingOnlyHighBaselineTranslatesOpenAndEffectiveCn0) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry geometry{};
    make_synthetic_geometry(0.0, 60.0, &receiver, &geometry);

    gnss_sim::UrbanReceivedPathSet lower_paths{};
    gnss_sim::UrbanReceivedPathSet higher_paths{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_received_path_set(normalized_model(gps_l1.signal_id, 45.0), scene,
                                                         config.urban_rf, gps_l1, 0, test_time(), receiver, geometry,
                                                         &lower_paths, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_received_path_set(normalized_model(gps_l1.signal_id, 48.0), scene,
                                                         config.urban_rf, gps_l1, 0, test_time(), receiver, geometry,
                                                         &higher_paths, &error_message))
        << error_message;
    EXPECT_NEAR(higher_paths.open_cn0_dbhz - lower_paths.open_cn0_dbhz, 3.0, 1.0e-12);
    ASSERT_EQ(higher_paths.path_count, lower_paths.path_count);
    for (int index = 0; index < lower_paths.path_count; ++index) {
        EXPECT_NEAR(higher_paths.paths[index].code_delay_sec, lower_paths.paths[index].code_delay_sec, 1.0e-18);
        EXPECT_NEAR(std::abs(higher_paths.paths[index].complex_voltage - lower_paths.paths[index].complex_voltage),
                    0.0, 1.0e-14);
    }

    gnss_sim::UrbanEffectiveCn0 lower{};
    gnss_sim::UrbanEffectiveCn0 higher{};
    ASSERT_TRUE(gnss_sim::compute_urban_effective_cn0(gps_l1, lower_paths, lower_paths.paths[0].code_delay_sec, &lower,
                                                      &error_message));
    ASSERT_TRUE(gnss_sim::compute_urban_effective_cn0(gps_l1, higher_paths, higher_paths.paths[0].code_delay_sec,
                                                      &higher, &error_message));
    EXPECT_NEAR(higher.effective_cn0_dbhz - lower.effective_cn0_dbhz, 3.0, 1.0e-12);
}

TEST(UrbanReceivedPower, ClearSideRoofTransitionIsContinuous) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    double skyline_rad = 0.0;
    double wall_distance_m = 0.0;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_skyline_elevation(scene, 0.0, &skyline_rad, &wall_distance_m, &error_message));
    const double skyline_deg = skyline_rad / kDegreesToRadians;

    gnss_sim::ReceiverTruth receiver_low{};
    gnss_sim::ReceiverTruth receiver_high{};
    gnss_sim::SatelliteGeometry geometry_low{};
    gnss_sim::SatelliteGeometry geometry_high{};
    make_synthetic_geometry(0.0, skyline_deg - 0.001, &receiver_low, &geometry_low);
    make_synthetic_geometry(0.0, skyline_deg + 0.001, &receiver_high, &geometry_high);
    gnss_sim::UrbanReceivedPathSet low{};
    gnss_sim::UrbanReceivedPathSet high{};
    const gnss_sim::Cn0Model model = normalized_model(gps_l1.signal_id, 47.0);
    ASSERT_TRUE(gnss_sim::compute_urban_received_path_set(model, scene, config.urban_rf, gps_l1, 0, test_time(),
                                                         receiver_low, geometry_low, &low, &error_message));
    ASSERT_TRUE(gnss_sim::compute_urban_received_path_set(model, scene, config.urban_rf, gps_l1, 0, test_time(),
                                                         receiver_high, geometry_high, &high, &error_message));
    ASSERT_GT(low.path_count, 0);
    ASSERT_GT(high.path_count, 0);
    EXPECT_FALSE(low.direct_geometry.line_of_sight);
    EXPECT_TRUE(high.direct_geometry.line_of_sight);
    EXPECT_NEAR(low.paths[0].code_delay_sec, low.diffraction.excess_delay_sec, 1.0e-18);
    EXPECT_NEAR(high.paths[0].code_delay_sec, 0.0, 1.0e-18);
    EXPECT_LT(std::abs(std::abs(high.direct_voltage) - std::abs(low.direct_voltage)), 0.01);
}

TEST(UrbanReceivedPower, InvalidInputsFailExplicitly) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllPath path[] = {{0.0, {1.0, 0.0}}};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::compute_effective_cn0_from_paths(std::numeric_limits<double>::quiet_NaN(), gps_l1, path, 1,
                                                            0.0, nullptr, &error_message));
}

} // namespace
