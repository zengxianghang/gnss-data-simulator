#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "model/urban_reflection_paths.h"

#include <cmath>
#include <complex>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kSyntheticSatelliteDistanceM = 20000000.0;

void make_synthetic_geometry(double azimuth_deg, double elevation_deg, double sagnac_offset_m,
                             gnss_sim::ReceiverTruth* receiver, gnss_sim::SatelliteGeometry* geometry) {
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
    geometry->geometric_range_m = kSyntheticSatelliteDistanceM + sagnac_offset_m;
}

const gnss_sim::SignalDefinition& signal(gnss_sim::SignalId signal_id) {
    const gnss_sim::SignalDefinition* definition = gnss_sim::find_signal_definition(signal_id);
    EXPECT_NE(definition, nullptr);
    return *definition;
}

TEST(UrbanReflectionPaths, SouthSkyProducesOneValidNorthWallReflection) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig sim_config = gnss_sim::default_sim_config();
    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry geometry{};
    make_synthetic_geometry(180.0, 20.0, 25.0, &receiver, &geometry);

    gnss_sim::UrbanFirstOrderReflectionSet reflections{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_first_order_reflections(
        scene, sim_config.urban_rf, signal(gnss_sim::SignalId::kGpsL1Ca), 0, receiver, geometry, &reflections,
        &error_message))
        << error_message;

    ASSERT_EQ(reflections.path_count, 1);
    const gnss_sim::UrbanFirstOrderReflectionPath& path = reflections.paths[0];
    EXPECT_EQ(path.wall_id, gnss_sim::UrbanWallId::NORTH);
    EXPECT_EQ(reflections.candidate_status[0], gnss_sim::UrbanReflectionCandidateStatus::VALID);
    EXPECT_EQ(reflections.candidate_status[2], gnss_sim::UrbanReflectionCandidateStatus::BACKSIDE);
    EXPECT_NEAR(path.reflection_point_enu_m.east_m, 0.0, 1.0e-8);
    EXPECT_NEAR(path.reflection_point_enu_m.north_m, 10.0, 1.0e-8);
    EXPECT_NEAR(path.reflection_point_enu_m.up_m, 5.1397, 1.0e-3);
    EXPECT_GT(path.excess_path_length_m, 0.0);
    EXPECT_GT(path.excess_delay_sec, 0.0);
    EXPECT_NEAR(std::abs(path.geometric_phase_factor), 1.0, 1.0e-12);
    EXPECT_TRUE(std::isfinite(path.rf_response.gamma_te_tangent.real()));
    EXPECT_TRUE(std::isfinite(path.rf_response.gamma_tm_tangent.imag()));
}

TEST(UrbanReflectionPaths, SpecularDirectionsObeyReflectionLaw) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig sim_config = gnss_sim::default_sim_config();
    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry geometry{};
    make_synthetic_geometry(180.0, 20.0, 0.0, &receiver, &geometry);

    gnss_sim::UrbanFirstOrderReflectionSet reflections{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_first_order_reflections(
        scene, sim_config.urban_rf, signal(gnss_sim::SignalId::kGpsL1Ca), 0, receiver, geometry, &reflections,
        &error_message))
        << error_message;
    ASSERT_EQ(reflections.path_count, 1);

    gnss_sim::UrbanWallPlane wall{};
    ASSERT_TRUE(gnss_sim::urban_wall_plane(scene, reflections.paths[0].wall_id, &wall, &error_message));
    const gnss_sim::UrbanFirstOrderReflectionPath& path = reflections.paths[0];
    double incident_dot_normal = 0.0;
    for (int index = 0; index < 3; ++index) {
        incident_dot_normal += path.incident_direction_enu[index] * wall.inward_normal_enu[index];
    }
    for (int index = 0; index < 3; ++index) {
        const double expected =
            path.incident_direction_enu[index] - 2.0 * incident_dot_normal * wall.inward_normal_enu[index];
        EXPECT_NEAR(path.reflected_direction_enu[index], expected, 1.0e-10);
    }
}

TEST(UrbanReflectionPaths, BacksideAndFacadeHeightAreExplicitRejections) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig sim_config = gnss_sim::default_sim_config();
    const gnss_sim::UrbanRfResolvedConfig rf{sim_config.urban_rf.default_material, sim_config.urban_rf.default_antenna};
    const gnss_sim::EnuPoint receiver{0.0, 0.0, scene.antenna_height_m};
    const double wavelength_m = 299792458.0 / 1575.42e6;
    std::string error_message;

    const gnss_sim::EnuPoint north_source{0.0, 1000.0, 500.0};
    gnss_sim::UrbanFirstOrderReflectionPath path{};
    gnss_sim::UrbanReflectionCandidateStatus status{};
    ASSERT_TRUE(gnss_sim::compute_urban_one_wall_reflection(scene, rf, gnss_sim::UrbanWallId::NORTH, north_source,
                                                            receiver, 1200.0, 1575.42e6, wavelength_m, &path, &status,
                                                            &error_message));
    EXPECT_EQ(status, gnss_sim::UrbanReflectionCandidateStatus::BACKSIDE);

    const gnss_sim::EnuPoint high_south_source{0.0, -1000.0, 2000.0};
    ASSERT_TRUE(gnss_sim::compute_urban_one_wall_reflection(scene, rf, gnss_sim::UrbanWallId::NORTH,
                                                            high_south_source, receiver, 2500.0, 1575.42e6,
                                                            wavelength_m, &path, &status, &error_message));
    EXPECT_EQ(status, gnss_sim::UrbanReflectionCandidateStatus::OUTSIDE_FACADE_HEIGHT);
}

TEST(UrbanReflectionPaths, SkewedCandidateIsRejectedByAnotherFacade) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig sim_config = gnss_sim::default_sim_config();
    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry geometry{};
    make_synthetic_geometry(250.0, 10.0, 0.0, &receiver, &geometry);

    gnss_sim::UrbanFirstOrderReflectionSet reflections{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_first_order_reflections(
        scene, sim_config.urban_rf, signal(gnss_sim::SignalId::kGpsL1Ca), 0, receiver, geometry, &reflections,
        &error_message))
        << error_message;
    EXPECT_EQ(reflections.path_count, 0);
    EXPECT_EQ(reflections.candidate_status[0], gnss_sim::UrbanReflectionCandidateStatus::RECEIVER_OCCLUDED);
    EXPECT_EQ(reflections.candidate_status[1], gnss_sim::UrbanReflectionCandidateStatus::SOURCE_OCCLUDED);
}

TEST(UrbanReflectionPaths, DiagonalDoubleImageCandidatesRemainBlockedInFrozenInfiniteWallScene) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig sim_config = gnss_sim::default_sim_config();
    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry geometry{};
    make_synthetic_geometry(225.0, 20.0, 0.0, &receiver, &geometry);

    gnss_sim::UrbanFirstOrderReflectionSet reflections{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_first_order_reflections(
        scene, sim_config.urban_rf, signal(gnss_sim::SignalId::kGpsL1Ca), 0, receiver, geometry, &reflections,
        &error_message))
        << error_message;
    EXPECT_EQ(reflections.path_count, 0);
    EXPECT_EQ(reflections.candidate_status[0], gnss_sim::UrbanReflectionCandidateStatus::SOURCE_OCCLUDED);
    EXPECT_EQ(reflections.candidate_status[1], gnss_sim::UrbanReflectionCandidateStatus::SOURCE_OCCLUDED);
    EXPECT_EQ(reflections.candidate_status[2], gnss_sim::UrbanReflectionCandidateStatus::BACKSIDE);
    EXPECT_EQ(reflections.candidate_status[3], gnss_sim::UrbanReflectionCandidateStatus::BACKSIDE);
}

TEST(UrbanReflectionPaths, ExcessRangeUsesEuclideanDifferenceThenAddsRtklibBaseline) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig sim_config = gnss_sim::default_sim_config();
    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry geometry{};
    constexpr double kSyntheticSagnacOffsetM = 25.0;
    make_synthetic_geometry(180.0, 20.0, kSyntheticSagnacOffsetM, &receiver, &geometry);

    gnss_sim::UrbanFirstOrderReflectionSet reflections{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_first_order_reflections(
        scene, sim_config.urban_rf, signal(gnss_sim::SignalId::kGpsL1Ca), 0, receiver, geometry, &reflections,
        &error_message))
        << error_message;
    ASSERT_EQ(reflections.path_count, 1);
    const gnss_sim::UrbanFirstOrderReflectionPath& path = reflections.paths[0];

    EXPECT_NEAR(path.direct_euclidean_range_m, kSyntheticSatelliteDistanceM, 1.0e-6);
    EXPECT_NEAR(path.direct_model_range_m - path.direct_euclidean_range_m, kSyntheticSagnacOffsetM, 1.0e-6);
    EXPECT_NEAR(path.reflected_euclidean_range_m - path.direct_euclidean_range_m, path.excess_path_length_m, 1.0e-9);
    EXPECT_NEAR(path.model_path_range_m - path.direct_model_range_m, path.excess_path_length_m, 1.0e-9);
    EXPECT_NEAR(path.model_path_range_m - path.reflected_euclidean_range_m, kSyntheticSagnacOffsetM, 1.0e-6);
}

TEST(UrbanReflectionPaths, UsesCentralFixedAndGlonassFdmaFrequencies) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig sim_config = gnss_sim::default_sim_config();
    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry geometry{};
    make_synthetic_geometry(180.0, 20.0, 0.0, &receiver, &geometry);
    std::string error_message;

    gnss_sim::UrbanFirstOrderReflectionSet gps_reflections{};
    const gnss_sim::SignalDefinition& gps = signal(gnss_sim::SignalId::kGpsL1Ca);
    ASSERT_TRUE(gnss_sim::compute_urban_first_order_reflections(scene, sim_config.urban_rf, gps, 0, receiver, geometry,
                                                                &gps_reflections, &error_message));
    ASSERT_EQ(gps_reflections.path_count, 1);
    double expected_gps_hz = 0.0;
    ASSERT_TRUE(gnss_sim::signal_carrier_frequency_hz(gps, 0, &expected_gps_hz));
    EXPECT_DOUBLE_EQ(gps_reflections.paths[0].carrier_frequency_hz, expected_gps_hz);

    gnss_sim::UrbanFirstOrderReflectionSet glo_reflections{};
    const gnss_sim::SignalDefinition& glo = signal(gnss_sim::SignalId::kGlonassG1);
    constexpr int kFcn = 5;
    ASSERT_TRUE(gnss_sim::compute_urban_first_order_reflections(scene, sim_config.urban_rf, glo, kFcn, receiver,
                                                                geometry, &glo_reflections, &error_message));
    ASSERT_EQ(glo_reflections.path_count, 1);
    double expected_glo_hz = 0.0;
    ASSERT_TRUE(gnss_sim::signal_carrier_frequency_hz(glo, kFcn, &expected_glo_hz));
    EXPECT_DOUBLE_EQ(glo_reflections.paths[0].carrier_frequency_hz, expected_glo_hz);
    EXPECT_NE(expected_glo_hz, glo.nominal_frequency_hz);

    gnss_sim::UrbanFirstOrderReflectionSet invalid{};
    EXPECT_FALSE(gnss_sim::compute_urban_first_order_reflections(scene, sim_config.urban_rf, glo, 20, receiver,
                                                                 geometry, &invalid, &error_message));
}

TEST(UrbanReflectionPaths, HighElevationCanProduceZeroValidPathsWithoutInventingFallback) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig sim_config = gnss_sim::default_sim_config();
    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry geometry{};
    make_synthetic_geometry(180.0, 60.0, 0.0, &receiver, &geometry);

    gnss_sim::UrbanFirstOrderReflectionSet reflections{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_first_order_reflections(
        scene, sim_config.urban_rf, signal(gnss_sim::SignalId::kGpsL1Ca), 0, receiver, geometry, &reflections,
        &error_message));
    EXPECT_EQ(reflections.path_count, 0);
    EXPECT_EQ(reflections.candidate_status[0], gnss_sim::UrbanReflectionCandidateStatus::OUTSIDE_FACADE_HEIGHT);
}

TEST(UrbanReflectionPaths, RepeatedEvaluationIsNumericallyIdentical) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const gnss_sim::SimConfig sim_config = gnss_sim::default_sim_config();
    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry geometry{};
    make_synthetic_geometry(180.0, 20.0, 17.0, &receiver, &geometry);

    gnss_sim::UrbanFirstOrderReflectionSet first{};
    gnss_sim::UrbanFirstOrderReflectionSet second{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_first_order_reflections(
        scene, sim_config.urban_rf, signal(gnss_sim::SignalId::kGpsL1Ca), 0, receiver, geometry, &first, &error_message));
    ASSERT_TRUE(gnss_sim::compute_urban_first_order_reflections(
        scene, sim_config.urban_rf, signal(gnss_sim::SignalId::kGpsL1Ca), 0, receiver, geometry, &second,
        &error_message));
    ASSERT_EQ(first.path_count, second.path_count);
    ASSERT_EQ(first.path_count, 1);
    for (int index = 0; index < gnss_sim::kUrbanFirstOrderWallCount; ++index) {
        EXPECT_EQ(first.candidate_status[index], second.candidate_status[index]);
    }
    const gnss_sim::UrbanFirstOrderReflectionPath& left = first.paths[0];
    const gnss_sim::UrbanFirstOrderReflectionPath& right = second.paths[0];
    EXPECT_EQ(left.wall_id, right.wall_id);
    EXPECT_DOUBLE_EQ(left.excess_path_length_m, right.excess_path_length_m);
    EXPECT_DOUBLE_EQ(left.excess_delay_sec, right.excess_delay_sec);
    EXPECT_DOUBLE_EQ(left.excess_carrier_phase_rad, right.excess_carrier_phase_rad);
    EXPECT_EQ(left.geometric_phase_factor, right.geometric_phase_factor);
    EXPECT_EQ(left.rf_response.gamma_te_tangent, right.rf_response.gamma_te_tangent);
    EXPECT_EQ(left.rf_response.gamma_tm_tangent, right.rf_response.gamma_tm_tangent);
    EXPECT_EQ(left.antenna_rhcp_voltage, right.antenna_rhcp_voltage);
    EXPECT_EQ(left.antenna_lhcp_voltage, right.antenna_lhcp_voltage);
}

} // namespace
