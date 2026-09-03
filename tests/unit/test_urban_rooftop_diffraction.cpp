#include "gnss/signal_definitions.h"
#include "model/urban_rooftop_diffraction.h"

#include <cmath>
#include <complex>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;
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

double cardinal_skyline_deg(const gnss_sim::UrbanSceneGeometryConfig& scene) {
    double skyline_rad = 0.0;
    double horizontal_distance_m = 0.0;
    std::string error_message;
    EXPECT_TRUE(
        gnss_sim::compute_urban_skyline_elevation(scene, 0.0, &skyline_rad, &horizontal_distance_m, &error_message))
        << error_message;
    EXPECT_NEAR(horizontal_distance_m, scene.wall_distance_m, 1.0e-12);
    return skyline_rad * kRadiansToDegrees;
}

gnss_sim::UrbanRooftopDiffractionPath evaluate(double elevation_deg, const gnss_sim::SignalDefinition& definition,
                                               int glonass_fcn, double sagnac_offset_m,
                                               gnss_sim::UrbanRooftopDiffractionStatus* status) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    gnss_sim::ReceiverTruth receiver{};
    gnss_sim::SatelliteGeometry geometry{};
    make_synthetic_geometry(0.0, elevation_deg, sagnac_offset_m, &receiver, &geometry);

    gnss_sim::UrbanRooftopDiffractionPath result{};
    std::string error_message;
    EXPECT_TRUE(gnss_sim::compute_urban_rooftop_diffraction(scene, definition, glonass_fcn, receiver, geometry, &result,
                                                            status, &error_message))
        << error_message;
    return result;
}

TEST(UrbanRooftopDiffraction, ComplexFresnelAnchorsMatchKnifeEdgeLimits) {
    std::string error_message;
    std::complex<double> grazing{};
    std::complex<double> positive_half{};
    std::complex<double> positive_one{};
    std::complex<double> positive_two{};
    std::complex<double> clear_far{};

    ASSERT_TRUE(gnss_sim::compute_complex_knife_edge_coefficient(0.0, &grazing, &error_message)) << error_message;
    ASSERT_TRUE(gnss_sim::compute_complex_knife_edge_coefficient(0.5, &positive_half, &error_message));
    ASSERT_TRUE(gnss_sim::compute_complex_knife_edge_coefficient(1.0, &positive_one, &error_message));
    ASSERT_TRUE(gnss_sim::compute_complex_knife_edge_coefficient(2.0, &positive_two, &error_message));
    ASSERT_TRUE(gnss_sim::compute_complex_knife_edge_coefficient(-10.0, &clear_far, &error_message));

    EXPECT_NEAR(grazing.real(), 0.5, 1.0e-14);
    EXPECT_NEAR(grazing.imag(), 0.0, 1.0e-14);
    EXPECT_NEAR(-20.0 * std::log10(std::abs(grazing)), 6.020599913279624, 1.0e-12);
    EXPECT_GT(std::abs(positive_half), std::abs(positive_one));
    EXPECT_GT(std::abs(positive_one), std::abs(positive_two));
    EXPECT_NEAR(positive_one.real(), -0.10907627388358884, 1.0e-12);
    EXPECT_NEAR(positive_one.imag(), -0.17081712649323413, 1.0e-12);
    EXPECT_NEAR(std::abs(clear_far), 1.0, 0.03);
}

TEST(UrbanRooftopDiffraction, ComplexCoefficientIsContinuousAcrossGrazing) {
    std::string error_message;
    std::complex<double> negative{};
    std::complex<double> positive{};
    ASSERT_TRUE(gnss_sim::compute_complex_knife_edge_coefficient(-1.0e-4, &negative, &error_message));
    ASSERT_TRUE(gnss_sim::compute_complex_knife_edge_coefficient(1.0e-4, &positive, &error_message));
    EXPECT_LT(std::abs(negative - positive), 3.0e-4);
}

TEST(UrbanRooftopDiffraction, ExactCardinalSkylineUsesActualNorthRoofEdge) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const double skyline_deg = cardinal_skyline_deg(scene);
    gnss_sim::UrbanRooftopDiffractionStatus status{};
    const gnss_sim::UrbanRooftopDiffractionPath path =
        evaluate(skyline_deg, signal(gnss_sim::SignalId::kGpsL1Ca), 0, 25.0, &status);

    ASSERT_EQ(status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    EXPECT_EQ(path.wall_id, gnss_sim::UrbanWallId::NORTH);
    EXPECT_NEAR(path.diffraction_point_enu_m.east_m, 0.0, 1.0e-8);
    EXPECT_NEAR(path.diffraction_point_enu_m.north_m, scene.wall_distance_m, 1.0e-8);
    EXPECT_NEAR(path.diffraction_point_enu_m.up_m, scene.wall_height_m, 1.0e-8);
    EXPECT_NEAR(path.signed_clearance_m, 0.0, 1.0e-8);
    EXPECT_NEAR(path.fresnel_v, 0.0, 1.0e-8);
    EXPECT_NEAR(std::abs(path.fresnel_coefficient), 0.5, 1.0e-8);
    EXPECT_NEAR(path.model_path_range_m - path.direct_model_range_m, path.excess_path_length_m, 1.0e-12);
    EXPECT_NEAR(path.direct_model_range_m - path.direct_euclidean_range_m, 25.0, 1.0e-6);

    const std::complex<double> reconstructed = path.edge_reference_coefficient * path.edge_geometric_phase_factor;
    EXPECT_NEAR(reconstructed.real(), path.fresnel_coefficient.real(), 1.0e-12);
    EXPECT_NEAR(reconstructed.imag(), path.fresnel_coefficient.imag(), 1.0e-12);
}

TEST(UrbanRooftopDiffraction, ShadowAndClearSidesUseOppositeSignedFresnelV) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const double skyline_deg = cardinal_skyline_deg(scene);
    gnss_sim::UrbanRooftopDiffractionStatus shadow_status{};
    gnss_sim::UrbanRooftopDiffractionStatus clear_status{};

    const gnss_sim::UrbanRooftopDiffractionPath shadow =
        evaluate(skyline_deg - 1.0, signal(gnss_sim::SignalId::kGpsL1Ca), 0, 0.0, &shadow_status);
    const gnss_sim::UrbanRooftopDiffractionPath clear =
        evaluate(skyline_deg + 1.0, signal(gnss_sim::SignalId::kGpsL1Ca), 0, 0.0, &clear_status);

    ASSERT_EQ(shadow_status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    ASSERT_EQ(clear_status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    EXPECT_GT(shadow.signed_clearance_m, 0.0);
    EXPECT_GT(shadow.fresnel_v, 0.0);
    EXPECT_LT(std::abs(shadow.fresnel_coefficient), 0.5);
    EXPECT_LT(clear.signed_clearance_m, 0.0);
    EXPECT_LT(clear.fresnel_v, 0.0);
    EXPECT_GT(std::abs(clear.fresnel_coefficient), 0.5);
}

TEST(UrbanRooftopDiffraction, StandardFresnelParameterUsesSignedClearanceAndEdgeLegs) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const double elevation_deg = cardinal_skyline_deg(scene) - 1.0;
    gnss_sim::UrbanRooftopDiffractionStatus status{};
    const gnss_sim::UrbanRooftopDiffractionPath path =
        evaluate(elevation_deg, signal(gnss_sim::SignalId::kGpsL1Ca), 0, 0.0, &status);

    ASSERT_EQ(status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    const double expected_v =
        path.signed_clearance_m *
        std::sqrt(2.0 * path.edge_path_euclidean_range_m /
                  (path.wavelength_m * path.source_edge_distance_m * path.receiver_edge_distance_m));
    EXPECT_NEAR(path.fresnel_v, expected_v, 1.0e-12);
    EXPECT_NEAR(path.fresnel_v, 0.20497, 1.0e-4);

    const double paraxial_v_squared = 4.0 * path.excess_path_length_m / path.wavelength_m;
    EXPECT_NEAR(path.fresnel_v * path.fresnel_v, paraxial_v_squared, 1.0e-4 * paraxial_v_squared);
}

TEST(UrbanRooftopDiffraction, GeometryAndComplexFieldRemainContinuousNearSkyline) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const double skyline_deg = cardinal_skyline_deg(scene);
    gnss_sim::UrbanRooftopDiffractionStatus below_status{};
    gnss_sim::UrbanRooftopDiffractionStatus above_status{};

    const gnss_sim::UrbanRooftopDiffractionPath below =
        evaluate(skyline_deg - 0.01, signal(gnss_sim::SignalId::kGpsL1Ca), 0, 0.0, &below_status);
    const gnss_sim::UrbanRooftopDiffractionPath above =
        evaluate(skyline_deg + 0.01, signal(gnss_sim::SignalId::kGpsL1Ca), 0, 0.0, &above_status);

    ASSERT_EQ(below_status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    ASSERT_EQ(above_status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    EXPECT_NEAR(below.excess_path_length_m, above.excess_path_length_m, 1.0e-9);
    EXPECT_LT(std::abs(below.fresnel_coefficient - above.fresnel_coefficient), 3.0e-3);
}

TEST(UrbanRooftopDiffraction, UsesCentralSignalWavelengthForFresnelParameter) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const double elevation_deg = cardinal_skyline_deg(scene) - 1.0;
    gnss_sim::UrbanRooftopDiffractionStatus l1_status{};
    gnss_sim::UrbanRooftopDiffractionStatus l5_status{};

    const gnss_sim::UrbanRooftopDiffractionPath l1 =
        evaluate(elevation_deg, signal(gnss_sim::SignalId::kGpsL1Ca), 0, 0.0, &l1_status);
    const gnss_sim::UrbanRooftopDiffractionPath l5 =
        evaluate(elevation_deg, signal(gnss_sim::SignalId::kGpsL5Q), 0, 0.0, &l5_status);

    ASSERT_EQ(l1_status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    ASSERT_EQ(l5_status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    EXPECT_DOUBLE_EQ(l1.excess_path_length_m, l5.excess_path_length_m);
    EXPECT_LT(l1.wavelength_m, l5.wavelength_m);
    EXPECT_GT(l1.fresnel_v, l5.fresnel_v);
}

TEST(UrbanRooftopDiffraction, GlonassFdmaChannelChangesWavelengthAndResponse) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const double elevation_deg = cardinal_skyline_deg(scene) - 2.0;
    const gnss_sim::SignalDefinition& glonass = signal(gnss_sim::SignalId::kGlonassG1);
    gnss_sim::UrbanRooftopDiffractionStatus low_status{};
    gnss_sim::UrbanRooftopDiffractionStatus high_status{};

    const gnss_sim::UrbanRooftopDiffractionPath low = evaluate(elevation_deg, glonass, -7, 0.0, &low_status);
    const gnss_sim::UrbanRooftopDiffractionPath high = evaluate(elevation_deg, glonass, 6, 0.0, &high_status);

    ASSERT_EQ(low_status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    ASSERT_EQ(high_status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    EXPECT_NE(low.carrier_frequency_hz, high.carrier_frequency_hz);
    EXPECT_NE(low.wavelength_m, high.wavelength_m);
    EXPECT_NE(low.fresnel_v, high.fresnel_v);
    EXPECT_NE(low.fresnel_coefficient, high.fresnel_coefficient);
}

TEST(UrbanRooftopDiffraction, ExcessPathUsesEuclideanGeometryThenAddsRtklibBaseline) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const double elevation_deg = cardinal_skyline_deg(scene) - 2.0;
    gnss_sim::UrbanRooftopDiffractionStatus zero_status{};
    gnss_sim::UrbanRooftopDiffractionStatus shifted_status{};

    const gnss_sim::UrbanRooftopDiffractionPath zero =
        evaluate(elevation_deg, signal(gnss_sim::SignalId::kGpsL1Ca), 0, 0.0, &zero_status);
    const gnss_sim::UrbanRooftopDiffractionPath shifted =
        evaluate(elevation_deg, signal(gnss_sim::SignalId::kGpsL1Ca), 0, 25.0, &shifted_status);

    ASSERT_EQ(zero_status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    ASSERT_EQ(shifted_status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    EXPECT_DOUBLE_EQ(zero.excess_path_length_m, shifted.excess_path_length_m);
    EXPECT_DOUBLE_EQ(zero.fresnel_v, shifted.fresnel_v);
    EXPECT_EQ(zero.fresnel_coefficient, shifted.fresnel_coefficient);
    EXPECT_NEAR(shifted.model_path_range_m - zero.model_path_range_m, 25.0, 1.0e-9);
}

TEST(UrbanRooftopDiffraction, BelowLocalHorizonIsExplicitlyNonApplicable) {
    gnss_sim::UrbanRooftopDiffractionStatus status{};
    const gnss_sim::UrbanRooftopDiffractionPath path =
        evaluate(-1.0, signal(gnss_sim::SignalId::kGpsL1Ca), 0, 0.0, &status);
    EXPECT_EQ(status, gnss_sim::UrbanRooftopDiffractionStatus::BELOW_LOCAL_HORIZON);
    EXPECT_EQ(path.wall_id, gnss_sim::UrbanWallId::NONE);
}

TEST(UrbanRooftopDiffraction, RepeatedEvaluationIsNumericallyIdentical) {
    const gnss_sim::UrbanSceneGeometryConfig scene = gnss_sim::default_urban_scene_geometry_config();
    const double elevation_deg = cardinal_skyline_deg(scene) - 0.5;
    gnss_sim::UrbanRooftopDiffractionStatus first_status{};
    gnss_sim::UrbanRooftopDiffractionStatus second_status{};

    const gnss_sim::UrbanRooftopDiffractionPath first =
        evaluate(elevation_deg, signal(gnss_sim::SignalId::kGpsL1Ca), 0, 17.0, &first_status);
    const gnss_sim::UrbanRooftopDiffractionPath second =
        evaluate(elevation_deg, signal(gnss_sim::SignalId::kGpsL1Ca), 0, 17.0, &second_status);

    ASSERT_EQ(first_status, second_status);
    ASSERT_EQ(first_status, gnss_sim::UrbanRooftopDiffractionStatus::VALID);
    EXPECT_DOUBLE_EQ(first.diffraction_point_enu_m.east_m, second.diffraction_point_enu_m.east_m);
    EXPECT_DOUBLE_EQ(first.diffraction_point_enu_m.north_m, second.diffraction_point_enu_m.north_m);
    EXPECT_DOUBLE_EQ(first.diffraction_point_enu_m.up_m, second.diffraction_point_enu_m.up_m);
    EXPECT_DOUBLE_EQ(first.signed_clearance_m, second.signed_clearance_m);
    EXPECT_DOUBLE_EQ(first.fresnel_v, second.fresnel_v);
    EXPECT_EQ(first.fresnel_coefficient, second.fresnel_coefficient);
    EXPECT_EQ(first.edge_reference_coefficient, second.edge_reference_coefficient);
}

} // namespace
