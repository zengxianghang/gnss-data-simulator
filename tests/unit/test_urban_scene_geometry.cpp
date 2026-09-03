#include "model/urban_scene_geometry.h"

#include <cmath>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;

TEST(UrbanSceneGeometry, DefaultConfigMatchesFrozenIssue115Scene) {
    const gnss_sim::UrbanSceneGeometryConfig config = gnss_sim::default_urban_scene_geometry_config();
    EXPECT_DOUBLE_EQ(config.antenna_height_m, 1.5);
    EXPECT_DOUBLE_EQ(config.wall_distance_m, 10.0);
    EXPECT_DOUBLE_EQ(config.wall_height_m, 10.0);

    std::string error_message;
    EXPECT_TRUE(gnss_sim::validate_urban_scene_geometry_config(config, &error_message)) << error_message;
}

TEST(UrbanSceneGeometry, CardinalAndCornerSkylinesMatchAnalyticalAnchors) {
    const gnss_sim::UrbanSceneGeometryConfig config = gnss_sim::default_urban_scene_geometry_config();
    std::string error_message;

    double skyline_rad = 0.0;
    double horizontal_distance_m = 0.0;
    ASSERT_TRUE(
        gnss_sim::compute_urban_skyline_elevation(config, 0.0, &skyline_rad, &horizontal_distance_m, &error_message))
        << error_message;
    EXPECT_NEAR(horizontal_distance_m, 10.0, 1.0e-12);
    EXPECT_NEAR(skyline_rad * kRadiansToDegrees, 40.36453657309736, 1.0e-10);

    ASSERT_TRUE(gnss_sim::compute_urban_skyline_elevation(config, 45.0 * kDegreesToRadians, &skyline_rad,
                                                          &horizontal_distance_m, &error_message))
        << error_message;
    EXPECT_NEAR(horizontal_distance_m, 10.0 * std::sqrt(2.0), 1.0e-12);
    EXPECT_NEAR(skyline_rad * kRadiansToDegrees, 31.007583006867137, 1.0e-10);
}

TEST(UrbanSceneGeometry, NorthWallBlocksBelowSkylineAndClearsAboveRoof) {
    const gnss_sim::UrbanSceneGeometryConfig config = gnss_sim::default_urban_scene_geometry_config();
    std::string error_message;

    gnss_sim::UrbanDirectPathGeometry blocked{};
    ASSERT_TRUE(
        gnss_sim::compute_urban_direct_path_geometry(config, 0.0, 30.0 * kDegreesToRadians, &blocked, &error_message))
        << error_message;
    EXPECT_FALSE(blocked.line_of_sight);
    EXPECT_TRUE(blocked.blocked_by_wall);
    EXPECT_FALSE(blocked.grazing_roof);
    EXPECT_EQ(blocked.primary_wall, gnss_sim::UrbanWallId::NORTH);
    ASSERT_EQ(blocked.first_wall_count, 1);
    EXPECT_NEAR(blocked.first_wall_intersections[0].point_enu_m.east_m, 0.0, 1.0e-12);
    EXPECT_NEAR(blocked.first_wall_intersections[0].point_enu_m.north_m, 10.0, 1.0e-12);
    EXPECT_NEAR(blocked.first_wall_intersections[0].point_enu_m.up_m, 7.273502691896257, 1.0e-12);
    EXPECT_LT(blocked.roof_clearance_m, 0.0);

    gnss_sim::UrbanDirectPathGeometry clear{};
    ASSERT_TRUE(
        gnss_sim::compute_urban_direct_path_geometry(config, 0.0, 50.0 * kDegreesToRadians, &clear, &error_message))
        << error_message;
    EXPECT_TRUE(clear.line_of_sight);
    EXPECT_FALSE(clear.blocked_by_wall);
    EXPECT_EQ(clear.primary_wall, gnss_sim::UrbanWallId::NORTH);
    EXPECT_GT(clear.roof_clearance_m, 0.0);
}

TEST(UrbanSceneGeometry, ExactCornerExposesBothFirstWallsWithoutHidingTie) {
    const gnss_sim::UrbanSceneGeometryConfig config = gnss_sim::default_urban_scene_geometry_config();
    std::string error_message;

    gnss_sim::UrbanDirectPathGeometry geometry{};
    ASSERT_TRUE(gnss_sim::compute_urban_direct_path_geometry(config, 45.0 * kDegreesToRadians, 30.0 * kDegreesToRadians,
                                                             &geometry, &error_message))
        << error_message;
    EXPECT_FALSE(geometry.line_of_sight);
    EXPECT_TRUE(geometry.blocked_by_wall);
    ASSERT_EQ(geometry.first_wall_count, 2);
    EXPECT_EQ(geometry.first_wall_intersections[0].wall_id, gnss_sim::UrbanWallId::NORTH);
    EXPECT_EQ(geometry.first_wall_intersections[1].wall_id, gnss_sim::UrbanWallId::EAST);
    for (int index = 0; index < geometry.first_wall_count; ++index) {
        EXPECT_NEAR(geometry.first_wall_intersections[index].point_enu_m.east_m, 10.0, 1.0e-10);
        EXPECT_NEAR(geometry.first_wall_intersections[index].point_enu_m.north_m, 10.0, 1.0e-10);
        EXPECT_NEAR(geometry.first_wall_intersections[index].point_enu_m.up_m, 9.66496580927726, 1.0e-10);
    }
}

TEST(UrbanSceneGeometry, CardinalRotationKeepsGeometrySymmetric) {
    const gnss_sim::UrbanSceneGeometryConfig config = gnss_sim::default_urban_scene_geometry_config();
    const double azimuths_deg[] = {0.0, 90.0, 180.0, 270.0};
    const gnss_sim::UrbanWallId walls[] = {gnss_sim::UrbanWallId::NORTH, gnss_sim::UrbanWallId::EAST,
                                           gnss_sim::UrbanWallId::SOUTH, gnss_sim::UrbanWallId::WEST};
    std::string error_message;

    for (int index = 0; index < 4; ++index) {
        gnss_sim::UrbanDirectPathGeometry geometry{};
        ASSERT_TRUE(gnss_sim::compute_urban_direct_path_geometry(config, azimuths_deg[index] * kDegreesToRadians,
                                                                 30.0 * kDegreesToRadians, &geometry, &error_message))
            << error_message;
        EXPECT_FALSE(geometry.line_of_sight);
        EXPECT_TRUE(geometry.blocked_by_wall);
        EXPECT_EQ(geometry.primary_wall, walls[index]);
        EXPECT_EQ(geometry.first_wall_count, 1);
        EXPECT_NEAR(geometry.horizontal_distance_to_first_wall_m, 10.0, 1.0e-12);
        EXPECT_NEAR(geometry.roof_clearance_m, -2.726497308103743, 1.0e-10);
    }
}

TEST(UrbanSceneGeometry, GrazingRoofIsDeterministicallyClassifiedAsBlocked) {
    const gnss_sim::UrbanSceneGeometryConfig config = gnss_sim::default_urban_scene_geometry_config();
    const double grazing_elevation_rad = std::atan2(8.5, 10.0);
    std::string error_message;

    gnss_sim::UrbanDirectPathGeometry geometry{};
    ASSERT_TRUE(
        gnss_sim::compute_urban_direct_path_geometry(config, 0.0, grazing_elevation_rad, &geometry, &error_message))
        << error_message;
    EXPECT_FALSE(geometry.line_of_sight);
    EXPECT_TRUE(geometry.blocked_by_wall);
    EXPECT_TRUE(geometry.grazing_roof);
    EXPECT_NEAR(geometry.roof_clearance_m, 0.0, 1.0e-10);
    EXPECT_EQ(geometry.primary_wall, gnss_sim::UrbanWallId::NORTH);
}

TEST(UrbanSceneGeometry, WallPlaneExposesStableRoofEdgeForDownstreamDiffraction) {
    const gnss_sim::UrbanSceneGeometryConfig config = gnss_sim::default_urban_scene_geometry_config();
    std::string error_message;

    gnss_sim::UrbanWallPlane east{};
    ASSERT_TRUE(gnss_sim::urban_wall_plane(config, gnss_sim::UrbanWallId::EAST, &east, &error_message))
        << error_message;
    EXPECT_DOUBLE_EQ(east.plane_anchor_enu_m.east_m, 10.0);
    EXPECT_DOUBLE_EQ(east.plane_anchor_enu_m.north_m, 0.0);
    EXPECT_DOUBLE_EQ(east.roof_edge_anchor_enu_m.east_m, 10.0);
    EXPECT_DOUBLE_EQ(east.roof_edge_anchor_enu_m.up_m, 10.0);
    EXPECT_DOUBLE_EQ(east.inward_normal_enu[0], -1.0);
    EXPECT_DOUBLE_EQ(east.inward_normal_enu[1], 0.0);
    EXPECT_DOUBLE_EQ(east.roof_edge_direction_enu[0], 0.0);
    EXPECT_DOUBLE_EQ(east.roof_edge_direction_enu[1], 1.0);
}

TEST(UrbanSceneGeometry, ZenithDoesNotIntersectAnyWall) {
    const gnss_sim::UrbanSceneGeometryConfig config = gnss_sim::default_urban_scene_geometry_config();
    std::string error_message;

    gnss_sim::UrbanDirectPathGeometry geometry{};
    ASSERT_TRUE(gnss_sim::compute_urban_direct_path_geometry(config, 123.0 * kDegreesToRadians, 0.5 * kPi, &geometry,
                                                             &error_message))
        << error_message;
    EXPECT_TRUE(geometry.line_of_sight);
    EXPECT_FALSE(geometry.blocked_by_wall);
    EXPECT_EQ(geometry.primary_wall, gnss_sim::UrbanWallId::NONE);
    EXPECT_EQ(geometry.first_wall_count, 0);
}

TEST(UrbanSceneGeometry, InvalidConfigurationFailsClearly) {
    gnss_sim::UrbanSceneGeometryConfig config = gnss_sim::default_urban_scene_geometry_config();
    config.wall_distance_m = 0.0;
    std::string error_message;
    EXPECT_FALSE(gnss_sim::validate_urban_scene_geometry_config(config, &error_message));
    EXPECT_FALSE(error_message.empty());

    config = gnss_sim::default_urban_scene_geometry_config();
    config.wall_height_m = config.antenna_height_m;
    error_message.clear();
    EXPECT_FALSE(gnss_sim::validate_urban_scene_geometry_config(config, &error_message));
    EXPECT_FALSE(error_message.empty());
}

TEST(UrbanSceneGeometry, RepeatedEvaluationIsNumericallyIdentical) {
    const gnss_sim::UrbanSceneGeometryConfig config = gnss_sim::default_urban_scene_geometry_config();
    std::string error_message;
    gnss_sim::UrbanDirectPathGeometry first{};
    gnss_sim::UrbanDirectPathGeometry second{};

    ASSERT_TRUE(gnss_sim::compute_urban_direct_path_geometry(config, 27.0 * kDegreesToRadians, 22.0 * kDegreesToRadians,
                                                             &first, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::compute_urban_direct_path_geometry(config, 27.0 * kDegreesToRadians, 22.0 * kDegreesToRadians,
                                                             &second, &error_message))
        << error_message;

    EXPECT_EQ(first.primary_wall, second.primary_wall);
    EXPECT_EQ(first.first_wall_count, second.first_wall_count);
    EXPECT_DOUBLE_EQ(first.skyline_elevation_rad, second.skyline_elevation_rad);
    EXPECT_DOUBLE_EQ(first.horizontal_distance_to_first_wall_m, second.horizontal_distance_to_first_wall_m);
    EXPECT_DOUBLE_EQ(first.roof_clearance_m, second.roof_clearance_m);
    ASSERT_EQ(first.first_wall_count, second.first_wall_count);
    for (int index = 0; index < first.first_wall_count; ++index) {
        EXPECT_EQ(first.first_wall_intersections[index].wall_id, second.first_wall_intersections[index].wall_id);
        EXPECT_DOUBLE_EQ(first.first_wall_intersections[index].ray_distance_m,
                         second.first_wall_intersections[index].ray_distance_m);
        EXPECT_DOUBLE_EQ(first.first_wall_intersections[index].point_enu_m.east_m,
                         second.first_wall_intersections[index].point_enu_m.east_m);
        EXPECT_DOUBLE_EQ(first.first_wall_intersections[index].point_enu_m.north_m,
                         second.first_wall_intersections[index].point_enu_m.north_m);
        EXPECT_DOUBLE_EQ(first.first_wall_intersections[index].point_enu_m.up_m,
                         second.first_wall_intersections[index].point_enu_m.up_m);
    }
}

} // namespace
