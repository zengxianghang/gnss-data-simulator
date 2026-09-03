#include "model/urban_scene_geometry.h"

#include <algorithm>
#include <cmath>

namespace gnss_sim {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kHorizontalDirectionEpsilon = 1.0e-12;
constexpr double kIntersectionTieRelativeTolerance = 1.0e-10;
constexpr double kRoofGrazingRelativeTolerance = 1.0e-10;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool finite_positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

void set_vector3(double value[3], double first, double second, double third) {
    value[0] = first;
    value[1] = second;
    value[2] = third;
}

struct WallCandidate {
    UrbanWallId wall_id;
    double ray_distance_m;
};

bool add_candidate(UrbanWallId wall_id, double ray_component, double wall_coordinate_m, WallCandidate candidates[4],
                   int* candidate_count) {
    if (candidates == nullptr || candidate_count == nullptr ||
        std::fabs(ray_component) <= kHorizontalDirectionEpsilon) {
        return true;
    }
    const double ray_distance_m = wall_coordinate_m / ray_component;
    if (!std::isfinite(ray_distance_m) || ray_distance_m <= 0.0) {
        return true;
    }
    if (*candidate_count >= 4) {
        return false;
    }
    candidates[*candidate_count].wall_id = wall_id;
    candidates[*candidate_count].ray_distance_m = ray_distance_m;
    ++(*candidate_count);
    return true;
}

} // namespace

UrbanSceneGeometryConfig default_urban_scene_geometry_config() {
    UrbanSceneGeometryConfig config{};
    config.antenna_height_m = 1.5;
    config.wall_distance_m = 10.0;
    config.wall_height_m = 10.0;
    return config;
}

bool validate_urban_scene_geometry_config(const UrbanSceneGeometryConfig& config, std::string* error_message) {
    if (!std::isfinite(config.antenna_height_m) || config.antenna_height_m < 0.0) {
        set_error(error_message, "urban scene antenna height must be finite and non-negative");
        return false;
    }
    if (!finite_positive(config.wall_distance_m)) {
        set_error(error_message, "urban scene wall distance must be finite and positive");
        return false;
    }
    if (!finite_positive(config.wall_height_m) || config.wall_height_m <= config.antenna_height_m) {
        set_error(error_message, "urban scene wall height must be finite and above the antenna");
        return false;
    }
    return true;
}

bool urban_wall_plane(const UrbanSceneGeometryConfig& config, UrbanWallId wall_id, UrbanWallPlane* wall_plane,
                      std::string* error_message) {
    if (wall_plane == nullptr || !validate_urban_scene_geometry_config(config, error_message)) {
        if (wall_plane == nullptr) {
            set_error(error_message, "urban wall-plane output is null");
        }
        return false;
    }

    UrbanWallPlane result{};
    result.wall_id = wall_id;
    switch (wall_id) {
        case UrbanWallId::NORTH:
            result.plane_anchor_enu_m = {0.0, config.wall_distance_m, 0.0};
            set_vector3(result.inward_normal_enu, 0.0, -1.0, 0.0);
            set_vector3(result.horizontal_tangent_enu, 1.0, 0.0, 0.0);
            break;
        case UrbanWallId::EAST:
            result.plane_anchor_enu_m = {config.wall_distance_m, 0.0, 0.0};
            set_vector3(result.inward_normal_enu, -1.0, 0.0, 0.0);
            set_vector3(result.horizontal_tangent_enu, 0.0, 1.0, 0.0);
            break;
        case UrbanWallId::SOUTH:
            result.plane_anchor_enu_m = {0.0, -config.wall_distance_m, 0.0};
            set_vector3(result.inward_normal_enu, 0.0, 1.0, 0.0);
            set_vector3(result.horizontal_tangent_enu, 1.0, 0.0, 0.0);
            break;
        case UrbanWallId::WEST:
            result.plane_anchor_enu_m = {-config.wall_distance_m, 0.0, 0.0};
            set_vector3(result.inward_normal_enu, 1.0, 0.0, 0.0);
            set_vector3(result.horizontal_tangent_enu, 0.0, 1.0, 0.0);
            break;
        case UrbanWallId::NONE:
        default:
            set_error(error_message, "urban wall id is invalid");
            return false;
    }

    result.roof_edge_anchor_enu_m = result.plane_anchor_enu_m;
    result.roof_edge_anchor_enu_m.up_m = config.wall_height_m;
    for (int index = 0; index < 3; ++index) {
        result.roof_edge_direction_enu[index] = result.horizontal_tangent_enu[index];
    }

    *wall_plane = result;
    return true;
}

bool compute_urban_skyline_elevation(const UrbanSceneGeometryConfig& config, double azimuth_rad,
                                     double* skyline_elevation_rad, double* horizontal_distance_to_wall_m,
                                     std::string* error_message) {
    if (skyline_elevation_rad == nullptr || horizontal_distance_to_wall_m == nullptr ||
        !validate_urban_scene_geometry_config(config, error_message) || !std::isfinite(azimuth_rad)) {
        if (skyline_elevation_rad == nullptr || horizontal_distance_to_wall_m == nullptr) {
            set_error(error_message, "urban skyline output is null");
        } else if (!std::isfinite(azimuth_rad)) {
            set_error(error_message, "urban skyline azimuth is not finite");
        }
        return false;
    }

    const double east_horizontal = std::sin(azimuth_rad);
    const double north_horizontal = std::cos(azimuth_rad);
    const double dominant_horizontal = std::max(std::fabs(east_horizontal), std::fabs(north_horizontal));
    if (!std::isfinite(dominant_horizontal) || dominant_horizontal <= 0.0) {
        set_error(error_message, "urban skyline horizontal direction is invalid");
        return false;
    }

    const double horizontal_distance_m = config.wall_distance_m / dominant_horizontal;
    const double skyline_rad = std::atan2(config.wall_height_m - config.antenna_height_m, horizontal_distance_m);
    if (!std::isfinite(horizontal_distance_m) || !std::isfinite(skyline_rad)) {
        set_error(error_message, "urban skyline calculation produced a non-finite result");
        return false;
    }

    *horizontal_distance_to_wall_m = horizontal_distance_m;
    *skyline_elevation_rad = skyline_rad;
    return true;
}

bool compute_urban_direct_path_geometry(const UrbanSceneGeometryConfig& config, double azimuth_rad,
                                        double elevation_rad, UrbanDirectPathGeometry* geometry,
                                        std::string* error_message) {
    if (geometry == nullptr || !validate_urban_scene_geometry_config(config, error_message) ||
        !std::isfinite(azimuth_rad) || !std::isfinite(elevation_rad) || elevation_rad < -0.5 * kPi ||
        elevation_rad > 0.5 * kPi) {
        if (geometry == nullptr) {
            set_error(error_message, "urban direct-path output is null");
        } else if (!std::isfinite(azimuth_rad) || !std::isfinite(elevation_rad) || elevation_rad < -0.5 * kPi ||
                   elevation_rad > 0.5 * kPi) {
            set_error(error_message, "urban direct-path azimuth/elevation is invalid");
        }
        return false;
    }

    UrbanDirectPathGeometry result{};
    result.azimuth_rad = azimuth_rad;
    result.elevation_rad = elevation_rad;
    result.ray_origin_enu_m = {0.0, 0.0, config.antenna_height_m};
    result.primary_wall = UrbanWallId::NONE;
    result.above_local_horizon = elevation_rad >= 0.0;

    if (!compute_urban_skyline_elevation(config, azimuth_rad, &result.skyline_elevation_rad,
                                         &result.horizontal_distance_to_first_wall_m, error_message)) {
        return false;
    }

    const double cos_elevation = std::cos(elevation_rad);
    const double ray_east = cos_elevation * std::sin(azimuth_rad);
    const double ray_north = cos_elevation * std::cos(azimuth_rad);
    const double ray_up = std::sin(elevation_rad);
    set_vector3(result.ray_direction_enu, ray_east, ray_north, ray_up);

    WallCandidate candidates[4]{};
    int candidate_count = 0;
    if (!add_candidate(UrbanWallId::NORTH, ray_north, config.wall_distance_m, candidates, &candidate_count) ||
        !add_candidate(UrbanWallId::EAST, ray_east, config.wall_distance_m, candidates, &candidate_count) ||
        !add_candidate(UrbanWallId::SOUTH, ray_north, -config.wall_distance_m, candidates, &candidate_count) ||
        !add_candidate(UrbanWallId::WEST, ray_east, -config.wall_distance_m, candidates, &candidate_count)) {
        set_error(error_message, "urban direct-path wall candidate capacity exceeded");
        return false;
    }

    if (candidate_count == 0) {
        result.line_of_sight = result.above_local_horizon;
        *geometry = result;
        return true;
    }

    double first_distance_m = candidates[0].ray_distance_m;
    for (int index = 1; index < candidate_count; ++index) {
        first_distance_m = std::min(first_distance_m, candidates[index].ray_distance_m);
    }
    const double tie_tolerance_m = kIntersectionTieRelativeTolerance * std::max(1.0, first_distance_m);

    for (int index = 0; index < candidate_count; ++index) {
        if (std::fabs(candidates[index].ray_distance_m - first_distance_m) > tie_tolerance_m) {
            continue;
        }
        if (result.first_wall_count >= 2) {
            set_error(error_message, "urban direct path intersects more than two first walls");
            return false;
        }
        UrbanRayWallIntersection& intersection = result.first_wall_intersections[result.first_wall_count];
        intersection.wall_id = candidates[index].wall_id;
        intersection.ray_distance_m = candidates[index].ray_distance_m;
        intersection.point_enu_m.east_m = result.ray_origin_enu_m.east_m + ray_east * intersection.ray_distance_m;
        intersection.point_enu_m.north_m = result.ray_origin_enu_m.north_m + ray_north * intersection.ray_distance_m;
        intersection.point_enu_m.up_m = result.ray_origin_enu_m.up_m + ray_up * intersection.ray_distance_m;
        ++result.first_wall_count;
    }

    if (result.first_wall_count <= 0) {
        set_error(error_message, "urban direct path could not identify the first wall");
        return false;
    }

    result.primary_wall = result.first_wall_intersections[0].wall_id;
    result.roof_clearance_m = result.first_wall_intersections[0].point_enu_m.up_m - config.wall_height_m;
    const double grazing_tolerance_m = kRoofGrazingRelativeTolerance * std::max(1.0, config.wall_height_m);
    result.grazing_roof = std::fabs(result.roof_clearance_m) <= grazing_tolerance_m;
    result.blocked_by_wall = result.roof_clearance_m <= grazing_tolerance_m;
    result.line_of_sight = result.above_local_horizon && !result.blocked_by_wall;

    *geometry = result;
    return true;
}

const char* urban_wall_id_name(UrbanWallId wall_id) {
    switch (wall_id) {
        case UrbanWallId::NORTH:
            return "NORTH";
        case UrbanWallId::EAST:
            return "EAST";
        case UrbanWallId::SOUTH:
            return "SOUTH";
        case UrbanWallId::WEST:
            return "WEST";
        case UrbanWallId::NONE:
        default:
            return "NONE";
    }
}

} // namespace gnss_sim
