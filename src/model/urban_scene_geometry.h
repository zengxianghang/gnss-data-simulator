#ifndef GNSS_SIM_SRC_MODEL_URBAN_SCENE_GEOMETRY_H_
#define GNSS_SIM_SRC_MODEL_URBAN_SCENE_GEOMETRY_H_

#include <string>

namespace gnss_sim {

enum class UrbanWallId {
    NONE = 0,
    NORTH,
    EAST,
    SOUTH,
    WEST,
};

struct EnuPoint {
    double east_m;
    double north_m;
    double up_m;
};

struct UrbanSceneGeometryConfig {
    double antenna_height_m;
    double wall_distance_m;
    double wall_height_m;
};

struct UrbanWallPlane {
    UrbanWallId wall_id;
    EnuPoint plane_anchor_enu_m;
    double inward_normal_enu[3];
    double horizontal_tangent_enu[3];
    EnuPoint roof_edge_anchor_enu_m;
    double roof_edge_direction_enu[3];
};

struct UrbanRayWallIntersection {
    UrbanWallId wall_id;
    EnuPoint point_enu_m;
    double ray_distance_m;
};

struct UrbanDirectPathGeometry {
    double azimuth_rad;
    double elevation_rad;
    EnuPoint ray_origin_enu_m;
    double ray_direction_enu[3];
    double skyline_elevation_rad;
    double horizontal_distance_to_first_wall_m;
    double roof_clearance_m;
    UrbanWallId primary_wall;
    UrbanRayWallIntersection first_wall_intersections[2];
    int first_wall_count;
    bool above_local_horizon;
    bool line_of_sight;
    bool blocked_by_wall;
    bool grazing_roof;
};

UrbanSceneGeometryConfig default_urban_scene_geometry_config();
bool validate_urban_scene_geometry_config(const UrbanSceneGeometryConfig& config, std::string* error_message);
bool urban_wall_plane(const UrbanSceneGeometryConfig& config, UrbanWallId wall_id, UrbanWallPlane* wall_plane,
                      std::string* error_message);
bool compute_urban_skyline_elevation(const UrbanSceneGeometryConfig& config, double azimuth_rad,
                                     double* skyline_elevation_rad, double* horizontal_distance_to_wall_m,
                                     std::string* error_message);
bool compute_urban_direct_path_geometry(const UrbanSceneGeometryConfig& config, double azimuth_rad,
                                        double elevation_rad, UrbanDirectPathGeometry* geometry,
                                        std::string* error_message);
const char* urban_wall_id_name(UrbanWallId wall_id);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_URBAN_SCENE_GEOMETRY_H_
