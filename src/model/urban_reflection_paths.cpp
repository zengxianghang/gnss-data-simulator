#include "model/urban_reflection_paths.h"

#include <algorithm>
#include <cmath>

namespace gnss_sim {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kSideToleranceM = 1.0e-9;
constexpr double kSegmentParameterTolerance = 1.0e-12;
constexpr double kFacadeHeightToleranceM = 1.0e-9;
constexpr double kExcessLengthToleranceM = 1.0e-6;
constexpr double kDirectionTolerance = 1.0e-10;

constexpr UrbanWallId kStableWallOrder[kUrbanFirstOrderWallCount] = {
    UrbanWallId::NORTH,
    UrbanWallId::EAST,
    UrbanWallId::SOUTH,
    UrbanWallId::WEST,
};

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool finite_positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool finite_point(const EnuPoint& point) {
    return std::isfinite(point.east_m) && std::isfinite(point.north_m) && std::isfinite(point.up_m);
}

bool finite_vector3(const double value[3]) {
    return value != nullptr && std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

double dot3(const double left[3], const double right[3]) {
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

void point_difference(const EnuPoint& left, const EnuPoint& right, double difference[3]) {
    difference[0] = left.east_m - right.east_m;
    difference[1] = left.north_m - right.north_m;
    difference[2] = left.up_m - right.up_m;
}

double signed_plane_distance(const EnuPoint& point, const UrbanWallPlane& plane) {
    double offset[3]{};
    point_difference(point, plane.plane_anchor_enu_m, offset);
    return dot3(offset, plane.inward_normal_enu);
}

EnuPoint add_scaled(const EnuPoint& point, const double direction[3], double scale) {
    return {point.east_m + direction[0] * scale, point.north_m + direction[1] * scale,
            point.up_m + direction[2] * scale};
}

double point_distance(const EnuPoint& first, const EnuPoint& second) {
    double difference[3]{};
    point_difference(first, second, difference);
    return std::sqrt(dot3(difference, difference));
}

bool unit_direction(const EnuPoint& from, const EnuPoint& to, double direction[3], double* length_m) {
    if (direction == nullptr) {
        return false;
    }
    double difference[3]{};
    point_difference(to, from, difference);
    const double length = std::sqrt(dot3(difference, difference));
    if (!finite_positive(length)) {
        return false;
    }
    for (int index = 0; index < 3; ++index) {
        direction[index] = difference[index] / length;
    }
    if (length_m != nullptr) {
        *length_m = length;
    }
    return finite_vector3(direction);
}

bool segment_blocked_by_wall(const UrbanSceneGeometryConfig& scene_config, UrbanWallId wall_id,
                             const EnuPoint& segment_start, const EnuPoint& segment_end, bool* blocked,
                             std::string* error_message) {
    if (blocked == nullptr) {
        set_error(error_message, "urban reflection occlusion output is null");
        return false;
    }
    UrbanWallPlane wall{};
    if (!urban_wall_plane(scene_config, wall_id, &wall, error_message)) {
        return false;
    }

    const double start_distance = signed_plane_distance(segment_start, wall);
    const double end_distance = signed_plane_distance(segment_end, wall);
    const double denominator = end_distance - start_distance;
    if (!std::isfinite(start_distance) || !std::isfinite(end_distance) || !std::isfinite(denominator)) {
        set_error(error_message, "urban reflection occlusion plane distance is non-finite");
        return false;
    }
    if (std::fabs(denominator) <= kSideToleranceM) {
        *blocked = false;
        return true;
    }

    const double parameter = -start_distance / denominator;
    if (!std::isfinite(parameter) || parameter <= kSegmentParameterTolerance ||
        parameter >= 1.0 - kSegmentParameterTolerance) {
        *blocked = false;
        return true;
    }

    const double east = segment_start.east_m + parameter * (segment_end.east_m - segment_start.east_m);
    const double north = segment_start.north_m + parameter * (segment_end.north_m - segment_start.north_m);
    const double up = segment_start.up_m + parameter * (segment_end.up_m - segment_start.up_m);
    if (!std::isfinite(east) || !std::isfinite(north) || !std::isfinite(up)) {
        set_error(error_message, "urban reflection occlusion intersection is non-finite");
        return false;
    }

    *blocked = up >= -kFacadeHeightToleranceM && up <= scene_config.wall_height_m + kFacadeHeightToleranceM;
    return true;
}

bool segment_blocked_by_other_wall(const UrbanSceneGeometryConfig& scene_config, UrbanWallId target_wall,
                                   const EnuPoint& segment_start, const EnuPoint& segment_end, bool* blocked,
                                   std::string* error_message) {
    if (blocked == nullptr) {
        set_error(error_message, "urban reflection occlusion output is null");
        return false;
    }
    for (UrbanWallId wall_id : kStableWallOrder) {
        if (wall_id == target_wall) {
            continue;
        }
        bool wall_blocked = false;
        if (!segment_blocked_by_wall(scene_config, wall_id, segment_start, segment_end, &wall_blocked,
                                     error_message)) {
            return false;
        }
        if (wall_blocked) {
            *blocked = true;
            return true;
        }
    }
    *blocked = false;
    return true;
}

bool verify_specular_law(const double incident_direction[3], const double reflected_direction[3],
                         const double inward_normal[3]) {
    const double incident_normal = dot3(incident_direction, inward_normal);
    double expected[3]{};
    double squared_error = 0.0;
    for (int index = 0; index < 3; ++index) {
        expected[index] = incident_direction[index] - 2.0 * incident_normal * inward_normal[index];
        const double difference = expected[index] - reflected_direction[index];
        squared_error += difference * difference;
    }
    return std::isfinite(squared_error) && squared_error <= kDirectionTolerance * kDirectionTolerance;
}

} // namespace

bool reconstruct_urban_satellite_enu(const UrbanSceneGeometryConfig& scene_config, const ReceiverTruth& receiver,
                                     const SatelliteGeometry& satellite_geometry, EnuPoint* receiver_enu_m,
                                     EnuPoint* satellite_enu_m, double* direct_euclidean_range_m,
                                     std::string* error_message) {
    if (receiver_enu_m == nullptr || satellite_enu_m == nullptr || direct_euclidean_range_m == nullptr ||
        !validate_urban_scene_geometry_config(scene_config, error_message) ||
        !finite_vector3(receiver.position_ecef_m) || !finite_vector3(satellite_geometry.satellite_state.position_ecef_m) ||
        !std::isfinite(satellite_geometry.azimuth_rad) || !std::isfinite(satellite_geometry.elevation_rad) ||
        satellite_geometry.elevation_rad < -0.5 * kPi || satellite_geometry.elevation_rad > 0.5 * kPi) {
        set_error(error_message, "urban reflection satellite reconstruction request is invalid");
        return false;
    }

    double ecef_difference[3]{};
    double squared_distance = 0.0;
    for (int index = 0; index < 3; ++index) {
        ecef_difference[index] =
            satellite_geometry.satellite_state.position_ecef_m[index] - receiver.position_ecef_m[index];
        squared_distance += ecef_difference[index] * ecef_difference[index];
    }
    const double euclidean_range_m = std::sqrt(squared_distance);
    if (!finite_positive(euclidean_range_m)) {
        set_error(error_message, "urban reflection raw satellite-receiver range is invalid");
        return false;
    }

    const double cos_elevation = std::cos(satellite_geometry.elevation_rad);
    const double direction[3] = {
        cos_elevation * std::sin(satellite_geometry.azimuth_rad),
        cos_elevation * std::cos(satellite_geometry.azimuth_rad),
        std::sin(satellite_geometry.elevation_rad),
    };
    if (!finite_vector3(direction)) {
        set_error(error_message, "urban reflection local satellite direction is invalid");
        return false;
    }

    const EnuPoint local_receiver{0.0, 0.0, scene_config.antenna_height_m};
    const EnuPoint local_satellite = add_scaled(local_receiver, direction, euclidean_range_m);
    if (!finite_point(local_satellite)) {
        set_error(error_message, "urban reflection local satellite point is non-finite");
        return false;
    }

    *receiver_enu_m = local_receiver;
    *satellite_enu_m = local_satellite;
    *direct_euclidean_range_m = euclidean_range_m;
    return true;
}

bool compute_urban_one_wall_reflection(const UrbanSceneGeometryConfig& scene_config,
                                       const UrbanRfResolvedConfig& rf_config, UrbanWallId wall_id,
                                       const EnuPoint& satellite_enu_m, const EnuPoint& receiver_enu_m,
                                       double direct_model_range_m, double carrier_frequency_hz, double wavelength_m,
                                       UrbanFirstOrderReflectionPath* path, UrbanReflectionCandidateStatus* status,
                                       std::string* error_message) {
    if (path == nullptr || status == nullptr || !validate_urban_scene_geometry_config(scene_config, error_message) ||
        !validate_urban_rf_material_config(rf_config.material, error_message) ||
        !validate_urban_rf_antenna_config(rf_config.antenna, error_message) || !finite_point(satellite_enu_m) ||
        !finite_point(receiver_enu_m) || !finite_positive(direct_model_range_m) ||
        !finite_positive(carrier_frequency_hz) || !finite_positive(wavelength_m)) {
        set_error(error_message, "urban one-wall reflection request is invalid");
        return false;
    }

    UrbanWallPlane wall{};
    if (!urban_wall_plane(scene_config, wall_id, &wall, error_message)) {
        return false;
    }

    const double source_side_m = signed_plane_distance(satellite_enu_m, wall);
    const double receiver_side_m = signed_plane_distance(receiver_enu_m, wall);
    if (!std::isfinite(source_side_m) || !std::isfinite(receiver_side_m)) {
        set_error(error_message, "urban reflection wall-side calculation is non-finite");
        return false;
    }
    if (source_side_m <= kSideToleranceM || receiver_side_m <= kSideToleranceM) {
        *status = UrbanReflectionCandidateStatus::BACKSIDE;
        return true;
    }

    const EnuPoint image_source = add_scaled(satellite_enu_m, wall.inward_normal_enu, -2.0 * source_side_m);
    double receiver_to_image[3]{};
    point_difference(image_source, receiver_enu_m, receiver_to_image);
    const double denominator = dot3(receiver_to_image, wall.inward_normal_enu);
    if (!std::isfinite(denominator) || std::fabs(denominator) <= kSideToleranceM) {
        *status = UrbanReflectionCandidateStatus::NO_SPECULAR_INTERSECTION;
        return true;
    }

    const double parameter = -receiver_side_m / denominator;
    if (!std::isfinite(parameter) || parameter <= kSegmentParameterTolerance ||
        parameter >= 1.0 - kSegmentParameterTolerance) {
        *status = UrbanReflectionCandidateStatus::NO_SPECULAR_INTERSECTION;
        return true;
    }
    const EnuPoint reflection_point = add_scaled(receiver_enu_m, receiver_to_image, parameter);
    if (!finite_point(reflection_point)) {
        set_error(error_message, "urban reflection specular point is non-finite");
        return false;
    }
    if (reflection_point.up_m < -kFacadeHeightToleranceM ||
        reflection_point.up_m > scene_config.wall_height_m + kFacadeHeightToleranceM) {
        *status = UrbanReflectionCandidateStatus::OUTSIDE_FACADE_HEIGHT;
        return true;
    }

    bool source_occluded = false;
    if (!segment_blocked_by_other_wall(scene_config, wall_id, satellite_enu_m, reflection_point, &source_occluded,
                                       error_message)) {
        return false;
    }
    if (source_occluded) {
        *status = UrbanReflectionCandidateStatus::SOURCE_OCCLUDED;
        return true;
    }

    bool receiver_occluded = false;
    if (!segment_blocked_by_other_wall(scene_config, wall_id, reflection_point, receiver_enu_m, &receiver_occluded,
                                       error_message)) {
        return false;
    }
    if (receiver_occluded) {
        *status = UrbanReflectionCandidateStatus::RECEIVER_OCCLUDED;
        return true;
    }

    UrbanFirstOrderReflectionPath result{};
    result.wall_id = wall_id;
    result.satellite_enu_m = satellite_enu_m;
    result.receiver_enu_m = receiver_enu_m;
    result.reflection_point_enu_m = reflection_point;
    result.direct_model_range_m = direct_model_range_m;
    result.carrier_frequency_hz = carrier_frequency_hz;
    result.wavelength_m = wavelength_m;

    double incident_length_m = 0.0;
    double reflected_leg_length_m = 0.0;
    if (!unit_direction(satellite_enu_m, reflection_point, result.incident_direction_enu, &incident_length_m) ||
        !unit_direction(reflection_point, receiver_enu_m, result.reflected_direction_enu, &reflected_leg_length_m) ||
        !unit_direction(receiver_enu_m, reflection_point, result.arrival_direction_enu, nullptr)) {
        set_error(error_message, "urban reflection path direction calculation failed");
        return false;
    }
    if (!verify_specular_law(result.incident_direction_enu, result.reflected_direction_enu,
                             wall.inward_normal_enu)) {
        set_error(error_message, "urban reflection image solution violates the specular law");
        return false;
    }

    const double incidence_cosine = -dot3(result.incident_direction_enu, wall.inward_normal_enu);
    if (!std::isfinite(incidence_cosine) || incidence_cosine <= 0.0) {
        set_error(error_message, "urban reflection incidence angle is invalid");
        return false;
    }
    result.incidence_angle_rad = std::acos(std::clamp(incidence_cosine, 0.0, 1.0));
    result.arrival_elevation_rad = std::asin(std::clamp(result.arrival_direction_enu[2], -1.0, 1.0));
    if (!std::isfinite(result.incidence_angle_rad) || !std::isfinite(result.arrival_elevation_rad) ||
        result.arrival_elevation_rad < -kDirectionTolerance) {
        set_error(error_message, "urban reflection arrival geometry is invalid");
        return false;
    }
    result.arrival_elevation_rad = std::max(0.0, result.arrival_elevation_rad);

    result.direct_euclidean_range_m = point_distance(satellite_enu_m, receiver_enu_m);
    result.reflected_euclidean_range_m = incident_length_m + reflected_leg_length_m;
    result.excess_path_length_m = result.reflected_euclidean_range_m - result.direct_euclidean_range_m;
    if (!finite_positive(result.direct_euclidean_range_m) || !finite_positive(result.reflected_euclidean_range_m) ||
        !std::isfinite(result.excess_path_length_m) || result.excess_path_length_m < -kExcessLengthToleranceM) {
        set_error(error_message, "urban reflection excess-path calculation is invalid");
        return false;
    }
    if (result.excess_path_length_m < 0.0) {
        result.excess_path_length_m = 0.0;
    }
    result.model_path_range_m = direct_model_range_m + result.excess_path_length_m;
    result.excess_delay_sec = result.excess_path_length_m / kSpeedOfLightMps;
    result.excess_carrier_phase_rad = -2.0 * kPi * result.excess_path_length_m / wavelength_m;
    result.geometric_phase_factor = std::polar(1.0, result.excess_carrier_phase_rad);
    if (!std::isfinite(result.model_path_range_m) || !std::isfinite(result.excess_delay_sec) ||
        !std::isfinite(result.excess_carrier_phase_rad) || !std::isfinite(result.geometric_phase_factor.real()) ||
        !std::isfinite(result.geometric_phase_factor.imag())) {
        set_error(error_message, "urban reflection path phase/delay calculation is non-finite");
        return false;
    }

    if (!compute_low_e_curtain_wall_reflection(rf_config.material, carrier_frequency_hz, result.incidence_angle_rad,
                                               &result.rf_response, error_message) ||
        !evaluate_urban_antenna_response(rf_config.antenna, UrbanCircularPolarization::kRhcp,
                                         result.arrival_elevation_rad, &result.antenna_rhcp_voltage, error_message) ||
        !evaluate_urban_antenna_response(rf_config.antenna, UrbanCircularPolarization::kLhcp,
                                         result.arrival_elevation_rad, &result.antenna_lhcp_voltage, error_message)) {
        return false;
    }

    *path = result;
    *status = UrbanReflectionCandidateStatus::VALID;
    return true;
}

bool compute_urban_first_order_reflections(const UrbanSceneGeometryConfig& scene_config,
                                           const UrbanRfConfig& rf_config, const SignalDefinition& signal,
                                           int glonass_fcn, const ReceiverTruth& receiver,
                                           const SatelliteGeometry& satellite_geometry,
                                           UrbanFirstOrderReflectionSet* reflections, std::string* error_message) {
    if (reflections == nullptr || !finite_positive(satellite_geometry.geometric_range_m)) {
        set_error(error_message, "urban first-order reflection request is invalid");
        return false;
    }

    UrbanRfResolvedConfig resolved_rf{};
    if (!resolve_urban_rf_signal_config(rf_config, signal, &resolved_rf, error_message)) {
        return false;
    }
    double carrier_frequency_hz = 0.0;
    double wavelength_m = 0.0;
    if (!signal_carrier_frequency_hz(signal, glonass_fcn, &carrier_frequency_hz) ||
        !signal_wavelength_m(signal, glonass_fcn, &wavelength_m)) {
        set_error(error_message, "urban reflection signal frequency/wavelength resolution failed");
        return false;
    }

    UrbanFirstOrderReflectionSet result{};
    if (!reconstruct_urban_satellite_enu(scene_config, receiver, satellite_geometry, &result.receiver_enu_m,
                                         &result.satellite_enu_m, &result.direct_euclidean_range_m, error_message)) {
        return false;
    }
    result.direct_model_range_m = satellite_geometry.geometric_range_m;

    if (satellite_geometry.elevation_rad < 0.0) {
        for (int index = 0; index < kUrbanFirstOrderWallCount; ++index) {
            result.candidate_status[index] = UrbanReflectionCandidateStatus::NO_SPECULAR_INTERSECTION;
        }
        *reflections = result;
        return true;
    }

    for (int index = 0; index < kUrbanFirstOrderWallCount; ++index) {
        UrbanFirstOrderReflectionPath candidate{};
        UrbanReflectionCandidateStatus candidate_status = UrbanReflectionCandidateStatus::NO_SPECULAR_INTERSECTION;
        if (!compute_urban_one_wall_reflection(scene_config, resolved_rf, kStableWallOrder[index],
                                               result.satellite_enu_m, result.receiver_enu_m,
                                               satellite_geometry.geometric_range_m, carrier_frequency_hz, wavelength_m,
                                               &candidate, &candidate_status, error_message)) {
            return false;
        }
        result.candidate_status[index] = candidate_status;
        if (candidate_status == UrbanReflectionCandidateStatus::VALID) {
            if (result.path_count >= kUrbanFirstOrderWallCount) {
                set_error(error_message, "urban first-order reflection path capacity exceeded physical wall count");
                return false;
            }
            result.paths[result.path_count] = candidate;
            ++result.path_count;
        }
    }

    *reflections = result;
    return true;
}

const char* urban_reflection_candidate_status_name(UrbanReflectionCandidateStatus status) {
    switch (status) {
        case UrbanReflectionCandidateStatus::VALID:
            return "VALID";
        case UrbanReflectionCandidateStatus::BACKSIDE:
            return "BACKSIDE";
        case UrbanReflectionCandidateStatus::NO_SPECULAR_INTERSECTION:
            return "NO_SPECULAR_INTERSECTION";
        case UrbanReflectionCandidateStatus::OUTSIDE_FACADE_HEIGHT:
            return "OUTSIDE_FACADE_HEIGHT";
        case UrbanReflectionCandidateStatus::SOURCE_OCCLUDED:
            return "SOURCE_OCCLUDED";
        case UrbanReflectionCandidateStatus::RECEIVER_OCCLUDED:
            return "RECEIVER_OCCLUDED";
    }
    return "UNKNOWN";
}

} // namespace gnss_sim
