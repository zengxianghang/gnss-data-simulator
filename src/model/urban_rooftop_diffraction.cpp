#include "model/urban_rooftop_diffraction.h"

#include "model/urban_reflection_paths.h"

#include <algorithm>
#include <cmath>
#include <complex>

namespace gnss_sim {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kDirectionTolerance = 1.0e-12;
constexpr double kExcessToleranceM = 1.0e-9;
constexpr double kFresnelSeriesLimit = 1.5;
constexpr int kFresnelMaxIterations = 100;
constexpr double kFresnelEpsilon = 1.0e-14;
constexpr double kFresnelMin = 1.0e-300;

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

bool finite_complex(const std::complex<double>& value) {
    return std::isfinite(value.real()) && std::isfinite(value.imag());
}

void point_difference(const EnuPoint& left, const EnuPoint& right, double difference[3]) {
    difference[0] = left.east_m - right.east_m;
    difference[1] = left.north_m - right.north_m;
    difference[2] = left.up_m - right.up_m;
}

double dot3(const double left[3], const double right[3]) {
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

void cross3(const double left[3], const double right[3], double result[3]) {
    result[0] = left[1] * right[2] - left[2] * right[1];
    result[1] = left[2] * right[0] - left[0] * right[2];
    result[2] = left[0] * right[1] - left[1] * right[0];
}

double norm3(const double value[3]) {
    return std::sqrt(dot3(value, value));
}

EnuPoint add_scaled(const EnuPoint& point, const double direction[3], double scale) {
    return {point.east_m + direction[0] * scale, point.north_m + direction[1] * scale,
            point.up_m + direction[2] * scale};
}

bool unit_direction(const EnuPoint& from, const EnuPoint& to, double direction[3], double* distance_m) {
    if (direction == nullptr || distance_m == nullptr) {
        return false;
    }
    double difference[3]{};
    point_difference(to, from, difference);
    const double distance = norm3(difference);
    if (!finite_positive(distance)) {
        return false;
    }
    for (int index = 0; index < 3; ++index) {
        direction[index] = difference[index] / distance;
    }
    *distance_m = distance;
    return true;
}

bool fresnel_integrals(double argument, double* cosine_integral, double* sine_integral) {
    if (cosine_integral == nullptr || sine_integral == nullptr || !std::isfinite(argument)) {
        return false;
    }

    const double absolute_argument = std::fabs(argument);
    double sine_value = 0.0;
    double cosine_value = 0.0;

    if (absolute_argument < std::sqrt(kFresnelMin)) {
        cosine_value = absolute_argument;
    } else if (absolute_argument <= kFresnelSeriesLimit) {
        double sum = 0.0;
        double sine_sum = 0.0;
        double cosine_sum = absolute_argument;
        double sign = 1.0;
        const double factor = 0.5 * kPi * absolute_argument * absolute_argument;
        bool odd_term = true;
        double term = absolute_argument;
        int denominator = 3;

        for (int iteration = 1; iteration <= kFresnelMaxIterations; ++iteration) {
            term *= factor / static_cast<double>(iteration);
            sum += sign * term / static_cast<double>(denominator);
            const double convergence = std::max(1.0, std::fabs(sum)) * kFresnelEpsilon;
            if (odd_term) {
                sign = -sign;
                sine_sum = sum;
                sum = cosine_sum;
            } else {
                cosine_sum = sum;
                sum = sine_sum;
            }
            if (std::fabs(term) <= convergence) {
                break;
            }
            odd_term = !odd_term;
            denominator += 2;
        }
        sine_value = sine_sum;
        cosine_value = cosine_sum;
    } else {
        const double pi_x_squared = kPi * absolute_argument * absolute_argument;
        std::complex<double> b(1.0, -pi_x_squared);
        std::complex<double> c(1.0 / kFresnelMin, 0.0);
        std::complex<double> d = 1.0 / b;
        std::complex<double> h = d;
        int odd_index = -1;

        for (int iteration = 2; iteration <= kFresnelMaxIterations; ++iteration) {
            odd_index += 2;
            const double a = -static_cast<double>(odd_index * (odd_index + 1));
            b += std::complex<double>(4.0, 0.0);
            d = 1.0 / (a * d + b);
            c = b + a / c;
            const std::complex<double> delta = c * d;
            h *= delta;
            if (std::fabs(delta.real() - 1.0) + std::fabs(delta.imag()) <= kFresnelEpsilon) {
                break;
            }
        }

        h *= std::complex<double>(absolute_argument, -absolute_argument);
        const std::complex<double> phase = std::exp(std::complex<double>(0.0, 0.5 * pi_x_squared));
        const std::complex<double> integrals =
            std::complex<double>(0.5, 0.5) * (std::complex<double>(1.0, 0.0) - phase * h);
        cosine_value = integrals.real();
        sine_value = integrals.imag();
    }

    if (argument < 0.0) {
        cosine_value = -cosine_value;
        sine_value = -sine_value;
    }
    if (!std::isfinite(cosine_value) || !std::isfinite(sine_value)) {
        return false;
    }
    *cosine_integral = cosine_value;
    *sine_integral = sine_value;
    return true;
}

bool edge_coordinate_and_radius(const EnuPoint& point, const EnuPoint& edge_anchor, const double edge_direction[3],
                                double* edge_coordinate_m, double* perpendicular_radius_m) {
    if (edge_coordinate_m == nullptr || perpendicular_radius_m == nullptr) {
        return false;
    }
    double relative[3]{};
    point_difference(point, edge_anchor, relative);
    const double coordinate = dot3(relative, edge_direction);
    double perpendicular[3]{};
    for (int index = 0; index < 3; ++index) {
        perpendicular[index] = relative[index] - coordinate * edge_direction[index];
    }
    const double radius = norm3(perpendicular);
    if (!std::isfinite(coordinate) || !finite_positive(radius)) {
        return false;
    }
    *edge_coordinate_m = coordinate;
    *perpendicular_radius_m = radius;
    return true;
}

bool compute_signed_edge_clearance(const EnuPoint& receiver_enu_m, const EnuPoint& satellite_enu_m,
                                   const UrbanWallPlane& wall, const UrbanDirectPathGeometry& direct_geometry,
                                   double* signed_clearance_m) {
    if (signed_clearance_m == nullptr) {
        return false;
    }

    double direct_direction[3]{};
    double direct_distance_m = 0.0;
    if (!unit_direction(receiver_enu_m, satellite_enu_m, direct_direction, &direct_distance_m)) {
        return false;
    }
    (void)direct_distance_m;

    double normal_to_lines[3]{};
    cross3(direct_direction, wall.roof_edge_direction_enu, normal_to_lines);
    const double normal_length = norm3(normal_to_lines);
    if (!finite_positive(normal_length) || normal_length <= kDirectionTolerance) {
        return false;
    }

    double edge_offset[3]{};
    point_difference(wall.roof_edge_anchor_enu_m, receiver_enu_m, edge_offset);
    const double clearance_abs = std::fabs(dot3(edge_offset, normal_to_lines)) / normal_length;
    if (!std::isfinite(clearance_abs)) {
        return false;
    }

    if (direct_geometry.grazing_roof || clearance_abs <= kDirectionTolerance) {
        *signed_clearance_m = 0.0;
    } else if (direct_geometry.roof_clearance_m < 0.0) {
        *signed_clearance_m = clearance_abs;
    } else {
        *signed_clearance_m = -clearance_abs;
    }
    return true;
}

} // namespace

bool compute_complex_knife_edge_coefficient(double fresnel_v, std::complex<double>* coefficient,
                                            std::string* error_message) {
    if (coefficient == nullptr || !std::isfinite(fresnel_v)) {
        set_error(error_message, "knife-edge Fresnel coefficient request is invalid");
        return false;
    }

    double cosine_integral = 0.0;
    double sine_integral = 0.0;
    if (!fresnel_integrals(fresnel_v, &cosine_integral, &sine_integral)) {
        set_error(error_message, "knife-edge Fresnel integral evaluation failed");
        return false;
    }

    const std::complex<double> tail(0.5 - cosine_integral, -(0.5 - sine_integral));
    const std::complex<double> result = 0.5 * std::complex<double>(1.0, 1.0) * tail;
    if (!finite_complex(result)) {
        set_error(error_message, "knife-edge Fresnel coefficient is non-finite");
        return false;
    }
    *coefficient = result;
    return true;
}

bool compute_urban_rooftop_diffraction(const UrbanSceneGeometryConfig& scene_config,
                                       const SignalDefinition& signal, int glonass_fcn,
                                       const ReceiverTruth& receiver, const SatelliteGeometry& satellite_geometry,
                                       UrbanRooftopDiffractionPath* diffraction,
                                       UrbanRooftopDiffractionStatus* status, std::string* error_message) {
    if (diffraction == nullptr || status == nullptr || !validate_urban_scene_geometry_config(scene_config, error_message) ||
        !finite_positive(satellite_geometry.geometric_range_m)) {
        set_error(error_message, "urban rooftop diffraction request is invalid");
        return false;
    }

    double carrier_frequency_hz = 0.0;
    double wavelength_m = 0.0;
    if (!signal_carrier_frequency_hz(signal, glonass_fcn, &carrier_frequency_hz) ||
        !signal_wavelength_m(signal, glonass_fcn, &wavelength_m)) {
        set_error(error_message, "urban rooftop diffraction signal frequency/wavelength resolution failed");
        return false;
    }

    UrbanRooftopDiffractionPath result{};
    if (!reconstruct_urban_satellite_enu(scene_config, receiver, satellite_geometry, &result.receiver_enu_m,
                                         &result.satellite_enu_m, &result.direct_euclidean_range_m, error_message)) {
        return false;
    }
    result.direct_model_range_m = satellite_geometry.geometric_range_m;
    result.carrier_frequency_hz = carrier_frequency_hz;
    result.wavelength_m = wavelength_m;

    UrbanDirectPathGeometry direct_geometry{};
    if (!compute_urban_direct_path_geometry(scene_config, satellite_geometry.azimuth_rad,
                                            satellite_geometry.elevation_rad, &direct_geometry, error_message)) {
        return false;
    }
    if (!direct_geometry.above_local_horizon) {
        *diffraction = result;
        *status = UrbanRooftopDiffractionStatus::BELOW_LOCAL_HORIZON;
        return true;
    }
    if (direct_geometry.primary_wall == UrbanWallId::NONE || direct_geometry.first_wall_count <= 0) {
        *diffraction = result;
        *status = UrbanRooftopDiffractionStatus::NO_BLOCKING_ROOF_EDGE;
        return true;
    }

    UrbanWallPlane wall{};
    if (!urban_wall_plane(scene_config, direct_geometry.primary_wall, &wall, error_message)) {
        return false;
    }
    const double edge_direction_norm = norm3(wall.roof_edge_direction_enu);
    if (!std::isfinite(edge_direction_norm) || std::fabs(edge_direction_norm - 1.0) > kDirectionTolerance) {
        set_error(error_message, "urban rooftop edge direction is not unit length");
        return false;
    }

    double source_coordinate_m = 0.0;
    double receiver_coordinate_m = 0.0;
    double source_radius_m = 0.0;
    double receiver_radius_m = 0.0;
    if (!edge_coordinate_and_radius(result.satellite_enu_m, wall.roof_edge_anchor_enu_m,
                                    wall.roof_edge_direction_enu, &source_coordinate_m, &source_radius_m) ||
        !edge_coordinate_and_radius(result.receiver_enu_m, wall.roof_edge_anchor_enu_m,
                                    wall.roof_edge_direction_enu, &receiver_coordinate_m, &receiver_radius_m)) {
        set_error(error_message, "urban rooftop equivalent-edge geometry is degenerate");
        return false;
    }

    const double radius_sum_m = source_radius_m + receiver_radius_m;
    if (!finite_positive(radius_sum_m)) {
        set_error(error_message, "urban rooftop equivalent-edge radius sum is invalid");
        return false;
    }
    const double diffraction_coordinate_m =
        (receiver_radius_m * source_coordinate_m + source_radius_m * receiver_coordinate_m) / radius_sum_m;
    result.diffraction_point_enu_m =
        add_scaled(wall.roof_edge_anchor_enu_m, wall.roof_edge_direction_enu, diffraction_coordinate_m);
    if (!finite_point(result.diffraction_point_enu_m)) {
        set_error(error_message, "urban rooftop equivalent diffraction point is non-finite");
        return false;
    }

    double source_direction[3]{};
    double receiver_direction[3]{};
    if (!unit_direction(result.diffraction_point_enu_m, result.satellite_enu_m, source_direction,
                        &result.source_edge_distance_m) ||
        !unit_direction(result.diffraction_point_enu_m, result.receiver_enu_m, receiver_direction,
                        &result.receiver_edge_distance_m)) {
        set_error(error_message, "urban rooftop edge-leg geometry is invalid");
        return false;
    }

    result.edge_path_euclidean_range_m = result.source_edge_distance_m + result.receiver_edge_distance_m;
    double direction_sum_squared = 0.0;
    for (int index = 0; index < 3; ++index) {
        const double sum = source_direction[index] + receiver_direction[index];
        direction_sum_squared += sum * sum;
    }
    const double excess_denominator_m = result.edge_path_euclidean_range_m + result.direct_euclidean_range_m;
    if (!finite_positive(excess_denominator_m) || !std::isfinite(direction_sum_squared) ||
        direction_sum_squared < 0.0) {
        set_error(error_message, "urban rooftop stable excess-path geometry is invalid");
        return false;
    }
    result.excess_path_length_m = result.source_edge_distance_m * result.receiver_edge_distance_m *
                                  direction_sum_squared / excess_denominator_m;
    if (!std::isfinite(result.excess_path_length_m) || result.excess_path_length_m < -kExcessToleranceM) {
        set_error(error_message, "urban rooftop excess-path calculation is invalid");
        return false;
    }
    result.excess_path_length_m = std::max(0.0, result.excess_path_length_m);
    result.model_path_range_m = result.direct_model_range_m + result.excess_path_length_m;
    result.excess_delay_sec = result.excess_path_length_m / kSpeedOfLightMps;

    if (!compute_signed_edge_clearance(result.receiver_enu_m, result.satellite_enu_m, wall, direct_geometry,
                                       &result.signed_clearance_m)) {
        set_error(error_message, "urban rooftop signed Fresnel clearance calculation failed");
        return false;
    }

    const double fresnel_magnitude = std::sqrt(2.0 * result.excess_path_length_m / wavelength_m);
    if (!std::isfinite(fresnel_magnitude)) {
        set_error(error_message, "urban rooftop Fresnel parameter magnitude is non-finite");
        return false;
    }
    if (result.signed_clearance_m > 0.0) {
        result.fresnel_v = fresnel_magnitude;
    } else if (result.signed_clearance_m < 0.0) {
        result.fresnel_v = -fresnel_magnitude;
    } else {
        result.fresnel_v = 0.0;
    }

    if (!compute_complex_knife_edge_coefficient(result.fresnel_v, &result.fresnel_coefficient, error_message)) {
        return false;
    }
    result.excess_carrier_phase_rad = -2.0 * kPi * result.excess_path_length_m / wavelength_m;
    result.edge_geometric_phase_factor = std::polar(1.0, result.excess_carrier_phase_rad);
    result.edge_reference_coefficient = result.fresnel_coefficient * std::conj(result.edge_geometric_phase_factor);
    if (!std::isfinite(result.model_path_range_m) || !std::isfinite(result.excess_delay_sec) ||
        !std::isfinite(result.excess_carrier_phase_rad) || !finite_complex(result.edge_geometric_phase_factor) ||
        !finite_complex(result.edge_reference_coefficient)) {
        set_error(error_message, "urban rooftop diffraction phase/delay result is non-finite");
        return false;
    }

    result.wall_id = direct_geometry.primary_wall;
    *diffraction = result;
    *status = UrbanRooftopDiffractionStatus::VALID;
    return true;
}

const char* urban_rooftop_diffraction_status_name(UrbanRooftopDiffractionStatus status) {
    switch (status) {
        case UrbanRooftopDiffractionStatus::VALID:
            return "VALID";
        case UrbanRooftopDiffractionStatus::BELOW_LOCAL_HORIZON:
            return "BELOW_LOCAL_HORIZON";
        case UrbanRooftopDiffractionStatus::NO_BLOCKING_ROOF_EDGE:
            return "NO_BLOCKING_ROOF_EDGE";
    }
    return "UNKNOWN";
}

} // namespace gnss_sim
