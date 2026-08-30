#include "gnss/satellite_engine.h"

#include "gnss_sim/sim_time.h"

#include <cmath>

namespace gnss_sim {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kEarthRotationRateRadPerSec = 7.2921151467e-5;
constexpr double kInitialPropagationTimeSec = 0.075;
constexpr double kPropagationConvergenceSec = 1.0e-11;
constexpr int kMaxPropagationIterations = 12;
constexpr double kDegreesToRadians = 3.141592653589793238462643383279502884 / 180.0;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool finite_vector3(const double value[3]) {
    return value != nullptr && std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

bool compute_range_rate(const RtklibSatelliteState& satellite_state, const ReceiverTruth& receiver,
                        const double line_of_sight_ecef[3], double* range_rate_mps) {
    if (range_rate_mps == nullptr || !finite_vector3(satellite_state.position_ecef_m) ||
        !finite_vector3(satellite_state.velocity_ecef_mps) || !finite_vector3(receiver.position_ecef_m) ||
        !finite_vector3(receiver.velocity_ecef_mps) || !finite_vector3(line_of_sight_ecef)) {
        return false;
    }

    double relative_velocity_ecef_mps[3]{};
    for (int index = 0; index < 3; ++index) {
        relative_velocity_ecef_mps[index] =
            satellite_state.velocity_ecef_mps[index] - receiver.velocity_ecef_mps[index];
    }

    double rate_mps = 0.0;
    for (int index = 0; index < 3; ++index) {
        rate_mps += relative_velocity_ecef_mps[index] * line_of_sight_ecef[index];
    }

    // Match RTKLIB pntpos.c/resdop(): range rate includes the time derivative
    // of geodist()'s first-order Earth-rotation correction.
    rate_mps += kEarthRotationRateRadPerSec / kSpeedOfLightMps *
                (satellite_state.velocity_ecef_mps[1] * receiver.position_ecef_m[0] +
                 satellite_state.position_ecef_m[1] * receiver.velocity_ecef_mps[0] -
                 satellite_state.velocity_ecef_mps[0] * receiver.position_ecef_m[1] -
                 satellite_state.position_ecef_m[0] * receiver.velocity_ecef_mps[1]);

    *range_rate_mps = rate_mps;
    return std::isfinite(rate_mps);
}

struct RtklibStateProviderContext {
    const RtklibNavStore* nav_store;
    int selection_gps_week;
    double selection_sow_sec;
};

bool rtklib_state_provider(const void* context, int gps_week, double sow_sec, int satellite_number,
                           RtklibSatelliteState* state, std::string* error_message) {
    const RtklibStateProviderContext* provider_context = static_cast<const RtklibStateProviderContext*>(context);
    if (provider_context == nullptr) {
        set_error(error_message, "RTKLIB satellite-state provider context is null");
        return false;
    }
    return get_rtklib_satellite_state_with_selection_time(
        provider_context->nav_store, gps_week, sow_sec, provider_context->selection_gps_week,
        provider_context->selection_sow_sec, satellite_number, state, error_message);
}

} // namespace

bool subtract_propagation_time(const SimTime& receive_time, double propagation_time_sec, int* transmit_gps_week,
                               double* transmit_sow_sec) {
    if (transmit_gps_week == nullptr || transmit_sow_sec == nullptr || receive_time.gps_week < 0 ||
        receive_time.tow_ns < 0 || receive_time.tow_ns >= GPS_WEEK_NANOSECONDS ||
        !std::isfinite(propagation_time_sec) || propagation_time_sec < 0.0) {
        return false;
    }

    int week = receive_time.gps_week;
    double sow_sec =
        static_cast<double>(receive_time.tow_ns) / static_cast<double>(NANOSECONDS_PER_SECOND) - propagation_time_sec;
    while (sow_sec < 0.0) {
        if (week == 0) {
            return false;
        }
        --week;
        sow_sec += static_cast<double>(GPS_WEEK_SECONDS);
    }
    while (sow_sec >= static_cast<double>(GPS_WEEK_SECONDS)) {
        ++week;
        sow_sec -= static_cast<double>(GPS_WEEK_SECONDS);
    }

    *transmit_gps_week = week;
    *transmit_sow_sec = sow_sec;
    return true;
}

bool elevation_passes_mask(double elevation_rad, double elevation_mask_deg) {
    if (!std::isfinite(elevation_rad) || !std::isfinite(elevation_mask_deg) || elevation_mask_deg < -90.0 ||
        elevation_mask_deg > 90.0) {
        return false;
    }
    return elevation_rad >= elevation_mask_deg * kDegreesToRadians;
}

bool compute_satellite_geometry_with_provider(SatelliteStateProvider state_provider, const void* state_context,
                                              const ReceiverTruth& receiver, const SimTime& receive_time,
                                              int satellite_number, double elevation_mask_deg,
                                              SatelliteGeometry* geometry, std::string* error_message) {
    if (state_provider == nullptr || state_context == nullptr || geometry == nullptr || satellite_number <= 0 ||
        receive_time.gps_week < 0 || receive_time.tow_ns < 0 || receive_time.tow_ns >= GPS_WEEK_NANOSECONDS ||
        !finite_vector3(receiver.position_ecef_m) || !finite_vector3(receiver.velocity_ecef_mps) ||
        !std::isfinite(elevation_mask_deg) || elevation_mask_deg < -90.0 || elevation_mask_deg > 90.0) {
        set_error(error_message, "satellite-geometry request has invalid arguments");
        return false;
    }

    SatelliteGeometry result{};
    result.receive_time = receive_time;
    result.satellite_number = satellite_number;

    double propagation_time_sec = kInitialPropagationTimeSec;
    bool converged = false;
    for (int iteration = 0; iteration < kMaxPropagationIterations; ++iteration) {
        int transmit_week = 0;
        double transmit_sow_sec = 0.0;
        if (!subtract_propagation_time(receive_time, propagation_time_sec, &transmit_week, &transmit_sow_sec)) {
            set_error(error_message, "cannot derive satellite transmit time from receive time");
            return false;
        }

        RtklibSatelliteState satellite_state{};
        if (!state_provider(state_context, transmit_week, transmit_sow_sec, satellite_number, &satellite_state,
                            error_message)) {
            return false;
        }

        double line_of_sight_ecef[3]{};
        double geometric_range_m = 0.0;
        if (!rtklib_geometric_distance(satellite_state.position_ecef_m, receiver.position_ecef_m, &geometric_range_m,
                                       line_of_sight_ecef)) {
            set_error(error_message, "RTKLIB geometric-distance calculation failed");
            return false;
        }

        const double next_propagation_time_sec = geometric_range_m / kSpeedOfLightMps;
        if (!std::isfinite(next_propagation_time_sec) || next_propagation_time_sec <= 0.0) {
            set_error(error_message, "computed propagation time is invalid");
            return false;
        }

        result.transmit_gps_week = transmit_week;
        result.transmit_sow_sec = transmit_sow_sec;
        result.satellite_state = satellite_state;
        result.geometric_range_m = geometric_range_m;
        result.propagation_time_sec = next_propagation_time_sec;
        result.iteration_count = iteration + 1;
        for (int index = 0; index < 3; ++index) {
            result.line_of_sight_ecef[index] = line_of_sight_ecef[index];
        }

        if (std::fabs(next_propagation_time_sec - propagation_time_sec) <= kPropagationConvergenceSec) {
            converged = true;
            break;
        }
        propagation_time_sec = next_propagation_time_sec;
    }

    if (!converged) {
        set_error(error_message, "satellite transmit-time iteration did not converge");
        return false;
    }

    // Re-evaluate the state at the converged propagation time so transmit_time,
    // satellite_state, and geometric_range describe the same final iterate.
    if (!subtract_propagation_time(receive_time, result.propagation_time_sec, &result.transmit_gps_week,
                                   &result.transmit_sow_sec) ||
        !state_provider(state_context, result.transmit_gps_week, result.transmit_sow_sec, satellite_number,
                        &result.satellite_state, error_message) ||
        !rtklib_geometric_distance(result.satellite_state.position_ecef_m, receiver.position_ecef_m,
                                   &result.geometric_range_m, result.line_of_sight_ecef) ||
        !rtklib_azimuth_elevation(receiver.position_ecef_m, result.line_of_sight_ecef, &result.azimuth_rad,
                                  &result.elevation_rad) ||
        !compute_range_rate(result.satellite_state, receiver, result.line_of_sight_ecef, &result.range_rate_mps)) {
        set_error(error_message, "final satellite geometry evaluation failed");
        return false;
    }

    result.propagation_time_sec = result.geometric_range_m / kSpeedOfLightMps;
    result.healthy = result.satellite_state.health == 0;
    result.above_elevation_mask = elevation_passes_mask(result.elevation_rad, elevation_mask_deg);
    result.visible = result.healthy && result.above_elevation_mask;

    *geometry = result;
    return true;
}

bool compute_satellite_geometry(const RtklibNavStore* nav_store, const ReceiverTruth& receiver,
                                const SimTime& receive_time, int satellite_number, double elevation_mask_deg,
                                SatelliteGeometry* geometry, std::string* error_message) {
    if (nav_store == nullptr) {
        set_error(error_message, "satellite-geometry navigation store is null");
        return false;
    }
    const RtklibStateProviderContext provider_context{
        nav_store, receive_time.gps_week, sim_time_sow_sec(receive_time)};
    return compute_satellite_geometry_with_provider(rtklib_state_provider, &provider_context, receiver, receive_time,
                                                    satellite_number, elevation_mask_deg, geometry, error_message);
}

} // namespace gnss_sim
