#include "gnss/rtklib_adapter.h"

extern "C" {
#include <rtklib.h>
#include <rtklib_pntvel_ext.h>
}

#include <cmath>
#include <cstring>

namespace gnss_sim {

struct RtklibNavStore {
    nav_t nav;
};

namespace {

constexpr double kRadiansPerDegree = 3.141592653589793238462643383279502884 / 180.0;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_time(int gps_week, double sow_sec) {
    return gps_week >= 0 && std::isfinite(sow_sec) && sow_sec >= 0.0 && sow_sec < 604800.0;
}

bool valid_position_hint(const double position_ecef_m[3]) {
    return position_ecef_m != nullptr && std::isfinite(position_ecef_m[0]) && std::isfinite(position_ecef_m[1]) &&
           std::isfinite(position_ecef_m[2]) &&
           (position_ecef_m[0] != 0.0 || position_ecef_m[1] != 0.0 || position_ecef_m[2] != 0.0);
}

prcopt_t solution_options(double elevation_mask_deg, bool broadcast_atmosphere) {
    prcopt_t options = prcopt_default;
    options.mode = PMODE_SINGLE;
    options.nf = 1;
    options.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_CMP;
    options.elmin = elevation_mask_deg * kRadiansPerDegree;
    options.sateph = EPHOPT_BRDC;
    options.ionoopt = broadcast_atmosphere ? IONOOPT_BRDC : IONOOPT_OFF;
    options.tropopt = broadcast_atmosphere ? TROPOPT_SAAS : TROPOPT_OFF;
    options.posopt[4] = 0;
    return options;
}

unsigned char snr_quarter_dbhz(double cn0_dbhz) {
    if (!std::isfinite(cn0_dbhz) || cn0_dbhz <= 0.0) {
        return 0;
    }
    double scaled = cn0_dbhz * 4.0;
    if (scaled > 255.0) {
        scaled = 255.0;
    }
    return static_cast<unsigned char>(scaled + 0.5);
}

bool fill_observation(const RtklibSolutionObservation& source, gtime_t time, obsd_t* destination) {
    if (destination == nullptr || source.satellite_number <= 0 || source.satellite_number > MAXSAT ||
        source.observation_code < 0 || source.observation_code > 255 || !std::isfinite(source.pseudorange_m) ||
        source.pseudorange_m <= 0.0 || !std::isfinite(source.doppler_hz)) {
        return false;
    }

    std::memset(destination, 0, sizeof(*destination));
    destination->time = time;
    destination->sat = static_cast<unsigned char>(source.satellite_number);
    destination->rcv = 1;
    destination->P[0] = source.pseudorange_m;
    destination->D[0] = static_cast<float>(source.doppler_hz);
    destination->SNR[0] = snr_quarter_dbhz(source.cn0_dbhz);
    destination->code[0] = static_cast<unsigned char>(source.observation_code);
    return true;
}

} // namespace

bool rtklib_solve_single_position(const RtklibNavStore* receiver_nav, int gps_week, double sow_sec,
                                  const RtklibSolutionObservation* observations, int observation_count,
                                  double elevation_mask_deg, bool broadcast_atmosphere,
                                  RtklibPositionSolution* solution, std::string* error_message) {
    if (receiver_nav == nullptr || observations == nullptr || observation_count <= 0 || solution == nullptr ||
        !valid_time(gps_week, sow_sec) || !std::isfinite(elevation_mask_deg) || elevation_mask_deg < -90.0 ||
        elevation_mask_deg > 90.0) {
        set_error(error_message, "position-solution request has invalid arguments");
        return false;
    }

    obsd_t rtklib_observations[MAXOBS]{};
    const gtime_t epoch_time = gpst2time(gps_week, sow_sec);
    int usable_count = 0;
    for (int index = 0; index < observation_count && usable_count < MAXOBS; ++index) {
        if (!observations[index].pseudorange_valid) {
            continue;
        }
        if (!fill_observation(observations[index], epoch_time, &rtklib_observations[usable_count])) {
            continue;
        }
        ++usable_count;
    }
    if (usable_count < 4) {
        set_error(error_message, "insufficient valid pseudorange observations");
        return false;
    }

    prcopt_t options = solution_options(elevation_mask_deg, broadcast_atmosphere);
    sol_t rtklib_solution{};
    char message[128]{};
    if (!pntpos(rtklib_observations, usable_count, &receiver_nav->nav, &options, &rtklib_solution, nullptr, nullptr,
                message)) {
        set_error(error_message, message[0] != '\0' ? message : "RTKLIB position solution failed");
        return false;
    }

    RtklibPositionSolution result{};
    for (int index = 0; index < 3; ++index) {
        result.position_ecef_m[index] = rtklib_solution.rr[index];
    }
    double position_rad[3]{};
    ecef2pos(result.position_ecef_m, position_rad);
    result.latitude_deg = position_rad[0] / kRadiansPerDegree;
    result.longitude_deg = position_rad[1] / kRadiansPerDegree;
    result.height_m = position_rad[2];
    result.receiver_clock_bias_m = rtklib_solution.dtr[0] * CLIGHT;
    for (int index = 0; index < 6; ++index) {
        result.covariance_ecef_m2[index] = static_cast<double>(rtklib_solution.qr[index]);
    }
    result.used_satellites = static_cast<int>(rtklib_solution.ns);
    *solution = result;
    return true;
}

bool rtklib_solve_single_velocity(const RtklibNavStore* receiver_nav, int gps_week, double sow_sec,
                                  const RtklibSolutionObservation* observations, int observation_count,
                                  const double position_hint_ecef_m[3], double elevation_mask_deg,
                                  RtklibVelocitySolution* solution, std::string* error_message) {
    if (receiver_nav == nullptr || observations == nullptr || observation_count <= 0 || solution == nullptr ||
        !valid_time(gps_week, sow_sec) || !valid_position_hint(position_hint_ecef_m) ||
        !std::isfinite(elevation_mask_deg) || elevation_mask_deg < -90.0 || elevation_mask_deg > 90.0) {
        set_error(error_message, "velocity-solution request has invalid arguments");
        return false;
    }

    obsd_t rtklib_observations[MAXOBS]{};
    unsigned char doppler_valid[MAXOBS]{};
    double wavelength_m[MAXOBS]{};
    const gtime_t epoch_time = gpst2time(gps_week, sow_sec);
    int usable_count = 0;
    for (int index = 0; index < observation_count && usable_count < MAXOBS; ++index) {
        const RtklibSolutionObservation& source = observations[index];
        if (!source.doppler_valid || !std::isfinite(source.wavelength_m) || source.wavelength_m <= 0.0) {
            continue;
        }
        if (!fill_observation(source, epoch_time, &rtklib_observations[usable_count])) {
            continue;
        }
        doppler_valid[usable_count] = 1;
        wavelength_m[usable_count] = source.wavelength_m;
        ++usable_count;
    }
    if (usable_count < 4) {
        set_error(error_message, "insufficient valid Doppler observations");
        return false;
    }

    prcopt_t options = solution_options(elevation_mask_deg, false);
    double velocity_ecef_mps[3]{};
    double receiver_clock_drift_mps = 0.0;
    int used_satellites = 0;
    char message[128]{};
    if (!rtklib_pntvel_ext(rtklib_observations, doppler_valid, wavelength_m, usable_count, &receiver_nav->nav, &options,
                           position_hint_ecef_m, velocity_ecef_mps, &receiver_clock_drift_mps, &used_satellites,
                           message)) {
        set_error(error_message, message[0] != '\0' ? message : "RTKLIB velocity solution failed");
        return false;
    }

    RtklibVelocitySolution result{};
    for (int index = 0; index < 3; ++index) {
        result.velocity_ecef_mps[index] = velocity_ecef_mps[index];
    }
    result.receiver_clock_drift_mps = receiver_clock_drift_mps;
    result.used_satellites = used_satellites;
    *solution = result;
    return true;
}

} // namespace gnss_sim
