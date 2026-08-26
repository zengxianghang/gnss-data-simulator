#include "gnss/rtklib_adapter.h"

extern "C" {
#include <rtklib.h>
#include <rtklib_pntvel_ext.h>
}

#include <cmath>
#include <cstdio>
#include <cstring>

namespace gnss_sim {

struct RtklibNavStore {
    nav_t nav;
};

namespace {

constexpr double kSpeedOfLightMps = 299792458.0;
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

void copy_diagnostic(char destination[128], const char* source) {
    if (source == nullptr) {
        destination[0] = '\0';
        return;
    }
    std::snprintf(destination, 128, "%s", source);
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

bool legacy_prange_adjustment_m(const nav_t& nav, int satellite_number, int observation_code,
                                double* adjustment_m) {
    if (adjustment_m == nullptr || satellite_number <= 0 || satellite_number > MAXSAT || observation_code <= 0 ||
        observation_code > 255) {
        return false;
    }

    const int system = satsys(satellite_number, nullptr);
    const int second_frequency_index = (system & (SYS_GAL | SYS_SBS)) ? 2 : 1;
    const double first_wavelength_m = nav.lam[satellite_number - 1][0];
    const double second_wavelength_m = nav.lam[satellite_number - 1][second_frequency_index];
    if (!(first_wavelength_m > 0.0) || !(second_wavelength_m > 0.0)) {
        return false;
    }

    const double ratio = second_wavelength_m / first_wavelength_m;
    const double gamma = ratio * ratio;
    if (!std::isfinite(gamma) || std::fabs(1.0 - gamma) < 1.0e-12) {
        return false;
    }

    double p1_p2_m = nav.cbias[satellite_number - 1][0];
    const double p1_c1_m = nav.cbias[satellite_number - 1][1];
    if (p1_p2_m == 0.0 && (system & (SYS_GPS | SYS_GAL | SYS_QZS))) {
        for (int index = 0; index < nav.n; ++index) {
            if (nav.eph[index].sat == satellite_number) {
                p1_p2_m = (1.0 - gamma) * kSpeedOfLightMps * nav.eph[index].tgd[0];
                break;
            }
        }
    }

    double adjustment = -p1_p2_m / (1.0 - gamma);
    if (observation_code == CODE_L1C) {
        adjustment += p1_c1_m;
    }
    *adjustment_m = adjustment;
    return std::isfinite(adjustment);
}

bool solver_pseudorange_m(const nav_t& nav, const RtklibSolutionObservation& observation,
                          double* pseudorange_m) {
    if (pseudorange_m == nullptr || !std::isfinite(observation.pseudorange_m) || observation.pseudorange_m <= 0.0 ||
        !std::isfinite(observation.code_bias_m)) {
        return false;
    }

    double legacy_adjustment_m = 0.0;
    if (!legacy_prange_adjustment_m(nav, observation.satellite_number, observation.observation_code,
                                    &legacy_adjustment_m)) {
        return false;
    }

    // #11 emits raw signal-specific pseudorange. Remove its explicit
    // TGD/BGD/ISC term, then pre-compensate the adjustment that the pinned
    // RTKLIB prange() applies. The PC seen by rescode() is therefore the
    // broadcast-clock-referenced pseudorange without modifying pntpos().
    *pseudorange_m = observation.pseudorange_m - observation.code_bias_m - legacy_adjustment_m;
    return std::isfinite(*pseudorange_m) && *pseudorange_m > 0.0;
}

void fill_common_observation(const RtklibSolutionObservation& source, gtime_t time, obsd_t* destination) {
    std::memset(destination, 0, sizeof(*destination));
    destination->time = time;
    destination->sat = static_cast<unsigned char>(source.satellite_number);
    destination->rcv = 1;
    destination->SNR[0] = snr_quarter_dbhz(source.cn0_dbhz);
    destination->code[0] = static_cast<unsigned char>(source.observation_code);
}

} // namespace

bool rtklib_solve_single_position(const RtklibNavStore* receiver_nav, int gps_week, double sow_sec,
                                  const RtklibSolutionObservation* observations, int observation_count,
                                  double elevation_mask_deg, bool broadcast_atmosphere,
                                  RtklibPositionSolution* solution, std::string* error_message) {
    if (receiver_nav == nullptr || observations == nullptr || observation_count < 0 || solution == nullptr ||
        !valid_time(gps_week, sow_sec) || !std::isfinite(elevation_mask_deg) || elevation_mask_deg < -90.0 ||
        elevation_mask_deg > 90.0) {
        set_error(error_message, "position-solution request has invalid arguments");
        return false;
    }

    RtklibPositionSolution result{};
    obsd_t rtklib_observations[MAXOBS]{};
    const gtime_t epoch_time = gpst2time(gps_week, sow_sec);
    bool used_satellite[MAXSAT]{};
    int usable_count = 0;

    for (int index = 0; index < observation_count && usable_count < MAXOBS; ++index) {
        const RtklibSolutionObservation& source = observations[index];
        if (!source.pseudorange_valid || source.satellite_number <= 0 || source.satellite_number > MAXSAT ||
            source.observation_code <= 0 || source.observation_code > 255 ||
            used_satellite[source.satellite_number - 1]) {
            continue;
        }
        double pseudorange_m = 0.0;
        if (!solver_pseudorange_m(receiver_nav->nav, source, &pseudorange_m)) {
            continue;
        }
        fill_common_observation(source, epoch_time, &rtklib_observations[usable_count]);
        rtklib_observations[usable_count].P[0] = pseudorange_m;
        used_satellite[source.satellite_number - 1] = true;
        ++usable_count;
    }

    if (usable_count < 4) {
        copy_diagnostic(result.diagnostic, "insufficient valid pseudorange observations");
        *solution = result;
        return true;
    }

    prcopt_t options = solution_options(elevation_mask_deg, broadcast_atmosphere);
    sol_t rtklib_solution{};
    char message[128]{};
    const int status = pntpos(rtklib_observations, usable_count, &receiver_nav->nav, &options, &rtklib_solution, nullptr,
                              nullptr, message);
    copy_diagnostic(result.diagnostic, message);
    if (status == 0) {
        *solution = result;
        return true;
    }

    result.valid = true;
    for (int index = 0; index < 3; ++index) {
        result.position_ecef_m[index] = rtklib_solution.rr[index];
    }
    double position_rad[3]{};
    ecef2pos(result.position_ecef_m, position_rad);
    result.latitude_deg = position_rad[0] / kRadiansPerDegree;
    result.longitude_deg = position_rad[1] / kRadiansPerDegree;
    result.height_m = position_rad[2];
    result.receiver_clock_bias_m = rtklib_solution.dtr[0] * kSpeedOfLightMps;
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
    if (receiver_nav == nullptr || observations == nullptr || observation_count < 0 || solution == nullptr ||
        !valid_time(gps_week, sow_sec) || !valid_position_hint(position_hint_ecef_m) ||
        !std::isfinite(elevation_mask_deg) || elevation_mask_deg < -90.0 || elevation_mask_deg > 90.0) {
        set_error(error_message, "velocity-solution request has invalid arguments");
        return false;
    }

    RtklibVelocitySolution result{};
    obsd_t rtklib_observations[MAXOBS]{};
    unsigned char doppler_valid[MAXOBS]{};
    double wavelength_m[MAXOBS]{};
    const gtime_t epoch_time = gpst2time(gps_week, sow_sec);
    bool used_satellite[MAXSAT]{};
    int usable_count = 0;

    for (int index = 0; index < observation_count && usable_count < MAXOBS; ++index) {
        const RtklibSolutionObservation& source = observations[index];
        if (!source.doppler_valid || source.satellite_number <= 0 || source.satellite_number > MAXSAT ||
            source.observation_code <= 0 || source.observation_code > 255 ||
            !std::isfinite(source.doppler_hz) || !std::isfinite(source.wavelength_m) || source.wavelength_m <= 0.0 ||
            !std::isfinite(source.pseudorange_m) || source.pseudorange_m <= 0.0 ||
            used_satellite[source.satellite_number - 1]) {
            continue;
        }
        fill_common_observation(source, epoch_time, &rtklib_observations[usable_count]);
        rtklib_observations[usable_count].P[0] = source.pseudorange_m;
        rtklib_observations[usable_count].D[0] = static_cast<float>(source.doppler_hz);
        doppler_valid[usable_count] = 1;
        wavelength_m[usable_count] = source.wavelength_m;
        used_satellite[source.satellite_number - 1] = true;
        ++usable_count;
    }

    if (usable_count < 4) {
        copy_diagnostic(result.diagnostic, "insufficient valid Doppler observations");
        *solution = result;
        return true;
    }

    prcopt_t options = solution_options(elevation_mask_deg, false);
    char message[128]{};
    int used_count = 0;
    const int status = rtklib_pntvel_ext(rtklib_observations, doppler_valid, wavelength_m, usable_count,
                                         &receiver_nav->nav, &options, position_hint_ecef_m,
                                         result.velocity_ecef_mps, &result.receiver_clock_drift_mps, &used_count,
                                         message);
    copy_diagnostic(result.diagnostic, message);
    if (status == 0) {
        *solution = result;
        return true;
    }

    result.valid = true;
    result.used_satellites = used_count;
    *solution = result;
    return true;
}

} // namespace gnss_sim
