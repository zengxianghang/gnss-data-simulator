#include "gnss/rtklib_adapter.h"

extern "C" {
#include <rtklib.h>
#include <rtklib_signal_bias_ext.h>
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

int required_message_mask(int system, RtklibBroadcastMessageFamily family) {
    switch (family) {
        case RtklibBroadcastMessageFamily::kLegacy:
            if (system == SYS_GPS || system == SYS_QZS) {
                return NAV_LNAV;
            }
            if (system == SYS_CMP) {
                return NAV_D1 | NAV_D2 | NAV_D1D2;
            }
            break;
        case RtklibBroadcastMessageFamily::kCnav:
            return NAV_CNAV;
        case RtklibBroadcastMessageFamily::kCnav2:
            return NAV_CNV2;
        case RtklibBroadcastMessageFamily::kGalileoInav:
            return NAV_INAV;
        case RtklibBroadcastMessageFamily::kGalileoFnav:
            return NAV_FNAV;
        case RtklibBroadcastMessageFamily::kBeidouBcnav1:
            return NAV_CNV1;
        case RtklibBroadcastMessageFamily::kBeidouBcnav2:
            return NAV_CNV2;
        case RtklibBroadcastMessageFamily::kBeidouBcnav3:
            return NAV_CNV3;
        case RtklibBroadcastMessageFamily::kGlonassFdma:
            return NAV_FDMA;
        case RtklibBroadcastMessageFamily::kGlonassL3Oc:
            return NAV_L3OC;
        case RtklibBroadcastMessageFamily::kUnknown:
            break;
    }
    return 0;
}

bool append_solution_ephemeris(const nav_t& source_nav, gtime_t time, const RtklibRawCodeObservation& observation,
                               nav_t* solver_nav, eph_t solver_eph[MAXOBS], geph_t solver_geph[MAXOBS]) {
    if (solver_nav == nullptr || observation.satellite_number <= 0 || observation.observation_code <= 0 ||
        observation.observation_code > 255) {
        return false;
    }
    const int system = satsys(observation.satellite_number, nullptr);
    const int mask = required_message_mask(system, observation.message_family);
    if (mask == 0) {
        return false;
    }

    eph_t eph{};
    geph_t geph{};
    if (rtklib_signal_ephemeris_ext(time, observation.satellite_number,
                                    static_cast<unsigned char>(observation.observation_code), mask, &source_nav, &eph,
                                    &geph, nullptr) != 1) {
        return false;
    }
    if (system == SYS_GLO) {
        if (solver_nav->ng >= MAXOBS) {
            return false;
        }
        solver_geph[solver_nav->ng++] = geph;
    } else {
        if (solver_nav->n >= MAXOBS) {
            return false;
        }
        solver_eph[solver_nav->n++] = eph;
    }
    return true;
}

void fill_observation(const RtklibRawCodeObservation& source, gtime_t time, obsd_t* destination) {
    std::memset(destination, 0, sizeof(*destination));
    destination->time = time;
    destination->sat = static_cast<unsigned char>(source.satellite_number);
    destination->rcv = 1;
    destination->P[0] = source.pseudorange_m;
    destination->SNR[0] = snr_quarter_dbhz(source.cn0_dbhz);
    destination->code[0] = static_cast<unsigned char>(source.observation_code);
}

} // namespace

bool rtklib_solve_raw_single_position(const RtklibNavStore* receiver_nav, int gps_week, double sow_sec,
                                      const RtklibRawCodeObservation* observations, int observation_count,
                                      double elevation_mask_deg, bool broadcast_atmosphere,
                                      RtklibPositionSolution* solution, std::string* error_message) {
    if (receiver_nav == nullptr || observations == nullptr || observation_count < 0 || solution == nullptr ||
        !valid_time(gps_week, sow_sec) || !std::isfinite(elevation_mask_deg) || elevation_mask_deg < -90.0 ||
        elevation_mask_deg > 90.0) {
        set_error(error_message, "raw position-solution request has invalid arguments");
        return false;
    }

    RtklibPositionSolution result{};
    obsd_t rtklib_observations[MAXOBS]{};
    eph_t solver_eph[MAXOBS]{};
    geph_t solver_geph[MAXOBS]{};
    nav_t solver_nav = receiver_nav->nav;
    solver_nav.eph = solver_eph;
    solver_nav.n = 0;
    solver_nav.nmax = MAXOBS;
    solver_nav.geph = solver_geph;
    solver_nav.ng = 0;
    solver_nav.ngmax = MAXOBS;
    const gtime_t epoch_time = gpst2time(gps_week, sow_sec);
    bool used_satellite[MAXSAT]{};
    int usable_count = 0;

    for (int index = 0; index < observation_count && usable_count < MAXOBS; ++index) {
        const RtklibRawCodeObservation& source = observations[index];
        if (!source.pseudorange_valid || source.satellite_number <= 0 || source.satellite_number > MAXSAT ||
            source.observation_code <= 0 || source.observation_code > 255 || !std::isfinite(source.pseudorange_m) ||
            source.pseudorange_m <= 0.0 || !std::isfinite(source.cn0_dbhz) ||
            used_satellite[source.satellite_number - 1]) {
            continue;
        }
        if (!append_solution_ephemeris(receiver_nav->nav, epoch_time, source, &solver_nav, solver_eph, solver_geph)) {
            continue;
        }
        fill_observation(source, epoch_time, &rtklib_observations[usable_count]);
        used_satellite[source.satellite_number - 1] = true;
        ++usable_count;
    }

    if (usable_count < 4) {
        copy_diagnostic(result.diagnostic, "insufficient valid raw pseudorange observations");
        *solution = result;
        return true;
    }

    prcopt_t options = solution_options(elevation_mask_deg, broadcast_atmosphere);
    sol_t rtklib_solution{};
    char message[128]{};
    const int status =
        pntpos(rtklib_observations, usable_count, &solver_nav, &options, &rtklib_solution, nullptr, nullptr, message);
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

} // namespace gnss_sim
