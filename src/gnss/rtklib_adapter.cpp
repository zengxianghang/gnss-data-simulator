#include "gnss/rtklib_adapter.h"

extern "C" {
#include <rtklib.h>
}

#include <cmath>
#include <cstring>
#include <new>

namespace gnss_sim {

struct RtklibNavStore {
    nav_t nav;
};

namespace {

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

void reset_nav(nav_t* nav) {
    if (nav == nullptr) {
        return;
    }
    freenav(nav, 0xFF);
    std::memset(nav, 0, sizeof(*nav));
}

bool valid_gps_time(int gps_week, double sow_sec) {
    return gps_week >= 0 && std::isfinite(sow_sec) && sow_sec >= 0.0 && sow_sec < 604800.0;
}

void count_system(int system, RtklibNavCounts* counts) {
    switch (system) {
        case SYS_GPS:
            ++counts->gps_eph_count;
            break;
        case SYS_GLO:
            ++counts->glo_eph_count;
            break;
        case SYS_GAL:
            ++counts->gal_eph_count;
            break;
        case SYS_CMP:
            ++counts->bds_eph_count;
            break;
        case SYS_QZS:
            ++counts->qzss_eph_count;
            break;
        default:
            ++counts->other_eph_count;
            break;
    }
}

} // namespace

RtklibNavStore* create_rtklib_nav_store() {
    return new (std::nothrow) RtklibNavStore{};
}

void destroy_rtklib_nav_store(RtklibNavStore* store) {
    if (store == nullptr) {
        return;
    }
    reset_nav(&store->nav);
    delete store;
}

bool load_rinex_nav_file(RtklibNavStore* store, const char* file_path, std::string* error_message) {
    if (store == nullptr || file_path == nullptr || file_path[0] == '\0') {
        set_error(error_message, "NAV store and RINEX NAV path must be valid");
        return false;
    }

    reset_nav(&store->nav);

    obs_t obs{};
    sta_t station{};
    const int status = readrnx(file_path, 1, "", &obs, &store->nav, &station);
    freeobs(&obs);
    if (status == 0) {
        reset_nav(&store->nav);
        set_error(error_message, std::string("RTKLIB failed to read RINEX NAV: ") + file_path);
        return false;
    }

    uniqnav(&store->nav);
    if (store->nav.n <= 0 && store->nav.ng <= 0 && store->nav.ns <= 0) {
        reset_nav(&store->nav);
        set_error(error_message, std::string("RINEX file contains no usable broadcast ephemeris: ") + file_path);
        return false;
    }
    return true;
}

bool get_rtklib_nav_counts(const RtklibNavStore* store, RtklibNavCounts* counts) {
    if (store == nullptr || counts == nullptr) {
        return false;
    }

    *counts = RtklibNavCounts{};
    for (int index = 0; index < store->nav.n; ++index) {
        int prn = 0;
        const int system = satsys(store->nav.eph[index].sat, &prn);
        static_cast<void>(prn);
        count_system(system, counts);
    }
    for (int index = 0; index < store->nav.ng; ++index) {
        int prn = 0;
        const int system = satsys(store->nav.geph[index].sat, &prn);
        static_cast<void>(prn);
        count_system(system, counts);
    }
    return true;
}

bool rtklib_satellite_id_to_number(const char* satellite_id, int* satellite_number) {
    if (satellite_id == nullptr || satellite_number == nullptr) {
        return false;
    }
    const int satellite = satid2no(satellite_id);
    if (satellite <= 0) {
        return false;
    }
    *satellite_number = satellite;
    return true;
}

bool get_rtklib_satellite_state(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,
                                RtklibSatelliteState* state, std::string* error_message) {
    if (store == nullptr || state == nullptr || satellite_number <= 0 || !valid_gps_time(gps_week, sow_sec)) {
        set_error(error_message, "satellite-state request has invalid arguments");
        return false;
    }

    const gtime_t time = gpst2time(gps_week, sow_sec);
    double state_and_velocity[6]{};
    double clock[2]{};
    double variance_m2 = 0.0;
    int health = 0;
    if (satpos(time, time, satellite_number, EPHOPT_BRDC, &store->nav, state_and_velocity, clock, &variance_m2,
               &health) == 0) {
        set_error(error_message, "RTKLIB could not compute broadcast satellite state at requested GPST");
        return false;
    }

    for (int index = 0; index < 3; ++index) {
        state->position_ecef_m[index] = state_and_velocity[index];
        state->velocity_ecef_mps[index] = state_and_velocity[index + 3];
    }
    state->clock_bias_sec = clock[0];
    state->clock_drift_sec_per_sec = clock[1];
    state->variance_m2 = variance_m2;
    state->health = health;
    return true;
}

bool rtklib_llh_to_ecef(double latitude_deg, double longitude_deg, double height_m, double ecef_m[3]) {
    if (ecef_m == nullptr || !std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) ||
        !std::isfinite(height_m) || latitude_deg < -90.0 || latitude_deg > 90.0 || longitude_deg < -180.0 ||
        longitude_deg > 180.0) {
        return false;
    }

    const double position_rad_m[3] = {latitude_deg * D2R, longitude_deg * D2R, height_m};
    pos2ecef(position_rad_m, ecef_m);
    return true;
}

bool rtklib_ecef_to_llh(const double ecef_m[3], double* latitude_deg, double* longitude_deg, double* height_m) {
    if (ecef_m == nullptr || latitude_deg == nullptr || longitude_deg == nullptr || height_m == nullptr) {
        return false;
    }
    for (int index = 0; index < 3; ++index) {
        if (!std::isfinite(ecef_m[index])) {
            return false;
        }
    }

    double position_rad_m[3]{};
    ecef2pos(ecef_m, position_rad_m);
    *latitude_deg = position_rad_m[0] * R2D;
    *longitude_deg = position_rad_m[1] * R2D;
    *height_m = position_rad_m[2];
    return true;
}

} // namespace gnss_sim
