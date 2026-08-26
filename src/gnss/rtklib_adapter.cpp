#include "gnss/rtklib_adapter.h"

extern "C" {
#include <rtklib.h>
#include <rtklib_obs_ext.h>
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

bool finite_vector3(const double vector[3]) {
    return vector != nullptr && std::isfinite(vector[0]) && std::isfinite(vector[1]) && std::isfinite(vector[2]);
}

std::string rtklib_file_path(const char* file_path) {
    std::string path(file_path);
#ifdef _WIN32
    // RTKLIB expath() only recognizes '\\' as the Windows directory separator.
    // CMake and callers commonly provide absolute paths with '/'. Normalize at
    // the adapter boundary so the same simulator path works on MSVC and GCC.
    for (char& character : path) {
        if (character == '/') {
            character = '\\';
        }
    }
#endif
    return path;
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

    const std::string normalized_path = rtklib_file_path(file_path);
    obs_t obs{};
    sta_t station{};
    const int status = readrnx(normalized_path.c_str(), 1, "", &obs, &store->nav, &station);
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

bool rtklib_observation_code(const char* rinex_signal_code, int* observation_code, int* frequency_index) {
    if (rinex_signal_code == nullptr || observation_code == nullptr || frequency_index == nullptr) {
        return false;
    }
    int frequency = 0;
    const unsigned char code = obs2code_ext(rinex_signal_code, &frequency);
    if (code == CODE_NONE || frequency <= 0) {
        return false;
    }
    *observation_code = static_cast<int>(code);
    *frequency_index = frequency;
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
    if (!finite_vector3(ecef_m) || latitude_deg == nullptr || longitude_deg == nullptr || height_m == nullptr) {
        return false;
    }

    double position_rad_m[3]{};
    ecef2pos(ecef_m, position_rad_m);
    *latitude_deg = position_rad_m[0] * R2D;
    *longitude_deg = position_rad_m[1] * R2D;
    *height_m = position_rad_m[2];
    return true;
}

bool rtklib_geometric_distance(const double satellite_ecef_m[3], const double receiver_ecef_m[3], double* range_m,
                               double line_of_sight_ecef[3]) {
    if (!finite_vector3(satellite_ecef_m) || !finite_vector3(receiver_ecef_m) || range_m == nullptr ||
        line_of_sight_ecef == nullptr) {
        return false;
    }
    const double range = geodist(satellite_ecef_m, receiver_ecef_m, line_of_sight_ecef);
    if (!std::isfinite(range) || range <= 0.0) {
        return false;
    }
    *range_m = range;
    return true;
}

bool rtklib_azimuth_elevation(const double receiver_ecef_m[3], const double line_of_sight_ecef[3], double* azimuth_rad,
                              double* elevation_rad) {
    if (!finite_vector3(receiver_ecef_m) || !finite_vector3(line_of_sight_ecef) || azimuth_rad == nullptr ||
        elevation_rad == nullptr) {
        return false;
    }
    double position_rad_m[3]{};
    double azel[2]{};
    ecef2pos(receiver_ecef_m, position_rad_m);
    satazel(position_rad_m, line_of_sight_ecef, azel);
    *azimuth_rad = azel[0];
    *elevation_rad = azel[1];
    return std::isfinite(*azimuth_rad) && std::isfinite(*elevation_rad);
}

} // namespace gnss_sim
