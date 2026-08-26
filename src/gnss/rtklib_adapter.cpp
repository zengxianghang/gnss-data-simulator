#include "gnss/rtklib_adapter.h"

extern "C" {
#include <rtklib.h>
#include <rtklib_obs_ext.h>
}

#include <cmath>
#include <cstdlib>
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

bool zero_time(gtime_t time) {
    return time.time == 0 && time.sec == 0.0;
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

void copy_nav_metadata(const nav_t& source, nav_t* destination, bool include_ion) {
    destination->leaps = source.leaps;
    std::memcpy(destination->lam, source.lam, sizeof(destination->lam));
    std::memcpy(destination->glo_fcn, source.glo_fcn, sizeof(destination->glo_fcn));
    std::memcpy(destination->glo_cpbias, source.glo_cpbias, sizeof(destination->glo_cpbias));
    if (!include_ion) {
        return;
    }
    std::memcpy(destination->utc_gps, source.utc_gps, sizeof(destination->utc_gps));
    std::memcpy(destination->utc_glo, source.utc_glo, sizeof(destination->utc_glo));
    std::memcpy(destination->utc_gal, source.utc_gal, sizeof(destination->utc_gal));
    std::memcpy(destination->utc_qzs, source.utc_qzs, sizeof(destination->utc_qzs));
    std::memcpy(destination->utc_cmp, source.utc_cmp, sizeof(destination->utc_cmp));
    std::memcpy(destination->utc_sbs, source.utc_sbs, sizeof(destination->utc_sbs));
    std::memcpy(destination->ion_gps, source.ion_gps, sizeof(destination->ion_gps));
    std::memcpy(destination->ion_gal, source.ion_gal, sizeof(destination->ion_gal));
    std::memcpy(destination->ion_qzs, source.ion_qzs, sizeof(destination->ion_qzs));
    std::memcpy(destination->ion_cmp, source.ion_cmp, sizeof(destination->ion_cmp));
}

bool reserve_eph(nav_t* nav, int required) {
    if (required <= nav->nmax) {
        return true;
    }
    int capacity = nav->nmax > 0 ? nav->nmax : 32;
    while (capacity < required) {
        capacity *= 2;
    }
    void* memory = std::realloc(nav->eph, sizeof(eph_t) * static_cast<std::size_t>(capacity));
    if (memory == nullptr) {
        return false;
    }
    nav->eph = static_cast<eph_t*>(memory);
    nav->nmax = capacity;
    return true;
}

bool reserve_geph(nav_t* nav, int required) {
    if (required <= nav->ngmax) {
        return true;
    }
    int capacity = nav->ngmax > 0 ? nav->ngmax : 16;
    while (capacity < required) {
        capacity *= 2;
    }
    void* memory = std::realloc(nav->geph, sizeof(geph_t) * static_cast<std::size_t>(capacity));
    if (memory == nullptr) {
        return false;
    }
    nav->geph = static_cast<geph_t*>(memory);
    nav->ngmax = capacity;
    return true;
}

bool reserve_ion(nav_t* nav, int required) {
    if (required <= nav->nionmax) {
        return true;
    }
    int capacity = nav->nionmax > 0 ? nav->nionmax : 8;
    while (capacity < required) {
        capacity *= 2;
    }
    void* memory = std::realloc(nav->ion, sizeof(ion_t) * static_cast<std::size_t>(capacity));
    if (memory == nullptr) {
        return false;
    }
    nav->ion = static_cast<ion_t*>(memory);
    nav->nionmax = capacity;
    return true;
}

bool append_eph(const eph_t& eph, nav_t* destination) {
    if (!reserve_eph(destination, destination->n + 1)) {
        return false;
    }
    destination->eph[destination->n++] = eph;
    return true;
}

bool append_geph(const geph_t& geph, nav_t* destination) {
    if (!reserve_geph(destination, destination->ng + 1)) {
        return false;
    }
    destination->geph[destination->ng++] = geph;
    return true;
}

bool append_ion(const ion_t& ion, nav_t* destination) {
    if (!reserve_ion(destination, destination->nion + 1)) {
        return false;
    }
    destination->ion[destination->nion++] = ion;
    return true;
}

gtime_t eph_transmission_time(const eph_t& eph) {
    return zero_time(eph.ttr) ? eph.toe : eph.ttr;
}

gtime_t geph_transmission_time(const geph_t& geph) {
    return zero_time(geph.tof) ? geph.toe : geph.tof;
}

bool time_to_week_sow(gtime_t time, int* week, double* sow_sec) {
    if (week == nullptr || sow_sec == nullptr) {
        return false;
    }
    *sow_sec = time2gpst(time, week);
    return *week >= 0 && std::isfinite(*sow_sec);
}

bool record_is_available(gtime_t record_time, gtime_t snapshot_time) {
    return timediff(record_time, snapshot_time) <= 0.0;
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

int rtklib_nav_record_count(const RtklibNavStore* store) {
    if (store == nullptr) {
        return 0;
    }
    return store->nav.n + store->nav.ng + store->nav.nion;
}

bool rtklib_nav_record_info(const RtklibNavStore* store, int record_index, RtklibNavRecordInfo* info) {
    if (store == nullptr || info == nullptr || record_index < 0 || record_index >= rtklib_nav_record_count(store)) {
        return false;
    }

    RtklibNavRecordInfo result{};
    if (record_index < store->nav.n) {
        const eph_t& eph = store->nav.eph[record_index];
        result.kind = RtklibNavRecordKind::kEphemeris;
        result.satellite_number = eph.sat;
        result.system = satsys(eph.sat, &result.prn);
        result.message_type = eph.hdr.msg_type;
        result.iode = eph.iode;
        result.iodc = eph.iodc;
        if (!time_to_week_sow(eph_transmission_time(eph), &result.gps_week, &result.transmit_sow_sec)) {
            return false;
        }
        int toe_week = 0;
        result.toe_sow_sec = time2gpst(eph.toe, &toe_week);
    }
    else if (record_index < store->nav.n + store->nav.ng) {
        const geph_t& geph = store->nav.geph[record_index - store->nav.n];
        result.kind = RtklibNavRecordKind::kGlonassEphemeris;
        result.satellite_number = geph.sat;
        result.system = satsys(geph.sat, &result.prn);
        result.message_type = geph.hdr.msg_type;
        result.iode = geph.iode;
        result.iodc = -1;
        if (!time_to_week_sow(geph_transmission_time(geph), &result.gps_week, &result.transmit_sow_sec)) {
            return false;
        }
        int toe_week = 0;
        result.toe_sow_sec = time2gpst(geph.toe, &toe_week);
    }
    else {
        const ion_t& ion = store->nav.ion[record_index - store->nav.n - store->nav.ng];
        result.kind = RtklibNavRecordKind::kIonosphere;
        result.satellite_number = 0;
        result.system = ion.hdr.sys;
        result.prn = ion.hdr.prn;
        result.message_type = ion.hdr.msg_type;
        result.iode = -1;
        result.iodc = -1;
        if (!time_to_week_sow(ion.trans_time, &result.gps_week, &result.transmit_sow_sec)) {
            return false;
        }
        result.toe_sow_sec = result.transmit_sow_sec;
    }

    *info = result;
    return true;
}

bool rtklib_clear_nav_store(RtklibNavStore* store) {
    if (store == nullptr) {
        return false;
    }
    reset_nav(&store->nav);
    return true;
}

bool rtklib_copy_nav_snapshot(const RtklibNavStore* source, int gps_week, double sow_sec, RtklibNavStore* destination,
                              std::string* error_message) {
    if (source == nullptr || destination == nullptr || source == destination || !valid_gps_time(gps_week, sow_sec)) {
        set_error(error_message, "navigation snapshot request has invalid arguments");
        return false;
    }

    reset_nav(&destination->nav);
    copy_nav_metadata(source->nav, &destination->nav, true);
    const gtime_t snapshot_time = gpst2time(gps_week, sow_sec);

    int selected_eph[MAXSAT];
    int selected_geph[MAXSAT];
    for (int sat_index = 0; sat_index < MAXSAT; ++sat_index) {
        selected_eph[sat_index] = -1;
        selected_geph[sat_index] = -1;
    }

    for (int index = 0; index < source->nav.n; ++index) {
        const eph_t& eph = source->nav.eph[index];
        if (eph.sat <= 0 || eph.sat > MAXSAT || !record_is_available(eph_transmission_time(eph), snapshot_time)) {
            continue;
        }
        const int sat_index = eph.sat - 1;
        const int selected = selected_eph[sat_index];
        if (selected < 0 || timediff(eph_transmission_time(eph), eph_transmission_time(source->nav.eph[selected])) > 0.0) {
            selected_eph[sat_index] = index;
        }
    }
    for (int index = 0; index < source->nav.ng; ++index) {
        const geph_t& geph = source->nav.geph[index];
        if (geph.sat <= 0 || geph.sat > MAXSAT || !record_is_available(geph_transmission_time(geph), snapshot_time)) {
            continue;
        }
        const int sat_index = geph.sat - 1;
        const int selected = selected_geph[sat_index];
        if (selected < 0 ||
            timediff(geph_transmission_time(geph), geph_transmission_time(source->nav.geph[selected])) > 0.0) {
            selected_geph[sat_index] = index;
        }
    }

    for (int index = 0; index < source->nav.n; ++index) {
        const int sat = source->nav.eph[index].sat;
        if (sat > 0 && sat <= MAXSAT && selected_eph[sat - 1] == index &&
            !append_eph(source->nav.eph[index], &destination->nav)) {
            reset_nav(&destination->nav);
            set_error(error_message, "cannot allocate receiver ephemeris snapshot");
            return false;
        }
    }
    for (int index = 0; index < source->nav.ng; ++index) {
        const int sat = source->nav.geph[index].sat;
        if (sat > 0 && sat <= MAXSAT && selected_geph[sat - 1] == index &&
            !append_geph(source->nav.geph[index], &destination->nav)) {
            reset_nav(&destination->nav);
            set_error(error_message, "cannot allocate receiver GLONASS ephemeris snapshot");
            return false;
        }
    }
    for (int index = 0; index < source->nav.nion; ++index) {
        if (record_is_available(source->nav.ion[index].trans_time, snapshot_time) &&
            !append_ion(source->nav.ion[index], &destination->nav)) {
            reset_nav(&destination->nav);
            set_error(error_message, "cannot allocate receiver ionosphere snapshot");
            return false;
        }
    }
    return true;
}

bool rtklib_copy_nav_record(const RtklibNavStore* source, int record_index, RtklibNavStore* destination,
                            std::string* error_message) {
    if (source == nullptr || destination == nullptr || source == destination || record_index < 0 ||
        record_index >= rtklib_nav_record_count(source)) {
        set_error(error_message, "navigation record copy request has invalid arguments");
        return false;
    }

    if (record_index < source->nav.n) {
        copy_nav_metadata(source->nav, &destination->nav, false);
        if (!append_eph(source->nav.eph[record_index], &destination->nav)) {
            set_error(error_message, "cannot allocate receiver ephemeris record");
            return false;
        }
        return true;
    }
    if (record_index < source->nav.n + source->nav.ng) {
        copy_nav_metadata(source->nav, &destination->nav, false);
        if (!append_geph(source->nav.geph[record_index - source->nav.n], &destination->nav)) {
            set_error(error_message, "cannot allocate receiver GLONASS ephemeris record");
            return false;
        }
        return true;
    }

    copy_nav_metadata(source->nav, &destination->nav, true);
    if (!append_ion(source->nav.ion[record_index - source->nav.n - source->nav.ng], &destination->nav)) {
        set_error(error_message, "cannot allocate receiver ionosphere record");
        return false;
    }
    return true;
}

bool rtklib_nav_store_has_satellite_ephemeris(const RtklibNavStore* store, int satellite_number) {
    if (store == nullptr || satellite_number <= 0) {
        return false;
    }
    for (int index = 0; index < store->nav.n; ++index) {
        if (store->nav.eph[index].sat == satellite_number) {
            return true;
        }
    }
    for (int index = 0; index < store->nav.ng; ++index) {
        if (store->nav.geph[index].sat == satellite_number) {
            return true;
        }
    }
    return false;
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
