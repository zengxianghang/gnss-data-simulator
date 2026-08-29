#include "gnss/nav_output_record.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

extern "C" {
#include <rtklib.h>
}
namespace gnss_sim {

struct RtklibNavStore {
    nav_t nav;
};

namespace {

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool reserve_ephemeris(nav_t* nav, int required) {
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

bool reserve_glonass_ephemeris(nav_t* nav, int required) {
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

int rtklib_system(NavOutputSystem system) {
    switch (system) {
        case NavOutputSystem::kGps:
            return SYS_GPS;
        case NavOutputSystem::kGlonass:
            return SYS_GLO;
        case NavOutputSystem::kGalileo:
            return SYS_GAL;
        case NavOutputSystem::kBeidou:
            return SYS_CMP;
        case NavOutputSystem::kQzss:
            return SYS_QZS;
        case NavOutputSystem::kNavic:
            return SYS_IRN;
        case NavOutputSystem::kUnknown:
            break;
    }
    return SYS_NONE;
}

int rtklib_message_type(NavOutputSystem system, RtklibBroadcastMessageFamily family) {
    switch (family) {
        case RtklibBroadcastMessageFamily::kLegacy:
            if (system == NavOutputSystem::kGps || system == NavOutputSystem::kQzss) {
                return NAV_LNAV;
            }
            if (system == NavOutputSystem::kBeidou) {
                return NAV_D1D2;
            }
            if (system == NavOutputSystem::kNavic) {
                return NAV_LNAV;
            }
            break;
        case RtklibBroadcastMessageFamily::kGalileoInav:
            return NAV_INAV;
        case RtklibBroadcastMessageFamily::kGalileoFnav:
            return NAV_FNAV;
        case RtklibBroadcastMessageFamily::kGlonassFdma:
            return NAV_FDMA;
        case RtklibBroadcastMessageFamily::kCnav:
            return NAV_CNAV;
        case RtklibBroadcastMessageFamily::kCnav2:
            return NAV_CNV2;
        case RtklibBroadcastMessageFamily::kBeidouBcnav1:
            return NAV_CNV1;
        case RtklibBroadcastMessageFamily::kBeidouBcnav2:
            return NAV_CNV2;
        case RtklibBroadcastMessageFamily::kBeidouBcnav3:
            return NAV_CNV3;
        case RtklibBroadcastMessageFamily::kGlonassL3Oc:
            return NAV_L3OC;
        case RtklibBroadcastMessageFamily::kUnknown:
            break;
    }
    return 0;
}

bool valid_week_sow(int week, double sow_sec) {
    return week >= 0 && std::isfinite(sow_sec) && sow_sec >= 0.0 && sow_sec < 604800.0;
}

void refresh_wavelengths(nav_t* nav) {
    for (int satellite = 1; satellite <= MAXSAT; ++satellite) {
        for (int frequency = 0; frequency < NFREQ; ++frequency) {
            nav->lam[satellite - 1][frequency] = satwavelen(satellite, frequency, nav);
        }
    }
}

bool append_keplerian(RtklibNavStore* store, const KeplerianNavOutputData& source, std::string* error_message) {
    const int system = rtklib_system(source.system);
    const int message_type = rtklib_message_type(source.system, source.message_family);
    if (system == SYS_NONE || message_type == 0 || source.satellite_number <= 0 || source.satellite_number > MAXSAT ||
        source.prn <= 0 || !valid_week_sow(source.toe_week, source.toe_sow_sec) ||
        !valid_week_sow(source.toc_week, source.toc_sow_sec) ||
        !valid_week_sow(source.transmit_week, source.transmit_sow_sec) || !std::isfinite(source.semi_major_axis_m) ||
        source.semi_major_axis_m <= 0.0) {
        set_error(error_message, "serialized Keplerian navigation record is invalid");
        return false;
    }
    if (!reserve_ephemeris(&store->nav, store->nav.n + 1)) {
        set_error(error_message, "cannot allocate RTKLIB serialized ephemeris store");
        return false;
    }

    eph_t eph{};
    eph.sat = source.satellite_number;
    eph.iode = source.iode;
    eph.iodc = source.iodc;
    eph.sva = source.sva;
    eph.svh = source.svh;
    eph.week = source.toe_week;
    eph.code = source.code;
    eph.flag = source.flag;
    eph.toe = gpst2time(source.toe_week, source.toe_sow_sec);
    eph.toc = gpst2time(source.toc_week, source.toc_sow_sec);
    eph.ttr = gpst2time(source.transmit_week, source.transmit_sow_sec);
    eph.A = source.semi_major_axis_m;
    eph.e = source.eccentricity;
    eph.i0 = source.inclination_rad;
    eph.OMG0 = source.omega0_rad;
    eph.omg = source.argument_of_perigee_rad;
    eph.M0 = source.mean_anomaly_rad;
    eph.deln = source.delta_mean_motion_radps;
    eph.OMGd = source.omega_dot_radps;
    eph.idot = source.inclination_dot_radps;
    eph.crc = source.crc_m;
    eph.crs = source.crs_m;
    eph.cuc = source.cuc_rad;
    eph.cus = source.cus_rad;
    eph.cic = source.cic_rad;
    eph.cis = source.cis_rad;
    // Serialized BD2EPHEMA Toe is GPST by the receiver-log contract, while
    // RTKLIB keeps eph.toes in native BDT seconds-of-week for BeiDou's
    // Earth-rotation term. Keep eph.toe as the absolute GPST epoch and only
    // restore the raw Toe SOW field to BDT here.
    eph.toes = source.toe_sow_sec;
    if (system == SYS_CMP) {
        eph.toes -= 14.0;
        if (eph.toes < 0.0) {
            eph.toes += 604800.0;
        }
    }
    eph.fit = source.fit_hours;
    eph.f0 = source.clock_bias_sec;
    eph.f1 = source.clock_drift_sec_per_sec;
    eph.f2 = source.clock_drift_rate_sec_per_sec2;
    std::memcpy(eph.tgd, source.tgd_sec, sizeof(eph.tgd));
    std::memcpy(eph.isc, source.isc_sec, sizeof(eph.isc));
    eph.hdr.data_type = NAV_EPH;
    eph.hdr.sys = system;
    eph.hdr.prn = source.prn;
    eph.hdr.msg_type = message_type;

    store->nav.eph[store->nav.n++] = eph;
    refresh_wavelengths(&store->nav);
    return true;
}

bool append_glonass(RtklibNavStore* store, const GlonassNavOutputData& source, std::string* error_message) {
    if (source.message_family != RtklibBroadcastMessageFamily::kGlonassFdma || source.satellite_number <= 0 ||
        source.satellite_number > MAXSAT || source.prn <= 0 || source.prn > MAXPRNGLO ||
        !valid_week_sow(source.toe_week, source.toe_sow_sec) ||
        !valid_week_sow(source.frame_week, source.frame_sow_sec)) {
        set_error(error_message, "serialized GLONASS navigation record is invalid");
        return false;
    }
    if (!reserve_glonass_ephemeris(&store->nav, store->nav.ng + 1)) {
        set_error(error_message, "cannot allocate RTKLIB serialized GLONASS ephemeris store");
        return false;
    }

    geph_t geph{};
    geph.sat = source.satellite_number;
    geph.iode = source.iode;
    geph.frq = source.frequency_channel;
    geph.svh = source.svh;
    geph.sva = source.sva;
    geph.age = source.age_days;
    geph.flag = source.flags;
    geph.toe = gpst2time(source.toe_week, source.toe_sow_sec);
    geph.tof = gpst2time(source.frame_week, source.frame_sow_sec);
    std::memcpy(geph.pos, source.position_ecef_m, sizeof(geph.pos));
    std::memcpy(geph.vel, source.velocity_ecef_mps, sizeof(geph.vel));
    std::memcpy(geph.acc, source.acceleration_ecef_mps2, sizeof(geph.acc));
    geph.taun = source.clock_bias_sec;
    geph.gamn = source.relative_frequency_bias;
    geph.dtaun = source.differential_delay_sec;
    geph.hdr.data_type = NAV_EPH;
    geph.hdr.sys = SYS_GLO;
    geph.hdr.prn = source.prn;
    geph.hdr.msg_type = NAV_FDMA;

    store->nav.geph[store->nav.ng++] = geph;
    store->nav.glo_fcn[source.prn - 1] = static_cast<char>(source.frequency_channel + 8);
    refresh_wavelengths(&store->nav);
    return true;
}

bool apply_ionosphere(RtklibNavStore* store, const IonosphereNavOutputData& source, std::string* error_message) {
    if (source.coefficient_count < 0 || source.coefficient_count > 9 || source.transmit_week < 0 ||
        !std::isfinite(source.transmit_sow_sec)) {
        set_error(error_message, "serialized ionosphere navigation record is invalid");
        return false;
    }
    if (source.system == NavOutputSystem::kGps && source.coefficient_count >= 8) {
        std::memcpy(store->nav.ion_gps, source.coefficients, sizeof(store->nav.ion_gps));
        std::memcpy(store->nav.utc_gps, source.utc, sizeof(store->nav.utc_gps));
        store->nav.leaps = source.leap_seconds;
        return true;
    }
    if (source.system == NavOutputSystem::kQzss && source.coefficient_count >= 8) {
        std::memcpy(store->nav.ion_qzs, source.coefficients, sizeof(store->nav.ion_qzs));
        std::memcpy(store->nav.utc_qzs, source.utc, sizeof(store->nav.utc_qzs));
        store->nav.leaps = source.leap_seconds;
        return true;
    }
    if (source.system == NavOutputSystem::kBeidou && source.coefficient_count >= 8) {
        std::memcpy(store->nav.ion_cmp, source.coefficients, sizeof(store->nav.ion_cmp));
        std::memcpy(store->nav.utc_cmp, source.utc, sizeof(store->nav.utc_cmp));
        store->nav.leaps = source.leap_seconds;
        return true;
    }
    set_error(error_message, "serialized ionosphere model is unsupported by RTKLIB NAV import");
    return false;
}

} // namespace

bool rtklib_append_nav_output_record(RtklibNavStore* store, const NavOutputRecord& record, std::string* error_message) {
    if (store == nullptr) {
        set_error(error_message, "RTKLIB serialized navigation store is null");
        return false;
    }
    switch (record.kind) {
        case RtklibNavRecordKind::kEphemeris:
            return append_keplerian(store, record.ephemeris, error_message);
        case RtklibNavRecordKind::kGlonassEphemeris:
            return append_glonass(store, record.glonass, error_message);
        case RtklibNavRecordKind::kIonosphere:
            return apply_ionosphere(store, record.ionosphere, error_message);
    }
    set_error(error_message, "serialized navigation record kind is unsupported");
    return false;
}

} // namespace gnss_sim
