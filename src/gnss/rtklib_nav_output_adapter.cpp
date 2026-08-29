#include "gnss/nav_output_record.h"

#include <cmath>
#include <cstring>

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

NavOutputSystem output_system(int system) {
    switch (system) {
        case SYS_GPS:
            return NavOutputSystem::kGps;
        case SYS_GLO:
            return NavOutputSystem::kGlonass;
        case SYS_GAL:
            return NavOutputSystem::kGalileo;
        case SYS_CMP:
            return NavOutputSystem::kBeidou;
        case SYS_QZS:
            return NavOutputSystem::kQzss;
        case SYS_IRN:
            return NavOutputSystem::kNavic;
        default:
            return NavOutputSystem::kUnknown;
    }
}

RtklibBroadcastMessageFamily message_family(int system, int message_type) {
    if (system == SYS_GLO) {
        return message_type == NAV_L3OC ? RtklibBroadcastMessageFamily::kGlonassL3Oc
                                        : RtklibBroadcastMessageFamily::kGlonassFdma;
    }
    switch (message_type) {
        case NAV_CNAV:
            return RtklibBroadcastMessageFamily::kCnav;
        case NAV_CNV1:
            return RtklibBroadcastMessageFamily::kBeidouBcnav1;
        case NAV_CNV2:
            if (system == SYS_CMP) {
                return RtklibBroadcastMessageFamily::kBeidouBcnav2;
            }
            return RtklibBroadcastMessageFamily::kCnav2;
        case NAV_CNV3:
            return RtklibBroadcastMessageFamily::kBeidouBcnav3;
        case NAV_INAV:
            return RtklibBroadcastMessageFamily::kGalileoInav;
        case NAV_FNAV:
            return RtklibBroadcastMessageFamily::kGalileoFnav;
        default:
            return RtklibBroadcastMessageFamily::kLegacy;
    }
}

bool time_week_sow(gtime_t time, int* week, double* sow_sec) {
    if (week == nullptr || sow_sec == nullptr || (time.time == 0 && time.sec == 0.0)) {
        return false;
    }
    *sow_sec = time2gpst(time, week);
    return *week >= 0 && std::isfinite(*sow_sec);
}

bool array_has_nonzero(const double* values, int count) {
    for (int index = 0; index < count; ++index) {
        if (std::isfinite(values[index]) && values[index] != 0.0) {
            return true;
        }
    }
    return false;
}

bool has_explicit_ion(const nav_t& nav, int system) {
    for (int index = 0; index < nav.nion; ++index) {
        if (nav.ion[index].hdr.sys == system) {
            return true;
        }
    }
    return false;
}

int legacy_ion_systems(const nav_t& nav, int systems[4]) {
    int count = 0;
    if (!has_explicit_ion(nav, SYS_GPS) && array_has_nonzero(nav.ion_gps, 8)) {
        systems[count++] = SYS_GPS;
    }
    if (!has_explicit_ion(nav, SYS_QZS) && array_has_nonzero(nav.ion_qzs, 8)) {
        systems[count++] = SYS_QZS;
    }
    if (!has_explicit_ion(nav, SYS_GAL) && array_has_nonzero(nav.ion_gal, 4)) {
        systems[count++] = SYS_GAL;
    }
    if (!has_explicit_ion(nav, SYS_CMP) && array_has_nonzero(nav.ion_cmp, 8)) {
        systems[count++] = SYS_CMP;
    }
    return count;
}

bool same_galileo_navigation_instance(const eph_t& source, const eph_t& candidate) {
    // Galileo broadcast navigation identity: same satellite, same IODnav (RTKLIB stores
    // the Galileo IODnav in iode), and the exact same Toe epoch. INAV and FNAV update
    // asynchronously, so a historical record of one family must never pair with a newer
    // instance of the other family, and no looser Toe proximity may substitute here.
    return source.sat == candidate.sat && source.iode == candidate.iode && timediff(source.toe, candidate.toe) == 0.0;
}

void fill_galileo_companion_clocks(const nav_t& nav, const eph_t& source, KeplerianNavOutputData* output) {
    for (int index = 0; index < nav.n; ++index) {
        const eph_t& candidate = nav.eph[index];
        if (!same_galileo_navigation_instance(source, candidate)) {
            continue;
        }
        const RtklibBroadcastMessageFamily family = message_family(SYS_GAL, candidate.hdr.msg_type);
        int toc_week = 0;
        double toc_sow_sec = 0.0;
        if (!time_week_sow(candidate.toc, &toc_week, &toc_sow_sec)) {
            continue;
        }
        if (family == RtklibBroadcastMessageFamily::kGalileoFnav) {
            output->galileo_fnav_received = true;
            output->galileo_fnav_toc_sow_sec = toc_sow_sec;
            output->galileo_fnav_clock[0] = candidate.f0;
            output->galileo_fnav_clock[1] = candidate.f1;
            output->galileo_fnav_clock[2] = candidate.f2;
        } else if (family == RtklibBroadcastMessageFamily::kGalileoInav) {
            output->galileo_inav_received = true;
            output->galileo_inav_toc_sow_sec = toc_sow_sec;
            output->galileo_inav_clock[0] = candidate.f0;
            output->galileo_inav_clock[1] = candidate.f1;
            output->galileo_inav_clock[2] = candidate.f2;
        }
    }
}

bool fill_ephemeris(const nav_t& nav, int index, NavOutputRecord* record) {
    const eph_t& eph = nav.eph[index];
    int prn = 0;
    const int system = satsys(eph.sat, &prn);
    KeplerianNavOutputData output{};
    output.system = output_system(system);
    output.message_family = message_family(system, eph.hdr.msg_type);
    output.satellite_number = eph.sat;
    output.prn = prn;
    output.message_type = eph.hdr.msg_type;
    output.iode = eph.iode;
    output.iodc = eph.iodc;
    output.sva = static_cast<double>(eph.sva);
    output.svh = eph.svh;
    output.code = eph.code;
    output.flag = eph.flag;
    if (!time_week_sow(eph.toe, &output.toe_week, &output.toe_sow_sec) ||
        !time_week_sow(eph.toc, &output.toc_week, &output.toc_sow_sec) ||
        !time_week_sow(eph.ttr, &output.transmit_week, &output.transmit_sow_sec)) {
        return false;
    }
    output.semi_major_axis_m = eph.A;
    output.eccentricity = eph.e;
    output.inclination_rad = eph.i0;
    output.omega0_rad = eph.OMG0;
    output.argument_of_perigee_rad = eph.omg;
    output.mean_anomaly_rad = eph.M0;
    output.delta_mean_motion_radps = eph.deln;
    output.omega_dot_radps = eph.OMGd;
    output.inclination_dot_radps = eph.idot;
    output.crc_m = eph.crc;
    output.crs_m = eph.crs;
    output.cuc_rad = eph.cuc;
    output.cus_rad = eph.cus;
    output.cic_rad = eph.cic;
    output.cis_rad = eph.cis;
    output.clock_bias_sec = eph.f0;
    output.clock_drift_sec_per_sec = eph.f1;
    output.clock_drift_rate_sec_per_sec2 = eph.f2;
    std::memcpy(output.tgd_sec, eph.tgd, sizeof(output.tgd_sec));
    std::memcpy(output.isc_sec, eph.isc, sizeof(output.isc_sec));
    output.fit_hours = eph.fit;
    if (system == SYS_GAL) {
        fill_galileo_companion_clocks(nav, eph, &output);
        if (!output.galileo_fnav_received && output.message_family == RtklibBroadcastMessageFamily::kGalileoFnav) {
            output.galileo_fnav_received = true;
            output.galileo_fnav_toc_sow_sec = output.toc_sow_sec;
            output.galileo_fnav_clock[0] = eph.f0;
            output.galileo_fnav_clock[1] = eph.f1;
            output.galileo_fnav_clock[2] = eph.f2;
        }
        if (!output.galileo_inav_received && output.message_family == RtklibBroadcastMessageFamily::kGalileoInav) {
            output.galileo_inav_received = true;
            output.galileo_inav_toc_sow_sec = output.toc_sow_sec;
            output.galileo_inav_clock[0] = eph.f0;
            output.galileo_inav_clock[1] = eph.f1;
            output.galileo_inav_clock[2] = eph.f2;
        }
    }
    record->kind = RtklibNavRecordKind::kEphemeris;
    record->ephemeris = output;
    return output.system != NavOutputSystem::kUnknown;
}

int glonass_day_number(gtime_t gpst) {
    double epoch[6]{};
    time2epoch(gpst2utc(gpst), epoch);
    const int year = static_cast<int>(epoch[0]);
    int cycle_start_year = year - ((year - 1996) % 4 + 4) % 4;
    double start_epoch[6] = {static_cast<double>(cycle_start_year), 1.0, 1.0, 0.0, 0.0, 0.0};
    const double days = timediff(gpst2utc(gpst), epoch2time(start_epoch)) / 86400.0;
    return static_cast<int>(std::floor(days)) + 1;
}

bool fill_glonass(const nav_t& nav, int index, NavOutputRecord* record) {
    const geph_t& geph = nav.geph[index];
    int prn = 0;
    if (satsys(geph.sat, &prn) != SYS_GLO) {
        return false;
    }
    GlonassNavOutputData output{};
    output.message_type = geph.hdr.msg_type != 0 ? geph.hdr.msg_type : NAV_FDMA;
    output.message_family = message_family(SYS_GLO, output.message_type);
    output.satellite_number = geph.sat;
    output.prn = prn;
    output.iode = geph.iode;
    output.frequency_channel = geph.frq;
    output.svh = geph.svh;
    output.sva = geph.sva;
    output.age_days = geph.age;
    output.flags = geph.flag;
    if (!time_week_sow(geph.toe, &output.toe_week, &output.toe_sow_sec) ||
        !time_week_sow(geph.tof, &output.frame_week, &output.frame_sow_sec)) {
        return false;
    }
    std::memcpy(output.position_ecef_m, geph.pos, sizeof(output.position_ecef_m));
    std::memcpy(output.velocity_ecef_mps, geph.vel, sizeof(output.velocity_ecef_mps));
    std::memcpy(output.acceleration_ecef_mps2, geph.acc, sizeof(output.acceleration_ecef_mps2));
    output.clock_bias_sec = geph.taun;
    output.relative_frequency_bias = geph.gamn;
    output.differential_delay_sec = geph.dtaun;
    record->kind = RtklibNavRecordKind::kGlonassEphemeris;
    record->glonass = output;
    return true;
}

bool legacy_beidou_ion_message(int message_type) {
    return message_type == NAV_D1 || message_type == NAV_D2 || message_type == NAV_D1D2;
}

bool fill_explicit_ion(const nav_t& nav, int index, NavOutputRecord* record) {
    const ion_t& ion = nav.ion[index];
    IonosphereNavOutputData output{};
    output.system = output_system(ion.hdr.sys);
    output.prn = ion.hdr.prn;
    output.message_type = ion.hdr.msg_type;
    if (!time_week_sow(ion.trans_time, &output.transmit_week, &output.transmit_sow_sec)) {
        return false;
    }
    output.coefficient_count = ion.ndata > 0 ? ion.ndata : 9;
    if (output.coefficient_count > 9) {
        output.coefficient_count = 9;
    }
    std::memcpy(output.coefficients, ion.alpha, sizeof(output.coefficients));
    output.region = ion.region;
    output.leap_seconds = nav.leaps;
    output.legacy_metadata = ion.hdr.sys == SYS_CMP && legacy_beidou_ion_message(ion.hdr.msg_type);
    record->kind = RtklibNavRecordKind::kIonosphere;
    record->ionosphere = output;
    return output.system != NavOutputSystem::kUnknown;
}

bool fill_legacy_ion(const nav_t& nav, int system, NavOutputRecord* record) {
    IonosphereNavOutputData output{};
    output.system = output_system(system);
    output.legacy_metadata = true;
    output.leap_seconds = nav.leaps;
    output.transmit_week = 0;
    output.transmit_sow_sec = 0.0;
    if (system == SYS_GPS) {
        std::memcpy(output.coefficients, nav.ion_gps, sizeof(double) * 8);
        std::memcpy(output.utc, nav.utc_gps, sizeof(output.utc));
        output.coefficient_count = 8;
    } else if (system == SYS_QZS) {
        std::memcpy(output.coefficients, nav.ion_qzs, sizeof(double) * 8);
        std::memcpy(output.utc, nav.utc_qzs, sizeof(output.utc));
        output.coefficient_count = 8;
    } else if (system == SYS_GAL) {
        std::memcpy(output.coefficients, nav.ion_gal, sizeof(double) * 4);
        std::memcpy(output.utc, nav.utc_gal, sizeof(output.utc));
        output.coefficient_count = 3;
    } else if (system == SYS_CMP) {
        std::memcpy(output.coefficients, nav.ion_cmp, sizeof(double) * 8);
        std::memcpy(output.utc, nav.utc_cmp, sizeof(output.utc));
        output.coefficient_count = 8;
    } else {
        return false;
    }
    record->kind = RtklibNavRecordKind::kIonosphere;
    record->ionosphere = output;
    return true;
}

} // namespace

int rtklib_nav_output_record_count(const RtklibNavStore* store) {
    if (store == nullptr) {
        return 0;
    }
    int systems[4]{};
    const int metadata_count = legacy_ion_systems(store->nav, systems);
    return store->nav.n + store->nav.ng + store->nav.nion + metadata_count;
}

bool rtklib_nav_output_record(const RtklibNavStore* store, int output_record_index, NavOutputRecord* record,
                              std::string* error_message) {
    if (store == nullptr || record == nullptr || output_record_index < 0 ||
        output_record_index >= rtklib_nav_output_record_count(store)) {
        set_error(error_message, "navigation output record request has invalid arguments");
        return false;
    }
    *record = NavOutputRecord{};
    if (output_record_index < store->nav.n) {
        if (!fill_ephemeris(store->nav, output_record_index, record)) {
            set_error(error_message, "cannot project RTKLIB ephemeris for navigation output");
            return false;
        }
        return true;
    }
    output_record_index -= store->nav.n;
    if (output_record_index < store->nav.ng) {
        if (!fill_glonass(store->nav, output_record_index, record)) {
            set_error(error_message, "cannot project RTKLIB GLONASS ephemeris for navigation output");
            return false;
        }
        return true;
    }
    output_record_index -= store->nav.ng;
    if (output_record_index < store->nav.nion) {
        if (!fill_explicit_ion(store->nav, output_record_index, record)) {
            set_error(error_message, "cannot project RTKLIB ionosphere record for navigation output");
            return false;
        }
        return true;
    }
    output_record_index -= store->nav.nion;
    int systems[4]{};
    const int metadata_count = legacy_ion_systems(store->nav, systems);
    if (output_record_index >= metadata_count || !fill_legacy_ion(store->nav, systems[output_record_index], record)) {
        set_error(error_message, "cannot project legacy ionosphere metadata for navigation output");
        return false;
    }
    return true;
}

const char* nav_output_system_name(NavOutputSystem system) {
    switch (system) {
        case NavOutputSystem::kGps:
            return "GPS";
        case NavOutputSystem::kGlonass:
            return "GLO";
        case NavOutputSystem::kGalileo:
            return "GAL";
        case NavOutputSystem::kBeidou:
            return "BDS";
        case NavOutputSystem::kQzss:
            return "QZS";
        case NavOutputSystem::kNavic:
            return "IRN";
        case NavOutputSystem::kUnknown:
            return "UNKNOWN";
    }
    return "UNKNOWN";
}

} // namespace gnss_sim
