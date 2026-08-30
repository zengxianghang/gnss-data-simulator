#include "gnss/rtklib_adapter.h"

#include <cmath>
#include <cstring>

extern "C" {
#include <rtklib.h>
#include <rtklib_signal_bias_ext.h>
}
namespace gnss_sim {

// RtklibNavStore is intentionally opaque outside the RTKLIB adapter boundary.
// This translation unit completes the same internal type used by
// rtklib_adapter.cpp so broadcast-bias access remains inside the adapter.
struct RtklibNavStore {
    nav_t nav;
};

namespace {

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_gps_time(int gps_week, double sow_sec) {
    return gps_week >= 0 && std::isfinite(sow_sec) && sow_sec >= 0.0 && sow_sec < 604800.0;
}

double maximum_ephemeris_age_sec(int system) {
    switch (system) {
        case SYS_QZS:
            return MAXDTOE_QZS + 1.0;
        case SYS_GAL:
            return MAXDTOE_GAL + 1.0;
        case SYS_CMP:
            return MAXDTOE_CMP + 1.0;
        default:
            return MAXDTOE + 1.0;
    }
}

const eph_t* select_ephemeris(const nav_t& nav, gtime_t time, int satellite_number) {
    const int system = satsys(satellite_number, nullptr);
    const double maximum_age_sec = maximum_ephemeris_age_sec(system);
    const eph_t* selected = nullptr;
    double selected_age_sec = maximum_age_sec + 1.0;
    for (int index = 0; index < nav.n; ++index) {
        const eph_t& eph = nav.eph[index];
        if (eph.sat != satellite_number) {
            continue;
        }
        const double age_sec = std::fabs(timediff(eph.toe, time));
        if (age_sec > maximum_age_sec) {
            continue;
        }
        if (selected == nullptr || age_sec <= selected_age_sec) {
            selected = &eph;
            selected_age_sec = age_sec;
        }
    }
    return selected;
}

RtklibBroadcastMessageFamily glonass_message_family(const geph_t& geph) {
    const int message_type = geph.hdr.msg_type != 0 ? geph.hdr.msg_type : NAV_FDMA;
    if (message_type == NAV_L3OC) {
        return RtklibBroadcastMessageFamily::kGlonassL3Oc;
    }
    if (message_type == NAV_FDMA) {
        return RtklibBroadcastMessageFamily::kGlonassFdma;
    }
    return RtklibBroadcastMessageFamily::kUnknown;
}

const geph_t* select_glonass_ephemeris(const nav_t& nav, gtime_t time, int satellite_number,
                                       RtklibBroadcastMessageFamily requested_family) {
    if (requested_family == RtklibBroadcastMessageFamily::kUnknown) {
        requested_family = RtklibBroadcastMessageFamily::kGlonassFdma;
    }
    const geph_t* selected = nullptr;
    double selected_age_sec = MAXDTOE_GLO + 1.0;
    for (int index = 0; index < nav.ng; ++index) {
        const geph_t& geph = nav.geph[index];
        if (geph.sat != satellite_number || glonass_message_family(geph) != requested_family) {
            continue;
        }
        const double age_sec = std::fabs(timediff(geph.toe, time));
        if (age_sec > MAXDTOE_GLO) {
            continue;
        }
        if (selected == nullptr || age_sec < selected_age_sec ||
            (std::fabs(age_sec - selected_age_sec) < 1.0e-9 && timediff(geph.tof, selected->tof) > 0.0)) {
            selected = &geph;
            selected_age_sec = age_sec;
        }
    }
    return selected;
}

RtklibBroadcastMessageFamily message_family(const eph_t& eph, int system) {
    const int message_type = eph.hdr.msg_type;
    if (system == SYS_GAL) {
        if (message_type == NAV_INAV) {
            return RtklibBroadcastMessageFamily::kGalileoInav;
        }
        if (message_type == NAV_FNAV) {
            return RtklibBroadcastMessageFamily::kGalileoFnav;
        }
        // RINEX 3 stores Galileo message/clock source in eph.code. Bit 8
        // identifies E5a/E1 (F/NAV) clock data; bit 9 identifies E5b/E1
        // (I/NAV) clock data. Prefer the clock-source bits over lower signal
        // source bits when both are present.
        if ((eph.code & (1 << 9)) != 0) {
            return RtklibBroadcastMessageFamily::kGalileoInav;
        }
        if ((eph.code & (1 << 8)) != 0) {
            return RtklibBroadcastMessageFamily::kGalileoFnav;
        }
        if ((eph.code & ((1 << 0) | (1 << 2))) != 0) {
            return RtklibBroadcastMessageFamily::kGalileoInav;
        }
        if ((eph.code & (1 << 1)) != 0) {
            return RtklibBroadcastMessageFamily::kGalileoFnav;
        }
        return RtklibBroadcastMessageFamily::kUnknown;
    }
    if (system == SYS_CMP) {
        if (message_type == NAV_CNV1) {
            return RtklibBroadcastMessageFamily::kBeidouBcnav1;
        }
        if (message_type == NAV_CNV2) {
            return RtklibBroadcastMessageFamily::kBeidouBcnav2;
        }
        if (message_type == NAV_CNV3) {
            return RtklibBroadcastMessageFamily::kBeidouBcnav3;
        }
        return RtklibBroadcastMessageFamily::kLegacy;
    }
    if (system == SYS_GPS || system == SYS_QZS) {
        if (message_type == NAV_CNAV) {
            return RtklibBroadcastMessageFamily::kCnav;
        }
        if (message_type == NAV_CNV2) {
            return RtklibBroadcastMessageFamily::kCnav2;
        }
        return RtklibBroadcastMessageFamily::kLegacy;
    }
    return RtklibBroadcastMessageFamily::kUnknown;
}

const eph_t* select_ephemeris_for_family(const nav_t& nav, gtime_t time, int satellite_number,
                                         RtklibBroadcastMessageFamily requested_family) {
    if (requested_family == RtklibBroadcastMessageFamily::kUnknown) {
        return select_ephemeris(nav, time, satellite_number);
    }
    const int system = satsys(satellite_number, nullptr);
    const double maximum_age_sec = maximum_ephemeris_age_sec(system);
    const eph_t* selected = nullptr;
    double selected_age_sec = maximum_age_sec + 1.0;
    for (int index = 0; index < nav.n; ++index) {
        const eph_t& eph = nav.eph[index];
        if (eph.sat != satellite_number || message_family(eph, system) != requested_family)
            continue;
        const double age_sec = std::fabs(timediff(eph.toe, time));
        if (age_sec > maximum_age_sec)
            continue;
        if (selected == nullptr || age_sec < selected_age_sec ||
            (std::fabs(age_sec - selected_age_sec) < 1.0e-9 && timediff(eph.toc, selected->toc) > 0.0)) {
            selected = &eph;
            selected_age_sec = age_sec;
        }
    }
    return selected;
}

} // namespace

bool rtklib_broadcast_bias_data_for_family(const RtklibNavStore* store, int gps_week, double sow_sec,
                                           int satellite_number, RtklibBroadcastMessageFamily requested_message_family,
                                           RtklibBroadcastBiasData* data, std::string* error_message,
                                           RtklibSelectedEphemerisInfo* selected_identity) {
    if (store == nullptr || data == nullptr || satellite_number <= 0 || !valid_gps_time(gps_week, sow_sec)) {
        set_error(error_message, "broadcast-bias request has invalid arguments");
        return false;
    }

    const gtime_t time = gpst2time(gps_week, sow_sec);
    const int system = satsys(satellite_number, nullptr);
    RtklibBroadcastBiasData result{};
    result.system = system;
    result.iode = -1;
    result.glonass_fcn = 0;

    if (system == SYS_GLO) {
        const geph_t* geph = select_glonass_ephemeris(store->nav, time, satellite_number, requested_message_family);
        if (geph == nullptr) {
            set_error(error_message, "no matching GLONASS ephemeris family for broadcast bias");
            return false;
        }
        result.message_family = glonass_message_family(*geph);
        result.iode = geph->iode;
        if (result.message_family == RtklibBroadcastMessageFamily::kGlonassFdma) {
            result.glonass_fcn = geph->frq;
            result.glonass_dtaun_sec = geph->dtaun;
        } else if (result.message_family == RtklibBroadcastMessageFamily::kGlonassL3Oc) {
            result.glonass_fcn = 0;
            result.glonass_isc_l3ocp_sec = geph->isc_l3ocp;
        }
        if (selected_identity != nullptr) {
            // Identity comes from the same selected geph_t that produced the bias data.
            selected_identity->satellite_number = satellite_number;
            selected_identity->message_family = result.message_family;
            selected_identity->iode = geph->iode;
            selected_identity->iodc = 0;
            selected_identity->toe_sow_sec = time2gpst(geph->toe, &selected_identity->toe_week);
            selected_identity->toc_week = selected_identity->toe_week;
            selected_identity->toc_sow_sec = selected_identity->toe_sow_sec;
            selected_identity->transmit_sow_sec = time2gpst(geph->tof, &selected_identity->transmit_week);
        }
        *data = result;
        return true;
    }

    const eph_t* eph = select_ephemeris_for_family(store->nav, time, satellite_number, requested_message_family);
    if (eph == nullptr) {
        set_error(error_message, "no matching broadcast ephemeris for code bias");
        return false;
    }

    result.message_family = message_family(*eph, system);
    result.iode = eph->iode;
    std::memcpy(result.tgd_sec, eph->tgd, sizeof(result.tgd_sec));
    std::memcpy(result.isc_sec, eph->isc, sizeof(result.isc_sec));
    if (selected_identity != nullptr) {
        // Identity comes from the same selected eph_t that produced the bias data.
        selected_identity->satellite_number = satellite_number;
        selected_identity->message_family = result.message_family;
        selected_identity->iode = eph->iode;
        selected_identity->iodc = eph->iodc;
        selected_identity->toe_sow_sec = time2gpst(eph->toe, &selected_identity->toe_week);
        selected_identity->toc_sow_sec = time2gpst(eph->toc, &selected_identity->toc_week);
        selected_identity->transmit_sow_sec = time2gpst(eph->ttr, &selected_identity->transmit_week);
    }
    *data = result;
    return true;
}

bool rtklib_broadcast_bias_data(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,
                                RtklibBroadcastBiasData* data, std::string* error_message) {
    return rtklib_broadcast_bias_data_for_family(store, gps_week, sow_sec, satellite_number,
                                                 RtklibBroadcastMessageFamily::kUnknown, data, error_message);
}

bool rtklib_signal_health_for_family(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,
                                     const char* rinex_signal_code,
                                     RtklibBroadcastMessageFamily requested_message_family, int* signal_health,
                                     std::string* error_message) {
    if (store == nullptr || rinex_signal_code == nullptr || rinex_signal_code[0] == '\0' || signal_health == nullptr ||
        satellite_number <= 0 || !valid_gps_time(gps_week, sow_sec)) {
        set_error(error_message, "signal-health request has invalid arguments");
        return false;
    }

    int observation_code = 0;
    int frequency_index = 0;
    if (!rtklib_observation_code(rinex_signal_code, &observation_code, &frequency_index)) {
        set_error(error_message, "signal-health request has unsupported observation code");
        return false;
    }
    static_cast<void>(frequency_index);

    int required_message_mask = 0;
    switch (requested_message_family) {
        case RtklibBroadcastMessageFamily::kCnav:
            required_message_mask = NAV_CNAV;
            break;
        case RtklibBroadcastMessageFamily::kCnav2:
            required_message_mask = NAV_CNV2;
            break;
        case RtklibBroadcastMessageFamily::kGalileoInav:
            required_message_mask = NAV_INAV;
            break;
        case RtklibBroadcastMessageFamily::kGalileoFnav:
            required_message_mask = NAV_FNAV;
            break;
        case RtklibBroadcastMessageFamily::kBeidouBcnav1:
            required_message_mask = NAV_CNV1;
            break;
        case RtklibBroadcastMessageFamily::kBeidouBcnav2:
            required_message_mask = NAV_CNV2;
            break;
        case RtklibBroadcastMessageFamily::kBeidouBcnav3:
            required_message_mask = NAV_CNV3;
            break;
        case RtklibBroadcastMessageFamily::kGlonassFdma:
            required_message_mask = NAV_FDMA;
            break;
        case RtklibBroadcastMessageFamily::kGlonassL3Oc:
            required_message_mask = NAV_L3OC;
            break;
        case RtklibBroadcastMessageFamily::kLegacy:
        case RtklibBroadcastMessageFamily::kUnknown:
            required_message_mask = 0;
            break;
    }

    const gtime_t time = gpst2time(gps_week, sow_sec);
    eph_t eph{};
    geph_t geph{};
    rtklib_signal_bias_info_ext_t info{};
    const int status = rtklib_signal_ephemeris_ext(time, satellite_number, static_cast<unsigned char>(observation_code),
                                                   required_message_mask, &store->nav, &eph, &geph, &info);
    if (status == 0) {
        *signal_health = 1;
        return true;
    }
    if (status < 0) {
        set_error(error_message, "signal/message-family status lookup failed");
        return false;
    }

    const int raw_health = info.system == SYS_GLO ? geph.svh : eph.svh;
    *signal_health = rtklib_signal_health_ext(info.system, info.message_type,
                                              static_cast<unsigned char>(observation_code), raw_health);
    return true;
}

} // namespace gnss_sim
