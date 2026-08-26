#include "gnss/rtklib_adapter.h"

extern "C" {
#include <rtklib.h>
}

#include <cmath>
#include <cstring>

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

const geph_t* select_glonass_ephemeris(const nav_t& nav, gtime_t time, int satellite_number) {
    const geph_t* selected = nullptr;
    double selected_age_sec = MAXDTOE_GLO + 1.0;
    for (int index = 0; index < nav.ng; ++index) {
        const geph_t& geph = nav.geph[index];
        if (geph.sat != satellite_number) {
            continue;
        }
        const double age_sec = std::fabs(timediff(geph.toe, time));
        if (age_sec > MAXDTOE_GLO) {
            continue;
        }
        if (selected == nullptr || age_sec <= selected_age_sec) {
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

} // namespace

bool rtklib_broadcast_bias_data(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,
                                RtklibBroadcastBiasData* data, std::string* error_message) {
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
        const geph_t* geph = select_glonass_ephemeris(store->nav, time, satellite_number);
        if (geph == nullptr) {
            set_error(error_message, "no matching GLONASS ephemeris for broadcast bias");
            return false;
        }
        result.message_family = RtklibBroadcastMessageFamily::kGlonassFdma;
        result.iode = geph->iode;
        result.glonass_fcn = geph->frq;
        result.glonass_dtaun_sec = geph->dtaun;
        *data = result;
        return true;
    }

    const eph_t* eph = select_ephemeris(store->nav, time, satellite_number);
    if (eph == nullptr) {
        set_error(error_message, "no matching broadcast ephemeris for code bias");
        return false;
    }

    result.message_family = message_family(*eph, system);
    result.iode = eph->iode;
    std::memcpy(result.tgd_sec, eph->tgd, sizeof(result.tgd_sec));
    std::memcpy(result.isc_sec, eph->isc, sizeof(result.isc_sec));
    *data = result;
    return true;
}

} // namespace gnss_sim
