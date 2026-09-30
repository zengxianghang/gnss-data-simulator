#include "gnss/nav_output_record.h"

#include <cmath>

extern "C" {
#include <rtklib.h>
}
namespace gnss_sim {
namespace {

constexpr double kGpsMu = 3.986005e14;
constexpr double kOtherMu = 3.986004418e14;

int calendar_day_in_glonass_cycle(gtime_t gpst) {
    const gtime_t utc = gpst2utc(gpst);
    double epoch[6]{};
    time2epoch(utc, epoch);
    const int year = static_cast<int>(epoch[0]);
    const int cycle_start_year = year - ((year - 1996) % 4 + 4) % 4;
    double start_epoch[6] = {static_cast<double>(cycle_start_year), 1.0, 1.0, 0.0, 0.0, 0.0};
    return static_cast<int>(std::floor(timediff(utc, epoch2time(start_epoch)) / 86400.0)) + 1;
}

double glonass_day_seconds(gtime_t gpst) {
    double epoch[6]{};
    time2epoch(gpst2utc(gpst), epoch);
    double seconds = epoch[3] * 3600.0 + epoch[4] * 60.0 + epoch[5] + 10800.0;
    seconds = std::fmod(seconds, 86400.0);
    if (seconds < 0.0) {
        seconds += 86400.0;
    }
    return seconds;
}

void decode_galileo_health(KeplerianNavOutputData* eph) {
    if (eph->system != NavOutputSystem::kGalileo) {
        return;
    }
    // Pinned RTKLIB stores the RINEX Galileo SV health word as:
    // bit 0 E1B DVS, bits 1-2 E1B HS, bit 3 E5a DVS, bits 4-5 E5a HS,
    // bit 6 E5b DVS, bits 7-8 E5b HS.
    eph->galileo_e1b_dvs = eph->svh & 0x1;
    eph->galileo_e1b_health = (eph->svh >> 1) & 0x3;
    eph->galileo_e5a_dvs = (eph->svh >> 3) & 0x1;
    eph->galileo_e5a_health = (eph->svh >> 4) & 0x3;
    eph->galileo_e5b_dvs = (eph->svh >> 6) & 0x1;
    eph->galileo_e5b_health = (eph->svh >> 7) & 0x3;
}

// Galileo OS SIS ICD SISA index from the RINEX SISA (m): 0-49 in 1 cm steps
// from 0 m, 50-74 in 2 cm steps from 0.5 m, 75-99 in 4 cm steps from 1 m,
// 100-125 in 16 cm steps from 2 m; RINEX -1 (no accuracy prediction) is 255.
int galileo_sisa_index(double sisa_m) {
    if (!std::isfinite(sisa_m)) {
        return -1;
    }
    if (sisa_m < 0.0) {
        return 255;
    }
    struct Band {
        double start_m;
        double step_m;
        int first;
        int last;
    };
    static const Band kBands[] = {{0.0, 0.01, 0, 49}, {0.5, 0.02, 50, 74}, {1.0, 0.04, 75, 99}, {2.0, 0.16, 100, 125}};
    for (const Band& band : kBands) {
        const long steps = std::lround((sisa_m - band.start_m) / band.step_m);
        if (steps >= 0 && band.first + steps <= band.last) {
            return band.first + static_cast<int>(steps);
        }
    }
    return -1;
}

// RINEX 4.01 GLONASS status flags: P bits 0-1, P1 bits 2-3, P2 bit 4, P3 bit
// 5, P4 bit 6, M bits 7-8.
void derive_glonass_flags(GlonassNavOutputData* glo) {
    const int flags = glo->flags;
    glo->time_offset_parameter = flags & 0x3;
    glo->vendor_flags =
        ((flags >> 2) & 0x3) | (((flags >> 4) & 0x1) << 2) | (((flags >> 5) & 0x1) << 3) | (((flags >> 6) & 0x1) << 4);
}

} // namespace

double galileo_sisa_metres(int index) {
    if (index == 255) {
        return -1.0;
    }
    double metres = -1.0;
    if (index >= 0 && index <= 49) {
        metres = index * 0.01;
    } else if (index >= 50 && index <= 74) {
        metres = 0.5 + (index - 50) * 0.02;
    } else if (index >= 75 && index <= 99) {
        metres = 1.0 + (index - 75) * 0.04;
    } else if (index >= 100 && index <= 125) {
        metres = 2.0 + (index - 100) * 0.16;
    } else {
        return std::nan("");
    }
    return std::round(metres * 100.0) / 100.0; // every ICD value is a whole centimetre
}

bool finalize_nav_output_record_metadata(NavOutputRecord* record) {
    if (record == nullptr) {
        return false;
    }
    if (record->kind == RtklibNavRecordKind::kEphemeris) {
        KeplerianNavOutputData& eph = record->ephemeris;
        if (!std::isfinite(eph.semi_major_axis_m) || eph.semi_major_axis_m <= 0.0) {
            return false;
        }
        eph.sqrt_semi_major_axis_sqrt_m = std::sqrt(eph.semi_major_axis_m);
        const double mu =
            eph.system == NavOutputSystem::kGps || eph.system == NavOutputSystem::kQzss ? kGpsMu : kOtherMu;
        eph.corrected_mean_motion_radps =
            std::sqrt(mu / (eph.semi_major_axis_m * eph.semi_major_axis_m * eph.semi_major_axis_m)) +
            eph.delta_mean_motion_radps;
        decode_galileo_health(&eph);
        eph.galileo_sisa_index = eph.system == NavOutputSystem::kGalileo ? galileo_sisa_index(eph.sva) : -1;
        return std::isfinite(eph.sqrt_semi_major_axis_sqrt_m) && std::isfinite(eph.corrected_mean_motion_radps);
    }
    if (record->kind == RtklibNavRecordKind::kGlonassEphemeris) {
        GlonassNavOutputData& glo = record->glonass;
        if (glo.prn <= 0 || glo.toe_week < 0 || glo.frame_week < 0 || !std::isfinite(glo.toe_sow_sec) ||
            !std::isfinite(glo.frame_sow_sec)) {
            return false;
        }
        glo.slot_offset = glo.prn + 37;
        glo.frequency_offset = glo.frequency_channel + 7;
        const gtime_t toe = gpst2time(glo.toe_week, glo.toe_sow_sec);
        const gtime_t frame = gpst2time(glo.frame_week, glo.frame_sow_sec);
        const int leap_seconds = static_cast<int>(std::llround(timediff(toe, gpst2utc(toe))));
        glo.gps_glonass_time_offset_sec = 10800 - leap_seconds;
        glo.calendar_day_number = calendar_day_in_glonass_cycle(toe);
        glo.frame_time_glonass_day_sec = glonass_day_seconds(frame);
        derive_glonass_flags(&glo);
        return true;
    }
    return record->kind == RtklibNavRecordKind::kIonosphere;
}

} // namespace gnss_sim
