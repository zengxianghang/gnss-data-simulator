#include "gnss/nav_output_record.h"

extern "C" {
#include <rtklib.h>
}

#include <cmath>

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

} // namespace

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
        return true;
    }
    return record->kind == RtklibNavRecordKind::kIonosphere;
}

} // namespace gnss_sim
