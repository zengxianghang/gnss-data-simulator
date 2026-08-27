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
        const double mu = eph.system == NavOutputSystem::kGps || eph.system == NavOutputSystem::kQzss ? kGpsMu : kOtherMu;
        eph.corrected_mean_motion_radps =
            std::sqrt(mu / (eph.semi_major_axis_m * eph.semi_major_axis_m * eph.semi_major_axis_m)) +
            eph.delta_mean_motion_radps;
        return std::isfinite(eph.corrected_mean_motion_radps);
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
