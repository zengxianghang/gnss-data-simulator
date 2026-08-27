#include "gnss/rtklib_adapter.h"

extern "C" {
#include <rtklib.h>
}

#include <cmath>

namespace gnss_sim {

struct RtklibNavStore {
    nav_t nav;
};

namespace {

constexpr double kRelativeHumidity = 0.7;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_gps_time(int gps_week, double sow_sec) {
    return gps_week >= 0 && std::isfinite(sow_sec) && sow_sec >= 0.0 && sow_sec < 604800.0;
}

bool finite_vector3(const double vector[3]) {
    return vector != nullptr && std::isfinite(vector[0]) && std::isfinite(vector[1]) && std::isfinite(vector[2]);
}

bool finite_azel(double azimuth_rad, double elevation_rad) {
    return std::isfinite(azimuth_rad) && std::isfinite(elevation_rad);
}

} // namespace

bool rtklib_broadcast_ionosphere_reference_delay(const RtklibNavStore* store, RtklibIonosphereSystem system,
                                                 int gps_week, double sow_sec, const double receiver_ecef_m[3],
                                                 double azimuth_rad, double elevation_rad,
                                                 RtklibIonosphereResult* result, std::string* error_message) {
    if (store == nullptr || result == nullptr || !valid_gps_time(gps_week, sow_sec) ||
        !finite_vector3(receiver_ecef_m) || !finite_azel(azimuth_rad, elevation_rad)) {
        set_error(error_message, "broadcast ionosphere request has invalid arguments");
        return false;
    }

    switch (system) {
        case RtklibIonosphereSystem::kGps:
        case RtklibIonosphereSystem::kQzss:
        case RtklibIonosphereSystem::kBeidouLegacy:
        case RtklibIonosphereSystem::kGalileo:
        case RtklibIonosphereSystem::kGlonass:
        case RtklibIonosphereSystem::kBeidouModern:
            break;
    }

    // The pinned pntpos() implementation uses IONOOPT_BRDC for every
    // constellation. ionocorr() therefore evaluates nav.ion_gps on GPS L1 and
    // deliberately retains RTKLIB's built-in fallback when those coefficients
    // are all zero. Do not pre-classify zero coefficients as missing here: that
    // would make generated pseudorange diverge from the solver correction.
    const gtime_t time = gpst2time(gps_week, sow_sec);
    double position_rad_m[3]{};
    const double azel[2] = {azimuth_rad, elevation_rad};
    ecef2pos(receiver_ecef_m, position_rad_m);

    double ionosphere_delay_m = 0.0;
    double variance_m2 = 0.0;
    const int representative_satellite = satno(SYS_GPS, 1);
    if (representative_satellite <= 0 ||
        ionocorr(time, &store->nav, representative_satellite, position_rad_m, azel, IONOOPT_BRDC, &ionosphere_delay_m,
                 &variance_m2) == 0 ||
        !std::isfinite(ionosphere_delay_m) || ionosphere_delay_m < 0.0) {
        set_error(error_message, "RTKLIB returned an invalid broadcast ionosphere delay");
        return false;
    }

    RtklibIonosphereResult output{};
    output.status = RtklibIonosphereStatus::kApplied;
    output.reference_delay_m = ionosphere_delay_m;
    output.reference_frequency_hz = FREQ1;
    *result = output;
    return true;
}

bool rtklib_troposphere_delay(int gps_week, double sow_sec, const double receiver_ecef_m[3], double azimuth_rad,
                              double elevation_rad, double* delay_m, std::string* error_message) {
    if (delay_m == nullptr || !valid_gps_time(gps_week, sow_sec) || !finite_vector3(receiver_ecef_m) ||
        !finite_azel(azimuth_rad, elevation_rad)) {
        set_error(error_message, "troposphere request has invalid arguments");
        return false;
    }

    const gtime_t time = gpst2time(gps_week, sow_sec);
    double position_rad_m[3]{};
    const double azel[2] = {azimuth_rad, elevation_rad};
    ecef2pos(receiver_ecef_m, position_rad_m);
    const double delay = tropmodel(time, position_rad_m, azel, kRelativeHumidity);
    if (!std::isfinite(delay) || delay < 0.0) {
        set_error(error_message, "RTKLIB returned an invalid troposphere delay");
        return false;
    }
    *delay_m = delay;
    return true;
}

} // namespace gnss_sim
