#include "gnss/rtklib_residual_adapter.h"

extern "C" {
#include <rtklib.h>
#include <rtklib_residual_ext.h>
}

#include <cmath>
#include <cstring>

namespace gnss_sim {

struct RtklibNavStore {
    nav_t nav;
};

namespace {

constexpr double kRadiansPerDegree = 3.141592653589793238462643383279502884 / 180.0;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool finite_vector3(const double value[3]) {
    return value != nullptr && std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

bool valid_time(int gps_week, double sow_sec) {
    return gps_week >= 0 && std::isfinite(sow_sec) && sow_sec >= 0.0 && sow_sec < 604800.0;
}

int required_message_mask(RtklibBroadcastMessageFamily family, int system) {
    switch (family) {
        case RtklibBroadcastMessageFamily::kUnknown:
            return 0;
        case RtklibBroadcastMessageFamily::kLegacy:
            if (system == SYS_GPS || system == SYS_QZS) {
                return NAV_LNAV;
            }
            if (system == SYS_CMP) {
                return NAV_D1D2 | NAV_D1 | NAV_D2;
            }
            return 0;
        case RtklibBroadcastMessageFamily::kCnav:
            return NAV_CNAV;
        case RtklibBroadcastMessageFamily::kCnav2:
            return NAV_CNV2;
        case RtklibBroadcastMessageFamily::kGalileoInav:
            return NAV_INAV;
        case RtklibBroadcastMessageFamily::kGalileoFnav:
            return NAV_FNAV;
        case RtklibBroadcastMessageFamily::kBeidouBcnav1:
            return NAV_CNV1;
        case RtklibBroadcastMessageFamily::kBeidouBcnav2:
            return NAV_CNV2;
        case RtklibBroadcastMessageFamily::kBeidouBcnav3:
            return NAV_CNV3;
        case RtklibBroadcastMessageFamily::kGlonassFdma:
            return NAV_FDMA;
    }
    return 0;
}

RtklibBroadcastMessageFamily message_family_from_type(int system, int message_type) {
    if (system == SYS_GPS || system == SYS_QZS) {
        if (message_type == NAV_CNAV) {
            return RtklibBroadcastMessageFamily::kCnav;
        }
        if (message_type == NAV_CNV2) {
            return RtklibBroadcastMessageFamily::kCnav2;
        }
        if (message_type == NAV_LNAV) {
            return RtklibBroadcastMessageFamily::kLegacy;
        }
    }
    if (system == SYS_GAL) {
        if (message_type == NAV_INAV) {
            return RtklibBroadcastMessageFamily::kGalileoInav;
        }
        if (message_type == NAV_FNAV) {
            return RtklibBroadcastMessageFamily::kGalileoFnav;
        }
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
        if ((message_type & (NAV_D1D2 | NAV_D1 | NAV_D2)) != 0) {
            return RtklibBroadcastMessageFamily::kLegacy;
        }
    }
    if (system == SYS_GLO && message_type == NAV_FDMA) {
        return RtklibBroadcastMessageFamily::kGlonassFdma;
    }
    return RtklibBroadcastMessageFamily::kUnknown;
}

prcopt_t residual_options(double elevation_mask_deg, bool broadcast_atmosphere) {
    prcopt_t options = prcopt_default;
    options.mode = PMODE_SINGLE;
    options.nf = 1;
    options.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_CMP;
    options.elmin = elevation_mask_deg * kRadiansPerDegree;
    options.sateph = EPHOPT_BRDC;
    options.ionoopt = broadcast_atmosphere ? IONOOPT_BRDC : IONOOPT_OFF;
    options.tropopt = broadcast_atmosphere ? TROPOPT_SAAS : TROPOPT_OFF;
    options.posopt[4] = 0;
    return options;
}

} // namespace

bool rtklib_truth_state_signal_residuals(const RtklibNavStore* nav_store, int gps_week, double sow_sec,
                                         int satellite_number, int observation_code,
                                         RtklibBroadcastMessageFamily required_message_family,
                                         double pseudorange_m, double doppler_hz, double wavelength_m,
                                         const double receiver_ecef_m[3], const double receiver_velocity_ecef_mps[3],
                                         double receiver_clock_bias_m, double receiver_system_bias_m,
                                         double receiver_clock_drift_mps, double elevation_mask_deg,
                                         bool broadcast_atmosphere, RtklibSignalResidualResult* result,
                                         std::string* error_message) {
    if (nav_store == nullptr || result == nullptr || satellite_number <= 0 || observation_code <= 0 ||
        observation_code > 255 || !valid_time(gps_week, sow_sec) || !std::isfinite(pseudorange_m) ||
        pseudorange_m <= 0.0 || !std::isfinite(doppler_hz) || !std::isfinite(wavelength_m) || wavelength_m <= 0.0 ||
        !finite_vector3(receiver_ecef_m) || !finite_vector3(receiver_velocity_ecef_mps) ||
        !std::isfinite(receiver_clock_bias_m) || !std::isfinite(receiver_system_bias_m) ||
        !std::isfinite(receiver_clock_drift_mps) || !std::isfinite(elevation_mask_deg) || elevation_mask_deg < -90.0 ||
        elevation_mask_deg > 90.0) {
        set_error(error_message, "truth-state residual request has invalid arguments");
        return false;
    }

    RtklibSignalResidualResult output{};
    const int system = satsys(satellite_number, nullptr);
    if (system == SYS_NONE) {
        set_error(error_message, "truth-state residual satellite has no RTKLIB system");
        return false;
    }

    obsd_t observation{};
    observation.time = gpst2time(gps_week, sow_sec);
    observation.sat = static_cast<unsigned char>(satellite_number);
    observation.rcv = 1;
    observation.P[0] = pseudorange_m;
    observation.D[0] = static_cast<float>(doppler_hz);
    observation.SNR[0] = 200;
    observation.code[0] = static_cast<unsigned char>(observation_code);

    prcopt_t options = residual_options(elevation_mask_deg, broadcast_atmosphere);
    rtklib_signal_bias_info_ext_t bias_info{};
    double code_azel[2]{};
    const int code_status = rtklib_rescode_signal_ext(
        &observation, &nav_store->nav, &options, receiver_ecef_m, receiver_clock_bias_m, receiver_system_bias_m,
        required_message_mask(required_message_family, system), wavelength_m, &output.code_residual_m, code_azel,
        &bias_info);
    if (code_status < 0) {
        set_error(error_message, "RTKLIB signal code residual evaluation failed");
        return false;
    }
    output.code_available = code_status > 0;
    if (output.code_available) {
        output.rtklib_code_bias_m = bias_info.raw_code_bias_m;
        output.selected_message_family = message_family_from_type(system, bias_info.message_type);
        output.selected_iode = bias_info.iode;
        output.azimuth_rad = code_azel[0];
        output.elevation_rad = code_azel[1];
    }

    double doppler_azel[2]{};
    const int doppler_status = rtklib_resdop_signal_ext(
        &observation, &nav_store->nav, &options, receiver_ecef_m, receiver_velocity_ecef_mps,
        receiver_clock_drift_mps, wavelength_m, &output.doppler_residual_mps, doppler_azel);
    if (doppler_status < 0) {
        set_error(error_message, "RTKLIB signal Doppler residual evaluation failed");
        return false;
    }
    output.doppler_available = doppler_status > 0;
    if (!output.code_available && output.doppler_available) {
        output.azimuth_rad = doppler_azel[0];
        output.elevation_rad = doppler_azel[1];
    }

    *result = output;
    return true;
}

} // namespace gnss_sim
