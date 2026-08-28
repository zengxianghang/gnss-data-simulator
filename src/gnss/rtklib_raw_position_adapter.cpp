#include "gnss/rtklib_adapter.h"

extern "C" {
#include <rtklib.h>
#include <rtklib_signal_bias_ext.h>
}

#include <cmath>
#include <cstdio>

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

bool valid_time(int gps_week, double sow_sec) {
    return gps_week >= 0 && std::isfinite(sow_sec) && sow_sec >= 0.0 && sow_sec < 604800.0;
}

int required_message_mask(int system, RtklibBroadcastMessageFamily family) {
    switch (family) {
        case RtklibBroadcastMessageFamily::kLegacy:
            if (system == SYS_GPS || system == SYS_QZS) {
                return NAV_LNAV;
            }
            if (system == SYS_CMP) {
                return NAV_D1 | NAV_D2 | NAV_D1D2;
            }
            break;
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
        case RtklibBroadcastMessageFamily::kGlonassL3Oc:
            return NAV_L3OC;
        case RtklibBroadcastMessageFamily::kUnknown:
            break;
    }
    return 0;
}

bool prepare_raw_observation(const RtklibNavStore* receiver_snapshot, gtime_t epoch_time,
                             const RtklibRawCodeObservation& source, int index,
                             RtklibSolutionObservation* destination, std::string* error_message) {
    if (receiver_snapshot == nullptr || destination == nullptr || !source.pseudorange_valid ||
        source.satellite_number <= 0 || source.satellite_number > MAXSAT || source.observation_code <= 0 ||
        source.observation_code > 255 || !std::isfinite(source.pseudorange_m) || source.pseudorange_m <= 0.0 ||
        !std::isfinite(source.cn0_dbhz)) {
        if (error_message != nullptr) {
            char message[160]{};
            std::snprintf(message, sizeof(message), "raw RANGEA observation %d is invalid", index);
            *error_message = message;
        }
        return false;
    }

    const int system = satsys(source.satellite_number, nullptr);
    const int message_mask = required_message_mask(system, source.message_family);
    if (message_mask == 0) {
        if (error_message != nullptr) {
            char message[192]{};
            std::snprintf(message, sizeof(message),
                          "raw RANGEA observation %d has unsupported message family: sat=%d code=%d", index,
                          source.satellite_number, source.observation_code);
            *error_message = message;
        }
        return false;
    }

    double rtklib_code_bias_m = 0.0;
    rtklib_signal_bias_info_ext_t bias_info{};
    const int bias_status =
        rtklib_signal_code_bias_ext(epoch_time, source.satellite_number,
                                    static_cast<unsigned char>(source.observation_code), message_mask,
                                    &receiver_snapshot->nav, &rtklib_code_bias_m, &bias_info);
    if (bias_status != 1 || !std::isfinite(rtklib_code_bias_m)) {
        if (error_message != nullptr) {
            char message[224]{};
            std::snprintf(message, sizeof(message),
                          "raw RANGEA observation %d has no receiver-available matching ephemeris/code bias: "
                          "sat=%d code=%d mask=%d status=%d",
                          index, source.satellite_number, source.observation_code, message_mask, bias_status);
            *error_message = message;
        }
        return false;
    }

    RtklibSolutionObservation converted{};
    converted.satellite_number = source.satellite_number;
    converted.observation_code = source.observation_code;
    converted.message_family = source.message_family;
    converted.pseudorange_m = source.pseudorange_m;
    // The maintained solution adapter computes:
    // solver_P = observation_P - code_bias_m + RTKLIB_code_bias.
    // Supplying the same RTKLIB bias here preserves the serialized raw
    // pseudorange exactly before pntpos() applies its own code-bias correction.
    converted.code_bias_m = rtklib_code_bias_m;
    converted.cn0_dbhz = source.cn0_dbhz;
    converted.pseudorange_valid = true;
    *destination = converted;
    return true;
}

} // namespace

bool rtklib_solve_raw_single_position(const RtklibNavStore* receiver_nav, int gps_week, double sow_sec,
                                      const RtklibRawCodeObservation* observations, int observation_count,
                                      double elevation_mask_deg, bool broadcast_atmosphere,
                                      RtklibPositionSolution* solution, std::string* error_message) {
    if (receiver_nav == nullptr || observations == nullptr || observation_count < 0 || observation_count > MAXOBS ||
        solution == nullptr || !valid_time(gps_week, sow_sec) || !std::isfinite(elevation_mask_deg) ||
        elevation_mask_deg < -90.0 || elevation_mask_deg > 90.0) {
        set_error(error_message, "raw position-solution request has invalid arguments or exceeds MAXOBS");
        return false;
    }

    RtklibNavStore* receiver_snapshot = create_rtklib_nav_store();
    if (receiver_snapshot == nullptr) {
        set_error(error_message, "cannot allocate receiver-available NAV snapshot for raw RANGEA positioning");
        return false;
    }
    if (!rtklib_copy_nav_snapshot(receiver_nav, gps_week, sow_sec, receiver_snapshot, error_message)) {
        destroy_rtklib_nav_store(receiver_snapshot);
        return false;
    }

    const gtime_t epoch_time = gpst2time(gps_week, sow_sec);
    RtklibSolutionObservation converted[MAXOBS]{};
    bool seen_satellite[MAXSAT]{};
    for (int index = 0; index < observation_count; ++index) {
        const RtklibRawCodeObservation& source = observations[index];
        if (source.satellite_number > 0 && source.satellite_number <= MAXSAT &&
            seen_satellite[source.satellite_number - 1]) {
            if (error_message != nullptr) {
                char message[160]{};
                std::snprintf(message, sizeof(message), "raw RANGEA positioning received duplicate satellite %d",
                              source.satellite_number);
                *error_message = message;
            }
            destroy_rtklib_nav_store(receiver_snapshot);
            return false;
        }
        if (!prepare_raw_observation(receiver_snapshot, epoch_time, source, index, &converted[index], error_message)) {
            destroy_rtklib_nav_store(receiver_snapshot);
            return false;
        }
        seen_satellite[source.satellite_number - 1] = true;
    }

    const bool solved = rtklib_solve_single_position(receiver_snapshot, gps_week, sow_sec, converted, observation_count,
                                                      elevation_mask_deg, broadcast_atmosphere, solution, error_message);
    destroy_rtklib_nav_store(receiver_snapshot);
    return solved;
}

} // namespace gnss_sim
