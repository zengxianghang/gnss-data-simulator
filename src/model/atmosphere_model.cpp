#include "model/atmosphere_model.h"

#include "gnss_sim/sim_time.h"

#include <cmath>

namespace gnss_sim {
namespace {

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool finite_vector3(const double vector[3]) {
    return vector != nullptr && std::isfinite(vector[0]) && std::isfinite(vector[1]) && std::isfinite(vector[2]);
}

bool valid_time(const SimTime& time) {
    return time.gps_week >= 0 && time.tow_ns >= 0 && time.tow_ns < GPS_WEEK_NANOSECONDS;
}

bool ionosphere_system_for_signal(SignalId signal_id, RtklibIonosphereSystem* system) {
    if (system == nullptr) {
        return false;
    }
    const SignalDefinition* definition = find_signal_definition(signal_id);
    if (definition == nullptr) {
        return false;
    }

    switch (definition->constellation) {
        case GnssConstellation::kGps:
            *system = RtklibIonosphereSystem::kGps;
            return true;
        case GnssConstellation::kQzss:
            *system = RtklibIonosphereSystem::kQzss;
            return true;
        case GnssConstellation::kGlonass:
            *system = RtklibIonosphereSystem::kGlonass;
            return true;
        case GnssConstellation::kGalileo:
            *system = RtklibIonosphereSystem::kGalileo;
            return true;
        case GnssConstellation::kBeidou:
            switch (signal_id) {
                case SignalId::kBeidouB1I:
                case SignalId::kBeidouB3I:
                    *system = RtklibIonosphereSystem::kBeidouLegacy;
                    return true;
                case SignalId::kBeidouB1C:
                case SignalId::kBeidouB2A:
                case SignalId::kBeidouB2B:
                    *system = RtklibIonosphereSystem::kBeidouModern;
                    return true;
                default:
                    return false;
            }
    }
    return false;
}

IonosphereCorrectionStatus convert_status(RtklibIonosphereStatus status) {
    switch (status) {
        case RtklibIonosphereStatus::kApplied:
            return IonosphereCorrectionStatus::kApplied;
        case RtklibIonosphereStatus::kMissingParameters:
            return IonosphereCorrectionStatus::kMissingParameters;
        case RtklibIonosphereStatus::kUnsupportedModel:
            return IonosphereCorrectionStatus::kUnsupportedBroadcastModel;
    }
    return IonosphereCorrectionStatus::kUnsupportedBroadcastModel;
}

} // namespace

bool compute_atmosphere_correction(AtmosphereMode mode, const RtklibNavStore* nav_store, const SimTime& time,
                                   SignalId signal_id, int glonass_fcn, const double receiver_ecef_m[3],
                                   double azimuth_rad, double elevation_rad, AtmosphereCorrection* correction,
                                   std::string* error_message) {
    if (correction == nullptr || !valid_time(time) || find_signal_definition(signal_id) == nullptr ||
        !finite_vector3(receiver_ecef_m) || !std::isfinite(azimuth_rad) || !std::isfinite(elevation_rad)) {
        set_error(error_message, "atmosphere correction request has invalid arguments");
        return false;
    }

    AtmosphereCorrection result{};
    result.mode = mode;
    result.ionosphere_status = IonosphereCorrectionStatus::kDisabled;
    result.ionosphere_code_delay_m = 0.0;
    result.troposphere_delay_m = 0.0;
    if (mode == AtmosphereMode::NONE) {
        *correction = result;
        return true;
    }
    if (mode != AtmosphereMode::BROADCAST || nav_store == nullptr) {
        set_error(error_message, "broadcast atmosphere correction requires a navigation store and explicit mode");
        return false;
    }

    const double sow_sec = sim_time_sow_sec(time);
    RtklibIonosphereSystem ionosphere_system{};
    if (!ionosphere_system_for_signal(signal_id, &ionosphere_system)) {
        set_error(error_message, "cannot map signal to broadcast ionosphere model");
        return false;
    }

    RtklibIonosphereResult ionosphere{};
    if (!rtklib_broadcast_ionosphere_reference_delay(nav_store, ionosphere_system, time.gps_week, sow_sec,
                                                     receiver_ecef_m, azimuth_rad, elevation_rad, &ionosphere,
                                                     error_message)) {
        return false;
    }
    result.ionosphere_status = convert_status(ionosphere.status);
    if (ionosphere.status == RtklibIonosphereStatus::kApplied) {
        const SignalDefinition* definition = find_signal_definition(signal_id);
        double signal_frequency_hz = 0.0;
        if (definition == nullptr ||
            !signal_carrier_frequency_hz(*definition, glonass_fcn, &signal_frequency_hz) ||
            !std::isfinite(signal_frequency_hz) || signal_frequency_hz <= 0.0 ||
            ionosphere.reference_frequency_hz <= 0.0) {
            set_error(error_message, "cannot determine signal frequency for ionosphere scaling");
            return false;
        }
        const double frequency_ratio = ionosphere.reference_frequency_hz / signal_frequency_hz;
        result.ionosphere_code_delay_m = ionosphere.reference_delay_m * frequency_ratio * frequency_ratio;
    }

    if (!rtklib_troposphere_delay(time.gps_week, sow_sec, receiver_ecef_m, azimuth_rad, elevation_rad,
                                  &result.troposphere_delay_m, error_message)) {
        return false;
    }
    *correction = result;
    return true;
}

const char* ionosphere_correction_status_name(IonosphereCorrectionStatus status) {
    switch (status) {
        case IonosphereCorrectionStatus::kDisabled:
            return "disabled";
        case IonosphereCorrectionStatus::kApplied:
            return "applied";
        case IonosphereCorrectionStatus::kMissingParameters:
            return "missing_parameters";
        case IonosphereCorrectionStatus::kUnsupportedBroadcastModel:
            return "unsupported_broadcast_model";
    }
    return "unknown";
}

} // namespace gnss_sim
