#include "model/measurement_model.h"

#include "gnss_sim/sim_time.h"

#include <cmath>
#include <cstdint>

namespace gnss_sim {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kGpsL1Hz = 1575.42e6;
constexpr double kGalileoE1Hz = 1575.42e6;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool finite_measurement_input(const SatelliteGeometry& geometry, const AtmosphereCorrection& atmosphere) {
    return geometry.satellite_number > 0 && std::isfinite(geometry.geometric_range_m) &&
           geometry.geometric_range_m > 0.0 && std::isfinite(geometry.range_rate_mps) &&
           std::isfinite(geometry.satellite_state.clock_bias_sec) &&
           std::isfinite(geometry.satellite_state.clock_drift_sec_per_sec) &&
           std::isfinite(atmosphere.ionosphere_code_delay_m) && std::isfinite(atmosphere.troposphere_delay_m);
}

bool family_is_gps_modern(RtklibBroadcastMessageFamily family) {
    return family == RtklibBroadcastMessageFamily::kCnav || family == RtklibBroadcastMessageFamily::kCnav2;
}

bool family_is_beidou_modern(RtklibBroadcastMessageFamily family) {
    return family == RtklibBroadcastMessageFamily::kBeidouBcnav1 ||
           family == RtklibBroadcastMessageFamily::kBeidouBcnav2 ||
           family == RtklibBroadcastMessageFamily::kBeidouBcnav3;
}

RtklibBroadcastMessageFamily requested_bias_family(NavMessageFamily family) {
    switch (family) {
        case NavMessageFamily::kGpsLnav:
        case NavMessageFamily::kQzssLnav:
        case NavMessageFamily::kBeidouD1D2:
            return RtklibBroadcastMessageFamily::kLegacy;
        case NavMessageFamily::kGpsCnav:
        case NavMessageFamily::kQzssCnav:
            return RtklibBroadcastMessageFamily::kCnav;
        case NavMessageFamily::kGpsCnav2:
        case NavMessageFamily::kQzssCnav2:
            return RtklibBroadcastMessageFamily::kCnav2;
        case NavMessageFamily::kGlonassFdma:
            return RtklibBroadcastMessageFamily::kGlonassFdma;
        case NavMessageFamily::kGlonassL3Oc:
            return RtklibBroadcastMessageFamily::kGlonassL3Oc;
        case NavMessageFamily::kGalileoInav:
            return RtklibBroadcastMessageFamily::kGalileoInav;
        case NavMessageFamily::kGalileoFnav:
            return RtklibBroadcastMessageFamily::kGalileoFnav;
        case NavMessageFamily::kBeidouBcnav1:
            return RtklibBroadcastMessageFamily::kBeidouBcnav1;
        case NavMessageFamily::kBeidouBcnav2:
            return RtklibBroadcastMessageFamily::kBeidouBcnav2;
        case NavMessageFamily::kBeidouBcnav3:
            return RtklibBroadcastMessageFamily::kBeidouBcnav3;
        case NavMessageFamily::kGalileoCnav:
            return RtklibBroadcastMessageFamily::kUnknown;
    }
    return RtklibBroadcastMessageFamily::kUnknown;
}

double frequency_ratio_squared(double reference_frequency_hz, double signal_frequency_hz) {
    const double ratio = reference_frequency_hz / signal_frequency_hz;
    return ratio * ratio;
}

void set_bias(double seconds, double* code_bias_m, BroadcastCodeBiasStatus* status) {
    *code_bias_m = kSpeedOfLightMps * seconds;
    *status = BroadcastCodeBiasStatus::kApplied;
}

void set_no_bias(double* code_bias_m, BroadcastCodeBiasStatus* status) {
    *code_bias_m = 0.0;
    *status = BroadcastCodeBiasStatus::kNoCorrection;
}

void set_unavailable(double* code_bias_m, BroadcastCodeBiasStatus* status) {
    *code_bias_m = 0.0;
    *status = BroadcastCodeBiasStatus::kUnavailableForMessageFamily;
}

std::uint64_t mix64(std::uint64_t value) {
    value ^= value >> 30U;
    value *= UINT64_C(0xbf58476d1ce4e5b9);
    value ^= value >> 27U;
    value *= UINT64_C(0x94d049bb133111eb);
    value ^= value >> 31U;
    return value;
}

std::int64_t deterministic_ambiguity_cycles(SignalId signal_id, int satellite_number, const SimTime& tracking_start) {
    std::uint64_t value = static_cast<std::uint64_t>(satellite_number);
    value ^= static_cast<std::uint64_t>(static_cast<int>(signal_id) + 1) << 16U;
    value ^= static_cast<std::uint64_t>(tracking_start.gps_week) << 32U;
    value ^= static_cast<std::uint64_t>(tracking_start.tow_ns);
    return static_cast<std::int64_t>(100000U + mix64(value) % 900000U);
}

bool ambiguity_matches_tracker(const CarrierAmbiguityState& state, const SignalTracker& tracker, int satellite_number) {
    return state.initialized && state.signal_id == tracker.signal_id && state.satellite_number == satellite_number &&
           compare_sim_time(state.tracking_start_time, tracker.tracking_start_time) == 0;
}

struct SignalStateProviderContext {
    const RtklibNavStore* nav_store;
    int observation_code;
    RtklibBroadcastMessageFamily message_family;
};

bool signal_state_provider(const void* context, int gps_week, double sow_sec, int satellite_number,
                           RtklibSatelliteState* state, std::string* error_message) {
    const SignalStateProviderContext* provider = static_cast<const SignalStateProviderContext*>(context);
    if (provider == nullptr || provider->nav_store == nullptr || provider->observation_code <= 0) {
        set_error(error_message, "signal-specific geometry provider is invalid");
        return false;
    }
    return get_rtklib_signal_satellite_state(provider->nav_store, gps_week, sow_sec, satellite_number,
                                             provider->observation_code, provider->message_family, state,
                                             error_message);
}

bool atmosphere_line_of_sight_changed(const SatelliteGeometry& original, const SatelliteGeometry& refined) {
    constexpr double kAngleToleranceRad = 1.0e-14;
    return std::fabs(original.azimuth_rad - refined.azimuth_rad) > kAngleToleranceRad ||
           std::fabs(original.elevation_rad - refined.elevation_rad) > kAngleToleranceRad;
}

} // namespace

void reset_carrier_ambiguity_state(CarrierAmbiguityState* state) {
    if (state != nullptr) {
        *state = {};
    }
}

bool compute_broadcast_code_bias_m(const SignalDefinition& signal, const RtklibBroadcastBiasData& bias_data,
                                   double* code_bias_m, BroadcastCodeBiasStatus* status, std::string* error_message) {
    if (code_bias_m == nullptr || status == nullptr) {
        set_error(error_message, "code-bias output arguments are invalid");
        return false;
    }

    double signal_frequency_hz = 0.0;
    if (!signal_carrier_frequency_hz(signal, bias_data.glonass_fcn, &signal_frequency_hz) ||
        !std::isfinite(signal_frequency_hz) || signal_frequency_hz <= 0.0) {
        set_error(error_message, "cannot determine carrier frequency for code-bias model");
        return false;
    }

    switch (signal.code_bias_model) {
        case CodeBiasModel::kGpsL1Ca:
        case CodeBiasModel::kQzssL1Ca:
            if (family_is_gps_modern(bias_data.message_family)) {
                set_bias(bias_data.tgd_sec[0] - bias_data.isc_sec[0], code_bias_m, status);
            } else {
                set_bias(bias_data.tgd_sec[0], code_bias_m, status);
            }
            return true;
        case CodeBiasModel::kGpsL1C:
        case CodeBiasModel::kQzssL1C:
            if (bias_data.message_family != RtklibBroadcastMessageFamily::kCnav2) {
                set_unavailable(code_bias_m, status);
            } else {
                set_bias(bias_data.tgd_sec[0] - bias_data.isc_sec[5], code_bias_m, status);
            }
            return true;
        case CodeBiasModel::kGpsL2P:
            set_bias(frequency_ratio_squared(kGpsL1Hz, signal_frequency_hz) * bias_data.tgd_sec[0], code_bias_m,
                     status);
            return true;
        case CodeBiasModel::kGpsL2C:
        case CodeBiasModel::kQzssL2C:
            if (!family_is_gps_modern(bias_data.message_family)) {
                set_unavailable(code_bias_m, status);
            } else {
                set_bias(bias_data.tgd_sec[0] - bias_data.isc_sec[1], code_bias_m, status);
            }
            return true;
        case CodeBiasModel::kGpsL5Q:
        case CodeBiasModel::kQzssL5Q:
            if (!family_is_gps_modern(bias_data.message_family)) {
                set_unavailable(code_bias_m, status);
            } else {
                set_bias(bias_data.tgd_sec[0] - bias_data.isc_sec[3], code_bias_m, status);
            }
            return true;
        case CodeBiasModel::kGlonassG1:
            set_no_bias(code_bias_m, status);
            return true;
        case CodeBiasModel::kGlonassG2:
            set_bias(bias_data.glonass_dtaun_sec, code_bias_m, status);
            return true;
        case CodeBiasModel::kGlonassG3:
            if (bias_data.message_family != RtklibBroadcastMessageFamily::kGlonassL3Oc) {
                set_unavailable(code_bias_m, status);
            } else {
                set_bias(-bias_data.glonass_isc_l3ocp_sec, code_bias_m, status);
            }
            return true;
        case CodeBiasModel::kGalileoE1:
            if (bias_data.message_family == RtklibBroadcastMessageFamily::kGalileoFnav) {
                set_bias(bias_data.tgd_sec[0], code_bias_m, status);
            } else if (bias_data.message_family == RtklibBroadcastMessageFamily::kGalileoInav) {
                set_bias(bias_data.tgd_sec[1], code_bias_m, status);
            } else {
                set_unavailable(code_bias_m, status);
            }
            return true;
        case CodeBiasModel::kGalileoE5A:
            if (bias_data.message_family != RtklibBroadcastMessageFamily::kGalileoFnav) {
                set_unavailable(code_bias_m, status);
            } else {
                set_bias(frequency_ratio_squared(kGalileoE1Hz, signal_frequency_hz) * bias_data.tgd_sec[0], code_bias_m,
                         status);
            }
            return true;
        case CodeBiasModel::kGalileoE5B:
            if (bias_data.message_family != RtklibBroadcastMessageFamily::kGalileoInav) {
                set_unavailable(code_bias_m, status);
            } else {
                set_bias(frequency_ratio_squared(kGalileoE1Hz, signal_frequency_hz) * bias_data.tgd_sec[1], code_bias_m,
                         status);
            }
            return true;
        case CodeBiasModel::kGalileoE6:
            set_unavailable(code_bias_m, status);
            return true;
        case CodeBiasModel::kBeidouB1I:
            if (family_is_beidou_modern(bias_data.message_family)) {
                set_unavailable(code_bias_m, status);
            } else {
                set_bias(bias_data.tgd_sec[0], code_bias_m, status);
            }
            return true;
        case CodeBiasModel::kBeidouB3I:
            set_no_bias(code_bias_m, status);
            return true;
        case CodeBiasModel::kBeidouB1C:
            if (bias_data.message_family != RtklibBroadcastMessageFamily::kBeidouBcnav1 &&
                bias_data.message_family != RtklibBroadcastMessageFamily::kBeidouBcnav2) {
                set_unavailable(code_bias_m, status);
            } else {
                set_bias(bias_data.tgd_sec[0], code_bias_m, status);
            }
            return true;
        case CodeBiasModel::kBeidouB2A:
            if (bias_data.message_family != RtklibBroadcastMessageFamily::kBeidouBcnav1 &&
                bias_data.message_family != RtklibBroadcastMessageFamily::kBeidouBcnav2) {
                set_unavailable(code_bias_m, status);
            } else {
                set_bias(bias_data.tgd_sec[1], code_bias_m, status);
            }
            return true;
        case CodeBiasModel::kBeidouB2B:
            if (bias_data.message_family != RtklibBroadcastMessageFamily::kBeidouBcnav3) {
                set_unavailable(code_bias_m, status);
            } else {
                set_bias(bias_data.tgd_sec[0], code_bias_m, status);
            }
            return true;
    }

    set_error(error_message, "unknown signal code-bias model");
    return false;
}

namespace {

struct CodeModelSelection {
    RtklibBroadcastBiasData bias_data;
    double code_bias_m;
    BroadcastCodeBiasStatus code_bias_status;
};

bool finish_zero_noise_measurement(const SignalDefinition& signal, const SatelliteGeometry& geometry,
                                   const ReceiverTruth& receiver, const SignalTracker& tracker,
                                   const AtmosphereCorrection& atmosphere, const CodeModelSelection& code_model,
                                   CarrierAmbiguityState* ambiguity_state, MeasurementObservation* observation,
                                   std::string* error_message) {
    double wavelength_m = 0.0;
    if (!signal_wavelength_m(signal, code_model.bias_data.glonass_fcn, &wavelength_m) || !std::isfinite(wavelength_m) ||
        wavelength_m <= 0.0) {
        set_error(error_message, "cannot determine signal wavelength");
        return false;
    }

    MeasurementObservation result{};
    result.signal_id = tracker.signal_id;
    result.satellite_number = geometry.satellite_number;
    result.glonass_fcn = code_model.bias_data.glonass_fcn;
    result.wavelength_m = wavelength_m;
    result.geometric_range_m = geometry.geometric_range_m;
    result.range_rate_mps = geometry.range_rate_mps;
    result.satellite_clock_bias_m = kSpeedOfLightMps * geometry.satellite_state.clock_bias_sec;
    result.satellite_clock_drift_mps = kSpeedOfLightMps * geometry.satellite_state.clock_drift_sec_per_sec;
    result.broadcast_message_family = code_model.bias_data.message_family;
    for (int index = 0; index < 4; ++index) {
        result.tgd_sec[index] = code_model.bias_data.tgd_sec[index];
    }
    for (int index = 0; index < 6; ++index) {
        result.isc_sec[index] = code_model.bias_data.isc_sec[index];
    }
    result.glonass_dtaun_sec = code_model.bias_data.glonass_dtaun_sec;
    result.code_bias_m = code_model.code_bias_m;
    result.code_bias_status = code_model.code_bias_status;
    result.ionosphere_code_delay_m = atmosphere.ionosphere_code_delay_m;
    result.troposphere_delay_m = atmosphere.troposphere_delay_m;
    result.cn0_dbhz = tracker.cn0_dbhz;
    result.lock_time_ns = tracker.lock_time_ns;

    const double clock_corrected_range_m = result.geometric_range_m - result.satellite_clock_bias_m;
    result.pseudorange_m =
        clock_corrected_range_m + result.code_bias_m + result.ionosphere_code_delay_m + result.troposphere_delay_m;
    result.doppler_hz = -(result.range_rate_mps - result.satellite_clock_drift_mps) / wavelength_m;

    if (tracker.phase == SignalTrackingPhase::kTracking) {
        if (!ambiguity_matches_tracker(*ambiguity_state, tracker, geometry.satellite_number)) {
            ambiguity_state->signal_id = tracker.signal_id;
            ambiguity_state->satellite_number = geometry.satellite_number;
            ambiguity_state->tracking_start_time = tracker.tracking_start_time;
            ambiguity_state->ambiguity_cycles = deterministic_ambiguity_cycles(
                tracker.signal_id, geometry.satellite_number, tracker.tracking_start_time);
            ambiguity_state->initialized = true;
        }
        result.ambiguity_cycles = ambiguity_state->ambiguity_cycles;
        const double carrier_range_m =
            clock_corrected_range_m - result.ionosphere_code_delay_m + result.troposphere_delay_m;
        result.adr_cycles = carrier_range_m / wavelength_m + static_cast<double>(result.ambiguity_cycles);
    } else {
        reset_carrier_ambiguity_state(ambiguity_state);
        result.ambiguity_cycles = 0;
        result.adr_cycles = 0.0;
    }

    const bool measurement_geometry_usable = geometry.above_elevation_mask;
    const bool code_bias_available = result.code_bias_status != BroadcastCodeBiasStatus::kUnavailableForMessageFamily;
    result.observation_available = measurement_geometry_usable && tracker.observation_available;
    result.pseudorange_valid = measurement_geometry_usable && tracker.psr_valid && code_bias_available;
    result.doppler_valid = measurement_geometry_usable && tracker.doppler_valid;
    result.adr_valid = measurement_geometry_usable && tracker.adr_valid && ambiguity_state->initialized;

    static_cast<void>(receiver);
    *observation = result;
    return true;
}

} // namespace

bool generate_zero_noise_measurement(const RtklibNavStore* nav_store, SatelliteGeometry& geometry,
                                     const ReceiverTruth& receiver, const SignalTracker& tracker,
                                     const AtmosphereCorrection& atmosphere, CarrierAmbiguityState* ambiguity_state,
                                     MeasurementObservation* observation, std::string* error_message) {
    const SignalDefinition* signal = find_signal_definition(tracker.signal_id);
    if (nav_store == nullptr || ambiguity_state == nullptr || observation == nullptr || signal == nullptr ||
        !finite_measurement_input(geometry, atmosphere) || atmosphere.mode == AtmosphereMode::UNSPECIFIED) {
        set_error(error_message, "zero-noise measurement request has invalid arguments");
        return false;
    }

    const RtklibBroadcastMessageFamily requested_family = requested_bias_family(signal->nav_message_family);
    RtklibBroadcastBiasData bias_data{};
    const bool signal_family_bias_available = rtklib_broadcast_bias_data_for_family(
        nav_store, geometry.transmit_gps_week, geometry.transmit_sow_sec, geometry.satellite_number, requested_family,
        &bias_data, nullptr);
    if (!signal_family_bias_available &&
        !rtklib_broadcast_bias_data(nav_store, geometry.transmit_gps_week, geometry.transmit_sow_sec,
                                    geometry.satellite_number, &bias_data, error_message)) {
        return false;
    }

    CodeModelSelection code_model{};
    code_model.bias_data = bias_data;
    if (!signal_family_bias_available) {
        set_unavailable(&code_model.code_bias_m, &code_model.code_bias_status);
    } else if (!compute_broadcast_code_bias_m(*signal, bias_data, &code_model.code_bias_m, &code_model.code_bias_status,
                                              error_message)) {
        return false;
    }

    AtmosphereCorrection final_atmosphere = atmosphere;
    const bool family_code_bias_available =
        signal_family_bias_available &&
        code_model.code_bias_status != BroadcastCodeBiasStatus::kUnavailableForMessageFamily;
    if (family_code_bias_available) {
        int observation_code = 0;
        int frequency_index = 0;
        if (!signal_rtklib_observation_code(*signal, &observation_code, &frequency_index)) {
            set_error(error_message, "cannot map signal to RTKLIB observation code for final geometry");
            return false;
        }
        static_cast<void>(frequency_index);

        const SignalStateProviderContext provider_context{nav_store, observation_code, bias_data.message_family};
        SatelliteGeometry final_geometry{};
        if (!compute_satellite_geometry_with_provider(signal_state_provider, &provider_context, receiver,
                                                      geometry.receive_time, geometry.satellite_number,
                                                      geometry.elevation_mask_deg, &final_geometry, error_message)) {
            return false;
        }

        RtklibBroadcastBiasData final_bias_data{};
        if (!rtklib_broadcast_bias_data_for_family(nav_store, final_geometry.transmit_gps_week,
                                                   final_geometry.transmit_sow_sec, final_geometry.satellite_number,
                                                   requested_family, &final_bias_data, error_message) ||
            !compute_broadcast_code_bias_m(*signal, final_bias_data, &code_model.code_bias_m,
                                           &code_model.code_bias_status, error_message)) {
            return false;
        }
        code_model.bias_data = final_bias_data;

        if (atmosphere_line_of_sight_changed(geometry, final_geometry) &&
            !compute_atmosphere_correction(atmosphere.mode, nav_store, final_geometry.receive_time, tracker.signal_id,
                                           final_bias_data.glonass_fcn, receiver.position_ecef_m,
                                           final_geometry.azimuth_rad, final_geometry.elevation_rad, &final_atmosphere,
                                           error_message)) {
            return false;
        }
        geometry = final_geometry;
    }

    return finish_zero_noise_measurement(*signal, geometry, receiver, tracker, final_atmosphere, code_model,
                                         ambiguity_state, observation, error_message);
}

bool generate_zero_noise_measurement_with_explicit_code_bias(
    const SatelliteGeometry& geometry, const ReceiverTruth& receiver, const SignalTracker& tracker,
    const AtmosphereCorrection& atmosphere, double code_bias_m, CarrierAmbiguityState* ambiguity_state,
    MeasurementObservation* observation, std::string* error_message) {
    const SignalDefinition* signal = find_signal_definition(tracker.signal_id);
    if (ambiguity_state == nullptr || observation == nullptr || signal == nullptr || !std::isfinite(code_bias_m) ||
        !finite_measurement_input(geometry, atmosphere) || atmosphere.mode == AtmosphereMode::UNSPECIFIED) {
        set_error(error_message, "explicit-code zero-noise measurement request has invalid arguments");
        return false;
    }

    CodeModelSelection code_model{};
    code_model.bias_data.message_family = RtklibBroadcastMessageFamily::kUnknown;
    code_model.code_bias_m = code_bias_m;
    code_model.code_bias_status = BroadcastCodeBiasStatus::kApplied;
    return finish_zero_noise_measurement(*signal, geometry, receiver, tracker, atmosphere, code_model, ambiguity_state,
                                         observation, error_message);
}

const char* broadcast_code_bias_status_name(BroadcastCodeBiasStatus status) {
    switch (status) {
        case BroadcastCodeBiasStatus::kApplied:
            return "APPLIED";
        case BroadcastCodeBiasStatus::kNoCorrection:
            return "NO_CORRECTION";
        case BroadcastCodeBiasStatus::kUnavailableForMessageFamily:
            return "UNAVAILABLE_FOR_MESSAGE_FAMILY";
    }
    return "UNKNOWN";
}

} // namespace gnss_sim
