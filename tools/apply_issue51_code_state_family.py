from pathlib import Path


def replace_once(path, old, new):
    p = Path(path)
    text = p.read_text()
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one guarded match, found {count}")
    p.write_text(text.replace(old, new, 1))


replace_once(
    "src/gnss/rtklib_adapter.h",
    '''bool get_rtklib_satellite_state(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,
                                RtklibSatelliteState* state, std::string* error_message);
bool rtklib_broadcast_bias_data_for_family''',
    '''bool get_rtklib_satellite_state(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,
                                RtklibSatelliteState* state, std::string* error_message);
bool get_rtklib_signal_satellite_state(const RtklibNavStore* store, int gps_week, double sow_sec,
                                       int satellite_number, int observation_code,
                                       RtklibBroadcastMessageFamily requested_message_family,
                                       RtklibSatelliteState* state, std::string* error_message);
bool rtklib_broadcast_bias_data_for_family''',
)

replace_once(
    "src/gnss/rtklib_adapter.cpp",
    '''#include <rtklib.h>
#include <rtklib_obs_ext.h>
}''',
    '''#include <rtklib.h>
#include <rtklib_obs_ext.h>
#include <rtklib_signal_bias_ext.h>
}''',
)

replace_once(
    "src/gnss/rtklib_adapter.cpp",
    '''bool get_rtklib_satellite_state(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,
                                RtklibSatelliteState* state, std::string* error_message) {
    if (store == nullptr || state == nullptr || satellite_number <= 0 || !valid_gps_time(gps_week, sow_sec)) {
        set_error(error_message, "satellite-state request has invalid arguments");
        return false;
    }

    const gtime_t time = gpst2time(gps_week, sow_sec);
    double state_and_velocity[6]{};
    double clock[2]{};
    double variance_m2 = 0.0;
    int health = 0;
    if (satpos(time, time, satellite_number, EPHOPT_BRDC, &store->nav, state_and_velocity, clock, &variance_m2,
               &health) == 0) {
        set_error(error_message, "RTKLIB could not compute broadcast satellite state at requested GPST");
        return false;
    }

    for (int index = 0; index < 3; ++index) {
        state->position_ecef_m[index] = state_and_velocity[index];
        state->velocity_ecef_mps[index] = state_and_velocity[index + 3];
    }
    state->clock_bias_sec = clock[0];
    state->clock_drift_sec_per_sec = clock[1];
    state->variance_m2 = variance_m2;
    state->health = health;
    return true;
}
''',
    '''bool get_rtklib_satellite_state(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,
                                RtklibSatelliteState* state, std::string* error_message) {
    if (store == nullptr || state == nullptr || satellite_number <= 0 || !valid_gps_time(gps_week, sow_sec)) {
        set_error(error_message, "satellite-state request has invalid arguments");
        return false;
    }

    const gtime_t time = gpst2time(gps_week, sow_sec);
    double state_and_velocity[6]{};
    double clock[2]{};
    double variance_m2 = 0.0;
    int health = 0;
    if (satpos(time, time, satellite_number, EPHOPT_BRDC, &store->nav, state_and_velocity, clock, &variance_m2,
               &health) == 0) {
        set_error(error_message, "RTKLIB could not compute broadcast satellite state at requested GPST");
        return false;
    }

    for (int index = 0; index < 3; ++index) {
        state->position_ecef_m[index] = state_and_velocity[index];
        state->velocity_ecef_mps[index] = state_and_velocity[index + 3];
    }
    state->clock_bias_sec = clock[0];
    state->clock_drift_sec_per_sec = clock[1];
    state->variance_m2 = variance_m2;
    state->health = health;
    return true;
}

bool get_rtklib_signal_satellite_state(const RtklibNavStore* store, int gps_week, double sow_sec,
                                       int satellite_number, int observation_code,
                                       RtklibBroadcastMessageFamily requested_message_family,
                                       RtklibSatelliteState* state, std::string* error_message) {
    if (store == nullptr || state == nullptr || satellite_number <= 0 || observation_code <= 0 ||
        observation_code > 255 || !valid_gps_time(gps_week, sow_sec)) {
        set_error(error_message, "signal satellite-state request has invalid arguments");
        return false;
    }
    if (requested_message_family == RtklibBroadcastMessageFamily::kUnknown) {
        return get_rtklib_satellite_state(store, gps_week, sow_sec, satellite_number, state, error_message);
    }

    const int system = satsys(satellite_number, nullptr);
    int required_message_mask = 0;
    switch (requested_message_family) {
        case RtklibBroadcastMessageFamily::kLegacy:
            if (system == SYS_GPS || system == SYS_QZS) {
                required_message_mask = NAV_LNAV;
            } else if (system == SYS_CMP) {
                required_message_mask = NAV_D1 | NAV_D2 | NAV_D1D2;
            }
            break;
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
        case RtklibBroadcastMessageFamily::kUnknown:
            break;
    }
    if (required_message_mask == 0) {
        set_error(error_message, "unsupported signal NAV family for satellite state");
        return false;
    }

    const gtime_t time = gpst2time(gps_week, sow_sec);
    eph_t eph{};
    geph_t geph{};
    rtklib_signal_bias_info_ext_t info{};
    const int status = rtklib_signal_ephemeris_ext(time, satellite_number, static_cast<unsigned char>(observation_code),
                                                   required_message_mask, &store->nav, &eph, &geph, &info);
    if (status != 1) {
        set_error(error_message, "no compatible signal-family ephemeris for satellite state");
        return false;
    }

    double position[3]{};
    double next_position[3]{};
    double clock_bias_sec = 0.0;
    double next_clock_bias_sec = 0.0;
    double variance_m2 = 0.0;
    double next_variance_m2 = 0.0;
    constexpr double kDifferenceSec = 1.0e-3;
    if (info.system == SYS_GLO) {
        geph2pos(time, &geph, position, &clock_bias_sec, &variance_m2);
        geph2pos(timeadd(time, kDifferenceSec), &geph, next_position, &next_clock_bias_sec, &next_variance_m2);
        state->health = geph.svh;
    } else {
        eph2pos(time, &eph, position, &clock_bias_sec, &variance_m2);
        eph2pos(timeadd(time, kDifferenceSec), &eph, next_position, &next_clock_bias_sec, &next_variance_m2);
        state->health = eph.svh;
    }
    for (int index = 0; index < 3; ++index) {
        state->position_ecef_m[index] = position[index];
        state->velocity_ecef_mps[index] = (next_position[index] - position[index]) / kDifferenceSec;
    }
    state->clock_bias_sec = clock_bias_sec;
    state->clock_drift_sec_per_sec = (next_clock_bias_sec - clock_bias_sec) / kDifferenceSec;
    state->variance_m2 = variance_m2;
    return true;
}
''',
)

replace_once(
    "src/model/measurement_model.h",
    '''bool generate_zero_noise_measurement(const RtklibNavStore* nav_store, const SatelliteGeometry& geometry,
                                     const SignalTracker& tracker, const AtmosphereCorrection& atmosphere,''',
    '''bool generate_zero_noise_measurement(const RtklibNavStore* nav_store, const SatelliteGeometry& geometry,
                                     const ReceiverTruth& receiver, const SignalTracker& tracker,
                                     const AtmosphereCorrection& atmosphere,''',
)

replace_once(
    "src/model/measurement_model.cpp",
    '''bool generate_zero_noise_measurement(const RtklibNavStore* nav_store, const SatelliteGeometry& geometry,
                                     const SignalTracker& tracker, const AtmosphereCorrection& atmosphere,
                                     CarrierAmbiguityState* ambiguity_state, MeasurementObservation* observation,
                                     std::string* error_message) {''',
    '''bool generate_zero_noise_measurement(const RtklibNavStore* nav_store, const SatelliteGeometry& geometry,
                                     const ReceiverTruth& receiver, const SignalTracker& tracker,
                                     const AtmosphereCorrection& atmosphere, CarrierAmbiguityState* ambiguity_state,
                                     MeasurementObservation* observation, std::string* error_message) {''',
)

replace_once(
    "src/model/measurement_model.cpp",
    '''    double wavelength_m = 0.0;
    if (!signal_wavelength_m(*signal, bias_data.glonass_fcn, &wavelength_m) || !std::isfinite(wavelength_m) ||
        wavelength_m <= 0.0) {''',
    '''    RtklibSatelliteState code_state = geometry.satellite_state;
    double code_geometric_range_m = geometry.geometric_range_m;
    if (signal_family_bias_available) {
        int observation_code = 0;
        int frequency_index = 0;
        double code_line_of_sight_ecef[3]{};
        if (!signal_rtklib_observation_code(*signal, &observation_code, &frequency_index) ||
            !get_rtklib_signal_satellite_state(nav_store, geometry.transmit_gps_week, geometry.transmit_sow_sec,
                                               geometry.satellite_number, observation_code, bias_data.message_family,
                                               &code_state, error_message) ||
            !rtklib_geometric_distance(code_state.position_ecef_m, receiver.position_ecef_m,
                                       &code_geometric_range_m, code_line_of_sight_ecef)) {
            return false;
        }
    }

    double wavelength_m = 0.0;
    if (!signal_wavelength_m(*signal, bias_data.glonass_fcn, &wavelength_m) || !std::isfinite(wavelength_m) ||
        wavelength_m <= 0.0) {''',
)

replace_once(
    "src/model/measurement_model.cpp",
    '''    result.geometric_range_m = geometry.geometric_range_m;
    result.range_rate_mps = geometry.range_rate_mps;
    result.satellite_clock_bias_m = kSpeedOfLightMps * geometry.satellite_state.clock_bias_sec;
    result.satellite_clock_drift_mps = kSpeedOfLightMps * geometry.satellite_state.clock_drift_sec_per_sec;''',
    '''    // Code/carrier terms use the broadcast state from the same NAV family as
    // the code bias. Doppler deliberately remains on the generic stock satpos
    // state because it has no signal-specific group-delay dependency.
    result.geometric_range_m = code_geometric_range_m;
    result.range_rate_mps = geometry.range_rate_mps;
    result.satellite_clock_bias_m = kSpeedOfLightMps * code_state.clock_bias_sec;
    result.satellite_clock_drift_mps = kSpeedOfLightMps * geometry.satellite_state.clock_drift_sec_per_sec;''',
)

replace_once(
    "src/core/simulator.cpp",
    '''if (!generate_zero_noise_measurement(truth_nav, geometry, signal.tracker, atmosphere, &signal.ambiguity,
                                                 &observation, error_message) ||''',
    '''if (!generate_zero_noise_measurement(truth_nav, geometry, runtime->receiver, signal.tracker, atmosphere,
                                                 &signal.ambiguity, &observation, error_message) ||''',
)

p = Path("tests/unit/test_measurement_model.cpp")
text = p.read_text()
old = "generate_zero_noise_measurement(nav.store, geometry, "
count = text.count(old)
if count != 4:
    raise RuntimeError(f"tests/unit/test_measurement_model.cpp: expected 4 geometry calls, found {count}")
text = text.replace(old, "generate_zero_noise_measurement(nav.store, geometry, receiver, ")
old = "generate_zero_noise_measurement(nav.store, g0, "
if text.count(old) != 1:
    raise RuntimeError("expected one g0 measurement call")
text = text.replace(old, "generate_zero_noise_measurement(nav.store, g0, receiver, ", 1)
old = "generate_zero_noise_measurement(nav.store, g1, "
count = text.count(old)
if count != 3:
    raise RuntimeError(f"expected three g1 measurement calls, found {count}")
text = text.replace(old, "generate_zero_noise_measurement(nav.store, g1, receiver, ")
p.write_text(text)

print("family-specific code state patch applied")
