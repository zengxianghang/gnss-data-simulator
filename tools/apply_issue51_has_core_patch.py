from pathlib import Path
import re


def replace_once(path, old, new):
    text = Path(path).read_text()
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one exact match, got {count}")
    Path(path).write_text(text.replace(old, new, 1))


def regex_once(path, pattern, replacement):
    text = Path(path).read_text()
    new_text, count = re.subn(pattern, replacement, text, count=1, flags=re.S)
    if count != 1:
        raise RuntimeError(f"{path}: expected one regex match, got {count}")
    Path(path).write_text(new_text)


# 1. Satellite geometry: make the propagation-time solver reusable with an
# arbitrary satellite-state provider. The existing RTKLIB-nav entry point wraps
# this provider, so broadcast behavior remains unchanged.
replace_once(
    "src/gnss/satellite_engine.h",
    "};\n\nbool subtract_propagation_time",
    "};\n\nusing SatelliteStateProvider = bool (*)(const void* context, int gps_week, double sow_sec, int satellite_number,\n"
    "                                        RtklibSatelliteState* state, std::string* error_message);\n\n"
    "bool compute_satellite_geometry_with_provider(SatelliteStateProvider state_provider, const void* state_context,\n"
    "                                              const ReceiverTruth& receiver, const SimTime& receive_time,\n"
    "                                              int satellite_number, double elevation_mask_deg,\n"
    "                                              SatelliteGeometry* geometry, std::string* error_message);\n\n"
    "bool subtract_propagation_time",
)

replace_once(
    "src/gnss/satellite_engine.cpp",
    "    *range_rate_mps = rate_mps;\n    return std::isfinite(rate_mps);\n}\n\n} // namespace",
    "    *range_rate_mps = rate_mps;\n    return std::isfinite(rate_mps);\n}\n\n"
    "bool rtklib_state_provider(const void* context, int gps_week, double sow_sec, int satellite_number,\n"
    "                           RtklibSatelliteState* state, std::string* error_message) {\n"
    "    return get_rtklib_satellite_state(static_cast<const RtklibNavStore*>(context), gps_week, sow_sec,\n"
    "                                      satellite_number, state, error_message);\n"
    "}\n\n} // namespace",
)

replace_once(
    "src/gnss/satellite_engine.cpp",
    "bool compute_satellite_geometry(const RtklibNavStore* nav_store, const ReceiverTruth& receiver,\n"
    "                                const SimTime& receive_time, int satellite_number, double elevation_mask_deg,\n"
    "                                SatelliteGeometry* geometry, std::string* error_message) {\n"
    "    if (nav_store == nullptr || geometry == nullptr || satellite_number <= 0 || receive_time.gps_week < 0 ||",
    "bool compute_satellite_geometry_with_provider(SatelliteStateProvider state_provider, const void* state_context,\n"
    "                                              const ReceiverTruth& receiver, const SimTime& receive_time,\n"
    "                                              int satellite_number, double elevation_mask_deg,\n"
    "                                              SatelliteGeometry* geometry, std::string* error_message) {\n"
    "    if (state_provider == nullptr || state_context == nullptr || geometry == nullptr || satellite_number <= 0 ||\n"
    "        receive_time.gps_week < 0 ||",
)

text = Path("src/gnss/satellite_engine.cpp").read_text()
count = text.count("get_rtklib_satellite_state(nav_store,")
if count != 2:
    raise RuntimeError(f"satellite_engine.cpp: expected two state calls, got {count}")
text = text.replace("get_rtklib_satellite_state(nav_store,", "state_provider(state_context,")
Path("src/gnss/satellite_engine.cpp").write_text(text)

replace_once(
    "src/gnss/satellite_engine.cpp",
    "    *geometry = result;\n    return true;\n}\n\n} // namespace gnss_sim",
    "    *geometry = result;\n    return true;\n}\n\n"
    "bool compute_satellite_geometry(const RtklibNavStore* nav_store, const ReceiverTruth& receiver,\n"
    "                                const SimTime& receive_time, int satellite_number, double elevation_mask_deg,\n"
    "                                SatelliteGeometry* geometry, std::string* error_message) {\n"
    "    if (nav_store == nullptr) {\n"
    "        set_error(error_message, \"satellite-geometry navigation store is null\");\n"
    "        return false;\n"
    "    }\n"
    "    return compute_satellite_geometry_with_provider(rtklib_state_provider, nav_store, receiver, receive_time,\n"
    "                                                    satellite_number, elevation_mask_deg, geometry, error_message);\n"
    "}\n\n} // namespace gnss_sim",
)

# 2. Measurement model: add a narrow explicit-code-bias path. Shared finishing
# logic ensures pseudorange, Doppler, ADR, validity and metadata stay identical
# to the broadcast path except for the externally supplied code bias/state.
replace_once(
    "src/model/measurement_model.h",
    "bool generate_zero_noise_measurement(const RtklibNavStore* nav_store, const SatelliteGeometry& geometry,\n"
    "                                     const ReceiverTruth& receiver, const SignalTracker& tracker,\n"
    "                                     const AtmosphereCorrection& atmosphere, CarrierAmbiguityState* ambiguity_state,\n"
    "                                     MeasurementObservation* observation, std::string* error_message);",
    "bool generate_zero_noise_measurement(const RtklibNavStore* nav_store, const SatelliteGeometry& geometry,\n"
    "                                     const ReceiverTruth& receiver, const SignalTracker& tracker,\n"
    "                                     const AtmosphereCorrection& atmosphere, CarrierAmbiguityState* ambiguity_state,\n"
    "                                     MeasurementObservation* observation, std::string* error_message);\n\n"
    "bool generate_zero_noise_measurement_with_explicit_code_bias(\n"
    "    const SatelliteGeometry& geometry, const ReceiverTruth& receiver, const SignalTracker& tracker,\n"
    "    const AtmosphereCorrection& atmosphere, double code_bias_m, CarrierAmbiguityState* ambiguity_state,\n"
    "    MeasurementObservation* observation, std::string* error_message);",
)

new_measurement_block = r'''namespace {

struct CodeModelSelection {
    RtklibSatelliteState satellite_state;
    double geometric_range_m;
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
    if (!signal_wavelength_m(signal, code_model.bias_data.glonass_fcn, &wavelength_m) ||
        !std::isfinite(wavelength_m) || wavelength_m <= 0.0) {
        set_error(error_message, "cannot determine signal wavelength");
        return false;
    }

    MeasurementObservation result{};
    result.signal_id = tracker.signal_id;
    result.satellite_number = geometry.satellite_number;
    result.glonass_fcn = code_model.bias_data.glonass_fcn;
    result.wavelength_m = wavelength_m;
    result.geometric_range_m = code_model.geometric_range_m;
    result.range_rate_mps = geometry.range_rate_mps;
    result.satellite_clock_bias_m = kSpeedOfLightMps * code_model.satellite_state.clock_bias_sec;
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
            ambiguity_state->ambiguity_cycles =
                deterministic_ambiguity_cycles(tracker.signal_id, geometry.satellite_number, tracker.tracking_start_time);
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

    *observation = result;
    return true;
}

} // namespace

bool generate_zero_noise_measurement(const RtklibNavStore* nav_store, const SatelliteGeometry& geometry,
                                     const ReceiverTruth& receiver, const SignalTracker& tracker,
                                     const AtmosphereCorrection& atmosphere, CarrierAmbiguityState* ambiguity_state,
                                     MeasurementObservation* observation, std::string* error_message) {
    const SignalDefinition* signal = find_signal_definition(tracker.signal_id);
    if (nav_store == nullptr || ambiguity_state == nullptr || observation == nullptr || signal == nullptr ||
        !finite_measurement_input(geometry, atmosphere) || atmosphere.mode == AtmosphereMode::UNSPECIFIED) {
        set_error(error_message, "zero-noise measurement request has invalid arguments");
        return false;
    }

    RtklibBroadcastBiasData bias_data{};
    const bool signal_family_bias_available = rtklib_broadcast_bias_data_for_family(
        nav_store, geometry.transmit_gps_week, geometry.transmit_sow_sec, geometry.satellite_number,
        requested_bias_family(signal->nav_message_family), &bias_data, nullptr);
    if (!signal_family_bias_available &&
        !rtklib_broadcast_bias_data(nav_store, geometry.transmit_gps_week, geometry.transmit_sow_sec,
                                    geometry.satellite_number, &bias_data, error_message)) {
        return false;
    }

    CodeModelSelection code_model{};
    code_model.satellite_state = geometry.satellite_state;
    code_model.geometric_range_m = geometry.geometric_range_m;
    code_model.bias_data = bias_data;
    if (!signal_family_bias_available) {
        set_unavailable(&code_model.code_bias_m, &code_model.code_bias_status);
    } else if (!compute_broadcast_code_bias_m(*signal, bias_data, &code_model.code_bias_m,
                                              &code_model.code_bias_status, error_message)) {
        return false;
    }

    const bool family_code_bias_available =
        signal_family_bias_available &&
        code_model.code_bias_status != BroadcastCodeBiasStatus::kUnavailableForMessageFamily;
    if (family_code_bias_available) {
        int observation_code = 0;
        int frequency_index = 0;
        double code_line_of_sight_ecef[3]{};
        if (!signal_rtklib_observation_code(*signal, &observation_code, &frequency_index) ||
            !get_rtklib_signal_satellite_state(nav_store, geometry.transmit_gps_week, geometry.transmit_sow_sec,
                                               geometry.satellite_number, observation_code, bias_data.message_family,
                                               &code_model.satellite_state, error_message) ||
            !rtklib_geometric_distance(code_model.satellite_state.position_ecef_m, receiver.position_ecef_m,
                                       &code_model.geometric_range_m, code_line_of_sight_ecef)) {
            return false;
        }
    }

    return finish_zero_noise_measurement(*signal, geometry, receiver, tracker, atmosphere, code_model, ambiguity_state,
                                         observation, error_message);
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
    code_model.satellite_state = geometry.satellite_state;
    code_model.geometric_range_m = geometry.geometric_range_m;
    code_model.bias_data.message_family = RtklibBroadcastMessageFamily::kUnknown;
    code_model.code_bias_m = code_bias_m;
    code_model.code_bias_status = BroadcastCodeBiasStatus::kApplied;
    return finish_zero_noise_measurement(*signal, geometry, receiver, tracker, atmosphere, code_model, ambiguity_state,
                                         observation, error_message);
}

'''

regex_once(
    "src/model/measurement_model.cpp",
    r"bool generate_zero_noise_measurement\(const RtklibNavStore\* nav_store,.*?\n}\n\n(?=const char\* broadcast_code_bias_status_name)",
    new_measurement_block,
)

# 3. Unit-test the new explicit-bias path on a real Galileo E01 broadcast
# geometry. The state source is irrelevant to this unit test; the test proves
# that an external observable-specific bias is applied with the intended sign
# and makes E6 pseudorange valid without reinterpreting an INAV/FNAV BGD.
replace_once(
    "tests/unit/test_measurement_model.cpp",
    "\n} // namespace\n",
    r'''

TEST(ZeroNoiseMeasurement, GalileoE6ExplicitCodeBiasUsesExternalObservableBiasWithoutBroadcastBgd) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, mixed_nav_path().c_str(), &error_message));
    gnss_sim::ReceiverTruth receiver{};
    ASSERT_TRUE(make_test_receiver(&receiver, &error_message));
    gnss_sim::SimTime receive_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 172900.0, &receive_time));
    int satellite_number = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("E01", &satellite_number));
    gnss_sim::SatelliteGeometry geometry{};
    ASSERT_TRUE(gnss_sim::compute_satellite_geometry(nav.store, receiver, receive_time, satellite_number, -90.0,
                                                     &geometry, &error_message));

    gnss_sim::SignalTracker tracker = tracking_tracker(gnss_sim::SignalId::kGalileoE6, receive_time);
    gnss_sim::AtmosphereCorrection atmosphere{};
    atmosphere.mode = gnss_sim::AtmosphereMode::NONE;
    gnss_sim::CarrierAmbiguityState ambiguity{};
    gnss_sim::MeasurementObservation observation{};
    constexpr double kExternalC6cOsbM = -0.4;
    ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement_with_explicit_code_bias(
        geometry, receiver, tracker, atmosphere, kExternalC6cOsbM, &ambiguity, &observation, &error_message));
    EXPECT_EQ(observation.code_bias_status, gnss_sim::BroadcastCodeBiasStatus::kApplied);
    EXPECT_EQ(observation.broadcast_message_family, gnss_sim::RtklibBroadcastMessageFamily::kUnknown);
    EXPECT_DOUBLE_EQ(observation.code_bias_m, kExternalC6cOsbM);
    EXPECT_TRUE(observation.pseudorange_valid);
    EXPECT_NEAR(observation.pseudorange_m,
                geometry.geometric_range_m - kSpeedOfLightMps * geometry.satellite_state.clock_bias_sec +
                    kExternalC6cOsbM,
                1.0e-9);
}

} // namespace
''',
)

print("issue51 HAS core patch applied")
