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


# Optional HAS product triplet in the run API. Existing aggregate initializers
# leave these null and preserve all current behavior.
replace_once(
    "include/gnss_sim/simulator.h",
    "    const char* cn0_model_path;\n};",
    "    const char* cn0_model_path;\n"
    "    const char* galileo_has_sp3_path;\n"
    "    const char* galileo_has_clock_path;\n"
    "    const char* galileo_has_bias_path;\n};",
)

# Simulator wiring.
replace_once(
    "src/core/simulator.cpp",
    '#include "gnss/nav_message_scheduler.h"\n',
    '#include "gnss/galileo_has_adapter.h"\n#include "gnss/nav_message_scheduler.h"\n',
)
replace_once(
    "src/core/simulator.cpp",
    "#include <fstream>\n#include <string>",
    "#include <fstream>\n#include <memory>\n#include <string>",
)
replace_once(
    "src/core/simulator.cpp",
    "struct RuntimeState {\n    NavigationState* navigation;",
    "struct RuntimeState {\n    NavigationState* navigation;\n    const GalileoHasStore* galileo_has;",
)

replace_once(
    "src/core/simulator.cpp",
    "bool valid_run_options(const SimulatorRunOptions& options) {\n"
    "    return options.rinex_nav_path != nullptr && options.rinex_nav_path[0] != '\\0' &&\n"
    "           options.output_log_path != nullptr && options.output_log_path[0] != '\\0' &&\n"
    "           options.start_time.gps_week >= 0 && options.start_time.tow_ns >= 0 &&\n"
    "           options.start_time.tow_ns < GPS_WEEK_NANOSECONDS;\n}",
    "bool nonempty_path(const char* path) {\n"
    "    return path != nullptr && path[0] != '\\0';\n"
    "}\n\n"
    "bool has_complete_galileo_has_paths(const SimulatorRunOptions& options) {\n"
    "    return nonempty_path(options.galileo_has_sp3_path) && nonempty_path(options.galileo_has_clock_path) &&\n"
    "           nonempty_path(options.galileo_has_bias_path);\n"
    "}\n\n"
    "bool valid_run_options(const SimulatorRunOptions& options) {\n"
    "    const int has_path_count = static_cast<int>(nonempty_path(options.galileo_has_sp3_path)) +\n"
    "                               static_cast<int>(nonempty_path(options.galileo_has_clock_path)) +\n"
    "                               static_cast<int>(nonempty_path(options.galileo_has_bias_path));\n"
    "    return nonempty_path(options.rinex_nav_path) && nonempty_path(options.output_log_path) &&\n"
    "           options.start_time.gps_week >= 0 && options.start_time.tow_ns >= 0 &&\n"
    "           options.start_time.tow_ns < GPS_WEEK_NANOSECONDS && (has_path_count == 0 || has_path_count == 3);\n"
    "}",
)

replace_once(
    "src/core/simulator.cpp",
    "bool write_message(std::ofstream* output, const std::string& message, std::string* error_message) {",
    "bool galileo_has_state_provider(const void* context, int gps_week, double sow_sec, int satellite_number,\n"
    "                                RtklibSatelliteState* state, std::string* error_message) {\n"
    "    if (context == nullptr || state == nullptr) {\n"
    "        set_error(error_message, \"Galileo HAS state-provider request is invalid\");\n"
    "        return false;\n"
    "    }\n"
    "    GalileoHasE6Correction correction{};\n"
    "    if (!galileo_has_e6_correction(static_cast<const GalileoHasStore*>(context), gps_week, sow_sec,\n"
    "                                    satellite_number, &correction, error_message)) {\n"
    "        return false;\n"
    "    }\n"
    "    *state = correction.satellite_state;\n"
    "    return true;\n"
    "}\n\n"
    "bool write_message(std::ofstream* output, const std::string& message, std::string* error_message) {",
)

new_update_function = r'''bool update_tracking_and_measurements(RuntimeState* runtime, const SimConfig& config,
                                      const ScenarioEpochState& scenario,
                                      std::vector<MeasurementObservation>* measurements, int* tracked_satellites,
                                      TruthWriter* truth_writer, std::string* error_message) {
    measurements->clear();
    *tracked_satellites = 0;
    const RtklibNavStore* truth_nav = truth_navigation_store(runtime->navigation);
    const AcquisitionContext initial_context = startup_context(runtime->startup_mode);
    const double receive_sow_sec =
        static_cast<double>(scenario.time.tow_ns) / static_cast<double>(NANOSECONDS_PER_SECOND);

    for (SatelliteRuntime& satellite : runtime->satellites) {
        const bool broadcast_geometry_available =
            rtklib_satellite_state_available(truth_nav, scenario.time.gps_week, receive_sow_sec,
                                             satellite.satellite_number);
        SatelliteGeometry broadcast_geometry{};
        if (broadcast_geometry_available &&
            !compute_satellite_geometry(truth_nav, runtime->receiver, scenario.time, satellite.satellite_number,
                                        config.elevation_mask_deg, &broadcast_geometry, error_message)) {
            return false;
        }

        int glonass_fcn = 0;
        if (broadcast_geometry_available && satellite.constellation == GnssConstellation::kGlonass) {
            RtklibBroadcastBiasData bias_data{};
            if (!rtklib_broadcast_bias_data(truth_nav, broadcast_geometry.transmit_gps_week,
                                            broadcast_geometry.transmit_sow_sec, satellite.satellite_number, &bias_data,
                                            error_message)) {
                return false;
            }
            glonass_fcn = bias_data.glonass_fcn;
        }

        bool satellite_tracking = false;
        for (SignalRuntime& signal : satellite.signals) {
            const SignalDefinition* definition = find_signal_definition(signal.tracker.signal_id);
            if (definition == nullptr) {
                set_error(error_message, "signal definition is missing during tracking update");
                return false;
            }

            SatelliteGeometry signal_geometry{};
            bool use_has_e6 = false;
            double has_e6_code_bias_m = 0.0;
            if (definition->signal_id == SignalId::kGalileoE6 && runtime->galileo_has != nullptr) {
                GalileoHasE6Correction receive_probe{};
                if (galileo_has_e6_correction(runtime->galileo_has, scenario.time.gps_week, receive_sow_sec,
                                              satellite.satellite_number, &receive_probe, nullptr)) {
                    std::string has_error;
                    if (!compute_satellite_geometry_with_provider(
                            galileo_has_state_provider, runtime->galileo_has, runtime->receiver, scenario.time,
                            satellite.satellite_number, config.elevation_mask_deg, &signal_geometry, &has_error)) {
                        set_error(error_message, has_error);
                        return false;
                    }
                    GalileoHasE6Correction transmit_correction{};
                    if (!galileo_has_e6_correction(runtime->galileo_has, signal_geometry.transmit_gps_week,
                                                   signal_geometry.transmit_sow_sec, satellite.satellite_number,
                                                   &transmit_correction, error_message)) {
                        return false;
                    }
                    has_e6_code_bias_m = transmit_correction.code_osb_m;
                    use_has_e6 = true;
                }
            }
            if (!use_has_e6) {
                if (!broadcast_geometry_available) {
                    continue;
                }
                signal_geometry = broadcast_geometry;
            }

            bool signal_healthy = signal_geometry.healthy;
            RtklibBroadcastMessageFamily health_family = RtklibBroadcastMessageFamily::kUnknown;
            if (definition->nav_message_family == NavMessageFamily::kGpsCnav) {
                health_family = RtklibBroadcastMessageFamily::kCnav;
            } else if (definition->nav_message_family == NavMessageFamily::kGpsCnav2) {
                health_family = RtklibBroadcastMessageFamily::kCnav2;
            }
            if (health_family != RtklibBroadcastMessageFamily::kUnknown) {
                int signal_health = 0;
                if (!rtklib_signal_health_for_family(truth_nav, signal_geometry.transmit_gps_week,
                                                     signal_geometry.transmit_sow_sec, satellite.satellite_number,
                                                     definition->rinex_signal_code, health_family, &signal_health,
                                                     error_message)) {
                    return false;
                }
                signal_healthy = signal_health == 0;
            }

            const bool signal_available =
                scenario.signal_available &&
                (health_family != RtklibBroadcastMessageFamily::kUnknown ? signal_geometry.above_elevation_mask
                                                                         : signal_geometry.visible);
            if (health_family != RtklibBroadcastMessageFamily::kUnknown) {
                signal_geometry.healthy = signal_healthy;
                signal_geometry.visible = signal_geometry.above_elevation_mask && signal_healthy;
            }
            const double elevation_deg = signal_geometry.elevation_rad * kRadiansToDegrees;
            if (signal_available && !signal.tracker.scheduled) {
                const AcquisitionContext context =
                    signal.ever_scheduled ? AcquisitionContext::kReacquisition : initial_context;
                SimTime search_ready_time = scenario.time;
                if (!signal.ever_scheduled &&
                    compare_sim_time(runtime->startup_search_ready_time, search_ready_time) > 0) {
                    search_ready_time = runtime->startup_search_ready_time;
                }
                if (!schedule_signal_acquisition(&signal.tracker, context, scenario.time, search_ready_time,
                                                 elevation_deg, runtime->cn0_model, runtime->tracking_config,
                                                 &runtime->rng, error_message)) {
                    return false;
                }
                signal.ever_scheduled = true;
            }
            if (!update_signal_tracker(&signal.tracker, scenario.time, signal_available, elevation_deg,
                                       runtime->cn0_model, error_message)) {
                return false;
            }
            if (signal.tracker.phase != SignalTrackingPhase::kTracking) {
                reset_carrier_ambiguity_state(&signal.ambiguity);
                continue;
            }
            satellite_tracking = true;

            AtmosphereCorrection atmosphere{};
            if (!compute_atmosphere_correction(config.atmosphere_mode, truth_nav, scenario.time,
                                               signal.tracker.signal_id, glonass_fcn,
                                               runtime->receiver.position_ecef_m, signal_geometry.azimuth_rad,
                                               signal_geometry.elevation_rad, &atmosphere, error_message)) {
                return false;
            }
            MeasurementObservation observation{};
            const bool measurement_ok =
                use_has_e6
                    ? generate_zero_noise_measurement_with_explicit_code_bias(
                          signal_geometry, runtime->receiver, signal.tracker, atmosphere, has_e6_code_bias_m,
                          &signal.ambiguity, &observation, error_message)
                    : generate_zero_noise_measurement(truth_nav, signal_geometry, runtime->receiver, signal.tracker,
                                                      atmosphere, &signal.ambiguity, &observation, error_message);
            if (!measurement_ok ||
                !truth_writer_write_observation(truth_writer, runtime->receiver, signal_geometry, signal.tracker,
                                                observation, error_message)) {
                return false;
            }
            measurements->push_back(observation);
        }
        if (satellite_tracking) {
            ++(*tracked_satellites);
        }
    }
    return true;
}

'''
regex_once(
    "src/core/simulator.cpp",
    r"bool update_tracking_and_measurements\(RuntimeState\* runtime,.*?\n}\n\n(?=bool emit_epoch_logs)",
    new_update_function,
)

# Load HAS products once per run. RAII guarantees cleanup on every existing
# early-return path without disturbing navigation-state ownership.
replace_once(
    "src/core/simulator.cpp",
    "    RuntimeState runtime{};\n    runtime.navigation = create_navigation_state();",
    "    RuntimeState runtime{};\n"
    "    std::unique_ptr<GalileoHasStore, void (*)(GalileoHasStore*)> galileo_has_store(nullptr,\n"
    "                                                                                  destroy_galileo_has_store);\n"
    "    if (has_complete_galileo_has_paths(options)) {\n"
    "        galileo_has_store.reset(create_galileo_has_store());\n"
    "        if (galileo_has_store == nullptr ||\n"
    "            !load_galileo_has_products(galileo_has_store.get(), options.galileo_has_sp3_path,\n"
    "                                        options.galileo_has_clock_path, options.galileo_has_bias_path,\n"
    "                                        error_message)) {\n"
    "            if (galileo_has_store == nullptr) {\n"
    "                set_error(error_message, \"cannot allocate Galileo HAS product store\");\n"
    "            }\n"
    "            return false;\n"
    "        }\n"
    "        runtime.galileo_has = galileo_has_store.get();\n"
    "    }\n"
    "    runtime.navigation = create_navigation_state();",
)

# Dedicated real-JRC HAS E6 companion acceptance. The 2025 BRD400 fixture is
# intentionally only a satellite/signal roster at this 2026 epoch; E6 state and
# C6C bias must come from the JRC HAS products.
replace_once(
    "tests/integration/test_all_signal_residuals.cpp",
    "std::string gps_cnv2_nav_path() {\n    return std::string(GNSS_SIM_TEST_DATA_DIR) + \"/gps_cnv2_g04_2022278.rnx\";\n}\n",
    "std::string gps_cnv2_nav_path() {\n    return std::string(GNSS_SIM_TEST_DATA_DIR) + \"/gps_cnv2_g04_2022278.rnx\";\n}\n\n"
    "std::string jrc_has_sp3_path() {\n    return std::string(GNSS_SIM_TEST_DATA_DIR) + \"/jrc_has_2026001_e02.sp3\";\n}\n\n"
    "std::string jrc_has_clock_path() {\n    return std::string(GNSS_SIM_TEST_DATA_DIR) + \"/jrc_has_2026001_e02.clk\";\n}\n\n"
    "std::string jrc_has_bias_path() {\n    return std::string(GNSS_SIM_TEST_DATA_DIR) + \"/jrc_has_2026001_e02_c6c.bia\";\n}\n",
)

has_block = r'''
    // Official JRC Galileo HAS E6 companion. The RINEX NAV path supplies only
    // the simulator's constellation/signal roster here; its 2025 ephemerides
    // are deliberately stale at this 2026 epoch. Galileo E6 must therefore be
    // generated from JRC HAS precise orbit/clock + C6C OSB, and the RTKLIB
    // explicit-state oracle validates the exact truth state written by the
    // simulator without reinterpreting HAS as INAV/FNAV broadcast data.
    {
        gnss_sim::SimConfig config = base_config;
        config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
        config.receiver = {-43.2162386, -15.4759141, 100.0};
        config.duration_ns = 60LL * gnss_sim::NANOSECONDS_PER_SECOND;
        gnss_sim::SimTime has_start{};
        ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2399, 346100.0, &has_start));

        const std::filesystem::path has_directory = directory / "galileo_e6_jrc_has";
        ASSERT_TRUE(std::filesystem::create_directories(has_directory, filesystem_error));
        ASSERT_FALSE(filesystem_error);
        const std::filesystem::path has_output_path = has_directory / "simulated.log";
        const std::string has_output_text = has_output_path.string();
        const std::string has_nav_text = brd4_nav_path();
        const std::string has_sp3_text = jrc_has_sp3_path();
        const std::string has_clock_text = jrc_has_clock_path();
        const std::string has_bias_text = jrc_has_bias_path();
        const gnss_sim::SimulatorRunOptions has_options{has_nav_text.c_str(),   has_output_text.c_str(), has_start,
                                                        nullptr,                has_sp3_text.c_str(),    has_clock_text.c_str(),
                                                        has_bias_text.c_str()};
        gnss_sim::SimulatorRunSummary has_summary{};
        std::string has_error_message;
        ASSERT_TRUE(gnss_sim::run_simulator(config, has_options, &has_summary, &has_error_message))
            << has_error_message;

        prcopt_t has_residual_options = residual_options;
        has_residual_options.ionoopt = IONOOPT_OFF;
        has_residual_options.tropopt = TROPOPT_OFF;

        std::ifstream input(has_directory / "observation_truth.csv");
        ASSERT_TRUE(input.good());
        std::string line;
        ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
        const std::map<std::string, std::size_t> column = header_columns(line);
        std::uint64_t has_e6_rows = 0;
        while (std::getline(input, line)) {
            if (line.empty())
                continue;
            const std::vector<std::string> fields = split_csv(line);
            ASSERT_EQ(fields.size(), column.size());
            if (fields[column.at("signal_name")] != "Galileo E6")
                continue;

            seen_signals.insert("Galileo E6");
            SignalResidualStats& signal_stats = stats["Galileo E6"];
            ++signal_stats.rows;
            ASSERT_EQ(fields[column.at("broadcast_message_family")], "UNKNOWN");
            ASSERT_EQ(fields[column.at("code_bias_status")], "APPLIED");
            ASSERT_EQ(fields[column.at("pseudorange_valid")], "1");
            ASSERT_EQ(fields[column.at("doppler_valid")], "1");

            const gnss_sim::SignalDefinition* definition =
                gnss_sim::find_signal_definition(gnss_sim::SignalId::kGalileoE6);
            ASSERT_NE(definition, nullptr);
            int observation_code = 0;
            int frequency_index = 0;
            ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*definition, &observation_code, &frequency_index));
            static_cast<void>(frequency_index);

            obsd_t observation{};
            observation.time =
                gpst2time(std::stoi(fields[column.at("gps_week")]), std::stod(fields[column.at("sow_sec")]));
            observation.sat = static_cast<unsigned char>(std::stoi(fields[column.at("satellite_number")]));
            observation.rcv = 1;
            observation.code[0] = static_cast<unsigned char>(observation_code);
            observation.SNR[0] = static_cast<unsigned char>(
                std::clamp(std::lround(std::stod(fields[column.at("cn0_dbhz")]) * 4.0), 0L, 255L));
            observation.P[0] = std::stod(fields[column.at("pseudorange_m")]);
            observation.D[0] = static_cast<float>(std::stod(fields[column.at("doppler_hz")]));

            char satellite_id[16]{};
            satno2id(observation.sat, satellite_id);
            ASSERT_STREQ(satellite_id, "E02");

            const double receiver_position_m[3] = {std::stod(fields[column.at("receiver_x_m")]),
                                                   std::stod(fields[column.at("receiver_y_m")]),
                                                   std::stod(fields[column.at("receiver_z_m")])};
            const double receiver_velocity_mps[3] = {std::stod(fields[column.at("receiver_vx_mps")]),
                                                     std::stod(fields[column.at("receiver_vy_mps")]),
                                                     std::stod(fields[column.at("receiver_vz_mps")])};
            const double satellite_state[6] = {std::stod(fields[column.at("satellite_x_m")]),
                                               std::stod(fields[column.at("satellite_y_m")]),
                                               std::stod(fields[column.at("satellite_z_m")]),
                                               std::stod(fields[column.at("satellite_vx_mps")]),
                                               std::stod(fields[column.at("satellite_vy_mps")]),
                                               std::stod(fields[column.at("satellite_vz_mps")])};
            const double satellite_clock[2] = {std::stod(fields[column.at("satellite_clock_bias_m")]) / CLIGHT,
                                               std::stod(fields[column.at("satellite_clock_drift_mps")]) / CLIGHT};
            const double wavelength_m = std::stod(fields[column.at("wavelength_m")]);
            const double code_bias_m = std::stod(fields[column.at("code_bias_m")]);

            double code_residual_m = 0.0;
            const int code_status = rtklib_rescode_state_ext(
                &observation, &nav, &has_residual_options, receiver_position_m, 0.0, 0.0, satellite_state,
                satellite_clock, 0, code_bias_m, wavelength_m, &code_residual_m, nullptr);
            ASSERT_EQ(code_status, 1) << "JRC HAS E6 code residual failed at sow=" << fields[column.at("sow_sec")];
            ++signal_stats.code_residuals;
            signal_stats.max_abs_code_m = (std::max)(signal_stats.max_abs_code_m, std::fabs(code_residual_m));

            double doppler_residual_mps = 0.0;
            const int doppler_status = rtklib_resdop_state_ext(
                &observation, &has_residual_options, receiver_position_m, receiver_velocity_mps, 0.0, satellite_state,
                satellite_clock, 0, wavelength_m, &doppler_residual_mps, nullptr);
            ASSERT_EQ(doppler_status, 1)
                << "JRC HAS E6 Doppler residual failed at sow=" << fields[column.at("sow_sec")];
            ++signal_stats.doppler_residuals;
            signal_stats.max_abs_doppler_mps =
                (std::max)(signal_stats.max_abs_doppler_mps, std::fabs(doppler_residual_mps));
            ++has_e6_rows;
        }
        ASSERT_GT(has_e6_rows, 0U) << "official JRC HAS fixture must exercise Galileo E6 residuals";
        std::fprintf(stderr,
                     "GALILEO_HAS_E6_COVERAGE rows=%llu max_abs_code=%.9f max_abs_doppler=%.9f source=JRC_HAS_2026001_E02\n",
                     static_cast<unsigned long long>(has_e6_rows), stats["Galileo E6"].max_abs_code_m,
                     stats["Galileo E6"].max_abs_doppler_mps);
    }

'''
replace_once(
    "tests/integration/test_all_signal_residuals.cpp",
    "    std::size_t definition_count = 0;\n",
    has_block + "    std::size_t definition_count = 0;\n",
)

replace_once(
    "tests/integration/test_all_signal_residuals.cpp",
    "        if (signal_name == \"Galileo E6\") {\n"
    "            EXPECT_EQ(signal_stats.code_residuals, 0U) << \"E6 must remain unavailable until HAS code bias is modeled\";\n"
    "            EXPECT_GT(signal_stats.code_unavailable, 0U);\n"
    "        } else if (signal_name == \"GPS L1C\") {",
    "        if (signal_name == \"Galileo E6\") {\n"
    "            EXPECT_GT(signal_stats.code_residuals, 0U)\n"
    "                << \"official JRC HAS products must exercise Galileo E6 code residuals\";\n"
    "            EXPECT_GT(signal_stats.code_unavailable, 0U)\n"
    "                << \"the compact BRD400 fixture must still expose missing HAS explicitly\";\n"
    "        } else if (signal_name == \"GPS L1C\") {",
)
replace_once(
    "tests/integration/test_all_signal_residuals.cpp",
    "    EXPECT_EQ(code_covered_signal_count, 20U)\n"
    "        << \"only Galileo E6/HAS may remain code-unavailable after real GPS CNV2 validation\";",
    "    EXPECT_EQ(code_covered_signal_count, 21U)\n"
    "        << \"official JRC HAS E6 companion must complete all 21 V1 code-residual paths\";",
)

print("issue51 Galileo HAS E6 pipeline patch applied")
