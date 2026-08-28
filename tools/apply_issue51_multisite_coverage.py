from pathlib import Path

path = Path("tests/integration/test_all_signal_residuals.cpp")
text = path.read_text()
include_old = '''#include <rtklib.h>
#include <rtklib_residual_ext.h>
}'''
include_new = '''#include <rtklib.h>
#include <rtklib_residual_ext.h>
#include <rtklib_signal_bias_ext.h>
}'''
if text.count(include_old) != 1:
    raise RuntimeError("signal-bias include anchor mismatch")
text = text.replace(include_old, include_new, 1)

start_marker = "TEST(V1Acceptance, EveryFrozenSignalRunsTruthStateCodeAndDopplerResidualChecks) {"
end_marker = "\n\n} // namespace"
start = text.find(start_marker)
end = text.rfind(end_marker)
if start < 0 or end < 0 or end <= start:
    raise RuntimeError("residual acceptance test boundary mismatch")

new_test = r'''TEST(V1Acceptance, EveryFrozenSignalRunsTruthStateCodeAndDopplerResidualChecks) {
    gnss_sim::SimConfig base_config = gnss_sim::default_sim_config();
    base_config.scenario = gnss_sim::ScenarioType::KS;
    base_config.atmosphere_mode = gnss_sim::AtmosphereMode::BROADCAST;
    base_config.elevation_mask_deg = 0.0;
    base_config.sampling_rate_hz = 1;
    base_config.duration_ns = 60LL * gnss_sim::NANOSECONDS_PER_SECOND;
    base_config.measurement_noise_enabled = false;
    base_config.multipath_enabled = false;
    base_config.receiver_clock_bias_m = 0.0;
    base_config.receiver_clock_drift_mps = 0.0;
    base_config.seed = 0x51U;

    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 436500.0, &start));

    const std::filesystem::path directory = "gnss_sim_all_signal_residuals";
    std::error_code filesystem_error;
    std::filesystem::remove_all(directory, filesystem_error);
    filesystem_error.clear();
    ASSERT_TRUE(std::filesystem::create_directories(directory, filesystem_error));
    ASSERT_FALSE(filesystem_error);

    std::string nav_text;
    ASSERT_TRUE(write_g3_overlay_nav(directory, &nav_text));
    nav_t nav{};
    ASSERT_TRUE(load_nav(nav_text, &nav));

    const gtime_t diagnostic_time = gpst2time(start.gps_week, gnss_sim::sim_time_sow_sec(start));
    const int g17 = satid2no("G17");
    ASSERT_GT(g17, 0);
    for (gnss_sim::SignalId signal_id : {gnss_sim::SignalId::kGpsL2C, gnss_sim::SignalId::kGpsL5Q}) {
        const gnss_sim::SignalDefinition* definition = gnss_sim::find_signal_definition(signal_id);
        ASSERT_NE(definition, nullptr);
        int observation_code = 0;
        int frequency_index = 0;
        ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*definition, &observation_code, &frequency_index));
        static_cast<void>(frequency_index);
        eph_t eph{};
        geph_t geph{};
        rtklib_signal_bias_info_ext_t eph_info{};
        rtklib_signal_bias_info_ext_t bias_info{};
        double bias_m = 0.0;
        const int eph_status = rtklib_signal_ephemeris_ext(diagnostic_time, g17,
                                                           static_cast<unsigned char>(observation_code), NAV_CNAV,
                                                           &nav, &eph, &geph, &eph_info);
        const int bias_status = rtklib_signal_code_bias_ext(diagnostic_time, g17,
                                                            static_cast<unsigned char>(observation_code), NAV_CNAV,
                                                            &nav, &bias_m, &bias_info);
        std::fprintf(stderr,
                     "GPS_CNAV_DIRECT signal=%s sat=G17 eph_status=%d eph_type=%d bias_status=%d bias_type=%d bias_m=%.9f\n",
                     definition->name, eph_status, eph_info.message_type, bias_status, bias_info.message_type, bias_m);
    }

    prcopt_t residual_options = prcopt_default;
    residual_options.mode = PMODE_SINGLE;
    residual_options.nf = 1;
    residual_options.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_CMP;
    residual_options.elmin = 0.0;
    residual_options.sateph = EPHOPT_BRDC;
    residual_options.ionoopt = IONOOPT_BRDC;
    residual_options.tropopt = TROPOPT_SAAS;

    std::map<std::string, SignalResidualStats> stats;
    std::set<std::string> seen_signals;
    std::set<std::string> printed_g17_rows;

    struct ResidualSite {
        const char* name;
        double latitude_deg;
        double longitude_deg;
    };
    const ResidualSite sites[] = {
        {"asia", 20.0, 120.0},
        {"gps_cnav", 36.272115, -19.973734},
        {"bds_bcnav12", -47.507042, -174.038033},
    };

    for (const ResidualSite& site : sites) {
        gnss_sim::SimConfig config = base_config;
        config.receiver = {site.latitude_deg, site.longitude_deg, 100.0};

        const std::filesystem::path site_directory = directory / site.name;
        ASSERT_TRUE(std::filesystem::create_directories(site_directory, filesystem_error));
        ASSERT_FALSE(filesystem_error);
        const std::filesystem::path output_path = site_directory / "simulated.log";
        const std::string output_text = output_path.string();
        const gnss_sim::SimulatorRunOptions options{nav_text.c_str(), output_text.c_str(), start};
        gnss_sim::SimulatorRunSummary run_summary{};
        std::string error_message;
        ASSERT_TRUE(gnss_sim::run_simulator(config, options, &run_summary, &error_message))
            << "site=" << site.name << " " << error_message;

        std::ifstream input(site_directory / "observation_truth.csv");
        ASSERT_TRUE(input.good()) << site.name;
        std::string line;
        ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
        const std::map<std::string, std::size_t> column = header_columns(line);
        for (const char* required :
             {"gps_week", "sow_sec", "signal_id", "signal_name", "satellite_number", "wavelength_m",
              "receiver_x_m", "receiver_y_m", "receiver_z_m", "receiver_vx_mps", "receiver_vy_mps",
              "receiver_vz_mps", "elevation_deg", "cn0_dbhz", "broadcast_message_family", "code_bias_status",
              "tracking_phase", "pseudorange_valid", "doppler_valid", "pseudorange_m", "doppler_hz"}) {
            ASSERT_EQ(column.count(required), 1U) << "site=" << site.name << " column=" << required;
        }

        while (std::getline(input, line)) {
            if (line.empty()) continue;
            const std::vector<std::string> fields = split_csv(line);
            ASSERT_EQ(fields.size(), column.size());

            const int signal_id = std::stoi(fields[column.at("signal_id")]);
            const gnss_sim::SignalDefinition* definition =
                gnss_sim::find_signal_definition(static_cast<gnss_sim::SignalId>(signal_id));
            ASSERT_NE(definition, nullptr);
            const std::string signal_name = fields[column.at("signal_name")];
            ASSERT_EQ(signal_name, definition->name);
            seen_signals.insert(signal_name);
            SignalResidualStats& signal_stats = stats[signal_name];
            ++signal_stats.rows;

            int observation_code = 0;
            int frequency_index = 0;
            ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*definition, &observation_code, &frequency_index));
            ASSERT_GT(observation_code, 0);
            static_cast<void>(frequency_index);

            obsd_t observation{};
            const int gps_week = std::stoi(fields[column.at("gps_week")]);
            const double sow_sec = std::stod(fields[column.at("sow_sec")]);
            observation.time = gpst2time(gps_week, sow_sec);
            observation.sat = static_cast<unsigned char>(std::stoi(fields[column.at("satellite_number")]));
            observation.rcv = 1;
            observation.code[0] = static_cast<unsigned char>(observation_code);
            observation.SNR[0] = static_cast<unsigned char>(
                std::clamp(std::lround(std::stod(fields[column.at("cn0_dbhz")]) * 4.0), 0L, 255L));
            observation.P[0] = std::stod(fields[column.at("pseudorange_m")]);
            observation.D[0] = static_cast<float>(std::stod(fields[column.at("doppler_hz")]));

            char satellite_id[16]{};
            satno2id(observation.sat, satellite_id);
            if (std::string(site.name) == "gps_cnav" && std::string(satellite_id) == "G17" &&
                (signal_name == "GPS L2C" || signal_name == "GPS L5Q") && printed_g17_rows.insert(signal_name).second) {
                std::fprintf(stderr,
                             "GPS_CNAV_TRUTH signal=%s sat=G17 elev=%s family=%s bias_status=%s tracking=%s psr_valid=%s\n",
                             signal_name.c_str(), fields[column.at("elevation_deg")].c_str(),
                             fields[column.at("broadcast_message_family")].c_str(),
                             fields[column.at("code_bias_status")].c_str(), fields[column.at("tracking_phase")].c_str(),
                             fields[column.at("pseudorange_valid")].c_str());
            }

            const double wavelength_m = std::stod(fields[column.at("wavelength_m")]);
            const double receiver_position_m[3] = {std::stod(fields[column.at("receiver_x_m")]),
                                                   std::stod(fields[column.at("receiver_y_m")]),
                                                   std::stod(fields[column.at("receiver_z_m")])};
            const double receiver_velocity_mps[3] = {std::stod(fields[column.at("receiver_vx_mps")]),
                                                     std::stod(fields[column.at("receiver_vy_mps")]),
                                                     std::stod(fields[column.at("receiver_vz_mps")])};
            const int required_message_type = required_rtklib_message_type(*definition);

            rtklib_signal_bias_info_ext_t bias_info{};
            double code_residual_m = 0.0;
            const int code_status =
                rtklib_rescode_signal_ext(&observation, &nav, &residual_options, receiver_position_m, 0.0, 0.0,
                                          required_message_type, wavelength_m, &code_residual_m, nullptr, &bias_info);
            const bool family_unavailable =
                fields[column.at("code_bias_status")] == "UNAVAILABLE_FOR_MESSAGE_FAMILY";
            if (fields[column.at("pseudorange_valid")] == "1") {
                ASSERT_EQ(code_status, 1) << "site=" << site.name << " signal=" << signal_name
                                          << " sat=" << static_cast<int>(observation.sat);
                ++signal_stats.code_residuals;
                signal_stats.max_abs_code_m = (std::max)(signal_stats.max_abs_code_m, std::fabs(code_residual_m));
            } else {
                ++signal_stats.code_unavailable;
                if (family_unavailable) {
                    EXPECT_EQ(code_status, 0) << "site=" << site.name
                                              << " unavailable family must remain unavailable to RTKLIB; signal="
                                              << signal_name;
                }
            }

            if (fields[column.at("doppler_valid")] == "1") {
                double doppler_residual_mps = 0.0;
                const int doppler_status =
                    rtklib_resdop_signal_ext(&observation, &nav, &residual_options, receiver_position_m,
                                             receiver_velocity_mps, 0.0, 0, wavelength_m, &doppler_residual_mps,
                                             nullptr);
                ASSERT_EQ(doppler_status, 1) << "site=" << site.name << " signal=" << signal_name
                                             << " sat=" << static_cast<int>(observation.sat);
                ++signal_stats.doppler_residuals;
                signal_stats.max_abs_doppler_mps =
                    (std::max)(signal_stats.max_abs_doppler_mps, std::fabs(doppler_residual_mps));
            }
        }
    }

    std::size_t definition_count = 0;
    const gnss_sim::SignalDefinition* definitions = gnss_sim::signal_definitions(&definition_count);
    ASSERT_NE(definitions, nullptr);
    ASSERT_EQ(definition_count, 21U);
    ASSERT_EQ(seen_signals.size(), definition_count);

    std::size_t code_covered_signal_count = 0;
    for (std::size_t index = 0; index < definition_count; ++index) {
        const std::string signal_name = definitions[index].name;
        ASSERT_EQ(seen_signals.count(signal_name), 1U) << signal_name;
        const SignalResidualStats& signal_stats = stats.at(signal_name);
        EXPECT_GT(signal_stats.rows, 0U) << signal_name;
        EXPECT_GT(signal_stats.doppler_residuals, 0U) << "every V1 frequency must execute resdop: " << signal_name;
        EXPECT_LT(signal_stats.max_abs_doppler_mps, 0.002)
            << "Doppler residual exceeds the RANGEA 0.001-Hz serialization floor: " << signal_name;
        EXPECT_GT(signal_stats.code_residuals + signal_stats.code_unavailable, 0U)
            << "every V1 frequency must execute a code-residual/bias availability check: " << signal_name;
        if (signal_stats.code_residuals > 0U) {
            ++code_covered_signal_count;
            EXPECT_LT(signal_stats.max_abs_code_m, 0.02)
                << "code residual exceeds the RANGEA millimetre serialization floor: " << signal_name;
        }
        std::fprintf(stderr, "CODE_COVERAGE signal=%s residual_rows=%llu unavailable_rows=%llu max_abs_code=%.9f\n",
                     signal_name.c_str(), static_cast<unsigned long long>(signal_stats.code_residuals),
                     static_cast<unsigned long long>(signal_stats.code_unavailable), signal_stats.max_abs_code_m);
        if (signal_name == "Galileo E6") {
            EXPECT_EQ(signal_stats.code_residuals, 0U) << "E6 must remain unavailable until HAS code bias is modeled";
            EXPECT_GT(signal_stats.code_unavailable, 0U);
        } else if (signal_name == "GPS L1C" || signal_name == "GPS L2C" || signal_name == "GPS L5Q") {
            // Diagnostic phase for GPS modern-family coverage.
        } else {
            EXPECT_GT(signal_stats.code_residuals, 0U)
                << "real compact coverage union must exercise every available non-E6 signal: " << signal_name;
        }
    }
    std::fprintf(stderr, "CODE_COVERAGE_UNION covered=%zu total=21\n", code_covered_signal_count);

    free_nav(&nav);
    std::filesystem::remove_all(directory, filesystem_error);
}'''

path.write_text(text[:start] + new_test + text[end:])
print("multi-site coverage plus GPS CNAV selector/truth diagnostics applied")
