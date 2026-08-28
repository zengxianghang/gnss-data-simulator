from pathlib import Path

path = Path("tests/integration/test_all_signal_residuals.cpp")
text = path.read_text()

old_path = '''std::string brd4_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}
'''
new_path = '''std::string brd4_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

std::string gps_cnv2_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/gps_cnv2_g04_2022278.rnx";
}
'''
if text.count(old_path) != 1:
    raise RuntimeError(f"nav path anchor count={text.count(old_path)}")
text = text.replace(old_path, new_path, 1)

anchor = '''    std::size_t definition_count = 0;
    const gnss_sim::SignalDefinition* definitions = gnss_sim::signal_definitions(&definition_count);
'''
block = r'''    // Real GPS CNAV-2 companion, provenance-fixed from Orekit commit
    // c5b14ff008eb02482ad4e2a2347c97ce4800969c, which records that the
    // G04 block was extracted from BRD400DLR_S_20222780000_01D_MN.rnx.
    // The broadcast health is 1, so strict positioning use must reject it;
    // diagnostic residual validation may ignore health while preserving the
    // real CNAV-2 orbit/clock and L1C ISC/code-bias model.
    {
        nav_t cnv2_nav{};
        ASSERT_TRUE(load_nav(gps_cnv2_nav_path(), &cnv2_nav));

        gnss_sim::SimConfig config = base_config;
        config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
        config.receiver = {7.04, -36.05, 100.0};
        config.duration_ns = 60LL * gnss_sim::NANOSECONDS_PER_SECOND;
        gnss_sim::SimTime cnv2_start{};
        ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2230, 275400.0, &cnv2_start));

        const std::filesystem::path cnv2_directory = directory / "gps_cnv2_real";
        ASSERT_TRUE(std::filesystem::create_directories(cnv2_directory, filesystem_error));
        ASSERT_FALSE(filesystem_error);
        const std::filesystem::path cnv2_output_path = cnv2_directory / "simulated.log";
        const std::string cnv2_output_text = cnv2_output_path.string();
        const std::string cnv2_nav_text = gps_cnv2_nav_path();
        const gnss_sim::SimulatorRunOptions cnv2_options{cnv2_nav_text.c_str(), cnv2_output_text.c_str(), cnv2_start};
        gnss_sim::SimulatorRunSummary cnv2_summary{};
        std::string cnv2_error_message;
        ASSERT_TRUE(gnss_sim::run_simulator(config, cnv2_options, &cnv2_summary, &cnv2_error_message))
            << cnv2_error_message;

        prcopt_t cnv2_residual_options = residual_options;
        cnv2_residual_options.ionoopt = IONOOPT_OFF;
        cnv2_residual_options.tropopt = TROPOPT_OFF;

        std::ifstream input(cnv2_directory / "observation_truth.csv");
        ASSERT_TRUE(input.good());
        std::string line;
        ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
        const std::map<std::string, std::size_t> column = header_columns(line);
        std::uint64_t l1c_diagnostic_rows = 0;
        while (std::getline(input, line)) {
            if (line.empty())
                continue;
            const std::vector<std::string> fields = split_csv(line);
            ASSERT_EQ(fields.size(), column.size());
            if (fields[column.at("signal_name")] != "GPS L1C")
                continue;

            seen_signals.insert("GPS L1C");
            SignalResidualStats& signal_stats = stats["GPS L1C"];
            ++signal_stats.rows;

            const gnss_sim::SignalDefinition* definition =
                gnss_sim::find_signal_definition(gnss_sim::SignalId::kGpsL1C);
            ASSERT_NE(definition, nullptr);
            int observation_code = 0;
            int frequency_index = 0;
            ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*definition, &observation_code, &frequency_index));
            static_cast<void>(frequency_index);

            ASSERT_EQ(fields[column.at("broadcast_message_family")], "CNV2");
            ASSERT_EQ(fields[column.at("code_bias_status")], "APPLIED");
            if (fields[column.at("pseudorange_valid")] != "1") {
                ++signal_stats.code_unavailable;
                continue;
            }

            obsd_t observation{};
            observation.time = gpst2time(std::stoi(fields[column.at("gps_week")]),
                                         std::stod(fields[column.at("sow_sec")]));
            observation.sat = static_cast<unsigned char>(std::stoi(fields[column.at("satellite_number")]));
            observation.rcv = 1;
            observation.code[0] = static_cast<unsigned char>(observation_code);
            observation.SNR[0] = static_cast<unsigned char>(
                std::clamp(std::lround(std::stod(fields[column.at("cn0_dbhz")]) * 4.0), 0L, 255L));
            observation.P[0] = std::stod(fields[column.at("pseudorange_m")]);

            const double receiver_position_m[3] = {std::stod(fields[column.at("receiver_x_m")]),
                                                   std::stod(fields[column.at("receiver_y_m")]),
                                                   std::stod(fields[column.at("receiver_z_m")])};
            const double wavelength_m = std::stod(fields[column.at("wavelength_m")]);
            double code_residual_m = 0.0;
            rtklib_signal_bias_info_ext_t bias_info{};
            const int strict_status = rtklib_rescode_signal_ext(
                &observation, &cnv2_nav, &cnv2_residual_options, receiver_position_m, 0.0, 0.0, NAV_CNV2,
                wavelength_m, &code_residual_m, nullptr, &bias_info);
            ASSERT_EQ(strict_status, 0) << "strict L1C residual must preserve CNV2 broadcast-health exclusion";

            const int diagnostic_status = rtklib_rescode_signal_diagnostic_ext(
                &observation, &cnv2_nav, &cnv2_residual_options, receiver_position_m, 0.0, 0.0, NAV_CNV2,
                wavelength_m, &code_residual_m, nullptr, &bias_info);
            ASSERT_EQ(diagnostic_status, 1)
                << "real G04 CNV2 diagnostic L1C code residual failed at sow=" << fields[column.at("sow_sec")];
            EXPECT_EQ(bias_info.message_type, NAV_CNV2);
            ++signal_stats.code_residuals;
            ++l1c_diagnostic_rows;
            signal_stats.max_abs_code_m = (std::max)(signal_stats.max_abs_code_m, std::fabs(code_residual_m));
        }
        ASSERT_GT(l1c_diagnostic_rows, 0U) << "real G04 CNV2 fixture must exercise GPS L1C code residual";
        std::fprintf(stderr,
                     "GPS_CNV2_L1C_COVERAGE diagnostic_rows=%llu max_abs_code=%.9f source=BRD400DLR_2022278_G04\n",
                     static_cast<unsigned long long>(l1c_diagnostic_rows), stats["GPS L1C"].max_abs_code_m);
        free_nav(&cnv2_nav);
    }

'''
if text.count(anchor) != 1:
    raise RuntimeError(f"aggregate anchor count={text.count(anchor)}")
text = text.replace(anchor, block + anchor, 1)

old_assert = '''        if (signal_name == "Galileo E6") {
            EXPECT_EQ(signal_stats.code_residuals, 0U) << "E6 must remain unavailable until HAS code bias is modeled";
            EXPECT_GT(signal_stats.code_unavailable, 0U);
        } else if (signal_name == "GPS L1C") {
            EXPECT_EQ(signal_stats.code_residuals, 0U)
                << "GPS L1C code must remain unavailable while no real CNAV-2 navigation data are broadcast";
            EXPECT_GT(signal_stats.code_unavailable, 0U);
        } else {
            EXPECT_GT(signal_stats.code_residuals, 0U)
                << "real compact coverage union must exercise every available non-E6 signal: " << signal_name;
        }
'''
new_assert = '''        if (signal_name == "Galileo E6") {
            EXPECT_EQ(signal_stats.code_residuals, 0U) << "E6 must remain unavailable until HAS code bias is modeled";
            EXPECT_GT(signal_stats.code_unavailable, 0U);
        } else if (signal_name == "GPS L1C") {
            EXPECT_GT(signal_stats.code_residuals, 0U)
                << "provenance-fixed real G04 CNV2 must exercise L1C diagnostic code residual";
            EXPECT_GT(signal_stats.code_unavailable, 0U)
                << "the 2025 compact BRD4 fixture must still record its missing CNV2 family explicitly";
        } else {
            EXPECT_GT(signal_stats.code_residuals, 0U)
                << "real compact coverage union must exercise every available non-E6 signal: " << signal_name;
        }
'''
if text.count(old_assert) != 1:
    raise RuntimeError(f"coverage assertion anchor count={text.count(old_assert)}")
text = text.replace(old_assert, new_assert, 1)

old_count = '''    EXPECT_EQ(code_covered_signal_count, 19U)
        << "only developmental GPS L1C and Galileo E6/HAS may remain code-unavailable";
'''
new_count = '''    EXPECT_EQ(code_covered_signal_count, 20U)
        << "only Galileo E6/HAS may remain code-unavailable after real GPS CNV2 validation";
'''
if text.count(old_count) != 1:
    raise RuntimeError(f"coverage count anchor count={text.count(old_count)}")
text = text.replace(old_count, new_count, 1)

path.write_text(text)
print("real GPS G04 CNV2 L1C diagnostic code coverage patch applied")
