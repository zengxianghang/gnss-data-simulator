from pathlib import Path

path = Path("tests/integration/test_all_signal_residuals.cpp")
text = path.read_text()

old_code = '''            rtklib_signal_bias_info_ext_t bias_info{};
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
'''
new_code = '''            rtklib_signal_bias_info_ext_t bias_info{};
            double code_residual_m = 0.0;
            int code_status =
                rtklib_rescode_signal_ext(&observation, &nav, &residual_options, receiver_position_m, 0.0, 0.0,
                                          required_message_type, wavelength_m, &code_residual_m, nullptr, &bias_info);
            const bool family_unavailable =
                fields[column.at("code_bias_status")] == "UNAVAILABLE_FOR_MESSAGE_FAMILY";
            const bool gps_l1c_developmental = signal_name == "GPS L1C";
            const bool gps_l5_preoperational = signal_name == "GPS L5Q";
            const bool pseudorange_valid = fields[column.at("pseudorange_valid")] == "1";
            const bool doppler_valid = fields[column.at("doppler_valid")] == "1";

            if (gps_l5_preoperational && !family_unavailable && pseudorange_valid) {
                // L5 CNAV is intentionally broadcast unhealthy while pre-operational.
                // Raw RANGE validity is independent of that navigation-health flag,
                // but strict RTKLIB residual use must still reject it.
                ASSERT_EQ(code_status, 0)
                    << "strict code residual must preserve L5 broadcast-health exclusion";
                code_status = rtklib_rescode_signal_diagnostic_ext(
                    &observation, &nav, &residual_options, receiver_position_m, 0.0, 0.0, required_message_type,
                    wavelength_m, &code_residual_m, nullptr, &bias_info);
                ASSERT_EQ(code_status, 1) << "diagnostic L5Q code residual failed; site=" << site.name
                                          << " sat=" << static_cast<int>(observation.sat);
                ++signal_stats.code_residuals;
                signal_stats.max_abs_code_m = (std::max)(signal_stats.max_abs_code_m, std::fabs(code_residual_m));
            } else if (pseudorange_valid) {
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

            if (gps_l1c_developmental && doppler_valid) {
                // GPS L1C currently carries no CNAV-2 navigation data. Code
                // bias therefore remains unavailable, but Doppler needs no
                // signal-specific code bias and can be checked against the
                // same-satellite generic broadcast orbit/clock state.
                double doppler_residual_mps = 0.0;
                const int doppler_status =
                    rtklib_resdop_signal_ext(&observation, &nav, &residual_options, receiver_position_m,
                                             receiver_velocity_mps, 0.0, 0, wavelength_m, &doppler_residual_mps,
                                             nullptr);
                ASSERT_EQ(doppler_status, 1) << "generic-state L1C Doppler residual failed; site=" << site.name
                                             << " sat=" << static_cast<int>(observation.sat);
                ++signal_stats.doppler_residuals;
                signal_stats.max_abs_doppler_mps =
                    (std::max)(signal_stats.max_abs_doppler_mps, std::fabs(doppler_residual_mps));
            } else if (gps_l5_preoperational && doppler_valid) {
                double doppler_residual_mps = 0.0;
                const int doppler_status = rtklib_resdop_signal_diagnostic_ext(
                    &observation, &nav, &residual_options, receiver_position_m, receiver_velocity_mps, 0.0, 0,
                    wavelength_m, &doppler_residual_mps, nullptr);
                ASSERT_EQ(doppler_status, 1) << "diagnostic L5Q Doppler residual failed; site=" << site.name
                                             << " sat=" << static_cast<int>(observation.sat);
                ++signal_stats.doppler_residuals;
                signal_stats.max_abs_doppler_mps =
                    (std::max)(signal_stats.max_abs_doppler_mps, std::fabs(doppler_residual_mps));
            } else if (doppler_valid) {
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
'''
if text.count(old_code) != 1:
    raise RuntimeError(f"diagnostic residual body anchor count={text.count(old_code)}")
text = text.replace(old_code, new_code, 1)

old_expectation = '''        } else if (signal_name == "GPS L1C" || signal_name == "GPS L2C" || signal_name == "GPS L5Q") {
            // Diagnostic phase: compact GPS modern-family health/fixture
            // suitability is resolved before restoring strict expectations.
        } else {'''
new_expectation = '''        } else if (signal_name == "GPS L1C") {
            EXPECT_EQ(signal_stats.code_residuals, 0U)
                << "GPS L1C code must remain unavailable while no real CNAV-2 navigation data are broadcast";
            EXPECT_GT(signal_stats.code_unavailable, 0U);
        } else {'''
if text.count(old_expectation) != 1:
    raise RuntimeError(f"GPS modern expectation anchor count={text.count(old_expectation)}")
text = text.replace(old_expectation, new_expectation, 1)

old_union = '''    std::fprintf(stderr, "CODE_COVERAGE_UNION covered=%zu total=21\\n", code_covered_signal_count);'''
new_union = '''    std::fprintf(stderr, "CODE_COVERAGE_UNION covered=%zu total=21\\n", code_covered_signal_count);
    EXPECT_EQ(code_covered_signal_count, 19U)
        << "only developmental GPS L1C and Galileo E6/HAS may remain code-unavailable";'''
if text.count(old_union) != 1:
    raise RuntimeError(f"code coverage union anchor count={text.count(old_union)}")
text = text.replace(old_union, new_union, 1)

path.write_text(text)
print("GPS developmental/pre-operational residual policy patch applied")
