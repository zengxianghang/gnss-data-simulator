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
            const bool gps_preoperational = signal_name == "GPS L1C" || signal_name == "GPS L5Q";
            if (fields[column.at("pseudorange_valid")] == "1") {
                ASSERT_EQ(code_status, 1) << "site=" << site.name << " signal=" << signal_name
                                          << " sat=" << static_cast<int>(observation.sat);
                ++signal_stats.code_residuals;
                signal_stats.max_abs_code_m = (std::max)(signal_stats.max_abs_code_m, std::fabs(code_residual_m));
            } else if (gps_preoperational && !family_unavailable) {
                ASSERT_EQ(code_status, 0)
                    << "strict code residual must preserve broadcast-health exclusion for " << signal_name;
                ++signal_stats.code_unavailable;
                code_status = rtklib_rescode_signal_diagnostic_ext(
                    &observation, &nav, &residual_options, receiver_position_m, 0.0, 0.0, required_message_type,
                    wavelength_m, &code_residual_m, nullptr, &bias_info);
                ASSERT_EQ(code_status, 1) << "diagnostic code residual failed for real pre-operational signal; site="
                                          << site.name << " signal=" << signal_name
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
            } else if (gps_preoperational && !family_unavailable) {
                double doppler_residual_mps = 0.0;
                const int strict_doppler_status =
                    rtklib_resdop_signal_ext(&observation, &nav, &residual_options, receiver_position_m,
                                             receiver_velocity_mps, 0.0, required_message_type, wavelength_m,
                                             &doppler_residual_mps, nullptr);
                ASSERT_EQ(strict_doppler_status, 0)
                    << "strict Doppler residual must preserve broadcast-health exclusion for " << signal_name;
                const int diagnostic_doppler_status = rtklib_resdop_signal_diagnostic_ext(
                    &observation, &nav, &residual_options, receiver_position_m, receiver_velocity_mps, 0.0,
                    required_message_type, wavelength_m, &doppler_residual_mps, nullptr);
                ASSERT_EQ(diagnostic_doppler_status, 1)
                    << "diagnostic Doppler residual failed for real pre-operational signal; site=" << site.name
                    << " signal=" << signal_name << " sat=" << static_cast<int>(observation.sat);
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
new_expectation = '''        } else if (signal_name == "GPS L1C" || signal_name == "GPS L2C" || signal_name == "GPS L5Q") {
            EXPECT_GT(signal_stats.code_residuals, 0U)
                << "GPS modern signal must close through strict or explicitly diagnostic residual validation: "
                << signal_name;
        } else {'''
if text.count(old_expectation) != 1:
    raise RuntimeError(f"GPS modern expectation anchor count={text.count(old_expectation)}")
text = text.replace(old_expectation, new_expectation, 1)

path.write_text(text)
print("GPS pre-operational diagnostic residual validation patch applied")
