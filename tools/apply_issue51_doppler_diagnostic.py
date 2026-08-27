from pathlib import Path

path = Path("tests/integration/test_all_signal_residuals.cpp")
text = path.read_text()
include_old = "#include <cmath>\n#include <cstdint>\n"
include_new = "#include <cmath>\n#include <cstdio>\n#include <cstdint>\n"
if text.count(include_old) != 1:
    raise RuntimeError("test_all_signal_residuals.cpp: expected include insertion point once")
text = text.replace(include_old, include_new, 1)
old = '''            ASSERT_EQ(doppler_status, 1) << "signal=" << signal_name << " sat=" << static_cast<int>(observation.sat);'''
new = '''            if (doppler_status != 1) {
                double debug_rs[6]{};
                double debug_dts[2]{};
                double debug_var = 0.0;
                int debug_svh = 0;
                rtklib_signal_bias_info_ext_t debug_info{};
                const int debug_state_status =
                    rtklib_signal_state_ext(observation.time, observation.P[0], observation.sat, observation.code[0], 0,
                                            &nav, debug_rs, debug_dts, &debug_var, &debug_svh, &debug_info);
                double debug_e[3]{};
                double debug_pos[3]{};
                double debug_azel[2]{};
                double debug_range = 0.0;
                double debug_elevation = -1.0;
                int debug_excluded = -1;
                if (debug_state_status > 0) {
                    debug_range = geodist(debug_rs, receiver_position_m, debug_e);
                    ecef2pos(receiver_position_m, debug_pos);
                    if (debug_range > 0.0) {
                        debug_elevation = satazel(debug_pos, debug_e, debug_azel);
                    }
                    debug_excluded = satexclude(observation.sat, debug_svh, &residual_options);
                }
                std::fprintf(stderr,
                             "DOPPLER_DEBUG signal=%s sat=%d code_status=%d required_type=%d state=%d selected_type=%d "
                             "speed=%.15f range=%.3f elevation=%.15f svh=%d excluded=%d D=%.9f\\n",
                             signal_name.c_str(), static_cast<int>(observation.sat), code_status,
                             required_message_type, debug_state_status, debug_info.message_type, norm(debug_rs + 3, 3),
                             debug_range, debug_elevation, debug_svh, debug_excluded,
                             static_cast<double>(observation.D[0]));
            }
            ASSERT_EQ(doppler_status, 1) << "signal=" << signal_name << " sat=" << static_cast<int>(observation.sat);'''
count = text.count(old)
if count != 1:
    raise RuntimeError(f"test_all_signal_residuals.cpp: expected Doppler assertion once, found {count}")
path.write_text(text.replace(old, new, 1))
print("Doppler early-return diagnostic injected")
