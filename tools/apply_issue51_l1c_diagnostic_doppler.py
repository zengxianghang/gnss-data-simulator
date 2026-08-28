from pathlib import Path

path = Path("tests/integration/test_all_signal_residuals.cpp")
text = path.read_text()

old = '''            if (gps_l1c_developmental && doppler_valid) {
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
'''
new = '''            if (gps_l1c_developmental && doppler_valid) {
                // GPS L1C currently carries no CNAV-2 navigation data. Code
                // bias therefore remains unavailable. Doppler needs no
                // observable-specific code bias, so validate against a
                // same-satellite generic orbit/clock state. The generic EPH
                // health bit is not an L1C/CNAV-2 health observation, so use
                // the diagnostic API to ignore broadcast health only; navsys
                // and explicit exsats exclusions remain enforced by RTKLIB.
                double doppler_residual_mps = 0.0;
                const int doppler_status = rtklib_resdop_signal_diagnostic_ext(
                    &observation, &nav, &residual_options, receiver_position_m, receiver_velocity_mps, 0.0, 0,
                    wavelength_m, &doppler_residual_mps, nullptr);
                ASSERT_EQ(doppler_status, 1) << "generic diagnostic L1C Doppler residual failed; site=" << site.name
                                             << " sat=" << static_cast<int>(observation.sat);
'''
if text.count(old) != 1:
    raise RuntimeError(f"L1C Doppler anchor count={text.count(old)}")
path.write_text(text.replace(old, new, 1))
print("GPS L1C Doppler now uses generic diagnostic broadcast state")
