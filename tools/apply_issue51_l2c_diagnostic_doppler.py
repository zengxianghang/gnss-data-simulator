from pathlib import Path

path = Path("tests/integration/test_all_signal_residuals.cpp")
text = path.read_text()

old = '''            } else if (gps_l5_preoperational && doppler_valid) {
                double doppler_residual_mps = 0.0;
                const int doppler_status = rtklib_resdop_signal_diagnostic_ext(
'''
new = '''            } else if ((signal_name == "GPS L2C" || gps_l5_preoperational) && doppler_valid) {
                // GPS modern Doppler uses the same-satellite generic orbit/clock
                // state and does not consume a code bias.  The health attached
                // to whichever generic ephemeris wins selection is not a
                // reliable per-signal L2C/L5Q health observation, so ignore
                // broadcast health for this diagnostic residual only.
                double doppler_residual_mps = 0.0;
                const int doppler_status = rtklib_resdop_signal_diagnostic_ext(
'''
if text.count(old) != 1:
    raise RuntimeError(f"L2C/L5Q Doppler anchor count={text.count(old)}")
path.write_text(text.replace(old, new, 1))
print("GPS L2C/L5Q Doppler now uses generic diagnostic broadcast state")
