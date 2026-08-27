from pathlib import Path

path = Path("tests/integration/test_all_signal_residuals.cpp")
text = path.read_text()
old = '''                rtklib_resdop_signal_ext(&observation, &nav, &residual_options, receiver_position_m,
                                         receiver_velocity_mps, 0.0, required_message_type, wavelength_m,
                                         &doppler_residual_mps, nullptr);'''
new = '''                rtklib_resdop_signal_ext(&observation, &nav, &residual_options, receiver_position_m,
                                         receiver_velocity_mps, 0.0, 0, wavelength_m, &doppler_residual_mps,
                                         nullptr);'''
count = text.count(old)
if count != 1:
    raise RuntimeError(f"test_all_signal_residuals.cpp: expected staged Doppler call once, found {count}")
path.write_text(text.replace(old, new, 1))
print("Doppler validator now follows simulator generic broadcast-state selection")
