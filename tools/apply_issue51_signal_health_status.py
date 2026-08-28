from pathlib import Path

bias_path = Path("src/gnss/rtklib_bias_adapter.cpp")
bias_text = bias_path.read_text()
old_status = '''    if (status <= 0) {
        set_error(error_message, "no matching signal/message-family ephemeris for health");
        return false;
    }

    const int raw_health = info.system == SYS_GLO ? geph.svh : eph.svh;
'''
new_status = '''    if (status == 0) {
        *signal_health = 1;
        return true;
    }
    if (status < 0) {
        set_error(error_message, "signal/message-family status lookup failed");
        return false;
    }

    const int raw_health = info.system == SYS_GLO ? geph.svh : eph.svh;
'''
if bias_text.count(old_status) != 1:
    raise RuntimeError(f"signal status anchor count={bias_text.count(old_status)}")
bias_path.write_text(bias_text.replace(old_status, new_status, 1))

sim_path = Path("src/core/simulator.cpp")
sim_text = sim_path.read_text()
old_availability = '''            const bool signal_available =
                scenario.signal_available && geometry.above_elevation_mask && signal_healthy;
            if (signal_available && !signal.tracker.scheduled) {'''
new_availability = '''            // Broadcast health controls measurement validity, not RF tracking.
            // Preserve legacy behavior outside GPS CNAV/CNV2.
            const bool signal_available =
                scenario.signal_available &&
                (health_family != RtklibBroadcastMessageFamily::kUnknown ? geometry.above_elevation_mask
                                                                         : geometry.visible);
            SatelliteGeometry signal_geometry = geometry;
            if (health_family != RtklibBroadcastMessageFamily::kUnknown) {
                signal_geometry.healthy = signal_healthy;
                signal_geometry.visible = geometry.above_elevation_mask && signal_healthy;
            }
            if (signal_available && !signal.tracker.scheduled) {'''
if sim_text.count(old_availability) != 1:
    raise RuntimeError(f"signal availability anchor count={sim_text.count(old_availability)}")
sim_text = sim_text.replace(old_availability, new_availability, 1)

old_generate = "generate_zero_noise_measurement(truth_nav, geometry,"
new_generate = "generate_zero_noise_measurement(truth_nav, signal_geometry,"
if sim_text.count(old_generate) != 1:
    raise RuntimeError(f"measurement geometry anchor count={sim_text.count(old_generate)}")
sim_text = sim_text.replace(old_generate, new_generate, 1)

old_truth = "truth_writer_write_observation(truth_writer, runtime->receiver, geometry,"
new_truth = "truth_writer_write_observation(truth_writer, runtime->receiver, signal_geometry,"
if sim_text.count(old_truth) != 1:
    raise RuntimeError(f"truth geometry anchor count={sim_text.count(old_truth)}")
sim_path.write_text(sim_text.replace(old_truth, new_truth, 1))

print("missing family is non-fatal and GPS modern RF tracking is separated from broadcast-health validity")
