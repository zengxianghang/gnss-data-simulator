from pathlib import Path

path = Path("src/core/simulator.cpp")
text = path.read_text()
old = '''            const bool signal_available =
                scenario.signal_available && geometry.above_elevation_mask && signal_healthy;
            if (signal_available && !signal.tracker.scheduled) {'''
new = '''            // Broadcast health controls whether the measurement is usable, not
            // whether the RF signal can be acquired/tracked. Preserve legacy
            // behavior outside the modern GPS families while allowing an
            // unhealthy/absent CNAV/CNV2 signal to remain observable in RANGE.
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
if text.count(old) != 1:
    raise RuntimeError(f"signal availability anchor count={text.count(old)}")
text = text.replace(old, new, 1)
old_measurement = '''            if (!generate_zero_noise_measurement(truth_nav, geometry, signal.tracker, atmosphere, &signal.ambiguity,
                                                 &observation, error_message) ||
                !truth_writer_write_observation(truth_writer, runtime->receiver, geometry, signal.tracker, observation,
                                                error_message)) {'''
new_measurement = '''            if (!generate_zero_noise_measurement(truth_nav, signal_geometry, signal.tracker, atmosphere,
                                                 &signal.ambiguity, &observation, error_message) ||
                !truth_writer_write_observation(truth_writer, runtime->receiver, signal_geometry, signal.tracker,
                                                observation, error_message)) {'''
if text.count(old_measurement) != 1:
    raise RuntimeError(f"signal geometry measurement anchor count={text.count(old_measurement)}")
text = text.replace(old_measurement, new_measurement, 1)
path.write_text(text)
print("modern GPS tracking separated from signal-specific broadcast-health validity")
