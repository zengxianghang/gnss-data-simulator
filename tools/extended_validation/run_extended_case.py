#!/usr/bin/env python3
"""Exact-count and continuous-NAV front-end for extended validation."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

import run_extended_validation as core

MU_GPS = 3.9860050e14


def periodic_on_epochs(total_epochs: int, rate_hz: int, on_sec: float, off_sec: float) -> int:
    if rate_hz <= 0 or core.NS % rate_hz != 0:
        raise ValueError("extended validation requires an integer-nanosecond sampling interval")
    step_ns = core.NS // rate_hz
    on_ns = round(on_sec * core.NS)
    off_ns = round(off_sec * core.NS)
    period_ns = on_ns + off_ns
    if on_ns <= 0 or off_ns < 0 or period_ns <= 0 or period_ns % step_ns != 0:
        raise ValueError("extended scenario period must align to the sampling interval")
    period_epochs = period_ns // step_ns
    full_cycles, remainder = divmod(total_epochs, period_epochs)
    on_per_cycle = sum(1 for index in range(period_epochs) if index * step_ns < on_ns)
    on_remainder = sum(1 for index in range(remainder) if index * step_ns < on_ns)
    return full_cycles * on_per_cycle + on_remainder


def expected_epoch_counts(config):
    rate_hz = int(config["sampling_rate_hz"])
    scheduled = int(config["duration_sec"]) * rate_hz
    scenario = config["scenario"]
    if scenario == "KS":
        powered = scheduled
        signal_on = scheduled
    elif scenario == "REA":
        powered = scheduled
        signal_on = periodic_on_epochs(
            scheduled,
            rate_hz,
            float(config["rea"]["signal_on_sec"]),
            float(config["rea"]["signal_off_sec"]),
        )
    elif scenario == "TTFF":
        powered = periodic_on_epochs(
            scheduled,
            rate_hz,
            float(config["ttff"]["power_on_sec"]),
            float(config["ttff"]["power_off_sec"]),
        )
        signal_on = powered
    else:
        raise ValueError(f"unsupported scenario: {scenario}")
    return {
        "scheduled_epochs": scheduled,
        "powered_epochs": powered,
        "signal_on_epochs": signal_on,
        "signal_off_epochs": scheduled - signal_on,
        "range_messages": powered,
        "psrpos_messages": powered,
        "psrvel_messages": powered,
    }


def validate_exact(run, config):
    failures = []
    manifest = run.get("manifest")
    if not isinstance(manifest, dict) or not isinstance(manifest.get("run_summary"), dict):
        stderr = run.get("stderr_tail", "")
        suffix = f"; stderr: {stderr}" if stderr else ""
        return [f"run_manifest.json/run_summary missing{suffix}"]
    summary = manifest["run_summary"]

    for key, expected in expected_epoch_counts(config).items():
        actual = int(summary.get(key, -1))
        if actual != expected:
            failures.append(f"{key}: expected {expected}, got {actual}")

    for key, expected in core.expected_events(config).items():
        actual = int(run.get("event_counts", {}).get(key, 0))
        if actual != expected:
            failures.append(f"{key}: expected {expected}, got {actual}")

    if run.get("exit_code") != 0:
        failures.append(f"simulator exit code {run.get('exit_code')}")
    if int(run.get("peak_rss_kib", 0)) <= 0:
        failures.append("peak RSS unavailable")
    failures.extend(run.get("capture_errors", []))
    return failures


def field_value(line: str, first_column: int, field_index: int) -> float:
    start = first_column + field_index * 19
    end = start + 19
    if len(line.rstrip("\n")) < end:
        raise ValueError("short RINEX NAV field")
    return float(line[start:end].replace("D", "E"))


def replace_field(line: str, first_column: int, field_index: int, value: float) -> str:
    start = first_column + field_index * 19
    end = start + 19
    if len(line.rstrip("\n")) < end:
        raise ValueError("short RINEX NAV field")
    return line[:start] + f"{value:19.12E}" + line[end:]


def shift_gps_ephemeris_reference(base, toe: int, week: int):
    """Move a GPS LNAV reference epoch without introducing an orbit/clock jump.

    RTKLIB evaluates M from M0+n*tk, inclination from i0+IDOT*tk and
    longitude of ascending node from OMG0+(OMGd-OMGE)*tk-OMGE*toes.
    Therefore moving toe by dt requires M0+=n*dt, i0+=IDOT*dt and
    OMG0+=OMGd*dt. The clock polynomial is shifted to the new Toc as well.
    """
    rec = list(base)
    base_toe = field_value(base[3], 4, 0)
    delta = float(toe) - base_toe

    sqrt_a = field_value(base[2], 4, 3)
    semi_major_axis = sqrt_a * sqrt_a
    delta_n = field_value(base[1], 4, 2)
    mean_motion = math.sqrt(MU_GPS / (semi_major_axis**3)) + delta_n

    m0 = field_value(base[1], 4, 3)
    omega0 = field_value(base[3], 4, 2)
    i0 = field_value(base[4], 4, 0)
    omega_dot = field_value(base[4], 4, 3)
    i_dot = field_value(base[5], 4, 0)
    f0 = field_value(base[0], 23, 0)
    f1 = field_value(base[0], 23, 1)
    f2 = field_value(base[0], 23, 2)

    epoch = core.GPS_EPOCH + core.dt.timedelta(weeks=week, seconds=toe)
    prefix = (
        f"{rec[0][:3]} {epoch.year:04d} {epoch.month:02d} {epoch.day:02d} "
        f"{epoch.hour:02d} {epoch.minute:02d} {epoch.second:02d}"
    )
    rec[0] = prefix + rec[0][23:]
    rec[0] = replace_field(rec[0], 23, 0, f0 + f1 * delta + f2 * delta * delta)
    rec[0] = replace_field(rec[0], 23, 1, f1 + 2.0 * f2 * delta)
    rec[1] = replace_field(rec[1], 4, 3, m0 + mean_motion * delta)
    rec[3] = replace_field(rec[3], 4, 0, float(toe))
    rec[3] = replace_field(rec[3], 4, 2, omega0 + omega_dot * delta)
    rec[4] = replace_field(rec[4], 4, 0, i0 + i_dot * delta)
    rec[5] = replace_field(rec[5], 4, 2, float(week))
    rec[7] = replace_field(rec[7], 4, 0, float(toe))
    return rec


def materialize_continuous_long_gps_nav(source, destination, week, start_sow, duration_sec):
    header, records = core.split_gps_rinex(source)
    interval = 3600
    first_toe = (start_sow // interval) * interval
    last_toe = start_sow + duration_sec + interval
    out = list(header)
    for toe in range(first_toe, last_toe + 1, interval):
        if not 0 <= toe < 604800:
            raise ValueError("long GPS NAV fixture currently stays within one GPS week")
        for base in records:
            out.extend(shift_gps_ephemeris_reference(base, toe, week))
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("".join(out), encoding="ascii", newline="\n")


def self_test():
    core.self_test()

    ks = core.base_config("KS", 10, 5)
    assert expected_epoch_counts(ks) == {
        "scheduled_epochs": 50,
        "powered_epochs": 50,
        "signal_on_epochs": 50,
        "signal_off_epochs": 0,
        "range_messages": 50,
        "psrpos_messages": 50,
        "psrvel_messages": 50,
    }

    rea = core.base_config("REA", 6, 10)
    rea["rea"] = {"signal_on_sec": 2.0, "signal_off_sec": 1.0}
    counts = expected_epoch_counts(rea)
    assert counts["scheduled_epochs"] == 60
    assert counts["powered_epochs"] == 60
    assert counts["signal_on_epochs"] == 40
    assert counts["signal_off_epochs"] == 20
    assert counts["range_messages"] == 60

    ttff = core.base_config("TTFF", 8, 10)
    ttff["ttff"] = {"startup_mode": "HOT", "power_on_sec": 3.0, "power_off_sec": 1.0}
    counts = expected_epoch_counts(ttff)
    assert counts["scheduled_epochs"] == 80
    assert counts["powered_epochs"] == 60
    assert counts["signal_on_epochs"] == 60
    assert counts["signal_off_epochs"] == 20
    assert counts["range_messages"] == 60

    # Verify the reference-epoch transform itself instead of merely checking
    # that the materializer emits syntactically valid RINEX.
    sample = [
        "G01 2023 03 14 00 00 00 1.000000000000E-04 2.000000000000E-12 3.000000000000E-20\n",
        "     1.000000000000E+00 0.000000000000E+00 4.000000000000E-09 5.000000000000E-01\n",
        "     0.000000000000E+00 1.000000000000E-02 0.000000000000E+00 5.153795477500E+03\n",
        "     1.728000000000E+05 0.000000000000E+00 1.000000000000E+00 0.000000000000E+00\n",
        "     9.500000000000E-01 0.000000000000E+00 2.000000000000E+00-8.000000000000E-09\n",
        "     1.000000000000E-10 0.000000000000E+00 2.253000000000E+03 0.000000000000E+00\n",
        "     2.000000000000E+00 0.000000000000E+00 0.000000000000E+00 1.000000000000E+00\n",
        "     1.728000000000E+05 4.000000000000E+00 0.000000000000E+00 0.000000000000E+00\n",
    ]
    shifted = shift_gps_ephemeris_reference(sample, 176400, 2253)
    delta = 3600.0
    a = field_value(sample[2], 4, 3) ** 2
    n = math.sqrt(MU_GPS / (a**3)) + field_value(sample[1], 4, 2)
    assert math.isclose(field_value(shifted[1], 4, 3), 0.5 + n * delta, rel_tol=0.0, abs_tol=1e-11)
    assert math.isclose(field_value(shifted[3], 4, 2), 1.0 - 8.0e-9 * delta, rel_tol=0.0, abs_tol=1e-11)
    assert math.isclose(field_value(shifted[4], 4, 0), 0.95 + 1.0e-10 * delta, rel_tol=0.0, abs_tol=1e-11)
    assert math.isclose(field_value(shifted[0], 23, 0), 1.0e-4 + 2.0e-12 * delta + 3.0e-20 * delta**2,
                        rel_tol=0.0, abs_tol=1e-15)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", type=Path)
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    parser.add_argument("--case", choices=core.ALL_CASES)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    core.validate = validate_exact
    core.materialize_long_gps_nav = materialize_continuous_long_gps_nav

    if args.self_test:
        self_test()
        print("extended exact-count/continuous-NAV self-test: PASS")
        return 0
    if args.simulator is None or args.case is None or args.output_dir is None:
        parser.error("--simulator, --case and --output-dir are required unless --self-test is used")
    if os.name != "posix" or not Path("/proc").exists():
        raise SystemExit("extended validation requires a Linux/POSIX runner with /proc")

    simulator = args.simulator.resolve()
    if not simulator.is_file():
        raise SystemExit(f"simulator executable not found: {simulator}")
    return 0 if core.run_case(simulator, args.repo_root.resolve(), args.case, args.output_dir.resolve()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
