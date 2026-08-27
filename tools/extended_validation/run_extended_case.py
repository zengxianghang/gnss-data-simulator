#!/usr/bin/env python3
"""Exact epoch-count front-end for the extended validation harness."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import run_extended_validation as core


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
        return ["run_manifest.json/run_summary missing"]
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", type=Path)
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    parser.add_argument("--case", choices=core.ALL_CASES)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    core.validate = validate_exact

    if args.self_test:
        self_test()
        print("extended exact-count self-test: PASS")
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
