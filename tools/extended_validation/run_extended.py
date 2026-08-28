#!/usr/bin/env python3
"""Bounded long-duration/resource validation for gnss-data-simulator."""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time

NS = 1_000_000_000
MEMORY_HEADROOM_KIB = 64 * 1024
FIFO_NAMES = ("simulated.log", "event_truth.csv", "observation_truth.csv", "solution_truth.csv")
REGULAR_NAMES = ("scenario.json", "run_manifest.json")
ALL_CASES = (
    "ks_8h",
    "rea_8h",
    "ttff_8h",
    "stress_50hz_1h",
    "determinism_50hz_10m",
    "memory_trend_50hz",
)

PINNED_REAL_NAV_FILENAME = "BRD400DLR_S_20250030000_01D_MN.rnx"
PINNED_REAL_NAV_SHA256 = "b11c638eea42978b8bd6aa8b65a5099fe6556dfe527bc037ed481d2b239afc42"
PINNED_REAL_NAV_SOURCE_URL = (
    "ftp://igs.gnsswhu.cn/pub/gps/data/daily/2025/brdc/"
    "BRD400DLR_S_20250030000_01D_MN.rnx.gz"
)
PINNED_REAL_NAV_WEEK = 2347
PINNED_REAL_NAV_SOW = 436500


def base_config(scenario: str, duration_sec: int, rate_hz: int) -> dict:
    return {
        "schema_version": 1,
        "scenario": scenario,
        "duration_sec": duration_sec,
        "sampling_rate_hz": rate_hz,
        "elevation_mask_deg": 0.0,
        "output_eph": True,
        "output_ion": True,
        "measurement_noise_enabled": False,
        "multipath_enabled": False,
        "receiver_clock_bias_m": 0.0,
        "receiver_clock_drift_mps": 0.0,
        "atmosphere_mode": "none",
        "receiver": {"latitude_deg": 20.0, "longitude_deg": 120.0, "height_m": 100.0},
        "ttff": {"startup_mode": "HOT", "power_on_sec": 300.0, "power_off_sec": 30.0},
        "rea": {"signal_on_sec": 300.0, "signal_off_sec": 10.0},
        "seed": 1,
    }


def case_definition(name: str):
    if name == "ks_8h":
        return base_config("KS", 28800, 10), "real_full_day", PINNED_REAL_NAV_WEEK, PINNED_REAL_NAV_SOW
    if name == "rea_8h":
        return base_config("REA", 28800, 10), "real_full_day", PINNED_REAL_NAV_WEEK, PINNED_REAL_NAV_SOW
    if name == "ttff_8h":
        return base_config("TTFF", 28800, 10), "real_full_day", PINNED_REAL_NAV_WEEK, PINNED_REAL_NAV_SOW
    if name == "stress_50hz_1h":
        return base_config("KS", 3600, 50), "real_full_day", PINNED_REAL_NAV_WEEK, PINNED_REAL_NAV_SOW
    if name == "determinism_50hz_10m":
        return base_config("KS", 600, 50), "brd4", PINNED_REAL_NAV_WEEK, PINNED_REAL_NAV_SOW
    if name == "memory_trend_50hz":
        return base_config("KS", 900, 50), "brd4", PINNED_REAL_NAV_WEEK, PINNED_REAL_NAV_SOW
    raise ValueError(f"unknown extended case: {name}")


class Capture:
    def __init__(self, path: Path, retain: bool = False):
        self.path = path
        self.retain = retain
        self.size = 0
        self.hash = hashlib.sha256()
        self.data = bytearray()
        self.error = ""
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        try:
            with self.path.open("rb", buffering=0) as source:
                while True:
                    chunk = source.read(1024 * 1024)
                    if not chunk:
                        break
                    self.size += len(chunk)
                    self.hash.update(chunk)
                    if self.retain:
                        self.data.extend(chunk)
        except Exception as exc:  # diagnostic path
            self.error = f"{type(exc).__name__}: {exc}"


def rss_kib(pid: int) -> int:
    try:
        for line in Path(f"/proc/{pid}/status").read_text(encoding="ascii").splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1])
    except (FileNotFoundError, ProcessLookupError, ValueError):
        pass
    return 0


def hash_file(path: Path) -> dict:
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as source:
        while True:
            chunk = source.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
            size += len(chunk)
    return {"sha256": digest.hexdigest(), "size_bytes": size}


def resolve_nav(repo: Path, nav_kind: str, real_nav: Path | None) -> tuple[Path, dict]:
    if nav_kind == "brd4":
        nav = repo / "tests/data/minimal/brd400dlr_rinex4_acceptance_nav.rnx"
        if not nav.is_file():
            raise ValueError(f"real RINEX 4 fixture is missing: {nav}")
        identity = hash_file(nav)
        identity.update({"source": "committed real WHU RINEX subset", "pinned_full_day": False})
        return nav, identity

    if nav_kind != "real_full_day":
        raise ValueError(f"unsupported NAV kind: {nav_kind}")
    if real_nav is None:
        raise ValueError(
            "long-duration validation requires --real-nav pointing to the unmodified real RINEX NAV file; "
            f"download {PINNED_REAL_NAV_SOURCE_URL} instead of synthesizing ephemeris"
        )

    nav = real_nav.resolve()
    if not nav.is_file():
        raise ValueError(f"real RINEX NAV file is missing: {nav}")
    identity = hash_file(nav)
    if identity["sha256"].lower() != PINNED_REAL_NAV_SHA256:
        raise ValueError(
            "real RINEX NAV SHA256 mismatch: expected "
            f"{PINNED_REAL_NAV_SHA256}, got {identity['sha256']}; "
            "do not alter or regenerate navigation records"
        )
    identity.update(
        {
            "source": PINNED_REAL_NAV_SOURCE_URL,
            "source_filename": PINNED_REAL_NAV_FILENAME,
            "pinned_full_day": True,
        }
    )
    return nav, identity


def tail_text(path: Path, limit: int = 4096) -> str:
    if not path.exists():
        return ""
    with path.open("rb") as source:
        source.seek(0, os.SEEK_END)
        size = source.tell()
        source.seek(max(0, size - limit))
        return source.read().decode("utf-8", errors="replace").strip()


def parse_events(data: bytes) -> dict:
    counts = {}
    for row in csv.DictReader(io.StringIO(data.decode("utf-8"))):
        kind = row.get("event_type", "")
        if kind:
            counts[kind] = counts.get(kind, 0) + 1
    return counts


def boundary_count(duration_ns: int, first_ns: int, period_ns: int) -> int:
    if first_ns >= duration_ns:
        return 0
    return 1 + (duration_ns - 1 - first_ns) // period_ns


def expected_events(config: dict) -> dict:
    duration_ns = int(config["duration_sec"]) * NS
    expected = {"POWER_ON": 1, "POWER_OFF": 0, "SIGNAL_ON": 1, "SIGNAL_OFF": 0}
    if config["scenario"] == "REA":
        on_ns = round(float(config["rea"]["signal_on_sec"]) * NS)
        off_ns = round(float(config["rea"]["signal_off_sec"]) * NS)
        period_ns = on_ns + off_ns
        expected["SIGNAL_OFF"] = boundary_count(duration_ns, on_ns, period_ns)
        expected["SIGNAL_ON"] += boundary_count(duration_ns, period_ns, period_ns)
    elif config["scenario"] == "TTFF":
        on_ns = round(float(config["ttff"]["power_on_sec"]) * NS)
        off_ns = round(float(config["ttff"]["power_off_sec"]) * NS)
        period_ns = on_ns + off_ns
        off_count = boundary_count(duration_ns, on_ns, period_ns)
        on_count = boundary_count(duration_ns, period_ns, period_ns)
        expected["POWER_OFF"] = off_count
        expected["SIGNAL_OFF"] = off_count
        expected["POWER_ON"] += on_count
        expected["SIGNAL_ON"] += on_count
    elif config["scenario"] != "KS":
        raise ValueError(f"unsupported scenario: {config['scenario']}")
    return expected


def periodic_on_epochs(total_epochs: int, rate_hz: int, on_sec: float, off_sec: float) -> int:
    if rate_hz <= 0 or NS % rate_hz != 0:
        raise ValueError("extended validation requires an integer-nanosecond sampling interval")
    step_ns = NS // rate_hz
    on_ns = round(on_sec * NS)
    off_ns = round(off_sec * NS)
    period_ns = on_ns + off_ns
    if on_ns <= 0 or off_ns < 0 or period_ns <= 0 or period_ns % step_ns != 0 or on_ns % step_ns != 0:
        raise ValueError("extended scenario period must align to the sampling interval")
    period_epochs = period_ns // step_ns
    on_epochs = on_ns // step_ns
    full_cycles, remainder = divmod(total_epochs, period_epochs)
    return full_cycles * on_epochs + min(remainder, on_epochs)


def expected_epoch_counts(config: dict) -> dict:
    rate_hz = int(config["sampling_rate_hz"])
    scheduled = int(config["duration_sec"]) * rate_hz
    scenario = config["scenario"]
    if scenario == "KS":
        powered = scheduled
        signal_on = scheduled
        signal_off = 0
    elif scenario == "REA":
        powered = scheduled
        signal_on = periodic_on_epochs(
            scheduled,
            rate_hz,
            float(config["rea"]["signal_on_sec"]),
            float(config["rea"]["signal_off_sec"]),
        )
        signal_off = scheduled - signal_on
    elif scenario == "TTFF":
        powered = periodic_on_epochs(
            scheduled,
            rate_hz,
            float(config["ttff"]["power_on_sec"]),
            float(config["ttff"]["power_off_sec"]),
        )
        signal_on = powered
        signal_off = 0
    else:
        raise ValueError(f"unsupported scenario: {scenario}")
    return {
        "scheduled_epochs": scheduled,
        "powered_epochs": powered,
        "signal_on_epochs": signal_on,
        "signal_off_epochs": signal_off,
        "range_messages": powered,
        "psrpos_messages": powered,
        "psrvel_messages": powered,
    }


def run_once(simulator: Path, nav: Path, config: dict, week: int, sow: int, run_dir: Path) -> dict:
    if run_dir.exists():
        shutil.rmtree(run_dir)
    run_dir.mkdir(parents=True)
    (run_dir / "config.json").write_text(json.dumps(config, indent=2, sort_keys=True) + "\n")

    captures = {}
    for name in FIFO_NAMES:
        path = run_dir / name
        os.mkfifo(path)
        capture = Capture(path, retain=name == "event_truth.csv")
        captures[name] = capture
        capture.thread.start()

    command = [
        str(simulator),
        "--config",
        str(run_dir / "config.json"),
        "--nav",
        str(nav),
        "--output",
        str(run_dir / "simulated.log"),
        "--week",
        str(week),
        "--sow",
        str(sow),
    ]
    started = time.monotonic()
    peak = 0
    stdout_path = run_dir / "stdout.txt"
    stderr_path = run_dir / "stderr.txt"
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        process = subprocess.Popen(command, stdout=stdout, stderr=stderr)
        while process.poll() is None:
            peak = max(peak, rss_kib(process.pid))
            time.sleep(0.10)
        exit_code = process.wait()
    elapsed = time.monotonic() - started

    for capture in captures.values():
        if capture.thread.is_alive():
            try:
                fd = os.open(capture.path, os.O_WRONLY | os.O_NONBLOCK)
                os.close(fd)
            except OSError:
                pass
    for capture in captures.values():
        capture.thread.join(timeout=10)

    streams = {}
    capture_errors = []
    for name, capture in captures.items():
        streams[name] = {"sha256": capture.hash.hexdigest(), "size_bytes": capture.size}
        if capture.thread.is_alive():
            capture_errors.append(f"{name}: capture thread did not terminate")
        if capture.error:
            capture_errors.append(f"{name}: {capture.error}")

    regular = {}
    for name in REGULAR_NAMES:
        path = run_dir / name
        if path.exists():
            regular[name] = hash_file(path)

    manifest = None
    manifest_path = run_dir / "run_manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())

    events = parse_events(bytes(captures["event_truth.csv"].data))
    for name in FIFO_NAMES:
        (run_dir / name).unlink(missing_ok=True)

    return {
        "command": command,
        "elapsed_sec": elapsed,
        "peak_rss_kib": peak,
        "exit_code": exit_code,
        "stdout_tail": tail_text(stdout_path),
        "stderr_tail": tail_text(stderr_path),
        "stream_outputs": streams,
        "regular_outputs": regular,
        "event_counts": events,
        "manifest": manifest,
        "capture_errors": capture_errors,
    }


def validate(run: dict, config: dict) -> list[str]:
    failures = []
    manifest = run.get("manifest")
    if not isinstance(manifest, dict) or not isinstance(manifest.get("run_summary"), dict):
        failures.append("run_manifest.json/run_summary missing")
    else:
        summary = manifest["run_summary"]
        for key, expected in expected_epoch_counts(config).items():
            actual = int(summary.get(key, -1))
            if actual != expected:
                failures.append(f"{key}: expected {expected}, got {actual}")
        if int(summary.get("max_observations_per_epoch", 0)) <= 0:
            failures.append("max_observations_per_epoch is zero")
        if int(summary.get("valid_position_epochs", 0)) <= 0:
            failures.append("no valid position epoch")
        if int(summary.get("valid_velocity_epochs", 0)) <= 0:
            failures.append("no valid velocity epoch")

    for key, expected in expected_events(config).items():
        actual = int(run.get("event_counts", {}).get(key, 0))
        if actual != expected:
            failures.append(f"{key}: expected {expected}, got {actual}")
    if run.get("exit_code") != 0:
        message = f"simulator exit code {run.get('exit_code')}"
        if run.get("stderr_tail"):
            message += f": {run['stderr_tail']}"
        failures.append(message)
    if int(run.get("peak_rss_kib", 0)) <= 0:
        failures.append("peak RSS unavailable")
    failures.extend(run.get("capture_errors", []))
    return failures


def fingerprint(run: dict) -> dict:
    output = {}
    for group in ("stream_outputs", "regular_outputs"):
        for name, value in run.get(group, {}).items():
            output[name] = value["sha256"]
    return output


def logical_bytes(run: dict) -> int:
    return sum(
        int(value["size_bytes"])
        for group in ("stream_outputs", "regular_outputs")
        for value in run.get(group, {}).values()
    )


def write_report(name: str, report: dict, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "summary.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    lines = [f"# Extended validation: `{name}`", "", f"- Result: **{'PASS' if report['passed'] else 'FAIL'}**"]
    for index, run in enumerate(report["runs"], 1):
        lines.append(
            f"- Run {index}: elapsed `{run['elapsed_sec']:.3f} s`, "
            f"peak RSS `{run['peak_rss_kib'] / 1024.0:.1f} MiB`, "
            f"logical output `{logical_bytes(run) / 1024.0**3:.3f} GiB`"
        )
    if "memory_guardrail_kib" in report:
        lines.append(f"- Memory guardrail: `{report['memory_guardrail_kib'] / 1024.0:.1f} MiB`")
    if report["failures"]:
        lines += ["", "## Failures"] + [f"- {item}" for item in report["failures"]]
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n")


def run_case(simulator: Path, repo: Path, name: str, out_dir: Path, real_nav: Path | None) -> bool:
    config, nav_kind, week, sow = case_definition(name)
    nav, nav_identity = resolve_nav(repo, nav_kind, real_nav)

    report = {
        "schema_version": 1,
        "case": name,
        "config": config,
        "nav_path": str(nav),
        "nav_identity": nav_identity,
        "runs": [],
        "failures": [],
    }
    failures = report["failures"]

    if name == "memory_trend_50hz":
        short = dict(config)
        short["duration_sec"] = 60
        first = run_once(simulator, nav, short, week, sow, out_dir / "short")
        second = run_once(simulator, nav, config, week, sow, out_dir / "long")
        report["runs"] = [first, second]
        failures += validate(first, short) + validate(second, config)
        short_rss = int(first["peak_rss_kib"])
        long_rss = int(second["peak_rss_kib"])
        limit = max(short_rss * 2, short_rss + MEMORY_HEADROOM_KIB)
        report["memory_guardrail_kib"] = limit
        report["memory_growth_kib"] = long_rss - short_rss
        if long_rss > limit:
            failures.append(
                f"bounded-memory guardrail exceeded: 60 s={short_rss} KiB, "
                f"900 s={long_rss} KiB, limit={limit} KiB"
            )
    elif name == "determinism_50hz_10m":
        first = run_once(simulator, nav, config, week, sow, out_dir / "first")
        second = run_once(simulator, nav, config, week, sow, out_dir / "second")
        report["runs"] = [first, second]
        failures += validate(first, config) + validate(second, config)
        first_fingerprint = fingerprint(first)
        second_fingerprint = fingerprint(second)
        report["deterministic_fingerprint"] = first_fingerprint
        for output in sorted(set(first_fingerprint) | set(second_fingerprint)):
            if first_fingerprint.get(output) != second_fingerprint.get(output):
                failures.append(
                    f"determinism mismatch for {output}: "
                    f"{first_fingerprint.get(output)} != {second_fingerprint.get(output)}"
                )
    else:
        run = run_once(simulator, nav, config, week, sow, out_dir / "run")
        report["runs"] = [run]
        failures += validate(run, config)

    report["passed"] = not failures
    write_report(name, report, out_dir)
    return report["passed"]


def self_test() -> None:
    ks = base_config("KS", 10, 5)
    assert expected_epoch_counts(ks) == {
        "scheduled_epochs": 50,
        "powered_epochs": 50,
        "signal_on_epochs": 50,
        "signal_off_epochs": 0,
        "range_messages": 50,
        "psrpos_messages": 50,
        "psrvel_messages": 50,
    }
    assert expected_events(ks) == {"POWER_ON": 1, "POWER_OFF": 0, "SIGNAL_ON": 1, "SIGNAL_OFF": 0}

    rea = base_config("REA", 6, 10)
    rea["rea"] = {"signal_on_sec": 2.0, "signal_off_sec": 1.0}
    assert expected_epoch_counts(rea)["signal_on_epochs"] == 40
    assert expected_epoch_counts(rea)["signal_off_epochs"] == 20
    assert expected_events(rea) == {"POWER_ON": 1, "POWER_OFF": 0, "SIGNAL_ON": 2, "SIGNAL_OFF": 2}

    ttff = base_config("TTFF", 8, 10)
    ttff["ttff"] = {"startup_mode": "HOT", "power_on_sec": 3.0, "power_off_sec": 1.0}
    counts = expected_epoch_counts(ttff)
    assert counts["scheduled_epochs"] == 80
    assert counts["powered_epochs"] == 60
    assert counts["signal_on_epochs"] == 60
    assert counts["signal_off_epochs"] == 0
    assert counts["scheduled_epochs"] - counts["powered_epochs"] == 20
    assert counts["range_messages"] == 60
    assert expected_events(ttff) == {"POWER_ON": 2, "POWER_OFF": 2, "SIGNAL_ON": 2, "SIGNAL_OFF": 2}

    assert case_definition("stress_50hz_1h")[1:] == (
        "real_full_day",
        PINNED_REAL_NAV_WEEK,
        PINNED_REAL_NAV_SOW,
    )
    assert case_definition("determinism_50hz_10m")[1] == "brd4"
    assert len(PINNED_REAL_NAV_SHA256) == 64
    assert MEMORY_HEADROOM_KIB == 64 * 1024


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", type=Path)
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    parser.add_argument("--case", choices=ALL_CASES)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--real-nav",
        type=Path,
        help="Unmodified full-day real RINEX NAV for long-duration cases; SHA256 is pinned and verified.",
    )
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        print("extended validation self-test: PASS")
        return 0
    if args.simulator is None or args.case is None or args.output_dir is None:
        parser.error("--simulator, --case and --output-dir are required unless --self-test is used")
    if os.name != "posix" or not Path("/proc").exists():
        raise SystemExit("extended validation requires a Linux/POSIX runner with /proc")

    simulator = args.simulator.resolve()
    if not simulator.is_file():
        raise SystemExit(f"simulator executable not found: {simulator}")
    try:
        passed = run_case(
            simulator,
            args.repo_root.resolve(),
            args.case,
            args.output_dir.resolve(),
            args.real_nav,
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
