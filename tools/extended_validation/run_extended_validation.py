#!/usr/bin/env python3
"""Run long GNSS simulator validation with bounded streaming capture.

The simulator still serializes its normal receiver/truth outputs. Large streams
are connected to FIFOs and consumed incrementally so the validation harness does
not retain or persist multi-gigabyte histories.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time
from typing import Dict, Iterable, List, Mapping, MutableMapping, Sequence, Tuple

GPS_EPOCH = dt.datetime(1980, 1, 6)
NANOSECONDS_PER_SECOND = 1_000_000_000
FIFO_NAMES = (
    "simulated.log",
    "event_truth.csv",
    "observation_truth.csv",
    "solution_truth.csv",
)
HASHED_REGULAR_NAMES = ("scenario.json", "run_manifest.json")
ALL_CASES = (
    "ks_8h",
    "rea_8h",
    "ttff_8h",
    "stress_50hz_1h",
    "determinism_50hz_10m",
    "memory_trend_50hz",
)


def base_config(scenario: str, duration_sec: int, sampling_rate_hz: int) -> Dict[str, object]:
    return {
        "schema_version": 1,
        "scenario": scenario,
        "duration_sec": duration_sec,
        "sampling_rate_hz": sampling_rate_hz,
        "elevation_mask_deg": 0.0,
        "output_eph": True,
        "output_ion": True,
        "measurement_noise_enabled": False,
        "multipath_enabled": False,
        "receiver_clock_bias_m": 0.0,
        "receiver_clock_drift_mps": 0.0,
        "atmosphere_mode": "none",
        "receiver": {
            "latitude_deg": 20.0,
            "longitude_deg": 120.0,
            "height_m": 100.0,
        },
        "ttff": {
            "startup_mode": "HOT",
            "power_on_sec": 300.0,
            "power_off_sec": 30.0,
        },
        "rea": {
            "signal_on_sec": 300.0,
            "signal_off_sec": 10.0,
        },
        "seed": 1,
    }


def format_nav_field(value: float) -> str:
    return f"{value:19.12E}"


def replace_nav_field(line: str, field_index: int, value: float) -> str:
    start = 4 + field_index * 19
    end = start + 19
    if len(line.rstrip("\n")) < end:
        raise ValueError("RINEX navigation line is too short for fixed-width field replacement")
    return line[:start] + format_nav_field(value) + line[end:]


def split_rinex3_gps(source: Path) -> Tuple[List[str], List[List[str]]]:
    lines = source.read_text(encoding="ascii").splitlines(keepends=True)
    header: List[str] = []
    body_index = -1
    for index, line in enumerate(lines):
        header.append(line)
        if "END OF HEADER" in line:
            body_index = index + 1
            break
    if body_index < 0:
        raise ValueError("GPS loopback RINEX has no END OF HEADER")

    body = lines[body_index:]
    if len(body) % 8 != 0:
        raise ValueError("GPS loopback RINEX does not contain complete 8-line GPS ephemerides")
    records = [body[index : index + 8] for index in range(0, len(body), 8)]
    if not records:
        raise ValueError("GPS loopback RINEX contains no ephemerides")
    for record in records:
        if len(record[0]) < 3 or not record[0].startswith("G"):
            raise ValueError("long-duration NAV materializer expects a GPS-only source fixture")
    return header, records


def gpst_calendar(gps_week: int, sow_sec: int) -> dt.datetime:
    return GPS_EPOCH + dt.timedelta(weeks=gps_week, seconds=sow_sec)


def materialize_long_gps_nav(
    source: Path,
    destination: Path,
    gps_week: int,
    start_sow_sec: int,
    duration_sec: int,
    interval_sec: int = 3600,
) -> None:
    """Repeat the synthetic full-sky GPS fixture with fresh Toe/TTR values.

    This fixture is only for long streaming/resource validation. Orbital
    parameters stay fixed while Toc/Toe/TTR advance, which keeps the RTKLIB
    ephemeris-selection path supplied without downloading live navigation data.
    """

    header, base_records = split_rinex3_gps(source)
    base_toe = (start_sow_sec // interval_sec) * interval_sec
    if base_toe > start_sow_sec:
        base_toe -= interval_sec
    final_sow = start_sow_sec + duration_sec
    toe_values = list(range(base_toe, final_sow + interval_sec + 1, interval_sec))

    output: List[str] = list(header)
    for toe in toe_values:
        if toe >= 604800:
            raise ValueError("long GPS NAV materializer currently expects a single GPS week")
        calendar = gpst_calendar(gps_week, toe)
        toc_prefix = (
            f"{{sat}} {calendar.year:04d} {calendar.month:02d} {calendar.day:02d} "
            f"{calendar.hour:02d} {calendar.minute:02d} {calendar.second:02d}"
        )
        for base in base_records:
            record = list(base)
            sat = record[0][:3]
            if len(record[0]) < 24:
                raise ValueError("invalid RINEX first ephemeris line")
            record[0] = toc_prefix.format(sat=sat) + record[0][23:]
            record[3] = replace_nav_field(record[3], 0, float(toe))
            record[5] = replace_nav_field(record[5], 2, float(gps_week))
            record[7] = replace_nav_field(record[7], 0, float(toe))
            output.extend(record)

    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("".join(output), encoding="ascii", newline="\n")


class StreamCapture:
    def __init__(self, path: Path, capture_bytes: bool = False) -> None:
        self.path = path
        self.capture_bytes = capture_bytes
        self.size_bytes = 0
        self.sha256 = hashlib.sha256()
        self.captured = bytearray()
        self.error: str | None = None
        self.thread = threading.Thread(target=self._run, name=f"capture-{path.name}", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        try:
            with self.path.open("rb", buffering=0) as stream:
                while True:
                    chunk = stream.read(1024 * 1024)
                    if not chunk:
                        break
                    self.size_bytes += len(chunk)
                    self.sha256.update(chunk)
                    if self.capture_bytes:
                        self.captured.extend(chunk)
        except Exception as exc:  # pragma: no cover - emitted into diagnostic summary
            self.error = f"{type(exc).__name__}: {exc}"

    def digest(self) -> str:
        return self.sha256.hexdigest()


def read_peak_rss_kib(pid: int) -> int:
    status = Path(f"/proc/{pid}/status")
    try:
        for line in status.read_text(encoding="ascii").splitlines():
            if line.startswith("VmRSS:"):
                fields = line.split()
                return int(fields[1])
    except (FileNotFoundError, ProcessLookupError, ValueError):
        return 0
    return 0


def hash_file(path: Path) -> Dict[str, object]:
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
            size += len(chunk)
    return {"sha256": digest.hexdigest(), "size_bytes": size}


def release_blocked_fifo_readers(captures: Iterable[StreamCapture]) -> None:
    for capture in captures:
        if not capture.thread.is_alive():
            continue
        try:
            fd = os.open(capture.path, os.O_WRONLY | os.O_NONBLOCK)
        except OSError:
            continue
        os.close(fd)


def parse_events(data: bytes) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    if not data:
        return counts
    text = data.decode("utf-8")
    for row in csv.DictReader(io.StringIO(text)):
        event_type = row.get("event_type", "")
        if event_type:
            counts[event_type] = counts.get(event_type, 0) + 1
    return counts


def boundary_count(duration_ns: int, first_ns: int, period_ns: int) -> int:
    if period_ns <= 0:
        raise ValueError("period must be positive")
    if first_ns >= duration_ns:
        return 0
    return 1 + (duration_ns - 1 - first_ns) // period_ns


def expected_events(config: Mapping[str, object]) -> Dict[str, int]:
    duration_ns = int(config["duration_sec"]) * NANOSECONDS_PER_SECOND
    scenario = str(config["scenario"])
    result = {"POWER_ON": 1, "POWER_OFF": 0, "SIGNAL_ON": 0, "SIGNAL_OFF": 0}
    if scenario == "REA":
        rea = config["rea"]
        assert isinstance(rea, Mapping)
        on_ns = int(round(float(rea["signal_on_sec"]) * NANOSECONDS_PER_SECOND))
        off_ns = int(round(float(rea["signal_off_sec"]) * NANOSECONDS_PER_SECOND))
        period_ns = on_ns + off_ns
        result["SIGNAL_OFF"] = boundary_count(duration_ns, on_ns, period_ns)
        result["SIGNAL_ON"] = boundary_count(duration_ns, period_ns, period_ns)
    elif scenario == "TTFF":
        ttff = config["ttff"]
        assert isinstance(ttff, Mapping)
        on_ns = int(round(float(ttff["power_on_sec"]) * NANOSECONDS_PER_SECOND))
        off_ns = int(round(float(ttff["power_off_sec"]) * NANOSECONDS_PER_SECOND))
        period_ns = on_ns + off_ns
        result["POWER_OFF"] = boundary_count(duration_ns, on_ns, period_ns)
        result["POWER_ON"] += boundary_count(duration_ns, period_ns, period_ns)
    elif scenario != "KS":
        raise ValueError(f"unsupported scenario in extended validation: {scenario}")
    return result


def validate_run(
    run: Mapping[str, object],
    config: Mapping[str, object],
    expected_event_counts: Mapping[str, int],
) -> List[str]:
    failures: List[str] = []
    expected_epochs = int(config["duration_sec"]) * int(config["sampling_rate_hz"])
    manifest = run.get("manifest")
    if not isinstance(manifest, Mapping):
        return ["run manifest was not produced"]

    summary = manifest.get("run_summary")
    if not isinstance(summary, Mapping):
        return ["run manifest has no run_summary"]

    for key in ("scheduled_epochs", "range_messages", "psrpos_messages", "psrvel_messages"):
        actual = int(summary.get(key, -1))
        if actual != expected_epochs:
            failures.append(f"{key}: expected {expected_epochs}, got {actual}")

    actual_events = run.get("event_counts", {})
    if not isinstance(actual_events, Mapping):
        failures.append("event counts are unavailable")
    else:
        for event_type, expected in expected_event_counts.items():
            actual = int(actual_events.get(event_type, 0))
            if actual != expected:
                failures.append(f"{event_type}: expected {expected}, got {actual}")

    if int(run.get("exit_code", -1)) != 0:
        failures.append(f"simulator exited with code {run.get('exit_code')}")
    if int(run.get("peak_rss_kib", 0)) <= 0:
        failures.append("peak RSS measurement is unavailable")
    return failures


def run_once(
    simulator: Path,
    nav_path: Path,
    config: Mapping[str, object],
    gps_week: int,
    sow_sec: int,
    run_dir: Path,
) -> Dict[str, object]:
    if run_dir.exists():
        shutil.rmtree(run_dir)
    run_dir.mkdir(parents=True)
    config_path = run_dir / "config.json"
    config_path.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    captures: Dict[str, StreamCapture] = {}
    for name in FIFO_NAMES:
        path = run_dir / name
        os.mkfifo(path)
        capture = StreamCapture(path, capture_bytes=(name == "event_truth.csv"))
        captures[name] = capture
        capture.start()

    stdout_path = run_dir / "stdout.txt"
    stderr_path = run_dir / "stderr.txt"
    command = [
        str(simulator),
        "--config",
        str(config_path),
        "--nav",
        str(nav_path),
        "--output",
        str(run_dir / "simulated.log"),
        "--week",
        str(gps_week),
        "--sow",
        str(sow_sec),
    ]

    started = time.monotonic()
    peak_rss_kib = 0
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        process = subprocess.Popen(command, stdout=stdout, stderr=stderr)
        while process.poll() is None:
            peak_rss_kib = max(peak_rss_kib, read_peak_rss_kib(process.pid))
            time.sleep(0.10)
        peak_rss_kib = max(peak_rss_kib, read_peak_rss_kib(process.pid))
        exit_code = process.wait()
    elapsed_sec = time.monotonic() - started

    release_blocked_fifo_readers(captures.values())
    for capture in captures.values():
        capture.thread.join(timeout=10.0)

    stream_metrics: Dict[str, object] = {}
    capture_errors: List[str] = []
    for name, capture in captures.items():
        stream_metrics[name] = {
            "sha256": capture.digest(),
            "size_bytes": capture.size_bytes,
        }
        if capture.thread.is_alive():
            capture_errors.append(f"{name}: capture thread did not terminate")
        if capture.error:
            capture_errors.append(f"{name}: {capture.error}")

    regular_metrics: Dict[str, object] = {}
    for name in HASHED_REGULAR_NAMES:
        path = run_dir / name
        if path.exists():
            regular_metrics[name] = hash_file(path)

    event_counts = parse_events(bytes(captures["event_truth.csv"].captured))
    manifest_path = run_dir / "run_manifest.json"
    manifest: object = None
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    for name in FIFO_NAMES:
        try:
            (run_dir / name).unlink()
        except FileNotFoundError:
            pass

    return {
        "command": command,
        "elapsed_sec": elapsed_sec,
        "peak_rss_kib": peak_rss_kib,
        "exit_code": exit_code,
        "stream_outputs": stream_metrics,
        "regular_outputs": regular_metrics,
        "event_counts": event_counts,
        "manifest": manifest,
        "capture_errors": capture_errors,
    }


def deterministic_fingerprint(run: Mapping[str, object]) -> Dict[str, str]:
    result: Dict[str, str] = {}
    stream_outputs = run.get("stream_outputs", {})
    regular_outputs = run.get("regular_outputs", {})
    if isinstance(stream_outputs, Mapping):
        for name, value in stream_outputs.items():
            if isinstance(value, Mapping):
                result[str(name)] = str(value.get("sha256", ""))
    if isinstance(regular_outputs, Mapping):
        for name, value in regular_outputs.items():
            if isinstance(value, Mapping):
                result[str(name)] = str(value.get("sha256", ""))
    return result


def case_definition(case_name: str) -> Dict[str, object]:
    if case_name == "ks_8h":
        return {"config": base_config("KS", 8 * 3600, 10), "nav": "long_gps", "week": 2253, "sow": 172900}
    if case_name == "rea_8h":
        return {"config": base_config("REA", 8 * 3600, 10), "nav": "long_gps", "week": 2253, "sow": 172900}
    if case_name == "ttff_8h":
        return {"config": base_config("TTFF", 8 * 3600, 10), "nav": "long_gps", "week": 2253, "sow": 172900}
    if case_name == "stress_50hz_1h":
        return {"config": base_config("KS", 3600, 50), "nav": "brd4", "week": 2347, "sow": 436500}
    if case_name == "determinism_50hz_10m":
        return {"config": base_config("KS", 600, 50), "nav": "brd4", "week": 2347, "sow": 436500}
    if case_name == "memory_trend_50hz":
        return {"config": base_config("KS", 900, 50), "nav": "brd4", "week": 2347, "sow": 436500}
    raise ValueError(f"unknown extended case: {case_name}")


def total_output_bytes(run: Mapping[str, object]) -> int:
    total = 0
    for group_name in ("stream_outputs", "regular_outputs"):
        group = run.get(group_name, {})
        if isinstance(group, Mapping):
            for value in group.values():
                if isinstance(value, Mapping):
                    total += int(value.get("size_bytes", 0))
    return total


def write_report(case_name: str, report: MutableMapping[str, object], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    lines = [
        f"# Extended validation: `{case_name}`",
        "",
        f"- Result: **{'PASS' if report.get('passed') else 'FAIL'}**",
    ]
    runs = report.get("runs", [])
    if isinstance(runs, Sequence):
        for index, run in enumerate(runs, start=1):
            if not isinstance(run, Mapping):
                continue
            lines.extend(
                [
                    f"- Run {index}: elapsed `{float(run.get('elapsed_sec', 0.0)):.3f} s`, "
                    f"peak RSS `{int(run.get('peak_rss_kib', 0)) / 1024.0:.1f} MiB`, "
                    f"logical output `{total_output_bytes(run) / (1024.0 ** 3):.3f} GiB`",
                ]
            )
    failures = report.get("failures", [])
    if isinstance(failures, Sequence) and failures:
        lines.append("")
        lines.append("## Failures")
        for failure in failures:
            lines.append(f"- {failure}")
    lines.append("")
    (output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")


def run_case(simulator: Path, repo_root: Path, case_name: str, output_dir: Path) -> bool:
    definition = case_definition(case_name)
    config = definition["config"]
    assert isinstance(config, Mapping)
    week = int(definition["week"])
    sow = int(definition["sow"])

    nav_kind = str(definition["nav"])
    if nav_kind == "brd4":
        nav_path = repo_root / "tests/data/minimal/brd400dlr_rinex4_acceptance_nav.rnx"
    elif nav_kind == "long_gps":
        nav_path = output_dir / "long_gps_nav.rnx"
        materialize_long_gps_nav(
            repo_root / "tests/data/minimal/gps_loopback_nav.rnx",
            nav_path,
            week,
            sow,
            int(config["duration_sec"]),
        )
    else:
        raise ValueError(f"unsupported NAV kind: {nav_kind}")

    failures: List[str] = []
    report: Dict[str, object] = {
        "schema_version": 1,
        "case": case_name,
        "config": config,
        "nav_path": nav_path.name,
        "runs": [],
        "failures": failures,
    }

    if case_name == "memory_trend_50hz":
        short_config = dict(config)
        short_config["duration_sec"] = 60
        short_run = run_once(simulator, nav_path, short_config, week, sow, output_dir / "short")
        long_run = run_once(simulator, nav_path, config, week, sow, output_dir / "long")
        report["runs"] = [short_run, long_run]
        failures.extend(validate_run(short_run, short_config, expected_events(short_config)))
        failures.extend(validate_run(long_run, config, expected_events(config)))
        short_rss = int(short_run.get("peak_rss_kib", 0))
        long_rss = int(long_run.get("peak_rss_kib", 0))
        guardrail_kib = max(short_rss * 2, short_rss + 256 * 1024)
        report["memory_guardrail_kib"] = guardrail_kib
        if long_rss > guardrail_kib:
            failures.append(
                f"bounded-memory guardrail exceeded: 60 s peak={short_rss} KiB, "
                f"900 s peak={long_rss} KiB, limit={guardrail_kib} KiB"
            )
    elif case_name == "determinism_50hz_10m":
        first = run_once(simulator, nav_path, config, week, sow, output_dir / "first")
        second = run_once(simulator, nav_path, config, week, sow, output_dir / "second")
        report["runs"] = [first, second]
        failures.extend(validate_run(first, config, expected_events(config)))
        failures.extend(validate_run(second, config, expected_events(config)))
        first_hashes = deterministic_fingerprint(first)
        second_hashes = deterministic_fingerprint(second)
        report["deterministic_fingerprint"] = first_hashes
        if first_hashes != second_hashes:
            for name in sorted(set(first_hashes) | set(second_hashes)):
                if first_hashes.get(name) != second_hashes.get(name):
                    failures.append(
                        f"determinism mismatch for {name}: "
                        f"{first_hashes.get(name, '<missing>')} != {second_hashes.get(name, '<missing>')}"
                    )
    else:
        run = run_once(simulator, nav_path, config, week, sow, output_dir / "run")
        report["runs"] = [run]
        failures.extend(validate_run(run, config, expected_events(config)))

    for run in report["runs"]:
        if isinstance(run, Mapping):
            for capture_error in run.get("capture_errors", []):
                failures.append(str(capture_error))

    report["passed"] = not failures
    write_report(case_name, report, output_dir)
    return not failures


def self_test() -> None:
    ks = base_config("KS", 10, 1)
    assert expected_events(ks) == {"POWER_ON": 1, "POWER_OFF": 0, "SIGNAL_ON": 0, "SIGNAL_OFF": 0}

    rea = base_config("REA", 6, 10)
    rea["rea"] = {"signal_on_sec": 2.0, "signal_off_sec": 1.0}
    assert expected_events(rea) == {"POWER_ON": 1, "POWER_OFF": 0, "SIGNAL_ON": 2, "SIGNAL_OFF": 2}

    ttff = base_config("TTFF", 8, 10)
    ttff["ttff"] = {"startup_mode": "HOT", "power_on_sec": 3.0, "power_off_sec": 1.0}
    assert expected_events(ttff) == {"POWER_ON": 2, "POWER_OFF": 2, "SIGNAL_ON": 0, "SIGNAL_OFF": 0}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", type=Path)
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    parser.add_argument("--case", choices=ALL_CASES)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        print("extended validation self-test: PASS")
        return 0

    if args.simulator is None or args.case is None or args.output_dir is None:
        parser.error("--simulator, --case and --output-dir are required unless --self-test is used")

    simulator = args.simulator.resolve()
    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    if os.name != "posix" or not Path("/proc").exists():
        raise SystemExit("extended validation currently requires a Linux/POSIX runner with /proc")
    if not simulator.is_file():
        raise SystemExit(f"simulator executable not found: {simulator}")

    passed = run_case(simulator, repo_root, args.case, output_dir)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
