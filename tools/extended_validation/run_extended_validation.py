#!/usr/bin/env python3
"""Bounded long-duration/resource validation for gnss-data-simulator."""

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

GPS_EPOCH = dt.datetime(1980, 1, 6)
NS = 1_000_000_000
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


def base_config(scenario, duration_sec, rate_hz):
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


def case_definition(name):
    if name == "ks_8h":
        return base_config("KS", 28800, 10), "long_gps", 2253, 172900
    if name == "rea_8h":
        return base_config("REA", 28800, 10), "long_gps", 2253, 172900
    if name == "ttff_8h":
        return base_config("TTFF", 28800, 10), "long_gps", 2253, 172900
    if name == "stress_50hz_1h":
        return base_config("KS", 3600, 50), "brd4", 2347, 436500
    if name == "determinism_50hz_10m":
        return base_config("KS", 600, 50), "brd4", 2347, 436500
    if name == "memory_trend_50hz":
        return base_config("KS", 900, 50), "brd4", 2347, 436500
    raise ValueError(f"unknown extended case: {name}")


def split_gps_rinex(path):
    lines = path.read_text(encoding="ascii").splitlines(keepends=True)
    for i, line in enumerate(lines):
        if "END OF HEADER" in line:
            header, body = lines[: i + 1], lines[i + 1 :]
            break
    else:
        raise ValueError("GPS loopback RINEX has no END OF HEADER")
    if not body or len(body) % 8:
        raise ValueError("GPS loopback RINEX must contain complete 8-line GPS ephemerides")
    records = [body[i : i + 8] for i in range(0, len(body), 8)]
    if any(not rec[0].startswith("G") for rec in records):
        raise ValueError("long NAV source must be GPS-only")
    return header, records


def nav_field(line, field_index, value):
    start = 4 + field_index * 19
    end = start + 19
    if len(line.rstrip("\n")) < end:
        raise ValueError("short RINEX NAV field")
    return line[:start] + f"{value:19.12E}" + line[end:]


def materialize_long_gps_nav(source, destination, week, start_sow, duration_sec):
    header, records = split_gps_rinex(source)
    interval = 3600
    first_toe = (start_sow // interval) * interval
    last_toe = start_sow + duration_sec + interval
    out = list(header)
    for toe in range(first_toe, last_toe + 1, interval):
        if not 0 <= toe < 604800:
            raise ValueError("long GPS NAV fixture currently stays within one GPS week")
        epoch = GPS_EPOCH + dt.timedelta(weeks=week, seconds=toe)
        for base in records:
            rec = list(base)
            prefix = (
                f"{rec[0][:3]} {epoch.year:04d} {epoch.month:02d} {epoch.day:02d} "
                f"{epoch.hour:02d} {epoch.minute:02d} {epoch.second:02d}"
            )
            rec[0] = prefix + rec[0][23:]
            rec[3] = nav_field(rec[3], 0, float(toe))
            rec[5] = nav_field(rec[5], 2, float(week))
            rec[7] = nav_field(rec[7], 0, float(toe))
            out.extend(rec)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("".join(out), encoding="ascii", newline="\n")


class Capture:
    def __init__(self, path, retain=False):
        self.path = path
        self.retain = retain
        self.size = 0
        self.hash = hashlib.sha256()
        self.data = bytearray()
        self.error = ""
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        try:
            with self.path.open("rb", buffering=0) as src:
                while True:
                    chunk = src.read(1024 * 1024)
                    if not chunk:
                        break
                    self.size += len(chunk)
                    self.hash.update(chunk)
                    if self.retain:
                        self.data.extend(chunk)
        except Exception as exc:  # diagnostic path
            self.error = f"{type(exc).__name__}: {exc}"


def rss_kib(pid):
    try:
        for line in Path(f"/proc/{pid}/status").read_text(encoding="ascii").splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1])
    except (FileNotFoundError, ProcessLookupError, ValueError):
        pass
    return 0


def hash_file(path):
    h = hashlib.sha256()
    size = 0
    with path.open("rb") as src:
        while True:
            chunk = src.read(1024 * 1024)
            if not chunk:
                break
            h.update(chunk)
            size += len(chunk)
    return {"sha256": h.hexdigest(), "size_bytes": size}


def parse_events(data):
    counts = {}
    for row in csv.DictReader(io.StringIO(data.decode("utf-8"))):
        kind = row.get("event_type", "")
        if kind:
            counts[kind] = counts.get(kind, 0) + 1
    return counts


def boundary_count(duration_ns, first_ns, period_ns):
    if first_ns >= duration_ns:
        return 0
    return 1 + (duration_ns - 1 - first_ns) // period_ns


def expected_events(config):
    duration_ns = int(config["duration_sec"]) * NS
    # ScenarioEngine marks both initial powered and initial signal-available
    # states as transitions at t=0.
    expected = {"POWER_ON": 1, "POWER_OFF": 0, "SIGNAL_ON": 1, "SIGNAL_OFF": 0}
    if config["scenario"] == "REA":
        on_ns = round(float(config["rea"]["signal_on_sec"]) * NS)
        off_ns = round(float(config["rea"]["signal_off_sec"]) * NS)
        period = on_ns + off_ns
        expected["SIGNAL_OFF"] = boundary_count(duration_ns, on_ns, period)
        expected["SIGNAL_ON"] += boundary_count(duration_ns, period, period)
    elif config["scenario"] == "TTFF":
        on_ns = round(float(config["ttff"]["power_on_sec"]) * NS)
        off_ns = round(float(config["ttff"]["power_off_sec"]) * NS)
        period = on_ns + off_ns
        off_count = boundary_count(duration_ns, on_ns, period)
        on_count = boundary_count(duration_ns, period, period)
        expected["POWER_OFF"] = off_count
        expected["SIGNAL_OFF"] = off_count
        expected["POWER_ON"] += on_count
        expected["SIGNAL_ON"] += on_count
    elif config["scenario"] != "KS":
        raise ValueError(f"unsupported scenario: {config['scenario']}")
    return expected


def run_once(simulator, nav, config, week, sow, run_dir):
    if run_dir.exists():
        shutil.rmtree(run_dir)
    run_dir.mkdir(parents=True)
    (run_dir / "config.json").write_text(json.dumps(config, indent=2, sort_keys=True) + "\n")

    captures = {}
    for name in FIFO_NAMES:
        path = run_dir / name
        os.mkfifo(path)
        cap = Capture(path, retain=name == "event_truth.csv")
        captures[name] = cap
        cap.thread.start()

    cmd = [
        str(simulator),
        "--config", str(run_dir / "config.json"),
        "--nav", str(nav),
        "--output", str(run_dir / "simulated.log"),
        "--week", str(week),
        "--sow", str(sow),
    ]
    started = time.monotonic()
    peak = 0
    with (run_dir / "stdout.txt").open("wb") as stdout, (run_dir / "stderr.txt").open("wb") as stderr:
        proc = subprocess.Popen(cmd, stdout=stdout, stderr=stderr)
        while proc.poll() is None:
            peak = max(peak, rss_kib(proc.pid))
            time.sleep(0.10)
        code = proc.wait()
    elapsed = time.monotonic() - started

    # If startup failed before every writer opened, release readers blocked in open().
    for cap in captures.values():
        if cap.thread.is_alive():
            try:
                fd = os.open(cap.path, os.O_WRONLY | os.O_NONBLOCK)
                os.close(fd)
            except OSError:
                pass
    for cap in captures.values():
        cap.thread.join(timeout=10)

    streams = {}
    errors = []
    for name, cap in captures.items():
        streams[name] = {"sha256": cap.hash.hexdigest(), "size_bytes": cap.size}
        if cap.thread.is_alive():
            errors.append(f"{name}: capture thread did not terminate")
        if cap.error:
            errors.append(f"{name}: {cap.error}")

    regular = {}
    for name in REGULAR_NAMES:
        path = run_dir / name
        if path.exists():
            regular[name] = hash_file(path)

    manifest = None
    if (run_dir / "run_manifest.json").exists():
        manifest = json.loads((run_dir / "run_manifest.json").read_text())

    events = parse_events(bytes(captures["event_truth.csv"].data))
    for name in FIFO_NAMES:
        (run_dir / name).unlink(missing_ok=True)

    return {
        "command": cmd,
        "elapsed_sec": elapsed,
        "peak_rss_kib": peak,
        "exit_code": code,
        "stream_outputs": streams,
        "regular_outputs": regular,
        "event_counts": events,
        "manifest": manifest,
        "capture_errors": errors,
    }


def validate(run, config):
    failures = []
    manifest = run.get("manifest")
    if not isinstance(manifest, dict) or not isinstance(manifest.get("run_summary"), dict):
        return ["run_manifest.json/run_summary missing"]
    summary = manifest["run_summary"]
    expected_epochs = int(config["duration_sec"]) * int(config["sampling_rate_hz"])
    for key in ("scheduled_epochs", "range_messages", "psrpos_messages", "psrvel_messages"):
        if int(summary.get(key, -1)) != expected_epochs:
            failures.append(f"{key}: expected {expected_epochs}, got {summary.get(key)}")
    for key, value in expected_events(config).items():
        actual = int(run.get("event_counts", {}).get(key, 0))
        if actual != value:
            failures.append(f"{key}: expected {value}, got {actual}")
    if run.get("exit_code") != 0:
        failures.append(f"simulator exit code {run.get('exit_code')}")
    if int(run.get("peak_rss_kib", 0)) <= 0:
        failures.append("peak RSS unavailable")
    failures.extend(run.get("capture_errors", []))
    return failures


def fingerprint(run):
    out = {}
    for group in ("stream_outputs", "regular_outputs"):
        for name, value in run.get(group, {}).items():
            out[name] = value["sha256"]
    return out


def logical_bytes(run):
    return sum(
        int(value["size_bytes"])
        for group in ("stream_outputs", "regular_outputs")
        for value in run.get(group, {}).values()
    )


def write_report(name, report, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "summary.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    lines = [f"# Extended validation: `{name}`", "", f"- Result: **{'PASS' if report['passed'] else 'FAIL'}**"]
    for i, run in enumerate(report["runs"], 1):
        lines.append(
            f"- Run {i}: elapsed `{run['elapsed_sec']:.3f} s`, "
            f"peak RSS `{run['peak_rss_kib'] / 1024.0:.1f} MiB`, "
            f"logical output `{logical_bytes(run) / 1024.0**3:.3f} GiB`"
        )
    if report["failures"]:
        lines += ["", "## Failures"] + [f"- {item}" for item in report["failures"]]
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n")


def run_case(simulator, repo, name, out_dir):
    config, nav_kind, week, sow = case_definition(name)
    if nav_kind == "brd4":
        nav = repo / "tests/data/minimal/brd400dlr_rinex4_acceptance_nav.rnx"
    else:
        nav = out_dir / "long_gps_nav.rnx"
        materialize_long_gps_nav(
            repo / "tests/data/minimal/gps_loopback_nav.rnx",
            nav,
            week,
            sow,
            int(config["duration_sec"]),
        )

    report = {"schema_version": 1, "case": name, "config": config, "nav_path": nav.name, "runs": [], "failures": []}
    failures = report["failures"]

    if name == "memory_trend_50hz":
        short = dict(config)
        short["duration_sec"] = 60
        a = run_once(simulator, nav, short, week, sow, out_dir / "short")
        b = run_once(simulator, nav, config, week, sow, out_dir / "long")
        report["runs"] = [a, b]
        failures += validate(a, short) + validate(b, config)
        short_rss, long_rss = int(a["peak_rss_kib"]), int(b["peak_rss_kib"])
        limit = max(short_rss * 2, short_rss + 256 * 1024)
        report["memory_guardrail_kib"] = limit
        if long_rss > limit:
            failures.append(
                f"bounded-memory guardrail exceeded: 60 s={short_rss} KiB, "
                f"900 s={long_rss} KiB, limit={limit} KiB"
            )
    elif name == "determinism_50hz_10m":
        a = run_once(simulator, nav, config, week, sow, out_dir / "first")
        b = run_once(simulator, nav, config, week, sow, out_dir / "second")
        report["runs"] = [a, b]
        failures += validate(a, config) + validate(b, config)
        fa, fb = fingerprint(a), fingerprint(b)
        report["deterministic_fingerprint"] = fa
        for output in sorted(set(fa) | set(fb)):
            if fa.get(output) != fb.get(output):
                failures.append(f"determinism mismatch for {output}: {fa.get(output)} != {fb.get(output)}")
    else:
        run = run_once(simulator, nav, config, week, sow, out_dir / "run")
        report["runs"] = [run]
        failures += validate(run, config)

    report["passed"] = not failures
    write_report(name, report, out_dir)
    return report["passed"]


def self_test():
    ks = base_config("KS", 10, 1)
    assert expected_events(ks) == {"POWER_ON": 1, "POWER_OFF": 0, "SIGNAL_ON": 1, "SIGNAL_OFF": 0}
    rea = base_config("REA", 6, 10)
    rea["rea"] = {"signal_on_sec": 2.0, "signal_off_sec": 1.0}
    assert expected_events(rea) == {"POWER_ON": 1, "POWER_OFF": 0, "SIGNAL_ON": 2, "SIGNAL_OFF": 2}
    ttff = base_config("TTFF", 8, 10)
    ttff["ttff"] = {"startup_mode": "HOT", "power_on_sec": 3.0, "power_off_sec": 1.0}
    assert expected_events(ttff) == {"POWER_ON": 2, "POWER_OFF": 2, "SIGNAL_ON": 2, "SIGNAL_OFF": 2}


def main():
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
    if os.name != "posix" or not Path("/proc").exists():
        raise SystemExit("extended validation requires a Linux/POSIX runner with /proc")

    simulator = args.simulator.resolve()
    if not simulator.is_file():
        raise SystemExit(f"simulator executable not found: {simulator}")
    return 0 if run_case(simulator, args.repo_root.resolve(), args.case, args.output_dir.resolve()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
