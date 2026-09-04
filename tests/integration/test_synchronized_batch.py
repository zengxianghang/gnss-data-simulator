#!/usr/bin/env python3
"""End-to-end regression for synchronized KS/REA/TTFF batch generation."""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any

SCENARIOS = ("KS", "REA", "TTFF")
START_WEEK = 2347
START_SOW = 436500.0
DURATION_SEC = 20.0
COUNT = 8
EXPECTED_SEEDS = {
    "KS": list(range(1001, 1009)),
    "REA": list(range(2001, 2009)),
    "TTFF": list(range(3001, 3009)),
}
TRUTH_ARTIFACTS = (
    "scenario.json",
    "event_truth.csv",
    "observation_truth.csv",
    "solution_truth.csv",
    "run_manifest.json",
)
PHYSICAL_OBSERVATION_FIELDS = (
    "wavelength_m",
    "receiver_x_m",
    "receiver_y_m",
    "receiver_z_m",
    "receiver_vx_mps",
    "receiver_vy_mps",
    "receiver_vz_mps",
    "transmit_week",
    "transmit_tow_ns",
    "transmit_sow_sec",
    "satellite_x_m",
    "satellite_y_m",
    "satellite_z_m",
    "satellite_vx_mps",
    "satellite_vy_mps",
    "satellite_vz_mps",
    "azimuth_deg",
    "elevation_deg",
    "geometric_range_m",
    "range_rate_mps",
    "satellite_clock_bias_m",
    "satellite_clock_drift_mps",
    "receiver_clock_bias_m",
    "receiver_clock_drift_mps",
    "ionosphere_m",
    "troposphere_m",
    "broadcast_message_family",
    "tgd_sec_0",
    "tgd_sec_1",
    "tgd_sec_2",
    "tgd_sec_3",
    "isc_sec_0",
    "isc_sec_1",
    "isc_sec_2",
    "isc_sec_3",
    "isc_sec_4",
    "isc_sec_5",
    "glonass_dtaun_sec",
    "code_bias_m",
    "code_bias_status",
)


class ValidationError(RuntimeError):
    pass


def _check(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def _read_json(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValidationError(f"cannot read JSON {path}: {exc}") from exc
    _check(isinstance(data, dict), f"JSON root is not an object: {path}")
    return data


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(data, stream, indent=2, ensure_ascii=False)
        stream.write("\n")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _find_simulator(build_dir: Path) -> Path:
    candidates = (
        build_dir / "gnss-data-simulator",
        build_dir / "gnss-data-simulator.exe",
        build_dir / "Release" / "gnss-data-simulator.exe",
        build_dir / "Release" / "gnss-data-simulator",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise ValidationError(f"cannot find built gnss-data-simulator below {build_dir}")


def _make_short_config(source_dir: Path, output_path: Path) -> dict[str, Any]:
    config = _read_json(source_dir / "config" / "default_v1.json")
    config["duration_sec"] = DURATION_SEC
    config["sampling_rate_hz"] = 1
    config["output_eph"] = False
    config["output_ion"] = False
    config["multipath_enabled"] = False
    config["measurement_noise_enabled"] = False
    config["ttff"]["startup_mode"] = "HOT"
    config["ttff"]["power_on_sec"] = 6.0
    config["ttff"]["power_off_sec"] = 4.0
    config["rea"]["signal_on_sec"] = 6.0
    config["rea"]["signal_off_sec"] = 4.0
    _write_json(output_path, config)
    return config


def _run_batch(source_dir: Path, simulator: Path, config_path: Path, nav_path: Path, output_dir: Path) -> None:
    batch_tool = source_dir / "tools" / "batch_generation" / "generate_batch.py"
    command = [
        sys.executable,
        str(batch_tool),
        "--simulator",
        str(simulator),
        "--config",
        str(config_path),
        "--nav",
        str(nav_path),
        "--output-dir",
        str(output_dir),
        "--week",
        str(START_WEEK),
        "--sow",
        str(START_SOW),
        "--count",
        str(COUNT),
    ]
    try:
        subprocess.run(command, cwd=source_dir, check=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        raise ValidationError(f"synchronized batch generation failed: {exc}") from exc


def _run_map(manifest: dict[str, Any]) -> dict[tuple[str, int], dict[str, Any]]:
    runs = manifest.get("runs")
    _check(isinstance(runs, list), "batch manifest runs is not a list")
    result: dict[tuple[str, int], dict[str, Any]] = {}
    for item in runs:
        _check(isinstance(item, dict), "batch manifest run is not an object")
        scenario = item.get("scenario")
        index = item.get("realization_index")
        _check(isinstance(scenario, str) and scenario in SCENARIOS, f"invalid run scenario: {scenario}")
        _check(isinstance(index, int), f"invalid realization index: {index}")
        key = (scenario, index)
        _check(key not in result, f"duplicate run entry: {key}")
        result[key] = item
    return result


def _normalized_config(config: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(config)
    result["scenario"] = "__COMMON_SCENARIO__"
    result["seed"] = 0
    return result


def _read_events(path: Path) -> list[tuple[str, ...]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        _check(reader.fieldnames is not None, f"event CSV has no header: {path}")
        rows = []
        for row in reader:
            rows.append(
                (
                    row["gps_week"],
                    row["tow_ns"],
                    row["sow_sec"],
                    row["event_type"],
                    row["cycle_index"],
                    row["receiver_powered"],
                    row["signal_available"],
                    row["startup_mode"],
                )
            )
    return rows


def _observation_map(path: Path) -> dict[tuple[str, str, str, str], dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        _check(reader.fieldnames is not None, f"observation CSV has no header: {path}")
        rows: dict[tuple[str, str, str, str], dict[str, str]] = {}
        for row in reader:
            key = (row["gps_week"], row["tow_ns"], row["satellite_number"], row["signal_id"])
            _check(key not in rows, f"duplicate observation key in {path}: {key}")
            rows[key] = row
    return rows


def _validate_batch(
    batch_dir: Path,
    base_config: dict[str, Any],
    nav_path: Path,
) -> tuple[dict[str, Any], dict[tuple[str, int], dict[str, Any]]]:
    manifest_path = batch_dir / "batch_manifest.json"
    manifest = _read_json(manifest_path)
    _check(manifest.get("schema") == "gnss-synchronized-batch-v1", "unexpected batch manifest schema")
    _check(manifest.get("status") == "complete", "batch did not finish successfully")
    _check(manifest.get("completed_runs") == 24, "batch did not complete exactly 24 runs")

    common = manifest.get("common")
    _check(isinstance(common, dict), "batch common section is missing")
    _check(common.get("start_gpst") == {"week": START_WEEK, "sow_sec": START_SOW}, "common start GPST changed")
    _check(common.get("duration_sec") == DURATION_SEC, "common duration changed")
    _check(
        common.get("end_gpst") == {"week": START_WEEK, "sow_sec": START_SOW + DURATION_SEC},
        "common end GPST changed",
    )
    _check(common.get("realization_count_per_scenario") == COUNT, "batch realization count changed")

    inputs = manifest.get("inputs")
    _check(isinstance(inputs, dict) and isinstance(inputs.get("rinex_nav"), dict), "NAV identity missing")
    _check(inputs["rinex_nav"].get("sha256") == _sha256(nav_path), "batch manifest NAV SHA256 mismatch")
    _check(inputs["rinex_nav"].get("size_bytes") == nav_path.stat().st_size, "batch manifest NAV size mismatch")

    run_map = _run_map(manifest)
    _check(len(run_map) == 24, "batch manifest does not contain exactly 24 unique runs")
    all_seeds = [run["seed"] for run in run_map.values()]
    _check(len(all_seeds) == len(set(all_seeds)), "batch seeds are not globally unique")

    reference_resolved: dict[str, Any] | None = None
    reference_nav_identity: dict[str, Any] | None = None
    reference_simulator_commit: str | None = None
    reference_rtklib_commit: str | None = None

    for scenario in SCENARIOS:
        scenario_seeds = [run_map[(scenario, index)]["seed"] for index in range(1, COUNT + 1)]
        _check(scenario_seeds == EXPECTED_SEEDS[scenario], f"{scenario} seeds changed: {scenario_seeds}")
        for index in range(1, COUNT + 1):
            run = run_map[(scenario, index)]
            expected_stem = f"{scenario}_{index:02d}"
            expected_dir = f"runs/{expected_stem}"
            _check(run.get("run_dir") == expected_dir, f"unexpected run directory for {expected_stem}")
            _check(run.get("output") == f"{expected_dir}/{expected_stem}.log", f"unexpected log path for {expected_stem}")
            _check(run.get("config") == f"{expected_dir}/config.json", f"unexpected config path for {expected_stem}")
            _check(run.get("status") == "complete", f"run is not complete: {expected_stem}")

            run_dir = batch_dir / expected_dir
            output_path = batch_dir / run["output"]
            config_path = batch_dir / run["config"]
            _check(output_path.is_file() and output_path.stat().st_size > 0, f"receiver log missing: {output_path}")
            _check(run.get("output_sha256") == _sha256(output_path), f"receiver log hash mismatch: {expected_stem}")
            _check(run.get("config_sha256") == _sha256(config_path), f"config hash mismatch: {expected_stem}")

            effective = _read_json(config_path)
            _check(effective.get("scenario") == scenario, f"config scenario mismatch: {expected_stem}")
            _check(effective.get("seed") == run["seed"], f"config seed mismatch: {expected_stem}")
            _check(
                _normalized_config(effective) == _normalized_config(base_config),
                f"shared physical config changed for {expected_stem}",
            )

            for artifact in TRUTH_ARTIFACTS:
                artifact_path = run_dir / artifact
                _check(artifact_path.is_file() and artifact_path.stat().st_size > 0, f"truth artifact missing: {artifact_path}")

            run_manifest = _read_json(run_dir / "run_manifest.json")
            _check(run_manifest.get("random_seed") == run["seed"], f"truth random_seed mismatch: {expected_stem}")
            _check(
                run_manifest.get("start_time") == {"gps_week": START_WEEK, "tow_ns": int(START_SOW * 1_000_000_000)},
                f"truth start GPST mismatch: {expected_stem}",
            )
            resolved = run_manifest.get("resolved_config")
            _check(isinstance(resolved, dict), f"resolved config missing: {expected_stem}")
            _check(resolved.get("duration_ns") == int(DURATION_SEC * 1_000_000_000), f"duration mismatch: {expected_stem}")
            _check(resolved.get("scenario") == scenario, f"resolved scenario mismatch: {expected_stem}")
            _check(resolved.get("seed") == run["seed"], f"resolved seed mismatch: {expected_stem}")

            normalized_resolved = _normalized_config(resolved)
            if reference_resolved is None:
                reference_resolved = normalized_resolved
            else:
                _check(normalized_resolved == reference_resolved, f"resolved shared config drifted: {expected_stem}")

            nav_identity = run_manifest.get("rinex_nav")
            _check(isinstance(nav_identity, dict), f"truth NAV identity missing: {expected_stem}")
            _check(nav_identity.get("name") == nav_path.name, f"truth NAV filename changed: {expected_stem}")
            if reference_nav_identity is None:
                reference_nav_identity = nav_identity
            else:
                _check(nav_identity == reference_nav_identity, f"truth NAV identity drifted: {expected_stem}")

            simulator_commit = run_manifest.get("simulator_commit_sha")
            rtklib_commit = run_manifest.get("rtklib_commit_sha")
            _check(isinstance(simulator_commit, str) and simulator_commit, f"simulator commit missing: {expected_stem}")
            _check(isinstance(rtklib_commit, str) and rtklib_commit, f"RTKLIB commit missing: {expected_stem}")
            if reference_simulator_commit is None:
                reference_simulator_commit = simulator_commit
                reference_rtklib_commit = rtklib_commit
            else:
                _check(simulator_commit == reference_simulator_commit, f"simulator commit drifted: {expected_stem}")
                _check(rtklib_commit == reference_rtklib_commit, f"RTKLIB commit drifted: {expected_stem}")

    for scenario, required_events in (("REA", {"SIGNAL_ON", "SIGNAL_OFF"}), ("TTFF", {"POWER_ON", "POWER_OFF"})):
        event_bytes: bytes | None = None
        first_events: list[tuple[str, ...]] | None = None
        for index in range(1, COUNT + 1):
            event_path = batch_dir / run_map[(scenario, index)]["run_dir"] / "event_truth.csv"
            current_bytes = event_path.read_bytes()
            if event_bytes is None:
                event_bytes = current_bytes
                first_events = _read_events(event_path)
            else:
                _check(current_bytes == event_bytes, f"{scenario} event schedule differs across seeds")
        assert first_events is not None
        event_types = {row[3] for row in first_events}
        _check(required_events.issubset(event_types), f"{scenario} short regression did not exercise required transitions")

    ks_observations = [
        _observation_map(batch_dir / run_map[("KS", index)]["run_dir"] / "observation_truth.csv")
        for index in range(1, COUNT + 1)
    ]
    diversity_found = False
    for left_index in range(COUNT):
        for right_index in range(left_index + 1, COUNT):
            common_keys = sorted(set(ks_observations[left_index]).intersection(ks_observations[right_index]))
            for key in common_keys:
                left = ks_observations[left_index][key]
                right = ks_observations[right_index][key]
                if left["cn0_dbhz"] == right["cn0_dbhz"]:
                    continue
                for field in PHYSICAL_OBSERVATION_FIELDS:
                    _check(left[field] == right[field], f"seed changed physical truth field {field} at {key}")
                diversity_found = True
                break
            if diversity_found:
                break
        if diversity_found:
            break
    _check(diversity_found, "different KS seeds produced no CN0 diversity on any common observation")

    return manifest, run_map


def _validate_rerun(
    first_dir: Path,
    second_dir: Path,
    first_manifest: dict[str, Any],
    second_manifest: dict[str, Any],
    first_runs: dict[tuple[str, int], dict[str, Any]],
    second_runs: dict[tuple[str, int], dict[str, Any]],
) -> None:
    _check(first_manifest == second_manifest, "full batch manifest changed across identical rerun")
    for key in sorted(first_runs):
        first = first_runs[key]
        second = second_runs[key]
        _check(first["output_sha256"] == second["output_sha256"], f"receiver log hash changed across rerun: {key}")
        first_run_dir = first_dir / first["run_dir"]
        second_run_dir = second_dir / second["run_dir"]
        for relative in (Path(first["output"]).name, "config.json", *TRUTH_ARTIFACTS):
            first_path = first_run_dir / relative
            second_path = second_run_dir / relative
            _check(_sha256(first_path) == _sha256(second_path), f"artifact changed across rerun: {key} {relative}")


def validate(args: argparse.Namespace) -> int:
    source_dir = Path(args.source_dir).resolve()
    build_dir = Path(args.build_dir).resolve()
    work_dir = Path(args.work_dir).resolve()
    simulator = _find_simulator(build_dir)
    nav_path = source_dir / "tests" / "data" / "minimal" / "brd400dlr_rinex4_acceptance_nav.rnx"
    _check(nav_path.is_file(), f"real RINEX NAV fixture missing: {nav_path}")

    if work_dir.exists():
        shutil.rmtree(work_dir)
    work_dir.mkdir(parents=True)
    base_config_path = work_dir / "base_config.json"
    base_config = _make_short_config(source_dir, base_config_path)

    first_dir = work_dir / "batch_a"
    second_dir = work_dir / "batch_b"
    _run_batch(source_dir, simulator, base_config_path, nav_path, first_dir)
    first_manifest, first_runs = _validate_batch(first_dir, base_config, nav_path)
    _run_batch(source_dir, simulator, base_config_path, nav_path, second_dir)
    second_manifest, second_runs = _validate_batch(second_dir, base_config, nav_path)
    _validate_rerun(first_dir, second_dir, first_manifest, second_manifest, first_runs, second_runs)

    print(
        "synchronized batch integration: PASS "
        "(24 runs x 2, common GPST, event alignment, seed diversity, physical-truth invariance, reproducibility)"
    )
    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate synchronized multi-seed batch generation end to end.")
    parser.add_argument("--source-dir", required=True)
    parser.add_argument("--build-dir", required=True)
    parser.add_argument("--work-dir", required=True)
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    try:
        return validate(args)
    except ValidationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
