#!/usr/bin/env python3
"""Generate synchronized KS/REA/TTFF batches from one common simulator config."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Any

SCENARIOS = ("KS", "REA", "TTFF")
BATCH_SCHEMA = "gnss-synchronized-batch-v1"
GPS_WEEK_SEC = 604800.0
UINT64_MAX = (1 << 64) - 1


class BatchError(RuntimeError):
    pass


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_json_object(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise BatchError(f"cannot read JSON config {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise BatchError(f"config root must be a JSON object: {path}")
    return data


def _duration_sec(config: dict[str, Any]) -> float:
    value = config.get("duration_sec")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BatchError("base config must contain numeric duration_sec")
    duration = float(value)
    if not math.isfinite(duration) or duration <= 0.0:
        raise BatchError("base config duration_sec must be finite and > 0")
    return duration


def _normalize_gpst(week: int, sow_sec: float) -> tuple[int, float]:
    if week < 0:
        raise BatchError("GPS week must be non-negative")
    if not math.isfinite(sow_sec) or sow_sec < 0.0 or sow_sec >= GPS_WEEK_SEC:
        raise BatchError("GPS SOW must be finite and within [0, 604800)")
    return week, sow_sec


def _add_gpst_seconds(week: int, sow_sec: float, delta_sec: float) -> tuple[int, float]:
    total = sow_sec + delta_sec
    if not math.isfinite(total) or total < 0.0:
        raise BatchError("invalid GPST duration")
    week_delta = int(total // GPS_WEEK_SEC)
    end_sow = total - week_delta * GPS_WEEK_SEC
    if end_sow >= GPS_WEEK_SEC:
        week_delta += 1
        end_sow -= GPS_WEEK_SEC
    return week + week_delta, end_sow


def _seed_for(seed_base: int, seed_stride: int, scenario_index: int, realization_index: int) -> int:
    return seed_base + scenario_index * seed_stride + (realization_index - 1)


def _build_plan(
    config: dict[str, Any],
    week: int,
    sow_sec: float,
    count: int,
    seed_base: int,
    seed_stride: int,
) -> dict[str, Any]:
    _normalize_gpst(week, sow_sec)
    duration = _duration_sec(config)
    if count <= 0:
        raise BatchError("realization count must be > 0")
    if seed_base < 0 or seed_base > UINT64_MAX:
        raise BatchError("seed base must fit uint64")
    if seed_stride <= 0:
        raise BatchError("seed stride must be > 0")
    if count > seed_stride:
        raise BatchError("realization count must not exceed seed stride")

    end_week, end_sow = _add_gpst_seconds(week, sow_sec, duration)
    width = max(2, len(str(count)))
    runs: list[dict[str, Any]] = []
    seen_seeds: set[int] = set()

    for scenario_index, scenario in enumerate(SCENARIOS):
        for realization_index in range(1, count + 1):
            seed = _seed_for(seed_base, seed_stride, scenario_index, realization_index)
            if seed < 0 or seed > UINT64_MAX:
                raise BatchError("seed schedule overflows uint64")
            if seed in seen_seeds:
                raise BatchError(f"seed collision detected: {seed}")
            seen_seeds.add(seed)
            stem = f"{scenario}_{realization_index:0{width}d}"
            runs.append(
                {
                    "scenario": scenario,
                    "realization_index": realization_index,
                    "seed": seed,
                    "output": f"{stem}.log",
                    "config": f"configs/{stem}.json",
                    "status": "planned",
                }
            )

    return {
        "schema": BATCH_SCHEMA,
        "status": "planned",
        "common": {
            "start_gpst": {"week": week, "sow_sec": sow_sec},
            "duration_sec": duration,
            "end_gpst": {"week": end_week, "sow_sec": end_sow},
            "realization_count_per_scenario": count,
            "seed_base": seed_base,
            "seed_stride": seed_stride,
        },
        "runs": runs,
    }


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(path.name + ".tmp")
    with tmp_path.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(data, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    os.replace(tmp_path, path)


def _write_effective_configs(output_dir: Path, base_config: dict[str, Any], manifest: dict[str, Any]) -> None:
    for run in manifest["runs"]:
        effective = copy.deepcopy(base_config)
        effective["scenario"] = run["scenario"]
        effective["seed"] = run["seed"]
        config_path = output_dir / run["config"]
        _write_json(config_path, effective)
        run["config_sha256"] = _sha256_file(config_path)


def _resolve_executable(value: str) -> Path:
    supplied = Path(value)
    if supplied.is_file():
        return supplied.resolve()
    found = shutil.which(value)
    if found:
        return Path(found).resolve()
    raise BatchError(f"simulator executable not found: {value}")


def _simulator_identity(executable: Path) -> dict[str, str]:
    try:
        completed = subprocess.run(
            [str(executable), "--version"],
            check=True,
            text=True,
            capture_output=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise BatchError(f"failed to query simulator version: {exc}") from exc

    lines = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    identity: dict[str, str] = {"executable": executable.name}
    if lines:
        identity["version"] = lines[0]
    for line in lines[1:]:
        if line.lower().startswith("rtklib commit:"):
            identity["rtklib_commit"] = line.split(":", 1)[1].strip()
        elif line.lower().startswith("source commit:"):
            identity["source_commit"] = line.split(":", 1)[1].strip()
    return identity


def _input_identity(path: Path, original_argument: str) -> dict[str, Any]:
    if not path.is_file():
        raise BatchError(f"input file not found: {path}")
    return {
        "path": original_argument,
        "size_bytes": path.stat().st_size,
        "sha256": _sha256_file(path),
    }


def _prepare_output_paths(output_dir: Path, manifest_path: Path, manifest: dict[str, Any], overwrite: bool) -> None:
    if manifest_path.exists() and not overwrite:
        raise BatchError(f"manifest already exists: {manifest_path}; use --overwrite to replace it")
    for run in manifest["runs"]:
        for relative in (run["output"], run["config"]):
            path = output_dir / relative
            if path.exists():
                if not overwrite:
                    raise BatchError(f"planned batch path already exists: {path}; use --overwrite to replace it")
                if path.is_dir():
                    raise BatchError(f"planned batch path is a directory: {path}")
                path.unlink()


def _run_one(
    executable: Path,
    config_path: Path,
    nav_path: Path,
    output_path: Path,
    week: int,
    sow_sec: float,
    cn0_model_path: Path | None,
) -> None:
    command = [
        str(executable),
        "--config",
        str(config_path),
        "--nav",
        str(nav_path),
        "--output",
        str(output_path),
        "--week",
        str(week),
        "--sow",
        format(sow_sec, ".12g"),
    ]
    if cn0_model_path is not None:
        command.extend(["--cn0-model", str(cn0_model_path)])
    try:
        subprocess.run(command, check=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        raise BatchError(f"simulator invocation failed for {output_path.name}: {exc}") from exc


def generate_batch(args: argparse.Namespace) -> int:
    config_path = Path(args.config)
    nav_path = Path(args.nav)
    output_dir = Path(args.output_dir)
    cn0_model_path = Path(args.cn0_model) if args.cn0_model else None
    manifest_path = output_dir / args.manifest_name

    if not config_path.is_file():
        raise BatchError(f"base config not found: {config_path}")
    if not nav_path.is_file():
        raise BatchError(f"RINEX NAV input not found: {nav_path}")
    if cn0_model_path is not None and not cn0_model_path.is_file():
        raise BatchError(f"CN0 model input not found: {cn0_model_path}")

    base_config = _read_json_object(config_path)
    manifest = _build_plan(
        base_config,
        args.week,
        args.sow,
        args.count,
        args.seed_base,
        args.seed_stride,
    )
    _prepare_output_paths(output_dir, manifest_path, manifest, args.overwrite)

    manifest["inputs"] = {
        "base_config": _input_identity(config_path, args.config),
        "rinex_nav": _input_identity(nav_path, args.nav),
    }
    if cn0_model_path is not None:
        manifest["inputs"]["cn0_model"] = _input_identity(cn0_model_path, args.cn0_model)

    executable: Path | None = None
    if not args.plan_only:
        if not args.simulator:
            raise BatchError("--simulator is required unless --plan-only is used")
        executable = _resolve_executable(args.simulator)
        manifest["simulator"] = _simulator_identity(executable)
    elif args.simulator:
        executable = _resolve_executable(args.simulator)
        manifest["simulator"] = _simulator_identity(executable)

    output_dir.mkdir(parents=True, exist_ok=True)
    _write_effective_configs(output_dir, base_config, manifest)
    _write_json(manifest_path, manifest)

    if args.plan_only:
        print(f"planned {len(manifest['runs'])} synchronized runs: {manifest_path}")
        return 0

    start = manifest["common"]["start_gpst"]
    completed_count = 0
    try:
        assert executable is not None
        for run in manifest["runs"]:
            run["status"] = "running"
            _write_json(manifest_path, manifest)
            output_path = output_dir / run["output"]
            config_snapshot = output_dir / run["config"]
            if output_path.exists():
                output_path.unlink()
            _run_one(
                executable,
                config_snapshot,
                nav_path,
                output_path,
                start["week"],
                start["sow_sec"],
                cn0_model_path,
            )
            run["output_sha256"] = _sha256_file(output_path)
            run["status"] = "complete"
            completed_count += 1
            _write_json(manifest_path, manifest)
    except BatchError as exc:
        manifest["status"] = "failed"
        manifest["completed_runs"] = completed_count
        manifest["error"] = str(exc)
        for run in manifest["runs"]:
            if run["status"] == "running":
                run["status"] = "failed"
                break
        _write_json(manifest_path, manifest)
        raise

    manifest["status"] = "complete"
    manifest["completed_runs"] = completed_count
    manifest.pop("error", None)
    _write_json(manifest_path, manifest)
    print(f"generated {completed_count} synchronized runs: {manifest_path}")
    return 0


def _assert(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def _self_test() -> int:
    base = {
        "schema_version": 1,
        "scenario": "KS",
        "duration_sec": 30.0,
        "sampling_rate_hz": 10,
        "receiver": {"latitude_deg": 20.0, "longitude_deg": 120.0, "height_m": 100.0},
        "ttff": {"startup_mode": "HOT", "power_on_sec": 300.0, "power_off_sec": 30.0},
        "rea": {"signal_on_sec": 300.0, "signal_off_sec": 10.0},
        "seed": 1,
    }

    first = _build_plan(base, 2347, 604790.0, 8, 1001, 1000)
    second = _build_plan(base, 2347, 604790.0, 8, 1001, 1000)
    _assert(first == second, "planner is not deterministic")
    _assert(len(first["runs"]) == 24, "expected 24 planned runs")
    _assert(first["common"]["start_gpst"] == {"week": 2347, "sow_sec": 604790.0}, "start GPST changed")
    _assert(first["common"]["end_gpst"] == {"week": 2348, "sow_sec": 20.0}, "week rollover is wrong")

    names = [run["output"] for run in first["runs"]]
    _assert(names[0] == "KS_01.log" and names[7] == "KS_08.log", "KS names are wrong")
    _assert(names[8] == "REA_01.log" and names[16] == "TTFF_01.log", "scenario names are wrong")

    seeds = [run["seed"] for run in first["runs"]]
    _assert(len(seeds) == len(set(seeds)), "batch seeds are not globally unique")
    _assert(seeds[:8] == list(range(1001, 1009)), "KS seed range is wrong")
    _assert(seeds[8:16] == list(range(2001, 2009)), "REA seed range is wrong")
    _assert(seeds[16:24] == list(range(3001, 3009)), "TTFF seed range is wrong")

    with tempfile.TemporaryDirectory() as temp_dir:
        output_dir = Path(temp_dir)
        manifest = copy.deepcopy(first)
        _write_effective_configs(output_dir, base, manifest)
        for run in manifest["runs"]:
            snapshot = _read_json_object(output_dir / run["config"])
            _assert(snapshot["scenario"] == run["scenario"], "scenario snapshot mismatch")
            _assert(snapshot["seed"] == run["seed"], "seed snapshot mismatch")
            restored = copy.deepcopy(snapshot)
            restored["scenario"] = base["scenario"]
            restored["seed"] = base["seed"]
            _assert(restored == base, "batch snapshot changed a shared physical config field")
            _assert("config_sha256" in run, "config snapshot identity is missing")

    for bad_count, bad_stride in ((0, 1000), (1001, 1000)):
        try:
            _build_plan(base, 2347, 100.0, bad_count, 1001, bad_stride)
        except BatchError:
            pass
        else:
            raise AssertionError("invalid count/stride was accepted")

    bad = copy.deepcopy(base)
    bad["duration_sec"] = 0
    try:
        _build_plan(bad, 2347, 100.0, 8, 1001, 1000)
    except BatchError:
        pass
    else:
        raise AssertionError("invalid duration was accepted")

    try:
        _build_plan(base, 2347, 100.0, 8, UINT64_MAX - 3, 1000)
    except BatchError:
        pass
    else:
        raise AssertionError("seed overflow was accepted")

    print("batch generation self-test: PASS")
    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate synchronized KS/REA/TTFF batches with one GPST window and deterministic seeds."
    )
    parser.add_argument("--self-test", action="store_true", help="run deterministic planner self-tests")
    parser.add_argument("--simulator", help="path/name of gnss-data-simulator executable")
    parser.add_argument("--config", help="common base simulator JSON config")
    parser.add_argument("--nav", help="real RINEX NAV input")
    parser.add_argument("--output-dir", help="batch output directory")
    parser.add_argument("--week", type=int, help="common GPS week")
    parser.add_argument("--sow", type=float, help="common GPS second of week")
    parser.add_argument("--cn0-model", help="optional CN0 model passed unchanged to every run")
    parser.add_argument("--count", type=int, default=8, help="realizations per scenario (default: 8)")
    parser.add_argument("--seed-base", type=int, default=1001, help="KS realization-1 seed (default: 1001)")
    parser.add_argument(
        "--seed-stride",
        type=int,
        default=1000,
        help="seed offset between scenario blocks (default: 1000; REA starts at 2001, TTFF at 3001)",
    )
    parser.add_argument("--manifest-name", default="batch_manifest.json", help="manifest filename")
    parser.add_argument("--plan-only", action="store_true", help="write config snapshots/manifest without running simulator")
    parser.add_argument("--overwrite", action="store_true", help="replace planned batch files in the output directory")
    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    if args.self_test:
        return _self_test()

    required = ("config", "nav", "output_dir", "week", "sow")
    missing = [f"--{name.replace('_', '-')}" for name in required if getattr(args, name) is None]
    if missing:
        parser.error("missing required arguments: " + ", ".join(missing))

    try:
        return generate_batch(args)
    except BatchError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
