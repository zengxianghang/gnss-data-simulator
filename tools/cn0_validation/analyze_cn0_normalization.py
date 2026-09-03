#!/usr/bin/env python3
"""Quantify cross-station CN0 dispersion before and after normalization.

This tool consumes production gnss-cn0-metadata-v2 output. It does not
recompute builder normalization. Per-source absolute bin P50 is reconstructed
as reference P50 + the production per-source normalized delta.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "gnss-cn0-validation-v1"
INPUT_SCHEMA_VERSION = "gnss-cn0-metadata-v2"
INPUT_SEMANTIC = "NORMALIZED_ELEVATION_SHAPE"
DEFAULT_MIN_STATIONS = 3
PASS_REDUCTION_PERCENT = 30.0
PARTIAL_REDUCTION_PERCENT = 10.0
PASS_NONWORSE_FRACTION = 0.60
PARTIAL_NONWORSE_FRACTION = 0.50
TOLERANCE_DB = 1.0e-12

SignalKey = tuple[str, str]
GroupKey = tuple[str, str, float]


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _median(values: list[float]) -> float:
    if not values:
        raise ValueError("cannot take median of an empty sequence")
    return float(statistics.median(values))


def _pairwise_abs_differences(values: list[float]) -> list[float]:
    differences: list[float] = []
    for left in range(len(values)):
        for right in range(left + 1, len(values)):
            differences.append(abs(values[left] - values[right]))
    return differences


def _mpad(values: list[float]) -> float:
    return _median(_pairwise_abs_differences(values))


def _range(values: list[float]) -> float:
    return max(values) - min(values)


def _validate_metadata(metadata: dict[str, Any]) -> None:
    if metadata.get("schema_version") != INPUT_SCHEMA_VERSION:
        raise ValueError(f"expected {INPUT_SCHEMA_VERSION} metadata")
    if metadata.get("model_semantic") != INPUT_SEMANTIC:
        raise ValueError(f"expected model semantic {INPUT_SEMANTIC}")
    bins = metadata.get("elevation_bins")
    if not isinstance(bins, dict) or not _finite_number(bins.get("width_deg")) or float(bins["width_deg"]) <= 0.0:
        raise ValueError("metadata elevation bin width is missing or invalid")
    if not isinstance(metadata.get("sources"), list):
        raise ValueError("metadata sources must be an array")


def _source_label(source: dict[str, Any], index: int) -> str:
    station = str(source.get("station_name") or "").strip()
    observation = str(source.get("observation_file") or "").strip()
    return station or observation or f"source-{index}"


def _reference_map(source: dict[str, Any]) -> dict[SignalKey, float]:
    references: dict[SignalKey, float] = {}
    raw = source.get("references")
    if not isinstance(raw, list):
        return references
    for reference in raw:
        if not isinstance(reference, dict):
            continue
        constellation = reference.get("constellation")
        signal = reference.get("signal")
        if (
            isinstance(constellation, str)
            and constellation
            and isinstance(signal, str)
            and signal
            and reference.get("status") == "READY"
            and _finite_number(reference.get("p50_dbhz"))
        ):
            key = (constellation, signal)
            if key in references:
                raise ValueError(f"duplicate READY reference for constellation={constellation} signal={signal}")
            references[key] = float(reference["p50_dbhz"])
    return references


def _collect_groups(metadata: dict[str, Any]) -> tuple[dict[GroupKey, list[dict[str, Any]]], list[dict[str, Any]]]:
    groups: dict[GroupKey, list[dict[str, Any]]] = {}
    source_rows: list[dict[str, Any]] = []
    width_deg = float(metadata["elevation_bins"]["width_deg"])
    for source_index, source in enumerate(metadata["sources"]):
        if not isinstance(source, dict):
            continue
        references = _reference_map(source)
        label = _source_label(source, source_index)
        bins = source.get("bins")
        if not isinstance(bins, list):
            continue
        for bin_row in bins:
            if not isinstance(bin_row, dict):
                continue
            constellation = bin_row.get("constellation")
            signal = bin_row.get("signal")
            elevation_min = bin_row.get("elevation_min_deg")
            delta = bin_row.get("delta_p50_db")
            sample_count = bin_row.get("sample_count")
            reference_key = (constellation, signal)
            if (
                not isinstance(constellation, str)
                or not constellation
                or not isinstance(signal, str)
                or not signal
                or not _finite_number(elevation_min)
                or bin_row.get("status") != "READY"
                or bin_row.get("reference_ready") is not True
                or not isinstance(sample_count, int)
                or sample_count <= 0
                or not _finite_number(delta)
                or reference_key not in references
            ):
                continue
            delta_db = float(delta)
            reference_dbhz = references[reference_key]
            row = {
                "source": label,
                "observation_file": str(source.get("observation_file") or ""),
                "observation_fnv1a64": str(source.get("observation_fnv1a64") or ""),
                "navigation_file": str(source.get("navigation_file") or ""),
                "navigation_fnv1a64": str(source.get("navigation_fnv1a64") or ""),
                "receiver_type": str(source.get("receiver_type") or ""),
                "antenna_type": str(source.get("antenna_type") or ""),
                "constellation": constellation,
                "signal": signal,
                "elevation_min_deg": float(elevation_min),
                "elevation_max_deg": float(elevation_min) + width_deg,
                "sample_count": sample_count,
                "reference_p50_dbhz": reference_dbhz,
                "delta_p50_db": delta_db,
                "absolute_p50_dbhz": reference_dbhz + delta_db,
            }
            source_rows.append(row)
            groups.setdefault((constellation, signal, float(elevation_min)), []).append(row)
    return groups, source_rows


def _bin_metrics(groups: dict[GroupKey, list[dict[str, Any]]], min_stations: int, width_deg: float) -> list[dict[str, Any]]:
    metrics: list[dict[str, Any]] = []
    for (constellation, signal, elevation_min), rows in sorted(groups.items(), key=lambda item: item[0]):
        unique_sources = {row["source"] for row in rows}
        if len(unique_sources) < min_stations:
            continue
        if len(unique_sources) != len(rows):
            raise ValueError(
                f"duplicate source contribution for constellation={constellation} signal={signal} "
                f"elevation_min={elevation_min}"
            )
        absolute = [float(row["absolute_p50_dbhz"]) for row in rows]
        normalized = [float(row["delta_p50_db"]) for row in rows]
        raw_mpad = _mpad(absolute)
        normalized_mpad = _mpad(normalized)
        improvement_percent = None
        if raw_mpad > TOLERANCE_DB:
            improvement_percent = 100.0 * (raw_mpad - normalized_mpad) / raw_mpad
        metrics.append(
            {
                "constellation": constellation,
                "signal": signal,
                "elevation_min_deg": elevation_min,
                "elevation_max_deg": elevation_min + width_deg,
                "station_count": len(rows),
                "raw_mpad_db": raw_mpad,
                "normalized_mpad_db": normalized_mpad,
                "raw_range_db": _range(absolute),
                "normalized_range_db": _range(normalized),
                "improvement_percent": improvement_percent,
                "nonworse": normalized_mpad <= raw_mpad + TOLERANCE_DB,
            }
        )
    return metrics


def _signal_summaries(metrics: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[SignalKey, list[dict[str, Any]]] = {}
    for row in metrics:
        key = (str(row["constellation"]), str(row["signal"]))
        grouped.setdefault(key, []).append(row)
    result: list[dict[str, Any]] = []
    for constellation, signal in sorted(grouped):
        rows = grouped[(constellation, signal)]
        raw = _median([float(row["raw_mpad_db"]) for row in rows])
        normalized = _median([float(row["normalized_mpad_db"]) for row in rows])
        reduction = None if raw <= TOLERANCE_DB else 100.0 * (raw - normalized) / raw
        result.append(
            {
                "constellation": constellation,
                "signal": signal,
                "eligible_bin_count": len(rows),
                "median_raw_mpad_db": raw,
                "median_normalized_mpad_db": normalized,
                "reduction_percent": reduction,
                "nonworse_fraction": sum(bool(row["nonworse"]) for row in rows) / len(rows),
            }
        )
    return result


def _classification(metrics: list[dict[str, Any]]) -> dict[str, Any]:
    if not metrics:
        return {
            "result": "FAIL",
            "reason": "no constellation/signal/elevation bins have the required cross-station support",
            "median_raw_mpad_db": None,
            "median_normalized_mpad_db": None,
            "reduction_percent": None,
            "nonworse_fraction": None,
        }
    raw = _median([float(row["raw_mpad_db"]) for row in metrics])
    normalized = _median([float(row["normalized_mpad_db"]) for row in metrics])
    reduction = None if raw <= TOLERANCE_DB else 100.0 * (raw - normalized) / raw
    nonworse = sum(bool(row["nonworse"]) for row in metrics) / len(metrics)
    if reduction is not None and reduction >= PASS_REDUCTION_PERCENT and nonworse >= PASS_NONWORSE_FRACTION:
        result = "PASS"
        reason = "cross-station dispersion materially decreased under the predeclared criteria"
    elif (reduction is not None and reduction >= PARTIAL_REDUCTION_PERCENT) or nonworse >= PARTIAL_NONWORSE_FRACTION:
        result = "PARTIAL"
        reason = "normalization improves portability only partially under the predeclared criteria"
    else:
        result = "FAIL"
        reason = "normalization does not materially reduce cross-station dispersion under the predeclared criteria"
    return {
        "result": result,
        "reason": reason,
        "median_raw_mpad_db": raw,
        "median_normalized_mpad_db": normalized,
        "reduction_percent": reduction,
        "nonworse_fraction": nonworse,
    }


def analyze(metadata: dict[str, Any], min_stations: int = DEFAULT_MIN_STATIONS) -> dict[str, Any]:
    _validate_metadata(metadata)
    if min_stations < 2:
        raise ValueError("min_stations must be at least 2")
    groups, source_rows = _collect_groups(metadata)
    width_deg = float(metadata["elevation_bins"]["width_deg"])
    metrics = _bin_metrics(groups, min_stations, width_deg)
    result = {
        "schema_version": SCHEMA_VERSION,
        "input_schema_version": INPUT_SCHEMA_VERSION,
        "model_semantic": INPUT_SEMANTIC,
        "metric": "median_pairwise_absolute_difference_db",
        "identity_key": "constellation+signal+elevation_bin",
        "eligibility": {
            "min_stations": min_stations,
            "source_bin_status": "READY",
            "reference_status": "READY",
            "missing_bins_interpolated": False,
        },
        "predeclared_thresholds": {
            "pass_reduction_percent": PASS_REDUCTION_PERCENT,
            "pass_nonworse_fraction": PASS_NONWORSE_FRACTION,
            "partial_reduction_percent": PARTIAL_REDUCTION_PERCENT,
            "partial_nonworse_fraction": PARTIAL_NONWORSE_FRACTION,
        },
        "source_count": len(metadata["sources"]),
        "eligible_source_bin_rows": len(source_rows),
        "eligible_cross_station_bins": len(metrics),
        "classification": _classification(metrics),
        "signal_summaries": _signal_summaries(metrics),
        "bins": metrics,
    }
    return result


def _write_csv(path: Path, analysis: dict[str, Any]) -> None:
    fields = [
        "constellation",
        "signal",
        "elevation_min_deg",
        "elevation_max_deg",
        "station_count",
        "raw_mpad_db",
        "normalized_mpad_db",
        "raw_range_db",
        "normalized_range_db",
        "improvement_percent",
        "nonworse",
    ]
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for row in analysis["bins"]:
            writer.writerow({field: row.get(field) for field in fields})


def _write_outputs(prefix: Path, analysis: dict[str, Any]) -> None:
    json_path = Path(str(prefix) + ".json")
    csv_path = Path(str(prefix) + ".csv")
    json_path.parent.mkdir(parents=True, exist_ok=True)
    with json_path.open("w", encoding="utf-8", newline="\n") as output:
        json.dump(analysis, output, sort_keys=True, indent=2, ensure_ascii=False)
        output.write("\n")
    _write_csv(csv_path, analysis)


def _self_test_metadata() -> dict[str, Any]:
    def source(name: str, reference: float) -> dict[str, Any]:
        return {
            "station_name": name,
            "observation_file": f"{name}.rnx",
            "observation_fnv1a64": name,
            "navigation_file": "real.nav",
            "navigation_fnv1a64": "nav",
            "receiver_type": f"RX-{name}",
            "antenna_type": f"ANT-{name}",
            "references": [
                {
                    "constellation": "GPS",
                    "signal": "1C",
                    "status": "READY",
                    "count": 100,
                    "p50_dbhz": reference,
                },
                {
                    "constellation": "QZSS",
                    "signal": "1C",
                    "status": "READY",
                    "count": 100,
                    "p50_dbhz": reference + 3.0,
                },
            ],
            "bins": [
                {
                    "constellation": "GPS",
                    "signal": "1C",
                    "elevation_min_deg": 10.0,
                    "status": "READY",
                    "sample_count": 100,
                    "reference_ready": True,
                    "delta_p50_db": -10.0,
                },
                {
                    "constellation": "GPS",
                    "signal": "1C",
                    "elevation_min_deg": 60.0,
                    "status": "READY",
                    "sample_count": 100,
                    "reference_ready": True,
                    "delta_p50_db": -2.0,
                },
                {
                    "constellation": "GPS",
                    "signal": "1C",
                    "elevation_min_deg": 30.0,
                    "status": "SPARSE",
                    "sample_count": 1,
                    "reference_ready": True,
                    "delta_p50_db": -5.0,
                },
                {
                    "constellation": "QZSS",
                    "signal": "1C",
                    "elevation_min_deg": 10.0,
                    "status": "READY",
                    "sample_count": 100,
                    "reference_ready": True,
                    "delta_p50_db": -6.0,
                },
                {
                    "constellation": "QZSS",
                    "signal": "1C",
                    "elevation_min_deg": 60.0,
                    "status": "READY",
                    "sample_count": 100,
                    "reference_ready": True,
                    "delta_p50_db": -1.0,
                },
            ],
        }

    return {
        "schema_version": INPUT_SCHEMA_VERSION,
        "model_schema_version": "gnss-cn0-model-v2",
        "model_semantic": INPUT_SEMANTIC,
        "elevation_bins": {"min_deg": 0.0, "max_deg": 90.0, "width_deg": 5.0},
        "sources": [source("A", 48.0), source("B", 56.0), source("C", 52.0)],
    }


def self_test() -> int:
    analysis = analyze(_self_test_metadata(), min_stations=3)
    if analysis["classification"]["result"] != "PASS":
        raise AssertionError("constant station offsets should produce PASS after normalization")
    if analysis["eligible_cross_station_bins"] != 4:
        raise AssertionError("GPS/QZSS identities must remain separate and SPARSE bins must be excluded")
    if len(analysis["signal_summaries"]) != 2:
        raise AssertionError("same RINEX signal code in different constellations must produce separate summaries")
    if {(row["constellation"], row["signal"]) for row in analysis["signal_summaries"]} != {
        ("GPS", "1C"),
        ("QZSS", "1C"),
    }:
        raise AssertionError("constellation/signal identity was not preserved")
    if abs(float(analysis["classification"]["median_normalized_mpad_db"])) > TOLERANCE_DB:
        raise AssertionError("normalized MPAD should be zero for equal shapes")
    reversed_metadata = _self_test_metadata()
    reversed_metadata["sources"] = list(reversed(reversed_metadata["sources"]))
    if analyze(reversed_metadata, min_stations=3) != analysis:
        raise AssertionError("analysis must be source-order deterministic")
    print("cn0 normalization analyzer self-test: PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metadata", type=Path, help="production gnss-cn0-metadata-v2 JSON")
    parser.add_argument("--output-prefix", type=Path, help="output path prefix for .json and .csv")
    parser.add_argument("--min-stations", type=int, default=DEFAULT_MIN_STATIONS)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        return self_test()
    if args.metadata is None or args.output_prefix is None:
        parser.error("--metadata and --output-prefix are required unless --self-test is used")
    with args.metadata.open("r", encoding="utf-8") as source:
        metadata = json.load(source)
    analysis = analyze(metadata, min_stations=args.min_stations)
    _write_outputs(args.output_prefix, analysis)
    classification = analysis["classification"]
    print(
        f"CN0 normalization validation: {classification['result']} "
        f"eligible_bins={analysis['eligible_cross_station_bins']} "
        f"raw_mpad={classification['median_raw_mpad_db']} "
        f"normalized_mpad={classification['median_normalized_mpad_db']} "
        f"reduction_percent={classification['reduction_percent']}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
