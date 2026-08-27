#!/usr/bin/env python3
"""Reduce a real BRD400DLR RINEX 4 NAV file to a deterministic CI fixture."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Set, Tuple

TARGET_EPOCH = dt.datetime(2025, 1, 3, 1, 15, 0)
SIM_SYSTEMS = {"G", "R", "E", "C", "J"}
LEGACY_FAMILIES = {
    "G": ("LNAV",),
    "R": ("FDMA",),
    "E": ("INAV", "FNAV"),
    "C": ("D1", "D2"),
    "J": ("LNAV",),
}
KNOWN_MODERN = {
    ("G", "CNAV"), ("G", "CNV2"),
    ("J", "CNAV"), ("J", "CNV2"),
    ("C", "CNV1"), ("C", "CNV2"), ("C", "CNV3"),
}
REQUIRED_MODERN = {
    ("G", "CNAV"),
    ("J", "CNAV"), ("J", "CNV2"),
    ("C", "CNV1"), ("C", "CNV2"), ("C", "CNV3"),
}
AUX_TYPES = {"STO", "EOP", "ION"}


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def split_header_records(text: str) -> Tuple[List[str], List[List[str]]]:
    lines = text.splitlines(keepends=True)
    header: List[str] = []
    body_start = -1
    for index, line in enumerate(lines):
        header.append(line)
        if "END OF HEADER" in line:
            body_start = index + 1
            break
    if body_start < 0:
        raise ValueError("RINEX file has no END OF HEADER")
    records: List[List[str]] = []
    current: List[str] = []
    for line in lines[body_start:]:
        if line.startswith(">"):
            if current:
                records.append(current)
            current = [line]
        elif current:
            current.append(line)
        elif line.strip():
            raise ValueError("non-empty RINEX 4 body text before first record marker")
    if current:
        records.append(current)
    return header, records


def marker_fields(record: Sequence[str]) -> List[str]:
    if not record or not record[0].startswith(">"):
        raise ValueError("invalid RINEX 4 record marker")
    return record[0].split()


def record_type(record: Sequence[str]) -> str:
    fields = marker_fields(record)
    return fields[1] if len(fields) >= 2 else ""


def eph_identity(record: Sequence[str]) -> Tuple[str, str, str]:
    fields = marker_fields(record)
    if len(fields) < 4 or fields[1] != "EPH" or len(fields[2]) < 2:
        return "", "", ""
    return fields[2][0], fields[2], fields[3]


def record_epoch(record: Sequence[str]) -> dt.datetime | None:
    if len(record) < 2:
        return None
    fields = record[1].split()
    if len(fields) < 7:
        return None
    try:
        second = float(fields[6])
        whole_second = int(second)
        microsecond = int(round((second - whole_second) * 1_000_000.0))
        return dt.datetime(int(fields[1]), int(fields[2]), int(fields[3]), int(fields[4]), int(fields[5]),
                           whole_second, microsecond)
    except (ValueError, OverflowError):
        return None


def aux_key(record: Sequence[str]) -> Tuple[str, str, str]:
    fields = marker_fields(record)
    kind = fields[1] if len(fields) >= 2 else ""
    system = fields[2][0] if len(fields) >= 3 and fields[2] else "?"
    subtype = fields[3] if len(fields) >= 4 else ""
    return kind, system, subtype


def time_score(epoch: dt.datetime) -> Tuple[int, float]:
    delta = (epoch - TARGET_EPOCH).total_seconds()
    return (1 if delta > 0.0 else 0, abs(delta))


def select_records(records: Sequence[Sequence[str]]) -> Tuple[List[int], Dict[str, object]]:
    selected: Set[int] = set()
    legacy_best: Dict[str, Tuple[Tuple[int, float, int, int], int]] = {}
    modern_best: Dict[Tuple[str, str], Tuple[Tuple[int, float, int], int]] = {}
    aux_seen: Set[Tuple[str, str, str]] = set()

    for index, record in enumerate(records):
        kind = record_type(record)
        if kind == "EPH":
            system, satellite, message = eph_identity(record)
            epoch = record_epoch(record)
            if system in SIM_SYSTEMS and satellite and epoch is not None and message in LEGACY_FAMILIES[system]:
                family_rank = LEGACY_FAMILIES[system].index(message)
                score = (*time_score(epoch), family_rank, index)
                previous = legacy_best.get(satellite)
                if previous is None or score < previous[0]:
                    legacy_best[satellite] = (score, index)
            if (system, message) in KNOWN_MODERN and epoch is not None:
                score = (*time_score(epoch), index)
                previous = modern_best.get((system, message))
                if previous is None or score < previous[0]:
                    modern_best[(system, message)] = (score, index)
        elif kind in AUX_TYPES:
            key = aux_key(record)
            if key not in aux_seen:
                selected.add(index)
                aux_seen.add(key)

    selected.update(value[1] for value in legacy_best.values())
    selected.update(value[1] for value in modern_best.values())

    missing_modern = sorted(REQUIRED_MODERN - set(modern_best))
    if missing_modern:
        formatted = ", ".join(f"{system}:{message}" for system, message in missing_modern)
        raise ValueError(f"BRD400DLR source is missing representative modern families: {formatted}")

    selected_systems = {satellite[0] for satellite in legacy_best}
    missing_systems = sorted(SIM_SYSTEMS - selected_systems)
    if missing_systems:
        raise ValueError("fixture would miss legacy simulator constellation(s): " + ", ".join(missing_systems))

    selected_indices = sorted(selected)
    counts: Dict[str, int] = {}
    for index in selected_indices:
        kind = record_type(records[index]) or "UNKNOWN"
        counts[kind] = counts.get(kind, 0) + 1

    legacy_counts: Dict[str, int] = {system: 0 for system in sorted(SIM_SYSTEMS)}
    for satellite in legacy_best:
        legacy_counts[satellite[0]] += 1

    metadata: Dict[str, object] = {
        "selection_target_epoch": TARGET_EPOCH.isoformat() + "Z",
        "legacy_ephemeris_count_by_system": legacy_counts,
        "required_representative_modern_message_families": [
            f"{system}:{message}" for system, message in sorted(REQUIRED_MODERN)
        ],
        "observed_known_modern_message_families": [
            f"{system}:{message}" for system, message in sorted(modern_best)
        ],
        "selected_record_count": len(selected_indices),
        "selected_record_types": counts,
    }
    return selected_indices, metadata


def normalize_lf(lines: Iterable[str]) -> str:
    return "".join(line.rstrip("\r\n") + "\n" for line in lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("metadata", type=Path)
    parser.add_argument("--source-url", required=True)
    parser.add_argument("--compressed-sha256", required=True)
    args = parser.parse_args()

    source_bytes = args.source.read_bytes()
    text = source_bytes.decode("ascii")
    header, records = split_header_records(text)
    if not header or len(header[0]) < 9:
        raise ValueError("invalid RINEX header")
    try:
        version = float(header[0][:9])
    except ValueError as exc:
        raise ValueError("invalid RINEX version field") from exc
    if version < 4.0:
        raise ValueError(f"expected RINEX 4.x source, got {version:.2f}")

    selected_indices, selection_metadata = select_records(records)
    output_text = normalize_lf(header)
    output_text += "".join(normalize_lf(records[index]) for index in selected_indices)
    output_bytes = output_text.encode("ascii")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(output_bytes)

    metadata = {
        "schema_version": 1,
        "source_product": "BRD400DLR",
        "source_filename": args.source.name,
        "source_url": args.source_url,
        "source_compressed_sha256": args.compressed_sha256.lower(),
        "source_uncompressed_sha256": sha256_bytes(source_bytes),
        "fixture_filename": args.output.name,
        "fixture_sha256": sha256_bytes(output_bytes),
        "rinex_version": version,
        **selection_metadata,
    }
    args.metadata.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
