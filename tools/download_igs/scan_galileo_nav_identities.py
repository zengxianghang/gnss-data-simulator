#!/usr/bin/env python3
"""Report Galileo INAV/FNAV broadcast-identity pairings in real RINEX 4 NAV files.

Read-only provenance tool for issue #94: it parses ``> EPH E.. INAV/FNAV``
records from local RINEX 4 navigation files and reports same-satellite identity
tuples so the companion-selection regressions can cite real, reproducible search
results.

The tool never writes navigation data. Values are taken field-for-field from the
source records; only the report text is produced. A Galileo navigation identity
is ``(satellite, family, IODnav, broadcast week, Toe SOW)``; the broadcast week
comes from the same record field RTKLIB decodes into ``eph.week`` (``data[21]``,
``rinex.c`` decode "gal week = gps week"), so Toe comparisons are week-aware and
cannot conflate equal SOW values from different GPS weeks.

Search categories (Galileo INAV/FNAV companion identity, cf. issue #92/#94):

- ``matching``: same satellite + same IODnav + same week + same Toe, both families;
- ``same_toe_different_iodnav``: same satellite + same week + same Toe + different IODnav;
- ``same_iodnav_different_toe``: same satellite + same IODnav + different (week, Toe).
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Sequence, Set, Tuple

Identity = Tuple[str, str, int, int, float]

GALILEO_FAMILIES = ("INAV", "FNAV")


def data_field(line: str, index: int) -> str:
    """Return the index-th 19-character RINEX data field of a record body line."""
    return line[4 + 19 * index : 23 + 19 * index].strip()


def split_records(text: str) -> List[List[str]]:
    records: List[List[str]] = []
    current: List[str] = []
    for line in text.splitlines(keepends=True):
        if line.startswith(">"):
            if current:
                records.append(current)
            current = [line]
        elif current:
            current.append(line)
    if current:
        records.append(current)
    return records


def galileo_identities(source: Path) -> List[Identity]:
    """Return (satellite, family, IODnav, week, Toe SOW) per Galileo EPH record.

    The week is the broadcast week field RTKLIB decodes into ``eph.week``
    (``data[21]``, the third 19-character field of the record's sixth data line).
    Only INAV and FNAV records are collected; any other family is ignored rather
    than implicitly routed into the FNAV bucket.
    """
    text = source.read_text(encoding="ascii")
    identities: List[Identity] = []
    for record in split_records(text):
        marker = record[0].split()
        if len(marker) < 4 or marker[1] != "EPH" or not marker[2].startswith("E"):
            continue
        family = marker[3]
        if family not in GALILEO_FAMILIES or len(record) < 7:
            continue
        satellite = marker[2]
        iodnav = int(float(data_field(record[2], 0)))
        week = int(float(data_field(record[6], 2)))
        toe = float(data_field(record[4], 0))
        identities.append((satellite, family, iodnav, week, toe))
    return identities


def scan(identities: Sequence[Identity]) -> Dict[str, object]:
    by_satellite: Dict[str, List[Tuple[str, int, int, float]]] = {}
    for satellite, family, iodnav, week, toe in identities:
        by_satellite.setdefault(satellite, []).append((family, iodnav, week, toe))

    matching: Set[Tuple[str, int, int, float]] = set()
    same_toe_different_iodnav: Set[Tuple[str, int, str, int, str, int]] = set()
    same_iodnav_different_toe: Set[Tuple[str, int, int, float, float]] = set()

    for satellite, records in by_satellite.items():
        inav = {(iodnav, week, toe) for family, iodnav, week, toe in records if family == "INAV"}
        fnav = {(iodnav, week, toe) for family, iodnav, week, toe in records if family == "FNAV"}
        matching.update((satellite, iodnav, week, toe) for iodnav, week, toe in inav & fnav)
        for family_a, iodnav_a, week_a, toe_a in records:
            for family_b, iodnav_b, week_b, toe_b in records:
                if family_a == family_b or week_a != week_b or toe_a != toe_b or iodnav_a == iodnav_b:
                    continue
                first = min((family_a, iodnav_a), (family_b, iodnav_b))
                second = max((family_a, iodnav_a), (family_b, iodnav_b))
                same_toe_different_iodnav.add((satellite, week_a, first[0], first[1], second[0], second[1]))
        inav_toes: Dict[int, Set[Tuple[int, float]]] = {}
        fnav_toes: Dict[int, Set[Tuple[int, float]]] = {}
        for family, iodnav, week, toe in records:
            (inav_toes if family == "INAV" else fnav_toes).setdefault(iodnav, set()).add((week, toe))
        for iodnav in sorted(set(inav_toes) & set(fnav_toes)):
            for week_a, toe_a in sorted(inav_toes[iodnav]):
                for week_b, toe_b in sorted(fnav_toes[iodnav]):
                    if (week_a, toe_a) != (week_b, toe_b):
                        key = (satellite, iodnav, week_a, toe_a, week_b, toe_b)
                        swapped = (satellite, iodnav, week_b, toe_b, week_a, toe_a)
                        normalized = min(key, swapped)
                        same_iodnav_different_toe.add((normalized[0], normalized[1], normalized[2], normalized[3],
                                                       normalized[4]))
    return {
        "galileo_record_count": len(identities),
        "satellite_count": len(by_satellite),
        "matching_pairs": len(matching),
        "matching_pair_examples": sorted(matching)[:8],
        "same_toe_different_iodnav_count": len(same_toe_different_iodnav),
        "same_toe_different_iodnav_examples": sorted(same_toe_different_iodnav)[:8],
        "same_iodnav_different_toe_count": len(same_iodnav_different_toe),
        "same_iodnav_different_toe_examples": sorted(same_iodnav_different_toe)[:8],
    }


def self_test() -> int:
    """Deterministic checks against the committed real fixture and pure identities."""
    import json

    repo_root = Path(__file__).resolve().parents[2]
    fixture = repo_root / "tests/data/minimal/brd400dlr_rinex4_galileo_companion_nav.rnx"
    metadata = json.loads(
        (repo_root / "tests/data/minimal/brd400dlr_rinex4_galileo_companion_nav.meta.json").read_text())
    identities = galileo_identities(fixture)
    result = scan(identities)
    assert metadata["selected_galileo_record_count"] == len(identities) == 6, result
    assert result["satellite_count"] == 2, result
    assert result["matching_pairs"] == 2, result
    assert result["same_toe_different_iodnav_count"] == 0, result
    assert result["same_iodnav_different_toe_count"] == 1, result

    # Week awareness: equal Toe SOW in different broadcast weeks is a different
    # navigation instance, so a week-blind scan would pair the first two records
    # below while the week-aware scan must not. These are plain identity tuples;
    # no ephemeris record is created or mutated.
    cross_week = scan([
        ("E02", "INAV", 5, 2347, 100000.0),
        ("E02", "FNAV", 5, 2348, 100000.0),
    ])
    assert cross_week["matching_pairs"] == 0, cross_week
    assert cross_week["same_toe_different_iodnav_count"] == 0, cross_week
    assert cross_week["same_iodnav_different_toe_count"] == 1, cross_week
    same_week = scan([
        ("E02", "INAV", 5, 2347, 100000.0),
        ("E02", "FNAV", 5, 2347, 100000.0),
    ])
    assert same_week["matching_pairs"] == 1, same_week
    same_week_iod = scan([
        ("E02", "INAV", 5, 2348, 100000.0),
        ("E02", "FNAV", 6, 2348, 100000.0),
    ])
    assert same_week_iod["matching_pairs"] == 0, same_week_iod
    assert same_week_iod["same_toe_different_iodnav_count"] == 1, same_week_iod
    print("scan_galileo_nav_identities self-test: OK")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sources", type=Path, nargs="*", help="local RINEX 4 NAV file(s)")
    parser.add_argument("--self-test", action="store_true", help="run deterministic self-test and exit")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if not args.sources:
        parser.error("at least one source file is required unless --self-test is given")

    all_identities: List[Identity] = []
    for source in sorted(args.sources):
        identities = galileo_identities(source)
        all_identities.extend(identities)
        print(f"# {source.name}: {len(identities)} Galileo INAV/FNAV EPH records")
    result = scan(all_identities)
    print(f"# aggregate over {len(args.sources)} file(s)")
    for key in sorted(result):
        print(f"{key}: {result[key]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
