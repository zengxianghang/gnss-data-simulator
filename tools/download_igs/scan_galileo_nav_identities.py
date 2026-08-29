#!/usr/bin/env python3
"""Report Galileo INAV/FNAV broadcast-identity pairings in real RINEX 4 NAV files.

Read-only provenance tool for issue #94: it parses ``> EPH E..`` records from
local RINEX 4 navigation files and reports same-satellite identity tuples so the
companion-selection regressions can cite real, reproducible search results.

The tool never writes navigation data. Values are taken field-for-field from the
source records; only the report text is produced.

Search categories (Galileo INAV/FNAV companion identity, cf. issue #92/#94):

- ``matching``: same satellite + same IODnav + same Toe, both families present;
- ``same_toe_different_iodnav``: same satellite + same Toe + different IODnav;
- ``same_iodnav_different_toe``: same satellite + same IODnav + different Toe.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Sequence, Set, Tuple


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


def galileo_identities(source: Path) -> List[Tuple[str, str, int, float]]:
    """Return (satellite, family, IODnav, Toe) for every Galileo EPH record."""
    text = source.read_text(encoding="ascii")
    identities: List[Tuple[str, str, int, float]] = []
    for record in split_records(text):
        marker = record[0].split()
        if len(marker) < 4 or marker[1] != "EPH" or not marker[2].startswith("E"):
            continue
        satellite = marker[2]
        family = marker[3]
        iodnav = int(float(data_field(record[2], 0)))
        toe = float(data_field(record[4], 0))
        identities.append((satellite, family, iodnav, toe))
    return identities


def scan(identities: Sequence[Tuple[str, str, int, float]]) -> Dict[str, object]:
    by_satellite: Dict[str, List[Tuple[float, str, int]]] = {}
    for satellite, family, iodnav, toe in identities:
        by_satellite.setdefault(satellite, []).append((toe, family, iodnav))

    matching: Set[Tuple[str, int, float]] = set()
    same_toe_different_iodnav: Set[Tuple[str, float, str, int, str, int]] = set()
    same_iodnav_different_toe: Set[Tuple[str, int, float, float]] = set()

    for satellite, records in by_satellite.items():
        inav = {(iodnav, toe) for toe, family, iodnav in records if family == "INAV"}
        fnav = {(iodnav, toe) for toe, family, iodnav in records if family == "FNAV"}
        matching.update((satellite, iodnav, toe) for iodnav, toe in inav & fnav)
        for toe_a, family_a, iodnav_a in records:
            for toe_b, family_b, iodnav_b in records:
                if family_a != family_b and toe_a == toe_b and iodnav_a != iodnav_b:
                    first = min((family_a, iodnav_a), (family_b, iodnav_b))
                    second = max((family_a, iodnav_a), (family_b, iodnav_b))
                    same_toe_different_iodnav.add(
                        (satellite, toe_a, first[0], first[1], second[0], second[1]))
        inav_toes: Dict[int, Set[float]] = {}
        fnav_toes: Dict[int, Set[float]] = {}
        for toe, family, iodnav in records:
            (inav_toes if family == "INAV" else fnav_toes).setdefault(iodnav, set()).add(toe)
        for iodnav in sorted(set(inav_toes) & set(fnav_toes)):
            for toe_a in sorted(inav_toes[iodnav]):
                for toe_b in sorted(fnav_toes[iodnav]):
                    if toe_a != toe_b:
                        key = (satellite, iodnav, toe_a, toe_b)
                        same_iodnav_different_toe.add(min(key, (satellite, iodnav, toe_b, toe_a)))

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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sources", type=Path, nargs="+", help="local RINEX 4 NAV file(s)")
    args = parser.parse_args()

    all_identities: List[Tuple[str, str, int, float]] = []
    for source in sorted(args.sources):
        identities = galileo_identities(source)
        all_identities.extend(identities)
        print(f"# {source.name}: {len(identities)} Galileo EPH records")
    result = scan(all_identities)
    print(f"# aggregate over {len(args.sources)} file(s)")
    for key in sorted(result):
        print(f"{key}: {result[key]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
