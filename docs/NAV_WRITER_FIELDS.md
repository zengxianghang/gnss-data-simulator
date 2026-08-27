# Navigation Writer Field Provenance

This document records the V1 source of navigation-log fields. The writers are serialization layers: they consume normalized `NavOutputRecord` values projected from Receiver NAV and do not parse RINEX or recompute satellite state.

## Common source path

```text
RINEX NAV -> pinned RTKLIB parser -> Receiver NAV -> rtklib_nav_output_adapter
          -> NavOutputRecord -> NovAtel/Unicore ASCII writer
```

`NavigationUpdateEvent::receiver_record_index` identifies the record copied into Receiver NAV by a COLD/runtime update. Duplicate truth-record delivery produces no new event and therefore no duplicate NAV log.

## Keplerian ephemeris

Direct RTKLIB/Receiver-NAV fields include PRN, message family, IODE/IODC, health, accuracy, Toe/Toc/transmit time, orbit parameters, harmonic corrections, clock polynomial, TGD/BGD/ISC and fit interval. `sqrt(A)` and corrected mean motion are deterministic derived metadata computed in `nav_output_metadata.cpp` before serialization.

For Galileo, the pinned RTKLIB parser preserves the RINEX SV-health word with the documented bit packing: E1B DVS/HS, E5a DVS/HS and E5b DVS/HS. The normalized adapter decodes these bits before `GALEPHEMERISA`/`GALEPHA` formatting. Separate F/NAV and I/NAV clock fields are populated only from Receiver-NAV records for the same satellite; the writer does not search Truth NAV.

BeiDou legacy D1/D2 maps to `BD2EPHEMA` and `BDSEPHA`. RINEX 4 B-CNAV1/2/3 maps to Unicore `BD3EPHA`; there is no frozen NovAtel OEM7 modern-BDS ephemeris record in V1, so the NovAtel writer reports that normalized record as unsupported instead of inventing a record family.

`IRNSSEPHA` consumes a normalized NavIC ephemeris when present in Receiver NAV. This output capability does not modify the frozen 21-signal V1 observation table and does not enable NavIC RANGE generation.

## GLONASS ephemeris

Position, velocity, acceleration, `tau_n`, `gamma_n`, `delta_tau_n`, frequency channel, health, age and issue come directly from Receiver NAV `geph_t` through the adapter. Slot numbering, OEM/Unicore frequency offset representation, GPS-to-GLONASS time offset, GLONASS four-year-cycle day number and frame seconds-of-day are deterministic protocol metadata computed before the writer.

## Ionosphere/UTC

RINEX 4 explicit ION records are projected from RTKLIB `ion_t`. RINEX 2/3 header ionosphere arrays and UTC/leap-second metadata are exposed as deterministic normalized metadata records when no explicit record of the same system exists.

- GPS legacy parameters -> NovAtel `IONUTCA`, Unicore `GPSIONA`.
- BeiDou legacy parameters -> NovAtel `BD2IONUTCA`, Unicore `BDSIONA`.
- Galileo NeQuick coefficients -> Unicore `GALIONA`.
- BeiDou-3 nine-parameter explicit ionosphere data -> Unicore `BD3IONA`.

QZSS ionosphere metadata is retained by Receiver NAV but is not emitted by the V1 Unicore set because no QZSS ION family is frozen in `NAV_RECORDS.md`.

## Headers and CRC

NovAtel NAV records reuse the deterministic OEM7 ASCII framing introduced with the observation writers. Unicore N4 uses deterministic V1 header metadata (`GPS/FINE`, version 18, status/reserved zero) and the same documented reflected CRC-32 polynomial `0xEDB88320` with initial value zero. Header metadata is protocol/fallback metadata only and is not used to alter Receiver NAV contents.

## Unsupported/missing fields

A writer must not fabricate a new navigation parser. If a required modern RINEX field is not retained by the pinned RTKLIB data model, the normalized adapter must be extended (and, if necessary, the pinned RTKLIB parser/structs must be extended) before that field is serialized. Reserved protocol fields may use deterministic zero values only where the protocol defines them as reserved/not modeled; health, orbit, clock, TGD/BGD/ISC and acquisition state must come from Receiver NAV.
