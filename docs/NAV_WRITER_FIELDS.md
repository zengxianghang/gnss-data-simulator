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

### NovAtel field semantics

The NovAtel writer follows the OEM7 log definitions (`GPSEPHEMERIS`,
`QZSSEPHEMERIS`, `GLOEPHEMERIS`) and, for `GALEPHEMERIS`, which OEM7 no longer
documents, the OEM6 layout that RTKLIB `decode_galephemerisb` reads:

- `GPSEPHEMERISA`, `QZSSEPHEMERISA`: field 32 is the URA variance, the square
  of the RINEX URA in metres. `BD2EPHEMA` has no NovAtel definition and keeps
  the simulator's contract (the URA in metres).
- `GALEPHEMERISA`: SISA is the Galileo SISA index; the field after it is
  reserved (zero).
- `GLOEPHEMERISA`: health is 0 for a healthy and 4 for an unhealthy RINEX
  record (OEM7: 0-3 good, 4-15 bad); tau_n, delta_tau_n, gamma in the OEM7
  order; P is the RINEX time-offset parameter; Flags use the OEM7 coding
  (bits 0-1 P1, bit 2 P2, bit 3 P3, bit 4 P4).

The derived values (Galileo SISA index, GLONASS P and vendor flags) are
computed once in `finalize_nav_output_record_metadata()` and shared by both
writers. A record whose URA, SISA or F_T cannot be represented is not written.
The serialized-NAV parser (`tools/rangea_roundtrip`) inverts each mapping; the
RINEX GLONASS type bits (M) are not in the log and are not restored.

### Unicore N4 field semantics

The Unicore writer follows the *Unicore Reference Commands Manual for N4 High
Precision Products* (field order and types of each log). Protocol metadata
that the writer derives from the RTKLIB record:

- `GPSEPHA`, `QZSSEPHA`, `BDSEPHA` (7.3.38, 7.3.80, 7.3.15): the AS field is
  written between af2 and N from `eph_t.flag`, as the NovAtel `GPSEPHEMERISA`
  writer does (RINEX has no anti-spoofing flag); URA is the variance, the
  square of the RINEX URA in metres; QZSS PRN is 1-10 (RTKLIB 193-202);
  BDSEPH Toe and toc are BDT seconds of week (maintainer confirmation), while
  Week, Z Week and Tow stay on the GPS axis as N4 describes them. Only
  LNAV (and BDS D1/D2) records are written; GPS/QZSS CNAV/CNAV-2 records are
  not relabelled as these logs.
- `IRNSSEPHA` (7.3.45): TOWC is the transmission time in 12 s units; L5 and S
  health come from the RINEX NavIC health (L5 the more significant bit);
  IODEC is `eph_t.iode`; the two reserved fields and the Alert/AutoNav Flag
  (not in RINEX) are zero; URA is the variance.
- `GALEPHA` (7.3.34): SISA is the Galileo OS SIS ICD index of the RINEX SISA
  in metres (1/2/4/16 cm bands; RINEX -1 is 255, NAPA); the reserved field
  after it is zero.
- `GLOEPHA` (7.3.37): tau_n, delta_tau_n, gamma in the N4 order; P is the
  RINEX time-offset parameter (status flag bits 0-1); the Flags field
  repacks the RINEX status flags into Table 7-102 (bits 0-1 P1, bit 2 P2,
  bit 3 P3).
- `GALIONA` (7.3.35): SF1..SF5 come from the RINEX 4.01 IFNV disturbance
  flags (bit 4 region 1 ... bit 0 region 5).

A record whose URA, SISA or F_T cannot be represented in its N4 field is not
written. `GPSIONA`, `BDSIONA` and `BD3IONA` needed no change.

`BD3EPHA` follows the Unicore N4 reference book (7.3.12) field order: PRN,
Health, SatType, SISMAI, IODE, IODC, Week, Zweek, Tow, Toe, DeltaA, dDeltaA,
DeltaN, dDeltaN, M0, Ecc, omega, Cuc, Cus, Crc, Crs, Cic, Cis, I0, IDOT,
Omega0, OmegaDot, toc, Tgdb1cp, Tgdb2ap, Tgdb2bI, Tgdb2bQ, ISCb2ad, ISCb1cd,
af0, af1, af2, iTop, SISAIoe, SISAIocb, SISAIoc1, SISAIoc2, two reserved
fields and FreqType (0 B-CNAV1/B1C, 1 B-CNAV2/B2a, 2 B-CNAV3/B2b). The values
come from the RINEX 4 CNV1/2/3 record as the pinned RTKLIB keeps it
(`eph_t.flag` is SatType, `sva` SISMAI, `Adot`, `delta_n0_dot`, `top`,
`sisai`). Deterministic protocol metadata computed by the writer:

- DeltaA = A - A_ref, with A_ref 27906100 m (MEO) or 42162200 m (IGSO/GEO)
  from the BDS B1C/B2a/B2b ICDs;
- Week/Zweek are the GPS week of Toe; Tow, Toe and toc are native BDT seconds
  of week, and iTop is t_op in its 300 s units, as in the N4 example;
- SISAI are the unsigned 5/5/3/3-bit ICD fields (a RINEX producer may write
  the signed reading, for example -5 for 27);
- group delays that the record's family does not broadcast are zero
  (B-CNAV1: Tgdb1cp, Tgdb2ap, ISCb1cd; B-CNAV2: Tgdb1cp, Tgdb2ap, ISCb2ad;
  B-CNAV3: Tgdb2bI), and IODE/IODC are zero for B-CNAV3, where N4 reserves
  them.

A record whose SatType, SISMAI, SISAI or t_op cannot be represented is not
written.

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
