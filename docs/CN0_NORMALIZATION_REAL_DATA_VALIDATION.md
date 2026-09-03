# Real-data validation of normalized C/N0 portability

This document records the real multi-station validation for Issue #108 and PR #114.
The validation checks whether the normalized C/N0 model introduced by #104/#106
reduces cross-station dispersion without changing or fabricating any RINEX
observation or navigation values.

## Result

**PASS** under the criteria declared in PR #114 before inspecting the real-data
results.

| Metric | Result | PASS threshold |
| --- | ---: | ---: |
| Eligible cross-station bins | 143 | n/a |
| Median raw MPAD | 3.25 dB | n/a |
| Median normalized MPAD | 1.00 dB | n/a |
| Median MPAD reduction | 69.23% | >= 30% |
| Non-worse eligible-bin fraction | 95.10% | >= 60% |

MPAD is the median pairwise absolute difference across stations for the same
`(constellation, signal, elevation_bin)` identity. An eligible bin requires at
least three contributing stations, a `READY` source bin, a `READY` source
reference, and no interpolation of missing/sparse bins.

The result supports the intended semantic: removing each station/signal's
high-elevation absolute C/N0 reference before cross-station aggregation
materially improves portability of the elevation-dependent shape on this data
set.

## Frozen validation criteria

The following criteria were declared in PR #114 before the real multi-station
run:

- primary metric: median pairwise absolute difference (MPAD, dB)
- identity key: `(constellation, signal, elevation_bin)`
- minimum contributing stations: 3
- only `READY` source bins and `READY` source references are eligible
- missing/sparse bins are excluded rather than interpolated
- PASS: median MPAD reduction >= 30% and non-worse fraction >= 0.60
- PARTIAL: reduction >= 10% or non-worse fraction >= 0.50
- FAIL otherwise

The analyzer consumes production `gnss-cn0-metadata-v2` output. Raw per-source
bin P50 is reconstructed only as production `reference_p50_dbhz + delta_p50_db`;
it does not implement an alternative normalization algorithm.

## Exact run

- GitHub Actions run: `33716044745`
- real-data validation job: `validate-real-whu`
- tested repository commit: `027450974f239f82a0a7c76ddd6cf1377208a074`
- pinned RTKLIB commit: `c03a768a180caea7c85586fe0f3c6842f832df27`
- Compact RINEX restoration package: `hatanaka==2.8.1`
- evidence artifact: `issue-108-real-whu-cn0-evidence`
- evidence artifact ID: `9878734721`
- evidence artifact ZIP SHA-256: `cc8992cffb8db97a1904d63f6052ea9e68eb661e374c461535b5f02f654af134`

The workflow downloaded authentic WHU/IGS files, restored Compact RINEX using
the pinned standard Hatanaka tool where needed, recorded provenance, ran the
production `build-cn0-model`, and then ran the predeclared analyzer. Large OBS
and NAV files were not committed to the repository.

## Data set and provenance

All data are for 2025-01-03 (DOY 003), 30 s observation interval.
Every observation header used in the run explicitly declares
`SIGNAL STRENGTH UNIT = DBHZ`.

| Station | RINEX | Receiver | Antenna |
| --- | --- | --- | --- |
| WTZR | 3.04 | LEICA GR50 | LEIAR25.R3 LEIT |
| BRUX | 3.04 | SEPT POLARX5TR | JAVRINGANT_DM SCIS |
| TWTF | 3.04 | SEPT POLARX4TR | SEPCHOKE_B3E6 SPKE |
| AUCK00NZL | 3.05 | TRIMBLE ALLOY | TRM115000.00 NONE |
| SCTB00ATA | 3.05 | TRIMBLE ALLOY | TRM115000.00 NONE |

Matching broadcast navigation input:

- `BRD400DLR_S_20250030000_01D_MN.rnx`
- RINEX navigation version 4.02
- merged GPS/GLO/GAL/BDS/QZS/SBAS/IRNSS broadcast navigation
- DLR/GSOC BRD4 product

No ephemeris, ionosphere, or observation values were modified, retargeted,
interpolated, or fabricated.

### Production builder input SHA-256

| File | Bytes | SHA-256 |
| --- | ---: | --- |
| `WTZR00DEU_R_20250030000_01D_30S_MO.rnx` | 33,385,138 | `a905b872ce8d5a46af0bb9242766e2aa738827360891728031a9a07dee042a7a` |
| `BRUX00BEL_R_20250030000_01D_30S_MO.rnx` | 48,205,273 | `240de624b4c2b76c4c3d5e2fc9a4d2987bc81fe448170fdf5e6462c3972d37ce` |
| `TWTF00TWN_R_20250030000_01D_30S_MO.rnx` | 23,571,474 | `69cc7aedac9c686beaeb8948cdf4a22e287ea1b127e523061fa69bf5e3815baa` |
| `AUCK00NZL_R_20250030000_01D_30S_MO.rnx` | 26,724,012 | `be682497a8d01368e0e839c961439b02f0b4f718f2443faf827293a910982270` |
| `SCTB00ATA_R_20250030000_01D_30S_MO.rnx` | 25,047,376 | `601c9768cc1ae64f40683a8954ce63def9dc14ea396c867099b54888c9fbda6a` |
| `BRD400DLR_S_20250030000_01D_MN.rnx` | 10,963,931 | `b11c638eea42978b8bd6aa8b65a5099fe6556dfe527bc037ed481d2b239afc42` |

### Downloaded compressed-source SHA-256

| File | SHA-256 |
| --- | --- |
| `WTZR00DEU_R_20250030000_01D_30S_MO.crx.gz` | `1ccf0ecd459ce271f263aadc7414f1ae0f42b1aa6c34a577d88545f17e90b9cd` |
| `BRUX00BEL_R_20250030000_01D_30S_MO.crx.gz` | `fd82c69014e336013e212317d864eb3e4b751f26552c0d8d1915885897f4e829` |
| `TWTF00TWN_R_20250030000_01D_30S_MO.rnx.gz` | `8becc977b4475e3853e47f46b8c42baab23146feba5ffabb8eb3b3e7d75be764` |
| `AUCK00NZL_R_20250030000_01D_30S_MO.rnx.gz` | `a4d402ec74faf7002d3c3d0f1ad0509ec47698a339b902236defd049197b02b4` |
| `SCTB00ATA_R_20250030000_01D_30S_MO.rnx.gz` | `a67cc068378aa6aa350105925766a074876b635bfe8a09d8b6ff96953d4f8bee` |
| `BRD400DLR_S_20250030000_01D_MN.rnx.gz` | `fb84d4046b06e905e8e4ec0efb82f0e9ad044bc44d664cc0209cb9b4c92b9512` |

## Production normalization configuration

The production defaults were used rather than tuned to the result:

- elevation range: 0-90 degrees
- bin width: 5 degrees
- high-elevation reference range: 60-90 degrees
- reference statistic: `P50_R7`
- minimum reference samples: 20
- cross-source statistic: `P50_EQUAL_SOURCE_WEIGHT`
- minimum sources per aggregate bin: 1
- model semantic: `NORMALIZED_ELEVATION_SHAPE`

The production builder completed with five sources and 378 aggregate bins.
The validation analyzer found 832 eligible source-bin rows and 143 eligible
cross-station bins under the stricter minimum-three-station criterion.

## Per-constellation/signal summary

Only identities with at least three stations of eligible data appear below.
This table must not be interpreted as coverage of every signal present in the
RINEX files.

| Constellation | Signal | Eligible bins | Raw median MPAD | Normalized median MPAD | Reduction | Non-worse fraction |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| BEIDOU | 2I | 18 | 4.000 dB | 1.688 dB | 57.81% | 88.89% |
| GALILEO | 1C | 18 | 6.500 dB | 2.750 dB | 57.69% | 100.00% |
| GALILEO | 5Q | 18 | 3.000 dB | 0.625 dB | 79.17% | 100.00% |
| GALILEO | 7Q | 18 | 3.500 dB | 0.875 dB | 75.00% | 100.00% |
| GLONASS | 1C | 18 | 2.500 dB | 1.438 dB | 42.50% | 72.22% |
| GLONASS | 2C | 18 | 1.500 dB | 0.688 dB | 54.17% | 100.00% |
| GPS | 1C | 18 | 3.125 dB | 1.563 dB | 50.00% | 100.00% |
| GPS | 5Q | 17 | 3.000 dB | 1.250 dB | 58.33% | 100.00% |

All eight eligible constellation/signal summaries show lower median MPAD after
normalization. Individual bins are not guaranteed to improve: for example,
some low-elevation BeiDou 2I and GLONASS 1C bins were worse. That is reflected
in the non-worse metric rather than hidden or excluded after the fact.

## Interpretation and limits

This run is strong evidence for the portability benefit claimed by the
normalized elevation-shape semantic because:

1. the metric and thresholds were fixed before result inspection;
2. the data come from five geographically separated real IGS stations;
3. the stations include different receiver and antenna families;
4. production builder output is analyzed directly;
5. constellation is part of the identity key, so same-code labels such as GPS
   `1C` and QZSS `1C` cannot collide;
6. sparse/missing regions are excluded rather than filled to improve results;
7. the real RINEX observations and BRD4 navigation are used unchanged.

The run does **not** prove universal portability across all receiver hardware,
all stations, all seasons, all days, or every GNSS signal. It is a one-day,
five-station validation. Some signals do not have three `READY` references and
therefore are intentionally absent from the quantitative comparison. Additional
days/stations may be used later as broader calibration evidence without changing
the frozen semantics or this recorded result.
