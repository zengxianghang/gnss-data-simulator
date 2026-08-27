# CN0 model builder

This directory is the offline data-preparation path for the simulator CN0 model. Issue #37 provides the streaming RINEX OBS ingestion layer, #38 adds deterministic elevation-bin aggregation, and #39 connects the generated compact model to runtime simulation.

## RINEX OBS boundary

The reader uses the pinned RTKLIB `rnxctr_t` / `input_rnxctr()` incremental interface. One observation epoch is decoded into RTKLIB's fixed `MAXOBS` buffer at a time; the tool does not call `readrnxt()` and does not retain the full observation file in memory.

The simulator build sets `NEXOBS=16` consistently for the pinned RTKLIB target and all consumers. The pinned fork already provides extended observation-code helpers for modern BeiDou `5P` (B2a) and `7D` (B2b). `src/gnss/rtklib_rinex_obs_ext.c` compiles the pinned `rinex.c` against those helpers without changing the submodule commit. The builder reconstructs RTKLIB's header-to-slot assignment so valid S-only observables can still be associated with a signal even when `obsd_t.code[]` was not populated by a code measurement.

Signal identities always resolve through `src/gnss/signal_definitions.*`; unsupported `S*` observables are reported and are never assigned to a guessed fallback signal.

## Signal-strength semantics

A RINEX `Snn` observable is accepted as comparable CN0 only when the observation header contains an unambiguous `SIGNAL STRENGTH UNIT` value of `DBHZ` (case-insensitive). The raw RTKLIB-decoded signal-strength value is still exposed for missing, unsupported, or conflicting unit declarations, but `cn0_dbhz` remains unavailable and the sample is classified as `AMBIGUOUS_SIGNAL_STRENGTH_UNIT`.

RTKLIB stores the decoded S value in a one-byte `obsd_t.SNR` field with 0.25 dB-Hz resolution. A zero, blank, or non-numeric S field is treated as missing and is not emitted as a sample.

Observation epochs are normalized to simulator GPST. UTC/GLONASS epochs use RTKLIB `utc2gpst()`, BeiDou BDT uses `bdt2gpst()`, and GPS/Galileo/QZSS use their GPST-aligned coarse epoch timeline. Out-of-order normalized epochs are rejected deterministically.

## Geometry

Station coordinates come from `APPROX POSITION XYZ`. Associated NAV is loaded through the normal simulator RTKLIB adapter, and azimuth/elevation is computed with `compute_satellite_geometry()` using the same transmit-time iteration, ECEF geometry and elevation convention as simulation. The unit regression compares the builder result with a direct simulator geometry call to `1e-12` rad.

If navigation geometry is unavailable, a DBHZ value can still be preserved in `cn0_dbhz`, but the sample is classified as `GEOMETRY_UNAVAILABLE` and is rejected by the elevation aggregation stage.

## Elevation-bin statistics

The default model covers 0 through 90 degrees in 5-degree bins. Bins are half-open `[lower, upper)`; only the final bin includes 90 degrees exactly. Empty and under-supported bins remain explicit `EMPTY` or `SPARSE` rows and are never extrapolated during model construction.

Because accepted CN0 is already on RTKLIB's 0.25 dB-Hz grid, each constellation/signal/elevation bin uses fixed 256-entry histograms rather than storing every observation. The same fixed grid is used for absolute consecutive Delta-CN0. Memory therefore scales with the number of model keys, not with OBS duration.

Per-bin statistics are defined as follows:

- P05/P10/P25/P50/P75/P90/P95 use R type-7 linear quantiles, `h=(n-1)*p`.
- Mean and standard deviation use the population definition (`ddof=0`).
- MAD is the type-7 median of `abs(CN0 - P50)`.
- Delta-CN0 is the absolute difference between consecutive accepted samples.
- Delta pairs and optional AR(1) never cross source, satellite, signal, elevation-bin or observation-gap boundaries. A temporal pair is valid only when the source has a positive RINEX `INTERVAL` and the normalized GPST gap agrees with it within the configured tolerance.
- AR(1) is reported only when the configured pair support is met and both lagged series have non-zero variance; otherwise metadata states `INSUFFICIENT_SUPPORT` or `ZERO_VARIANCE`.

Model rows are emitted in the central frozen signal-definition order and ascending elevation-bin order. Numeric formatting is locale-independent and fixed to six decimals.

## Reproducibility metadata

`cn0_model.meta.json` records the model/statistics schema versions, bin/filter rules, all aggregation rejection counters, RINEX/station/receiver/antenna information and per-source stream diagnostics. Source identity is basename + byte size + streaming FNV-1a64. Absolute source paths, wall-clock timestamps and output paths are intentionally excluded so relocating the same files does not change the generated metadata bytes.

## CLI

```text
build-cn0-model \
  --source <rinex.obs> <rinex.nav> \
  [--source <rinex.obs> <rinex.nav> ...] \
  --output <cn0_model.csv> \
  --metadata <cn0_model.meta.json> \
  [--bin-width-deg 5] \
  [--min-bin-count 20] \
  [--min-temporal-pairs 20]
```

The canonical source for real calibration OBS/NAV material is the Wuhan University IGS Data Center (`igs.gnsswhu.cn`). CI uses compact checked-in deterministic fixtures rather than downloading live data. Real open-sky WHU/IGS observation files are calibration inputs for offline model production, not normal CI dependencies.
