# CN0 model builder

This directory is the offline data-preparation path for the simulator CN0 model. Issue #37 provides only the streaming RINEX OBS ingestion layer; elevation-bin statistics/model fitting belong to #38 and runtime model loading belongs to #39.

## RINEX OBS boundary

The reader uses the pinned RTKLIB `rnxctr_t` / `input_rnxctr()` incremental interface. One observation epoch is decoded into RTKLIB's fixed `MAXOBS` buffer at a time; the tool does not call `readrnxt()` and does not retain the full observation file in memory.

The simulator build sets `NEXOBS=16` consistently for the pinned RTKLIB target and all consumers. The pinned fork already provides extended observation-code helpers for modern BeiDou `5P` (B2a) and `7D` (B2b). `src/gnss/rtklib_rinex_obs_ext.c` compiles the pinned `rinex.c` against those helpers without changing the submodule commit. The builder reconstructs RTKLIB's header-to-slot assignment so valid S-only observables can still be associated with a signal even when `obsd_t.code[]` was not populated by a code measurement.

Signal identities always resolve through `src/gnss/signal_definitions.*`; unsupported `S*` observables are reported and are never assigned to a guessed fallback signal.

## Signal-strength semantics

A RINEX `Snn` observable is accepted as comparable CN0 only when the observation header contains an unambiguous `SIGNAL STRENGTH UNIT` value of `DBHZ` (case-insensitive). The raw decoded signal-strength value is still exposed for missing, unsupported, or conflicting unit declarations, but `cn0_dbhz` remains unavailable and the sample is classified as `AMBIGUOUS_SIGNAL_STRENGTH_UNIT`.

A zero, blank, or non-numeric S field is treated as missing and is not emitted as a sample. This preserves the pinned RTKLIB decoding semantics rather than inventing a replacement value.

Observation epochs are normalized to simulator GPST. UTC/GLONASS epochs use RTKLIB `utc2gpst()`, BeiDou BDT uses `bdt2gpst()`, and GPS/Galileo/QZSS use their GPST-aligned coarse epoch timeline. Out-of-order normalized epochs are rejected deterministically.

## Geometry

Station coordinates come from `APPROX POSITION XYZ`. Associated NAV is loaded through the normal simulator RTKLIB adapter, and azimuth/elevation is computed with `compute_satellite_geometry()` using the same transmit-time iteration, ECEF geometry and elevation convention as simulation. The unit regression compares the builder result with a direct simulator geometry call to `1e-12` rad.

If navigation geometry is unavailable, a DBHZ value can still be preserved in `cn0_dbhz`, but the sample is classified as `GEOMETRY_UNAVAILABLE` and cannot be used by the later elevation-model fitting stage.

## CLI

```text
build-cn0-model --obs <rinex.obs> --nav <rinex.nav> --output <samples.csv>
```

The CLI writes the streaming sample records to CSV. It does not compute elevation bins or fit a model.

The canonical source for real calibration OBS/NAV material is the Wuhan University IGS Data Center (`igs.gnsswhu.cn`). CI must use compact, checked-in deterministic fixtures rather than downloading live data.
