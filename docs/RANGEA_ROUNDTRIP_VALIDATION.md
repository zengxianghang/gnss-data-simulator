# RANGEA black-box round-trip validation

Issue #60 adds an independent serialization-boundary validation for the receiver RANGE output.

## Why this exists

The simulator already had an in-memory loopback:

`real RINEX NAV -> MeasurementObservation -> RTKLIB SPP`

The same `MeasurementObservation` objects were also passed to the RANGEA writer. That proves the measurement and solution models are internally consistent, but it does not prove that the serialized receiver interface preserves the same observation semantics.

The black-box gate instead executes:

`real RINEX NAV -> simulator -> simulated.log RANGEA -> independent parser -> raw RTKLIB code observations -> maintained RTKLIB SPP adapter -> pntpos() -> truth comparison`

The RTKLIB observations used by this gate come only from parsed RANGEA text. The evaluator does not read `observation_truth.csv` and does not receive the simulator's in-memory `MeasurementObservation` objects.

## Independence rules

- RANGEA framing and CRC are decoded independently; the parser does not call the RANGEA writer or its framing helper.
- Serialized observation count and field count must agree exactly.
- Constellation and signal identity are decoded from the serialized tracking-status word and RANGE PRN representation.
- Canonical signal semantics are resolved through the frozen signal-definition table after the serialized constellation/signal type has been decoded.
- Unsupported constellation/signal mappings fail explicitly.
- Invalid numeric fields, CRC mismatches, inconsistent pseudorange-valid bits, and malformed GLONASS frequency fields fail explicitly.
- The parser streams the log line-by-line and does not require a long log to be loaded into memory.

## Navigation scope for the current accuracy characterization

The current baseline intentionally gives RTKLIB the complete provenance-traceable real RINEX NAV store. Receiver ephemeris acquisition order and runtime navigation-update timing are not simulated by the black-box evaluator at this stage.

This isolates the generated-observation -> RANGEA serialization -> independent parse -> RTKLIB SPP path before adding receiver-NAV timing as another variable. `Use all available EPH` still means real, unchanged RINEX records only; no synthetic ephemeris fallback is permitted.

## Raw RTKLIB positioning boundary

The round-trip evaluator deliberately does not reconstruct or reuse simulator-owned `code_bias_m`.

The raw adapter is intentionally thin. It does not implement a second SPP solver and does not call `pntpos()` directly. For each selected serialized observation it:

1. validates the satellite, observation code, message-family mapping, pseudorange, and CN0;
2. verifies that the complete real RINEX NAV contains a compatible ephemeris/code-bias definition;
3. obtains RTKLIB's broadcast code bias for that observation;
4. passes the serialized pseudorange plus matching bias metadata into the maintained `rtklib_solve_single_position()` adapter so its preprocessing preserves the serialized raw pseudorange unchanged;
5. lets RTKLIB `pntpos()` apply the final TGD/BGD/ISC/DCB convention.

This reuses the production SPP adapter for solution options, ephemeris staging, `pntpos()` invocation, solution conversion, and diagnostics. The raw adapter only owns serialization-boundary validation and raw-pseudorange reconciliation.

## Compact CI gate

`RangeaRoundtripIntegration.RealWhuRinex4SerializedRangeaPositionsWithinHalfMeter` uses the provenance-traceable WHU-derived `BRD400DLR` RINEX 4 acceptance fixture:

- KS scenario;
- fixed receiver truth: 20 deg N, 120 deg E, 100 m;
- 1 Hz;
- 60 s;
- zero measurement noise;
- zero multipath;
- broadcast ionosphere/troposphere enabled;
- serialized RANGEA parsed back from the normal `simulated.log`;
- RANGEA epoch count must equal the simulator's emitted RANGE count;
- reconstructed valid-position epoch count must equal the simulator's maintained in-memory SPP valid-position count;
- maximum valid 3D position error must be `< 0.5 m`.

The successful compact baseline measured a maximum 3D error of `0.000817122 m` at GPST `2347/436501.000`.

Two additional gates require malformed serialized RANGEA to fail explicitly and same-input/same-seed runs to produce byte-identical logs plus identical round-trip positioning summaries.

## Real full-day WHU extended gate

`.github/workflows/rangea-roundtrip-extended.yml` runs the same parser and maintained RTKLIB SPP path against the pinned unmodified full-day WHU `BRD400DLR_S_20250030000_01D_MN.rnx` source introduced by Issue #58.

The scheduled/manual gate:

- downloads the exact pinned WHU RINEX NAV and verifies both compressed and uncompressed SHA256 values;
- runs a 10 minute KS simulation at 1 Hz starting at GPST week 2347 / SOW 436500;
- uses zero measurement noise and zero multipath;
- enables broadcast ionosphere/troposphere;
- parses only the generated `simulated.log` RANGEA records;
- lets RTKLIB use all available real EPH in the supplied RINEX for this baseline stage;
- requires RANGEA epoch count to equal the simulator `range_messages` count;
- requires round-trip valid-position epoch count to equal the simulator maintained-SPP count;
- requires at least four selected serialized observations per valid position epoch in aggregate;
- requires maximum 3D position error `< 0.5 m`.

With an intentionally pathological `0 deg` positioning mask, the 10 minute run measured `0.138821 m` maximum 3D error at GPST `2347/437021.000`.

### Confirmed cause of the 0-deg error spike

The `0.138821 m` peak is not caused by RANGEA serialization. RANGEA writes pseudorange to `0.001 m`, while the same epoch also shows an error spike before serialization in the simulator's in-memory SPP path.

The peak is caused by accepting a satellite essentially on the geometric horizon together with a geometry-consistency weakness in the generated atmosphere terms:

- BeiDou satellite 144 is at about `0.010385 deg` elevation at GPST `2347/437021`.
- The generated Saastamoinen troposphere delay for that observation is about `13241.954 m` because the mapping function is extremely sensitive near zero elevation.
- The simulator first computes atmosphere from the generic broadcast `SatelliteGeometry`.
- For a code-valid signal, `generate_zero_noise_measurement()` can then reselect the signal/message-family-specific ephemeris and recompute satellite state and geometric range.
- The already-computed ionosphere/troposphere terms are retained instead of being recomputed from the final family-specific line of sight.
- At normal elevations the geometric difference is negligible. Near zero elevation the troposphere mapping derivative is so large that the tiny line-of-sight difference becomes a many-metre pseudorange-model inconsistency.

Independent residual validation confirms that the outlier is concentrated in near-horizon observations. BeiDou satellite 144 reaches code residuals of about `18.601 m` on B1I at `0.006757 deg` and `34.473 m` on B3I at `0.004943 deg`; ordinary-elevation observations remain around sub-millimetre to tens-of-micrometre residuals.

Elevation-mask A/B validation confirms causality:

| Positioning mask | Maximum 3D round-trip error |
| ---: | ---: |
| 0.0 deg | 0.138821 m |
| 0.5 deg | 0.000559 m |
| 1.0 deg | 0.000559 m |
| 3.0 deg | 0.000669 m |
| 5.0 deg | 0.000722 m |
| 10.0 deg | 0.000867 m |

Therefore the 13.9 cm value is a pathological near-horizon atmosphere/geometry-consistency artifact exposed by the diagnostic `0 deg` mask. At the project's normal `3 deg` mask, the same full-day WHU 10 minute RANGEA round-trip remains sub-millimetre (`0.000669 m`).

The full-day source is never modified and no synthetic ephemeris fallback exists.

## Reusable validator

The `validate-rangea-roundtrip` executable exposes the same streaming evaluator for retained logs:

```text
validate-rangea-roundtrip \
  --log simulated.log \
  --nav BRD400DLR_S_20250030000_01D_MN.rnx \
  --truth-lat 20 \
  --truth-lon 120 \
  --truth-height 100 \
  --elevation-mask 3 \
  --broadcast-atmosphere
```

It reports RANGE epoch count, parsed observation count, selected SPP observation count, valid RTKLIB position epochs, maximum 3D position error, and the GPST of the maximum error.

The parser itself is streaming. Receiver-realistic NAV timing can be added later at the RTKLIB adapter boundary without changing the RANGEA parser or introducing a second solution implementation.

## Navigation provenance

Issue #58 remains binding. Navigation truth for this validation must come from actual RINEX NAV records. The round-trip validator does not synthesize ephemeris and has no synthetic-NAV fallback.
