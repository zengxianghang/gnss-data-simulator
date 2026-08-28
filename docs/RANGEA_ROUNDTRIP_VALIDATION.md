# RANGEA black-box round-trip validation

Issue #60 adds an independent serialization-boundary validation for the receiver RANGE output.

## Why this exists

The simulator already had an in-memory loopback:

`real RINEX NAV -> MeasurementObservation -> RTKLIB SPP`

The same `MeasurementObservation` objects were also passed to the RANGEA writer. That proves the measurement and solution models are internally consistent, but it does not prove that the serialized receiver interface preserves the same observation semantics.

The black-box gate instead executes:

`real RINEX NAV -> simulator -> simulated.log RANGEA -> independent parser -> raw RTKLIB code observations -> maintained RTKLIB SPP adapter -> pntpos() -> truth comparison`

The RTKLIB observations used by this gate come only from parsed RANGEA text. The evaluator does not read `observation_truth.csv` and does not receive the simulator's in-memory `MeasurementObservation` objects.

## Elevation-mask semantics

The simulator keeps measurement visibility and navigation-solution filtering independent:

- `elevation_mask_deg` controls simulated tracking/measurement availability and therefore which low-elevation observations can appear in RANGEA;
- `solution_elevation_mask_deg` controls the RTKLIB position/velocity solution elevation cutoff;
- `solution_elevation_mask_deg` defaults to **5 deg** when omitted, including for existing schema-version-1 configs;
- lowering `elevation_mask_deg` does not lower the RTKLIB SPP cutoff unless `solution_elevation_mask_deg` is changed explicitly.

This matches a normal receiver architecture: RANGE can retain measurements that the navigation engine elects not to use. Issue #62 validation deliberately uses `elevation_mask_deg = 0 deg` and `solution_elevation_mask_deg = 5 deg` to prove that observations below 5 deg remain serialized while normal SPP rejects them.

The separate low-elevation atmosphere/geometry consistency defect is tracked by Issue #63. The 5 deg solution policy is not considered a physical-model fix for that defect.

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

`RangeaRoundtripIntegration.LowElevationRangeIsRetainedWhileSppUsesFiveDegreeMask` uses the provenance-traceable WHU-derived `BRD400DLR` RINEX 4 acceptance fixture:

- KS scenario;
- fixed receiver truth: 20 deg N, 120 deg E, 100 m;
- 1 Hz;
- 60 s;
- measurement/tracking elevation mask: `0 deg`;
- RTKLIB solution elevation mask: `5 deg`;
- zero measurement noise;
- zero multipath;
- broadcast ionosphere/troposphere enabled;
- at least one valid pseudorange below 5 deg must remain in the generated observation/RANGE path;
- serialized RANGEA is parsed back from the normal `simulated.log`;
- RANGEA epoch count must equal the simulator's emitted RANGE count;
- reconstructed valid-position epoch count must equal the simulator's maintained in-memory SPP valid-position count;
- maximum valid 3D position error must be `< 0.01 m`.

The exact maximum 3D error and its GPST are printed by Ubuntu CI on every successful compact run.

Two additional gates require malformed serialized RANGEA to fail explicitly and same-input/same-seed runs to produce byte-identical logs plus identical round-trip positioning summaries.

## Real full-day WHU extended gate

`.github/workflows/rangea-roundtrip-extended.yml` runs the same parser and maintained RTKLIB SPP path against the pinned unmodified full-day WHU `BRD400DLR_S_20250030000_01D_MN.rnx` source introduced by Issue #58.

The scheduled/manual gate:

- downloads the exact pinned WHU RINEX NAV and verifies both compressed and uncompressed SHA256 values;
- runs a 10 minute KS simulation at 1 Hz starting at GPST week 2347 / SOW 436500;
- uses a `0 deg` measurement/tracking mask so low-elevation RANGE observations are retained;
- uses a `5 deg` RTKLIB SPP mask independently;
- records both masks explicitly in `run_manifest.json` and verifies them in the gate;
- uses zero measurement noise and zero multipath;
- enables broadcast ionosphere/troposphere;
- parses only the generated `simulated.log` RANGEA records;
- lets RTKLIB use all available real EPH in the supplied RINEX for this baseline stage;
- requires RANGEA epoch count to equal the simulator `range_messages` count;
- requires round-trip valid-position epoch count to equal the simulator maintained-SPP count;
- requires at least four selected serialized observations per valid position epoch in aggregate;
- requires maximum 3D position error `< 0.01 m` with the 5 deg SPP mask.

### Confirmed cause of the old 0-deg SPP error spike

Before the masks were separated, a diagnostic also used `0 deg` as the **positioning** mask. That produced a `0.138821 m` maximum 3D error at GPST `2347/437021.000`. The peak was not caused by RANGEA serialization; the same epoch already showed the spike in the simulator's in-memory SPP path.

The root cause is a near-horizon atmosphere/geometry consistency weakness:

- BeiDou satellite 144 is at about `0.010385 deg` elevation at GPST `2347/437021`.
- The generated Saastamoinen troposphere delay for that observation is about `13241.954 m` because the mapping function is extremely sensitive near zero elevation.
- The simulator first computes atmosphere from the generic broadcast `SatelliteGeometry`.
- For a code-valid signal, `generate_zero_noise_measurement()` can then reselect the signal/message-family-specific ephemeris and recompute satellite state and geometric range.
- The already-computed ionosphere/troposphere terms are retained instead of being recomputed from the final family-specific line of sight.
- At normal elevations the geometric difference is negligible. Near zero elevation the troposphere mapping derivative is so large that the tiny line-of-sight difference becomes a many-metre pseudorange-model inconsistency.

Independent residual validation found about `18.601 m` B1I and `34.473 m` B3I maximum code residuals on that near-horizon satellite. An A/B position solve using the same generated data showed:

| RTKLIB positioning mask | Maximum 3D round-trip error |
| ---: | ---: |
| 0.0 deg | 0.138821 m |
| 0.5 deg | 0.000559 m |
| 1.0 deg | 0.000559 m |
| 3.0 deg | 0.000669 m |
| 5.0 deg | 0.000722 m |
| 10.0 deg | 0.000867 m |

The normal 5 deg solution mask therefore prevents this pathological near-horizon observation from entering routine SPP, while the 0 deg measurement mask can still preserve it in RANGE for downstream analysis. Issue #63 remains responsible for making the generated physical model itself self-consistent at low elevation.

The full-day source is never modified and no synthetic ephemeris fallback exists.

## Reusable validator

The `validate-rangea-roundtrip` executable exposes the same streaming evaluator for retained logs. Its `--elevation-mask` argument is a **solution** mask for the RTKLIB round-trip solve; it does not alter observations already present in a retained RANGE log:

```text
validate-rangea-roundtrip \
  --log simulated.log \
  --nav BRD400DLR_S_20250030000_01D_MN.rnx \
  --truth-lat 20 \
  --truth-lon 120 \
  --truth-height 100 \
  --elevation-mask 5 \
  --broadcast-atmosphere
```

It reports RANGE epoch count, parsed observation count, selected SPP observation count, valid RTKLIB position epochs, maximum 3D position error, and the GPST of the maximum error.

The parser itself is streaming. Receiver-realistic NAV timing can be added later at the RTKLIB adapter boundary without changing the RANGEA parser or introducing a second solution implementation.

## Navigation provenance

Issue #58 remains binding. Navigation truth for this validation must come from actual RINEX NAV records. The round-trip validator does not synthesize ephemeris and has no synthetic-NAV fallback.
