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

The current round-trip accuracy gate intentionally gives RTKLIB the complete provenance-traceable real RINEX NAV store. All ephemerides available in that real RINEX input may be selected by RTKLIB according to its normal broadcast-ephemeris selection rules.

This stage does **not** simulate receiver ephemeris acquisition order, transmission-time availability, or incremental old/new ephemeris updates. The purpose is first to isolate and measure the positioning error of the serialized observation path itself:

`generated physical observation -> RANGEA serialization -> independent RANGEA parse -> RTKLIB SPP`

Once this baseline error is established, receiver-realistic navigation acquisition/update timing can be added as a separate validation layer without conflating serialization/model errors with NAV-availability effects.

Issue #58 remains binding: every ephemeris used here still comes from an actual RINEX NAV record. "Use all available EPH" means unrestricted use of real RINEX records, never generated or modified ephemeris.

## Raw RTKLIB positioning boundary

The round-trip evaluator deliberately does not reconstruct or reuse simulator-owned `code_bias_m`.

The raw adapter is intentionally thin. It does not implement a second SPP solver and does not call `pntpos()` directly. For each selected serialized observation it:

1. validates the satellite, observation code, message-family mapping, pseudorange, and CN0;
2. verifies that the complete real RINEX NAV contains a compatible ephemeris/code-bias definition;
3. obtains RTKLIB's signal-specific broadcast code bias from that real NAV store;
4. passes the serialized pseudorange plus matching bias metadata into the maintained `rtklib_solve_single_position()` adapter so its preprocessing preserves the serialized raw pseudorange unchanged;
5. lets RTKLIB `pntpos()` perform the final broadcast ephemeris, TGD/BGD/ISC/DCB, atmosphere, and SPP calculations.

If a selected serialized satellite/signal/message-family combination cannot be supported by the complete real RINEX NAV, validation fails explicitly. There is no silent fallback to another signal or message family.

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
- complete real fixture NAV available to RTKLIB for the current accuracy-characterization stage;
- RANGEA epoch count must equal the simulator's emitted RANGE count;
- reconstructed valid-position epoch count must equal the simulator's maintained in-memory SPP valid-position count;
- maximum valid 3D position error must be `< 0.5 m`;
- the exact maximum 3D error and its GPST are printed in CI logs.

Two additional gates require malformed serialized RANGEA to fail explicitly and same-input/same-seed runs to produce byte-identical logs plus identical round-trip positioning summaries.

## Real full-day WHU extended gate

`.github/workflows/rangea-roundtrip-extended.yml` runs the same parser and maintained RTKLIB SPP path against the pinned unmodified full-day WHU `BRD400DLR_S_20250030000_01D_MN.rnx` source introduced by Issue #58.

The scheduled/manual gate:

- downloads the exact pinned WHU RINEX NAV and verifies both compressed and uncompressed SHA256 values;
- runs a 10 minute KS simulation at 1 Hz starting at GPST week 2347 / SOW 436500;
- uses zero measurement noise and zero multipath;
- enables broadcast ionosphere/troposphere;
- gives RTKLIB the complete real full-day RINEX NAV for this baseline accuracy stage;
- parses only the generated `simulated.log` RANGEA records;
- requires RANGEA epoch count to equal the simulator `range_messages` count;
- requires round-trip valid-position epoch count to equal the simulator maintained-SPP count;
- requires at least four selected serialized observations per valid position epoch in aggregate;
- requires maximum 3D position error `< 0.5 m`.

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
  --elevation-mask 0 \
  --broadcast-atmosphere
```

It reports RANGE epoch count, parsed observation count, selected SPP observation count, valid RTKLIB position epochs, maximum 3D position error, and the GPST of the maximum error.

The parser itself is streaming and the SPP boundary reuses the maintained RTKLIB solution adapter, so later receiver-NAV timing validation can be added without changing the RANGEA parser or duplicating the positioning implementation.

## Navigation provenance

Issue #58 remains binding. Navigation truth for this validation must come from actual RINEX NAV records. The round-trip validator does not synthesize ephemeris and has no synthetic-NAV fallback.
