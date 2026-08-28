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

## Receiver-available navigation state

The round-trip validator loads the provenance-traceable real RINEX NAV as the source of truth, but it does not hand the full-day NAV store directly to `pntpos()`.

For every RANGEA epoch, `rtklib_solve_raw_single_position()` first builds an RTKLIB NAV snapshot at that GPST using the existing `rtklib_copy_nav_snapshot()` rule. Only navigation records whose broadcast/transmission time is at or before the RANGEA epoch are available to the solve. Records transmitted later in the full-day RINEX file cannot be selected as future ephemeris.

This is the HOT/KS availability convention already used when the simulator initializes receiver navigation from real RINEX. It makes the black-box gate safe across navigation handovers while retaining real RINEX values unchanged.

A raw RANGE observation can legitimately be valid before the receiver has acquired the corresponding broadcast ephemeris. Such an observation is not sent to SPP for that epoch. This is distinguished from a mapping/provenance error: if the receiver snapshot lacks the family but the complete real RINEX contains a compatible record, the observation is classified as receiver-NAV-unavailable; if even the complete real RINEX cannot provide the serialized satellite/signal/message-family combination, validation fails explicitly.

## Raw RTKLIB positioning boundary

The round-trip evaluator deliberately does not reconstruct or reuse simulator-owned `code_bias_m`.

The raw adapter is intentionally thin. It does not implement a second SPP solver and does not call `pntpos()` directly. For each selected serialized observation it:

1. validates the satellite, observation code, message-family mapping, pseudorange, and CN0;
2. builds the receiver-available NAV snapshot for the RANGEA epoch;
3. checks whether the receiver currently owns a compatible ephemeris/code-bias definition;
4. skips only the legitimate receiver-NAV-unavailable case described above, while a combination unsupported by the complete real RINEX is a hard failure;
5. obtains RTKLIB's broadcast code bias for each observation that can actually enter SPP;
6. passes the serialized pseudorange plus that matching bias metadata into the maintained `rtklib_solve_single_position()` adapter so its preprocessing preserves the serialized raw pseudorange unchanged;
7. lets RTKLIB `pntpos()` apply the final TGD/BGD/ISC/DCB convention.

This reuses the production SPP adapter for solution options, ephemeris staging, `pntpos()` invocation, solution conversion, and diagnostics. The raw adapter only owns serialization-boundary validation, receiver-NAV availability, and raw-pseudorange reconciliation.

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

Two additional gates require malformed serialized RANGEA to fail explicitly and same-input/same-seed runs to produce byte-identical logs plus identical round-trip positioning summaries.

## Real full-day WHU extended gate

`.github/workflows/rangea-roundtrip-extended.yml` runs the same parser and maintained RTKLIB SPP path against the pinned unmodified full-day WHU `BRD400DLR_S_20250030000_01D_MN.rnx` source introduced by Issue #58.

The scheduled/manual gate:

- downloads the exact pinned WHU RINEX NAV and verifies both compressed and uncompressed SHA256 values;
- runs a 10 minute KS simulation at 1 Hz starting at GPST week 2347 / SOW 436500;
- uses zero measurement noise and zero multipath;
- enables broadcast ionosphere/troposphere;
- crosses the real navigation-update interval around SOW 436800;
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

The parser itself is streaming. Receiver-available NAV snapshots are created at the RTKLIB adapter boundary so the same evaluator can be applied to longer retained logs without changing the RANGEA parser or introducing a second solution implementation.

## Navigation provenance

Issue #58 remains binding. Navigation truth for this validation must come from actual RINEX NAV records. The round-trip validator does not synthesize ephemeris and has no synthetic-NAV fallback.
