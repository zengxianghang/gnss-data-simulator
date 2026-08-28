# RANGEA black-box round-trip validation

Issue #60 adds an independent serialization-boundary validation for the receiver RANGE output.

## Why this exists

The simulator already had an in-memory loopback:

`real RINEX NAV -> MeasurementObservation -> RTKLIB SPP`

The same `MeasurementObservation` objects were also passed to the RANGEA writer. That proves the measurement and solution models are internally consistent, but it does not prove that the serialized receiver interface preserves the same observation semantics.

The black-box gate instead executes:

`real RINEX NAV -> simulator -> simulated.log RANGEA -> independent parser -> raw RTKLIB code observations -> pntpos() -> truth comparison`

The RTKLIB observations used by this gate come only from parsed RANGEA text. The evaluator does not read `observation_truth.csv` and does not receive the simulator's in-memory `MeasurementObservation` objects.

## Independence rules

- RANGEA framing and CRC are decoded independently; the parser does not call the RANGEA writer or its framing helper.
- Serialized observation count and field count must agree exactly.
- Constellation and signal identity are decoded from the serialized tracking-status word and RANGE PRN representation.
- Canonical signal semantics are resolved through the frozen signal-definition table after the serialized constellation/signal type has been decoded.
- Unsupported constellation/signal mappings fail explicitly.
- Invalid numeric fields, CRC mismatches, inconsistent pseudorange-valid bits, and malformed GLONASS frequency fields fail explicitly.
- The parser streams the log line-by-line and does not require a long log to be loaded into memory.

## Raw RTKLIB positioning boundary

The round-trip evaluator deliberately does not reconstruct or reuse simulator-owned `code_bias_m`.

Parsed pseudorange is passed unchanged to `rtklib_solve_raw_single_position()`. That adapter selects the signal/message-family-compatible real broadcast ephemeris and then calls RTKLIB `pntpos()`. RTKLIB therefore applies its own TGD/BGD/ISC/DCB convention to the serialized raw code observation.

This is intentionally different from the normal in-memory solution adapter, which carries the simulator's generated code-bias term so it can reconcile that internal representation before the normal RTKLIB SPP call.

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
- at least one valid RTKLIB SPP epoch required;
- maximum valid 3D position error must be `< 0.5 m`.

The test also checks that RANGEA epoch count matches the simulator's emitted RANGE count.

Two additional gates require malformed serialized RANGEA to fail explicitly and same-input/same-seed runs to produce byte-identical logs plus identical round-trip positioning summaries.

## Reusable validator

The `validate-rangea-roundtrip` executable exposes the same streaming evaluator for retained or FIFO-fed logs:

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

The evaluator is streaming so the same path can be connected to the real full-day WHU source used by extended validation without introducing a second parser or loading an 8-hour RANGE log into memory.

## Navigation provenance

Issue #58 remains binding. Navigation truth for this validation must come from actual RINEX NAV records. The round-trip validator does not synthesize ephemeris and has no synthetic-NAV fallback.
