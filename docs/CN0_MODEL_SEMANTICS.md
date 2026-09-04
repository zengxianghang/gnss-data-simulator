# CN0 model semantic contract

This document freezes the compatibility contract introduced by issue #105, the first implementation step of parent issue #104.

## Why two semantics are required

The legacy CN0 calibration model stores **absolute station-specific C/N0 in dB-Hz**. A normalized portable model instead stores an **elevation-dependent offset in dB**. These are different physical quantities and must never share an ambiguous schema.

## `gnss-cn0-model-v1` — `ABSOLUTE_STATION_CN0`

The existing v1 CSV header and row format are retained byte-for-byte. `p50_dbhz` is an absolute station/receiver/antenna C/N0 value. Existing interpolation and built-in fallback behavior remain unchanged.

Loading a v1 model sets:

```text
semantic = ABSOLUTE_STATION_CN0
```

No v1 file is silently reinterpreted as a normalized model.

## `gnss-cn0-model-v2` — `NORMALIZED_ELEVATION_SHAPE`

The normalized runtime CSV contract is intentionally compact:

```text
schema_version,model_semantic,constellation,signal,elevation_min_deg,elevation_max_deg,upper_edge_inclusive,status,contributing_source_count,delta_p50_db
```

Rows must use:

```text
schema_version = gnss-cn0-model-v2
model_semantic = NORMALIZED_ELEVATION_SHAPE
```

`delta_p50_db` is a relative C/N0 offset in **dB**, not an absolute dB-Hz value. `contributing_source_count` records how many eligible normalized calibration sources contributed to the aggregate bin. `EMPTY` rows have zero contributing sources and an empty delta value. `SPARSE`/`READY` rows require a positive source count and a finite delta value.

Issue #105 only introduces the explicit semantic/schema boundary. A normalized model is deliberately **not** interpreted as absolute C/N0 by `cn0_model_estimate_dbhz()` until issue #107 composes it with a receiver-specific high-elevation baseline.

## Receiver high-elevation baseline configuration

Receiver/antenna-specific absolute levels are configured independently in simulator JSON:

```json
{
  "cn0_high_dbhz": {
    "GPS L1 C/A": 47.25,
    "GPS L5Q": 49.00,
    "Galileo E1": 48.00
  }
}
```

Keys must exactly match `SignalDefinition::name` from the repository's central signal-definition table. Unknown signals, duplicate keys, non-numeric values, and non-finite values are rejected deterministically.

The default baseline list is empty. The simulator does **not** invent receiver-specific absolute C/N0 values. Production values must be configured or calibrated from suitable real receiver/antenna observations.

## Compatibility rules

- Explicitly selected malformed/unknown model schemas fail; there is no silent fallback to a different semantic.
- `gnss-cn0-model-v1` remains the legacy absolute-station format.
- `gnss-cn0-model-v2` is reserved for normalized elevation-shape output produced by issue #106.
- A normalized delta must never be treated as dB-Hz.
- An absolute built-in/station CN0 value must never be treated as an elevation delta.
- Fixed model/config/input bytes remain deterministic and reproducible.

## Follow-on issues

- #106 produces v2 normalized elevation-shape models by normalizing each real calibration source before cross-station aggregation.
- #107 composes `CN0_high_dbhz(signal) + Delta_CN0_elevation(signal,elevation)` at runtime.
- #108 validates the abstraction with deterministic fixtures and multiple real WHU/IGS stations.
