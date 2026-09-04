# Urban truth diagnostics

Issue #123 adds a deterministic, serialization-only truth extension for the V1 urban propagation/tracking model.

## Scope and invariants

The urban truth writer consumes the exact `UrbanSignalEpochResult` and `UrbanCarrierTemporalResult` objects already used by the simulator runtime to synthesize receiver observables. It must not recompute scene geometry, reflection paths, rooftop diffraction, DLL roots, effective CN0, tracking state, carrier phase, or environmental path rate.

Normal receiver `RANGEA` output remains receiver-observable only. Simulator-only propagation/material/tracking details are written only to the dedicated truth files below.

No NAV/EPH/ION record is generated, altered, interpolated, retargeted, or fabricated by this diagnostic layer.

## Schema version

The urban truth extension has its own schema version independent of the existing base truth bundle:

```text
URBAN_TRUTH_SCHEMA_VERSION = 1
```

Every row in both urban CSV files starts with `truth_schema_version` so a consumer can reject an unsupported extension version before interpreting later columns. Adding these files does not reinterpret or renumber the existing `observation_truth.csv` schema.

When `multipath_enabled=false`, the files are still created with their versioned headers but contain no urban data rows. The legacy/open-sky observation path is unchanged.

## `urban_signal_truth.csv`

One row is written for every urban-enabled satellite/signal epoch for which production signal geometry is available, including states that do not emit a normal receiver observation.

Important field groups:

- identity/time: GPS week/TOW, satellite number, signal id/name, GLONASS FCN;
- geometry: azimuth/elevation, whether propagation was evaluated, direct LOS, blocking wall, roof-grazing state, diffraction status;
- path inventory: retained reflection count and common received-path count;
- tracking: `LOS`, `LOS_MULTIPATH`, `NLOS_TRACKED`, or `BLOCKED`, tracking phase, loss reason, lock time, reacquisition and validity flags;
- received power: open-sky CN0, effective CN0, composite power ratio;
- DLL: root count/search status, deterministic selection mode/root, code phase and physical pseudorange code bias;
- carrier/temporal: tracked composite complex correlation, wavelength, wrapped/unwrapped phase, carrier-range bias, environmental range rate, continuity and cycle-slip state;
- observation consistency: whether the runtime produced a zero-noise receiver observation and, when present, the exact pseudorange/Doppler/ADR values after urban mapping but before the independent measurement-noise layer.

If propagation was not evaluated because the signal was unavailable at the runtime boundary, geometry/path-dependent fields are explicitly blank or marked `NOT_EVALUATED`; the tracking/loss state is still recorded.

This permits `BLOCKED` and signal-loss events to be diagnosed without inferring them only from missing `RANGEA` observations.

## `urban_path_truth.csv`

This file contains only already-computed propagation components from the same `UrbanSignalEpochResult`.

Stable ordering per satellite/signal/epoch is:

1. `path_index=0`: the roof-affected direct component (`DIRECT_ROOF`), represented exactly once;
2. `path_index=1..N`: every retained #117 first-order reflection in the existing stable wall/path order.

The roof transition therefore never appears as an unmodified full direct path plus a second full diffraction path.

Per-path fields include, where applicable:

- path kind and source wall/roof edge;
- reflection/diffraction ENU point;
- model path range, excess path and code delay;
- normalized complex receiver-domain voltage, amplitude and phase;
- rooftop Fresnel `v` and complex coefficient;
- reflection incidence angle;
- RHCP/LHCP material response and receive-antenna complex response.

Fields that are not applicable to a path type are left blank rather than filled with invented values.

## Relationship to other truth files

`observation_truth.csv` continues to describe the zero-noise receiver observation and authentic-NAV/broadcast-bias inputs. For an emitted urban observation, `urban_signal_truth.csv` carries the same post-urban pseudorange, Doppler and ADR values, which is regression-tested against `observation_truth.csv` by matching epoch + satellite + signal.

`event_truth.csv`, `solution_truth.csv`, `scenario.json`, and `run_manifest.json` retain their existing base truth semantics.

## Determinism

For fixed simulator commit, authentic NAV bytes, configuration, CN0 model and seed:

- urban signal rows are deterministic;
- path arrays use stable ordering;
- the two urban CSV files are byte-repeatable;
- the diagnostic writer has no RNG and does not alter receiver state.
