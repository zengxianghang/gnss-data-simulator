# Carrier Tracking Runtime Integration

This document records the Issue #161 runtime integration of the standalone carrier-tracking core from Issue #160.

## Scope

The runtime layer is deliberately separate from physical propagation. It does not recompute satellite geometry, reflections, diffraction, DLL roots, coherent received power, or the urban carrier path derivative.

The observation chain is:

```text
clean authentic-NAV measurement
        |
        v
urban physical propagation (when enabled)
        |
        |  range_rate += environmental_range_rate_mps
        |  Doppler    -= environmental_range_rate_mps / wavelength
        v
physical observation / existing truth outputs
        |
        v
receiver carrier tracker
        |
        |  Doppler    += carrier_tracking_error_hz
        |  range_rate -= wavelength * carrier_tracking_error_hz
        v
reported observation
        |
        v
existing generic measurement-error model (when enabled)
```

Therefore the `environmental_range_rate_mps` validated by Issue #152 remains an independent physical quantity. Carrier tracking does not replace or modify it.

## Configuration

Carrier tracking is explicitly opt-in:

```json
{
  "carrier_tracking": {
    "enabled": true,
    "coherent_integration_sec": 0.020,
    "pll_noise_bandwidth_hz": 5.0,
    "fll_noise_bandwidth_hz": 4.0,
    "fll_pull_in_bandwidth_hz": 8.0,
    "fll_pull_in_duration_sec": 0.5,
    "pll_enter_cn0_dbhz": 30.0,
    "pll_exit_cn0_dbhz": 27.0,
    "pll_enter_persistence_sec": 1.0,
    "pll_exit_persistence_sec": 0.3,
    "fll_enter_cn0_dbhz": 22.0,
    "fll_exit_cn0_dbhz": 18.0,
    "fll_enter_persistence_sec": 0.2,
    "fll_exit_persistence_sec": 0.5,
    "doppler_valid_delay_sec": 0.2,
    "adr_valid_after_pll_sec": 1.0
  }
}
```

The default is `enabled=false`. All numerical parameters are still validated while disabled, so an invalid dormant configuration cannot silently become active later.

## Effective C/N0 source

The carrier tracker consumes the C/N0 already produced by the receiver signal chain:

- urban/multipath enabled: `UrbanSignalEpochResult::effective_cn0_dbhz`;
- open sky: `SignalTracker::cn0_dbhz`.

No independent attenuation, fixed Doppler sigma, or target-PVT error term is introduced.

## Time stepping

Simulator output rates are 1, 5, 10, 20, or 50 Hz, while the frozen coherent integration interval is 20 ms. A carrier state machine update only once per output epoch would skip 0.2/0.3/0.5 s persistence boundaries at low output rates.

The runtime therefore divides each elapsed output interval into substeps no larger than `coherent_integration_sec`. C/N0 is held constant during those substeps in V1. This preserves the configured persistence, FLL pull-in, Doppler-valid, and PLL/ADR timing semantics without pretending that new propagation samples exist between receiver output epochs.

The first epoch on which code tracking is alive establishes the carrier runtime timestamp only. It does not retroactively advance carrier acquisition through an interval during which no carrier runtime state existed. Consequently Doppler is not restored instantly on the first code-tracking/reacquisition sample.

## Per-signal RNG ownership

Each satellite + signal owns an independent deterministic PCG stream derived from:

- simulator seed;
- satellite number;
- central `SignalId`.

The carrier runtime never advances `RuntimeState::rng`, which remains the pre-existing receiver startup/acquisition RNG. It also does not use the generic measurement-error noise stream/state.

When `carrier_tracking.enabled=false`, the runtime does not call the carrier updater and consumes no carrier samples. Changing dormant carrier parameters therefore cannot reorder or perturb legacy receiver output.

Power/signal/code-tracking hard resets clear carrier tracking and continuity state but intentionally retain that signal's independent RNG stream. This models a receiver channel whose random process continues deterministically while lock state is restarted, without coupling unrelated signals.

## Doppler and range-rate sign convention

The existing clean measurement convention is:

```text
D = -(range_rate - satellite_clock_drift) / wavelength
```

For carrier frequency error `e_f` in hertz, Issue #159 defines:

```text
D_measured = D_physical + e_f
```

The wavelength-consistent range-rate representation must therefore be:

```text
range_rate_measured = range_rate_physical - wavelength * e_f
```

The runtime applies both terms together. Pseudorange is not changed by the carrier-tracking layer.

## Independent validity

Carrier validity is ANDed with the already-existing code/urban validity rather than replacing it.

- `PLL_TRACK`: Doppler can be valid after the carrier acquisition delay; ADR becomes valid only after the configured PLL confirmation age.
- `FLL_TRACK`: pseudorange can remain valid; Doppler can remain valid; ADR is invalid.
- `CARRIER_UNLOCKED`: pseudorange can remain valid while Doppler and ADR are invalid.
- no code tracking / BLOCKED: no normal carrier result is emitted and carrier state is hard-reset.

Thus code, Doppler, and ADR validity are intentionally independent.

## PLL continuity and ambiguity segments

The pre-existing `CarrierAmbiguityState` is keyed to code-tracker `tracking_start_time`. A PLL-to-FLL transition can break carrier-phase continuity while code tracking remains alive, so resetting that existing ambiguity object alone would regenerate the same integer ambiguity.

Issue #161 therefore maintains a separate deterministic carrier phase-segment identifier. On the first transition from `PLL_TRACK` to a non-PLL state:

1. ADR becomes invalid immediately through the #160 result;
2. the phase-segment identifier increments;
3. a deterministic non-zero integer cycle offset is derived from the per-signal ambiguity key and new segment id;
4. when a later PLL segment satisfies the ADR-valid delay, that offset is added to both reported ADR cycles and the reported ambiguity-cycle field.

The code tracker `tracking_start_time` is not falsified or rewritten. Power/signal/code loss already establishes a fresh code-tracking ambiguity and resets this carrier phase-segment overlay.

## Truth boundary for Issue #161

Existing truth files are intentionally written before the receiver carrier-tracking layer. This keeps the existing physical/urban truth schema stable and preserves the Issue #152 path-rate evidence chain.

Issue #162 will add exact in-memory carrier-tracking diagnostics including mode, active bandwidth, theoretical sigma, actual tracking error, phase segment/cycle-slip status, and final carrier validity. Issue #161 does not change truth CSV schemas.

## V1 exclusions

This runtime work does not add:

- receiver oscillator/common-mode phase noise;
- moving receiver or reflector models;
- IF/IQ/baseband samples;
- arbitrary weak-signal Doppler noise;
- PVT-target tuning;
- any NAV/EPH/ION fabrication or modification.
