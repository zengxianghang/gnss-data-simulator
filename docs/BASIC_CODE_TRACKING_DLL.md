# Basic Noncoherent Early-Late Power DLL

Issue #119 implements the receiver-layer DLL engine after #133 centralized signal correlation metadata and #134 implemented the pure signal-specific ideal code correlation functions.

## Ownership boundary

The DLL consumes normalized path-domain inputs:

- relative code delay in seconds;
- normalized complex voltage contribution.

The voltage value is already in one common receiver-domain convention. The DLL does **not** reinterpret wall TE/TM response, polarization, receive-antenna gain, free-space/path loss, Fresnel diffraction, open-sky C/N0, or any other RF/power term.

Issue #120 owns the production mapping from #116/#117/#118 plus open-sky C/N0 into this common complex-voltage convention.

This boundary is important for #118/#130: rooftop diffraction modifies the direct component. It must be represented exactly once by the producer of the direct complex voltage. The DLL must never construct `full direct + full diffraction` itself.

## Composite correlator

For one signal and normalized propagation paths `i`, the local composite correlator is

`Z(epsilon) = sum_i V_i * R(epsilon - tau_i)`

where:

- `epsilon` is the candidate local code phase relative to the caller's reference path;
- `tau_i` is each relative path delay;
- `V_i` is the supplied complex voltage;
- `R` is the signal-specific ideal correlation from #134.

All paths for one call therefore use the same central `SignalDefinition`; there is no DLL-local signal mapping table.

## Early/Late discriminator

The frozen default **total** Early/Late spacing is

`early_late_total_spacing_chips = 0.2`.

Thus the default taps are `-0.1 chip` and `+0.1 chip` around the candidate code phase.

The noncoherent power discriminator is

`D(epsilon) = |Z(epsilon - s/2)|^2 - |Z(epsilon + s/2)|^2`

where `s` is the total spacing.

The V1 configuration API accepts finite spacing in `(0, 2] chip`; it does not implement a dynamic DLL loop filter.

## Stability sign convention

The documented conceptual correction law is

`epsilon_next = epsilon - k * D(epsilon)`, with `k > 0`.

Under this sign convention a locally stable equilibrium has

`dD/depsilon > 0`.

The implementation estimates that local slope symmetrically around each refined root and records both the slope and the resulting `stable` classification. This avoids an unlabeled sign convention.

## Bounded deterministic root search

Only the active correlation-support interval is searched. For path delays measured in chips, with total spacing `s`, the interval is bounded by

`[min(tau) - 1 - s/2, max(tau) + 1 + s/2]`.

This excludes the infinite external region where both E/L correlators are identically zero and prevents those zero plateaus from being reported as tracking equilibria.

The V1 solver:

1. scans the active interval at a fixed maximum step of `1/512 chip`;
2. ignores samples numerically indistinguishable from zero while retaining the last nonzero sign;
3. brackets every detected sign change;
4. refines each bracket deterministically by bisection;
5. de-duplicates numerically coincident roots;
6. computes prompt power and local discriminator slope for every root;
7. surfaces stable and unstable roots rather than hiding side-peak solutions.

The search is deliberately bounded. If a caller supplies a path-delay span requiring more than the fixed V1 scan budget, the solver fails explicitly instead of silently coarsening the grid and risking missed BOC roots. The required interval count is validated as a finite bounded floating-point value **before** conversion to the integer loop count, so pathological delay spans cannot rely on an out-of-range floating-to-integer conversion.

## Root selection

Two deterministic policies are exposed:

### Tracked

Among stable roots, choose the root nearest the previous tracked code phase. Exact-distance ties prefer larger prompt power, then lower code phase.

### Acquisition / reacquisition

Among stable roots, choose the largest prompt-power candidate. Exact-power ties prefer the candidate nearest zero relative delay, then lower code phase.

The root set remains visible to later tracking-state code; #119 does not hide alternative stable roots behind a first-match rule. Unknown selection-mode enum values fail explicitly rather than inheriting a first-candidate result.

## Physical regression behavior

Permanent tests cover:

- one normalized LOS path: zero-bias stable root at the reference code phase;
- delayed in-phase second path: positive/later code bias;
- delayed opposite-phase path: negative near-direct bias plus additional stable solution(s);
- GPS L1C TMBOC side-peak roots: multiple roots are surfaced and deterministic selection remains explicit;
- changing total E/L spacing changes the two-path equilibrium;
- complex path voltages are summed as supplied without RF reinterpretation;
- unsupported signal profiles and malformed inputs fail explicitly;
- excessive path-delay spans and unknown root-selection modes fail explicitly;
- repeated root enumeration is bitwise deterministic for identical inputs.

## Explicit exclusions

Issue #119 does not implement:

- open-sky or effective C/N0;
- conversion of #116/#117/#118 physics into final path voltage amplitudes (#120);
- tracking C/N0 thresholds/hysteresis/reacquisition timers (#121);
- pseudorange/Doppler/carrier/ADR synthesis (#122);
- carrier smoothing or proprietary multipath mitigation;
- navigation-layer multipath rejection;
- IF/baseband samples;
- NAV/EPH/ION generation, modification, interpolation, or retargeting.
