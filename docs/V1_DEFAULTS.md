# V1 Default Simulation Parameters

This document records the currently frozen V1 defaults. All values below are defaults and must remain configurable unless explicitly stated otherwise.

## Visibility mask

- Default satellite elevation mask: **3 degrees**.
- Satellites below the configured elevation mask are not used for simulated observations/solutions.
- The elevation mask must be configurable.

## Navigation output

V1 outputs both **ephemeris (EPH)** and **ionosphere/navigation auxiliary (ION)** records in addition to RANGEA, PSRPOSA, and PSRVELA.

The exact supported navigation record families are defined in [`NAV_RECORDS.md`](NAV_RECORDS.md).

Navigation output must reflect the simulated Receiver NAV state:

- HOT/WARM startup restores cached EPH/ION immediately and queues their log output.
- COLD startup outputs EPH progressively as the corresponding navigation message fragments are collected and the ephemeris becomes usable.
- REA signal-off retains Receiver NAV and continues normal navigation-state behavior.

## Measurement noise

- **V1 does not add stochastic measurement noise.**
- Pseudorange measurement-noise contribution is zero.
- Doppler measurement-noise contribution is zero.
- Carrier-phase/ADR measurement-noise contribution is zero.
- The observation-generation architecture should keep measurement noise as an independent future extension point, but noise injection is not part of the V1 required implementation.
- CN0 is still generated from the elevation/signal model and is still used by the tracking/acquisition state machine. It does not introduce measurement noise into PSR, Doppler, or ADR in V1.
- This zero-noise policy is intended to make V1 a deterministic physics/format/state-machine validation baseline and to permit near-zero RTKLIB loopback residuals where the same physical corrections are applied consistently.

## Receiver clock

- **V1 receiver clock bias is fixed to zero.**
- **V1 receiver clock drift is fixed to zero.**
- V1 does not implement receiver clock random walk, oscillator noise, drift instability, or power-cycle-dependent clock offsets.
- The observation equations must still keep receiver clock bias/drift as explicit model terms so a realistic oscillator model can be added later without changing the measurement-model interfaces.
- Satellite clock bias and satellite clock drift remain active and must come from the selected navigation truth model; this section only disables simulated **receiver** clock error.

Therefore the V1 receiver-side clock contribution is:

```text
receiver_clock_bias_m   = 0.0
receiver_clock_drift_mps = 0.0
```

For REA, the receiver remains powered, but the zero-valued V1 receiver clock state simply remains zero through the signal outage. For TTFF power cycles it is reinitialized to the same zero-valued V1 state.

## Multipath

- **V1 does not add a multipath error model.**
- The pseudorange/phase/Doppler architecture should keep multipath as a separate future extension point, but the V1 default and V1 required implementation use zero multipath contribution.
- Observation realism in V1 comes from real satellite geometry/navigation data, signal-specific code-bias corrections, configured atmosphere models, CN0 modeling, and tracking-state behavior rather than synthetic multipath or measurement noise.

## Default generated duration

- Default total generated test-data duration: **8 hours**.
- Duration must be configurable.
- This default applies to KS, REA, and TTFF unless a scenario configuration explicitly overrides it.
- REA and TTFF continue repeating their configured ON/OFF cycles until the configured total duration is reached.

Current default cycles remain:

```text
REA
SIGNAL ON  = 300 s
SIGNAL OFF = 10 s
repeat until duration is reached

TTFF
POWER ON  = 300 s
POWER OFF = 30 s
repeat until duration is reached
```

For TTFF, the configured startup mode (HOT/WARM/COLD, default HOT) is applied at each POWER OFF -> POWER ON transition unless a future per-cycle startup schedule is explicitly configured.

## Startup and reacquisition timing defaults

Detailed semantics and calibration references are defined in [`STARTUP_RECOVERY_MODEL.md`](STARTUP_RECOVERY_MODEL.md).

The frozen V1 default open-sky targets are:

```text
HOT
  TTFF P50        ~= 2 s
  TTFF P95        ~= 4 s
  nominal target  <= 5 s

WARM
  TTFF P50        ~= 6-8 s
  TTFF P95        ~= 15-20 s
  nominal target  <= 30 s

COLD
  NAV-message-driven; no artificial fixed TTFF
  typical/P50 region ~= 20-30 s
  P95 target         <= 45 s
  approximately 10-20 s is allowed when real multi-GNSS NAV timing permits it

REA
  position recovery P50 ~= 0.8-1.2 s
  position recovery P95 <= 2 s
```

HOT and WARM both restore all cached EPH/ION immediately. WARM is slower because its approximate time/position prior is less accurate and therefore its signal-acquisition search space is wider.

COLD is fundamentally different: Receiver NAV starts without usable EPH and ephemeris availability is determined by the actual navigation-message frame/page/message collection logic. The implementation must not insert a random fixed cold-start delay solely to hit the target TTFF statistics.

REA remains powered and retains NAV/time/position state, so its reacquisition timing model is separate from HOT startup and should be materially faster.

Recommended HOT per-signal acquisition-delay calibration before small deterministic timing jitter:

```text
CN0 >= 40 dB-Hz : about 0.2-0.6 s
35-40 dB-Hz     : about 0.3-1.0 s
30-35 dB-Hz     : about 0.6-2.0 s
<30 dB-Hz       : about 1.0-4.0 s
```

Recommended receiver-level delay calibration:

```text
HOT common startup delay
  P50 ~= 0.8 s
  P95 ~= 1.5 s

WARM extra search-uncertainty delay
  P50 ~= 4-5 s
  P95 ~= 12-15 s
```

All stochastic timing behavior must use the deterministic seeded PRNG and remain configurable. This timing randomness is independent of the V1 zero measurement-noise and zero receiver-clock-error policies.

## Summary

```text
elevation_mask_deg       = 3.0
output_eph               = true
output_ion               = true
measurement_noise        = false
psr_noise_m              = 0.0
doppler_noise            = 0.0
adr_noise                = 0.0
receiver_clock_bias_m    = 0.0
receiver_clock_drift_mps = 0.0
receiver_clock_random_walk = false
multipath_enabled        = false
duration_sec             = 28800  # 8 h

ttff.default_mode        = HOT

HOT target:
  P50 ~= 2 s
  P95 ~= 4 s

WARM target:
  P50 ~= 6-8 s
  P95 ~= 15-20 s

COLD:
  navigation-message-driven
  P95 target <= 45 s

REA position recovery:
  P50 ~= 0.8-1.2 s
  P95 <= 2 s
```
