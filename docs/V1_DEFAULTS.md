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

## Multipath

- **V1 does not add a multipath error model.**
- The pseudorange/phase/Doppler architecture should keep multipath as a separate future extension point, but the V1 default and V1 required implementation use zero multipath contribution.
- Observation realism in V1 comes from real satellite geometry/navigation data, signal-specific code-bias corrections, atmosphere/noise models where enabled, CN0 modeling, and tracking-state behavior.

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

Recommended HOT per-signal acquisition-delay calibration before small deterministic jitter:

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

All stochastic timing behavior must use the deterministic seeded PRNG and remain configurable.

## Summary

```text
elevation_mask_deg = 3.0
output_eph         = true
output_ion         = true
multipath_enabled  = false
duration_sec       = 28800  # 8 h

ttff.default_mode  = HOT

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
