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

## Summary

```text
elevation_mask_deg = 3.0
output_eph         = true
output_ion         = true
multipath_enabled  = false
duration_sec       = 28800  # 8 h
```
