# Urban signal tracking and propagation-state semantics

Issue #121 extends the existing per-signal `SignalTracker`; it does not create a second acquisition/tracking state machine.

## Two independent state axes

`SignalTrackingPhase` continues to describe receiver progress:

- `SIGNAL_OFF`
- `SEARCHING`
- `ACQUIRING`
- `TRACKING`

`UrbanSignalState` describes the propagation/tracking interpretation:

- `LOS`
- `LOS_MULTIPATH`
- `NLOS_TRACKED`
- `BLOCKED`

An open-sky signal may therefore be `SEARCHING + LOS`. `BLOCKED` is not used as a synonym for startup delay.

## V1 engineering defaults

These values are deterministic receiver-model assumptions. They are configurable in `SignalTrackingModelConfig`; they are not measurements of a named commercial receiver and must not be tuned to force a desired PVT result.

| Parameter | Default |
| --- | ---: |
| minimum tracking C/N0 | 10.0 dB-Hz |
| acquisition/reacquisition C/N0 | 18.0 dB-Hz |
| acquisition C/N0 persistence | 200 ms |
| tracking-loss C/N0 persistence | 500 ms |
| abrupt DLL root jump | > 0.25 chip/update |
| LOS multipath C/N0 significance | >= 0.5 dB |
| LOS multipath DLL bias significance | >= 0.02 chip |

The 10/18 dB-Hz pair provides explicit acquisition/tracking hysteresis. Threshold persistence prevents one-epoch chatter.

## Acquisition and reacquisition

The existing seeded acquisition/reacquisition delay distributions remain authoritative. The urban-aware update adds a physical eligibility gate around that schedule:

1. Search must already be ready.
2. Effective C/N0 from the common #120 result must remain at or above 18 dB-Hz continuously for 200 ms.
3. The existing sampled acquisition/reacquisition completion time must have elapsed.
4. A stable #119 DLL root must exist.

Only after all four conditions are satisfied does the tracker enter a fresh lock. PSR, Doppler, ADR, and carrier-continuity validity delays restart from the actual new-lock epoch.

After a physical loss, `reacquisition_pending=true` and `scheduled=false`. The existing `schedule_signal_acquisition(..., kReacquisition, ...)` function is then used to sample/schedule the deterministic reacquisition delay; the urban state machine does not invent a second delay distribution.

## Tracking loss

While tracking:

- effective C/N0 >= 10 dB-Hz immediately resets the low-C/N0 loss timer;
- effective C/N0 < 10 dB-Hz starts/continues the timer;
- a low-C/N0 interval shorter than 500 ms preserves lock;
- at 500 ms the lock is lost deterministically;
- disappearance of every stable DLL root causes immediate loss;
- a selected root jump greater than 0.25 chip relative to the prior accepted root causes immediate loss.

Loss clears observation validity, lock time, ADR/carrier continuity, and DLL continuity. The deterministic reason is retained as one of `LOW_CN0`, `NO_STABLE_DLL_ROOT`, `ABRUPT_DLL_ROOT_SWITCH`, or `SIGNAL_UNAVAILABLE`.

## DLL continuity

The state machine consumes #119 roots directly:

- acquisition/reacquisition uses #119 `ACQUISITION` selection;
- tracking uses #119 `TRACKED` selection with the previous selected code phase;
- root-jump comparison uses `CodeTrackingDllRoot::code_phase_chips`, which is already derived from signal-specific code/correlation metadata.

No generic chip-rate table is introduced in #121.

## LOS classification

Direct geometry comes from #115. For direct LOS, the label is based on the common #119/#120 receiver-domain result rather than a Fresnel-v cutoff:

- `LOS_MULTIPATH` if `abs(CN0_effective - CN0_open) >= 0.5 dB`, or
- `LOS_MULTIPATH` if `abs(selected DLL code bias) >= 0.02 chip`, or
- otherwise `LOS`.

A retained #117 reflection is neither required nor sufficient as a special-case state rule. This preserves #130: clear-side rooftop diffraction can make `LOS_MULTIPATH` reachable without weakening the frozen four-wall occlusion geometry.

When direct geometry is blocked, the signal is `NLOS_TRACKED` only while the receiver actually retains a tracking lock. Otherwise it is `BLOCKED`.

## Scope

#121 owns receiver state, hysteresis, lock continuity, and classification. It does not synthesize final pseudorange/Doppler/carrier/ADR (#122), serialize truth (#123), modify authentic NAV/EPH/ION, add random urban attenuation, or tune final PVT errors.
