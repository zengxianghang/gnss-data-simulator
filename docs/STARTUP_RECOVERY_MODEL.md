# Startup and Reacquisition Timing Model

Status: frozen V1 default model. All timing parameters are defaults and must remain configurable.

## 1. Purpose

The simulator must not generate TTFF as a single fixed random number. HOT, WARM, COLD, and REA timing must emerge from receiver startup state, per-signal acquisition/reacquisition, navigation-data availability, and the actual RTKLIB position/velocity solution conditions.

Vendor TTFF terminology is not fully consistent across products. Therefore this project uses its own startup-state definitions from `DESIGN_SPEC.md`; vendor specifications are used only to calibrate realistic timing distributions.

## 2. Public receiver benchmarks used for calibration

Representative current/high-end multi-frequency GNSS receivers report approximately:

| Receiver | Cold | Warm | Hot | Reacquisition |
| --- | ---: | ---: | ---: | ---: |
| u-blox ZED-F9P-15B | about 25-30 s depending on constellation configuration | not used here | about 2 s | product dependent |
| Unicore UM980/UM980C | <12 s | not normally specified | <4 s | product dependent |
| NovAtel OEM729 | <34 s typical | not separately specified | <20 s typical | L1 <0.5 s; L2/L5 <1.0 s typical |
| Trimble BD99x family | <45 s | <30 s | terminology differs | <2 s |
| Septentrio mosaic-X5 | <45 s | <20 s | terminology differs | about 1 s class |

These values must not be interpreted as one universal definition of HOT/WARM/COLD. In particular, some vendors define warm start without valid ephemeris while others define warm start with ephemeris retained.

Primary reference material:

- u-blox ZED-F9P-15B data sheet: https://content.u-blox.com/sites/default/files/documents/ZED-F9P-15B_DataSheet_UBX-23009090.pdf
- Unicore UM980 product page/manual: https://en.unicore.com/products/um980/
- NovAtel OEM729 performance specifications: https://docs.novatel.com/OEM7/Content/Technical_Specs_Receiver/OEM729_Performance_Specs.htm
- NovAtel OEM7 hot-start application note: https://hexagondownloads.blob.core.windows.net/public/Novatel/assets/Documents/Papers/APN-107-HotStart-on-OEM7/APN-107-HotStart-on-OEM7.pdf
- Trimble BD99x performance specifications: http://receiverhelp.trimble.com/oem-gnss/specs-performance-bd940-bd99x.html
- Septentrio mosaic-X5 data sheet: https://media.digikey.com/pdf/Data%20Sheets/Septentrio%20PDFs/Mosaic-X5_Datasheet.pdf

## 3. Simulator startup definitions

The simulator definitions remain authoritative.

### HOT

- Receiver NAV EPH/ION restored from Flash immediately at power-on.
- Approximate time is accurate.
- Approximate position is accurate.
- Tracking, lock time, and ADR state are reset.
- EPH availability does not limit the first solution.
- TTFF is dominated by boot/startup latency, signal acquisition, measurement-valid latency, and solution latency.

### WARM

- Receiver NAV EPH/ION restored from Flash immediately at power-on, same as HOT.
- Approximate time/position priors are less accurate than HOT.
- Tracking, lock time, and ADR state are reset.
- Wider acquisition search uncertainty makes acquisition slower than HOT.
- EPH availability does not limit the first solution.

### COLD

- Receiver NAV starts without usable EPH.
- Tracking, lock time, and ADR state are reset.
- Signals are acquired and navigation fragments are collected progressively.
- EPH becomes available only after the required real navigation-message fragments are collected and their issue-of-data checks pass.
- The first position occurs only after enough healthy observations have both valid measurements and usable Receiver NAV ephemerides.

### REA

- Receiver remains powered.
- Receiver NAV, time, position prior, and clock state are retained.
- Signal tracking is lost during SIGNAL OFF.
- Recovery uses a dedicated reacquisition model and must be faster than a new HOT startup.

## 4. Common timing decomposition

For HOT/WARM, model TTFF as an emergent result:

```text
power-on
  -> common receiver startup delay
  -> per-signal acquisition
  -> PSR-valid measurements become available
  -> enough healthy/visible signals are usable
  -> RTKLIB SPP succeeds
  -> PSRPOSA = SOL_COMPUTED / SINGLE
```

Do not directly assign `ttff_seconds` and then force a fix at that time.

Conceptually:

```text
signal_usable_time = startup_common_delay
                   + signal_acquisition_delay(CN0, elevation, signal)
                   + measurement_valid_delay
                   + small deterministic jitter
```

For WARM, add a receiver-level search uncertainty term:

```text
signal_usable_time += warm_search_uncertainty_delay
```

All stochastic terms must use the simulator's deterministic seeded PRNG.

## 5. HOT default calibration

Target open-sky behavior:

```text
TTFF P50        ~= 2 s
TTFF P95        ~= 4 s
nominal target  <= 5 s
```

Recommended common startup-delay calibration:

```text
startup_common_delay P50 ~= 0.8 s
startup_common_delay P95 ~= 1.5 s
```

Recommended per-signal acquisition delay bands before additional small jitter:

```text
CN0 >= 40 dB-Hz : about 0.2-0.6 s
35-40 dB-Hz     : about 0.3-1.0 s
30-35 dB-Hz     : about 0.6-2.0 s
<30 dB-Hz       : about 1.0-4.0 s
```

These are model-calibration defaults, not protocol constants. They must be configurable and may later be replaced by statistics learned from target receiver data.

## 6. WARM default calibration

WARM uses the same cached EPH/ION behavior as HOT but a wider acquisition search caused by less accurate time/position priors.

Default target open-sky behavior:

```text
TTFF P50        ~= 6-8 s
TTFF P95        ~= 15-20 s
nominal target  <= 30 s
```

Recommended warm-search uncertainty calibration:

```text
warm_search_uncertainty_delay P50 ~= 4-5 s
warm_search_uncertainty_delay P95 ~= 12-15 s
```

The final first-fix time is still determined by actual signal acquisition and RTKLIB solution availability, not by directly sampling a WARM TTFF value.

## 7. COLD default calibration

COLD must remain navigation-message-driven. Do not add an artificial fixed delay merely to match a desired TTFF number.

The core rule remains:

```text
EPH availability time
  = signal acquisition time
  + time to collect the required NAV fragments from the actual message phase
```

Then:

```text
first fix
  = first epoch where RTKLIB has enough valid observations
    with corresponding usable Receiver NAV EPH
```

Validation targets for an open-sky multi-GNSS dataset:

```text
typical/P50 region : about 20-30 s
P95 target         : <= 45 s
```

These are validation envelopes, not forced timing. Modern multi-constellation conditions are explicitly allowed to produce approximately 10-20 s cold starts when the navigation-message timing and available constellations permit it. This is important because current receivers such as UM980 advertise cold-start performance below 12 s under favorable test conditions.

Cold-start behavior must therefore be validated in two ways:

1. NAV-message collection logic is protocol-consistent.
2. Aggregate TTFF statistics remain plausible relative to current receiver benchmarks.

Protocol correctness takes precedence over artificially matching a target P50.

## 8. REA default reacquisition calibration

REA is not modeled as HOT startup because the receiver remains powered and retains NAV/time/position state.

Default open-sky recovery targets:

```text
first strong-signal reacquisition : typically <0.5 s
majority strong-signal recovery   : about 0.5-1.5 s
position recovery P50             : about 0.8-1.2 s
position recovery P95             : <= 2 s
```

Individual low-CN0 or low-elevation signals may recover later.

The same per-signal state machine controls:

- CN0 recovery
- pseudorange validity
- Doppler validity
- ADR validity
- lock-time restart

The simulator should therefore naturally produce progressive RANGE observation-count recovery after SIGNAL ON.

## 9. Distribution requirements

Do not use a simple uniform distribution such as:

```text
Random(0, 4 s)
```

for acquisition or TTFF.

Use a bounded/truncated distribution or deterministic empirical distribution whose parameters depend on startup mode and signal quality. The exact mathematical distribution is an implementation detail, but it must satisfy:

- deterministic repeatability from seed;
- no negative delay;
- bounded pathological tails in the default model;
- monotonic tendency for lower CN0 to produce slower acquisition;
- aggregate HOT/WARM/REA statistics close to the target ranges above.

## 10. Solution-status behavior

Until a valid solution exists:

```text
PSRPOSA = INSUFFICIENT_OBS / NONE
PSRVELA = INSUFFICIENT_OBS / NONE
```

Position and velocity validity must be evaluated independently.

After RTKLIB succeeds:

```text
PSRPOSA = SOL_COMPUTED / SINGLE
PSRVELA = SOL_COMPUTED / corresponding valid velocity type
```

If enough observations or EPH are lost again, the output returns to the invalid state.

## 11. Configuration intent

The final configuration schema may choose different names, but V1 should expose equivalent configurable parameters for at least:

```text
startup.hot.common_delay_model
startup.hot.signal_acquisition_model
startup.warm.common_delay_model
startup.warm.search_uncertainty_model
startup.warm.signal_acquisition_model
startup.cold.nav_message_driven = true
rea.reacquisition_model
```

The benchmark target percentiles must also be represented in regression tests, not hidden only in implementation constants.

## 12. Acceptance intent

A sufficiently large deterministic simulation set should verify that:

```text
HOT  : P50 near 2 s, P95 near 4 s, nominal <= 5 s
WARM : P50 about 6-8 s, P95 about 15-20 s, nominal <= 30 s
COLD : message-driven; typical about 20-30 s, P95 <= 45 s without forcing minimum TTFF
REA  : position recovery P50 about 0.8-1.2 s, P95 <= 2 s
```

These targets apply to the default open-sky profile. Different CN0 models, elevation masks, satellite geometry, receiver profiles, or explicit fault scenarios are allowed to produce materially different distributions.