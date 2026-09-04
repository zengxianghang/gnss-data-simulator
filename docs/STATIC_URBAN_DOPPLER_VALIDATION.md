# Static-receiver urban multipath Doppler validation

Issue: #152

## Verdict

PASS for the frozen **static receiver + static four-wall scene** within the same authentic BeiDou correlation-supported scope used by #124.

The urban model does apply building-induced propagation changes to Doppler. In this static case the resulting per-observation environmental range-rate is physically small because it is the **time derivative** of the composite carrier-path bias, not the absolute multipath path excess. The largest validated environmental range-rate is below 0.5 mm/s even though the same worst observation carries a DLL code bias of about 17.6 m.

The validation does not add an independent Doppler-noise generator, does not modify NAV/EPH/ION, and does not tune scene/tracking parameters or velocity output to obtain a desired result.

## Authentic navigation provenance

This validation reuses the #124 provenance-fixed RINEX 4.02 BRD400DLR fixture:

- source product: `BRD400DLR`;
- source file: `BRD400DLR_S_20250030000_01D_MN.rnx`;
- repository fixture: `tests/data/minimal/brd400dlr_rinex4_acceptance_nav.rnx`;
- fixture SHA256: `17c6bb00a8a0ef8f732b803925311e7b1ead658ae2e11f62635371eb915e9781`.

As in #124, the harness mechanically copies **all BeiDou EPH records** from the fixture. It does not select satellites by desired geometry or result and does not alter, interpolate, retarget, synthesize, or repair any ephemeris field.

The BeiDou-only scope is retained because all five frozen BeiDou V1 code-correlation profiles used by the urban DLL are explicitly modeled. Unsupported GPS/QZSS correlation profiles remain explicit fail-fast limitations rather than being silently approximated.

## Validation configuration

Primary 1 Hz comparison:

- start: GPST week 2347, SOW 436500 s;
- duration: 60 s;
- receiver: latitude 20 deg, longitude 120 deg, height 100 m;
- scenario: KS / static receiver;
- seed: `0x152`;
- atmosphere: NONE;
- measurement noise: disabled;
- measurement elevation mask: 0 deg;
- solution elevation mask: 5 deg;
- urban ON run and otherwise identical urban OFF run.

Sampling-rate check:

- same NAV, receiver, scene, time interval and deterministic configuration;
- urban ON at 1 Hz;
- urban ON at 10 Hz;
- compare 1 Hz `environmental_range_rate_mps` with the 10 Hz carrier-range-bias difference over the same one-second interval.

## Production Doppler mapping check

For continuous-lock observations the production mapping is:

`range_rate_urban = range_rate_clean + environmental_range_rate_mps`

`Doppler_urban = Doppler_clean - environmental_range_rate_mps / wavelength`

The validator matches urban ON and OFF observations by GPST + satellite + signal and compares the emitted values directly; it does not recompute propagation physics.

Reference CI #731 results:

- matched valid ON/OFF observations: **3134**;
- maximum Doppler mapping mismatch: **1.13202e-13 Hz**;
- maximum range-rate mapping mismatch: **2.84213e-14 m/s**;
- BLOCKED rows with valid Doppler: **0**.

This is numerical-precision agreement and directly confirms that the existing urban environmental range-rate is present in the emitted Doppler/range-rate observation.

## Per-observation urban Doppler statistics

Absolute `environmental_range_rate_mps` statistics:

| State | Count | RMS m/s | P50 m/s | P95 m/s | P99 m/s | Max m/s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| all valid | 3134 | 1.65374e-4 | 8.8978e-6 | 4.30847e-4 | 4.49272e-4 | **4.66691e-4** |
| LOS | 1254 | 9.09393e-6 | 7.28645e-6 | 1.75309e-5 | 1.96464e-5 | 2.08733e-5 |
| LOS_MULTIPATH | 741 | 1.21445e-5 | 8.72932e-6 | 2.26146e-5 | 2.27795e-5 | 2.29253e-5 |
| NLOS_TRACKED | 1139 | 2.73978e-4 | 3.01806e-4 | 4.47982e-4 | 4.54880e-4 | **4.66691e-4** |

Equivalent absolute Doppler perturbation statistics:

| State | RMS Hz | P50 Hz | P95 Hz | P99 Hz | Max Hz |
| --- | ---: | ---: | ---: | ---: | ---: |
| all valid | 7.54296e-4 | 4.29508e-5 | 1.82371e-3 | 2.33962e-3 | **2.45248e-3** |
| LOS | 4.18701e-5 | 3.16769e-5 | 8.22620e-5 | 9.19322e-5 | 9.24632e-5 |
| LOS_MULTIPATH | 5.06682e-5 | 4.37949e-5 | 9.53991e-5 | 9.58586e-5 | 9.58776e-5 |
| NLOS_TRACKED | 1.24977e-3 | 1.24714e-3 | 2.24387e-3 | 2.39041e-3 | **2.45248e-3** |

The dominant static Doppler effect is therefore the tracked NLOS population. LOS and roof-edge LOS_MULTIPATH remain much smaller in this frozen geometry.

## Worst single observation

Reference metric run worst valid observation:

- GPST: `2347:436545000000000`;
- satellite number: **116**;
- signal: **BeiDou B1C**;
- state: **NLOS_TRACKED**;
- azimuth: **180.711 deg**;
- elevation: **27.8682 deg**;
- blocking wall: **SOUTH**;
- received paths: **2**;
- retained reflections: **1**;
- open-sky CN0: **37.556 dB-Hz**;
- effective CN0: **37.2537 dB-Hz**;
- DLL code bias: **17.5942 m**;
- environmental range-rate: **-0.000466691 m/s**;
- equivalent Doppler perturbation: **+0.00245248 Hz**.

This is the key interpretation for the original question: a roughly **17.6 m code bias coexists with only 0.467 mm/s path-rate bias**. A large static multipath path excess does not imply a comparably large Doppler error when that path excess changes slowly.

## Invalid Doppler / continuity handling

In the 60 s KS run:

- rows with invalid environmental range-rate: **4066**;
- forced reacquisition rows: **0**;
- cycle-slip rows: **0**;
- BLOCKED rows leaking valid Doppler: **0**.

The KS interval itself does not force a loss/reacquisition event. Existing #124 authentic-NAV REA evidence separately exercises 55 reacquisitions / 55 carrier reset events and confirms that broken carrier-continuity segments are not differentiated across the gap.

## RTKLIB velocity and receiver clock drift

Urban ON:

| Metric | Result |
| --- | ---: |
| valid velocity epochs | 57 |
| 3-D velocity RMS | 0.00495729 m/s |
| 3-D velocity P95 | 0.00500283 m/s |
| maximum 3-D velocity error | **0.00501839 m/s** |
| horizontal RMS | 0.000799979 m/s |
| horizontal P95 | 0.000839593 m/s |
| horizontal maximum | 0.000854336 m/s |
| vertical RMS | 0.00489232 m/s |
| vertical P95 | 0.00493387 m/s |
| vertical maximum | 0.00494861 m/s |
| clock-drift RMS error | 0.00358198 m/s |
| clock-drift P95 error | 0.00362785 m/s |
| clock-drift maximum error | 0.00364146 m/s |

Worst velocity epoch:

- GPST `2347:436527000000000`;
- velocity used satellites: **11**;
- urban composition at the epoch: LOS 22 / LOS_MULTIPATH 13 / NLOS_TRACKED 20 / BLOCKED 65.

Urban OFF baseline:

- valid velocity epochs: **58**;
- 3-D velocity RMS: **2.65488e-5 m/s**;
- P95: **4.41106e-5 m/s**;
- maximum: **5.70979e-5 m/s**.

Thus the building model does measurably degrade the static RTKLIB velocity solution, from tens of micrometres per second in the clean baseline to roughly 5 mm/s. The final velocity error is larger than any one environmental range-rate bias because the multi-satellite velocity/clock-drift solve redistributes observation errors through geometry and clock-drift estimation; the largest component here is vertical, with a material clock-drift error as well.

No minimum velocity degradation is an acceptance target.

## 1 Hz versus 10 Hz sampling-rate validation

The 10 Hz run provides an independent reconstruction of the carrier-range change over each one-second interval. The initial CI #731 comparison produced:

- matched continuous-lock one-second intervals: **3134**;
- RMS 1 Hz vs reconstructed 10 Hz rate mismatch: **3.2025e-22 m/s**;
- maximum mismatch: **1.35525e-20 m/s**;
- one-second environmental phase change magnitude: approximately **0.0154094 rad**;
- pi: approximately **3.14159 rad**.

The validator has subsequently been tightened so the phase-change bound itself is computed from the **10 Hz reconstructed one-second carrier-range difference**, not from the 1 Hz `std::remainder` result. This prevents the alias check from being self-referential. The final PR CI must pass this independent version before merge.

The observed phase motion is over two orders of magnitude below pi, so the current frozen static scene has substantial margin against one-second phase-rate aliasing.

## Interpretation

The result explains why #124's static velocity error was small:

1. Building multipath **is** present in Doppler.
2. Pseudorange/code bias depends on the path excess itself; Doppler depends on the **time derivative** of the tracked composite carrier path.
3. With a static receiver and static 10 m-scale walls, only the slowly changing satellite direction drives that derivative.
4. Consequently, NLOS code bias may be tens of metres while environmental range-rate remains sub-mm/s.
5. Loss/BLOCKED periods are not converted into fabricated large Doppler spikes; invalid continuity is excluded from the velocity solve.
6. RTKLIB velocity geometry and receiver clock-drift estimation transform the small per-observation perturbations into a roughly 5 mm/s final 3-D error in this particular case.

## Explicit limitations

- This report validates **only a static receiver and static reflectors**.
- Moving receiver / vehicle urban Doppler behavior is intentionally out of scope for #152 and must be validated separately.
- The validation uses the correlation-supported BeiDou scope for the same reason as #124; it does not claim unsupported GPS/QZSS urban DLL profiles are implemented.
- Atmosphere and stochastic measurement noise are disabled to isolate the urban propagation/tracking contribution.
- These magnitudes are evidence from one frozen authentic-NAV interval and scene, not universal urban-environment error bounds.
- No measured moving or static urban receiver data set is used here for statistical calibration.

## Acceptance conclusion

For the frozen static scene, the production chain

`authentic NAV -> urban complex field -> tracked carrier phase -> environmental range-rate -> Doppler/range-rate -> RTKLIB velocity/clock drift`

is internally consistent and produces a real but small Doppler degradation. The small magnitude is explained by the slow variation of static-building path excess, not by omission of building multipath from Doppler.