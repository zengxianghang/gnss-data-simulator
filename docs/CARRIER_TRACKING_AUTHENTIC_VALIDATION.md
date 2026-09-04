# Authentic-NAV carrier-tracking validation

Issue: #163  
Parent: #159  
Result: **PASS**

This report records the permanent V1 validation evidence for the CN0-driven carrier-tracking layer introduced by #160-#162. The validation PR changes tests, CI reporting, and this document only. It does not change production carrier-tracking equations, runtime state transitions, urban propagation, navigation handling, or the frozen V1 defaults.

## Reproducible evidence

- Validation base: `3d2e2b562b35834805ee4e711403327d99bd00df` (`#162` merged into `integration/issues-103-104`).
- Validation implementation head used for the measured evidence: `30fc5f7943ad10461797eee16fb3d91a698ca221`.
- GitHub Actions run: CI #758, run id `33864233860`.
- Linux metrics job: `build-test (ubuntu-latest)`, job id `100995316472`.
- Windows full build/test: PASS.
- Ubuntu full build/test: PASS.
- Ubuntu `ctest`: **369/369 tests passed**.
- Focused suite: `CarrierTrackingAuthenticValidation.*`: **4/4 tests passed**.

The focused suite is part of the normal `ctest` executable on both Windows and Linux; the extra Linux CI step only prints the permanent numerical evidence.

## Authentic navigation provenance

The validation reuses the provenance-fixed authentic BRD400DLR RINEX 4.02 fixture already established by #124/#152.

- Source product/file: `BRD400DLR_S_20250030000_01D_MN.rnx`.
- Repository fixture: `tests/data/minimal/brd400dlr_rinex4_acceptance_nav.rnx`.
- Fixture SHA-256: `17c6bb00a8a0ef8f732b803925311e7b1ead658ae2e11f62635371eb915e9781`.
- Runtime scope: all BeiDou `EPH` records are copied mechanically into the temporary validation NAV.
- No ephemeris is interpolated, retargeted, synthesized, repaired, or selected to obtain a desired result.

The existing authentic urban E2E validation reports the mechanically filtered NAV identity as FNV-1a64 `2552251af489a9ec`.

## Static validation configuration

- Start: GPST week 2347, SOW 436500 s.
- Receiver truth: latitude 20 deg, longitude 120 deg, height 100 m.
- Duration: 12 s.
- Sampling rate: 10 Hz.
- Seed: `0x163`.
- Urban multipath: enabled.
- Generic measurement noise: disabled.
- Atmosphere: none.
- Measurement elevation mask: 0 deg.
- Solution elevation mask: 5 deg.
- Carrier tracking: enabled for the ON run; disabled for the compatibility control.

The ON case is repeated twice with identical configuration and seed. The OFF control is also repeated while changing dormant carrier-tracking bandwidth settings. These pairs verify deterministic output and disabled-feature compatibility rather than merely comparing summary statistics.

## Observation-level carrier-tracking results

The authentic static run produced:

- carrier result rows: **5892**
- Doppler-valid carrier rows: **5672**
- FLL rows with code+Doppler valid but ADR invalid: **184**
- carrier-unlocked rows with code valid but Doppler/ADR invalid: **23**
- FLL pull-in rows: **275**
- mode changes: **110**
- new carrier segments: **55**
- static-run cycle-slip events: **0**

The zero static-run cycle-slip count is expected: this short static case contains initial acquisition/mode progression but no later PLL-loss event. The dedicated REA case below verifies post-lock loss/reacquisition and ambiguity discontinuity.

### Statistics by carrier mode

| Mode | Rows | Doppler-valid | Error RMS (Hz) | P50 abs (Hz) | P95 abs (Hz) | P99 abs (Hz) | Max abs (Hz) | Error RMS (m/s) | P95 abs (m/s) | Max abs (m/s) | Theoretical sigma RMS (Hz) | Normalized error RMS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| CARRIER_UNLOCKED | 110 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| FLL_TRACK | 550 | 440 | 0.418945 | 0.213197 | 0.838282 | 1.44259 | 2.40042 | 0.091953 | 0.187786 | 0.460975 | 0.431158 | 0.967276 |
| PLL_TRACK | 5232 | 5232 | 0.203899 | 0.106467 | 0.428708 | 0.70254 | 1.11736 | 0.0454257 | 0.0975309 | 0.264069 | 0.199136 | 1.00514 |

The realized tracking-error RMS follows the theoretical jitter scale without any PVT-target tuning. The normalized RMS is about 0.97 for FLL and 1.01 for PLL. Because the tracking error is intentionally time-correlated, these numbers are evidence for the error scale, not a claim that successive samples are independent Gaussian draws.

### Statistics by effective C/N0 bin

| Effective C/N0 bin (dB-Hz) | Rows | Doppler-valid | Error RMS (Hz) | P95 abs (Hz) | Max abs (Hz) | Theoretical sigma RMS (Hz) | Normalized error RMS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| <18 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 18-22 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 22-27 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 27-30 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| >=30 | 5892 | 5672 | 0.227958 | 0.459245 | 2.40042 | 0.225831 | 1.00225 |

**Important limitation:** this authentic short static fixture/run organically exercises only effective C/N0 >=30 dB-Hz. The empty lower bins are reported as empty rather than populated with synthetic attenuation merely to satisfy a table. Low-C/N0 model response is checked separately by the frozen thermal-jitter test, while future calibration against measured weak-signal data remains required before claiming receiver-specific low-C/N0 fidelity.

## Theoretical thermal-jitter response

The frozen model responds in the physically expected directions:

- FLL sigma at 20 dB-Hz: **3.89848 Hz**.
- FLL sigma at 40 dB-Hz: **0.319105 Hz**.
- Reference 35 dB-Hz, 4 Hz FLL bandwidth, 20 ms integration: **0.570501 Hz**.
- Increasing FLL bandwidth to 8 Hz: **0.806811 Hz**.
- Increasing coherent integration to 40 ms: **0.284138 Hz**.
- At 0.19 m wavelength: sigma **0.108395 m/s**.
- Doubling wavelength to 0.38 m: sigma **0.216791 m/s** while sigma in Hz remains unchanged.

This confirms that C/N0, loop bandwidth, coherent integration time, and wavelength drive the model rather than a fixed empirical final Doppler sigma.

## Physical propagation remains separate from receiver tracking

For the 12 s #163 run, the physical urban environmental range-rate term was:

- RMS: **0.00016598 m/s**
- P95 absolute: **0.000427493 m/s**
- maximum absolute: **0.000447791 m/s**

The exact cross-file comparison between `carrier_tracking_truth.csv` and `urban_signal_truth.csv` produced:

- maximum environmental range-rate mismatch: **0 m/s**.

The carrier application identities also close to floating-point roundoff:

- max `D_post - D_physical - tracking_error_hz`: **1.13506e-13 Hz**.
- max `range_rate_post - range_rate_physical + tracking_error_mps`: **2.8387e-14 m/s**.

Therefore the physical urban path-rate remains an independent truth quantity; the much larger carrier-tracking error is a receiver tracking-layer effect and does not replace or mutate the physical propagation term.

For reference, the existing longer authentic #152 physical-only validation in the same CI reports `STATIC_DOPPLER_ALL rate_rms_mps=0.000165374` and `rate_max_mps=0.000466691`, consistent with the same physical mechanism over a longer interval.

## Independent validity states

Observed post-carrier validity combinations (`code,doppler,ADR`) included:

- `111`: **1952** rows
- `110`: **414** rows
- `100`: **69** rows

Specific carrier-state evidence includes 184 FLL rows with code+Doppler valid and ADR invalid, plus 23 carrier-unlocked rows where code remains valid while Doppler and ADR are invalid. This confirms that code, Doppler, and ADR validity are not collapsed into one shared flag.

## Worst carrier observation in the static run

The largest absolute tracking error was:

- GPST: week 2347, TOW `436502100000000` ns
- satellite: 134
- signal: BeiDou B1I
- effective C/N0: **35.3704 dB-Hz**
- carrier mode: `FLL_TRACK`
- FLL phase: `PULL_IN`
- theoretical sigma: **0.772633 Hz**
- realized tracking error: **2.40042 Hz / 0.460975 m/s**
- simultaneous physical environmental range rate: **0.000330717 m/s**
- urban state: `NLOS_TRACKED`
- blocking wall: `WEST`
- received paths: 2
- reflections: 1

This context is diagnostic evidence only; the model was not adjusted to reduce this worst case.

## Output-rate sanity

Matching PLL observations at identical GPST/satellite/signal epochs between 1 Hz and 10 Hz runs produced:

- matched PLL observations: **437**
- maximum theoretical sigma mismatch: **0 Hz**
- 1 Hz tracking-error RMS: **0.208418 Hz**
- 10 Hz tracking-error RMS: **0.222627 Hz**
- RMS ratio: **0.936175**

The exact theoretical sigma at matched epochs confirms that changing output rate does not change the frozen loop-noise scale. The realized correlated sequence need not be sample-by-sample identical across output rates, so the comparison is intentionally a scale sanity check rather than an equality requirement.

## Deterministic loss and reacquisition

The authentic-NAV REA case uses 10 Hz output, a 6 s signal-on interval followed by 2 s signal-off, and a 14 s total duration. It produced:

- code-not-tracking/reset rows: **11171**
- reacquisition carrier-result rows: **3037**
- reacquisition carrier-unlocked code-only rows: **23**
- reacquisition FLL code+Doppler/no-ADR rows: **184**
- new carrier segments: **55**

One complete deterministic signal sequence (`satellite:signal = 111:16`) was:

| Event | Elapsed time (s) |
| --- | ---: |
| first carrier result after code reacquisition | 8.8 |
| first FLL | 9.0 |
| first Doppler-valid | 9.2 |
| first PLL | 10.0 |
| first ADR-valid | 11.0 |

The first FLL uses the frozen **8 Hz pull-in bandwidth**. The sequence shows the intended 0.2 s FLL qualification, 0.2 s Doppler-valid delay, PLL confirmation, and 1.0 s additional ADR-valid delay. The first post-reacquisition ADR differs from the previous segment by **261169 cycles**, proving that ADR continuity is not carried across the loss/reacquisition boundary.

## Secondary RTKLIB velocity and clock-drift result

RTKLIB velocity is a secondary emergent check, not the target used to choose tracking parameters.

| Carrier tracking | Valid velocity epochs | 3D RMS (m/s) | 3D P95 (m/s) | 3D max (m/s) | Clock-drift RMS (m/s) | Clock-drift max (m/s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| ON | 105 | 0.217632 | 0.339921 | 1.51026 | 0.175136 | 1.39511 |
| OFF | 107 | 0.00578352 | 0.00716576 | 0.0244718 | 0.00425518 | 0.0186139 |

The degradation appears naturally when observation-level carrier tracking is enabled. No minimum velocity degradation was required and no parameter was tuned to manufacture this difference.

## Determinism and compatibility

The validation verifies all of the following:

1. Same authentic NAV + config + seed with carrier tracking enabled produces byte-identical receiver output and byte-identical carrier truth.
2. With carrier tracking disabled, changing dormant carrier-tracking bandwidth settings leaves receiver output and physical truth byte-identical.
3. `observation_truth.csv` and `urban_signal_truth.csv` are identical between carrier ON and OFF for the same physical simulation, proving the carrier layer does not mutate physical truth.
4. Simulator-only carrier columns such as `carrier_mode` and `tracking_error_hz` do not leak into normal RANGE output.
5. Physical environmental range-rate truth is exactly preserved and independently traceable.

## V1 limitations and calibration plan

This validation establishes architecture, determinism, state/validity semantics, sign/units, physical separation, and physically directed jitter scaling. It does **not** establish vendor-specific receiver fidelity.

Known V1 limitations include:

- the authentic static fixture used here contains no effective-C/N0 samples below 30 dB-Hz;
- the frozen loop bandwidths, thresholds, persistence values, and integration time remain engineering assumptions rather than measurements of a specific receiver;
- no full baseband correlator/NCO/oscillator implementation is simulated;
- no common-mode receiver oscillator process is introduced beyond the existing receiver clock behavior;
- this report is static/REA focused and does not calibrate moving-receiver dynamics or vendor-specific loop adaptation;
- RTKLIB PVT is secondary and must not become a tuning target.

A future calibration dataset should include measured low-C/N0 static and moving observations with trustworthy receiver status/lock information. Calibration should compare CN0-conditioned Doppler error scale, FLL/PLL occupancy, loss/reacquisition timing, and ADR continuity. Frozen model parameters should be changed only when those observation-level measurements justify the change, never to force a desired final velocity metric.

## Reproduction commands

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
./build/gnss_sim_integration_tests --gtest_filter=CarrierTrackingAuthenticValidation.*
```

On Windows, use the corresponding Release executable path produced by CMake/Visual Studio; the focused tests are already included in normal `ctest` discovery.
