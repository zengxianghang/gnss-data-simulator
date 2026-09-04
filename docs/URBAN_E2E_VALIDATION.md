# Authentic-NAV urban end-to-end validation

Issue: #124

## Verdict

PASS for the V1 urban chain within the signal-correlation scope that is explicitly modeled by the repository.

The end-to-end acceptance run uses authentic BRD400DLR broadcast navigation data, production urban propagation/tracking/measurement code, production solution code, serialized NovAtel RANGEA, and pinned RTKLIB. The run reaches all four frozen urban states and the resulting position/velocity degradation is produced by the observation model rather than by editing navigation data or forcing a target PVT error.

This verdict is intentionally scoped. A full mixed-GNSS BRD400DLR urban run is not yet a valid acceptance path because some frozen GPS/QZSS signal definitions deliberately have unsupported code-correlation profiles. The urban DLL fails fast for those profiles rather than silently substituting an unverified BPSK model. That limitation is preserved by this validation.

## Authentic navigation provenance

Source product: `BRD400DLR`

Source file: `BRD400DLR_S_20250030000_01D_MN.rnx`

Source URL recorded by the fixture metadata:

`ftp://igs.gnsswhu.cn/pub/gps/data/daily/2025/brdc/BRD400DLR_S_20250030000_01D_MN.rnx.gz`

Source compressed SHA256: `fb84d4046b06e905e8e4ec0efb82f0e9ad044bc44d664cc0209cb9b4c92b9512`

Source uncompressed SHA256: `b11c638eea42978b8bd6aa8b65a5099fe6556dfe527bc037ed481d2b239afc42`

Repository fixture: `tests/data/minimal/brd400dlr_rinex4_acceptance_nav.rnx`

Fixture SHA256: `17c6bb00a8a0ef8f732b803925311e7b1ead658ae2e11f62635371eb915e9781`

The fixture is RINEX 4.02 and its provenance is frozen in `brd400dlr_rinex4_acceptance_nav.meta.json`. `.gitattributes` pins this provenance-fixed fixture to LF line endings so that its working-tree bytes do not vary between Linux and Windows checkouts.

## Validation NAV subset

The #124 harness mechanically copies every BeiDou `EPH` record from the authentic BRD400DLR fixture. It does not select satellites based on desired visibility, multipath state, or PVT outcome, and it does not alter, interpolate, retarget, synthesize, or repair any ephemeris field.

The generated subset therefore means:

- all BeiDou EPH records in the frozen authentic fixture;
- original record field values preserved;
- canonical LF output from the harness;
- at least four EPH records and at least four distinct BeiDou satellites are required, otherwise the test fails;
- no generated ION data is used by this test (`atmosphere_mode=NONE`).

The BeiDou-only scope is selected because the five frozen BeiDou V1 signal correlations required by the urban code path are explicitly modeled. It is not selected to manufacture a particular satellite geometry or error magnitude.

Reference filtered-NAV FNV-1a-64 from CI: `2552251af489a9ec`.

## Validation identity and commands

Validated PR head: `f87e8e18769d720208f3f2fd2d5e7728ccb0f51a`.

Authoritative metric and cross-platform validation run: GitHub Actions CI #729 (`33832212284`). The run completed successfully on Linux and Windows; both platforms passed all 337 tests. The Linux focused E2E step reproduced the metrics recorded below.

Commands represented by the CI validation are:

```text
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
./build/gnss_sim_integration_tests --gtest_filter=UrbanE2EValidation.*
```

The commit that adds this evidence paragraph is documentation-only; the validated implementation/test head above is the exact code revision exercised by CI #729.

## Frozen acceptance configuration

- start: GPST week 2347, SOW 436500 s;
- duration: 180 s;
- sampling rate: 1 Hz;
- static receiver: latitude 20 deg, longitude 120 deg, height 100 m;
- seed: `0x124`;
- measurement noise: disabled;
- urban multipath: enabled;
- atmosphere: NONE;
- measurement elevation mask: 0 deg;
- solution elevation mask: 5 deg;
- EPH output: enabled;
- ION output: disabled.

The test runs the same NAV/config/seed twice and requires byte-identical receiver and truth outputs.

## Urban-state and path coverage

CI #729 produced these signal-state counts:

| State | Count |
| --- | ---: |
| LOS | 3762 |
| LOS_MULTIPATH | 2538 |
| NLOS_TRACKED | 3559 |
| BLOCKED | 11741 |
| BLOCKED with propagation evaluated | 1841 |

Path diagnostics:

| Diagnostic | Count |
| --- | ---: |
| DIRECT_ROOF | 11700 |
| first-order REFLECTION | 3600 |
| direct LOS with retained reflection | 0 |

The zero direct-LOS/reflection overlap is consistent with the #130 frozen-scene reachability result. `LOS_MULTIPATH` in this closed four-wall scene is reached through the physically continuous roof-edge-affected direct field; the diffraction transfer function is not added as an independent second copy of the direct signal. The test therefore does not create direct-plus-full-diffraction double counting merely to make `LOS_MULTIPATH` reachable.

The run also checks that evaluated `BLOCKED` rows do not leak receiver observations and that `NLOS_TRACKED` is supported by indirect propagation rather than a fabricated direct path.

## Position and velocity results

Production solution path, CI #729:

| Metric | Result |
| --- | ---: |
| valid position epochs | 177 |
| maximum horizontal error | 2.29263 m |
| maximum vertical error | 39.592 m |
| maximum 3D position error | 39.6384 m |
| valid velocity epochs | 177 |
| maximum velocity error | 0.00535238 m/s |

The maximum-position-error epoch is:

`GPST 2347:436679000000000 ns`

Urban states at that epoch:

- LOS: 21
- LOS_MULTIPATH: 14
- NLOS_TRACKED: 20
- BLOCKED: 65

These error values are observations from the fixed authentic-NAV physical simulation. **They are not acceptance targets, tuning objectives, injected offsets, or minimum/maximum requirements.** No wall parameter, threshold, ephemeris value, observation value, or receiver solution was adjusted to obtain the approximately 39.6 m 3D error.

## Independent RTKLIB consumption

The same serialized RANGEA was independently consumed by the pinned RTKLIB path in two ways:

| Navigation used by validator | Valid position epochs | Maximum 3D error |
| --- | ---: | ---: |
| authentic filtered RINEX NAV | 177 | 39.6387 m |
| navigation serialized into receiver log and reconstructed | 177 | 39.6387 m |

This establishes that the degradation is present in the serialized observations and remains explainable when the observations are positioned outside the simulator's maintained in-memory solution state.

The repository's existing non-urban RANGEA round-trip regressions also remain green; #124 does not replace or weaken them.

## Loss, reacquisition, and ADR reset

A separate authentic-NAV REA run is used to exercise signal loss and reacquisition deterministically instead of waiting for incidental satellite motion.

CI #729 produced:

- reacquisition events: 55
- cycle-slip/reset events: 55
- signal-off epochs: 10
- signal-on epochs: 40

The test requires actual `SIGNAL_OFF`/`SIGNAL_ON` events and verifies that lock/carrier continuity is restarted after reacquisition.

The primary 180 s KS run has zero forced reacquisition and zero forced cycle-slip events; those transitions are not injected into the main position-error result.

## Determinism and traceability

For fixed authentic NAV, configuration, receiver position, and seed, the harness requires repeated runs to produce byte-identical:

- `simulated.log`
- `scenario.json`
- `observation_truth.csv`
- `solution_truth.csv`
- `urban_signal_truth.csv`
- `urban_path_truth.csv`
- `run_manifest.json`
- mechanically filtered BeiDou NAV subset

The state/path assertions are derived from the same truth outputs written by the production synthesis path. Position and velocity metrics are calculated from `solution_truth.csv`, while an independent serialized RANGEA validation is performed through pinned RTKLIB.

## Explicit limitations

1. The #124 PASS is not a claim that every frozen V1 signal has an authoritative urban DLL correlation model. Unsupported GPS/QZSS correlation profiles continue to fail explicitly.
2. The acceptance case uses BeiDou because its frozen V1 correlation coverage is complete for this code path, not because the navigation was modified to create a desired geometry.
3. Atmosphere is disabled in this urban acceptance run so that the test isolates the urban observation effects. Existing broadcast-atmosphere tests remain responsible for atmosphere-model validation.
4. The scene remains the frozen deterministic four-wall V1 model. No finite-width street-canyon extension was introduced for #124.
5. No suitable representative measured-urban observation/log data set is frozen in this issue's validation inputs, so measured-urban statistical comparison remains an explicit future validation limitation rather than a claimed result.
6. The approximately 39.6 m worst 3D error is evidence from this one frozen case, not a promise about error distribution in arbitrary real urban environments.

## Acceptance conclusion

Within the explicitly supported correlation scope, the chain

`authentic BRD400DLR NAV -> broadcast satellite state -> four-wall propagation -> coherent received field -> DLL/tracking -> pseudorange/Doppler/ADR/CN0 -> RANGEA -> RTKLIB PVT -> truth diagnostics`

is deterministic, traceable, and end-to-end consumable. The validation does not fabricate navigation information and does not tune the final position solution.
