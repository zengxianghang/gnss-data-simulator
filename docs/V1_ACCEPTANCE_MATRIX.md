# V1 Short Acceptance Matrix

This document is the executable acceptance map for issue #40. It distinguishes full simulator/integration coverage from lower-layer deterministic unit coverage so that a unit test is never presented as a substitute for an end-to-end requirement.

Long-duration 8-hour and resource/50 Hz stress validation belongs to #41, not this matrix.

## Status legend

- **PASS**: covered by an existing or #40 test at the required layer.
- **PARTIAL**: deterministic lower-layer coverage exists, but the required end-to-end acceptance case is still missing.
- **PENDING**: #40 must add coverage before it can close.

## Matrix

| Requirement | Status | Evidence / executable test | Notes |
| --- | --- | --- | --- |
| GPS-only ideal static loopback | PASS | `SolutionEngine.IdealStaticZeroNoiseLoopbackRecoversTruth`; `StreamingSimulator.KsProducesOneLogSetPerEpoch` | Solver test uses eight GPS satellites and tight zero-noise position/velocity tolerances; streaming test exercises the simulator orchestration path. |
| Five-system ideal static loopback from real RINEX 3 NAV | PASS | `V1Acceptance.RealMixedNavProducesAllFiveV1ConstellationsAndValidSolution` | GPS, GLONASS, Galileo, BeiDou and QZSS are verified from simulator-produced `observation_truth.csv`; position and velocity must both become valid through Receiver NAV. |
| Real BRD400DLR RINEX 4 parsing | PASS | `Rinex4Brd400.ProjectLoaderPreservesFiveSystemsAndModernEphemerisFamilies`; `Rinex4Brd400.PinnedRtklibConsumesStoEopAndIonRecords` | Reduced real WHU BRD400DLR RINEX 4.02 data covers all five V1 constellations, GPS CNAV, QZSS CNAV/CNV2, BDS CNV1/CNV2/CNV3 and STO/EOP/ION records. |
| Five-system simulator loopback from real BRD400DLR RINEX 4 NAV | PASS | `V1Acceptance.RealBrd400DlrRinex4RunsFiveSystemReceiverNavLoopback` | The BRD400DLR fixture goes through the normal pinned-RTKLIB Truth-NAV/Receiver-NAV, satellite, measurement, RANGE and solution path; all five systems must appear and position/velocity must become valid. |
| Every frozen V1 signal mapping | PASS | `SignalDefinitions.CoversEveryFrozenV1SignalExactlyOnce`; `SignalDefinitions.FrozenMappingsMatchExpectedValuesAndRoundTrip` | Explicit table contains all 21 frozen V1 signals. |
| 1/5/10/20/50 Hz full streaming pipeline | PASS | `V1Acceptance.FrozenRatesRunThroughFullStreamingPipeline` | #40 runs the actual simulator for 10 s at each frozen rate and checks exact message/epoch counts plus truth artifacts. |
| GPST week crossover | PASS | `ScenarioEngine.KsRemainsPoweredAndSignalOn`; satellite-engine cross-week tests | Integer-time scenario state already crosses week 2300 -> 2301 without false transitions. |
| HOT TTFF | PASS | `StreamingSimulator.HotTtffSuppressesAllReceiverLogsWhilePowerIsOff` | Full simulator scenario behavior. |
| WARM TTFF | PASS | `StreamingSimulator.WarmTtffSuppressesAllReceiverLogsWhilePowerIsOff` | Full simulator scenario behavior. |
| COLD TTFF / no premature solve | PASS | `StreamingSimulator.ColdTtffAcquiresEphemerisBeforeSolutionBecomesValid`; `SolutionEngine.ColdReceiverCannotUseTruthOnlyEphemeris` | Covers both orchestration and Truth-NAV/Receiver-NAV isolation. |
| REA signal-off/reacquisition | PASS | `StreamingSimulator.ReaKeepsLogsRunningWithZeroRangeDuringSignalOff`; signal-tracking REA unit tests | Full simulator verifies continued logs, zero RANGE observations and invalid solutions while RF is off. |
| GLONASS FDMA channel-dependent carrier | PASS | `SignalDefinitions.GlonassG1AndG2UseFcnDependentFrequencyAndWavelength`; `RtklibBroadcastBiasAdapter.SelectsLegacyTgdGalileoFamilyAndGlonassFcn` | FCN-dependent frequency/wavelength plus parsed FCN are explicit. |
| GPS TGD/ISC corrections | PASS | `BroadcastCodeBias.GpsLegacyAndModernFollowIcdClockCorrections` | Legacy, CNAV and CNAV2 paths covered. |
| Galileo BGD corrections | PASS | `BroadcastCodeBias.GalileoUsesClockFamilySpecificBgd` | I/NAV and F/NAV paths covered. |
| BeiDou TGD/B-CNAV corrections | PASS | `BroadcastCodeBias.BeidouAndGlonassUseTheirBroadcastDelayDefinitions` | Legacy and B-CNAV paths covered. |
| GLONASS dtaun code bias | PASS | `BroadcastCodeBias.BeidouAndGlonassUseTheirBroadcastDelayDefinitions` | G1 no-correction and G2 dtaun behavior covered. |
| Receiver NAV vs Truth NAV isolation | PASS | `SolutionEngine.ColdReceiverCannotUseTruthOnlyEphemeris`; navigation-state isolation tests | Truth NAV is not accepted as Receiver NAV during COLD startup. |
| Observation truth decomposition / join keys | PASS | `TruthOutputs.HeadersAreVersionedAndExplicit` | Schema includes GPST/index/system/PRN/signal, geometry, clocks, atmosphere, raw broadcast-bias provenance, GLONASS FCN/wavelength, tracking/validity, ambiguity and final observables. |
| Deterministic truth rerun | PASS | `TruthOutputs.SameInputConfigAndSeedAreByteIdenticalAcrossOutputDirectories` | Byte-identical scenario/event/observation/solution/manifest truth files. |
| Deterministic complete run rerun | PASS | `V1Acceptance.SameInputConfigAndSeedProduceByteIdenticalReceiverAndTruthOutputs` | #40 additionally byte-compares `simulated.log` together with every truth artifact. |
| OEM7 RANGEA/PSRPOSA/PSRVELA representative golden/format coverage | PASS | `test_output_writers.cpp` suite | Existing writer tests cover valid/invalid and tracking-status serialization. |
| OEM7/N4 navigation output across five constellations | PASS | `NavOutputWriter.LegacyMixedRinexCoversFiveEphemerisFamilies`; `NavOutputWriter.LegacyIonosphereMetadataMapsToFrozenFamilies` | Same Receiver-NAV records are serialized into both output families. |
| Normal CI on Ubuntu/GCC + Windows/MSVC + format | PENDING | PR CI | #40 may merge only after all three jobs are green on the final head. |

## Real-navigation fixture policy

The short suite deliberately contains both RINEX 3 and RINEX 4 real broadcast-navigation fixtures. The RINEX 4 case is derived from `BRD400DLR`, not from a hand-authored synthetic navigation file, so modern message-family parsing is exercised with real field layouts.

Large upstream products are never fetched during normal CI. `tools/download_igs/materialize_brd4_fixture.py` deterministically reduces the selected BRD400DLR day to a compact checked-in fixture and writes source/fixture hashes and the selected message inventory to `brd400dlr_rinex4_acceptance_nav.meta.json`.

## Five-system loopback completion rule

Issue #40 requires deterministic short end-to-end runs proving that GPS, GLONASS, Galileo, BeiDou and QZSS observations are all produced from a common truth state in the same run and that the receiver solution is valid without bypassing the normal simulator/Receiver-NAV path. This is now exercised independently with real RINEX 3 mixed NAV and real BRD400DLR RINEX 4 NAV.

The acceptance tests record participating systems from `observation_truth.csv` rather than inferring participation merely because a NAV fixture contains records for five constellations. They do not relax solver validity or use Truth NAV directly to manufacture a PASS.

## Numerical policy

Existing ideal static loopback tolerances are intentionally tight:

- latitude/longitude: `1e-6 deg`;
- height and receiver clock bias: `0.10 m`;
- ECEF velocity and receiver clock drift: `1e-3 m/s`.

These tolerances must not be widened solely to make CI pass. Any change requires a documented numerical reason tied to the RTKLIB processing path.
