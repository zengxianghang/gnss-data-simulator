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
| Five-system ideal static loopback | PENDING | `NavOutputWriter.LegacyMixedRinexCoversFiveEphemerisFamilies` proves NAV parsing/writer coverage only | A compact five-system NAV fixture exists, but it contains only one non-GPS satellite per system and does not yet prove a five-system full simulator/SPP loopback. #40 must add an end-to-end case before closing. |
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
| Normal CI on Ubuntu/GCC + Windows/MSVC + format | PENDING | PR CI | #40 may merge only after all three jobs are green. |

## Five-system loopback completion rule

Issue #40 remains open until one deterministic short end-to-end run proves that GPS, GLONASS, Galileo, BeiDou and QZSS observations are all produced from a common truth state in the same run and the receiver solution is valid without bypassing the normal simulator/Receiver-NAV path.

The acceptance test must record the participating systems from `observation_truth.csv` (or an equivalent simulator-produced truth artifact) rather than inferring participation merely because the NAV fixture contains records for five constellations.

If the current compact fixtures cannot provide sufficient simultaneously visible satellites and Receiver-NAV observability for a five-system SPP solution, #40 must add a compact deterministic fixture. It must not relax solver validity or use Truth NAV directly to manufacture a PASS.

## Numerical policy

Existing ideal static loopback tolerances are intentionally tight:

- latitude/longitude: `1e-6 deg`;
- height and receiver clock bias: `0.10 m`;
- ECEF velocity and receiver clock drift: `1e-3 m/s`.

These tolerances must not be widened solely to make CI pass. Any change requires a documented numerical reason tied to the RTKLIB processing path.
