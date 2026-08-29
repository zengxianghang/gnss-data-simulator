# TTFF / REA transient black-box validation

## Purpose

`validate-transient-observations` validates receiver-grade TTFF and REA observation transients from serialized receiver output. It intentionally does not use the simulator's in-memory `MeasurementObservation` objects as the acceptance path.

The contract is:

```text
real RINEX NAV
    -> simulator
    -> zero-noise observation_truth.csv
    -> serialized simulated.log / #RANGEA

observation_truth.csv -------------------+
                                         |
serialized #RANGEA -> independent parser +-> GPST/satellite/signal matching
                                             -> error statistics
                                             -> TTFF/REA timing checks
                                             -> REA loss/recovery checks

serialized #RANGEA -> independent parser -> RTKLIB pntpos()
                                             -> transient peak position error
                                             -> final recovered position error
```

Navigation truth must come from real RINEX NAV. The validator does not generate, repair, interpolate, or fabricate ephemeris records.

## Statistics

For pseudorange, Doppler converted to range-rate, ADR converted to metres, and reported CN0, the JSON summary contains:

- sample count;
- mean error;
- RMS;
- population standard deviation;
- P50 absolute error;
- P95 absolute error;
- maximum absolute error.

The normal lock-time windows are:

- `early_0_1s`: serialized observation lock time `< 1 s`;
- `recovery_1_3s`: serialized observation lock time `1-3 s`;
- `settled_ge_3s`: serialized observation lock time `>= 3 s`.

The JSON also reports top-level `first_valid` delays for pseudorange, Doppler, ADR, and CN0. REA additionally reports pre-loss fade, reacquisition early/recovery/settled windows, zero-observation signal-off epochs, signal-on recovery timing, the last serialized observation before signal-off, the first serialized observation after signal-on, and ADR ambiguity identity changes across the loss.

## Matching and validity

Serialized RANGEA records are parsed independently, including NovAtel CRC32, GPST, constellation, PRN, OEM7 signal type, pseudorange-valid status, ADR validity, Doppler value, CN0, and lock time. They are matched to `observation_truth.csv` by normalized GPST, satellite number, and canonical signal id.

Pseudorange first-valid timing is derived from the serialized pseudorange-valid bit. ADR first-valid timing is derived from serialized phase/ADR validity bits. CN0 first-valid timing requires a serialized observation. The current RANGEA writer does not serialize an independent Doppler-valid bit; therefore Doppler first-valid timing requires both a matched serialized observation and the zero-noise truth Doppler-valid state for that exact epoch/satellite/signal. This limitation is explicit and must not be mistaken for an independently encoded Doppler validity flag.

`observation_truth.csv` remains the zero-noise reference for error magnitude and ADR ambiguity identity. Serialized RANGEA establishes whether the observation actually exists and carries the measured value. The integer ambiguity identity itself is not recoverable from RANGEA, so the pre-loss/post-reacquisition "fresh ambiguity" check compares truth ambiguity metadata for signals that are independently proven to exist in the black-box validation path.

## Position recovery semantics

The round-trip SPP path uses the complete real RINEX NAV store, the independently parsed serialized RANGEA code observations, RTKLIB `pntpos()`, and the normal 5 degree solution elevation mask.

Two position metrics are retained:

- `max_position_error_m`: the largest error over every valid SPP epoch, including intended TTFF/REA transient epochs;
- `final_position_error_m`: the last valid SPP epoch, used to verify recovery.

This distinction is intentional. Transient acquisition/reacquisition epochs can produce substantially larger SPP errors than the settled solution. Those peaks are retained as evidence rather than hidden by tuning the observations or loosening the statistic. Acceptance requires the final valid SPP epoch to recover below 5 m; it does not cap the transient maximum.

## Compact CI

`TransientValidatorIntegration` uses the provenance-traceable real `tests/data/minimal/brd400dlr_rinex4_acceptance_nav.rnx` fixture at GPST week 2347 / sow 436500.

The compact tests cover TTFF HOT and REA fade/reacquisition. They require:

- serialized/truth matching without unmatched observations;
- TTFF pseudorange, Doppler, and CN0 RMS to decay through `early > recovery > settled`;
- survey-grade settled noise bounds;
- empty RANGEA observations while REA signal is off;
- bounded serialized first-valid and signal-on recovery timing;
- fresh ADR ambiguity after reacquisition;
- recovered RTKLIB SPP;
- identical acquisition/reacquisition timing for the same seed when `measurement_noise_enabled` is toggled on and off.

The compact tests deliberately use statistical/envelope assertions instead of exact stochastic sample values.

## Extended real-WHU survey

`.github/workflows/transient-validation-extended.yml` is a manual/monthly survey using the pinned real WHU navigation file:

```text
BRD400DLR_S_20250030000_01D_MN.rnx.gz
```

Pinned hashes:

```text
gzip SHA256:         fb84d4046b06e905e8e4ec0efb82f0e9ad044bc44d664cc0209cb9b4c92b9512
uncompressed SHA256: b11c638eea42978b8bd6aa8b65a5099fe6556dfe527bc037ed481d2b239afc42
```

The extended survey covers TTFF HOT, WARM, COLD, REA hard-cut, and REA fade using fixed seeds. Each case writes a JSON summary and the workflow uploads summaries, manifests, and validator diagnostics.

The final TTFF gates require pseudorange, Doppler, and CN0 RMS to decrease through all three windows, not merely between the endpoints:

```text
early_0_1s > recovery_1_3s > settled_ge_3s
```

### Pre-merge real-WHU result

A full five-scenario survey was executed in GitHub Actions run `33230616700` using the pinned WHU file and both SHA256 checks. The run completed successfully. The observed TTFF envelopes also satisfy the final three-stage gate with wide margin:

| Scenario | PSR RMS early -> recovery -> settled (m) | Doppler RMS early -> recovery -> settled (m/s) | Settled ADR RMS (m) | Max transient SPP error (m) |
| --- | --- | --- | --- | --- |
| TTFF HOT | 0.367038 -> 0.204716 -> 0.081367 | 0.064916 -> 0.034994 -> 0.030016 | 0.000993 | 2.547964 |
| TTFF WARM | 0.442217 -> 0.278356 -> 0.081871 | 0.091114 -> 0.046890 -> 0.030073 | 0.000998 | 3.993733 |
| TTFF COLD | 0.629439 -> 0.442173 -> 0.084113 | 0.117659 -> 0.066533 -> 0.030133 | 0.000999 | 5.550588 |

The COLD transient maximum exceeds 5 m, which is acceptable by design because the recovery gate applies to `final_position_error_m`, not `max_position_error_m`.

REA results from the same run:

- hard-cut and fade each produced 60 serialized RANGEA signal-off epochs with zero non-empty observation epochs;
- each case exercised 3 reacquisition cycles;
- first recovered pseudorange, Doppler, and ADR delays were at most 0.2 s, 0.3 s, and 0.6 s respectively;
- all 721 comparable pre-loss/post-reacquisition ambiguity identities changed in each REA case;
- maximum transient SPP error was 15.671076 m for hard-cut and 7.238175 m for fade;
- both cases passed the final recovered SPP `< 5 m` gate;
- the fade case contained the expected finite pre-loss degradation window, while hard-cut had no fade-window samples.

The artifact from that run is named `transient-real-whu-33230616700` and contains the per-case JSON summaries and diagnostics.

## CLI example

```bash
validate-transient-observations \
  --log run/simulated.log \
  --truth run/observation_truth.csv \
  --events run/event_truth.csv \
  --nav BRD400DLR_S_20250030000_01D_MN.rnx \
  --scenario REA_FADE \
  --fade-duration 0.25 \
  --output run/transient_summary.json \
  --truth-lat 20 \
  --truth-lon 120 \
  --truth-height 100 \
  --elevation-mask 5 \
  --broadcast-atmosphere
```

## Non-goals

The validator does not tune observations to force RTKLIB truth, simulate multipath/obstruction geometry, fabricate navigation data, or emulate receiver ephemeris acquisition order inside the black-box round-trip solver. Receiver-NAV acquisition timing remains a separate concern from this observation-error validation.
