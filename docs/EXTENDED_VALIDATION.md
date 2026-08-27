# V1 Extended Validation

Issue #41 owns long-duration and resource validation. Normal pull-request CI remains the short deterministic suite from #40; expensive runs live in `.github/workflows/extended.yml` and use the single harness `tools/extended_validation/run_extended.py`.

## Cases

| Case | Scenario | Duration / rate | Purpose |
| --- | --- | --- | --- |
| `ks_8h` | KS | 8 h @ 10 Hz | Default V1 long static run and exact epoch/message counts. |
| `rea_8h` | REA | 8 h @ 10 Hz | Long repeated signal-off/reacquisition transitions. |
| `ttff_8h` | TTFF HOT | 8 h @ 10 Hz | Long repeated power cycling and restart transitions. |
| `stress_50hz_1h` | KS | 1 h @ 50 Hz | Sustained high-rate streaming/resource stress with continuous deterministic GPS NAV. |
| `determinism_50hz_10m` | KS | 10 min @ 50 Hz, repeated twice | Streamed SHA256 equality for receiver log and all required truth/manifest outputs using the compact real RINEX 4 fixture. |
| `memory_trend_50hz` | KS | 60 s vs 15 min @ 50 Hz | Detect observation/solution history that grows with run duration using the compact real RINEX 4 fixture. |

Manual dispatch can select one case or `all`. The scheduled run intentionally selects `ks_8h` plus `memory_trend_50hz`; the other expensive cases remain independently dispatchable so a weekly schedule does not consume unnecessary CI minutes.

## Bounded stream capture

The simulator is not put into a reduced-output or null-physics mode for extended validation. It still formats the normal receiver log and event/observation/solution truth streams.

To avoid retaining multi-gigabyte output histories, the Linux harness creates named pipes for:

- `simulated.log`;
- `event_truth.csv`;
- `observation_truth.csv`;
- `solution_truth.csv`.

Reader threads consume those streams incrementally in fixed-size chunks. The harness records SHA256 and logical byte count but does not retain the large byte streams. `event_truth.csv` is small and is additionally retained in memory only long enough to count scenario transition rows. `scenario.json` and `run_manifest.json` remain regular small files and are hashed normally.

This means output serialization cost is still exercised while harness memory and artifact size stay bounded.

## Long-duration NAV fixture

The 8-hour cases and `stress_50hz_1h` use the committed deterministic full-sky GPS loopback fixture as their source. The harness materializes hourly broadcast ephemeris records across the required test window.

The hourly records are not created by changing Toe/Toc alone. When the reference epoch is shifted by `dt`, the harness applies the corresponding broadcast-orbit/clock reference transformation used by RTKLIB:

- `M0 += (sqrt(mu/A^3) + delta_n) * dt`;
- `Omega0 += OmegaDot * dt`;
- `i0 += IDOT * dt`;
- the satellite clock polynomial is shifted to the new Toc;
- Toe, GPS week and transmission time are moved consistently.

This preserves a continuous deterministic orbit/clock across refresh boundaries and prevents a synthetic ephemeris switch from creating a non-physical satellite-state jump.

The compact real WHU `BRD400DLR` RINEX 4 fixture from #40 remains the source for short RINEX 4 / modern navigation correctness coverage and is also used by the 10-minute determinism and 15-minute memory-trend cases. It is intentionally not stretched beyond its reduced ephemeris coverage window for the 1-hour stress case.

No network access or live IGS download is used at extended-test runtime.

## Exact correctness gates

For every run the harness requires:

- `scheduled_epochs == duration_sec * sampling_rate_hz`;
- `powered_epochs` to equal the exact powered portion of the scenario timeline;
- RANGE, PSRPOS and PSRVEL message counts to equal `powered_epochs`;
- REA `signal_on_epochs` / `signal_off_epochs` to match the exact RF-on/RF-off timeline;
- TTFF power-off duration to be verified by `scheduled_epochs - powered_epochs`; TTFF `signal_off_epochs` remains zero because the simulator counts RF-off epochs only while the receiver is powered;
- scenario transition counts to match integer-time boundary calculations over the half-open `[0, duration)` interval;
- at least one observation, valid position and valid velocity epoch;
- simulator exit status to be zero;
- `run_manifest.json` to exist and contain the run summary;
- a non-zero peak RSS measurement to be available.

## Memory guardrail

`memory_trend_50hz` compares a 60-second run with a 900-second run using the same RINEX 4 input, receiver configuration and seed. The long-run peak RSS must not exceed:

`max(2 * short_peak_rss, short_peak_rss + 64 MiB)`

The duration ratio is 15:1. The 64 MiB headroom is deliberately conservative relative to the measured implementation while still detecting retained per-epoch histories. The measured short/long peaks, growth and resolved limit are written to `summary.json`.

This is intentionally a trend guardrail, not a fragile absolute performance benchmark.

## Determinism

`determinism_50hz_10m` runs the exact same extended configuration twice. The harness compares SHA256 values for:

- streamed receiver log;
- streamed event truth;
- streamed observation truth;
- streamed solution truth;
- `scenario.json`;
- `run_manifest.json`.

Comparison is streamed and bounded; no complete 10-minute observation history is loaded into memory.

## Implementation evidence

During #41 implementation, real GitHub-hosted Ubuntu runners produced the following passing evidence before final workflow cleanup:

| Case | Result | Peak RSS | Logical output |
| --- | --- | ---: | ---: |
| `ks_8h` | PASS | 5.9 MiB | 6.952 GiB |
| `rea_8h` | PASS | 6.0 MiB | 6.705 GiB |
| `ttff_8h` | PASS | 5.9 MiB | 6.269 GiB |
| `stress_50hz_1h` | PASS | 6.0 MiB | 8.205 GiB |
| `determinism_50hz_10m` | PASS | bounded streamed comparison | two identical output fingerprints |
| `memory_trend_50hz` | PASS | 6.8 MiB -> 6.9 MiB | 0.360 GiB -> 5.565 GiB |

The memory-trend run increased duration/output volume by 15x while peak RSS changed by only about 0.1 MiB, providing direct evidence that the simulator/harness path does not retain output history linearly with run duration.

The final merge gate reruns the matrix through the unified harness rather than relying only on these intermediate implementation artifacts.

## Diagnostics

Each case writes a small artifact directory containing:

- resolved `config.json`;
- `scenario.json` and `run_manifest.json` when produced;
- simulator `stdout.txt` / `stderr.txt`;
- `summary.json` with elapsed time, peak RSS, logical output sizes, SHA256 values and event counts;
- `summary.md` for the GitHub Actions job summary;
- generated long-duration GPS NAV when applicable.

`summary.json` also retains small stdout/stderr tails so missing-manifest or non-zero-exit failures are diagnosable without first downloading a multi-gigabyte output stream.

Artifacts are uploaded with `if: always()` so a failed case does not become a false PASS and still leaves useful evidence when the runner permits cleanup steps to execute.
