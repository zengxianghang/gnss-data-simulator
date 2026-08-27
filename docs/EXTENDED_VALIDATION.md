# V1 Extended Validation

Issue #41 owns long-duration and resource validation. Normal pull-request CI remains the short deterministic suite from #40; expensive runs live in `.github/workflows/extended.yml`.

## Cases

| Case | Scenario | Duration / rate | Purpose |
| --- | --- | --- | --- |
| `ks_8h` | KS | 8 h @ 10 Hz | Default V1 long static run and exact epoch/message counts. |
| `rea_8h` | REA | 8 h @ 10 Hz | Long repeated signal-off/reacquisition transitions. |
| `ttff_8h` | TTFF HOT | 8 h @ 10 Hz | Long repeated power cycling and restart transitions. |
| `stress_50hz_1h` | KS | 1 h @ 50 Hz | High-rate streaming stress using the real BRD400DLR RINEX 4 fixture. |
| `determinism_50hz_10m` | KS | 10 min @ 50 Hz, repeated twice | Streamed SHA256 equality for receiver log and all required truth/manifest outputs. |
| `memory_trend_50hz` | KS | 60 s vs 15 min @ 50 Hz | Detect observation/solution history that grows with run duration. |

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

The 8-hour cases use the committed synthetic full-sky GPS loopback fixture as their deterministic source. The harness materializes fresh hourly Toc/Toe/TTR copies of those same ephemerides for the test window. This is a test-only resource fixture: orbital parameters remain fixed while ephemeris reference/transmit times advance so RTKLIB does not reject the single original epoch as stale during an 8-hour run.

No network access or live IGS download is used at extended-test runtime. The 50 Hz cases use the committed real WHU `BRD400DLR` RINEX 4 fixture from #40.

## Exact correctness gates

For every run the harness requires:

- `scheduled_epochs == duration_sec * sampling_rate_hz`;
- RANGE, PSRPOS and PSRVEL message counts equal the exact scheduled epoch count when the selected scenario is powered for the corresponding epochs;
- scenario transition counts match integer-time boundary calculations;
- simulator exit status is zero;
- `run_manifest.json` exists and contains the run summary;
- a non-zero peak RSS measurement is available.

REA and TTFF transition expectations are calculated using integer nanoseconds and the same half-open `[0, duration)` run interval used by the simulator.

## Memory guardrail

`memory_trend_50hz` compares a 60-second run with a 900-second run using the same BRD400DLR input, receiver configuration and seed. The long-run peak RSS must not exceed:

`max(2 * short_peak_rss, short_peak_rss + 256 MiB)`

The duration ratio is 15:1, so this conservative limit tolerates allocator/runner variation while still rejecting linear retention of per-epoch observation/solution history. The measured short/long peaks and the resolved limit are written to `summary.json`.

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

## Diagnostics

Each case writes a small artifact directory containing:

- resolved `config.json`;
- `scenario.json` and `run_manifest.json` when produced;
- simulator `stdout.txt` / `stderr.txt`;
- `summary.json` with elapsed time, peak RSS, logical output sizes, SHA256 values and event counts;
- `summary.md` for the GitHub Actions job summary;
- generated long-duration GPS NAV when applicable.

Artifacts are uploaded with `if: always()` so a failed or timed-out case does not become a false PASS and still leaves useful evidence when the runner permits cleanup steps to execute.
