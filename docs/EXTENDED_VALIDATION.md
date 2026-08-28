# V1 Extended Validation

Issue #41 owns long-duration and resource validation. Issue #58 freezes navigation provenance for those tests: extended validation must use real RINEX NAV records and must never manufacture ephemeris. Normal pull-request CI remains the short deterministic suite from #40; expensive runs live in `.github/workflows/extended.yml` and use `tools/extended_validation/run_extended.py`.

## Cases

| Case | Scenario | Duration / rate | NAV input | Purpose |
| --- | --- | --- | --- | --- |
| `ks_8h` | KS | 8 h @ 10 Hz | Real full-day WHU `BRD400DLR` | Default V1 long static run and exact epoch/message counts. |
| `rea_8h` | REA | 8 h @ 10 Hz | Real full-day WHU `BRD400DLR` | Long repeated signal-off/reacquisition transitions. |
| `ttff_8h` | TTFF HOT | 8 h @ 10 Hz | Real full-day WHU `BRD400DLR` | Long repeated power cycling and restart transitions. |
| `stress_50hz_1h` | KS | 1 h @ 50 Hz | Real full-day WHU `BRD400DLR` | Sustained high-rate streaming/resource stress across real broadcast navigation. |
| `determinism_50hz_10m` | KS | 10 min @ 50 Hz, repeated twice | Committed real WHU RINEX 4 subset | Streamed SHA256 equality for receiver log and truth/manifest outputs. |
| `memory_trend_50hz` | KS | 60 s vs 15 min @ 50 Hz | Committed real WHU RINEX 4 subset | Detect observation/solution history that grows with run duration. |

Manual dispatch can select one case or `all`. The scheduled run intentionally selects `ks_8h` plus `memory_trend_50hz`; the other expensive cases remain independently dispatchable so a weekly schedule does not consume unnecessary CI minutes.

## Frozen NAV provenance rule

Broadcast ephemeris is input truth. Extended validation must not create navigation coverage by modifying an existing record.

Forbidden operations include creating a new EPH record by changing any of the following from another record:

- Toe or Toc;
- transmit time or GPS week;
- `M0`, `delta_n`, `Omega0`, `OmegaDot`, `i0`, `IDOT`, eccentricity, semi-major axis, harmonic corrections, or other orbit parameters;
- clock polynomial values;
- IODE/IODC/issue, health, accuracy, fit interval;
- TGD/BGD/ISC or GLONASS timing/frequency-channel navigation values.

The previous `shift_gps_ephemeris_reference()` / `materialize_long_gps_nav()` path was removed by #58. There is no synthetic fallback. If real NAV coverage is absent, the validation must fail or obtain the real RINEX file from an authoritative data center.

Allowed preparation is limited to downloading, decompressing, copying, concatenating, filtering, or extracting a time window from real RINEX data while preserving the retained parsed navigation values unchanged.

## Pinned real full-day RINEX source

The current long-duration cases use the real full-day RINEX 4 broadcast navigation product already recorded by the repository's RINEX 4 acceptance-fixture metadata:

- Data center: Wuhan University IGS Data Center (WHU/IGS).
- Product: `BRD400DLR`.
- Source file: `BRD400DLR_S_20250030000_01D_MN.rnx.gz`.
- Primary source URL: `ftp://igs.gnsswhu.cn/pub/gps/data/daily/2025/brdc/BRD400DLR_S_20250030000_01D_MN.rnx.gz`.
- HTTP fallback at the same WHU data center: `http://igs.gnsswhu.cn/pub/gps/data/daily/2025/brdc/BRD400DLR_S_20250030000_01D_MN.rnx.gz`.
- Compressed SHA256: `fb84d4046b06e905e8e4ec0efb82f0e9ad044bc44d664cc0209cb9b4c92b9512`.
- Uncompressed SHA256: `b11c638eea42978b8bd6aa8b65a5099fe6556dfe527bc037ed481d2b239afc42`.
- Long-validation start: GPS week `2347`, SOW `436500`.
- Eight-hour window: SOW `436500` through `465300`.

The workflow downloads the compressed file before the long-duration validation job, verifies the compressed SHA256, decompresses it without changing RINEX contents, verifies the uncompressed SHA256, and then passes that file to the harness using `--real-nav`.

The harness independently verifies the uncompressed SHA256 again before starting a long-duration case. A missing file or hash mismatch is a hard failure. The harness never extends the file by creating ephemeris records.

For a future validation date, obtain the corresponding real daily RINEX NAV from WHU/IGS or another documented authoritative GNSS data center, pin its identity/hash, and change the test window to match that real source. Do not stretch an older fixture into a new time range.

## Committed short RINEX 4 fixture

`tests/data/minimal/brd400dlr_rinex4_acceptance_nav.rnx` is an unchanged-record subset of the same real WHU `BRD400DLR` source. Its provenance is recorded in:

`tests/data/minimal/brd400dlr_rinex4_acceptance_nav.meta.json`

The reduction tool selects complete real records and normalizes container line endings; it does not recalculate or rewrite ephemeris fields. This compact fixture remains suitable for short RINEX 4 / modern-navigation correctness, determinism, and memory-trend coverage. It is not used to fake long-duration coverage.

## Bounded stream capture

The simulator is not put into a reduced-output or null-physics mode for extended validation. It still formats the normal receiver log and event/observation/solution truth streams.

To avoid retaining multi-gigabyte output histories, the Linux harness creates named pipes for:

- `simulated.log`;
- `event_truth.csv`;
- `observation_truth.csv`;
- `solution_truth.csv`.

Reader threads consume those streams incrementally in fixed-size chunks. The harness records SHA256 and logical byte count but does not retain the large byte streams. `event_truth.csv` is small and is retained only long enough to count scenario transition rows. `scenario.json` and `run_manifest.json` remain regular small files and are hashed normally.

This exercises normal output serialization while keeping harness memory and artifact size bounded.

## Exact correctness gates

For every run the harness requires:

- `scheduled_epochs == duration_sec * sampling_rate_hz`;
- `powered_epochs` to equal the exact powered portion of the scenario timeline;
- RANGE, PSRPOS and PSRVEL message counts to equal `powered_epochs`;
- REA `signal_on_epochs` / `signal_off_epochs` to match the exact RF-on/RF-off timeline;
- TTFF power-off duration to be verified by `scheduled_epochs - powered_epochs`;
- scenario transition counts to match integer-time boundary calculations over the half-open `[0, duration)` interval;
- at least one observation, valid position and valid velocity epoch;
- simulator exit status to be zero;
- `run_manifest.json` to exist and contain the run summary;
- a non-zero peak RSS measurement;
- for long-duration cases, the real full-day RINEX NAV SHA256 to match the pinned source exactly.

## Memory guardrail

`memory_trend_50hz` compares a 60-second run with a 900-second run using the same committed real RINEX 4 input, receiver configuration and seed. The long-run peak RSS must not exceed:

`max(2 * short_peak_rss, short_peak_rss + 64 MiB)`

The duration ratio is 15:1. The 64 MiB headroom is deliberately conservative while still detecting retained per-epoch histories. The measured short/long peaks, growth and resolved limit are written to `summary.json`.

## Determinism

`determinism_50hz_10m` runs the exact same extended configuration twice. The harness compares SHA256 values for:

- streamed receiver log;
- streamed event truth;
- streamed observation truth;
- streamed solution truth;
- `scenario.json`;
- `run_manifest.json`.

Comparison is streamed and bounded; no complete 10-minute observation history is loaded into memory.

## Historical evidence

The resource numbers recorded during #41 were produced before #58 and used the then-current synthetic long-NAV helper for the 8-hour/stress cases. They remain historical performance evidence only and are not navigation-provenance acceptance evidence after #58.

After #58, a passing long-duration result is valid only when the job has downloaded and hash-verified the pinned real RINEX NAV described above.

## Diagnostics

Each case writes a small artifact directory containing:

- resolved `config.json`;
- `scenario.json` and `run_manifest.json` when produced;
- simulator `stdout.txt` / `stderr.txt`;
- `summary.json` with elapsed time, peak RSS, logical output sizes, SHA256 values, event counts, and NAV identity/provenance;
- `summary.md` for the GitHub Actions job summary.

The full-day RINEX NAV is a downloaded test input, not a generated diagnostic artifact. No generated NAV file should appear in the result directory.

Artifacts are uploaded with `if: always()` so a failed case does not become a false PASS and still leaves useful evidence when the runner permits cleanup steps to execute.
