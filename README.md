# GNSS Data Simulator

Deterministic C++17 GNSS receiver-data simulator for reproducible positioning,
tracking, multipath, Doppler, and recovery experiments using real broadcast
navigation data and a pinned RTKLIB revision.

The project is built for engineering validation rather than visual-only demos.
It emits receiver-like logs together with versioned truth and provenance
artifacts so changes can be checked against deterministic expectations and
independent RTKLIB-based positioning results.

## Highlights

- **Reproducible KS / REA / TTFF scenarios** with deterministic seeds, event
  truth, observation truth, solution truth, and run manifests.
- **Real broadcast navigation input** from offline RINEX data; test fixtures
  include a reduced real WHU `BRD400DLR` RINEX 4.02 navigation data set with
  provenance metadata.
- **Urban propagation and receiver effects**, including deterministic
  first-order wall reflections, rooftop Fresnel diffraction, signal-specific
  code correlation, DLL behavior, coherent received power, and tracking state.
- **Carrier and Doppler modeling**, including time-correlated path rate,
  configurable PLL/FLL tracking, reacquisition, Doppler/range-rate validity,
  and exact diagnostic truth outputs.
- **RTKLIB end-to-end validation** for serialized observations and navigation,
  including positioning, velocity, clock-drift, and signal-level residual
  checks where supported by the pinned dependency.
- **Cross-platform regression coverage** on Ubuntu and Windows, with separate
  extended validation for long high-rate runs.
- **Synchronized multi-seed batches** for repeatable cross-board KS/REA/TTFF
  comparisons over one common physical GPST window.

## Build

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

The repository uses Git submodules. For a fresh clone, initialize them before
building:

```bash
git submodule update --init --recursive
```

## Run

```bash
./build/gnss-data-simulator \
  --config config/default_v1.json \
  --nav tests/data/minimal/brd400dlr_rinex4_acceptance_nav.rnx \
  --output simulated.log \
  --week 2347 \
  --sow 436500
```

The simulator writes the receiver log plus versioned truth/manifest files beside
it. Normal pull-request CI runs deterministic short acceptance tests; long
8-hour / 50 Hz resource validation is defined separately in
`.github/workflows/extended.yml` and documented in
[`docs/EXTENDED_VALIDATION.md`](docs/EXTENDED_VALIDATION.md).

An optional finite `receiver_clock_drift_mps` in the JSON config generates a deterministic constant receiver clock rate. The default remains zero; `receiver_clock_bias_m` is still required to be zero at the run start. Per-epoch truth records the accumulated bias and rate. See [`docs/V1_DEFAULTS.md`](docs/V1_DEFAULTS.md#receiver-clock) for the measurement semantics.

## Synchronized multi-seed batches

For simultaneous cross-board KS/REA/TTFF comparison, use the batch wrapper so
every realization shares one physical GPST window and one common base
configuration while receiving a distinct deterministic seed:

```bash
python3 tools/batch_generation/generate_batch.py \
  --simulator build/gnss-data-simulator \
  --config config/default_v1.json \
  --nav /path/to/real_broadcast_nav.rnx \
  --output-dir output/batch_001 \
  --week 2347 \
  --sow 436500
```

The default batch is KS x8 + REA x8 + TTFF x8 with seed ranges `1001..1008`,
`2001..2008`, and `3001..3008`. See
[`docs/SYNCHRONIZED_BATCH_GENERATION.md`](docs/SYNCHRONIZED_BATCH_GENERATION.md)
for the invariant-time contract, manifest layout, reproducibility rules, and CI
validation.

## Validation and engineering documentation

- [`docs/DESIGN_SPEC.md`](docs/DESIGN_SPEC.md) — architecture and V1 scope.
- [`docs/ENGINEERING_RULES.md`](docs/ENGINEERING_RULES.md) — repository layout,
  coding rules, CI policy, and test principles.
- [`docs/NAV_RECORDS.md`](docs/NAV_RECORDS.md) — supported NovAtel OEM7 and
  Unicore N4 navigation records.
- [`docs/V1_DEFAULTS.md`](docs/V1_DEFAULTS.md) — frozen V1 default parameters.
- [`docs/STARTUP_RECOVERY_MODEL.md`](docs/STARTUP_RECOVERY_MODEL.md) — HOT/WARM/COLD
  TTFF and REA recovery model.
- [`docs/V1_ACCEPTANCE_MATRIX.md`](docs/V1_ACCEPTANCE_MATRIX.md) — short
  deterministic V1 acceptance coverage.
- [`docs/URBAN_E2E_VALIDATION.md`](docs/URBAN_E2E_VALIDATION.md) — authentic-NAV
  urban end-to-end validation and RTKLIB round-trip evidence.
- [`docs/STATIC_URBAN_DOPPLER_VALIDATION.md`](docs/STATIC_URBAN_DOPPLER_VALIDATION.md)
  — static urban Doppler, range-rate, and velocity validation.
- [`docs/CARRIER_TRACKING_CORE.md`](docs/CARRIER_TRACKING_CORE.md) — configurable
  PLL/FLL carrier-tracking model and assumptions.
- [`docs/CARRIER_TRACKING_AUTHENTIC_VALIDATION.md`](docs/CARRIER_TRACKING_AUTHENTIC_VALIDATION.md)
  — authentic-NAV observation-level carrier-tracking validation.
- [`docs/EXTENDED_VALIDATION.md`](docs/EXTENDED_VALIDATION.md) — 8-hour / 50 Hz
  streaming, determinism, and memory validation.
- [`docs/SYNCHRONIZED_BATCH_GENERATION.md`](docs/SYNCHRONIZED_BATCH_GENERATION.md)
  — synchronized x8 multi-seed generation and reproducibility contract.

## Related GNSS tools

This repository is the simulator component of a set of focused public GNSS
engineering tools maintained separately:

- [`gnss-data-parser`](https://github.com/zengxianghang/gnss-data-parser) —
  streaming Python/MATLAB parsers and cross-language validation for receiver
  logs.
- [`FastExtractor`](https://github.com/zengxianghang/FastExtractor) — fast
  GPST-window extraction for large NovAtel/Unicore observation logs.
- [`LogMerger`](https://github.com/zengxianghang/LogMerger) — GNSS log merge
  tooling.
- [`RTKLIB`](https://github.com/zengxianghang/RTKLIB) — the RTKLIB fork used by
  simulator integration and validation work.

## Validation data

The committed test suite includes compact deterministic fixtures, including a
reduced real WHU `BRD400DLR` RINEX 4.02 broadcast-navigation fixture with
provenance metadata. Normal and extended CI do not download live IGS data at
test runtime.

## License

Original `gnss-data-simulator` code is licensed under the
[MIT License](LICENSE). Third-party submodules retain their own licenses; see
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) for attribution and license
boundaries.
