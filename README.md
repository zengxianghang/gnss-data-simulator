# GNSS Data Simulator

Deterministic C++17 GNSS log simulator for KS / REA / TTFF scenarios, backed by a pinned RTKLIB revision and offline RINEX navigation data.

## Build

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
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

The simulator writes the receiver log plus versioned truth/manifest files beside it. Normal pull-request CI runs deterministic short acceptance tests; long 8-hour / 50 Hz resource validation is defined separately in `.github/workflows/extended.yml` and documented in `docs/EXTENDED_VALIDATION.md`.

## Synchronized multi-seed batches

For simultaneous cross-board KS/REA/TTFF comparison, use the batch wrapper so every realization shares one physical GPST window and one common base configuration while receiving a distinct deterministic seed:

```bash
python3 tools/batch_generation/generate_batch.py \
  --simulator build/gnss-data-simulator \
  --config config/default_v1.json \
  --nav /path/to/real_broadcast_nav.rnx \
  --output-dir output/batch_001 \
  --week 2347 \
  --sow 436500
```

The default batch is KS x8 + REA x8 + TTFF x8 with seed ranges `1001..1008`, `2001..2008`, and `3001..3008`. See [`docs/SYNCHRONIZED_BATCH_GENERATION.md`](docs/SYNCHRONIZED_BATCH_GENERATION.md) for the invariant-time contract, manifest layout, reproducibility rules, and CI validation.

## Design and V1 behavior

- [`docs/DESIGN_SPEC.md`](docs/DESIGN_SPEC.md) — architecture and V1 scope.
- [`docs/ENGINEERING_RULES.md`](docs/ENGINEERING_RULES.md) — repository layout, coding rules, CI policy, and test principles.
- [`docs/NAV_RECORDS.md`](docs/NAV_RECORDS.md) — supported NovAtel OEM7 and Unicore N4 navigation records.
- [`docs/V1_DEFAULTS.md`](docs/V1_DEFAULTS.md) — frozen V1 default parameters.
- [`docs/STARTUP_RECOVERY_MODEL.md`](docs/STARTUP_RECOVERY_MODEL.md) — HOT/WARM/COLD TTFF and REA recovery model.
- [`docs/V1_ACCEPTANCE_MATRIX.md`](docs/V1_ACCEPTANCE_MATRIX.md) — short deterministic V1 acceptance coverage.
- [`docs/EXTENDED_VALIDATION.md`](docs/EXTENDED_VALIDATION.md) — 8-hour / 50 Hz streaming, determinism, and memory validation.
- [`docs/SYNCHRONIZED_BATCH_GENERATION.md`](docs/SYNCHRONIZED_BATCH_GENERATION.md) — synchronized x8 multi-seed batch generation and reproducibility contract.

## Validation data

The committed test suite includes compact deterministic fixtures, including a reduced real WHU `BRD400DLR` RINEX 4.02 broadcast-navigation fixture with provenance metadata. Normal and extended CI do not download live IGS data at test runtime.
