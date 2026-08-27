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

## Validation data

The committed test suite includes compact deterministic fixtures, including a reduced real WHU `BRD400DLR` RINEX 4.02 broadcast-navigation fixture with provenance metadata. Normal and extended CI do not download live IGS data at test runtime.
