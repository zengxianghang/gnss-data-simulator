# Engineering Rules

Status: frozen implementation rules for V1 unless a later design decision explicitly changes them.

This document defines the required repository layout, source-file naming, C/C++ coding rules, module boundaries, GitHub Actions policy, and unit/integration test principles for `gnss-data-simulator`.

## 1. Build baseline

- Primary implementation language: C++17 with C-compatible/C-style module interfaces where practical.
- Primary build system: CMake.
- Primary Windows toolchain: Visual Studio 2022 / MSVC.
- CI secondary toolchain: GCC on Ubuntu.
- RTKLIB must be consumed through a fixed/pinned repository revision for a reproducible build. The simulator release/run manifest must record the exact RTKLIB commit SHA.
- Project code must compile cleanly on both Windows and Linux. Platform-specific code must be isolated behind a narrow adapter and must not leak into the GNSS physics/model layers.
- Production code must not require a network connection at runtime. Downloading IGS data is a separate tooling concern.

## 2. Frozen repository layout

The implementation must use the following top-level layout. New directories require an explicit design reason.

```text
/
├─ CMakeLists.txt
├─ README.md
├─ .clang-format
├─ .gitignore
├─ cmake/
│  └─ CompilerWarnings.cmake
├─ config/
│  └─ default_v1.json
├─ docs/
│  ├─ DESIGN_SPEC.md
│  ├─ ENGINEERING_RULES.md
│  ├─ NAV_RECORDS.md
│  ├─ STARTUP_RECOVERY_MODEL.md
│  └─ V1_DEFAULTS.md
├─ include/
│  └─ gnss_sim/
│     ├─ sim_config.h
│     ├─ sim_time.h
│     ├─ sim_types.h
│     └─ simulator.h
├─ src/
│  ├─ app/
│  │  └─ main.cpp
│  ├─ core/
│  │  ├─ sim_config.cpp
│  │  ├─ sim_time.cpp
│  │  ├─ simulator.cpp
│  │  └─ deterministic_rng.cpp
│  ├─ gnss/
│  │  ├─ rtklib_adapter.cpp
│  │  ├─ signal_definitions.cpp
│  │  ├─ satellite_engine.cpp
│  │  ├─ navigation_state.cpp
│  │  └─ nav_message_scheduler.cpp
│  ├─ model/
│  │  ├─ receiver_truth.cpp
│  │  ├─ atmosphere_model.cpp
│  │  ├─ measurement_model.cpp
│  │  ├─ cn0_model.cpp
│  │  └─ signal_tracking.cpp
│  ├─ scenario/
│  │  └─ scenario_engine.cpp
│  ├─ solution/
│  │  └─ solution_engine.cpp
│  └─ output/
│     ├─ novatel_range_writer.cpp
│     ├─ novatel_solution_writer.cpp
│     ├─ novatel_nav_writer.cpp
│     ├─ unicore_nav_writer.cpp
│     └─ truth_writer.cpp
├─ tests/
│  ├─ unit/
│  │  ├─ test_sim_time.cpp
│  │  ├─ test_deterministic_rng.cpp
│  │  ├─ test_signal_definitions.cpp
│  │  ├─ test_satellite_engine.cpp
│  │  ├─ test_measurement_model.cpp
│  │  ├─ test_signal_tracking.cpp
│  │  ├─ test_navigation_state.cpp
│  │  ├─ test_nav_message_scheduler.cpp
│  │  ├─ test_scenario_engine.cpp
│  │  └─ test_output_writers.cpp
│  ├─ integration/
│  │  ├─ test_static_loopback.cpp
│  │  ├─ test_startup_modes.cpp
│  │  ├─ test_rea.cpp
│  │  ├─ test_week_rollover.cpp
│  │  └─ test_nav_updates.cpp
│  ├─ data/
│  │  ├─ README.md
│  │  └─ minimal/
│  └─ golden/
│     ├─ novatel/
│     └─ unicore/
├─ tools/
│  ├─ build_cn0_model/
│  └─ download_igs/
├─ third_party/
│  └─ RTKLIB/
└─ .github/
   └─ workflows/
      ├─ ci.yml
      └─ extended.yml
```

### Layout rules

- `include/gnss_sim/` contains only public interfaces needed by `main.cpp`, tests, or future external users.
- Internal module declarations should remain private to `src/` unless they genuinely form part of the public simulator API.
- Do not create a generic `utils.cpp` or `common.cpp`. Functions must live in the module that owns their semantics.
- Do not create one monolithic `gnss_simulator.cpp` containing multiple subsystems.
- Do not put formatter logic into measurement/physics modules.
- Do not put scenario-specific branches directly into RTKLIB adapters or observation equations.
- `tools/` code is not linked into the runtime simulator executable.
- Test input files must be small, deterministic, and checked in under `tests/data/` unless licensing/size prevents it.

## 3. File naming

- Source/header names: lower snake case, for example `measurement_model.cpp`.
- Test source names: `test_<module>.cpp`.
- Configuration files: lower snake case, for example `default_v1.json`.
- Markdown specification names: upper snake case, for example `DESIGN_SPEC.md`.
- Do not use spaces in repository file names.
- One production `.cpp` file should normally own one primary module responsibility.
- If a module becomes too large, split by responsibility rather than adding `_part1`, `_part2`, or numeric suffixes.

## 4. C/C++ coding style

### 4.1 General style

Production code should remain close to C-style C++:

- Prefer plain structs and free functions over deep class hierarchies.
- Avoid inheritance in V1.
- Avoid exceptions in production code. Return explicit status/error values.
- Avoid RTTI-dependent designs.
- Keep control flow simple and explicit.
- Declare variables close to first use.
- Avoid hidden side effects.
- Avoid mutable global state.
- State that changes during simulation must be owned by an explicit simulator/context structure.
- Large working buffers must not be allocated repeatedly inside the per-epoch/per-signal hot loop.
- Prefer preallocation/reuse for data structures whose maximum useful size is known.
- Runtime generation must be streaming. The program must not retain the full 8-hour output dataset in memory.

### 4.2 STL policy

- STL may be used at configuration/file/API boundaries when it materially simplifies safe code.
- The high-frequency epoch/satellite/signal processing path should prefer fixed-size arrays, preallocated storage, or reusable vectors with reserved capacity.
- Do not repeatedly construct strings, maps, vectors, streams, or regex objects inside the inner observation loop.
- Do not rely on unordered-container iteration order for any output that participates in deterministic regression testing.

### 4.3 Naming

Use the following conventions:

```text
Types/structs/enums:        PascalCase
Functions:                  lower_snake_case
Variables/fields:           lower_snake_case
Compile-time constants:     UPPER_SNAKE_CASE
Enum values:                UPPER_SNAKE_CASE
Files:                      lower_snake_case
```

Examples:

```cpp
struct ReceiverTruthState;
enum class StartupMode { HOT, WARM, COLD };

bool load_rinex_nav(...);
double geometric_range_m;
constexpr int MAX_SIM_SIGNALS = 256;
```

### 4.4 Units

Physical units must be explicit in variable/field names whenever ambiguity is possible.

Preferred suffixes:

```text
_ns       nanoseconds
_sec      seconds
_m        meters
_mps      meters/second
_mps2     meters/second^2
_hz       hertz
_deg      degrees
_rad      radians
_dbhz     dB-Hz
_cycles   carrier cycles
```

Examples:

```cpp
int64_t epoch_time_ns;
double pseudorange_m;
double doppler_hz;
double elevation_deg;
double clock_drift_mps;
```

Do not use generic names such as `time`, `range`, `speed`, or `freq` in physics code when the unit is not obvious from the immediate scope.

### 4.5 Headers and interfaces

- Headers must be self-contained.
- Use include guards of the form `GNSS_SIM_<PATH>_H_`.
- Minimize transitive includes.
- Do not expose RTKLIB internal structures in the public `include/gnss_sim/` API.
- RTKLIB types should be contained inside `rtklib_adapter` / navigation implementation boundaries.
- Prefer `const` input pointers/references when data is read-only.
- Functions should return explicit success/failure status rather than silently falling back when input data is invalid.

### 4.6 Error handling

- Invalid configuration: fail before simulation starts with a clear message.
- Missing required RINEX coverage: fail before simulation starts.
- Internal impossible state or violated invariant: fail loudly; do not silently continue and emit plausible-looking data.
- Expected receiver state such as `INSUFFICIENT_OBS/NONE` is simulation output, not an application error.
- Output-write failure is fatal for the run.

### 4.7 Determinism

Determinism is a product requirement, not only a test convenience.

- All random behavior must use the project deterministic PRNG.
- Never use C `rand()` or implementation-defined library random behavior for simulation results.
- The seed must be explicit and recorded in `run_manifest.json`.
- Output ordering must be stable.
- Floating-point formatting must not depend on system locale.
- A run manifest must record simulator version/commit, RTKLIB commit, config, seed, and input file identity/hash as practical.
- Same simulator revision + same RTKLIB revision + same config + same input + same seed must reproduce the same numerical result. Byte-identical output should be the goal for deterministic text outputs.

## 5. Frozen module ownership

### `sim_config`
Owns configuration parsing/defaults/validation only.

### `sim_time`
Owns integer simulation scheduling, GPST conversion, week rollover, epoch alignment, and event-to-epoch rules.

### `deterministic_rng`
Owns the single specified PRNG algorithm and deterministic sampling helpers.

### `rtklib_adapter`
Owns all direct interactions with RTKLIB, including RINEX NAV loading, satellite state calls, coordinate/time helpers, and SPP/velocity calls.

### `signal_definitions`
Single source of truth for constellation/signal/RINEX-code/RTKLIB-code/frequency/group-delay/OEM signal mappings.

No formatter or scenario module may maintain a second independent signal mapping table.

### `satellite_engine`
Owns transmission-time iteration, satellite state at transmit time, LOS, azimuth/elevation, geometric range, and range rate.

### `navigation_state`
Owns Truth NAV vs Receiver NAV state and usable navigation-data lifecycle.

### `nav_message_scheduler`
Owns COLD navigation fragment/page/message acquisition timing and EPH completion logic.

### `receiver_truth`
Owns receiver truth position/velocity/clock state. V1 clock bias/drift are zero, but the fields remain explicit.

### `atmosphere_model`
Owns ionosphere/troposphere corrections and `none`/broadcast modes.

### `measurement_model`
Owns final PSR/Doppler/ADR physical equations. It does not own acquisition state or text formatting.

### `cn0_model`
Owns system+signal+elevation CN0 lookup/interpolation and fallback behavior.

### `signal_tracking`
Owns SIGNAL_OFF/SEARCHING/ACQUIRING/TRACKING, lock time, valid flags, and acquisition/reacquisition delays.

### `scenario_engine`
Owns KS/REA/TTFF event scheduling and power/signal state. It must not contain satellite-orbit or observation equations.

### `solution_engine`
Owns construction of RTKLIB observations from simulated measurements and generation of PSRPOS/PSRVEL solution state.

### Output writers
Writers only serialize already-computed state. They must not recalculate physics or invent receiver state.

## 6. CMake target rules

Use explicit targets:

```text
gnss_sim_core       library containing simulator production logic
gnss_data_simulator executable containing only CLI/app entry logic
gnss_sim_tests      unit/integration test executable(s)
```

Third-party RTKLIB warnings must not be treated as project-code warnings.

Project code should use strict warnings:

```text
MSVC: /W4
GCC/Clang: -Wall -Wextra -Wpedantic
```

New warnings introduced by project code are not acceptable. CI may promote project warnings to errors once the initial build baseline is clean.

## 7. GitHub Actions policy

Two workflows are frozen for V1.

### `.github/workflows/ci.yml`

Purpose: required fast PR/main validation.

Triggers:

```text
pull_request
push to main
workflow_dispatch
```

Required jobs:

1. Windows / MSVC build + tests.
2. Ubuntu / GCC build + tests.
3. Formatting check.

Required fast test scope:

- all unit tests;
- small deterministic integration tests;
- representative GPS-only and multi-GNSS zero-noise loopback;
- output golden tests;
- startup/REA state-machine tests using short synthetic durations;
- week-rollover boundary test.

Rules:

- PR CI must not generate the normal 8-hour dataset.
- Tests must not depend on live IGS/network availability.
- CI must use checked-in minimal test fixtures.
- A test failure must fail the workflow; no `continue-on-error` for required checks.
- Avoid retry loops that hide flaky tests.
- Cancel stale in-progress CI runs for the same PR when a newer commit is pushed.
- Do not upload large generated simulator logs by default.
- On failure, small diagnostic logs/test reports may be uploaded as artifacts.

### `.github/workflows/extended.yml`

Purpose: expensive/full validation.

Triggers:

```text
workflow_dispatch
scheduled run
```

Extended scope should include:

- longer multi-GNSS runs;
- all supported sampling rates 1/5/10/20/50 Hz;
- all supported signal mappings;
- HOT/WARM/COLD/REA scenario regression;
- navigation update behavior;
- deterministic same-seed repeat comparison;
- 8-hour default-run smoke/regression when runtime cost is acceptable;
- memory-growth/streaming validation;
- optional sanitizer job on Linux.

Extended CI is not a substitute for fast PR CI. A PR must remain testable without waiting for an 8-hour-generation job.

## 8. Unit test principles

### 8.1 General rules

- Every deterministic algorithm/module must have unit tests before or together with its implementation.
- Test public behavior and important invariants, not private implementation trivia.
- A bug fix must add a regression test that fails before the fix and passes after it.
- Tests must be deterministic and independent of execution order.
- Unit tests must not use current wall-clock time, online services, random unrecorded seeds, or machine-specific paths.
- Do not loosen numerical tolerances merely to make a failing physics test pass. First determine the physical/numerical reason for the difference.
- Tests should be small enough that developers run them locally before every PR.

### 8.2 Test framework

- Use GoogleTest for C++ unit/integration tests.
- Integrate tests through CTest.
- Pin the GoogleTest revision/version just like other build dependencies.
- Test-only code may use STL/features more freely than production hot-path code.

### 8.3 Required unit coverage categories

#### Time

- 1/5/10/20/50 Hz exact tick intervals;
- no cumulative floating-time drift;
- GPST week crossover;
- event effective at first epoch where `epoch_time >= event_time`.

#### Signal definitions

- every supported constellation/signal maps to exactly one intended RINEX/RTKLIB/OEM definition;
- GLONASS G1/G2 FCN-dependent wavelength;
- no duplicate or conflicting mapping.

#### Satellite geometry

- transmit-time iteration converges;
- azimuth/elevation/range against trusted RTKLIB/reference cases;
- 3 degree visibility mask boundary.

#### Measurement model

- zero-noise PSR equation term-by-term;
- Doppler/range-rate sign convention;
- ADR continuity while tracking;
- correct ionosphere sign between code and carrier;
- TGD/ISC/BGD applied only to the intended signal.

#### Tracking

- lock time increments only under continuous tracking;
- loss of signal resets lock/ADR validity as specified;
- HOT/WARM/COLD acquisition timing is deterministic for fixed seed/config;
- REA recovery does not clear Receiver NAV.

#### Navigation

- Truth NAV and Receiver NAV never alias accidentally;
- HOT/WARM restore cached EPH/ION;
- COLD does not expose future/full Truth NAV before fragments complete;
- IOD consistency checks;
- RINEX NAV update causes correct Receiver NAV/log update.

#### Output

- exact OEM7/N4 record family names;
- exact status/type transitions including `INSUFFICIENT_OBS/NONE`;
- numeric formatting/field ordering;
- CRC/checksum behavior where applicable;
- golden output lines compared byte-for-byte where practical.

## 9. Integration/acceptance test principles

### Zero-noise static loopback

Given the same broadcast NAV/corrections and static truth, simulated observations processed by RTKLIB must recover the truth within a tight numerical tolerance appropriate to the exact model path.

This is the primary physics correctness oracle for V1.

### Position/velocity independence

PSRPOS and PSRVEL must be produced from the simulated measurements/Receiver NAV. Integration tests must catch any accidental direct use of receiver truth to fill solution outputs.

### Startup modes

Tests must verify state semantics, not only TTFF number:

- HOT/WARM EPH available immediately from receiver cache;
- COLD EPH progressively available through navigation-message collection;
- TTFF occurs only once actual solution conditions are met.

### REA

During SIGNAL OFF:

```text
RANGEA count = 0
PSRPOSA = INSUFFICIENT_OBS / NONE
PSRVELA = INSUFFICIENT_OBS / NONE
Receiver NAV retained
GPST continuous
```

After SIGNAL ON, observations and solution must recover progressively according to tracking state.

### Determinism

Run the same short scenario twice with the same config/input/seed and compare hashes or exact output bytes. Run with a different seed and verify only intentionally stochastic timing/state fields change.

### Cross-platform

Numerical physics results should agree within defined floating-point tolerances between MSVC and GCC. Text-format golden tests should be designed to remain platform-independent (explicit line endings/locale/formatting).

## 10. Test data policy

- Keep CI fixtures minimal; do not commit GB-scale IGS/RINEX files.
- Each fixture must document its source and purpose in `tests/data/README.md`.
- Prefer the smallest RINEX excerpt that still exercises the intended condition.
- Golden files are specifications and must not be regenerated automatically during CI.
- Updating a golden file requires review of why the external format changed.

## 11. Pull request rules

Every implementation PR must include:

- concise scope;
- linked issue when applicable;
- implementation notes describing important design choices;
- tests added/updated;
- local test command/results;
- any format/config compatibility impact.

A PR is not complete when only the new code works manually. The corresponding deterministic tests and documentation updates are part of the implementation.

Keep PRs focused. Avoid combining unrelated formatter, physics, scenario, and CI refactors in one PR unless the dependency is unavoidable.

## 12. Commit rules

Use short English imperative/conventional-style commit subjects, for example:

```text
feat: add GPS LNAV cold-start scheduler
test: add week-rollover timing coverage
fix: preserve receiver nav during REA outage
docs: define V1 output record rules
ci: add Windows and Ubuntu test matrix
```

Do not use vague messages such as `update`, `fix bug`, or `changes`.

## 13. Definition of done for V1 implementation work

A module/change is done only when all applicable items are true:

1. code follows the frozen directory/file/module ownership;
2. public behavior is documented where necessary;
3. unit tests cover deterministic behavior and boundaries;
4. integration/regression tests cover cross-module behavior where applicable;
5. local CMake build and CTest pass;
6. Windows/MSVC CI passes;
7. Ubuntu/GCC CI passes;
8. no new project-code warnings;
9. deterministic outputs remain reproducible;
10. no 8-hour in-memory buffering or other obvious duration-proportional memory growth is introduced.
