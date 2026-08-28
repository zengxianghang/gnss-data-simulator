# Engineering Rules

Status: frozen implementation rules for V1 unless a later design decision explicitly changes them.

This document defines the required repository layout, source-file naming, C/C++ coding rules, module boundaries, GitHub Actions policy, and unit/integration test principles for `gnss-data-simulator`.

## 1. Build baseline

- Primary implementation language: C++17 with C-compatible/C-style module interfaces where practical.
- Primary build system: CMake.
- Primary Windows toolchain: Visual Studio 2022 / MSVC.
- CI secondary toolchain: GCC on Ubuntu.
- RTKLIB must be consumed through a fixed/pinned repository revision for a reproducible build. The simulator release/run manifest must record the exact RTKLIB commit SHA.
- JSON configuration parsing uses the pinned cJSON dependency through `sim_config`; cJSON types must not leak outside that module boundary.
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
│  │  ├─ test_sim_config.cpp
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
│  ├─ RTKLIB/
│  └─ cJSON/
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
std::int64_t epoch_time_ns;
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
- cJSON types must remain private to `sim_config.cpp`.
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
Owns configuration parsing/defaults/validation only. The JSON parser is an implementation detail and must not leak into other modules.

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

### `receiver_truth`
Owns receiver truth position/velocity/clock state and future trajectory interpolation.

### `atmosphere_model`
Owns ionosphere and troposphere truth/correction models.

### `measurement_model`
Owns pseudorange, Doppler, carrier phase/ADR generation and signal-specific correction application.

### `cn0_model`
Owns CN0-vs-elevation runtime lookup/model evaluation.

### `signal_tracking`
Owns per-signal acquisition/tracking/lock/validity state.

### `navigation_state`
Owns Truth NAV and Receiver NAV stores and their update/availability state.

### `nav_message_scheduler`
Owns COLD-start frame/subframe/page/message fragment acquisition timing.

### `scenario_engine`
Owns KS/REA/TTFF events only.

### `solution_engine`
Owns conversion of generated observations + Receiver NAV into RTKLIB SPP/velocity results.

### output writers
Own formatting/serialization only. Writers must not recompute geometry, navigation corrections, tracking state, or solution validity.

## 6. GitHub Actions policy

Normal PR/push CI is fast and mandatory:

```text
.github/workflows/ci.yml
```

It must include:

- formatting check;
- Windows / Visual Studio 2022 build;
- Ubuntu / GCC build;
- unit tests;
- short deterministic integration tests when they exist;
- no live IGS download;
- no default 8-hour generation run.

Long/expensive validation belongs in:

```text
.github/workflows/extended.yml
```

It is intended for:

- `workflow_dispatch` manual execution;
- scheduled execution if useful;
- 1/5/10/20/50 Hz matrix;
- full constellation/signal matrix;
- HOT/WARM/COLD;
- REA;
- cross-week tests;
- deterministic rerun checks;
- long-duration streaming/memory tests;
- default 8-hour run;
- selected 50 Hz runs.

Do not make ordinary documentation-only changes consume an 8-hour simulation test.

## 7. Unit-test principles

Use GoogleTest for C/C++ unit tests and register tests through CTest.

### 7.1 General rules

- One production module should have focused unit tests.
- Every bug fix must add a regression test that fails before the fix and passes after it.
- Tests must not depend on current wall-clock time, network availability, machine locale, or unspecified random state.
- Test random behavior with explicit fixed seeds.
- Do not weaken numerical tolerances merely to make CI pass.
- Test both normal operation and boundary/error conditions.
- Parser/formatter tests should use small representative fixtures rather than GB-sized logs.
- Unit tests should normally complete in seconds.

### 7.2 Deterministic physical model tests

For numerical physics functions:

1. Test a simple analytically checkable case where possible.
2. Test against RTKLIB/reference calculations where RTKLIB owns the reference algorithm.
3. Use explicitly justified absolute/relative tolerances.
4. Include constellation-specific edge cases, especially GLONASS FDMA.

### 7.3 State-machine tests

State-machine tests must check transitions and invariants, not only final values.

Examples:

- lock time resets on signal loss;
- ADR validity resets on power/signal loss as designed;
- Receiver NAV is retained during REA;
- Receiver NAV is empty at COLD startup;
- EPH becomes available only after required NAV fragments;
- HOT/WARM EPH is available immediately;
- PSRPOS and PSRVEL validity can differ.

### 7.4 Output/golden tests

For NovAtel OEM7 and Unicore N4 formatters:

- maintain small golden files under `tests/golden/`;
- compare the complete serialized line/record where practical;
- field formatting/precision/ordering is part of the test contract;
- changes to golden output require explicit review and a reason;
- golden files must never be regenerated silently merely because a test failed.

## 8. Integration-test principles

Integration tests verify module boundaries and physical consistency.

Required V1 integration coverage eventually includes:

- GPS-only ideal static loopback;
- multi-GNSS ideal static loopback;
- all supported signals;
- RTKLIB SPP recovery of the configured receiver position;
- static velocity recovery near zero;
- HOT/WARM/COLD startup;
- REA signal off/on;
- Truth NAV vs Receiver NAV isolation;
- COLD EPH progressive acquisition;
- EPH/ION runtime updates;
- GPST week rollover;
- 1/5/10/20/50 Hz;
- deterministic same-seed repeatability.

Normal CI uses short fixtures/durations. Long-duration versions of these tests belong in extended CI.

## 9. Test data policy

- Keep only minimal necessary test data in Git.
- Do not commit multi-GB IGS archives.
- Every fixture should document its origin and any transformation applied.
- Prefer synthetic minimal RINEX fixtures for parser boundary tests if they faithfully exercise the target format.
- Real IGS snippets may be used when needed for reference/compatibility testing and redistribution is appropriate.
- CI tests must not download test data from the internet during the test itself.

## 10. Pull request and commit rules

- Every independently actionable bug, feature, validation gap, or engineering problem must be tracked by a dedicated GitHub Issue before implementation begins.
- Issues should be kept as small and independently implementable, verifiable, and reviewable as practical. If an Issue contains multiple independently actionable concerns, split it into smaller Issues before implementation.
- Prefer a strict one-Issue-to-one-PR workflow: one Issue defines one implementation scope, and one PR closes that Issue.
- A PR must not bundle unrelated Issues or opportunistic fixes. Newly discovered unrelated problems must be recorded as separate Issues and handled by separate PRs.
- For a large feature or engineering goal that requires multiple implementation stages, use a parent Issue for the overall goal and small child Issues/work packages for the independently reviewable stages. Each child Issue should normally have its own PR.
- One PR should primarily solve one issue/work package.
- Keep PRs reviewable; avoid one giant V1 implementation PR.
- PR descriptions must include:
  - what changed;
  - why;
  - tests run;
  - any limitations/follow-up issues;
  - Implementation Notes relevant to the issue.
- Do not mix broad formatting/refactoring with unrelated GNSS behavior changes.
- Update design/engineering documentation in the same PR when behavior or a frozen interface changes.
- CI must pass before merge.
- Do not bypass failing deterministic tests without root-cause analysis.

## 11. Review checklist

Before merge, verify:

- module ownership is respected;
- no duplicate signal mapping was introduced;
- Truth NAV and Receiver NAV remain separate;
- no hidden random source was introduced;
- units are explicit;
- no full-run unbounded memory accumulation was introduced;
- output writers only serialize;
- tests cover the new behavior;
- Windows and Linux builds remain green;
- long tests have not leaked into normal PR CI.