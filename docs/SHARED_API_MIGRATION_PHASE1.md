# Shared RTKLIB API migration — Phase 1 (Issue #102)

Status: **Phase 1 complete** (inventory + old-vs-shared parity baseline +
safe migration map). No generic adapter code was removed and no simulator
behavior changed. This document is the input for the incremental migration
batches that follow; it does not authorize any removal by itself.

## 1. Pinned revision and ABI provenance

The RTKLIB submodule was advanced to the revision that carries the shared
GNSS adapter API (fork PR #17 / #25):

```text
RTKLIB commit SHA      767337d1668afe761e3e2f1042c0d2442b3d805b
shared adapter ABI     1.0 (numeric 0x00010000)
```

The full simulator suite passes on the new pin (375/375 including the new
parity tests), so the pin update is backward-compatible with every existing
adapter call.

### 1.1 Windows path-separator correction on the pin

The Phase-1 baseline exposed a Windows-only defect in the shared RINEX loader:
`expath()` recognizes only `\` as a directory separator, so an absolute path
supplied with `/` was reduced to a bare basename and the load failed with
`RTKLIB_SHARED_IO_ERROR`. The simulator's own adapter had always normalized
separators through `rtklib_file_path()`, which is why only the shared path
failed. The correction normalizes separators at the shared API boundary
(`zengxianghang/RTKLIB` PR #26, commit
`767337d1668afe761e3e2f1042c0d2442b3d805b`), so supported RINEX NAV input
loads identically on Windows, Linux and macOS; the public ABI, source
identity and fail-closed statuses are unchanged. `rtklib_shared_api.c` is compiled into the existing
`rtklib_pinned` target; no second RTKLIB build exists.

## 2. Old-vs-shared parity baseline (established, green)

`tests/integration/test_shared_api_parity.cpp` loads the same real RINEX
NAV fixture (`tests/data/minimal/mixed_nav_2019.rnx`, reduced real Stanford
BRDM: GPS/GLONASS/Galileo/BeiDou/QZSS) through both paths and compares:

| Comparison | Result |
| --- | --- |
| Linked shared ABI | 1.0 (0x00010000) verified |
| Record sets (kind/system/prn/iode/iodc/toe) | identical on every record |
| Satellite state on unique-record satellites (GPS, GLO, GAL, BDS) | max position/velocity/clock/drift delta = 0.0 (bit-exact) |
| Coverage | ≥1 comparison per system, all four systems green |

Interpretation: both paths compile the same pinned RTKLIB core, so the
parity is bit-exact where the two APIs expose the same semantics. Two
boundary findings are recorded for the migration design:

- The shared loader fails closed on records outside its system vocabulary
  (the `multi_gnss_acceptance_nav.rnx` fixture contains IRNSS/SBAS records
  the shared API does not classify). Fixtures used for shared-API parity
  must stay inside the supported system set.
- The shared signal/state queries derive a family mask from the RINEX
  observation code, and the shared code table differs from RTKLIB's raw
  `obs2code` numbering for some codes (Galileo accepts `1C`, not `1B`/`5I`,
  with mask `INAV|FNAV`). Callers must resolve codes through the shared
  signal query semantics, exactly as the analyzer does.

## 3. Function-level inventory and classification

Classification vocabulary: `GENERIC_RTKLIB_INTEGRATION` (candidate for the
shared API) versus `SIMULATOR_SEMANTIC` (stays local). Line numbers refer to
the Phase-1 head.

### 3.1 `rtklib_adapter.cpp` / `rtklib_adapter.h`

| Function | Site | Class |
| --- | --- | --- |
| `create_rtklib_nav_store` / `destroy_rtklib_nav_store` | adapter.cpp:206/210 | GENERIC |
| `load_rinex_nav_file` | adapter.cpp:218 | GENERIC |
| `get_rtklib_nav_counts` / `rtklib_nav_record_count` / `rtklib_nav_record_info` | adapter.cpp:246/267/274 | GENERIC |
| `rtklib_clear_nav_store` / `rtklib_copy_nav_snapshot` / `rtklib_copy_nav_record` / `rtklib_nav_store_has_satellite_ephemeris` | adapter.cpp:325/333/410/443 | GENERIC |
| `rtklib_satellite_id_to_number` (adapter) / `rtklib_satellite_number_to_id` (output_adapter.cpp:10) | — | GENERIC |
| `rtklib_observation_code` | adapter.cpp:472 | GENERIC |
| `rtklib_satellite_state_available` | adapter.cpp:486 | GENERIC |
| `get_rtklib_satellite_state_with_selection_time` (teph-preserving) | adapter.cpp:499 | GENERIC |
| `get_rtklib_satellite_state` | adapter.cpp:532 | GENERIC |
| `get_rtklib_signal_satellite_state` (family-restricted + identity) | adapter.cpp:538 | GENERIC |
| `rtklib_broadcast_ionosphere_model_state` | adapter.cpp:652 | GENERIC |
| `rtklib_llh_to_ecef` / `rtklib_ecef_to_llh` / `rtklib_geometric_distance` / `rtklib_azimuth_elevation` | adapter.cpp:689/701/714/728 | GENERIC |
| `rtklib_signal_health_for_family` | bias_adapter.cpp:256 | GENERIC |
| `rtklib_broadcast_bias_data_for_family` / `rtklib_broadcast_bias_data` | bias_adapter.cpp:180/250 | GENERIC |
| `rtklib_solve_single_position` | solution_adapter.cpp:217 | GENERIC (bias-bridge caveat, §5) |
| `rtklib_solve_single_velocity` | solution_adapter.cpp:308 | GENERIC |
| `rtklib_raw_code_observation_navigation_available` / `rtklib_solve_raw_single_position` | raw_position_adapter.cpp:103/126 | GENERIC |
| `rtklib_broadcast_ionosphere_reference_delay` / `rtklib_troposphere_delay` | atmosphere_adapter.cpp:38/87 | GENERIC |

### 3.2 Serialization-shaped surfaces (mixed)

| Function | Site | Class | Note |
| --- | --- | --- | --- |
| `rtklib_nav_output_record_count` / `rtklib_nav_output_record` | nav_output_adapter.cpp:312/321 | SIMULATOR_SEMANTIC | `NavOutputRecord` is a simulator serialization shape consumed by the EPH/ION writers and tools; the decode internals are generic |
| `rtklib_append_nav_output_record` | nav_input_adapter.cpp | SIMULATOR_SEMANTIC | inverse of the above |
| `nav_output_system_name` / `finalize_nav_output_record_metadata` | nav_output_adapter.cpp:362 / nav_output_metadata.cpp:52 | SIMULATOR_SEMANTIC | display/post-decode fill |

### 3.3 Simulator-owned (stay local, no migration)

- `navigation_state.*` — Truth NAV vs Receiver NAV machine, HOT/WARM/COLD
  snapshots, delivery bookkeeping (all functions).
- `nav_message_scheduler.*` — cold-acquisition fragment plans per family,
  delivery gating (all functions).
- `satellite_engine.*` — `compute_satellite_geometry*` truth-path transmit
  iteration vs `ReceiverTruth` (`subtract_propagation_time` and
  `elevation_passes_mask` are pure geometry helpers, GENERIC but trivial).
- `galileo_has_adapter.*` — HAS truth feature (`readsp3`/`readrnxc`/`peph2pos`
  are generic primitives underneath).
- `signal_definitions.*` — `signal_definitions`/`find_signal_definition*`/
  `signal_carrier_frequency_hz`/`signal_wavelength_m`/
  `signal_rtklib_observation_code` are GENERIC metadata;
  `validate_code_correlation_profile`, `signal_has_supported_code_correlation`
  and `signal_single_point_priority` are SIMULATOR_SEMANTIC (ideal-DLL
  correlation model and SPP-loopback signal policy).
- `rtklib_rinex_obs_ext.c` — build-time shim (see §5 risk notes).

## 4. Safe migration map (next batches, in order)

1. **Family taxonomy consolidation (behavior-preserving, no API change).**
   Reconcile `RtklibBroadcastMessageFamily` (11 members) with
   `NavMessageFamily` (15 members); collapse the three family→`NAV_*` mask
   mappings (solution_adapter.cpp:71, raw_position_adapter.cpp:28,
   adapter.cpp:553-590) into one; unify the two message-family classifiers
   (bias_adapter.cpp:102, nav_output_adapter.cpp:42). Pure refactor guarded
   by the existing suites.
2. **RINEX load + store lifecycle.** Replace `load_rinex_nav_file`/
   store create/destroy/introspection with the shared store API once the
   `RtklibNavStore` handle is unified (today seven adapter TUs each define
   `struct RtklibNavStore { nav_t nav; }` privately; the shared API owns the
   canonical handle). Keep `rtklib_copy_nav_snapshot` local if the shared
   API has no equivalent — it serves startup snapshots (simulator semantic
   over a generic store copy).
3. **State/identity/bias/signal metadata.** Replace `satpos`-based state,
   selected-identity and TGD/ISC/BGD/GLO calls with shared
   state/bias/signal queries, family by family, starting with GPS LNAV (the
   analyzer-parity-covered scope). Preserve the fixed observation-epoch
   ephemeris-selection (`teph`) semantics from the transmit-time
   convergence fix — the parity test probes `teph = epoch`.
4. **Atmosphere helpers.** `ionmodel`/`tropmodel` wrappers are trivial
   shared-API candidates once ion-state access exists.
5. **SPP/velocity.** Migrate `rtklib_solve_raw_single_position` first; keep
   `rtklib_solve_single_position`'s bias bridge (`solver_pseudorange_m`,
   `legacy_prange_adjustment_m`) local unless the shared API grows the same
   bridge — migrating the solver without it breaks equivalence.
6. **Serialization-shaped surfaces last** (§3.2) — they depend on the
   simulator-owned `NavOutputRecord` schema; only their generic internals
   (system/msg_type→family classification) move.

Not scheduled: `navigation_state`, `nav_message_scheduler`,
`satellite_engine` truth geometry, `galileo_has_adapter`,
`signal_single_point_priority` (simulator-owned, §3.3).

## 5. Risk notes carried into the next batch

- `NEXOBS=16`, the commit-string macro and the `rtklib_rinex_obs_ext.c`
  macro-shadowing shim (recompiles `rinex.c` with fork `obs2code_ext`) must
  travel with any build-system change; the shared API builds the same
  `rinex.c` once, so the shim's role needs re-evaluation at step 2.
- Shared-loader system vocabulary is fail-closed (no IRNSS/SBAS); fixtures
  and any production NAV input must stay inside the supported set or the
  simulator must keep its legacy loader for those inputs.
- Determinism contract (`docs/ENGINEERING_RULES.md`): same RTKLIB revision +
  config → identical output. The shared selector's age-then-toc tie-break
  and GPST conventions (GLONASS IODC=0, Toc=Toe) must be preserved exactly;
  the parity baseline plus the adapter unit suites are the safety net for
  each step.
