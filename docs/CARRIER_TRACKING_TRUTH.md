# Carrier-tracking truth diagnostics

Issue: #162  
Parent design: #159  
Runtime integration: #161

## Purpose and ownership

`carrier_tracking_truth.csv` is a separate, explicitly versioned diagnostic extension for the receiver carrier-tracking layer. It is intentionally separate from `observation_truth.csv` and `urban_signal_truth.csv` so the physical-truth boundary established before carrier tracking remains unchanged.

The simulator assembles a `CarrierTrackingTruthSnapshot` from the exact in-memory objects used by runtime. The CSV writer only serializes that snapshot. It does not recompute PLL/FLL state, hysteresis or persistence, jitter, CN0, urban propagation, validity, ambiguity, or measurement noise, and it never consumes random numbers.

The observation chain represented by the file is:

```text
clean observation
    -> physical urban propagation (when enabled)
    -> physical observation / existing truth outputs
    -> carrier-tracking result
    -> post-carrier observation recorded here
    -> generic measurement-error layer (when enabled)
    -> normal RANGE output
```

Thus `physical_doppler_hz` and `physical_range_rate_mps` are values immediately before the carrier layer, while `post_carrier_doppler_hz` and `post_carrier_range_rate_mps` are values immediately after it and before generic measurement noise.

## Schema

`carrier_truth_schema_version` is currently `1`. The file is created for every simulator run, including when carrier tracking is disabled. Stable simulator satellite/signal iteration order gives deterministic row ordering.

### Identity and receiver state

- `gps_week`, `tow_ns`, `sow_sec`: receiver epoch.
- `satellite_number`, `signal_id`, `signal_name`, `glonass_fcn`: signal identity.
- `tracking_phase`, `acquisition_context`, `signal_loss_reason`: exact code/signal-tracker state used by runtime.
- `carrier_tracking_enabled`: resolved feature toggle.
- `carrier_result_available`: `1` only when code tracking is active and the carrier runtime was actually advanced for that epoch.
- `carrier_reset_reason`: `NONE`, `FEATURE_DISABLED`, or `CODE_NOT_TRACKING`. `CODE_NOT_TRACKING` records the boundary at which the carrier runtime is hard-reset after the diagnostic row is captured.

### Exact carrier input, state, and result

When `carrier_result_available=1`, these fields are copied directly from the same runtime result/state that is subsequently applied to the observation:

- `coherent_integration_sec`: configured carrier integration interval, seconds.
- `effective_cn0_dbhz`: exact C/N0 input used by the carrier runtime, dB-Hz.
- `cn0_linear_hz`: exact linear C/N0 stored in the runtime jitter result, Hz.
- `carrier_mode`: `CARRIER_UNLOCKED`, `FLL_TRACK`, or `PLL_TRACK`.
- `fll_phase`: `NONE`, `PULL_IN`, or `STEADY`.
- `active_bandwidth_hz`: active loop bandwidth, Hz.
- `phase_sigma_rad`: theoretical phase jitter stored by the core, radians.
- `sigma_hz`, `sigma_mps`: theoretical carrier tracking jitter scale in Hz and m/s.
- `correlation_tau_sec`, `correlation_alpha`: exact first-order correlation parameters used by the core.
- `tracking_error_hz`, `tracking_error_mps`: actual deterministic filtered carrier error applied at that epoch.
- `mode_age_sec`, `carrier_lock_age_sec`, `pll_age_sec`: exact post-update carrier state ages.
- `fll_enter_persistence_sec`, `fll_exit_persistence_sec`, `pll_enter_persistence_sec`, `pll_exit_persistence_sec`: exact post-update hysteresis/persistence accumulators.
- `carrier_segment_id`: core carrier-acquisition segment identifier.
- `phase_segment_id`, `adr_cycle_offset_cycles`: runtime carrier-phase continuity overlay from #161.
- `mode_changed`, `new_carrier_segment`, `cycle_slip_event`: exact runtime transition/event flags.

When no carrier result exists, result-only fields are blank rather than synthesized from config or other truth rows. `coherent_integration_sec` remains present because it is resolved configuration, not a reconstructed runtime result.

## Physical propagation and observation boundary

- `environmental_range_rate_applicable`: `1` when the urban physical propagation model is active for the run.
- `environmental_range_rate_valid`: exact validity from `UrbanCarrierTemporalResult`.
- `environmental_range_rate_mps`: exact physical urban path-rate term, m/s, and blank when it is not valid. Open-sky runs do not invent a zero-valued urban term; they mark it not applicable.

The physical observation block contains the exact observation before carrier tracking:

- `physical_snapshot_available`
- `physical_observation_available`
- `physical_code_valid`
- `physical_doppler_valid`
- `physical_adr_valid`
- `physical_range_rate_valid`
- `physical_range_rate_mps`
- `physical_doppler_hz`
- `physical_adr_cycles`

The post-carrier block contains the exact observation after `apply_carrier_tracking_runtime_result()` and before generic measurement noise:

- `post_carrier_snapshot_available`
- `post_carrier_observation_available`
- `post_carrier_code_valid`
- `post_carrier_doppler_valid`
- `post_carrier_adr_valid`
- `post_carrier_range_rate_valid`
- `post_carrier_range_rate_mps`
- `post_carrier_doppler_hz`
- `post_carrier_adr_cycles`

`range_rate_valid` follows the same receiver-carrier observable validity as Doppler because range rate is the wavelength-equivalent representation used by the simulator.

For a valid active carrier result, #161's sign convention remains directly checkable from one row:

```text
post_carrier_doppler_hz
  = physical_doppler_hz + tracking_error_hz

post_carrier_range_rate_mps
  = physical_range_rate_mps - tracking_error_mps
```

`tracking_error_mps = wavelength * tracking_error_hz` is already computed by the carrier core; the truth writer does not derive it.

## Disabled and unavailable semantics

When `carrier_tracking.enabled=false`, normal receiver behavior and RNG consumption remain unchanged. The carrier truth row has `carrier_tracking_enabled=0`, `carrier_result_available=0`, and `carrier_reset_reason=FEATURE_DISABLED` while a code-tracked physical/post-carrier observation can still be recorded; those two observation snapshots are identical because the carrier layer was bypassed.

When code tracking is not active, `carrier_result_available=0`, `carrier_reset_reason=CODE_NOT_TRACKING`, and observation snapshots are unavailable. Signal tracking phase/loss reason remain available so loss/reset epochs are diagnosable without reconstructing them in the writer.

## Scope boundary

This diagnostic file does not change normal RANGE serialization, the physical urban truth schemas, carrier equations/defaults, navigation data, or the measurement-noise layer. Authentic-NAV statistical validation and RTKLIB outcome analysis remain in #163.
