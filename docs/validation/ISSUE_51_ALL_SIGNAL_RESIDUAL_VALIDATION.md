# Issue 51: all-signal RTKLIB truth-state residual validation

This validation is the deterministic measurement-level acceptance gate for the frozen V1 signal set.

## Scope

All 21 signals execute both code and Doppler residual checks using their actual RINEX observation code and actual carrier wavelength:

- GPS: L1 C/A, L1C, L2P, L2C, L5Q
- QZSS: L1 C/A, L1C, L2C, L5Q
- GLONASS: G1, G2, G3
- Galileo: E1, E5a, E5b, E6-C
- BeiDou: B1I, B3I, B1C, B2a, B2b

The frozen Galileo E6 observable is RINEX `6C` / OEM7 RANGE signal type 7. HAS is disseminated on E6-B, but the JRC/HAS code-bias product used by this validation is `C6C`; E6-B/C6B and E6-C/C6C must not be mixed.

A row appearing in `observation_truth.csv` is not sufficient coverage. A signal passes only after at least one valid RTKLIB code residual and at least one valid RTKLIB Doppler residual are evaluated. The compact acceptance test enforces this independently for every frozen signal.

A signal-specific NAV family missing at an epoch is a per-signal availability condition, not a simulator-fatal error. The simulator keeps unrelated signals running and validates message families over the union of real family-availability windows rather than requiring every family to exist at one instant.

## Shared residual validator

`validate-residuals` and `residual_validator_core` are the maintained residual evaluation path for compact CI, the full WHU 10-minute case, and the later 8-hour KS case. The validator streams `observation_truth.csv`, calls the RTKLIB-owned residual APIs, and reports signal, signal+family, and signal+family+satellite statistics including count, RMS, P95, and maximum absolute residual.

The compact all-signal acceptance no longer contains its own direct `rtklib_rescode_*` / `rtklib_resdop_*` equations. `tests/integration/test_all_signal_residuals.cpp` now only constructs the coverage fixtures/scenarios, runs `residual_validator_core`, merges the per-signal evidence across the coverage union, and applies the 21/21 thresholds. The compact G3 synthetic L3OC overlay remains test-only for G3 code-bias coverage and must never be used by real WHU validation.

The validator deliberately preserves the raw truth pseudorange in `obs->P[0]` even when `pseudorange_valid=0`. RTKLIB's Doppler residual path uses that raw pseudorange only to reconstruct signal transmit time. `pseudorange_valid` still exclusively controls whether a code residual is evaluated, so this does not promote an invalid code measurement to valid. This distinction is required for signals such as GPS L1C whose code-bias NAV family can be unavailable while Doppler remains physically valid.

Doppler validation with `required_message_mask=0` must use the nearest generic broadcast satellite state, because Doppler itself consumes no observable-specific code bias. Shared-validator parity exposed that RTKLIB PR #7 previously documented this behavior but still routed zero-mask requests through the signal-compatible ephemeris selector, which failed for GLONASS G3 when no L3OC record existed even though a generic GLONASS broadcast state was available. RTKLIB head `a5bc61990133830704f5040b7b9983b9b1e02681` fixes the contract by routing zero-mask Doppler state selection through `satposs()`; nonzero message masks remain signal/family constrained.

Compact integration coverage exercises both the normal broadcast path and the JRC HAS explicit-state E6-C path through this shared evaluator before it is used on long-run data.

## Truth-state code residual

For normal broadcast signals, the receiver position is fixed to truth. V1 receiver clock bias and GLO/GAL/BDS receiver-system offsets are zero. `rtklib_rescode_signal_ext()` selects the signal/message-family-specific broadcast satellite state and code-bias correction, applies the configured RTKLIB ionosphere/troposphere model, and evaluates the residual using the signal's actual wavelength.

For a generated raw pseudorange

`P = rho - c*dts + code_bias + ion + trop`

RTKLIB subtracts the same signal-specific `code_bias` before evaluating the residual. A physically correct raw RANGEA measurement is not changed merely to emulate an old first-frequency RTKLIB limitation.

Galileo E6-C is intentionally different. Normal Galileo broadcast I/NAV/F/NAV does not provide the coherent E6 HAS orbit/clock plus E6 observable-specific bias needed for this acceptance. The E6-C gate therefore uses real JRC Galileo HAS `SSRC00JRC0` precise SP3/CLK products together with the matching E02 `C6C` OSB from Bias-SINEX. The simulator computes the E6-C measurement from that coherent external state/bias, and the residual is evaluated by RTKLIB-owned `rtklib_rescode_state_ext()` rather than by a duplicate simulator residual equation.

The compact gate requires maximum absolute code residual `< 0.02 m` for every signal.

## Truth-state Doppler residual

The receiver ECEF position and velocity are fixed to truth and receiver clock drift is zero. The RTKLIB residual equation is

`v = -lambda*D - (range_rate + receiver_clock_drift - c*satellite_clock_drift)`.

Each signal uses its own wavelength; GLONASS G1/G2 use the actual FCN. Broadcast signals use `rtklib_resdop_signal_ext()`. Galileo E6-C uses the explicit HAS transmit-time state through `rtklib_resdop_state_ext()`. Both RTKLIB paths retain the same Earth-rotation range-rate term.

The compact gate requires maximum absolute Doppler residual `< 0.002 m/s` for every signal.

## BeiDou RTKLIB correction

RTKLIB issue `zengxianghang/RTKLIB#6` / PR `zengxianghang/RTKLIB#7` corrects the legacy single-frequency model according to the BDS ICD and adds the all-signal residual APIs. The intended raw-pseudorange bias convention is:

- B1I: `+c*TGD1` relative to the B3I clock reference;
- B3I: 0 additional group-delay term;
- B1C pilot: B1C pilot TGD from the matching B-CNAV record;
- B2a pilot: B2a pilot TGD from the matching B-CNAV record;
- B2b data: B2b-I TGD from the matching B-CNAV3 record.

Bias ephemeris selection is observation-epoch and message-family aware; it must never use the first matching satellite record from a full-day NAV file.

## Corrected compact acceptance evidence

The simulator currently pins RTKLIB Draft PR #7 head `a5bc61990133830704f5040b7b9983b9b1e02681`.

CI run `33151383933` (run #374) established shared-validator parity before the old duplicate acceptance equations were removed:

- clang-format: PASS;
- Ubuntu/GCC build + full CTest: PASS, 155/155 tests;
- Windows/MSVC build + full CTest: PASS;
- `ResidualValidatorIntegration.CompactBroadcastTruthUsesSharedRtklibEvaluator`: PASS;
- `ResidualValidatorIntegration.GalileoHasE6UsesSharedExplicitStateEvaluator`: PASS;
- `V1Acceptance.EveryFrozenSignalRunsTruthStateCodeAndDopplerResidualChecks`: PASS.

The rewrite at simulator commit `04d9ae69b5341571c1f84078744faeb85a3f80c5` then removed the direct residual equations from the all-signal acceptance. Ubuntu CI run #375 passed its full test job on that rewritten test; the only failing job in that run was formatting, which was subsequently applied without changing logic.

Shared-validator parity found and corrected two implementation gaps without changing thresholds: preserving raw pseudorange for Doppler transmit-time reconstruction when code validity is false, and honoring RTKLIB's generic zero-message-mask Doppler-state contract. The acceptance limits remain code `< 0.02 m` and Doppler `< 0.002 m/s`.

## Remaining real-data gates

Issue #51 remains open until the external validation chain is complete:

1. rerun the full WHU `BRD400DLR_S_20250030000_01D_MN.rnx.gz` 10-minute KS case with broadcast atmosphere and retain per-signal/per-family residual evidence for the broadcast-supported paths;
2. retain the official JRC HAS E6-C precise-state/C6C-OSB segment as the coherent real-data E6-C code gate rather than mixing a 2025 broadcast state with an unrelated HAS bias;
3. apply the same finalized validator to the retained/generated 8-hour KS dataset;
4. review and merge RTKLIB PR #7, then advance the simulator submodule/pin from the draft dependency SHA to the reviewed RTKLIB merge commit and rerun the final regression gate.
