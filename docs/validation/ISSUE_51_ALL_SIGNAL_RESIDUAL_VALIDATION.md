# Issue 51: all-signal RTKLIB truth-state residual validation

This validation is the deterministic measurement-level acceptance gate for the frozen V1 signal set.

## Scope

All 21 signals execute both code and Doppler residual checks using their actual RINEX observation code and actual carrier wavelength:

- GPS: L1 C/A, L1C, L2P, L2C, L5Q
- QZSS: L1 C/A, L1C, L2C, L5Q
- GLONASS: G1, G2, G3
- Galileo: E1, E5a, E5b, E6
- BeiDou: B1I, B3I, B1C, B2a, B2b

A row appearing in `observation_truth.csv` is not sufficient coverage. A signal passes only after at least one valid RTKLIB code residual and at least one valid RTKLIB Doppler residual are evaluated. The compact acceptance test enforces this independently for every frozen signal.

A signal-specific NAV family missing at an epoch is a per-signal availability condition, not a simulator-fatal error. The simulator keeps unrelated signals running and validates message families over the union of real family-availability windows rather than requiring every family to exist at one instant.

## Truth-state code residual

For normal broadcast signals, the receiver position is fixed to truth. V1 receiver clock bias and GLO/GAL/BDS receiver-system offsets are zero. `rtklib_rescode_signal_ext()` selects the signal/message-family-specific broadcast satellite state and code-bias correction, applies the configured RTKLIB ionosphere/troposphere model, and evaluates the residual using the signal's actual wavelength.

For a generated raw pseudorange

`P = rho - c*dts + code_bias + ion + trop`

RTKLIB subtracts the same signal-specific `code_bias` before evaluating the residual. A physically correct raw RANGEA measurement is not changed merely to emulate an old first-frequency RTKLIB limitation.

Galileo E6 is intentionally different. Normal Galileo broadcast I/NAV/F/NAV does not provide the coherent E6 HAS orbit/clock plus E6 observable-specific bias needed for this acceptance. The E6 gate therefore uses real JRC Galileo HAS `SSRC00JRC0` precise SP3/CLK products together with the matching E02 `C6C` OSB from Bias-SINEX. The simulator computes the E6 measurement from that coherent external state/bias, and the residual is evaluated by RTKLIB-owned `rtklib_rescode_state_ext()` rather than by a duplicate simulator residual equation.

The compact gate requires maximum absolute code residual `< 0.02 m` for every signal.

## Truth-state Doppler residual

The receiver ECEF position and velocity are fixed to truth and receiver clock drift is zero. The RTKLIB residual equation is

`v = -lambda*D - (range_rate + receiver_clock_drift - c*satellite_clock_drift)`.

Each signal uses its own wavelength; GLONASS G1/G2 use the actual FCN. Broadcast signals use `rtklib_resdop_signal_ext()`. Galileo E6 uses the explicit HAS transmit-time state through `rtklib_resdop_state_ext()`. Both RTKLIB paths retain the same Earth-rotation range-rate term.

The compact gate requires maximum absolute Doppler residual `< 0.002 m/s` for every signal.

## BeiDou RTKLIB correction

RTKLIB issue `zengxianghang/RTKLIB#6` / PR `zengxianghang/RTKLIB#7` corrects the legacy single-frequency model according to the BDS ICD and adds the all-signal residual APIs. The intended raw-pseudorange bias convention is:

- B1I: `+c*TGD1` relative to the B3I clock reference;
- B3I: 0 additional group-delay term;
- B1C pilot: B1C pilot TGD from the matching B-CNAV record;
- B2a pilot: B2a pilot TGD from the matching B-CNAV record;
- B2b data: B2b-I TGD from the matching B-CNAV3 record.

Bias ephemeris selection is observation-epoch and message-family aware; it must never use the first matching satellite record from a full-day NAV file.

## Compact acceptance evidence

The simulator is pinned to RTKLIB commit `81e15b717ade0a5a91e365ed91511d414803dc05` from Draft PR `zengxianghang/RTKLIB#7` while this validation is under review.

Normal PR CI run `33148575831` (run #356) on simulator head `1c99713e314840287b844395c2baeef4704a87bc` passed:

- clang-format: PASS;
- Ubuntu/GCC build + full CTest: PASS, 152/152 tests;
- Windows/MSVC build + full CTest: PASS;
- `V1Acceptance.EveryFrozenSignalRunsTruthStateCodeAndDopplerResidualChecks`: PASS.

That acceptance test requires nonzero code and Doppler residual counts for each of all 21 frozen signals and applies the strict `< 0.02 m` / `< 0.002 m/s` maxima. Therefore compact residual coverage is now **21/21 code and 21/21 Doppler** on both supported CI platforms. No residual threshold was widened to obtain this result.

## Remaining real-data gates

Issue #51 remains open until the external validation chain is complete:

1. rerun the full WHU `BRD400DLR_S_20250030000_01D_MN.rnx.gz` 10-minute KS case with broadcast atmosphere and retain per-signal/per-family residual evidence for the broadcast-supported paths;
2. retain the official JRC HAS E6 precise-state/C6C-OSB segment as the coherent real-data E6 code gate rather than mixing a 2025 broadcast state with an unrelated HAS bias;
3. apply the finalized validator to the retained/generated 8-hour KS dataset;
4. review and merge RTKLIB PR #7, then advance the simulator submodule/pin from the draft dependency SHA to the reviewed RTKLIB merge commit and rerun the final regression gate.
