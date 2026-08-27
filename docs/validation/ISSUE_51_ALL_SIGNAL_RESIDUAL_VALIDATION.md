# Issue 51: all-signal RTKLIB truth-state residual validation

This validation is the deterministic measurement-level acceptance gate for the frozen V1 signal set.

## Scope

All 21 signals must eventually execute both code and Doppler residual checks using their actual RINEX observation code and actual carrier wavelength:

- GPS: L1 C/A, L1C, L2P, L2C, L5Q
- QZSS: L1 C/A, L1C, L2C, L5Q
- GLONASS: G1, G2, G3
- Galileo: E1, E5a, E5b, E6
- BeiDou: B1I, B3I, B1C, B2a, B2b

A row appearing in `observation_truth.csv` is not sufficient coverage. A signal passes only after at least one valid `rescode` and at least one valid `resdop` residual are evaluated.

A signal-specific NAV family missing at an epoch is a per-signal availability condition, not a simulator-fatal error. The simulator may keep geometry/Doppler available from the usable broadcast state while marking that signal's pseudorange unavailable until its required bias/message-family data are available. Missing one signal family must never abort unrelated signals or the whole epoch.

## Truth-state code residual

The receiver position is fixed to truth. V1 receiver clock bias and GLO/GAL/BDS receiver-system offsets are zero. The RTKLIB residual path uses broadcast `satposs()`, the signal-specific broadcast code-bias correction, broadcast ionosphere when enabled, and Saastamoinen troposphere when enabled.

For a generated raw pseudorange

`P = rho - c*dts + code_bias + ion + trop`

RTKLIB must subtract exactly the same signal-specific `code_bias` before evaluating the residual. A physically correct raw RANGEA measurement must not be changed merely to emulate an old RTKLIB limitation.

The primary in-memory truth residual target is floating-point floor. A second disk/RANGEA pass will use a serialization-floor threshold consistent with 0.001 m pseudorange formatting.

## Truth-state Doppler residual

The receiver ECEF position and velocity are fixed to truth and receiver clock drift is zero. The RTKLIB residual equation is

`v = -lambda*D - (range_rate + receiver_clock_drift - c*satellite_clock_drift)`.

Each signal uses its own wavelength; GLONASS G1/G2 use the actual FCN. The in-memory target is floating-point/float-storage floor; the disk/RANGEA pass accounts for 0.001 Hz Doppler formatting.

## BeiDou RTKLIB correction

RTKLIB issue `zengxianghang/RTKLIB#6` corrects the legacy single-frequency model according to the BDS ICD and adds an all-signal residual API. The intended raw-pseudorange bias convention is:

- B1I: `+c*TGD1` relative to the B3I clock reference;
- B3I: 0 additional group-delay term;
- B1C pilot: B1C pilot TGD from the matching B-CNAV record;
- B2a pilot: B2a pilot TGD from the matching B-CNAV record;
- B2b data: B2b-I TGD from the matching B-CNAV3 record.

Bias ephemeris selection is observation-epoch and message-family aware; it must never use the first matching satellite record from a full-day NAV file.

## Current blockers

GLONASS G3/L3OC and Galileo E6 are not considered passing merely because Doppler can be evaluated. Their pseudorange code-bias reference must be established from the applicable ICD/message data and represented in the NAV/parser model before code residual coverage can be counted as 21/21. Until then, Issue #51 remains open.

## Real-data gate

After compact offline GCC/MSVC tests are green, rerun the full WHU `BRD400DLR_S_20250030000_01D_MN.rnx.gz` 10-minute KS validation and retain per-signal residual evidence. Then apply the finalized validator to the 8-hour KS dataset.