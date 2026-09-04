# Urban Received Power and Effective C/N0

Issue #120 defines the V1 receiver-domain amplitude/power convention that connects the completed open-sky C/N0 model (#104/#105-#108), Low-E/polarization/antenna model (#116), first-order reflections (#117), rooftop diffraction (#118), and the signal-specific correlator/DLL layer (#119).

## Ownership boundary

This layer produces and consumes the same narrow path representation introduced by #119:

`{ code_delay_sec, normalized complex voltage }`.

The voltage is dimensionless and referenced to the unobstructed open-sky direct receiver-domain field for the same satellite/signal. #121 may use the resulting effective C/N0 for tracking decisions, and #122 may use the same physical path set for observable synthesis.

No second signal table, propagation table, or independent urban attenuation random process is introduced.

## Open-sky reference

The open-sky input is obtained through the existing `cn0_model_estimate_dbhz()` path. For the normalized external model its nominal semantics remain

`CN0_open(signal,elevation) = CN0_high_dbhz(receiver,signal) + Delta_CN0_elevation(signal,elevation)`,

followed by the already-existing deterministic time-correlated open-sky variation and compatibility behavior of the C/N0 model.

`CN0_high_dbhz` is receiver/signal specific, and `Delta_CN0_elevation` is the validated normalized real-IGS elevation shape. Issue #120 does not replace either term with a handwritten urban curve.

The corresponding unobstructed direct normalized complex voltage is defined as

`V_direct,open = 1 + j0`.

This is a receiver-domain reference, not a claim that the physical antenna has unit absolute gain. The empirical/configured open-sky C/N0 already establishes the receiver/antenna signal level and elevation behavior. The #116 receive-antenna complex response is therefore used as a **relative multipath polarization/arrival-direction response** when a reflected field is mapped back to this direct reference. This avoids silently applying the receiver/antenna calibration twice.

A future separately calibrated absolute antenna-pattern layer would require an explicit model/schema convention; it must not be added by multiplying the current open-sky baseline again without such a definition.

## Rooftop-affected direct field

The #118 Fresnel coefficient is already referenced to the unobstructed direct field:

`V_direct,roof = F(v)`.

It is the complete roof-transition transfer factor. The V1 receiver path set therefore contains exactly one roof-affected direct component. It never constructs

`full direct + full diffraction`.

The exposed #118 geometric phase is not multiplied into `F(v)` again. #118 already defines the phase-reference identity that prevents this double counting.

### Code-delay convention

The direct-referenced Fresnel field must also recover ordinary LOS code timing on the clear side. Therefore:

- clear/LOS side: roof-affected direct `code_delay_sec = 0`;
- blocked/shadow side: use #118 `excess_delay_sec` as the deterministic equivalent-edge code delay;
- exact grazing: #118 excess path tends to zero, so the two rules meet continuously.

This avoids the nonphysical result in which `F(v) -> 1` far into clear LOS while the entire direct code remains shifted by an edge-path delay.

## First-order reflection voltage

For each retained #117 reflection, let

- `D0` be the direct Euclidean source-receiver distance;
- `Dr` be the reflected Euclidean path length;
- `G` be #117 `geometric_phase_factor = exp(-j 2*pi*DeltaL/lambda)`;
- `Gamma_R` and `Gamma_L` be the #116 reflected RHCP/LHCP field coefficients from incident GNSS RHCP;
- `A_R(arrival)` and `A_L(arrival)` be the #116 receive-antenna complex voltage responses at the reflected arrival elevation;
- `A_D(elevation)` be the #116 RHCP receive-antenna response in the direct satellite direction.

The polarization-weighted reflected receiver voltage is

`V_pol = Gamma_R * A_R(arrival) + Gamma_L * A_L(arrival)`.

The V1 spherical-spreading field ratio is

`S = D0 / Dr`.

The normalized reflected voltage consumed by #119 is

`V_reflection = S * V_pol / A_D(elevation) * G`.

Thus material response, polarization, receive-antenna relative response, geometric spreading, and excess carrier phase are each applied exactly once.

The reflection code delay is the #117 `excess_delay_sec`. No arbitrary NLOS pseudorange offset is added.

## Composite correlator power

For the signal-specific ideal correlation `R` from #134 and normalized paths `i`, #119 defines

`Z(epsilon) = sum_i V_i * R(epsilon - tau_i)`.

Issue #120 converts this same composite field into effective correlator C/N0 at a supplied local code phase:

`P_ratio(epsilon) = |Z(epsilon)|^2`

`(C/N0)_effective_linear = 10^(CN0_open/10) * P_ratio`

and, for nonzero power,

`CN0_effective = CN0_open + 10*log10(P_ratio)`.

Constructive and destructive interference therefore emerge from the same complex path phases used by the DLL. A delayed path is naturally reduced at a prompt through the signal-specific code correlation rather than through an unrelated attenuation rule.

## Exact cancellation

If the composite correlator field cancels exactly,

`P_ratio = 0`.

The physical linear carrier-to-noise-density ratio is zero. The API records no finite dB-Hz value (`-infinity` plus an explicit `finite_effective_cn0=false`) rather than inventing an arbitrary C/N0 floor. #121 owns tracking thresholds, persistence, and BLOCKED transitions.

## Continuity

For continuous geometry:

- #117 path delay/phase evolves from geometry;
- #118 Fresnel amplitude/phase evolves continuously across clear/grazing/shadow;
- the clear/shadow direct code-delay convention joins at zero excess delay at grazing;
- #119 signal-specific correlation maps continuous path evolution into continuous composite correlator power except at genuine path appearance/disappearance or later tracking-root/state discontinuities.

No independent per-epoch urban white-noise attenuation is introduced.

## Runtime integration boundary

Issue #120 provides the physical normalized path set and effective-C/N0 evaluation. The existing open-sky C/N0 runtime remains unchanged when urban propagation is not consumed.

Issue #121 is the first tracking-state consumer of this result. It owns the 10 dB-Hz tracking threshold, the higher acquisition/reacquisition threshold, hysteresis/persistence, root continuity, and LOS/LOS_MULTIPATH/NLOS_TRACKED/BLOCKED state transitions. Issue #120 does not pre-empt those decisions.

Issue #122 subsequently maps the tracked physical path/DLL result to pseudorange, Doppler, carrier/ADR, lock, and final receiver observations.

## Regression anchors

Permanent #120 tests cover:

- constructive and destructive same-delay field addition in the power domain;
- signal-code decorrelation of a one-chip-separated path at the direct prompt;
- exact cancellation without an arbitrary C/N0 floor;
- rooftop grazing with `F(0)=0.5`, giving approximately 6.0206 dB C/N0 loss with no second direct field;
- shadow-side direct use of #118 edge excess delay and clear-side direct delay fixed at zero;
- retained reflection coexistence with the single roof-affected direct component in frozen blocked geometry;
- high-baseline translation changing open/effective C/N0 without changing normalized propagation paths;
- continuous clear/grazing/shadow roof response.

## Explicit exclusions

Issue #120 does not implement:

- acquisition/tracking thresholds, hysteresis, persistence, or reacquisition (#121);
- DLL root policy beyond consuming #119's existing correlator interface;
- pseudorange/Doppler/carrier/ADR synthesis (#122);
- truth-output expansion (#123);
- a dynamic IF/baseband simulation;
- independent random urban attenuation;
- NAV/EPH/ION generation, modification, interpolation, retargeting, or fabrication;
- parameter tuning to force a desired final PVT error.
