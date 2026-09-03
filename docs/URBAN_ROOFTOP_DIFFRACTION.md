# Urban Rooftop Diffraction Model

This document freezes the V1 single-rooftop diffraction contract implemented for Issue #118.

## Scope

The model represents the roof-affected direct component at the **actual first blocking wall** selected by `urban_scene_geometry`. It does not add an independent full-strength direct path plus an independent full-strength diffracted path. The Fresnel result is the transition field itself.

V1 excludes double-edge diffraction, UTD, finite roof width, and a full electromagnetic solver.

## Input geometry

Satellite state, azimuth/elevation, and RTKLIB geometric range are taken from the already-converged `SatelliteGeometry`. No second satellite-state solution is performed and no NAV/EPH/ION record is generated, modified, interpolated, or retargeted.

The local finite source is reconstructed using the same contract as first-order reflections:

- receiver ENU: `R = (0, 0, antenna_height)`;
- Euclidean satellite distance: `D0 = |S_ecef - R_ecef|`;
- ENU direction from the existing azimuth/elevation;
- satellite ENU: `S = R + D0 * u_enu`.

The roof edge is the infinite line from the `UrbanWallPlane` of the direct path's `primary_wall`:

`E(q) = E0 + q * t`,

where `E0` is `roof_edge_anchor_enu_m` and `t` is the unit roof-edge direction.

## Deterministic 3-D equivalent diffraction point

For a point `X`, define its coordinate and perpendicular radius relative to the roof edge:

`q_X = dot(X - E0, t)`

`r_X = |(X - E0) - q_X t|`.

The unique point that minimizes `|S-Q| + |Q-R|` along the infinite edge is

`q_Q = (r_R q_S + r_S q_R) / (r_S + r_R)`

`Q = E0 + q_Q t`.

The two edge legs are

`d_S = |S-Q|`, `d_R = |R-Q|`,

and the explicit edge-path length is

`L_edge = d_S + d_R`.

## Stable excess path

Directly subtracting two approximately 20,000 km path lengths is numerically weak near roof grazing. The implementation therefore uses an algebraically equivalent non-negative form.

Let `u_S` and `u_R` be unit vectors from `Q` toward the satellite and receiver. Then

`DeltaL = d_S d_R |u_S + u_R|^2 / (L_edge + D0)`.

At exact grazing, `u_S = -u_R`, so `DeltaL = 0` without catastrophic cancellation.

The RTKLIB/Sagnac-safe modeled path is then

`L_model = SatelliteGeometry.geometric_range_m + DeltaL`.

The RTKLIB corrected direct range is never subtracted from a Euclidean edge path.

## Signed Fresnel clearance and parameter

The clearance magnitude is the shortest distance between the infinite direct ray line and the infinite roof-edge line. Its sign follows the direct roof-clearance classification from `urban_scene_geometry`:

- positive: edge intrudes into the direct path / shadow side;
- zero: roof grazing;
- negative: direct ray clears the roof.

The equivalent knife-edge parameter uses the exact excess path and actual signal wavelength:

`v = sign(clearance) * sqrt(2 DeltaL / lambda)`.

This is equivalent to the usual paraxial knife-edge relation near the transition while remaining directly tied to the exact 3-D edge geometry.

Carrier frequency and wavelength come only from the central `SignalDefinition` helpers, including GLONASS FDMA FCN handling.

## Complex Fresnel coefficient

The repository convention is

- time dependence: `exp(+j omega t)`;
- propagation: `exp(-j k L)`.

With Fresnel integrals

`C(v) = integral_0^v cos(pi t^2 / 2) dt`

`S(v) = integral_0^v sin(pi t^2 / 2) dt`,

the direct-referenced knife-edge coefficient is

`F(v) = (1+j)/2 * [(1/2-C(v)) - j(1/2-S(v))]`.

It satisfies the required anchors:

- `v -> -infinity`: `F(v) -> 1`;
- `v = 0`: `F(0) = 0.5`, which is `6.0206 dB` amplitude loss;
- increasing positive `v`: increasing deep-shadow attenuation;
- amplitude and phase vary continuously through `v = 0`.

The implementation evaluates the Fresnel integrals deterministically with a convergent power series for small arguments and a continued-fraction representation for larger arguments.

## Phase-reference contract and no double counting

`fresnel_coefficient` is referenced to the unobstructed direct phase. Therefore later coherent composition may use it directly as the roof transfer factor relative to the direct field.

For diagnostics and future path-rate work, the module also exposes

`G_edge = exp(-j 2 pi DeltaL / lambda)`

and an edge-referenced coefficient

`F_edge = F(v) * conj(G_edge)`.

The identity

`F_edge * G_edge = F(v)`

is regression-tested. This prevents the Fresnel phase from being counted a second time by blindly multiplying `F(v)` by another full `exp(-j k DeltaL)` term.

## Integration boundary

Issue #118 stops at propagation-layer outputs: roof edge, equivalent point/path, excess delay, signed clearance, `v`, complex coefficient, and phase references.

It does **not** implement DLL bias, coherent CN0 composition, tracking state transitions, pseudorange/Doppler/ADR synthesis, or final RANGE output. Those remain owned by later #119-#123 work.
