# Frozen Four-Wall LOS_MULTIPATH Reachability

This document records the Issue #130 reachability decision for the frozen V1 urban scene defined by Issue #103 and implemented by #115, #117, and #118.

## Question

The frozen scene is a square of four horizontally infinite facades around the centered receiver. Issue #117 established that a geometrically valid first-order specular reflection does not coexist with direct LOS in this scene. Issue #118 added the roof-edge Fresnel transition field.

The remaining question is whether `LOS_MULTIPATH` is physically reachable without weakening reflection/occlusion rules and without adding a full direct field on top of a full diffracted field.

## Deterministic sweep

The permanent regression in `test_urban_rooftop_diffraction.cpp` sweeps:

- azimuth: `0..355 deg` in `5 deg` steps: 72 azimuths;
- elevation: local skyline `-5..+10 deg` in `0.25 deg` steps: 61 elevations per azimuth;
- total: 4,392 deterministic geometry samples;
- signal: GPS L1 C/A for the reachability proof;
- finite source distance: 20,000 km, matching the existing propagation unit-test convention.

Each sample calls the production geometry/propagation interfaces directly:

1. `compute_urban_direct_path_geometry()` from #115;
2. `compute_urban_first_order_reflections()` from #117;
3. `compute_urban_rooftop_diffraction()` from #118.

The synthetic finite source is only a unit-test geometry input. It does not create, modify, interpolate, or retarget NAV/EPH/ION records.

## Diagnostic edge-transition reference

For reachability evidence only, the sweep uses `v = -0.78` as the classical knife-edge boundary below which diffraction is commonly treated as negligible/unobstructed in the standard engineering approximation.

This value is **not** frozen here as the production #121 tracking/state threshold. Its purpose is only to distinguish a physically meaningful clear-side roof-edge transition from an asymptotically negligible Fresnel correction.

The diagnostic regions are therefore:

- shadow: blocked direct geometry with `v > 0`;
- LOS edge transition: direct LOS with `-0.78 < v < 0`;
- far-clear LOS: direct LOS with `v <= -0.78`.

## Regression decision

The sweep requires all of the following to remain true:

- zero samples with direct LOS and a retained #117 first-order specular reflection;
- every sampled azimuth has at least one blocked/shadow sample;
- every sampled azimuth has at least one direct-LOS sample in the clear-side roof-edge transition;
- every sampled azimuth also reaches a far-clear LOS region where the edge effect becomes negligible.

Therefore the frozen scene has a non-zero, physically defensible `LOS_MULTIPATH` region **only through the roof-edge-affected direct field**, not through simultaneous LOS + first-order specular reflection.

## No direct/diffraction double counting

The #118 Fresnel coefficient is the transfer factor of the roof-affected direct component. In the edge-transition region the observation model must use that field in place of an unmodified full-strength direct field.

It must not synthesize

`full direct + full diffracted`

as two independent components. Doing so would double count the same transition field.

A later coherent composition may combine the single roof-affected direct component with independent reflected paths when such reflected paths are geometrically valid. The frozen four-wall scene simply does not produce a direct-LOS + retained first-order reflection overlap.

## V1 scene decision

No geometry extension is required merely to make the Issue #103 `LOS_MULTIPATH` state reachable, because #103 explicitly allows an edge-affected LOS component and the clear-side Fresnel transition is non-zero over a finite region.

Keep the frozen four-wall geometry unchanged for V1.

However, this does **not** make `LOS + one specular reflection` or `LOS + multiple specular reflections` reachable in the frozen scene. If later acceptance criteria require those exact simultaneous path combinations, they need an explicit separate scene extension such as a finite-width/open street-canyon facade. Reflection occlusion must not be weakened to manufacture them.

## Consequences for later issues

- #121 should classify states without representing diffraction as a second full direct-like path.
- #122 should synthesize the roof-affected direct field once, then combine only genuinely independent paths.
- #124 should not require frozen-scene `LOS + specular reflection` scenarios that the geometry cannot produce. It may validate `LOS + roof-edge effect` for `LOS_MULTIPATH`, and validate retained reflection behavior in geometrically blocked/NLOS cases.
- If product requirements still demand simultaneous LOS + specular multipath, create and review a separate finite/open-facade scene issue before #122/#124 are frozen.
