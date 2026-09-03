# First-order urban reflection paths

This document freezes the geometry and range conventions implemented by Issue #117. It is the handoff contract for later diffraction, DLL/correlation, coherent CN0, tracking, observable synthesis, and truth-output work.

## Scope

V1 searches deterministic single-bounce specular reflections from the four walls defined by `urban_scene_geometry`:

```text
NORTH -> EAST -> SOUTH -> WEST
```

Every geometrically valid candidate is retained. There is no strongest-path selection, relative-power cutoff, or power-based path-count limit. The array capacity of four is simply the number of physical wall candidates in the frozen scene.

The model does not modify NAV, EPH, ION, satellite state, or the RTKLIB transmit-time solution.

## Finite satellite source

The input `SatelliteGeometry` is already the converged RTKLIB-backed satellite solution. #117 must not solve satellite transmit time again.

For local image-method geometry, use the raw Euclidean ECEF satellite-receiver distance together with the existing RTKLIB-derived azimuth/elevation:

```text
d_euclid = norm(S_ecef - R_ecef)
R_enu    = (0, 0, antenna_height)
u_enu    = (cos(el)*sin(az), cos(el)*cos(az), sin(el))
S_enu    = R_enu + d_euclid*u_enu
```

This creates a finite source rather than an infinite plane-wave ray while preserving the existing direction and source distance.

## Sagnac-safe excess range

`SatelliteGeometry::geometric_range_m` comes from RTKLIB `geodist()` and includes RTKLIB's first-order Earth-rotation correction. A reflected path built from local Euclidean segments must therefore **not** be directly subtracted from that corrected range.

For receiver `R`, finite satellite `S`, and valid specular point `Q`:

```text
L0_euclid = |S-R|
Li_euclid = |S-Q| + |Q-R|
DeltaL    = Li_euclid - L0_euclid
Li_model  = SatelliteGeometry::geometric_range_m + DeltaL
DeltaTau  = DeltaL / c
```

Only `DeltaL` is the environmental excess path. This keeps the direct RTKLIB range convention unchanged and prevents Earth-rotation terms from becoming artificial multipath bias.

Tiny negative `DeltaL` caused solely by floating-point cancellation may be clamped to zero within the implementation tolerance. A materially negative value is an error because the image-method reflected path cannot be shorter than the direct Euclidean path.

## One-wall image method

Each candidate uses the wall plane and inward normal provided by #115.

A candidate is rejected as backside unless both receiver and finite satellite lie on the inward/receiver-facing side of that wall.

The source is mirrored across the target plane. The line from receiver to mirrored source intersects the target plane at the specular point. Accepted geometry must satisfy the vector reflection law:

```text
k_reflected = k_incident - 2*(k_incident dot n)*n
```

where `k_incident` is the propagation direction from satellite to specular point and `n` is the wall inward normal.

The specular point must lie within facade height `0..wall_height`.

## Occlusion

For a target wall, both open path segments are tested against each of the other three walls:

```text
satellite -> specular point
specular point -> receiver
```

An intersection blocks the segment only when it lies strictly inside the segment and its height is within the physical facade height range. An intersection above the wall top is clear. The target-wall endpoint is not treated as self-occlusion.

V1 does not invent a horizontal roof slab and does not truncate the horizontally infinite facade defined by #115.

## Frozen-scene multiplicity

The implementation can retain up to four valid wall candidates, but the current closed four-infinite-wall geometry is more restrictive.

With the receiver at the center, front-face illumination, 0..10 m facade height, and other-wall occlusion, systematic analytical and azimuth/elevation checks show that one satellite has at most one retained first-order reflection.

At diagonal azimuths two image-method candidates can exist algebraically. Before either target is reached, however, the ray crosses the nearer side facade below its roof. If elevation is raised enough to clear that facade, the target specular point rises above the 10 m facade and becomes invalid.

Therefore frozen-scene regression tests cover:

- zero retained paths;
- one retained path;
- simultaneous algebraic candidates rejected by backside/height/occlusion checks.

The code must not weaken physical occlusion merely to manufacture a multi-path test. If a future scene uses finite-width facades or an open street canyon, multiple valid retained reflections can naturally use the same four-candidate API.

## Signal and phase contract

#117 does not own code-chip metadata. It emits metric excess length and time delay:

```text
DeltaTau = DeltaL / c
```

#119 owns signal-specific correlation/code-chip conversion.

Carrier frequency and wavelength come only from the central signal-definition API, including GLONASS FDMA channel-dependent frequency.

The frozen carrier convention inherited from #116 is:

```text
time dependence:      exp(+j*omega*t)
forward propagation:  exp(-j*k*L)
excess phase:         -2*pi*DeltaL/lambda
```

The path stores the excess geometric phase separately from the #116 complex TE/TM and RHCP/LHCP material response and separately from the RHCP/LHCP receive-antenna voltage responses. #117 does not perform the final receiver-port coherent sum; that belongs to #120, after all required path/basis geometry is available.

## Determinism

For fixed scene configuration, RF configuration, signal/GLONASS FCN, receiver truth, and `SatelliteGeometry`, candidate order, rejection status, retained paths, excess range/delay, RF response, antenna response, and geometric phase are deterministic.
