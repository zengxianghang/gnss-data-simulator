# Urban Low-E Curtain-Wall RF Model

This document defines the V1 RF/material/polarization contract introduced by Issue #116. The path-search geometry is owned by later propagation work; this module only evaluates the complex facade and receive-antenna response for supplied frequency/incidence/arrival geometry.

## Scope and non-negotiable constraints

The urban RF model changes propagation/received-signal behavior only. It never creates, changes, retargets, interpolates, or fabricates RINEX NAV/EPH/ION data, and it never injects a desired final PVT error.

V1 does not implement arbitrary 3-D meshes, diffuse rough-surface scattering, mullions/frames, generic multi-bounce propagation, rooftop diffraction, DLL tracking, or final observable synthesis.

## Representative V1 Low-E IGU preset

The default engineering stack is:

```text
outside air
  -> 6 mm glass
  -> Low-E conductive sheet on surface #2
  -> 12 mm cavity (RF approximation: air)
  -> 6 mm glass
  -> inside air
```

The 6/12/6 geometry is representative, not a claim that all curtain walls share one construction. A commercial 6/12/6 Low-E IGU example with Low-E coating on surface #2 is documented by Australian Glass Group:

- https://agg.com.au/product/insulglass-lowe-max/

All thicknesses and the sheet resistance remain configurable.

## Glass electrical properties

The default glass parameters follow the ITU-R P.2040 building-material model:

```text
epsilon_r_real = 6.27
sigma(f) = 0.0043 * f_GHz^1.1925  S/m
```

Reference:

- Recommendation ITU-R P.2040, Effects of building materials and structures on radiowave propagation above about 100 MHz: https://www.itu.int/rec/R-REC-P.2040

Under the frozen phasor convention below, the simulator constructs:

```text
epsilon_r_complex(f)
  = epsilon_r_real - j * sigma(f) / (omega * epsilon_0)
```

No independent loss-tangent term is added to the same default glass model, avoiding double counting of dielectric loss.

## Low-E conductive sheet

The coating is represented as a zero-thickness conductive sheet:

```text
Y_sheet = 1 / R_sheet
```

Published 24 GHz free-space measurements of architectural Low-E glass report estimated metallic-film sheet resistance of about 1.9 to 14 ohm/square and float-glass relative permittivity/loss values consistent with a strongly dielectric glass substrate:

- Masuko et al., "Characterization of Architectural Glasses by Free-Space Method in 24 GHz Band", 2025: https://doi.org/10.3130/aije.90.412
- J-STAGE article page: https://www.jstage.jst.go.jp/article/aije/90/835/90_412/_article/-char/en

The V1 default is:

```text
coating_sheet_resistance_ohm_sq = 5.0
```

This is an engineering preset inside reported/typical Low-E ranges. It is not claimed to be a GNSS-L-band measurement of one specific commercial window and must not be tuned to manufacture a desired positioning error.

## Frozen complex convention

All downstream coherent propagation code must use the same convention:

```text
time dependence:       exp(+j * omega * t)
forward propagation:   exp(-j * k * L)
lossy dielectric:      epsilon* = epsilon' - j * epsilon''
path carrier phase:    exp(-j * 2*pi*L/lambda)
```

`urban_rf_model` returns complex **field/voltage** coefficients. Material phase is retained and later multiplies, rather than replaces, geometric path phase.

For passive glass under this convention, the selected square-root branch gives a forward propagation constant whose imaginary part is non-positive, so `exp(-j*k*z)` attenuates with increasing `z`.

## TE/TM transfer matrix

For outside-air incidence angle `theta`, non-magnetic layer relative permittivity `epsilon`, and layer thickness `d`:

```text
q = sqrt(epsilon - sin(theta)^2)

y_TE = q
y_TM = epsilon / q

delta = k0 * d * q

M_layer = [ cos(delta),       j*sin(delta)/y
            j*y*sin(delta),   cos(delta)     ]
```

The sheet uses normalized shunt admittance:

```text
y_sheet = eta0 / R_sheet

M_sheet = [ 1,       0
            y_sheet, 1 ]
```

The V1 stack is:

```text
M = M_outer_glass * M_sheet * M_cavity * M_inner_glass
```

The result is terminated into inside air. The API returns tangential-electric-field reflection coefficients:

```text
Gamma_TE_tangent
Gamma_TM_tangent
```

These are not scalar dB losses.

## TE/TM to RHCP/LHCP convention

For specular geometry supplied by the later path-search module, define:

```text
s   = unit vector perpendicular to the plane of incidence
p_i = k_i x s
p_r = k_r x s
```

with `(s, p_i, k_i)` and `(s, p_r, k_r)` right-handed.

Because the transfer-matrix TM coefficient references tangential E while incident/reflected `p` bases have opposite tangential orientation at specular reflection:

```text
Gamma_s_basis =  Gamma_TE_tangent
Gamma_p_basis = -Gamma_TM_tangent
```

Under the frozen `exp(+j*omega*t)` convention:

```text
E_R = (s - j*p) / sqrt(2)
E_L = (s + j*p) / sqrt(2)
```

For an incident RHCP field, the model exposes:

```text
Gamma_RHCP_from_RHCP = (Gamma_TE_tangent - Gamma_TM_tangent) / 2
Gamma_LHCP_from_RHCP = (Gamma_TE_tangent + Gamma_TM_tangent) / 2
```

At normal incidence `Gamma_TE_tangent == Gamma_TM_tangent`, so the ideal planar reflection has approximately zero reflected RHCP and reverses handedness into LHCP. This is a required regression contract.

## Receive-antenna V1 approximation

V1 uses an axisymmetric complex voltage-response approximation. RHCP and LHCP each configure:

```text
gain_db_horizon
gain_db_zenith
phase_deg_horizon
phase_deg_zenith
```

Gain and phase are linearly interpolated in arrival elevation from 0 to 90 degrees. Power-gain dB is converted to voltage amplitude using:

```text
A = 10^(gain_db / 20)
```

and combined with configured phase as a complex voltage factor.

Neutral defaults are 0 dB / 0 degrees for both RHCP and LHCP at horizon and zenith. This intentionally avoids inventing a proprietary antenna cross-polar pattern. A measured target-antenna response can later be configured without changing the RF equations.

## JSON configuration

`urban_rf` contains default material/antenna parameters plus optional per-signal overrides. Signal keys must use the existing central `SignalDefinition::name`; the RF model does not maintain a second GNSS frequency table.

Example:

```json
{
  "urban_rf": {
    "material": {
      "relative_permittivity_real": 6.27,
      "conductivity_c_s_per_m": 0.0043,
      "conductivity_exponent": 1.1925,
      "outer_glass_thickness_m": 0.006,
      "cavity_thickness_m": 0.012,
      "inner_glass_thickness_m": 0.006,
      "coating_sheet_resistance_ohm_sq": 5.0
    },
    "antenna": {
      "rhcp": {
        "gain_db_horizon": 0.0,
        "gain_db_zenith": 0.0,
        "phase_deg_horizon": 0.0,
        "phase_deg_zenith": 0.0
      },
      "lhcp": {
        "gain_db_horizon": 0.0,
        "gain_db_zenith": 0.0,
        "phase_deg_horizon": 0.0,
        "phase_deg_zenith": 0.0
      }
    },
    "signals": {
      "GPS L1 C/A": {
        "material": {
          "coating_sheet_resistance_ohm_sq": 7.0
        },
        "antenna": {
          "lhcp": {
            "gain_db_zenith": -15.0
          }
        }
      }
    }
  }
}
```

A signal override starts from the default and changes only fields explicitly supplied in that signal block. Unknown signal names, duplicate signal entries, non-finite values, non-positive thickness/sheet resistance, invalid frequency, and invalid incidence/elevation requests fail clearly rather than silently falling back.

## Frequency ownership

Fixed-frequency signals use the central signal-definition frequency. GLONASS G1/G2 use the existing FCN-dependent frequency calculation. `urban_rf_model` accepts the resolved carrier frequency as an input; it never defines a parallel signal-to-frequency mapping.

## Downstream contract

- #117 supplies specular path geometry and incidence/arrival directions, then consumes the complex material/polarization response.
- #120 combines complex path amplitudes coherently with the normalized open-sky CN0 model.
- #122 later combines material phase with the geometric `exp(-j*2*pi*L/lambda)` path phase when synthesizing consistent observables.

Changing any phasor, TE/TM reference, circular-basis, or voltage/power convention requires an explicit design update and regression-test change; downstream modules must not silently reinterpret these coefficients.
