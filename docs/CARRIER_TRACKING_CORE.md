# Carrier tracking core V1

Issue: #160  
Parent design: #159

## Scope

This document describes only the standalone carrier-tracking core. Runtime observation wiring, truth CSV integration, and authentic-NAV validation belong to #161, #162, and #163 respectively.

The core does not modify urban propagation physics and does not own an RNG. The caller supplies one explicit standard-normal sample per update, so later runtime integration can choose a deterministic per-satellite/per-signal RNG convention without changing this model.

## Frozen defaults

| Parameter | V1 default |
| --- | ---: |
| coherent integration time | 0.020 s |
| PLL noise bandwidth | 5 Hz |
| FLL steady bandwidth | 4 Hz |
| FLL pull-in bandwidth | 8 Hz |
| FLL pull-in duration | 0.5 s |
| PLL enter | >=30 dB-Hz for 1.0 s |
| PLL exit | <27 dB-Hz for 0.3 s |
| FLL acquire | >=22 dB-Hz for 0.2 s |
| FLL loss | <18 dB-Hz for 0.5 s |
| Doppler-valid delay after fresh carrier acquisition | 0.2 s |
| ADR-valid delay after PLL entry | 1.0 s |

All defaults are engineering assumptions for the V1 receiver model. They are configurable and are not calibrated universal receiver constants or PVT-error targets.

## State semantics

The standalone carrier state is one of:

- `CARRIER_UNLOCKED`: carrier Doppler and ADR invalid;
- `FLL_TRACK`: Doppler may become valid after the acquisition delay; ADR invalid;
- `PLL_TRACK`: Doppler valid once carrier acquisition has matured; ADR becomes valid after the PLL confirmation delay.

Fresh unlocked -> FLL acquisition uses the 8 Hz pull-in bandwidth for 0.5 s. PLL -> FLL fallback uses the 4 Hz steady FLL bandwidth because carrier frequency is already acquired. A fresh unlocked -> FLL transition starts a new carrier segment. Loss to unlocked clears the filtered carrier error.

Threshold comparisons preserve hysteresis: equality at the acquisition/entry threshold counts as eligible, while equality at the loss/exit threshold does not count as a loss sample.

## CN0 conversion

Input `effective_cn0_dbhz` is converted to linear `C/N0` in Hz as:

`cn0_linear = 10^(cn0_dbhz / 10)`.

The implementation bounds the linear value to a finite numerical range only to prevent floating-point overflow/underflow for extreme test inputs. Ordinary GNSS CN0 values are unaffected.

## FLL thermal jitter

For the V1 differential-phase FLL, thermal velocity jitter follows the standard form:

`sigma_v = lambda/(2*pi*T) * sqrt(4*F*Bn/(C/N0) * (1 + 1/(T*C/N0)))`.

V1 uses `F = 1.0`, corresponding to the arctangent/high-CN0 discriminator reference convention. This choice is explicit rather than silently adding an empirical weak-signal sigma. A future measured-data calibration may justify a configurable low-CN0 discriminator factor, but #160 does not tune it to produce a desired velocity error.

The equivalent frequency jitter is `sigma_f = sigma_v/lambda`.

Reference: ESA Navipedia, *Frequency Lock Loop (FLL)*, which gives the same thermal-jitter form and identifies `F=1` at high C/N0 and `F=2` near the threshold.

## PLL thermal jitter and equivalent Doppler scale

For an arctangent PLL, V1 uses the standard thermal phase jitter:

`sigma_phi = sqrt(Bn/(C/N0) * (1 + 1/(2*T*C/N0)))` rad.

The standalone model needs an equivalent frequency-error scale for later Doppler synthesis. V1 converts one coherent-integration phase jitter to an equivalent frequency jitter by the explicit engineering approximation:

`sigma_f = sigma_phi/(2*pi*T)`.

Then `sigma_v = lambda*sigma_f`.

This does not claim to be a full discrete PLL transfer-function simulation. It is a simplified CN0/T/Bn-driven receiver tracking model that is intentionally more physical than a fixed final Doppler sigma while remaining outside full IF/IQ/baseband simulation.

Reference: ESA Navipedia, *Phase Lock Loop (PLL)*, for the thermal phase-jitter expression.

## Time correlation

The carrier error is first-order filtered:

`e_k = alpha*e_(k-1) + sigma_k*sqrt(1-alpha^2)*w_k`

with:

`tau = 1/(2*pi*Bn)`

`alpha = exp(-dt/tau)`.

`w_k` is the caller-provided standard-normal sample. No arbitrary long-memory `rho` is configured. With the V1 loop bandwidths, thermal error itself can be nearly uncorrelated at a 1 Hz simulator output rate; slower observable correlation is expected to come later from effective-CN0/fading evolution, propagation state, and tracking-mode persistence.

## Determinism contract

- The core has no global/static RNG.
- The core never samples wall-clock state.
- Equal config, input sequence, initial state, and standard-normal sequence produce identical results.
- `carrier_segment_id` increments only when a fresh unlocked carrier is acquired into FLL.
- Runtime RNG ownership and legacy-RNG compatibility are deferred to #161.

## Explicit exclusions

- no simulator runtime Doppler/range-rate mutation;
- no truth CSV/schema change;
- no receiver oscillator/common-mode phase noise;
- no moving receiver/reflector model;
- no arbitrary weak-signal Doppler sigma;
- no calibration to a target RTKLIB velocity error.
