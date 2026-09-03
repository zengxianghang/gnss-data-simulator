# Ideal Signal-Specific Code Correlation

Issue #134 implements the pure code-domain correlation layer required before Issue #119 adds multipath path composition and a noncoherent Early/Late discriminator.

This layer consumes only the central `CodeCorrelationProfile` metadata completed by Issue #133. It does not switch on signal names and does not contain a second signal table.

## V1 ideal-code convention

The V1 model treats different spreading-code chips as uncorrelated for the purpose of the ideal local correlation shape. Therefore only overlap inside the same normalized code-chip interval contributes, and

`R(tau) = 0` for `|tau| >= 1 chip`.

Within one chip, the spreading chip is multiplied by the signal-specific subcarrier waveform. The implementation evaluates its autocorrelation exactly from piecewise-constant waveform intervals rather than by time sampling.

The returned correlation is normalized so that `R(0) = 1`.

## BPSK

For a BPSK-style rectangular spreading chip,

`R(tau) = 1 - |tau|`, for `|tau| < 1`,

and zero otherwise.

The normalized chip-domain shape is the same for GPS L1 C/A and GPS L5, but the physical delay scale is not: the chip duration is `1 / chip_rate`, and the chip length is `c / chip_rate`. Thus a 10.23 Mcps signal has one tenth the meter-scale main-lobe width of a 1.023 Mcps signal.

## Sine-BOC primitive

Composite profiles use the standard sine-BOC square subcarrier primitive. For BOC(n,1), one code chip contains `2n` constant half-subcarrier intervals with alternating `+1/-1` signs.

The implementation converts the metadata subcarrier-rate / chip-rate ratio to an integer `n`. Non-integer or unsupported ratios fail explicitly; they are not approximated or silently rounded into another waveform.

For a piecewise-constant chip waveform `w(t)`, positive-delay autocorrelation is evaluated by exact interval overlap:

`R(tau) = integral w(t) conj(w(t - tau)) dt / E`,

where `E = integral |w(t)|^2 dt` over one normalized chip. Negative delays use the Hermitian identity

`R(-tau) = conj(R(tau))`.

## TMBOC

GPS L1C pilot metadata uses TMBOC(6,1,4/33). TMBOC is time-multiplexed across code chips, so V1 uses the ideal ensemble/time-average correlation

`R_TMBOC = (29/33) R_BOC(1,1) + (4/33) R_BOC(6,1)`.

There is no coherent BOC(1,1)-to-BOC(6,1) cross term in this ideal TMBOC average because the two subcarriers occupy different multiplexed chip positions.

Representative permanent anchors are:

- `R(1/12 chip) = 0.5479797979797979`;
- `R(1/4 chip) = 0.12878787878787884`;
- `R(1/2 chip) = -0.37878787878787884`.

These anchors make it impossible for L1C to silently collapse to a BPSK triangle.

## CBOC

Galileo E1-C pilot metadata uses CBOC(6,1,1/11) with the minus/anti-phase combination. The normalized one-chip waveform is formed as

`w = sqrt(10/11) BOC(1,1) - sqrt(1/11) BOC(6,1)`.

Because the two subcarriers are coherently summed in the same chip, their cross terms are retained by the exact interval-overlap integral.

Representative permanent anchors are:

- `R(1/12 chip) = 0.5505715506035093`;
- `R(1/4 chip) = 0.11117761120957005`;
- `R(1/2 chip) = -0.409090909090909`.

## QMBOC

BeiDou B1C pilot metadata uses QMBOC(6,1,4/33). Under the B1C ICD complex-envelope convention the one-chip subcarrier is

`w = sqrt(29/33) BOC(1,1) - j sqrt(4/33) BOC(6,1)`.

Issue #133 therefore preserves the secondary term as **negative quadrature**. The generic pure-correlation evaluator consumes that signed phase when constructing the chip waveform.

For ideal autocorrelation of the same complex pilot waveform, the orthogonal cross terms cancel and the resulting current B1C anchors equal the 29/33 : 4/33 power-weighted BOC autocorrelation:

- `R(1/12 chip) = 0.5479797979797979`;
- `R(1/4 chip) = 0.12878787878787884`;
- `R(1/2 chip) = -0.37878787878787884`.

The implementation still returns `std::complex<double>` and preserves Hermitian semantics so the signed waveform convention is not lost at the API boundary.

## Galileo E5 sidebands

The central #133 convention models Galileo E5a-Q and E5b-Q as the independently tracked 10.23 Mcps sideband components, not as a full-band E5 AltBOC receiver. Their V1 local code correlation is therefore the BPSK-style 10.23 Mcps component correlation.

## Supported delay APIs

Two narrow public evaluations are provided:

- `ideal_code_correlation_chips(profile, delay_chips, ...)` for pure normalized profile tests;
- `ideal_signal_code_correlation(definition, delay_seconds, ...)` for later propagation/DLL integration.

`signal_code_chip_duration_s()` and `signal_code_chip_length_m()` expose the physical scale from the same central chip-rate metadata.

## Explicit failure behavior

- `kUnsupported` central profiles fail rather than falling back to BPSK.
- malformed #133 metadata fails validation;
- composite subcarrier rates that are not representable as supported integer BOC(n,1) multiples fail explicitly;
- non-finite delays fail;
- no waveform parameter is inferred from carrier frequency, RINEX code, or signal name.

## Scope boundary

Issue #134 does **not** implement:

- reflected/diffracted/direct path summation;
- propagation complex amplitudes or carrier phases;
- Early/Late spacing or discriminator output;
- S-curve root search/stability classification;
- acquisition/tracking root policy;
- CN0, lock, pseudorange, Doppler, carrier/ADR, or final observation synthesis;
- IF/baseband samples;
- NAV/EPH/ION changes.

Those remain in #119 and later issues. In particular, the #130 rule still applies: rooftop diffraction modifies the direct component and must not later be inserted as a second full-strength direct-like path.
