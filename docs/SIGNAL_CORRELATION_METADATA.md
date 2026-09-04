# Central Signal Correlation Metadata

Issue #133 adds the minimum code-domain metadata required by Issue #119 and its pure-correlation prerequisite #134. The metadata lives in the central `SignalDefinition` table so DLL code does not create a second signal mapping table.

## Scope and interpretation

`CodeCorrelationProfile` describes the **tracked RINEX signal component** used by the simulator. It is intentionally not a complete RF/IF/baseband signal description.

For example:

- Galileo E5a-Q / E5b-Q are represented as their independently tracked 10.23 Mcps sideband components, not as the full E5 AltBOC composite waveform.
- BeiDou B1C `1P` represents the pilot component, whose ranging code is modulated by QMBOC(6,1,4/33).
- GPS L1C `1L` represents the pilot component, whose spreading waveform uses TMBOC.

The metadata exists only to let #134 evaluate idealized code correlation and to let #119 convert chip-domain delay to signal-specific DLL behavior. Carrier phase, RF polarization, material response, path amplitudes, diffraction, CN0, tracking, and observation synthesis remain in their existing/later layers.

## Metadata fields

Each `SignalDefinition` carries:

- `model`: supported ideal correlation family or explicit `kUnsupported`;
- `chip_rate_hz`: ranging-code chip rate for the tracked component;
- `primary_subcarrier_rate_hz`: low-rate BOC subcarrier for composite BOC profiles;
- `secondary_subcarrier_rate_hz`: high-rate BOC subcarrier for composite profiles;
- `secondary_power_fraction`: power fraction of the secondary subcarrier contribution;
- `secondary_phase`: in-phase, anti-phase, **positive quadrature (+90 deg)**, **negative quadrature (-90 deg)**, or not applicable.

The sign of a quadrature relation is preserved because the later complex correlation function can have a signed imaginary component. TMBOC uses a time-multiplex fraction rather than a fixed subcarrier phase, so `secondary_phase` is `kNotApplicable`.

## Authoritative source baseline

The table is based on public system interface specifications rather than carrier-frequency inference or RINEX-name heuristics.

### GPS

- IS-GPS-200N, 1 Aug 2022, with currently published interface revision notices: L1 C/A and L2 P(Y)/L2C signal definitions.
- IS-GPS-705J, 1 Aug 2022, with currently published interface revision notices: L5 signal definition.
- IS-GPS-800J, 1 Aug 2022, with currently published interface revision notices: L1C signal definition. L1C-P uses TMBOC based on BOC(1,1) and BOC(6,1); the BOC(6,1) time/power fraction used for the idealized profile is 4/33.
- Current public documents are indexed by GPS.gov under Interface Control Documents / Interface Specifications.

Representative central profiles:

- GPS L1 C/A: BPSK, 1.023 Mcps.
- GPS L2 P: BPSK, 10.23 Mcps.
- GPS L5 Q: BPSK, 10.23 Mcps.
- GPS L1C pilot (`1L`): TMBOC(6,1,4/33), 1.023 Mcps code.

GPS L2C `2S` remains explicitly unsupported in V1 correlation metadata. The RINEX component and the L2C CM/CL time-multiplexed waveform need a dedicated correlation convention in #134; assigning a generic triangle here would hide that decision.

### QZSS

- Cabinet Office, Japan, IS-QZSS-PNT-006, 11 Jul 2024, is the current PNT interface specification used as the QZSS signal baseline.
- QZSS L1 C/A and L5 follow the compatible public PNT component definitions and are represented as 1.023 Mcps and 10.23 Mcps BPSK-style tracked components respectively.

QZSS L1C and L2C are explicitly unsupported in the V1 metadata. Current QZSS generations/blocks do not give the existing central `SignalDefinition` enough satellite-block/component identity to freeze one unambiguous ideal correlation profile without adding more state. #133 does not guess.

### Galileo

- Galileo Open Service SIS ICD v2.2, November 2025, is the current in-force OS SIS reference.
- E1-C is the pilot component of the E1 CBOC(6,1,1/11) signal. The pilot uses the minus/anti-phase CBOC combination, represented here as a 1/11 BOC(6,1) secondary power fraction relative to BOC(1,1).
- E5a-Q and E5b-Q use 10.23 Mcps ranging codes. For V1 code correlation they are treated as independently tracked sideband components rather than a full-band AltBOC correlator.
- E6-C uses a 5.115 Mcps BPSK-style tracked code component.

### BeiDou

- BDS-SIS-ICD-B1I-3.0, February 2019: B1I, BPSK, 2.046 Mcps.
- BDS-SIS-ICD-B3I-1.0, February 2018: B3I, 10.23 Mcps.
- BDS-SIS-ICD-B1C-1.0, December 2017: B1C pilot code rate 1.023 Mcps and QMBOC(6,1,4/33). Under the ICD complex-envelope convention, the pilot subcarrier is represented as `sqrt(29/33) * BOC(1,1) - j * sqrt(4/33) * BOC(6,1)`. Therefore the central B1C pilot profile stores the BOC(6,1) term as **negative quadrature**, not an unsigned generic quadrature relation.
- BDS-SIS-ICD-B2a-1.0, December 2017: B2a pilot/data ranging code rate 10.23 Mcps and BPSK(10).
- BDS-SIS-ICD-B2b-1.0, July 2020: B2b-I ranging code rate 10.23 Mcps and BPSK(10).

The generic QMBOC profile validator accepts either positive or negative quadrature structurally; the **specific B1C central entry** locks the negative sign required by the referenced ICD. This keeps the central type reusable without erasing the signal-specific phase convention.

### GLONASS

The existing central signals represent the open FDMA G1/G2 components and the L3OC-Q component. Their public signal definitions use:

- G1 open code: 0.511 Mcps BPSK-style code component;
- G2 open code: 0.511 Mcps BPSK-style code component;
- L3OC data/pilot components: 10.23 Mcps BPSK(10) components combined in quadrature at RF.

These profiles are code-component metadata only; GLONASS carrier FDMA frequency/channel handling remains in the existing carrier-frequency model and is not duplicated here.

## Unsupported means explicit, not fallback

A `kUnsupported` profile must carry zero waveform parameters. `validate_code_correlation_profile()` rejects guessed parameters attached to an unsupported entry. Later correlation code must call `signal_has_supported_code_correlation()` (or otherwise check the profile) and fail explicitly for unsupported signals.

There is deliberately no fallback such as "unknown -> triangular BPSK".

## Validation contract

`tests/unit/test_signal_correlation_metadata.cpp` permanently checks that:

- every central signal has a structurally valid profile;
- the unsupported set is explicit and stable;
- #119 minimum signals have the required representative profile and chip-rate metadata;
- GPS L1C, Galileo E1-C, and BeiDou B1C retain their distinct TMBOC/CBOC/QMBOC identities;
- BDS B1C retains the ICD-required negative-quadrature sign while the generic QMBOC validator can represent either signed quadrature orientation;
- invalid/non-finite/inconsistent profiles are rejected;
- existing signal lookup/carrier/NAV/bias/OEM7 behavior remains owned by the existing `SignalDefinition` regression suite.

## Explicit exclusions

Issue #133 does not implement:

- the autocorrelation function itself (#134);
- composite path correlation or coherent path summation (#119);
- Early/Late discrimination or root solving (#119);
- CN0/tracking/observables (#120-#122);
- IF/baseband samples;
- any NAV/EPH/ION generation, modification, interpolation, or retargeting.
