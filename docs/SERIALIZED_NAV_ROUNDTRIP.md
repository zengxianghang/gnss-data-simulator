# Serialized NAV round-trip positioning

Issue #81 adds a black-box validation path that positions the simulator's NovAtel ASCII output without reopening the source RINEX NAV after simulation.

## Data path

```text
real RINEX NAV -> simulator -> simulated.log
                               |
                               +-> CRC-checked RANGEA parser
                               +-> CRC-checked EPHA/IONA parser
                               +-> empty RTKLIB nav_t populated in log order
                               +-> RTKLIB pntpos()
```

The source RINEX file is simulator input only. The serialized-NAV validator API intentionally has no RINEX NAV path and must not call `load_rinex_nav_file()` or `readrnx()`.

## Supported NovAtel ASCII navigation scope

The compact V1 validator reconstructs receiver-visible records that the simulator can serialize without inventing navigation content:

- GPS legacy/LNAV `GPSEPHEMA`
- GLONASS FDMA `GLOEPHEMERISA`
- Galileo `GALEPHEMERISA`
- BeiDou legacy D1/D2 `BD2EPHEMA`
- QZSS legacy/LNAV `QZSSEPHEMERISA`
- GPS legacy/LNAV Klobuchar `IONUTCA`
- BeiDou legacy `BD2IONUTCA`

Modern GPS/QZSS CNAV/CNAV2 records are not relabeled as legacy EPHA. Unsupported families stay explicit rather than being fabricated.

`GALEPHEMERISA` is a combined decoded receiver log rather than a raw RINEX family record. For E1 SPP reconstruction the validator uses the serialized I/NAV clock when I/NAV is marked received, otherwise the serialized F/NAV clock. Common orbit, health and BGD fields are taken directly from the same ASCII record.

RINEX 4 can contain multiple GPS ionosphere message families. Only GPS LNAV Klobuchar records are eligible for `IONUTCA`; CNVX ionosphere records are not masqueraded as the legacy message.

### BeiDou time semantics

The simulator's `BD2EPHEMA` receiver-log contract serializes the ephemeris week and Toe as GPST. RTKLIB, however, stores the absolute `eph.toe` epoch in GPST while retaining `eph.toes` as the native BeiDou BDT seconds-of-week used by the BeiDou Earth-rotation term in `eph2pos()`.

The serialized-NAV import adapter therefore keeps the parsed absolute Toe epoch unchanged and converts only the internal raw Toe SOW from GPST to BDT by subtracting 14 seconds, with week wrap handling. Omitting this conversion rotates BeiDou broadcast positions by roughly 16–43 km in the compact real-NAV case. This conversion is an RTKLIB internal representation requirement; it is not a change to the serialized `BD2EPHEMA` fields.

`BD2IONUTCA` similarly carries BDT-UTC leap seconds. The importer adds 14 seconds when restoring RTKLIB `nav.leaps`, whose convention is GPST-UTC. The validator also refuses broadcast-atmosphere positioning until a serialized GPS `IONUTCA` has appeared, so RTKLIB cannot silently fall back to its built-in default Klobuchar coefficients.

The compact five-system BRD400DLR fixture does not contain a NovAtel-serializable legacy GPS `IONUTCA`; its normal broadcast-atmosphere path intentionally inherits pinned RTKLIB default-Klobuchar behavior. The self-contained five-system positioning gate therefore runs both simulator and validator with atmosphere disabled, while independent real-RINEX GPS/BDS ION round-trip tests verify `IONUTCA`/`BD2IONUTCA` reconstruction. When broadcast atmosphere is requested from the serialized-log validator, a real serialized GPS `IONUTCA` is mandatory.

## Causality

Navigation records are applied to an initially empty RTKLIB navigation store as they appear in the generated log. A `RANGEA` epoch can use only navigation that appeared earlier in the stream. The validator never scans ahead for future EPH/ION records.

## Relationship to the existing validator

The existing RANGEA round-trip path remains unchanged and separately loads the original RINEX NAV. It is retained as a reference baseline. The serialized-NAV path is stricter: a generated log must be self-contained enough to produce valid broadcast SPP using only its own RANGEA/EPHA/IONA content.

No synthetic ephemeris or ionosphere data is introduced by this validation path.

## Navigation conversion numerical equivalence (issue 97)

`tests/integration/test_nav_equivalence.cpp` validates that the production conversion path
preserves the navigation solution numerically, not merely plausibly. Two independent
navigation stores are built from the same authoritative real RINEX NAV fixture: the
reference store loads the original RINEX directly, and the round-trip store is constructed
exclusively through the production chain (projected record -> NovAtel NAV writer ->
independent serialized-NAV parser -> production RTKLIB input adapter). The reference store
is never consulted while building or using the round-trip store.

Four deterministic regressions run on the real BRD400DLR fixture and its documented
Galileo companion fixture:

1. `SatelliteStateMatchesAcrossProductionRoundTrip` — for every serialized record of all
   five constellations, satellite ECEF position, velocity, clock bias, clock drift, and
   health are compared at Toe-Δ, Toe, and Toe+Δ epochs through the same RTKLIB broadcast
   state calculation. Selection is family-restricted so both paths evaluate the same real
   broadcast instance even when the reference store also holds modern-family records
   outside the receiver output contract.
2. `SatelliteStateSelectionMatchesAcrossRealEphemerisTransition` — the real consecutive
   E02 INAV instances (IODnav 1 and 2) are sampled before, between, and after the
   transition so older/newer record selection is exercised.
3. `CorrectionParametersSurviveProductionRoundTrip` — TGD/BGD/ISC-class group-delay and
   ionosphere/leap-second values are compared at the serialization boundary, and the
   reconstructed store must hold exactly the serialized instance identity set
   (satellite + family + IODE/IODnav).
4. `PairedPositioningMatchesBetweenOriginalRinexAndConvertedNavigation` — the same
   generated RANGEA epochs are solved twice with identical RTKLIB options (broadcast
   atmosphere and TGD active on both paths): original-RINEX navigation versus converted
   EPH/ION navigation.

Measured results on the compact real fixture (frozen tolerances are roughly 75x these
maxima; see the test constants):

- satellite state: 108 compared states, max position difference 1.3e-8 m (one ulp of the
  orbit-radius double), max velocity difference 1.3e-5 m/s (RTKLIB differentiates the
  propagated position, amplifying that ulp over its finite-difference step), clock
  bias/drift exactly 0;
- corrections: TGD/BGD and ionosphere/leap-second values round-trip within 1e-12 relative;
- paired positioning: 59 epochs, max 3D difference 3.2e-8 m, RMS 6.9e-9 m, receiver-clock
  difference 8.9e-9 m, zero solution-status or used-satellite divergences.

Thirteen records of the fixture (GPS/QZSS CNAV/CNAV2, BeiDou CNVX) are outside the frozen
receiver output contract and are reported as such; they are not part of the equivalence
set. No navigation value is synthesized, modified, retargeted, or interpolated anywhere in
this validation.
