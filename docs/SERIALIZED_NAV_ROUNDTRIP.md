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

## Causality

Navigation records are applied to an initially empty RTKLIB navigation store as they appear in the generated log. A `RANGEA` epoch can use only navigation that appeared earlier in the stream. The validator never scans ahead for future EPH/ION records.

## Relationship to the existing validator

The existing RANGEA round-trip path remains unchanged and separately loads the original RINEX NAV. It is retained as a reference baseline. The serialized-NAV path is stricter: a generated log must be self-contained enough to produce valid broadcast SPP using only its own RANGEA/EPHA/IONA content.

No synthetic ephemeris or ionosphere data is introduced by this validation path.
