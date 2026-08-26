# Supported Navigation Log Records

This document freezes the navigation-record output scope for the GNSS data simulator.

The simulator shall use one common internal Receiver NAV state populated from RINEX navigation data. Manufacturer-specific output is handled only by formatter/serializer layers. NovAtel OEM7 and Unicore N4 formatters must not implement independent ephemeris-selection or navigation-state logic.

## NovAtel OEM7

Supported navigation records:

- `GLOEPHEMERISA`
- `QZSSEPHEMERISA`
- `GALEPHEMERISA`
- `GPSEPHEMA`
- `BD2EPHEMA`
- `IONUTCA`
- `BD2IONUTCA`

The ASCII `A` suffix is part of the emitted ASCII log name where applicable.

## Unicore N4

Supported navigation record families:

- `GPSION`
- `BD3ION`
- `BDSION`
- `GALION`
- `GPSEPH`
- `QZSSEPH`
- `BD3EPH`
- `BDSEPH`
- `GLOEPH`
- `GALEPH`
- `IRNSSEPH`

For Unicore ASCII logs, the emitted record normally carries the trailing `A`, for example `GPSEPHA`. Internally, record-family identifiers should remain independent from ASCII suffix handling so binary/other output formats can be added later without changing navigation logic.

## Output architecture

Recommended boundary:

```text
RINEX NAV
   |
   +--> Truth NAV
   |
   +--> Receiver NAV
            |
            +--> NovAtel OEM7 NAV formatter
            |
            +--> Unicore N4 NAV formatter
```

The Receiver NAV state controls when a record is considered available to the simulated receiver and when the corresponding navigation log may be emitted.

### HOT/WARM TTFF

At power-on, all retained navigation records are restored from the simulated Flash cache into Receiver NAV immediately. EPH/ION logs may then be queued over a short configurable output interval to simulate Flash -> RAM -> logger behavior.

### COLD TTFF

Navigation records are not emitted merely because Truth NAV contains them. EPH records become available and are emitted only after the simulated receiver has collected the required navigation-message fragments according to the constellation/navigation-message schedule. The completed ephemeris is inserted into Receiver NAV at the same logical availability time.

### REA

Receiver NAV remains retained while RF signal is off. Navigation state is not cleared by a REA signal interruption.

## RINEX mapping

RINEX is the source of broadcast navigation content, including orbit, clock, health, and applicable TGD/ISC/BGD/ION parameters. The formatter layer converts the common decoded navigation state into the corresponding receiver-specific ASCII record.

Where the selected RTKLIB baseline does not preserve a required RINEX 4 field, prefer extending the RTKLIB parser/navigation structures rather than adding a second independent RINEX parser inside the simulator.

## IRNSS note

`IRNSSEPH` serialization is in the supported Unicore N4 navigation-record scope. IRNSS observation/RANGE signal generation is not currently part of the frozen V1 observation-signal set; supporting the navigation record does not by itself add IRNSS measurement simulation.
