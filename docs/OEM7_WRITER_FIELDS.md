# OEM7 Writer Field Sources

This document freezes the V1 source of every field emitted by the `RANGEA`, `PSRPOSA`, and `PSRVELA` writers.
Writers serialize existing simulator/solution state only. They do not recompute satellite geometry, navigation corrections,
tracking validity, or solution validity.

## Common OEM7 ASCII header

| Field | V1 source |
| --- | --- |
| message | Fixed per writer: `RANGEA`, `PSRPOSA`, or `PSRVELA` |
| port | Fixed protocol value `COM1` |
| sequence | Fixed `0` |
| idle time | Fixed `0.0` |
| time status | Fixed `FINE`; V1 GPST is deterministic and receiver clock bias/drift are zero |
| GPS week / SOW | `SimTime`, rounded only to the OEM7 ASCII millisecond representation |
| receiver status | Fixed `00000000` |
| reserved | Fixed `0` |
| software version | Fixed `0` |
| CRC | NovAtel CRC-32 over the ASCII payload between `#` and `*`, polynomial `0xEDB88320`, initial value zero |

## RANGEA

| Field | V1 source |
| --- | --- |
| observation count | Count of `MeasurementObservation::observation_available == true` |
| PRN/slot | RTKLIB canonical satellite ID plus OEM7 PRN mapping: GPS/Galileo/BeiDou direct, GLONASS slot + 37, QZSS PRN + 192 |
| glofreq | `glonass_fcn + 7` for GLONASS; otherwise `0` |
| pseudorange | `MeasurementObservation::pseudorange_m` when valid, otherwise `0` |
| pseudorange sigma | Deterministic nominal metadata: `0.500 m` when valid, otherwise `0`; this does **not** inject measurement noise |
| ADR | `MeasurementObservation::adr_cycles` when valid, otherwise `0` |
| ADR sigma | Deterministic nominal metadata: `0.050 cycles` when valid, otherwise `0`; this does **not** inject phase noise |
| Doppler | `MeasurementObservation::doppler_hz` when valid, otherwise `0` |
| C/N0 | `MeasurementObservation::cn0_dbhz` |
| lock time | `MeasurementObservation::lock_time_ns`, converted to seconds for serialization |
| tracking state | `4` (PLL tracking) when ADR is valid; `7` (FLL) for an emitted observation without valid ADR |
| SV channel number | Fixed `0`; V1 does not model receiver hardware channel allocation |
| phase lock | `adr_valid` |
| parity known | `adr_valid`; V1 has no unresolved half-cycle state after ADR becomes valid |
| code lock | `pseudorange_valid` |
| satellite-system bits | Central `SignalDefinition::constellation` mapping |
| signal-type bits | Central `SignalDefinition::novatel_oem7_signal_type` only |
| all remaining tracking-status bits | Fixed `0` in V1 |

RANGE records are sorted deterministically by RTKLIB satellite number and then `SignalId`. During REA signal-off the writer still
emits a scheduled `RANGEA` message with observation count `0`.

## PSRPOSA

| Field | V1 source |
| --- | --- |
| solution status / position type | `PositionSolution::status` / `type` |
| latitude / longitude / height | `PositionSolution`; V1 uses the RTKLIB WGS84 solution |
| undulation | Fixed `0.0000`; no geoid model is present in V1, so serialized height equals the WGS84 ellipsoidal height |
| datum | Fixed `WGS84` |
| latitude / longitude / height sigma | ENU standard deviations precomputed by the solution layer from RTKLIB ECEF covariance |
| station ID | Empty string |
| differential age / solution age | Fixed `0.000` |
| #SVs | Caller-supplied tracked-satellite count from receiver state |
| #solnSVs | `PositionSolution::used_satellites` when valid, otherwise `0` |
| reserved Uchar fields | Fixed `0` |
| reserved hex field | Fixed `00` |
| extended solution status | Fixed `00` |
| Galileo/BeiDou signal-used mask | Fixed `00` in V1 |
| GPS/GLONASS signal-used mask | Fixed `00` in V1 |

An invalid position is still serialized as `INSUFFICIENT_OBS,NONE` with zero numeric solution fields.

## PSRVELA

| Field | V1 source |
| --- | --- |
| solution status / velocity type | `VelocitySolution::status` / `type` |
| latency | Fixed `0.000`; V1 does not model receiver tracking-loop output latency |
| differential age | Fixed `0.000` |
| horizontal speed | `VelocitySolution::horizontal_speed_mps`, precomputed by the solution layer |
| track over ground | `VelocitySolution::track_over_ground_deg`, precomputed from the position hint used by the velocity solver |
| vertical speed | `VelocitySolution::vertical_speed_mps`, positive up |
| reserved | Fixed `0` |

Velocity validity remains independent of current position validity. If Doppler velocity is valid using a retained historical position
hint, PSRVELA remains valid even when the same epoch's PSRPOSA is invalid.
