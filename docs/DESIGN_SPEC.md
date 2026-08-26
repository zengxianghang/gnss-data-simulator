# GNSS Data Simulator Design Specification

Status: initial design freeze for V1 planning

## 1. Purpose

This project will implement a C/C++ GNSS data simulator for generating deterministic KS, REA, and TTFF test datasets, including detailed per-signal RANGE observations.

The simulator is intended primarily for validating GNSS analysis pipelines. Generated observations must be physically consistent with real satellite geometry and with the generated navigation solutions, while still allowing deterministic receiver behavior and future fault injection.

The simulator must not generate RANGE, PSRPOS, and PSRVEL as unrelated random datasets. They must share the same underlying satellite state, receiver truth, measurement model, and receiver tracking state.

## 2. Core design principles

1. Use real GNSS navigation data to drive satellite states.
2. Use RTKLIB as the common GNSS navigation/positioning engine instead of reimplementing satellite orbit/clock propagation.
3. Separate simulator truth from the navigation data currently available to the simulated receiver.
4. Generate pseudorange, Doppler, carrier phase/ADR, CN0, lock time, and tracking status from a common per-signal state.
5. Generate PSRPOS/PSRVEL from the generated observations, preferably through RTKLIB SPP/velocity processing, instead of independently adding random position/velocity errors.
6. Keep KS, REA, and TTFF in one common scenario/event/state-machine framework.
7. All stochastic behavior must be reproducible from an explicit random seed.
8. Store complete ground truth so deterministic analysis results can be compared directly with the simulator truth.
9. Keep data-source/model building separate from runtime simulation. Large IGS observation datasets should not be required during every simulation run.
10. Prefer configuration over hard-coded scenario timing or thresholds.

## 3. RTKLIB baseline

Use:

- Repository: https://github.com/zengxianghang/RTKLIB
- Baseline policy: latest primary/mainline development version when implementation begins.
- Repository note: at the time this specification was created, the repository's GitHub default branch is `master`; the project requirement is to follow the latest mainline code rather than pinning to an older upstream RTKLIB release.

The RTKLIB integration should be isolated behind an adapter layer instead of exposing `nav_t`, `eph_t`, `geph_t`, and other RTKLIB internals throughout the simulator.

Recommended boundary:

```text
rtklib_adapter
  -> load RINEX NAV
  -> load SP3/CLK (future)
  -> satellite position/velocity
  -> satellite clock/clock drift
  -> wavelength/frequency helpers
  -> SPP/velocity solution
```

Existing RINEX OBS -> NovAtel RANGEA signal mapping already present in the RTKLIB fork should be reused/refactored where practical so the simulator and converter do not maintain conflicting signal definitions.

## 4. Supported constellations and signals

### GPS

- L1 C/A
- L1C
- L2P
- L2C
- L5Q

### QZSS

- L1 C/A
- L1C
- L2C
- L5Q

QZSS does not use GPS L2P as a broadcast QZSS signal, so it is intentionally not listed as a QZSS signal.

### GLONASS

- G1
- G2
- G3

GLONASS FDMA channel-dependent carrier frequency and wavelength must be handled explicitly. G1/G2 must not be treated as fixed GPS-like CDMA frequencies.

### Galileo

- E1
- E5a
- E5b
- E6

### BeiDou

- B1I
- B3I
- B1C
- B2a
- B2b

A central signal-definition table must map each simulated signal to:

```text
constellation
signal name
RINEX observation code
RTKLIB observation code
carrier frequency / wavelength
broadcast NAV message family
TGD / ISC / BGD mapping
NovAtel OEM7 RANGE signal type
```

## 5. External GNSS data

Primary data can be downloaded from the Wuhan University IGS data center:

http://www.igs.gnsswhu.cn/index.php/article/show/id/104/nid/83.html

The implementation must not hard-code a single download provider. Data downloading should be a separate helper/tooling layer so another IGS data center can be used if necessary.

### V1 runtime input

RINEX NAV is the primary truth input in V1.

Use real broadcast navigation data for:

- satellite orbit
- satellite velocity
- satellite clock
- satellite clock drift
- health
- group-delay/navigation signal parameters such as TGD, ISC, and BGD when present in the RINEX navigation records and supported by the RTKLIB parser

V1 is a **broadcast self-consistent** simulator: measurements are generated from broadcast navigation data and can be processed using the same broadcast model for deterministic loopback testing.

### RINEX OBS model-building input

IGS/MGEX RINEX observation files should be used offline to learn realistic receiver-independent/open-sky observation statistics, especially CN0 behavior.

Runtime simulation should consume a compact model file such as `cn0_model.csv` rather than reading large IGS OBS archives on each run.

### Future precise-truth mode

V2 may add:

- IGS SP3 precise orbit
- IGS CLK precise clock

In that mode observations can be generated from precise products while the simulated receiver/analysis side still uses broadcast navigation data, naturally producing broadcast orbit/clock residual effects.

## 6. Receiver truth

### V1 static receiver

Default position:

```text
latitude  = 20.0 deg
longitude = 120.0 deg
height    = 100.0 m
```

The position must be configurable.

Internal receiver position/velocity should use ECEF as the canonical representation. LLH is an input/output representation.

### Future dynamic receiver

The architecture must allow a truth trajectory file containing position and velocity, for example:

```text
Week,Sow,X,Y,Z,Vx,Vy,Vz
```

Dynamic support does not need to be completed in the first implementation milestone.

## 7. Simulation rates and time representation

Supported base sampling rates:

- 1 Hz
- 5 Hz
- 10 Hz
- 20 Hz
- 50 Hz

Do not advance the main simulation clock by repeated floating-point accumulation such as `sow += 1.0/rate`.

Use an integer time unit internally, preferably nanoseconds, and convert to GPST week/SOW only at boundaries.

Exact nominal intervals:

```text
1 Hz  = 1,000,000,000 ns
5 Hz  =   200,000,000 ns
10 Hz =   100,000,000 ns
20 Hz =    50,000,000 ns
50 Hz =    20,000,000 ns
```

The architecture should permit separate output rates for RANGE, PSRPOS, and PSRVEL, even if V1 defaults all three to the selected simulation rate.

## 8. V1 output logs

V1 must generate at least:

- NovAtel OEM7 RANGEA
- PSRPOSA
- PSRVELA
- event/scenario truth
- observation truth
- solution truth / run manifest

Navigation EPH/ION logging should be generated from the receiver navigation state so a simulated log can be processed independently by the downstream GNSS analysis pipeline.

## 9. Observation generation

Each per-satellite/per-signal observation should derive from shared truth state.

### 9.1 Pseudorange

Conceptual model:

```text
P = geometric range
  + receiver clock bias
  - satellite clock correction
  + ionosphere
  + troposphere
  + signal-specific broadcast code bias
  + multipath
  + measurement noise
  + injected fault
```

Transmission time must be solved iteratively rather than computing satellite position only at receive time.

Earth-rotation/Sagnac handling must be consistent with the RTKLIB processing model.

### 9.2 Doppler

Doppler must be generated from satellite velocity, receiver velocity, LOS range rate, receiver clock drift, satellite clock drift, carrier wavelength, and measurement error.

The generation formula should be designed as closely as practical as the inverse of the RTKLIB Doppler residual/velocity model. This enables near-zero residual deterministic loopback tests.

### 9.3 Carrier phase / ADR

Carrier phase must be temporally continuous while a signal remains locked.

The model must maintain a per-signal ambiguity and tracking state. ADR must not be independently randomized each epoch.

Loss of lock, cycle slip, receiver reset, and reacquisition may reset/change ambiguity according to the scenario.

### 9.4 TGD / ISC / BGD

Broadcast group-delay/code-bias parameters must come from the corresponding RINEX NAV message when available.

Satellite clock state and signal-specific group-delay correction should remain separate internal quantities. Apply the proper correction only when generating the observation for a specific signal.

If the selected RTKLIB version does not preserve a required RINEX 4 TGD/ISC/BGD field, prefer extending the RTKLIB parser/navigation structure rather than parsing RINEX NAV independently inside the simulator.

## 10. CN0 model

CN0 should be driven primarily by satellite elevation and statistics learned from real IGS/MGEX RINEX OBS data.

Do not use one global CN0-vs-elevation curve for every signal.

At minimum model by:

```text
constellation + signal + elevation bin
```

Recommended model statistics per bin:

```text
count
P05/P10/P25/P50/P75/P90/P95
mean/std
MAD
optional AR(1) temporal coefficient
Delta-CN0 P50/P90/P99
```

A practical V1 model is elevation-bin interpolation rather than forcing a global polynomial fit.

Conceptual runtime model:

```text
CN0 = nominal_system_signal_CN0(elevation)
    + receiver offset
    + optional satellite offset
    + slowly correlated variation
    + fast noise
    - scenario attenuation
```

IGS stations should primarily define an open-sky nominal baseline. Receiver-specific offsets/profiles should remain configurable because IGS geodetic receivers/antennas are not identical to the target receiver hardware.

CN0, lock time, pseudorange validity, Doppler validity, and ADR validity must share the same signal-tracking state machine.

## 11. Common signal tracking state machine

Suggested states:

```text
SIGNAL_OFF
SEARCHING
ACQUIRING
TRACKING
```

Per-signal state should control:

- reported CN0
- pseudorange validity
- Doppler validity
- ADR validity
- lock time
- observation availability

Lock time increments only while continuously tracking and resets on loss of lock/power reset as appropriate.

## 12. Scenario framework

KS, REA, and TTFF must share a common simulation loop and event/state-machine engine.

Do not implement three unrelated generators.

Conceptual loop:

```text
update scenario events
update receiver truth
update satellite truth
update receiver NAV state
update signal tracking states
generate measurements
run position/velocity solution
write receiver logs
write truth
```

## 13. KS

KS is continuous receiver operation with no intentional power/signal interruption unless additional events are configured.

Duration must be configurable.

A reasonable default may be supplied by the implementation, but duration is not a fixed protocol property.

KS will be the base scenario for later controlled fault injection such as:

- pseudorange bias
- Doppler bias
- CN0 degradation
- cycle slip
- satellite loss
- constellation-level degradation
- receiver clock events

## 14. REA behavior

Default cycle:

```text
SIGNAL ON  = 300 s
SIGNAL OFF = 10 s
repeat
```

Both values and cycle count must be configurable.

### SIGNAL OFF behavior

The receiver remains powered.

Logs continue at their configured rates:

```text
RANGEA   -> continue, observation count = 0
PSRPOSA  -> INSUFFICIENT_OBS / NONE
PSRVELA  -> INSUFFICIENT_OBS / NONE
GPST     -> continuous
NAV cache -> retained
receiver clock -> continues
```

Do not model REA signal-off as a receiver power-off.

### SIGNAL ON behavior

Signals reacquire independently.

Observation count should recover progressively instead of all signals becoming valid at exactly the same epoch.

CN0, PSR-valid, Doppler-valid, ADR-valid, and lock-time recovery are driven by each signal's tracking state.

Receiver NAV cache remains available through the outage.

## 15. TTFF behavior

Default cycle:

```text
POWER ON  = 300 s
POWER OFF = 30 s
repeat
```

Both values and cycle count must be configurable.

TTFF startup modes:

- HOT
- WARM
- COLD

Default mode: **HOT**.

### POWER OFF

The simulated receiver is off. Receiver observation and solution logs should not be emitted during the powered-off interval.

Tracking state, lock time, and ADR tracking state reset according to startup semantics.

### HOT startup

- Flash NAV cache: retained/all available EPH restored
- ION/UTC/navigation auxiliary parameters: retained
- approximate time: accurate
- approximate position: accurate
- tracking state: reset
- lock/ADR state: reset
- receiver NAV EPH availability: immediate at power-on
- EPH logs: emitted immediately/queued over a short output interval to simulate Flash -> RAM -> logger output

### WARM startup

- Flash NAV cache: retained/all available EPH restored
- ION/UTC/navigation auxiliary parameters: retained
- approximate time/position prior: less accurate than HOT
- tracking state: reset
- lock/ADR state: reset
- receiver NAV EPH availability: immediate at power-on
- EPH logs: emitted immediately/queued over a short output interval

HOT and WARM are therefore not distinguished by whether EPH exists. They are distinguished primarily by receiver prior-state quality/acquisition behavior.

### COLD startup

- Flash NAV cache: empty
- receiver NAV starts without usable EPH
- tracking/lock/ADR state: reset
- EPH becomes available progressively as navigation messages are collected
- each newly completed EPH is inserted into Receiver NAV and its EPH log is emitted

COLD must not expose the complete truth NAV database directly to the simulated receiver.

## 16. Truth NAV vs Receiver NAV

Maintain two logically independent navigation stores:

```text
RINEX NAV
  |
  +--> Truth NAV
  |      complete data available to the simulator
  |      used to generate physically correct observations
  |
  +--> Receiver NAV
         only data currently available to the simulated receiver
         used by PSRPOS/PSRVEL processing
```

This distinction is critical for COLD TTFF.

A receiver may already track a satellite and output RANGE observations before a complete EPH for that satellite has been decoded. That observation must not be usable for the simulated receiver's SPP until its Receiver NAV contains the required EPH.

## 17. Cold-start ephemeris acquisition

Do not implement COLD EPH availability as one arbitrary per-satellite random delay.

Model navigation-message acquisition from the actual navigation-message family, its frame/page/message schedule, and the receiver's acquisition time relative to that schedule.

General rule:

```text
EPH availability time
  = signal acquisition time
  + time required to collect all required NAV fragments
    from the actual frame/page/message phase
```

Low CN0 may later be used to simulate failed NAV-message CRC/decoding and waiting for a repeat.

### GPS LNAV

- 6 s per subframe
- 30 s frame
- complete normal broadcast clock/orbit requires the relevant information from subframes 1, 2, and 3
- track collection state rather than assigning a fixed 30 s delay

Example state:

```text
SF1 received
SF2 received
SF3 received
IODE/IODC consistent
-> EPH AVAILABLE
```

### GPS/QZSS modern NAV

Use the appropriate CNAV/CNAV-2 message family for the signal instead of assuming LNAV timing.

For message-type-based CNAV families, completion must depend on collecting all required message types, not simply on one message period.

### GLONASS FDMA

Use its 2 s string / 30 s frame organization.

The immediate-data strings required for current satellite ephemeris should be tracked as fragments; the roughly 30-minute ephemeris content update interval must not be confused with the much shorter time required by a cold receiver to collect a currently broadcast ephemeris.

### Galileo

I/NAV and F/NAV must use their own page/word schedules.

For I/NAV, complete normal CED requires the corresponding WT fragments with consistent `IODnav`.

Reduced CED/RedCED may be added later as a separate fast-TTFF feature. V1 should not silently treat reduced CED as the same thing as a fully decoded normal RTKLIB ephemeris unless explicitly implemented.

### BeiDou

D1, D2, B-CNAV1, B-CNAV2, and B-CNAV3 must not share one fixed ephemeris delay.

For message-type-based B-CNAV formats, collect the required EPH and clock message types and validate the relevant issue-of-data consistency before marking EPH available.

### Navigation phase

Where practical, derive frame/page/message phase from the actual simulation GNSS time rather than restarting each NAV schedule at `POWER_ON`.

This naturally makes COLD TTFF depend on the power-on epoch without requiring arbitrary random TTFF values.

## 18. PSRPOS/PSRVEL solution behavior

PSRPOS and PSRVEL should be generated from the simulated observation/navigation state using RTKLIB processing where practical.

### Valid solution

On successful SPP/velocity solution:

```text
Solution Status = SOL_COMPUTED
Position/Velocity Type = SINGLE
```

### No solution

When positioning/velocity cannot be solved, continue outputting the corresponding log and use:

```text
Solution Status = INSUFFICIENT_OBS
Position/Velocity Type = NONE
```

Position and velocity validity must be evaluated independently. Do not use one shared boolean for both.

It must therefore be possible, if the data supports it, to have:

```text
PSRPOS = SOL_COMPUTED / SINGLE
PSRVEL = INSUFFICIENT_OBS / NONE
```

or the reverse when appropriate.

## 19. Deterministic randomness

All stochastic behavior must use one controlled PRNG framework with an explicit seed, including:

- CN0 perturbations
- measurement noise
- reacquisition delay variation
- multipath
- future fault injection
- NAV decode failures if modeled probabilistically

Requirement:

```text
same simulator version
+ same inputs/configuration
+ same seed
=> reproducible numerical output
```

Prefer byte-identical output where practical.

## 20. Ground-truth outputs

Truth data should be more detailed than the minimum currently needed by the analysis pipeline.

Recommended observation truth fields include:

```text
Week,Sow
System,PRN,Signal
RxX,RxY,RxZ,RxVx,RxVy,RxVz
TxTime
SatX,SatY,SatZ,SatVx,SatVy,SatVz
Azimuth,Elevation
GeometricRange,RangeRate
ReceiverClockBias,ReceiverClockDrift
SatelliteClockBias,SatelliteClockDrift
Iono,Trop
TGD,ISC,BGD,FinalCodeBias
NominalCN0,FinalCN0
PsrNoise,PsrMultipath,PsrFault
DopplerNoise,DopplerFault
Ambiguity,CycleSlip
LockTime,PsrValid,DopplerValid,AdrValid
FinalPseudorange,FinalDoppler,FinalADR
```

Recommended event truth:

```text
EventId,Type,Week,Sow
```

Examples:

- POWER_ON
- POWER_OFF
- SIGNAL_ON
- SIGNAL_OFF
- EPH_AVAILABLE
- FIRST_POSITION_SOLUTION
- FIRST_VELOCITY_SOLUTION

## 21. Configuration

Use a configuration file rather than compile-time scenario values.

JSON is the preferred V1 configuration format.

Example conceptual TTFF configuration:

```json
{
  "scenario": "TTFF",
  "sampling_rate_hz": 10,
  "cycle_count": 10,
  "receiver": {
    "lat_deg": 20.0,
    "lon_deg": 120.0,
    "height_m": 100.0
  },
  "ttff": {
    "power_on_sec": 300.0,
    "power_off_sec": 30.0,
    "startup_mode": "HOT"
  },
  "seed": 12345678
}
```

Example conceptual REA configuration:

```json
{
  "scenario": "REA",
  "sampling_rate_hz": 10,
  "cycle_count": 10,
  "receiver": {
    "lat_deg": 20.0,
    "lon_deg": 120.0,
    "height_m": 100.0
  },
  "rea": {
    "signal_on_sec": 300.0,
    "signal_off_sec": 10.0
  },
  "seed": 12345678
}
```

The exact final schema may evolve, but scenario timing and startup mode must remain configurable.

## 22. Suggested module layout

```text
src/
  main.cpp
  rtklib_adapter.cpp
  sim_time.cpp
  sim_scenario.cpp
  sim_receiver.cpp
  sim_satellite.cpp
  sim_nav_acquisition.cpp
  sim_signal_tracking.cpp
  sim_measurement.cpp
  sim_solution.cpp
  sim_random.cpp
  novatel_range.cpp
  novatel_navigation.cpp
  novatel_psrpos.cpp
  novatel_psrvel.cpp
  truth_writer.cpp

model_builder/
  cn0_model_builder...

config/
  ks_normal.json
  rea_normal.json
  ttff_hot.json
  ttff_warm.json
  ttff_cold.json

tests/
```

The code style should remain compatible with straightforward C-style C/C++ implementation and avoid unnecessary framework complexity.

## 23. Recommended implementation phases

### Phase 1 - deterministic broadcast geometry

- RTKLIB adapter
- RINEX NAV input
- static receiver
- supported signal table
- satellite position/velocity/clock
- transmission-time iteration
- zero-noise pseudorange/Doppler/carrier truth
- RANGEA output
- truth output

### Phase 2 - solution loopback

- RTKLIB SPP/velocity using Receiver NAV
- PSRPOSA
- PSRVELA
- `SOL_COMPUTED/SINGLE`
- `INSUFFICIENT_OBS/NONE`

### Phase 3 - signal tracking and scenario engine

- CN0 model
- lock time
- PSR/Doppler/ADR validity
- KS
- REA
- TTFF HOT/WARM

### Phase 4 - COLD navigation acquisition

- per-satellite NAV fragment acquisition
- EPH availability state
- navigation-message-family timing
- Receiver NAV population
- EPH log emission

### Phase 5 - observation realism and faults

- CN0-correlated measurement noise
- multipath
- cycle slips
- PSR/Doppler biases
- constellation/satellite faults

### Phase 6 - precise-truth mode

- SP3
- CLK
- broadcast-vs-precise truth tests

## 24. Minimum validation requirements

The implementation should include deterministic automated tests for at least:

1. zero-noise geometric pseudorange loopback
2. zero-noise Doppler/velocity loopback
3. RTKLIB SPP returns the configured static receiver position
4. static PSRVEL is near zero when expected
5. signal-specific TGD/ISC/BGD application
6. GLONASS FDMA frequency/wavelength handling
7. 1/5/10/20/50 Hz exact time stepping
8. GPS-week boundary crossing
9. REA signal-off logs continue with RANGE count zero and solution status `INSUFFICIENT_OBS/NONE`
10. REA reacquisition progressively restores observations
11. HOT/WARM EPH is immediately available from simulated Flash
12. COLD EPH becomes progressively available from NAV-message acquisition
13. PSRPOS and PSRVEL validity are independent
14. same seed/configuration reproduces the same outputs
15. every supported constellation/signal maps correctly to RINEX/RTKLIB/OEM7 identifiers

## 25. Items intentionally left configurable or for later refinement

The following do not need to be hard-coded into the first architecture:

- elevation mask default
- exact per-signal measurement noise coefficients
- multipath model complexity
- receiver-specific CN0 calibration offsets
- exact HOT/WARM acquisition delay distributions
- NAV message decode failure model versus CN0
- precise SP3/CLK truth
- dynamic receiver trajectory
- advanced Galileo RedCED handling
- detailed fault-injection catalog

These should be implemented as configuration/model extensions without changing the core Truth NAV / Receiver NAV / tracking-state architecture.
