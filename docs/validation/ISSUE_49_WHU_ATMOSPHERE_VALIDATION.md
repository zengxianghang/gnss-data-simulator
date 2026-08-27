# Issue #49 WHU atmosphere validation

This note records the external real-data acceptance evidence for issue #49 / PR #50. The network download and validation workflow itself is intentionally not part of normal CI; normal CI remains offline and uses the checked-in compact BRD400DLR fixture.

## Source

- Product: `BRD400DLR_S_20250030000_01D_MN.rnx.gz`
- Source: WHU IGS data center daily broadcast-navigation archive
- RINEX: BRD400DLR RINEX 4.02 multi-GNSS navigation
- Pinned RTKLIB commit used by the simulator/validator: `dc596ba725ccaa5ab5963d7e7ec85b52ae743969`
- Live-validation compressed SHA-256: `fb84d4046b06e905e8e4ec0efb82f0e9ad044bc44d664cc0209cb9b4c92b9512`
- Live-validation uncompressed SHA-256: `b11c638eea42978b8bd6aa8b65a5099fe6556dfe527bc037ed481d2b239afc42`

## Simulation

- Scenario: KS
- GPST start: week 2347 / SOW 436500
- Duration: 600 s
- Rate: 1 Hz
- Receiver truth: 20 deg N, 120 deg E, 100 m
- Elevation mask: 3 deg
- Atmosphere: `broadcast`
- Measurement noise, multipath, receiver clock bias/drift: zero
- Simulator result: 600 RANGE epochs, 598 valid position epochs, 598 valid velocity epochs

## Per-observation atmosphere parity

The independent validator compared each valid primary observation against pinned RTKLIB `IONOOPT_BRDC` + `TROPOPT_SAAS` using the full WHU navigation file.

- Observation-truth rows: 126,062
- Valid primary observations: 28,458
- RTKLIB correction failures: 0
- Overall ionosphere simulator mean: 5.651545198 m
- Overall ionosphere RTKLIB mean: 5.651545198 m
- Ionosphere RMS delta: 0 m
- Ionosphere maximum absolute delta: 0 m
- Troposphere simulator/RTKLIB mean: 7.428508295 m
- Troposphere RMS delta: 0 m
- Troposphere maximum absolute delta: 0 m
- Combined ionosphere + troposphere maximum absolute delta: 0 m
- Pseudorange decomposition RMS delta: 2e-9 m
- Pseudorange decomposition maximum absolute delta: 7e-9 m

GPS, GLONASS, Galileo, BeiDou and QZSS each independently had zero maximum ionosphere, troposphere and combined-atmosphere delta. GLONASS therefore also validates use of the actual broadcast FCN in signal-frequency scaling.

## Independent RANGEA -> RTKLIB pntpos loopback

Generated OEM7 RANGEA primary observations were parsed independently and solved by pinned RTKLIB `pntpos()` with broadcast ionosphere and Saastamoinen troposphere enabled.

- RANGE epochs: 600
- RANGE parse/CRC failures: 0
- Valid all-system solutions: 598 / 600 (99.6667%)
- First valid epoch: 3
- 3D RMS: 0.091829014 m
- 3D P50: 0.088492923 m
- 3D P95: 0.098593030 m
- 3D P99: 0.099022586 m
- 3D maximum: 0.178487910 m

The issue #49 acceptance gate (at least 590 valid epochs and 3D maximum < 0.5 m) passes. The remaining roughly 0.09 m RMS residual is not an atmosphere mismatch because the per-observation atmosphere delta is exactly zero; it should be investigated separately against pinned RTKLIB code-bias/`prange()` conventions.

## Evidence

External validation workflow run: `33055133199`.
Artifact: `whu-issue49-atmosphere-validation`, digest `sha256:1aa66ef8f08079e71bda1d9388a894e9b501254709a09c9578d88bc18f5da0b2`.
