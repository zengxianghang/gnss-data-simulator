# GNSS Data Simulator

C/C++ GNSS test-data simulator for generating deterministic KS, REA, and TTFF datasets with physically consistent satellite measurements.

The simulator is planned to use the latest primary development branch of [zengxianghang/RTKLIB](https://github.com/zengxianghang/RTKLIB) as the GNSS navigation engine, use real IGS RINEX navigation data to compute satellite states, and generate NovAtel-style RANGEA/PSRPOSA/PSRVELA logs together with ground-truth files.

Current V1 defaults include a 3 degree elevation mask, EPH+ION navigation output, no multipath model, and 8 hours of generated data per scenario unless overridden by configuration.

See:

- [`docs/DESIGN_SPEC.md`](docs/DESIGN_SPEC.md) for the agreed architecture and V1 scope.
- [`docs/NAV_RECORDS.md`](docs/NAV_RECORDS.md) for supported NovAtel OEM7 and Unicore N4 navigation records.
- [`docs/V1_DEFAULTS.md`](docs/V1_DEFAULTS.md) for frozen V1 default parameters.
- [`docs/STARTUP_RECOVERY_MODEL.md`](docs/STARTUP_RECOVERY_MODEL.md) for the HOT/WARM/COLD TTFF and REA reacquisition timing model calibrated against current receiver specifications.
