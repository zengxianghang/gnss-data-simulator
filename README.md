# GNSS Data Simulator

C/C++ GNSS test-data simulator for generating deterministic KS, REA, and TTFF datasets with physically consistent satellite measurements.

The simulator is planned to use the latest primary development branch of [zengxianghang/RTKLIB](https://github.com/zengxianghang/RTKLIB) as the GNSS navigation engine, use real IGS RINEX navigation data to compute satellite states, and generate NovAtel-style RANGEA/PSRPOSA/PSRVELA logs together with ground-truth files.

See:

- [`docs/DESIGN_SPEC.md`](docs/DESIGN_SPEC.md) for the current agreed design principles and V1 scope.
- [`docs/NAV_RECORDS.md`](docs/NAV_RECORDS.md) for the frozen NovAtel OEM7 and Unicore N4 navigation-record output scope.
