# Test data

Keep checked-in fixtures small, deterministic, redistributable, and independent of live network access.

`minimal/` contains compact fixtures used by unit/integration tests.

## `minimal/mixed_nav_2019.rnx`

This is a deliberately reduced RINEX 3.03 mixed-navigation fixture containing one representative broadcast record for GPS, GLONASS, Galileo, BeiDou, and QZSS. It was reduced from the public test file `brdm0500.19p` in Stanford GPS Lab's `navsu` repository (`navsu-unit-testing/test-data/brdm0500.19p`, commit `9485c964f9ef845b9cca0626cb7634c432fad37c`). Only the records needed to exercise RTKLIB mixed-NAV parsing are retained.

The fixture is test input only. It is not used as a physical-performance reference or as runtime simulator data.
