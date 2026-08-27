# Minimal GNSS test data

`multi_gnss_acceptance_nav.rnx` is a compact real RINEX 3.04 mixed broadcast-navigation fixture used by the V1 short acceptance suite. The original file name is `BRDM00DLR_S_20230730000_01D_MN.rnx`; its header identifies it as a DLR/GSOC merge based on CONGO and IGS tracking data and containing GPS/GLO/GAL/BDS/QZS/SBAS/IRNSS navigation records.

The canonical project data source for real RINEX navigation and observation material is the Wuhan University IGS Data Center (`igs.gnsswhu.cn`). The data center provides free HTTP/FTP downloads and exposes separate observation and broadcast-ephemeris searches. CI must not download data at test time: any source file used by a deterministic regression must be reduced to a small checked-in fixture first.

For V1 acceptance, the mixed NAV fixture is used only to exercise the simulator's normal RTKLIB truth/Receiver-NAV path and prove that GPS, GLONASS, Galileo, BeiDou and QZSS observations participate in one deterministic end-to-end run. Future CN0 / observation-characteristic calibration should use matching real RINEX observation files from the same data center rather than deriving empirical receiver characteristics from the navigation fixture.
