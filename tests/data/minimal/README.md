# Minimal GNSS test data

`multi_gnss_acceptance_nav.rnx` is a compact real RINEX 3.04 mixed broadcast-navigation fixture used by the V1 short acceptance suite. The original file name is `BRDM00DLR_S_20230730000_01D_MN.rnx`; its header identifies it as a DLR/GSOC merge based on CONGO and IGS tracking data and containing GPS/GLO/GAL/BDS/QZS/SBAS/IRNSS navigation records.

`brd400dlr_rinex4_acceptance_nav.rnx` is the RINEX 4 counterpart used by the same acceptance suite. It is deterministically reduced by `tools/download_igs/materialize_brd4_fixture.py` from the real Wuhan University IGS Data Center product `BRD400DLR_S_20250030000_01D_MN.rnx.gz`. The selected source is RINEX 4.02. The checked-in fixture contains 87 navigation records: 66 EPH, 9 STO, 4 EOP and 8 ION. Legacy-compatible ephemerides cover GPS, GLONASS, Galileo, BeiDou and QZSS, while representative modern records cover GPS CNAV, QZSS CNAV/CNV2 and BeiDou CNV1/CNV2/CNV3. Exact source URL, source hashes, fixture hash, record counts and message-family inventory are frozen in `brd400dlr_rinex4_acceptance_nav.meta.json`.

`cn0_stream_acceptance_obs.rnx` is a deterministic synthetic RINEX 3.04 OBS fixture for the offline CN0 ingestion boundary. Its station/time are aligned with `multi_gnss_acceptance_nav.rnx`, while the OBS header/body deliberately exercises all 21 frozen V1 simulator signals across GPS, GLONASS, Galileo, BeiDou and QZSS. It includes modern BeiDou `5P`/`7D` observables and declares `SIGNAL STRENGTH UNIT = DBHZ`. The observation values are test-only deterministic values; this fixture validates parser/mapping/geometry semantics and is not empirical CN0 calibration evidence.

`jrc_has_2026001_e02.sp3`, `jrc_has_2026001_e02.clk`, and `jrc_has_2026001_e02_c6c.bia` are compact real Galileo HAS fixtures used to close the V1 Galileo E6 truth-state code-residual path with a coherent precise orbit/clock and E6-C (`C6C`) observable-specific code bias. They are reduced from the European Commission Joint Research Centre (JRC) GALILEO-HAS `SSRC00JRC0` daily products for 2026-001: `SSRC00JRC0_20260010000_01D_50S_ORB.SP3.gz`, `SSRC00JRC0_20260010000_01D_10S_CLK.CLK.gz`, and `SSRC00JRC0_20260010000_01D_50S_BIA.BIA.gz`, under `https://jeodpp.jrc.ec.europa.eu/ftp/jrc-opendata/GALILEO-HAS/PRODUCTS/daily/2026/001/`. The compact fixtures preserve original E02 numeric records: SP3 E02 orbit epochs around 00:01:40–00:14:10 GPS time, RINEX clock E02 records over 00:02:00–00:14:00, and Bias-SINEX E02 `C6C` OSB rows over 00:03:00–00:13:00. The SP3 header identifies the source as JRC Ispra/HASCQM and correction channel `SSRC00JRC0`. These files are precise/HAS acceptance evidence only; they do not replace the simulator's normal broadcast-NAV source for the other V1 signals.

The canonical project data source for real RINEX navigation and observation material is the Wuhan University IGS Data Center (`igs.gnsswhu.cn`). The data center provides free HTTP/FTP downloads and exposes separate observation and broadcast-ephemeris searches. Normal CI must not download data at test time: source files used by deterministic regressions are reduced to small checked-in fixtures first, with provenance sufficient to reproduce the reduction. Galileo HAS precise-product acceptance additionally uses the official JRC GALILEO-HAS source documented above; normal CI likewise consumes only its checked-in compact fixtures.

For V1 acceptance, the RINEX 3 and RINEX 4 mixed NAV fixtures exercise the simulator's normal pinned-RTKLIB Truth-NAV/Receiver-NAV path and prove that GPS, GLONASS, Galileo, BeiDou and QZSS observations can participate in deterministic end-to-end runs. The RINEX 4 fixture additionally verifies that modern navigation message types and RINEX 4 STO/EOP/ION records survive parsing. The JRC HAS fixture independently verifies Galileo E6 with a coherent precise state and `C6C` OSB rather than forcing an unsupported broadcast-E6 code-bias convention. CN0 parser tests use the synthetic OBS fixture only to freeze deterministic ingestion semantics. Empirical CN0 calibration under #38 must use real matching RINEX observation files from the canonical data source.
## GLONASS L3OC real-NAV availability

Issue #66 removed the former compact-test-only synthetic GLONASS L3OC overlay.
The test suite must not manufacture L3OC ephemeris or ISC values.

On 2026-08-29, the following authoritative DLR BRD400DLR products were downloaded
from the BKG IGS BRDC archive and scanned for records matching
`> EPH Rxx L3OC`. All files identified themselves as RINEX 4.02 and contained
zero L3OC records:

| Product day | gzip SHA256 | uncompressed RINEX SHA256 | L3OC records |
| --- | --- | --- | ---: |
| 2025-003 | `467f35074eb0f948df8d814ea460be8ede8daa521148d21e14be787c0c057ff0` | `ffef82cce9e3c75dbb693f8aaee67da1c5742f9f2bfeb3f723cff67dda3b32d7` | 0 |
| 2025-100 | `41ceaf963eea65f9f715d97fc86e74a099948146b82fbc185ba727242ae9fd65` | `fdb720e57a865b79a071a43c3de5cbe8fcdbb3ab4fecb8530ede7cffb2618024` | 0 |
| 2025-200 | `cc719286f5e414125c61f7d5189923e539fb32ac9252e34d4e44bb88b4f9843d` | `e2489163840c20f5a2497d444b6833780d36ae07b0d64973e1d662861e483a0c` | 0 |
| 2025-300 | `8f8d2f57b380f7a7bd1366f2fb44b17a30e7df47269a79e926aac17334654665` | `ae5bcff125521b0efeb76e450f915605d9c540c4c53fe4b48d6256cf6e6de2d5` | 0 |
| 2026-001 | `ff11082ccb87c8c678f9aa258487ec5d8e838e330f1c5e92ce289134ef15f805` | `72c0eaeb28e597244077872105c5b49372a38d1f2589de151d79f10015fdca9f` | 0 |
| 2026-100 | `14500ce323bc7b13541fa17eb04e483879ab25159c84ee264fa5b2ac7ea32dbd` | `8b21fb009fd11d25fc5ef12839d44020da76a23a59518477721b3e72cc86d407` | 0 |
| 2026-200 | `1bca00e827e2ed812c9fb1ad27a622b045e29f2c53374c5dfa1a0e4d5ca50f26` | `ad4820c951c89b149a0df7fd3010e6f020e0c8192b1b507f5dcfcd880b0a10cc` | 0 |

Archive pattern:
`https://igs.bkg.bund.de/root_ftp/IGS/BRDC/YYYY/DDD/BRD400DLR_S_YYYYDDD0000_01D_MN.rnx.gz`

Until a provenance-traceable authoritative L3OC record is available, compact CI
keeps GLONASS G3 observation/Doppler validation but requires its code-bias
coverage to be reported explicitly as unavailable. A synthetic fallback is forbidden.

## Galileo INAV/FNAV companion-identity fixture

`brd400dlr_rinex4_galileo_companion_nav.rnx` supports the real same-satellite
Galileo INAV/FNAV companion-identity regressions for issue #94. It is a verbatim
record filter/copy of the real Wuhan University IGS Data Center product
`BRD400DLR_S_20250030000_01D_MN.rnx.gz` (the same authoritative source file as
`brd400dlr_rinex4_acceptance_nav.rnx`); the six retained E02/E05 records are
byte-identical to their source records after LF line-ending normalization, and no
navigation field was modified, synthesized, or interpolated. Source URL, source
hashes, fixture hash, per-record identities (satellite, message family, IODnav,
Toe) and the extraction method are frozen in
`brd400dlr_rinex4_galileo_companion_nav.meta.json`.

Real cases covered: E02 carries two consecutive broadcast instances (IODnav 1 at
Toe 457800 and IODnav 2 at Toe 458400, each with both families), and E05 carries
a real same-IODnav/different-Toe pair (IODnav 87, INAV at Toe 433200 vs FNAV at
Toe 509400). The retained Toe SOW values are GPS week 2347 epochs and map to
2025-01-03 UTC wall-clock times via the 18 s GPS/GST-UTC leap (e.g. Toe 457800
is 07:09:42Z). A real same-Toe/different-IODnav case was searched with
`tools/download_igs/scan_galileo_nav_identities.py` over seven consecutive
BRD400DLR daily products (2025-001 through 2025-007, spanning GPS weeks 2347
and 2348; 41897 Galileo INAV/FNAV records, 29 satellites) using week-aware
navigation identities: 20605 matching pairs and 25199 same-IODnav/
different-Toe pairs exist, but zero same-Toe/different-IODnav pairs occur. The
case is documented as absent and was not fabricated; per-product hashes and
counts are frozen in `brd400dlr_rinex4_galileo_companion_nav.meta.json`, and
the scanner carries a deterministic self-test in the tooling CI job.
