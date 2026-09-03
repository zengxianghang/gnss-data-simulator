# Signal correlation metadata source index

This companion source index records the public specifications used for Issue #133. It is intentionally separated from implementation code so later Issue #134 reviews can verify correlation equations against the same source baseline.

- GPS: GPS.gov current Interface Control Documents / Interface Specifications index; IS-GPS-200N, IS-GPS-705J, IS-GPS-800J and their published revision notices.
- QZSS: Cabinet Office, Japan, IS-QZSS-PNT-006 (11 July 2024).
- Galileo: Galileo Open Service SIS ICD v2.2 (November 2025), current in-force Open Service reference published by the European GNSS Service Centre.
- BeiDou B1I: BDS-SIS-ICD-B1I-3.0 (February 2019).
- BeiDou B3I: BDS-SIS-ICD-B3I-1.0 (February 2018).
- BeiDou B1C: BDS-SIS-ICD-B1C-1.0 (December 2017). The pilot QMBOC complex-envelope convention uses the BOC(6,1) term with `-j` relative to BOC(1,1), so the central profile preserves **negative quadrature** rather than unsigned quadrature.
- BeiDou B2a: BDS-SIS-ICD-B2a-1.0 (December 2017).
- BeiDou B2b: BDS-SIS-ICD-B2b-1.0 (July 2020).
- GLONASS: public GLONASS open-signal interface definitions for legacy L1/L2 OF and L3OC; the central metadata records only the already-supported tracked code components and does not change FDMA carrier handling.

Verified profile anchors used by the permanent tests include GPS L1C pilot TMBOC, GPS L5 10.23 Mcps, Galileo E1-C CBOC(-), Galileo E5a/E5b 10.23 Mcps sideband components, BeiDou B1C pilot QMBOC(6,1,4/33) with the ICD-required negative-quadrature sign, and BeiDou B2a/B2b BPSK(10).

If a later implementation needs a signal whose current central entry is `kUnsupported`, that issue must identify the exact tracked component and authoritative waveform convention first. It must not replace `kUnsupported` with a generic BPSK triangle merely to make the signal pass through the DLL.
