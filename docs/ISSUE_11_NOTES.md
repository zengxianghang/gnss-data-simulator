# Issue #11 Implementation Notes

This change keeps measurement generation deterministic and inverse-compatible with the pinned RTKLIB broadcast solution model.

- Receiver clock bias/drift, measurement noise, and multipath remain zero in V1.
- Satellite clock bias/drift come from the selected broadcast ephemeris through the RTKLIB adapter.
- Signal-specific TGD/BGD/ISC/GLONASS differential delay data remain behind the RTKLIB adapter boundary.
- Atmosphere is injected explicitly; the measurement model does not select an atmosphere default.
- Carrier ambiguity is a deterministic integer keyed by satellite, signal, and tracking-start epoch. No PRNG is called in the V1 measurement path.
- Unsupported broadcast bias combinations are surfaced explicitly instead of silently applying the wrong message-family field.
- Doppler sign follows RTKLIB `resdop()` so zero receiver velocity/clock drift produces a zero-residual inverse measurement.
