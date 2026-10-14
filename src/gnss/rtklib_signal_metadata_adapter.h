#ifndef GNSS_SIM_SRC_GNSS_RTKLIB_SIGNAL_METADATA_ADAPTER_H_
#define GNSS_SIM_SRC_GNSS_RTKLIB_SIGNAL_METADATA_ADAPTER_H_

namespace gnss_sim {

enum class RtklibSignalSystem {
    kGps,
    kGlonass,
    kGalileo,
    kBeidou,
    kQzss,
};

struct RtklibSignalMetadata {
    int observation_code;
    // Preserve the simulator's existing RTKLIB frequency-index convention:
    // one-based, unlike the shared API's documented zero-based field.
    int frequency_index;
    double carrier_frequency_hz;
    double wavelength_m;
};

// Query immutable signal metadata through the pinned RTKLIB shared API.
// glonass_fcn is consulted only for GLONASS; non-GLONASS queries explicitly
// pass the shared API's unknown-FCN sentinel. The output is written only on
// success.
bool rtklib_signal_metadata(RtklibSignalSystem system, int prn, const char* rinex_signal_code, int glonass_fcn,
                            RtklibSignalMetadata* metadata);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_GNSS_RTKLIB_SIGNAL_METADATA_ADAPTER_H_
