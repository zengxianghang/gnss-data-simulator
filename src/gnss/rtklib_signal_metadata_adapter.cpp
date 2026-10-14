#include "gnss/rtklib_signal_metadata_adapter.h"

#include <cmath>
#include <cstdint>

extern "C" {
#include <rtklib_shared_api.h>
}

namespace gnss_sim {
namespace {

constexpr int kGlonassMinFcn = -7;
constexpr int kGlonassMaxFcn = 6;

uint32_t shared_system(RtklibSignalSystem system) {
    switch (system) {
        case RtklibSignalSystem::kGps:
            return RTKLIB_SHARED_SYS_GPS;
        case RtklibSignalSystem::kGlonass:
            return RTKLIB_SHARED_SYS_GLO;
        case RtklibSignalSystem::kGalileo:
            return RTKLIB_SHARED_SYS_GAL;
        case RtklibSignalSystem::kBeidou:
            return RTKLIB_SHARED_SYS_BDS;
        case RtklibSignalSystem::kQzss:
            return RTKLIB_SHARED_SYS_QZS;
    }
    return 0;
}

} // namespace

bool rtklib_signal_metadata(RtklibSignalSystem system, int prn, const char* rinex_signal_code, int glonass_fcn,
                            RtklibSignalMetadata* metadata) {
    if (prn <= 0 || rinex_signal_code == nullptr || metadata == nullptr) {
        return false;
    }
    if (system == RtklibSignalSystem::kGlonass && (glonass_fcn < kGlonassMinFcn || glonass_fcn > kGlonassMaxFcn)) {
        return false;
    }

    const uint32_t system_value = shared_system(system);
    if (system_value == 0) {
        return false;
    }

    rtklib_shared_signal_result_t shared{};
    shared.abi_version = RTKLIB_SHARED_ABI_VERSION;
    shared.struct_size = static_cast<uint32_t>(sizeof(shared));
    const int32_t shared_fcn =
        system == RtklibSignalSystem::kGlonass ? static_cast<int32_t>(glonass_fcn) : RTKLIB_SHARED_GLO_FCN_UNKNOWN;
    const int status = rtklib_shared_signal_query(system_value, static_cast<uint32_t>(prn), rinex_signal_code,
                                                  shared_fcn, nullptr, &shared);
    if (status != RTKLIB_SHARED_OK || shared.rtklib_code == 0 || shared.frequency_index < 0 ||
        !std::isfinite(shared.carrier_frequency_hz) || shared.carrier_frequency_hz <= 0.0 ||
        !std::isfinite(shared.wavelength_m) || shared.wavelength_m <= 0.0) {
        return false;
    }

    RtklibSignalMetadata result{};
    result.observation_code = static_cast<int>(shared.rtklib_code);
    result.frequency_index = shared.frequency_index + 1;
    result.carrier_frequency_hz = shared.carrier_frequency_hz;
    result.wavelength_m = shared.wavelength_m;
    *metadata = result;
    return true;
}

} // namespace gnss_sim
