#ifndef GNSS_SIM_ATMOSPHERE_TYPES_H_
#define GNSS_SIM_ATMOSPHERE_TYPES_H_

namespace gnss_sim {

enum class AtmosphereMode {
    UNSPECIFIED,
    NONE,
    BROADCAST,
};

const char* atmosphere_mode_name(AtmosphereMode atmosphere_mode);

} // namespace gnss_sim

#endif // GNSS_SIM_ATMOSPHERE_TYPES_H_
