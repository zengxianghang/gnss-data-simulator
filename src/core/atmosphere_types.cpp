#include "gnss_sim/atmosphere_types.h"

namespace gnss_sim {

const char* atmosphere_mode_name(AtmosphereMode atmosphere_mode) {
    switch (atmosphere_mode) {
        case AtmosphereMode::UNSPECIFIED:
            return "unspecified";
        case AtmosphereMode::NONE:
            return "none";
        case AtmosphereMode::BROADCAST:
            return "broadcast";
    }
    return "unknown";
}

} // namespace gnss_sim
