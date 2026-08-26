#ifndef GNSS_SIM_SIM_TYPES_H_
#define GNSS_SIM_SIM_TYPES_H_

#include <cstdint>

namespace gnss_sim {

struct SimTime {
    int gps_week;
    std::int64_t tow_ns;
};

}  // namespace gnss_sim

#endif  // GNSS_SIM_SIM_TYPES_H_
