#pragma once

#include <string>

namespace gnss_sim {

inline const std::string& simulator_device_marker() {
    static const std::string marker = "devicename=gnss-data-simulator\r\n";
    return marker;
}

} // namespace gnss_sim
