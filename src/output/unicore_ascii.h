#ifndef GNSS_SIM_SRC_OUTPUT_UNICORE_ASCII_H_
#define GNSS_SIM_SRC_OUTPUT_UNICORE_ASCII_H_

#include "gnss_sim/sim_time.h"
#include "output/novatel_ascii.h"

#include <cstdint>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

namespace gnss_sim {
namespace unicore_ascii {

constexpr unsigned int kCpuIdle = 0U;
constexpr const char* kTimeReference = "GPS";
constexpr const char* kTimeStatus = "FINE";
constexpr unsigned int kReceiverStatus = 0U;
constexpr unsigned int kReserved = 0U;
constexpr unsigned int kVersion = 18U;
constexpr unsigned int kSequence = 0U;

inline bool frame(const char* log_name, const SimTime& time, const std::string& body, std::string* message) {
    if (log_name == nullptr || log_name[0] == '\0' || message == nullptr) {
        return false;
    }
    int gps_week = 0;
    std::int64_t tow_milliseconds = 0;
    if (!novatel_ascii::rounded_header_time(time, &gps_week, &tow_milliseconds)) {
        return false;
    }

    std::ostringstream payload;
    payload.imbue(std::locale::classic());
    payload << log_name << ',' << kCpuIdle << ',' << kTimeReference << ',' << kTimeStatus << ',' << gps_week << ','
            << tow_milliseconds << ',' << kReceiverStatus << ',' << kReserved << ',' << kVersion << ',' << kSequence
            << ';' << body;

    const std::string payload_text = payload.str();
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << '#' << payload_text << '*' << std::hex << std::nouppercase << std::setw(8) << std::setfill('0')
           << novatel_ascii::crc32(payload_text) << "\r\n";
    *message = output.str();
    return true;
}

} // namespace unicore_ascii
} // namespace gnss_sim

#endif // GNSS_SIM_SRC_OUTPUT_UNICORE_ASCII_H_
