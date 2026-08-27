#ifndef GNSS_SIM_SRC_OUTPUT_NOVATEL_ASCII_H_
#define GNSS_SIM_SRC_OUTPUT_NOVATEL_ASCII_H_

#include "gnss_sim/sim_time.h"

#include <cstdint>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

namespace gnss_sim {
namespace novatel_ascii {

constexpr const char* kPort = "COM1";
constexpr const char* kTimeStatus = "FINE";
constexpr double kIdleTime = 0.0;
constexpr std::uint32_t kReceiverStatus = 0U;
constexpr unsigned int kReserved = 0U;
constexpr unsigned int kSoftwareVersion = 0U;

inline std::uint32_t crc32(const std::string& payload) {
    constexpr std::uint32_t kPolynomial = UINT32_C(0xEDB88320);
    std::uint32_t crc = 0U;
    for (unsigned char byte : payload) {
        crc ^= static_cast<std::uint32_t>(byte);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) != 0U ? (crc >> 1U) ^ kPolynomial : crc >> 1U;
        }
    }
    return crc;
}

inline bool rounded_header_time(const SimTime& time, int* gps_week, std::int64_t* tow_milliseconds) {
    if (gps_week == nullptr || tow_milliseconds == nullptr || time.gps_week < 0 || time.tow_ns < 0 ||
        time.tow_ns >= GPS_WEEK_NANOSECONDS) {
        return false;
    }
    constexpr std::int64_t kNanosecondsPerMillisecond = 1000000LL;
    constexpr std::int64_t kMillisecondsPerWeek = GPS_WEEK_SECONDS * 1000LL;
    std::int64_t rounded_ms = (time.tow_ns + kNanosecondsPerMillisecond / 2) / kNanosecondsPerMillisecond;
    int week = time.gps_week;
    if (rounded_ms >= kMillisecondsPerWeek) {
        rounded_ms -= kMillisecondsPerWeek;
        ++week;
    }
    *gps_week = week;
    *tow_milliseconds = rounded_ms;
    return true;
}

inline bool frame(const char* log_name, const SimTime& time, const std::string& body, std::string* message) {
    if (log_name == nullptr || log_name[0] == '\0' || message == nullptr) {
        return false;
    }
    int gps_week = 0;
    std::int64_t tow_milliseconds = 0;
    if (!rounded_header_time(time, &gps_week, &tow_milliseconds)) {
        return false;
    }

    std::ostringstream payload;
    payload.imbue(std::locale::classic());
    payload << log_name << ',' << kPort << ",0," << std::fixed << std::setprecision(1) << kIdleTime << ','
            << kTimeStatus << ',' << gps_week << ',' << (tow_milliseconds / 1000) << '.' << std::setw(3)
            << std::setfill('0') << (tow_milliseconds % 1000) << ',' << std::hex << std::nouppercase << std::setw(8)
            << std::setfill('0') << kReceiverStatus << std::dec << ',' << kReserved << ',' << kSoftwareVersion << ';'
            << body;

    const std::string payload_text = payload.str();
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << '#' << payload_text << '*' << std::hex << std::nouppercase << std::setw(8) << std::setfill('0')
           << crc32(payload_text) << "\r\n";
    *message = output.str();
    return true;
}

} // namespace novatel_ascii
} // namespace gnss_sim

#endif // GNSS_SIM_SRC_OUTPUT_NOVATEL_ASCII_H_
