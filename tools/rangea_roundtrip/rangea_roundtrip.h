#ifndef GNSS_SIM_TOOLS_RANGEA_ROUNDTRIP_RANGEA_ROUNDTRIP_H_
#define GNSS_SIM_TOOLS_RANGEA_ROUNDTRIP_RANGEA_ROUNDTRIP_H_

#include <cstdint>
#include <istream>
#include <string>

namespace gnss_sim {

struct RangeaRoundtripSummary {
    std::uint64_t range_epochs;
    std::uint64_t parsed_observations;
    std::uint64_t selected_position_observations;
    std::uint64_t valid_position_epochs;
    double max_position_error_m;
    int max_error_gps_week;
    double max_error_sow_sec;
};

bool validate_rangea_roundtrip_stream(std::istream* input, const char* rinex_nav_path, double truth_latitude_deg,
                                      double truth_longitude_deg, double truth_height_m, double elevation_mask_deg,
                                      bool broadcast_atmosphere, RangeaRoundtripSummary* summary,
                                      std::string* error_message);

bool validate_rangea_roundtrip_file(const char* log_path, const char* rinex_nav_path, double truth_latitude_deg,
                                    double truth_longitude_deg, double truth_height_m, double elevation_mask_deg,
                                    bool broadcast_atmosphere, RangeaRoundtripSummary* summary,
                                    std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_TOOLS_RANGEA_ROUNDTRIP_RANGEA_ROUNDTRIP_H_
