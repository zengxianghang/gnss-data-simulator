#ifndef GNSS_SIM_TOOLS_SERIALIZED_NAV_ROUNDTRIP_SERIALIZED_NAV_ROUNDTRIP_H_
#define GNSS_SIM_TOOLS_SERIALIZED_NAV_ROUNDTRIP_SERIALIZED_NAV_ROUNDTRIP_H_

#include <cstdint>
#include <istream>
#include <string>

namespace gnss_sim {

struct SerializedNavRoundtripSummary {
    std::uint64_t nav_records;
    std::uint64_t gps_ephemeris_records;
    std::uint64_t glonass_ephemeris_records;
    std::uint64_t galileo_ephemeris_records;
    std::uint64_t beidou_ephemeris_records;
    std::uint64_t qzss_ephemeris_records;
    std::uint64_t ionosphere_records;
    std::uint64_t range_epochs;
    std::uint64_t parsed_observations;
    std::uint64_t selected_position_observations;
    std::uint64_t valid_position_epochs;
    int first_valid_position_gps_week;
    double first_valid_position_sow_sec;
    double max_position_error_m;
    int max_error_gps_week;
    double max_error_sow_sec;
    double final_position_error_m;
    int final_position_gps_week;
    double final_position_sow_sec;
};

bool validate_serialized_nav_roundtrip_stream(std::istream* input, double truth_latitude_deg,
                                              double truth_longitude_deg, double truth_height_m,
                                              double elevation_mask_deg, bool broadcast_atmosphere,
                                              SerializedNavRoundtripSummary* summary, std::string* error_message);

bool validate_serialized_nav_roundtrip_file(const char* log_path, double truth_latitude_deg, double truth_longitude_deg,
                                            double truth_height_m, double elevation_mask_deg, bool broadcast_atmosphere,
                                            SerializedNavRoundtripSummary* summary, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_TOOLS_SERIALIZED_NAV_ROUNDTRIP_SERIALIZED_NAV_ROUNDTRIP_H_
