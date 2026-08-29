#ifndef GNSS_SIM_TOOLS_RANGEA_ROUNDTRIP_RANGEA_ROUNDTRIP_H_
#define GNSS_SIM_TOOLS_RANGEA_ROUNDTRIP_RANGEA_ROUNDTRIP_H_

#include <cstdint>
#include <istream>
#include <string>
#include <vector>

namespace gnss_sim {

struct ParsedRangeObservation {
    int satellite_number;
    int signal_id;
    double pseudorange_m;
    double adr_cycles;
    double doppler_hz;
    double cn0_dbhz;
    double lock_time_sec;
    unsigned int tracking_status;
    bool pseudorange_valid;
    bool adr_valid;
};

struct ParsedRangeEpoch {
    int gps_week;
    double sow_sec;
    std::vector<ParsedRangeObservation> observations;
};

bool parse_rangea_line_independent(const std::string& raw_line, ParsedRangeEpoch* epoch, std::string* error_message);

struct RangeaRoundtripSummary {
    std::uint64_t range_epochs;
    std::uint64_t parsed_observations;
    std::uint64_t selected_position_observations;
    std::uint64_t valid_position_epochs;
    double max_position_error_m;
    int max_error_gps_week;
    double max_error_sow_sec;
    double final_position_error_m;
    int final_position_gps_week;
    double final_position_sow_sec;
};

struct SerializedNavRoundtripSummary {
    RangeaRoundtripSummary position;
    std::uint64_t parsed_nav_records;
    std::uint64_t skipped_position_observations_without_nav;
    std::uint64_t parsed_ephemeris_records;
    std::uint64_t parsed_ionosphere_records;
    std::uint64_t gps_ionosphere_records;
    std::uint64_t skipped_position_epochs_without_ionosphere;
    std::uint64_t gps_ephemeris_records;
    std::uint64_t glonass_ephemeris_records;
    std::uint64_t galileo_ephemeris_records;
    std::uint64_t beidou_ephemeris_records;
    std::uint64_t qzss_ephemeris_records;
};

bool validate_serialized_navigation_roundtrip_stream(std::istream* input, double truth_latitude_deg,
                                                     double truth_longitude_deg, double truth_height_m,
                                                     double elevation_mask_deg, bool broadcast_atmosphere,
                                                     SerializedNavRoundtripSummary* summary,
                                                     std::string* error_message);

bool validate_serialized_navigation_roundtrip_file(const char* log_path, double truth_latitude_deg,
                                                   double truth_longitude_deg, double truth_height_m,
                                                   double elevation_mask_deg, bool broadcast_atmosphere,
                                                   SerializedNavRoundtripSummary* summary, std::string* error_message);

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
