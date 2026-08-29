#include "serialized_nav_roundtrip.h"

#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 5 || argc > 7) {
        std::cerr << "usage: validate-serialized-nav-roundtrip <log> <lat_deg> <lon_deg> <height_m> "
                     "[elevation_mask_deg] [broadcast_atmosphere:0|1]\n";
        return 2;
    }
    const double latitude_deg = std::strtod(argv[2], nullptr);
    const double longitude_deg = std::strtod(argv[3], nullptr);
    const double height_m = std::strtod(argv[4], nullptr);
    const double elevation_mask_deg = argc >= 6 ? std::strtod(argv[5], nullptr) : 5.0;
    const bool broadcast_atmosphere = argc < 7 || std::string(argv[6]) != "0";

    gnss_sim::SerializedNavRoundtripSummary summary{};
    std::string error_message;
    if (!gnss_sim::validate_serialized_nav_roundtrip_file(argv[1], latitude_deg, longitude_deg, height_m,
                                                          elevation_mask_deg, broadcast_atmosphere, &summary,
                                                          &error_message)) {
        std::cerr << error_message << '\n';
        return 1;
    }
    std::cout << "nav_records=" << summary.nav_records << '\n';
    std::cout << "gps_eph=" << summary.gps_ephemeris_records << '\n';
    std::cout << "glo_eph=" << summary.glonass_ephemeris_records << '\n';
    std::cout << "gal_eph=" << summary.galileo_ephemeris_records << '\n';
    std::cout << "bds_eph=" << summary.beidou_ephemeris_records << '\n';
    std::cout << "qzss_eph=" << summary.qzss_ephemeris_records << '\n';
    std::cout << "ion=" << summary.ionosphere_records << '\n';
    std::cout << "range_epochs=" << summary.range_epochs << '\n';
    std::cout << "valid_position_epochs=" << summary.valid_position_epochs << '\n';
    std::cout << "max_3d_error_m=" << summary.max_position_error_m << '\n';
    std::cout << "final_3d_error_m=" << summary.final_position_error_m << '\n';
    return 0;
}
