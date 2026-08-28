#include "rangea_roundtrip.h"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <locale>
#include <string>

namespace {

struct Options {
    std::string log_path;
    std::string nav_path;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double height_m = 0.0;
    double elevation_mask_deg = 0.0;
    bool have_latitude = false;
    bool have_longitude = false;
    bool have_height = false;
    bool broadcast_atmosphere = false;
};

bool parse_double(const char* text, double* value) {
    if (text == nullptr || value == nullptr) {
        return false;
    }
    char* end = nullptr;
    const double parsed = std::strtod(text, &end);
    if (end == text || end == nullptr || *end != '\0') {
        return false;
    }
    *value = parsed;
    return true;
}

bool parse_args(int argc, char** argv, Options* options) {
    if (options == nullptr) {
        return false;
    }
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "--broadcast-atmosphere") {
            options->broadcast_atmosphere = true;
            continue;
        }
        if (index + 1 >= argc) {
            return false;
        }
        const char* value = argv[++index];
        if (arg == "--log") {
            options->log_path = value;
        } else if (arg == "--nav") {
            options->nav_path = value;
        } else if (arg == "--truth-lat") {
            options->have_latitude = parse_double(value, &options->latitude_deg);
            if (!options->have_latitude) {
                return false;
            }
        } else if (arg == "--truth-lon") {
            options->have_longitude = parse_double(value, &options->longitude_deg);
            if (!options->have_longitude) {
                return false;
            }
        } else if (arg == "--truth-height") {
            options->have_height = parse_double(value, &options->height_m);
            if (!options->have_height) {
                return false;
            }
        } else if (arg == "--elevation-mask") {
            if (!parse_double(value, &options->elevation_mask_deg)) {
                return false;
            }
        } else {
            return false;
        }
    }
    return !options->log_path.empty() && !options->nav_path.empty() && options->have_latitude &&
           options->have_longitude && options->have_height;
}

void usage() {
    std::cerr << "usage: validate-rangea-roundtrip --log FILE --nav FILE --truth-lat DEG --truth-lon DEG "
                 "--truth-height M [--elevation-mask DEG] [--broadcast-atmosphere]\n";
}

} // namespace

int main(int argc, char** argv) {
    Options options{};
    if (!parse_args(argc, argv, &options)) {
        usage();
        return 2;
    }

    gnss_sim::RangeaRoundtripSummary summary{};
    std::string error_message;
    if (!gnss_sim::validate_rangea_roundtrip_file(
            options.log_path.c_str(), options.nav_path.c_str(), options.latitude_deg, options.longitude_deg,
            options.height_m, options.elevation_mask_deg, options.broadcast_atmosphere, &summary, &error_message)) {
        std::cerr << error_message << '\n';
        return 1;
    }

    std::cout.imbue(std::locale::classic());
    std::cout << "range_epochs=" << summary.range_epochs << '\n'
              << "parsed_observations=" << summary.parsed_observations << '\n'
              << "selected_position_observations=" << summary.selected_position_observations << '\n'
              << "valid_position_epochs=" << summary.valid_position_epochs << '\n'
              << std::fixed << std::setprecision(6) << "max_position_error_m=" << summary.max_position_error_m << '\n'
              << "max_error_gpst=" << summary.max_error_gps_week << '/' << summary.max_error_sow_sec << '\n';
    return 0;
}
