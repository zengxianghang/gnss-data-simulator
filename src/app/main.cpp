#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

namespace {

void print_usage(const char* program) {
    std::cerr << "Usage:\n  " << program
              << " --config <config.json> --nav <input.rnx> --output <simulated.log> --week <gps-week> --sow <seconds>"
                 " [--cn0-model <cn0_model.csv>]\n";
}

bool parse_int(const char* text, int* value) {
    if (text == nullptr || value == nullptr || text[0] == '\0') {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed < 0 || parsed > INT_MAX) {
        return false;
    }
    *value = static_cast<int>(parsed);
    return true;
}

bool parse_double(const char* text, double* value) {
    if (text == nullptr || value == nullptr || text[0] == '\0') {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const double parsed = std::strtod(text, &end);
    if (errno != 0 || end == text || *end != '\0') {
        return false;
    }
    *value = parsed;
    return true;
}

} // namespace

int main(int argc, char** argv) {
    const char* config_path = nullptr;
    const char* nav_path = nullptr;
    const char* output_path = nullptr;
    const char* cn0_model_path = nullptr;
    int gps_week = -1;
    double sow_sec = -1.0;

    if (argc == 2 && (std::strcmp(argv[1], "--version") == 0 || std::strcmp(argv[1], "-v") == 0)) {
        std::cout << "gnss-data-simulator " << gnss_sim::simulator_version() << '\n';
        std::cout << "RTKLIB commit: " << gnss_sim::rtklib_commit_sha() << '\n';
        return 0;
    }

    for (int index = 1; index < argc; ++index) {
        if (std::strcmp(argv[index], "--help") == 0 || std::strcmp(argv[index], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        if (index + 1 >= argc) {
            print_usage(argv[0]);
            return 2;
        }
        const char* value = argv[++index];
        if (std::strcmp(argv[index - 1], "--config") == 0) {
            config_path = value;
        } else if (std::strcmp(argv[index - 1], "--nav") == 0) {
            nav_path = value;
        } else if (std::strcmp(argv[index - 1], "--output") == 0) {
            output_path = value;
        } else if (std::strcmp(argv[index - 1], "--cn0-model") == 0) {
            cn0_model_path = value;
        } else if (std::strcmp(argv[index - 1], "--week") == 0) {
            if (!parse_int(value, &gps_week)) {
                std::cerr << "ERROR: --week must be a non-negative integer\n";
                return 2;
            }
        } else if (std::strcmp(argv[index - 1], "--sow") == 0) {
            if (!parse_double(value, &sow_sec)) {
                std::cerr << "ERROR: --sow must be a finite GPS second-of-week value\n";
                return 2;
            }
        } else {
            std::cerr << "ERROR: unknown option " << argv[index - 1] << '\n';
            print_usage(argv[0]);
            return 2;
        }
    }

    if (config_path == nullptr || nav_path == nullptr || output_path == nullptr || gps_week < 0 || sow_sec < 0.0) {
        print_usage(argv[0]);
        return 2;
    }

    gnss_sim::SimConfig config{};
    std::string error_message;
    if (!gnss_sim::load_sim_config_json(config_path, &config, &error_message)) {
        std::cerr << "ERROR: " << error_message << '\n';
        return 1;
    }
    gnss_sim::SimTime start_time{};
    if (!gnss_sim::sim_time_from_week_sow(gps_week, sow_sec, &start_time)) {
        std::cerr << "ERROR: --sow must be within the GPS week\n";
        return 2;
    }

    const gnss_sim::SimulatorRunOptions options{nav_path, output_path, start_time, cn0_model_path};
    gnss_sim::SimulatorRunSummary summary{};
    if (!gnss_sim::run_simulator(config, options, &summary, &error_message)) {
        std::cerr << "ERROR: " << error_message << '\n';
        return 1;
    }

    std::cout << "scenario=" << gnss_sim::scenario_type_name(config.scenario) << " epochs=" << summary.scheduled_epochs
              << " powered=" << summary.powered_epochs << " range=" << summary.range_messages
              << " psrpos=" << summary.psrpos_messages << " psrvel=" << summary.psrvel_messages
              << " nav=" << summary.nav_messages << " valid_pos=" << summary.valid_position_epochs
              << " valid_vel=" << summary.valid_velocity_epochs << " cn0_model=" << summary.cn0_model_source << '\n';
    return 0;
}
