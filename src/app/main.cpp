#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

namespace {

struct CliOptions {
    std::string config_path;
    std::string nav_path;
    std::string output_path;
    int start_week;
    double start_sow_sec;
    gnss_sim::AtmosphereMode atmosphere_override;
    bool have_start_week;
    bool have_start_sow;
    bool have_atmosphere_override;
};

void print_usage() {
    std::cerr << "Usage: gnss-data-simulator --config FILE --nav FILE --output FILE "
                 "--start-week WEEK --start-sow SEC [--atmosphere none|broadcast]\n";
}

bool parse_int(const char* text, int* value) {
    if (text == nullptr || value == nullptr || text[0] == '\0') {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (errno != 0 || end == nullptr || *end != '\0' || parsed < INT_MIN || parsed > INT_MAX) {
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
    if (errno != 0 || end == nullptr || *end != '\0') {
        return false;
    }
    *value = parsed;
    return true;
}

bool parse_atmosphere(const char* text, gnss_sim::AtmosphereMode* mode) {
    if (text == nullptr || mode == nullptr) {
        return false;
    }
    const std::string value(text);
    if (value == "none") {
        *mode = gnss_sim::AtmosphereMode::NONE;
        return true;
    }
    if (value == "broadcast") {
        *mode = gnss_sim::AtmosphereMode::BROADCAST;
        return true;
    }
    return false;
}

bool parse_cli(int argc, char** argv, CliOptions* options) {
    if (options == nullptr) {
        return false;
    }
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (index + 1 >= argc) {
            return false;
        }
        const char* value = argv[++index];
        if (argument == "--config") {
            options->config_path = value;
        } else if (argument == "--nav") {
            options->nav_path = value;
        } else if (argument == "--output") {
            options->output_path = value;
        } else if (argument == "--start-week") {
            if (!parse_int(value, &options->start_week)) {
                return false;
            }
            options->have_start_week = true;
        } else if (argument == "--start-sow") {
            if (!parse_double(value, &options->start_sow_sec)) {
                return false;
            }
            options->have_start_sow = true;
        } else if (argument == "--atmosphere") {
            if (!parse_atmosphere(value, &options->atmosphere_override)) {
                return false;
            }
            options->have_atmosphere_override = true;
        } else {
            return false;
        }
    }
    return !options->config_path.empty() && !options->nav_path.empty() && !options->output_path.empty() &&
           options->have_start_week && options->have_start_sow;
}

bool write_file_line(gnss_sim::SimulationLogKind, const gnss_sim::SimTime&, const char* data, std::size_t size,
                     void* user_data) {
    std::ofstream* output = static_cast<std::ofstream*>(user_data);
    if (output == nullptr || data == nullptr) {
        return false;
    }
    output->write(data, static_cast<std::streamsize>(size));
    return output->good();
}

} // namespace

int main(int argc, char** argv) {
    CliOptions options{};
    if (!parse_cli(argc, argv, &options)) {
        print_usage();
        return 2;
    }

    gnss_sim::SimConfig config{};
    std::string error_message;
    if (!gnss_sim::load_sim_config_json(options.config_path.c_str(), &config, &error_message)) {
        std::cerr << "Configuration error: " << error_message << '\n';
        return 2;
    }
    if (options.have_atmosphere_override) {
        config.atmosphere_mode = options.atmosphere_override;
    }
    if (config.atmosphere_mode == gnss_sim::AtmosphereMode::UNSPECIFIED) {
        std::cerr << "Configuration error: atmosphere_mode is unresolved; pass --atmosphere none|broadcast\n";
        return 2;
    }

    gnss_sim::SimTime start_time{};
    if (!gnss_sim::sim_time_from_week_sow(options.start_week, options.start_sow_sec, &start_time)) {
        std::cerr << "Invalid simulation start GPST\n";
        return 2;
    }

    std::ofstream output(options.output_path, std::ios::binary | std::ios::trunc);
    if (!output) {
        std::cerr << "Cannot open output file: " << options.output_path << '\n';
        return 3;
    }

    gnss_sim::SimulationRequest request{config, start_time, options.nav_path.c_str()};
    gnss_sim::SimulationOutputSink sink{write_file_line, &output};
    gnss_sim::SimulationRunStats stats{};
    if (!gnss_sim::run_simulation(request, sink, &stats, &error_message)) {
        std::cerr << "Simulation failed: " << error_message << '\n';
        return 4;
    }
    output.flush();
    if (!output.good()) {
        std::cerr << "Simulation output write failed\n";
        return 5;
    }

    std::cout << "gnss-data-simulator " << gnss_sim::simulator_version() << '\n';
    std::cout << "RTKLIB commit: " << gnss_sim::rtklib_commit_sha() << '\n';
    std::cout << "epochs=" << stats.total_epochs << " powered_epochs=" << stats.powered_epochs
              << " range_logs=" << stats.range_log_count << " nav_logs=" << stats.navigation_log_count << '\n';
    return 0;
}
