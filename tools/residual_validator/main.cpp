#include "residual_validator.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void print_usage() {
    std::cerr << "Usage: validate-residuals --nav <rinex-nav> --truth <observation_truth.csv> --output <summary.csv> "
                 "[--atmosphere broadcast|none] [--code-limit-m <value>] [--doppler-limit-mps <value>]\n";
}

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

} // namespace

int main(int argc, char** argv) {
    gnss_sim::residual_validator::ValidationOptions options{};
    std::string output_path;
    double code_limit_m = 0.02;
    double doppler_limit_mps = 0.002;

    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        auto require_value = [&](const char* name) -> const char* {
            if (index + 1 >= argc) {
                std::cerr << "Missing value for " << name << '\n';
                return nullptr;
            }
            return argv[++index];
        };

        if (argument == "--nav") {
            const char* value = require_value("--nav");
            if (value == nullptr)
                return 2;
            options.nav_path = value;
        } else if (argument == "--truth") {
            const char* value = require_value("--truth");
            if (value == nullptr)
                return 2;
            options.observation_truth_path = value;
        } else if (argument == "--output") {
            const char* value = require_value("--output");
            if (value == nullptr)
                return 2;
            output_path = value;
        } else if (argument == "--atmosphere") {
            const char* value = require_value("--atmosphere");
            if (value == nullptr)
                return 2;
            const std::string mode = value;
            if (mode == "broadcast") {
                options.atmosphere_mode = gnss_sim::residual_validator::AtmosphereMode::kBroadcast;
            } else if (mode == "none") {
                options.atmosphere_mode = gnss_sim::residual_validator::AtmosphereMode::kNone;
            } else {
                std::cerr << "Unsupported atmosphere mode: " << mode << '\n';
                return 2;
            }
        } else if (argument == "--code-limit-m") {
            const char* value = require_value("--code-limit-m");
            if (value == nullptr || !parse_double(value, &code_limit_m) || code_limit_m <= 0.0) {
                std::cerr << "Invalid --code-limit-m\n";
                return 2;
            }
        } else if (argument == "--doppler-limit-mps") {
            const char* value = require_value("--doppler-limit-mps");
            if (value == nullptr || !parse_double(value, &doppler_limit_mps) || doppler_limit_mps <= 0.0) {
                std::cerr << "Invalid --doppler-limit-mps\n";
                return 2;
            }
        } else if (argument == "--no-diagnostic-health") {
            options.allow_diagnostic_health = false;
        } else if (argument == "--help" || argument == "-h") {
            print_usage();
            return 0;
        } else {
            std::cerr << "Unknown argument: " << argument << '\n';
            print_usage();
            return 2;
        }
    }

    if (options.nav_path.empty() || options.observation_truth_path.empty() || output_path.empty()) {
        print_usage();
        return 2;
    }

    gnss_sim::residual_validator::ValidationReport report{};
    std::string error_message;
    if (!gnss_sim::residual_validator::validate_observation_truth(options, &report, &error_message)) {
        std::cerr << "Residual validation failed: " << error_message << '\n';
        return 1;
    }
    if (!gnss_sim::residual_validator::write_summary_csv(report, output_path, &error_message)) {
        std::cerr << "Residual summary write failed: " << error_message << '\n';
        return 1;
    }

    bool limits_pass = true;
    std::size_t signal_rows = 0;
    for (const auto& row : report.rows) {
        if (row.scope != "signal") {
            continue;
        }
        ++signal_rows;
        if ((row.code_residuals > 0U && row.code_max_abs_m >= code_limit_m) ||
            (row.doppler_residuals > 0U && row.doppler_max_abs_mps >= doppler_limit_mps)) {
            limits_pass = false;
        }
        std::cout << row.signal_name << " code=" << row.code_residuals << " unavailable=" << row.code_unavailable
                  << " code_rms_m=" << row.code_rms_m << " code_p95_m=" << row.code_p95_abs_m
                  << " code_max_m=" << row.code_max_abs_m << " doppler=" << row.doppler_residuals
                  << " doppler_rms_mps=" << row.doppler_rms_mps << " doppler_p95_mps=" << row.doppler_p95_abs_mps
                  << " doppler_max_mps=" << row.doppler_max_abs_mps << '\n';
    }
    std::cout << "Validated input rows: " << report.input_rows << ", signal summaries: " << signal_rows << '\n';
    std::cout << "Limits: code < " << code_limit_m << " m, Doppler < " << doppler_limit_mps
              << " m/s => " << (limits_pass ? "PASS" : "FAIL") << '\n';
    return limits_pass ? 0 : 3;
}
