#include "transient_validator.h"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <locale>
#include <string>

namespace {

struct Options {
    std::string log_path;
    std::string truth_path;
    std::string events_path;
    std::string nav_path;
    std::string scenario;
    std::string output_path;
    double fade_duration_sec = 0.0;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double height_m = 0.0;
    double elevation_mask_deg = 5.0;
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
        const std::string argument = argv[index];
        if (argument == "--broadcast-atmosphere") {
            options->broadcast_atmosphere = true;
            continue;
        }
        if (index + 1 >= argc) {
            return false;
        }
        const char* value = argv[++index];
        if (argument == "--log") {
            options->log_path = value;
        } else if (argument == "--truth") {
            options->truth_path = value;
        } else if (argument == "--events") {
            options->events_path = value;
        } else if (argument == "--nav") {
            options->nav_path = value;
        } else if (argument == "--scenario") {
            options->scenario = value;
        } else if (argument == "--output") {
            options->output_path = value;
        } else if (argument == "--fade-duration") {
            if (!parse_double(value, &options->fade_duration_sec)) {
                return false;
            }
        } else if (argument == "--truth-lat") {
            options->have_latitude = parse_double(value, &options->latitude_deg);
            if (!options->have_latitude) {
                return false;
            }
        } else if (argument == "--truth-lon") {
            options->have_longitude = parse_double(value, &options->longitude_deg);
            if (!options->have_longitude) {
                return false;
            }
        } else if (argument == "--truth-height") {
            options->have_height = parse_double(value, &options->height_m);
            if (!options->have_height) {
                return false;
            }
        } else if (argument == "--elevation-mask") {
            if (!parse_double(value, &options->elevation_mask_deg)) {
                return false;
            }
        } else {
            return false;
        }
    }
    return !options->log_path.empty() && !options->truth_path.empty() && !options->events_path.empty() &&
           !options->nav_path.empty() && !options->scenario.empty() && !options->output_path.empty() &&
           options->have_latitude && options->have_longitude && options->have_height;
}

void usage() {
    std::cerr << "usage: validate-transient-observations --log FILE --truth observation_truth.csv --events "
                 "event_truth.csv --nav FILE --scenario LABEL --output summary.json --truth-lat DEG --truth-lon DEG "
                 "--truth-height M [--fade-duration SEC] [--elevation-mask DEG] [--broadcast-atmosphere]\n";
}

void print_window(const char* name, const gnss_sim::ObservationWindowStatistics& statistics) {
    std::cout << name << ".psr_samples=" << statistics.pseudorange_m.sample_count << '\n'
              << name << ".psr_rms_m=" << statistics.pseudorange_m.rms << '\n'
              << name << ".doppler_samples=" << statistics.doppler_mps.sample_count << '\n'
              << name << ".doppler_rms_mps=" << statistics.doppler_mps.rms << '\n'
              << name << ".adr_samples=" << statistics.adr_m.sample_count << '\n'
              << name << ".adr_rms_m=" << statistics.adr_m.rms << '\n'
              << name << ".cn0_samples=" << statistics.cn0_dbhz.sample_count << '\n'
              << name << ".cn0_rms_dbhz=" << statistics.cn0_dbhz.rms << '\n';
}

} // namespace

int main(int argc, char** argv) {
    Options options{};
    if (!parse_args(argc, argv, &options)) {
        usage();
        return 2;
    }

    const gnss_sim::TransientValidationOptions validator_options{
        options.scenario.c_str(), options.fade_duration_sec,  options.latitude_deg,         options.longitude_deg,
        options.height_m,         options.elevation_mask_deg, options.broadcast_atmosphere,
    };
    gnss_sim::TransientValidationSummary summary{};
    std::string error_message;
    if (!gnss_sim::validate_transient_observations_files(options.log_path.c_str(), options.truth_path.c_str(),
                                                         options.events_path.c_str(), options.nav_path.c_str(),
                                                         validator_options, &summary, &error_message)) {
        std::cerr << error_message << '\n';
        return 1;
    }
    if (!gnss_sim::write_transient_validation_json(options.output_path.c_str(), validator_options, summary,
                                                   &error_message)) {
        std::cerr << error_message << '\n';
        return 1;
    }

    std::cout.imbue(std::locale::classic());
    std::cout << std::fixed << std::setprecision(6) << "range_epochs=" << summary.range_epochs << '\n'
              << "parsed_observations=" << summary.parsed_observations << '\n'
              << "matched_observations=" << summary.matched_observations << '\n'
              << "unmatched_observations=" << summary.unmatched_observations << '\n';
    print_window("early", summary.early);
    print_window("recovery", summary.recovery);
    print_window("settled", summary.settled);
    print_window("fade", summary.fade);
    print_window("reacquisition_early", summary.reacquisition_early);
    std::cout << "rea.signal_off_range_epochs=" << summary.rea.signal_off_range_epochs << '\n'
              << "rea.signal_off_nonzero_epochs=" << summary.rea.signal_off_nonzero_epochs << '\n'
              << "rea.reacquisition_cycles=" << summary.rea.reacquisition_cycles << '\n'
              << "rea.max_first_psr_delay_sec=" << summary.rea.max_first_psr_delay_sec << '\n'
              << "rea.max_first_doppler_delay_sec=" << summary.rea.max_first_doppler_delay_sec << '\n'
              << "rea.max_first_adr_delay_sec=" << summary.rea.max_first_adr_delay_sec << '\n'
              << "rea.ambiguity_pairs_checked=" << summary.rea.ambiguity_pairs_checked << '\n'
              << "rea.ambiguity_pairs_changed=" << summary.rea.ambiguity_pairs_changed << '\n'
              << "positioning.valid_position_epochs=" << summary.positioning.valid_position_epochs << '\n'
              << "positioning.max_position_error_m=" << summary.positioning.max_position_error_m << '\n';
    return 0;
}
