#include "tools/build_cn0_model/rinex_obs_stream.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

std::string csv_escape(const std::string& value) {
    if (value.find_first_of(",\"\r\n") == std::string::npos) {
        return value;
    }
    std::string result = "\"";
    for (const char character : value) {
        if (character == '\"') {
            result += "\"\"";
        } else {
            result += character;
        }
    }
    result += '\"';
    return result;
}

void usage(const char* program) {
    std::cerr << "Usage: " << program << " --obs <rinex.obs> --nav <rinex.nav> --output <samples.csv>\n";
}

} // namespace

int main(int argc, char** argv) {
    std::string observation_path;
    std::string navigation_path;
    std::string output_path;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if ((argument == "--obs" || argument == "--nav" || argument == "--output") && index + 1 < argc) {
            const std::string value = argv[++index];
            if (argument == "--obs") {
                observation_path = value;
            } else if (argument == "--nav") {
                navigation_path = value;
            } else {
                output_path = value;
            }
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (observation_path.empty() || navigation_path.empty() || output_path.empty()) {
        usage(argv[0]);
        return 2;
    }

    std::ofstream output(output_path);
    if (!output) {
        std::cerr << "Cannot open output CSV: " << output_path << '\n';
        return 1;
    }
    output << "gps_week,sow_sec,station,constellation,prn,signal,signal_strength,cn0_dbhz,azimuth_rad,"
              "elevation_rad,validity,signal_strength_unit_status\n";
    output << std::setprecision(15);

    gnss_sim::cn0_builder::RinexObsProvenance provenance{};
    gnss_sim::cn0_builder::RinexObsStreamSummary summary{};
    std::string error;
    const bool ok = gnss_sim::cn0_builder::stream_rinex_cn0_samples(
        observation_path, navigation_path,
        [&](const gnss_sim::cn0_builder::RinexCn0Sample& sample) {
            output << sample.time.gps_week << ','
                   << static_cast<double>(sample.time.tow_ns) / 1000000000.0 << ','
                   << csv_escape(provenance.station_name) << ','
                   << gnss_sim::cn0_builder::constellation_name(sample.constellation) << ',' << sample.prn << ','
                   << sample.rinex_signal_code << ',' << sample.signal_strength_value << ',';
            if (std::isfinite(sample.cn0_dbhz)) {
                output << sample.cn0_dbhz;
            }
            output << ',';
            if (std::isfinite(sample.azimuth_rad)) {
                output << sample.azimuth_rad;
            }
            output << ',';
            if (std::isfinite(sample.elevation_rad)) {
                output << sample.elevation_rad;
            }
            output << ',' << gnss_sim::cn0_builder::cn0_sample_validity_name(sample.validity) << ','
                   << gnss_sim::cn0_builder::signal_strength_unit_status_name(provenance.signal_strength_unit_status)
                   << '\n';
            return static_cast<bool>(output);
        },
        &provenance, &summary, &error);

    if (!ok) {
        std::cerr << "CN0 extraction failed: " << error << '\n';
        return 1;
    }
    output.close();
    if (!output) {
        std::cerr << "Failed while writing output CSV: " << output_path << '\n';
        return 1;
    }

    std::cerr << "RINEX " << provenance.rinex_version << ", station=" << provenance.station_name
              << ", signal-strength-unit="
              << gnss_sim::cn0_builder::signal_strength_unit_status_name(provenance.signal_strength_unit_status)
              << ", epochs=" << summary.epochs << ", records=" << summary.observation_records
              << ", samples=" << summary.emitted_samples << ", valid_dbhz=" << summary.valid_dbhz_samples
              << ", missing_s=" << summary.missing_signal_strength
              << ", unsupported=" << summary.unsupported_signal_observables << '\n';
    for (const std::string& observable : summary.unsupported_observables) {
        std::cerr << "Unsupported S observable: " << observable << '\n';
    }
    return 0;
}
