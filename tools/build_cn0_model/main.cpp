#include "tools/build_cn0_model/cn0_builder.h"

#include <cerrno>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool parse_double(const char* text, double* value) {
    if (text == nullptr || value == nullptr) {
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

bool parse_u64(const char* text, std::uint64_t* value) {
    if (text == nullptr || value == nullptr || text[0] == '-') {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const unsigned long long parsed = std::strtoull(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0') {
        return false;
    }
    *value = static_cast<std::uint64_t>(parsed);
    return true;
}

void usage(const char* program) {
    std::cerr << "Usage: " << program
              << " --source <rinex.obs> <rinex.nav> [--source <rinex.obs> <rinex.nav> ...]"
                 " --output <cn0_model.csv> --metadata <cn0_model.meta.json>"
                 " [--model-semantic absolute|normalized] [--bin-width-deg <deg>] [--min-bin-count <n>]"
                 " [--min-temporal-pairs <n>] [--reference-min-deg <deg>] [--reference-max-deg <deg>]"
                 " [--min-reference-count <n>] [--min-source-count <n>]\n";
}

} // namespace

int main(int argc, char** argv) {
    std::vector<gnss_sim::cn0_builder::Cn0InputSource> sources;
    gnss_sim::cn0_builder::Cn0AggregationConfig config{};
    gnss_sim::cn0_builder::Cn0NormalizationConfig normalization{};
    bool normalized = false;
    std::string output_path;
    std::string metadata_path;

    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--source" && index + 2 < argc) {
            sources.push_back({argv[index + 1], argv[index + 2]});
            index += 2;
        } else if (argument == "--output" && index + 1 < argc) {
            output_path = argv[++index];
        } else if (argument == "--metadata" && index + 1 < argc) {
            metadata_path = argv[++index];
        } else if (argument == "--model-semantic" && index + 1 < argc) {
            const std::string semantic = argv[++index];
            if (semantic == "absolute") {
                normalized = false;
            } else if (semantic == "normalized") {
                normalized = true;
            } else {
                std::cerr << "Invalid --model-semantic value\n";
                return 2;
            }
        } else if (argument == "--bin-width-deg" && index + 1 < argc) {
            if (!parse_double(argv[++index], &config.elevation_bin_width_deg)) {
                std::cerr << "Invalid --bin-width-deg value\n";
                return 2;
            }
        } else if (argument == "--min-bin-count" && index + 1 < argc) {
            if (!parse_u64(argv[++index], &config.min_samples_per_bin)) {
                std::cerr << "Invalid --min-bin-count value\n";
                return 2;
            }
        } else if (argument == "--min-temporal-pairs" && index + 1 < argc) {
            if (!parse_u64(argv[++index], &config.min_temporal_pairs)) {
                std::cerr << "Invalid --min-temporal-pairs value\n";
                return 2;
            }
        } else if (argument == "--reference-min-deg" && index + 1 < argc) {
            if (!parse_double(argv[++index], &normalization.reference_elevation_min_deg)) {
                std::cerr << "Invalid --reference-min-deg value\n";
                return 2;
            }
        } else if (argument == "--reference-max-deg" && index + 1 < argc) {
            if (!parse_double(argv[++index], &normalization.reference_elevation_max_deg)) {
                std::cerr << "Invalid --reference-max-deg value\n";
                return 2;
            }
        } else if (argument == "--min-reference-count" && index + 1 < argc) {
            if (!parse_u64(argv[++index], &normalization.min_reference_samples)) {
                std::cerr << "Invalid --min-reference-count value\n";
                return 2;
            }
        } else if (argument == "--min-source-count" && index + 1 < argc) {
            if (!parse_u64(argv[++index], &normalization.min_sources_per_bin)) {
                std::cerr << "Invalid --min-source-count value\n";
                return 2;
            }
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    if (sources.empty() || output_path.empty() || metadata_path.empty()) {
        usage(argv[0]);
        return 2;
    }

    std::string error;
    if (normalized) {
        gnss_sim::cn0_builder::Cn0NormalizedBuildResult result{};
        if (!gnss_sim::cn0_builder::build_normalized_cn0_model(sources, config, normalization, &result, &error)) {
            std::cerr << "Normalized CN0 model build failed: " << error << '\n';
            return 1;
        }
        if (!gnss_sim::cn0_builder::write_normalized_cn0_model_csv(output_path, result.bins, &error) ||
            !gnss_sim::cn0_builder::write_normalized_cn0_metadata_json(metadata_path, config, normalization, result,
                                                                       &error)) {
            std::cerr << "Normalized CN0 model write failed: " << error << '\n';
            return 1;
        }
        std::cerr << "Normalized CN0 model built: sources=" << result.sources.size() << ", bins=" << result.bins.size()
                  << '\n';
        return 0;
    }

    gnss_sim::cn0_builder::Cn0BuildResult result{};
    if (!gnss_sim::cn0_builder::build_cn0_model(sources, config, &result, &error)) {
        std::cerr << "CN0 model build failed: " << error << '\n';
        return 1;
    }
    if (!gnss_sim::cn0_builder::write_cn0_model_csv(output_path, config, result.aggregation_summary, result.bins,
                                                    &error) ||
        !gnss_sim::cn0_builder::write_cn0_metadata_json(metadata_path, config, result, &error)) {
        std::cerr << "CN0 model write failed: " << error << '\n';
        return 1;
    }

    std::cerr << "CN0 model built: sources=" << result.aggregation_summary.sources
              << ", input_samples=" << result.aggregation_summary.input_samples
              << ", accepted_samples=" << result.aggregation_summary.accepted_samples
              << ", temporal_pairs=" << result.aggregation_summary.temporal_pairs << ", bins=" << result.bins.size()
              << '\n';
    return 0;
}
