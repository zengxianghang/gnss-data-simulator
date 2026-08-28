#ifndef GNSS_SIM_TOOLS_RESIDUAL_VALIDATOR_RESIDUAL_VALIDATOR_H_
#define GNSS_SIM_TOOLS_RESIDUAL_VALIDATOR_RESIDUAL_VALIDATOR_H_

#include <cstdint>
#include <string>
#include <vector>

namespace gnss_sim::residual_validator {

enum class AtmosphereMode {
    kNone,
    kBroadcast,
};

struct ValidationOptions {
    std::string nav_path;
    std::string observation_truth_path;
    AtmosphereMode atmosphere_mode = AtmosphereMode::kBroadcast;
    bool allow_diagnostic_health = true;
};

struct SummaryRow {
    std::string scope;
    int signal_id = -1;
    std::string signal_name;
    std::string rinex_code;
    int oem7_signal_type = -1;
    std::string family;
    int satellite_number = 0;
    std::uint64_t rows = 0;
    std::uint64_t code_residuals = 0;
    std::uint64_t code_unavailable = 0;
    std::uint64_t diagnostic_code_rows = 0;
    std::uint64_t doppler_residuals = 0;
    std::uint64_t diagnostic_doppler_rows = 0;
    double code_rms_m = 0.0;
    double code_p95_abs_m = 0.0;
    double code_max_abs_m = 0.0;
    double doppler_rms_mps = 0.0;
    double doppler_p95_abs_mps = 0.0;
    double doppler_max_abs_mps = 0.0;
};

struct ValidationReport {
    std::uint64_t input_rows = 0;
    std::vector<SummaryRow> rows;
};

bool validate_observation_truth(const ValidationOptions& options, ValidationReport* report,
                                std::string* error_message);
bool write_summary_csv(const ValidationReport& report, const std::string& output_path, std::string* error_message);

const SummaryRow* find_signal_summary(const ValidationReport& report, int signal_id);

} // namespace gnss_sim::residual_validator

#endif // GNSS_SIM_TOOLS_RESIDUAL_VALIDATOR_RESIDUAL_VALIDATOR_H_
