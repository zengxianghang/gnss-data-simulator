#ifndef GNSS_SIM_TOOLS_TRANSIENT_VALIDATOR_TRANSIENT_VALIDATOR_H_
#define GNSS_SIM_TOOLS_TRANSIENT_VALIDATOR_TRANSIENT_VALIDATOR_H_

#include "rangea_roundtrip.h"

#include <cstdint>
#include <string>

namespace gnss_sim {

struct ErrorMetricStatistics {
    std::uint64_t sample_count;
    double mean;
    double rms;
    double standard_deviation;
    double p50_absolute;
    double p95_absolute;
    double max_absolute;
};

struct ObservationWindowStatistics {
    ErrorMetricStatistics pseudorange_m;
    ErrorMetricStatistics doppler_mps;
    ErrorMetricStatistics adr_m;
    ErrorMetricStatistics cn0_dbhz;
};

struct FirstValidTimingStatistics {
    double pseudorange_delay_sec;
    double doppler_delay_sec;
    double adr_delay_sec;
    double cn0_delay_sec;
};

struct ReaTransientStatistics {
    std::uint64_t signal_off_range_epochs;
    std::uint64_t signal_off_nonzero_epochs;
    std::uint64_t reacquisition_cycles;
    double max_first_psr_delay_sec;
    double max_first_doppler_delay_sec;
    double max_first_adr_delay_sec;
    double max_last_observation_to_signal_off_sec;
    double max_first_observation_after_signal_on_sec;
    std::uint64_t ambiguity_pairs_checked;
    std::uint64_t ambiguity_pairs_changed;
};

struct TransientValidationOptions {
    const char* scenario_label;
    double fade_duration_sec;
    double truth_latitude_deg;
    double truth_longitude_deg;
    double truth_height_m;
    double solution_elevation_mask_deg;
    bool broadcast_atmosphere;
};

struct TransientValidationSummary {
    std::uint64_t range_epochs;
    std::uint64_t parsed_observations;
    std::uint64_t matched_observations;
    std::uint64_t unmatched_observations;
    FirstValidTimingStatistics first_valid;
    ObservationWindowStatistics early;
    ObservationWindowStatistics recovery;
    ObservationWindowStatistics settled;
    ObservationWindowStatistics fade;
    ObservationWindowStatistics reacquisition_early;
    ObservationWindowStatistics reacquisition_recovery;
    ObservationWindowStatistics reacquisition_settled;
    ReaTransientStatistics rea;
    RangeaRoundtripSummary positioning;
};

bool validate_transient_observations_files(const char* log_path, const char* observation_truth_path,
                                           const char* event_truth_path, const char* rinex_nav_path,
                                           const TransientValidationOptions& options,
                                           TransientValidationSummary* summary, std::string* error_message);

bool write_transient_validation_json(const char* output_path, const TransientValidationOptions& options,
                                     const TransientValidationSummary& summary, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_TOOLS_TRANSIENT_VALIDATOR_TRANSIENT_VALIDATOR_H_
