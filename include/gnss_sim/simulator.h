#ifndef GNSS_SIM_SIMULATOR_H_
#define GNSS_SIM_SIMULATOR_H_

#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_types.h"

#include <cstdint>
#include <string>

namespace gnss_sim {

struct SimulatorRunOptions {
    const char* rinex_nav_path;
    const char* output_log_path;
    SimTime start_time;
    const char* cn0_model_path;
    const char* galileo_has_sp3_path;
    const char* galileo_has_clock_path;
    const char* galileo_has_bias_path;
};

struct SimulatorRunSummary {
    std::uint64_t scheduled_epochs;
    std::uint64_t powered_epochs;
    std::uint64_t signal_on_epochs;
    std::uint64_t signal_off_epochs;
    std::uint64_t range_messages;
    std::uint64_t psrpos_messages;
    std::uint64_t psrvel_messages;
    std::uint64_t nav_messages;
    std::uint64_t power_on_events;
    std::uint64_t power_off_events;
    std::uint64_t signal_on_events;
    std::uint64_t signal_off_events;
    std::uint64_t valid_position_epochs;
    std::uint64_t valid_velocity_epochs;
    int max_observations_per_epoch;
    std::string cn0_model_source;
    std::string cn0_model_schema_version;
    std::string cn0_model_name;
    std::string cn0_model_hash;
    std::uint64_t cn0_model_size_bytes;
};

bool run_simulator(const SimConfig& config, const SimulatorRunOptions& options, SimulatorRunSummary* summary,
                   std::string* error_message);

const char* simulator_version();
const char* simulator_commit_sha();
const char* rtklib_commit_sha();

} // namespace gnss_sim

#endif // GNSS_SIM_SIMULATOR_H_
