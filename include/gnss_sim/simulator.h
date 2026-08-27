#ifndef GNSS_SIM_SIMULATOR_H_
#define GNSS_SIM_SIMULATOR_H_

#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_types.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace gnss_sim {

enum class SimulationLogKind {
    kRange,
    kPosition,
    kVelocity,
    kNavigation,
};

using SimulationLineCallback = bool (*)(SimulationLogKind kind, const SimTime& time, const char* data,
                                        std::size_t size, void* user_data);

struct SimulationOutputSink {
    SimulationLineCallback write_line;
    void* user_data;
};

struct SimulationRequest {
    SimConfig config;
    SimTime start_time;
    const char* rinex_nav_path;
};

struct SimulationRunStats {
    std::uint64_t total_epochs;
    std::uint64_t powered_epochs;
    std::uint64_t range_log_count;
    std::uint64_t position_log_count;
    std::uint64_t velocity_log_count;
    std::uint64_t navigation_log_count;
    std::uint64_t power_on_event_count;
    std::uint64_t power_off_event_count;
    std::uint64_t signal_on_event_count;
    std::uint64_t signal_off_event_count;
    std::uint64_t startup_event_count;
    std::uint64_t navigation_update_count;
    std::uint64_t maximum_observations_in_epoch;
    std::uint64_t runtime_signal_count;
};

bool run_simulation(const SimulationRequest& request, const SimulationOutputSink& sink, SimulationRunStats* stats,
                    std::string* error_message);

const char* simulation_log_kind_name(SimulationLogKind kind);
const char* simulator_version();
const char* rtklib_commit_sha();

} // namespace gnss_sim

#endif // GNSS_SIM_SIMULATOR_H_
