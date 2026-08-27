#ifndef GNSS_SIM_SRC_OUTPUT_TRUTH_WRITER_H_
#define GNSS_SIM_SRC_OUTPUT_TRUTH_WRITER_H_

#include "gnss/satellite_engine.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_types.h"
#include "model/measurement_model.h"
#include "model/receiver_truth.h"
#include "model/signal_tracking.h"
#include "scenario/scenario_engine.h"
#include "solution/solution_engine.h"

#include <string>

namespace gnss_sim {

struct SimulatorRunSummary;
struct TruthWriter;

constexpr int TRUTH_OUTPUT_SCHEMA_VERSION = 1;

TruthWriter* create_truth_writer(const char* receiver_log_path, const char* rinex_nav_path, const SimConfig& config,
                                 const SimTime& start_time, std::string* error_message);
void destroy_truth_writer(TruthWriter* writer);

bool truth_writer_write_scenario_events(TruthWriter* writer, const ScenarioEpochState& scenario,
                                        StartupMode startup_mode, std::string* error_message);
bool truth_writer_write_observation(TruthWriter* writer, const ReceiverTruth& receiver,
                                    const SatelliteGeometry& geometry, const SignalTracker& tracker,
                                    const MeasurementObservation& observation, std::string* error_message);
bool truth_writer_write_solution(TruthWriter* writer, const SolutionEpoch& solution, int tracked_satellites,
                                 std::string* error_message);

bool finalize_truth_writer(TruthWriter* writer, const SimulatorRunSummary& summary, const char* simulator_version,
                           const char* simulator_commit_sha, const char* rtklib_commit_sha, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_OUTPUT_TRUTH_WRITER_H_
