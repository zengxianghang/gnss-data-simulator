#ifndef GNSS_SIM_SRC_SOLUTION_SOLUTION_ENGINE_H_
#define GNSS_SIM_SRC_SOLUTION_SOLUTION_ENGINE_H_

#include "gnss/rtklib_adapter.h"
#include "gnss_sim/atmosphere_types.h"
#include "gnss_sim/sim_types.h"
#include "model/measurement_model.h"

#include <string>

namespace gnss_sim {

enum class ReceiverSolutionStatus {
    kInsufficientObs,
    kSolComputed,
};

enum class ReceiverSolutionType {
    kNone,
    kSingle,
};

struct PositionSolution {
    bool valid;
    ReceiverSolutionStatus status;
    ReceiverSolutionType type;
    double position_ecef_m[3];
    double latitude_deg;
    double longitude_deg;
    double height_m;
    double receiver_clock_bias_m;
    double covariance_ecef_m2[6];
    int used_satellites;
    char diagnostic[128];
};

struct VelocitySolution {
    bool valid;
    ReceiverSolutionStatus status;
    ReceiverSolutionType type;
    double velocity_ecef_mps[3];
    double receiver_clock_drift_mps;
    int used_satellites;
    char diagnostic[128];
};

struct SolutionEpoch {
    SimTime time;
    PositionSolution position;
    VelocitySolution velocity;
};

struct SolutionEngineState {
    bool has_position_hint;
    double last_position_ecef_m[3];
};

void reset_solution_engine_state(SolutionEngineState* state);

bool solve_receiver_epoch(const RtklibNavStore* receiver_nav, const SimTime& epoch_time,
                          const MeasurementObservation* observations, int observation_count, double elevation_mask_deg,
                          AtmosphereMode atmosphere_mode, SolutionEngineState* state, SolutionEpoch* solution,
                          std::string* error_message);

const char* receiver_solution_status_name(ReceiverSolutionStatus status);
const char* receiver_solution_type_name(ReceiverSolutionType type);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_SOLUTION_SOLUTION_ENGINE_H_
