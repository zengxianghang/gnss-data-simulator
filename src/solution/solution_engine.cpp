#include "solution/solution_engine.h"

#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_time.h"

#include <cmath>

namespace gnss_sim {
namespace {

constexpr int kMaxSolverSatellites = 64;

struct SelectedObservation {
    int satellite_number;
    int priority;
    RtklibSolutionObservation observation;
};

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_epoch_time(const SimTime& time) {
    return time.gps_week >= 0 && time.tow_ns >= 0 && time.tow_ns < GPS_WEEK_NANOSECONDS;
}

bool make_rtklib_observation(const MeasurementObservation& source, RtklibSolutionObservation* destination) {
    const SignalDefinition* definition = find_signal_definition(source.signal_id);
    if (definition == nullptr || destination == nullptr || source.satellite_number <= 0 ||
        !std::isfinite(source.pseudorange_m) || source.pseudorange_m <= 0.0 || !std::isfinite(source.doppler_hz) ||
        !std::isfinite(source.wavelength_m) || source.wavelength_m <= 0.0 || !std::isfinite(source.code_bias_m) ||
        !std::isfinite(source.cn0_dbhz)) {
        return false;
    }

    int observation_code = 0;
    int frequency_index = 0;
    if (!signal_rtklib_observation_code(*definition, &observation_code, &frequency_index)) {
        return false;
    }
    static_cast<void>(frequency_index);

    RtklibSolutionObservation result{};
    result.satellite_number = source.satellite_number;
    result.observation_code = observation_code;
    result.pseudorange_m = source.pseudorange_m;
    result.code_bias_m = source.code_bias_m;
    result.doppler_hz = source.doppler_hz;
    result.wavelength_m = source.wavelength_m;
    result.cn0_dbhz = source.cn0_dbhz;
    result.pseudorange_valid = source.pseudorange_valid &&
                               source.code_bias_status != BroadcastCodeBiasStatus::kUnavailableForMessageFamily;
    result.doppler_valid = source.doppler_valid;
    *destination = result;
    return true;
}

int find_selected_satellite(const SelectedObservation* selected, int count, int satellite_number) {
    for (int index = 0; index < count; ++index) {
        if (selected[index].satellite_number == satellite_number) {
            return index;
        }
    }
    return -1;
}

void consider_selected_observation(const MeasurementObservation& measurement, bool for_position,
                                   SelectedObservation selected[kMaxSolverSatellites], int* count) {
    const int priority = signal_single_point_priority(measurement.signal_id);
    if (priority < 0) {
        return;
    }

    RtklibSolutionObservation candidate{};
    if (!make_rtklib_observation(measurement, &candidate)) {
        return;
    }
    if ((for_position && !candidate.pseudorange_valid) || (!for_position && !candidate.doppler_valid)) {
        return;
    }

    const int existing = find_selected_satellite(selected, *count, candidate.satellite_number);
    if (existing >= 0) {
        if (priority < selected[existing].priority) {
            selected[existing].priority = priority;
            selected[existing].observation = candidate;
        }
        return;
    }
    if (*count >= kMaxSolverSatellites) {
        return;
    }

    selected[*count].satellite_number = candidate.satellite_number;
    selected[*count].priority = priority;
    selected[*count].observation = candidate;
    ++(*count);
}

void sort_selected_observations(SelectedObservation selected[kMaxSolverSatellites], int count) {
    for (int index = 1; index < count; ++index) {
        const SelectedObservation value = selected[index];
        int insert = index;
        while (insert > 0 && selected[insert - 1].satellite_number > value.satellite_number) {
            selected[insert] = selected[insert - 1];
            --insert;
        }
        selected[insert] = value;
    }
}

void copy_position_solution(const RtklibPositionSolution& source, PositionSolution* destination) {
    destination->valid = true;
    destination->status = ReceiverSolutionStatus::kSolComputed;
    destination->type = ReceiverSolutionType::kSingle;
    for (int index = 0; index < 3; ++index) {
        destination->position_ecef_m[index] = source.position_ecef_m[index];
    }
    destination->latitude_deg = source.latitude_deg;
    destination->longitude_deg = source.longitude_deg;
    destination->height_m = source.height_m;
    destination->receiver_clock_bias_m = source.receiver_clock_bias_m;
    for (int index = 0; index < 6; ++index) {
        destination->covariance_ecef_m2[index] = source.covariance_ecef_m2[index];
    }
    destination->used_satellites = source.used_satellites;
}

void copy_velocity_solution(const RtklibVelocitySolution& source, VelocitySolution* destination) {
    destination->valid = true;
    destination->status = ReceiverSolutionStatus::kSolComputed;
    destination->type = ReceiverSolutionType::kSingle;
    for (int index = 0; index < 3; ++index) {
        destination->velocity_ecef_mps[index] = source.velocity_ecef_mps[index];
    }
    destination->receiver_clock_drift_mps = source.receiver_clock_drift_mps;
    destination->used_satellites = source.used_satellites;
}

} // namespace

void reset_solution_engine_state(SolutionEngineState* state) {
    if (state != nullptr) {
        *state = {};
    }
}

bool solve_receiver_epoch(const RtklibNavStore* receiver_nav, const SimTime& epoch_time,
                          const MeasurementObservation* observations, int observation_count,
                          double elevation_mask_deg, AtmosphereMode atmosphere_mode, SolutionEngineState* state,
                          SolutionEpoch* solution, std::string* error_message) {
    if (receiver_nav == nullptr || observation_count < 0 || (observation_count > 0 && observations == nullptr) ||
        state == nullptr || solution == nullptr || !valid_epoch_time(epoch_time) ||
        !std::isfinite(elevation_mask_deg) || elevation_mask_deg < -90.0 || elevation_mask_deg > 90.0 ||
        atmosphere_mode == AtmosphereMode::UNSPECIFIED) {
        set_error(error_message, "receiver-solution request has invalid arguments");
        return false;
    }

    SolutionEpoch result{};
    result.time = epoch_time;
    result.position.status = ReceiverSolutionStatus::kInsufficientObs;
    result.position.type = ReceiverSolutionType::kNone;
    result.velocity.status = ReceiverSolutionStatus::kInsufficientObs;
    result.velocity.type = ReceiverSolutionType::kNone;

    SelectedObservation position_selected[kMaxSolverSatellites]{};
    SelectedObservation velocity_selected[kMaxSolverSatellites]{};
    int position_count = 0;
    int velocity_count = 0;
    for (int index = 0; index < observation_count; ++index) {
        consider_selected_observation(observations[index], true, position_selected, &position_count);
        consider_selected_observation(observations[index], false, velocity_selected, &velocity_count);
    }
    sort_selected_observations(position_selected, position_count);
    sort_selected_observations(velocity_selected, velocity_count);

    RtklibSolutionObservation position_observations[kMaxSolverSatellites]{};
    RtklibSolutionObservation velocity_observations[kMaxSolverSatellites]{};
    for (int index = 0; index < position_count; ++index) {
        position_observations[index] = position_selected[index].observation;
    }
    for (int index = 0; index < velocity_count; ++index) {
        velocity_observations[index] = velocity_selected[index].observation;
    }

    const double sow_sec = sim_time_sow_sec(epoch_time);
    RtklibPositionSolution rtklib_position{};
    if (!rtklib_solve_single_position(receiver_nav, epoch_time.gps_week, sow_sec, position_observations,
                                      position_count, elevation_mask_deg, atmosphere_mode == AtmosphereMode::BROADCAST,
                                      &rtklib_position, error_message)) {
        return false;
    }
    if (rtklib_position.valid) {
        copy_position_solution(rtklib_position, &result.position);
        state->has_position_hint = true;
        for (int index = 0; index < 3; ++index) {
            state->last_position_ecef_m[index] = rtklib_position.position_ecef_m[index];
        }
    }

    const double* position_hint = nullptr;
    if (result.position.valid) {
        position_hint = result.position.position_ecef_m;
    } else if (state->has_position_hint) {
        position_hint = state->last_position_ecef_m;
    }

    if (position_hint != nullptr) {
        RtklibVelocitySolution rtklib_velocity{};
        if (!rtklib_solve_single_velocity(receiver_nav, epoch_time.gps_week, sow_sec, velocity_observations,
                                          velocity_count, position_hint, elevation_mask_deg, &rtklib_velocity,
                                          error_message)) {
            return false;
        }
        if (rtklib_velocity.valid) {
            copy_velocity_solution(rtklib_velocity, &result.velocity);
        }
    }

    *solution = result;
    return true;
}

const char* receiver_solution_status_name(ReceiverSolutionStatus status) {
    switch (status) {
        case ReceiverSolutionStatus::kInsufficientObs:
            return "INSUFFICIENT_OBS";
        case ReceiverSolutionStatus::kSolComputed:
            return "SOL_COMPUTED";
    }
    return "UNKNOWN";
}

const char* receiver_solution_type_name(ReceiverSolutionType type) {
    switch (type) {
        case ReceiverSolutionType::kNone:
            return "NONE";
        case ReceiverSolutionType::kSingle:
            return "SINGLE";
    }
    return "UNKNOWN";
}

} // namespace gnss_sim
