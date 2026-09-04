#include "model/carrier_tracking_runtime.h"

#include "gnss_sim/sim_time.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gnss_sim {
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kMinimumUniform = 1.0 / 9007199254740992.0;
constexpr double kNegativeInfinityCn0FloorDbhz = -120.0;
constexpr std::uint64_t kSatelliteSalt = 0x9E3779B97F4A7C15ULL;
constexpr std::uint64_t kSignalSalt = 0xBF58476D1CE4E5B9ULL;
constexpr std::uint64_t kSequenceSalt = 0x94D049BB133111EBULL;
constexpr std::uint64_t kAmbiguitySalt = 0xD6E8FEB86659FD93ULL;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

std::uint64_t mix64(std::uint64_t value) {
    value += 0x9E3779B97F4A7C15ULL;
    value = (value ^ (value >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    value = (value ^ (value >> 27U)) * 0x94D049BB133111EBULL;
    return value ^ (value >> 31U);
}

std::uint64_t signal_key(std::uint64_t simulator_seed, int satellite_number, SignalId signal_id) {
    const std::uint64_t satellite = static_cast<std::uint64_t>(static_cast<std::uint32_t>(satellite_number));
    const std::uint64_t signal = static_cast<std::uint64_t>(signal_id);
    return mix64(simulator_seed ^ mix64(satellite + kSatelliteSalt) ^ mix64(signal + kSignalSalt));
}

double standard_normal_sample(DeterministicRng* rng) {
    const double u1 = std::max(rng_uniform_01(rng), kMinimumUniform);
    const double u2 = rng_uniform_01(rng);
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(kTwoPi * u2);
}

std::int64_t segment_cycle_offset(std::uint64_t ambiguity_key, std::uint64_t segment_id) {
    if (segment_id == 0U) {
        return 0;
    }
    const std::uint64_t mixed = mix64(ambiguity_key ^ mix64(segment_id + kAmbiguitySalt));
    constexpr std::int64_t kSpan = 200001;
    std::int64_t offset = static_cast<std::int64_t>(mixed % static_cast<std::uint64_t>(kSpan)) - 100000;
    if (offset == 0) {
        offset = 1;
    }
    return offset;
}

void fill_runtime_result(const CarrierTrackingRuntimeState& state, const CarrierTrackingResult& tracking,
                         bool cycle_slip_event, CarrierTrackingRuntimeResult* result) {
    result->tracking = tracking;
    result->phase_segment_id = state.phase_segment_id;
    result->adr_cycle_offset_cycles = state.adr_cycle_offset_cycles;
    result->cycle_slip_event = cycle_slip_event;
}

CarrierTrackingResult unlocked_result(const CarrierTrackingRuntimeState& state) {
    CarrierTrackingResult result{};
    result.mode = CarrierTrackingMode::kCarrierUnlocked;
    result.fll_phase = CarrierTrackingFllPhase::kNone;
    result.carrier_segment_id = state.tracking.carrier_segment_id;
    return result;
}

double normalized_cn0(double effective_cn0_dbhz) {
    if (std::isinf(effective_cn0_dbhz) && effective_cn0_dbhz < 0.0) {
        return kNegativeInfinityCn0FloorDbhz;
    }
    return effective_cn0_dbhz;
}

} // namespace

CarrierTrackingConfig carrier_tracking_core_config(const CarrierTrackingReceiverConfig& config) {
    CarrierTrackingConfig output{};
    output.coherent_integration_sec = config.coherent_integration_sec;
    output.pll_noise_bandwidth_hz = config.pll_noise_bandwidth_hz;
    output.fll_noise_bandwidth_hz = config.fll_noise_bandwidth_hz;
    output.fll_pull_in_bandwidth_hz = config.fll_pull_in_bandwidth_hz;
    output.fll_pull_in_duration_sec = config.fll_pull_in_duration_sec;
    output.pll_enter_cn0_dbhz = config.pll_enter_cn0_dbhz;
    output.pll_exit_cn0_dbhz = config.pll_exit_cn0_dbhz;
    output.pll_enter_persistence_sec = config.pll_enter_persistence_sec;
    output.pll_exit_persistence_sec = config.pll_exit_persistence_sec;
    output.fll_enter_cn0_dbhz = config.fll_enter_cn0_dbhz;
    output.fll_exit_cn0_dbhz = config.fll_exit_cn0_dbhz;
    output.fll_enter_persistence_sec = config.fll_enter_persistence_sec;
    output.fll_exit_persistence_sec = config.fll_exit_persistence_sec;
    output.doppler_valid_delay_sec = config.doppler_valid_delay_sec;
    output.adr_valid_after_pll_sec = config.adr_valid_after_pll_sec;
    return output;
}

void initialize_carrier_tracking_runtime_state(std::uint64_t simulator_seed, int satellite_number, SignalId signal_id,
                                               CarrierTrackingRuntimeState* state) {
    if (state == nullptr) {
        return;
    }
    *state = CarrierTrackingRuntimeState{};
    const std::uint64_t key = signal_key(simulator_seed, satellite_number, signal_id);
    seed_rng(&state->rng, key, mix64(key ^ kSequenceSalt));
    state->ambiguity_key = mix64(key ^ kAmbiguitySalt);
    state->initialized = true;
    reset_carrier_tracking_state(&state->tracking);
}

void reset_carrier_tracking_runtime_state(CarrierTrackingRuntimeState* state) {
    if (state == nullptr || !state->initialized) {
        return;
    }
    reset_carrier_tracking_state(&state->tracking);
    state->last_update_time = SimTime{};
    state->phase_segment_id = 0U;
    state->adr_cycle_offset_cycles = 0;
    state->time_initialized = false;
}

bool update_carrier_tracking_runtime(const CarrierTrackingReceiverConfig& config, const SimTime& current_time,
                                     bool code_tracking, double effective_cn0_dbhz, double wavelength_m,
                                     CarrierTrackingRuntimeState* state, CarrierTrackingRuntimeResult* result,
                                     std::string* error_message) {
    if (state == nullptr || result == nullptr || !state->initialized) {
        set_error(error_message, "carrier tracking runtime state/result is invalid");
        return false;
    }
    *result = CarrierTrackingRuntimeResult{};
    const CarrierTrackingConfig core_config = carrier_tracking_core_config(config);
    if (!validate_carrier_tracking_config(core_config, error_message)) {
        return false;
    }
    if (!std::isfinite(wavelength_m) || wavelength_m <= 0.0) {
        set_error(error_message, "carrier tracking runtime wavelength must be finite and positive");
        return false;
    }
    const double cn0_dbhz = normalized_cn0(effective_cn0_dbhz);
    if (!std::isfinite(cn0_dbhz)) {
        set_error(error_message, "carrier tracking runtime effective CN0 is invalid");
        return false;
    }

    if (!code_tracking) {
        reset_carrier_tracking_runtime_state(state);
        fill_runtime_result(*state, unlocked_result(*state), false, result);
        return true;
    }

    if (!state->time_initialized) {
        state->last_update_time = current_time;
        state->time_initialized = true;
        fill_runtime_result(*state, unlocked_result(*state), false, result);
        return true;
    }

    std::int64_t elapsed_ns = 0;
    if (!difference_time_ns(current_time, state->last_update_time, &elapsed_ns) || elapsed_ns <= 0) {
        set_error(error_message, "carrier tracking runtime time must advance monotonically");
        return false;
    }

    double remaining_sec = static_cast<double>(elapsed_ns) / static_cast<double>(NANOSECONDS_PER_SECOND);
    CarrierTrackingResult tracking = unlocked_result(*state);
    bool cycle_slip_event = false;
    while (remaining_sec > 0.0) {
        const double dt_sec = std::min(remaining_sec, core_config.coherent_integration_sec);
        const CarrierTrackingMode previous_mode = state->tracking.mode;
        CarrierTrackingInput input{};
        input.signal_available = true;
        input.effective_cn0_dbhz = cn0_dbhz;
        input.wavelength_m = wavelength_m;
        input.dt_sec = dt_sec;
        input.standard_normal_sample = standard_normal_sample(&state->rng);
        if (!update_carrier_tracking(core_config, input, &state->tracking, &tracking, error_message)) {
            return false;
        }
        if (previous_mode == CarrierTrackingMode::kPllTrack && tracking.mode != CarrierTrackingMode::kPllTrack) {
            ++state->phase_segment_id;
            state->adr_cycle_offset_cycles = segment_cycle_offset(state->ambiguity_key, state->phase_segment_id);
            cycle_slip_event = true;
        }
        remaining_sec -= dt_sec;
        if (remaining_sec < std::numeric_limits<double>::epsilon()) {
            remaining_sec = 0.0;
        }
    }

    state->last_update_time = current_time;
    fill_runtime_result(*state, tracking, cycle_slip_event, result);
    return true;
}

bool apply_carrier_tracking_runtime_result(const CarrierTrackingRuntimeResult& carrier,
                                           MeasurementObservation* observation, std::string* error_message) {
    if (observation == nullptr || !std::isfinite(observation->wavelength_m) || observation->wavelength_m <= 0.0 ||
        !std::isfinite(observation->range_rate_mps) || !std::isfinite(observation->doppler_hz) ||
        !std::isfinite(observation->adr_cycles) || !std::isfinite(carrier.tracking.tracking_error_hz) ||
        !std::isfinite(carrier.tracking.tracking_error_mps)) {
        set_error(error_message, "carrier tracking measurement application has invalid arguments");
        return false;
    }

    MeasurementObservation output = *observation;
    if (carrier.tracking.mode != CarrierTrackingMode::kCarrierUnlocked) {
        output.doppler_hz += carrier.tracking.tracking_error_hz;
        output.range_rate_mps -= carrier.tracking.tracking_error_mps;
    }

    output.doppler_valid = output.doppler_valid && carrier.tracking.doppler_valid;
    output.adr_valid = output.adr_valid && carrier.tracking.adr_valid;
    if (output.adr_valid) {
        if ((carrier.adr_cycle_offset_cycles > 0 &&
             output.ambiguity_cycles > std::numeric_limits<std::int64_t>::max() - carrier.adr_cycle_offset_cycles) ||
            (carrier.adr_cycle_offset_cycles < 0 &&
             output.ambiguity_cycles < std::numeric_limits<std::int64_t>::min() - carrier.adr_cycle_offset_cycles)) {
            set_error(error_message, "carrier tracking ambiguity offset overflow");
            return false;
        }
        output.ambiguity_cycles += carrier.adr_cycle_offset_cycles;
        output.adr_cycles += static_cast<double>(carrier.adr_cycle_offset_cycles);
    }

    if (!output.observation_available) {
        output.pseudorange_valid = false;
        output.doppler_valid = false;
        output.adr_valid = false;
    }

    *observation = output;
    return true;
}

} // namespace gnss_sim
