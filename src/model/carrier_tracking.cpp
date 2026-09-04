#include "model/carrier_tracking.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gnss_sim {
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kElapsedToleranceSec = 1e-12;
constexpr double kMinimumCn0LinearHz = 1e-12;
constexpr double kMaximumCn0LinearHz = 1e12;
constexpr double kFllNoiseFactor = 1.0;

bool fail(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
    return false;
}

bool positive_finite(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool finite_value(double value) {
    return std::isfinite(value);
}

bool elapsed_reached(double elapsed_sec, double threshold_sec) {
    return elapsed_sec + kElapsedToleranceSec >= threshold_sec;
}

double cn0_dbhz_to_linear(double cn0_dbhz) {
    const double exponent = cn0_dbhz / 10.0;
    double value = std::pow(10.0, exponent);
    if (!std::isfinite(value)) {
        value = exponent > 0.0 ? kMaximumCn0LinearHz : kMinimumCn0LinearHz;
    }
    return std::clamp(value, kMinimumCn0LinearHz, kMaximumCn0LinearHz);
}

void clear_persistence(CarrierTrackingState* state) {
    state->fll_enter_persistence_sec = 0.0;
    state->fll_exit_persistence_sec = 0.0;
    state->pll_enter_persistence_sec = 0.0;
    state->pll_exit_persistence_sec = 0.0;
}

void enter_unlocked(CarrierTrackingState* state) {
    state->mode = CarrierTrackingMode::kCarrierUnlocked;
    state->fll_pull_in_active = false;
    state->mode_age_sec = 0.0;
    state->carrier_lock_age_sec = 0.0;
    state->pll_age_sec = 0.0;
    state->tracking_error_hz = 0.0;
    clear_persistence(state);
}

void enter_fll_from_unlocked(CarrierTrackingState* state) {
    state->mode = CarrierTrackingMode::kFllTrack;
    state->fll_pull_in_active = true;
    state->mode_age_sec = 0.0;
    state->carrier_lock_age_sec = 0.0;
    state->pll_age_sec = 0.0;
    state->tracking_error_hz = 0.0;
    clear_persistence(state);
    ++state->carrier_segment_id;
}

void enter_fll_from_pll(CarrierTrackingState* state) {
    state->mode = CarrierTrackingMode::kFllTrack;
    state->fll_pull_in_active = false;
    state->mode_age_sec = 0.0;
    state->pll_age_sec = 0.0;
    state->fll_enter_persistence_sec = 0.0;
    state->fll_exit_persistence_sec = 0.0;
    state->pll_enter_persistence_sec = 0.0;
    state->pll_exit_persistence_sec = 0.0;
}

void enter_pll(CarrierTrackingState* state) {
    state->mode = CarrierTrackingMode::kPllTrack;
    state->fll_pull_in_active = false;
    state->mode_age_sec = 0.0;
    state->pll_age_sec = 0.0;
    state->fll_enter_persistence_sec = 0.0;
    state->fll_exit_persistence_sec = 0.0;
    state->pll_enter_persistence_sec = 0.0;
    state->pll_exit_persistence_sec = 0.0;
}

CarrierTrackingFllPhase fll_phase_from_state(const CarrierTrackingState& state) {
    if (state.mode != CarrierTrackingMode::kFllTrack) {
        return CarrierTrackingFllPhase::kNone;
    }
    return state.fll_pull_in_active ? CarrierTrackingFllPhase::kPullIn : CarrierTrackingFllPhase::kSteady;
}

void fill_unlocked_result(const CarrierTrackingState& state, bool mode_changed, CarrierTrackingResult* result) {
    result->mode = state.mode;
    result->fll_phase = CarrierTrackingFllPhase::kNone;
    result->jitter = CarrierTrackingJitter{};
    result->tracking_error_hz = 0.0;
    result->tracking_error_mps = 0.0;
    result->carrier_lock_age_sec = 0.0;
    result->pll_age_sec = 0.0;
    result->carrier_segment_id = state.carrier_segment_id;
    result->doppler_valid = false;
    result->adr_valid = false;
    result->mode_changed = mode_changed;
    result->new_carrier_segment = false;
}

} // namespace

CarrierTrackingConfig default_carrier_tracking_config() {
    CarrierTrackingConfig config{};
    config.coherent_integration_sec = 0.020;
    config.pll_noise_bandwidth_hz = 5.0;
    config.fll_noise_bandwidth_hz = 4.0;
    config.fll_pull_in_bandwidth_hz = 8.0;
    config.fll_pull_in_duration_sec = 0.5;

    config.pll_enter_cn0_dbhz = 30.0;
    config.pll_exit_cn0_dbhz = 27.0;
    config.pll_enter_persistence_sec = 1.0;
    config.pll_exit_persistence_sec = 0.3;

    config.fll_enter_cn0_dbhz = 22.0;
    config.fll_exit_cn0_dbhz = 18.0;
    config.fll_enter_persistence_sec = 0.2;
    config.fll_exit_persistence_sec = 0.5;

    config.doppler_valid_delay_sec = 0.2;
    config.adr_valid_after_pll_sec = 1.0;
    return config;
}

bool validate_carrier_tracking_config(const CarrierTrackingConfig& config, std::string* error_message) {
    if (!positive_finite(config.coherent_integration_sec)) {
        return fail(error_message, "carrier coherent integration time must be finite and positive");
    }
    if (!positive_finite(config.pll_noise_bandwidth_hz) || !positive_finite(config.fll_noise_bandwidth_hz) ||
        !positive_finite(config.fll_pull_in_bandwidth_hz)) {
        return fail(error_message, "carrier loop bandwidths must be finite and positive");
    }
    if (config.fll_pull_in_bandwidth_hz < config.fll_noise_bandwidth_hz) {
        return fail(error_message, "FLL pull-in bandwidth must be at least the steady FLL bandwidth");
    }
    if (!positive_finite(config.fll_pull_in_duration_sec) || !positive_finite(config.pll_enter_persistence_sec) ||
        !positive_finite(config.pll_exit_persistence_sec) || !positive_finite(config.fll_enter_persistence_sec) ||
        !positive_finite(config.fll_exit_persistence_sec) || !positive_finite(config.doppler_valid_delay_sec) ||
        !positive_finite(config.adr_valid_after_pll_sec)) {
        return fail(error_message, "carrier tracking durations must be finite and positive");
    }
    if (!finite_value(config.pll_enter_cn0_dbhz) || !finite_value(config.pll_exit_cn0_dbhz) ||
        !finite_value(config.fll_enter_cn0_dbhz) || !finite_value(config.fll_exit_cn0_dbhz)) {
        return fail(error_message, "carrier tracking CN0 thresholds must be finite");
    }
    if (!(config.pll_enter_cn0_dbhz > config.pll_exit_cn0_dbhz &&
          config.pll_exit_cn0_dbhz > config.fll_enter_cn0_dbhz &&
          config.fll_enter_cn0_dbhz > config.fll_exit_cn0_dbhz)) {
        return fail(error_message,
                    "carrier tracking CN0 thresholds must satisfy PLL-enter > PLL-exit > FLL-enter > FLL-exit");
    }
    return true;
}

void reset_carrier_tracking_state(CarrierTrackingState* state) {
    if (state == nullptr) {
        return;
    }
    *state = CarrierTrackingState{};
    state->mode = CarrierTrackingMode::kCarrierUnlocked;
}

bool compute_carrier_tracking_jitter(const CarrierTrackingConfig& config, CarrierTrackingMode mode,
                                     bool fll_pull_in_active, double effective_cn0_dbhz, double wavelength_m,
                                     double dt_sec, CarrierTrackingJitter* jitter, std::string* error_message) {
    if (jitter == nullptr) {
        return fail(error_message, "carrier tracking jitter output is null");
    }
    *jitter = CarrierTrackingJitter{};
    if (!validate_carrier_tracking_config(config, error_message)) {
        return false;
    }
    if (!finite_value(effective_cn0_dbhz)) {
        return fail(error_message, "effective CN0 must be finite");
    }
    if (!positive_finite(wavelength_m)) {
        return fail(error_message, "carrier wavelength must be finite and positive");
    }
    if (!positive_finite(dt_sec)) {
        return fail(error_message, "carrier tracking dt must be finite and positive");
    }
    if (mode == CarrierTrackingMode::kCarrierUnlocked) {
        return true;
    }

    const double cn0_linear_hz = cn0_dbhz_to_linear(effective_cn0_dbhz);
    const double integration_sec = config.coherent_integration_sec;
    double bandwidth_hz = config.pll_noise_bandwidth_hz;
    double phase_sigma_rad = 0.0;
    double sigma_hz = 0.0;

    if (mode == CarrierTrackingMode::kFllTrack) {
        bandwidth_hz = fll_pull_in_active ? config.fll_pull_in_bandwidth_hz : config.fll_noise_bandwidth_hz;
        const double correction = 1.0 + 1.0 / (integration_sec * cn0_linear_hz);
        const double variance_term = 4.0 * kFllNoiseFactor * bandwidth_hz / cn0_linear_hz * correction;
        sigma_hz = std::sqrt(variance_term) / (kTwoPi * integration_sec);
    } else if (mode == CarrierTrackingMode::kPllTrack) {
        bandwidth_hz = config.pll_noise_bandwidth_hz;
        const double correction = 1.0 + 1.0 / (2.0 * integration_sec * cn0_linear_hz);
        const double phase_variance_rad2 = bandwidth_hz / cn0_linear_hz * correction;
        phase_sigma_rad = std::sqrt(phase_variance_rad2);
        sigma_hz = phase_sigma_rad / (kTwoPi * integration_sec);
    } else {
        return fail(error_message, "unsupported carrier tracking mode");
    }

    const double tau_sec = 1.0 / (kTwoPi * bandwidth_hz);
    const double alpha = std::exp(-dt_sec / tau_sec);
    const double sigma_mps = wavelength_m * sigma_hz;
    if (!std::isfinite(phase_sigma_rad) || !std::isfinite(sigma_hz) || !std::isfinite(sigma_mps) ||
        !std::isfinite(tau_sec) || !std::isfinite(alpha)) {
        return fail(error_message, "carrier tracking jitter calculation produced a non-finite value");
    }

    jitter->cn0_linear_hz = cn0_linear_hz;
    jitter->active_bandwidth_hz = bandwidth_hz;
    jitter->phase_sigma_rad = phase_sigma_rad;
    jitter->sigma_hz = sigma_hz;
    jitter->sigma_mps = sigma_mps;
    jitter->correlation_tau_sec = tau_sec;
    jitter->correlation_alpha = alpha;
    return true;
}

bool update_carrier_tracking(const CarrierTrackingConfig& config, const CarrierTrackingInput& input,
                             CarrierTrackingState* state, CarrierTrackingResult* result, std::string* error_message) {
    if (state == nullptr || result == nullptr) {
        return fail(error_message, "carrier tracking state/result is null");
    }
    *result = CarrierTrackingResult{};
    if (!validate_carrier_tracking_config(config, error_message)) {
        return false;
    }
    if (!finite_value(input.effective_cn0_dbhz)) {
        return fail(error_message, "effective CN0 must be finite");
    }
    if (!positive_finite(input.wavelength_m)) {
        return fail(error_message, "carrier wavelength must be finite and positive");
    }
    if (!positive_finite(input.dt_sec)) {
        return fail(error_message, "carrier tracking dt must be finite and positive");
    }
    if (!finite_value(input.standard_normal_sample)) {
        return fail(error_message, "carrier tracking Gaussian input must be finite");
    }

    const CarrierTrackingMode previous_mode = state->mode;
    bool new_carrier_segment = false;

    if (!input.signal_available) {
        enter_unlocked(state);
        fill_unlocked_result(*state, previous_mode != state->mode, result);
        return true;
    }

    switch (state->mode) {
        case CarrierTrackingMode::kCarrierUnlocked:
            if (input.effective_cn0_dbhz >= config.fll_enter_cn0_dbhz) {
                state->fll_enter_persistence_sec += input.dt_sec;
            } else {
                state->fll_enter_persistence_sec = 0.0;
            }
            if (elapsed_reached(state->fll_enter_persistence_sec, config.fll_enter_persistence_sec)) {
                enter_fll_from_unlocked(state);
                new_carrier_segment = true;
            }
            break;

        case CarrierTrackingMode::kFllTrack:
            state->mode_age_sec += input.dt_sec;
            state->carrier_lock_age_sec += input.dt_sec;
            if (state->fll_pull_in_active && elapsed_reached(state->mode_age_sec, config.fll_pull_in_duration_sec)) {
                state->fll_pull_in_active = false;
            }

            if (input.effective_cn0_dbhz < config.fll_exit_cn0_dbhz) {
                state->fll_exit_persistence_sec += input.dt_sec;
            } else {
                state->fll_exit_persistence_sec = 0.0;
            }
            if (elapsed_reached(state->fll_exit_persistence_sec, config.fll_exit_persistence_sec)) {
                enter_unlocked(state);
                break;
            }

            if (input.effective_cn0_dbhz >= config.pll_enter_cn0_dbhz) {
                state->pll_enter_persistence_sec += input.dt_sec;
            } else {
                state->pll_enter_persistence_sec = 0.0;
            }
            if (elapsed_reached(state->pll_enter_persistence_sec, config.pll_enter_persistence_sec)) {
                enter_pll(state);
            }
            break;

        case CarrierTrackingMode::kPllTrack:
            state->mode_age_sec += input.dt_sec;
            state->carrier_lock_age_sec += input.dt_sec;
            state->pll_age_sec += input.dt_sec;
            if (input.effective_cn0_dbhz < config.pll_exit_cn0_dbhz) {
                state->pll_exit_persistence_sec += input.dt_sec;
            } else {
                state->pll_exit_persistence_sec = 0.0;
            }
            if (elapsed_reached(state->pll_exit_persistence_sec, config.pll_exit_persistence_sec)) {
                enter_fll_from_pll(state);
            }
            break;
    }

    if (state->mode == CarrierTrackingMode::kCarrierUnlocked) {
        fill_unlocked_result(*state, previous_mode != state->mode, result);
        return true;
    }

    CarrierTrackingJitter jitter{};
    if (!compute_carrier_tracking_jitter(config, state->mode, state->fll_pull_in_active, input.effective_cn0_dbhz,
                                         input.wavelength_m, input.dt_sec, &jitter, error_message)) {
        return false;
    }

    const double innovation_scale = std::sqrt(std::max(0.0, 1.0 - jitter.correlation_alpha * jitter.correlation_alpha));
    state->tracking_error_hz = jitter.correlation_alpha * state->tracking_error_hz +
                               jitter.sigma_hz * innovation_scale * input.standard_normal_sample;
    if (!std::isfinite(state->tracking_error_hz)) {
        return fail(error_message, "carrier tracking filtered error became non-finite");
    }

    result->mode = state->mode;
    result->fll_phase = fll_phase_from_state(*state);
    result->jitter = jitter;
    result->tracking_error_hz = state->tracking_error_hz;
    result->tracking_error_mps = input.wavelength_m * state->tracking_error_hz;
    result->carrier_lock_age_sec = state->carrier_lock_age_sec;
    result->pll_age_sec = state->pll_age_sec;
    result->carrier_segment_id = state->carrier_segment_id;
    result->doppler_valid = elapsed_reached(state->carrier_lock_age_sec, config.doppler_valid_delay_sec);
    result->adr_valid = state->mode == CarrierTrackingMode::kPllTrack &&
                        elapsed_reached(state->pll_age_sec, config.adr_valid_after_pll_sec);
    result->mode_changed = previous_mode != state->mode;
    result->new_carrier_segment = new_carrier_segment;
    return true;
}

const char* carrier_tracking_mode_name(CarrierTrackingMode mode) {
    switch (mode) {
        case CarrierTrackingMode::kCarrierUnlocked:
            return "CARRIER_UNLOCKED";
        case CarrierTrackingMode::kFllTrack:
            return "FLL_TRACK";
        case CarrierTrackingMode::kPllTrack:
            return "PLL_TRACK";
    }
    return "UNKNOWN";
}

const char* carrier_tracking_fll_phase_name(CarrierTrackingFllPhase phase) {
    switch (phase) {
        case CarrierTrackingFllPhase::kNone:
            return "NONE";
        case CarrierTrackingFllPhase::kPullIn:
            return "PULL_IN";
        case CarrierTrackingFllPhase::kSteady:
            return "STEADY";
    }
    return "UNKNOWN";
}

} // namespace gnss_sim
