#include "model/measurement_error_model.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace gnss_sim {
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kNanosecondsPerSecond = 1000000000.0;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

std::uint64_t mix64(std::uint64_t value) {
    value ^= value >> 30U;
    value *= UINT64_C(0xbf58476d1ce4e5b9);
    value ^= value >> 27U;
    value *= UINT64_C(0x94d049bb133111eb);
    value ^= value >> 31U;
    return value;
}

bool finite_nonnegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool same_lock(const MeasurementErrorState& state, const SignalTracker& tracker, int satellite_number) {
    return state.initialized && state.signal_id == tracker.signal_id && state.satellite_number == satellite_number &&
           compare_sim_time(state.tracking_start_time, tracker.tracking_start_time) == 0;
}

std::uint64_t lock_seed(std::uint64_t run_seed, const SignalTracker& tracker, int satellite_number) {
    std::uint64_t value = mix64(run_seed ^ UINT64_C(0x6a09e667f3bcc909));
    value ^=
        mix64(static_cast<std::uint64_t>(static_cast<unsigned int>(satellite_number)) + UINT64_C(0x9e3779b97f4a7c15));
    value ^= mix64(static_cast<std::uint64_t>(static_cast<unsigned int>(tracker.signal_id) + 1U) << 17U);
    value ^= mix64(static_cast<std::uint64_t>(static_cast<unsigned int>(tracker.tracking_start_time.gps_week)) << 32U);
    value ^= mix64(static_cast<std::uint64_t>(tracker.tracking_start_time.tow_ns));
    return mix64(value);
}

void initialize_lock_state(MeasurementErrorState* state, std::uint64_t run_seed, const SignalTracker& tracker,
                           int satellite_number) {
    const std::uint64_t seed = lock_seed(run_seed, tracker, satellite_number);
    const std::uint64_t sequence = mix64(seed ^ UINT64_C(0xbb67ae8584caa73b));
    *state = {};
    state->signal_id = tracker.signal_id;
    state->satellite_number = satellite_number;
    state->tracking_start_time = tracker.tracking_start_time;
    state->last_lock_time_ns = tracker.lock_time_ns;
    seed_rng(&state->rng, seed, sequence);
    state->initialized = true;
}

double gaussian(MeasurementErrorState* state) {
    if (state->have_spare_gaussian) {
        state->have_spare_gaussian = false;
        return state->spare_gaussian;
    }

    double u1 = 0.0;
    do {
        u1 = rng_uniform_01(&state->rng);
    } while (u1 <= 0.0);
    const double u2 = rng_uniform_01(&state->rng);
    const double radius = std::sqrt(-2.0 * std::log(u1));
    const double angle = kTwoPi * u2;
    state->spare_gaussian = radius * std::sin(angle);
    state->have_spare_gaussian = true;
    return radius * std::cos(angle);
}

const MeasurementTransientErrorConfig* transient_config(const MeasurementErrorConfig& config,
                                                        MeasurementErrorPhase phase) {
    switch (phase) {
        case MeasurementErrorPhase::kTtffHot:
            return &config.ttff_hot;
        case MeasurementErrorPhase::kTtffWarm:
            return &config.ttff_warm;
        case MeasurementErrorPhase::kTtffCold:
            return &config.ttff_cold;
        case MeasurementErrorPhase::kReaReacquisition:
            return &config.rea_reacquisition;
        case MeasurementErrorPhase::kStable:
        case MeasurementErrorPhase::kReaFadeOut:
            return nullptr;
    }
    return nullptr;
}

bool valid_context(const MeasurementErrorContext& context) {
    if (context.phase == MeasurementErrorPhase::kReaFadeOut) {
        return std::isfinite(context.rea_fade_progress) && context.rea_fade_progress >= 0.0 &&
               context.rea_fade_progress <= 1.0;
    }
    return std::isfinite(context.rea_fade_progress);
}

} // namespace

void reset_measurement_error_state(MeasurementErrorState* state) {
    if (state != nullptr) {
        *state = {};
    }
}

bool compute_measurement_error_sigmas(const MeasurementErrorConfig& config, const MeasurementErrorContext& context,
                                      std::int64_t lock_time_ns, MeasurementErrorSigmas* sigmas,
                                      std::string* error_message) {
    if (sigmas == nullptr || lock_time_ns < 0 || !valid_context(context) || !finite_nonnegative(config.psr_sigma_m) ||
        !finite_nonnegative(config.doppler_sigma_mps) || !finite_nonnegative(config.adr_sigma_m) ||
        !finite_nonnegative(config.cn0_sigma_dbhz) || !std::isfinite(config.psr_correlation_tau_sec) ||
        config.psr_correlation_tau_sec <= 0.0) {
        set_error(error_message, "measurement error sigma request has invalid arguments");
        return false;
    }

    MeasurementErrorSigmas result{};
    result.psr_sigma_m = config.psr_sigma_m;
    result.doppler_sigma_mps = config.doppler_sigma_mps;
    result.adr_sigma_m = config.adr_sigma_m;
    result.cn0_sigma_dbhz = config.cn0_sigma_dbhz;
    result.cn0_drop_db = 0.0;

    if (context.phase == MeasurementErrorPhase::kReaFadeOut) {
        const double progress = context.rea_fade_progress;
        result.psr_sigma_m = std::hypot(result.psr_sigma_m, config.rea_fade.psr_extra_sigma_m * progress);
        result.doppler_sigma_mps =
            std::hypot(result.doppler_sigma_mps, config.rea_fade.doppler_extra_sigma_mps * progress);
        result.cn0_drop_db = config.rea_fade.cn0_drop_db * progress;
        *sigmas = result;
        return true;
    }

    const MeasurementTransientErrorConfig* transient = transient_config(config, context.phase);
    if (transient != nullptr) {
        if (!finite_nonnegative(transient->psr_extra_sigma_m) ||
            !finite_nonnegative(transient->doppler_extra_sigma_mps) ||
            !finite_nonnegative(transient->cn0_extra_sigma_dbhz) || !std::isfinite(transient->decay_tau_sec) ||
            transient->decay_tau_sec <= 0.0) {
            set_error(error_message, "measurement transient configuration is invalid");
            return false;
        }
        const double elapsed_sec = static_cast<double>(lock_time_ns) / kNanosecondsPerSecond;
        const double decay = std::exp(-elapsed_sec / transient->decay_tau_sec);
        result.psr_sigma_m = std::hypot(result.psr_sigma_m, transient->psr_extra_sigma_m * decay);
        result.doppler_sigma_mps = std::hypot(result.doppler_sigma_mps, transient->doppler_extra_sigma_mps * decay);
        result.cn0_sigma_dbhz = std::hypot(result.cn0_sigma_dbhz, transient->cn0_extra_sigma_dbhz * decay);
    }

    *sigmas = result;
    return true;
}

bool apply_measurement_error(const MeasurementErrorConfig& config, std::uint64_t run_seed,
                             const MeasurementErrorContext& context, const SignalTracker& tracker,
                             const MeasurementObservation& zero_noise_observation, MeasurementErrorState* state,
                             MeasurementObservation* reported_observation, std::string* error_message) {
    if (state == nullptr || reported_observation == nullptr || zero_noise_observation.satellite_number <= 0 ||
        zero_noise_observation.signal_id != tracker.signal_id || !std::isfinite(zero_noise_observation.wavelength_m) ||
        zero_noise_observation.wavelength_m <= 0.0 || tracker.lock_time_ns < 0 || !valid_context(context)) {
        set_error(error_message, "measurement error application has invalid arguments");
        return false;
    }

    MeasurementObservation result = zero_noise_observation;
    if (tracker.phase != SignalTrackingPhase::kTracking) {
        reset_measurement_error_state(state);
        *reported_observation = result;
        return true;
    }

    const bool new_lock = !same_lock(*state, tracker, zero_noise_observation.satellite_number);
    if (new_lock) {
        initialize_lock_state(state, run_seed, tracker, zero_noise_observation.satellite_number);
    } else if (tracker.lock_time_ns < state->last_lock_time_ns) {
        set_error(error_message, "measurement error lock time is non-monotonic");
        return false;
    }

    MeasurementErrorSigmas sigmas{};
    if (!compute_measurement_error_sigmas(config, context, tracker.lock_time_ns, &sigmas, error_message)) {
        return false;
    }

    if (new_lock) {
        state->psr_correlated_error_m = sigmas.psr_sigma_m * gaussian(state);
    } else {
        const double dt_sec =
            static_cast<double>(tracker.lock_time_ns - state->last_lock_time_ns) / kNanosecondsPerSecond;
        const double phi = std::exp(-dt_sec / config.psr_correlation_tau_sec);
        const double innovation_scale = std::sqrt(std::max(0.0, 1.0 - phi * phi));
        state->psr_correlated_error_m =
            phi * state->psr_correlated_error_m + sigmas.psr_sigma_m * innovation_scale * gaussian(state);
    }
    state->last_lock_time_ns = tracker.lock_time_ns;

    if (result.pseudorange_valid) {
        result.pseudorange_m += state->psr_correlated_error_m;
    }
    if (result.doppler_valid) {
        result.doppler_hz += sigmas.doppler_sigma_mps * gaussian(state) / result.wavelength_m;
    }
    if (result.adr_valid) {
        result.adr_cycles += sigmas.adr_sigma_m * gaussian(state) / result.wavelength_m;
    }
    result.cn0_dbhz = std::max(0.0, result.cn0_dbhz + sigmas.cn0_sigma_dbhz * gaussian(state) - sigmas.cn0_drop_db);

    *reported_observation = result;
    return true;
}

} // namespace gnss_sim
