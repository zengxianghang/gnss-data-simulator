#include "model/signal_tracking.h"

#include "gnss_sim/sim_time.h"

#include <cmath>
#include <cstdint>

namespace gnss_sim {
namespace {

constexpr std::uint32_t kQuantileScale = 65536U;
constexpr std::uint32_t kP50Threshold = 32768U;
constexpr std::uint32_t kP95Threshold = 62259U;

constexpr std::int64_t milliseconds(std::int64_t value) {
    return value * 1000000LL;
}

constexpr std::int64_t seconds(std::int64_t value) {
    return value * NANOSECONDS_PER_SECOND;
}

DelayDistribution make_delay_ms(std::int64_t minimum_ms, std::int64_t p50_ms, std::int64_t p95_ms,
                                std::int64_t maximum_ms) {
    return {milliseconds(minimum_ms), milliseconds(p50_ms), milliseconds(p95_ms), milliseconds(maximum_ms)};
}

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_time(const SimTime& time) {
    return time.gps_week >= 0 && time.tow_ns >= 0 && time.tow_ns < GPS_WEEK_NANOSECONDS;
}

bool valid_delay_distribution(const DelayDistribution& distribution) {
    return distribution.minimum_ns >= 0 && distribution.minimum_ns <= distribution.p50_ns &&
           distribution.p50_ns <= distribution.p95_ns && distribution.p95_ns <= distribution.maximum_ns;
}

std::int64_t interpolate_integer(std::int64_t lower, std::int64_t upper, std::uint32_t numerator,
                                 std::uint32_t denominator) {
    if (upper <= lower || denominator == 0U) {
        return lower;
    }
    const std::int64_t span = upper - lower;
    return lower + (span * static_cast<std::int64_t>(numerator)) / static_cast<std::int64_t>(denominator);
}

std::int64_t sample_delay(const DelayDistribution& distribution, DeterministicRng* rng) {
    const std::uint32_t quantile = rng_next_u32(rng) >> 16U;
    if (quantile < kP50Threshold) {
        return interpolate_integer(distribution.minimum_ns, distribution.p50_ns, quantile, kP50Threshold);
    }
    if (quantile < kP95Threshold) {
        return interpolate_integer(distribution.p50_ns, distribution.p95_ns, quantile - kP50Threshold,
                                   kP95Threshold - kP50Threshold);
    }
    return interpolate_integer(distribution.p95_ns, distribution.maximum_ns, quantile - kP95Threshold,
                               kQuantileScale - kP95Threshold);
}

int cn0_band(double cn0_dbhz) {
    if (cn0_dbhz >= 40.0) {
        return 0;
    }
    if (cn0_dbhz >= 35.0) {
        return 1;
    }
    if (cn0_dbhz >= 30.0) {
        return 2;
    }
    return 3;
}

bool add_delay(const SimTime& time, std::int64_t delay_ns, SimTime* result) {
    return delay_ns >= 0 && add_time_ns(time, delay_ns, result);
}

void clear_observation_state(SignalTracker* tracker) {
    tracker->cn0_dbhz = 0.0;
    tracker->lock_time_ns = 0;
    tracker->psr_valid = false;
    tracker->doppler_valid = false;
    tracker->adr_valid = false;
    tracker->observation_available = false;
}

} // namespace

SignalTrackingModelConfig default_signal_tracking_model_config() {
    SignalTrackingModelConfig config{};
    config.hot_common_startup = make_delay_ms(400, 800, 1500, 2000);
    config.warm_common_startup = make_delay_ms(400, 800, 1500, 2000);
    config.warm_search_uncertainty = {seconds(2), seconds(5), seconds(15), seconds(25)};

    config.hot_signal_acquisition[0] = make_delay_ms(200, 350, 550, 700);
    config.hot_signal_acquisition[1] = make_delay_ms(300, 600, 950, 1200);
    config.hot_signal_acquisition[2] = make_delay_ms(600, 1100, 1900, 2400);
    config.hot_signal_acquisition[3] = make_delay_ms(1000, 2200, 3600, 4000);

    config.reacquisition[0] = make_delay_ms(80, 300, 800, 1100);
    config.reacquisition[1] = make_delay_ms(100, 500, 1100, 1500);
    config.reacquisition[2] = make_delay_ms(200, 850, 1600, 2000);
    config.reacquisition[3] = make_delay_ms(500, 1300, 2200, 3000);

    config.psr_valid_delay_ns = milliseconds(100);
    config.doppler_valid_delay_ns = milliseconds(150);
    config.adr_valid_delay_ns = milliseconds(500);
    return config;
}

bool validate_signal_tracking_model_config(const SignalTrackingModelConfig& config, std::string* error_message) {
    if (!valid_delay_distribution(config.hot_common_startup) || !valid_delay_distribution(config.warm_common_startup) ||
        !valid_delay_distribution(config.warm_search_uncertainty)) {
        set_error(error_message, "startup delay distribution is invalid");
        return false;
    }
    for (int index = 0; index < 4; ++index) {
        if (!valid_delay_distribution(config.hot_signal_acquisition[index]) ||
            !valid_delay_distribution(config.reacquisition[index])) {
            set_error(error_message, "signal acquisition delay distribution is invalid");
            return false;
        }
    }
    if (config.psr_valid_delay_ns < 0 || config.doppler_valid_delay_ns < 0 || config.adr_valid_delay_ns < 0) {
        set_error(error_message, "measurement-valid delays must be nonnegative");
        return false;
    }
    return true;
}

bool sample_receiver_startup_timing(StartupMode startup_mode, const SignalTrackingModelConfig& config,
                                    DeterministicRng* rng, ReceiverStartupTiming* timing) {
    if (rng == nullptr || timing == nullptr || !validate_signal_tracking_model_config(config, nullptr)) {
        return false;
    }

    ReceiverStartupTiming result{};
    result.startup_mode = startup_mode;
    if (startup_mode == StartupMode::WARM) {
        result.common_startup_delay_ns = sample_delay(config.warm_common_startup, rng);
        result.search_uncertainty_delay_ns = sample_delay(config.warm_search_uncertainty, rng);
    } else {
        result.common_startup_delay_ns = sample_delay(config.hot_common_startup, rng);
        result.search_uncertainty_delay_ns = 0;
    }
    result.total_search_ready_delay_ns = result.common_startup_delay_ns + result.search_uncertainty_delay_ns;
    *timing = result;
    return true;
}

void reset_signal_tracker(SignalTracker* tracker, SignalId signal_id, const SimTime& reset_time) {
    if (tracker == nullptr) {
        return;
    }
    *tracker = {};
    tracker->signal_id = signal_id;
    tracker->phase = SignalTrackingPhase::kSignalOff;
    tracker->acquisition_context = AcquisitionContext::kHot;
    tracker->state_since = reset_time;
    tracker->search_ready_time = reset_time;
    tracker->acquisition_complete_time = reset_time;
    tracker->tracking_start_time = reset_time;
    tracker->psr_valid_time = reset_time;
    tracker->doppler_valid_time = reset_time;
    tracker->adr_valid_time = reset_time;
    clear_observation_state(tracker);
}

bool schedule_signal_acquisition(SignalTracker* tracker, AcquisitionContext context, const SimTime& signal_on_time,
                                 const SimTime& search_ready_time, double elevation_deg, const Cn0Model& cn0_model,
                                 const SignalTrackingModelConfig& config, DeterministicRng* rng,
                                 std::string* error_message) {
    if (tracker == nullptr || rng == nullptr || !valid_time(signal_on_time) || !valid_time(search_ready_time) ||
        compare_sim_time(search_ready_time, signal_on_time) < 0 || !std::isfinite(elevation_deg) ||
        find_signal_definition(tracker->signal_id) == nullptr || !validate_signal_tracking_model_config(config, nullptr)) {
        set_error(error_message, "signal acquisition schedule has invalid arguments");
        return false;
    }

    double acquisition_cn0_dbhz = 0.0;
    if (!cn0_model_estimate_dbhz(cn0_model, tracker->signal_id, elevation_deg, search_ready_time,
                                 &acquisition_cn0_dbhz)) {
        set_error(error_message, "cannot evaluate CN0 for signal acquisition");
        return false;
    }

    const int band = cn0_band(acquisition_cn0_dbhz);
    const DelayDistribution& distribution = context == AcquisitionContext::kReacquisition
                                                ? config.reacquisition[band]
                                                : config.hot_signal_acquisition[band];
    const std::int64_t acquisition_delay_ns = sample_delay(distribution, rng);

    SimTime acquisition_complete_time{};
    SimTime psr_valid_time{};
    SimTime doppler_valid_time{};
    SimTime adr_valid_time{};
    if (!add_delay(search_ready_time, acquisition_delay_ns, &acquisition_complete_time) ||
        !add_delay(acquisition_complete_time, config.psr_valid_delay_ns, &psr_valid_time) ||
        !add_delay(acquisition_complete_time, config.doppler_valid_delay_ns, &doppler_valid_time) ||
        !add_delay(acquisition_complete_time, config.adr_valid_delay_ns, &adr_valid_time)) {
        set_error(error_message, "signal acquisition schedule overflows simulation time");
        return false;
    }

    tracker->phase = SignalTrackingPhase::kSearching;
    tracker->acquisition_context = context;
    tracker->state_since = signal_on_time;
    tracker->search_ready_time = search_ready_time;
    tracker->acquisition_complete_time = acquisition_complete_time;
    tracker->tracking_start_time = acquisition_complete_time;
    tracker->psr_valid_time = psr_valid_time;
    tracker->doppler_valid_time = doppler_valid_time;
    tracker->adr_valid_time = adr_valid_time;
    tracker->cn0_dbhz = acquisition_cn0_dbhz;
    tracker->scheduled = true;
    tracker->lock_time_ns = 0;
    tracker->psr_valid = false;
    tracker->doppler_valid = false;
    tracker->adr_valid = false;
    tracker->observation_available = false;
    return true;
}

bool update_signal_tracker(SignalTracker* tracker, const SimTime& current_time, bool signal_available,
                           double elevation_deg, const Cn0Model& cn0_model, std::string* error_message) {
    if (tracker == nullptr || !valid_time(current_time) || !std::isfinite(elevation_deg) ||
        find_signal_definition(tracker->signal_id) == nullptr) {
        set_error(error_message, "signal tracking update has invalid arguments");
        return false;
    }

    if (!signal_available) {
        tracker->phase = SignalTrackingPhase::kSignalOff;
        tracker->state_since = current_time;
        tracker->scheduled = false;
        clear_observation_state(tracker);
        return true;
    }
    if (!tracker->scheduled) {
        set_error(error_message, "signal is available but acquisition has not been scheduled");
        return false;
    }
    if (compare_sim_time(current_time, tracker->state_since) < 0) {
        set_error(error_message, "signal tracking update precedes the scheduled signal-on time");
        return false;
    }

    if (!cn0_model_estimate_dbhz(cn0_model, tracker->signal_id, elevation_deg, current_time, &tracker->cn0_dbhz)) {
        set_error(error_message, "cannot evaluate CN0 for signal tracking");
        return false;
    }

    if (compare_sim_time(current_time, tracker->search_ready_time) < 0) {
        tracker->phase = SignalTrackingPhase::kSearching;
        tracker->lock_time_ns = 0;
        tracker->psr_valid = false;
        tracker->doppler_valid = false;
        tracker->adr_valid = false;
        tracker->observation_available = false;
        return true;
    }
    if (compare_sim_time(current_time, tracker->acquisition_complete_time) < 0) {
        if (tracker->phase != SignalTrackingPhase::kAcquiring) {
            tracker->phase = SignalTrackingPhase::kAcquiring;
            tracker->state_since = tracker->search_ready_time;
        }
        tracker->lock_time_ns = 0;
        tracker->psr_valid = false;
        tracker->doppler_valid = false;
        tracker->adr_valid = false;
        tracker->observation_available = false;
        return true;
    }

    tracker->phase = SignalTrackingPhase::kTracking;
    tracker->state_since = tracker->tracking_start_time;
    std::int64_t lock_time_ns = 0;
    if (!difference_time_ns(current_time, tracker->tracking_start_time, &lock_time_ns) || lock_time_ns < 0) {
        set_error(error_message, "cannot compute signal lock time");
        return false;
    }
    tracker->lock_time_ns = lock_time_ns;
    tracker->psr_valid = compare_sim_time(current_time, tracker->psr_valid_time) >= 0;
    tracker->doppler_valid = compare_sim_time(current_time, tracker->doppler_valid_time) >= 0;
    tracker->adr_valid = compare_sim_time(current_time, tracker->adr_valid_time) >= 0;
    tracker->observation_available = tracker->psr_valid;
    return true;
}

const char* signal_tracking_phase_name(SignalTrackingPhase phase) {
    switch (phase) {
        case SignalTrackingPhase::kSignalOff:
            return "SIGNAL_OFF";
        case SignalTrackingPhase::kSearching:
            return "SEARCHING";
        case SignalTrackingPhase::kAcquiring:
            return "ACQUIRING";
        case SignalTrackingPhase::kTracking:
            return "TRACKING";
    }
    return "UNKNOWN";
}

} // namespace gnss_sim
