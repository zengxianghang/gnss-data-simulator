#include "model/urban_carrier_temporal.h"

#include <cmath>
#include <cstdint>

namespace gnss_sim {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kNanosecondsPerSecond = 1000000000.0;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool finite_complex(const std::complex<double>& value) {
    return std::isfinite(value.real()) && std::isfinite(value.imag());
}

bool tracking_phasor_available(const UrbanSignalEpochResult& epoch) {
    return epoch.tracking_phase == SignalTrackingPhase::kTracking && epoch.selected_root_valid &&
           epoch.tracked_composite_correlation_valid;
}

bool start_new_segment(const SignalDefinition& signal, int glonass_fcn, const SimTime& current_time,
                       const UrbanSignalEpochResult& epoch, double wavelength_m, UrbanCarrierTemporalState* state,
                       UrbanCarrierTemporalResult* result, std::string* error_message) {
    const std::complex<double> phasor = epoch.tracked_composite_correlation;
    if (!finite_complex(phasor) || !(std::norm(phasor) > 0.0)) {
        set_error(error_message, "urban carrier temporal state requires a finite non-zero tracked phasor");
        return false;
    }

    const double wrapped_phase_rad = std::arg(phasor);
    const double carrier_range_bias_m = -wrapped_phase_rad * wavelength_m / kTwoPi;
    if (!std::isfinite(wrapped_phase_rad) || !std::isfinite(carrier_range_bias_m)) {
        set_error(error_message, "urban carrier temporal fresh-lock phase is not finite");
        return false;
    }

    const bool prior_lock_segment = state->ever_had_lock;
    state->signal_id = signal.signal_id;
    state->glonass_fcn = glonass_fcn;
    state->previous_time = current_time;
    state->previous_wrapped_phase_rad = wrapped_phase_rad;
    state->unwrapped_phase_rad = wrapped_phase_rad;
    state->previous_carrier_range_bias_m = carrier_range_bias_m;
    state->configured = true;
    state->continuity_active = true;
    state->ever_had_lock = true;

    result->wavelength_m = wavelength_m;
    result->wrapped_phase_rad = wrapped_phase_rad;
    result->unwrapped_phase_rad = wrapped_phase_rad;
    result->carrier_range_bias_m = carrier_range_bias_m;
    result->environmental_range_rate_mps = 0.0;
    result->tracking_lock_valid = true;
    result->carrier_adr_valid = epoch.adr_valid && epoch.carrier_continuity_valid;
    result->phase_continuity_valid = true;
    result->environmental_range_rate_valid = false;
    result->cycle_slip_event = epoch.reacquisition_event || prior_lock_segment;
    return true;
}

} // namespace

void reset_urban_carrier_temporal_state(UrbanCarrierTemporalState* state) {
    if (state != nullptr) {
        *state = {};
    }
}

bool update_urban_carrier_temporal_state(const SignalDefinition& signal, int glonass_fcn,
                                         const SimTime& current_time, const UrbanSignalEpochResult& epoch,
                                         UrbanCarrierTemporalState* state, UrbanCarrierTemporalResult* result,
                                         std::string* error_message) {
    if (state == nullptr || result == nullptr || current_time.gps_week < 0 || current_time.tow_ns < 0 ||
        current_time.tow_ns >= GPS_WEEK_NANOSECONDS) {
        set_error(error_message, "urban carrier temporal update has invalid arguments");
        return false;
    }

    double wavelength_m = 0.0;
    if (!signal_wavelength_m(signal, glonass_fcn, &wavelength_m) || !std::isfinite(wavelength_m) ||
        !(wavelength_m > 0.0)) {
        set_error(error_message, "urban carrier temporal update cannot determine signal wavelength");
        return false;
    }
    if (state->configured && (state->signal_id != signal.signal_id || state->glonass_fcn != glonass_fcn)) {
        set_error(error_message, "urban carrier temporal state was configured for a different signal or GLONASS FCN");
        return false;
    }

    UrbanCarrierTemporalResult output{};
    output.wavelength_m = wavelength_m;
    if (!tracking_phasor_available(epoch)) {
        output.tracking_lock_valid = false;
        output.carrier_adr_valid = false;
        output.phase_continuity_valid = false;
        output.environmental_range_rate_valid = false;
        output.cycle_slip_event = state->continuity_active;
        state->continuity_active = false;
        *result = output;
        return true;
    }

    if (!state->continuity_active || epoch.reacquisition_event) {
        if (!start_new_segment(signal, glonass_fcn, current_time, epoch, wavelength_m, state, &output, error_message)) {
            return false;
        }
        *result = output;
        return true;
    }

    std::int64_t elapsed_ns = 0;
    if (!difference_time_ns(current_time, state->previous_time, &elapsed_ns) || elapsed_ns <= 0) {
        set_error(error_message, "urban carrier temporal tracking epochs must be strictly increasing");
        return false;
    }

    // A current lock younger than the elapsed time since our previous sample
    // proves a new lock started in the gap. Never differentiate across it even
    // if the explicit loss epoch was not supplied to this state object.
    if (epoch.lock_time_ns >= 0 && epoch.lock_time_ns < elapsed_ns) {
        state->continuity_active = false;
        if (!start_new_segment(signal, glonass_fcn, current_time, epoch, wavelength_m, state, &output, error_message)) {
            return false;
        }
        output.cycle_slip_event = true;
        *result = output;
        return true;
    }

    const std::complex<double> phasor = epoch.tracked_composite_correlation;
    if (!finite_complex(phasor) || !(std::norm(phasor) > 0.0)) {
        set_error(error_message, "urban carrier temporal state requires a finite non-zero tracked phasor");
        return false;
    }

    const double wrapped_phase_rad = std::arg(phasor);
    const double wrapped_delta_rad = std::remainder(wrapped_phase_rad - state->previous_wrapped_phase_rad, kTwoPi);
    const double unwrapped_phase_rad = state->unwrapped_phase_rad + wrapped_delta_rad;
    const double carrier_range_bias_m = -unwrapped_phase_rad * wavelength_m / kTwoPi;
    const double elapsed_sec = static_cast<double>(elapsed_ns) / kNanosecondsPerSecond;
    const double environmental_range_rate_mps =
        (carrier_range_bias_m - state->previous_carrier_range_bias_m) / elapsed_sec;
    if (!std::isfinite(wrapped_phase_rad) || !std::isfinite(wrapped_delta_rad) || !std::isfinite(unwrapped_phase_rad) ||
        !std::isfinite(carrier_range_bias_m) || !std::isfinite(environmental_range_rate_mps)) {
        set_error(error_message, "urban carrier temporal phase/range-rate result is not finite");
        return false;
    }

    state->previous_time = current_time;
    state->previous_wrapped_phase_rad = wrapped_phase_rad;
    state->unwrapped_phase_rad = unwrapped_phase_rad;
    state->previous_carrier_range_bias_m = carrier_range_bias_m;

    output.wavelength_m = wavelength_m;
    output.wrapped_phase_rad = wrapped_phase_rad;
    output.unwrapped_phase_rad = unwrapped_phase_rad;
    output.carrier_range_bias_m = carrier_range_bias_m;
    output.environmental_range_rate_mps = environmental_range_rate_mps;
    output.tracking_lock_valid = true;
    output.carrier_adr_valid = epoch.adr_valid && epoch.carrier_continuity_valid;
    output.phase_continuity_valid = true;
    output.environmental_range_rate_valid = true;
    output.cycle_slip_event = false;
    *result = output;
    return true;
}

} // namespace gnss_sim
