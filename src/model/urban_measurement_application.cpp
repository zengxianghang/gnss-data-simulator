#include "model/measurement_model.h"

#include "model/urban_carrier_temporal.h"
#include "model/urban_signal_epoch.h"

#include <cmath>

namespace gnss_sim {
namespace {

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_effective_cn0(double cn0_dbhz) {
    return std::isfinite(cn0_dbhz) || (std::isinf(cn0_dbhz) && cn0_dbhz < 0.0);
}

bool wavelength_matches(double clean_wavelength_m, double temporal_wavelength_m) {
    if (!std::isfinite(clean_wavelength_m) || !std::isfinite(temporal_wavelength_m) ||
        !(clean_wavelength_m > 0.0) || !(temporal_wavelength_m > 0.0)) {
        return false;
    }
    const double tolerance_m = 1.0e-12 * clean_wavelength_m + 1.0e-15;
    return std::abs(clean_wavelength_m - temporal_wavelength_m) <= tolerance_m;
}

} // namespace

bool apply_urban_measurement_effects(const UrbanSignalEpochResult& epoch,
                                     const UrbanCarrierTemporalResult& temporal,
                                     MeasurementObservation* observation, std::string* error_message) {
    if (observation == nullptr || !wavelength_matches(observation->wavelength_m, temporal.wavelength_m) ||
        !std::isfinite(observation->pseudorange_m) || !std::isfinite(observation->range_rate_mps) ||
        !std::isfinite(observation->doppler_hz) || !std::isfinite(observation->adr_cycles) ||
        !std::isfinite(epoch.code_bias_m) || !valid_effective_cn0(epoch.effective_cn0_dbhz) ||
        epoch.lock_time_ns < 0 || !std::isfinite(temporal.carrier_range_bias_m) ||
        !std::isfinite(temporal.environmental_range_rate_mps)) {
        set_error(error_message, "urban measurement application has invalid arguments");
        return false;
    }

    const bool epoch_tracking_phasor = epoch.tracking_phase == SignalTrackingPhase::kTracking &&
                                       epoch.selected_root_valid && epoch.tracked_composite_correlation_valid;
    if (temporal.tracking_lock_valid != epoch_tracking_phasor ||
        (temporal.phase_continuity_valid && !temporal.tracking_lock_valid) ||
        (temporal.environmental_range_rate_valid && !temporal.phase_continuity_valid) ||
        (temporal.carrier_adr_valid && !temporal.phase_continuity_valid)) {
        set_error(error_message, "urban measurement epoch and temporal states are inconsistent");
        return false;
    }

    MeasurementObservation output = *observation;

    // Keep MeasurementObservation::code_bias_m as the broadcast/explicit code
    // bias diagnostic. The environmental DLL bias is a separate physical term
    // and therefore only shifts the final pseudorange here.
    if (epoch.selected_root_valid) {
        output.pseudorange_m += epoch.code_bias_m;
    }

    // The #141 environmental range rate is additive to the authentic-NAV clean
    // range rate exactly once. Preserve a finite numeric baseline when the
    // derivative is unavailable, but mark Doppler invalid below rather than
    // silently assuming zero environmental rate.
    if (temporal.environmental_range_rate_valid) {
        output.range_rate_mps += temporal.environmental_range_rate_mps;
        output.doppler_hz -= temporal.environmental_range_rate_mps / output.wavelength_m;
    }

    // Clean ADR already contains clock/ionosphere/troposphere and the existing
    // deterministic ambiguity segment. Add only the continuous environmental
    // carrier-range term from #141; never create a second ambiguity here.
    if (temporal.phase_continuity_valid) {
        output.adr_cycles += temporal.carrier_range_bias_m / output.wavelength_m;
    }

    output.cn0_dbhz = epoch.effective_cn0_dbhz;
    output.lock_time_ns = epoch.lock_time_ns;

    const bool urban_observation_available = epoch.observation_available && temporal.tracking_lock_valid;
    output.observation_available = output.observation_available && urban_observation_available;
    output.pseudorange_valid = output.pseudorange_valid && epoch.psr_valid && epoch.selected_root_valid;
    output.doppler_valid = output.doppler_valid && epoch.doppler_valid && temporal.environmental_range_rate_valid;
    output.adr_valid = output.adr_valid && epoch.adr_valid && temporal.carrier_adr_valid;

    if (!output.observation_available) {
        output.pseudorange_valid = false;
        output.doppler_valid = false;
        output.adr_valid = false;
    }

    *observation = output;
    return true;
}

} // namespace gnss_sim
