#ifndef GNSS_SIM_SRC_MODEL_CARRIER_TRACKING_H_
#define GNSS_SIM_SRC_MODEL_CARRIER_TRACKING_H_

#include <cstdint>
#include <string>

namespace gnss_sim {

enum class CarrierTrackingMode {
    kCarrierUnlocked,
    kFllTrack,
    kPllTrack,
};

enum class CarrierTrackingFllPhase {
    kNone,
    kPullIn,
    kSteady,
};

struct CarrierTrackingConfig {
    double coherent_integration_sec;
    double pll_noise_bandwidth_hz;
    double fll_noise_bandwidth_hz;
    double fll_pull_in_bandwidth_hz;
    double fll_pull_in_duration_sec;

    double pll_enter_cn0_dbhz;
    double pll_exit_cn0_dbhz;
    double pll_enter_persistence_sec;
    double pll_exit_persistence_sec;

    double fll_enter_cn0_dbhz;
    double fll_exit_cn0_dbhz;
    double fll_enter_persistence_sec;
    double fll_exit_persistence_sec;

    double doppler_valid_delay_sec;
    double adr_valid_after_pll_sec;
};

struct CarrierTrackingState {
    CarrierTrackingMode mode;
    bool fll_pull_in_active;
    double mode_age_sec;
    double carrier_lock_age_sec;
    double pll_age_sec;
    double fll_enter_persistence_sec;
    double fll_exit_persistence_sec;
    double pll_enter_persistence_sec;
    double pll_exit_persistence_sec;
    double tracking_error_hz;
    std::uint64_t carrier_segment_id;
};

struct CarrierTrackingInput {
    bool signal_available;
    double effective_cn0_dbhz;
    double wavelength_m;
    double dt_sec;
    // One explicit N(0,1) draw owned by the caller. The core never owns or
    // advances a random-number generator, so runtime integration can preserve
    // deterministic per-signal RNG ordering.
    double standard_normal_sample;
};

struct CarrierTrackingJitter {
    double cn0_linear_hz;
    double active_bandwidth_hz;
    double phase_sigma_rad;
    double sigma_hz;
    double sigma_mps;
    double correlation_tau_sec;
    double correlation_alpha;
};

struct CarrierTrackingResult {
    CarrierTrackingMode mode;
    CarrierTrackingFllPhase fll_phase;
    CarrierTrackingJitter jitter;
    double tracking_error_hz;
    double tracking_error_mps;
    double carrier_lock_age_sec;
    double pll_age_sec;
    std::uint64_t carrier_segment_id;
    bool doppler_valid;
    bool adr_valid;
    bool mode_changed;
    bool new_carrier_segment;
};

CarrierTrackingConfig default_carrier_tracking_config();
bool validate_carrier_tracking_config(const CarrierTrackingConfig& config, std::string* error_message);

void reset_carrier_tracking_state(CarrierTrackingState* state);

// Compute the thermal-jitter scale and loop-derived first-order correlation
// coefficient for one active carrier-tracking mode. The FLL uses the standard
// differential-phase thermal frequency-jitter expression with discriminator
// factor F=1.0. For PLL mode, phase jitter from an arctangent PLL is converted
// to an equivalent one-integration frequency jitter by sigma_f=sigma_phi/(2*pi*T).
bool compute_carrier_tracking_jitter(const CarrierTrackingConfig& config, CarrierTrackingMode mode,
                                     bool fll_pull_in_active, double effective_cn0_dbhz, double wavelength_m,
                                     double dt_sec, CarrierTrackingJitter* jitter, std::string* error_message);

// Advance one per-satellite/per-signal carrier tracker. State transitions are
// driven by effective CN0 hysteresis/persistence. The supplied Gaussian sample
// is filtered with alpha=exp(-dt/tau), tau=1/(2*pi*Bn).
bool update_carrier_tracking(const CarrierTrackingConfig& config, const CarrierTrackingInput& input,
                             CarrierTrackingState* state, CarrierTrackingResult* result, std::string* error_message);

const char* carrier_tracking_mode_name(CarrierTrackingMode mode);
const char* carrier_tracking_fll_phase_name(CarrierTrackingFllPhase phase);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_CARRIER_TRACKING_H_
