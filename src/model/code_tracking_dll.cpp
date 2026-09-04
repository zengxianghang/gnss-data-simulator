#include "model/code_tracking_dll.h"

#include "model/code_correlation.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gnss_sim {
namespace {

constexpr double kDefaultEarlyLateTotalSpacingChips = 0.2;
constexpr double kMaxEarlyLateTotalSpacingChips = 2.0;
constexpr double kRootScanStepChips = 1.0 / 512.0;
constexpr int kMaxRootScanIntervals = 32768;
constexpr int kRootBisectionIterations = 80;
constexpr double kRootToleranceChips = 1.0e-11;
constexpr double kRootDedupToleranceChips = 1.0e-7;
constexpr double kSlopeProbeChips = 1.0e-5;
constexpr double kDiscriminatorZeroRelativeTolerance = 1.0e-13;
constexpr double kStableSlopeRelativeTolerance = 1.0e-10;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool finite_complex(const std::complex<double>& value) {
    return std::isfinite(value.real()) && std::isfinite(value.imag());
}

bool validate_paths(const SignalDefinition& signal, const CodeTrackingDllPath* paths, int path_count,
                    double* chip_rate_hz, double* power_scale, double* min_delay_chips, double* max_delay_chips,
                    std::string* error_message) {
    if (paths == nullptr || path_count <= 0) {
        set_error(error_message, "code tracking DLL requires at least one path");
        return false;
    }
    if (!signal_has_supported_code_correlation(signal)) {
        set_error(error_message, "signal has no supported ideal code-correlation profile");
        return false;
    }

    const double rate_hz = signal.code_correlation.chip_rate_hz;
    double sum_abs_voltage = 0.0;
    double min_delay = std::numeric_limits<double>::infinity();
    double max_delay = -std::numeric_limits<double>::infinity();
    for (int index = 0; index < path_count; ++index) {
        if (!std::isfinite(paths[index].code_delay_sec) || !finite_complex(paths[index].complex_voltage)) {
            set_error(error_message, "code tracking DLL path contains a non-finite delay or complex voltage");
            return false;
        }
        const double magnitude = std::abs(paths[index].complex_voltage);
        sum_abs_voltage += magnitude;
        const double delay_chips = paths[index].code_delay_sec * rate_hz;
        if (!std::isfinite(delay_chips) || !std::isfinite(sum_abs_voltage)) {
            set_error(error_message, "code tracking DLL path scale is outside the supported numeric range");
            return false;
        }
        min_delay = std::min(min_delay, delay_chips);
        max_delay = std::max(max_delay, delay_chips);
    }
    if (!(sum_abs_voltage > 0.0)) {
        set_error(error_message, "code tracking DLL requires non-zero path voltage");
        return false;
    }

    if (chip_rate_hz != nullptr) {
        *chip_rate_hz = rate_hz;
    }
    if (power_scale != nullptr) {
        *power_scale = sum_abs_voltage * sum_abs_voltage;
        if (!std::isfinite(*power_scale)) {
            set_error(error_message, "code tracking DLL voltage power scale is not finite");
            return false;
        }
    }
    if (min_delay_chips != nullptr) {
        *min_delay_chips = min_delay;
    }
    if (max_delay_chips != nullptr) {
        *max_delay_chips = max_delay;
    }
    return true;
}

bool composite_correlation_chips(const SignalDefinition& signal, const CodeTrackingDllPath* paths, int path_count,
                                 double local_code_phase_chips, std::complex<double>* correlation) {
    if (correlation == nullptr) {
        return false;
    }
    *correlation = {0.0, 0.0};
    const double rate_hz = signal.code_correlation.chip_rate_hz;
    for (int index = 0; index < path_count; ++index) {
        const double path_delay_chips = paths[index].code_delay_sec * rate_hz;
        std::complex<double> path_correlation{};
        if (!ideal_code_correlation_chips(signal.code_correlation, local_code_phase_chips - path_delay_chips,
                                          &path_correlation, nullptr)) {
            return false;
        }
        *correlation += paths[index].complex_voltage * path_correlation;
    }
    return finite_complex(*correlation);
}

bool discriminator_chips(const SignalDefinition& signal, const CodeTrackingDllPath* paths, int path_count,
                         const CodeTrackingDllConfig& config, double local_code_phase_chips, double* discriminator) {
    if (discriminator == nullptr) {
        return false;
    }
    const double half_spacing_chips = 0.5 * config.early_late_total_spacing_chips;
    std::complex<double> early{};
    std::complex<double> late{};
    if (!composite_correlation_chips(signal, paths, path_count, local_code_phase_chips - half_spacing_chips, &early) ||
        !composite_correlation_chips(signal, paths, path_count, local_code_phase_chips + half_spacing_chips, &late)) {
        return false;
    }
    *discriminator = std::norm(early) - std::norm(late);
    return std::isfinite(*discriminator);
}

bool prompt_power_chips(const SignalDefinition& signal, const CodeTrackingDllPath* paths, int path_count,
                        double local_code_phase_chips, double* prompt_power) {
    if (prompt_power == nullptr) {
        return false;
    }
    std::complex<double> prompt{};
    if (!composite_correlation_chips(signal, paths, path_count, local_code_phase_chips, &prompt)) {
        return false;
    }
    *prompt_power = std::norm(prompt);
    return std::isfinite(*prompt_power);
}

bool refine_sign_change_root(const SignalDefinition& signal, const CodeTrackingDllPath* paths, int path_count,
                             const CodeTrackingDllConfig& config, double left, double right, double left_value,
                             double right_value, double* root) {
    if (root == nullptr || !(left < right) || !std::isfinite(left_value) || !std::isfinite(right_value) ||
        left_value * right_value >= 0.0) {
        return false;
    }

    double a = left;
    double b = right;
    double fa = left_value;
    for (int iteration = 0; iteration < kRootBisectionIterations; ++iteration) {
        const double mid = 0.5 * (a + b);
        double fm = 0.0;
        if (!discriminator_chips(signal, paths, path_count, config, mid, &fm)) {
            return false;
        }
        if ((b - a) <= kRootToleranceChips) {
            a = mid;
            b = mid;
            break;
        }
        if (fm == 0.0) {
            a = mid;
            b = mid;
            break;
        }
        if ((fa < 0.0 && fm < 0.0) || (fa > 0.0 && fm > 0.0)) {
            a = mid;
            fa = fm;
        } else {
            b = mid;
        }
    }
    *root = 0.5 * (a + b);
    return std::isfinite(*root);
}

bool append_root(const SignalDefinition& signal, const CodeTrackingDllPath* paths, int path_count,
                 const CodeTrackingDllConfig& config, double root_chips, double power_scale, CodeTrackingDllRoot* roots,
                 int root_capacity, int* root_count, std::string* error_message) {
    if (roots == nullptr || root_count == nullptr || *root_count < 0 || *root_count > root_capacity) {
        set_error(error_message, "invalid code tracking DLL root output buffer");
        return false;
    }
    if (*root_count > 0 && std::abs(root_chips - roots[*root_count - 1].code_phase_chips) <= kRootDedupToleranceChips) {
        return true;
    }
    if (*root_count >= root_capacity || *root_count >= kMaxCodeTrackingDllRoots) {
        set_error(error_message, "code tracking DLL root output capacity is insufficient");
        return false;
    }

    double discriminator = 0.0;
    double left_value = 0.0;
    double right_value = 0.0;
    double prompt_power = 0.0;
    if (!discriminator_chips(signal, paths, path_count, config, root_chips, &discriminator) ||
        !discriminator_chips(signal, paths, path_count, config, root_chips - kSlopeProbeChips, &left_value) ||
        !discriminator_chips(signal, paths, path_count, config, root_chips + kSlopeProbeChips, &right_value) ||
        !prompt_power_chips(signal, paths, path_count, root_chips, &prompt_power)) {
        set_error(error_message, "failed to evaluate code tracking DLL root metrics");
        return false;
    }

    const double slope = (right_value - left_value) / (2.0 * kSlopeProbeChips);
    CodeTrackingDllRoot& output = roots[*root_count];
    output.code_phase_chips = root_chips;
    output.code_phase_sec = root_chips / signal.code_correlation.chip_rate_hz;
    output.discriminator = discriminator;
    output.discriminator_slope_per_chip = slope;
    output.prompt_power = prompt_power;
    output.stable = std::isfinite(slope) && slope > kStableSlopeRelativeTolerance * power_scale;
    ++(*root_count);
    return true;
}

bool prefer_tracked_candidate(const CodeTrackingDllRoot& candidate, const CodeTrackingDllRoot& current,
                              double previous_code_phase_sec) {
    const double candidate_distance = std::abs(candidate.code_phase_sec - previous_code_phase_sec);
    const double current_distance = std::abs(current.code_phase_sec - previous_code_phase_sec);
    if (candidate_distance != current_distance) {
        return candidate_distance < current_distance;
    }
    if (candidate.prompt_power != current.prompt_power) {
        return candidate.prompt_power > current.prompt_power;
    }
    return candidate.code_phase_sec < current.code_phase_sec;
}

bool prefer_acquisition_candidate(const CodeTrackingDllRoot& candidate, const CodeTrackingDllRoot& current) {
    if (candidate.prompt_power != current.prompt_power) {
        return candidate.prompt_power > current.prompt_power;
    }
    const double candidate_abs_phase = std::abs(candidate.code_phase_sec);
    const double current_abs_phase = std::abs(current.code_phase_sec);
    if (candidate_abs_phase != current_abs_phase) {
        return candidate_abs_phase < current_abs_phase;
    }
    return candidate.code_phase_sec < current.code_phase_sec;
}

} // namespace

CodeTrackingDllConfig default_code_tracking_dll_config() {
    CodeTrackingDllConfig config{};
    config.early_late_total_spacing_chips = kDefaultEarlyLateTotalSpacingChips;
    return config;
}

bool validate_code_tracking_dll_config(const CodeTrackingDllConfig& config, std::string* error_message) {
    if (!std::isfinite(config.early_late_total_spacing_chips) || config.early_late_total_spacing_chips <= 0.0 ||
        config.early_late_total_spacing_chips > kMaxEarlyLateTotalSpacingChips) {
        set_error(error_message, "early_late_total_spacing_chips must be finite and in (0, 2]");
        return false;
    }
    return true;
}

bool compute_code_tracking_composite_correlation(const SignalDefinition& signal, const CodeTrackingDllPath* paths,
                                                 int path_count, double local_code_phase_sec,
                                                 std::complex<double>* correlation, std::string* error_message) {
    if (correlation == nullptr) {
        set_error(error_message, "code tracking composite-correlation output pointer is null");
        return false;
    }
    *correlation = {0.0, 0.0};
    if (!std::isfinite(local_code_phase_sec)) {
        set_error(error_message, "code tracking local code phase is not finite");
        return false;
    }
    double chip_rate_hz = 0.0;
    if (!validate_paths(signal, paths, path_count, &chip_rate_hz, nullptr, nullptr, nullptr, error_message)) {
        return false;
    }
    const double local_phase_chips = local_code_phase_sec * chip_rate_hz;
    if (!std::isfinite(local_phase_chips) ||
        !composite_correlation_chips(signal, paths, path_count, local_phase_chips, correlation)) {
        set_error(error_message, "failed to evaluate code tracking composite correlation");
        return false;
    }
    return true;
}

bool compute_code_tracking_dll_discriminator(const SignalDefinition& signal, const CodeTrackingDllPath* paths,
                                             int path_count, const CodeTrackingDllConfig& config,
                                             double local_code_phase_sec, double* discriminator,
                                             std::string* error_message) {
    if (discriminator == nullptr) {
        set_error(error_message, "code tracking DLL discriminator output pointer is null");
        return false;
    }
    *discriminator = 0.0;
    if (!validate_code_tracking_dll_config(config, error_message) || !std::isfinite(local_code_phase_sec)) {
        if (!std::isfinite(local_code_phase_sec)) {
            set_error(error_message, "code tracking local code phase is not finite");
        }
        return false;
    }
    double chip_rate_hz = 0.0;
    if (!validate_paths(signal, paths, path_count, &chip_rate_hz, nullptr, nullptr, nullptr, error_message)) {
        return false;
    }
    const double local_phase_chips = local_code_phase_sec * chip_rate_hz;
    if (!std::isfinite(local_phase_chips) ||
        !discriminator_chips(signal, paths, path_count, config, local_phase_chips, discriminator)) {
        set_error(error_message, "failed to evaluate code tracking DLL discriminator");
        return false;
    }
    return true;
}

bool find_code_tracking_dll_roots(const SignalDefinition& signal, const CodeTrackingDllPath* paths, int path_count,
                                  const CodeTrackingDllConfig& config, CodeTrackingDllRoot* roots, int root_capacity,
                                  int* root_count, std::string* error_message) {
    if (root_count == nullptr || roots == nullptr || root_capacity <= 0 || root_capacity > kMaxCodeTrackingDllRoots) {
        set_error(error_message, "invalid code tracking DLL root output arguments");
        return false;
    }
    *root_count = 0;
    if (!validate_code_tracking_dll_config(config, error_message)) {
        return false;
    }

    double power_scale = 0.0;
    double min_delay_chips = 0.0;
    double max_delay_chips = 0.0;
    if (!validate_paths(signal, paths, path_count, nullptr, &power_scale, &min_delay_chips, &max_delay_chips,
                        error_message)) {
        return false;
    }

    const double half_spacing = 0.5 * config.early_late_total_spacing_chips;
    const double lower = min_delay_chips - 1.0 - half_spacing;
    const double upper = max_delay_chips + 1.0 + half_spacing;
    const double width = upper - lower;
    if (!(width > 0.0) || !std::isfinite(width)) {
        set_error(error_message, "code tracking DLL root search interval is invalid");
        return false;
    }
    const double interval_count_double = std::ceil(width / kRootScanStepChips);
    if (!std::isfinite(interval_count_double) || interval_count_double < 1.0 ||
        interval_count_double > static_cast<double>(kMaxRootScanIntervals)) {
        set_error(error_message, "code tracking DLL path-delay span exceeds the bounded V1 root-search interval");
        return false;
    }
    const int interval_count = static_cast<int>(interval_count_double);

    const double zero_tolerance = kDiscriminatorZeroRelativeTolerance * power_scale;
    bool have_last_nonzero = false;
    double last_x = 0.0;
    double last_value = 0.0;
    for (int sample = 0; sample <= interval_count; ++sample) {
        const double fraction = static_cast<double>(sample) / static_cast<double>(interval_count);
        const double x = lower + fraction * width;
        double value = 0.0;
        if (!discriminator_chips(signal, paths, path_count, config, x, &value)) {
            set_error(error_message, "failed while scanning code tracking DLL discriminator roots");
            *root_count = 0;
            return false;
        }
        if (std::abs(value) <= zero_tolerance) {
            continue;
        }
        if (have_last_nonzero && ((last_value < 0.0 && value > 0.0) || (last_value > 0.0 && value < 0.0))) {
            double root_chips = 0.0;
            if (!refine_sign_change_root(signal, paths, path_count, config, last_x, x, last_value, value,
                                         &root_chips) ||
                !append_root(signal, paths, path_count, config, root_chips, power_scale, roots, root_capacity,
                             root_count, error_message)) {
                *root_count = 0;
                return false;
            }
        }
        have_last_nonzero = true;
        last_x = x;
        last_value = value;
    }

    if (*root_count == 0) {
        set_error(error_message, "code tracking DLL found no discriminator roots in the active support region");
        return false;
    }
    return true;
}

bool select_code_tracking_dll_root(const CodeTrackingDllRoot* roots, int root_count, CodeTrackingDllSelectionMode mode,
                                   double previous_code_phase_sec, int* selected_index, std::string* error_message) {
    if (roots == nullptr || root_count <= 0 || selected_index == nullptr) {
        set_error(error_message, "invalid code tracking DLL root-selection arguments");
        return false;
    }
    *selected_index = -1;
    if (mode != CodeTrackingDllSelectionMode::TRACKED && mode != CodeTrackingDllSelectionMode::ACQUISITION) {
        set_error(error_message, "unknown code tracking DLL root-selection mode");
        return false;
    }
    if (mode == CodeTrackingDllSelectionMode::TRACKED && !std::isfinite(previous_code_phase_sec)) {
        set_error(error_message, "tracked DLL root selection requires a finite previous code phase");
        return false;
    }

    for (int index = 0; index < root_count; ++index) {
        if (!roots[index].stable) {
            continue;
        }
        if (*selected_index < 0) {
            *selected_index = index;
            continue;
        }
        const CodeTrackingDllRoot& candidate = roots[index];
        const CodeTrackingDllRoot& current = roots[*selected_index];
        bool prefer = false;
        switch (mode) {
            case CodeTrackingDllSelectionMode::TRACKED:
                prefer = prefer_tracked_candidate(candidate, current, previous_code_phase_sec);
                break;
            case CodeTrackingDllSelectionMode::ACQUISITION:
                prefer = prefer_acquisition_candidate(candidate, current);
                break;
        }
        if (prefer) {
            *selected_index = index;
        }
    }

    if (*selected_index < 0) {
        set_error(error_message, "code tracking DLL root set contains no stable candidate");
        return false;
    }
    return true;
}

} // namespace gnss_sim
