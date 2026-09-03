#ifndef GNSS_SIM_SRC_MODEL_CODE_TRACKING_DLL_H_
#define GNSS_SIM_SRC_MODEL_CODE_TRACKING_DLL_H_

#include "gnss/signal_definitions.h"

#include <cmath>
#include <complex>
#include <string>

namespace gnss_sim {

constexpr int kMaxCodeTrackingDllRoots = 64;

struct CodeTrackingDllConfig {
    double early_late_total_spacing_chips;
};

struct CodeTrackingDllPath {
    // Relative propagation/code delay from the caller's chosen reference path.
    // #120 owns the production mapping from urban propagation into this value.
    double code_delay_sec;

    // Normalized complex voltage contribution in one common receiver-domain
    // convention. #119 does not reinterpret RF/material/antenna/CN0 terms.
    std::complex<double> complex_voltage;
};

struct CodeTrackingDllRoot {
    double code_phase_sec;
    double code_phase_chips;
    double discriminator;
    double discriminator_slope_per_chip;
    double prompt_power;
    bool stable;
};

enum class CodeTrackingDllSelectionMode {
    TRACKED = 0,
    ACQUISITION,
};

enum class CodeTrackingDllRootSearchStatus {
    kRootsFound = 0,
    kNoRoots,
};

CodeTrackingDllConfig default_code_tracking_dll_config();
bool validate_code_tracking_dll_config(const CodeTrackingDllConfig& config, std::string* error_message);

bool compute_code_tracking_composite_correlation(const SignalDefinition& signal, const CodeTrackingDllPath* paths,
                                                 int path_count, double local_code_phase_sec,
                                                 std::complex<double>* correlation, std::string* error_message);

// D(epsilon) = |Z(epsilon - spacing/2)|^2 - |Z(epsilon + spacing/2)|^2.
// With the documented correction law epsilon_next = epsilon - k*D(epsilon),
// stable roots have positive local dD/depsilon.
bool compute_code_tracking_dll_discriminator(const SignalDefinition& signal, const CodeTrackingDllPath* paths,
                                             int path_count, const CodeTrackingDllConfig& config,
                                             double local_code_phase_sec, double* discriminator,
                                             std::string* error_message);

// Enumerate deterministic discriminator roots in the active correlation support
// region. Both stable and unstable roots are surfaced; callers select only stable
// roots according to the explicit policy below.
bool find_code_tracking_dll_roots(const SignalDefinition& signal, const CodeTrackingDllPath* paths, int path_count,
                                  const CodeTrackingDllConfig& config, CodeTrackingDllRoot* roots, int root_capacity,
                                  int* root_count, std::string* error_message);

// Status-preserving compatibility layer for propagation/tracking orchestration.
// The existing root finder remains authoritative and unchanged. This wrapper
// only recognizes the exact receiver-domain coherent-null case where every
// equal-delay complex-voltage group cancels to zero; that valid physical state
// has no trackable DLL root and is returned as kNoRoots rather than an error.
inline bool find_code_tracking_dll_roots_with_status(const SignalDefinition& signal,
                                                     const CodeTrackingDllPath* paths, int path_count,
                                                     const CodeTrackingDllConfig& config, CodeTrackingDllRoot* roots,
                                                     int root_capacity, int* root_count,
                                                     CodeTrackingDllRootSearchStatus* status,
                                                     std::string* error_message) {
    if (roots == nullptr || root_count == nullptr || status == nullptr || root_capacity <= 0 ||
        root_capacity > kMaxCodeTrackingDllRoots) {
        if (error_message != nullptr) {
            *error_message = "invalid code tracking DLL root-status output arguments";
        }
        return false;
    }
    *root_count = 0;
    *status = CodeTrackingDllRootSearchStatus::kNoRoots;

    // Use the public correlator as the authoritative validation of signal
    // correlation support, path count, finite delays/voltages and non-zero
    // aggregate path voltage before classifying a coherent null.
    std::complex<double> validation_correlation{};
    if (!validate_code_tracking_dll_config(config, error_message) ||
        !compute_code_tracking_composite_correlation(signal, paths, path_count, 0.0, &validation_correlation,
                                                     error_message)) {
        return false;
    }

    bool all_delay_groups_cancel = true;
    for (int index = 0; index < path_count && all_delay_groups_cancel; ++index) {
        bool first_at_delay = true;
        for (int previous = 0; previous < index; ++previous) {
            if (paths[previous].code_delay_sec == paths[index].code_delay_sec) {
                first_at_delay = false;
                break;
            }
        }
        if (!first_at_delay) {
            continue;
        }

        std::complex<double> group_voltage{};
        for (int candidate = index; candidate < path_count; ++candidate) {
            if (paths[candidate].code_delay_sec == paths[index].code_delay_sec) {
                group_voltage += paths[candidate].complex_voltage;
            }
        }
        all_delay_groups_cancel = group_voltage == std::complex<double>{0.0, 0.0};
    }
    if (all_delay_groups_cancel) {
        return true;
    }

    if (!find_code_tracking_dll_roots(signal, paths, path_count, config, roots, root_capacity, root_count,
                                      error_message)) {
        return false;
    }
    *status = CodeTrackingDllRootSearchStatus::kRootsFound;
    return true;
}

bool select_code_tracking_dll_root(const CodeTrackingDllRoot* roots, int root_count, CodeTrackingDllSelectionMode mode,
                                   double previous_code_phase_sec, int* selected_index, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_CODE_TRACKING_DLL_H_
