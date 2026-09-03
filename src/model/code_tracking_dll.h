#ifndef GNSS_SIM_SRC_MODEL_CODE_TRACKING_DLL_H_
#define GNSS_SIM_SRC_MODEL_CODE_TRACKING_DLL_H_

#include "gnss/signal_definitions.h"

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

// Status-preserving wrapper for propagation/tracking orchestration. A valid
// receiver-domain coherent null (all equal-delay complex-voltage groups cancel)
// is a physical no-root condition rather than a computation error. All other
// root enumeration is delegated unchanged to find_code_tracking_dll_roots().
bool find_code_tracking_dll_roots_with_status(const SignalDefinition& signal, const CodeTrackingDllPath* paths,
                                              int path_count, const CodeTrackingDllConfig& config,
                                              CodeTrackingDllRoot* roots, int root_capacity, int* root_count,
                                              CodeTrackingDllRootSearchStatus* status, std::string* error_message);

bool select_code_tracking_dll_root(const CodeTrackingDllRoot* roots, int root_count, CodeTrackingDllSelectionMode mode,
                                   double previous_code_phase_sec, int* selected_index, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_CODE_TRACKING_DLL_H_
