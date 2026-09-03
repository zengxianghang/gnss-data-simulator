#include "model/code_tracking_dll.h"

#include <cmath>
#include <complex>

namespace gnss_sim {
namespace {

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool finite_complex(const std::complex<double>& value) {
    return std::isfinite(value.real()) && std::isfinite(value.imag());
}

bool equal_delay_groups_cancel(const CodeTrackingDllPath* paths, int path_count) {
    if (paths == nullptr || path_count <= 0) {
        return false;
    }

    double total_absolute_voltage = 0.0;
    for (int index = 0; index < path_count; ++index) {
        if (!std::isfinite(paths[index].code_delay_sec) || !finite_complex(paths[index].complex_voltage)) {
            return false;
        }
        total_absolute_voltage += std::abs(paths[index].complex_voltage);
        if (!std::isfinite(total_absolute_voltage)) {
            return false;
        }
    }
    if (!(total_absolute_voltage > 0.0)) {
        return false;
    }

    for (int index = 0; index < path_count; ++index) {
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
        if (group_voltage != std::complex<double>{0.0, 0.0}) {
            return false;
        }
    }
    return true;
}

} // namespace

bool find_code_tracking_dll_roots_with_status(const SignalDefinition& signal, const CodeTrackingDllPath* paths,
                                              int path_count, const CodeTrackingDllConfig& config,
                                              CodeTrackingDllRoot* roots, int root_capacity, int* root_count,
                                              CodeTrackingDllRootSearchStatus* status, std::string* error_message) {
    if (roots == nullptr || root_count == nullptr || status == nullptr || root_capacity <= 0 ||
        root_capacity > kMaxCodeTrackingDllRoots) {
        set_error(error_message, "invalid code tracking DLL root-status output arguments");
        return false;
    }
    *root_count = 0;
    *status = CodeTrackingDllRootSearchStatus::kNoRoots;

    if (!validate_code_tracking_dll_config(config, error_message)) {
        return false;
    }

    // Reuse the public composite correlator as the authoritative validation of
    // signal correlation support and path numeric validity before recognizing
    // a coherent null as a physical no-root state.
    std::complex<double> validation_correlation{};
    if (!compute_code_tracking_composite_correlation(signal, paths, path_count, 0.0, &validation_correlation,
                                                     error_message)) {
        return false;
    }

    if (equal_delay_groups_cancel(paths, path_count)) {
        *status = CodeTrackingDllRootSearchStatus::kNoRoots;
        return true;
    }

    if (!find_code_tracking_dll_roots(signal, paths, path_count, config, roots, root_capacity, root_count,
                                      error_message)) {
        return false;
    }
    *status = CodeTrackingDllRootSearchStatus::kRootsFound;
    return true;
}

} // namespace gnss_sim
