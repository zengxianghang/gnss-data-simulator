#include "model/code_correlation.h"

#include <algorithm>
#include <cmath>

namespace gnss_sim {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;
constexpr int kMaxBocMultiple = 64;
constexpr int kMaxCorrelationSegments = 2 * kMaxBocMultiple;
constexpr double kIntegerRatioTolerance = 1.0e-10;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

int gcd_int(int lhs, int rhs) {
    while (rhs != 0) {
        const int remainder = lhs % rhs;
        lhs = rhs;
        rhs = remainder;
    }
    return lhs;
}

bool boc_multiple(double subcarrier_rate_hz, double chip_rate_hz, int* multiple) {
    if (multiple == nullptr || !std::isfinite(subcarrier_rate_hz) || !std::isfinite(chip_rate_hz) ||
        subcarrier_rate_hz <= 0.0 || chip_rate_hz <= 0.0) {
        return false;
    }
    const double ratio = subcarrier_rate_hz / chip_rate_hz;
    const double rounded = std::round(ratio);
    if (rounded < 1.0 || rounded > static_cast<double>(kMaxBocMultiple) ||
        std::abs(ratio - rounded) > kIntegerRatioTolerance) {
        return false;
    }
    *multiple = static_cast<int>(rounded);
    return true;
}

bool boc_subcarrier_value(int multiple, int segment_index, int segment_count, double* value) {
    if (value == nullptr || multiple <= 0 || segment_count <= 0 || segment_index < 0 ||
        segment_index >= segment_count || segment_count % (2 * multiple) != 0) {
        return false;
    }
    const int segments_per_half_cycle = segment_count / (2 * multiple);
    const int half_cycle_index = segment_index / segments_per_half_cycle;
    *value = (half_cycle_index % 2 == 0) ? 1.0 : -1.0;
    return true;
}

std::complex<double> step_waveform_autocorrelation(const std::complex<double>* waveform, int segment_count,
                                                   double delay_chips) {
    if (delay_chips < 0.0) {
        return std::conj(step_waveform_autocorrelation(waveform, segment_count, -delay_chips));
    }
    if (delay_chips >= 1.0) {
        return {0.0, 0.0};
    }

    double energy = 0.0;
    for (int index = 0; index < segment_count; ++index) {
        energy += std::norm(waveform[index]);
    }
    energy /= static_cast<double>(segment_count);
    if (!(energy > 0.0) || !std::isfinite(energy)) {
        return {0.0, 0.0};
    }

    std::complex<double> sum{0.0, 0.0};
    const double inverse_segment_count = 1.0 / static_cast<double>(segment_count);
    for (int first = 0; first < segment_count; ++first) {
        const double first_start = static_cast<double>(first) * inverse_segment_count;
        const double first_end = static_cast<double>(first + 1) * inverse_segment_count;
        for (int second = 0; second < segment_count; ++second) {
            const double second_start = delay_chips + static_cast<double>(second) * inverse_segment_count;
            const double second_end = delay_chips + static_cast<double>(second + 1) * inverse_segment_count;
            const double overlap_start = std::max(first_start, second_start);
            const double overlap_end = std::min(first_end, second_end);
            if (overlap_end > overlap_start) {
                sum += waveform[first] * std::conj(waveform[second]) * (overlap_end - overlap_start);
            }
        }
    }
    return sum / energy;
}

bool boc_autocorrelation(int multiple, double delay_chips, std::complex<double>* correlation) {
    if (correlation == nullptr || multiple <= 0 || multiple > kMaxBocMultiple) {
        return false;
    }
    const int segment_count = 2 * multiple;
    std::complex<double> waveform[kMaxCorrelationSegments]{};
    for (int segment = 0; segment < segment_count; ++segment) {
        double value = 0.0;
        if (!boc_subcarrier_value(multiple, segment, segment_count, &value)) {
            return false;
        }
        waveform[segment] = {value, 0.0};
    }
    *correlation = step_waveform_autocorrelation(waveform, segment_count, delay_chips);
    return true;
}

bool composite_phase_factor(CompositeSubcarrierPhase phase, std::complex<double>* factor) {
    if (factor == nullptr) {
        return false;
    }
    switch (phase) {
        case CompositeSubcarrierPhase::kInPhase:
            *factor = {1.0, 0.0};
            return true;
        case CompositeSubcarrierPhase::kAntiPhase:
            *factor = {-1.0, 0.0};
            return true;
        case CompositeSubcarrierPhase::kPositiveQuadrature:
            *factor = {0.0, 1.0};
            return true;
        case CompositeSubcarrierPhase::kNegativeQuadrature:
            *factor = {0.0, -1.0};
            return true;
        case CompositeSubcarrierPhase::kNotApplicable:
            break;
    }
    return false;
}

bool composite_boc_autocorrelation(const CodeCorrelationProfile& profile, double delay_chips,
                                   std::complex<double>* correlation) {
    if (correlation == nullptr) {
        return false;
    }

    int primary_multiple = 0;
    int secondary_multiple = 0;
    if (!boc_multiple(profile.primary_subcarrier_rate_hz, profile.chip_rate_hz, &primary_multiple) ||
        !boc_multiple(profile.secondary_subcarrier_rate_hz, profile.chip_rate_hz, &secondary_multiple)) {
        return false;
    }

    const int common_multiple = primary_multiple / gcd_int(primary_multiple, secondary_multiple) * secondary_multiple;
    if (common_multiple <= 0 || common_multiple > kMaxBocMultiple) {
        return false;
    }
    const int segment_count = 2 * common_multiple;

    std::complex<double> secondary_factor{};
    if (!composite_phase_factor(profile.secondary_phase, &secondary_factor)) {
        return false;
    }

    const double primary_weight = std::sqrt(1.0 - profile.secondary_power_fraction);
    const double secondary_weight = std::sqrt(profile.secondary_power_fraction);
    std::complex<double> waveform[kMaxCorrelationSegments]{};
    for (int segment = 0; segment < segment_count; ++segment) {
        double primary = 0.0;
        double secondary = 0.0;
        if (!boc_subcarrier_value(primary_multiple, segment, segment_count, &primary) ||
            !boc_subcarrier_value(secondary_multiple, segment, segment_count, &secondary)) {
            return false;
        }
        waveform[segment] = primary_weight * primary + secondary_factor * secondary_weight * secondary;
    }

    *correlation = step_waveform_autocorrelation(waveform, segment_count, delay_chips);
    return true;
}

bool tmboc_autocorrelation(const CodeCorrelationProfile& profile, double delay_chips,
                           std::complex<double>* correlation) {
    if (correlation == nullptr) {
        return false;
    }
    int primary_multiple = 0;
    int secondary_multiple = 0;
    if (!boc_multiple(profile.primary_subcarrier_rate_hz, profile.chip_rate_hz, &primary_multiple) ||
        !boc_multiple(profile.secondary_subcarrier_rate_hz, profile.chip_rate_hz, &secondary_multiple)) {
        return false;
    }

    std::complex<double> primary{};
    std::complex<double> secondary{};
    if (!boc_autocorrelation(primary_multiple, delay_chips, &primary) ||
        !boc_autocorrelation(secondary_multiple, delay_chips, &secondary)) {
        return false;
    }
    *correlation = (1.0 - profile.secondary_power_fraction) * primary +
                   profile.secondary_power_fraction * secondary;
    return true;
}

} // namespace

bool ideal_code_correlation_chips(const CodeCorrelationProfile& profile, double delay_chips,
                                  std::complex<double>* correlation, std::string* error_message) {
    if (correlation == nullptr) {
        set_error(error_message, "ideal code correlation output pointer is null");
        return false;
    }
    *correlation = {0.0, 0.0};
    if (!std::isfinite(delay_chips)) {
        set_error(error_message, "ideal code correlation delay is not finite");
        return false;
    }
    if (!validate_code_correlation_profile(profile, error_message)) {
        return false;
    }
    if (profile.model == CodeCorrelationModel::kUnsupported) {
        set_error(error_message, "signal code-correlation profile is explicitly unsupported");
        return false;
    }

    switch (profile.model) {
        case CodeCorrelationModel::kBpsk: {
            const double magnitude = std::abs(delay_chips);
            *correlation = {(magnitude < 1.0) ? 1.0 - magnitude : 0.0, 0.0};
            return true;
        }
        case CodeCorrelationModel::kTmboc:
            if (tmboc_autocorrelation(profile, delay_chips, correlation)) {
                return true;
            }
            set_error(error_message, "TMBOC subcarrier rates are not supported integer BOC(n,1) multiples");
            return false;
        case CodeCorrelationModel::kCboc:
        case CodeCorrelationModel::kQmboc:
            if (composite_boc_autocorrelation(profile, delay_chips, correlation)) {
                return true;
            }
            set_error(error_message, "composite BOC subcarrier rates are not supported integer BOC(n,1) multiples");
            return false;
        case CodeCorrelationModel::kUnsupported:
            break;
    }

    set_error(error_message, "unknown ideal code-correlation model");
    return false;
}

bool ideal_signal_code_correlation(const SignalDefinition& definition, double delay_seconds,
                                   std::complex<double>* correlation, std::string* error_message) {
    if (!std::isfinite(delay_seconds)) {
        set_error(error_message, "ideal signal correlation delay is not finite");
        return false;
    }
    if (!signal_has_supported_code_correlation(definition)) {
        set_error(error_message, "signal has no supported ideal code-correlation profile");
        if (correlation != nullptr) {
            *correlation = {0.0, 0.0};
        }
        return false;
    }
    return ideal_code_correlation_chips(definition.code_correlation,
                                        delay_seconds * definition.code_correlation.chip_rate_hz, correlation,
                                        error_message);
}

bool signal_code_chip_duration_s(const SignalDefinition& definition, double* chip_duration_s,
                                 std::string* error_message) {
    if (chip_duration_s == nullptr) {
        set_error(error_message, "code chip-duration output pointer is null");
        return false;
    }
    *chip_duration_s = 0.0;
    if (!signal_has_supported_code_correlation(definition)) {
        set_error(error_message, "signal has no supported code chip-rate metadata");
        return false;
    }
    *chip_duration_s = 1.0 / definition.code_correlation.chip_rate_hz;
    return true;
}

bool signal_code_chip_length_m(const SignalDefinition& definition, double* chip_length_m,
                               std::string* error_message) {
    if (chip_length_m == nullptr) {
        set_error(error_message, "code chip-length output pointer is null");
        return false;
    }
    *chip_length_m = 0.0;
    double duration_s = 0.0;
    if (!signal_code_chip_duration_s(definition, &duration_s, error_message)) {
        return false;
    }
    *chip_length_m = kSpeedOfLightMps * duration_s;
    return true;
}

} // namespace gnss_sim
