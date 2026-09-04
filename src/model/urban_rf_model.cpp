#include "model/urban_rf_model.h"

#include <cmath>
#include <complex>

namespace gnss_sim {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kVacuumPermittivityFpm = 8.8541878128e-12;
constexpr double kFreeSpaceImpedanceOhm = 376.730313668;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kMinimumCosIncidence = 1.0e-12;

enum class LinearPolarization {
    kTe,
    kTm,
};

struct Matrix2 {
    std::complex<double> a;
    std::complex<double> b;
    std::complex<double> c;
    std::complex<double> d;
};

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool finite_complex(const std::complex<double>& value) {
    return std::isfinite(value.real()) && std::isfinite(value.imag());
}

bool finite_positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool finite_nonnegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool validate_polarization_response(const UrbanRfPolarizationResponseConfig& config) {
    return std::isfinite(config.gain_db_horizon) && std::isfinite(config.gain_db_zenith) &&
           std::isfinite(config.phase_deg_horizon) && std::isfinite(config.phase_deg_zenith);
}

Matrix2 multiply(const Matrix2& left, const Matrix2& right) {
    return {left.a * right.a + left.b * right.c, left.a * right.b + left.b * right.d,
            left.c * right.a + left.d * right.c, left.c * right.b + left.d * right.d};
}

std::complex<double> passive_forward_root(const std::complex<double>& value) {
    std::complex<double> root = std::sqrt(value);
    if (root.real() < 0.0 || (std::fabs(root.real()) <= 1.0e-15 && root.imag() > 0.0)) {
        root = -root;
    }
    return root;
}

bool layer_matrix(const std::complex<double>& relative_permittivity, double sin_incidence_squared, double thickness_m,
                  double k0_rad_per_m, LinearPolarization polarization, Matrix2* matrix) {
    if (matrix == nullptr) {
        return false;
    }
    const std::complex<double> q =
        passive_forward_root(relative_permittivity - std::complex<double>(sin_incidence_squared, 0.0));
    if (!finite_complex(q) || std::abs(q) <= 0.0) {
        return false;
    }
    const std::complex<double> admittance = polarization == LinearPolarization::kTe ? q : relative_permittivity / q;
    if (!finite_complex(admittance) || std::abs(admittance) <= 0.0) {
        return false;
    }
    const std::complex<double> delta = k0_rad_per_m * thickness_m * q;
    const std::complex<double> cosine = std::cos(delta);
    const std::complex<double> sine = std::sin(delta);
    const std::complex<double> j(0.0, 1.0);
    const Matrix2 result{cosine, j * sine / admittance, j * admittance * sine, cosine};
    if (!finite_complex(result.a) || !finite_complex(result.b) || !finite_complex(result.c) ||
        !finite_complex(result.d)) {
        return false;
    }
    *matrix = result;
    return true;
}

bool reflection_for_polarization(const UrbanRfMaterialConfig& config,
                                 const std::complex<double>& glass_relative_permittivity, double frequency_hz,
                                 double incidence_angle_rad, LinearPolarization polarization,
                                 std::complex<double>* reflection) {
    const double cosine_incidence = std::cos(incidence_angle_rad);
    if (reflection == nullptr || !std::isfinite(cosine_incidence) || cosine_incidence <= kMinimumCosIncidence) {
        return false;
    }
    const double sin_incidence = std::sin(incidence_angle_rad);
    const double sin_incidence_squared = sin_incidence * sin_incidence;
    const double k0_rad_per_m = 2.0 * kPi * frequency_hz / kSpeedOfLightMps;

    Matrix2 outer_glass{};
    Matrix2 cavity{};
    Matrix2 inner_glass{};
    if (!layer_matrix(glass_relative_permittivity, sin_incidence_squared, config.outer_glass_thickness_m, k0_rad_per_m,
                      polarization, &outer_glass) ||
        !layer_matrix({1.0, 0.0}, sin_incidence_squared, config.cavity_thickness_m, k0_rad_per_m, polarization,
                      &cavity) ||
        !layer_matrix(glass_relative_permittivity, sin_incidence_squared, config.inner_glass_thickness_m, k0_rad_per_m,
                      polarization, &inner_glass)) {
        return false;
    }

    const Matrix2 sheet{
        {1.0, 0.0}, {0.0, 0.0}, {kFreeSpaceImpedanceOhm / config.coating_sheet_resistance_ohm_sq, 0.0}, {1.0, 0.0}};
    const Matrix2 stack = multiply(multiply(multiply(outer_glass, sheet), cavity), inner_glass);
    const std::complex<double> q_air(cosine_incidence, 0.0);
    const std::complex<double> air_admittance =
        polarization == LinearPolarization::kTe ? q_air : std::complex<double>(1.0, 0.0) / q_air;
    const std::complex<double> denominator = stack.a + stack.b * air_admittance;
    if (!finite_complex(denominator) || std::abs(denominator) <= 0.0) {
        return false;
    }
    const std::complex<double> input_admittance = (stack.c + stack.d * air_admittance) / denominator;
    const std::complex<double> reflection_denominator = air_admittance + input_admittance;
    if (!finite_complex(input_admittance) || !finite_complex(reflection_denominator) ||
        std::abs(reflection_denominator) <= 0.0) {
        return false;
    }
    const std::complex<double> result = (air_admittance - input_admittance) / reflection_denominator;
    if (!finite_complex(result)) {
        return false;
    }
    *reflection = result;
    return true;
}

const UrbanRfPolarizationResponseConfig& polarization_config(const UrbanRfAntennaConfig& config,
                                                             UrbanCircularPolarization polarization) {
    return polarization == UrbanCircularPolarization::kRhcp ? config.rhcp : config.lhcp;
}

} // namespace

bool validate_urban_rf_material_config(const UrbanRfMaterialConfig& config, std::string* error_message) {
    if (!finite_positive(config.relative_permittivity_real) || !finite_nonnegative(config.conductivity_c_s_per_m) ||
        !finite_nonnegative(config.conductivity_exponent) || !finite_positive(config.outer_glass_thickness_m) ||
        !finite_positive(config.cavity_thickness_m) || !finite_positive(config.inner_glass_thickness_m) ||
        !finite_positive(config.coating_sheet_resistance_ohm_sq)) {
        set_error(error_message, "urban RF material configuration is invalid");
        return false;
    }
    return true;
}

bool validate_urban_rf_antenna_config(const UrbanRfAntennaConfig& config, std::string* error_message) {
    if (!validate_polarization_response(config.rhcp) || !validate_polarization_response(config.lhcp)) {
        set_error(error_message, "urban RF antenna configuration is invalid");
        return false;
    }
    return true;
}

bool resolve_urban_rf_signal_config(const UrbanRfConfig& config, const SignalDefinition& signal,
                                    UrbanRfResolvedConfig* resolved, std::string* error_message) {
    if (resolved == nullptr || signal.name == nullptr ||
        !validate_urban_rf_material_config(config.default_material, error_message) ||
        !validate_urban_rf_antenna_config(config.default_antenna, error_message)) {
        if (resolved == nullptr) {
            set_error(error_message, "urban RF resolved configuration output is null");
        }
        return false;
    }

    UrbanRfResolvedConfig result{config.default_material, config.default_antenna};
    for (const UrbanRfSignalOverrideConfig& override_config : config.signal_overrides) {
        if (override_config.signal_name == signal.name) {
            result.material = override_config.material;
            result.antenna = override_config.antenna;
            break;
        }
    }
    if (!validate_urban_rf_material_config(result.material, error_message) ||
        !validate_urban_rf_antenna_config(result.antenna, error_message)) {
        return false;
    }
    *resolved = result;
    return true;
}

bool compute_low_e_curtain_wall_reflection(const UrbanRfMaterialConfig& config, double frequency_hz,
                                           double incidence_angle_rad, UrbanRfReflectionResponse* response,
                                           std::string* error_message) {
    if (response == nullptr || !validate_urban_rf_material_config(config, error_message) ||
        !finite_positive(frequency_hz) || !std::isfinite(incidence_angle_rad) || incidence_angle_rad < 0.0 ||
        incidence_angle_rad >= 0.5 * kPi) {
        if (response == nullptr) {
            set_error(error_message, "urban RF reflection output is null");
        } else if (!finite_positive(frequency_hz)) {
            set_error(error_message, "urban RF carrier frequency must be finite and positive");
        } else if (!std::isfinite(incidence_angle_rad) || incidence_angle_rad < 0.0 ||
                   incidence_angle_rad >= 0.5 * kPi) {
            set_error(error_message, "urban RF incidence angle must be within [0, pi/2)");
        }
        return false;
    }

    const double frequency_ghz = frequency_hz / 1.0e9;
    const double conductivity = config.conductivity_c_s_per_m * std::pow(frequency_ghz, config.conductivity_exponent);
    if (!finite_nonnegative(conductivity)) {
        set_error(error_message, "urban RF glass conductivity calculation failed");
        return false;
    }
    const double omega = 2.0 * kPi * frequency_hz;
    const std::complex<double> relative_permittivity(config.relative_permittivity_real,
                                                     -conductivity / (omega * kVacuumPermittivityFpm));
    if (!finite_complex(relative_permittivity)) {
        set_error(error_message, "urban RF complex glass permittivity calculation failed");
        return false;
    }

    UrbanRfReflectionResponse result{};
    result.glass_conductivity_s_per_m = conductivity;
    result.glass_relative_permittivity = relative_permittivity;
    if (!reflection_for_polarization(config, relative_permittivity, frequency_hz, incidence_angle_rad,
                                     LinearPolarization::kTe, &result.gamma_te_tangent) ||
        !reflection_for_polarization(config, relative_permittivity, frequency_hz, incidence_angle_rad,
                                     LinearPolarization::kTm, &result.gamma_tm_tangent)) {
        set_error(error_message, "urban RF multilayer reflection calculation failed");
        return false;
    }

    const std::complex<double> gamma_p_basis = -result.gamma_tm_tangent;
    result.gamma_rhcp_from_rhcp = 0.5 * (result.gamma_te_tangent + gamma_p_basis);
    result.gamma_lhcp_from_rhcp = 0.5 * (result.gamma_te_tangent - gamma_p_basis);
    if (!finite_complex(result.gamma_rhcp_from_rhcp) || !finite_complex(result.gamma_lhcp_from_rhcp)) {
        set_error(error_message, "urban RF circular-polarization projection failed");
        return false;
    }
    *response = result;
    return true;
}

bool evaluate_urban_antenna_response(const UrbanRfAntennaConfig& config, UrbanCircularPolarization polarization,
                                     double arrival_elevation_rad, std::complex<double>* voltage_response,
                                     std::string* error_message) {
    if (voltage_response == nullptr || !validate_urban_rf_antenna_config(config, error_message) ||
        !std::isfinite(arrival_elevation_rad) || arrival_elevation_rad < 0.0 || arrival_elevation_rad > 0.5 * kPi) {
        if (voltage_response == nullptr) {
            set_error(error_message, "urban RF antenna response output is null");
        } else if (!std::isfinite(arrival_elevation_rad) || arrival_elevation_rad < 0.0 ||
                   arrival_elevation_rad > 0.5 * kPi) {
            set_error(error_message, "urban RF arrival elevation must be within [0, pi/2]");
        }
        return false;
    }

    const UrbanRfPolarizationResponseConfig& selected = polarization_config(config, polarization);
    const double fraction = arrival_elevation_rad / (0.5 * kPi);
    const double gain_db = selected.gain_db_horizon + fraction * (selected.gain_db_zenith - selected.gain_db_horizon);
    const double phase_deg =
        selected.phase_deg_horizon + fraction * (selected.phase_deg_zenith - selected.phase_deg_horizon);
    const double amplitude = std::pow(10.0, gain_db / 20.0);
    const std::complex<double> result = std::polar(amplitude, phase_deg * kDegreesToRadians);
    if (!finite_complex(result)) {
        set_error(error_message, "urban RF antenna interpolation failed");
        return false;
    }
    *voltage_response = result;
    return true;
}

} // namespace gnss_sim
