#ifndef GNSS_SIM_SRC_MODEL_URBAN_RF_MODEL_H_
#define GNSS_SIM_SRC_MODEL_URBAN_RF_MODEL_H_

#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"

#include <complex>
#include <string>

namespace gnss_sim {

enum class UrbanCircularPolarization {
    kRhcp,
    kLhcp,
};

struct UrbanRfResolvedConfig {
    UrbanRfMaterialConfig material;
    UrbanRfAntennaConfig antenna;
};

struct UrbanRfReflectionResponse {
    double glass_conductivity_s_per_m;
    std::complex<double> glass_relative_permittivity;
    std::complex<double> gamma_te_tangent;
    std::complex<double> gamma_tm_tangent;
    std::complex<double> gamma_rhcp_from_rhcp;
    std::complex<double> gamma_lhcp_from_rhcp;
};

bool validate_urban_rf_material_config(const UrbanRfMaterialConfig& config, std::string* error_message);
bool validate_urban_rf_antenna_config(const UrbanRfAntennaConfig& config, std::string* error_message);
bool resolve_urban_rf_signal_config(const UrbanRfConfig& config, const SignalDefinition& signal,
                                    UrbanRfResolvedConfig* resolved, std::string* error_message);
bool compute_low_e_curtain_wall_reflection(const UrbanRfMaterialConfig& config, double frequency_hz,
                                           double incidence_angle_rad, UrbanRfReflectionResponse* response,
                                           std::string* error_message);
bool evaluate_urban_antenna_response(const UrbanRfAntennaConfig& config, UrbanCircularPolarization polarization,
                                     double arrival_elevation_rad, std::complex<double>* voltage_response,
                                     std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_URBAN_RF_MODEL_H_
