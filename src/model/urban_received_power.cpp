#include "model/urban_received_power.h"

#include "model/urban_rf_model.h"

#include <cmath>
#include <limits>

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

bool reflection_normalized_voltage(const UrbanRfResolvedConfig& rf_config, const SatelliteGeometry& geometry,
                                   const UrbanFirstOrderReflectionPath& reflection,
                                   std::complex<double>* normalized_voltage, std::string* error_message) {
    if (normalized_voltage == nullptr || !std::isfinite(geometry.elevation_rad) ||
        !(reflection.direct_euclidean_range_m > 0.0) || !(reflection.reflected_euclidean_range_m > 0.0) ||
        !finite_complex(reflection.geometric_phase_factor) ||
        !finite_complex(reflection.rf_response.gamma_rhcp_from_rhcp) ||
        !finite_complex(reflection.rf_response.gamma_lhcp_from_rhcp) ||
        !finite_complex(reflection.antenna_rhcp_voltage) || !finite_complex(reflection.antenna_lhcp_voltage)) {
        set_error(error_message, "urban reflection received-voltage request is invalid");
        return false;
    }

    std::complex<double> direct_reference_antenna{};
    if (!evaluate_urban_antenna_response(rf_config.antenna, UrbanCircularPolarization::kRhcp,
                                         geometry.elevation_rad, &direct_reference_antenna, error_message)) {
        return false;
    }
    const double direct_reference_magnitude = std::abs(direct_reference_antenna);
    if (!(direct_reference_magnitude > 0.0) || !std::isfinite(direct_reference_magnitude)) {
        set_error(error_message, "urban direct RHCP antenna reference voltage is zero or non-finite");
        return false;
    }

    const std::complex<double> reflected_receiver_voltage =
        reflection.rf_response.gamma_rhcp_from_rhcp * reflection.antenna_rhcp_voltage +
        reflection.rf_response.gamma_lhcp_from_rhcp * reflection.antenna_lhcp_voltage;
    const double spreading_ratio = reflection.direct_euclidean_range_m / reflection.reflected_euclidean_range_m;
    const std::complex<double> result = spreading_ratio * reflected_receiver_voltage /
                                        direct_reference_antenna * reflection.geometric_phase_factor;
    if (!std::isfinite(spreading_ratio) || !(spreading_ratio > 0.0) || !finite_complex(result)) {
        set_error(error_message, "urban normalized reflection voltage is non-finite");
        return false;
    }
    *normalized_voltage = result;
    return true;
}

} // namespace

bool compute_urban_received_path_set(const Cn0Model& cn0_model, const UrbanSceneGeometryConfig& scene_config,
                                     const UrbanRfConfig& rf_config, const SignalDefinition& signal, int glonass_fcn,
                                     const SimTime& time, const ReceiverTruth& receiver,
                                     const SatelliteGeometry& satellite_geometry, UrbanReceivedPathSet* result,
                                     std::string* error_message) {
    if (result == nullptr || !std::isfinite(satellite_geometry.azimuth_rad) ||
        !std::isfinite(satellite_geometry.elevation_rad)) {
        set_error(error_message, "urban received-power request is invalid");
        return false;
    }

    UrbanReceivedPathSet output{};
    const double elevation_deg = satellite_geometry.elevation_rad * 180.0 / 3.141592653589793238462643383279502884;
    if (!cn0_model_estimate_dbhz(cn0_model, signal.signal_id, elevation_deg, time, &output.open_cn0_dbhz) ||
        !std::isfinite(output.open_cn0_dbhz)) {
        set_error(error_message, "cannot evaluate open-sky CN0 for urban received-power model");
        return false;
    }

    if (!compute_urban_direct_path_geometry(scene_config, satellite_geometry.azimuth_rad,
                                            satellite_geometry.elevation_rad, &output.direct_geometry,
                                            error_message) ||
        !compute_urban_rooftop_diffraction(scene_config, signal, glonass_fcn, receiver, satellite_geometry,
                                           &output.diffraction, &output.diffraction_status, error_message) ||
        !compute_urban_first_order_reflections(scene_config, rf_config, signal, glonass_fcn, receiver,
                                               satellite_geometry, &output.reflections, error_message)) {
        return false;
    }

    if (output.diffraction_status == UrbanRooftopDiffractionStatus::BELOW_LOCAL_HORIZON) {
        output.direct_voltage = {0.0, 0.0};
    } else if (output.diffraction_status == UrbanRooftopDiffractionStatus::VALID) {
        output.direct_voltage = output.diffraction.fresnel_coefficient;
        if (!finite_complex(output.direct_voltage)) {
            set_error(error_message, "urban rooftop direct transfer voltage is non-finite");
            return false;
        }
        const double direct_code_delay_sec =
            output.direct_geometry.line_of_sight ? 0.0 : output.diffraction.excess_delay_sec;
        output.paths[output.path_count++] = {direct_code_delay_sec, output.direct_voltage};
    } else {
        output.direct_voltage = {1.0, 0.0};
        output.paths[output.path_count++] = {0.0, output.direct_voltage};
    }

    UrbanRfResolvedConfig resolved_rf{};
    if (!resolve_urban_rf_signal_config(rf_config, signal, &resolved_rf, error_message)) {
        return false;
    }
    for (int index = 0; index < output.reflections.path_count; ++index) {
        if (output.path_count >= kMaxUrbanReceivedPaths) {
            set_error(error_message, "urban received path capacity exceeded");
            return false;
        }
        std::complex<double> reflection_voltage{};
        if (!reflection_normalized_voltage(resolved_rf, satellite_geometry, output.reflections.paths[index],
                                           &reflection_voltage, error_message)) {
            return false;
        }
        output.paths[output.path_count++] = {output.reflections.paths[index].excess_delay_sec, reflection_voltage};
    }

    if (output.path_count == 0) {
        set_error(error_message, "urban received-power model has no above-horizon propagation component");
        return false;
    }
    *result = output;
    return true;
}

bool compute_effective_cn0_from_paths(double open_cn0_dbhz, const SignalDefinition& signal,
                                      const CodeTrackingDllPath* paths, int path_count, double local_code_phase_sec,
                                      UrbanEffectiveCn0* result, std::string* error_message) {
    if (result == nullptr || !std::isfinite(open_cn0_dbhz) || !std::isfinite(local_code_phase_sec)) {
        set_error(error_message, "effective CN0 request is invalid");
        return false;
    }

    UrbanEffectiveCn0 output{};
    output.open_cn0_dbhz = open_cn0_dbhz;
    if (!compute_code_tracking_composite_correlation(signal, paths, path_count, local_code_phase_sec,
                                                     &output.composite_correlation, error_message)) {
        return false;
    }
    output.composite_power_ratio = std::norm(output.composite_correlation);
    if (!std::isfinite(output.composite_power_ratio) || output.composite_power_ratio < 0.0) {
        set_error(error_message, "effective CN0 composite power ratio is invalid");
        return false;
    }

    if (output.composite_power_ratio == 0.0) {
        output.carrier_to_noise_density_hz = 0.0;
        output.effective_cn0_dbhz = -std::numeric_limits<double>::infinity();
        output.finite_effective_cn0 = false;
        *result = output;
        return true;
    }

    const double open_linear = std::pow(10.0, 0.1 * open_cn0_dbhz);
    output.carrier_to_noise_density_hz = open_linear * output.composite_power_ratio;
    output.effective_cn0_dbhz = open_cn0_dbhz + 10.0 * std::log10(output.composite_power_ratio);
    output.finite_effective_cn0 = std::isfinite(output.carrier_to_noise_density_hz) &&
                                  output.carrier_to_noise_density_hz > 0.0 &&
                                  std::isfinite(output.effective_cn0_dbhz);
    if (!output.finite_effective_cn0) {
        set_error(error_message, "effective CN0 conversion is outside the supported numeric range");
        return false;
    }
    *result = output;
    return true;
}

bool compute_urban_effective_cn0(const SignalDefinition& signal, const UrbanReceivedPathSet& paths,
                                 double local_code_phase_sec, UrbanEffectiveCn0* result,
                                 std::string* error_message) {
    if (paths.path_count <= 0 || paths.path_count > kMaxUrbanReceivedPaths) {
        set_error(error_message, "urban effective CN0 path set is invalid");
        return false;
    }
    return compute_effective_cn0_from_paths(paths.open_cn0_dbhz, signal, paths.paths, paths.path_count,
                                            local_code_phase_sec, result, error_message);
}

} // namespace gnss_sim
