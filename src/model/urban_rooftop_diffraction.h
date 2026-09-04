#ifndef GNSS_SIM_SRC_MODEL_URBAN_ROOFTOP_DIFFRACTION_H_
#define GNSS_SIM_SRC_MODEL_URBAN_ROOFTOP_DIFFRACTION_H_

#include "gnss/satellite_engine.h"
#include "gnss/signal_definitions.h"
#include "model/receiver_truth.h"
#include "model/urban_scene_geometry.h"

#include <complex>
#include <string>

namespace gnss_sim {

enum class UrbanRooftopDiffractionStatus {
    VALID = 0,
    BELOW_LOCAL_HORIZON,
    NO_BLOCKING_ROOF_EDGE,
};

struct UrbanRooftopDiffractionPath {
    UrbanWallId wall_id;
    EnuPoint satellite_enu_m;
    EnuPoint receiver_enu_m;
    EnuPoint diffraction_point_enu_m;
    double direct_euclidean_range_m;
    double edge_path_euclidean_range_m;
    double direct_model_range_m;
    double model_path_range_m;
    double excess_path_length_m;
    double excess_delay_sec;
    double source_edge_distance_m;
    double receiver_edge_distance_m;
    double signed_clearance_m;
    double fresnel_v;
    double carrier_frequency_hz;
    double wavelength_m;
    double excess_carrier_phase_rad;
    std::complex<double> edge_geometric_phase_factor;
    std::complex<double> fresnel_coefficient;
    std::complex<double> edge_reference_coefficient;
};

// Complex scalar knife-edge field coefficient referenced to the unobstructed
// direct field. With the repository exp(+j*w*t), exp(-j*k*L) convention:
//   v -> -infinity : coefficient -> 1
//   v = 0           : coefficient = 0.5  (~6.02 dB loss)
//   v -> +infinity : coefficient -> 0
bool compute_complex_knife_edge_coefficient(double fresnel_v, std::complex<double>* coefficient,
                                            std::string* error_message);

// Compute the roof-affected direct component for the actual first blocking
// wall selected by #115 geometry. A non-applicable edge is reported through
// status and is not a model failure.
bool compute_urban_rooftop_diffraction(const UrbanSceneGeometryConfig& scene_config, const SignalDefinition& signal,
                                       int glonass_fcn, const ReceiverTruth& receiver,
                                       const SatelliteGeometry& satellite_geometry,
                                       UrbanRooftopDiffractionPath* diffraction, UrbanRooftopDiffractionStatus* status,
                                       std::string* error_message);

const char* urban_rooftop_diffraction_status_name(UrbanRooftopDiffractionStatus status);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_URBAN_ROOFTOP_DIFFRACTION_H_
