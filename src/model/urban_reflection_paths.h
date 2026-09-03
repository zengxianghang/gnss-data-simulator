#ifndef GNSS_SIM_SRC_MODEL_URBAN_REFLECTION_PATHS_H_
#define GNSS_SIM_SRC_MODEL_URBAN_REFLECTION_PATHS_H_

#include "gnss/satellite_engine.h"
#include "gnss/signal_definitions.h"
#include "model/receiver_truth.h"
#include "model/urban_rf_model.h"
#include "model/urban_scene_geometry.h"

#include <complex>
#include <string>

namespace gnss_sim {

constexpr int kUrbanFirstOrderWallCount = 4;

enum class UrbanReflectionCandidateStatus {
    VALID = 0,
    BACKSIDE,
    NO_SPECULAR_INTERSECTION,
    OUTSIDE_FACADE_HEIGHT,
    SOURCE_OCCLUDED,
    RECEIVER_OCCLUDED,
};

struct UrbanFirstOrderReflectionPath {
    UrbanWallId wall_id;
    EnuPoint satellite_enu_m;
    EnuPoint receiver_enu_m;
    EnuPoint reflection_point_enu_m;
    double incident_direction_enu[3];
    double reflected_direction_enu[3];
    double arrival_direction_enu[3];
    double incidence_angle_rad;
    double arrival_elevation_rad;
    double direct_euclidean_range_m;
    double reflected_euclidean_range_m;
    double direct_model_range_m;
    double model_path_range_m;
    double excess_path_length_m;
    double excess_delay_sec;
    double carrier_frequency_hz;
    double wavelength_m;
    double excess_carrier_phase_rad;
    std::complex<double> geometric_phase_factor;
    UrbanRfReflectionResponse rf_response;
    std::complex<double> antenna_rhcp_voltage;
    std::complex<double> antenna_lhcp_voltage;
};

struct UrbanFirstOrderReflectionSet {
    EnuPoint satellite_enu_m;
    EnuPoint receiver_enu_m;
    double direct_euclidean_range_m;
    double direct_model_range_m;
    UrbanReflectionCandidateStatus candidate_status[kUrbanFirstOrderWallCount];
    UrbanFirstOrderReflectionPath paths[kUrbanFirstOrderWallCount];
    int path_count;
};

bool reconstruct_urban_satellite_enu(const UrbanSceneGeometryConfig& scene_config, const ReceiverTruth& receiver,
                                     const SatelliteGeometry& satellite_geometry, EnuPoint* receiver_enu_m,
                                     EnuPoint* satellite_enu_m, double* direct_euclidean_range_m,
                                     std::string* error_message);

// A geometrically rejected wall is a normal result: the function returns true and
// reports the reason through status. false is reserved for malformed inputs or a
// numerical/model failure that should stop simulation.
bool compute_urban_one_wall_reflection(const UrbanSceneGeometryConfig& scene_config,
                                       const UrbanRfResolvedConfig& rf_config, UrbanWallId wall_id,
                                       const EnuPoint& satellite_enu_m, const EnuPoint& receiver_enu_m,
                                       double direct_model_range_m, double carrier_frequency_hz, double wavelength_m,
                                       UrbanFirstOrderReflectionPath* path, UrbanReflectionCandidateStatus* status,
                                       std::string* error_message);

bool compute_urban_first_order_reflections(const UrbanSceneGeometryConfig& scene_config,
                                           const UrbanRfConfig& rf_config, const SignalDefinition& signal,
                                           int glonass_fcn, const ReceiverTruth& receiver,
                                           const SatelliteGeometry& satellite_geometry,
                                           UrbanFirstOrderReflectionSet* reflections, std::string* error_message);

const char* urban_reflection_candidate_status_name(UrbanReflectionCandidateStatus status);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_URBAN_REFLECTION_PATHS_H_
