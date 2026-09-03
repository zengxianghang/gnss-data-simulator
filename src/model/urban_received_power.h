#ifndef GNSS_SIM_SRC_MODEL_URBAN_RECEIVED_POWER_H_
#define GNSS_SIM_SRC_MODEL_URBAN_RECEIVED_POWER_H_

#include "model/cn0_model.h"
#include "model/code_tracking_dll.h"
#include "model/receiver_truth.h"
#include "model/urban_reflection_paths.h"
#include "model/urban_rooftop_diffraction.h"

#include <complex>
#include <string>

namespace gnss_sim {

constexpr int kMaxUrbanReceivedPaths = 1 + kUrbanFirstOrderWallCount;

struct UrbanReceivedPathSet {
    double open_cn0_dbhz;
    std::complex<double> direct_voltage;
    UrbanRooftopDiffractionStatus diffraction_status;
    UrbanRooftopDiffractionPath diffraction;
    UrbanFirstOrderReflectionSet reflections;
    CodeTrackingDllPath paths[kMaxUrbanReceivedPaths];
    int path_count;
};

struct UrbanEffectiveCn0 {
    double open_cn0_dbhz;
    std::complex<double> composite_correlation;
    double composite_power_ratio;
    double carrier_to_noise_density_hz;
    double effective_cn0_dbhz;
    bool finite_effective_cn0;
};

// Build one common receiver-domain normalized path set. CN0_open already owns
// the nominal direct receiver/antenna response. Urban path voltages are
// dimensionless relative to that unobstructed direct reference voltage.
bool compute_urban_received_path_set(const Cn0Model& cn0_model, const UrbanSceneGeometryConfig& scene_config,
                                     const UrbanRfConfig& rf_config, const SignalDefinition& signal, int glonass_fcn,
                                     const SimTime& time, const ReceiverTruth& receiver,
                                     const SatelliteGeometry& satellite_geometry, UrbanReceivedPathSet* result,
                                     std::string* error_message);

// Convert an arbitrary normalized #119 path set into effective correlator C/N0
// at the supplied local code phase. Zero composite power is represented by
// finite_effective_cn0=false and effective_cn0_dbhz=-infinity.
bool compute_effective_cn0_from_paths(double open_cn0_dbhz, const SignalDefinition& signal,
                                      const CodeTrackingDllPath* paths, int path_count, double local_code_phase_sec,
                                      UrbanEffectiveCn0* result, std::string* error_message);

bool compute_urban_effective_cn0(const SignalDefinition& signal, const UrbanReceivedPathSet& paths,
                                 double local_code_phase_sec, UrbanEffectiveCn0* result,
                                 std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_URBAN_RECEIVED_POWER_H_
