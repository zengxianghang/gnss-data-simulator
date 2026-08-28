#ifndef GNSS_SIM_SRC_GNSS_SATELLITE_ENGINE_H_
#define GNSS_SIM_SRC_GNSS_SATELLITE_ENGINE_H_

#include "gnss/rtklib_adapter.h"
#include "gnss_sim/sim_types.h"
#include "model/receiver_truth.h"

#include <string>

namespace gnss_sim {

struct SatelliteGeometry {
    SimTime receive_time;
    int transmit_gps_week;
    double transmit_sow_sec;
    int satellite_number;
    RtklibSatelliteState satellite_state;
    double line_of_sight_ecef[3];
    double geometric_range_m;
    double range_rate_mps;
    double propagation_time_sec;
    double azimuth_rad;
    double elevation_rad;
    int iteration_count;
    bool healthy;
    bool above_elevation_mask;
    bool visible;
};

using SatelliteStateProvider = bool (*)(const void* context, int gps_week, double sow_sec, int satellite_number,
                                        RtklibSatelliteState* state, std::string* error_message);

bool compute_satellite_geometry_with_provider(SatelliteStateProvider state_provider, const void* state_context,
                                              const ReceiverTruth& receiver, const SimTime& receive_time,
                                              int satellite_number, double elevation_mask_deg,
                                              SatelliteGeometry* geometry, std::string* error_message);

bool subtract_propagation_time(const SimTime& receive_time, double propagation_time_sec, int* transmit_gps_week,
                               double* transmit_sow_sec);
bool elevation_passes_mask(double elevation_rad, double elevation_mask_deg);
bool compute_satellite_geometry(const RtklibNavStore* nav_store, const ReceiverTruth& receiver,
                                const SimTime& receive_time, int satellite_number, double elevation_mask_deg,
                                SatelliteGeometry* geometry, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_GNSS_SATELLITE_ENGINE_H_
