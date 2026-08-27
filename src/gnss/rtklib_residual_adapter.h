#ifndef GNSS_SIM_SRC_GNSS_RTKLIB_RESIDUAL_ADAPTER_H_
#define GNSS_SIM_SRC_GNSS_RTKLIB_RESIDUAL_ADAPTER_H_

#include "gnss/rtklib_adapter.h"

#include <string>

namespace gnss_sim {

struct RtklibSignalResidualResult {
    bool code_available;
    bool doppler_available;
    double code_residual_m;
    double doppler_residual_mps;
    double rtklib_code_bias_m;
    RtklibBroadcastMessageFamily selected_message_family;
    int selected_iode;
    double azimuth_rad;
    double elevation_rad;
};

bool rtklib_truth_state_signal_residuals(const RtklibNavStore* nav_store, int gps_week, double sow_sec,
                                         int satellite_number, int observation_code,
                                         RtklibBroadcastMessageFamily required_message_family,
                                         double pseudorange_m, double doppler_hz, double wavelength_m,
                                         const double receiver_ecef_m[3], const double receiver_velocity_ecef_mps[3],
                                         double receiver_clock_bias_m, double receiver_system_bias_m,
                                         double receiver_clock_drift_mps, double elevation_mask_deg,
                                         bool broadcast_atmosphere, RtklibSignalResidualResult* result,
                                         std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_GNSS_RTKLIB_RESIDUAL_ADAPTER_H_
