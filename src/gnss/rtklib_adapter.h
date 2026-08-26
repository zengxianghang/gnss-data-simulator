#ifndef GNSS_SIM_SRC_GNSS_RTKLIB_ADAPTER_H_
#define GNSS_SIM_SRC_GNSS_RTKLIB_ADAPTER_H_

#include <string>

namespace gnss_sim {

struct RtklibNavStore;

struct RtklibNavCounts {
    int gps_eph_count;
    int glo_eph_count;
    int gal_eph_count;
    int bds_eph_count;
    int qzss_eph_count;
    int other_eph_count;
};

struct RtklibSatelliteState {
    double position_ecef_m[3];
    double velocity_ecef_mps[3];
    double clock_bias_sec;
    double clock_drift_sec_per_sec;
    double variance_m2;
    int health;
};

RtklibNavStore* create_rtklib_nav_store();
void destroy_rtklib_nav_store(RtklibNavStore* store);

bool load_rinex_nav_file(RtklibNavStore* store, const char* file_path, std::string* error_message);
bool get_rtklib_nav_counts(const RtklibNavStore* store, RtklibNavCounts* counts);

bool rtklib_satellite_id_to_number(const char* satellite_id, int* satellite_number);
bool get_rtklib_satellite_state(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,
                                RtklibSatelliteState* state, std::string* error_message);

bool rtklib_llh_to_ecef(double latitude_deg, double longitude_deg, double height_m, double ecef_m[3]);
bool rtklib_ecef_to_llh(const double ecef_m[3], double* latitude_deg, double* longitude_deg, double* height_m);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_GNSS_RTKLIB_ADAPTER_H_
