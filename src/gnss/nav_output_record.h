#ifndef GNSS_SIM_SRC_GNSS_NAV_OUTPUT_RECORD_H_
#define GNSS_SIM_SRC_GNSS_NAV_OUTPUT_RECORD_H_

#include "gnss/rtklib_adapter.h"

#include <string>

namespace gnss_sim {

enum class NavOutputSystem {
    kUnknown,
    kGps,
    kGlonass,
    kGalileo,
    kBeidou,
    kQzss,
    kNavic,
};

struct KeplerianNavOutputData {
    NavOutputSystem system;
    RtklibBroadcastMessageFamily message_family;
    int satellite_number;
    int prn;
    int message_type;
    int iode;
    int iodc;
    double sva;
    int svh;
    int code;
    int flag;
    int toe_week;
    int toc_week;
    int transmit_week;
    double toe_sow_sec;
    double toc_sow_sec;
    double transmit_sow_sec;
    double semi_major_axis_m;
    double eccentricity;
    double inclination_rad;
    double omega0_rad;
    double argument_of_perigee_rad;
    double mean_anomaly_rad;
    double delta_mean_motion_radps;
    double corrected_mean_motion_radps;
    double omega_dot_radps;
    double inclination_dot_radps;
    double crc_m;
    double crs_m;
    double cuc_rad;
    double cus_rad;
    double cic_rad;
    double cis_rad;
    double clock_bias_sec;
    double clock_drift_sec_per_sec;
    double clock_drift_rate_sec_per_sec2;
    double tgd_sec[4];
    double isc_sec[6];
    double fit_hours;
    bool galileo_fnav_received;
    bool galileo_inav_received;
    double galileo_fnav_toc_sow_sec;
    double galileo_fnav_clock[3];
    double galileo_inav_toc_sow_sec;
    double galileo_inav_clock[3];
};

struct GlonassNavOutputData {
    int satellite_number;
    int prn;
    int slot_offset;
    int frequency_channel;
    int frequency_offset;
    int iode;
    int svh;
    int sva;
    int age_days;
    int flags;
    int toe_week;
    int frame_week;
    int gps_glonass_time_offset_sec;
    int calendar_day_number;
    double toe_sow_sec;
    double frame_sow_sec;
    double frame_time_glonass_day_sec;
    double position_ecef_m[3];
    double velocity_ecef_mps[3];
    double acceleration_ecef_mps2[3];
    double clock_bias_sec;
    double relative_frequency_bias;
    double differential_delay_sec;
};

struct IonosphereNavOutputData {
    NavOutputSystem system;
    int prn;
    int message_type;
    int transmit_week;
    double transmit_sow_sec;
    double coefficients[9];
    int coefficient_count;
    double region;
    double utc[4];
    int leap_seconds;
    bool legacy_metadata;
};

struct NavOutputRecord {
    RtklibNavRecordKind kind;
    KeplerianNavOutputData ephemeris;
    GlonassNavOutputData glonass;
    IonosphereNavOutputData ionosphere;
};

int rtklib_nav_output_record_count(const RtklibNavStore* store);
bool rtklib_nav_output_record(const RtklibNavStore* store, int output_record_index, NavOutputRecord* record,
                              std::string* error_message);

const char* nav_output_system_name(NavOutputSystem system);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_GNSS_NAV_OUTPUT_RECORD_H_
