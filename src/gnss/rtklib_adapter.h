#ifndef GNSS_SIM_SRC_GNSS_RTKLIB_ADAPTER_H_
#define GNSS_SIM_SRC_GNSS_RTKLIB_ADAPTER_H_

#include <cstdint>
#include <string>

namespace gnss_sim {

struct RtklibNavStore;
struct NavOutputRecord;

enum class RtklibNavRecordKind {
    kEphemeris,
    kGlonassEphemeris,
    kIonosphere,
};

enum class RtklibIonosphereSystem {
    kGps,
    kQzss,
    kBeidouLegacy,
    kGalileo,
    kGlonass,
    kBeidouModern,
};

enum class RtklibIonosphereStatus {
    kApplied,
    kMissingParameters,
    kUnsupportedModel,
};

enum class RtklibBroadcastMessageFamily {
    kUnknown,
    kLegacy,
    kCnav,
    kCnav2,
    kGalileoInav,
    kGalileoFnav,
    kBeidouBcnav1,
    kBeidouBcnav2,
    kBeidouBcnav3,
    kGlonassFdma,
    kGlonassL3Oc,
};

struct RtklibNavCounts {
    int gps_eph_count;
    int glo_eph_count;
    int gal_eph_count;
    int bds_eph_count;
    int qzss_eph_count;
    int other_eph_count;
};

struct RtklibNavRecordInfo {
    RtklibNavRecordKind kind;
    int satellite_number;
    int system;
    int prn;
    int message_type;
    int iode;
    int iodc;
    int gps_week;
    double transmit_sow_sec;
    double toe_sow_sec;
};

struct RtklibSatelliteState {
    double position_ecef_m[3];
    double velocity_ecef_mps[3];
    double clock_bias_sec;
    double clock_drift_sec_per_sec;
    double variance_m2;
    int health;
};

// Identity metadata of the broadcast ephemeris record that the family-restricted
// RTKLIB selection actually chose for a satellite-state calculation. Week/SOW values
// use the GPST convention of the projected records. GLONASS records carry no separate
// IODC/Toc: IODC is reported as 0 and Toc mirrors Toe.
struct RtklibSelectedEphemerisInfo {
    int satellite_number;
    RtklibBroadcastMessageFamily message_family;
    int iode;
    int iodc;
    int toe_week;
    double toe_sow_sec;
    int toc_week;
    double toc_sow_sec;
    int transmit_week;
    double transmit_sow_sec;
};

struct RtklibIonosphereResult {
    RtklibIonosphereStatus status;
    double reference_delay_m;
    double reference_frequency_hz;
};

// Read-only snapshot of the broadcast ionosphere model state stored in the RTKLIB
// navigation store (8 Klobuchar-style coefficients plus the GPST-UTC leap seconds).
// Used to verify correction values inside the final reconstructed store without
// exposing or mutating private nav_t internals.
struct RtklibIonosphereModelState {
    double coefficients[8];
    int leap_seconds;
};

struct RtklibBroadcastBiasData {
    RtklibBroadcastMessageFamily message_family;
    int system;
    int iode;
    int glonass_fcn;
    double tgd_sec[4];
    double isc_sec[6];
    double glonass_dtaun_sec;
    double glonass_isc_l3ocp_sec;
};

struct RtklibSolutionObservation {
    int satellite_number;
    int observation_code;
    RtklibBroadcastMessageFamily message_family;
    double pseudorange_m;
    double code_bias_m;
    double doppler_hz;
    double wavelength_m;
    double cn0_dbhz;
    bool pseudorange_valid;
    bool doppler_valid;
};

// Raw code observation used at serialization boundaries. Unlike
// RtklibSolutionObservation this contains no simulator-owned code-bias term;
// the serialized pseudorange is passed to RTKLIB unchanged and RTKLIB applies
// its own signal-specific broadcast code-bias correction inside pntpos().
struct RtklibRawCodeObservation {
    int satellite_number;
    int observation_code;
    RtklibBroadcastMessageFamily message_family;
    double pseudorange_m;
    double cn0_dbhz;
    bool pseudorange_valid;
};

struct RtklibPositionSolution {
    bool valid;
    double position_ecef_m[3];
    double latitude_deg;
    double longitude_deg;
    double height_m;
    double receiver_clock_bias_m;
    double covariance_ecef_m2[6];
    int used_satellites;
    // Exact set of satellites RTKLIB used in the solution, as a bit mask indexed by
    // (RTKLIB satellite number - 1). Seven 64-bit words cover RTKLIB MAXSAT (408);
    // unused satellites have cleared bits. Zero when the solution is invalid.
    std::uint64_t used_satellite_mask[7];
    char diagnostic[128];
};

struct RtklibVelocitySolution {
    bool valid;
    double velocity_ecef_mps[3];
    double receiver_clock_drift_mps;
    int used_satellites;
    char diagnostic[128];
};

RtklibNavStore* create_rtklib_nav_store();
void destroy_rtklib_nav_store(RtklibNavStore* store);

bool load_rinex_nav_file(RtklibNavStore* store, const char* file_path, std::string* error_message);
bool rtklib_append_nav_output_record(RtklibNavStore* store, const NavOutputRecord& record, std::string* error_message);
bool get_rtklib_nav_counts(const RtklibNavStore* store, RtklibNavCounts* counts);
int rtklib_nav_record_count(const RtklibNavStore* store);
bool rtklib_nav_record_info(const RtklibNavStore* store, int record_index, RtklibNavRecordInfo* info);
bool rtklib_clear_nav_store(RtklibNavStore* store);
bool rtklib_copy_nav_snapshot(const RtklibNavStore* source, int gps_week, double sow_sec, RtklibNavStore* destination,
                              std::string* error_message);
bool rtklib_copy_nav_record(const RtklibNavStore* source, int record_index, RtklibNavStore* destination,
                            std::string* error_message);
bool rtklib_nav_store_has_satellite_ephemeris(const RtklibNavStore* store, int satellite_number);

bool rtklib_satellite_state_available(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number);
bool rtklib_satellite_id_to_number(const char* satellite_id, int* satellite_number);
bool rtklib_satellite_number_to_id(int satellite_number, char satellite_id[4]);
bool rtklib_observation_code(const char* rinex_signal_code, int* observation_code, int* frequency_index);
bool rtklib_signal_health_for_family(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,
                                     const char* rinex_signal_code,
                                     RtklibBroadcastMessageFamily requested_message_family, int* signal_health,
                                     std::string* error_message);
// Evaluate the satellite state at gps_week/sow_sec while keeping RTKLIB's
// broadcast-ephemeris selection epoch fixed at selection_gps_week/selection_sow_sec.
// This mirrors RTKLIB satposs(): transmission time is used for state evaluation,
// while the observation epoch (teph) is used for ephemeris selection.
bool get_rtklib_satellite_state_with_selection_time(const RtklibNavStore* store, int gps_week, double sow_sec,
                                                    int selection_gps_week, double selection_sow_sec,
                                                    int satellite_number, RtklibSatelliteState* state,
                                                    std::string* error_message);
bool get_rtklib_satellite_state(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,
                                RtklibSatelliteState* state, std::string* error_message);
bool get_rtklib_signal_satellite_state(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,
                                       int observation_code, RtklibBroadcastMessageFamily requested_message_family,
                                       RtklibSatelliteState* state, std::string* error_message,
                                       RtklibSelectedEphemerisInfo* selected_identity = nullptr);
bool rtklib_broadcast_bias_data_for_family(const RtklibNavStore* store, int gps_week, double sow_sec,
                                           int satellite_number, RtklibBroadcastMessageFamily requested_message_family,
                                           RtklibBroadcastBiasData* data, std::string* error_message,
                                           RtklibSelectedEphemerisInfo* selected_identity = nullptr);
bool rtklib_broadcast_bias_data(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,
                                RtklibBroadcastBiasData* data, std::string* error_message);

bool rtklib_solve_single_position(const RtklibNavStore* receiver_nav, int gps_week, double sow_sec,
                                  const RtklibSolutionObservation* observations, int observation_count,
                                  double elevation_mask_deg, bool broadcast_atmosphere,
                                  RtklibPositionSolution* solution, std::string* error_message);
bool rtklib_raw_code_observation_navigation_available(const RtklibNavStore* receiver_nav, int gps_week, double sow_sec,
                                                      const RtklibRawCodeObservation* observation, bool* available,
                                                      std::string* error_message);

bool rtklib_solve_raw_single_position(const RtklibNavStore* receiver_nav, int gps_week, double sow_sec,
                                      const RtklibRawCodeObservation* observations, int observation_count,
                                      double elevation_mask_deg, bool broadcast_atmosphere,
                                      RtklibPositionSolution* solution, std::string* error_message);
bool rtklib_solve_single_velocity(const RtklibNavStore* receiver_nav, int gps_week, double sow_sec,
                                  const RtklibSolutionObservation* observations, int observation_count,
                                  const double position_hint_ecef_m[3], double elevation_mask_deg,
                                  RtklibVelocitySolution* solution, std::string* error_message);

bool rtklib_llh_to_ecef(double latitude_deg, double longitude_deg, double height_m, double ecef_m[3]);
bool rtklib_ecef_to_llh(const double ecef_m[3], double* latitude_deg, double* longitude_deg, double* height_m);
bool rtklib_geometric_distance(const double satellite_ecef_m[3], const double receiver_ecef_m[3], double* range_m,
                               double line_of_sight_ecef[3]);
bool rtklib_azimuth_elevation(const double receiver_ecef_m[3], const double line_of_sight_ecef[3], double* azimuth_rad,
                              double* elevation_rad);

bool rtklib_broadcast_ionosphere_model_state(const RtklibNavStore* store, RtklibIonosphereSystem system,
                                             RtklibIonosphereModelState* state, std::string* error_message);
bool rtklib_broadcast_ionosphere_reference_delay(const RtklibNavStore* store, RtklibIonosphereSystem system,
                                                 int gps_week, double sow_sec, const double receiver_ecef_m[3],
                                                 double azimuth_rad, double elevation_rad,
                                                 RtklibIonosphereResult* result, std::string* error_message);
bool rtklib_troposphere_delay(int gps_week, double sow_sec, const double receiver_ecef_m[3], double azimuth_rad,
                              double elevation_rad, double* delay_m, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_GNSS_RTKLIB_ADAPTER_H_
