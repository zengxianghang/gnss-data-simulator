#ifndef GNSS_SIM_SIM_CONFIG_H_
#define GNSS_SIM_SIM_CONFIG_H_

#include "gnss_sim/atmosphere_types.h"

#include <cstdint>
#include <string>

namespace gnss_sim {

enum class ScenarioType {
    KS,
    REA,
    TTFF,
};

enum class StartupMode {
    HOT,
    WARM,
    COLD,
};

struct ReceiverConfig {
    double latitude_deg;
    double longitude_deg;
    double height_m;
};

struct TtffConfig {
    StartupMode startup_mode;
    std::int64_t power_on_ns;
    std::int64_t power_off_ns;
};

struct ReaConfig {
    std::int64_t signal_on_ns;
    std::int64_t signal_off_ns;
};

struct SimConfig {
    int schema_version;
    ScenarioType scenario;
    std::int64_t duration_ns;
    int sampling_rate_hz;
    double elevation_mask_deg;
    double solution_elevation_mask_deg;
    bool output_eph;
    bool output_ion;
    bool measurement_noise_enabled;
    bool multipath_enabled;
    double receiver_clock_bias_m;
    double receiver_clock_drift_mps;
    AtmosphereMode atmosphere_mode;
    ReceiverConfig receiver;
    TtffConfig ttff;
    ReaConfig rea;
    std::uint64_t seed;
};

SimConfig default_sim_config();
bool validate_sim_config(const SimConfig& config, std::string* error_message);
bool load_sim_config_json(const char* file_path, SimConfig* config, std::string* error_message);
const char* scenario_type_name(ScenarioType scenario);
const char* startup_mode_name(StartupMode startup_mode);
const char* atmosphere_mode_name(AtmosphereMode atmosphere_mode);

} // namespace gnss_sim

#endif // GNSS_SIM_SIM_CONFIG_H_
