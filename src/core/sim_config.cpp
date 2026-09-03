#include "gnss_sim/sim_config.h"

#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_time.h"

#include <cJSON.h>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>

namespace gnss_sim {
namespace {

constexpr double MAX_EXACT_JSON_INTEGER = 9007199254740991.0;

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool is_allowed_key(const char* key, const char* const* allowed_keys, std::size_t allowed_key_count) {
    for (std::size_t index = 0; index < allowed_key_count; ++index) {
        if (std::strcmp(key, allowed_keys[index]) == 0) {
            return true;
        }
    }
    return false;
}

bool validate_object_keys(const cJSON* object, const char* section_name, const char* const* allowed_keys,
                          std::size_t allowed_key_count, std::string* error_message) {
    if (!cJSON_IsObject(object)) {
        set_error(error_message, std::string(section_name) + " must be a JSON object");
        return false;
    }

    for (const cJSON* item = object->child; item != nullptr; item = item->next) {
        if (item->string == nullptr || !is_allowed_key(item->string, allowed_keys, allowed_key_count)) {
            set_error(error_message, std::string("unsupported key in ") + section_name + ": " +
                                         (item->string == nullptr ? "<null>" : item->string));
            return false;
        }
        for (const cJSON* previous = object->child; previous != item; previous = previous->next) {
            if (previous->string != nullptr && std::strcmp(previous->string, item->string) == 0) {
                set_error(error_message, std::string("duplicate key in ") + section_name + ": " + item->string);
                return false;
            }
        }
    }
    return true;
}

bool read_optional_number(const cJSON* object, const char* key, double* value, std::string* error_message) {
    const cJSON* item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (item == nullptr) {
        return true;
    }
    if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble)) {
        set_error(error_message, std::string(key) + " must be a finite JSON number");
        return false;
    }
    *value = item->valuedouble;
    return true;
}

bool read_optional_int(const cJSON* object, const char* key, int* value, std::string* error_message) {
    double number = static_cast<double>(*value);
    if (!read_optional_number(object, key, &number, error_message)) {
        return false;
    }
    if (cJSON_GetObjectItemCaseSensitive(object, key) == nullptr) {
        return true;
    }
    if (std::floor(number) != number || number < static_cast<double>(std::numeric_limits<int>::min()) ||
        number > static_cast<double>(std::numeric_limits<int>::max())) {
        set_error(error_message, std::string(key) + " must be an integer in range");
        return false;
    }
    *value = static_cast<int>(number);
    return true;
}

bool read_optional_bool(const cJSON* object, const char* key, bool* value, std::string* error_message) {
    const cJSON* item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (item == nullptr) {
        return true;
    }
    if (!cJSON_IsBool(item)) {
        set_error(error_message, std::string(key) + " must be a JSON boolean");
        return false;
    }
    *value = cJSON_IsTrue(item) != 0;
    return true;
}

bool read_optional_string(const cJSON* object, const char* key, const char** value, std::string* error_message) {
    const cJSON* item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (item == nullptr) {
        return true;
    }
    if (!cJSON_IsString(item) || item->valuestring == nullptr) {
        set_error(error_message, std::string(key) + " must be a JSON string");
        return false;
    }
    *value = item->valuestring;
    return true;
}

bool seconds_to_ns(double seconds, const char* field_name, std::int64_t* value_ns, std::string* error_message) {
    if (!std::isfinite(seconds) || seconds < 0.0) {
        set_error(error_message, std::string(field_name) + " must be finite and non-negative");
        return false;
    }

    const long double scaled = static_cast<long double>(seconds) * static_cast<long double>(NANOSECONDS_PER_SECOND);
    if (scaled > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
        set_error(error_message, std::string(field_name) + " is too large");
        return false;
    }
    *value_ns = static_cast<std::int64_t>(std::llround(scaled));
    return true;
}

bool parse_scenario(const char* value, ScenarioType* scenario, std::string* error_message) {
    if (std::strcmp(value, "KS") == 0) {
        *scenario = ScenarioType::KS;
        return true;
    }
    if (std::strcmp(value, "REA") == 0) {
        *scenario = ScenarioType::REA;
        return true;
    }
    if (std::strcmp(value, "TTFF") == 0) {
        *scenario = ScenarioType::TTFF;
        return true;
    }
    set_error(error_message, std::string("unsupported scenario: ") + value);
    return false;
}

bool parse_startup_mode(const char* value, StartupMode* startup_mode, std::string* error_message) {
    if (std::strcmp(value, "HOT") == 0) {
        *startup_mode = StartupMode::HOT;
        return true;
    }
    if (std::strcmp(value, "WARM") == 0) {
        *startup_mode = StartupMode::WARM;
        return true;
    }
    if (std::strcmp(value, "COLD") == 0) {
        *startup_mode = StartupMode::COLD;
        return true;
    }
    set_error(error_message, std::string("unsupported TTFF startup mode: ") + value);
    return false;
}

bool parse_atmosphere_mode(const char* value, AtmosphereMode* atmosphere_mode, std::string* error_message) {
    if (std::strcmp(value, "unspecified") == 0) {
        *atmosphere_mode = AtmosphereMode::UNSPECIFIED;
        return true;
    }
    if (std::strcmp(value, "none") == 0) {
        *atmosphere_mode = AtmosphereMode::NONE;
        return true;
    }
    if (std::strcmp(value, "broadcast") == 0) {
        *atmosphere_mode = AtmosphereMode::BROADCAST;
        return true;
    }
    set_error(error_message, std::string("unsupported atmosphere_mode: ") + value);
    return false;
}

const SignalDefinition* find_signal_definition_by_name(const char* name) {
    if (name == nullptr) {
        return nullptr;
    }
    std::size_t count = 0;
    const SignalDefinition* definitions = signal_definitions(&count);
    for (std::size_t index = 0; index < count; ++index) {
        if (std::strcmp(definitions[index].name, name) == 0) {
            return &definitions[index];
        }
    }
    return nullptr;
}

bool parse_receiver(const cJSON* root, SimConfig* config, std::string* error_message) {
    const cJSON* receiver = cJSON_GetObjectItemCaseSensitive(root, "receiver");
    if (receiver == nullptr) {
        return true;
    }
    const char* const allowed_keys[] = {"latitude_deg", "longitude_deg", "height_m"};
    if (!validate_object_keys(receiver, "receiver", allowed_keys, 3U, error_message)) {
        return false;
    }
    return read_optional_number(receiver, "latitude_deg", &config->receiver.latitude_deg, error_message) &&
           read_optional_number(receiver, "longitude_deg", &config->receiver.longitude_deg, error_message) &&
           read_optional_number(receiver, "height_m", &config->receiver.height_m, error_message);
}

bool parse_ttff(const cJSON* root, SimConfig* config, std::string* error_message) {
    const cJSON* ttff = cJSON_GetObjectItemCaseSensitive(root, "ttff");
    if (ttff == nullptr) {
        return true;
    }
    const char* const allowed_keys[] = {"startup_mode", "power_on_sec", "power_off_sec"};
    if (!validate_object_keys(ttff, "ttff", allowed_keys, 3U, error_message)) {
        return false;
    }

    const char* startup_mode = startup_mode_name(config->ttff.startup_mode);
    if (!read_optional_string(ttff, "startup_mode", &startup_mode, error_message) ||
        !parse_startup_mode(startup_mode, &config->ttff.startup_mode, error_message)) {
        return false;
    }

    double power_on_sec = static_cast<double>(config->ttff.power_on_ns) / static_cast<double>(NANOSECONDS_PER_SECOND);
    double power_off_sec = static_cast<double>(config->ttff.power_off_ns) / static_cast<double>(NANOSECONDS_PER_SECOND);
    return read_optional_number(ttff, "power_on_sec", &power_on_sec, error_message) &&
           read_optional_number(ttff, "power_off_sec", &power_off_sec, error_message) &&
           seconds_to_ns(power_on_sec, "ttff.power_on_sec", &config->ttff.power_on_ns, error_message) &&
           seconds_to_ns(power_off_sec, "ttff.power_off_sec", &config->ttff.power_off_ns, error_message);
}

bool parse_rea(const cJSON* root, SimConfig* config, std::string* error_message) {
    const cJSON* rea = cJSON_GetObjectItemCaseSensitive(root, "rea");
    if (rea == nullptr) {
        return true;
    }
    const char* const allowed_keys[] = {"signal_on_sec", "signal_off_sec"};
    if (!validate_object_keys(rea, "rea", allowed_keys, 2U, error_message)) {
        return false;
    }

    double signal_on_sec = static_cast<double>(config->rea.signal_on_ns) / static_cast<double>(NANOSECONDS_PER_SECOND);
    double signal_off_sec =
        static_cast<double>(config->rea.signal_off_ns) / static_cast<double>(NANOSECONDS_PER_SECOND);
    return read_optional_number(rea, "signal_on_sec", &signal_on_sec, error_message) &&
           read_optional_number(rea, "signal_off_sec", &signal_off_sec, error_message) &&
           seconds_to_ns(signal_on_sec, "rea.signal_on_sec", &config->rea.signal_on_ns, error_message) &&
           seconds_to_ns(signal_off_sec, "rea.signal_off_sec", &config->rea.signal_off_ns, error_message);
}

bool parse_bestpos_rtk(const cJSON* root, SimConfig* config, std::string* error_message) {
    const cJSON* object = cJSON_GetObjectItemCaseSensitive(root, "bestpos_rtk");
    if (object == nullptr) {
        return true;
    }
    const char* const allowed_keys[] = {"enabled", "stable_duration_sec", "min_used_satellites", "horizontal_std_m",
                                        "height_std_m"};
    if (!validate_object_keys(object, "bestpos_rtk", allowed_keys, 5U, error_message)) {
        return false;
    }
    double stable_duration_sec =
        static_cast<double>(config->bestpos_rtk.stable_duration_ns) / static_cast<double>(NANOSECONDS_PER_SECOND);
    return read_optional_bool(object, "enabled", &config->bestpos_rtk.enabled, error_message) &&
           read_optional_number(object, "stable_duration_sec", &stable_duration_sec, error_message) &&
           read_optional_int(object, "min_used_satellites", &config->bestpos_rtk.min_used_satellites, error_message) &&
           read_optional_number(object, "horizontal_std_m", &config->bestpos_rtk.horizontal_std_m, error_message) &&
           read_optional_number(object, "height_std_m", &config->bestpos_rtk.height_std_m, error_message) &&
           seconds_to_ns(stable_duration_sec, "bestpos_rtk.stable_duration_sec",
                         &config->bestpos_rtk.stable_duration_ns, error_message);
}

bool parse_measurement_transient(const cJSON* parent, const char* key, MeasurementTransientErrorConfig* config,
                                 std::string* error_message) {
    const cJSON* object = cJSON_GetObjectItemCaseSensitive(parent, key);
    if (object == nullptr) {
        return true;
    }
    const char* const allowed_keys[] = {"psr_extra_sigma_m", "doppler_extra_sigma_mps", "cn0_extra_sigma_dbhz",
                                        "decay_tau_sec"};
    const std::string section_name = std::string("measurement_error.") + key;
    if (!validate_object_keys(object, section_name.c_str(), allowed_keys, 4U, error_message)) {
        return false;
    }
    return read_optional_number(object, "psr_extra_sigma_m", &config->psr_extra_sigma_m, error_message) &&
           read_optional_number(object, "doppler_extra_sigma_mps", &config->doppler_extra_sigma_mps, error_message) &&
           read_optional_number(object, "cn0_extra_sigma_dbhz", &config->cn0_extra_sigma_dbhz, error_message) &&
           read_optional_number(object, "decay_tau_sec", &config->decay_tau_sec, error_message);
}

bool parse_measurement_fade(const cJSON* parent, MeasurementFadeErrorConfig* config, std::string* error_message) {
    const cJSON* object = cJSON_GetObjectItemCaseSensitive(parent, "rea_fade");
    if (object == nullptr) {
        return true;
    }
    const char* const allowed_keys[] = {"duration_sec", "psr_extra_sigma_m", "doppler_extra_sigma_mps", "cn0_drop_db"};
    if (!validate_object_keys(object, "measurement_error.rea_fade", allowed_keys, 4U, error_message)) {
        return false;
    }
    return read_optional_number(object, "duration_sec", &config->duration_sec, error_message) &&
           read_optional_number(object, "psr_extra_sigma_m", &config->psr_extra_sigma_m, error_message) &&
           read_optional_number(object, "doppler_extra_sigma_mps", &config->doppler_extra_sigma_mps, error_message) &&
           read_optional_number(object, "cn0_drop_db", &config->cn0_drop_db, error_message);
}

bool parse_measurement_error(const cJSON* root, SimConfig* config, std::string* error_message) {
    const cJSON* object = cJSON_GetObjectItemCaseSensitive(root, "measurement_error");
    if (object == nullptr) {
        return true;
    }
    const char* const allowed_keys[] = {
        "psr_sigma_m", "doppler_sigma_mps", "adr_sigma_m", "cn0_sigma_dbhz",    "psr_correlation_tau_sec",
        "ttff_hot",    "ttff_warm",         "ttff_cold",   "rea_reacquisition", "rea_fade"};
    if (!validate_object_keys(object, "measurement_error", allowed_keys, 10U, error_message)) {
        return false;
    }
    return read_optional_number(object, "psr_sigma_m", &config->measurement_error.psr_sigma_m, error_message) &&
           read_optional_number(object, "doppler_sigma_mps", &config->measurement_error.doppler_sigma_mps,
                                error_message) &&
           read_optional_number(object, "adr_sigma_m", &config->measurement_error.adr_sigma_m, error_message) &&
           read_optional_number(object, "cn0_sigma_dbhz", &config->measurement_error.cn0_sigma_dbhz, error_message) &&
           read_optional_number(object, "psr_correlation_tau_sec", &config->measurement_error.psr_correlation_tau_sec,
                                error_message) &&
           parse_measurement_transient(object, "ttff_hot", &config->measurement_error.ttff_hot, error_message) &&
           parse_measurement_transient(object, "ttff_warm", &config->measurement_error.ttff_warm, error_message) &&
           parse_measurement_transient(object, "ttff_cold", &config->measurement_error.ttff_cold, error_message) &&
           parse_measurement_transient(object, "rea_reacquisition", &config->measurement_error.rea_reacquisition,
                                       error_message) &&
           parse_measurement_fade(object, &config->measurement_error.rea_fade, error_message);
}

bool parse_cn0_high_dbhz(const cJSON* root, SimConfig* config, std::string* error_message) {
    const cJSON* object = cJSON_GetObjectItemCaseSensitive(root, "cn0_high_dbhz");
    if (object == nullptr) {
        return true;
    }
    if (!cJSON_IsObject(object)) {
        set_error(error_message, "cn0_high_dbhz must be a JSON object keyed by central signal name");
        return false;
    }

    config->cn0_high_dbhz.clear();
    for (const cJSON* item = object->child; item != nullptr; item = item->next) {
        if (item->string == nullptr || find_signal_definition_by_name(item->string) == nullptr) {
            set_error(error_message, std::string("cn0_high_dbhz contains unknown central signal name: ") +
                                         (item->string == nullptr ? "<null>" : item->string));
            return false;
        }
        if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble)) {
            set_error(error_message, std::string("cn0_high_dbhz value must be finite for signal: ") + item->string);
            return false;
        }
        for (const Cn0HighBaselineConfig& previous : config->cn0_high_dbhz) {
            if (previous.signal_name == item->string) {
                set_error(error_message, std::string("duplicate cn0_high_dbhz signal: ") + item->string);
                return false;
            }
        }
        config->cn0_high_dbhz.push_back({item->string, item->valuedouble});
    }
    return true;
}

bool parse_seed(const cJSON* root, SimConfig* config, std::string* error_message) {
    const cJSON* seed = cJSON_GetObjectItemCaseSensitive(root, "seed");
    if (seed == nullptr) {
        return true;
    }
    if (!cJSON_IsNumber(seed) || !std::isfinite(seed->valuedouble) ||
        std::floor(seed->valuedouble) != seed->valuedouble || seed->valuedouble < 0.0 ||
        seed->valuedouble > MAX_EXACT_JSON_INTEGER) {
        set_error(error_message, "seed must be an integer from 0 through 2^53-1");
        return false;
    }
    config->seed = static_cast<std::uint64_t>(seed->valuedouble);
    return true;
}

bool finite_nonnegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool valid_measurement_transient(const MeasurementTransientErrorConfig& config) {
    return finite_nonnegative(config.psr_extra_sigma_m) && finite_nonnegative(config.doppler_extra_sigma_mps) &&
           finite_nonnegative(config.cn0_extra_sigma_dbhz) && std::isfinite(config.decay_tau_sec) &&
           config.decay_tau_sec > 0.0;
}

bool valid_measurement_fade(const MeasurementFadeErrorConfig& config) {
    return finite_nonnegative(config.duration_sec) && finite_nonnegative(config.psr_extra_sigma_m) &&
           finite_nonnegative(config.doppler_extra_sigma_mps) && finite_nonnegative(config.cn0_drop_db);
}

bool valid_bestpos_rtk_config(const BestposRtkConfig& config) {
    return config.stable_duration_ns >= 0 && config.min_used_satellites >= 4 && config.min_used_satellites <= 64 &&
           finite_nonnegative(config.horizontal_std_m) && finite_nonnegative(config.height_std_m);
}

bool valid_measurement_error_config(const MeasurementErrorConfig& config) {
    return finite_nonnegative(config.psr_sigma_m) && finite_nonnegative(config.doppler_sigma_mps) &&
           finite_nonnegative(config.adr_sigma_m) && finite_nonnegative(config.cn0_sigma_dbhz) &&
           std::isfinite(config.psr_correlation_tau_sec) && config.psr_correlation_tau_sec > 0.0 &&
           valid_measurement_transient(config.ttff_hot) && valid_measurement_transient(config.ttff_warm) &&
           valid_measurement_transient(config.ttff_cold) && valid_measurement_transient(config.rea_reacquisition) &&
           valid_measurement_fade(config.rea_fade);
}

bool valid_cn0_high_baselines(const std::vector<Cn0HighBaselineConfig>& baselines) {
    for (std::size_t index = 0; index < baselines.size(); ++index) {
        const Cn0HighBaselineConfig& baseline = baselines[index];
        if (!std::isfinite(baseline.cn0_dbhz) || find_signal_definition_by_name(baseline.signal_name.c_str()) == nullptr) {
            return false;
        }
        for (std::size_t previous = 0; previous < index; ++previous) {
            if (baselines[previous].signal_name == baseline.signal_name) {
                return false;
            }
        }
    }
    return true;
}

bool valid_atmosphere_mode(AtmosphereMode atmosphere_mode) {
    switch (atmosphere_mode) {
        case AtmosphereMode::UNSPECIFIED:
        case AtmosphereMode::NONE:
        case AtmosphereMode::BROADCAST:
            return true;
    }
    return false;
}

} // namespace

SimConfig default_sim_config() {
    SimConfig config{};
    config.schema_version = 1;
    config.scenario = ScenarioType::KS;
    config.duration_ns = 28800LL * NANOSECONDS_PER_SECOND;
    config.sampling_rate_hz = 10;
    config.elevation_mask_deg = 3.0;
    config.solution_elevation_mask_deg = 5.0;
    config.output_eph = true;
    config.output_ion = true;
    config.measurement_noise_enabled = false;
    config.multipath_enabled = false;
    config.receiver_clock_bias_m = 0.0;
    config.receiver_clock_drift_mps = 0.0;
    config.atmosphere_mode = AtmosphereMode::UNSPECIFIED;
    config.receiver = {20.0, 120.0, 100.0};
    config.ttff = {StartupMode::HOT, 300LL * NANOSECONDS_PER_SECOND, 30LL * NANOSECONDS_PER_SECOND};
    config.rea = {300LL * NANOSECONDS_PER_SECOND, 10LL * NANOSECONDS_PER_SECOND};
    config.bestpos_rtk = {true, 5LL * NANOSECONDS_PER_SECOND, 6, 0.01, 0.02};
    config.measurement_error = {0.08,
                                0.03,
                                0.001,
                                0.5,
                                1.5,
                                {0.40, 0.10, 1.5, 1.0},
                                {0.50, 0.12, 2.0, 1.5},
                                {0.70, 0.15, 2.5, 2.0},
                                {0.40, 0.10, 1.5, 0.8},
                                {0.25, 0.80, 0.20, 4.5}};
    config.seed = 1U;
    return config;
}

bool validate_sim_config(const SimConfig& config, std::string* error_message) {
    std::int64_t interval_ns = 0;
    if (config.schema_version != 1) {
        set_error(error_message, "schema_version must be 1");
        return false;
    }
    if (config.duration_ns <= 0) {
        set_error(error_message, "duration_sec must be greater than zero");
        return false;
    }
    if (!sampling_interval_ns(config.sampling_rate_hz, &interval_ns)) {
        set_error(error_message, "sampling_rate_hz must be one of 1, 5, 10, 20, 50");
        return false;
    }
    if (!std::isfinite(config.elevation_mask_deg) || config.elevation_mask_deg < 0.0 ||
        config.elevation_mask_deg > 90.0) {
        set_error(error_message, "elevation_mask_deg must be within [0, 90]");
        return false;
    }
    if (!std::isfinite(config.solution_elevation_mask_deg) || config.solution_elevation_mask_deg < 0.0 ||
        config.solution_elevation_mask_deg > 90.0) {
        set_error(error_message, "solution_elevation_mask_deg must be within [0, 90]");
        return false;
    }
    if (!valid_bestpos_rtk_config(config.bestpos_rtk)) {
        set_error(error_message, "bestpos_rtk configuration is invalid");
        return false;
    }
    if (!valid_measurement_error_config(config.measurement_error)) {
        set_error(error_message, "measurement_error configuration is invalid");
        return false;
    }
    if (!valid_cn0_high_baselines(config.cn0_high_dbhz)) {
        set_error(error_message, "cn0_high_dbhz configuration is invalid");
        return false;
    }
    if (config.multipath_enabled) {
        set_error(error_message, "multipath is not supported in V1");
        return false;
    }
    if (config.receiver_clock_bias_m != 0.0 || config.receiver_clock_drift_mps != 0.0) {
        set_error(error_message, "receiver clock bias and drift must be zero in V1");
        return false;
    }
    if (!valid_atmosphere_mode(config.atmosphere_mode)) {
        set_error(error_message, "atmosphere_mode is invalid");
        return false;
    }
    if (!std::isfinite(config.receiver.latitude_deg) || config.receiver.latitude_deg < -90.0 ||
        config.receiver.latitude_deg > 90.0) {
        set_error(error_message, "receiver.latitude_deg must be within [-90, 90]");
        return false;
    }
    if (!std::isfinite(config.receiver.longitude_deg) || config.receiver.longitude_deg < -180.0 ||
        config.receiver.longitude_deg > 180.0 || !std::isfinite(config.receiver.height_m)) {
        set_error(error_message, "receiver longitude/height is invalid");
        return false;
    }
    if (config.ttff.power_on_ns <= 0 || config.ttff.power_off_ns <= 0) {
        set_error(error_message, "TTFF power on/off durations must be greater than zero");
        return false;
    }
    if (config.rea.signal_on_ns <= 0 || config.rea.signal_off_ns <= 0) {
        set_error(error_message, "REA signal on/off durations must be greater than zero");
        return false;
    }
    return true;
}

bool load_sim_config_json(const char* file_path, SimConfig* config, std::string* error_message) {
    if (file_path == nullptr || config == nullptr) {
        set_error(error_message, "config path/output pointer must not be null");
        return false;
    }

    std::ifstream input(file_path, std::ios::binary);
    if (!input) {
        set_error(error_message, std::string("failed to open config file: ") + file_path);
        return false;
    }
    const std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    if (text.empty()) {
        set_error(error_message, "config file is empty");
        return false;
    }

    cJSON* root = cJSON_ParseWithLength(text.c_str(), text.size());
    if (root == nullptr) {
        const char* parse_error = cJSON_GetErrorPtr();
        std::string message = "invalid JSON";
        if (parse_error != nullptr) {
            message += ": near ";
            message += parse_error;
        }
        set_error(error_message, message);
        return false;
    }

    const char* const allowed_keys[] = {"schema_version",
                                        "scenario",
                                        "duration_sec",
                                        "sampling_rate_hz",
                                        "elevation_mask_deg",
                                        "solution_elevation_mask_deg",
                                        "output_eph",
                                        "output_ion",
                                        "measurement_noise_enabled",
                                        "bestpos_rtk",
                                        "measurement_error",
                                        "cn0_high_dbhz",
                                        "multipath_enabled",
                                        "receiver_clock_bias_m",
                                        "receiver_clock_drift_mps",
                                        "atmosphere_mode",
                                        "receiver",
                                        "ttff",
                                        "rea",
                                        "seed"};

    SimConfig parsed = default_sim_config();
    bool success = validate_object_keys(root, "root", allowed_keys, 20U, error_message);

    double duration_sec = static_cast<double>(parsed.duration_ns) / static_cast<double>(NANOSECONDS_PER_SECOND);
    const char* scenario_name = scenario_type_name(parsed.scenario);
    const char* atmosphere_name = atmosphere_mode_name(parsed.atmosphere_mode);
    if (success) {
        success = read_optional_int(root, "schema_version", &parsed.schema_version, error_message) &&
                  read_optional_string(root, "scenario", &scenario_name, error_message) &&
                  parse_scenario(scenario_name, &parsed.scenario, error_message) &&
                  read_optional_number(root, "duration_sec", &duration_sec, error_message) &&
                  seconds_to_ns(duration_sec, "duration_sec", &parsed.duration_ns, error_message) &&
                  read_optional_int(root, "sampling_rate_hz", &parsed.sampling_rate_hz, error_message) &&
                  read_optional_number(root, "elevation_mask_deg", &parsed.elevation_mask_deg, error_message) &&
                  read_optional_number(root, "solution_elevation_mask_deg", &parsed.solution_elevation_mask_deg,
                                       error_message) &&
                  read_optional_bool(root, "output_eph", &parsed.output_eph, error_message) &&
                  read_optional_bool(root, "output_ion", &parsed.output_ion, error_message);
    }
    if (success && cJSON_GetObjectItemCaseSensitive(root, "measurement_noise_enabled") == nullptr) {
        parsed.measurement_noise_enabled =
            parsed.scenario == ScenarioType::TTFF || parsed.scenario == ScenarioType::REA;
    }
    if (success) {
        success =
            read_optional_bool(root, "measurement_noise_enabled", &parsed.measurement_noise_enabled, error_message) &&
            read_optional_bool(root, "multipath_enabled", &parsed.multipath_enabled, error_message) &&
            read_optional_number(root, "receiver_clock_bias_m", &parsed.receiver_clock_bias_m, error_message) &&
            read_optional_number(root, "receiver_clock_drift_mps", &parsed.receiver_clock_drift_mps, error_message) &&
            read_optional_string(root, "atmosphere_mode", &atmosphere_name, error_message) &&
            parse_atmosphere_mode(atmosphere_name, &parsed.atmosphere_mode, error_message) &&
            parse_receiver(root, &parsed, error_message) && parse_ttff(root, &parsed, error_message) &&
            parse_rea(root, &parsed, error_message) && parse_bestpos_rtk(root, &parsed, error_message) &&
            parse_measurement_error(root, &parsed, error_message) &&
            parse_cn0_high_dbhz(root, &parsed, error_message) && parse_seed(root, &parsed, error_message) &&
            validate_sim_config(parsed, error_message);
    }

    cJSON_Delete(root);
    if (!success) {
        return false;
    }

    *config = parsed;
    return true;
}

const char* scenario_type_name(ScenarioType scenario) {
    switch (scenario) {
        case ScenarioType::KS:
            return "KS";
        case ScenarioType::REA:
            return "REA";
        case ScenarioType::TTFF:
            return "TTFF";
    }
    return "UNKNOWN";
}

const char* startup_mode_name(StartupMode startup_mode) {
    switch (startup_mode) {
        case StartupMode::HOT:
            return "HOT";
        case StartupMode::WARM:
            return "WARM";
        case StartupMode::COLD:
            return "COLD";
    }
    return "UNKNOWN";
}

} // namespace gnss_sim
