from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if text.count(old) != 1:
        raise RuntimeError(f"{label}: expected exactly one match, found {text.count(old)}")
    return text.replace(old, new, 1)


# sim_config.cpp
path = Path("src/core/sim_config.cpp")
text = path.read_text()

parse_helpers = r'''
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
           read_optional_number(object, "doppler_extra_sigma_mps", &config->doppler_extra_sigma_mps,
                                error_message) &&
           read_optional_number(object, "cn0_extra_sigma_dbhz", &config->cn0_extra_sigma_dbhz, error_message) &&
           read_optional_number(object, "decay_tau_sec", &config->decay_tau_sec, error_message);
}

bool parse_measurement_fade(const cJSON* parent, MeasurementFadeErrorConfig* config, std::string* error_message) {
    const cJSON* object = cJSON_GetObjectItemCaseSensitive(parent, "rea_fade");
    if (object == nullptr) {
        return true;
    }
    const char* const allowed_keys[] = {"duration_sec", "psr_extra_sigma_m", "doppler_extra_sigma_mps",
                                        "cn0_drop_db"};
    if (!validate_object_keys(object, "measurement_error.rea_fade", allowed_keys, 4U, error_message)) {
        return false;
    }
    return read_optional_number(object, "duration_sec", &config->duration_sec, error_message) &&
           read_optional_number(object, "psr_extra_sigma_m", &config->psr_extra_sigma_m, error_message) &&
           read_optional_number(object, "doppler_extra_sigma_mps", &config->doppler_extra_sigma_mps,
                                error_message) &&
           read_optional_number(object, "cn0_drop_db", &config->cn0_drop_db, error_message);
}

bool parse_measurement_error(const cJSON* root, SimConfig* config, std::string* error_message) {
    const cJSON* object = cJSON_GetObjectItemCaseSensitive(root, "measurement_error");
    if (object == nullptr) {
        return true;
    }
    const char* const allowed_keys[] = {"psr_sigma_m",
                                        "doppler_sigma_mps",
                                        "adr_sigma_m",
                                        "cn0_sigma_dbhz",
                                        "psr_correlation_tau_sec",
                                        "ttff_hot",
                                        "ttff_warm",
                                        "ttff_cold",
                                        "rea_reacquisition",
                                        "rea_fade"};
    if (!validate_object_keys(object, "measurement_error", allowed_keys, 10U, error_message)) {
        return false;
    }
    return read_optional_number(object, "psr_sigma_m", &config->measurement_error.psr_sigma_m, error_message) &&
           read_optional_number(object, "doppler_sigma_mps", &config->measurement_error.doppler_sigma_mps,
                                error_message) &&
           read_optional_number(object, "adr_sigma_m", &config->measurement_error.adr_sigma_m, error_message) &&
           read_optional_number(object, "cn0_sigma_dbhz", &config->measurement_error.cn0_sigma_dbhz,
                                error_message) &&
           read_optional_number(object, "psr_correlation_tau_sec",
                                &config->measurement_error.psr_correlation_tau_sec, error_message) &&
           parse_measurement_transient(object, "ttff_hot", &config->measurement_error.ttff_hot, error_message) &&
           parse_measurement_transient(object, "ttff_warm", &config->measurement_error.ttff_warm, error_message) &&
           parse_measurement_transient(object, "ttff_cold", &config->measurement_error.ttff_cold, error_message) &&
           parse_measurement_transient(object, "rea_reacquisition", &config->measurement_error.rea_reacquisition,
                                       error_message) &&
           parse_measurement_fade(object, &config->measurement_error.rea_fade, error_message);
}

'''
text = replace_once(text, "bool parse_seed(const cJSON* root, SimConfig* config, std::string* error_message) {",
                    parse_helpers + "bool parse_seed(const cJSON* root, SimConfig* config, std::string* error_message) {",
                    "insert measurement parsers")

validation_helpers = r'''
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

bool valid_measurement_error_config(const MeasurementErrorConfig& config) {
    return finite_nonnegative(config.psr_sigma_m) && finite_nonnegative(config.doppler_sigma_mps) &&
           finite_nonnegative(config.adr_sigma_m) && finite_nonnegative(config.cn0_sigma_dbhz) &&
           std::isfinite(config.psr_correlation_tau_sec) && config.psr_correlation_tau_sec > 0.0 &&
           valid_measurement_transient(config.ttff_hot) && valid_measurement_transient(config.ttff_warm) &&
           valid_measurement_transient(config.ttff_cold) && valid_measurement_transient(config.rea_reacquisition) &&
           valid_measurement_fade(config.rea_fade);
}

'''
text = replace_once(text, "bool valid_atmosphere_mode(AtmosphereMode atmosphere_mode) {",
                    validation_helpers + "bool valid_atmosphere_mode(AtmosphereMode atmosphere_mode) {",
                    "insert measurement validators")

text = replace_once(
    text,
    "    config.rea = {300LL * NANOSECONDS_PER_SECOND, 10LL * NANOSECONDS_PER_SECOND};\n    config.seed = 1U;",
    "    config.rea = {300LL * NANOSECONDS_PER_SECOND, 10LL * NANOSECONDS_PER_SECOND};\n"
    "    config.measurement_error = {0.08,\n"
    "                                0.03,\n"
    "                                0.001,\n"
    "                                0.5,\n"
    "                                1.5,\n"
    "                                {0.40, 0.10, 1.5, 1.0},\n"
    "                                {0.50, 0.12, 2.0, 1.5},\n"
    "                                {0.70, 0.15, 2.5, 2.0},\n"
    "                                {0.40, 0.10, 1.5, 0.8},\n"
    "                                {0.25, 0.80, 0.20, 4.5}};\n"
    "    config.seed = 1U;",
    "add measurement defaults",
)

text = replace_once(
    text,
    "    if (config.measurement_noise_enabled) {\n        set_error(error_message, \"measurement noise is not supported in V1\");",
    "    if (!valid_measurement_error_config(config.measurement_error)) {\n"
    "        set_error(error_message, \"measurement_error configuration is invalid\");\n"
    "        return false;\n"
    "    }\n"
    "    if (config.measurement_noise_enabled) {\n        set_error(error_message, \"measurement noise is not supported in V1\");",
    "validate measurement config",
)

text = replace_once(
    text,
    '                                        "measurement_noise_enabled",\n                                        "multipath_enabled",',
    '                                        "measurement_noise_enabled",\n                                        "measurement_error",\n                                        "multipath_enabled",',
    "allow measurement_error root key",
)
text = replace_once(text, 'validate_object_keys(root, "root", allowed_keys, 17U, error_message)',
                    'validate_object_keys(root, "root", allowed_keys, 18U, error_message)', "root key count")
text = replace_once(
    text,
    "            parse_receiver(root, &parsed, error_message) && parse_ttff(root, &parsed, error_message) &&\n"
    "            parse_rea(root, &parsed, error_message) && parse_seed(root, &parsed, error_message) &&",
    "            parse_receiver(root, &parsed, error_message) && parse_ttff(root, &parsed, error_message) &&\n"
    "            parse_rea(root, &parsed, error_message) && parse_measurement_error(root, &parsed, error_message) &&\n"
    "            parse_seed(root, &parsed, error_message) &&",
    "parse measurement_error",
)
path.write_text(text)

# CMakeLists.txt
path = Path("CMakeLists.txt")
text = path.read_text()
text = replace_once(text, "    src/model/cn0_model.cpp\n    src/model/measurement_model.cpp",
                    "    src/model/cn0_model.cpp\n    src/model/measurement_error_model.cpp\n    src/model/measurement_model.cpp",
                    "add model source")
text = replace_once(text, "        tests/unit/test_low_elevation_signal_geometry.cpp\n        tests/unit/test_measurement_model.cpp",
                    "        tests/unit/test_low_elevation_signal_geometry.cpp\n        tests/unit/test_measurement_error_model.cpp\n        tests/unit/test_measurement_model.cpp",
                    "add model tests")
path.write_text(text)

# test_sim_config.cpp
path = Path("tests/unit/test_sim_config.cpp")
text = path.read_text()
text = replace_once(
    text,
    "    EXPECT_FALSE(config.measurement_noise_enabled);\n    EXPECT_FALSE(config.multipath_enabled);",
    "    EXPECT_FALSE(config.measurement_noise_enabled);\n"
    "    EXPECT_DOUBLE_EQ(config.measurement_error.psr_sigma_m, 0.08);\n"
    "    EXPECT_DOUBLE_EQ(config.measurement_error.doppler_sigma_mps, 0.03);\n"
    "    EXPECT_DOUBLE_EQ(config.measurement_error.adr_sigma_m, 0.001);\n"
    "    EXPECT_DOUBLE_EQ(config.measurement_error.cn0_sigma_dbhz, 0.5);\n"
    "    EXPECT_DOUBLE_EQ(config.measurement_error.psr_correlation_tau_sec, 1.5);\n"
    "    EXPECT_DOUBLE_EQ(config.measurement_error.ttff_hot.psr_extra_sigma_m, 0.40);\n"
    "    EXPECT_DOUBLE_EQ(config.measurement_error.ttff_cold.psr_extra_sigma_m, 0.70);\n"
    "    EXPECT_DOUBLE_EQ(config.measurement_error.rea_reacquisition.decay_tau_sec, 0.8);\n"
    "    EXPECT_DOUBLE_EQ(config.measurement_error.rea_fade.duration_sec, 0.25);\n"
    "    EXPECT_FALSE(config.multipath_enabled);",
    "assert measurement defaults",
)

extra_tests = r'''

TEST_F(SimConfigTest, LoadsMeasurementErrorOverridesWhileNoiseRemainsDisabled) {
    write_test_config(R"json({
        "measurement_error": {
            "psr_sigma_m": 0.12,
            "doppler_sigma_mps": 0.04,
            "adr_sigma_m": 0.002,
            "cn0_sigma_dbhz": 0.7,
            "psr_correlation_tau_sec": 2.5,
            "ttff_hot": {
                "psr_extra_sigma_m": 0.6,
                "doppler_extra_sigma_mps": 0.2,
                "cn0_extra_sigma_dbhz": 2.2,
                "decay_tau_sec": 1.2
            },
            "rea_fade": {
                "duration_sec": 0.4,
                "psr_extra_sigma_m": 0.9,
                "doppler_extra_sigma_mps": 0.25,
                "cn0_drop_db": 5.0
            }
        }
    })json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message)) << error_message;
    EXPECT_FALSE(config.measurement_noise_enabled);
    EXPECT_DOUBLE_EQ(config.measurement_error.psr_sigma_m, 0.12);
    EXPECT_DOUBLE_EQ(config.measurement_error.doppler_sigma_mps, 0.04);
    EXPECT_DOUBLE_EQ(config.measurement_error.adr_sigma_m, 0.002);
    EXPECT_DOUBLE_EQ(config.measurement_error.cn0_sigma_dbhz, 0.7);
    EXPECT_DOUBLE_EQ(config.measurement_error.psr_correlation_tau_sec, 2.5);
    EXPECT_DOUBLE_EQ(config.measurement_error.ttff_hot.psr_extra_sigma_m, 0.6);
    EXPECT_DOUBLE_EQ(config.measurement_error.ttff_hot.decay_tau_sec, 1.2);
    EXPECT_DOUBLE_EQ(config.measurement_error.rea_fade.duration_sec, 0.4);
    EXPECT_DOUBLE_EQ(config.measurement_error.rea_fade.cn0_drop_db, 5.0);
}

TEST_F(SimConfigTest, RejectsInvalidMeasurementErrorConfiguration) {
    write_test_config(R"json({"measurement_error":{"psr_sigma_m":-0.1}})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message));
    EXPECT_NE(error_message.find("measurement_error"), std::string::npos);
}

TEST_F(SimConfigTest, MeasurementNoiseMasterSwitchRemainsRejectedUntilIntegration) {
    write_test_config(R"json({"measurement_noise_enabled":true})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message));
    EXPECT_NE(error_message.find("measurement noise is not supported"), std::string::npos);
}
'''
text = replace_once(text, "\n} // namespace\n", extra_tests + "\n} // namespace\n", "append config tests")
path.write_text(text)
