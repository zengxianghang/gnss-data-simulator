#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"

#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr const char* TEST_CONFIG_PATH = "gnss_sim_test_config.json";

void write_test_config(const char* text) {
    std::ofstream output(TEST_CONFIG_PATH, std::ios::binary | std::ios::trunc);
    output << text;
}

class SimConfigTest : public ::testing::Test {
  protected:
    void TearDown() override {
        static_cast<void>(std::remove(TEST_CONFIG_PATH));
    }
};

TEST_F(SimConfigTest, FrozenDefaultsAreCentralized) {
    const gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    EXPECT_EQ(config.duration_ns, 28800LL * gnss_sim::NANOSECONDS_PER_SECOND);
    EXPECT_EQ(config.sampling_rate_hz, 10);
    EXPECT_DOUBLE_EQ(config.elevation_mask_deg, 3.0);
    EXPECT_DOUBLE_EQ(config.solution_elevation_mask_deg, 5.0);
    EXPECT_TRUE(config.output_eph);
    EXPECT_TRUE(config.output_ion);
    EXPECT_FALSE(config.measurement_noise_enabled);
    EXPECT_DOUBLE_EQ(config.measurement_error.psr_sigma_m, 0.08);
    EXPECT_DOUBLE_EQ(config.measurement_error.doppler_sigma_mps, 0.03);
    EXPECT_DOUBLE_EQ(config.measurement_error.adr_sigma_m, 0.001);
    EXPECT_DOUBLE_EQ(config.measurement_error.cn0_sigma_dbhz, 0.5);
    EXPECT_DOUBLE_EQ(config.measurement_error.psr_correlation_tau_sec, 1.5);
    EXPECT_DOUBLE_EQ(config.measurement_error.ttff_hot.psr_extra_sigma_m, 0.40);
    EXPECT_DOUBLE_EQ(config.measurement_error.ttff_cold.psr_extra_sigma_m, 0.70);
    EXPECT_DOUBLE_EQ(config.measurement_error.rea_reacquisition.decay_tau_sec, 0.8);
    EXPECT_DOUBLE_EQ(config.measurement_error.rea_fade.duration_sec, 0.25);
    EXPECT_TRUE(config.bestpos_rtk.enabled);
    EXPECT_EQ(config.bestpos_rtk.stable_duration_ns, 5LL * gnss_sim::NANOSECONDS_PER_SECOND);
    EXPECT_EQ(config.bestpos_rtk.min_used_satellites, 6);
    EXPECT_DOUBLE_EQ(config.bestpos_rtk.horizontal_std_m, 0.01);
    EXPECT_DOUBLE_EQ(config.bestpos_rtk.height_std_m, 0.02);
    EXPECT_FALSE(config.multipath_enabled);
    EXPECT_DOUBLE_EQ(config.receiver_clock_bias_m, 0.0);
    EXPECT_DOUBLE_EQ(config.receiver_clock_drift_mps, 0.0);
    EXPECT_EQ(config.atmosphere_mode, gnss_sim::AtmosphereMode::UNSPECIFIED);
    EXPECT_EQ(config.ttff.startup_mode, gnss_sim::StartupMode::HOT);
}

TEST_F(SimConfigTest, LoadsValidOverridesWithoutLeakingJsonTypes) {
    write_test_config(R"json({
        "scenario": "TTFF",
        "duration_sec": 60,
        "sampling_rate_hz": 50,
        "elevation_mask_deg": 2,
        "solution_elevation_mask_deg": 7,
        "atmosphere_mode": "broadcast",
        "receiver": {"latitude_deg": 21, "longitude_deg": 121, "height_m": 50},
        "ttff": {"startup_mode": "COLD", "power_on_sec": 20, "power_off_sec": 5},
        "seed": 42
    })json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message)) << error_message;
    EXPECT_EQ(config.scenario, gnss_sim::ScenarioType::TTFF);
    EXPECT_EQ(config.duration_ns, 60LL * gnss_sim::NANOSECONDS_PER_SECOND);
    EXPECT_EQ(config.sampling_rate_hz, 50);
    EXPECT_DOUBLE_EQ(config.elevation_mask_deg, 2.0);
    EXPECT_DOUBLE_EQ(config.solution_elevation_mask_deg, 7.0);
    EXPECT_EQ(config.atmosphere_mode, gnss_sim::AtmosphereMode::BROADCAST);
    EXPECT_EQ(config.ttff.startup_mode, gnss_sim::StartupMode::COLD);
    EXPECT_EQ(config.seed, 42U);
}

TEST_F(SimConfigTest, SupportsExplicitNoneWithoutFreezingProjectDefault) {
    write_test_config(R"json({"atmosphere_mode": "none"})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message)) << error_message;
    EXPECT_EQ(config.atmosphere_mode, gnss_sim::AtmosphereMode::NONE);
    EXPECT_STREQ(gnss_sim::atmosphere_mode_name(config.atmosphere_mode), "none");
}

TEST_F(SimConfigTest, RejectsUnsupportedAtmosphereMode) {
    write_test_config(R"json({"atmosphere_mode": "auto"})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message));
    EXPECT_NE(error_message.find("unsupported atmosphere_mode"), std::string::npos);
}

TEST_F(SimConfigTest, RejectsUnknownKey) {
    write_test_config(R"json({"sampling_rate_hz": 10, "samplng_rate_hz": 20})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message));
    EXPECT_NE(error_message.find("unsupported key"), std::string::npos);
}

TEST_F(SimConfigTest, OmittedSolutionMaskKeepsFiveDegreeDefault) {
    write_test_config(R"json({"elevation_mask_deg": 0})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message)) << error_message;
    EXPECT_DOUBLE_EQ(config.elevation_mask_deg, 0.0);
    EXPECT_DOUBLE_EQ(config.solution_elevation_mask_deg, 5.0);
}

TEST_F(SimConfigTest, RejectsInvalidSolutionElevationMask) {
    write_test_config(R"json({"solution_elevation_mask_deg": 91})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message));
    EXPECT_NE(error_message.find("solution_elevation_mask_deg"), std::string::npos);
}

TEST_F(SimConfigTest, RejectsUnsupportedRateAndV1Noise) {
    write_test_config(R"json({"sampling_rate_hz": 2, "measurement_noise_enabled": true})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message));
}

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

TEST_F(SimConfigTest, LoadsAndValidatesBestposRtkOverrides) {
    write_test_config(R"json({
        "bestpos_rtk": {
            "enabled": false,
            "stable_duration_sec": 3.5,
            "min_used_satellites": 7,
            "horizontal_std_m": 0.015,
            "height_std_m": 0.03
        }
    })json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message)) << error_message;
    EXPECT_FALSE(config.bestpos_rtk.enabled);
    EXPECT_EQ(config.bestpos_rtk.stable_duration_ns, 3500000000LL);
    EXPECT_EQ(config.bestpos_rtk.min_used_satellites, 7);
    EXPECT_DOUBLE_EQ(config.bestpos_rtk.horizontal_std_m, 0.015);
    EXPECT_DOUBLE_EQ(config.bestpos_rtk.height_std_m, 0.03);

    write_test_config(R"json({"bestpos_rtk":{"min_used_satellites":3}})json");
    EXPECT_FALSE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message));
    EXPECT_NE(error_message.find("bestpos_rtk"), std::string::npos);
}

TEST_F(SimConfigTest, RejectsInvalidMeasurementErrorConfiguration) {
    write_test_config(R"json({"measurement_error":{"psr_sigma_m":-0.1}})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message));
    EXPECT_NE(error_message.find("measurement_error"), std::string::npos);
}

TEST_F(SimConfigTest, MeasurementNoiseMasterSwitchIsAcceptedAfterIntegration) {
    write_test_config(R"json({"measurement_noise_enabled":true})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message)) << error_message;
    EXPECT_TRUE(config.measurement_noise_enabled);
}

TEST_F(SimConfigTest, TtffDefaultsMeasurementNoiseOnWhenFlagIsOmitted) {
    write_test_config(R"json({"scenario":"TTFF"})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message)) << error_message;
    EXPECT_TRUE(config.measurement_noise_enabled);
}

TEST_F(SimConfigTest, ReaDefaultsMeasurementNoiseOnWhenFlagIsOmitted) {
    write_test_config(R"json({"scenario":"REA"})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message)) << error_message;
    EXPECT_TRUE(config.measurement_noise_enabled);
}

TEST_F(SimConfigTest, KsKeepsMeasurementNoiseOffWhenFlagIsOmitted) {
    write_test_config(R"json({"scenario":"KS"})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message)) << error_message;
    EXPECT_FALSE(config.measurement_noise_enabled);
}

TEST_F(SimConfigTest, ExplicitFalseOverridesTtffMeasurementNoiseDefault) {
    write_test_config(R"json({"scenario":"TTFF","measurement_noise_enabled":false})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message)) << error_message;
    EXPECT_FALSE(config.measurement_noise_enabled);
}

} // namespace
