#include "gnss_sim/sim_config.h"

#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr const char* kConfigPath = "gnss_sim_carrier_tracking_config.json";

void write_config(const char* text) {
    std::ofstream output(kConfigPath, std::ios::binary | std::ios::trunc);
    output << text;
}

class CarrierTrackingConfigTest : public ::testing::Test {
  protected:
    void TearDown() override {
        static_cast<void>(std::remove(kConfigPath));
    }
};

TEST_F(CarrierTrackingConfigTest, FrozenDefaultsRemainOptIn) {
    const gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    EXPECT_FALSE(config.carrier_tracking.enabled);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.coherent_integration_sec, 0.020);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.pll_noise_bandwidth_hz, 5.0);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.fll_noise_bandwidth_hz, 4.0);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.fll_pull_in_bandwidth_hz, 8.0);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.fll_pull_in_duration_sec, 0.5);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.pll_enter_cn0_dbhz, 30.0);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.pll_exit_cn0_dbhz, 27.0);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.pll_enter_persistence_sec, 1.0);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.pll_exit_persistence_sec, 0.3);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.fll_enter_cn0_dbhz, 22.0);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.fll_exit_cn0_dbhz, 18.0);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.fll_enter_persistence_sec, 0.2);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.fll_exit_persistence_sec, 0.5);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.doppler_valid_delay_sec, 0.2);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.adr_valid_after_pll_sec, 1.0);
}

TEST_F(CarrierTrackingConfigTest, LoadsExplicitOverrides) {
    write_config(R"json({
        "carrier_tracking": {
            "enabled": true,
            "coherent_integration_sec": 0.01,
            "pll_noise_bandwidth_hz": 6.0,
            "fll_noise_bandwidth_hz": 5.0,
            "fll_pull_in_bandwidth_hz": 9.0,
            "fll_pull_in_duration_sec": 0.6,
            "pll_enter_cn0_dbhz": 31.0,
            "pll_exit_cn0_dbhz": 28.0,
            "pll_enter_persistence_sec": 1.2,
            "pll_exit_persistence_sec": 0.4,
            "fll_enter_cn0_dbhz": 23.0,
            "fll_exit_cn0_dbhz": 19.0,
            "fll_enter_persistence_sec": 0.25,
            "fll_exit_persistence_sec": 0.55,
            "doppler_valid_delay_sec": 0.25,
            "adr_valid_after_pll_sec": 1.1
        }
    })json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_sim_config_json(kConfigPath, &config, &error_message)) << error_message;
    EXPECT_TRUE(config.carrier_tracking.enabled);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.coherent_integration_sec, 0.01);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.pll_noise_bandwidth_hz, 6.0);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.fll_noise_bandwidth_hz, 5.0);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.fll_pull_in_bandwidth_hz, 9.0);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.pll_enter_cn0_dbhz, 31.0);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.fll_exit_cn0_dbhz, 19.0);
    EXPECT_DOUBLE_EQ(config.carrier_tracking.adr_valid_after_pll_sec, 1.1);
}

TEST_F(CarrierTrackingConfigTest, RejectsInvalidCoreOrderingEvenWhenDisabled) {
    write_config(R"json({"carrier_tracking":{"enabled":false,"pll_exit_cn0_dbhz":35.0}})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::load_sim_config_json(kConfigPath, &config, &error_message));
    EXPECT_NE(error_message.find("carrier_tracking"), std::string::npos) << error_message;
}

TEST_F(CarrierTrackingConfigTest, RejectsUnknownCarrierKey) {
    write_config(R"json({"carrier_tracking":{"doppler_sigma_mps":1.0}})json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::load_sim_config_json(kConfigPath, &config, &error_message));
    EXPECT_NE(error_message.find("carrier_tracking"), std::string::npos) << error_message;
}

} // namespace