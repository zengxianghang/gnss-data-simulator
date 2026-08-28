#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"
#include "rangea_roundtrip.h"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <sstream>
#include <string>

namespace {

std::string brd4_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

gnss_sim::SimTime start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 436500.0, &time));
    return time;
}

gnss_sim::SimConfig config() {
    gnss_sim::SimConfig value = gnss_sim::default_sim_config();
    value.scenario = gnss_sim::ScenarioType::KS;
    value.atmosphere_mode = gnss_sim::AtmosphereMode::BROADCAST;
    value.sampling_rate_hz = 1;
    value.duration_ns = 60LL * gnss_sim::NANOSECONDS_PER_SECOND;
    value.elevation_mask_deg = 0.0;
    value.receiver = {20.0, 120.0, 100.0};
    value.measurement_noise_enabled = false;
    value.multipath_enabled = false;
    value.output_eph = true;
    value.output_ion = true;
    value.seed = 0x60U;
    return value;
}

bool run_simulator(const std::filesystem::path& directory, gnss_sim::SimulatorRunSummary* summary,
                   std::string* error_message) {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    error.clear();
    if (!std::filesystem::create_directories(directory, error) || error) {
        if (error_message != nullptr) {
            *error_message = error.message();
        }
        return false;
    }
    const std::filesystem::path log_path = directory / "simulated.log";
    const std::string log_text = log_path.string();
    const std::string nav_text = brd4_nav_path();
    const gnss_sim::SimulatorRunOptions options{nav_text.c_str(), log_text.c_str(), start_time()};
    return gnss_sim::run_simulator(config(), options, summary, error_message);
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

void cleanup(const std::filesystem::path& path) {
    std::error_code error;
    std::filesystem::remove_all(path, error);
}

TEST(RangeaRoundtripIntegration, RealWhuRinex4SerializedRangeaPositionsWithinHalfMeter) {
    const std::filesystem::path directory = "gnss_sim_rangea_roundtrip_real_whu";
    gnss_sim::SimulatorRunSummary simulator_summary{};
    std::string error_message;
    ASSERT_TRUE(run_simulator(directory, &simulator_summary, &error_message)) << error_message;
    ASSERT_GT(simulator_summary.range_messages, 0U);

    gnss_sim::RangeaRoundtripSummary roundtrip{};
    const std::string log_path = (directory / "simulated.log").string();
    const std::string nav_path = brd4_nav_path();
    ASSERT_TRUE(gnss_sim::validate_rangea_roundtrip_file(log_path.c_str(), nav_path.c_str(), 20.0, 120.0, 100.0, 0.0,
                                                         true, &roundtrip, &error_message))
        << error_message;
    EXPECT_EQ(roundtrip.range_epochs, simulator_summary.range_messages);
    EXPECT_GT(roundtrip.parsed_observations, 0U);
    EXPECT_GT(roundtrip.selected_position_observations, 0U);
    EXPECT_GT(roundtrip.valid_position_epochs, 0U);
    EXPECT_LT(roundtrip.max_position_error_m, 0.5)
        << "serialized RANGEA must retain enough precision and mapping fidelity for RTKLIB SPP";

    cleanup(directory);
}

TEST(RangeaRoundtripIntegration, MalformedSerializedRangeaFailsExplicitly) {
    std::istringstream malformed("#RANGEA,COM1,0,0.0,FINE,2347,436500.000,00000000,0,0;1,1,0,20000000.000*00000000\r\n");
    gnss_sim::RangeaRoundtripSummary summary{};
    std::string error_message;
    const std::string nav_path = brd4_nav_path();
    EXPECT_FALSE(gnss_sim::validate_rangea_roundtrip_stream(&malformed, nav_path.c_str(), 20.0, 120.0, 100.0, 0.0,
                                                            true, &summary, &error_message));
    EXPECT_FALSE(error_message.empty());
}

TEST(RangeaRoundtripIntegration, SameInputProducesIdenticalRoundtripResult) {
    const std::filesystem::path first_directory = "gnss_sim_rangea_roundtrip_repeat_a";
    const std::filesystem::path second_directory = "gnss_sim_rangea_roundtrip_repeat_b";
    gnss_sim::SimulatorRunSummary first_simulator{};
    gnss_sim::SimulatorRunSummary second_simulator{};
    std::string error_message;
    ASSERT_TRUE(run_simulator(first_directory, &first_simulator, &error_message)) << error_message;
    ASSERT_TRUE(run_simulator(second_directory, &second_simulator, &error_message)) << error_message;
    EXPECT_EQ(read_file(first_directory / "simulated.log"), read_file(second_directory / "simulated.log"));

    const std::string nav_path = brd4_nav_path();
    const std::string first_log = (first_directory / "simulated.log").string();
    const std::string second_log = (second_directory / "simulated.log").string();
    gnss_sim::RangeaRoundtripSummary first_roundtrip{};
    gnss_sim::RangeaRoundtripSummary second_roundtrip{};
    ASSERT_TRUE(gnss_sim::validate_rangea_roundtrip_file(first_log.c_str(), nav_path.c_str(), 20.0, 120.0, 100.0, 0.0,
                                                         true, &first_roundtrip, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::validate_rangea_roundtrip_file(second_log.c_str(), nav_path.c_str(), 20.0, 120.0, 100.0,
                                                         0.0, true, &second_roundtrip, &error_message))
        << error_message;

    EXPECT_EQ(first_roundtrip.range_epochs, second_roundtrip.range_epochs);
    EXPECT_EQ(first_roundtrip.parsed_observations, second_roundtrip.parsed_observations);
    EXPECT_EQ(first_roundtrip.selected_position_observations, second_roundtrip.selected_position_observations);
    EXPECT_EQ(first_roundtrip.valid_position_epochs, second_roundtrip.valid_position_epochs);
    EXPECT_DOUBLE_EQ(first_roundtrip.max_position_error_m, second_roundtrip.max_position_error_m);
    EXPECT_EQ(first_roundtrip.max_error_gps_week, second_roundtrip.max_error_gps_week);
    EXPECT_DOUBLE_EQ(first_roundtrip.max_error_sow_sec, second_roundtrip.max_error_sow_sec);

    cleanup(first_directory);
    cleanup(second_directory);
}

} // namespace
