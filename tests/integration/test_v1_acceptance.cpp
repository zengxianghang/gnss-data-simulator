#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

#include <array>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>

namespace {

std::string nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/gps_loopback_nav.rnx";
}

gnss_sim::SimTime acceptance_start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2253, 172900.0, &time));
    return time;
}

gnss_sim::SimConfig acceptance_config() {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::KS;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    config.elevation_mask_deg = 0.0;
    config.output_eph = true;
    config.output_ion = true;
    config.seed = 0x40U;
    return config;
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

bool run_in_directory(const std::filesystem::path& directory, const gnss_sim::SimConfig& config,
                      gnss_sim::SimulatorRunSummary* summary, std::string* error_message) {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    error.clear();
    if (!std::filesystem::create_directories(directory, error) || error) {
        if (error_message != nullptr) {
            *error_message = "cannot create V1 acceptance output directory";
        }
        return false;
    }

    const std::filesystem::path output_path = directory / "simulated.log";
    const std::string input_path = nav_path();
    const std::string output_text = output_path.string();
    const gnss_sim::SimulatorRunOptions options{input_path.c_str(), output_text.c_str(), acceptance_start_time()};
    return gnss_sim::run_simulator(config, options, summary, error_message);
}

void expect_nonempty_file(const std::filesystem::path& path) {
    std::error_code error;
    ASSERT_TRUE(std::filesystem::exists(path, error)) << path.string();
    ASSERT_FALSE(error) << error.message();
    EXPECT_GT(std::filesystem::file_size(path, error), 0U) << path.string();
    EXPECT_FALSE(error) << error.message();
}

void cleanup(const std::filesystem::path& path) {
    std::error_code error;
    std::filesystem::remove_all(path, error);
}

TEST(V1Acceptance, FrozenRatesRunThroughFullStreamingPipeline) {
    constexpr std::int64_t kDurationSeconds = 10;
    for (const int rate : {1, 5, 10, 20, 50}) {
        gnss_sim::SimConfig config = acceptance_config();
        config.sampling_rate_hz = rate;
        config.duration_ns = kDurationSeconds * gnss_sim::NANOSECONDS_PER_SECOND;

        const std::filesystem::path directory = "gnss_sim_acceptance_rate_" + std::to_string(rate);
        gnss_sim::SimulatorRunSummary summary{};
        std::string error_message;
        ASSERT_TRUE(run_in_directory(directory, config, &summary, &error_message))
            << "rate=" << rate << " Hz: " << error_message;

        const std::uint64_t expected_epochs = static_cast<std::uint64_t>(kDurationSeconds * rate);
        EXPECT_EQ(summary.scheduled_epochs, expected_epochs) << "rate=" << rate;
        EXPECT_EQ(summary.powered_epochs, expected_epochs) << "rate=" << rate;
        EXPECT_EQ(summary.signal_on_epochs, expected_epochs) << "rate=" << rate;
        EXPECT_EQ(summary.range_messages, expected_epochs) << "rate=" << rate;
        EXPECT_EQ(summary.psrpos_messages, expected_epochs) << "rate=" << rate;
        EXPECT_EQ(summary.psrvel_messages, expected_epochs) << "rate=" << rate;
        EXPECT_EQ(summary.power_on_events, 1U) << "rate=" << rate;
        EXPECT_GT(summary.max_observations_per_epoch, 0) << "rate=" << rate;
        EXPECT_GT(summary.valid_position_epochs, 0U) << "rate=" << rate;
        EXPECT_GT(summary.valid_velocity_epochs, 0U) << "rate=" << rate;

        for (const char* file_name : {"simulated.log", "scenario.json", "event_truth.csv", "observation_truth.csv",
                                      "solution_truth.csv", "run_manifest.json"}) {
            expect_nonempty_file(directory / file_name);
        }
        const std::string manifest = read_file(directory / "run_manifest.json");
        EXPECT_NE(manifest.find("\"sampling_rate_hz\": " + std::to_string(rate)), std::string::npos) << "rate=" << rate;
        cleanup(directory);
    }
}

TEST(V1Acceptance, SameInputConfigAndSeedProduceByteIdenticalReceiverAndTruthOutputs) {
    gnss_sim::SimConfig config = acceptance_config();
    config.sampling_rate_hz = 10;
    config.duration_ns = 4LL * gnss_sim::NANOSECONDS_PER_SECOND;

    const std::filesystem::path first_directory = "gnss_sim_acceptance_repeat_a";
    const std::filesystem::path second_directory = "gnss_sim_acceptance_repeat_b";
    gnss_sim::SimulatorRunSummary first_summary{};
    gnss_sim::SimulatorRunSummary second_summary{};
    std::string error_message;
    ASSERT_TRUE(run_in_directory(first_directory, config, &first_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_in_directory(second_directory, config, &second_summary, &error_message)) << error_message;

    constexpr std::array<const char*, 6> kArtifacts = {"simulated.log",      "scenario.json",
                                                       "event_truth.csv",    "observation_truth.csv",
                                                       "solution_truth.csv", "run_manifest.json"};
    for (const char* file_name : kArtifacts) {
        EXPECT_EQ(read_file(first_directory / file_name), read_file(second_directory / file_name)) << file_name;
    }
    EXPECT_EQ(first_summary.scheduled_epochs, second_summary.scheduled_epochs);
    EXPECT_EQ(first_summary.range_messages, second_summary.range_messages);
    EXPECT_EQ(first_summary.nav_messages, second_summary.nav_messages);
    EXPECT_EQ(first_summary.valid_position_epochs, second_summary.valid_position_epochs);
    EXPECT_EQ(first_summary.valid_velocity_epochs, second_summary.valid_velocity_epochs);
    EXPECT_EQ(first_summary.max_observations_per_epoch, second_summary.max_observations_per_epoch);

    cleanup(first_directory);
    cleanup(second_directory);
}

} // namespace
