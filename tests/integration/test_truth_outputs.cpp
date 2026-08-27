#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>

namespace {

std::string nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/gps_loopback_nav.rnx";
}

std::string runtime_cn0_model_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/runtime_cn0_model.csv";
}

gnss_sim::SimTime start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2253, 172900.0, &time));
    return time;
}

gnss_sim::SimConfig truth_config() {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::KS;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    config.elevation_mask_deg = 0.0;
    config.sampling_rate_hz = 5;
    config.duration_ns = 2LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.seed = 7U;
    return config;
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

std::string first_line(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    std::string line;
    std::getline(input, line);
    return line;
}

bool run_in_directory(const std::filesystem::path& directory, gnss_sim::SimulatorRunSummary* summary,
                      std::string* error_message, const char* cn0_model_path = nullptr) {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    error.clear();
    if (!std::filesystem::create_directories(directory, error) || error) {
        if (error_message != nullptr) {
            *error_message = "cannot create truth-output test directory";
        }
        return false;
    }
    const std::filesystem::path output_path = directory / "simulated.log";
    const std::string input_path = nav_path();
    const std::string output_text = output_path.string();
    const gnss_sim::SimulatorRunOptions options{input_path.c_str(), output_text.c_str(), start_time(), cn0_model_path};
    return gnss_sim::run_simulator(truth_config(), options, summary, error_message);
}

void cleanup(const std::filesystem::path& path) {
    std::error_code error;
    std::filesystem::remove_all(path, error);
}

TEST(TruthOutputs, HeadersAreVersionedAndExplicit) {
    const std::filesystem::path directory = "gnss_sim_truth_schema";
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run_in_directory(directory, &summary, &error_message)) << error_message;

    EXPECT_EQ(first_line(directory / "event_truth.csv"),
              "gps_week,tow_ns,sow_sec,event_type,cycle_index,receiver_powered,signal_available,startup_mode");
    EXPECT_EQ(
        first_line(directory / "observation_truth.csv"),
        "gps_week,tow_ns,sow_sec,observation_index,system,prn,satellite_number,signal_id,signal_name,glonass_fcn,"
        "wavelength_m,receiver_x_m,receiver_y_m,receiver_z_m,receiver_vx_mps,receiver_vy_mps,receiver_vz_mps,"
        "transmit_week,transmit_tow_ns,transmit_sow_sec,satellite_x_m,satellite_y_m,satellite_z_m,"
        "satellite_vx_mps,satellite_vy_mps,satellite_vz_mps,azimuth_deg,elevation_deg,geometric_range_m,"
        "range_rate_mps,satellite_clock_bias_m,satellite_clock_drift_mps,receiver_clock_bias_m,"
        "receiver_clock_drift_mps,ionosphere_m,troposphere_m,broadcast_message_family,tgd_sec_0,tgd_sec_1,"
        "tgd_sec_2,tgd_sec_3,isc_sec_0,isc_sec_1,isc_sec_2,isc_sec_3,isc_sec_4,isc_sec_5,glonass_dtaun_sec,"
        "code_bias_m,code_bias_status,cn0_dbhz,tracking_phase,lock_time_ns,pseudorange_valid,doppler_valid,adr_valid,"
        "ambiguity_cycles,ambiguity_epoch_week,ambiguity_epoch_tow_ns,cycle_slip,pseudorange_m,doppler_hz,adr_cycles");
    EXPECT_EQ(first_line(directory / "solution_truth.csv"),
              "gps_week,tow_ns,sow_sec,tracked_satellites,position_valid,position_status,position_type,"
              "latitude_deg,longitude_deg,height_m,position_x_m,position_y_m,position_z_m,latitude_std_m,"
              "longitude_std_m,height_std_m,receiver_clock_bias_m,position_used_satellites,position_diagnostic,"
              "velocity_valid,velocity_status,velocity_type,velocity_x_mps,velocity_y_mps,velocity_z_mps,"
              "horizontal_speed_mps,track_over_ground_deg,vertical_speed_mps,receiver_clock_drift_mps,"
              "velocity_used_satellites,velocity_diagnostic");

    const std::string scenario = read_file(directory / "scenario.json");
    const std::string manifest = read_file(directory / "run_manifest.json");
    const std::string observations = read_file(directory / "observation_truth.csv");
    EXPECT_NE(scenario.find("\"truth_schema_version\": 1"), std::string::npos);
    EXPECT_NE(scenario.find("\"atmosphere_mode\": \"none\""), std::string::npos);
    EXPECT_NE(manifest.find("\"output_format_version\": 1"), std::string::npos);
    EXPECT_NE(manifest.find("\"rtklib_commit_sha\": \"dc596ba725ccaa5ab5963d7e7ec85b52ae743969\""), std::string::npos);
    EXPECT_NE(manifest.find("\"hash_algorithm\": \"fnv1a64\""), std::string::npos);
    EXPECT_NE(manifest.find("\"random_seed\": 7"), std::string::npos);
    EXPECT_NE(manifest.find("\"source\": \"BUILTIN_FALLBACK\""), std::string::npos);
    EXPECT_NE(manifest.find("\"schema_version\": \"builtin-cn0-v1\""), std::string::npos);
    EXPECT_NE(manifest.find("\"hash_algorithm\": \"none\""), std::string::npos);
    EXPECT_EQ(summary.cn0_model_source, "BUILTIN_FALLBACK");
    EXPECT_NE(read_file(directory / "event_truth.csv").find("POWER_ON"), std::string::npos);
    EXPECT_NE(observations.find(",LEGACY,"), std::string::npos);
    EXPECT_GT(observations.size(), first_line(directory / "observation_truth.csv").size());
    EXPECT_GT(read_file(directory / "solution_truth.csv").size(), first_line(directory / "solution_truth.csv").size());
    cleanup(directory);
}

TEST(TruthOutputs, SameInputConfigAndSeedAreByteIdenticalAcrossOutputDirectories) {
    const std::filesystem::path first_directory = "gnss_sim_truth_repeat_a";
    const std::filesystem::path second_directory = "gnss_sim_truth_repeat_b";
    gnss_sim::SimulatorRunSummary first_summary{};
    gnss_sim::SimulatorRunSummary second_summary{};
    std::string error_message;
    ASSERT_TRUE(run_in_directory(first_directory, &first_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_in_directory(second_directory, &second_summary, &error_message)) << error_message;

    for (const char* file_name :
         {"scenario.json", "event_truth.csv", "observation_truth.csv", "solution_truth.csv", "run_manifest.json"}) {
        EXPECT_EQ(read_file(first_directory / file_name), read_file(second_directory / file_name)) << file_name;
    }
    EXPECT_EQ(first_summary.scheduled_epochs, second_summary.scheduled_epochs);
    EXPECT_EQ(first_summary.max_observations_per_epoch, second_summary.max_observations_per_epoch);
    cleanup(first_directory);
    cleanup(second_directory);
}

TEST(TruthOutputs, ExternalCn0ModelIsManifestedAndByteRepeatable) {
    const std::filesystem::path first_directory = "gnss_sim_truth_cn0_a";
    const std::filesystem::path second_directory = "gnss_sim_truth_cn0_b";
    const std::filesystem::path builtin_directory = "gnss_sim_truth_cn0_builtin";
    const std::string model_path = runtime_cn0_model_path();
    gnss_sim::SimulatorRunSummary first_summary{};
    gnss_sim::SimulatorRunSummary second_summary{};
    gnss_sim::SimulatorRunSummary builtin_summary{};
    std::string error_message;
    ASSERT_TRUE(run_in_directory(first_directory, &first_summary, &error_message, model_path.c_str())) << error_message;
    ASSERT_TRUE(run_in_directory(second_directory, &second_summary, &error_message, model_path.c_str())) << error_message;
    ASSERT_TRUE(run_in_directory(builtin_directory, &builtin_summary, &error_message)) << error_message;

    for (const char* file_name : {"simulated.log", "scenario.json", "event_truth.csv", "observation_truth.csv",
                                  "solution_truth.csv", "run_manifest.json"}) {
        EXPECT_EQ(read_file(first_directory / file_name), read_file(second_directory / file_name)) << file_name;
    }
    const std::string manifest = read_file(first_directory / "run_manifest.json");
    EXPECT_EQ(first_summary.cn0_model_source, "CALIBRATED_CSV");
    EXPECT_EQ(first_summary.cn0_model_schema_version, "gnss-cn0-model-v1");
    EXPECT_EQ(first_summary.cn0_model_name, "runtime_cn0_model.csv");
    EXPECT_FALSE(first_summary.cn0_model_hash.empty());
    EXPECT_GT(first_summary.cn0_model_size_bytes, 0U);
    EXPECT_NE(manifest.find("\"source\": \"CALIBRATED_CSV\""), std::string::npos);
    EXPECT_NE(manifest.find("\"schema_version\": \"gnss-cn0-model-v1\""), std::string::npos);
    EXPECT_NE(manifest.find("\"name\": \"runtime_cn0_model.csv\""), std::string::npos);
    EXPECT_NE(read_file(first_directory / "observation_truth.csv"),
              read_file(builtin_directory / "observation_truth.csv"));

    cleanup(first_directory);
    cleanup(second_directory);
    cleanup(builtin_directory);
}

TEST(TruthOutputs, ExplicitMalformedCn0ModelFailsBeforeReceiverOutputCreation) {
    const std::filesystem::path directory = "gnss_sim_truth_bad_cn0";
    const std::filesystem::path bad_model = "gnss_sim_bad_runtime_cn0.csv";
    {
        std::ofstream output(bad_model, std::ios::binary | std::ios::trunc);
        output << "not-a-cn0-model\n";
    }
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    EXPECT_FALSE(run_in_directory(directory, &summary, &error_message, bad_model.string().c_str()));
    EXPECT_FALSE(error_message.empty());
    EXPECT_FALSE(std::filesystem::exists(directory / "simulated.log"));
    EXPECT_FALSE(std::filesystem::exists(directory / "run_manifest.json"));
    cleanup(directory);
    std::remove(bad_model.string().c_str());
}

} // namespace
