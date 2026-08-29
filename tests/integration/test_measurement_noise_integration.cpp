#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct RangeEpoch {
    int observation_count;
    std::vector<double> valid_pseudoranges_m;
};

std::string nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/gps_loopback_nav.rnx";
}

gnss_sim::SimTime start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2253, 172900.0, &time));
    return time;
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

std::vector<std::string> split_csv(const std::string& text) {
    std::vector<std::string> fields;
    std::stringstream stream(text);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

std::vector<RangeEpoch> read_range_epochs(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    std::vector<RangeEpoch> epochs;
    std::string line;
    while (std::getline(input, line)) {
        if (line.rfind("#RANGEA,", 0) != 0) {
            continue;
        }
        const std::size_t semicolon = line.find(';');
        const std::size_t star = line.rfind('*');
        if (semicolon == std::string::npos || star == std::string::npos || star <= semicolon + 1U) {
            continue;
        }
        const std::vector<std::string> fields = split_csv(line.substr(semicolon + 1U, star - semicolon - 1U));
        if (fields.empty()) {
            continue;
        }
        RangeEpoch epoch{};
        epoch.observation_count = std::stoi(fields[0]);
        if (fields.size() != 1U + static_cast<std::size_t>(epoch.observation_count) * 10U) {
            continue;
        }
        for (int index = 0; index < epoch.observation_count; ++index) {
            const std::size_t base = 1U + static_cast<std::size_t>(index) * 10U;
            const unsigned long status = std::stoul(fields[base + 9U], nullptr, 16);
            if ((status & (1UL << 12U)) != 0UL) {
                epoch.valid_pseudoranges_m.push_back(std::stod(fields[base + 2U]));
            }
        }
        epochs.push_back(epoch);
    }
    return epochs;
}

bool run_case(const std::filesystem::path& directory, const gnss_sim::SimConfig& config,
              gnss_sim::SimulatorRunSummary* summary, std::string* error_message) {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    error.clear();
    if (!std::filesystem::create_directories(directory, error) || error) {
        if (error_message != nullptr) {
            *error_message = "cannot create measurement-noise integration directory";
        }
        return false;
    }
    const std::string input = nav_path();
    const std::string output = (directory / "simulated.log").string();
    const gnss_sim::SimulatorRunOptions options{input.c_str(), output.c_str(), start_time()};
    return gnss_sim::run_simulator(config, options, summary, error_message);
}

void cleanup(const std::filesystem::path& path) {
    std::error_code error;
    std::filesystem::remove_all(path, error);
}

bool any_valid_psr_difference(const std::vector<RangeEpoch>& baseline, const std::vector<RangeEpoch>& noisy,
                              std::size_t begin_epoch, std::size_t end_epoch, double minimum_difference_m) {
    const std::size_t limit = std::min({baseline.size(), noisy.size(), end_epoch});
    for (std::size_t epoch_index = begin_epoch; epoch_index < limit; ++epoch_index) {
        const std::size_t observation_count =
            std::min(baseline[epoch_index].valid_pseudoranges_m.size(), noisy[epoch_index].valid_pseudoranges_m.size());
        for (std::size_t observation_index = 0; observation_index < observation_count; ++observation_index) {
            if (std::fabs(baseline[epoch_index].valid_pseudoranges_m[observation_index] -
                          noisy[epoch_index].valid_pseudoranges_m[observation_index]) >= minimum_difference_m) {
                return true;
            }
        }
    }
    return false;
}

gnss_sim::SimConfig ttff_config(bool noise_enabled) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::TTFF;
    config.ttff.startup_mode = gnss_sim::StartupMode::HOT;
    config.ttff.power_on_ns = 8LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.ttff.power_off_ns = 1LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.duration_ns = 5LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.sampling_rate_hz = 10;
    config.elevation_mask_deg = 0.0;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    config.measurement_noise_enabled = noise_enabled;
    config.measurement_error.psr_sigma_m = 0.0;
    config.measurement_error.doppler_sigma_mps = 0.0;
    config.measurement_error.adr_sigma_m = 0.0;
    config.measurement_error.cn0_sigma_dbhz = 0.0;
    config.measurement_error.ttff_hot = {5.0, 0.0, 0.0, 10.0};
    config.seed = 117U;
    return config;
}

gnss_sim::SimConfig rea_config(bool noise_enabled) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::REA;
    config.rea.signal_on_ns = 4LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_off_ns = 1LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.duration_ns = 9LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.sampling_rate_hz = 10;
    config.elevation_mask_deg = 0.0;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    config.measurement_noise_enabled = noise_enabled;
    config.measurement_error.psr_sigma_m = 0.0;
    config.measurement_error.doppler_sigma_mps = 0.0;
    config.measurement_error.adr_sigma_m = 0.0;
    config.measurement_error.cn0_sigma_dbhz = 0.0;
    config.measurement_error.rea_reacquisition = {5.0, 0.0, 0.0, 10.0};
    config.seed = 117U;
    return config;
}

TEST(MeasurementNoiseIntegration, TtffNoiseChangesReportedRangeButNotZeroNoiseTruthOrAcquisition) {
    const std::filesystem::path baseline_directory = "gnss_sim_ttff_noise_off";
    const std::filesystem::path noisy_directory = "gnss_sim_ttff_noise_on";
    gnss_sim::SimulatorRunSummary baseline_summary{};
    gnss_sim::SimulatorRunSummary noisy_summary{};
    std::string error_message;
    ASSERT_TRUE(run_case(baseline_directory, ttff_config(false), &baseline_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_case(noisy_directory, ttff_config(true), &noisy_summary, &error_message)) << error_message;

    EXPECT_EQ(read_file(baseline_directory / "event_truth.csv"), read_file(noisy_directory / "event_truth.csv"));
    EXPECT_EQ(read_file(baseline_directory / "observation_truth.csv"),
              read_file(noisy_directory / "observation_truth.csv"));

    const std::vector<RangeEpoch> baseline = read_range_epochs(baseline_directory / "simulated.log");
    const std::vector<RangeEpoch> noisy = read_range_epochs(noisy_directory / "simulated.log");
    ASSERT_EQ(baseline.size(), noisy.size());
    ASSERT_FALSE(baseline.empty());
    EXPECT_TRUE(any_valid_psr_difference(baseline, noisy, 0U, noisy.size(), 0.001));

    const std::string manifest = read_file(noisy_directory / "run_manifest.json");
    EXPECT_NE(manifest.find("\"measurement_noise_enabled\": true"), std::string::npos);
    EXPECT_NE(manifest.find("\"measurement_error\": {"), std::string::npos);
    EXPECT_NE(manifest.find("\"psr_sigma_m\": 0"), std::string::npos);
    EXPECT_NE(manifest.find("\"psr_extra_sigma_m\": 5"), std::string::npos);

    cleanup(baseline_directory);
    cleanup(noisy_directory);
}

TEST(MeasurementNoiseIntegration, ReaReacquisitionNoiseStartsOnlyAfterSignalReturnsAndOffWindowIsEmpty) {
    const std::filesystem::path baseline_directory = "gnss_sim_rea_noise_off";
    const std::filesystem::path noisy_directory = "gnss_sim_rea_noise_on";
    gnss_sim::SimulatorRunSummary baseline_summary{};
    gnss_sim::SimulatorRunSummary noisy_summary{};
    std::string error_message;
    ASSERT_TRUE(run_case(baseline_directory, rea_config(false), &baseline_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_case(noisy_directory, rea_config(true), &noisy_summary, &error_message)) << error_message;

    EXPECT_EQ(read_file(baseline_directory / "event_truth.csv"), read_file(noisy_directory / "event_truth.csv"));
    EXPECT_EQ(read_file(baseline_directory / "observation_truth.csv"),
              read_file(noisy_directory / "observation_truth.csv"));

    const std::vector<RangeEpoch> baseline = read_range_epochs(baseline_directory / "simulated.log");
    const std::vector<RangeEpoch> noisy = read_range_epochs(noisy_directory / "simulated.log");
    ASSERT_EQ(baseline.size(), 90U);
    ASSERT_EQ(noisy.size(), baseline.size());

    for (std::size_t epoch = 40U; epoch < 50U; ++epoch) {
        EXPECT_EQ(baseline[epoch].observation_count, 0) << epoch;
        EXPECT_EQ(noisy[epoch].observation_count, 0) << epoch;
    }

    EXPECT_FALSE(any_valid_psr_difference(baseline, noisy, 0U, 40U, 0.001));
    EXPECT_TRUE(any_valid_psr_difference(baseline, noisy, 50U, 90U, 0.001));

    cleanup(baseline_directory);
    cleanup(noisy_directory);
}

} // namespace
