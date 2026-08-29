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
    std::vector<double> pseudoranges_m;
    std::vector<double> cn0_dbhz;
};

struct AdrLockRecord {
    std::int64_t tow_ns;
    int satellite_number;
    int signal_id;
    std::int64_t ambiguity_cycles;
    std::int64_t ambiguity_epoch_tow_ns;
};

std::string nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/gps_loopback_nav.rnx";
}

gnss_sim::SimTime start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2253, 172900.0, &time));
    return time;
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
                epoch.pseudoranges_m.push_back(std::stod(fields[base + 2U]));
            }
            epoch.cn0_dbhz.push_back(std::stod(fields[base + 7U]));
        }
        epochs.push_back(epoch);
    }
    return epochs;
}

std::vector<AdrLockRecord> read_adr_locks(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    std::string header;
    if (!std::getline(input, header)) {
        return {};
    }
    const std::vector<std::string> names = split_csv(header);
    const auto column = [&names](const char* name) -> std::size_t {
        const auto iterator = std::find(names.begin(), names.end(), name);
        return iterator == names.end() ? names.size() : static_cast<std::size_t>(iterator - names.begin());
    };
    const std::size_t tow_column = column("tow_ns");
    const std::size_t satellite_column = column("satellite_number");
    const std::size_t signal_column = column("signal_id");
    const std::size_t adr_valid_column = column("adr_valid");
    const std::size_t ambiguity_column = column("ambiguity_cycles");
    const std::size_t epoch_column = column("ambiguity_epoch_tow_ns");
    if (tow_column == names.size() || satellite_column == names.size() || signal_column == names.size() ||
        adr_valid_column == names.size() || ambiguity_column == names.size() || epoch_column == names.size()) {
        return {};
    }

    std::vector<AdrLockRecord> records;
    std::string line;
    while (std::getline(input, line)) {
        const std::vector<std::string> fields = split_csv(line);
        if (fields.size() != names.size() || fields[adr_valid_column] != "1") {
            continue;
        }
        AdrLockRecord record{};
        record.tow_ns = std::stoll(fields[tow_column]);
        record.satellite_number = std::stoi(fields[satellite_column]);
        record.signal_id = std::stoi(fields[signal_column]);
        record.ambiguity_cycles = std::stoll(fields[ambiguity_column]);
        record.ambiguity_epoch_tow_ns = std::stoll(fields[epoch_column]);
        records.push_back(record);
    }
    return records;
}

bool run_case(const std::filesystem::path& directory, const gnss_sim::SimConfig& config,
              gnss_sim::SimulatorRunSummary* summary, std::string* error_message) {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    error.clear();
    if (!std::filesystem::create_directories(directory, error) || error) {
        if (error_message != nullptr) {
            *error_message = "cannot create REA fade integration directory";
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

gnss_sim::SimConfig fade_config(int sampling_rate_hz, double fade_duration_sec, std::int64_t duration_sec) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::REA;
    config.rea.signal_on_ns = 6LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_off_ns = 2LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.duration_ns = duration_sec * gnss_sim::NANOSECONDS_PER_SECOND;
    config.sampling_rate_hz = sampling_rate_hz;
    config.elevation_mask_deg = 0.0;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    config.measurement_noise_enabled = true;
    config.measurement_error.psr_sigma_m = 0.0;
    config.measurement_error.doppler_sigma_mps = 0.0;
    config.measurement_error.adr_sigma_m = 0.0;
    config.measurement_error.cn0_sigma_dbhz = 0.0;
    config.measurement_error.rea_reacquisition = {0.0, 0.0, 0.0, 1.0};
    config.measurement_error.rea_fade = {fade_duration_sec, 0.0, 0.0, 5.0};
    config.seed = 211U;
    return config;
}

double first_cn0_difference(const std::vector<RangeEpoch>& baseline, const std::vector<RangeEpoch>& faded,
                            std::size_t epoch) {
    if (epoch >= baseline.size() || epoch >= faded.size() || baseline[epoch].cn0_dbhz.empty() ||
        faded[epoch].cn0_dbhz.empty()) {
        return 0.0;
    }
    return baseline[epoch].cn0_dbhz[0] - faded[epoch].cn0_dbhz[0];
}

TEST(ReaFadeIntegration, HardCutModeDoesNotAddArtificialPreLossRamp) {
    const std::filesystem::path baseline_directory = "gnss_sim_rea_hardcut_off";
    const std::filesystem::path enabled_directory = "gnss_sim_rea_hardcut_on";
    gnss_sim::SimConfig baseline = fade_config(10, 0.0, 8);
    baseline.measurement_noise_enabled = false;
    const gnss_sim::SimConfig enabled = fade_config(10, 0.0, 8);
    gnss_sim::SimulatorRunSummary baseline_summary{};
    gnss_sim::SimulatorRunSummary enabled_summary{};
    std::string error_message;
    ASSERT_TRUE(run_case(baseline_directory, baseline, &baseline_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_case(enabled_directory, enabled, &enabled_summary, &error_message)) << error_message;

    std::ifstream baseline_log(baseline_directory / "simulated.log", std::ios::binary);
    std::ifstream enabled_log(enabled_directory / "simulated.log", std::ios::binary);
    const std::string baseline_text((std::istreambuf_iterator<char>(baseline_log)), std::istreambuf_iterator<char>());
    const std::string enabled_text((std::istreambuf_iterator<char>(enabled_log)), std::istreambuf_iterator<char>());
    EXPECT_EQ(baseline_text, enabled_text);

    const std::vector<RangeEpoch> epochs = read_range_epochs(enabled_directory / "simulated.log");
    ASSERT_EQ(epochs.size(), 80U);
    for (std::size_t epoch = 60U; epoch < 80U; ++epoch) {
        EXPECT_EQ(epochs[epoch].observation_count, 0) << epoch;
    }

    cleanup(baseline_directory);
    cleanup(enabled_directory);
}

TEST(ReaFadeIntegration, FadeProgressUsesPhysicalTimeAtOneTenAndFiftyHertz) {
    for (int rate : {1, 10, 50}) {
        const std::filesystem::path baseline_directory = "gnss_sim_rea_fade_base_" + std::to_string(rate);
        const std::filesystem::path faded_directory = "gnss_sim_rea_fade_on_" + std::to_string(rate);
        const gnss_sim::SimConfig baseline = fade_config(rate, 0.0, 8);
        const gnss_sim::SimConfig faded = fade_config(rate, 2.5, 8);
        gnss_sim::SimulatorRunSummary baseline_summary{};
        gnss_sim::SimulatorRunSummary faded_summary{};
        std::string error_message;
        ASSERT_TRUE(run_case(baseline_directory, baseline, &baseline_summary, &error_message)) << error_message;
        ASSERT_TRUE(run_case(faded_directory, faded, &faded_summary, &error_message)) << error_message;

        const std::vector<RangeEpoch> baseline_epochs = read_range_epochs(baseline_directory / "simulated.log");
        const std::vector<RangeEpoch> faded_epochs = read_range_epochs(faded_directory / "simulated.log");
        ASSERT_EQ(baseline_epochs.size(), static_cast<std::size_t>(8 * rate));
        ASSERT_EQ(faded_epochs.size(), baseline_epochs.size());

        const std::size_t before_fade_epoch = static_cast<std::size_t>(3 * rate);
        EXPECT_NEAR(first_cn0_difference(baseline_epochs, faded_epochs, before_fade_epoch), 0.0, 0.11) << rate;

        const std::size_t common_time_epoch = static_cast<std::size_t>(5 * rate);
        ASSERT_FALSE(baseline_epochs[common_time_epoch].cn0_dbhz.empty()) << rate;
        ASSERT_FALSE(faded_epochs[common_time_epoch].cn0_dbhz.empty()) << rate;
        EXPECT_NEAR(first_cn0_difference(baseline_epochs, faded_epochs, common_time_epoch), 3.0, 0.11) << rate;

        for (std::size_t epoch = static_cast<std::size_t>(6 * rate); epoch < static_cast<std::size_t>(8 * rate);
             ++epoch) {
            EXPECT_EQ(faded_epochs[epoch].observation_count, 0) << rate << ':' << epoch;
        }

        cleanup(baseline_directory);
        cleanup(faded_directory);
    }
}

TEST(ReaFadeIntegration, LossCreatesNewAdrLockAndFadeStateDoesNotLeakAcrossReacquisition) {
    const std::filesystem::path hardcut_directory = "gnss_sim_rea_reset_hardcut";
    const std::filesystem::path faded_directory = "gnss_sim_rea_reset_faded";
    gnss_sim::SimConfig hardcut = fade_config(10, 0.0, 14);
    gnss_sim::SimConfig faded = fade_config(10, 2.5, 14);
    hardcut.measurement_error.psr_sigma_m = 0.08;
    faded.measurement_error.psr_sigma_m = 0.08;
    hardcut.measurement_error.rea_reacquisition = {0.40, 0.0, 0.0, 0.8};
    faded.measurement_error.rea_reacquisition = {0.40, 0.0, 0.0, 0.8};
    faded.measurement_error.rea_fade.psr_extra_sigma_m = 5.0;
    gnss_sim::SimulatorRunSummary hardcut_summary{};
    gnss_sim::SimulatorRunSummary faded_summary{};
    std::string error_message;
    ASSERT_TRUE(run_case(hardcut_directory, hardcut, &hardcut_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_case(faded_directory, faded, &faded_summary, &error_message)) << error_message;

    const std::vector<RangeEpoch> hardcut_epochs = read_range_epochs(hardcut_directory / "simulated.log");
    const std::vector<RangeEpoch> faded_epochs = read_range_epochs(faded_directory / "simulated.log");
    ASSERT_EQ(hardcut_epochs.size(), 140U);
    ASSERT_EQ(faded_epochs.size(), hardcut_epochs.size());

    bool saw_pre_loss_difference = false;
    for (std::size_t epoch = 35U; epoch < 60U; ++epoch) {
        const std::size_t count =
            std::min(hardcut_epochs[epoch].pseudoranges_m.size(), faded_epochs[epoch].pseudoranges_m.size());
        for (std::size_t index = 0; index < count; ++index) {
            if (std::fabs(hardcut_epochs[epoch].pseudoranges_m[index] - faded_epochs[epoch].pseudoranges_m[index]) >=
                0.001) {
                saw_pre_loss_difference = true;
                break;
            }
        }
        if (saw_pre_loss_difference) {
            break;
        }
    }
    EXPECT_TRUE(saw_pre_loss_difference);

    std::size_t first_post_reacquisition = hardcut_epochs.size();
    for (std::size_t epoch = 80U; epoch < hardcut_epochs.size(); ++epoch) {
        if (!hardcut_epochs[epoch].pseudoranges_m.empty() && !faded_epochs[epoch].pseudoranges_m.empty()) {
            first_post_reacquisition = epoch;
            break;
        }
    }
    ASSERT_LT(first_post_reacquisition, hardcut_epochs.size());
    for (std::size_t epoch = first_post_reacquisition;
         epoch < std::min(first_post_reacquisition + 5U, hardcut_epochs.size()); ++epoch) {
        EXPECT_EQ(hardcut_epochs[epoch].pseudoranges_m, faded_epochs[epoch].pseudoranges_m) << epoch;
    }

    const std::vector<AdrLockRecord> locks = read_adr_locks(faded_directory / "observation_truth.csv");
    ASSERT_FALSE(locks.empty());
    const std::int64_t start_tow_ns = start_time().tow_ns;
    const std::int64_t first_off_tow_ns = start_tow_ns + 6LL * gnss_sim::NANOSECONDS_PER_SECOND;
    const std::int64_t second_on_tow_ns = start_tow_ns + 8LL * gnss_sim::NANOSECONDS_PER_SECOND;
    bool found_reset = false;
    for (const AdrLockRecord& before : locks) {
        if (before.tow_ns >= first_off_tow_ns) {
            continue;
        }
        for (const AdrLockRecord& after : locks) {
            if (after.tow_ns < second_on_tow_ns || after.satellite_number != before.satellite_number ||
                after.signal_id != before.signal_id) {
                continue;
            }
            EXPECT_NE(after.ambiguity_epoch_tow_ns, before.ambiguity_epoch_tow_ns);
            EXPECT_NE(after.ambiguity_cycles, before.ambiguity_cycles);
            found_reset = true;
            break;
        }
        if (found_reset) {
            break;
        }
    }
    EXPECT_TRUE(found_reset);

    cleanup(hardcut_directory);
    cleanup(faded_directory);
}

} // namespace
