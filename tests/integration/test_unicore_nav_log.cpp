#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <set>
#include <string>
#include <vector>

// nav_log_format = "unicore": the receiver NAV/ION records go through the
// Unicore N4/UT986 writer; every other log line is the NovAtel run's.

namespace {

std::string nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

gnss_sim::SimConfig config_for(gnss_sim::NavLogFormat format) {
    gnss_sim::SimConfig value = gnss_sim::default_sim_config();
    value.scenario = gnss_sim::ScenarioType::KS;
    value.atmosphere_mode = gnss_sim::AtmosphereMode::BROADCAST;
    value.sampling_rate_hz = 1;
    value.duration_ns = 30LL * gnss_sim::NANOSECONDS_PER_SECOND;
    value.receiver = {20.0, 120.0, 100.0};
    value.seed = 0x478U;
    value.nav_log_format = format;
    return value;
}

std::vector<std::string> run_log(gnss_sim::NavLogFormat format, const char* name) {
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "unicore_nav_log_run";
    std::filesystem::create_directories(directory);
    const std::filesystem::path log_path = directory / name;
    gnss_sim::SimTime start{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 436500.0, &start));
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    EXPECT_TRUE(gnss_sim::run_simulator(config_for(format), {nav_path().c_str(), log_path.string().c_str(), start},
                                        &summary, &error_message))
        << error_message;
    EXPECT_GT(summary.nav_messages, 0U);
    std::ifstream log(log_path.string(), std::ios::binary);
    std::vector<std::string> lines;
    for (std::string line; std::getline(log, line);) {
        lines.push_back(line);
    }
    return lines;
}

std::string log_name(const std::string& line) {
    return line.substr(1, line.find(',') - 1);
}

const std::set<std::string> kNovatelNav = {"GPSEPHEMA", "QZSSEPHEMERISA", "GLOEPHEMERISA", "GALEPHEMERISA",
                                           "BD2EPHEMA", "IONUTCA",        "BD2IONUTCA"};
const std::set<std::string> kUnicoreNav = {"GPSEPHA", "GPSCNAVEPHA", "QZSSEPHA", "BDSEPHA", "BD3EPHA", "GLOEPHA",
                                           "GALEPHA", "IRNSSEPHA",   "GPSIONA",  "BDSIONA", "BD3IONA", "GALIONA"};

} // namespace

TEST(UnicoreNavLog, OnlyTheNavRecordsChangeFormat) {
    const std::vector<std::string> novatel = run_log(gnss_sim::NavLogFormat::kNovatel, "novatel.log");
    const std::vector<std::string> unicore = run_log(gnss_sim::NavLogFormat::kUnicore, "unicore.log");
    std::vector<std::string> novatel_other;
    std::vector<std::string> unicore_other;
    std::set<std::string> unicore_names;
    for (const std::string& line : novatel) {
        if (!kNovatelNav.count(log_name(line))) {
            novatel_other.push_back(line);
        }
    }
    for (const std::string& line : unicore) {
        const std::string name = log_name(line);
        EXPECT_FALSE(kNovatelNav.count(name)) << name;
        if (kUnicoreNav.count(name)) {
            unicore_names.insert(name);
        } else {
            unicore_other.push_back(line);
        }
    }
    EXPECT_EQ(unicore_other, novatel_other);
    for (const char* expected : {"GPSEPHA", "GPSCNAVEPHA", "BDSEPHA", "BD3EPHA", "GLOEPHA", "GALEPHA", "GPSIONA"}) {
        EXPECT_TRUE(unicore_names.count(expected)) << expected;
    }
}
