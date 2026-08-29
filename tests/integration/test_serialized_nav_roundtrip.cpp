#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"
#include "rangea_roundtrip.h"
#include "serialized_nav_roundtrip.h"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string nav_path() {
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
    value.solution_elevation_mask_deg = 5.0;
    value.receiver = {20.0, 120.0, 100.0};
    value.measurement_noise_enabled = false;
    value.multipath_enabled = false;
    value.output_eph = true;
    value.output_ion = true;
    value.seed = 0x81U;
    return value;
}

bool run_case(const std::filesystem::path& directory, std::string* error_message) {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    error.clear();
    if (!std::filesystem::create_directories(directory, error) || error) {
        if (error_message != nullptr) {
            *error_message = error.message();
        }
        return false;
    }
    const std::string log = (directory / "simulated.log").string();
    const std::string nav = nav_path();
    const gnss_sim::SimulatorRunOptions options{nav.c_str(), log.c_str(), start_time()};
    gnss_sim::SimulatorRunSummary summary{};
    return gnss_sim::run_simulator(config(), options, &summary, error_message);
}

std::vector<std::string> read_lines(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        lines.push_back(line);
    }
    return lines;
}

void cleanup(const std::filesystem::path& path) {
    std::error_code error;
    std::filesystem::remove_all(path, error);
}

TEST(SerializedNavRoundtripIntegration, GeneratedNovatelLogPositionsWithoutOriginalRinexNav) {
    const std::filesystem::path directory = "gnss_sim_serialized_nav_roundtrip";
    std::string error_message;
    ASSERT_TRUE(run_case(directory, &error_message)) << error_message;

    gnss_sim::SerializedNavRoundtripSummary summary{};
    const std::string log = (directory / "simulated.log").string();
    ASSERT_TRUE(gnss_sim::validate_serialized_nav_roundtrip_file(log.c_str(), 20.0, 120.0, 100.0, 5.0, true, &summary,
                                                                 &error_message))
        << error_message;
    EXPECT_GT(summary.gps_ephemeris_records, 0U);
    EXPECT_GT(summary.glonass_ephemeris_records, 0U);
    EXPECT_GT(summary.galileo_ephemeris_records, 0U);
    EXPECT_GT(summary.beidou_ephemeris_records, 0U);
    EXPECT_GT(summary.qzss_ephemeris_records, 0U);
    EXPECT_GT(summary.ionosphere_records, 0U);
    EXPECT_EQ(summary.range_epochs, 60U);
    EXPECT_GT(summary.valid_position_epochs, 0U);
    EXPECT_LT(summary.max_position_error_m, 0.05)
        << "15-digit serialized orbital/clock fields should preserve centimeter-class black-box SPP agreement";
    cleanup(directory);
}

TEST(SerializedNavRoundtripIntegration, FutureNavRecordsCannotBeUsedBeforeTheyAppear) {
    const std::filesystem::path directory = "gnss_sim_serialized_nav_ordering";
    std::string error_message;
    ASSERT_TRUE(run_case(directory, &error_message)) << error_message;
    const std::vector<std::string> lines = read_lines(directory / "simulated.log");

    std::string moved_range;
    gnss_sim::ParsedRangeEpoch moved_epoch{};
    for (const std::string& line : lines) {
        if (line.rfind("#RANGEA,", 0) != 0) {
            continue;
        }
        gnss_sim::ParsedRangeEpoch candidate{};
        ASSERT_TRUE(gnss_sim::parse_rangea_line_independent(line, &candidate, &error_message)) << error_message;
        int valid_psr = 0;
        for (const gnss_sim::ParsedRangeObservation& observation : candidate.observations) {
            if (observation.pseudorange_valid) {
                ++valid_psr;
            }
        }
        if (valid_psr >= 4) {
            moved_range = line;
            moved_epoch = candidate;
            break;
        }
    }
    ASSERT_FALSE(moved_range.empty());

    std::ostringstream reordered;
    reordered << moved_range << '\n';
    bool removed_original = false;
    for (const std::string& line : lines) {
        if (!removed_original && line == moved_range) {
            removed_original = true;
            continue;
        }
        reordered << line << '\n';
    }
    std::istringstream input(reordered.str());
    gnss_sim::SerializedNavRoundtripSummary summary{};
    ASSERT_TRUE(gnss_sim::validate_serialized_nav_roundtrip_stream(&input, 20.0, 120.0, 100.0, 5.0, true, &summary,
                                                                   &error_message))
        << error_message;
    EXPECT_GT(summary.valid_position_epochs, 0U);
    EXPECT_TRUE(summary.first_valid_position_gps_week > moved_epoch.gps_week ||
                (summary.first_valid_position_gps_week == moved_epoch.gps_week &&
                 summary.first_valid_position_sow_sec > moved_epoch.sow_sec));
    cleanup(directory);
}

TEST(SerializedNavRoundtripIntegration, CorruptedSerializedEphemerisCrcFailsExplicitly) {
    const std::filesystem::path directory = "gnss_sim_serialized_nav_crc";
    std::string error_message;
    ASSERT_TRUE(run_case(directory, &error_message)) << error_message;
    std::vector<std::string> lines = read_lines(directory / "simulated.log");
    bool corrupted = false;
    std::ostringstream stream_text;
    for (std::string line : lines) {
        if (!corrupted && line.rfind("#GPSEPHEMA,", 0) == 0) {
            const std::size_t star = line.rfind('*');
            ASSERT_NE(star, std::string::npos);
            line.replace(star + 1U, 8U, "00000000");
            corrupted = true;
        }
        stream_text << line << '\n';
    }
    ASSERT_TRUE(corrupted);
    std::istringstream input(stream_text.str());
    gnss_sim::SerializedNavRoundtripSummary summary{};
    EXPECT_FALSE(gnss_sim::validate_serialized_nav_roundtrip_stream(&input, 20.0, 120.0, 100.0, 5.0, true, &summary,
                                                                    &error_message));
    EXPECT_NE(error_message.find("CRC mismatch"), std::string::npos) << error_message;
    cleanup(directory);
}

} // namespace
