#include "gnss/rtklib_adapter.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"
#include "rangea_roundtrip.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTruthLatitudeDeg = 20.0;
constexpr double kTruthLongitudeDeg = 120.0;
constexpr double kTruthHeightM = 100.0;
constexpr std::int64_t kPrimaryDurationSeconds = 180;
constexpr const char* kFixtureSha256 = "17c6bb00a8a0ef8f732b803925311e7b1ead658ae2e11f62635371eb915e9781";

std::string brd4_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

gnss_sim::SimTime brd4_start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 436500.0, &time));
    return time;
}

gnss_sim::SimConfig urban_config() {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::KS;
    config.duration_ns = kPrimaryDurationSeconds * gnss_sim::NANOSECONDS_PER_SECOND;
    config.sampling_rate_hz = 1;
    config.elevation_mask_deg = 0.0;
    config.solution_elevation_mask_deg = 5.0;
    config.output_eph = true;
    config.output_ion = true;
    config.measurement_noise_enabled = false;
    config.multipath_enabled = true;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    config.receiver = {kTruthLatitudeDeg, kTruthLongitudeDeg, kTruthHeightM};
    config.seed = 0x124U;
    return config;
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

std::vector<std::string> parse_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for (std::size_t index = 0; index < line.size(); ++index) {
        const char character = line[index];
        if (quoted) {
            if (character == '"' && index + 1U < line.size() && line[index + 1U] == '"') {
                field.push_back('"');
                ++index;
            } else if (character == '"') {
                quoted = false;
            } else {
                field.push_back(character);
            }
        } else if (character == '"') {
            quoted = true;
        } else if (character == ',') {
            fields.push_back(field);
            field.clear();
        } else {
            field.push_back(character);
        }
    }
    fields.push_back(field);
    return fields;
}

std::vector<std::vector<std::string>> read_csv(const std::filesystem::path& path) {
    std::vector<std::vector<std::string>> rows;
    std::ifstream input(path, std::ios::binary);
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        rows.push_back(parse_csv_line(line));
    }
    return rows;
}

int column_index(const std::vector<std::string>& header, const char* name) {
    for (std::size_t index = 0; index < header.size(); ++index) {
        if (header[index] == name) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

std::string epoch_key(const std::string& week, const std::string& tow_ns) {
    return week + ":" + tow_ns;
}

struct EpochStateCounts {
    std::uint64_t los = 0;
    std::uint64_t los_multipath = 0;
    std::uint64_t nlos_tracked = 0;
    std::uint64_t blocked = 0;
};

struct UrbanStats {
    std::uint64_t los = 0;
    std::uint64_t los_multipath = 0;
    std::uint64_t nlos_tracked = 0;
    std::uint64_t blocked = 0;
    std::uint64_t blocked_propagation_evaluated = 0;
    std::uint64_t blocked_without_observation = 0;
    std::uint64_t los_multipath_roof_edge = 0;
    std::uint64_t nlos_tracked_indirect = 0;
    std::uint64_t direct_los_with_reflection = 0;
    std::uint64_t reacquisition_events = 0;
    std::uint64_t cycle_slips = 0;
    std::uint64_t reflection_paths = 0;
    std::uint64_t direct_roof_paths = 0;
    std::uint64_t propagation_evaluated_rows = 0;
    std::set<std::string> signal_names;
    std::map<std::string, EpochStateCounts> epoch_states;
};

bool parse_urban_stats(const std::filesystem::path& signal_path, const std::filesystem::path& path_path,
                       UrbanStats* stats) {
    if (stats == nullptr) {
        return false;
    }
    const std::vector<std::vector<std::string>> rows = read_csv(signal_path);
    if (rows.size() <= 1U) {
        return false;
    }
    const std::vector<std::string>& header = rows.front();
    const int week = column_index(header, "gps_week");
    const int tow = column_index(header, "tow_ns");
    const int signal_name = column_index(header, "signal_name");
    const int propagation = column_index(header, "propagation_evaluated");
    const int direct_los = column_index(header, "direct_los");
    const int diffraction = column_index(header, "diffraction_status");
    const int reflection_count = column_index(header, "reflection_count");
    const int state = column_index(header, "urban_state");
    const int reacquisition = column_index(header, "reacquisition_event");
    const int cycle_slip = column_index(header, "cycle_slip_event");
    const int emitted = column_index(header, "receiver_observation_emitted");
    if (week < 0 || tow < 0 || signal_name < 0 || propagation < 0 || direct_los < 0 || diffraction < 0 ||
        reflection_count < 0 || state < 0 || reacquisition < 0 || cycle_slip < 0 || emitted < 0) {
        return false;
    }

    UrbanStats result{};
    for (std::size_t row_index = 1; row_index < rows.size(); ++row_index) {
        const std::vector<std::string>& row = rows[row_index];
        if (row.size() != header.size()) {
            return false;
        }
        result.signal_names.insert(row[static_cast<std::size_t>(signal_name)]);
        const bool evaluated = row[static_cast<std::size_t>(propagation)] == "1";
        const bool has_direct = row[static_cast<std::size_t>(direct_los)] == "1";
        const int reflections = std::stoi(row[static_cast<std::size_t>(reflection_count)]);
        const std::string& current_state = row[static_cast<std::size_t>(state)];
        EpochStateCounts& epoch =
            result.epoch_states[epoch_key(row[static_cast<std::size_t>(week)], row[static_cast<std::size_t>(tow)])];
        if (evaluated) {
            ++result.propagation_evaluated_rows;
        }
        if (current_state == "LOS") {
            ++result.los;
            ++epoch.los;
        } else if (current_state == "LOS_MULTIPATH") {
            ++result.los_multipath;
            ++epoch.los_multipath;
            if (evaluated && has_direct && row[static_cast<std::size_t>(diffraction)] == "VALID") {
                ++result.los_multipath_roof_edge;
            }
        } else if (current_state == "NLOS_TRACKED") {
            ++result.nlos_tracked;
            ++epoch.nlos_tracked;
            if (evaluated && !has_direct) {
                ++result.nlos_tracked_indirect;
            }
        } else if (current_state == "BLOCKED") {
            ++result.blocked;
            ++epoch.blocked;
            if (evaluated) {
                ++result.blocked_propagation_evaluated;
            }
            if (evaluated && row[static_cast<std::size_t>(emitted)] == "0") {
                ++result.blocked_without_observation;
            }
        }
        if (evaluated && has_direct && reflections > 0) {
            ++result.direct_los_with_reflection;
        }
        if (row[static_cast<std::size_t>(reacquisition)] == "1") {
            ++result.reacquisition_events;
        }
        if (row[static_cast<std::size_t>(cycle_slip)] == "1") {
            ++result.cycle_slips;
        }
    }

    const std::vector<std::vector<std::string>> path_rows = read_csv(path_path);
    if (path_rows.empty()) {
        return false;
    }
    const int path_kind = column_index(path_rows.front(), "path_kind");
    if (path_kind < 0) {
        return false;
    }
    for (std::size_t row_index = 1; row_index < path_rows.size(); ++row_index) {
        if (path_rows[row_index].size() != path_rows.front().size()) {
            return false;
        }
        const std::string& kind = path_rows[row_index][static_cast<std::size_t>(path_kind)];
        if (kind == "DIRECT_ROOF") {
            ++result.direct_roof_paths;
        } else if (kind == "REFLECTION") {
            ++result.reflection_paths;
        }
    }
    *stats = result;
    return true;
}

struct SolutionStats {
    std::uint64_t valid_position_epochs = 0;
    std::uint64_t valid_velocity_epochs = 0;
    double max_horizontal_error_m = 0.0;
    double max_vertical_error_m = 0.0;
    double max_position_3d_error_m = 0.0;
    double max_velocity_error_mps = 0.0;
    std::string max_position_epoch;
    std::string max_velocity_epoch;
};

bool parse_solution_stats(const std::filesystem::path& path, SolutionStats* stats) {
    if (stats == nullptr) {
        return false;
    }
    const std::vector<std::vector<std::string>> rows = read_csv(path);
    if (rows.size() <= 1U) {
        return false;
    }
    const std::vector<std::string>& header = rows.front();
    const int week = column_index(header, "gps_week");
    const int tow = column_index(header, "tow_ns");
    const int position_valid = column_index(header, "position_valid");
    const int x = column_index(header, "position_x_m");
    const int y = column_index(header, "position_y_m");
    const int z = column_index(header, "position_z_m");
    const int velocity_valid = column_index(header, "velocity_valid");
    const int vx = column_index(header, "velocity_x_mps");
    const int vy = column_index(header, "velocity_y_mps");
    const int vz = column_index(header, "velocity_z_mps");
    if (week < 0 || tow < 0 || position_valid < 0 || x < 0 || y < 0 || z < 0 || velocity_valid < 0 || vx < 0 ||
        vy < 0 || vz < 0) {
        return false;
    }

    double truth_ecef[3]{};
    if (!gnss_sim::rtklib_llh_to_ecef(kTruthLatitudeDeg, kTruthLongitudeDeg, kTruthHeightM, truth_ecef)) {
        return false;
    }
    const double latitude = kTruthLatitudeDeg * kPi / 180.0;
    const double longitude = kTruthLongitudeDeg * kPi / 180.0;
    const double sin_latitude = std::sin(latitude);
    const double cos_latitude = std::cos(latitude);
    const double sin_longitude = std::sin(longitude);
    const double cos_longitude = std::cos(longitude);
    const double east[3] = {-sin_longitude, cos_longitude, 0.0};
    const double north[3] = {-sin_latitude * cos_longitude, -sin_latitude * sin_longitude, cos_latitude};
    const double up[3] = {cos_latitude * cos_longitude, cos_latitude * sin_longitude, sin_latitude};

    SolutionStats result{};
    for (std::size_t row_index = 1; row_index < rows.size(); ++row_index) {
        const std::vector<std::string>& row = rows[row_index];
        if (row.size() != header.size()) {
            return false;
        }
        const std::string key = epoch_key(row[static_cast<std::size_t>(week)], row[static_cast<std::size_t>(tow)]);
        if (row[static_cast<std::size_t>(position_valid)] == "1") {
            const double delta[3] = {std::stod(row[static_cast<std::size_t>(x)]) - truth_ecef[0],
                                     std::stod(row[static_cast<std::size_t>(y)]) - truth_ecef[1],
                                     std::stod(row[static_cast<std::size_t>(z)]) - truth_ecef[2]};
            const double east_error = delta[0] * east[0] + delta[1] * east[1] + delta[2] * east[2];
            const double north_error = delta[0] * north[0] + delta[1] * north[1] + delta[2] * north[2];
            const double up_error = delta[0] * up[0] + delta[1] * up[1] + delta[2] * up[2];
            const double horizontal_error = std::hypot(east_error, north_error);
            const double position_error = std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
            ++result.valid_position_epochs;
            result.max_horizontal_error_m = std::max(result.max_horizontal_error_m, horizontal_error);
            result.max_vertical_error_m = std::max(result.max_vertical_error_m, std::abs(up_error));
            if (position_error >= result.max_position_3d_error_m) {
                result.max_position_3d_error_m = position_error;
                result.max_position_epoch = key;
            }
        }
        if (row[static_cast<std::size_t>(velocity_valid)] == "1") {
            const double velocity_error =
                std::sqrt(std::pow(std::stod(row[static_cast<std::size_t>(vx)]), 2.0) +
                          std::pow(std::stod(row[static_cast<std::size_t>(vy)]), 2.0) +
                          std::pow(std::stod(row[static_cast<std::size_t>(vz)]), 2.0));
            ++result.valid_velocity_epochs;
            if (velocity_error >= result.max_velocity_error_mps) {
                result.max_velocity_error_mps = velocity_error;
                result.max_velocity_epoch = key;
            }
        }
    }
    *stats = result;
    return true;
}

bool run_simulator_in_directory(const std::filesystem::path& directory, const gnss_sim::SimConfig& config,
                                gnss_sim::SimulatorRunSummary* summary, std::string* error_message) {
    std::error_code filesystem_error;
    std::filesystem::remove_all(directory, filesystem_error);
    filesystem_error.clear();
    if (!std::filesystem::create_directories(directory, filesystem_error) || filesystem_error) {
        if (error_message != nullptr) {
            *error_message = "cannot create urban E2E validation directory";
        }
        return false;
    }
    const std::string input_path = brd4_nav_path();
    const std::string output_path = (directory / "simulated.log").string();
    const gnss_sim::SimulatorRunOptions options{input_path.c_str(), output_path.c_str(), brd4_start_time()};
    return gnss_sim::run_simulator(config, options, summary, error_message);
}

void cleanup(const std::filesystem::path& path) {
    std::error_code error;
    std::filesystem::remove_all(path, error);
}

TEST(UrbanE2EValidation, AuthenticBrd4UrbanRunIsDeterministicTraceableAndRtklibConsumable) {
    const std::filesystem::path first_directory = "gnss_sim_urban_e2e_a";
    const std::filesystem::path second_directory = "gnss_sim_urban_e2e_b";
    const gnss_sim::SimConfig config = urban_config();
    gnss_sim::SimulatorRunSummary first_summary{};
    gnss_sim::SimulatorRunSummary second_summary{};
    std::string error_message;
    ASSERT_TRUE(run_simulator_in_directory(first_directory, config, &first_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_simulator_in_directory(second_directory, config, &second_summary, &error_message)) << error_message;

    for (const char* file_name : {"simulated.log", "scenario.json", "observation_truth.csv", "solution_truth.csv",
                                  "urban_signal_truth.csv", "urban_path_truth.csv", "run_manifest.json"}) {
        EXPECT_EQ(read_file(first_directory / file_name), read_file(second_directory / file_name)) << file_name;
    }

    UrbanStats urban{};
    ASSERT_TRUE(parse_urban_stats(first_directory / "urban_signal_truth.csv", first_directory / "urban_path_truth.csv",
                                  &urban));
    SolutionStats solution{};
    ASSERT_TRUE(parse_solution_stats(first_directory / "solution_truth.csv", &solution));

    gnss_sim::RangeaRoundtripSummary original_nav_roundtrip{};
    ASSERT_TRUE(gnss_sim::validate_rangea_roundtrip_file(
        (first_directory / "simulated.log").string().c_str(), brd4_nav_path().c_str(), kTruthLatitudeDeg,
        kTruthLongitudeDeg, kTruthHeightM, config.solution_elevation_mask_deg, false, &original_nav_roundtrip,
        &error_message))
        << error_message;
    gnss_sim::SerializedNavRoundtripSummary serialized_nav_roundtrip{};
    ASSERT_TRUE(gnss_sim::validate_serialized_navigation_roundtrip_file(
        (first_directory / "simulated.log").string().c_str(), kTruthLatitudeDeg, kTruthLongitudeDeg, kTruthHeightM,
        config.solution_elevation_mask_deg, false, &serialized_nav_roundtrip, &error_message))
        << error_message;

    const auto max_position_epoch = urban.epoch_states.find(solution.max_position_epoch);
    ASSERT_NE(max_position_epoch, urban.epoch_states.end());

    std::cout << "URBAN_E2E_NAV_SOURCE=BRD400DLR_S_20250030000_01D_MN.rnx\n"
              << "URBAN_E2E_NAV_FIXTURE_SHA256=" << kFixtureSha256 << '\n'
              << "URBAN_E2E_CONFIG=start_gpst_2347_436500,duration_s=" << kPrimaryDurationSeconds
              << ",rate_hz=1,receiver=20_120_100,seed=0x124,noise=off,multipath=on\n"
              << "URBAN_E2E_STATE_COUNTS LOS=" << urban.los << " LOS_MULTIPATH=" << urban.los_multipath
              << " NLOS_TRACKED=" << urban.nlos_tracked << " BLOCKED=" << urban.blocked
              << " BLOCKED_EVALUATED=" << urban.blocked_propagation_evaluated << '\n'
              << "URBAN_E2E_PATH_COUNTS DIRECT_ROOF=" << urban.direct_roof_paths
              << " REFLECTION=" << urban.reflection_paths
              << " DIRECT_LOS_WITH_REFLECTION=" << urban.direct_los_with_reflection << '\n'
              << "URBAN_E2E_TRANSITIONS reacquisition=" << urban.reacquisition_events
              << " cycle_slip=" << urban.cycle_slips << '\n'
              << "URBAN_E2E_INTERNAL_RTKLIB valid_position_epochs=" << solution.valid_position_epochs
              << " max_horizontal_m=" << solution.max_horizontal_error_m
              << " max_vertical_m=" << solution.max_vertical_error_m
              << " max_3d_m=" << solution.max_position_3d_error_m
              << " valid_velocity_epochs=" << solution.valid_velocity_epochs
              << " max_velocity_mps=" << solution.max_velocity_error_mps << '\n'
              << "URBAN_E2E_RANGEA_ORIGINAL_NAV valid_position_epochs=" << original_nav_roundtrip.valid_position_epochs
              << " max_3d_m=" << original_nav_roundtrip.max_position_error_m << '\n'
              << "URBAN_E2E_RANGEA_SERIALIZED_NAV valid_position_epochs="
              << serialized_nav_roundtrip.position.valid_position_epochs
              << " max_3d_m=" << serialized_nav_roundtrip.position.max_position_error_m << '\n'
              << "URBAN_E2E_MAX_POSITION_EPOCH=" << solution.max_position_epoch
              << " LOS=" << max_position_epoch->second.los
              << " LOS_MULTIPATH=" << max_position_epoch->second.los_multipath
              << " NLOS_TRACKED=" << max_position_epoch->second.nlos_tracked
              << " BLOCKED=" << max_position_epoch->second.blocked << '\n';

    EXPECT_EQ(first_summary.scheduled_epochs, static_cast<std::uint64_t>(kPrimaryDurationSeconds));
    EXPECT_GT(first_summary.valid_position_epochs, 0U);
    EXPECT_GT(first_summary.valid_velocity_epochs, 0U);
    EXPECT_EQ(solution.valid_position_epochs, first_summary.valid_position_epochs);
    EXPECT_EQ(solution.valid_velocity_epochs, first_summary.valid_velocity_epochs);
    EXPECT_GT(urban.signal_names.size(), 1U);
    EXPECT_GT(urban.los, 0U);
    EXPECT_GT(urban.los_multipath, 0U);
    EXPECT_GT(urban.nlos_tracked, 0U);
    EXPECT_GT(urban.blocked_propagation_evaluated, 0U);
    EXPECT_GT(urban.blocked_without_observation, 0U);
    EXPECT_GT(urban.los_multipath_roof_edge, 0U);
    EXPECT_GT(urban.nlos_tracked_indirect, 0U);
    EXPECT_GT(urban.reflection_paths, 0U);
    EXPECT_EQ(urban.direct_los_with_reflection, 0U);
    EXPECT_EQ(urban.direct_roof_paths, urban.propagation_evaluated_rows);
    EXPECT_GT(original_nav_roundtrip.valid_position_epochs, 0U);
    EXPECT_GT(serialized_nav_roundtrip.position.valid_position_epochs, 0U);
    EXPECT_TRUE(std::isfinite(solution.max_horizontal_error_m));
    EXPECT_TRUE(std::isfinite(solution.max_vertical_error_m));
    EXPECT_TRUE(std::isfinite(solution.max_position_3d_error_m));
    EXPECT_TRUE(std::isfinite(solution.max_velocity_error_mps));

    cleanup(first_directory);
    cleanup(second_directory);
}

TEST(UrbanE2EValidation, AuthenticBrd4ReaExercisesReacquisitionAndLockReset) {
    const std::filesystem::path directory = "gnss_sim_urban_e2e_rea";
    gnss_sim::SimConfig config = urban_config();
    config.scenario = gnss_sim::ScenarioType::REA;
    config.duration_ns = 50LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_on_ns = 20LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_off_ns = 5LL * gnss_sim::NANOSECONDS_PER_SECOND;

    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run_simulator_in_directory(directory, config, &summary, &error_message)) << error_message;
    UrbanStats urban{};
    ASSERT_TRUE(parse_urban_stats(directory / "urban_signal_truth.csv", directory / "urban_path_truth.csv", &urban));

    const std::string events = read_file(directory / "event_truth.csv");
    EXPECT_NE(events.find("SIGNAL_OFF"), std::string::npos);
    EXPECT_NE(events.find("SIGNAL_ON"), std::string::npos);
    EXPECT_GT(urban.reacquisition_events, 0U);
    EXPECT_GT(summary.signal_off_epochs, 0U);
    EXPECT_GT(summary.signal_on_epochs, 0U);

    std::cout << "URBAN_E2E_REA reacquisition_events=" << urban.reacquisition_events
              << " cycle_slips=" << urban.cycle_slips << " signal_off_epochs=" << summary.signal_off_epochs
              << " signal_on_epochs=" << summary.signal_on_epochs << '\n';
    cleanup(directory);
}

} // namespace
