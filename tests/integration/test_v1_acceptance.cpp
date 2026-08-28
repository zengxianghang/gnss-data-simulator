#include "gnss/rtklib_adapter.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/gps_loopback_nav.rnx";
}

std::string multi_gnss_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/multi_gnss_acceptance_nav.rnx";
}

std::string brd4_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

gnss_sim::SimTime acceptance_start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2253, 172900.0, &time));
    return time;
}

gnss_sim::SimTime multi_gnss_start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2253, 173700.0, &time));
    return time;
}

gnss_sim::SimTime brd4_start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 436500.0, &time));
    return time;
}

gnss_sim::SimTime brd4_gps_cnav_start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 437100.0, &time));
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

bool run_in_directory_with_nav(const std::filesystem::path& directory, const gnss_sim::SimConfig& config,
                               const std::string& input_path, const gnss_sim::SimTime& start_time,
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
    const std::string output_text = output_path.string();
    const gnss_sim::SimulatorRunOptions options{input_path.c_str(), output_text.c_str(), start_time};
    return gnss_sim::run_simulator(config, options, summary, error_message);
}

bool run_in_directory(const std::filesystem::path& directory, const gnss_sim::SimConfig& config,
                      gnss_sim::SimulatorRunSummary* summary, std::string* error_message) {
    return run_in_directory_with_nav(directory, config, nav_path(), acceptance_start_time(), summary, error_message);
}

void expect_nonempty_file(const std::filesystem::path& path) {
    std::error_code error;
    ASSERT_TRUE(std::filesystem::exists(path, error)) << path.string();
    ASSERT_FALSE(error) << error.message();
    EXPECT_GT(std::filesystem::file_size(path, error), 0U) << path.string();
    EXPECT_FALSE(error) << error.message();
}

void expect_five_system_observations(const std::filesystem::path& truth_path) {
    const std::string observations = read_file(truth_path);
    EXPECT_NE(observations.find(",GPS,"), std::string::npos);
    EXPECT_NE(observations.find(",GLONASS,"), std::string::npos);
    EXPECT_NE(observations.find(",GALILEO,"), std::string::npos);
    EXPECT_NE(observations.find(",BEIDOU,"), std::string::npos);
    EXPECT_NE(observations.find(",QZSS,"), std::string::npos);
}

unsigned int range_constellation_bits(gnss_sim::GnssConstellation constellation) {
    switch (constellation) {
        case gnss_sim::GnssConstellation::kGps:
            return 0U;
        case gnss_sim::GnssConstellation::kGlonass:
            return 1U;
        case gnss_sim::GnssConstellation::kGalileo:
            return 3U;
        case gnss_sim::GnssConstellation::kBeidou:
            return 4U;
        case gnss_sim::GnssConstellation::kQzss:
            return 5U;
    }
    return 7U;
}

unsigned int range_signal_key(const gnss_sim::SignalDefinition& definition) {
    return (range_constellation_bits(definition.constellation) << 5U) |
           (static_cast<unsigned int>(definition.novatel_oem7_signal_type) & 0x1FU);
}

std::set<unsigned int> emitted_range_signal_keys(const std::string& log_text) {
    std::set<unsigned int> keys;
    std::istringstream lines(log_text);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.rfind("#RANGEA", 0) != 0) {
            continue;
        }
        const std::size_t semicolon = line.find(';');
        if (semicolon == std::string::npos) {
            continue;
        }
        std::string body = line.substr(semicolon + 1);
        const std::size_t crc_separator = body.find('*');
        if (crc_separator != std::string::npos) {
            body.resize(crc_separator);
        }

        std::vector<std::string> fields;
        std::istringstream body_stream(body);
        std::string field;
        while (std::getline(body_stream, field, ',')) {
            fields.push_back(field);
        }
        if (fields.empty()) {
            continue;
        }

        const int observation_count = std::stoi(fields[0]);
        constexpr std::size_t kFieldsPerObservation = 10;
        const std::size_t expected_fields = 1U + static_cast<std::size_t>(observation_count) * kFieldsPerObservation;
        EXPECT_EQ(fields.size(), expected_fields) << "malformed RANGEA body: " << body;
        if (fields.size() < expected_fields) {
            continue;
        }
        for (int index = 0; index < observation_count; ++index) {
            const std::size_t status_index = 1U + static_cast<std::size_t>(index) * kFieldsPerObservation + 9U;
            const unsigned long status = std::stoul(fields[status_index], nullptr, 16);
            const unsigned int constellation = static_cast<unsigned int>((status >> 16U) & 0x7UL);
            const unsigned int signal_type = static_cast<unsigned int>((status >> 21U) & 0x1FUL);
            keys.insert((constellation << 5U) | signal_type);
        }
    }
    return keys;
}

void expect_frozen_signal_union_in_range_output(const std::vector<std::filesystem::path>& log_paths) {
    std::set<unsigned int> emitted;
    for (const std::filesystem::path& log_path : log_paths) {
        const std::set<unsigned int> window = emitted_range_signal_keys(read_file(log_path));
        emitted.insert(window.begin(), window.end());
    }

    std::size_t definition_count = 0;
    const gnss_sim::SignalDefinition* definitions = gnss_sim::signal_definitions(&definition_count);
    ASSERT_NE(definitions, nullptr);
    ASSERT_EQ(definition_count, 21U);
    for (std::size_t index = 0; index < definition_count; ++index) {
        const gnss_sim::SignalDefinition& definition = definitions[index];
        if (std::string(definition.name) == "GPS L1C") {
            // The provenance-fixed BRD400DLR fixture contains GPS LNAV/CNAV
            // but no GPS CNV2 record.  Do not manufacture an L1C navigation
            // family merely to make a single-window RANGEA assertion pass.
            continue;
        }
        EXPECT_EQ(emitted.count(range_signal_key(definition)), 1U)
            << "missing signal=" << definition.name
            << " from simulator->measurement->RANGEA union across real family-availability windows";
    }
}

double max_valid_position_error_m(const std::filesystem::path& solution_path, double truth_latitude_deg,
                                  double truth_longitude_deg, double truth_height_m,
                                  std::uint64_t* valid_position_count) {
    double truth_ecef[3]{};
    EXPECT_TRUE(gnss_sim::rtklib_llh_to_ecef(truth_latitude_deg, truth_longitude_deg, truth_height_m, truth_ecef));
    std::ifstream input(solution_path);
    std::string line;
    if (!std::getline(input, line)) {
        return std::numeric_limits<double>::infinity();
    }
    double maximum_error_m = 0.0;
    std::uint64_t count = 0;
    while (std::getline(input, line)) {
        std::vector<std::string> fields;
        std::istringstream stream(line);
        std::string field;
        while (std::getline(stream, field, ',')) {
            fields.push_back(field);
        }
        if (fields.size() < 13U || fields[4] != "1") {
            continue;
        }
        const double dx = std::stod(fields[10]) - truth_ecef[0];
        const double dy = std::stod(fields[11]) - truth_ecef[1];
        const double dz = std::stod(fields[12]) - truth_ecef[2];
        maximum_error_m = std::max(maximum_error_m, std::sqrt(dx * dx + dy * dy + dz * dz));
        ++count;
    }
    if (valid_position_count != nullptr) {
        *valid_position_count = count;
    }
    return count > 0 ? maximum_error_m : std::numeric_limits<double>::infinity();
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

TEST(V1Acceptance, RealMixedNavProducesAllFiveV1ConstellationsAndValidSolution) {
    gnss_sim::SimConfig config = acceptance_config();
    // This compact fixture validates five-system plumbing, not the production SPP cutoff.
    // Real BRD400 and RANGEA acceptance exercise the 5 degree solution default.
    config.solution_elevation_mask_deg = 0.0;
    config.sampling_rate_hz = 1;
    config.duration_ns = 60LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.receiver = {10.0, 80.0, 100.0};

    const std::filesystem::path directory = "gnss_sim_acceptance_five_system";
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run_in_directory_with_nav(directory, config, multi_gnss_nav_path(), multi_gnss_start_time(), &summary,
                                          &error_message))
        << error_message;

    expect_five_system_observations(directory / "observation_truth.csv");
    EXPECT_GT(summary.max_observations_per_epoch, 0);
    EXPECT_GT(summary.valid_position_epochs, 0U);
    EXPECT_GT(summary.valid_velocity_epochs, 0U);

    cleanup(directory);
}

TEST(V1Acceptance, RealBrd400DlrRinex4RunsFiveSystemReceiverNavLoopback) {
    gnss_sim::SimConfig config = acceptance_config();
    config.sampling_rate_hz = 1;
    config.duration_ns = 60LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.receiver = {20.0, 120.0, 100.0};

    const std::filesystem::path directory = "gnss_sim_acceptance_brd4_five_system";
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(
        run_in_directory_with_nav(directory, config, brd4_nav_path(), brd4_start_time(), &summary, &error_message))
        << error_message;

    const std::filesystem::path cnav_directory = "gnss_sim_acceptance_brd4_gps_cnav";
    gnss_sim::SimulatorRunSummary cnav_summary{};
    ASSERT_TRUE(run_in_directory_with_nav(cnav_directory, config, brd4_nav_path(), brd4_gps_cnav_start_time(),
                                          &cnav_summary, &error_message))
        << error_message;

    expect_five_system_observations(directory / "observation_truth.csv");
    expect_frozen_signal_union_in_range_output({directory / "simulated.log", cnav_directory / "simulated.log"});
    EXPECT_GT(summary.max_observations_per_epoch, 0);
    EXPECT_GT(summary.valid_position_epochs, 0U);
    EXPECT_GT(summary.valid_velocity_epochs, 0U);
    EXPECT_GT(cnav_summary.max_observations_per_epoch, 0);

    cleanup(directory);
    cleanup(cnav_directory);
}

TEST(V1Acceptance, RealBrd400DlrBroadcastAtmosphereMatchesReceiverSolutionConvention) {
    gnss_sim::SimConfig config = acceptance_config();
    config.atmosphere_mode = gnss_sim::AtmosphereMode::BROADCAST;
    config.sampling_rate_hz = 1;
    config.duration_ns = 60LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.receiver = {20.0, 120.0, 100.0};

    const std::filesystem::path directory = "gnss_sim_acceptance_brd4_broadcast_atmosphere";
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(
        run_in_directory_with_nav(directory, config, brd4_nav_path(), brd4_start_time(), &summary, &error_message))
        << error_message;

    expect_five_system_observations(directory / "observation_truth.csv");
    EXPECT_GT(summary.valid_position_epochs, 0U);
    std::uint64_t valid_position_count = 0;
    const double maximum_error_m =
        max_valid_position_error_m(directory / "solution_truth.csv", 20.0, 120.0, 100.0, &valid_position_count);
    EXPECT_EQ(valid_position_count, summary.valid_position_epochs);
    EXPECT_LT(maximum_error_m, 0.5)
        << "broadcast-atmosphere zero-noise loopback must not hide ion/trop convention mismatches";

    cleanup(directory);
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
