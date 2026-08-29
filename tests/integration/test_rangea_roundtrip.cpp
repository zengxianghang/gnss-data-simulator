#include "gnss/nav_output_record.h"
#include "gnss/rtklib_adapter.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"
#include "output/novatel_nav_writer.h"
#include "rangea_roundtrip.h"
#include "serialized_nav_parser.h"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

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
    value.solution_elevation_mask_deg = 5.0;
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

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> fields;
    std::istringstream input(line);
    std::string field;
    while (std::getline(input, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

bool has_valid_pseudorange_below_mask(const std::filesystem::path& path, double mask_deg) {
    std::ifstream input(path);
    std::string line;
    if (!std::getline(input, line)) {
        return false;
    }
    const std::vector<std::string> header = split_csv(line);
    std::size_t elevation_index = header.size();
    std::size_t validity_index = header.size();
    for (std::size_t index = 0; index < header.size(); ++index) {
        if (header[index] == "elevation_deg") {
            elevation_index = index;
        } else if (header[index] == "pseudorange_valid") {
            validity_index = index;
        }
    }
    if (elevation_index >= header.size() || validity_index >= header.size()) {
        return false;
    }

    while (std::getline(input, line)) {
        const std::vector<std::string> fields = split_csv(line);
        if (fields.size() <= elevation_index || fields.size() <= validity_index || fields[validity_index] != "1") {
            continue;
        }
        try {
            const double elevation_deg = std::stod(fields[elevation_index]);
            if (elevation_deg >= 0.0 && elevation_deg < mask_deg) {
                return true;
            }
        } catch (...) {
            return false;
        }
    }
    return false;
}

void cleanup(const std::filesystem::path& path) {
    std::error_code error;
    std::filesystem::remove_all(path, error);
}

TEST(RangeaRoundtripIntegration, LowElevationRangeIsRetainedWhileSppUsesFiveDegreeMask) {
    const std::filesystem::path directory = "gnss_sim_rangea_roundtrip_real_whu";
    gnss_sim::SimulatorRunSummary simulator_summary{};
    std::string error_message;
    ASSERT_TRUE(run_simulator(directory, &simulator_summary, &error_message)) << error_message;
    ASSERT_GT(simulator_summary.range_messages, 0U);
    ASSERT_GT(simulator_summary.valid_position_epochs, 0U);
    ASSERT_TRUE(has_valid_pseudorange_below_mask(directory / "observation_truth.csv", 5.0))
        << "measurement mask 0 deg should preserve at least one valid observation below the 5 deg SPP mask";

    gnss_sim::RangeaRoundtripSummary roundtrip{};
    const std::string log_path = (directory / "simulated.log").string();
    const std::string nav_path = brd4_nav_path();
    ASSERT_TRUE(gnss_sim::validate_rangea_roundtrip_file(log_path.c_str(), nav_path.c_str(), 20.0, 120.0, 100.0, 5.0,
                                                         true, &roundtrip, &error_message))
        << error_message;
    EXPECT_EQ(roundtrip.range_epochs, simulator_summary.range_messages);
    EXPECT_GT(roundtrip.parsed_observations, 0U);
    EXPECT_GT(roundtrip.selected_position_observations, 0U);
    EXPECT_EQ(roundtrip.valid_position_epochs, simulator_summary.valid_position_epochs)
        << "serialized RANGEA and the maintained in-memory SPP path must agree on position availability";
    std::cout << "rangea_roundtrip_max_3d_error_m=" << roundtrip.max_position_error_m << '\n';
    std::cout << "rangea_roundtrip_max_error_gpst=" << roundtrip.max_error_gps_week << '/'
              << roundtrip.max_error_sow_sec << '\n';
    std::cout << "rangea_roundtrip_valid_position_epochs=" << roundtrip.valid_position_epochs << '\n';
    EXPECT_LT(roundtrip.max_position_error_m, 0.01)
        << "5 deg RTKLIB SPP mask should exclude the known near-horizon pathology from normal positioning";

    cleanup(directory);
}

TEST(RangeaRoundtripIntegration, SerializedRangeEphemerisAndIonPositionWithoutOriginalRinexNav) {
    const std::filesystem::path directory = "gnss_sim_serialized_nav_roundtrip_real_whu";
    gnss_sim::SimulatorRunSummary simulator_summary{};
    std::string error_message;
    ASSERT_TRUE(run_simulator(directory, &simulator_summary, &error_message)) << error_message;
    ASSERT_GT(simulator_summary.range_messages, 0U);
    ASSERT_GT(simulator_summary.nav_messages, 0U);

    gnss_sim::SerializedNavRoundtripSummary roundtrip{};
    const std::string log_path = (directory / "simulated.log").string();
    ASSERT_TRUE(gnss_sim::validate_serialized_navigation_roundtrip_file(log_path.c_str(), 20.0, 120.0, 100.0, 5.0, true,
                                                                        &roundtrip, &error_message))
        << error_message;
    EXPECT_EQ(roundtrip.position.range_epochs, simulator_summary.range_messages);
    EXPECT_GT(roundtrip.parsed_nav_records, 0U);
    EXPECT_GT(roundtrip.parsed_ionosphere_records, 0U);
    EXPECT_GT(roundtrip.gps_ephemeris_records, 0U);
    EXPECT_GT(roundtrip.glonass_ephemeris_records, 0U);
    EXPECT_GT(roundtrip.galileo_ephemeris_records, 0U);
    EXPECT_GT(roundtrip.beidou_ephemeris_records, 0U);
    EXPECT_GT(roundtrip.qzss_ephemeris_records, 0U);
    EXPECT_GT(roundtrip.position.valid_position_epochs, 0U);
    EXPECT_GT(roundtrip.skipped_position_observations_without_nav, 0U)
        << "real receiver-visible NAV timing should leave some early observations without a usable ephemeris";
    std::cout << "serialized_nav_roundtrip_skipped_without_nav=" << roundtrip.skipped_position_observations_without_nav
              << '\n';
    std::cout << "serialized_nav_roundtrip_max_3d_error_m=" << roundtrip.position.max_position_error_m << '\n';
    std::cout << "serialized_nav_roundtrip_valid_position_epochs=" << roundtrip.position.valid_position_epochs << '\n';
    EXPECT_LT(roundtrip.position.max_position_error_m, 0.1)
        << "serialized EPHA/IONA precision should preserve survey-grade compact SPP without source RINEX NAV";

    cleanup(directory);
}

TEST(RangeaRoundtripIntegration, SerializedNavigationDoesNotUseFutureNavBeforeItAppearsInLog) {
    const std::filesystem::path directory = "gnss_sim_serialized_nav_causal_order";
    gnss_sim::SimulatorRunSummary simulator_summary{};
    std::string error_message;
    ASSERT_TRUE(run_simulator(directory, &simulator_summary, &error_message)) << error_message;

    const std::string original_log = read_file(directory / "simulated.log");
    std::istringstream input(original_log);
    std::string line;
    std::string first_valid_range;
    while (std::getline(input, line)) {
        if (line.rfind("#RANGEA,", 0) != 0) {
            continue;
        }
        gnss_sim::ParsedRangeEpoch epoch{};
        std::string parse_error;
        ASSERT_TRUE(gnss_sim::parse_rangea_line_independent(line, &epoch, &parse_error)) << parse_error;
        bool has_valid_pseudorange = false;
        for (const gnss_sim::ParsedRangeObservation& observation : epoch.observations) {
            if (observation.pseudorange_valid) {
                has_valid_pseudorange = true;
                break;
            }
        }
        if (has_valid_pseudorange) {
            first_valid_range = line;
            break;
        }
    }
    ASSERT_FALSE(first_valid_range.empty());

    // Put a valid observation epoch before the complete original log. All of
    // the real serialized EPHA/IONA records still exist later in the same
    // stream. A causal validator must skip that early observation rather than
    // looking ahead; once the original log later delivers NAV, normal epochs
    // can still position successfully.
    std::istringstream reordered(first_valid_range + "\n" + original_log);
    gnss_sim::SerializedNavRoundtripSummary summary{};
    error_message.clear();
    ASSERT_TRUE(gnss_sim::validate_serialized_navigation_roundtrip_stream(&reordered, 20.0, 120.0, 100.0, 5.0, true,
                                                                          &summary, &error_message))
        << error_message;
    EXPECT_GT(summary.skipped_position_observations_without_nav, 0U);
    EXPECT_GT(summary.position.valid_position_epochs, 0U);

    cleanup(directory);
}

TEST(RangeaRoundtripIntegration, MalformedSerializedNavigationCrcFailsExplicitly) {
    std::istringstream malformed("#IONUTCA,COM1,0,0.0,FINE,2347,436500.000,00000000,0,0;"
                                 "0,0,0,0,0,0,0,0,2347,0,0,0,2347,0,18,18,0*00000000\r\n");
    gnss_sim::SerializedNavRoundtripSummary summary{};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::validate_serialized_navigation_roundtrip_stream(&malformed, 20.0, 120.0, 100.0, 5.0, true,
                                                                           &summary, &error_message));
    EXPECT_NE(error_message.find("CRC"), std::string::npos);
}

TEST(RangeaRoundtripIntegration, MalformedSerializedRangeaFailsExplicitly) {
    std::istringstream malformed(
        "#RANGEA,COM1,0,0.0,FINE,2347,436500.000,00000000,0,0;1,1,0,20000000.000*00000000\r\n");
    gnss_sim::RangeaRoundtripSummary summary{};
    std::string error_message;
    const std::string nav_path = brd4_nav_path();
    EXPECT_FALSE(gnss_sim::validate_rangea_roundtrip_stream(&malformed, nav_path.c_str(), 20.0, 120.0, 100.0, 5.0, true,
                                                            &summary, &error_message));
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
    ASSERT_TRUE(gnss_sim::validate_rangea_roundtrip_file(first_log.c_str(), nav_path.c_str(), 20.0, 120.0, 100.0, 5.0,
                                                         true, &first_roundtrip, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::validate_rangea_roundtrip_file(second_log.c_str(), nav_path.c_str(), 20.0, 120.0, 100.0, 5.0,
                                                         true, &second_roundtrip, &error_message))
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

TEST(RangeaRoundtripIntegration, RealBeidouLegacyAsciiRestoresRtklibBroadcastState) {
    gnss_sim::RtklibNavStore* full = gnss_sim::create_rtklib_nav_store();
    gnss_sim::RtklibNavStore* source_record = gnss_sim::create_rtklib_nav_store();
    gnss_sim::RtklibNavStore* rebuilt = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(full, nullptr);
    ASSERT_NE(source_record, nullptr);
    ASSERT_NE(rebuilt, nullptr);

    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(full, brd4_nav_path().c_str(), &error_message)) << error_message;

    bool compared = false;
    const int output_count = gnss_sim::rtklib_nav_output_record_count(full);
    for (int index = 0; index < output_count; ++index) {
        gnss_sim::NavOutputRecord source{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(full, index, &source, &error_message)) << error_message;
        if (source.kind != gnss_sim::RtklibNavRecordKind::kEphemeris ||
            source.ephemeris.system != gnss_sim::NavOutputSystem::kBeidou ||
            source.ephemeris.message_family != gnss_sim::RtklibBroadcastMessageFamily::kLegacy) {
            continue;
        }

        ASSERT_TRUE(gnss_sim::rtklib_copy_nav_record(full, index, source_record, &error_message)) << error_message;
        gnss_sim::SimTime output_time{};
        ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(source.ephemeris.transmit_week, source.ephemeris.transmit_sow_sec,
                                                     &output_time));

        std::string message;
        bool supported = false;
        ASSERT_TRUE(
            gnss_sim::format_novatel_nav_output_record(source, output_time, &message, &supported, &error_message))
            << error_message;
        ASSERT_TRUE(supported);
        ASSERT_EQ(message.rfind("#BD2EPHEMA,", 0), 0U);

        gnss_sim::ParsedSerializedNavRecord parsed{};
        bool recognized = false;
        ASSERT_TRUE(
            gnss_sim::parse_serialized_novatel_nav_line_independent(message, &parsed, &recognized, &error_message))
            << error_message;
        ASSERT_TRUE(recognized);
        ASSERT_TRUE(gnss_sim::rtklib_append_nav_output_record(rebuilt, parsed.record, &error_message)) << error_message;

        int observation_code = 0;
        int frequency_index = 0;
        ASSERT_TRUE(gnss_sim::rtklib_observation_code("2I", &observation_code, &frequency_index));
        static_cast<void>(frequency_index);

        int state_week = parsed.output_gps_week;
        double state_sow = parsed.output_sow_sec + 60.0;
        if (state_sow >= 604800.0) {
            state_sow -= 604800.0;
            ++state_week;
        }
        gnss_sim::RtklibSatelliteState expected{};
        gnss_sim::RtklibSatelliteState actual{};
        ASSERT_TRUE(gnss_sim::get_rtklib_signal_satellite_state(
            source_record, state_week, state_sow, source.ephemeris.satellite_number, observation_code,
            gnss_sim::RtklibBroadcastMessageFamily::kLegacy, &expected, &error_message))
            << error_message;
        ASSERT_TRUE(gnss_sim::get_rtklib_signal_satellite_state(
            rebuilt, state_week, state_sow, source.ephemeris.satellite_number, observation_code,
            gnss_sim::RtklibBroadcastMessageFamily::kLegacy, &actual, &error_message))
            << error_message;

        const double dx = expected.position_ecef_m[0] - actual.position_ecef_m[0];
        const double dy = expected.position_ecef_m[1] - actual.position_ecef_m[1];
        const double dz = expected.position_ecef_m[2] - actual.position_ecef_m[2];
        const double position_difference_m = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double clock_difference_m = std::abs(expected.clock_bias_sec - actual.clock_bias_sec) * 299792458.0;
        EXPECT_LT(position_difference_m, 1.0e-3);
        EXPECT_LT(clock_difference_m, 1.0e-6);
        EXPECT_EQ(expected.health, actual.health);
        compared = true;
        break;
    }

    EXPECT_TRUE(compared) << "real BRD400 fixture must contain a NovAtel-serializable BeiDou legacy ephemeris";
    gnss_sim::destroy_rtklib_nav_store(rebuilt);
    gnss_sim::destroy_rtklib_nav_store(source_record);
    gnss_sim::destroy_rtklib_nav_store(full);
}

} // namespace
