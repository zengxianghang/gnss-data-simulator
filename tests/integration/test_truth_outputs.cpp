#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"
#include "output/urban_truth_writer.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <vector>

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

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::size_t start = 0;
    while (true) {
        const std::size_t separator = line.find(',', start);
        if (separator == std::string::npos) {
            fields.push_back(line.substr(start));
            return fields;
        }
        fields.push_back(line.substr(start, separator - start));
        start = separator + 1U;
    }
}

std::vector<std::vector<std::string>> read_simple_csv(const std::filesystem::path& path) {
    std::vector<std::vector<std::string>> rows;
    std::ifstream input(path, std::ios::binary);
    std::string line;
    while (std::getline(input, line)) {
        rows.push_back(split_csv_line(line));
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

void expect_consistent_simple_csv_columns(const std::filesystem::path& path) {
    const std::vector<std::vector<std::string>> rows = read_simple_csv(path);
    ASSERT_FALSE(rows.empty()) << path.string();
    const std::size_t expected_columns = rows.front().size();
    ASSERT_GT(expected_columns, 0U) << path.string();
    for (std::size_t row = 1; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row].size(), expected_columns) << path.string() << " row " << row + 1U;
    }
}

void expect_stable_urban_path_order(const std::filesystem::path& path) {
    const std::vector<std::vector<std::string>> rows = read_simple_csv(path);
    ASSERT_GT(rows.size(), 1U);
    const int week_column = column_index(rows.front(), "gps_week");
    const int tow_column = column_index(rows.front(), "tow_ns");
    const int satellite_column = column_index(rows.front(), "satellite_number");
    const int signal_column = column_index(rows.front(), "signal_id");
    const int path_index_column = column_index(rows.front(), "path_index");
    ASSERT_GE(week_column, 0);
    ASSERT_GE(tow_column, 0);
    ASSERT_GE(satellite_column, 0);
    ASSERT_GE(signal_column, 0);
    ASSERT_GE(path_index_column, 0);

    std::string previous_key;
    int expected_path_index = 0;
    for (std::size_t row_index = 1; row_index < rows.size(); ++row_index) {
        const std::vector<std::string>& row = rows[row_index];
        const std::string key = row[static_cast<std::size_t>(week_column)] + ":" +
                                row[static_cast<std::size_t>(tow_column)] + ":" +
                                row[static_cast<std::size_t>(satellite_column)] + ":" +
                                row[static_cast<std::size_t>(signal_column)];
        if (key != previous_key) {
            expected_path_index = 0;
            previous_key = key;
        }
        ASSERT_FALSE(row[static_cast<std::size_t>(path_index_column)].empty());
        EXPECT_EQ(std::stoi(row[static_cast<std::size_t>(path_index_column)]), expected_path_index) << key;
        ++expected_path_index;
    }
}

bool run_config_in_directory(const std::filesystem::path& directory, const gnss_sim::SimConfig& config,
                             gnss_sim::SimulatorRunSummary* summary, std::string* error_message,
                             const char* cn0_model_path = nullptr) {
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
    return gnss_sim::run_simulator(config, options, summary, error_message);
}

bool run_in_directory(const std::filesystem::path& directory, gnss_sim::SimulatorRunSummary* summary,
                      std::string* error_message, const char* cn0_model_path = nullptr) {
    return run_config_in_directory(directory, truth_config(), summary, error_message, cn0_model_path);
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
    EXPECT_EQ(first_line(directory / "urban_signal_truth.csv"),
              "truth_schema_version,gps_week,tow_ns,sow_sec,satellite_number,signal_id,signal_name,glonass_fcn,"
              "azimuth_deg,elevation_deg,propagation_evaluated,direct_los,blocking_wall,grazing_roof,diffraction_status,"
              "reflection_count,received_path_count,urban_state,tracking_phase,loss_reason,open_cn0_dbhz,effective_cn0_dbhz,"
              "effective_cn0_finite,composite_power_ratio,dll_root_count,root_search_status,selection_mode,"
              "selected_root_valid,dll_code_phase_sec,dll_code_phase_chips,code_bias_m,tracked_correlation_real,"
              "tracked_correlation_imag,lock_time_ns,observation_available,pseudorange_valid,doppler_valid,adr_valid,"
              "reacquisition_event,carrier_continuity_valid,wavelength_m,wrapped_phase_rad,unwrapped_phase_rad,"
              "carrier_range_bias_m,environmental_range_rate_mps,environmental_range_rate_valid,cycle_slip_event,"
              "receiver_observation_emitted,pseudorange_m,doppler_hz,adr_cycles");
    EXPECT_EQ(first_line(directory / "urban_path_truth.csv"),
              "truth_schema_version,gps_week,tow_ns,sow_sec,satellite_number,signal_id,path_index,path_kind,wall_id,"
              "point_e_m,point_n_m,point_u_m,model_path_range_m,excess_path_m,code_delay_sec,voltage_real,voltage_imag,"
              "voltage_amplitude,voltage_phase_rad,fresnel_v,fresnel_real,fresnel_imag,incidence_angle_deg,"
              "gamma_rhcp_real,gamma_rhcp_imag,gamma_lhcp_real,gamma_lhcp_imag,antenna_rhcp_real,antenna_rhcp_imag,"
              "antenna_lhcp_real,antenna_lhcp_imag");

    const std::string scenario = read_file(directory / "scenario.json");
    const std::string manifest = read_file(directory / "run_manifest.json");
    const std::string observations = read_file(directory / "observation_truth.csv");
    EXPECT_NE(scenario.find("\"truth_schema_version\": 1"), std::string::npos);
    EXPECT_NE(scenario.find("\"atmosphere_mode\": \"none\""), std::string::npos);
    EXPECT_NE(scenario.find("\"cn0_high_dbhz\": {}"), std::string::npos);
    EXPECT_NE(manifest.find("\"output_format_version\": 1"), std::string::npos);
    EXPECT_NE(manifest.find(std::string("\"rtklib_commit_sha\": \"") + GNSS_SIM_RTKLIB_COMMIT + "\""),
              std::string::npos);
    EXPECT_NE(manifest.find("\"hash_algorithm\": \"fnv1a64\""), std::string::npos);
    EXPECT_NE(manifest.find("\"random_seed\": 7"), std::string::npos);
    EXPECT_NE(manifest.find("\"bestpos_messages\": 10"), std::string::npos);
    EXPECT_NE(manifest.find("\"bestpos_rtk\": {"), std::string::npos);
    EXPECT_NE(manifest.find("\"stable_duration_ns\": 5000000000"), std::string::npos);
    EXPECT_EQ(summary.bestpos_messages, 10U);
    EXPECT_NE(manifest.find("\"source\": \"BUILTIN_FALLBACK\""), std::string::npos);
    EXPECT_NE(manifest.find("\"semantic\": \"BUILTIN_ABSOLUTE_CN0\""), std::string::npos);
    EXPECT_NE(manifest.find("\"schema_version\": \"builtin-cn0-v1\""), std::string::npos);
    EXPECT_NE(manifest.find("\"hash_algorithm\": \"none\""), std::string::npos);
    EXPECT_EQ(summary.cn0_model_source, "BUILTIN_FALLBACK");
    EXPECT_EQ(summary.cn0_model_semantic, "BUILTIN_ABSOLUTE_CN0");
    EXPECT_NE(read_file(directory / "event_truth.csv").find("POWER_ON"), std::string::npos);
    EXPECT_NE(observations.find(",LEGACY,"), std::string::npos);
    EXPECT_GT(observations.size(), first_line(directory / "observation_truth.csv").size());
    EXPECT_GT(read_file(directory / "solution_truth.csv").size(), first_line(directory / "solution_truth.csv").size());
    EXPECT_EQ(read_file(directory / "urban_signal_truth.csv"), first_line(directory / "urban_signal_truth.csv") + "\n");
    EXPECT_EQ(read_file(directory / "urban_path_truth.csv"), first_line(directory / "urban_path_truth.csv") + "\n");
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

    for (const char* file_name : {"scenario.json", "event_truth.csv", "observation_truth.csv", "solution_truth.csv",
                                  "urban_signal_truth.csv", "urban_path_truth.csv", "run_manifest.json"}) {
        EXPECT_EQ(read_file(first_directory / file_name), read_file(second_directory / file_name)) << file_name;
    }
    EXPECT_EQ(first_summary.scheduled_epochs, second_summary.scheduled_epochs);
    EXPECT_EQ(first_summary.max_observations_per_epoch, second_summary.max_observations_per_epoch);
    cleanup(first_directory);
    cleanup(second_directory);
}

TEST(TruthOutputs, UrbanTruthIsDeterministicOrderedAndMatchesSynthesizedObservation) {
    const std::filesystem::path first_directory = "gnss_sim_truth_urban_a";
    const std::filesystem::path second_directory = "gnss_sim_truth_urban_b";
    gnss_sim::SimConfig config = truth_config();
    config.sampling_rate_hz = 10;
    config.duration_ns = 6LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.measurement_noise_enabled = false;
    config.multipath_enabled = true;

    gnss_sim::SimulatorRunSummary first_summary{};
    gnss_sim::SimulatorRunSummary second_summary{};
    std::string error_message;
    ASSERT_TRUE(run_config_in_directory(first_directory, config, &first_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_config_in_directory(second_directory, config, &second_summary, &error_message)) << error_message;

    for (const char* file_name : {"simulated.log", "urban_signal_truth.csv", "urban_path_truth.csv"}) {
        EXPECT_EQ(read_file(first_directory / file_name), read_file(second_directory / file_name)) << file_name;
    }
    expect_consistent_simple_csv_columns(first_directory / "urban_signal_truth.csv");
    expect_consistent_simple_csv_columns(first_directory / "urban_path_truth.csv");
    expect_stable_urban_path_order(first_directory / "urban_path_truth.csv");

    const std::vector<std::vector<std::string>> urban_rows =
        read_simple_csv(first_directory / "urban_signal_truth.csv");
    const std::vector<std::vector<std::string>> observation_rows =
        read_simple_csv(first_directory / "observation_truth.csv");
    ASSERT_GT(urban_rows.size(), 1U);
    ASSERT_GT(observation_rows.size(), 1U);

    const int urban_week = column_index(urban_rows.front(), "gps_week");
    const int urban_tow = column_index(urban_rows.front(), "tow_ns");
    const int urban_satellite = column_index(urban_rows.front(), "satellite_number");
    const int urban_signal = column_index(urban_rows.front(), "signal_id");
    const int urban_state = column_index(urban_rows.front(), "urban_state");
    const int urban_emitted = column_index(urban_rows.front(), "receiver_observation_emitted");
    const int urban_pseudorange = column_index(urban_rows.front(), "pseudorange_m");
    const int urban_doppler = column_index(urban_rows.front(), "doppler_hz");
    const int urban_adr = column_index(urban_rows.front(), "adr_cycles");
    const int observation_week = column_index(observation_rows.front(), "gps_week");
    const int observation_tow = column_index(observation_rows.front(), "tow_ns");
    const int observation_satellite = column_index(observation_rows.front(), "satellite_number");
    const int observation_signal = column_index(observation_rows.front(), "signal_id");
    const int observation_pseudorange = column_index(observation_rows.front(), "pseudorange_m");
    const int observation_doppler = column_index(observation_rows.front(), "doppler_hz");
    const int observation_adr = column_index(observation_rows.front(), "adr_cycles");
    ASSERT_GE(urban_week, 0);
    ASSERT_GE(urban_tow, 0);
    ASSERT_GE(urban_satellite, 0);
    ASSERT_GE(urban_signal, 0);
    ASSERT_GE(urban_state, 0);
    ASSERT_GE(urban_emitted, 0);
    ASSERT_GE(urban_pseudorange, 0);
    ASSERT_GE(urban_doppler, 0);
    ASSERT_GE(urban_adr, 0);
    ASSERT_GE(observation_week, 0);
    ASSERT_GE(observation_tow, 0);
    ASSERT_GE(observation_satellite, 0);
    ASSERT_GE(observation_signal, 0);
    ASSERT_GE(observation_pseudorange, 0);
    ASSERT_GE(observation_doppler, 0);
    ASSERT_GE(observation_adr, 0);

    bool found_blocked_without_observation = false;
    const std::vector<std::string>* emitted_row = nullptr;
    for (std::size_t row_index = 1; row_index < urban_rows.size(); ++row_index) {
        const std::vector<std::string>& row = urban_rows[row_index];
        if (row[static_cast<std::size_t>(urban_state)] == "BLOCKED" &&
            row[static_cast<std::size_t>(urban_emitted)] == "0") {
            found_blocked_without_observation = true;
        }
        if (emitted_row == nullptr && row[static_cast<std::size_t>(urban_emitted)] == "1") {
            emitted_row = &row;
        }
    }
    EXPECT_TRUE(found_blocked_without_observation);
    ASSERT_NE(emitted_row, nullptr);

    const std::vector<std::string>* matching_observation = nullptr;
    for (std::size_t row_index = 1; row_index < observation_rows.size(); ++row_index) {
        const std::vector<std::string>& row = observation_rows[row_index];
        if (row[static_cast<std::size_t>(observation_week)] ==
                (*emitted_row)[static_cast<std::size_t>(urban_week)] &&
            row[static_cast<std::size_t>(observation_tow)] ==
                (*emitted_row)[static_cast<std::size_t>(urban_tow)] &&
            row[static_cast<std::size_t>(observation_satellite)] ==
                (*emitted_row)[static_cast<std::size_t>(urban_satellite)] &&
            row[static_cast<std::size_t>(observation_signal)] ==
                (*emitted_row)[static_cast<std::size_t>(urban_signal)]) {
            matching_observation = &row;
            break;
        }
    }
    ASSERT_NE(matching_observation, nullptr);
    EXPECT_EQ((*emitted_row)[static_cast<std::size_t>(urban_pseudorange)],
              (*matching_observation)[static_cast<std::size_t>(observation_pseudorange)]);
    EXPECT_EQ((*emitted_row)[static_cast<std::size_t>(urban_doppler)],
              (*matching_observation)[static_cast<std::size_t>(observation_doppler)]);
    EXPECT_EQ((*emitted_row)[static_cast<std::size_t>(urban_adr)],
              (*matching_observation)[static_cast<std::size_t>(observation_adr)]);

    cleanup(first_directory);
    cleanup(second_directory);
}

TEST(TruthOutputs, UrbanTruthSerializerCoversAllFourPropagationStates) {
    const std::filesystem::path directory = "gnss_sim_truth_urban_states";
    cleanup(directory);
    std::error_code filesystem_error;
    ASSERT_TRUE(std::filesystem::create_directories(directory, filesystem_error));
    ASSERT_FALSE(filesystem_error);

    const std::filesystem::path receiver_log_path = directory / "simulated.log";
    std::string error_message;
    gnss_sim::UrbanTruthWriter* writer =
        gnss_sim::create_urban_truth_writer(receiver_log_path.string().c_str(), &error_message);
    ASSERT_NE(writer, nullptr) << error_message;
    const gnss_sim::SignalDefinition* signal = gnss_sim::find_signal_definition(gnss_sim::SignalId::kGpsL1Ca);
    ASSERT_NE(signal, nullptr);
    double wavelength_m = 0.0;
    ASSERT_TRUE(gnss_sim::signal_wavelength_m(*signal, 0, &wavelength_m));

    const gnss_sim::UrbanSignalState states[] = {gnss_sim::UrbanSignalState::kLos,
                                                 gnss_sim::UrbanSignalState::kLosMultipath,
                                                 gnss_sim::UrbanSignalState::kNlosTracked,
                                                 gnss_sim::UrbanSignalState::kBlocked};
    const gnss_sim::SimTime base_time = start_time();
    for (int state_index = 0; state_index < 4; ++state_index) {
        gnss_sim::SatelliteGeometry geometry{};
        ASSERT_TRUE(gnss_sim::add_time_ns(base_time, state_index * gnss_sim::NANOSECONDS_PER_SECOND,
                                          &geometry.receive_time));
        geometry.satellite_number = 1;
        geometry.geometric_range_m = 21000000.0 + state_index;
        geometry.azimuth_rad = 0.4 + 0.1 * state_index;
        geometry.elevation_rad = 0.6;

        gnss_sim::UrbanSignalEpochResult epoch{};
        epoch.urban_state = states[state_index];
        epoch.tracking_phase = state_index == 3 ? gnss_sim::SignalTrackingPhase::kSearching
                                                : gnss_sim::SignalTrackingPhase::kTracking;
        epoch.loss_reason = state_index == 3 ? gnss_sim::SignalTrackingLossReason::kLowCn0
                                             : gnss_sim::SignalTrackingLossReason::kNone;
        epoch.lock_time_ns = state_index == 3 ? 0 : state_index * gnss_sim::NANOSECONDS_PER_SECOND;
        epoch.observation_available = state_index != 3;
        epoch.psr_valid = state_index != 3;
        epoch.doppler_valid = state_index != 3;
        epoch.adr_valid = state_index != 3;
        epoch.reacquisition_event = state_index == 2;
        epoch.carrier_continuity_valid = state_index != 3;

        gnss_sim::UrbanCarrierTemporalResult temporal{};
        temporal.wavelength_m = wavelength_m;
        temporal.tracking_lock_valid = state_index != 3;
        temporal.carrier_adr_valid = state_index != 3;
        temporal.phase_continuity_valid = state_index != 3;
        temporal.cycle_slip_event = state_index == 2;

        if (state_index != 3) {
            gnss_sim::UrbanReceivedPathSet& received = epoch.received_paths;
            received.path_count = 1;
            received.open_cn0_dbhz = 45.0;
            received.direct_geometry.line_of_sight = state_index < 2;
            received.direct_geometry.primary_wall = gnss_sim::UrbanWallId::NORTH;
            received.direct_geometry.grazing_roof = state_index == 1;
            received.paths[0].code_delay_sec = state_index == 0 ? 0.0 : 1.0e-8 * state_index;
            received.paths[0].complex_voltage = {1.0 - 0.1 * state_index, 0.05 * state_index};
            if (state_index == 0) {
                received.diffraction_status = gnss_sim::UrbanRooftopDiffractionStatus::NO_BLOCKING_ROOF_EDGE;
            } else {
                received.diffraction_status = gnss_sim::UrbanRooftopDiffractionStatus::VALID;
                received.diffraction.wall_id = gnss_sim::UrbanWallId::NORTH;
                received.diffraction.diffraction_point_enu_m = {0.0, 10.0, 10.0};
                received.diffraction.model_path_range_m = geometry.geometric_range_m + 1.0 + state_index;
                received.diffraction.excess_path_length_m = 1.0 + state_index;
                received.diffraction.fresnel_v = state_index == 1 ? -0.1 : 0.8;
                received.diffraction.fresnel_coefficient = received.paths[0].complex_voltage;
            }
            epoch.effective_cn0.open_cn0_dbhz = received.open_cn0_dbhz;
            epoch.effective_cn0.composite_correlation = received.paths[0].complex_voltage;
            epoch.effective_cn0.composite_power_ratio = std::norm(received.paths[0].complex_voltage);
            epoch.effective_cn0.effective_cn0_dbhz = 44.0 - state_index;
            epoch.effective_cn0.finite_effective_cn0 = true;
            epoch.effective_cn0_dbhz = epoch.effective_cn0.effective_cn0_dbhz;
            epoch.dll_root_count = 1;
            epoch.root_search_status = gnss_sim::CodeTrackingDllRootSearchStatus::kRootsFound;
            epoch.selection_mode = gnss_sim::CodeTrackingDllSelectionMode::TRACKED;
            epoch.selected_root_valid = true;
            epoch.preselected_root.code_phase_sec = received.paths[0].code_delay_sec;
            epoch.preselected_root.code_phase_chips = 0.01 * state_index;
            epoch.code_bias_m = 299792458.0 * epoch.preselected_root.code_phase_sec;
            epoch.tracked_composite_correlation = received.paths[0].complex_voltage;
            temporal.wrapped_phase_rad = 0.1 * state_index;
            temporal.unwrapped_phase_rad = 0.1 * state_index;
            temporal.carrier_range_bias_m = -temporal.unwrapped_phase_rad * wavelength_m /
                                            (2.0 * 3.141592653589793238462643383279502884);
            temporal.environmental_range_rate_mps = 0.01 * state_index;
            temporal.environmental_range_rate_valid = state_index > 0;
        }

        ASSERT_TRUE(gnss_sim::urban_truth_writer_write_signal(writer, geometry, *signal, 0, epoch, temporal, nullptr,
                                                               &error_message))
            << error_message;
    }
    ASSERT_TRUE(gnss_sim::finalize_urban_truth_writer(writer, &error_message)) << error_message;
    gnss_sim::destroy_urban_truth_writer(writer);

    expect_consistent_simple_csv_columns(directory / "urban_signal_truth.csv");
    expect_consistent_simple_csv_columns(directory / "urban_path_truth.csv");
    const std::vector<std::vector<std::string>> rows = read_simple_csv(directory / "urban_signal_truth.csv");
    ASSERT_EQ(rows.size(), 5U);
    const int state_column = column_index(rows.front(), "urban_state");
    const int loss_column = column_index(rows.front(), "loss_reason");
    const int reacquisition_column = column_index(rows.front(), "reacquisition_event");
    const int slip_column = column_index(rows.front(), "cycle_slip_event");
    ASSERT_GE(state_column, 0);
    ASSERT_GE(loss_column, 0);
    ASSERT_GE(reacquisition_column, 0);
    ASSERT_GE(slip_column, 0);
    EXPECT_EQ(rows[1][static_cast<std::size_t>(state_column)], "LOS");
    EXPECT_EQ(rows[2][static_cast<std::size_t>(state_column)], "LOS_MULTIPATH");
    EXPECT_EQ(rows[3][static_cast<std::size_t>(state_column)], "NLOS_TRACKED");
    EXPECT_EQ(rows[4][static_cast<std::size_t>(state_column)], "BLOCKED");
    EXPECT_EQ(rows[3][static_cast<std::size_t>(reacquisition_column)], "1");
    EXPECT_EQ(rows[3][static_cast<std::size_t>(slip_column)], "1");
    EXPECT_EQ(rows[4][static_cast<std::size_t>(loss_column)], "LOW_CN0");
    EXPECT_EQ(read_simple_csv(directory / "urban_path_truth.csv").size(), 4U);
    cleanup(directory);
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
    ASSERT_TRUE(run_in_directory(second_directory, &second_summary, &error_message, model_path.c_str()))
        << error_message;
    ASSERT_TRUE(run_in_directory(builtin_directory, &builtin_summary, &error_message)) << error_message;

    for (const char* file_name : {"simulated.log", "scenario.json", "event_truth.csv", "observation_truth.csv",
                                  "solution_truth.csv", "urban_signal_truth.csv", "urban_path_truth.csv",
                                  "run_manifest.json"}) {
        EXPECT_EQ(read_file(first_directory / file_name), read_file(second_directory / file_name)) << file_name;
    }
    const std::string manifest = read_file(first_directory / "run_manifest.json");
    EXPECT_EQ(first_summary.cn0_model_source, "CALIBRATED_CSV");
    EXPECT_EQ(first_summary.cn0_model_semantic, "ABSOLUTE_STATION_CN0");
    EXPECT_EQ(first_summary.cn0_model_schema_version, "gnss-cn0-model-v1");
    EXPECT_EQ(first_summary.cn0_model_name, "runtime_cn0_model.csv");
    EXPECT_FALSE(first_summary.cn0_model_hash.empty());
    EXPECT_GT(first_summary.cn0_model_size_bytes, 0U);
    EXPECT_NE(manifest.find("\"source\": \"CALIBRATED_CSV\""), std::string::npos);
    EXPECT_NE(manifest.find("\"semantic\": \"ABSOLUTE_STATION_CN0\""), std::string::npos);
    EXPECT_NE(manifest.find("\"schema_version\": \"gnss-cn0-model-v1\""), std::string::npos);
    EXPECT_NE(manifest.find("\"name\": \"runtime_cn0_model.csv\""), std::string::npos);
    EXPECT_NE(read_file(first_directory / "observation_truth.csv"),
              read_file(builtin_directory / "observation_truth.csv"));

    cleanup(first_directory);
    cleanup(second_directory);
    cleanup(builtin_directory);
}

TEST(TruthOutputs, NormalizedCn0ManifestRecordsSemanticAndReceiverBaseline) {
    const std::filesystem::path directory = "gnss_sim_truth_cn0_normalized";
    const std::filesystem::path model_path = "gnss_sim_runtime_cn0_normalized.csv";
    {
        std::ofstream output(model_path, std::ios::binary | std::ios::trunc);
        output << "schema_version,model_semantic,constellation,signal,elevation_min_deg,elevation_max_deg,"
                  "upper_edge_inclusive,status,contributing_source_count,delta_p50_db\n";
        output << "gnss-cn0-model-v2,NORMALIZED_ELEVATION_SHAPE,GPS,1C,0,90,1,READY,2,-2.000000\n";
    }

    gnss_sim::SimConfig config = truth_config();
    config.cn0_high_dbhz = {{"GPS L1 C/A", 47.0}};
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run_config_in_directory(directory, config, &summary, &error_message, model_path.string().c_str()))
        << error_message;

    const std::string scenario = read_file(directory / "scenario.json");
    const std::string manifest = read_file(directory / "run_manifest.json");
    EXPECT_EQ(summary.cn0_model_semantic, "NORMALIZED_ELEVATION_SHAPE");
    EXPECT_NE(manifest.find("\"semantic\": \"NORMALIZED_ELEVATION_SHAPE\""), std::string::npos);
    EXPECT_NE(scenario.find("\"GPS L1 C/A\": 47"), std::string::npos);
    EXPECT_NE(manifest.find("\"GPS L1 C/A\": 47"), std::string::npos);

    cleanup(directory);
    std::remove(model_path.string().c_str());
}

TEST(TruthOutputs, NormalizedCn0MissingRequiredBaselineFailsBeforeReceiverOutputCreation) {
    const std::filesystem::path directory = "gnss_sim_truth_cn0_missing_baseline";
    const std::filesystem::path model_path = "gnss_sim_runtime_cn0_missing_baseline.csv";
    {
        std::ofstream output(model_path, std::ios::binary | std::ios::trunc);
        output << "schema_version,model_semantic,constellation,signal,elevation_min_deg,elevation_max_deg,"
                  "upper_edge_inclusive,status,contributing_source_count,delta_p50_db\n";
        output << "gnss-cn0-model-v2,NORMALIZED_ELEVATION_SHAPE,GPS,1C,0,90,1,READY,2,-2.000000\n";
    }

    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    EXPECT_FALSE(
        run_config_in_directory(directory, truth_config(), &summary, &error_message, model_path.string().c_str()));
    EXPECT_NE(error_message.find("cn0_high_dbhz"), std::string::npos);
    EXPECT_FALSE(std::filesystem::exists(directory / "simulated.log"));
    EXPECT_FALSE(std::filesystem::exists(directory / "run_manifest.json"));

    cleanup(directory);
    std::remove(model_path.string().c_str());
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
