#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"
#include "output/carrier_tracking_truth_writer.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

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

std::vector<std::vector<std::string>> read_csv(const std::filesystem::path& path) {
    std::vector<std::vector<std::string>> rows;
    std::ifstream input(path, std::ios::binary);
    std::string line;
    while (std::getline(input, line)) {
        rows.push_back(split_csv_line(line));
    }
    return rows;
}

std::unordered_map<std::string, std::size_t> header_index(const std::vector<std::string>& header) {
    std::unordered_map<std::string, std::size_t> result;
    for (std::size_t index = 0; index < header.size(); ++index) {
        result.emplace(header[index], index);
    }
    return result;
}

bool run_sim(const std::filesystem::path& directory, const gnss_sim::SimConfig& config,
             gnss_sim::SimulatorRunSummary* summary, std::string* error_message) {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    error.clear();
    if (!std::filesystem::create_directories(directory, error) || error) {
        if (error_message != nullptr) {
            *error_message = "cannot create carrier truth test directory";
        }
        return false;
    }
    const std::filesystem::path output_path = directory / "simulated.log";
    const std::string nav = nav_path();
    const std::string output = output_path.string();
    const gnss_sim::SimulatorRunOptions options{nav.c_str(), output.c_str(), start_time(), nullptr};
    return gnss_sim::run_simulator(config, options, summary, error_message);
}

void cleanup(const std::filesystem::path& path) {
    std::error_code error;
    std::filesystem::remove_all(path, error);
}

void expect_consistent_columns(const std::vector<std::vector<std::string>>& rows) {
    ASSERT_GT(rows.size(), 1U);
    const std::size_t width = rows.front().size();
    for (std::size_t row = 1; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row].size(), width) << "row " << row + 1U;
    }
}

TEST(CarrierTrackingTruth, WriterSerializesExactSnapshotWithoutRecomputation) {
    const std::filesystem::path directory = "gnss_sim_carrier_truth_writer";
    cleanup(directory);
    ASSERT_TRUE(std::filesystem::create_directories(directory));

    std::string error_message;
    const std::filesystem::path receiver_path = directory / "simulated.log";
    gnss_sim::CarrierTrackingTruthWriter* writer =
        gnss_sim::create_carrier_tracking_truth_writer(receiver_path.string().c_str(), &error_message);
    ASSERT_NE(writer, nullptr) << error_message;

    gnss_sim::SatelliteGeometry geometry{};
    geometry.satellite_number = 1;
    geometry.receive_time = start_time();
    const gnss_sim::SignalDefinition* signal = gnss_sim::find_signal_definition(gnss_sim::SignalId::kGpsL1Ca);
    ASSERT_NE(signal, nullptr);

    gnss_sim::SignalTracker tracker{};
    tracker.phase = gnss_sim::SignalTrackingPhase::kTracking;
    tracker.acquisition_context = gnss_sim::AcquisitionContext::kReacquisition;
    tracker.loss_reason = gnss_sim::SignalTrackingLossReason::kLowCn0;

    gnss_sim::CarrierTrackingTruthSnapshot snapshot{};
    snapshot.carrier_tracking_enabled = true;
    snapshot.result_available = true;
    snapshot.reset_reason = gnss_sim::CarrierTrackingTruthResetReason::kNone;
    snapshot.coherent_integration_sec = 0.02;
    snapshot.effective_cn0_dbhz = 26.5;
    snapshot.runtime_result.tracking.mode = gnss_sim::CarrierTrackingMode::kFllTrack;
    snapshot.runtime_result.tracking.fll_phase = gnss_sim::CarrierTrackingFllPhase::kSteady;
    snapshot.runtime_result.tracking.jitter.cn0_linear_hz = 446.683592150963;
    snapshot.runtime_result.tracking.jitter.active_bandwidth_hz = 4.0;
    snapshot.runtime_result.tracking.jitter.phase_sigma_rad = 0.125;
    snapshot.runtime_result.tracking.jitter.sigma_hz = 1.25;
    snapshot.runtime_result.tracking.jitter.sigma_mps = 0.25;
    snapshot.runtime_result.tracking.jitter.correlation_tau_sec = 0.04;
    snapshot.runtime_result.tracking.jitter.correlation_alpha = 0.6;
    snapshot.runtime_result.tracking.tracking_error_hz = 2.5;
    snapshot.runtime_result.tracking.tracking_error_mps = 0.5;
    snapshot.runtime_result.tracking.carrier_segment_id = 7U;
    snapshot.runtime_result.tracking.doppler_valid = true;
    snapshot.runtime_result.tracking.adr_valid = false;
    snapshot.runtime_result.tracking.mode_changed = true;
    snapshot.runtime_result.tracking.new_carrier_segment = false;
    snapshot.runtime_result.phase_segment_id = 3U;
    snapshot.runtime_result.adr_cycle_offset_cycles = -17;
    snapshot.runtime_result.cycle_slip_event = true;
    snapshot.runtime_state.mode = gnss_sim::CarrierTrackingMode::kFllTrack;
    snapshot.runtime_state.mode_age_sec = 0.7;
    snapshot.runtime_state.carrier_lock_age_sec = 1.7;
    snapshot.runtime_state.pll_age_sec = 0.0;
    snapshot.runtime_state.fll_enter_persistence_sec = 0.1;
    snapshot.runtime_state.fll_exit_persistence_sec = 0.2;
    snapshot.runtime_state.pll_enter_persistence_sec = 0.3;
    snapshot.runtime_state.pll_exit_persistence_sec = 0.4;
    snapshot.environmental_range_rate_applicable = true;
    snapshot.environmental_range_rate_valid = true;
    snapshot.environmental_range_rate_mps = -0.75;
    snapshot.physical_snapshot_available = true;
    snapshot.physical_observation.observation_available = true;
    snapshot.physical_observation.pseudorange_valid = true;
    snapshot.physical_observation.doppler_valid = true;
    snapshot.physical_observation.adr_valid = true;
    snapshot.physical_observation.range_rate_mps = 12.0;
    snapshot.physical_observation.doppler_hz = -50.0;
    snapshot.physical_observation.adr_cycles = 1000.25;
    snapshot.physical_range_rate_valid = true;
    snapshot.post_carrier_snapshot_available = true;
    snapshot.post_carrier_observation = snapshot.physical_observation;
    snapshot.post_carrier_observation.range_rate_mps = 11.5;
    snapshot.post_carrier_observation.doppler_hz = -47.5;
    snapshot.post_carrier_observation.adr_valid = false;
    snapshot.post_carrier_range_rate_valid = true;

    ASSERT_TRUE(gnss_sim::carrier_tracking_truth_writer_write_signal(writer, geometry, *signal, 0, tracker, snapshot,
                                                                     &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::finalize_carrier_tracking_truth_writer(writer, &error_message)) << error_message;
    gnss_sim::destroy_carrier_tracking_truth_writer(writer);

    const std::vector<std::vector<std::string>> rows = read_csv(directory / "carrier_tracking_truth.csv");
    ASSERT_EQ(rows.size(), 2U);
    expect_consistent_columns(rows);
    const auto columns = header_index(rows[0]);
    const std::vector<std::string>& row = rows[1];
    EXPECT_EQ(row.at(columns.at("carrier_truth_schema_version")), "1");
    EXPECT_EQ(row.at(columns.at("carrier_mode")), "FLL_TRACK");
    EXPECT_EQ(row.at(columns.at("fll_phase")), "STEADY");
    EXPECT_EQ(row.at(columns.at("acquisition_context")), "REACQUISITION");
    EXPECT_EQ(row.at(columns.at("signal_loss_reason")), "LOW_CN0");
    EXPECT_DOUBLE_EQ(std::stod(row.at(columns.at("tracking_error_hz"))), 2.5);
    EXPECT_DOUBLE_EQ(std::stod(row.at(columns.at("tracking_error_mps"))), 0.5);
    EXPECT_DOUBLE_EQ(std::stod(row.at(columns.at("mode_age_sec"))), 0.7);
    EXPECT_DOUBLE_EQ(std::stod(row.at(columns.at("pll_enter_persistence_sec"))), 0.3);
    EXPECT_EQ(std::stoull(row.at(columns.at("carrier_segment_id"))), 7U);
    EXPECT_EQ(std::stoull(row.at(columns.at("phase_segment_id"))), 3U);
    EXPECT_EQ(std::stoll(row.at(columns.at("adr_cycle_offset_cycles"))), -17);
    EXPECT_EQ(row.at(columns.at("cycle_slip_event")), "1");
    EXPECT_DOUBLE_EQ(std::stod(row.at(columns.at("environmental_range_rate_mps"))), -0.75);
    EXPECT_DOUBLE_EQ(std::stod(row.at(columns.at("physical_doppler_hz"))), -50.0);
    EXPECT_DOUBLE_EQ(std::stod(row.at(columns.at("post_carrier_doppler_hz"))), -47.5);
    EXPECT_EQ(row.at(columns.at("physical_adr_valid")), "1");
    EXPECT_EQ(row.at(columns.at("post_carrier_adr_valid")), "0");
    cleanup(directory);
}

TEST(CarrierTrackingTruth, EnabledSimulatorRowsAreDeterministicAndExposeCarrierBoundary) {
    const std::filesystem::path first_dir = "gnss_sim_carrier_truth_enabled_a";
    const std::filesystem::path second_dir = "gnss_sim_carrier_truth_enabled_b";
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::KS;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    config.elevation_mask_deg = 0.0;
    config.sampling_rate_hz = 10;
    config.duration_ns = 6LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.seed = 7U;
    config.multipath_enabled = true;
    config.measurement_noise_enabled = false;
    config.carrier_tracking.enabled = true;

    gnss_sim::SimulatorRunSummary first_summary{};
    gnss_sim::SimulatorRunSummary second_summary{};
    std::string error_message;
    ASSERT_TRUE(run_sim(first_dir, config, &first_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_sim(second_dir, config, &second_summary, &error_message)) << error_message;
    EXPECT_EQ(read_file(first_dir / "carrier_tracking_truth.csv"),
              read_file(second_dir / "carrier_tracking_truth.csv"));

    const std::vector<std::vector<std::string>> rows = read_csv(first_dir / "carrier_tracking_truth.csv");
    expect_consistent_columns(rows);
    const auto columns = header_index(rows[0]);
    bool saw_result = false;
    bool saw_environmental_range_rate = false;
    bool saw_unlocked_code_only = false;
    bool saw_fll_doppler_without_adr = false;
    bool saw_pull_in = false;
    bool saw_pll = false;
    for (std::size_t index = 1; index < rows.size(); ++index) {
        const std::vector<std::string>& row = rows[index];
        if (row.at(columns.at("environmental_range_rate_applicable")) == "1" &&
            row.at(columns.at("environmental_range_rate_valid")) == "1") {
            saw_environmental_range_rate = true;
        }
        if (row.at(columns.at("carrier_result_available")) != "1" ||
            row.at(columns.at("physical_snapshot_available")) != "1" ||
            row.at(columns.at("post_carrier_snapshot_available")) != "1") {
            continue;
        }
        saw_result = true;
        const double physical_doppler = std::stod(row.at(columns.at("physical_doppler_hz")));
        const double post_doppler = std::stod(row.at(columns.at("post_carrier_doppler_hz")));
        const double error_hz = std::stod(row.at(columns.at("tracking_error_hz")));
        const double physical_rate = std::stod(row.at(columns.at("physical_range_rate_mps")));
        const double post_rate = std::stod(row.at(columns.at("post_carrier_range_rate_mps")));
        const double error_mps = std::stod(row.at(columns.at("tracking_error_mps")));
        EXPECT_NEAR(post_doppler, physical_doppler + error_hz, 1e-10);
        EXPECT_NEAR(post_rate, physical_rate - error_mps, 1e-10);

        const std::string& mode = row.at(columns.at("carrier_mode"));
        const bool code_valid = row.at(columns.at("post_carrier_code_valid")) == "1";
        const bool doppler_valid = row.at(columns.at("post_carrier_doppler_valid")) == "1";
        const bool adr_valid = row.at(columns.at("post_carrier_adr_valid")) == "1";
        if (mode == "CARRIER_UNLOCKED" && code_valid && !doppler_valid && !adr_valid) {
            saw_unlocked_code_only = true;
        }
        if (mode == "FLL_TRACK" && code_valid && doppler_valid && !adr_valid) {
            saw_fll_doppler_without_adr = true;
        }
        if (row.at(columns.at("fll_phase")) == "PULL_IN") {
            saw_pull_in = true;
        }
        if (mode == "PLL_TRACK") {
            saw_pll = true;
        }
    }
    EXPECT_TRUE(saw_result);
    EXPECT_TRUE(saw_environmental_range_rate);
    EXPECT_TRUE(saw_unlocked_code_only);
    EXPECT_TRUE(saw_fll_doppler_without_adr);
    EXPECT_TRUE(saw_pull_in);
    EXPECT_TRUE(saw_pll);
    EXPECT_EQ(read_file(first_dir / "simulated.log").find("carrier_mode"), std::string::npos);
    EXPECT_EQ(read_file(first_dir / "simulated.log").find("tracking_error_hz"), std::string::npos);

    cleanup(first_dir);
    cleanup(second_dir);
}

TEST(CarrierTrackingTruth, DisabledFeatureIsExplicitWhileCarrierBoundaryIsIdentity) {
    const std::filesystem::path directory = "gnss_sim_carrier_truth_disabled";
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::KS;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    config.elevation_mask_deg = 0.0;
    config.sampling_rate_hz = 10;
    config.duration_ns = 4LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.seed = 7U;
    ASSERT_FALSE(config.carrier_tracking.enabled);

    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run_sim(directory, config, &summary, &error_message)) << error_message;
    const std::vector<std::vector<std::string>> rows = read_csv(directory / "carrier_tracking_truth.csv");
    expect_consistent_columns(rows);
    const auto columns = header_index(rows[0]);
    bool saw_disabled_tracking_row = false;
    bool saw_code_not_tracking_row = false;
    for (std::size_t index = 1; index < rows.size(); ++index) {
        const std::vector<std::string>& row = rows[index];
        EXPECT_EQ(row.at(columns.at("carrier_tracking_enabled")), "0");
        EXPECT_EQ(row.at(columns.at("carrier_result_available")), "0");
        if (row.at(columns.at("carrier_reset_reason")) == "CODE_NOT_TRACKING") {
            saw_code_not_tracking_row = true;
        }
        if (row.at(columns.at("carrier_reset_reason")) != "FEATURE_DISABLED" ||
            row.at(columns.at("physical_snapshot_available")) != "1") {
            continue;
        }
        saw_disabled_tracking_row = true;
        EXPECT_EQ(row.at(columns.at("physical_doppler_hz")), row.at(columns.at("post_carrier_doppler_hz")));
        EXPECT_EQ(row.at(columns.at("physical_range_rate_mps")), row.at(columns.at("post_carrier_range_rate_mps")));
        EXPECT_EQ(row.at(columns.at("physical_adr_cycles")), row.at(columns.at("post_carrier_adr_cycles")));
    }
    EXPECT_TRUE(saw_disabled_tracking_row);
    EXPECT_TRUE(saw_code_not_tracking_row);
    cleanup(directory);
}

} // namespace
