#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"
#include "model/carrier_tracking_runtime.h"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr double kWavelengthM = 0.190293672798365;

std::string nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/gps_loopback_nav.rnx";
}

gnss_sim::SimTime make_time(double sow_sec) {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2253, sow_sec, &time));
    return time;
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

bool run_sim(const std::filesystem::path& directory, const gnss_sim::SimConfig& config,
             gnss_sim::SimulatorRunSummary* summary, std::string* error_message) {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    error.clear();
    if (!std::filesystem::create_directories(directory, error) || error) {
        if (error_message != nullptr) {
            *error_message = "cannot create carrier tracking runtime test directory";
        }
        return false;
    }
    const std::filesystem::path output_path = directory / "simulated.log";
    const std::string nav = nav_path();
    const std::string output = output_path.string();
    const gnss_sim::SimulatorRunOptions options{nav.c_str(), output.c_str(), make_time(172900.0), nullptr};
    return gnss_sim::run_simulator(config, options, summary, error_message);
}

void cleanup(const std::filesystem::path& path) {
    std::error_code error;
    std::filesystem::remove_all(path, error);
}

TEST(CarrierTrackingRuntime, SubstepsLowRateOutputAcrossPersistenceBoundaries) {
    gnss_sim::CarrierTrackingRuntimeState state{};
    gnss_sim::initialize_carrier_tracking_runtime_state(7U, 1, gnss_sim::SignalId::kGpsL1Ca, &state);
    const gnss_sim::CarrierTrackingReceiverConfig config = gnss_sim::default_sim_config().carrier_tracking;
    gnss_sim::CarrierTrackingRuntimeResult result{};
    std::string error_message;

    ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(100.0), true, 35.0, kWavelengthM, &state,
                                                          &result, &error_message))
        << error_message;
    EXPECT_EQ(result.tracking.mode, gnss_sim::CarrierTrackingMode::kCarrierUnlocked);
    EXPECT_FALSE(result.tracking.doppler_valid);

    ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(100.2), true, 35.0, kWavelengthM, &state,
                                                          &result, &error_message))
        << error_message;
    EXPECT_EQ(result.tracking.mode, gnss_sim::CarrierTrackingMode::kFllTrack);
    EXPECT_EQ(result.tracking.fll_phase, gnss_sim::CarrierTrackingFllPhase::kPullIn);
    EXPECT_FALSE(result.tracking.doppler_valid);

    ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(100.4), true, 35.0, kWavelengthM, &state,
                                                          &result, &error_message))
        << error_message;
    EXPECT_EQ(result.tracking.mode, gnss_sim::CarrierTrackingMode::kFllTrack);
    EXPECT_TRUE(result.tracking.doppler_valid);
    EXPECT_FALSE(result.tracking.adr_valid);

    ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(101.2), true, 35.0, kWavelengthM, &state,
                                                          &result, &error_message))
        << error_message;
    EXPECT_EQ(result.tracking.mode, gnss_sim::CarrierTrackingMode::kPllTrack);
    EXPECT_FALSE(result.tracking.adr_valid);

    ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(102.2), true, 35.0, kWavelengthM, &state,
                                                          &result, &error_message))
        << error_message;
    EXPECT_EQ(result.tracking.mode, gnss_sim::CarrierTrackingMode::kPllTrack);
    EXPECT_TRUE(result.tracking.doppler_valid);
    EXPECT_TRUE(result.tracking.adr_valid);
}

TEST(CarrierTrackingRuntime, PllLossImmediatelyBreaksAdrAndStartsNewPhaseSegment) {
    gnss_sim::CarrierTrackingRuntimeState state{};
    gnss_sim::initialize_carrier_tracking_runtime_state(11U, 3, gnss_sim::SignalId::kGpsL1Ca, &state);
    const gnss_sim::CarrierTrackingReceiverConfig config = gnss_sim::default_sim_config().carrier_tracking;
    gnss_sim::CarrierTrackingRuntimeResult result{};
    std::string error_message;

    ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(200.0), true, 35.0, kWavelengthM, &state,
                                                          &result, &error_message));
    ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(202.2), true, 35.0, kWavelengthM, &state,
                                                          &result, &error_message))
        << error_message;
    ASSERT_EQ(result.tracking.mode, gnss_sim::CarrierTrackingMode::kPllTrack);
    ASSERT_TRUE(result.tracking.adr_valid);
    EXPECT_EQ(result.phase_segment_id, 0U);
    EXPECT_EQ(result.adr_cycle_offset_cycles, 0);

    ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(202.5), true, 26.0, kWavelengthM, &state,
                                                          &result, &error_message))
        << error_message;
    EXPECT_EQ(result.tracking.mode, gnss_sim::CarrierTrackingMode::kFllTrack);
    EXPECT_FALSE(result.tracking.adr_valid);
    EXPECT_TRUE(result.cycle_slip_event);
    EXPECT_EQ(result.phase_segment_id, 1U);
    EXPECT_NE(result.adr_cycle_offset_cycles, 0);

    ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(203.5), true, 35.0, kWavelengthM, &state,
                                                          &result, &error_message))
        << error_message;
    EXPECT_EQ(result.tracking.mode, gnss_sim::CarrierTrackingMode::kPllTrack);
    EXPECT_FALSE(result.tracking.adr_valid);

    ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(204.5), true, 35.0, kWavelengthM, &state,
                                                          &result, &error_message))
        << error_message;
    EXPECT_TRUE(result.tracking.adr_valid);
    EXPECT_EQ(result.phase_segment_id, 1U);
    EXPECT_NE(result.adr_cycle_offset_cycles, 0);
}

TEST(CarrierTrackingRuntime, MeasurementApplicationPreservesDopplerRangeRateSignConvention) {
    gnss_sim::MeasurementObservation observation{};
    observation.wavelength_m = 0.2;
    observation.range_rate_mps = 12.0;
    observation.doppler_hz = -50.0;
    observation.adr_cycles = 1234.0;
    observation.ambiguity_cycles = 100;
    observation.observation_available = true;
    observation.pseudorange_valid = true;
    observation.doppler_valid = true;
    observation.adr_valid = true;

    gnss_sim::CarrierTrackingRuntimeResult carrier{};
    carrier.tracking.mode = gnss_sim::CarrierTrackingMode::kFllTrack;
    carrier.tracking.tracking_error_hz = 3.0;
    carrier.tracking.tracking_error_mps = 0.6;
    carrier.tracking.doppler_valid = true;
    carrier.tracking.adr_valid = false;

    std::string error_message;
    ASSERT_TRUE(gnss_sim::apply_carrier_tracking_runtime_result(carrier, &observation, &error_message)) << error_message;
    EXPECT_DOUBLE_EQ(observation.doppler_hz, -47.0);
    EXPECT_DOUBLE_EQ(observation.range_rate_mps, 11.4);
    EXPECT_TRUE(observation.pseudorange_valid);
    EXPECT_TRUE(observation.doppler_valid);
    EXPECT_FALSE(observation.adr_valid);
}

TEST(CarrierTrackingRuntime, CarrierUnlockedKeepsCodeButInvalidatesDopplerAndAdr) {
    gnss_sim::MeasurementObservation observation{};
    observation.wavelength_m = 0.2;
    observation.range_rate_mps = 12.0;
    observation.doppler_hz = -50.0;
    observation.adr_cycles = 1234.0;
    observation.observation_available = true;
    observation.pseudorange_valid = true;
    observation.doppler_valid = true;
    observation.adr_valid = true;

    gnss_sim::CarrierTrackingRuntimeResult carrier{};
    carrier.tracking.mode = gnss_sim::CarrierTrackingMode::kCarrierUnlocked;
    carrier.tracking.doppler_valid = false;
    carrier.tracking.adr_valid = false;

    std::string error_message;
    ASSERT_TRUE(gnss_sim::apply_carrier_tracking_runtime_result(carrier, &observation, &error_message)) << error_message;
    EXPECT_DOUBLE_EQ(observation.doppler_hz, -50.0);
    EXPECT_DOUBLE_EQ(observation.range_rate_mps, 12.0);
    EXPECT_TRUE(observation.pseudorange_valid);
    EXPECT_FALSE(observation.doppler_valid);
    EXPECT_FALSE(observation.adr_valid);
}

TEST(CarrierTrackingRuntime, NewPllSegmentUsesDifferentIntegerAmbiguityOffset) {
    gnss_sim::MeasurementObservation observation{};
    observation.wavelength_m = 0.2;
    observation.range_rate_mps = 0.0;
    observation.doppler_hz = 0.0;
    observation.adr_cycles = 1000.25;
    observation.ambiguity_cycles = 500;
    observation.observation_available = true;
    observation.pseudorange_valid = true;
    observation.doppler_valid = true;
    observation.adr_valid = true;

    gnss_sim::CarrierTrackingRuntimeResult carrier{};
    carrier.tracking.mode = gnss_sim::CarrierTrackingMode::kPllTrack;
    carrier.tracking.doppler_valid = true;
    carrier.tracking.adr_valid = true;
    carrier.adr_cycle_offset_cycles = 37;
    carrier.phase_segment_id = 1U;

    std::string error_message;
    ASSERT_TRUE(gnss_sim::apply_carrier_tracking_runtime_result(carrier, &observation, &error_message)) << error_message;
    EXPECT_EQ(observation.ambiguity_cycles, 537);
    EXPECT_DOUBLE_EQ(observation.adr_cycles, 1037.25);
}

TEST(CarrierTrackingRuntime, SamePerSignalSeedAndInputsAreDeterministic) {
    gnss_sim::CarrierTrackingRuntimeState first{};
    gnss_sim::CarrierTrackingRuntimeState second{};
    gnss_sim::initialize_carrier_tracking_runtime_state(42U, 5, gnss_sim::SignalId::kGpsL1Ca, &first);
    gnss_sim::initialize_carrier_tracking_runtime_state(42U, 5, gnss_sim::SignalId::kGpsL1Ca, &second);
    const gnss_sim::CarrierTrackingReceiverConfig config = gnss_sim::default_sim_config().carrier_tracking;
    std::string first_error;
    std::string second_error;

    for (int index = 0; index <= 30; ++index) {
        const double cn0 = index < 18 ? 35.0 : 26.0;
        gnss_sim::CarrierTrackingRuntimeResult first_result{};
        gnss_sim::CarrierTrackingRuntimeResult second_result{};
        ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(300.0 + 0.1 * index), true, cn0,
                                                              kWavelengthM, &first, &first_result, &first_error))
            << first_error;
        ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(300.0 + 0.1 * index), true, cn0,
                                                              kWavelengthM, &second, &second_result, &second_error))
            << second_error;
        EXPECT_EQ(first_result.tracking.mode, second_result.tracking.mode);
        EXPECT_DOUBLE_EQ(first_result.tracking.tracking_error_hz, second_result.tracking.tracking_error_hz);
        EXPECT_DOUBLE_EQ(first_result.tracking.tracking_error_mps, second_result.tracking.tracking_error_mps);
        EXPECT_EQ(first_result.tracking.doppler_valid, second_result.tracking.doppler_valid);
        EXPECT_EQ(first_result.tracking.adr_valid, second_result.tracking.adr_valid);
        EXPECT_EQ(first_result.phase_segment_id, second_result.phase_segment_id);
        EXPECT_EQ(first_result.adr_cycle_offset_cycles, second_result.adr_cycle_offset_cycles);
    }
}

TEST(CarrierTrackingRuntime, HardResetClearsTrackingAndContinuityState) {
    gnss_sim::CarrierTrackingRuntimeState state{};
    gnss_sim::initialize_carrier_tracking_runtime_state(19U, 7, gnss_sim::SignalId::kGpsL1Ca, &state);
    const gnss_sim::CarrierTrackingReceiverConfig config = gnss_sim::default_sim_config().carrier_tracking;
    gnss_sim::CarrierTrackingRuntimeResult result{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(400.0), true, 35.0, kWavelengthM, &state,
                                                          &result, &error_message));
    ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(402.2), true, 35.0, kWavelengthM, &state,
                                                          &result, &error_message));
    ASSERT_TRUE(result.tracking.adr_valid);

    gnss_sim::reset_carrier_tracking_runtime_state(&state);
    EXPECT_EQ(state.tracking.mode, gnss_sim::CarrierTrackingMode::kCarrierUnlocked);
    EXPECT_FALSE(state.time_initialized);
    EXPECT_EQ(state.phase_segment_id, 0U);
    EXPECT_EQ(state.adr_cycle_offset_cycles, 0);

    ASSERT_TRUE(gnss_sim::update_carrier_tracking_runtime(config, make_time(500.0), true, 35.0, kWavelengthM, &state,
                                                          &result, &error_message));
    EXPECT_EQ(result.tracking.mode, gnss_sim::CarrierTrackingMode::kCarrierUnlocked);
    EXPECT_FALSE(result.tracking.doppler_valid);
    EXPECT_FALSE(result.tracking.adr_valid);
}

TEST(CarrierTrackingRuntime, DisabledSimulatorFeaturePreservesBytesAndPhysicalUrbanTruth) {
    const std::filesystem::path baseline_dir = "gnss_sim_carrier_disabled_baseline";
    const std::filesystem::path altered_dir = "gnss_sim_carrier_disabled_altered";
    gnss_sim::SimConfig baseline = gnss_sim::default_sim_config();
    baseline.scenario = gnss_sim::ScenarioType::KS;
    baseline.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    baseline.elevation_mask_deg = 0.0;
    baseline.sampling_rate_hz = 10;
    baseline.duration_ns = 4LL * gnss_sim::NANOSECONDS_PER_SECOND;
    baseline.seed = 7U;
    baseline.multipath_enabled = true;
    ASSERT_FALSE(baseline.carrier_tracking.enabled);

    gnss_sim::SimConfig altered = baseline;
    altered.carrier_tracking.pll_noise_bandwidth_hz = 6.0;
    altered.carrier_tracking.fll_noise_bandwidth_hz = 5.0;
    altered.carrier_tracking.fll_pull_in_bandwidth_hz = 9.0;

    gnss_sim::SimulatorRunSummary first_summary{};
    gnss_sim::SimulatorRunSummary second_summary{};
    std::string error_message;
    ASSERT_TRUE(run_sim(baseline_dir, baseline, &first_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_sim(altered_dir, altered, &second_summary, &error_message)) << error_message;
    EXPECT_EQ(read_file(baseline_dir / "simulated.log"), read_file(altered_dir / "simulated.log"));
    EXPECT_EQ(read_file(baseline_dir / "urban_signal_truth.csv"), read_file(altered_dir / "urban_signal_truth.csv"));
    EXPECT_EQ(read_file(baseline_dir / "urban_path_truth.csv"), read_file(altered_dir / "urban_path_truth.csv"));

    cleanup(baseline_dir);
    cleanup(altered_dir);
}

TEST(CarrierTrackingRuntime, EnabledSimulatorFeatureIsDeterministicAndLeavesPhysicalTruthUnchanged) {
    const std::filesystem::path disabled_dir = "gnss_sim_carrier_physical_truth";
    const std::filesystem::path enabled_a_dir = "gnss_sim_carrier_enabled_a";
    const std::filesystem::path enabled_b_dir = "gnss_sim_carrier_enabled_b";
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::KS;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    config.elevation_mask_deg = 0.0;
    config.sampling_rate_hz = 10;
    config.duration_ns = 6LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.seed = 7U;
    config.multipath_enabled = true;

    gnss_sim::SimulatorRunSummary disabled_summary{};
    gnss_sim::SimulatorRunSummary enabled_a_summary{};
    gnss_sim::SimulatorRunSummary enabled_b_summary{};
    std::string error_message;
    ASSERT_TRUE(run_sim(disabled_dir, config, &disabled_summary, &error_message)) << error_message;

    config.carrier_tracking.enabled = true;
    ASSERT_TRUE(run_sim(enabled_a_dir, config, &enabled_a_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_sim(enabled_b_dir, config, &enabled_b_summary, &error_message)) << error_message;

    EXPECT_EQ(read_file(enabled_a_dir / "simulated.log"), read_file(enabled_b_dir / "simulated.log"));
    EXPECT_NE(read_file(disabled_dir / "simulated.log"), read_file(enabled_a_dir / "simulated.log"));
    EXPECT_EQ(read_file(disabled_dir / "urban_signal_truth.csv"), read_file(enabled_a_dir / "urban_signal_truth.csv"));
    EXPECT_EQ(read_file(disabled_dir / "urban_path_truth.csv"), read_file(enabled_a_dir / "urban_path_truth.csv"));

    cleanup(disabled_dir);
    cleanup(enabled_a_dir);
    cleanup(enabled_b_dir);
}

} // namespace