#include "gnss/navigation_state.h"
#include "gnss/rtklib_adapter.h"
#include "gnss/satellite_engine.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "model/atmosphere_model.h"
#include "model/measurement_model.h"
#include "model/receiver_truth.h"
#include "model/signal_tracking.h"
#include "solution/solution_engine.h"

#include <cmath>
#include <cstdio>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr int kSatelliteCount = 8;
constexpr double kLoopbackElevationMaskDeg = -90.0;

std::string loopback_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/gps_loopback_nav.rnx";
}

struct NavGuard {
    gnss_sim::RtklibNavStore* store;
    ~NavGuard() {
        gnss_sim::destroy_rtklib_nav_store(store);
    }
};

struct NavigationGuard {
    gnss_sim::NavigationState* state;
    ~NavigationGuard() {
        gnss_sim::destroy_navigation_state(state);
    }
};

bool make_receiver(gnss_sim::ReceiverTruth* receiver, std::string* error_message) {
    gnss_sim::ReceiverConfig config{};
    config.latitude_deg = 20.0;
    config.longitude_deg = 120.0;
    config.height_m = 100.0;
    return gnss_sim::make_static_receiver_truth(config, receiver, error_message);
}

gnss_sim::SignalTracker tracking_gps_l1ca(const gnss_sim::SimTime& tracking_start) {
    gnss_sim::SignalTracker tracker{};
    tracker.signal_id = gnss_sim::SignalId::kGpsL1Ca;
    tracker.phase = gnss_sim::SignalTrackingPhase::kTracking;
    tracker.tracking_start_time = tracking_start;
    tracker.cn0_dbhz = 45.0;
    tracker.lock_time_ns = 10 * gnss_sim::NANOSECONDS_PER_SECOND;
    tracker.scheduled = true;
    tracker.observation_available = true;
    tracker.psr_valid = true;
    tracker.doppler_valid = true;
    tracker.adr_valid = true;
    return tracker;
}

bool generate_measurements(const gnss_sim::RtklibNavStore* truth_nav, const gnss_sim::SimTime& epoch_time,
                           gnss_sim::MeasurementObservation measurements[kSatelliteCount], std::string* error_message) {
    gnss_sim::ReceiverTruth receiver{};
    if (!make_receiver(&receiver, error_message)) {
        return false;
    }

    gnss_sim::SimTime tracking_start{};
    if (!gnss_sim::add_time_ns(epoch_time, -10 * gnss_sim::NANOSECONDS_PER_SECOND, &tracking_start)) {
        return false;
    }

    gnss_sim::AtmosphereCorrection atmosphere{};
    atmosphere.mode = gnss_sim::AtmosphereMode::NONE;

    for (int index = 0; index < kSatelliteCount; ++index) {
        char satellite_id[4]{};
        std::snprintf(satellite_id, sizeof(satellite_id), "G%02d", index + 1);
        int satellite_number = 0;
        if (!gnss_sim::rtklib_satellite_id_to_number(satellite_id, &satellite_number)) {
            return false;
        }

        gnss_sim::SatelliteGeometry geometry{};
        if (!gnss_sim::compute_satellite_geometry(truth_nav, receiver, epoch_time, satellite_number,
                                                  kLoopbackElevationMaskDeg, &geometry, error_message)) {
            return false;
        }

        gnss_sim::SignalTracker tracker = tracking_gps_l1ca(tracking_start);
        gnss_sim::CarrierAmbiguityState ambiguity{};
        if (!gnss_sim::generate_zero_noise_measurement(truth_nav, geometry, tracker, atmosphere, &ambiguity,
                                                       &measurements[index], error_message)) {
            return false;
        }
    }
    return true;
}

void expect_static_truth(const gnss_sim::SolutionEpoch& solution) {
    ASSERT_TRUE(solution.position.valid);
    ASSERT_TRUE(solution.velocity.valid);
    EXPECT_EQ(solution.position.status, gnss_sim::ReceiverSolutionStatus::kSolComputed);
    EXPECT_EQ(solution.position.type, gnss_sim::ReceiverSolutionType::kSingle);
    EXPECT_EQ(solution.velocity.status, gnss_sim::ReceiverSolutionStatus::kSolComputed);
    EXPECT_EQ(solution.velocity.type, gnss_sim::ReceiverSolutionType::kSingle);
    EXPECT_NEAR(solution.position.latitude_deg, 20.0, 1.0e-6);
    EXPECT_NEAR(solution.position.longitude_deg, 120.0, 1.0e-6);
    EXPECT_NEAR(solution.position.height_m, 100.0, 0.10);
    EXPECT_NEAR(solution.position.receiver_clock_bias_m, 0.0, 0.10);
    EXPECT_NEAR(solution.velocity.velocity_ecef_mps[0], 0.0, 1.0e-3);
    EXPECT_NEAR(solution.velocity.velocity_ecef_mps[1], 0.0, 1.0e-3);
    EXPECT_NEAR(solution.velocity.velocity_ecef_mps[2], 0.0, 1.0e-3);
    EXPECT_NEAR(solution.velocity.receiver_clock_drift_mps, 0.0, 1.0e-3);
    EXPECT_GE(solution.position.used_satellites, 4);
    EXPECT_GE(solution.velocity.used_satellites, 4);
}

TEST(SolutionEngine, IdealStaticZeroNoiseLoopbackRecoversTruth) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, loopback_nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::SimTime epoch_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2253, 172900.0, &epoch_time));
    gnss_sim::MeasurementObservation measurements[kSatelliteCount]{};
    ASSERT_TRUE(generate_measurements(nav.store, epoch_time, measurements, &error_message)) << error_message;

    gnss_sim::SolutionEngineState state{};
    gnss_sim::SolutionEpoch solution{};
    ASSERT_TRUE(gnss_sim::solve_receiver_epoch(nav.store, epoch_time, measurements, kSatelliteCount,
                                               kLoopbackElevationMaskDeg, gnss_sim::AtmosphereMode::NONE, &state,
                                               &solution, &error_message))
        << error_message;
    expect_static_truth(solution);
    EXPECT_TRUE(state.has_position_hint);
}

TEST(SolutionEngine, PositionAndVelocityValidityAreIndependent) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, loopback_nav_path().c_str(), &error_message));

    gnss_sim::SimTime epoch_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2253, 172900.0, &epoch_time));
    gnss_sim::MeasurementObservation measurements[kSatelliteCount]{};
    ASSERT_TRUE(generate_measurements(nav.store, epoch_time, measurements, &error_message));

    gnss_sim::MeasurementObservation position_only[kSatelliteCount]{};
    for (int index = 0; index < kSatelliteCount; ++index) {
        position_only[index] = measurements[index];
        position_only[index].doppler_valid = false;
    }
    gnss_sim::SolutionEngineState position_state{};
    gnss_sim::SolutionEpoch position_solution{};
    ASSERT_TRUE(gnss_sim::solve_receiver_epoch(nav.store, epoch_time, position_only, kSatelliteCount,
                                               kLoopbackElevationMaskDeg, gnss_sim::AtmosphereMode::NONE,
                                               &position_state, &position_solution, &error_message));
    EXPECT_TRUE(position_solution.position.valid);
    EXPECT_FALSE(position_solution.velocity.valid);
    EXPECT_EQ(position_solution.velocity.status, gnss_sim::ReceiverSolutionStatus::kInsufficientObs);
    EXPECT_EQ(position_solution.velocity.type, gnss_sim::ReceiverSolutionType::kNone);

    gnss_sim::SolutionEngineState velocity_state{};
    gnss_sim::SolutionEpoch seed_solution{};
    ASSERT_TRUE(gnss_sim::solve_receiver_epoch(nav.store, epoch_time, measurements, kSatelliteCount,
                                               kLoopbackElevationMaskDeg, gnss_sim::AtmosphereMode::NONE,
                                               &velocity_state, &seed_solution, &error_message));
    ASSERT_TRUE(seed_solution.position.valid);

    gnss_sim::SimTime next_time{};
    ASSERT_TRUE(gnss_sim::add_time_ns(epoch_time, gnss_sim::NANOSECONDS_PER_SECOND, &next_time));
    gnss_sim::MeasurementObservation velocity_only[kSatelliteCount]{};
    ASSERT_TRUE(generate_measurements(nav.store, next_time, velocity_only, &error_message));
    for (int index = 0; index < kSatelliteCount; ++index) {
        velocity_only[index].pseudorange_valid = false;
    }
    gnss_sim::SolutionEpoch velocity_solution{};
    ASSERT_TRUE(gnss_sim::solve_receiver_epoch(nav.store, next_time, velocity_only, kSatelliteCount,
                                               kLoopbackElevationMaskDeg, gnss_sim::AtmosphereMode::NONE,
                                               &velocity_state, &velocity_solution, &error_message));
    EXPECT_FALSE(velocity_solution.position.valid);
    EXPECT_TRUE(velocity_solution.velocity.valid);
    EXPECT_EQ(velocity_solution.position.status, gnss_sim::ReceiverSolutionStatus::kInsufficientObs);
    EXPECT_EQ(velocity_solution.position.type, gnss_sim::ReceiverSolutionType::kNone);
    EXPECT_NEAR(velocity_solution.velocity.velocity_ecef_mps[0], 0.0, 1.0e-3);
    EXPECT_NEAR(velocity_solution.velocity.velocity_ecef_mps[1], 0.0, 1.0e-3);
    EXPECT_NEAR(velocity_solution.velocity.velocity_ecef_mps[2], 0.0, 1.0e-3);
}

TEST(SolutionEngine, VelocityWithoutAnyPositionHintRemainsInvalid) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, loopback_nav_path().c_str(), &error_message));
    gnss_sim::SimTime epoch_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2253, 172900.0, &epoch_time));
    gnss_sim::MeasurementObservation measurements[kSatelliteCount]{};
    ASSERT_TRUE(generate_measurements(nav.store, epoch_time, measurements, &error_message));
    for (int index = 0; index < kSatelliteCount; ++index) {
        measurements[index].pseudorange_valid = false;
    }

    gnss_sim::SolutionEngineState state{};
    gnss_sim::SolutionEpoch solution{};
    ASSERT_TRUE(gnss_sim::solve_receiver_epoch(nav.store, epoch_time, measurements, kSatelliteCount,
                                               kLoopbackElevationMaskDeg, gnss_sim::AtmosphereMode::NONE, &state,
                                               &solution, &error_message));
    EXPECT_FALSE(solution.position.valid);
    EXPECT_FALSE(solution.velocity.valid);
}

TEST(SolutionEngine, ColdReceiverCannotUseTruthOnlyEphemeris) {
    NavigationGuard navigation{gnss_sim::create_navigation_state()};
    ASSERT_NE(navigation.state, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_truth_navigation(navigation.state, loopback_nav_path().c_str(), &error_message));
    gnss_sim::SimTime epoch_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2253, 172900.0, &epoch_time));
    ASSERT_TRUE(gnss_sim::initialize_receiver_navigation(navigation.state, gnss_sim::StartupMode::COLD, epoch_time,
                                                        &error_message));

    const gnss_sim::RtklibNavStore* truth_nav = gnss_sim::truth_navigation_store(navigation.state);
    const gnss_sim::RtklibNavStore* receiver_nav = gnss_sim::receiver_navigation_store(navigation.state);
    ASSERT_NE(truth_nav, nullptr);
    ASSERT_NE(receiver_nav, nullptr);
    gnss_sim::MeasurementObservation measurements[kSatelliteCount]{};
    ASSERT_TRUE(generate_measurements(truth_nav, epoch_time, measurements, &error_message));

    gnss_sim::SolutionEngineState state{};
    gnss_sim::SolutionEpoch solution{};
    ASSERT_TRUE(gnss_sim::solve_receiver_epoch(receiver_nav, epoch_time, measurements, kSatelliteCount,
                                               kLoopbackElevationMaskDeg, gnss_sim::AtmosphereMode::NONE, &state,
                                               &solution, &error_message));
    EXPECT_FALSE(solution.position.valid);
    EXPECT_FALSE(solution.velocity.valid);
    EXPECT_FALSE(state.has_position_hint);
}

TEST(SolutionEngine, ZeroObservationEpochProducesInvalidStatesWithoutFailure) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, loopback_nav_path().c_str(), &error_message));
    gnss_sim::SimTime epoch_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2253, 172900.0, &epoch_time));

    gnss_sim::SolutionEngineState state{};
    gnss_sim::SolutionEpoch solution{};
    ASSERT_TRUE(gnss_sim::solve_receiver_epoch(nav.store, epoch_time, nullptr, 0, kLoopbackElevationMaskDeg,
                                               gnss_sim::AtmosphereMode::NONE, &state, &solution, &error_message));
    EXPECT_FALSE(solution.position.valid);
    EXPECT_EQ(solution.position.status, gnss_sim::ReceiverSolutionStatus::kInsufficientObs);
    EXPECT_EQ(solution.position.type, gnss_sim::ReceiverSolutionType::kNone);
    EXPECT_FALSE(solution.velocity.valid);
    EXPECT_EQ(solution.velocity.status, gnss_sim::ReceiverSolutionStatus::kInsufficientObs);
    EXPECT_EQ(solution.velocity.type, gnss_sim::ReceiverSolutionType::kNone);
    EXPECT_STREQ(gnss_sim::receiver_solution_status_name(solution.position.status), "INSUFFICIENT_OBS");
    EXPECT_STREQ(gnss_sim::receiver_solution_type_name(solution.position.type), "NONE");
}

} // namespace
