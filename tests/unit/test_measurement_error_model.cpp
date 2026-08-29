#include "gnss_sim/sim_time.h"
#include "model/measurement_error_model.h"

#include <cmath>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr std::int64_t kSecondNs = gnss_sim::NANOSECONDS_PER_SECOND;

gnss_sim::SimTime make_time(std::int64_t tow_ns) {
    gnss_sim::SimTime time{};
    time.gps_week = 2300;
    time.tow_ns = tow_ns;
    return time;
}

gnss_sim::SignalTracker make_tracker(gnss_sim::SignalId signal_id, std::int64_t tracking_start_ns,
                                     std::int64_t lock_time_ns) {
    gnss_sim::SignalTracker tracker{};
    tracker.signal_id = signal_id;
    tracker.phase = gnss_sim::SignalTrackingPhase::kTracking;
    tracker.acquisition_context = gnss_sim::AcquisitionContext::kHot;
    tracker.tracking_start_time = make_time(tracking_start_ns);
    tracker.lock_time_ns = lock_time_ns;
    tracker.psr_valid = true;
    tracker.doppler_valid = true;
    tracker.adr_valid = true;
    tracker.observation_available = true;
    return tracker;
}

gnss_sim::MeasurementObservation make_observation(gnss_sim::SignalId signal_id, int satellite_number,
                                                  double wavelength_m = 0.190293672798) {
    gnss_sim::MeasurementObservation observation{};
    observation.signal_id = signal_id;
    observation.satellite_number = satellite_number;
    observation.wavelength_m = wavelength_m;
    observation.pseudorange_m = 24000000.0;
    observation.doppler_hz = -1234.5;
    observation.adr_cycles = 126000000.0;
    observation.cn0_dbhz = 42.0;
    observation.observation_available = true;
    observation.pseudorange_valid = true;
    observation.doppler_valid = true;
    observation.adr_valid = true;
    return observation;
}

gnss_sim::MeasurementErrorContext stable_context() {
    return {gnss_sim::MeasurementErrorPhase::kStable, 0.0};
}

TEST(MeasurementErrorModel, FrozenSurveyGradeEnvelopeDecaysFromTtffHotToStable) {
    const gnss_sim::MeasurementErrorConfig config = gnss_sim::default_sim_config().measurement_error;
    const gnss_sim::MeasurementErrorContext context{gnss_sim::MeasurementErrorPhase::kTtffHot, 0.0};

    gnss_sim::MeasurementErrorSigmas initial{};
    gnss_sim::MeasurementErrorSigmas after_two_seconds{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_measurement_error_sigmas(config, context, 0, &initial, &error_message))
        << error_message;
    ASSERT_TRUE(
        gnss_sim::compute_measurement_error_sigmas(config, context, 2 * kSecondNs, &after_two_seconds, &error_message))
        << error_message;

    EXPECT_NEAR(initial.psr_sigma_m, std::hypot(0.08, 0.40), 1e-12);
    EXPECT_NEAR(initial.doppler_sigma_mps, std::hypot(0.03, 0.10), 1e-12);
    EXPECT_NEAR(initial.cn0_sigma_dbhz, std::hypot(0.5, 1.5), 1e-12);
    EXPECT_LT(after_two_seconds.psr_sigma_m, initial.psr_sigma_m);
    EXPECT_LT(after_two_seconds.doppler_sigma_mps, initial.doppler_sigma_mps);
    EXPECT_LT(after_two_seconds.cn0_sigma_dbhz, initial.cn0_sigma_dbhz);
    EXPECT_GT(after_two_seconds.psr_sigma_m, config.psr_sigma_m);
}

TEST(MeasurementErrorModel, ReaFadeUsesNormalizedProgressAndCn0Drop) {
    const gnss_sim::MeasurementErrorConfig config = gnss_sim::default_sim_config().measurement_error;
    const gnss_sim::MeasurementErrorContext context{gnss_sim::MeasurementErrorPhase::kReaFadeOut, 0.5};

    gnss_sim::MeasurementErrorSigmas sigmas{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_measurement_error_sigmas(config, context, 10 * kSecondNs, &sigmas, &error_message))
        << error_message;

    EXPECT_NEAR(sigmas.psr_sigma_m, std::hypot(0.08, 0.40), 1e-12);
    EXPECT_NEAR(sigmas.doppler_sigma_mps, std::hypot(0.03, 0.10), 1e-12);
    EXPECT_DOUBLE_EQ(sigmas.adr_sigma_m, 0.001);
    EXPECT_DOUBLE_EQ(sigmas.cn0_sigma_dbhz, 0.5);
    EXPECT_DOUBLE_EQ(sigmas.cn0_drop_db, 2.25);
}

TEST(MeasurementErrorModel, FixedSeedAndLockProduceIdenticalSequences) {
    const gnss_sim::MeasurementErrorConfig config = gnss_sim::default_sim_config().measurement_error;
    const gnss_sim::MeasurementObservation truth = make_observation(gnss_sim::SignalId::kGpsL1Ca, 3);
    gnss_sim::MeasurementErrorState first{};
    gnss_sim::MeasurementErrorState second{};
    std::string error_message;

    for (int index = 0; index < 4; ++index) {
        const gnss_sim::SignalTracker tracker =
            make_tracker(gnss_sim::SignalId::kGpsL1Ca, 100 * kSecondNs, index * 100000000LL);
        gnss_sim::MeasurementObservation first_reported{};
        gnss_sim::MeasurementObservation second_reported{};
        ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 42U, stable_context(), tracker, truth, &first,
                                                      &first_reported, &error_message))
            << error_message;
        ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 42U, stable_context(), tracker, truth, &second,
                                                      &second_reported, &error_message))
            << error_message;
        EXPECT_DOUBLE_EQ(first_reported.pseudorange_m, second_reported.pseudorange_m);
        EXPECT_DOUBLE_EQ(first_reported.doppler_hz, second_reported.doppler_hz);
        EXPECT_DOUBLE_EQ(first_reported.adr_cycles, second_reported.adr_cycles);
        EXPECT_DOUBLE_EQ(first_reported.cn0_dbhz, second_reported.cn0_dbhz);
    }
}

TEST(MeasurementErrorModel, DifferentRunSeedChangesMeasurementSequence) {
    const gnss_sim::MeasurementErrorConfig config = gnss_sim::default_sim_config().measurement_error;
    const gnss_sim::SignalTracker tracker = make_tracker(gnss_sim::SignalId::kGpsL1Ca, 100 * kSecondNs, 0);
    const gnss_sim::MeasurementObservation truth = make_observation(gnss_sim::SignalId::kGpsL1Ca, 3);
    gnss_sim::MeasurementErrorState first{};
    gnss_sim::MeasurementErrorState second{};
    gnss_sim::MeasurementObservation first_reported{};
    gnss_sim::MeasurementObservation second_reported{};
    std::string error_message;

    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 1U, stable_context(), tracker, truth, &first, &first_reported,
                                                  &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 2U, stable_context(), tracker, truth, &second,
                                                  &second_reported, &error_message))
        << error_message;
    EXPECT_NE(first_reported.pseudorange_m, second_reported.pseudorange_m);
}

TEST(MeasurementErrorModel, PerSignalStreamsDoNotCoupleUnrelatedSignals) {
    const gnss_sim::MeasurementErrorConfig config = gnss_sim::default_sim_config().measurement_error;
    const gnss_sim::MeasurementObservation truth_a = make_observation(gnss_sim::SignalId::kGpsL1Ca, 3);
    const gnss_sim::MeasurementObservation truth_b = make_observation(gnss_sim::SignalId::kGpsL2C, 3);
    gnss_sim::MeasurementErrorState interleaved_a{};
    gnss_sim::MeasurementErrorState interleaved_b{};
    gnss_sim::MeasurementErrorState isolated_a{};
    std::string error_message;

    gnss_sim::MeasurementObservation interleaved_a0{};
    gnss_sim::MeasurementObservation interleaved_b0{};
    gnss_sim::MeasurementObservation interleaved_a1{};
    gnss_sim::MeasurementObservation isolated_a0{};
    gnss_sim::MeasurementObservation isolated_a1{};

    const gnss_sim::SignalTracker a0 = make_tracker(gnss_sim::SignalId::kGpsL1Ca, 100 * kSecondNs, 0);
    const gnss_sim::SignalTracker a1 = make_tracker(gnss_sim::SignalId::kGpsL1Ca, 100 * kSecondNs, 100000000LL);
    const gnss_sim::SignalTracker b0 = make_tracker(gnss_sim::SignalId::kGpsL2C, 100 * kSecondNs, 0);

    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 99U, stable_context(), a0, truth_a, &interleaved_a,
                                                  &interleaved_a0, &error_message));
    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 99U, stable_context(), b0, truth_b, &interleaved_b,
                                                  &interleaved_b0, &error_message));
    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 99U, stable_context(), a1, truth_a, &interleaved_a,
                                                  &interleaved_a1, &error_message));

    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 99U, stable_context(), a0, truth_a, &isolated_a, &isolated_a0,
                                                  &error_message));
    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 99U, stable_context(), a1, truth_a, &isolated_a, &isolated_a1,
                                                  &error_message));

    EXPECT_DOUBLE_EQ(interleaved_a0.pseudorange_m, isolated_a0.pseudorange_m);
    EXPECT_DOUBLE_EQ(interleaved_a1.pseudorange_m, isolated_a1.pseudorange_m);
    EXPECT_DOUBLE_EQ(interleaved_a1.doppler_hz, isolated_a1.doppler_hz);
    EXPECT_NE(interleaved_b0.pseudorange_m, interleaved_a0.pseudorange_m);
}

TEST(MeasurementErrorModel, NewTrackingStartResetsStateToIndependentLockStream) {
    const gnss_sim::MeasurementErrorConfig config = gnss_sim::default_sim_config().measurement_error;
    const gnss_sim::MeasurementObservation truth = make_observation(gnss_sim::SignalId::kGpsL1Ca, 3);
    gnss_sim::MeasurementErrorState reused{};
    gnss_sim::MeasurementErrorState fresh{};
    std::string error_message;

    const gnss_sim::SignalTracker old_lock = make_tracker(gnss_sim::SignalId::kGpsL1Ca, 100 * kSecondNs, 0);
    const gnss_sim::SignalTracker new_lock = make_tracker(gnss_sim::SignalId::kGpsL1Ca, 200 * kSecondNs, 0);
    gnss_sim::MeasurementObservation old_reported{};
    gnss_sim::MeasurementObservation reused_new{};
    gnss_sim::MeasurementObservation fresh_new{};

    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 7U, stable_context(), old_lock, truth, &reused, &old_reported,
                                                  &error_message));
    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 7U, stable_context(), new_lock, truth, &reused, &reused_new,
                                                  &error_message));
    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 7U, stable_context(), new_lock, truth, &fresh, &fresh_new,
                                                  &error_message));

    EXPECT_DOUBLE_EQ(reused_new.pseudorange_m, fresh_new.pseudorange_m);
    EXPECT_DOUBLE_EQ(reused_new.doppler_hz, fresh_new.doppler_hz);
    EXPECT_DOUBLE_EQ(reused_new.adr_cycles, fresh_new.adr_cycles);
    EXPECT_NE(old_reported.pseudorange_m, reused_new.pseudorange_m);
}

TEST(MeasurementErrorModel, AdrNoiseIsConvertedFromMetersUsingWavelength) {
    gnss_sim::MeasurementErrorConfig config = gnss_sim::default_sim_config().measurement_error;
    config.psr_sigma_m = 0.0;
    config.doppler_sigma_mps = 0.0;
    config.cn0_sigma_dbhz = 0.0;
    config.adr_sigma_m = 0.001;

    gnss_sim::SignalTracker tracker = make_tracker(gnss_sim::SignalId::kGpsL1Ca, 100 * kSecondNs, 0);
    tracker.psr_valid = false;
    tracker.doppler_valid = false;
    gnss_sim::MeasurementObservation long_wave = make_observation(gnss_sim::SignalId::kGpsL1Ca, 3, 0.20);
    gnss_sim::MeasurementObservation short_wave = make_observation(gnss_sim::SignalId::kGpsL1Ca, 3, 0.10);
    long_wave.pseudorange_valid = false;
    long_wave.doppler_valid = false;
    short_wave.pseudorange_valid = false;
    short_wave.doppler_valid = false;

    gnss_sim::MeasurementErrorState long_state{};
    gnss_sim::MeasurementErrorState short_state{};
    gnss_sim::MeasurementObservation long_reported{};
    gnss_sim::MeasurementObservation short_reported{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 123U, stable_context(), tracker, long_wave, &long_state,
                                                  &long_reported, &error_message));
    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 123U, stable_context(), tracker, short_wave, &short_state,
                                                  &short_reported, &error_message));

    const double long_delta_cycles = long_reported.adr_cycles - long_wave.adr_cycles;
    const double short_delta_cycles = short_reported.adr_cycles - short_wave.adr_cycles;
    EXPECT_NEAR(short_delta_cycles, 2.0 * long_delta_cycles, 1e-9);
}

TEST(MeasurementErrorModel, NonTrackingInputIsPassedThroughAndStateIsReset) {
    const gnss_sim::MeasurementErrorConfig config = gnss_sim::default_sim_config().measurement_error;
    gnss_sim::SignalTracker tracker = make_tracker(gnss_sim::SignalId::kGpsL1Ca, 100 * kSecondNs, 0);
    const gnss_sim::MeasurementObservation truth = make_observation(gnss_sim::SignalId::kGpsL1Ca, 3);
    gnss_sim::MeasurementErrorState state{};
    gnss_sim::MeasurementObservation reported{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 5U, stable_context(), tracker, truth, &state, &reported,
                                                  &error_message));
    ASSERT_TRUE(state.initialized);

    tracker.phase = gnss_sim::SignalTrackingPhase::kSignalOff;
    ASSERT_TRUE(gnss_sim::apply_measurement_error(config, 5U, stable_context(), tracker, truth, &state, &reported,
                                                  &error_message));
    EXPECT_FALSE(state.initialized);
    EXPECT_DOUBLE_EQ(reported.pseudorange_m, truth.pseudorange_m);
    EXPECT_DOUBLE_EQ(reported.doppler_hz, truth.doppler_hz);
    EXPECT_DOUBLE_EQ(reported.adr_cycles, truth.adr_cycles);
    EXPECT_DOUBLE_EQ(reported.cn0_dbhz, truth.cn0_dbhz);
}

} // namespace
