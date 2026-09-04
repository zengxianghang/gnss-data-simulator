#include "model/carrier_tracking.h"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <vector>

namespace gnss_sim {
namespace {

constexpr double kL1WavelengthM = 0.190293672798;

CarrierTrackingResult step_tracker(const CarrierTrackingConfig& config, CarrierTrackingState* state, double cn0_dbhz,
                                   double dt_sec = 0.1, double standard_normal_sample = 0.0,
                                   bool signal_available = true) {
    CarrierTrackingInput input{};
    input.signal_available = signal_available;
    input.effective_cn0_dbhz = cn0_dbhz;
    input.wavelength_m = kL1WavelengthM;
    input.dt_sec = dt_sec;
    input.standard_normal_sample = standard_normal_sample;

    CarrierTrackingResult result{};
    std::string error;
    EXPECT_TRUE(update_carrier_tracking(config, input, state, &result, &error)) << error;
    return result;
}

TEST(CarrierTracking, DefaultConfigMatchesFrozenV1) {
    const CarrierTrackingConfig config = default_carrier_tracking_config();

    EXPECT_DOUBLE_EQ(config.coherent_integration_sec, 0.020);
    EXPECT_DOUBLE_EQ(config.pll_noise_bandwidth_hz, 5.0);
    EXPECT_DOUBLE_EQ(config.fll_noise_bandwidth_hz, 4.0);
    EXPECT_DOUBLE_EQ(config.fll_pull_in_bandwidth_hz, 8.0);
    EXPECT_DOUBLE_EQ(config.fll_pull_in_duration_sec, 0.5);
    EXPECT_DOUBLE_EQ(config.pll_enter_cn0_dbhz, 30.0);
    EXPECT_DOUBLE_EQ(config.pll_exit_cn0_dbhz, 27.0);
    EXPECT_DOUBLE_EQ(config.pll_enter_persistence_sec, 1.0);
    EXPECT_DOUBLE_EQ(config.pll_exit_persistence_sec, 0.3);
    EXPECT_DOUBLE_EQ(config.fll_enter_cn0_dbhz, 22.0);
    EXPECT_DOUBLE_EQ(config.fll_exit_cn0_dbhz, 18.0);
    EXPECT_DOUBLE_EQ(config.fll_enter_persistence_sec, 0.2);
    EXPECT_DOUBLE_EQ(config.fll_exit_persistence_sec, 0.5);
    EXPECT_DOUBLE_EQ(config.doppler_valid_delay_sec, 0.2);
    EXPECT_DOUBLE_EQ(config.adr_valid_after_pll_sec, 1.0);

    std::string error;
    EXPECT_TRUE(validate_carrier_tracking_config(config, &error)) << error;
}

TEST(CarrierTracking, ConfigValidationRejectsInvalidValuesAndOrdering) {
    std::string error;

    CarrierTrackingConfig config = default_carrier_tracking_config();
    config.coherent_integration_sec = 0.0;
    EXPECT_FALSE(validate_carrier_tracking_config(config, &error));

    config = default_carrier_tracking_config();
    config.fll_pull_in_bandwidth_hz = 3.0;
    EXPECT_FALSE(validate_carrier_tracking_config(config, &error));

    config = default_carrier_tracking_config();
    config.pll_exit_cn0_dbhz = config.pll_enter_cn0_dbhz;
    EXPECT_FALSE(validate_carrier_tracking_config(config, &error));

    config = default_carrier_tracking_config();
    config.fll_enter_cn0_dbhz = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(validate_carrier_tracking_config(config, &error));

    config = default_carrier_tracking_config();
    config.adr_valid_after_pll_sec = -1.0;
    EXPECT_FALSE(validate_carrier_tracking_config(config, &error));
}

TEST(CarrierTracking, HysteresisAndPersistenceDriveUnlockedFllAndPll) {
    const CarrierTrackingConfig config = default_carrier_tracking_config();
    CarrierTrackingState state{};
    reset_carrier_tracking_state(&state);

    for (int index = 0; index < 10; ++index) {
        const CarrierTrackingResult result = step_tracker(config, &state, 21.9);
        EXPECT_EQ(result.mode, CarrierTrackingMode::kCarrierUnlocked);
    }

    EXPECT_EQ(step_tracker(config, &state, 22.0).mode, CarrierTrackingMode::kCarrierUnlocked);
    CarrierTrackingResult result = step_tracker(config, &state, 22.0);
    EXPECT_EQ(result.mode, CarrierTrackingMode::kFllTrack);
    EXPECT_TRUE(result.new_carrier_segment);
    EXPECT_EQ(result.carrier_segment_id, 1U);

    for (int index = 0; index < 10; ++index) {
        result = step_tracker(config, &state, 21.0);
        EXPECT_EQ(result.mode, CarrierTrackingMode::kFllTrack);
    }

    for (int index = 0; index < 9; ++index) {
        result = step_tracker(config, &state, 30.0);
        EXPECT_EQ(result.mode, CarrierTrackingMode::kFllTrack);
    }
    result = step_tracker(config, &state, 30.0);
    EXPECT_EQ(result.mode, CarrierTrackingMode::kPllTrack);

    for (int index = 0; index < 5; ++index) {
        result = step_tracker(config, &state, 27.0);
        EXPECT_EQ(result.mode, CarrierTrackingMode::kPllTrack);
    }
    EXPECT_EQ(step_tracker(config, &state, 26.9).mode, CarrierTrackingMode::kPllTrack);
    EXPECT_EQ(step_tracker(config, &state, 26.9).mode, CarrierTrackingMode::kPllTrack);
    result = step_tracker(config, &state, 26.9);
    EXPECT_EQ(result.mode, CarrierTrackingMode::kFllTrack);
    EXPECT_EQ(result.fll_phase, CarrierTrackingFllPhase::kSteady);

    for (int index = 0; index < 6; ++index) {
        result = step_tracker(config, &state, 18.0);
        EXPECT_EQ(result.mode, CarrierTrackingMode::kFllTrack);
    }
    for (int index = 0; index < 4; ++index) {
        result = step_tracker(config, &state, 17.9);
        EXPECT_EQ(result.mode, CarrierTrackingMode::kFllTrack);
    }
    result = step_tracker(config, &state, 17.9);
    EXPECT_EQ(result.mode, CarrierTrackingMode::kCarrierUnlocked);
    EXPECT_FALSE(result.doppler_valid);
    EXPECT_FALSE(result.adr_valid);
}

TEST(CarrierTracking, FreshAcquisitionUsesPullInThenSteadyFll) {
    const CarrierTrackingConfig config = default_carrier_tracking_config();
    CarrierTrackingState state{};
    reset_carrier_tracking_state(&state);

    step_tracker(config, &state, 25.0);
    CarrierTrackingResult result = step_tracker(config, &state, 25.0);
    ASSERT_EQ(result.mode, CarrierTrackingMode::kFllTrack);
    EXPECT_EQ(result.fll_phase, CarrierTrackingFllPhase::kPullIn);
    EXPECT_DOUBLE_EQ(result.jitter.active_bandwidth_hz, 8.0);

    for (int index = 0; index < 4; ++index) {
        result = step_tracker(config, &state, 25.0);
        EXPECT_EQ(result.fll_phase, CarrierTrackingFllPhase::kPullIn);
        EXPECT_DOUBLE_EQ(result.jitter.active_bandwidth_hz, 8.0);
    }
    result = step_tracker(config, &state, 25.0);
    EXPECT_EQ(result.fll_phase, CarrierTrackingFllPhase::kSteady);
    EXPECT_DOUBLE_EQ(result.jitter.active_bandwidth_hz, 4.0);
}

TEST(CarrierTracking, DopplerAndAdrValidityHaveIndependentDelays) {
    const CarrierTrackingConfig config = default_carrier_tracking_config();
    CarrierTrackingState state{};
    reset_carrier_tracking_state(&state);

    step_tracker(config, &state, 25.0);
    CarrierTrackingResult result = step_tracker(config, &state, 25.0);
    ASSERT_EQ(result.mode, CarrierTrackingMode::kFllTrack);
    EXPECT_FALSE(result.doppler_valid);
    EXPECT_FALSE(result.adr_valid);

    result = step_tracker(config, &state, 25.0);
    EXPECT_FALSE(result.doppler_valid);
    result = step_tracker(config, &state, 25.0);
    EXPECT_TRUE(result.doppler_valid);
    EXPECT_FALSE(result.adr_valid);

    for (int index = 0; index < 9; ++index) {
        result = step_tracker(config, &state, 30.0);
        ASSERT_EQ(result.mode, CarrierTrackingMode::kFllTrack);
        EXPECT_FALSE(result.adr_valid);
    }
    result = step_tracker(config, &state, 30.0);
    ASSERT_EQ(result.mode, CarrierTrackingMode::kPllTrack);
    EXPECT_TRUE(result.doppler_valid);
    EXPECT_FALSE(result.adr_valid);

    for (int index = 0; index < 9; ++index) {
        result = step_tracker(config, &state, 30.0);
        EXPECT_FALSE(result.adr_valid);
    }
    result = step_tracker(config, &state, 30.0);
    EXPECT_TRUE(result.adr_valid);
}

TEST(CarrierTracking, PllLossImmediatelyInvalidatesAdrButKeepsDopplerInFll) {
    const CarrierTrackingConfig config = default_carrier_tracking_config();
    CarrierTrackingState state{};
    reset_carrier_tracking_state(&state);

    step_tracker(config, &state, 35.0);
    step_tracker(config, &state, 35.0);
    for (int index = 0; index < 10; ++index) {
        step_tracker(config, &state, 35.0);
    }
    CarrierTrackingResult result{};
    for (int index = 0; index < 10; ++index) {
        result = step_tracker(config, &state, 35.0);
    }
    ASSERT_EQ(result.mode, CarrierTrackingMode::kPllTrack);
    ASSERT_TRUE(result.adr_valid);

    step_tracker(config, &state, 26.0);
    step_tracker(config, &state, 26.0);
    result = step_tracker(config, &state, 26.0);
    EXPECT_EQ(result.mode, CarrierTrackingMode::kFllTrack);
    EXPECT_TRUE(result.doppler_valid);
    EXPECT_FALSE(result.adr_valid);
}

TEST(CarrierTracking, JitterFallsWithCn0AndScalesWithBandwidthIntegrationAndWavelength) {
    const CarrierTrackingConfig config = default_carrier_tracking_config();
    CarrierTrackingJitter fll_low{};
    CarrierTrackingJitter fll_mid{};
    CarrierTrackingJitter fll_high{};
    CarrierTrackingJitter pll_low{};
    CarrierTrackingJitter pll_high{};
    std::string error;

    ASSERT_TRUE(compute_carrier_tracking_jitter(config, CarrierTrackingMode::kFllTrack, false, 25.0, kL1WavelengthM,
                                                0.1, &fll_low, &error))
        << error;
    ASSERT_TRUE(compute_carrier_tracking_jitter(config, CarrierTrackingMode::kFllTrack, false, 35.0, kL1WavelengthM,
                                                0.1, &fll_mid, &error))
        << error;
    ASSERT_TRUE(compute_carrier_tracking_jitter(config, CarrierTrackingMode::kFllTrack, false, 45.0, kL1WavelengthM,
                                                0.1, &fll_high, &error))
        << error;
    EXPECT_GT(fll_low.sigma_mps, fll_mid.sigma_mps);
    EXPECT_GT(fll_mid.sigma_mps, fll_high.sigma_mps);

    ASSERT_TRUE(compute_carrier_tracking_jitter(config, CarrierTrackingMode::kPllTrack, false, 25.0, kL1WavelengthM,
                                                0.1, &pll_low, &error))
        << error;
    ASSERT_TRUE(compute_carrier_tracking_jitter(config, CarrierTrackingMode::kPllTrack, false, 45.0, kL1WavelengthM,
                                                0.1, &pll_high, &error))
        << error;
    EXPECT_GT(pll_low.sigma_mps, pll_high.sigma_mps);

    CarrierTrackingConfig wide = config;
    wide.fll_noise_bandwidth_hz = 8.0;
    wide.fll_pull_in_bandwidth_hz = 8.0;
    CarrierTrackingJitter wider{};
    ASSERT_TRUE(compute_carrier_tracking_jitter(wide, CarrierTrackingMode::kFllTrack, false, 35.0, kL1WavelengthM, 0.1,
                                                &wider, &error))
        << error;
    EXPECT_GT(wider.sigma_hz, fll_mid.sigma_hz);

    CarrierTrackingConfig short_integration = config;
    short_integration.coherent_integration_sec = 0.010;
    CarrierTrackingJitter shorter{};
    ASSERT_TRUE(compute_carrier_tracking_jitter(short_integration, CarrierTrackingMode::kFllTrack, false, 35.0,
                                                kL1WavelengthM, 0.1, &shorter, &error))
        << error;
    EXPECT_GT(shorter.sigma_hz, fll_mid.sigma_hz);

    CarrierTrackingJitter short_wave{};
    CarrierTrackingJitter long_wave{};
    ASSERT_TRUE(compute_carrier_tracking_jitter(config, CarrierTrackingMode::kFllTrack, false, 35.0, 0.10, 0.1,
                                                &short_wave, &error))
        << error;
    ASSERT_TRUE(compute_carrier_tracking_jitter(config, CarrierTrackingMode::kFllTrack, false, 35.0, 0.20, 0.1,
                                                &long_wave, &error))
        << error;
    EXPECT_DOUBLE_EQ(short_wave.sigma_hz, long_wave.sigma_hz);
    EXPECT_NEAR(long_wave.sigma_mps, 2.0 * short_wave.sigma_mps, 1e-15);
}

TEST(CarrierTracking, IdenticalInputsAndGaussianSequenceProduceIdenticalState) {
    const CarrierTrackingConfig config = default_carrier_tracking_config();
    CarrierTrackingState first{};
    CarrierTrackingState second{};
    reset_carrier_tracking_state(&first);
    reset_carrier_tracking_state(&second);

    const std::vector<double> cn0 = {25.0, 25.0, 25.0, 31.0, 31.0, 31.0, 31.0, 31.0,
                                     31.0, 31.0, 31.0, 31.0, 31.0, 26.0, 26.0, 26.0};
    const std::vector<double> gaussian = {0.1, -0.3, 1.2,  0.4, -0.7, 0.0,  0.9, -1.1,
                                          0.2, 0.5,  -0.4, 0.8, 0.3,  -0.2, 1.0, -0.6};

    for (std::size_t index = 0; index < cn0.size(); ++index) {
        const CarrierTrackingResult first_result = step_tracker(config, &first, cn0[index], 0.1, gaussian[index]);
        const CarrierTrackingResult second_result = step_tracker(config, &second, cn0[index], 0.1, gaussian[index]);
        EXPECT_EQ(first_result.mode, second_result.mode);
        EXPECT_EQ(first_result.fll_phase, second_result.fll_phase);
        EXPECT_DOUBLE_EQ(first_result.tracking_error_hz, second_result.tracking_error_hz);
        EXPECT_DOUBLE_EQ(first_result.tracking_error_mps, second_result.tracking_error_mps);
        EXPECT_EQ(first_result.doppler_valid, second_result.doppler_valid);
        EXPECT_EQ(first_result.adr_valid, second_result.adr_valid);
        EXPECT_EQ(first_result.carrier_segment_id, second_result.carrier_segment_id);
    }
}

TEST(CarrierTracking, SupportedExtremeCn0ValuesRemainFinite) {
    const CarrierTrackingConfig config = default_carrier_tracking_config();
    CarrierTrackingJitter low{};
    CarrierTrackingJitter high{};
    std::string error;

    ASSERT_TRUE(compute_carrier_tracking_jitter(config, CarrierTrackingMode::kFllTrack, false, -100.0, kL1WavelengthM,
                                                0.1, &low, &error))
        << error;
    ASSERT_TRUE(compute_carrier_tracking_jitter(config, CarrierTrackingMode::kPllTrack, false, 100.0, kL1WavelengthM,
                                                0.1, &high, &error))
        << error;

    EXPECT_TRUE(std::isfinite(low.sigma_hz));
    EXPECT_TRUE(std::isfinite(low.sigma_mps));
    EXPECT_TRUE(std::isfinite(low.correlation_alpha));
    EXPECT_TRUE(std::isfinite(high.sigma_hz));
    EXPECT_TRUE(std::isfinite(high.sigma_mps));
    EXPECT_TRUE(std::isfinite(high.correlation_alpha));
}

TEST(CarrierTracking, SignalUnavailableImmediatelyUnlocksAndClearsTrackingError) {
    const CarrierTrackingConfig config = default_carrier_tracking_config();
    CarrierTrackingState state{};
    reset_carrier_tracking_state(&state);

    step_tracker(config, &state, 30.0, 0.1, 1.0);
    CarrierTrackingResult result = step_tracker(config, &state, 30.0, 0.1, 1.0);
    ASSERT_EQ(result.mode, CarrierTrackingMode::kFllTrack);

    result = step_tracker(config, &state, 30.0, 0.1, 1.0, false);
    EXPECT_EQ(result.mode, CarrierTrackingMode::kCarrierUnlocked);
    EXPECT_DOUBLE_EQ(result.tracking_error_hz, 0.0);
    EXPECT_FALSE(result.doppler_valid);
    EXPECT_FALSE(result.adr_valid);
}

} // namespace
} // namespace gnss_sim
