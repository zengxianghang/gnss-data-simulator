#include "model/signal_tracking.h"

#include "gnss_sim/sim_time.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace {

constexpr std::int64_t kMs = 1000000LL;
constexpr std::int64_t kSecond = gnss_sim::NANOSECONDS_PER_SECOND;

bool add_seconds(const gnss_sim::SimTime& time, double seconds, gnss_sim::SimTime* result) {
    return gnss_sim::add_time_ns(time, static_cast<std::int64_t>(seconds * static_cast<double>(kSecond)), result);
}

gnss_sim::DelayDistribution fixed_delay(std::int64_t delay_ns) {
    return {delay_ns, delay_ns, delay_ns, delay_ns};
}

gnss_sim::SignalTrackingModelConfig fixed_tracking_config() {
    gnss_sim::SignalTrackingModelConfig config = gnss_sim::default_signal_tracking_model_config();
    config.hot_common_startup = fixed_delay(0);
    config.warm_common_startup = fixed_delay(0);
    config.warm_search_uncertainty = fixed_delay(0);
    for (int index = 0; index < 4; ++index) {
        config.hot_signal_acquisition[index] = fixed_delay(500 * kMs);
        config.reacquisition[index] = fixed_delay(300 * kMs);
    }
    config.psr_valid_delay_ns = 100 * kMs;
    config.doppler_valid_delay_ns = 200 * kMs;
    config.adr_valid_delay_ns = 500 * kMs;
    return config;
}

TEST(SignalTrackingConfig, DefaultsAreValidAndInvalidQuantilesAreRejected) {
    gnss_sim::SignalTrackingModelConfig config = gnss_sim::default_signal_tracking_model_config();
    std::string error_message;
    EXPECT_TRUE(gnss_sim::validate_signal_tracking_model_config(config, &error_message));

    config.hot_common_startup.p50_ns = config.hot_common_startup.minimum_ns - 1;
    EXPECT_FALSE(gnss_sim::validate_signal_tracking_model_config(config, &error_message));
    EXPECT_FALSE(error_message.empty());
}

TEST(SignalTrackingStartup, SeededStartupSamplingIsExactlyRepeatableAndWarmIsSlower) {
    const gnss_sim::SignalTrackingModelConfig config = gnss_sim::default_signal_tracking_model_config();
    gnss_sim::DeterministicRng first_rng{};
    gnss_sim::DeterministicRng second_rng{};
    gnss_sim::seed_rng(&first_rng, 12345U);
    gnss_sim::seed_rng(&second_rng, 12345U);

    gnss_sim::ReceiverStartupTiming first{};
    gnss_sim::ReceiverStartupTiming second{};
    ASSERT_TRUE(gnss_sim::sample_receiver_startup_timing(gnss_sim::StartupMode::HOT, config, &first_rng, &first));
    ASSERT_TRUE(gnss_sim::sample_receiver_startup_timing(gnss_sim::StartupMode::HOT, config, &second_rng, &second));
    EXPECT_EQ(first.common_startup_delay_ns, second.common_startup_delay_ns);
    EXPECT_EQ(first.total_search_ready_delay_ns, second.total_search_ready_delay_ns);

    gnss_sim::DeterministicRng hot_rng{};
    gnss_sim::DeterministicRng warm_rng{};
    gnss_sim::seed_rng(&hot_rng, 77U);
    gnss_sim::seed_rng(&warm_rng, 77U);
    gnss_sim::ReceiverStartupTiming hot{};
    gnss_sim::ReceiverStartupTiming warm{};
    ASSERT_TRUE(gnss_sim::sample_receiver_startup_timing(gnss_sim::StartupMode::HOT, config, &hot_rng, &hot));
    ASSERT_TRUE(gnss_sim::sample_receiver_startup_timing(gnss_sim::StartupMode::WARM, config, &warm_rng, &warm));
    EXPECT_GT(warm.total_search_ready_delay_ns, hot.total_search_ready_delay_ns + kSecond);
}

TEST(SignalTrackingAcquisition, LowerCn0ProducesLongerDelayAtSameRandomQuantile) {
    const gnss_sim::SignalTrackingModelConfig config = gnss_sim::default_signal_tracking_model_config();
    const gnss_sim::Cn0Model cn0_model = gnss_sim::make_builtin_cn0_model(9U);
    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 100000.0, &start));

    gnss_sim::SignalTracker high{};
    gnss_sim::SignalTracker low{};
    gnss_sim::reset_signal_tracker(&high, gnss_sim::SignalId::kGpsL1Ca, start);
    gnss_sim::reset_signal_tracker(&low, gnss_sim::SignalId::kGpsL1Ca, start);
    gnss_sim::DeterministicRng high_rng{};
    gnss_sim::DeterministicRng low_rng{};
    gnss_sim::seed_rng(&high_rng, 4321U);
    gnss_sim::seed_rng(&low_rng, 4321U);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::schedule_signal_acquisition(high, gnss_sim::AcquisitionContext::kHot, start, start, 80.0,
                                                      cn0_model, config, &high_rng, &error_message));
    ASSERT_TRUE(gnss_sim::schedule_signal_acquisition(low, gnss_sim::AcquisitionContext::kHot, start, start, 3.0,
                                                      cn0_model, config, &low_rng, &error_message));
    EXPECT_GT(gnss_sim::compare_sim_time(low.acquisition_complete_time, high.acquisition_complete_time), 0);
}

TEST(SignalTrackingState, OneStateMachineControlsValidityLockAndSignalOffReset) {
    const gnss_sim::SignalTrackingModelConfig config = fixed_tracking_config();
    const gnss_sim::Cn0Model cn0_model = gnss_sim::make_builtin_cn0_model(11U);
    gnss_sim::SimTime signal_on{};
    gnss_sim::SimTime search_ready{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 200000.0, &signal_on));
    ASSERT_TRUE(add_seconds(signal_on, 1.0, &search_ready));

    gnss_sim::SignalTracker tracker{};
    gnss_sim::reset_signal_tracker(&tracker, gnss_sim::SignalId::kGalileoE1, signal_on);
    gnss_sim::DeterministicRng rng{};
    gnss_sim::seed_rng(&rng, 1U);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::schedule_signal_acquisition(&tracker, gnss_sim::AcquisitionContext::kHot, signal_on,
                                                      search_ready, 45.0, cn0_model, config, &rng, &error_message));

    gnss_sim::SimTime current{};
    ASSERT_TRUE(add_seconds(signal_on, 0.5, &current));
    ASSERT_TRUE(gnss_sim::update_signal_tracker(&tracker, current, true, 45.0, cn0_model, &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kSearching);
    EXPECT_FALSE(tracker.observation_available);

    ASSERT_TRUE(add_seconds(signal_on, 1.2, &current));
    ASSERT_TRUE(gnss_sim::update_signal_tracker(&tracker, current, true, 45.0, cn0_model, &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kAcquiring);
    EXPECT_EQ(tracker.lock_time_ns, 0);

    ASSERT_TRUE(add_seconds(signal_on, 1.5, &current));
    ASSERT_TRUE(gnss_sim::update_signal_tracker(&tracker, current, true, 45.0, cn0_model, &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kTracking);
    EXPECT_EQ(tracker.lock_time_ns, 0);
    EXPECT_FALSE(tracker.psr_valid);

    ASSERT_TRUE(add_seconds(signal_on, 1.6, &current));
    ASSERT_TRUE(gnss_sim::update_signal_tracker(&tracker, current, true, 45.0, cn0_model, &error_message));
    EXPECT_TRUE(tracker.psr_valid);
    EXPECT_TRUE(tracker.observation_available);
    EXPECT_FALSE(tracker.doppler_valid);
    EXPECT_FALSE(tracker.adr_valid);
    EXPECT_EQ(tracker.lock_time_ns, 100 * kMs);

    ASSERT_TRUE(add_seconds(signal_on, 2.0, &current));
    ASSERT_TRUE(gnss_sim::update_signal_tracker(&tracker, current, true, 45.0, cn0_model, &error_message));
    EXPECT_TRUE(tracker.psr_valid);
    EXPECT_TRUE(tracker.doppler_valid);
    EXPECT_TRUE(tracker.adr_valid);
    EXPECT_EQ(tracker.lock_time_ns, 500 * kMs);

    ASSERT_TRUE(add_seconds(signal_on, 3.0, &current));
    ASSERT_TRUE(gnss_sim::update_signal_tracker(&tracker, current, false, 45.0, cn0_model, &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kSignalOff);
    EXPECT_FALSE(tracker.scheduled);
    EXPECT_FALSE(tracker.observation_available);
    EXPECT_FALSE(tracker.psr_valid);
    EXPECT_FALSE(tracker.doppler_valid);
    EXPECT_FALSE(tracker.adr_valid);
    EXPECT_EQ(tracker.lock_time_ns, 0);
    EXPECT_DOUBLE_EQ(tracker.cn0_dbhz, 0.0);
}

TEST(SignalTrackingState, ReacquisitionRestartsLockAndUsesFasterModel) {
    const gnss_sim::SignalTrackingModelConfig config = fixed_tracking_config();
    const gnss_sim::Cn0Model cn0_model = gnss_sim::make_builtin_cn0_model(12U);
    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 300000.0, &start));
    gnss_sim::SignalTracker tracker{};
    gnss_sim::reset_signal_tracker(&tracker, gnss_sim::SignalId::kGpsL1C, start);
    gnss_sim::DeterministicRng rng{};
    gnss_sim::seed_rng(&rng, 2U);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::schedule_signal_acquisition(&tracker, gnss_sim::AcquisitionContext::kReacquisition, start,
                                                      start, 60.0, cn0_model, config, &rng, &error_message));

    gnss_sim::SimTime current{};
    ASSERT_TRUE(add_seconds(start, 0.4, &current));
    ASSERT_TRUE(gnss_sim::update_signal_tracker(&tracker, current, true, 60.0, cn0_model, &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kTracking);
    EXPECT_EQ(tracker.lock_time_ns, 100 * kMs);
    EXPECT_TRUE(tracker.psr_valid);
    EXPECT_FALSE(tracker.adr_valid);
}

std::int64_t fourth_psr_ready_delay_ns(std::uint64_t seed, gnss_sim::StartupMode startup_mode,
                                       gnss_sim::AcquisitionContext context) {
    const gnss_sim::SignalTrackingModelConfig config = gnss_sim::default_signal_tracking_model_config();
    const gnss_sim::Cn0Model cn0_model = gnss_sim::make_builtin_cn0_model(seed);
    const gnss_sim::SignalId signals[] = {
        gnss_sim::SignalId::kGpsL1Ca,    gnss_sim::SignalId::kGpsL1C,     gnss_sim::SignalId::kGpsL5Q,
        gnss_sim::SignalId::kQzssL1C,    gnss_sim::SignalId::kGalileoE1,  gnss_sim::SignalId::kGalileoE5A,
        gnss_sim::SignalId::kBeidouB1C,  gnss_sim::SignalId::kGpsL2C,     gnss_sim::SignalId::kGalileoE5B,
        gnss_sim::SignalId::kBeidouB1I,  gnss_sim::SignalId::kGlonassG1,  gnss_sim::SignalId::kBeidouB3I,
    };
    const double elevations[] = {75.0, 65.0, 55.0, 50.0, 45.0, 40.0, 35.0, 30.0, 25.0, 20.0, 15.0, 10.0};

    gnss_sim::SimTime start{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180000.0, &start));
    gnss_sim::DeterministicRng rng{};
    gnss_sim::seed_rng(&rng, seed);

    gnss_sim::SimTime search_ready = start;
    if (context != gnss_sim::AcquisitionContext::kReacquisition) {
        gnss_sim::ReceiverStartupTiming startup{};
        EXPECT_TRUE(gnss_sim::sample_receiver_startup_timing(startup_mode, config, &rng, &startup));
        EXPECT_TRUE(gnss_sim::add_time_ns(start, startup.total_search_ready_delay_ns, &search_ready));
    }

    std::int64_t delays[12]{};
    for (int index = 0; index < 12; ++index) {
        gnss_sim::SignalTracker tracker{};
        gnss_sim::reset_signal_tracker(&tracker, signals[index], start);
        std::string error_message;
        EXPECT_TRUE(gnss_sim::schedule_signal_acquisition(&tracker, context, start, search_ready, elevations[index],
                                                          cn0_model, config, &rng, &error_message));
        EXPECT_TRUE(gnss_sim::difference_time_ns(tracker.psr_valid_time, start, &delays[index]));
    }
    std::sort(delays, delays + 12);
    return delays[3];
}

int ceil_seconds(std::int64_t delay_ns) {
    return static_cast<int>((delay_ns + kSecond - 1) / kSecond);
}

int percentile(std::vector<int> values, int percentile_value) {
    std::sort(values.begin(), values.end());
    const std::size_t index = static_cast<std::size_t>(percentile_value) * (values.size() - 1U) / 100U;
    return values[index];
}

TEST(SignalTrackingCalibration, NominalOpenSkyEnsemblesStayNearFrozenHotWarmAndReaTargets) {
    std::vector<int> hot;
    std::vector<int> warm;
    std::vector<int> rea;
    for (std::uint64_t seed = 1U; seed <= 1024U; ++seed) {
        hot.push_back(ceil_seconds(fourth_psr_ready_delay_ns(seed, gnss_sim::StartupMode::HOT,
                                                            gnss_sim::AcquisitionContext::kHot)));
        warm.push_back(ceil_seconds(fourth_psr_ready_delay_ns(seed, gnss_sim::StartupMode::WARM,
                                                             gnss_sim::AcquisitionContext::kWarm)));
        rea.push_back(ceil_seconds(fourth_psr_ready_delay_ns(seed, gnss_sim::StartupMode::HOT,
                                                            gnss_sim::AcquisitionContext::kReacquisition)));
    }

    const int hot_p50 = percentile(hot, 50);
    const int hot_p95 = percentile(hot, 95);
    EXPECT_GE(hot_p50, 1);
    EXPECT_LE(hot_p50, 2);
    EXPECT_GE(hot_p95, 2);
    EXPECT_LE(hot_p95, 4);

    const int warm_p50 = percentile(warm, 50);
    const int warm_p95 = percentile(warm, 95);
    EXPECT_GE(warm_p50, 6);
    EXPECT_LE(warm_p50, 9);
    EXPECT_GE(warm_p95, 14);
    EXPECT_LE(warm_p95, 21);

    const int rea_p50 = percentile(rea, 50);
    const int rea_p95 = percentile(rea, 95);
    EXPECT_GE(rea_p50, 1);
    EXPECT_LE(rea_p50, 2);
    EXPECT_LE(rea_p95, 2);
}

} // namespace
