#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_time.h"
#include "model/signal_tracking.h"

#include <cstdint>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr std::int64_t kMs = 1000000LL;

bool add_ms(const gnss_sim::SimTime& time, std::int64_t milliseconds, gnss_sim::SimTime* result) {
    return gnss_sim::add_time_ns(time, milliseconds * kMs, result);
}

gnss_sim::DelayDistribution fixed_delay(std::int64_t delay_ns) {
    return {delay_ns, delay_ns, delay_ns, delay_ns};
}

gnss_sim::SignalTrackingModelConfig urban_test_config() {
    gnss_sim::SignalTrackingModelConfig config = gnss_sim::default_signal_tracking_model_config();
    config.hot_common_startup = fixed_delay(0);
    config.warm_common_startup = fixed_delay(0);
    config.warm_search_uncertainty = fixed_delay(0);
    for (int index = 0; index < 4; ++index) {
        config.hot_signal_acquisition[index] = fixed_delay(0);
        config.reacquisition[index] = fixed_delay(0);
    }
    config.psr_valid_delay_ns = 100 * kMs;
    config.doppler_valid_delay_ns = 150 * kMs;
    config.adr_valid_delay_ns = 500 * kMs;
    return config;
}

const gnss_sim::SignalDefinition& signal(gnss_sim::SignalId signal_id) {
    const gnss_sim::SignalDefinition* definition = gnss_sim::find_signal_definition(signal_id);
    EXPECT_NE(definition, nullptr);
    return *definition;
}

gnss_sim::CodeTrackingDllRoot root_for(const gnss_sim::SignalDefinition& definition, double code_phase_chips,
                                       double prompt_power = 1.0, bool stable = true) {
    gnss_sim::CodeTrackingDllRoot root{};
    root.code_phase_chips = code_phase_chips;
    root.code_phase_sec = code_phase_chips / definition.code_correlation.chip_rate_hz;
    root.discriminator = 0.0;
    root.discriminator_slope_per_chip = stable ? 1.0 : -1.0;
    root.prompt_power = prompt_power;
    root.stable = stable;
    return root;
}

gnss_sim::UrbanSignalTrackingInput input(double open_cn0_dbhz, double effective_cn0_dbhz, bool direct_line_of_sight,
                                         const gnss_sim::CodeTrackingDllRoot* roots, int root_count) {
    gnss_sim::UrbanSignalTrackingInput value{};
    value.signal_available = true;
    value.direct_line_of_sight = direct_line_of_sight;
    value.open_cn0_dbhz = open_cn0_dbhz;
    value.effective_cn0_dbhz = effective_cn0_dbhz;
    value.dll_roots = roots;
    value.dll_root_count = root_count;
    return value;
}

void schedule(gnss_sim::SignalTracker* tracker, gnss_sim::SignalId signal_id, const gnss_sim::SimTime& start,
              gnss_sim::AcquisitionContext context, const gnss_sim::SignalTrackingModelConfig& config,
              std::uint64_t seed = 1U) {
    gnss_sim::reset_signal_tracker(tracker, signal_id, start);
    gnss_sim::DeterministicRng rng{};
    gnss_sim::seed_rng(&rng, seed);
    const gnss_sim::Cn0Model cn0_model = gnss_sim::make_builtin_cn0_model(seed);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::schedule_signal_acquisition(tracker, context, start, start, 60.0, cn0_model, config, &rng,
                                                      &error_message))
        << error_message;
}

void schedule_existing(gnss_sim::SignalTracker* tracker, const gnss_sim::SimTime& start,
                       gnss_sim::AcquisitionContext context, const gnss_sim::SignalTrackingModelConfig& config,
                       std::uint64_t seed = 1U) {
    gnss_sim::DeterministicRng rng{};
    gnss_sim::seed_rng(&rng, seed);
    const gnss_sim::Cn0Model cn0_model = gnss_sim::make_builtin_cn0_model(seed);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::schedule_signal_acquisition(tracker, context, start, start, 60.0, cn0_model, config, &rng,
                                                      &error_message))
        << error_message;
}

void acquire_immediately(gnss_sim::SignalTracker* tracker, gnss_sim::SignalId signal_id, const gnss_sim::SimTime& start,
                         gnss_sim::SignalTrackingModelConfig* config, bool direct_line_of_sight = true) {
    ASSERT_NE(config, nullptr);
    config->acquisition_cn0_persistence_ns = 0;
    schedule(tracker, signal_id, start, gnss_sim::AcquisitionContext::kHot, *config);
    const gnss_sim::SignalDefinition& definition = signal(signal_id);
    const gnss_sim::CodeTrackingDllRoot root = root_for(definition, 0.0);
    const gnss_sim::UrbanSignalTrackingInput update = input(45.0, 45.0, direct_line_of_sight, &root, 1);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(tracker, start, update, *config, &error_message))
        << error_message;
    ASSERT_EQ(tracker->phase, gnss_sim::SignalTrackingPhase::kTracking);
}

TEST(UrbanSignalTrackingConfig, FrozenDefaultsAndHysteresisAreExplicit) {
    gnss_sim::SignalTrackingModelConfig config = gnss_sim::default_signal_tracking_model_config();
    std::string error_message;
    ASSERT_TRUE(gnss_sim::validate_signal_tracking_model_config(config, &error_message));
    EXPECT_DOUBLE_EQ(config.minimum_tracking_cn0_dbhz, 10.0);
    EXPECT_DOUBLE_EQ(config.acquisition_cn0_threshold_dbhz, 18.0);
    EXPECT_EQ(config.acquisition_cn0_persistence_ns, 200 * kMs);
    EXPECT_EQ(config.tracking_loss_cn0_persistence_ns, 500 * kMs);
    EXPECT_DOUBLE_EQ(config.dll_root_jump_threshold_chips, 0.25);
    EXPECT_DOUBLE_EQ(config.los_multipath_cn0_delta_db, 0.5);
    EXPECT_DOUBLE_EQ(config.los_multipath_code_bias_chips, 0.02);

    config.acquisition_cn0_threshold_dbhz = config.minimum_tracking_cn0_dbhz;
    EXPECT_FALSE(gnss_sim::validate_signal_tracking_model_config(config, &error_message));
}

TEST(UrbanSignalTrackingAcquisition, ThresholdQualificationMustBeContinuousForTwoHundredMilliseconds) {
    const gnss_sim::SignalTrackingModelConfig config = urban_test_config();
    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2400, 100000.0, &start));
    gnss_sim::SignalTracker tracker{};
    schedule(&tracker, gnss_sim::SignalId::kGpsL1Ca, start, gnss_sim::AcquisitionContext::kHot, config);
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllRoot root = root_for(gps_l1, 0.0);
    std::string error_message;

    gnss_sim::SimTime current = start;
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(18.0, 18.0, true, &root, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kAcquiring);

    ASSERT_TRUE(add_ms(start, 150, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(18.0, 18.0, true, &root, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kAcquiring);

    ASSERT_TRUE(add_ms(start, 175, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(18.0, 17.9, true, &root, 1), config,
                                                      &error_message));
    EXPECT_FALSE(tracker.above_acquisition_threshold_active);

    ASSERT_TRUE(add_ms(start, 200, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(18.0, 18.0, true, &root, 1), config,
                                                      &error_message));
    ASSERT_TRUE(add_ms(start, 399, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(18.0, 18.0, true, &root, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kAcquiring);

    ASSERT_TRUE(add_ms(start, 400, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(18.0, 18.0, true, &root, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kTracking);
    EXPECT_EQ(tracker.lock_time_ns, 0);
}

TEST(UrbanSignalTrackingCn0, WeakTrackedSignalUsesFiveHundredMillisecondLossPersistence) {
    gnss_sim::SignalTrackingModelConfig config = urban_test_config();
    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2400, 110000.0, &start));
    gnss_sim::SignalTracker tracker{};
    acquire_immediately(&tracker, gnss_sim::SignalId::kGpsL1Ca, start, &config, false);
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllRoot root = root_for(gps_l1, 0.0);
    std::string error_message;
    gnss_sim::SimTime current{};

    ASSERT_TRUE(add_ms(start, 50, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(12.0, 12.0, false, &root, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kTracking);
    EXPECT_EQ(tracker.urban_state, gnss_sim::UrbanSignalState::kNlosTracked);

    ASSERT_TRUE(add_ms(start, 100, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(9.0, 9.0, false, &root, 1), config,
                                                      &error_message));
    ASSERT_TRUE(add_ms(start, 599, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(9.0, 9.0, false, &root, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kTracking);
    EXPECT_TRUE(tracker.has_tracking_lock);

    ASSERT_TRUE(add_ms(start, 600, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(9.0, 9.0, false, &root, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kSearching);
    EXPECT_FALSE(tracker.has_tracking_lock);
    EXPECT_TRUE(tracker.reacquisition_pending);
    EXPECT_EQ(tracker.loss_reason, gnss_sim::SignalTrackingLossReason::kLowCn0);
    EXPECT_EQ(tracker.urban_state, gnss_sim::UrbanSignalState::kBlocked);
    EXPECT_EQ(tracker.lock_time_ns, 0);
    EXPECT_FALSE(tracker.psr_valid);
    EXPECT_FALSE(tracker.doppler_valid);
    EXPECT_FALSE(tracker.adr_valid);
    EXPECT_FALSE(tracker.carrier_continuity_valid);
}

TEST(UrbanSignalTrackingReacquisition, RequiresCn0PersistenceAndExistingReacquisitionDelay) {
    gnss_sim::SignalTrackingModelConfig config = urban_test_config();
    for (int index = 0; index < 4; ++index) {
        config.reacquisition[index] = fixed_delay(300 * kMs);
    }
    config.acquisition_cn0_persistence_ns = 0;
    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2400, 120000.0, &start));
    gnss_sim::SignalTracker tracker{};
    acquire_immediately(&tracker, gnss_sim::SignalId::kGpsL1Ca, start, &config, false);
    config.acquisition_cn0_persistence_ns = 200 * kMs;
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllRoot root = root_for(gps_l1, 0.0);
    std::string error_message;
    gnss_sim::SimTime loss_start{};
    gnss_sim::SimTime current{};

    ASSERT_TRUE(add_ms(start, 100, &loss_start));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, loss_start, input(9.0, 9.0, false, &root, 1), config,
                                                      &error_message));
    ASSERT_TRUE(add_ms(start, 600, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(9.0, 9.0, false, &root, 1), config,
                                                      &error_message));
    ASSERT_TRUE(tracker.reacquisition_pending);

    schedule_existing(&tracker, current, gnss_sim::AcquisitionContext::kReacquisition, config, 22U);
    const gnss_sim::SimTime reacquisition_start = current;

    ASSERT_TRUE(add_ms(reacquisition_start, 300, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(15.0, 15.0, false, &root, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kAcquiring);

    ASSERT_TRUE(add_ms(reacquisition_start, 400, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(18.0, 18.0, false, &root, 1), config,
                                                      &error_message));
    ASSERT_TRUE(add_ms(reacquisition_start, 599, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(18.0, 18.0, false, &root, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kAcquiring);

    ASSERT_TRUE(add_ms(reacquisition_start, 600, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(18.0, 18.0, false, &root, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kTracking);
    EXPECT_TRUE(tracker.reacquisition_event);
    EXPECT_EQ(tracker.lock_time_ns, 0);
    EXPECT_FALSE(tracker.adr_valid);
    EXPECT_FALSE(tracker.carrier_continuity_valid);
    EXPECT_EQ(tracker.loss_reason, gnss_sim::SignalTrackingLossReason::kLowCn0);

    ASSERT_TRUE(add_ms(reacquisition_start, 601, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(18.0, 18.0, false, &root, 1), config,
                                                      &error_message));
    EXPECT_FALSE(tracker.reacquisition_event);
}

TEST(UrbanSignalTrackingDll, SmoothRootMotionPreservesLockButAbruptSwitchDoesNot) {
    gnss_sim::SignalTrackingModelConfig config = urban_test_config();
    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2400, 130000.0, &start));
    gnss_sim::SignalTracker tracker{};
    acquire_immediately(&tracker, gnss_sim::SignalId::kGpsL1Ca, start, &config);
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    std::string error_message;
    gnss_sim::SimTime current{};

    const gnss_sim::CodeTrackingDllRoot smooth = root_for(gps_l1, 0.20);
    ASSERT_TRUE(add_ms(start, 100, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(45.0, 45.0, true, &smooth, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kTracking);
    EXPECT_NEAR(tracker.current_dll_code_phase_chips, 0.20, 1.0e-15);

    const gnss_sim::CodeTrackingDllRoot abrupt = root_for(gps_l1, 0.46);
    ASSERT_TRUE(add_ms(start, 200, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(45.0, 45.0, true, &abrupt, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kSearching);
    EXPECT_EQ(tracker.loss_reason, gnss_sim::SignalTrackingLossReason::kAbruptDllRootSwitch);
    EXPECT_TRUE(tracker.reacquisition_pending);
}

TEST(UrbanSignalTrackingDll, MissingStableRootCausesImmediateLoss) {
    gnss_sim::SignalTrackingModelConfig config = urban_test_config();
    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2400, 140000.0, &start));
    gnss_sim::SignalTracker tracker{};
    acquire_immediately(&tracker, gnss_sim::SignalId::kGpsL1Ca, start, &config);
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllRoot unstable = root_for(gps_l1, 0.0, 1.0, false);
    gnss_sim::SimTime current{};
    ASSERT_TRUE(add_ms(start, 1, &current));
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(45.0, 45.0, true, &unstable, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.phase, gnss_sim::SignalTrackingPhase::kSearching);
    EXPECT_EQ(tracker.loss_reason, gnss_sim::SignalTrackingLossReason::kNoStableDllRoot);
}

TEST(UrbanSignalTrackingClassification, UsesCommonCn0AndDllResultRatherThanReflectionPresence) {
    gnss_sim::SignalTrackingModelConfig config = urban_test_config();
    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2400, 150000.0, &start));
    gnss_sim::SignalTracker tracker{};
    acquire_immediately(&tracker, gnss_sim::SignalId::kGpsL1Ca, start, &config, true);
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    std::string error_message;
    gnss_sim::SimTime current{};

    const gnss_sim::CodeTrackingDllRoot zero_bias = root_for(gps_l1, 0.0);
    ASSERT_TRUE(add_ms(start, 1, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(45.0, 45.0, true, &zero_bias, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.urban_state, gnss_sim::UrbanSignalState::kLos);

    ASSERT_TRUE(add_ms(start, 2, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(45.0, 44.5, true, &zero_bias, 1), config,
                                                      &error_message));
    EXPECT_EQ(tracker.urban_state, gnss_sim::UrbanSignalState::kLosMultipath);

    const gnss_sim::CodeTrackingDllRoot code_biased = root_for(gps_l1, 0.02);
    ASSERT_TRUE(add_ms(start, 3, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(45.0, 45.0, true, &code_biased, 1),
                                                      config, &error_message));
    EXPECT_EQ(tracker.urban_state, gnss_sim::UrbanSignalState::kLosMultipath);

    ASSERT_TRUE(add_ms(start, 4, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&tracker, current, input(45.0, 45.0, false, &code_biased, 1),
                                                      config, &error_message));
    EXPECT_EQ(tracker.urban_state, gnss_sim::UrbanSignalState::kNlosTracked);

    gnss_sim::SignalTrackingModelConfig acquiring_config = urban_test_config();
    acquiring_config.acquisition_cn0_persistence_ns = 200 * kMs;
    gnss_sim::SignalTracker acquiring{};
    schedule(&acquiring, gnss_sim::SignalId::kGpsL1Ca, start, gnss_sim::AcquisitionContext::kHot, acquiring_config);
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&acquiring, start, input(45.0, 45.0, false, &zero_bias, 1),
                                                      acquiring_config, &error_message));
    EXPECT_EQ(acquiring.urban_state, gnss_sim::UrbanSignalState::kBlocked);
}

TEST(UrbanSignalTrackingDll, ChipDomainJumpThresholdWorksAcrossDifferentCodeRates) {
    gnss_sim::SignalTrackingModelConfig config = urban_test_config();
    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2400, 160000.0, &start));
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::SignalDefinition& gps_l5 = signal(gnss_sim::SignalId::kGpsL5Q);
    ASSERT_NE(gps_l1.code_correlation.chip_rate_hz, gps_l5.code_correlation.chip_rate_hz);

    gnss_sim::SignalTracker l1{};
    gnss_sim::SignalTracker l5{};
    acquire_immediately(&l1, gps_l1.signal_id, start, &config);
    acquire_immediately(&l5, gps_l5.signal_id, start, &config);

    const gnss_sim::CodeTrackingDllRoot l1_smooth = root_for(gps_l1, 0.20);
    const gnss_sim::CodeTrackingDllRoot l5_smooth = root_for(gps_l5, 0.20);
    EXPECT_NE(l1_smooth.code_phase_sec, l5_smooth.code_phase_sec);
    gnss_sim::SimTime current{};
    ASSERT_TRUE(add_ms(start, 10, &current));
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&l1, current, input(45.0, 45.0, true, &l1_smooth, 1), config,
                                                      &error_message));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&l5, current, input(45.0, 45.0, true, &l5_smooth, 1), config,
                                                      &error_message));
    EXPECT_EQ(l1.phase, gnss_sim::SignalTrackingPhase::kTracking);
    EXPECT_EQ(l5.phase, gnss_sim::SignalTrackingPhase::kTracking);

    const gnss_sim::CodeTrackingDllRoot l1_abrupt = root_for(gps_l1, 0.46);
    const gnss_sim::CodeTrackingDllRoot l5_abrupt = root_for(gps_l5, 0.46);
    ASSERT_TRUE(add_ms(start, 20, &current));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&l1, current, input(45.0, 45.0, true, &l1_abrupt, 1), config,
                                                      &error_message));
    ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&l5, current, input(45.0, 45.0, true, &l5_abrupt, 1), config,
                                                      &error_message));
    EXPECT_EQ(l1.loss_reason, gnss_sim::SignalTrackingLossReason::kAbruptDllRootSwitch);
    EXPECT_EQ(l5.loss_reason, gnss_sim::SignalTrackingLossReason::kAbruptDllRootSwitch);
}

TEST(UrbanSignalTrackingDeterminism, IdenticalInputsProduceIdenticalStateSequence) {
    gnss_sim::SignalTrackingModelConfig config = urban_test_config();
    config.acquisition_cn0_persistence_ns = 0;
    gnss_sim::SimTime start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2400, 170000.0, &start));
    gnss_sim::SignalTracker first{};
    gnss_sim::SignalTracker second{};
    schedule(&first, gnss_sim::SignalId::kGalileoE1, start, gnss_sim::AcquisitionContext::kHot, config, 99U);
    schedule(&second, gnss_sim::SignalId::kGalileoE1, start, gnss_sim::AcquisitionContext::kHot, config, 99U);
    const gnss_sim::SignalDefinition& gal_e1 = signal(gnss_sim::SignalId::kGalileoE1);
    const gnss_sim::CodeTrackingDllRoot root = root_for(gal_e1, 0.0);
    const std::int64_t offsets_ms[] = {0, 100, 300, 600};
    const double cn0_dbhz[] = {45.0, 9.0, 9.0, 9.0};
    std::string first_error;
    std::string second_error;

    for (int index = 0; index < 4; ++index) {
        gnss_sim::SimTime current{};
        ASSERT_TRUE(add_ms(start, offsets_ms[index], &current));
        const gnss_sim::UrbanSignalTrackingInput update = input(cn0_dbhz[index], cn0_dbhz[index], false, &root, 1);
        ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&first, current, update, config, &first_error));
        ASSERT_TRUE(gnss_sim::update_urban_signal_tracker(&second, current, update, config, &second_error));
        EXPECT_EQ(first.phase, second.phase);
        EXPECT_EQ(first.urban_state, second.urban_state);
        EXPECT_EQ(first.loss_reason, second.loss_reason);
        EXPECT_EQ(first.lock_time_ns, second.lock_time_ns);
        EXPECT_EQ(first.reacquisition_pending, second.reacquisition_pending);
        EXPECT_EQ(first.previous_dll_code_phase_chips, second.previous_dll_code_phase_chips);
    }
}

} // namespace
