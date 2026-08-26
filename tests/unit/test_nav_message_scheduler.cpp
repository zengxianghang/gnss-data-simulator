#include "gnss/nav_message_scheduler.h"

#include "gnss/navigation_state.h"
#include "gnss/rtklib_adapter.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_time.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

namespace {

std::string mixed_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/mixed_nav_2019.rnx";
}

std::string update_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/nav_updates_2019.rnx";
}

void expect_availability(gnss_sim::SignalId signal_id, double acquisition_sow_sec, double expected_sow_sec) {
    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, acquisition_sow_sec, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(signal_id, acquisition_time, 21, &plan, &error_message))
        << error_message;
    EXPECT_EQ(plan.availability_time.gps_week, 2300);
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(plan.availability_time), expected_sow_sec, 1.0e-9);
    EXPECT_TRUE(gnss_sim::nav_acquisition_complete(plan, plan.availability_time));
}

int find_truth_record(const gnss_sim::NavigationState* state, int satellite_number) {
    for (int index = 0; index < gnss_sim::navigation_truth_record_count(state); ++index) {
        gnss_sim::RtklibNavRecordInfo info{};
        if (gnss_sim::rtklib_nav_record_info(gnss_sim::truth_navigation_store(state), index, &info) &&
            info.satellite_number == satellite_number) {
            return index;
        }
    }
    return -1;
}

TEST(ColdNavSchedulerTiming, LegacyGpsAndQzssUseThirtySecondFramePhase) {
    expect_availability(gnss_sim::SignalId::kGpsL1Ca, 180000.0, 180018.0);
    expect_availability(gnss_sim::SignalId::kGpsL2P, 180000.0, 180018.0);
    expect_availability(gnss_sim::SignalId::kQzssL1Ca, 180000.0, 180018.0);

    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180000.001, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGpsL1Ca, acquisition_time, 21, &plan,
                                                         &error_message));
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(plan.availability_time), 180036.0, 1.0e-9);
    EXPECT_EQ(plan.fragment_count, 3);
    EXPECT_EQ(plan.fragments[0].fragment_id, 1);
    EXPECT_EQ(plan.fragments[1].fragment_id, 2);
    EXPECT_EQ(plan.fragments[2].fragment_id, 3);
}

TEST(ColdNavSchedulerTiming, CnavUsesSignalSpecificMessageRate) {
    expect_availability(gnss_sim::SignalId::kGpsL2C, 180000.0, 180036.0);
    expect_availability(gnss_sim::SignalId::kQzssL2C, 180000.0, 180036.0);
    expect_availability(gnss_sim::SignalId::kGpsL5Q, 180000.0, 180018.0);
    expect_availability(gnss_sim::SignalId::kQzssL5Q, 180000.0, 180018.0);

    gnss_sim::SimTime l2_acquisition{};
    gnss_sim::SimTime l5_acquisition{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180000.001, &l2_acquisition));
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180000.001, &l5_acquisition));
    gnss_sim::NavAcquisitionPlan l2_plan{};
    gnss_sim::NavAcquisitionPlan l5_plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGpsL2C, l2_acquisition, 21, &l2_plan,
                                                         &error_message));
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGpsL5Q, l5_acquisition, 21, &l5_plan,
                                                         &error_message));
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(l2_plan.availability_time), 180048.0, 1.0e-9);
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(l5_plan.availability_time), 180024.0, 1.0e-9);
    EXPECT_EQ(l2_plan.fragments[0].fragment_id, 10);
    EXPECT_EQ(l2_plan.fragments[1].fragment_id, 11);
    EXPECT_EQ(l2_plan.fragments[2].fragment_id, 30);
}

TEST(ColdNavSchedulerTiming, Cnav2RequiresCompleteEighteenSecondFrame) {
    expect_availability(gnss_sim::SignalId::kGpsL1C, 180000.0, 180018.0);
    expect_availability(gnss_sim::SignalId::kQzssL1C, 180000.0, 180018.0);

    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180000.001, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGpsL1C, acquisition_time, 21, &plan,
                                                         &error_message));
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(plan.availability_time), 180036.0, 1.0e-9);
    EXPECT_EQ(plan.fragment_count, 1);
}

TEST(ColdNavSchedulerTiming, GlonassFdmaCollectsStringsOneThroughFourWithoutThirtyMinuteWait) {
    expect_availability(gnss_sim::SignalId::kGlonassG1, 180000.0, 180008.0);
    expect_availability(gnss_sim::SignalId::kGlonassG2, 180000.0, 180008.0);

    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180000.001, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGlonassG1, acquisition_time, 5, &plan,
                                                         &error_message));
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(plan.availability_time), 180032.0, 1.0e-9);
    EXPECT_LT(gnss_sim::sim_time_sow_sec(plan.availability_time) - 180000.001, 33.0);
    EXPECT_EQ(plan.fragment_count, 4);
}

TEST(ColdNavSchedulerTiming, GlonassL3OcCollectsThreeImmediateStrings) {
    expect_availability(gnss_sim::SignalId::kGlonassG3, 180000.0, 180009.0);

    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180000.001, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGlonassG3, acquisition_time, 5, &plan,
                                                         &error_message));
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(plan.availability_time), 180021.0, 1.0e-9);
    EXPECT_EQ(plan.fragments[0].fragment_id, 10);
    EXPECT_EQ(plan.fragments[1].fragment_id, 11);
    EXPECT_EQ(plan.fragments[2].fragment_id, 12);
}

TEST(ColdNavSchedulerTiming, WeekBoundaryIsHandledWithIntegerSimTime) {
    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 604799.0, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGpsL1Ca, acquisition_time, 21, &plan,
                                                         &error_message));
    EXPECT_EQ(plan.availability_time.gps_week, 2301);
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(plan.availability_time), 18.0, 1.0e-9);
}

TEST(ColdNavSchedulerCollector, IssueChangeResetsPartialFragmentSet) {
    gnss_sim::NavFragmentCollector collector{};
    gnss_sim::reset_nav_fragment_collector(0x7U, &collector);
    bool complete = false;
    ASSERT_TRUE(gnss_sim::ingest_nav_fragment(&collector, 0x1U, 21, &complete));
    EXPECT_FALSE(complete);
    ASSERT_TRUE(gnss_sim::ingest_nav_fragment(&collector, 0x2U, 21, &complete));
    EXPECT_FALSE(complete);
    EXPECT_EQ(collector.received_mask, 0x3U);

    ASSERT_TRUE(gnss_sim::ingest_nav_fragment(&collector, 0x4U, 22, &complete));
    EXPECT_FALSE(complete);
    EXPECT_EQ(collector.issue_data, 22);
    EXPECT_EQ(collector.received_mask, 0x4U);

    ASSERT_TRUE(gnss_sim::ingest_nav_fragment(&collector, 0x1U, 22, &complete));
    ASSERT_TRUE(gnss_sim::ingest_nav_fragment(&collector, 0x2U, 22, &complete));
    EXPECT_TRUE(complete);
    EXPECT_EQ(collector.received_mask, 0x7U);
}

TEST(ColdNavSchedulerMask, FragmentsAppearOnlyAfterTheirCompleteMessageBoundary) {
    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180000.0, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGpsL1Ca, acquisition_time, 21, &plan,
                                                         &error_message));

    gnss_sim::SimTime at_six{};
    gnss_sim::SimTime before_eighteen{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180006.0, &at_six));
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180017.999, &before_eighteen));
    EXPECT_EQ(gnss_sim::nav_acquisition_received_mask(plan, at_six), 0x1U);
    EXPECT_EQ(gnss_sim::nav_acquisition_received_mask(plan, before_eighteen), 0x3U);
    EXPECT_FALSE(gnss_sim::nav_acquisition_complete(plan, before_eighteen));
    EXPECT_TRUE(gnss_sim::nav_acquisition_complete(plan, plan.availability_time));
}

TEST(ColdNavSchedulerIntegration, ColdReceiverNavGrowsOneCompletedAcquisitionAtATime) {
    gnss_sim::NavigationState* state = gnss_sim::create_navigation_state();
    ASSERT_NE(state, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_truth_navigation(state, mixed_nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::SimTime startup_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180000.0, &startup_time));
    ASSERT_TRUE(gnss_sim::initialize_receiver_navigation(state, gnss_sim::StartupMode::COLD, startup_time,
                                                        &error_message))
        << error_message;

    int gps_satellite = 0;
    int glo_satellite = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &gps_satellite));
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("R26", &glo_satellite));
    const int gps_record = find_truth_record(state, gps_satellite);
    const int glo_record = find_truth_record(state, glo_satellite);
    ASSERT_GE(gps_record, 0);
    ASSERT_GE(glo_record, 0);

    gnss_sim::RtklibNavRecordInfo gps_info{};
    gnss_sim::RtklibNavRecordInfo glo_info{};
    ASSERT_TRUE(gnss_sim::rtklib_nav_record_info(gnss_sim::truth_navigation_store(state), gps_record, &gps_info));
    ASSERT_TRUE(gnss_sim::rtklib_nav_record_info(gnss_sim::truth_navigation_store(state), glo_record, &glo_info));

    gnss_sim::NavAcquisitionPlan gps_plan{};
    gnss_sim::NavAcquisitionPlan glo_plan{};
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGpsL1Ca, startup_time, gps_info.iode,
                                                         &gps_plan, &error_message));
    gnss_sim::SimTime glo_acquisition{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180020.001, &glo_acquisition));
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGlonassG1, glo_acquisition,
                                                         glo_info.iode, &glo_plan, &error_message));

    bool emitted = false;
    ASSERT_TRUE(gnss_sim::deliver_cold_nav_plan_if_complete(gps_plan, gps_record, gps_plan.availability_time, state,
                                                           nullptr, &emitted, &error_message))
        << error_message;
    EXPECT_TRUE(emitted);

    gnss_sim::RtklibNavCounts counts{};
    ASSERT_TRUE(gnss_sim::get_rtklib_nav_counts(gnss_sim::receiver_navigation_store(state), &counts));
    EXPECT_EQ(counts.gps_eph_count, 1);
    EXPECT_EQ(counts.glo_eph_count, 0);

    emitted = true;
    ASSERT_TRUE(gnss_sim::deliver_cold_nav_plan_if_complete(glo_plan, glo_record, gps_plan.availability_time, state,
                                                           nullptr, &emitted, &error_message));
    EXPECT_FALSE(emitted);

    ASSERT_TRUE(gnss_sim::deliver_cold_nav_plan_if_complete(glo_plan, glo_record, glo_plan.availability_time, state,
                                                           nullptr, &emitted, &error_message))
        << error_message;
    EXPECT_TRUE(emitted);
    ASSERT_TRUE(gnss_sim::get_rtklib_nav_counts(gnss_sim::receiver_navigation_store(state), &counts));
    EXPECT_EQ(counts.gps_eph_count, 1);
    EXPECT_EQ(counts.glo_eph_count, 1);

    emitted = true;
    ASSERT_TRUE(gnss_sim::deliver_cold_nav_plan_if_complete(glo_plan, glo_record, glo_plan.availability_time, state,
                                                           nullptr, &emitted, &error_message));
    EXPECT_FALSE(emitted);
    gnss_sim::destroy_navigation_state(state);
}

TEST(ColdNavSchedulerIntegration, PlanCannotDeliverDifferentIssueOfData) {
    gnss_sim::NavigationState* state = gnss_sim::create_navigation_state();
    ASSERT_NE(state, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_truth_navigation(state, update_nav_path().c_str(), &error_message)) << error_message;
    gnss_sim::SimTime startup_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 179000.0, &startup_time));
    ASSERT_TRUE(gnss_sim::initialize_receiver_navigation(state, gnss_sim::StartupMode::COLD, startup_time,
                                                        &error_message));

    gnss_sim::NavAcquisitionPlan wrong_plan{};
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGpsL1Ca, startup_time, 999,
                                                         &wrong_plan, &error_message));
    bool emitted = false;
    EXPECT_FALSE(gnss_sim::deliver_cold_nav_plan_if_complete(wrong_plan, 0, wrong_plan.availability_time, state,
                                                            nullptr, &emitted, &error_message));
    EXPECT_FALSE(emitted);
    gnss_sim::destroy_navigation_state(state);
}

} // namespace
