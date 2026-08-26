#include "gnss/nav_message_scheduler.h"
#include "gnss/navigation_state.h"
#include "gnss/rtklib_adapter.h"
#include "gnss_sim/sim_time.h"

#include <gtest/gtest.h>
#include <string>

namespace {

std::string mixed_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/mixed_nav_2019.rnx";
}

void expect_availability(gnss_sim::SignalId signal_id, double acquisition_sow_sec, double expected_sow_sec) {
    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, acquisition_sow_sec, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(signal_id, acquisition_time, 7, &plan, &error_message))
        << error_message;
    EXPECT_EQ(plan.availability_time.gps_week, 2300);
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(plan.availability_time), expected_sow_sec, 1.0e-9);
    EXPECT_TRUE(gnss_sim::nav_acquisition_complete(plan, plan.availability_time));
}

void expect_bds_variant_availability(gnss_sim::SignalId signal_id, gnss_sim::NavScheduleVariant variant,
                                     double acquisition_sow_sec, double expected_sow_sec, int expected_fragments) {
    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, acquisition_sow_sec, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan_with_variant(signal_id, variant, acquisition_time, 7, &plan,
                                                                       &error_message))
        << error_message;
    EXPECT_EQ(plan.availability_time.gps_week, 2300);
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(plan.availability_time), expected_sow_sec, 1.0e-9);
    EXPECT_EQ(plan.fragment_count, expected_fragments);
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

TEST(ColdNavGalileo, InavUsesSignalSpecificNominalWordSwapSchedule) {
    expect_availability(gnss_sim::SignalId::kGalileoE1, 180000.0, 180025.0);
    expect_availability(gnss_sim::SignalId::kGalileoE5B, 180000.0, 180024.0);

    gnss_sim::SimTime e1_after_word2_start{};
    gnss_sim::SimTime e5b_after_word1_start{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180001.001, &e1_after_word2_start));
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180000.001, &e5b_after_word1_start));

    gnss_sim::NavAcquisitionPlan e1_plan{};
    gnss_sim::NavAcquisitionPlan e5b_plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGalileoE1, e1_after_word2_start, 7,
                                                         &e1_plan, &error_message));
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGalileoE5B, e5b_after_word1_start, 7,
                                                         &e5b_plan, &error_message));
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(e1_plan.availability_time), 180033.0, 1.0e-9);
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(e5b_plan.availability_time), 180032.0, 1.0e-9);
    EXPECT_EQ(e1_plan.fragment_count, 4);
    EXPECT_EQ(e5b_plan.fragment_count, 4);
}

TEST(ColdNavGalileo, FnavRequiresClockPageAndThreeEphemerisPages) {
    expect_availability(gnss_sim::SignalId::kGalileoE5A, 180000.0, 180040.0);

    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180000.001, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGalileoE5A, acquisition_time, 7, &plan,
                                                         &error_message));
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(plan.availability_time), 180060.0, 1.0e-9);
    ASSERT_EQ(plan.fragment_count, 4);
    EXPECT_EQ(plan.fragments[0].fragment_id, 1);
    EXPECT_EQ(plan.fragments[1].fragment_id, 2);
    EXPECT_EQ(plan.fragments[2].fragment_id, 3);
    EXPECT_EQ(plan.fragments[3].fragment_id, 4);
}

TEST(ColdNavGalileo, E6HasNoNormalBroadcastEphemerisAcquisitionPlan) {
    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180000.0, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGalileoE6, acquisition_time, 7, &plan,
                                                          &error_message));
    EXPECT_NE(error_message.find("does not provide the normal broadcast ephemeris"), std::string::npos);
}

TEST(ColdNavBeidou, D1AndD2AreDistinctAndUseBdtPhase) {
    // GPST 180014 corresponds to BDT 180000, exactly on a BDT frame boundary.
    expect_bds_variant_availability(gnss_sim::SignalId::kBeidouB1I, gnss_sim::NavScheduleVariant::kBeidouD1,
                                    180014.0, 180032.0, 3);
    expect_bds_variant_availability(gnss_sim::SignalId::kBeidouB1I, gnss_sim::NavScheduleVariant::kBeidouD2,
                                    180014.0, 180041.6, 10);
    expect_bds_variant_availability(gnss_sim::SignalId::kBeidouB3I, gnss_sim::NavScheduleVariant::kBeidouD1,
                                    180014.0, 180032.0, 3);
    expect_bds_variant_availability(gnss_sim::SignalId::kBeidouB3I, gnss_sim::NavScheduleVariant::kBeidouD2,
                                    180014.0, 180041.6, 10);

    gnss_sim::SimTime just_after_page1{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180014.001, &just_after_page1));
    gnss_sim::NavAcquisitionPlan d2_plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan_with_variant(
        gnss_sim::SignalId::kBeidouB1I, gnss_sim::NavScheduleVariant::kBeidouD2, just_after_page1, 7, &d2_plan,
        &error_message));
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(d2_plan.availability_time), 180044.6, 1.0e-9);
}

TEST(ColdNavBeidou, DefaultLegacyVariantIsD1) {
    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180014.0, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kBeidouB1I, acquisition_time, 7, &plan,
                                                         &error_message));
    EXPECT_EQ(plan.variant, gnss_sim::NavScheduleVariant::kBeidouD1);
    EXPECT_EQ(plan.fragment_count, 3);
}

TEST(ColdNavBeidou, ModernFamiliesUseDifferentFrameAndMessageRules) {
    expect_availability(gnss_sim::SignalId::kBeidouB1C, 180014.0, 180032.0);
    expect_availability(gnss_sim::SignalId::kBeidouB2A, 180014.0, 180023.0);
    expect_availability(gnss_sim::SignalId::kBeidouB2B, 180014.0, 180016.0);

    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180014.001, &acquisition_time));
    std::string error_message;

    gnss_sim::NavAcquisitionPlan b1c_plan{};
    gnss_sim::NavAcquisitionPlan b2a_plan{};
    gnss_sim::NavAcquisitionPlan b2b_plan{};
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kBeidouB1C, acquisition_time, 7,
                                                         &b1c_plan, &error_message));
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kBeidouB2A, acquisition_time, 7,
                                                         &b2a_plan, &error_message));
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kBeidouB2B, acquisition_time, 7,
                                                         &b2b_plan, &error_message));
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(b1c_plan.availability_time), 180050.0, 1.0e-9);
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(b2a_plan.availability_time), 180026.0, 1.0e-9);
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(b2b_plan.availability_time), 180017.0, 1.0e-9);
    EXPECT_EQ(b2a_plan.fragments[0].fragment_id, 10);
    EXPECT_EQ(b2a_plan.fragments[1].fragment_id, 11);
    EXPECT_EQ(b2a_plan.fragments[2].fragment_id, 30);
    EXPECT_EQ(b2b_plan.fragments[0].fragment_id, 10);
    EXPECT_EQ(b2b_plan.fragments[1].fragment_id, 30);
}

TEST(ColdNavBeidou, D1CrossesGpsWeekWhileKeepingBdtFramePhase) {
    // BDT 604770 is the last complete D1 frame start of the BDT week;
    // it corresponds to GPST week 2300 SOW 604784.
    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 604784.0, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan_with_variant(
        gnss_sim::SignalId::kBeidouB1I, gnss_sim::NavScheduleVariant::kBeidouD1, acquisition_time, 7, &plan,
        &error_message));
    EXPECT_EQ(plan.availability_time.gps_week, 2301);
    EXPECT_NEAR(gnss_sim::sim_time_sow_sec(plan.availability_time), 2.0, 1.0e-9);
}

TEST(ColdNavBeidou, VariantCannotBeAppliedToUnrelatedConstellation) {
    gnss_sim::SimTime acquisition_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 180000.0, &acquisition_time));
    gnss_sim::NavAcquisitionPlan plan{};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::build_cold_nav_acquisition_plan_with_variant(
        gnss_sim::SignalId::kGpsL1Ca, gnss_sim::NavScheduleVariant::kBeidouD2, acquisition_time, 7, &plan,
        &error_message));
    EXPECT_FALSE(error_message.empty());
}

TEST(ColdNavGalileoBeidouIntegration, ReceiverNavGrowsOnlyAfterEachConstellationPlanCompletes) {
    gnss_sim::NavigationState* state = gnss_sim::create_navigation_state();
    ASSERT_NE(state, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_truth_navigation(state, mixed_nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::SimTime startup_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180000.0, &startup_time));
    ASSERT_TRUE(
        gnss_sim::initialize_receiver_navigation(state, gnss_sim::StartupMode::COLD, startup_time, &error_message));

    int gal_satellite = 0;
    int bds_satellite = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("E01", &gal_satellite));
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("C01", &bds_satellite));
    const int gal_record = find_truth_record(state, gal_satellite);
    const int bds_record = find_truth_record(state, bds_satellite);
    ASSERT_GE(gal_record, 0);
    ASSERT_GE(bds_record, 0);

    gnss_sim::RtklibNavRecordInfo gal_info{};
    gnss_sim::RtklibNavRecordInfo bds_info{};
    ASSERT_TRUE(gnss_sim::rtklib_nav_record_info(gnss_sim::truth_navigation_store(state), gal_record, &gal_info));
    ASSERT_TRUE(gnss_sim::rtklib_nav_record_info(gnss_sim::truth_navigation_store(state), bds_record, &bds_info));

    gnss_sim::NavAcquisitionPlan gal_plan{};
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan(gnss_sim::SignalId::kGalileoE1, startup_time,
                                                         gal_info.iode, &gal_plan, &error_message));
    gnss_sim::SimTime bds_acquisition{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180014.0, &bds_acquisition));
    gnss_sim::NavAcquisitionPlan bds_plan{};
    ASSERT_TRUE(gnss_sim::build_cold_nav_acquisition_plan_with_variant(
        gnss_sim::SignalId::kBeidouB1I, gnss_sim::NavScheduleVariant::kBeidouD2, bds_acquisition, bds_info.iode,
        &bds_plan, &error_message));

    bool emitted = false;
    ASSERT_TRUE(gnss_sim::deliver_cold_nav_plan_if_complete(gal_plan, gal_record, gal_plan.availability_time, state,
                                                           nullptr, &emitted, &error_message));
    EXPECT_TRUE(emitted);
    gnss_sim::RtklibNavCounts counts{};
    ASSERT_TRUE(gnss_sim::get_rtklib_nav_counts(gnss_sim::receiver_navigation_store(state), &counts));
    EXPECT_EQ(counts.gal_eph_count, 1);
    EXPECT_EQ(counts.bds_eph_count, 0);

    emitted = true;
    ASSERT_TRUE(gnss_sim::deliver_cold_nav_plan_if_complete(bds_plan, bds_record, gal_plan.availability_time, state,
                                                           nullptr, &emitted, &error_message));
    EXPECT_FALSE(emitted);
    ASSERT_TRUE(gnss_sim::deliver_cold_nav_plan_if_complete(bds_plan, bds_record, bds_plan.availability_time, state,
                                                           nullptr, &emitted, &error_message));
    EXPECT_TRUE(emitted);
    ASSERT_TRUE(gnss_sim::get_rtklib_nav_counts(gnss_sim::receiver_navigation_store(state), &counts));
    EXPECT_EQ(counts.gal_eph_count, 1);
    EXPECT_EQ(counts.bds_eph_count, 1);

    gnss_sim::destroy_navigation_state(state);
}

} // namespace
