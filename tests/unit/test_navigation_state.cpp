#include "gnss/nav_output_record.h"
#include "gnss/navigation_state.h"
#include "gnss/rtklib_adapter.h"
#include "gnss_sim/sim_time.h"

#include <cmath>
#include <gtest/gtest.h>
#include <string>

namespace {

std::string mixed_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/mixed_nav_2019.rnx";
}

std::string update_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/nav_updates_2019.rnx";
}

class NavigationStateTest : public ::testing::Test {
  protected:
    void SetUp() override {
        state_ = gnss_sim::create_navigation_state();
        ASSERT_NE(state_, nullptr);
    }

    void TearDown() override {
        gnss_sim::destroy_navigation_state(state_);
    }

    gnss_sim::NavigationState* state_ = nullptr;
};

TEST_F(NavigationStateTest, ColdStartKeepsTruthAndReceiverNavigationPhysicallyIndependent) {
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_truth_navigation(state_, mixed_nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::SimTime startup_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180000.0, &startup_time));
    ASSERT_TRUE(
        gnss_sim::initialize_receiver_navigation(state_, gnss_sim::StartupMode::COLD, startup_time, &error_message))
        << error_message;

    gnss_sim::RtklibNavCounts truth_counts{};
    gnss_sim::RtklibNavCounts receiver_counts{};
    ASSERT_TRUE(gnss_sim::get_rtklib_nav_counts(gnss_sim::truth_navigation_store(state_), &truth_counts));
    ASSERT_TRUE(gnss_sim::get_rtklib_nav_counts(gnss_sim::receiver_navigation_store(state_), &receiver_counts));
    EXPECT_GE(truth_counts.gps_eph_count, 1);
    EXPECT_GE(truth_counts.glo_eph_count, 1);
    EXPECT_GE(truth_counts.gal_eph_count, 1);
    EXPECT_GE(truth_counts.bds_eph_count, 1);
    EXPECT_GE(truth_counts.qzss_eph_count, 1);
    EXPECT_EQ(receiver_counts.gps_eph_count, 0);
    EXPECT_EQ(receiver_counts.glo_eph_count, 0);
    EXPECT_EQ(receiver_counts.gal_eph_count, 0);
    EXPECT_EQ(receiver_counts.bds_eph_count, 0);
    EXPECT_EQ(receiver_counts.qzss_eph_count, 0);

    int gps_satellite = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &gps_satellite));
    gnss_sim::RtklibSatelliteState truth_state{};
    gnss_sim::RtklibSatelliteState receiver_state{};
    ASSERT_TRUE(gnss_sim::get_rtklib_satellite_state(gnss_sim::truth_navigation_store(state_), 2041, 180000.0,
                                                     gps_satellite, &truth_state, &error_message));
    EXPECT_FALSE(gnss_sim::get_rtklib_satellite_state(gnss_sim::receiver_navigation_store(state_), 2041, 180000.0,
                                                      gps_satellite, &receiver_state, &error_message));
}

TEST_F(NavigationStateTest, HotAndWarmRestoreCachedSnapshotImmediately) {
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_truth_navigation(state_, mixed_nav_path().c_str(), &error_message)) << error_message;
    gnss_sim::SimTime startup_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 300000.0, &startup_time));

    for (const gnss_sim::StartupMode mode : {gnss_sim::StartupMode::HOT, gnss_sim::StartupMode::WARM}) {
        ASSERT_TRUE(gnss_sim::initialize_receiver_navigation(state_, mode, startup_time, &error_message))
            << error_message;
        gnss_sim::RtklibNavCounts receiver_counts{};
        ASSERT_TRUE(gnss_sim::get_rtklib_nav_counts(gnss_sim::receiver_navigation_store(state_), &receiver_counts));
        EXPECT_GE(receiver_counts.gps_eph_count, 1);
        EXPECT_GE(receiver_counts.glo_eph_count, 1);
        EXPECT_GE(receiver_counts.gal_eph_count, 1);
        EXPECT_GE(receiver_counts.bds_eph_count, 1);
        EXPECT_GE(receiver_counts.qzss_eph_count, 1);

        for (int index = 0; index < gnss_sim::navigation_truth_record_count(state_); ++index) {
            gnss_sim::RtklibNavRecordInfo info{};
            ASSERT_TRUE(gnss_sim::rtklib_nav_record_info(gnss_sim::truth_navigation_store(state_), index, &info));
            gnss_sim::SimTime record_time{};
            ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(info.gps_week, info.transmit_sow_sec, &record_time));
            if (gnss_sim::compare_sim_time(record_time, startup_time) <= 0) {
                EXPECT_TRUE(gnss_sim::navigation_truth_record_delivered(state_, index));
            }
        }
    }
}

TEST_F(NavigationStateTest, RuntimeEphemerisUpdateEmitsExactlyOnceAndRetainsOldAndNewVersions) {
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_truth_navigation(state_, update_nav_path().c_str(), &error_message)) << error_message;
    ASSERT_EQ(gnss_sim::navigation_truth_record_count(state_), 2);

    gnss_sim::SimTime startup_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 179000.0, &startup_time));
    ASSERT_TRUE(
        gnss_sim::initialize_receiver_navigation(state_, gnss_sim::StartupMode::HOT, startup_time, &error_message))
        << error_message;

    gnss_sim::RtklibNavRecordInfo first_info{};
    gnss_sim::RtklibNavRecordInfo second_info{};
    ASSERT_TRUE(gnss_sim::rtklib_nav_record_info(gnss_sim::truth_navigation_store(state_), 0, &first_info));
    ASSERT_TRUE(gnss_sim::rtklib_nav_record_info(gnss_sim::truth_navigation_store(state_), 1, &second_info));
    EXPECT_EQ(first_info.iode, 21);
    EXPECT_EQ(second_info.iode, 22);
    EXPECT_TRUE(gnss_sim::navigation_truth_record_delivered(state_, 0));
    EXPECT_FALSE(gnss_sim::navigation_truth_record_delivered(state_, 1));

    gnss_sim::RtklibNavCounts receiver_counts{};
    ASSERT_TRUE(gnss_sim::get_rtklib_nav_counts(gnss_sim::receiver_navigation_store(state_), &receiver_counts));
    EXPECT_EQ(receiver_counts.gps_eph_count, 1);

    gnss_sim::SimTime availability_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180030.0, &availability_time));
    gnss_sim::NavigationUpdateEvent event{};
    bool emitted = false;
    ASSERT_TRUE(gnss_sim::apply_truth_navigation_record(state_, 1, availability_time, &event, &emitted, &error_message))
        << error_message;
    EXPECT_TRUE(emitted);
    EXPECT_EQ(event.truth_record_index, 1);
    EXPECT_EQ(event.iode, 22);
    EXPECT_EQ(event.kind, gnss_sim::RtklibNavRecordKind::kEphemeris);
    EXPECT_EQ(gnss_sim::compare_sim_time(event.availability_time, availability_time), 0);

    ASSERT_TRUE(gnss_sim::get_rtklib_nav_counts(gnss_sim::receiver_navigation_store(state_), &receiver_counts));
    EXPECT_EQ(receiver_counts.gps_eph_count, 2);

    emitted = true;
    ASSERT_TRUE(gnss_sim::apply_truth_navigation_record(state_, 1, availability_time, &event, &emitted, &error_message))
        << error_message;
    EXPECT_FALSE(emitted);
    ASSERT_TRUE(gnss_sim::get_rtklib_nav_counts(gnss_sim::receiver_navigation_store(state_), &receiver_counts));
    EXPECT_EQ(receiver_counts.gps_eph_count, 2);

    int gps_satellite = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &gps_satellite));
    for (const double sow_sec : {179000.0, 187200.0}) {
        gnss_sim::RtklibSatelliteState truth_state{};
        gnss_sim::RtklibSatelliteState receiver_state{};
        ASSERT_TRUE(gnss_sim::get_rtklib_satellite_state(gnss_sim::truth_navigation_store(state_), 2041, sow_sec,
                                                         gps_satellite, &truth_state, &error_message));
        ASSERT_TRUE(gnss_sim::get_rtklib_satellite_state(gnss_sim::receiver_navigation_store(state_), 2041, sow_sec,
                                                         gps_satellite, &receiver_state, &error_message));
        for (int axis = 0; axis < 3; ++axis) {
            EXPECT_NEAR(receiver_state.position_ecef_m[axis], truth_state.position_ecef_m[axis], 1.0e-6);
        }
    }
}

TEST_F(NavigationStateTest, ColdStartAcceptsSchedulerDeliveredEphemerisAfterStartup) {
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_truth_navigation(state_, update_nav_path().c_str(), &error_message)) << error_message;
    gnss_sim::SimTime startup_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 179000.0, &startup_time));
    ASSERT_TRUE(
        gnss_sim::initialize_receiver_navigation(state_, gnss_sim::StartupMode::COLD, startup_time, &error_message))
        << error_message;

    EXPECT_FALSE(gnss_sim::navigation_truth_record_delivered(state_, 0));
    int gps_satellite = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &gps_satellite));
    EXPECT_FALSE(
        gnss_sim::rtklib_nav_store_has_satellite_ephemeris(gnss_sim::receiver_navigation_store(state_), gps_satellite));

    gnss_sim::SimTime decoded_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 179020.0, &decoded_time));
    bool emitted = false;
    ASSERT_TRUE(gnss_sim::apply_truth_navigation_record(state_, 0, decoded_time, nullptr, &emitted, &error_message))
        << error_message;
    EXPECT_TRUE(emitted);
    EXPECT_TRUE(
        gnss_sim::rtklib_nav_store_has_satellite_ephemeris(gnss_sim::receiver_navigation_store(state_), gps_satellite));
}

TEST_F(NavigationStateTest, MixedTypeAppendReportsActualReceiverPartitionIndex) {
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_truth_navigation(state_, mixed_nav_path().c_str(), &error_message)) << error_message;
    gnss_sim::SimTime startup_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180000.0, &startup_time));
    ASSERT_TRUE(
        gnss_sim::initialize_receiver_navigation(state_, gnss_sim::StartupMode::COLD, startup_time, &error_message))
        << error_message;

    int glo_index = -1;
    int gps_index = -1;
    for (int index = 0; index < gnss_sim::navigation_truth_record_count(state_); ++index) {
        gnss_sim::RtklibNavRecordInfo info{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_record_info(gnss_sim::truth_navigation_store(state_), index, &info));
        if (info.kind == gnss_sim::RtklibNavRecordKind::kGlonassEphemeris && glo_index < 0) {
            glo_index = index;
        } else if (info.kind == gnss_sim::RtklibNavRecordKind::kEphemeris && info.satellite_number > 0 && gps_index < 0) {
            char satellite_id[4]{};
            ASSERT_TRUE(gnss_sim::rtklib_satellite_number_to_id(info.satellite_number, satellite_id));
            if (satellite_id[0] == 'G') {
                gps_index = index;
            }
        }
    }
    ASSERT_GE(glo_index, 0);
    ASSERT_GE(gps_index, 0);

    bool emitted = false;
    gnss_sim::NavigationUpdateEvent glo_event{};
    ASSERT_TRUE(gnss_sim::apply_truth_navigation_record(state_, glo_index, startup_time, &glo_event, &emitted,
                                                         &error_message))
        << error_message;
    ASSERT_TRUE(emitted);

    gnss_sim::NavigationUpdateEvent gps_event{};
    emitted = false;
    ASSERT_TRUE(gnss_sim::apply_truth_navigation_record(state_, gps_index, startup_time, &gps_event, &emitted,
                                                         &error_message))
        << error_message;
    ASSERT_TRUE(emitted);

    gnss_sim::NavOutputRecord output_record{};
    ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(gnss_sim::receiver_navigation_store(state_),
                                                   gps_event.receiver_record_index, &output_record, &error_message))
        << error_message;
    EXPECT_EQ(output_record.kind, gnss_sim::RtklibNavRecordKind::kEphemeris);
    EXPECT_EQ(output_record.ephemeris.satellite_number, gps_event.satellite_number);
    EXPECT_EQ(gps_event.receiver_record_index, 0);
}

} // namespace
