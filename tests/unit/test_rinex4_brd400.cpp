#include "gnss/rtklib_adapter.h"

extern "C" {
#include <rtklib.h>
}

#include <gtest/gtest.h>
#include <string>

namespace {

std::string brd4_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

TEST(Rinex4Brd400, ProjectLoaderPreservesFiveSystemsAndModernEphemerisFamilies) {
    gnss_sim::RtklibNavStore* store = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(store, nullptr);

    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(store, brd4_nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::RtklibNavCounts counts{};
    ASSERT_TRUE(gnss_sim::get_rtklib_nav_counts(store, &counts));
    EXPECT_GT(counts.gps_eph_count, 0);
    EXPECT_GT(counts.glo_eph_count, 0);
    EXPECT_GT(counts.gal_eph_count, 0);
    EXPECT_GT(counts.bds_eph_count, 0);
    EXPECT_GT(counts.qzss_eph_count, 0);

    bool gps_cnav = false;
    bool qzss_cnav = false;
    bool qzss_cnav2 = false;
    bool bds_cnav1 = false;
    bool bds_cnav2 = false;
    bool bds_cnav3 = false;
    const int record_count = gnss_sim::rtklib_nav_record_count(store);
    for (int index = 0; index < record_count; ++index) {
        gnss_sim::RtklibNavRecordInfo info{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_record_info(store, index, &info));
        if (info.kind != gnss_sim::RtklibNavRecordKind::kEphemeris) {
            continue;
        }
        if (info.system == SYS_GPS && info.message_type == NAV_CNAV) {
            gps_cnav = true;
        } else if (info.system == SYS_QZS && info.message_type == NAV_CNAV) {
            qzss_cnav = true;
        } else if (info.system == SYS_QZS && info.message_type == NAV_CNV2) {
            qzss_cnav2 = true;
        } else if (info.system == SYS_CMP && info.message_type == NAV_CNV1) {
            bds_cnav1 = true;
        } else if (info.system == SYS_CMP && info.message_type == NAV_CNV2) {
            bds_cnav2 = true;
        } else if (info.system == SYS_CMP && info.message_type == NAV_CNV3) {
            bds_cnav3 = true;
        }
    }
    EXPECT_TRUE(gps_cnav);
    EXPECT_TRUE(qzss_cnav);
    EXPECT_TRUE(qzss_cnav2);
    EXPECT_TRUE(bds_cnav1);
    EXPECT_TRUE(bds_cnav2);
    EXPECT_TRUE(bds_cnav3);

    gnss_sim::destroy_rtklib_nav_store(store);
}

TEST(Rinex4Brd400, PinnedRtklibConsumesStoEopAndIonRecords) {
    nav_t nav{};
    obs_t obs{};
    sta_t station{};
    const int status = readrnx(brd4_nav_path().c_str(), 1, "", &obs, &nav, &station);
    freeobs(&obs);
    ASSERT_NE(status, 0);

    EXPECT_GT(nav.nion, 0);
    EXPECT_GT(nav.neop, 0);
    EXPECT_GT(nav.nsto, 0);

    freenav(&nav, 0xFF);
}

} // namespace
