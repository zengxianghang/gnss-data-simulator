#include "gnss/nav_output_record.h"
#include "gnss/rtklib_adapter.h"
#include "gnss_sim/simulator.h"

#include <gtest/gtest.h>

extern "C" {
#include <rtklib.h>
}

#ifdef lock
#undef lock
#endif
#ifdef unlock
#undef unlock
#endif

#include <cstring>
#include <string>

namespace {

std::string test_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/mixed_nav_2019.rnx";
}

std::string rtklib_reference_nav_path() {
    std::string path = test_nav_path();
#ifdef _WIN32
    for (char& character : path) {
        if (character == '/') {
            character = '\\';
        }
    }
#endif
    return path;
}

std::string invalid_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/invalid_nav.rnx";
}

std::string rinex4_acceptance_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

class RtklibAdapterTest : public ::testing::Test {
  protected:
    void SetUp() override {
        store_ = gnss_sim::create_rtklib_nav_store();
        ASSERT_NE(store_, nullptr);
    }

    void TearDown() override {
        gnss_sim::destroy_rtklib_nav_store(store_);
    }

    gnss_sim::RtklibNavStore* store_ = nullptr;
};

TEST_F(RtklibAdapterTest, LoadsRepresentativeMixedNavigationFile) {
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(store_, test_nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::RtklibNavCounts counts{};
    ASSERT_TRUE(gnss_sim::get_rtklib_nav_counts(store_, &counts));
    EXPECT_GE(counts.gps_eph_count, 1);
    EXPECT_GE(counts.glo_eph_count, 1);
    EXPECT_GE(counts.gal_eph_count, 1);
    EXPECT_GE(counts.bds_eph_count, 1);
    EXPECT_GE(counts.qzss_eph_count, 1);
}

TEST(RtklibAdapterMetadata, ExposesExactPinnedRtklibCommit) {
    EXPECT_STREQ(gnss_sim::rtklib_commit_sha(), GNSS_SIM_RTKLIB_COMMIT);
    EXPECT_EQ(std::strlen(gnss_sim::rtklib_commit_sha()), 40U);
}

TEST_F(RtklibAdapterTest, SatelliteIdMappingCoversFrozenConstellations) {
    const char* const satellite_ids[] = {"G01", "R26", "E01", "C01", "J01"};
    for (const char* satellite_id : satellite_ids) {
        int satellite_number = 0;
        EXPECT_TRUE(gnss_sim::rtklib_satellite_id_to_number(satellite_id, &satellite_number));
        EXPECT_GT(satellite_number, 0);
    }
}

TEST_F(RtklibAdapterTest, BroadcastSatelliteStateMatchesDirectRtklibReference) {
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(store_, test_nav_path().c_str(), &error_message)) << error_message;

    int satellite_number = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &satellite_number));

    gnss_sim::RtklibSatelliteState adapter_state{};
    ASSERT_TRUE(
        gnss_sim::get_rtklib_satellite_state(store_, 2041, 180000.0, satellite_number, &adapter_state, &error_message))
        << error_message;

    nav_t reference_nav{};
    obs_t reference_obs{};
    sta_t reference_station{};
    const std::string reference_path = rtklib_reference_nav_path();
    ASSERT_NE(readrnx(reference_path.c_str(), 1, "", &reference_obs, &reference_nav, &reference_station), 0);
    freeobs(&reference_obs);
    uniqnav(&reference_nav);

    const gtime_t time = gpst2time(2041, 180000.0);
    double reference_state[6]{};
    double reference_clock[2]{};
    double reference_variance_m2 = 0.0;
    int reference_health = 0;
    ASSERT_NE(satpos(time, time, satellite_number, EPHOPT_BRDC, &reference_nav, reference_state, reference_clock,
                     &reference_variance_m2, &reference_health),
              0);

    for (int index = 0; index < 3; ++index) {
        EXPECT_NEAR(adapter_state.position_ecef_m[index], reference_state[index], 1e-6);
        EXPECT_NEAR(adapter_state.velocity_ecef_mps[index], reference_state[index + 3], 1e-9);
    }
    EXPECT_NEAR(adapter_state.clock_bias_sec, reference_clock[0], 1e-15);
    EXPECT_NEAR(adapter_state.clock_drift_sec_per_sec, reference_clock[1], 1e-18);
    EXPECT_NEAR(adapter_state.variance_m2, reference_variance_m2, 1e-12);
    EXPECT_EQ(adapter_state.health, reference_health);

    freenav(&reference_nav, 0xFF);
}

TEST_F(RtklibAdapterTest, SnapshotRetainsSameSatelliteLegacyAndModernMessageFamilies) {
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(store_, rinex4_acceptance_nav_path().c_str(), &error_message))
        << error_message;

    gnss_sim::RtklibNavStore* snapshot = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(snapshot, nullptr);
    ASSERT_TRUE(gnss_sim::rtklib_copy_nav_snapshot(store_, 2347, 437100.0, snapshot, &error_message)) << error_message;

    int g17 = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G17", &g17));
    bool have_lnav = false;
    bool have_cnav = false;
    int g17_ephemeris_count = 0;
    for (int index = 0; index < gnss_sim::rtklib_nav_record_count(snapshot); ++index) {
        gnss_sim::RtklibNavRecordInfo info{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_record_info(snapshot, index, &info));
        if (info.kind != gnss_sim::RtklibNavRecordKind::kEphemeris || info.satellite_number != g17) {
            continue;
        }
        ++g17_ephemeris_count;
        have_lnav = have_lnav || (info.message_type & NAV_LNAV) != 0;
        have_cnav = have_cnav || (info.message_type & NAV_CNAV) != 0;
    }
    EXPECT_TRUE(have_lnav);
    EXPECT_TRUE(have_cnav);
    EXPECT_GE(g17_ephemeris_count, 2);

    gnss_sim::destroy_rtklib_nav_store(snapshot);
}

TEST_F(RtklibAdapterTest, GpsCnavHealthIsInterpretedPerSignal) {
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(store_, rinex4_acceptance_nav_path().c_str(), &error_message))
        << error_message;

    int g17 = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G17", &g17));

    int l2c_health = -1;
    ASSERT_TRUE(gnss_sim::rtklib_signal_health_for_family(
        store_, 2347, 437100.0, g17, "2S", gnss_sim::RtklibBroadcastMessageFamily::kCnav, &l2c_health, &error_message))
        << error_message;
    EXPECT_EQ(l2c_health, 0);

    int l5q_health = -1;
    ASSERT_TRUE(gnss_sim::rtklib_signal_health_for_family(
        store_, 2347, 437100.0, g17, "5Q", gnss_sim::RtklibBroadcastMessageFamily::kCnav, &l5q_health, &error_message))
        << error_message;
    EXPECT_NE(l5q_health, 0);
}

TEST(RtklibAdapterCoordinates, LlhEcefRoundTripUsesRtklibReferenceFunctions) {
    double ecef_m[3]{};
    ASSERT_TRUE(gnss_sim::rtklib_llh_to_ecef(20.0, 120.0, 100.0, ecef_m));

    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double height_m = 0.0;
    ASSERT_TRUE(gnss_sim::rtklib_ecef_to_llh(ecef_m, &latitude_deg, &longitude_deg, &height_m));
    EXPECT_NEAR(latitude_deg, 20.0, 1e-10);
    EXPECT_NEAR(longitude_deg, 120.0, 1e-10);
    EXPECT_NEAR(height_m, 100.0, 1e-6);
}

TEST(RtklibAdapterErrors, MissingAndInvalidNavigationFilesFailClearly) {
    gnss_sim::RtklibNavStore* store = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(store, nullptr);

    std::string error_message;
    EXPECT_FALSE(gnss_sim::load_rinex_nav_file(store, "does-not-exist.rnx", &error_message));
    EXPECT_FALSE(error_message.empty());

    error_message.clear();
    EXPECT_FALSE(gnss_sim::load_rinex_nav_file(store, invalid_nav_path().c_str(), &error_message));
    EXPECT_FALSE(error_message.empty());

    gnss_sim::destroy_rtklib_nav_store(store);
}

bool find_projected_ephemeris(const gnss_sim::RtklibNavStore* store, gnss_sim::NavOutputSystem system, int prn,
                              gnss_sim::RtklibBroadcastMessageFamily family, gnss_sim::NavOutputRecord* result,
                              std::string* error_message) {
    const int count = gnss_sim::rtklib_nav_output_record_count(store);
    for (int index = 0; index < count; ++index) {
        gnss_sim::NavOutputRecord record{};
        if (!gnss_sim::rtklib_nav_output_record(store, index, &record, error_message)) {
            return false;
        }
        if (record.kind == gnss_sim::RtklibNavRecordKind::kEphemeris && record.ephemeris.system == system &&
            record.ephemeris.prn == prn && record.ephemeris.message_family == family) {
            *result = record;
            return true;
        }
    }
    return false;
}

TEST_F(RtklibAdapterTest, SelectedEphemerisIdentityCoversFamiliesAndToeInstances) {
    std::string error_message;
    const std::string companion_path =
        std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_galileo_companion_nav.rnx";
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(store_, companion_path.c_str(), &error_message)) << error_message;

    struct InstanceQuery {
        gnss_sim::NavOutputSystem system;
        int prn;
        gnss_sim::RtklibBroadcastMessageFamily family;
    };
    const InstanceQuery queries[] = {
        {gnss_sim::NavOutputSystem::kGalileo, 2, gnss_sim::RtklibBroadcastMessageFamily::kGalileoInav},
        {gnss_sim::NavOutputSystem::kGalileo, 2, gnss_sim::RtklibBroadcastMessageFamily::kGalileoFnav},
        {gnss_sim::NavOutputSystem::kGalileo, 5, gnss_sim::RtklibBroadcastMessageFamily::kGalileoInav},
        {gnss_sim::NavOutputSystem::kGalileo, 5, gnss_sim::RtklibBroadcastMessageFamily::kGalileoFnav},
    };
    for (const InstanceQuery& query : queries) {
        gnss_sim::NavOutputRecord record{};
        if (!find_projected_ephemeris(store_, query.system, query.prn, query.family, &record, &error_message)) {
            FAIL() << "no projected record for prn " << query.prn << " family " << static_cast<int>(query.family)
                   << ": " << error_message;
        }
        gnss_sim::RtklibSatelliteState state{};
        gnss_sim::RtklibSelectedEphemerisInfo identity{};
        const int toe_week = record.ephemeris.toe_week;
        const double toe_sow = record.ephemeris.toe_sow_sec;
        ASSERT_TRUE(gnss_sim::get_rtklib_signal_satellite_state(store_, toe_week, toe_sow,
                                                                record.ephemeris.satellite_number, 1, query.family,
                                                                &state, &error_message, &identity))
            << error_message;
        EXPECT_EQ(identity.satellite_number, record.ephemeris.satellite_number);
        EXPECT_EQ(identity.message_family, query.family);
        EXPECT_EQ(identity.iode, record.ephemeris.iode);
        EXPECT_EQ(identity.toe_week, toe_week);
        EXPECT_DOUBLE_EQ(identity.toe_sow_sec, toe_sow);
        EXPECT_EQ(identity.iodc, record.ephemeris.iodc);
        EXPECT_EQ(identity.toc_week, record.ephemeris.toc_week);
        EXPECT_DOUBLE_EQ(identity.toc_sow_sec, record.ephemeris.toc_sow_sec);
    }

    // Same satellite + family, different real Toe instance: the selection must return
    // the identity of the instance whose Toe matches the queried epoch, proving Toe is
    // part of the selected identity.
    gnss_sim::NavOutputRecord first{};
    gnss_sim::NavOutputRecord second{};
    ASSERT_TRUE(find_projected_ephemeris(store_, gnss_sim::NavOutputSystem::kGalileo, 2,
                                         gnss_sim::RtklibBroadcastMessageFamily::kGalileoInav, &first, &error_message))
        << error_message;
    const int first_index = static_cast<int>(&first - &first);
    static_cast<void>(first_index);
    int second_found = 0;
    for (int index = 0; index < gnss_sim::rtklib_nav_output_record_count(store_); ++index) {
        gnss_sim::NavOutputRecord record{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(store_, index, &record, &error_message)) << error_message;
        if (record.kind == gnss_sim::RtklibNavRecordKind::kEphemeris &&
            record.ephemeris.system == gnss_sim::NavOutputSystem::kGalileo && record.ephemeris.prn == 2 &&
            record.ephemeris.message_family == gnss_sim::RtklibBroadcastMessageFamily::kGalileoInav &&
            record.ephemeris.iode != first.ephemeris.iode) {
            second = record;
            ++second_found;
        }
    }
    ASSERT_EQ(second_found, 1) << "the fixture must hold two E02 INAV instances with distinct IODnav";
    gnss_sim::RtklibSatelliteState state{};
    gnss_sim::RtklibSelectedEphemerisInfo identity{};
    ASSERT_TRUE(gnss_sim::get_rtklib_signal_satellite_state(
        store_, second.ephemeris.toe_week, second.ephemeris.toe_sow_sec, second.ephemeris.satellite_number, 1,
        gnss_sim::RtklibBroadcastMessageFamily::kGalileoInav, &state, &error_message, &identity))
        << error_message;
    EXPECT_EQ(identity.iode, second.ephemeris.iode);
    EXPECT_EQ(identity.toe_week, second.ephemeris.toe_week);
    EXPECT_DOUBLE_EQ(identity.toe_sow_sec, second.ephemeris.toe_sow_sec);
    EXPECT_FALSE(identity.toe_week == first.ephemeris.toe_week && identity.toe_sow_sec == first.ephemeris.toe_sow_sec)
        << "the two real instances must be distinguishable by selected Toe identity";

    // Family coverage over the five-system fixture: GPS/QZSS legacy, GLONASS FDMA, and
    // BeiDou legacy records must all report their real selected identity.
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(store_, rinex4_acceptance_nav_path().c_str(), &error_message))
        << error_message;
    const InstanceQuery family_queries[] = {
        {gnss_sim::NavOutputSystem::kGps, 1, gnss_sim::RtklibBroadcastMessageFamily::kLegacy},
        {gnss_sim::NavOutputSystem::kQzss, 196, gnss_sim::RtklibBroadcastMessageFamily::kLegacy},
        {gnss_sim::NavOutputSystem::kBeidou, 6, gnss_sim::RtklibBroadcastMessageFamily::kLegacy},
    };
    for (const InstanceQuery& query : family_queries) {
        gnss_sim::NavOutputRecord record{};
        if (!find_projected_ephemeris(store_, query.system, query.prn, query.family, &record, &error_message)) {
            FAIL() << "no projected record for system " << static_cast<int>(query.system) << " prn " << query.prn
                   << ": " << error_message;
        }
        gnss_sim::RtklibSatelliteState state{};
        gnss_sim::RtklibSelectedEphemerisInfo identity{};
        const int observation_code = query.system == gnss_sim::NavOutputSystem::kBeidou ? 40 : 1;
        ASSERT_TRUE(gnss_sim::get_rtklib_signal_satellite_state(
            store_, record.ephemeris.toe_week, record.ephemeris.toe_sow_sec, record.ephemeris.satellite_number,
            observation_code, query.family, &state, &error_message, &identity))
            << "family query failed for system " << static_cast<int>(query.system) << " prn " << query.prn << ": "
            << error_message;
        EXPECT_EQ(identity.iode, record.ephemeris.iode);
        EXPECT_EQ(identity.toe_week, record.ephemeris.toe_week);
        EXPECT_DOUBLE_EQ(identity.toe_sow_sec, record.ephemeris.toe_sow_sec);
    }

    // GLONASS FDMA identity through the GLONASS ephemeris path.
    gnss_sim::NavOutputRecord glonass_record{};
    bool found_glonass = false;
    for (int index = 0; index < gnss_sim::rtklib_nav_output_record_count(store_); ++index) {
        gnss_sim::NavOutputRecord record{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(store_, index, &record, &error_message)) << error_message;
        if (record.kind == gnss_sim::RtklibNavRecordKind::kGlonassEphemeris) {
            glonass_record = record;
            found_glonass = true;
            break;
        }
    }
    ASSERT_TRUE(found_glonass) << "the fixture must contain a GLONASS record";
    gnss_sim::RtklibSatelliteState glonass_state{};
    gnss_sim::RtklibSelectedEphemerisInfo glonass_identity{};
    ASSERT_TRUE(gnss_sim::get_rtklib_signal_satellite_state(
        store_, glonass_record.glonass.toe_week, glonass_record.glonass.toe_sow_sec,
        glonass_record.glonass.satellite_number, 1, gnss_sim::RtklibBroadcastMessageFamily::kGlonassFdma,
        &glonass_state, &error_message, &glonass_identity))
        << error_message;
    EXPECT_EQ(glonass_identity.iode, glonass_record.glonass.iode);
    EXPECT_EQ(glonass_identity.toe_week, glonass_record.glonass.toe_week);
    EXPECT_DOUBLE_EQ(glonass_identity.toe_sow_sec, glonass_record.glonass.toe_sow_sec);
}

} // namespace
