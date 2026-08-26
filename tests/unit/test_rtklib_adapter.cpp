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
    EXPECT_STREQ(gnss_sim::rtklib_commit_sha(), "07e813b72c8667350c4e80293cb6679c519ef1a6");
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

} // namespace
