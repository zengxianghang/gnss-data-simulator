#include "gnss/rtklib_adapter.h"
#include "gnss_sim/sim_time.h"
#include "model/atmosphere_model.h"

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <string>

extern "C" {
#include <rtklib.h>
}

#ifdef lock
#undef lock
#endif
#ifdef unlock
#undef unlock
#endif
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

namespace {

constexpr double kPi = 3.1415926535897932;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kGpsL1Hz = 1575.42e6;
constexpr double kGpsL5Hz = 1176.45e6;

std::string ionosphere_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/ionosphere_nav_2019.rnx";
}

std::string mixed_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/mixed_nav_2019.rnx";
}

std::string brd4_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

double normalized_gpst_sow(int gps_week, double sow_sec) {
    const gtime_t time = gpst2time(gps_week, sow_sec);
    int normalized_week = 0;
    const double normalized_sow = time2gpst(time, &normalized_week);
    EXPECT_EQ(normalized_week, gps_week);
    return normalized_sow;
}

double reference_klobuchar(double sow_sec, const double ion[8], double latitude_rad, double longitude_rad,
                           double azimuth_rad, double elevation_rad) {
    const double psi = 0.0137 / (elevation_rad / kPi + 0.11) - 0.022;
    double phi = latitude_rad / kPi + psi * std::cos(azimuth_rad);
    phi = std::clamp(phi, -0.416, 0.416);
    const double lambda = longitude_rad / kPi + psi * std::sin(azimuth_rad) / std::cos(phi * kPi);
    phi += 0.064 * std::cos((lambda - 1.617) * kPi);
    double local_time = 43200.0 * lambda + sow_sec;
    local_time -= std::floor(local_time / 86400.0) * 86400.0;
    const double mapping = 1.0 + 16.0 * std::pow(0.53 - elevation_rad / kPi, 3.0);
    double amplitude = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
    double period = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
    amplitude = std::max(amplitude, 0.0);
    period = std::max(period, 72000.0);
    const double phase = 2.0 * kPi * (local_time - 50400.0) / period;
    const double base =
        std::abs(phase) < 1.57 ? 5.0e-9 + amplitude * (1.0 + phase * phase * (-0.5 + phase * phase / 24.0)) : 5.0e-9;
    return kSpeedOfLightMps * mapping * base;
}

double reference_troposphere(double latitude_rad, double height_m, double elevation_rad) {
    if (height_m < -100.0 || height_m > 10000.0 || elevation_rad <= 0.0) {
        return 0.0;
    }
    const double height = std::max(height_m, 0.0);
    const double pressure = 1013.25 * std::pow(1.0 - 2.2557e-5 * height, 5.2568);
    const double temperature = 15.0 - 6.5e-3 * height + 273.16;
    const double vapor_pressure = 6.108 * 0.7 * std::exp((17.15 * temperature - 4684.0) / (temperature - 38.45));
    const double cosine_zenith = std::sin(elevation_rad);
    const double dry = 0.0022768 * pressure /
                       (1.0 - 0.00266 * std::cos(2.0 * latitude_rad) - 0.00028 * height / 1.0e3) / cosine_zenith;
    const double wet = 0.002277 * (1255.0 / temperature + 0.05) * vapor_pressure / cosine_zenith;
    return dry + wet;
}

bool direct_rtklib_broadcast_ionosphere(const std::string& path, int gps_week, double sow_sec,
                                        const double receiver_ecef_m[3], double azimuth_rad, double elevation_rad,
                                        double* delay_m, double* ion_gps_norm, int* ion_record_count) {
    obs_t obs{};
    nav_t nav{};
    sta_t station{};
    if (readrnx(path.c_str(), 1, "", &obs, &nav, &station) == 0) {
        freeobs(&obs);
        freenav(&nav, 0xFF);
        return false;
    }
    freeobs(&obs);
    uniqnav(&nav);

    double position_rad_m[3]{};
    ecef2pos(receiver_ecef_m, position_rad_m);
    const double azel[2] = {azimuth_rad, elevation_rad};
    double variance_m2 = 0.0;
    const int satellite = satno(SYS_GPS, 1);
    const bool ok = satellite > 0 &&
                    ionocorr(gpst2time(gps_week, sow_sec), &nav, satellite, position_rad_m, azel, IONOOPT_BRDC,
                             delay_m, &variance_m2) != 0;
    if (ion_gps_norm != nullptr) {
        *ion_gps_norm = norm(nav.ion_gps, 8);
    }
    if (ion_record_count != nullptr) {
        *ion_record_count = nav.nion;
    }
    freenav(&nav, 0xFF);
    return ok;
}

struct NavGuard {
    gnss_sim::RtklibNavStore* store;
    ~NavGuard() {
        gnss_sim::destroy_rtklib_nav_store(store);
    }
};

TEST(AtmosphereNone, ReturnsExactZeroWithoutNavigationStore) {
    gnss_sim::SimTime time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 200000.0, &time));
    double receiver_ecef[3]{};
    ASSERT_TRUE(gnss_sim::rtklib_llh_to_ecef(20.0, 120.0, 100.0, receiver_ecef));
    gnss_sim::AtmosphereCorrection correction{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::NONE, nullptr, time,
                                                        gnss_sim::SignalId::kGpsL1Ca, 0, receiver_ecef, 1.0, 0.7,
                                                        &correction, &error_message));
    EXPECT_EQ(correction.ionosphere_status, gnss_sim::IonosphereCorrectionStatus::kDisabled);
    EXPECT_DOUBLE_EQ(correction.ionosphere_code_delay_m, 0.0);
    EXPECT_DOUBLE_EQ(correction.troposphere_delay_m, 0.0);
}

TEST(AtmosphereBroadcast, GpsMatchesReferenceKlobucharAndRtklibTroposphere) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, ionosphere_nav_path().c_str(), &error_message))
        << error_message;

    gnss_sim::SimTime time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 200000.0, &time));
    double receiver_ecef[3]{};
    ASSERT_TRUE(gnss_sim::rtklib_llh_to_ecef(20.0, 120.0, 100.0, receiver_ecef));
    double reference_latitude_deg = 0.0;
    double reference_longitude_deg = 0.0;
    double reference_height_m = 0.0;
    ASSERT_TRUE(gnss_sim::rtklib_ecef_to_llh(receiver_ecef, &reference_latitude_deg, &reference_longitude_deg,
                                             &reference_height_m));
    const double azimuth = 1.2;
    const double elevation = 0.7;
    gnss_sim::AtmosphereCorrection correction{};
    ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::BROADCAST, nav.store, time,
                                                        gnss_sim::SignalId::kGpsL1Ca, 0, receiver_ecef, azimuth,
                                                        elevation, &correction, &error_message));
    const double gps_ion[8] = {1.1176e-8, 2.2352e-8, -1.1921e-7, -1.1921e-7, 90112.0, 98304.0, -65536.0, -589820.0};
    const double expected_ion =
        reference_klobuchar(normalized_gpst_sow(2041, 200000.0), gps_ion, reference_latitude_deg * kPi / 180.0,
                            reference_longitude_deg * kPi / 180.0, azimuth, elevation);
    const double expected_trop =
        reference_troposphere(reference_latitude_deg * kPi / 180.0, reference_height_m, elevation);
    EXPECT_EQ(correction.ionosphere_status, gnss_sim::IonosphereCorrectionStatus::kApplied);
    EXPECT_NEAR(correction.ionosphere_code_delay_m, expected_ion, 1.0e-9);
    EXPECT_NEAR(correction.troposphere_delay_m, expected_trop, 1.0e-9);
}

TEST(AtmosphereBroadcast, MultiFrequencyIonosphereScalesWithInverseFrequencySquared) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, ionosphere_nav_path().c_str(), &error_message));
    gnss_sim::SimTime time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 210000.0, &time));
    double receiver_ecef[3]{};
    ASSERT_TRUE(gnss_sim::rtklib_llh_to_ecef(20.0, 120.0, 100.0, receiver_ecef));

    gnss_sim::AtmosphereCorrection l1{};
    gnss_sim::AtmosphereCorrection l5{};
    ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::BROADCAST, nav.store, time,
                                                        gnss_sim::SignalId::kGpsL1Ca, 0, receiver_ecef, 0.5, 0.8, &l1,
                                                        &error_message));
    ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::BROADCAST, nav.store, time,
                                                        gnss_sim::SignalId::kGpsL5Q, 0, receiver_ecef, 0.5, 0.8, &l5,
                                                        &error_message));
    const double expected_ratio = (kGpsL1Hz / kGpsL5Hz) * (kGpsL1Hz / kGpsL5Hz);
    EXPECT_NEAR(l5.ionosphere_code_delay_m / l1.ionosphere_code_delay_m, expected_ratio, 1.0e-12);
    EXPECT_DOUBLE_EQ(l5.troposphere_delay_m, l1.troposphere_delay_m);
}

TEST(AtmosphereBroadcast, Rinex4ZeroLegacyCoefficientsRetainPinnedRtklibFallback) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, brd4_nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::SimTime time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 436500.0, &time));
    double receiver_ecef[3]{};
    ASSERT_TRUE(gnss_sim::rtklib_llh_to_ecef(20.0, 120.0, 100.0, receiver_ecef));
    const double azimuth = 1.0;
    const double elevation = 0.7;

    double direct_delay_m = 0.0;
    double ion_gps_norm = -1.0;
    int ion_record_count = 0;
    ASSERT_TRUE(direct_rtklib_broadcast_ionosphere(brd4_nav_path(), time.gps_week, gnss_sim::sim_time_sow_sec(time),
                                                   receiver_ecef, azimuth, elevation, &direct_delay_m, &ion_gps_norm,
                                                   &ion_record_count));
    EXPECT_DOUBLE_EQ(ion_gps_norm, 0.0);
    EXPECT_GT(ion_record_count, 0);
    EXPECT_GT(direct_delay_m, 0.0);

    gnss_sim::AtmosphereCorrection correction{};
    ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::BROADCAST, nav.store, time,
                                                        gnss_sim::SignalId::kGpsL1Ca, 0, receiver_ecef, azimuth,
                                                        elevation, &correction, &error_message));
    EXPECT_EQ(correction.ionosphere_status, gnss_sim::IonosphereCorrectionStatus::kApplied);
    EXPECT_NEAR(correction.ionosphere_code_delay_m, direct_delay_m, 1.0e-12);
}

TEST(AtmosphereBroadcast, AllV1ConstellationsSharePinnedRtklibBaseAndUseSignalFrequencyScaling) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, brd4_nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::SimTime time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 436500.0, &time));
    double receiver_ecef[3]{};
    ASSERT_TRUE(gnss_sim::rtklib_llh_to_ecef(20.0, 120.0, 100.0, receiver_ecef));
    const double azimuth = 0.9;
    const double elevation = 0.65;

    double base_delay_m = 0.0;
    ASSERT_TRUE(direct_rtklib_broadcast_ionosphere(brd4_nav_path(), time.gps_week, gnss_sim::sim_time_sow_sec(time),
                                                   receiver_ecef, azimuth, elevation, &base_delay_m, nullptr,
                                                   nullptr));

    struct TestCase {
        gnss_sim::SignalId signal_id;
        int glonass_fcn;
    };
    const TestCase cases[] = {
        {gnss_sim::SignalId::kGpsL1Ca, 0},       {gnss_sim::SignalId::kQzssL1Ca, 0},
        {gnss_sim::SignalId::kGlonassG1, -7},   {gnss_sim::SignalId::kGlonassG1, 6},
        {gnss_sim::SignalId::kGalileoE1, 0},     {gnss_sim::SignalId::kBeidouB1I, 0},
        {gnss_sim::SignalId::kBeidouB1C, 0},     {gnss_sim::SignalId::kBeidouB2A, 0},
        {gnss_sim::SignalId::kBeidouB2B, 0},
    };

    for (const TestCase& test : cases) {
        const gnss_sim::SignalDefinition* definition = gnss_sim::find_signal_definition(test.signal_id);
        ASSERT_NE(definition, nullptr);
        double signal_frequency_hz = 0.0;
        ASSERT_TRUE(gnss_sim::signal_carrier_frequency_hz(*definition, test.glonass_fcn, &signal_frequency_hz));
        const double expected = base_delay_m * (kGpsL1Hz / signal_frequency_hz) * (kGpsL1Hz / signal_frequency_hz);

        gnss_sim::AtmosphereCorrection correction{};
        ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::BROADCAST, nav.store, time,
                                                            test.signal_id, test.glonass_fcn, receiver_ecef, azimuth,
                                                            elevation, &correction, &error_message));
        EXPECT_EQ(correction.ionosphere_status, gnss_sim::IonosphereCorrectionStatus::kApplied);
        EXPECT_NEAR(correction.ionosphere_code_delay_m, expected, 1.0e-10);
        EXPECT_GT(correction.troposphere_delay_m, 0.0);
    }
}

TEST(AtmosphereBroadcast, ZeroLegacyHeaderStillMatchesRtklibOnRinex3Input) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, mixed_nav_path().c_str(), &error_message));
    gnss_sim::SimTime time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 200000.0, &time));
    double receiver_ecef[3]{};
    ASSERT_TRUE(gnss_sim::rtklib_llh_to_ecef(20.0, 120.0, 100.0, receiver_ecef));

    double direct_delay_m = 0.0;
    ASSERT_TRUE(direct_rtklib_broadcast_ionosphere(mixed_nav_path(), time.gps_week, gnss_sim::sim_time_sow_sec(time),
                                                   receiver_ecef, 1.0, 0.7, &direct_delay_m, nullptr, nullptr));
    gnss_sim::AtmosphereCorrection correction{};
    ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::BROADCAST, nav.store, time,
                                                        gnss_sim::SignalId::kGpsL1Ca, 0, receiver_ecef, 1.0, 0.7,
                                                        &correction, &error_message));
    EXPECT_EQ(correction.ionosphere_status, gnss_sim::IonosphereCorrectionStatus::kApplied);
    EXPECT_NEAR(correction.ionosphere_code_delay_m, direct_delay_m, 1.0e-12);
}

} // namespace
