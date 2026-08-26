#include "gnss/rtklib_adapter.h"
#include "gnss_sim/sim_time.h"
#include "model/atmosphere_model.h"

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kGpsL1Hz = 1575.42e6;
constexpr double kGpsL5Hz = 1176.45e6;
constexpr double kBdsB1IHz = 1561.098e6;
constexpr double kBdsB3IHz = 1268.52e6;

std::string ionosphere_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/ionosphere_nav_2019.rnx";
}

std::string mixed_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/mixed_nav_2019.rnx";
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
    const double base = std::abs(phase) < 1.57
                            ? 5.0e-9 + amplitude * (1.0 + phase * phase * (-0.5 + phase * phase / 24.0))
                            : 5.0e-9;
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
    ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::kNone, nullptr, time,
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
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, ionosphere_nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::SimTime time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 200000.0, &time));
    double receiver_ecef[3]{};
    ASSERT_TRUE(gnss_sim::rtklib_llh_to_ecef(20.0, 120.0, 100.0, receiver_ecef));
    const double azimuth = 1.2;
    const double elevation = 0.7;
    gnss_sim::AtmosphereCorrection correction{};
    ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::kBroadcast, nav.store, time,
                                                        gnss_sim::SignalId::kGpsL1Ca, 0, receiver_ecef, azimuth,
                                                        elevation, &correction, &error_message));
    const double gps_ion[8] = {1.1176e-8, 2.2352e-8, -1.1921e-7, -1.1921e-7,
                               90112.0, 98304.0, -65536.0, -589824.0};
    const double expected_ion = reference_klobuchar(200000.0, gps_ion, 20.0 * kPi / 180.0, 120.0 * kPi / 180.0,
                                                    azimuth, elevation);
    const double expected_trop = reference_troposphere(20.0 * kPi / 180.0, 100.0, elevation);
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
    ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::kBroadcast, nav.store, time,
                                                        gnss_sim::SignalId::kGpsL1Ca, 0, receiver_ecef, 0.5, 0.8, &l1,
                                                        &error_message));
    ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::kBroadcast, nav.store, time,
                                                        gnss_sim::SignalId::kGpsL5Q, 0, receiver_ecef, 0.5, 0.8, &l5,
                                                        &error_message));
    const double expected_ratio = (kGpsL1Hz / kGpsL5Hz) * (kGpsL1Hz / kGpsL5Hz);
    EXPECT_NEAR(l5.ionosphere_code_delay_m / l1.ionosphere_code_delay_m, expected_ratio, 1.0e-12);
    EXPECT_DOUBLE_EQ(l5.troposphere_delay_m, l1.troposphere_delay_m);
}

TEST(AtmosphereBroadcast, BeidouLegacyUsesBdtPhaseAndB1IReferenceFrequency) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, ionosphere_nav_path().c_str(), &error_message));
    gnss_sim::SimTime time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 200014.0, &time));
    double receiver_ecef[3]{};
    ASSERT_TRUE(gnss_sim::rtklib_llh_to_ecef(20.0, 120.0, 100.0, receiver_ecef));
    const double azimuth = 0.9;
    const double elevation = 0.65;

    gnss_sim::AtmosphereCorrection b1i{};
    gnss_sim::AtmosphereCorrection b3i{};
    ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::kBroadcast, nav.store, time,
                                                        gnss_sim::SignalId::kBeidouB1I, 0, receiver_ecef, azimuth,
                                                        elevation, &b1i, &error_message));
    ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::kBroadcast, nav.store, time,
                                                        gnss_sim::SignalId::kBeidouB3I, 0, receiver_ecef, azimuth,
                                                        elevation, &b3i, &error_message));
    const double bds_ion[8] = {2.0e-8, -1.0e-8, 3.0e-8, -2.0e-8, 70000.0, 80000.0, -50000.0, -400000.0};
    const double expected_b1i = reference_klobuchar(200000.0, bds_ion, 20.0 * kPi / 180.0,
                                                   120.0 * kPi / 180.0, azimuth, elevation);
    EXPECT_EQ(b1i.ionosphere_status, gnss_sim::IonosphereCorrectionStatus::kApplied);
    EXPECT_NEAR(b1i.ionosphere_code_delay_m, expected_b1i, 1.0e-9);
    const double expected_ratio = (kBdsB1IHz / kBdsB3IHz) * (kBdsB1IHz / kBdsB3IHz);
    EXPECT_NEAR(b3i.ionosphere_code_delay_m / b1i.ionosphere_code_delay_m, expected_ratio, 1.0e-12);
}

TEST(AtmosphereBroadcast, MissingAndUnsupportedModelsAreExplicit) {
    NavGuard no_ion_nav{gnss_sim::create_rtklib_nav_store()};
    NavGuard ion_nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(no_ion_nav.store, nullptr);
    ASSERT_NE(ion_nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(no_ion_nav.store, mixed_nav_path().c_str(), &error_message));
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(ion_nav.store, ionosphere_nav_path().c_str(), &error_message));
    gnss_sim::SimTime time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 200000.0, &time));
    double receiver_ecef[3]{};
    ASSERT_TRUE(gnss_sim::rtklib_llh_to_ecef(20.0, 120.0, 100.0, receiver_ecef));

    gnss_sim::AtmosphereCorrection missing{};
    ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::kBroadcast, no_ion_nav.store, time,
                                                        gnss_sim::SignalId::kGpsL1Ca, 0, receiver_ecef, 1.0, 0.7,
                                                        &missing, &error_message));
    EXPECT_EQ(missing.ionosphere_status, gnss_sim::IonosphereCorrectionStatus::kMissingParameters);
    EXPECT_DOUBLE_EQ(missing.ionosphere_code_delay_m, 0.0);
    EXPECT_GT(missing.troposphere_delay_m, 0.0);

    const gnss_sim::SignalId unsupported_signals[] = {gnss_sim::SignalId::kGalileoE1,
                                                       gnss_sim::SignalId::kGlonassG1,
                                                       gnss_sim::SignalId::kBeidouB1C};
    for (gnss_sim::SignalId signal_id : unsupported_signals) {
        gnss_sim::AtmosphereCorrection unsupported{};
        ASSERT_TRUE(gnss_sim::compute_atmosphere_correction(gnss_sim::AtmosphereMode::kBroadcast, ion_nav.store, time,
                                                            signal_id, 0, receiver_ecef, 1.0, 0.7, &unsupported,
                                                            &error_message));
        EXPECT_EQ(unsupported.ionosphere_status,
                  gnss_sim::IonosphereCorrectionStatus::kUnsupportedBroadcastModel);
        EXPECT_DOUBLE_EQ(unsupported.ionosphere_code_delay_m, 0.0);
        EXPECT_GT(unsupported.troposphere_delay_m, 0.0);
    }
}

} // namespace
