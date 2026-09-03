#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_config.h"
#include "model/urban_rf_model.h"

#include <cmath>
#include <complex>
#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <string>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr const char* kConfigPath = "gnss_sim_urban_rf_test.json";

struct ConfigFileGuard {
    ~ConfigFileGuard() {
        static_cast<void>(std::remove(kConfigPath));
    }
};

void write_config(const char* text) {
    std::ofstream output(kConfigPath, std::ios::binary | std::ios::trunc);
    output << text;
}

double complex_distance(const std::complex<double>& left, const std::complex<double>& right) {
    return std::abs(left - right);
}

TEST(UrbanRfModel, FrozenDefaultPresetMatchesIssue116) {
    const gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    EXPECT_DOUBLE_EQ(config.urban_rf.default_material.relative_permittivity_real, 6.27);
    EXPECT_DOUBLE_EQ(config.urban_rf.default_material.conductivity_c_s_per_m, 0.0043);
    EXPECT_DOUBLE_EQ(config.urban_rf.default_material.conductivity_exponent, 1.1925);
    EXPECT_DOUBLE_EQ(config.urban_rf.default_material.outer_glass_thickness_m, 0.006);
    EXPECT_DOUBLE_EQ(config.urban_rf.default_material.cavity_thickness_m, 0.012);
    EXPECT_DOUBLE_EQ(config.urban_rf.default_material.inner_glass_thickness_m, 0.006);
    EXPECT_DOUBLE_EQ(config.urban_rf.default_material.coating_sheet_resistance_ohm_sq, 5.0);
    EXPECT_TRUE(config.urban_rf.signal_overrides.empty());
    EXPECT_DOUBLE_EQ(config.urban_rf.default_antenna.rhcp.gain_db_horizon, 0.0);
    EXPECT_DOUBLE_EQ(config.urban_rf.default_antenna.lhcp.gain_db_zenith, 0.0);
}

TEST(UrbanRfModel, NormalIncidenceHasEqualTangentialTeTmAndReversesHandedness) {
    const gnss_sim::UrbanRfMaterialConfig material = gnss_sim::default_sim_config().urban_rf.default_material;
    gnss_sim::UrbanRfReflectionResponse response{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_low_e_curtain_wall_reflection(material, 1575.42e6, 0.0, &response, &error_message))
        << error_message;
    EXPECT_LT(complex_distance(response.gamma_te_tangent, response.gamma_tm_tangent), 1.0e-12);
    EXPECT_LT(std::abs(response.gamma_rhcp_from_rhcp), 1.0e-12);
    EXPECT_GT(std::abs(response.gamma_lhcp_from_rhcp), 0.90);
    EXPECT_LT(std::abs(response.gamma_lhcp_from_rhcp), 1.0 + 1.0e-12);
    EXPECT_LT(response.glass_relative_permittivity.imag(), 0.0);
    EXPECT_GT(response.glass_conductivity_s_per_m, 0.0);
}

TEST(UrbanRfModel, FrequencyAngleThicknessAndSheetResistanceChangeComplexResponse) {
    const gnss_sim::UrbanRfMaterialConfig baseline = gnss_sim::default_sim_config().urban_rf.default_material;
    std::string error_message;
    gnss_sim::UrbanRfReflectionResponse l1_normal{};
    gnss_sim::UrbanRfReflectionResponse l5_normal{};
    gnss_sim::UrbanRfReflectionResponse l1_grazing{};
    ASSERT_TRUE(gnss_sim::compute_low_e_curtain_wall_reflection(baseline, 1575.42e6, 0.0, &l1_normal, &error_message));
    ASSERT_TRUE(gnss_sim::compute_low_e_curtain_wall_reflection(baseline, 1176.45e6, 0.0, &l5_normal, &error_message));
    ASSERT_TRUE(gnss_sim::compute_low_e_curtain_wall_reflection(baseline, 1575.42e6, 85.0 * kDegreesToRadians,
                                                                &l1_grazing, &error_message));
    EXPECT_GT(complex_distance(l1_normal.gamma_te_tangent, l5_normal.gamma_te_tangent), 1.0e-3);
    EXPECT_GT(std::abs(l1_grazing.gamma_te_tangent), std::abs(l1_normal.gamma_te_tangent));
    EXPECT_GT(std::fabs(std::arg(l1_normal.gamma_te_tangent)), 0.1);

    gnss_sim::UrbanRfMaterialConfig thicker = baseline;
    thicker.outer_glass_thickness_m = 0.008;
    gnss_sim::UrbanRfReflectionResponse thick_response{};
    ASSERT_TRUE(gnss_sim::compute_low_e_curtain_wall_reflection(thicker, 1575.42e6, 30.0 * kDegreesToRadians,
                                                                &thick_response, &error_message));
    gnss_sim::UrbanRfReflectionResponse baseline_oblique{};
    ASSERT_TRUE(gnss_sim::compute_low_e_curtain_wall_reflection(baseline, 1575.42e6, 30.0 * kDegreesToRadians,
                                                                &baseline_oblique, &error_message));
    EXPECT_GT(complex_distance(thick_response.gamma_te_tangent, baseline_oblique.gamma_te_tangent), 1.0e-4);

    gnss_sim::UrbanRfMaterialConfig different_sheet = baseline;
    different_sheet.coating_sheet_resistance_ohm_sq = 12.0;
    gnss_sim::UrbanRfReflectionResponse sheet_response{};
    ASSERT_TRUE(gnss_sim::compute_low_e_curtain_wall_reflection(different_sheet, 1575.42e6, 30.0 * kDegreesToRadians,
                                                                &sheet_response, &error_message));
    EXPECT_GT(complex_distance(sheet_response.gamma_te_tangent, baseline_oblique.gamma_te_tangent), 1.0e-3);
}

TEST(UrbanRfModel, AntennaInterpolatesComplexVoltageResponse) {
    gnss_sim::UrbanRfAntennaConfig antenna{};
    antenna.rhcp = {-20.0, 0.0, 0.0, 90.0};
    antenna.lhcp = {-30.0, -10.0, -90.0, 0.0};
    std::complex<double> response{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::evaluate_urban_antenna_response(antenna, gnss_sim::UrbanCircularPolarization::kRhcp,
                                                          45.0 * kDegreesToRadians, &response, &error_message))
        << error_message;
    EXPECT_NEAR(std::abs(response), std::pow(10.0, -10.0 / 20.0), 1.0e-12);
    EXPECT_NEAR(std::arg(response), 45.0 * kDegreesToRadians, 1.0e-12);
}

TEST(UrbanRfModel, JsonPerSignalOverrideStartsFromDefaultAndOnlyChangesNamedSignal) {
    ConfigFileGuard guard;
    write_config(R"json({
        "urban_rf": {
            "material": {"coating_sheet_resistance_ohm_sq": 6.0},
            "antenna": {"rhcp": {"gain_db_zenith": 1.0}},
            "signals": {
                "GPS L1 C/A": {
                    "material": {"coating_sheet_resistance_ohm_sq": 9.0},
                    "antenna": {"rhcp": {"gain_db_zenith": 3.0}}
                }
            }
        }
    })json");
    gnss_sim::SimConfig config{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_sim_config_json(kConfigPath, &config, &error_message)) << error_message;
    EXPECT_DOUBLE_EQ(config.urban_rf.default_material.coating_sheet_resistance_ohm_sq, 6.0);
    EXPECT_DOUBLE_EQ(config.urban_rf.default_antenna.rhcp.gain_db_zenith, 1.0);
    ASSERT_EQ(config.urban_rf.signal_overrides.size(), 1U);

    const gnss_sim::SignalDefinition* gps = gnss_sim::find_signal_definition(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::SignalDefinition* gal = gnss_sim::find_signal_definition(gnss_sim::SignalId::kGalileoE1);
    ASSERT_NE(gps, nullptr);
    ASSERT_NE(gal, nullptr);
    gnss_sim::UrbanRfResolvedConfig gps_config{};
    gnss_sim::UrbanRfResolvedConfig gal_config{};
    ASSERT_TRUE(gnss_sim::resolve_urban_rf_signal_config(config.urban_rf, *gps, &gps_config, &error_message));
    ASSERT_TRUE(gnss_sim::resolve_urban_rf_signal_config(config.urban_rf, *gal, &gal_config, &error_message));
    EXPECT_DOUBLE_EQ(gps_config.material.coating_sheet_resistance_ohm_sq, 9.0);
    EXPECT_DOUBLE_EQ(gps_config.antenna.rhcp.gain_db_zenith, 3.0);
    EXPECT_DOUBLE_EQ(gal_config.material.coating_sheet_resistance_ohm_sq, 6.0);
    EXPECT_DOUBLE_EQ(gal_config.antenna.rhcp.gain_db_zenith, 1.0);
}

TEST(UrbanRfModel, RejectsUnknownSignalAndNonphysicalParameters) {
    ConfigFileGuard guard;
    const char* cases[] = {
        R"json({"urban_rf":{"material":{"coating_sheet_resistance_ohm_sq":0}}})json",
        R"json({"urban_rf":{"material":{"outer_glass_thickness_m":-0.01}}})json",
        R"json({"urban_rf":{"signals":{"GPS L9":{"material":{"coating_sheet_resistance_ohm_sq":5}}}}})json",
    };
    for (const char* text : cases) {
        write_config(text);
        gnss_sim::SimConfig config{};
        std::string error_message;
        EXPECT_FALSE(gnss_sim::load_sim_config_json(kConfigPath, &config, &error_message));
        EXPECT_NE(error_message.find("urban_rf"), std::string::npos) << error_message;
    }

    gnss_sim::UrbanRfReflectionResponse response{};
    std::string error_message;
    const gnss_sim::UrbanRfMaterialConfig material = gnss_sim::default_sim_config().urban_rf.default_material;
    EXPECT_FALSE(gnss_sim::compute_low_e_curtain_wall_reflection(material, 0.0, 0.0, &response, &error_message));
    EXPECT_FALSE(
        gnss_sim::compute_low_e_curtain_wall_reflection(material, 1575.42e6, 0.5 * kPi, &response, &error_message));
}

TEST(UrbanRfModel, SameInputIsNumericallyDeterministic) {
    const gnss_sim::UrbanRfMaterialConfig material = gnss_sim::default_sim_config().urban_rf.default_material;
    gnss_sim::UrbanRfReflectionResponse first{};
    gnss_sim::UrbanRfReflectionResponse second{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_low_e_curtain_wall_reflection(material, 1575.42e6, 37.0 * kDegreesToRadians, &first,
                                                                &error_message));
    ASSERT_TRUE(gnss_sim::compute_low_e_curtain_wall_reflection(material, 1575.42e6, 37.0 * kDegreesToRadians, &second,
                                                                &error_message));
    EXPECT_EQ(first.glass_conductivity_s_per_m, second.glass_conductivity_s_per_m);
    EXPECT_EQ(first.glass_relative_permittivity, second.glass_relative_permittivity);
    EXPECT_EQ(first.gamma_te_tangent, second.gamma_te_tangent);
    EXPECT_EQ(first.gamma_tm_tangent, second.gamma_tm_tangent);
    EXPECT_EQ(first.gamma_rhcp_from_rhcp, second.gamma_rhcp_from_rhcp);
    EXPECT_EQ(first.gamma_lhcp_from_rhcp, second.gamma_lhcp_from_rhcp);
}

} // namespace
