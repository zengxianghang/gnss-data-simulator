#include "gnss/rtklib_adapter.h"
#include "gnss/signal_definitions.h"

#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <iterator>
#include <set>
#include <string>
#include <utility>

namespace {

struct ExpectedSignal {
    gnss_sim::SignalId signal_id;
    gnss_sim::GnssConstellation constellation;
    const char* rinex_code;
    int oem7_type;
};

constexpr ExpectedSignal kExpectedSignals[] = {
    {gnss_sim::SignalId::kGpsL1Ca, gnss_sim::GnssConstellation::kGps, "1C", 0},
    {gnss_sim::SignalId::kGpsL1C, gnss_sim::GnssConstellation::kGps, "1L", 16},
    {gnss_sim::SignalId::kGpsL2P, gnss_sim::GnssConstellation::kGps, "2P", 5},
    {gnss_sim::SignalId::kGpsL2C, gnss_sim::GnssConstellation::kGps, "2S", 17},
    {gnss_sim::SignalId::kGpsL5Q, gnss_sim::GnssConstellation::kGps, "5Q", 14},
    {gnss_sim::SignalId::kQzssL1Ca, gnss_sim::GnssConstellation::kQzss, "1C", 0},
    {gnss_sim::SignalId::kQzssL1C, gnss_sim::GnssConstellation::kQzss, "1L", 16},
    {gnss_sim::SignalId::kQzssL2C, gnss_sim::GnssConstellation::kQzss, "2S", 17},
    {gnss_sim::SignalId::kQzssL5Q, gnss_sim::GnssConstellation::kQzss, "5Q", 14},
    {gnss_sim::SignalId::kGlonassG1, gnss_sim::GnssConstellation::kGlonass, "1C", 0},
    {gnss_sim::SignalId::kGlonassG2, gnss_sim::GnssConstellation::kGlonass, "2C", 1},
    {gnss_sim::SignalId::kGlonassG3, gnss_sim::GnssConstellation::kGlonass, "3Q", 6},
    {gnss_sim::SignalId::kGalileoE1, gnss_sim::GnssConstellation::kGalileo, "1C", 2},
    {gnss_sim::SignalId::kGalileoE5A, gnss_sim::GnssConstellation::kGalileo, "5Q", 12},
    {gnss_sim::SignalId::kGalileoE5B, gnss_sim::GnssConstellation::kGalileo, "7Q", 17},
    {gnss_sim::SignalId::kGalileoE6, gnss_sim::GnssConstellation::kGalileo, "6C", 7},
    {gnss_sim::SignalId::kBeidouB1I, gnss_sim::GnssConstellation::kBeidou, "2I", 0},
    {gnss_sim::SignalId::kBeidouB3I, gnss_sim::GnssConstellation::kBeidou, "6I", 2},
    {gnss_sim::SignalId::kBeidouB1C, gnss_sim::GnssConstellation::kBeidou, "1P", 7},
    {gnss_sim::SignalId::kBeidouB2A, gnss_sim::GnssConstellation::kBeidou, "5P", 9},
    {gnss_sim::SignalId::kBeidouB2B, gnss_sim::GnssConstellation::kBeidou, "7D", 11},
};

TEST(SignalDefinitions, CoversEveryFrozenV1SignalExactlyOnce) {
    std::size_t count = 0;
    const gnss_sim::SignalDefinition* definitions = gnss_sim::signal_definitions(&count);
    ASSERT_NE(definitions, nullptr);
    ASSERT_EQ(count, std::size(kExpectedSignals));

    std::set<int> signal_ids;
    std::set<std::pair<int, std::string>> rinex_keys;
    std::set<std::pair<int, int>> oem7_keys;
    for (std::size_t index = 0; index < count; ++index) {
        const gnss_sim::SignalDefinition& definition = definitions[index];
        EXPECT_TRUE(signal_ids.insert(static_cast<int>(definition.signal_id)).second);
        EXPECT_TRUE(
            rinex_keys.insert({static_cast<int>(definition.constellation), std::string(definition.rinex_signal_code)})
                .second);
        EXPECT_TRUE(
            oem7_keys.insert({static_cast<int>(definition.constellation), definition.novatel_oem7_signal_type}).second);
        EXPECT_NE(definition.name, nullptr);
        EXPECT_NE(definition.rinex_signal_code, nullptr);
    }
    EXPECT_EQ(signal_ids.size(), count);
    EXPECT_EQ(rinex_keys.size(), count);
    EXPECT_EQ(oem7_keys.size(), count);
}

TEST(SignalDefinitions, FrozenMappingsMatchExpectedValuesAndRoundTrip) {
    for (const ExpectedSignal& expected : kExpectedSignals) {
        const gnss_sim::SignalDefinition* definition = gnss_sim::find_signal_definition(expected.signal_id);
        ASSERT_NE(definition, nullptr);
        EXPECT_EQ(definition->constellation, expected.constellation);
        EXPECT_STREQ(definition->rinex_signal_code, expected.rinex_code);
        EXPECT_EQ(definition->novatel_oem7_signal_type, expected.oem7_type);
        EXPECT_EQ(gnss_sim::find_signal_definition_by_rinex(expected.constellation, expected.rinex_code), definition);
        EXPECT_EQ(gnss_sim::find_signal_definition_by_oem7(expected.constellation, expected.oem7_type), definition);

        int rtklib_code = 0;
        int frequency_index = 0;
        EXPECT_TRUE(gnss_sim::signal_rtklib_observation_code(*definition, &rtklib_code, &frequency_index));
        EXPECT_GT(rtklib_code, 0);
        EXPECT_GT(frequency_index, 0);
    }
}

TEST(SignalDefinitions, GalileoE6UsesHasCodeBiasObservable) {
    const auto* e6 = gnss_sim::find_signal_definition(gnss_sim::SignalId::kGalileoE6);
    ASSERT_NE(e6, nullptr);
    EXPECT_STREQ(e6->rinex_signal_code, "6C");
    EXPECT_EQ(e6->novatel_oem7_signal_type, 7);
    EXPECT_EQ(gnss_sim::find_signal_definition_by_rinex(gnss_sim::GnssConstellation::kGalileo, "6B"), nullptr);
    EXPECT_EQ(gnss_sim::find_signal_definition_by_oem7(gnss_sim::GnssConstellation::kGalileo, 6), nullptr);
}

TEST(SignalDefinitions, ModernBeidouCodesResolveThroughPinnedRtklib) {
    const auto* b1c = gnss_sim::find_signal_definition(gnss_sim::SignalId::kBeidouB1C);
    const auto* b2a = gnss_sim::find_signal_definition(gnss_sim::SignalId::kBeidouB2A);
    const auto* b2b = gnss_sim::find_signal_definition(gnss_sim::SignalId::kBeidouB2B);
    ASSERT_NE(b1c, nullptr);
    ASSERT_NE(b2a, nullptr);
    ASSERT_NE(b2b, nullptr);

    int code = 0;
    int frequency = 0;
    ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*b1c, &code, &frequency));
    EXPECT_GT(code, 0);
    EXPECT_EQ(frequency, 1);
    ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*b2a, &code, &frequency));
    EXPECT_EQ(code, 49);
    EXPECT_EQ(frequency, 3);
    ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*b2b, &code, &frequency));
    EXPECT_EQ(code, 50);
    EXPECT_EQ(frequency, 5);
}

TEST(SignalDefinitions, UnknownCombinationsFailExplicitly) {
    EXPECT_EQ(gnss_sim::find_signal_definition_by_rinex(gnss_sim::GnssConstellation::kGps, "9Z"), nullptr);
    EXPECT_EQ(gnss_sim::find_signal_definition_by_rinex(gnss_sim::GnssConstellation::kGps, nullptr), nullptr);
    EXPECT_EQ(gnss_sim::find_signal_definition_by_oem7(gnss_sim::GnssConstellation::kGps, 999), nullptr);

    int code = 0;
    int frequency = 0;
    EXPECT_FALSE(gnss_sim::rtklib_observation_code("9Z", &code, &frequency));
}

TEST(SignalDefinitions, GlonassG1AndG2UseFcnDependentFrequencyAndWavelength) {
    const gnss_sim::SignalDefinition* g1 = gnss_sim::find_signal_definition(gnss_sim::SignalId::kGlonassG1);
    const gnss_sim::SignalDefinition* g2 = gnss_sim::find_signal_definition(gnss_sim::SignalId::kGlonassG2);
    ASSERT_NE(g1, nullptr);
    ASSERT_NE(g2, nullptr);

    double g1_minus7 = 0.0;
    double g1_plus6 = 0.0;
    double g2_minus7 = 0.0;
    double g2_plus6 = 0.0;
    ASSERT_TRUE(gnss_sim::signal_carrier_frequency_hz(*g1, -7, &g1_minus7));
    ASSERT_TRUE(gnss_sim::signal_carrier_frequency_hz(*g1, 6, &g1_plus6));
    ASSERT_TRUE(gnss_sim::signal_carrier_frequency_hz(*g2, -7, &g2_minus7));
    ASSERT_TRUE(gnss_sim::signal_carrier_frequency_hz(*g2, 6, &g2_plus6));

    EXPECT_NEAR(g1_minus7, 1598.0625e6, 1e-3);
    EXPECT_NEAR(g1_plus6, 1605.375e6, 1e-3);
    EXPECT_NEAR(g2_minus7, 1242.9375e6, 1e-3);
    EXPECT_NEAR(g2_plus6, 1248.625e6, 1e-3);
    EXPECT_FALSE(gnss_sim::signal_carrier_frequency_hz(*g1, -8, &g1_minus7));
    EXPECT_FALSE(gnss_sim::signal_carrier_frequency_hz(*g2, 7, &g2_plus6));

    double wavelength_minus7 = 0.0;
    double wavelength_plus6 = 0.0;
    ASSERT_TRUE(gnss_sim::signal_wavelength_m(*g1, -7, &wavelength_minus7));
    ASSERT_TRUE(gnss_sim::signal_wavelength_m(*g1, 6, &wavelength_plus6));
    EXPECT_NE(wavelength_minus7, wavelength_plus6);
}

TEST(SignalDefinitions, GlonassG3UsesFixedL3OcCarrier) {
    const gnss_sim::SignalDefinition* g3 = gnss_sim::find_signal_definition(gnss_sim::SignalId::kGlonassG3);
    ASSERT_NE(g3, nullptr);

    double frequency_minus7 = 0.0;
    double frequency_plus6 = 0.0;
    ASSERT_TRUE(gnss_sim::signal_carrier_frequency_hz(*g3, -7, &frequency_minus7));
    ASSERT_TRUE(gnss_sim::signal_carrier_frequency_hz(*g3, 6, &frequency_plus6));
    EXPECT_DOUBLE_EQ(frequency_minus7, 1202.025e6);
    EXPECT_DOUBLE_EQ(frequency_plus6, 1202.025e6);
}

TEST(SignalDefinitions, WavelengthMatchesSpeedOfLightOverCarrier) {
    const gnss_sim::SignalDefinition* gps_l1 = gnss_sim::find_signal_definition(gnss_sim::SignalId::kGpsL1Ca);
    ASSERT_NE(gps_l1, nullptr);

    double wavelength_m = 0.0;
    ASSERT_TRUE(gnss_sim::signal_wavelength_m(*gps_l1, 0, &wavelength_m));
    EXPECT_NEAR(wavelength_m, 299792458.0 / 1575.42e6, 1e-15);
}

} // namespace