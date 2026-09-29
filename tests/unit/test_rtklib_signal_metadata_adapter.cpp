#include "gnss/rtklib_adapter.h"
#include "gnss/rtklib_signal_metadata_adapter.h"
#include "gnss/signal_definitions.h"

#include <gtest/gtest.h>

namespace {

struct SignalMetadataCase {
    gnss_sim::RtklibSignalSystem system;
    gnss_sim::GnssConstellation constellation;
    int prn;
    const char* rinex_code;
    int glonass_fcn;
};

TEST(RtklibSignalMetadataAdapter, MatchesExistingMetadataPathsForRepresentativeSystems) {
    const SignalMetadataCase cases[] = {
        {gnss_sim::RtklibSignalSystem::kGps, gnss_sim::GnssConstellation::kGps, 1, "1C", 99},
        {gnss_sim::RtklibSignalSystem::kGlonass, gnss_sim::GnssConstellation::kGlonass, 1, "1C", -4},
        {gnss_sim::RtklibSignalSystem::kGalileo, gnss_sim::GnssConstellation::kGalileo, 1, "5Q", 99},
        {gnss_sim::RtklibSignalSystem::kBeidou, gnss_sim::GnssConstellation::kBeidou, 6, "2I", 99},
        // RTKLIB represents the first QZSS satellite with PRN 193 (RINEX J01).
        {gnss_sim::RtklibSignalSystem::kQzss, gnss_sim::GnssConstellation::kQzss, 193, "1C", 99},
    };

    for (const SignalMetadataCase& test_case : cases) {
        const gnss_sim::SignalDefinition* definition =
            gnss_sim::find_signal_definition_by_rinex(test_case.constellation, test_case.rinex_code);
        ASSERT_NE(definition, nullptr) << test_case.rinex_code;

        int legacy_code = 0;
        int legacy_frequency_index = 0;
        ASSERT_TRUE(gnss_sim::rtklib_observation_code(test_case.rinex_code, &legacy_code, &legacy_frequency_index));

        double legacy_frequency_hz = 0.0;
        double legacy_wavelength_m = 0.0;
        ASSERT_TRUE(gnss_sim::signal_carrier_frequency_hz(*definition, test_case.glonass_fcn, &legacy_frequency_hz));
        ASSERT_TRUE(gnss_sim::signal_wavelength_m(*definition, test_case.glonass_fcn, &legacy_wavelength_m));

        gnss_sim::RtklibSignalMetadata metadata{};
        ASSERT_TRUE(gnss_sim::rtklib_signal_metadata(test_case.system, test_case.prn, test_case.rinex_code,
                                                     test_case.glonass_fcn, &metadata))
            << test_case.rinex_code;
        EXPECT_EQ(metadata.observation_code, legacy_code) << test_case.rinex_code;
        EXPECT_EQ(metadata.frequency_index, legacy_frequency_index) << test_case.rinex_code;
        EXPECT_DOUBLE_EQ(metadata.carrier_frequency_hz, legacy_frequency_hz) << test_case.rinex_code;
        EXPECT_DOUBLE_EQ(metadata.wavelength_m, legacy_wavelength_m) << test_case.rinex_code;
    }
}

TEST(RtklibSignalMetadataAdapter, NonGlonassQueriesIgnoreCallerFcn) {
    gnss_sim::RtklibSignalMetadata first{};
    gnss_sim::RtklibSignalMetadata second{};
    ASSERT_TRUE(gnss_sim::rtklib_signal_metadata(gnss_sim::RtklibSignalSystem::kGps, 1, "1C", -999, &first));
    ASSERT_TRUE(gnss_sim::rtklib_signal_metadata(gnss_sim::RtklibSignalSystem::kGps, 1, "1C", 999, &second));
    EXPECT_EQ(first.observation_code, second.observation_code);
    EXPECT_EQ(first.frequency_index, second.frequency_index);
    EXPECT_DOUBLE_EQ(first.carrier_frequency_hz, second.carrier_frequency_hz);
    EXPECT_DOUBLE_EQ(first.wavelength_m, second.wavelength_m);
}

TEST(RtklibSignalMetadataAdapter, FailuresPreserveOutput) {
    const gnss_sim::RtklibSignalMetadata sentinel{77, 88, 99.0, 111.0};
    gnss_sim::RtklibSignalMetadata metadata = sentinel;

    EXPECT_FALSE(gnss_sim::rtklib_signal_metadata(gnss_sim::RtklibSignalSystem::kGps, 0, "1C", 0, &metadata));
    EXPECT_EQ(metadata.observation_code, sentinel.observation_code);
    EXPECT_EQ(metadata.frequency_index, sentinel.frequency_index);
    EXPECT_DOUBLE_EQ(metadata.carrier_frequency_hz, sentinel.carrier_frequency_hz);
    EXPECT_DOUBLE_EQ(metadata.wavelength_m, sentinel.wavelength_m);

    EXPECT_FALSE(gnss_sim::rtklib_signal_metadata(gnss_sim::RtklibSignalSystem::kGps, 1, "ZZ", 0, &metadata));
    EXPECT_EQ(metadata.observation_code, sentinel.observation_code);

    // The pinned shared API accepts a wider GLO FCN range, but the simulator's
    // established carrier model is -7..+6. Preserve that contract here.
    EXPECT_FALSE(gnss_sim::rtklib_signal_metadata(gnss_sim::RtklibSignalSystem::kGlonass, 1, "1C", 7, &metadata));
    EXPECT_EQ(metadata.observation_code, sentinel.observation_code);

    EXPECT_FALSE(gnss_sim::rtklib_signal_metadata(static_cast<gnss_sim::RtklibSignalSystem>(99), 1, "1C", 0,
                                                  &metadata));
    EXPECT_EQ(metadata.observation_code, sentinel.observation_code);

    EXPECT_FALSE(gnss_sim::rtklib_signal_metadata(gnss_sim::RtklibSignalSystem::kGps, 1, nullptr, 0, &metadata));
    EXPECT_EQ(metadata.observation_code, sentinel.observation_code);

    EXPECT_FALSE(gnss_sim::rtklib_signal_metadata(gnss_sim::RtklibSignalSystem::kGps, 1, "1C", 0, nullptr));
}

} // namespace
