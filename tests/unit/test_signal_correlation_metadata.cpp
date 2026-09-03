#include "gnss/signal_definitions.h"

#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <limits>
#include <set>
#include <string>

namespace {

const gnss_sim::SignalDefinition& signal(gnss_sim::SignalId signal_id) {
    const gnss_sim::SignalDefinition* definition = gnss_sim::find_signal_definition(signal_id);
    EXPECT_NE(definition, nullptr);
    return *definition;
}

TEST(SignalCorrelationMetadata, EveryCentralDefinitionHasAnExplicitValidProfile) {
    std::size_t count = 0;
    const gnss_sim::SignalDefinition* definitions = gnss_sim::signal_definitions(&count);
    ASSERT_NE(definitions, nullptr);
    ASSERT_GT(count, 0U);

    const std::set<gnss_sim::SignalId> expected_unsupported = {
        gnss_sim::SignalId::kGpsL2C,
        gnss_sim::SignalId::kQzssL1C,
        gnss_sim::SignalId::kQzssL2C,
    };

    std::size_t unsupported_count = 0;
    for (std::size_t index = 0; index < count; ++index) {
        const gnss_sim::SignalDefinition& definition = definitions[index];
        std::string error_message;
        EXPECT_TRUE(gnss_sim::validate_code_correlation_profile(definition.code_correlation, &error_message))
            << definition.name << ": " << error_message;

        const bool expected_supported = expected_unsupported.count(definition.signal_id) == 0;
        EXPECT_EQ(gnss_sim::signal_has_supported_code_correlation(definition), expected_supported) << definition.name;
        if (!expected_supported) {
            ++unsupported_count;
            EXPECT_EQ(definition.code_correlation.model, gnss_sim::CodeCorrelationModel::kUnsupported);
            EXPECT_DOUBLE_EQ(definition.code_correlation.chip_rate_hz, 0.0);
        }
    }
    EXPECT_EQ(unsupported_count, expected_unsupported.size());
}

TEST(SignalCorrelationMetadata, MinimumIssue119SignalsHaveAuthoritativeProfiles) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::SignalDefinition& gps_l5 = signal(gnss_sim::SignalId::kGpsL5Q);
    const gnss_sim::SignalDefinition& gal_e1 = signal(gnss_sim::SignalId::kGalileoE1);
    const gnss_sim::SignalDefinition& gal_e5a = signal(gnss_sim::SignalId::kGalileoE5A);
    const gnss_sim::SignalDefinition& gal_e5b = signal(gnss_sim::SignalId::kGalileoE5B);
    const gnss_sim::SignalDefinition& bds_b1i = signal(gnss_sim::SignalId::kBeidouB1I);
    const gnss_sim::SignalDefinition& bds_b1c = signal(gnss_sim::SignalId::kBeidouB1C);
    const gnss_sim::SignalDefinition& bds_b2a = signal(gnss_sim::SignalId::kBeidouB2A);

    EXPECT_EQ(gps_l1.code_correlation.model, gnss_sim::CodeCorrelationModel::kBpsk);
    EXPECT_DOUBLE_EQ(gps_l1.code_correlation.chip_rate_hz, 1.023e6);
    EXPECT_EQ(gps_l5.code_correlation.model, gnss_sim::CodeCorrelationModel::kBpsk);
    EXPECT_DOUBLE_EQ(gps_l5.code_correlation.chip_rate_hz, 10.23e6);

    EXPECT_EQ(gal_e1.code_correlation.model, gnss_sim::CodeCorrelationModel::kCboc);
    EXPECT_DOUBLE_EQ(gal_e1.code_correlation.chip_rate_hz, 1.023e6);
    EXPECT_DOUBLE_EQ(gal_e1.code_correlation.primary_subcarrier_rate_hz, 1.023e6);
    EXPECT_DOUBLE_EQ(gal_e1.code_correlation.secondary_subcarrier_rate_hz, 6.138e6);
    EXPECT_DOUBLE_EQ(gal_e1.code_correlation.secondary_power_fraction, 1.0 / 11.0);
    EXPECT_EQ(gal_e1.code_correlation.secondary_phase, gnss_sim::CompositeSubcarrierPhase::kAntiPhase);
    EXPECT_EQ(gal_e5a.code_correlation.model, gnss_sim::CodeCorrelationModel::kBpsk);
    EXPECT_EQ(gal_e5b.code_correlation.model, gnss_sim::CodeCorrelationModel::kBpsk);
    EXPECT_DOUBLE_EQ(gal_e5a.code_correlation.chip_rate_hz, 10.23e6);
    EXPECT_DOUBLE_EQ(gal_e5b.code_correlation.chip_rate_hz, 10.23e6);

    EXPECT_EQ(bds_b1i.code_correlation.model, gnss_sim::CodeCorrelationModel::kBpsk);
    EXPECT_DOUBLE_EQ(bds_b1i.code_correlation.chip_rate_hz, 2.046e6);
    EXPECT_EQ(bds_b1c.code_correlation.model, gnss_sim::CodeCorrelationModel::kQmboc);
    EXPECT_DOUBLE_EQ(bds_b1c.code_correlation.chip_rate_hz, 1.023e6);
    EXPECT_DOUBLE_EQ(bds_b1c.code_correlation.primary_subcarrier_rate_hz, 1.023e6);
    EXPECT_DOUBLE_EQ(bds_b1c.code_correlation.secondary_subcarrier_rate_hz, 6.138e6);
    EXPECT_DOUBLE_EQ(bds_b1c.code_correlation.secondary_power_fraction, 4.0 / 33.0);
    EXPECT_EQ(bds_b1c.code_correlation.secondary_phase, gnss_sim::CompositeSubcarrierPhase::kQuadrature);
    EXPECT_EQ(bds_b2a.code_correlation.model, gnss_sim::CodeCorrelationModel::kBpsk);
    EXPECT_DOUBLE_EQ(bds_b2a.code_correlation.chip_rate_hz, 10.23e6);
}

TEST(SignalCorrelationMetadata, GpsL1CPilotUsesTmbocProfile) {
    const gnss_sim::CodeCorrelationProfile& profile = signal(gnss_sim::SignalId::kGpsL1C).code_correlation;
    EXPECT_EQ(profile.model, gnss_sim::CodeCorrelationModel::kTmboc);
    EXPECT_DOUBLE_EQ(profile.chip_rate_hz, 1.023e6);
    EXPECT_DOUBLE_EQ(profile.primary_subcarrier_rate_hz, 1.023e6);
    EXPECT_DOUBLE_EQ(profile.secondary_subcarrier_rate_hz, 6.138e6);
    EXPECT_DOUBLE_EQ(profile.secondary_power_fraction, 4.0 / 33.0);
    EXPECT_EQ(profile.secondary_phase, gnss_sim::CompositeSubcarrierPhase::kNotApplicable);
}

TEST(SignalCorrelationMetadata, RepresentativeBpskRatesStaySignalSpecific) {
    EXPECT_DOUBLE_EQ(signal(gnss_sim::SignalId::kGpsL2P).code_correlation.chip_rate_hz, 10.23e6);
    EXPECT_DOUBLE_EQ(signal(gnss_sim::SignalId::kQzssL1Ca).code_correlation.chip_rate_hz, 1.023e6);
    EXPECT_DOUBLE_EQ(signal(gnss_sim::SignalId::kQzssL5Q).code_correlation.chip_rate_hz, 10.23e6);
    EXPECT_DOUBLE_EQ(signal(gnss_sim::SignalId::kGlonassG1).code_correlation.chip_rate_hz, 0.511e6);
    EXPECT_DOUBLE_EQ(signal(gnss_sim::SignalId::kGlonassG2).code_correlation.chip_rate_hz, 0.511e6);
    EXPECT_DOUBLE_EQ(signal(gnss_sim::SignalId::kGlonassG3).code_correlation.chip_rate_hz, 10.23e6);
    EXPECT_DOUBLE_EQ(signal(gnss_sim::SignalId::kGalileoE6).code_correlation.chip_rate_hz, 5.115e6);
    EXPECT_DOUBLE_EQ(signal(gnss_sim::SignalId::kBeidouB3I).code_correlation.chip_rate_hz, 10.23e6);
    EXPECT_DOUBLE_EQ(signal(gnss_sim::SignalId::kBeidouB2B).code_correlation.chip_rate_hz, 10.23e6);
}

TEST(SignalCorrelationMetadata, AmbiguousCompositeSignalsFailExplicitlyInsteadOfFallingBack) {
    for (gnss_sim::SignalId signal_id :
         {gnss_sim::SignalId::kGpsL2C, gnss_sim::SignalId::kQzssL1C, gnss_sim::SignalId::kQzssL2C}) {
        const gnss_sim::SignalDefinition& definition = signal(signal_id);
        EXPECT_EQ(definition.code_correlation.model, gnss_sim::CodeCorrelationModel::kUnsupported);
        EXPECT_FALSE(gnss_sim::signal_has_supported_code_correlation(definition));
    }
}

TEST(SignalCorrelationMetadata, InvalidProfilesAreRejectedWithoutSilentRepair) {
    std::string error_message;

    gnss_sim::CodeCorrelationProfile unsupported_with_guess{
        gnss_sim::CodeCorrelationModel::kUnsupported,      1.023e6, 0.0, 0.0, 0.0,
        gnss_sim::CompositeSubcarrierPhase::kNotApplicable};
    EXPECT_FALSE(gnss_sim::validate_code_correlation_profile(unsupported_with_guess, &error_message));

    gnss_sim::CodeCorrelationProfile bpsk_without_rate{
        gnss_sim::CodeCorrelationModel::kBpsk, 0.0, 0.0, 0.0, 0.0, gnss_sim::CompositeSubcarrierPhase::kNotApplicable};
    EXPECT_FALSE(gnss_sim::validate_code_correlation_profile(bpsk_without_rate, &error_message));

    gnss_sim::CodeCorrelationProfile cboc_wrong_phase{
        gnss_sim::CodeCorrelationModel::kCboc,          1.023e6, 1.023e6, 6.138e6, 1.0 / 11.0,
        gnss_sim::CompositeSubcarrierPhase::kQuadrature};
    EXPECT_FALSE(gnss_sim::validate_code_correlation_profile(cboc_wrong_phase, &error_message));

    gnss_sim::CodeCorrelationProfile qmboc_wrong_phase{
        gnss_sim::CodeCorrelationModel::kQmboc,        1.023e6, 1.023e6, 6.138e6, 4.0 / 33.0,
        gnss_sim::CompositeSubcarrierPhase::kAntiPhase};
    EXPECT_FALSE(gnss_sim::validate_code_correlation_profile(qmboc_wrong_phase, &error_message));

    gnss_sim::CodeCorrelationProfile nonfinite{gnss_sim::CodeCorrelationModel::kBpsk,
                                               std::numeric_limits<double>::quiet_NaN(),
                                               0.0,
                                               0.0,
                                               0.0,
                                               gnss_sim::CompositeSubcarrierPhase::kNotApplicable};
    EXPECT_FALSE(gnss_sim::validate_code_correlation_profile(nonfinite, &error_message));
}

} // namespace
