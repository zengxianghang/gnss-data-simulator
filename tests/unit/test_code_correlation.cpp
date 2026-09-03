#include "model/code_correlation.h"

#include <cmath>
#include <complex>
#include <gtest/gtest.h>
#include <limits>
#include <string>

namespace {

const gnss_sim::SignalDefinition& signal(gnss_sim::SignalId signal_id) {
    const gnss_sim::SignalDefinition* definition = gnss_sim::find_signal_definition(signal_id);
    EXPECT_NE(definition, nullptr);
    return *definition;
}

std::complex<double> correlation_chips(const gnss_sim::SignalDefinition& definition, double delay_chips) {
    std::complex<double> result{};
    std::string error_message;
    EXPECT_TRUE(gnss_sim::ideal_code_correlation_chips(definition.code_correlation, delay_chips, &result,
                                                       &error_message))
        << definition.name << ": " << error_message;
    return result;
}

TEST(CodeCorrelation, BpskProfilesProduceExactTriangularChipDomainResponse) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    EXPECT_EQ(gps_l1.code_correlation.model, gnss_sim::CodeCorrelationModel::kBpsk);

    EXPECT_EQ(correlation_chips(gps_l1, 0.0), std::complex<double>(1.0, 0.0));
    EXPECT_EQ(correlation_chips(gps_l1, 0.5), std::complex<double>(0.5, 0.0));
    EXPECT_EQ(correlation_chips(gps_l1, -0.5), std::complex<double>(0.5, 0.0));
    EXPECT_EQ(correlation_chips(gps_l1, 1.0), std::complex<double>(0.0, 0.0));
    EXPECT_EQ(correlation_chips(gps_l1, -1.0), std::complex<double>(0.0, 0.0));
    EXPECT_EQ(correlation_chips(gps_l1, 1.25), std::complex<double>(0.0, 0.0));
}

TEST(CodeCorrelation, SignalTimeAndMeterScaleFollowCentralChipRates) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::SignalDefinition& gps_l5 = signal(gnss_sim::SignalId::kGpsL5Q);

    double l1_duration_s = 0.0;
    double l5_duration_s = 0.0;
    double l1_length_m = 0.0;
    double l5_length_m = 0.0;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::signal_code_chip_duration_s(gps_l1, &l1_duration_s, &error_message));
    ASSERT_TRUE(gnss_sim::signal_code_chip_duration_s(gps_l5, &l5_duration_s, &error_message));
    ASSERT_TRUE(gnss_sim::signal_code_chip_length_m(gps_l1, &l1_length_m, &error_message));
    ASSERT_TRUE(gnss_sim::signal_code_chip_length_m(gps_l5, &l5_length_m, &error_message));

    EXPECT_NEAR(l1_duration_s / l5_duration_s, 10.0, 1.0e-14);
    EXPECT_NEAR(l1_length_m / l5_length_m, 10.0, 1.0e-14);

    std::complex<double> l1_half{};
    std::complex<double> l5_half{};
    ASSERT_TRUE(gnss_sim::ideal_signal_code_correlation(gps_l1, 0.5 * l1_duration_s, &l1_half, &error_message));
    ASSERT_TRUE(gnss_sim::ideal_signal_code_correlation(gps_l5, 0.5 * l5_duration_s, &l5_half, &error_message));
    EXPECT_NEAR(l1_half.real(), 0.5, 1.0e-14);
    EXPECT_NEAR(l5_half.real(), 0.5, 1.0e-14);
    EXPECT_NEAR(l1_half.imag(), 0.0, 1.0e-14);
    EXPECT_NEAR(l5_half.imag(), 0.0, 1.0e-14);
}

TEST(CodeCorrelation, GpsL1CTmbocUsesWeightedBocStructureInsteadOfTriangle) {
    const gnss_sim::SignalDefinition& gps_l1c = signal(gnss_sim::SignalId::kGpsL1C);
    ASSERT_EQ(gps_l1c.code_correlation.model, gnss_sim::CodeCorrelationModel::kTmboc);

    const std::complex<double> at_twelfth = correlation_chips(gps_l1c, 1.0 / 12.0);
    const std::complex<double> at_quarter = correlation_chips(gps_l1c, 0.25);
    const std::complex<double> at_half = correlation_chips(gps_l1c, 0.5);
    EXPECT_NEAR(at_twelfth.real(), 0.5479797979797979, 1.0e-12);
    EXPECT_NEAR(at_quarter.real(), 0.12878787878787884, 1.0e-12);
    EXPECT_NEAR(at_half.real(), -0.37878787878787884, 1.0e-12);
    EXPECT_NEAR(at_twelfth.imag(), 0.0, 1.0e-14);
    EXPECT_LT(at_twelfth.real(), 1.0 - 1.0 / 12.0);
}

TEST(CodeCorrelation, GalileoE1PilotCbocKeepsCoherentMinusCombination) {
    const gnss_sim::SignalDefinition& gal_e1 = signal(gnss_sim::SignalId::kGalileoE1);
    ASSERT_EQ(gal_e1.code_correlation.model, gnss_sim::CodeCorrelationModel::kCboc);
    ASSERT_EQ(gal_e1.code_correlation.secondary_phase, gnss_sim::CompositeSubcarrierPhase::kAntiPhase);

    const std::complex<double> at_twelfth = correlation_chips(gal_e1, 1.0 / 12.0);
    const std::complex<double> at_quarter = correlation_chips(gal_e1, 0.25);
    const std::complex<double> at_half = correlation_chips(gal_e1, 0.5);
    EXPECT_NEAR(at_twelfth.real(), 0.5505715506035093, 1.0e-12);
    EXPECT_NEAR(at_quarter.real(), 0.11117761120957005, 1.0e-12);
    EXPECT_NEAR(at_half.real(), -0.409090909090909, 1.0e-12);
    EXPECT_NEAR(at_twelfth.imag(), 0.0, 1.0e-14);
    EXPECT_NE(at_twelfth.real(), correlation_chips(signal(gnss_sim::SignalId::kGpsL1Ca), 1.0 / 12.0).real());
}

TEST(CodeCorrelation, BeidouB1CPilotQmbocUsesOrthogonalPowerComposition) {
    const gnss_sim::SignalDefinition& b1c = signal(gnss_sim::SignalId::kBeidouB1C);
    ASSERT_EQ(b1c.code_correlation.model, gnss_sim::CodeCorrelationModel::kQmboc);
    ASSERT_EQ(b1c.code_correlation.secondary_phase, gnss_sim::CompositeSubcarrierPhase::kNegativeQuadrature);

    const std::complex<double> at_twelfth = correlation_chips(b1c, 1.0 / 12.0);
    const std::complex<double> at_quarter = correlation_chips(b1c, 0.25);
    const std::complex<double> at_half = correlation_chips(b1c, 0.5);
    EXPECT_NEAR(at_twelfth.real(), 0.5479797979797979, 1.0e-12);
    EXPECT_NEAR(at_quarter.real(), 0.12878787878787884, 1.0e-12);
    EXPECT_NEAR(at_half.real(), -0.37878787878787884, 1.0e-12);
    EXPECT_NEAR(at_twelfth.imag(), 0.0, 1.0e-12);
    EXPECT_LT(at_twelfth.real(), 1.0 - 1.0 / 12.0);
}

TEST(CodeCorrelation, AutocorrelationIsHermitianAndNormalizedForCompositeProfiles) {
    for (gnss_sim::SignalId signal_id :
         {gnss_sim::SignalId::kGpsL1C, gnss_sim::SignalId::kGalileoE1, gnss_sim::SignalId::kBeidouB1C}) {
        const gnss_sim::SignalDefinition& definition = signal(signal_id);
        const std::complex<double> zero = correlation_chips(definition, 0.0);
        const std::complex<double> positive = correlation_chips(definition, 0.123);
        const std::complex<double> negative = correlation_chips(definition, -0.123);
        EXPECT_NEAR(zero.real(), 1.0, 1.0e-12) << definition.name;
        EXPECT_NEAR(zero.imag(), 0.0, 1.0e-12) << definition.name;
        EXPECT_NEAR(negative.real(), positive.real(), 1.0e-12) << definition.name;
        EXPECT_NEAR(negative.imag(), -positive.imag(), 1.0e-12) << definition.name;
    }
}

TEST(CodeCorrelation, UnsupportedAndInvalidProfilesFailExplicitly) {
    const gnss_sim::SignalDefinition& gps_l2c = signal(gnss_sim::SignalId::kGpsL2C);
    std::complex<double> result{7.0, 9.0};
    std::string error_message;
    EXPECT_FALSE(gnss_sim::ideal_code_correlation_chips(gps_l2c.code_correlation, 0.0, &result, &error_message));
    EXPECT_EQ(result, std::complex<double>(0.0, 0.0));

    gnss_sim::CodeCorrelationProfile noninteger_boc{gnss_sim::CodeCorrelationModel::kTmboc,
                                                    1.0e6,
                                                    1.5e6,
                                                    6.0e6,
                                                    4.0 / 33.0,
                                                    gnss_sim::CompositeSubcarrierPhase::kNotApplicable};
    EXPECT_TRUE(gnss_sim::validate_code_correlation_profile(noninteger_boc, &error_message));
    EXPECT_FALSE(gnss_sim::ideal_code_correlation_chips(noninteger_boc, 0.1, &result, &error_message));

    EXPECT_FALSE(gnss_sim::ideal_code_correlation_chips(signal(gnss_sim::SignalId::kGpsL1Ca).code_correlation,
                                                        std::numeric_limits<double>::quiet_NaN(), &result,
                                                        &error_message));
}

TEST(CodeCorrelation, RepeatedEvaluationIsNumericallyIdentical) {
    const gnss_sim::SignalDefinition& b1c = signal(gnss_sim::SignalId::kBeidouB1C);
    std::complex<double> first{};
    std::complex<double> second{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::ideal_code_correlation_chips(b1c.code_correlation, 0.137, &first, &error_message));
    ASSERT_TRUE(gnss_sim::ideal_code_correlation_chips(b1c.code_correlation, 0.137, &second, &error_message));
    EXPECT_EQ(first, second);
}

} // namespace
