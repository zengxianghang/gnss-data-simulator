#include "model/code_tracking_dll.h"

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

double chips_to_seconds(const gnss_sim::SignalDefinition& definition, double chips) {
    return chips / definition.code_correlation.chip_rate_hz;
}

int nearest_root_index(const gnss_sim::CodeTrackingDllRoot* roots, int root_count, double target_chips) {
    int best = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int index = 0; index < root_count; ++index) {
        const double distance = std::abs(roots[index].code_phase_chips - target_chips);
        if (distance < best_distance) {
            best = index;
            best_distance = distance;
        }
    }
    return best;
}

TEST(CodeTrackingDll, DefaultConfigUsesFrozenPointTwoChipTotalSpacing) {
    const gnss_sim::CodeTrackingDllConfig config = gnss_sim::default_code_tracking_dll_config();
    EXPECT_DOUBLE_EQ(config.early_late_total_spacing_chips, 0.2);
    std::string error_message;
    EXPECT_TRUE(gnss_sim::validate_code_tracking_dll_config(config, &error_message));
}

TEST(CodeTrackingDll, OnePathLosHasStableZeroBiasRoot) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllPath paths[] = {{0.0, {1.0, 0.0}}};
    const gnss_sim::CodeTrackingDllConfig config = gnss_sim::default_code_tracking_dll_config();

    double negative = 0.0;
    double zero = 0.0;
    double positive = 0.0;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_code_tracking_dll_discriminator(
        gps_l1, paths, 1, config, chips_to_seconds(gps_l1, -0.01), &negative, &error_message));
    ASSERT_TRUE(gnss_sim::compute_code_tracking_dll_discriminator(gps_l1, paths, 1, config, 0.0, &zero,
                                                                 &error_message));
    ASSERT_TRUE(gnss_sim::compute_code_tracking_dll_discriminator(
        gps_l1, paths, 1, config, chips_to_seconds(gps_l1, 0.01), &positive, &error_message));
    EXPECT_LT(negative, 0.0);
    EXPECT_NEAR(zero, 0.0, 1.0e-15);
    EXPECT_GT(positive, 0.0);

    gnss_sim::CodeTrackingDllRoot roots[gnss_sim::kMaxCodeTrackingDllRoots]{};
    int root_count = 0;
    ASSERT_TRUE(gnss_sim::find_code_tracking_dll_roots(gps_l1, paths, 1, config, roots,
                                                       gnss_sim::kMaxCodeTrackingDllRoots, &root_count,
                                                       &error_message));
    ASSERT_EQ(root_count, 1);
    EXPECT_TRUE(roots[0].stable);
    EXPECT_NEAR(roots[0].code_phase_chips, 0.0, 1.0e-8);
    EXPECT_NEAR(roots[0].prompt_power, 1.0, 1.0e-10);
    EXPECT_GT(roots[0].discriminator_slope_per_chip, 0.0);
}

TEST(CodeTrackingDll, ConstructiveDelayedPathPullsTrackedCodeLater) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllPath paths[] = {
        {0.0, {1.0, 0.0}},
        {chips_to_seconds(gps_l1, 0.3), {0.5, 0.0}},
    };
    const gnss_sim::CodeTrackingDllConfig config = gnss_sim::default_code_tracking_dll_config();
    gnss_sim::CodeTrackingDllRoot roots[gnss_sim::kMaxCodeTrackingDllRoots]{};
    int root_count = 0;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::find_code_tracking_dll_roots(gps_l1, paths, 2, config, roots,
                                                       gnss_sim::kMaxCodeTrackingDllRoots, &root_count,
                                                       &error_message));

    int selected = -1;
    ASSERT_TRUE(gnss_sim::select_code_tracking_dll_root(roots, root_count,
                                                       gnss_sim::CodeTrackingDllSelectionMode::ACQUISITION, 0.0,
                                                       &selected, &error_message));
    ASSERT_GE(selected, 0);
    EXPECT_TRUE(roots[selected].stable);
    EXPECT_NEAR(roots[selected].code_phase_chips, 0.05, 1.0e-6);
    EXPECT_GT(roots[selected].code_phase_chips, 0.0);
}

TEST(CodeTrackingDll, OppositePhasePathCanProduceNegativeBiasAndMultipleStableRoots) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllPath paths[] = {
        {0.0, {1.0, 0.0}},
        {chips_to_seconds(gps_l1, 0.3), {-0.5, 0.0}},
    };
    const gnss_sim::CodeTrackingDllConfig config = gnss_sim::default_code_tracking_dll_config();
    gnss_sim::CodeTrackingDllRoot roots[gnss_sim::kMaxCodeTrackingDllRoots]{};
    int root_count = 0;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::find_code_tracking_dll_roots(gps_l1, paths, 2, config, roots,
                                                       gnss_sim::kMaxCodeTrackingDllRoots, &root_count,
                                                       &error_message));

    int stable_count = 0;
    for (int index = 0; index < root_count; ++index) {
        if (roots[index].stable) {
            ++stable_count;
        }
    }
    EXPECT_GE(stable_count, 2);

    int selected = -1;
    ASSERT_TRUE(gnss_sim::select_code_tracking_dll_root(
        roots, root_count, gnss_sim::CodeTrackingDllSelectionMode::TRACKED, chips_to_seconds(gps_l1, -0.03),
        &selected, &error_message));
    ASSERT_GE(selected, 0);
    EXPECT_NEAR(roots[selected].code_phase_chips, -0.05, 1.0e-6);

    int acquisition = -1;
    ASSERT_TRUE(gnss_sim::select_code_tracking_dll_root(roots, root_count,
                                                       gnss_sim::CodeTrackingDllSelectionMode::ACQUISITION, 0.0,
                                                       &acquisition, &error_message));
    ASSERT_GE(acquisition, 0);
    EXPECT_NEAR(roots[acquisition].code_phase_chips, -0.05, 1.0e-6);
}

TEST(CodeTrackingDll, BocSidePeaksAreSurfacedAndSelectionPolicyIsDeterministic) {
    const gnss_sim::SignalDefinition& gps_l1c = signal(gnss_sim::SignalId::kGpsL1C);
    const gnss_sim::CodeTrackingDllPath paths[] = {{0.0, {1.0, 0.0}}};
    const gnss_sim::CodeTrackingDllConfig config = gnss_sim::default_code_tracking_dll_config();
    gnss_sim::CodeTrackingDllRoot roots[gnss_sim::kMaxCodeTrackingDllRoots]{};
    int root_count = 0;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::find_code_tracking_dll_roots(gps_l1c, paths, 1, config, roots,
                                                       gnss_sim::kMaxCodeTrackingDllRoots, &root_count,
                                                       &error_message));
    EXPECT_GT(root_count, 1);

    const int zero_root = nearest_root_index(roots, root_count, 0.0);
    ASSERT_GE(zero_root, 0);
    EXPECT_TRUE(roots[zero_root].stable);
    EXPECT_NEAR(roots[zero_root].code_phase_chips, 0.0, 1.0e-8);

    int acquisition = -1;
    ASSERT_TRUE(gnss_sim::select_code_tracking_dll_root(roots, root_count,
                                                       gnss_sim::CodeTrackingDllSelectionMode::ACQUISITION, 0.0,
                                                       &acquisition, &error_message));
    EXPECT_EQ(acquisition, zero_root);

    int tracked = -1;
    ASSERT_TRUE(gnss_sim::select_code_tracking_dll_root(
        roots, root_count, gnss_sim::CodeTrackingDllSelectionMode::TRACKED, chips_to_seconds(gps_l1c, 0.55),
        &tracked, &error_message));
    ASSERT_GE(tracked, 0);
    EXPECT_TRUE(roots[tracked].stable);
    EXPECT_GT(roots[tracked].code_phase_chips, 0.4);
}

TEST(CodeTrackingDll, ConfigurableSpacingChangesTwoPathEquilibrium) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllPath paths[] = {
        {0.0, {1.0, 0.0}},
        {chips_to_seconds(gps_l1, 0.3), {0.5, 0.0}},
    };
    gnss_sim::CodeTrackingDllConfig config = gnss_sim::default_code_tracking_dll_config();
    gnss_sim::CodeTrackingDllRoot roots[gnss_sim::kMaxCodeTrackingDllRoots]{};
    int root_count = 0;
    std::string error_message;

    config.early_late_total_spacing_chips = 0.4;
    ASSERT_TRUE(gnss_sim::find_code_tracking_dll_roots(gps_l1, paths, 2, config, roots,
                                                       gnss_sim::kMaxCodeTrackingDllRoots, &root_count,
                                                       &error_message));
    int selected = -1;
    ASSERT_TRUE(gnss_sim::select_code_tracking_dll_root(roots, root_count,
                                                       gnss_sim::CodeTrackingDllSelectionMode::ACQUISITION, 0.0,
                                                       &selected, &error_message));
    EXPECT_NEAR(roots[selected].code_phase_chips, 0.1, 1.0e-6);
}

TEST(CodeTrackingDll, CompositeCorrelationUsesNormalizedComplexVoltageWithoutReinterpretation) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllPath paths[] = {
        {0.0, {1.0, 0.0}},
        {0.0, {0.0, 0.5}},
    };
    std::complex<double> correlation{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::compute_code_tracking_composite_correlation(gps_l1, paths, 2, 0.0, &correlation,
                                                                      &error_message));
    EXPECT_NEAR(correlation.real(), 1.0, 1.0e-15);
    EXPECT_NEAR(correlation.imag(), 0.5, 1.0e-15);
}

TEST(CodeTrackingDll, InvalidInputsFailExplicitly) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::SignalDefinition& gps_l2c = signal(gnss_sim::SignalId::kGpsL2C);
    gnss_sim::CodeTrackingDllConfig config = gnss_sim::default_code_tracking_dll_config();
    std::string error_message;

    config.early_late_total_spacing_chips = 0.0;
    EXPECT_FALSE(gnss_sim::validate_code_tracking_dll_config(config, &error_message));

    const gnss_sim::CodeTrackingDllPath zero_path[] = {{0.0, {0.0, 0.0}}};
    gnss_sim::CodeTrackingDllRoot roots[gnss_sim::kMaxCodeTrackingDllRoots]{};
    int root_count = 0;
    EXPECT_FALSE(gnss_sim::find_code_tracking_dll_roots(gps_l1, zero_path, 1,
                                                        gnss_sim::default_code_tracking_dll_config(), roots,
                                                        gnss_sim::kMaxCodeTrackingDllRoots, &root_count,
                                                        &error_message));

    const gnss_sim::CodeTrackingDllPath path[] = {{0.0, {1.0, 0.0}}};
    EXPECT_FALSE(gnss_sim::find_code_tracking_dll_roots(gps_l2c, path, 1,
                                                        gnss_sim::default_code_tracking_dll_config(), roots,
                                                        gnss_sim::kMaxCodeTrackingDllRoots, &root_count,
                                                        &error_message));

    int selected = -1;
    gnss_sim::CodeTrackingDllRoot unstable{};
    unstable.stable = false;
    EXPECT_FALSE(gnss_sim::select_code_tracking_dll_root(&unstable, 1,
                                                         gnss_sim::CodeTrackingDllSelectionMode::TRACKED,
                                                         std::numeric_limits<double>::quiet_NaN(), &selected,
                                                         &error_message));
}

TEST(CodeTrackingDll, RepeatedRootEnumerationIsNumericallyIdentical) {
    const gnss_sim::SignalDefinition& gps_l1 = signal(gnss_sim::SignalId::kGpsL1Ca);
    const gnss_sim::CodeTrackingDllPath paths[] = {
        {0.0, {1.0, 0.0}},
        {chips_to_seconds(gps_l1, 0.3), {0.5, 0.0}},
    };
    const gnss_sim::CodeTrackingDllConfig config = gnss_sim::default_code_tracking_dll_config();
    gnss_sim::CodeTrackingDllRoot first[gnss_sim::kMaxCodeTrackingDllRoots]{};
    gnss_sim::CodeTrackingDllRoot second[gnss_sim::kMaxCodeTrackingDllRoots]{};
    int first_count = 0;
    int second_count = 0;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::find_code_tracking_dll_roots(gps_l1, paths, 2, config, first,
                                                       gnss_sim::kMaxCodeTrackingDllRoots, &first_count,
                                                       &error_message));
    ASSERT_TRUE(gnss_sim::find_code_tracking_dll_roots(gps_l1, paths, 2, config, second,
                                                       gnss_sim::kMaxCodeTrackingDllRoots, &second_count,
                                                       &error_message));
    ASSERT_EQ(first_count, second_count);
    for (int index = 0; index < first_count; ++index) {
        EXPECT_DOUBLE_EQ(first[index].code_phase_sec, second[index].code_phase_sec);
        EXPECT_DOUBLE_EQ(first[index].code_phase_chips, second[index].code_phase_chips);
        EXPECT_DOUBLE_EQ(first[index].prompt_power, second[index].prompt_power);
        EXPECT_EQ(first[index].stable, second[index].stable);
    }
}

} // namespace
