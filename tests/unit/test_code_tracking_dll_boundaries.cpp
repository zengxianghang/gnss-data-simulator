#include "model/code_tracking_dll.h"

#include <complex>
#include <gtest/gtest.h>
#include <string>

namespace {

const gnss_sim::SignalDefinition& gps_l1_ca() {
    const gnss_sim::SignalDefinition* definition = gnss_sim::find_signal_definition(gnss_sim::SignalId::kGpsL1Ca);
    EXPECT_NE(definition, nullptr);
    return *definition;
}

TEST(CodeTrackingDllBoundaries, RejectsExcessiveDelaySpanBeforeIntegerConversion) {
    const gnss_sim::SignalDefinition& signal = gps_l1_ca();
    const gnss_sim::CodeTrackingDllPath paths[] = {
        {0.0, {1.0, 0.0}},
        {1.0e12, {0.5, 0.0}},
    };
    gnss_sim::CodeTrackingDllRoot roots[gnss_sim::kMaxCodeTrackingDllRoots]{};
    int root_count = 0;
    std::string error_message;

    EXPECT_FALSE(gnss_sim::find_code_tracking_dll_roots(signal, paths, 2, gnss_sim::default_code_tracking_dll_config(),
                                                        roots, gnss_sim::kMaxCodeTrackingDllRoots, &root_count,
                                                        &error_message));
    EXPECT_EQ(root_count, 0);
    EXPECT_FALSE(error_message.empty());
}

TEST(CodeTrackingDllBoundaries, RejectsUnknownRootSelectionMode) {
    gnss_sim::CodeTrackingDllRoot root{};
    root.stable = true;
    root.prompt_power = 1.0;
    int selected_index = -1;
    std::string error_message;

    EXPECT_FALSE(gnss_sim::select_code_tracking_dll_root(
        &root, 1, static_cast<gnss_sim::CodeTrackingDllSelectionMode>(99), 0.0, &selected_index, &error_message));
    EXPECT_EQ(selected_index, -1);
    EXPECT_FALSE(error_message.empty());
}

} // namespace
