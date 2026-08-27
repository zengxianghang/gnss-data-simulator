#include "gnss/signal_definitions.h"

#include <gtest/gtest.h>

TEST(SolutionSignalPolicy, LegacySinglePointPrimarySignalsAreExplicit) {
    EXPECT_EQ(gnss_sim::signal_single_point_priority(gnss_sim::SignalId::kGpsL1Ca), 0);
    EXPECT_EQ(gnss_sim::signal_single_point_priority(gnss_sim::SignalId::kGpsL1C), 1);
    EXPECT_EQ(gnss_sim::signal_single_point_priority(gnss_sim::SignalId::kQzssL1Ca), 0);
    EXPECT_EQ(gnss_sim::signal_single_point_priority(gnss_sim::SignalId::kGlonassG1), 0);
    EXPECT_EQ(gnss_sim::signal_single_point_priority(gnss_sim::SignalId::kGalileoE1), 0);
    EXPECT_EQ(gnss_sim::signal_single_point_priority(gnss_sim::SignalId::kBeidouB1I), 0);
}

TEST(SolutionSignalPolicy, UnsupportedSecondarySignalsDoNotSilentlyEnterLegacySpp) {
    EXPECT_LT(gnss_sim::signal_single_point_priority(gnss_sim::SignalId::kGpsL2C), 0);
    EXPECT_LT(gnss_sim::signal_single_point_priority(gnss_sim::SignalId::kQzssL5Q), 0);
    EXPECT_LT(gnss_sim::signal_single_point_priority(gnss_sim::SignalId::kGlonassG2), 0);
    EXPECT_LT(gnss_sim::signal_single_point_priority(gnss_sim::SignalId::kGalileoE5A), 0);
    EXPECT_LT(gnss_sim::signal_single_point_priority(gnss_sim::SignalId::kBeidouB1C), 0);
}
