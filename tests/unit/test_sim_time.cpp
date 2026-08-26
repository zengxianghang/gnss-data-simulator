#include "gnss_sim/sim_types.h"
#include "gnss_sim/simulator.h"

#include <cstring>
#include <gtest/gtest.h>

namespace {

TEST(Bootstrap, SimTimeUsesIntegerNanoseconds) {
    const gnss_sim::SimTime time{2300, 123456789LL};
    EXPECT_EQ(time.gps_week, 2300);
    EXPECT_EQ(time.tow_ns, 123456789LL);
}

TEST(Bootstrap, RtklibRevisionIsPinned) {
    EXPECT_STREQ(gnss_sim::rtklib_commit_sha(), "47bb6cf9935b510109d16f46baa35c648e56f1b5");
    EXPECT_EQ(std::strlen(gnss_sim::rtklib_commit_sha()), 40U);
}

} // namespace
