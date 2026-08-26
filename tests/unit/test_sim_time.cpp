#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

#include <cstdint>
#include <cstring>
#include <gtest/gtest.h>

namespace {

TEST(SimTime, UsesIntegerNanosecondsAndPinnedRtklibRevision) {
    const gnss_sim::SimTime time{2300, 123456789LL};
    EXPECT_EQ(time.gps_week, 2300);
    EXPECT_EQ(time.tow_ns, 123456789LL);
    EXPECT_STREQ(gnss_sim::rtklib_commit_sha(), "07e813b72c8667350c4e80293cb6679c519ef1a6");
    EXPECT_EQ(std::strlen(gnss_sim::rtklib_commit_sha()), 40U);
}

TEST(SimTime, SupportsAllFrozenSamplingRatesExactly) {
    struct RateCase {
        int rate_hz;
        std::int64_t interval_ns;
    };
    const RateCase cases[] = {
        {1, 1000000000LL}, {5, 200000000LL}, {10, 100000000LL}, {20, 50000000LL}, {50, 20000000LL}};

    for (const RateCase& test_case : cases) {
        std::int64_t interval_ns = 0;
        ASSERT_TRUE(gnss_sim::sampling_interval_ns(test_case.rate_hz, &interval_ns));
        EXPECT_EQ(interval_ns, test_case.interval_ns);

        std::uint64_t epoch_count = 0;
        ASSERT_TRUE(
            gnss_sim::epoch_count_for_duration(gnss_sim::NANOSECONDS_PER_SECOND, test_case.rate_hz, &epoch_count));
        EXPECT_EQ(epoch_count, static_cast<std::uint64_t>(test_case.rate_hz));
    }
}

TEST(SimTime, CrossesGpsWeekWithoutFloatingAccumulation) {
    const gnss_sim::SimTime start{2300, gnss_sim::GPS_WEEK_NANOSECONDS - 50000000LL};
    gnss_sim::SimTime result{};
    ASSERT_TRUE(gnss_sim::add_time_ns(start, 100000000LL, &result));
    EXPECT_EQ(result.gps_week, 2301);
    EXPECT_EQ(result.tow_ns, 50000000LL);
}

TEST(SimTime, EventAppliesAtFirstEpochAtOrAfterEventTime) {
    const gnss_sim::SimTime start{2300, 0};
    const gnss_sim::SimTime event{2300, 250000000LL};
    std::uint64_t epoch_index = 0;
    ASSERT_TRUE(gnss_sim::epoch_index_at_or_after(start, event, 10, &epoch_index));
    EXPECT_EQ(epoch_index, 3U);

    gnss_sim::SimTime epoch_time{};
    ASSERT_TRUE(gnss_sim::epoch_time_at_index(start, 10, epoch_index, &epoch_time));
    EXPECT_EQ(epoch_time.tow_ns, 300000000LL);
}

TEST(SimTime, NormalizesSecondOfWeekAcrossBoundary) {
    gnss_sim::SimTime time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 604800.1, &time));
    EXPECT_EQ(time.gps_week, 2301);
    EXPECT_EQ(time.tow_ns, 100000000LL);
}

} // namespace
