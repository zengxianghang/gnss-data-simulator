#include "core/deterministic_rng.h"

#include <gtest/gtest.h>

namespace {

TEST(DeterministicRng, Pcg32ReferenceSequence) {
    gnss_sim::DeterministicRng rng{};
    gnss_sim::seed_rng(&rng, 42U, 54U);

    const std::uint32_t expected[] = {2707161783U, 2068313097U, 3122475824U, 2211639955U, 3215226955U, 3421331566U};
    for (const std::uint32_t value : expected) {
        EXPECT_EQ(gnss_sim::rng_next_u32(&rng), value);
    }
}

TEST(DeterministicRng, SameSeedProducesSameUniformSequence) {
    gnss_sim::DeterministicRng first{};
    gnss_sim::DeterministicRng second{};
    gnss_sim::seed_rng(&first, 123456U);
    gnss_sim::seed_rng(&second, 123456U);

    for (int index = 0; index < 100; ++index) {
        EXPECT_DOUBLE_EQ(gnss_sim::rng_uniform_01(&first), gnss_sim::rng_uniform_01(&second));
    }
}

} // namespace
