#include "core/deterministic_rng.h"

namespace gnss_sim {
namespace {

constexpr std::uint64_t PCG32_MULTIPLIER = 6364136223846793005ULL;
constexpr double TWO_POW_53 = 9007199254740992.0;

} // namespace

std::uint32_t rng_next_u32(DeterministicRng* rng) {
    if (rng == nullptr) {
        return 0U;
    }

    const std::uint64_t old_state = rng->state;
    rng->state = old_state * PCG32_MULTIPLIER + rng->increment;

    const std::uint32_t xorshifted = static_cast<std::uint32_t>(((old_state >> 18U) ^ old_state) >> 27U);
    const std::uint32_t rotation = static_cast<std::uint32_t>(old_state >> 59U);
    return static_cast<std::uint32_t>((xorshifted >> rotation) | (xorshifted << ((0U - rotation) & 31U)));
}

void seed_rng(DeterministicRng* rng, std::uint64_t seed, std::uint64_t sequence) {
    if (rng == nullptr) {
        return;
    }

    rng->state = 0U;
    rng->increment = (sequence << 1U) | 1U;
    static_cast<void>(rng_next_u32(rng));
    rng->state += seed;
    static_cast<void>(rng_next_u32(rng));
}

std::uint64_t rng_next_u64(DeterministicRng* rng) {
    const std::uint64_t high = static_cast<std::uint64_t>(rng_next_u32(rng));
    const std::uint64_t low = static_cast<std::uint64_t>(rng_next_u32(rng));
    return (high << 32U) | low;
}

double rng_uniform_01(DeterministicRng* rng) {
    const std::uint64_t random_53_bits = rng_next_u64(rng) >> 11U;
    return static_cast<double>(random_53_bits) / TWO_POW_53;
}

double rng_uniform(DeterministicRng* rng, double minimum, double maximum) {
    if (!(maximum > minimum)) {
        return minimum;
    }
    return minimum + (maximum - minimum) * rng_uniform_01(rng);
}

} // namespace gnss_sim
