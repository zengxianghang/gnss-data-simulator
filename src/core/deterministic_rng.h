#ifndef GNSS_SIM_SRC_CORE_DETERMINISTIC_RNG_H_
#define GNSS_SIM_SRC_CORE_DETERMINISTIC_RNG_H_

#include <cstdint>

namespace gnss_sim {

struct DeterministicRng {
    std::uint64_t state;
    std::uint64_t increment;
};

void seed_rng(DeterministicRng* rng, std::uint64_t seed, std::uint64_t sequence = 54U);
std::uint32_t rng_next_u32(DeterministicRng* rng);
std::uint64_t rng_next_u64(DeterministicRng* rng);
double rng_uniform_01(DeterministicRng* rng);
double rng_uniform(DeterministicRng* rng, double minimum, double maximum);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_CORE_DETERMINISTIC_RNG_H_
