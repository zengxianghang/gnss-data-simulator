#include "model/cn0_model.h"

#include "gnss_sim/sim_time.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace gnss_sim {
namespace {

constexpr std::int64_t kFastPeriodNs = 20LL * NANOSECONDS_PER_SECOND;
constexpr std::int64_t kSlowPeriodNs = 120LL * NANOSECONDS_PER_SECOND;
constexpr double kFastAmplitudeDb = 0.25;
constexpr double kSlowAmplitudeDb = 0.50;

bool valid_time(const SimTime& time) {
    return time.gps_week >= 0 && time.tow_ns >= 0 && time.tow_ns < GPS_WEEK_NANOSECONDS;
}

double signal_cn0_offset_db(SignalId signal_id) {
    switch (signal_id) {
        case SignalId::kGpsL1Ca:
            return 0.0;
        case SignalId::kGpsL1C:
            return 1.0;
        case SignalId::kGpsL2P:
            return -2.0;
        case SignalId::kGpsL2C:
            return -1.0;
        case SignalId::kGpsL5Q:
            return 0.5;
        case SignalId::kQzssL1Ca:
            return 0.5;
        case SignalId::kQzssL1C:
            return 1.0;
        case SignalId::kQzssL2C:
            return -0.5;
        case SignalId::kQzssL5Q:
            return 0.5;
        case SignalId::kGlonassG1:
            return -0.5;
        case SignalId::kGlonassG2:
            return -1.5;
        case SignalId::kGlonassG3:
            return -0.5;
        case SignalId::kGalileoE1:
            return 0.5;
        case SignalId::kGalileoE5A:
            return 1.0;
        case SignalId::kGalileoE5B:
            return 0.5;
        case SignalId::kGalileoE6:
            return -0.5;
        case SignalId::kBeidouB1I:
            return -0.5;
        case SignalId::kBeidouB3I:
            return -1.0;
        case SignalId::kBeidouB1C:
            return 0.5;
        case SignalId::kBeidouB2A:
            return 0.5;
        case SignalId::kBeidouB2B:
            return 0.0;
    }
    return 0.0;
}

double elevation_baseline_dbhz(double elevation_deg) {
    const double elevation = std::clamp(elevation_deg, 0.0, 90.0);
    if (elevation <= 5.0) {
        return 28.0 + 0.4 * elevation;
    }
    if (elevation <= 15.0) {
        return 30.0 + 0.35 * (elevation - 5.0);
    }
    if (elevation <= 30.0) {
        return 33.5 + 0.3 * (elevation - 15.0);
    }
    if (elevation <= 60.0) {
        return 38.0 + 0.2 * (elevation - 30.0);
    }
    return 44.0 + 0.1 * (elevation - 60.0);
}

std::int64_t positive_mod(std::int64_t value, std::int64_t modulus) {
    const std::int64_t remainder = value % modulus;
    return remainder < 0 ? remainder + modulus : remainder;
}

double triangle_wave(std::int64_t phase_ns, std::int64_t period_ns) {
    const double phase = static_cast<double>(positive_mod(phase_ns, period_ns)) / static_cast<double>(period_ns);
    if (phase < 0.5) {
        return -1.0 + 4.0 * phase;
    }
    return 3.0 - 4.0 * phase;
}

std::int64_t phase_offset_ns(std::uint64_t seed, SignalId signal_id, std::int64_t period_ns, std::uint64_t salt) {
    std::uint64_t value = seed ^ salt;
    value ^= static_cast<std::uint64_t>(static_cast<unsigned int>(signal_id) + 1U) * 0x9E3779B97F4A7C15ULL;
    value ^= value >> 30U;
    value *= 0xBF58476D1CE4E5B9ULL;
    value ^= value >> 27U;
    value *= 0x94D049BB133111EBULL;
    value ^= value >> 31U;
    return static_cast<std::int64_t>(value % static_cast<std::uint64_t>(period_ns));
}

} // namespace

Cn0Model make_builtin_cn0_model(std::uint64_t seed) {
    return {Cn0ModelSource::kBuiltinFallback, seed};
}

bool cn0_model_estimate_dbhz(const Cn0Model& model, SignalId signal_id, double elevation_deg, const SimTime& time,
                             double* cn0_dbhz) {
    if (cn0_dbhz == nullptr || !std::isfinite(elevation_deg) || !valid_time(time) ||
        find_signal_definition(signal_id) == nullptr || model.source != Cn0ModelSource::kBuiltinFallback) {
        return false;
    }

    const std::int64_t absolute_time_ns = static_cast<std::int64_t>(time.gps_week) * GPS_WEEK_NANOSECONDS + time.tow_ns;
    const std::int64_t fast_phase = absolute_time_ns + phase_offset_ns(model.seed, signal_id, kFastPeriodNs, 0xA5A5U);
    const std::int64_t slow_phase = absolute_time_ns + phase_offset_ns(model.seed, signal_id, kSlowPeriodNs, 0x5A5AU);

    double value = elevation_baseline_dbhz(elevation_deg) + signal_cn0_offset_db(signal_id);
    value += kFastAmplitudeDb * triangle_wave(fast_phase, kFastPeriodNs);
    value += kSlowAmplitudeDb * triangle_wave(slow_phase, kSlowPeriodNs);
    *cn0_dbhz = std::clamp(value, 20.0, 55.0);
    return true;
}

} // namespace gnss_sim
