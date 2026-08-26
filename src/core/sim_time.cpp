#include "gnss_sim/sim_time.h"

#include <cmath>
#include <limits>

namespace gnss_sim {
namespace {

bool checked_week_adjustment(int gps_week, std::int64_t week_adjustment, int* normalized_week) {
    if (normalized_week == nullptr) {
        return false;
    }

    const std::int64_t week = static_cast<std::int64_t>(gps_week) + week_adjustment;
    if (week < 0 || week > static_cast<std::int64_t>(std::numeric_limits<int>::max())) {
        return false;
    }

    *normalized_week = static_cast<int>(week);
    return true;
}

bool valid_normalized_time(const SimTime& time) {
    return time.gps_week >= 0 && time.tow_ns >= 0 && time.tow_ns < GPS_WEEK_NANOSECONDS;
}

} // namespace

bool normalize_sim_time(int gps_week, std::int64_t tow_ns, SimTime* out_time) {
    if (out_time == nullptr || gps_week < 0) {
        return false;
    }

    std::int64_t week_adjustment = tow_ns / GPS_WEEK_NANOSECONDS;
    std::int64_t normalized_tow_ns = tow_ns % GPS_WEEK_NANOSECONDS;
    if (normalized_tow_ns < 0) {
        normalized_tow_ns += GPS_WEEK_NANOSECONDS;
        --week_adjustment;
    }

    int normalized_week = 0;
    if (!checked_week_adjustment(gps_week, week_adjustment, &normalized_week)) {
        return false;
    }

    out_time->gps_week = normalized_week;
    out_time->tow_ns = normalized_tow_ns;
    return true;
}

bool sim_time_from_week_sow(int gps_week, double sow_sec, SimTime* out_time) {
    if (out_time == nullptr || gps_week < 0 || !std::isfinite(sow_sec)) {
        return false;
    }

    const long double tow_ns_value =
        static_cast<long double>(sow_sec) * static_cast<long double>(NANOSECONDS_PER_SECOND);
    const long double minimum = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
    const long double maximum = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
    if (tow_ns_value < minimum || tow_ns_value > maximum) {
        return false;
    }

    const std::int64_t tow_ns = static_cast<std::int64_t>(std::llround(tow_ns_value));
    return normalize_sim_time(gps_week, tow_ns, out_time);
}

double sim_time_sow_sec(const SimTime& time) {
    return static_cast<double>(time.tow_ns) / static_cast<double>(NANOSECONDS_PER_SECOND);
}

int compare_sim_time(const SimTime& lhs, const SimTime& rhs) {
    if (lhs.gps_week < rhs.gps_week) {
        return -1;
    }
    if (lhs.gps_week > rhs.gps_week) {
        return 1;
    }
    if (lhs.tow_ns < rhs.tow_ns) {
        return -1;
    }
    if (lhs.tow_ns > rhs.tow_ns) {
        return 1;
    }
    return 0;
}

bool add_time_ns(const SimTime& time, std::int64_t delta_ns, SimTime* out_time) {
    if (out_time == nullptr || !valid_normalized_time(time)) {
        return false;
    }

    const std::int64_t whole_weeks = delta_ns / GPS_WEEK_NANOSECONDS;
    const std::int64_t remainder_ns = delta_ns % GPS_WEEK_NANOSECONDS;
    int adjusted_week = 0;
    if (!checked_week_adjustment(time.gps_week, whole_weeks, &adjusted_week)) {
        return false;
    }

    return normalize_sim_time(adjusted_week, time.tow_ns + remainder_ns, out_time);
}

bool difference_time_ns(const SimTime& lhs, const SimTime& rhs, std::int64_t* difference_ns) {
    if (difference_ns == nullptr || !valid_normalized_time(lhs) || !valid_normalized_time(rhs)) {
        return false;
    }

    const std::int64_t week_delta = static_cast<std::int64_t>(lhs.gps_week) - static_cast<std::int64_t>(rhs.gps_week);
    const std::int64_t max_week_delta = std::numeric_limits<std::int64_t>::max() / GPS_WEEK_NANOSECONDS;
    if (week_delta > max_week_delta || week_delta < -max_week_delta) {
        return false;
    }

    const std::int64_t week_ns = week_delta * GPS_WEEK_NANOSECONDS;
    const std::int64_t tow_delta = lhs.tow_ns - rhs.tow_ns;
    if ((tow_delta > 0 && week_ns > std::numeric_limits<std::int64_t>::max() - tow_delta) ||
        (tow_delta < 0 && week_ns < std::numeric_limits<std::int64_t>::min() - tow_delta)) {
        return false;
    }

    *difference_ns = week_ns + tow_delta;
    return true;
}

bool sampling_interval_ns(int sampling_rate_hz, std::int64_t* interval_ns) {
    if (interval_ns == nullptr) {
        return false;
    }

    switch (sampling_rate_hz) {
        case 1:
            *interval_ns = 1000000000LL;
            return true;
        case 5:
            *interval_ns = 200000000LL;
            return true;
        case 10:
            *interval_ns = 100000000LL;
            return true;
        case 20:
            *interval_ns = 50000000LL;
            return true;
        case 50:
            *interval_ns = 20000000LL;
            return true;
        default:
            return false;
    }
}

bool epoch_time_at_index(const SimTime& start_time, int sampling_rate_hz, std::uint64_t epoch_index,
                         SimTime* epoch_time) {
    std::int64_t interval_ns = 0;
    if (epoch_time == nullptr || !sampling_interval_ns(sampling_rate_hz, &interval_ns)) {
        return false;
    }

    const std::uint64_t maximum_index =
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / interval_ns);
    if (epoch_index > maximum_index) {
        return false;
    }

    const std::int64_t delta_ns = static_cast<std::int64_t>(epoch_index) * interval_ns;
    return add_time_ns(start_time, delta_ns, epoch_time);
}

bool epoch_index_at_or_after(const SimTime& start_time, const SimTime& event_time, int sampling_rate_hz,
                             std::uint64_t* epoch_index) {
    std::int64_t interval_ns = 0;
    std::int64_t delta_ns = 0;
    if (epoch_index == nullptr || !sampling_interval_ns(sampling_rate_hz, &interval_ns) ||
        !difference_time_ns(event_time, start_time, &delta_ns)) {
        return false;
    }

    if (delta_ns <= 0) {
        *epoch_index = 0U;
        return true;
    }

    *epoch_index = static_cast<std::uint64_t>((delta_ns - 1) / interval_ns + 1);
    return true;
}

bool epoch_count_for_duration(std::int64_t duration_ns, int sampling_rate_hz, std::uint64_t* epoch_count) {
    std::int64_t interval_ns = 0;
    if (epoch_count == nullptr || duration_ns <= 0 || !sampling_interval_ns(sampling_rate_hz, &interval_ns)) {
        return false;
    }

    *epoch_count = static_cast<std::uint64_t>((duration_ns - 1) / interval_ns + 1);
    return true;
}

} // namespace gnss_sim
