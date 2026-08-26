#ifndef GNSS_SIM_SIM_TIME_H_
#define GNSS_SIM_SIM_TIME_H_

#include "gnss_sim/sim_types.h"

#include <cstdint>

namespace gnss_sim {

constexpr std::int64_t NANOSECONDS_PER_SECOND = 1000000000LL;
constexpr std::int64_t GPS_WEEK_SECONDS = 604800LL;
constexpr std::int64_t GPS_WEEK_NANOSECONDS = GPS_WEEK_SECONDS * NANOSECONDS_PER_SECOND;

bool normalize_sim_time(int gps_week, std::int64_t tow_ns, SimTime* out_time);
bool sim_time_from_week_sow(int gps_week, double sow_sec, SimTime* out_time);
double sim_time_sow_sec(const SimTime& time);
int compare_sim_time(const SimTime& lhs, const SimTime& rhs);
bool add_time_ns(const SimTime& time, std::int64_t delta_ns, SimTime* out_time);
bool difference_time_ns(const SimTime& lhs, const SimTime& rhs, std::int64_t* difference_ns);
bool sampling_interval_ns(int sampling_rate_hz, std::int64_t* interval_ns);
bool epoch_time_at_index(const SimTime& start_time, int sampling_rate_hz, std::uint64_t epoch_index,
                         SimTime* epoch_time);
bool epoch_index_at_or_after(const SimTime& start_time, const SimTime& event_time, int sampling_rate_hz,
                             std::uint64_t* epoch_index);
bool epoch_count_for_duration(std::int64_t duration_ns, int sampling_rate_hz, std::uint64_t* epoch_count);

} // namespace gnss_sim

#endif // GNSS_SIM_SIM_TIME_H_
