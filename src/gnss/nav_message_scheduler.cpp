#include "gnss/nav_message_scheduler.h"

#include "gnss_sim/sim_time.h"

#include <cstdint>

namespace gnss_sim {
namespace {

constexpr std::uint32_t kFragment1 = 1U << 0;
constexpr std::uint32_t kFragment2 = 1U << 1;
constexpr std::uint32_t kFragment3 = 1U << 2;
constexpr std::uint32_t kFragment4 = 1U << 3;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_sim_time(const SimTime& time) {
    return time.gps_week >= 0 && time.tow_ns >= 0 && time.tow_ns < GPS_WEEK_NANOSECONDS;
}

bool set_fragment(const SimTime& acquisition_time, std::int64_t cycle_ns, std::int64_t slot_start_ns,
                  std::int64_t duration_ns, int fragment_id, std::uint32_t mask_bit,
                  NavAcquisitionFragment* fragment) {
    if (fragment == nullptr || cycle_ns <= 0 || slot_start_ns < 0 || slot_start_ns >= cycle_ns || duration_ns <= 0) {
        return false;
    }

    const std::int64_t cycle_phase_ns = acquisition_time.tow_ns % cycle_ns;
    std::int64_t delta_to_slot_start_ns = slot_start_ns - cycle_phase_ns;
    if (delta_to_slot_start_ns < 0) {
        delta_to_slot_start_ns += cycle_ns;
    }

    SimTime completion_time{};
    if (!add_time_ns(acquisition_time, delta_to_slot_start_ns + duration_ns, &completion_time)) {
        return false;
    }
    fragment->fragment_id = fragment_id;
    fragment->mask_bit = mask_bit;
    fragment->completion_time = completion_time;
    return true;
}

void update_availability_time(NavAcquisitionPlan* plan, const SimTime& completion_time) {
    if (compare_sim_time(completion_time, plan->availability_time) > 0) {
        plan->availability_time = completion_time;
    }
}

bool add_periodic_fragment(const SimTime& acquisition_time, std::int64_t cycle_sec, std::int64_t slot_start_sec,
                           std::int64_t duration_sec, int fragment_id, std::uint32_t mask_bit,
                           NavAcquisitionPlan* plan) {
    if (plan->fragment_count >= MAX_NAV_ACQUISITION_FRAGMENTS) {
        return false;
    }
    NavAcquisitionFragment fragment{};
    if (!set_fragment(acquisition_time, cycle_sec * NANOSECONDS_PER_SECOND, slot_start_sec * NANOSECONDS_PER_SECOND,
                      duration_sec * NANOSECONDS_PER_SECOND, fragment_id, mask_bit, &fragment)) {
        return false;
    }
    plan->fragments[plan->fragment_count++] = fragment;
    update_availability_time(plan, fragment.completion_time);
    plan->required_mask |= mask_bit;
    return true;
}

bool build_lnav_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    // GPS/QZSS legacy NAV repeats a 30 s frame made of five 6 s subframes.
    // A complete broadcast ephemeris requires subframes 1, 2, and 3.
    return add_periodic_fragment(acquisition_time, 30, 0, 6, 1, kFragment1, plan) &&
           add_periodic_fragment(acquisition_time, 30, 6, 6, 2, kFragment2, plan) &&
           add_periodic_fragment(acquisition_time, 30, 12, 6, 3, kFragment3, plan);
}

bool build_cnav_plan(const SimTime& acquisition_time, std::int64_t message_duration_sec, NavAcquisitionPlan* plan) {
    // CNAV ephemeris requires MT10 + MT11 plus one clock-bearing MT30-37.
    // V1 models the nominal TTFF-critical repeating sequence MT10, MT11, MT30.
    const std::int64_t cycle_sec = 3 * message_duration_sec;
    return add_periodic_fragment(acquisition_time, cycle_sec, 0, message_duration_sec, 10, kFragment1, plan) &&
           add_periodic_fragment(acquisition_time, cycle_sec, message_duration_sec, message_duration_sec, 11,
                                 kFragment2, plan) &&
           add_periodic_fragment(acquisition_time, cycle_sec, 2 * message_duration_sec, message_duration_sec, 30,
                                 kFragment3, plan);
}

bool build_cnav2_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    // L1C CNAV-2 uses an 18 s frame; subframe 2 contains clock and ephemeris.
    // Require a complete frame after acquisition rather than granting data from
    // a frame that was already in progress when tracking began.
    return add_periodic_fragment(acquisition_time, 18, 0, 18, 2, kFragment1, plan);
}

bool build_glonass_fdma_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    // Legacy GLONASS FDMA has 15 two-second strings per 30 s frame.
    // Immediate ephemeris/clock data occupy strings 1-4 and repeat every frame.
    return add_periodic_fragment(acquisition_time, 30, 0, 2, 1, kFragment1, plan) &&
           add_periodic_fragment(acquisition_time, 30, 2, 2, 2, kFragment2, plan) &&
           add_periodic_fragment(acquisition_time, 30, 4, 2, 3, kFragment3, plan) &&
           add_periodic_fragment(acquisition_time, 30, 6, 2, 4, kFragment4, plan);
}

bool build_glonass_l3oc_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    // L3OC immediate orbit/clock data are string types 10, 11, and 12.
    // Nominal L3OC strings are 3 s; model them at the start of an 18 s
    // pseudoframe while leaving non-immediate string composition unspecified.
    return add_periodic_fragment(acquisition_time, 18, 0, 3, 10, kFragment1, plan) &&
           add_periodic_fragment(acquisition_time, 18, 3, 3, 11, kFragment2, plan) &&
           add_periodic_fragment(acquisition_time, 18, 6, 3, 12, kFragment3, plan);
}

} // namespace

bool build_cold_nav_acquisition_plan(SignalId signal_id, const SimTime& acquisition_time, int issue_data,
                                     NavAcquisitionPlan* plan, std::string* error_message) {
    if (plan == nullptr || !valid_sim_time(acquisition_time)) {
        set_error(error_message, "cold NAV acquisition plan has invalid arguments");
        return false;
    }

    const SignalDefinition* definition = find_signal_definition(signal_id);
    if (definition == nullptr) {
        set_error(error_message, "cold NAV acquisition uses an unknown signal");
        return false;
    }

    NavAcquisitionPlan result{};
    result.signal_id = signal_id;
    result.family = definition->nav_message_family;
    result.acquisition_time = acquisition_time;
    result.availability_time = acquisition_time;
    result.issue_data = issue_data;

    bool ok = false;
    switch (signal_id) {
        case SignalId::kGpsL1Ca:
        case SignalId::kGpsL2P:
        case SignalId::kQzssL1Ca:
            ok = build_lnav_plan(acquisition_time, &result);
            break;
        case SignalId::kGpsL2C:
        case SignalId::kQzssL2C:
            ok = build_cnav_plan(acquisition_time, 12, &result);
            break;
        case SignalId::kGpsL5Q:
        case SignalId::kQzssL5Q:
            ok = build_cnav_plan(acquisition_time, 6, &result);
            break;
        case SignalId::kGpsL1C:
        case SignalId::kQzssL1C:
            ok = build_cnav2_plan(acquisition_time, &result);
            break;
        case SignalId::kGlonassG1:
        case SignalId::kGlonassG2:
            ok = build_glonass_fdma_plan(acquisition_time, &result);
            break;
        case SignalId::kGlonassG3:
            ok = build_glonass_l3oc_plan(acquisition_time, &result);
            break;
        default:
            set_error(error_message, "signal family is not implemented by cold NAV scheduler issue #8");
            return false;
    }

    if (!ok || result.fragment_count <= 0 || result.required_mask == 0U) {
        set_error(error_message, "cannot build cold NAV acquisition fragment schedule");
        return false;
    }
    *plan = result;
    return true;
}

std::uint32_t nav_acquisition_received_mask(const NavAcquisitionPlan& plan, const SimTime& current_time) {
    if (!valid_sim_time(current_time) || compare_sim_time(current_time, plan.acquisition_time) < 0) {
        return 0U;
    }
    std::uint32_t mask = 0U;
    for (int index = 0; index < plan.fragment_count; ++index) {
        if (compare_sim_time(plan.fragments[index].completion_time, current_time) <= 0) {
            mask |= plan.fragments[index].mask_bit;
        }
    }
    return mask;
}

bool nav_acquisition_complete(const NavAcquisitionPlan& plan, const SimTime& current_time) {
    const std::uint32_t received_mask = nav_acquisition_received_mask(plan, current_time);
    return plan.required_mask != 0U && (received_mask & plan.required_mask) == plan.required_mask;
}

void reset_nav_fragment_collector(std::uint32_t required_mask, NavFragmentCollector* collector) {
    if (collector == nullptr) {
        return;
    }
    collector->required_mask = required_mask;
    collector->received_mask = 0U;
    collector->issue_data = 0;
    collector->has_issue_data = false;
}

bool ingest_nav_fragment(NavFragmentCollector* collector, std::uint32_t fragment_mask_bit, int issue_data,
                         bool* ephemeris_complete) {
    if (collector == nullptr || ephemeris_complete == nullptr || collector->required_mask == 0U ||
        fragment_mask_bit == 0U || (fragment_mask_bit & collector->required_mask) == 0U) {
        return false;
    }

    if (!collector->has_issue_data || collector->issue_data != issue_data) {
        collector->received_mask = 0U;
        collector->issue_data = issue_data;
        collector->has_issue_data = true;
    }
    collector->received_mask |= fragment_mask_bit;
    *ephemeris_complete = (collector->received_mask & collector->required_mask) == collector->required_mask;
    return true;
}

bool deliver_cold_nav_plan_if_complete(const NavAcquisitionPlan& plan, int truth_record_index,
                                       const SimTime& current_time, NavigationState* navigation_state,
                                       NavigationUpdateEvent* event, bool* emitted, std::string* error_message) {
    if (navigation_state == nullptr || emitted == nullptr || truth_record_index < 0 || !valid_sim_time(current_time)) {
        set_error(error_message, "cold NAV delivery request has invalid arguments");
        return false;
    }
    *emitted = false;
    if (!nav_acquisition_complete(plan, current_time)) {
        return true;
    }
    return apply_truth_navigation_record(navigation_state, truth_record_index, plan.availability_time, event, emitted,
                                         error_message);
}

} // namespace gnss_sim
