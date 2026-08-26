#include "gnss/nav_message_scheduler.h"

#include "gnss_sim/sim_time.h"

#include <cstdint>

namespace gnss_sim {
namespace {

constexpr std::int64_t kBdtFromGpstOffsetNs = -14LL * NANOSECONDS_PER_SECOND;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_sim_time(const SimTime& time) {
    return time.gps_week >= 0 && time.tow_ns >= 0 && time.tow_ns < GPS_WEEK_NANOSECONDS;
}

std::int64_t positive_mod(std::int64_t value, std::int64_t modulus) {
    std::int64_t result = value % modulus;
    if (result < 0) {
        result += modulus;
    }
    return result;
}

bool set_fragment(const SimTime& acquisition_time, std::int64_t cycle_ns, std::int64_t slot_start_ns,
                  std::int64_t duration_ns, std::int64_t phase_offset_ns, int fragment_id,
                  std::uint32_t mask_bit, NavAcquisitionFragment* fragment) {
    if (fragment == nullptr || cycle_ns <= 0 || slot_start_ns < 0 || slot_start_ns >= cycle_ns || duration_ns <= 0) {
        return false;
    }

    const std::int64_t cycle_phase_ns = positive_mod(acquisition_time.tow_ns + phase_offset_ns, cycle_ns);
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

bool add_periodic_fragment_ns(const SimTime& acquisition_time, std::int64_t cycle_ns, std::int64_t slot_start_ns,
                              std::int64_t duration_ns, std::int64_t phase_offset_ns, int fragment_id,
                              NavAcquisitionPlan* plan) {
    if (plan == nullptr || plan->fragment_count < 0 || plan->fragment_count >= MAX_NAV_ACQUISITION_FRAGMENTS ||
        plan->fragment_count >= 32) {
        return false;
    }
    const std::uint32_t mask_bit = 1U << plan->fragment_count;
    NavAcquisitionFragment fragment{};
    if (!set_fragment(acquisition_time, cycle_ns, slot_start_ns, duration_ns, phase_offset_ns, fragment_id, mask_bit,
                      &fragment)) {
        return false;
    }
    plan->fragments[plan->fragment_count++] = fragment;
    update_availability_time(plan, fragment.completion_time);
    plan->required_mask |= mask_bit;
    return true;
}

bool add_periodic_fragment(const SimTime& acquisition_time, std::int64_t cycle_sec, std::int64_t slot_start_sec,
                           std::int64_t duration_sec, int fragment_id, NavAcquisitionPlan* plan,
                           std::int64_t phase_offset_ns = 0) {
    return add_periodic_fragment_ns(acquisition_time, cycle_sec * NANOSECONDS_PER_SECOND,
                                    slot_start_sec * NANOSECONDS_PER_SECOND,
                                    duration_sec * NANOSECONDS_PER_SECOND, phase_offset_ns, fragment_id, plan);
}

bool build_lnav_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    return add_periodic_fragment(acquisition_time, 30, 0, 6, 1, plan) &&
           add_periodic_fragment(acquisition_time, 30, 6, 6, 2, plan) &&
           add_periodic_fragment(acquisition_time, 30, 12, 6, 3, plan);
}

bool build_cnav_plan(const SimTime& acquisition_time, std::int64_t message_duration_sec, NavAcquisitionPlan* plan) {
    const std::int64_t cycle_sec = 3 * message_duration_sec;
    return add_periodic_fragment(acquisition_time, cycle_sec, 0, message_duration_sec, 10, plan) &&
           add_periodic_fragment(acquisition_time, cycle_sec, message_duration_sec, message_duration_sec, 11, plan) &&
           add_periodic_fragment(acquisition_time, cycle_sec, 2 * message_duration_sec, message_duration_sec, 30,
                                 plan);
}

bool build_cnav2_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    return add_periodic_fragment(acquisition_time, 18, 0, 18, 2, plan);
}

bool build_glonass_fdma_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    return add_periodic_fragment(acquisition_time, 30, 0, 2, 1, plan) &&
           add_periodic_fragment(acquisition_time, 30, 2, 2, 2, plan) &&
           add_periodic_fragment(acquisition_time, 30, 4, 2, 3, plan) &&
           add_periodic_fragment(acquisition_time, 30, 6, 2, 4, plan);
}

bool build_glonass_l3oc_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    return add_periodic_fragment(acquisition_time, 18, 0, 3, 10, plan) &&
           add_periodic_fragment(acquisition_time, 18, 3, 3, 11, plan) &&
           add_periodic_fragment(acquisition_time, 18, 6, 3, 12, plan);
}

bool build_galileo_inav_e1_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    // Current nominal I/NAV E1-B subframe: WT2 completes at T0+3 s,
    // WT4 at +5 s, WT1 at +23 s, and WT3 at +25 s. RedCED is
    // deliberately not treated as a normal RTKLIB broadcast ephemeris.
    return add_periodic_fragment(acquisition_time, 30, 1, 2, 2, plan) &&
           add_periodic_fragment(acquisition_time, 30, 3, 2, 4, plan) &&
           add_periodic_fragment(acquisition_time, 30, 21, 2, 1, plan) &&
           add_periodic_fragment(acquisition_time, 30, 23, 2, 3, plan);
}

bool build_galileo_inav_e5b_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    // Current nominal I/NAV E5b-I subframe: WT1, WT3, WT2, WT4 occupy
    // complete two-second pages beginning at T0+0,+2,+20,+22 s.
    return add_periodic_fragment(acquisition_time, 30, 0, 2, 1, plan) &&
           add_periodic_fragment(acquisition_time, 30, 2, 2, 3, plan) &&
           add_periodic_fragment(acquisition_time, 30, 20, 2, 2, plan) &&
           add_periodic_fragment(acquisition_time, 30, 22, 2, 4, plan);
}

bool build_galileo_fnav_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    // E5a-I F/NAV has 10 s pages in a 50 s subframe. Page 1 contains
    // clock/BGD and pages 2-4 contain the three ephemeris parts.
    return add_periodic_fragment(acquisition_time, 50, 0, 10, 1, plan) &&
           add_periodic_fragment(acquisition_time, 50, 10, 10, 2, plan) &&
           add_periodic_fragment(acquisition_time, 50, 20, 10, 3, plan) &&
           add_periodic_fragment(acquisition_time, 50, 30, 10, 4, plan);
}

bool build_beidou_d1_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    // BDS D1 basic navigation data occupy subframes 1-3 of a 30 s frame.
    // Frame phase is defined in BDT; BDT = GPST - 14 s.
    return add_periodic_fragment(acquisition_time, 30, 0, 6, 1, plan, kBdtFromGpstOffsetNs) &&
           add_periodic_fragment(acquisition_time, 30, 6, 6, 2, plan, kBdtFromGpstOffsetNs) &&
           add_periodic_fragment(acquisition_time, 30, 12, 6, 3, plan, kBdtFromGpstOffsetNs);
}

bool build_beidou_d2_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    // GEO D2 basic navigation data are subcommutated over pages 1-10 of
    // subframe 1 across ten consecutive 3 s frames. Each subframe is 0.6 s.
    constexpr std::int64_t kCycleNs = 30LL * NANOSECONDS_PER_SECOND;
    constexpr std::int64_t kFrameNs = 3LL * NANOSECONDS_PER_SECOND;
    constexpr std::int64_t kSubframeNs = 600000000LL;
    for (int page = 1; page <= 10; ++page) {
        if (!add_periodic_fragment_ns(acquisition_time, kCycleNs, (page - 1) * kFrameNs, kSubframeNs,
                                      kBdtFromGpstOffsetNs, page, plan)) {
            return false;
        }
    }
    return true;
}

bool build_beidou_bcnav1_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    // B1C B-CNAV1 is an 18 s frame. Subframe 2 contains the complete
    // ephemeris, clock and group-delay data; V1 conservatively requires one
    // complete frame after acquisition so partial frame sync is never reused.
    return add_periodic_fragment(acquisition_time, 18, 0, 18, 2, plan, kBdtFromGpstOffsetNs);
}

bool build_beidou_bcnav2_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    // B2a B-CNAV2 uses 3 s frames. MT10 and MT11 carry ephemeris I/II and
    // are continuously paired; MT30 carries clock/TGD/ISC. Broadcast order is
    // dynamically adjustable, so V1 freezes the deterministic nominal
    // TTFF-critical schedule MT10 -> MT11 -> MT30.
    return add_periodic_fragment(acquisition_time, 9, 0, 3, 10, plan, kBdtFromGpstOffsetNs) &&
           add_periodic_fragment(acquisition_time, 9, 3, 3, 11, plan, kBdtFromGpstOffsetNs) &&
           add_periodic_fragment(acquisition_time, 9, 6, 3, 30, plan, kBdtFromGpstOffsetNs);
}

bool build_beidou_bcnav3_plan(const SimTime& acquisition_time, NavAcquisitionPlan* plan) {
    // B2b B-CNAV3 uses 1 s frames. MT10 contains ephemeris I+II while MT30
    // carries clock and TGD. Use a deterministic nominal two-frame schedule.
    return add_periodic_fragment(acquisition_time, 2, 0, 1, 10, plan, kBdtFromGpstOffsetNs) &&
           add_periodic_fragment(acquisition_time, 2, 1, 1, 30, plan, kBdtFromGpstOffsetNs);
}

bool variant_is_default(NavScheduleVariant variant) {
    return variant == NavScheduleVariant::kDefault;
}

} // namespace

bool build_cold_nav_acquisition_plan(SignalId signal_id, const SimTime& acquisition_time, int issue_data,
                                     NavAcquisitionPlan* plan, std::string* error_message) {
    return build_cold_nav_acquisition_plan_with_variant(signal_id, NavScheduleVariant::kDefault, acquisition_time,
                                                        issue_data, plan, error_message);
}

bool build_cold_nav_acquisition_plan_with_variant(SignalId signal_id, NavScheduleVariant variant,
                                                  const SimTime& acquisition_time, int issue_data,
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
    result.variant = variant;
    result.acquisition_time = acquisition_time;
    result.availability_time = acquisition_time;
    result.issue_data = issue_data;

    bool ok = false;
    switch (signal_id) {
        case SignalId::kGpsL1Ca:
        case SignalId::kGpsL2P:
        case SignalId::kQzssL1Ca:
            if (!variant_is_default(variant)) {
                set_error(error_message, "NAV schedule variant is invalid for GPS/QZSS LNAV");
                return false;
            }
            ok = build_lnav_plan(acquisition_time, &result);
            break;
        case SignalId::kGpsL2C:
        case SignalId::kQzssL2C:
            if (!variant_is_default(variant)) {
                set_error(error_message, "NAV schedule variant is invalid for GPS/QZSS CNAV");
                return false;
            }
            ok = build_cnav_plan(acquisition_time, 12, &result);
            break;
        case SignalId::kGpsL5Q:
        case SignalId::kQzssL5Q:
            if (!variant_is_default(variant)) {
                set_error(error_message, "NAV schedule variant is invalid for GPS/QZSS CNAV");
                return false;
            }
            ok = build_cnav_plan(acquisition_time, 6, &result);
            break;
        case SignalId::kGpsL1C:
        case SignalId::kQzssL1C:
            if (!variant_is_default(variant)) {
                set_error(error_message, "NAV schedule variant is invalid for GPS/QZSS CNAV-2");
                return false;
            }
            ok = build_cnav2_plan(acquisition_time, &result);
            break;
        case SignalId::kGlonassG1:
        case SignalId::kGlonassG2:
            if (!variant_is_default(variant)) {
                set_error(error_message, "NAV schedule variant is invalid for GLONASS FDMA");
                return false;
            }
            ok = build_glonass_fdma_plan(acquisition_time, &result);
            break;
        case SignalId::kGlonassG3:
            if (!variant_is_default(variant)) {
                set_error(error_message, "NAV schedule variant is invalid for GLONASS L3OC");
                return false;
            }
            ok = build_glonass_l3oc_plan(acquisition_time, &result);
            break;
        case SignalId::kGalileoE1:
            if (!variant_is_default(variant)) {
                set_error(error_message, "NAV schedule variant is invalid for Galileo I/NAV");
                return false;
            }
            ok = build_galileo_inav_e1_plan(acquisition_time, &result);
            break;
        case SignalId::kGalileoE5B:
            if (!variant_is_default(variant)) {
                set_error(error_message, "NAV schedule variant is invalid for Galileo I/NAV");
                return false;
            }
            ok = build_galileo_inav_e5b_plan(acquisition_time, &result);
            break;
        case SignalId::kGalileoE5A:
            if (!variant_is_default(variant)) {
                set_error(error_message, "NAV schedule variant is invalid for Galileo F/NAV");
                return false;
            }
            ok = build_galileo_fnav_plan(acquisition_time, &result);
            break;
        case SignalId::kGalileoE6:
            set_error(error_message,
                      "Galileo E6 C/NAV/HAS does not provide the normal broadcast ephemeris used by Receiver NAV");
            return false;
        case SignalId::kBeidouB1I:
        case SignalId::kBeidouB3I:
            if (variant == NavScheduleVariant::kDefault || variant == NavScheduleVariant::kBeidouD1) {
                result.variant = NavScheduleVariant::kBeidouD1;
                ok = build_beidou_d1_plan(acquisition_time, &result);
            } else if (variant == NavScheduleVariant::kBeidouD2) {
                ok = build_beidou_d2_plan(acquisition_time, &result);
            }
            break;
        case SignalId::kBeidouB1C:
            if (!variant_is_default(variant)) {
                set_error(error_message, "NAV schedule variant is invalid for BeiDou B-CNAV1");
                return false;
            }
            ok = build_beidou_bcnav1_plan(acquisition_time, &result);
            break;
        case SignalId::kBeidouB2A:
            if (!variant_is_default(variant)) {
                set_error(error_message, "NAV schedule variant is invalid for BeiDou B-CNAV2");
                return false;
            }
            ok = build_beidou_bcnav2_plan(acquisition_time, &result);
            break;
        case SignalId::kBeidouB2B:
            if (!variant_is_default(variant)) {
                set_error(error_message, "NAV schedule variant is invalid for BeiDou B-CNAV3");
                return false;
            }
            ok = build_beidou_bcnav3_plan(acquisition_time, &result);
            break;
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

    RtklibNavRecordInfo record_info{};
    if (!rtklib_nav_record_info(truth_navigation_store(navigation_state), truth_record_index, &record_info)) {
        set_error(error_message, "cold NAV plan references an invalid truth navigation record");
        return false;
    }
    if (record_info.kind == RtklibNavRecordKind::kIonosphere || record_info.iode != plan.issue_data) {
        set_error(error_message, "cold NAV fragment IOD does not match truth ephemeris record");
        return false;
    }

    return apply_truth_navigation_record(navigation_state, truth_record_index, plan.availability_time, event, emitted,
                                         error_message);
}

} // namespace gnss_sim
