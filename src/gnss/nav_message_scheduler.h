#ifndef GNSS_SIM_SRC_GNSS_NAV_MESSAGE_SCHEDULER_H_
#define GNSS_SIM_SRC_GNSS_NAV_MESSAGE_SCHEDULER_H_

#include "gnss/navigation_state.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_types.h"

#include <cstdint>
#include <string>

namespace gnss_sim {

constexpr int MAX_NAV_ACQUISITION_FRAGMENTS = 12;

enum class NavScheduleVariant {
    kDefault,
    kBeidouD1,
    kBeidouD2,
};

struct NavAcquisitionFragment {
    int fragment_id;
    std::uint32_t mask_bit;
    SimTime completion_time;
};

struct NavAcquisitionPlan {
    SignalId signal_id;
    NavMessageFamily family;
    NavScheduleVariant variant;
    SimTime acquisition_time;
    SimTime availability_time;
    int issue_data;
    std::uint32_t required_mask;
    int fragment_count;
    NavAcquisitionFragment fragments[MAX_NAV_ACQUISITION_FRAGMENTS];
};

struct NavFragmentCollector {
    std::uint32_t required_mask;
    std::uint32_t received_mask;
    int issue_data;
    bool has_issue_data;
};

bool build_cold_nav_acquisition_plan(SignalId signal_id, const SimTime& acquisition_time, int issue_data,
                                     NavAcquisitionPlan* plan, std::string* error_message);
bool build_cold_nav_acquisition_plan_with_variant(SignalId signal_id, NavScheduleVariant variant,
                                                  const SimTime& acquisition_time, int issue_data,
                                                  NavAcquisitionPlan* plan, std::string* error_message);
std::uint32_t nav_acquisition_received_mask(const NavAcquisitionPlan& plan, const SimTime& current_time);
bool nav_acquisition_complete(const NavAcquisitionPlan& plan, const SimTime& current_time);

void reset_nav_fragment_collector(std::uint32_t required_mask, NavFragmentCollector* collector);
bool ingest_nav_fragment(NavFragmentCollector* collector, std::uint32_t fragment_mask_bit, int issue_data,
                         bool* ephemeris_complete);

bool deliver_cold_nav_plan_if_complete(const NavAcquisitionPlan& plan, int truth_record_index,
                                       const SimTime& current_time, NavigationState* navigation_state,
                                       NavigationUpdateEvent* event, bool* emitted, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_GNSS_NAV_MESSAGE_SCHEDULER_H_
