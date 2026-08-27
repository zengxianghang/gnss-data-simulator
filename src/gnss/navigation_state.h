#ifndef GNSS_SIM_SRC_GNSS_NAVIGATION_STATE_H_
#define GNSS_SIM_SRC_GNSS_NAVIGATION_STATE_H_

#include "gnss/rtklib_adapter.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_types.h"

#include <string>

namespace gnss_sim {

struct NavigationState;

struct NavigationUpdateEvent {
    SimTime availability_time;
    int truth_record_index;
    int receiver_record_index;
    RtklibNavRecordKind kind;
    int satellite_number;
    int system;
    int prn;
    int message_type;
    int iode;
    int iodc;
};

NavigationState* create_navigation_state();
void destroy_navigation_state(NavigationState* state);

bool load_truth_navigation(NavigationState* state, const char* rinex_nav_path, std::string* error_message);
bool initialize_receiver_navigation(NavigationState* state, StartupMode startup_mode, const SimTime& startup_time,
                                    std::string* error_message);
bool apply_truth_navigation_record(NavigationState* state, int truth_record_index, const SimTime& availability_time,
                                   NavigationUpdateEvent* event, bool* emitted, std::string* error_message);
bool consume_truth_navigation_record_without_copy(NavigationState* state, int truth_record_index,
                                                  std::string* error_message);

const RtklibNavStore* truth_navigation_store(const NavigationState* state);
const RtklibNavStore* receiver_navigation_store(const NavigationState* state);
int navigation_truth_record_count(const NavigationState* state);
bool navigation_truth_record_delivered(const NavigationState* state, int truth_record_index);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_GNSS_NAVIGATION_STATE_H_
