#include "gnss/navigation_state.h"

#include "gnss_sim/sim_time.h"

#include <cstddef>
#include <cstring>
#include <new>

namespace gnss_sim {

struct NavigationState {
    RtklibNavStore* truth_nav;
    RtklibNavStore* receiver_nav;
    unsigned char* delivered_records;
    int delivered_record_count;
};

namespace {

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

void clear_delivery_state(NavigationState* state) {
    delete[] state->delivered_records;
    state->delivered_records = nullptr;
    state->delivered_record_count = 0;
}

bool valid_sim_time(const SimTime& time) {
    return time.gps_week >= 0 && time.tow_ns >= 0 && time.tow_ns < GPS_WEEK_NANOSECONDS;
}

bool allocate_delivery_state(NavigationState* state, int count, std::string* error_message) {
    clear_delivery_state(state);
    if (count <= 0) {
        return true;
    }
    state->delivered_records = new (std::nothrow) unsigned char[static_cast<std::size_t>(count)];
    if (state->delivered_records == nullptr) {
        set_error(error_message, "cannot allocate navigation delivery state");
        return false;
    }
    std::memset(state->delivered_records, 0, static_cast<std::size_t>(count));
    state->delivered_record_count = count;
    return true;
}

bool record_transmitted_at_or_before(const RtklibNavStore* truth_nav, int record_index, const SimTime& time) {
    RtklibNavRecordInfo info{};
    if (!rtklib_nav_record_info(truth_nav, record_index, &info)) {
        return false;
    }
    SimTime record_time{};
    if (!sim_time_from_week_sow(info.gps_week, info.transmit_sow_sec, &record_time)) {
        return false;
    }
    return compare_sim_time(record_time, time) <= 0;
}

bool same_record_identity(const RtklibNavRecordInfo& lhs, const RtklibNavRecordInfo& rhs) {
    if (lhs.kind != rhs.kind || lhs.satellite_number != rhs.satellite_number || lhs.system != rhs.system ||
        lhs.prn != rhs.prn || lhs.message_type != rhs.message_type || lhs.iode != rhs.iode || lhs.iodc != rhs.iodc ||
        lhs.gps_week != rhs.gps_week) {
        return false;
    }
    SimTime lhs_time{};
    SimTime rhs_time{};
    if (!sim_time_from_week_sow(lhs.gps_week, lhs.transmit_sow_sec, &lhs_time) ||
        !sim_time_from_week_sow(rhs.gps_week, rhs.transmit_sow_sec, &rhs_time)) {
        return false;
    }
    return compare_sim_time(lhs_time, rhs_time) == 0;
}

int find_receiver_record_index(const RtklibNavStore* receiver_nav, const RtklibNavRecordInfo& source_info) {
    const int count = rtklib_nav_record_count(receiver_nav);
    for (int index = count - 1; index >= 0; --index) {
        RtklibNavRecordInfo receiver_info{};
        if (rtklib_nav_record_info(receiver_nav, index, &receiver_info) &&
            same_record_identity(source_info, receiver_info)) {
            return index;
        }
    }
    return -1;
}

} // namespace

NavigationState* create_navigation_state() {
    NavigationState* state = new (std::nothrow) NavigationState{};
    if (state == nullptr) {
        return nullptr;
    }
    state->truth_nav = create_rtklib_nav_store();
    state->receiver_nav = create_rtklib_nav_store();
    if (state->truth_nav == nullptr || state->receiver_nav == nullptr) {
        destroy_rtklib_nav_store(state->truth_nav);
        destroy_rtklib_nav_store(state->receiver_nav);
        delete state;
        return nullptr;
    }
    return state;
}

void destroy_navigation_state(NavigationState* state) {
    if (state == nullptr) {
        return;
    }
    clear_delivery_state(state);
    destroy_rtklib_nav_store(state->truth_nav);
    destroy_rtklib_nav_store(state->receiver_nav);
    delete state;
}

bool load_truth_navigation(NavigationState* state, const char* rinex_nav_path, std::string* error_message) {
    if (state == nullptr || rinex_nav_path == nullptr || rinex_nav_path[0] == '\0') {
        set_error(error_message, "navigation state and RINEX NAV path must be valid");
        return false;
    }
    clear_delivery_state(state);
    rtklib_clear_nav_store(state->receiver_nav);
    return load_rinex_nav_file(state->truth_nav, rinex_nav_path, error_message);
}

bool initialize_receiver_navigation(NavigationState* state, StartupMode startup_mode, const SimTime& startup_time,
                                    std::string* error_message) {
    if (state == nullptr || !valid_sim_time(startup_time)) {
        set_error(error_message, "receiver navigation initialization has invalid arguments");
        return false;
    }

    const int record_count = rtklib_nav_record_count(state->truth_nav);
    if (record_count <= 0) {
        set_error(error_message, "truth navigation is empty");
        return false;
    }
    if (!allocate_delivery_state(state, record_count, error_message)) {
        return false;
    }

    if (startup_mode == StartupMode::COLD) {
        if (!rtklib_clear_nav_store(state->receiver_nav)) {
            set_error(error_message, "cannot clear receiver navigation for cold start");
            return false;
        }
        return true;
    }

    if (!rtklib_copy_nav_snapshot(state->truth_nav, startup_time.gps_week, sim_time_sow_sec(startup_time),
                                  state->receiver_nav, error_message)) {
        return false;
    }

    for (int record_index = 0; record_index < record_count; ++record_index) {
        if (record_transmitted_at_or_before(state->truth_nav, record_index, startup_time)) {
            state->delivered_records[record_index] = 1;
        }
    }
    return true;
}

bool apply_truth_navigation_record(NavigationState* state, int truth_record_index, const SimTime& availability_time,
                                   NavigationUpdateEvent* event, bool* emitted, std::string* error_message) {
    if (state == nullptr || emitted == nullptr || !valid_sim_time(availability_time) || truth_record_index < 0 ||
        truth_record_index >= state->delivered_record_count || state->delivered_records == nullptr) {
        set_error(error_message, "navigation update request has invalid arguments");
        return false;
    }

    *emitted = false;
    if (state->delivered_records[truth_record_index] != 0) {
        return true;
    }

    RtklibNavRecordInfo info{};
    if (!rtklib_nav_record_info(state->truth_nav, truth_record_index, &info)) {
        set_error(error_message, "cannot inspect truth navigation record");
        return false;
    }
    if (!rtklib_copy_nav_record(state->truth_nav, truth_record_index, state->receiver_nav, error_message)) {
        return false;
    }

    const int receiver_record_index = find_receiver_record_index(state->receiver_nav, info);
    if (receiver_record_index < 0) {
        set_error(error_message, "cannot locate copied receiver navigation record");
        return false;
    }

    state->delivered_records[truth_record_index] = 1;
    if (event != nullptr) {
        event->availability_time = availability_time;
        event->truth_record_index = truth_record_index;
        event->receiver_record_index = receiver_record_index;
        event->kind = info.kind;
        event->satellite_number = info.satellite_number;
        event->system = info.system;
        event->prn = info.prn;
        event->message_type = info.message_type;
        event->iode = info.iode;
        event->iodc = info.iodc;
    }
    *emitted = true;
    return true;
}

const RtklibNavStore* truth_navigation_store(const NavigationState* state) {
    return state == nullptr ? nullptr : state->truth_nav;
}

const RtklibNavStore* receiver_navigation_store(const NavigationState* state) {
    return state == nullptr ? nullptr : state->receiver_nav;
}

int navigation_truth_record_count(const NavigationState* state) {
    return state == nullptr ? 0 : rtklib_nav_record_count(state->truth_nav);
}

bool navigation_truth_record_delivered(const NavigationState* state, int truth_record_index) {
    return state != nullptr && state->delivered_records != nullptr && truth_record_index >= 0 &&
           truth_record_index < state->delivered_record_count && state->delivered_records[truth_record_index] != 0;
}

} // namespace gnss_sim
