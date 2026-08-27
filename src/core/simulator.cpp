#include "gnss_sim/simulator.h"

#include "core/deterministic_rng.h"
#include "gnss/nav_message_scheduler.h"
#include "gnss/nav_output_record.h"
#include "gnss/navigation_state.h"
#include "gnss/satellite_engine.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_time.h"
#include "model/atmosphere_model.h"
#include "model/cn0_model.h"
#include "model/measurement_model.h"
#include "model/receiver_truth.h"
#include "model/signal_tracking.h"
#include "output/novatel_nav_writer.h"
#include "output/novatel_range_writer.h"
#include "output/novatel_solution_writer.h"
#include "scenario/scenario_engine.h"
#include "solution/solution_engine.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <new>
#include <string>
#include <vector>

#ifndef GNSS_SIM_RTKLIB_COMMIT
#define GNSS_SIM_RTKLIB_COMMIT "unknown"
#endif

namespace gnss_sim {
namespace {

constexpr double kRadiansToDegrees = 180.0 / 3.141592653589793238462643383279502884;

struct SignalRuntime {
    int satellite_index;
    const SignalDefinition* definition;
    SignalTracker tracker;
    CarrierAmbiguityState ambiguity;
    bool startup_pending;
};

struct SatelliteRuntime {
    int satellite_number;
    GnssConstellation constellation;
    SatelliteGeometry geometry;
    bool geometry_valid;
    int first_signal_index;
    int signal_count;
    int pending_truth_record_index;
    int active_truth_record_index;
    NavAcquisitionPlan nav_plan;
    bool nav_plan_active;
};

struct NavScheduleEntry {
    int truth_record_index;
    SimTime transmit_time;
    RtklibNavRecordKind kind;
    int satellite_number;
    int iode;
    NavOutputSystem system;
    RtklibBroadcastMessageFamily family;
    int prn;
};

struct SimulatorRuntime {
    NavigationState* navigation_state;
    ReceiverTruth receiver_truth;
    ScenarioEngine scenario_engine;
    SolutionEngineState solution_engine;
    DeterministicRng rng;
    Cn0Model cn0_model;
    SignalTrackingModelConfig tracking_config;
    ReceiverStartupTiming startup_timing;
    SimTime startup_search_ready_time;
    StartupMode current_startup_mode;
    std::vector<SatelliteRuntime> satellites;
    std::vector<SignalRuntime> signals;
    std::vector<MeasurementObservation> observations;
    std::vector<NavScheduleEntry> nav_schedule;
    std::size_t next_nav_schedule_index;
    int pending_ion_record[7];
};

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_request(const SimulationRequest& request, std::string* error_message) {
    if (request.rinex_nav_path == nullptr || request.rinex_nav_path[0] == '\0' || request.start_time.gps_week < 0 ||
        request.start_time.tow_ns < 0 || request.start_time.tow_ns >= GPS_WEEK_NANOSECONDS) {
        set_error(error_message, "simulation request has invalid NAV path or start time");
        return false;
    }
    if (!validate_sim_config(request.config, error_message)) {
        return false;
    }
    if (request.config.atmosphere_mode == AtmosphereMode::UNSPECIFIED) {
        set_error(error_message, "simulation requires an explicit atmosphere_mode of none or broadcast");
        return false;
    }
    return true;
}

bool constellation_from_satellite_id(const char satellite_id[4], GnssConstellation* constellation) {
    if (satellite_id == nullptr || constellation == nullptr) {
        return false;
    }
    switch (satellite_id[0]) {
        case 'G':
            *constellation = GnssConstellation::kGps;
            return true;
        case 'R':
            *constellation = GnssConstellation::kGlonass;
            return true;
        case 'E':
            *constellation = GnssConstellation::kGalileo;
            return true;
        case 'C':
            *constellation = GnssConstellation::kBeidou;
            return true;
        case 'J':
            *constellation = GnssConstellation::kQzss;
            return true;
        default:
            return false;
    }
}

NavOutputSystem output_system_for_constellation(GnssConstellation constellation) {
    switch (constellation) {
        case GnssConstellation::kGps:
            return NavOutputSystem::kGps;
        case GnssConstellation::kGlonass:
            return NavOutputSystem::kGlonass;
        case GnssConstellation::kGalileo:
            return NavOutputSystem::kGalileo;
        case GnssConstellation::kBeidou:
            return NavOutputSystem::kBeidou;
        case GnssConstellation::kQzss:
            return NavOutputSystem::kQzss;
    }
    return NavOutputSystem::kUnknown;
}

int output_system_index(NavOutputSystem system) {
    switch (system) {
        case NavOutputSystem::kUnknown:
            return 0;
        case NavOutputSystem::kGps:
            return 1;
        case NavOutputSystem::kGlonass:
            return 2;
        case NavOutputSystem::kGalileo:
            return 3;
        case NavOutputSystem::kBeidou:
            return 4;
        case NavOutputSystem::kQzss:
            return 5;
        case NavOutputSystem::kNavic:
            return 6;
    }
    return 0;
}

int find_satellite_index(const SimulatorRuntime& runtime, int satellite_number) {
    for (std::size_t index = 0; index < runtime.satellites.size(); ++index) {
        if (runtime.satellites[index].satellite_number == satellite_number) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

bool build_runtime_satellites(SimulatorRuntime* runtime, std::string* error_message) {
    const RtklibNavStore* truth_nav = truth_navigation_store(runtime->navigation_state);
    const int record_count = navigation_truth_record_count(runtime->navigation_state);
    std::vector<int> satellite_numbers;
    satellite_numbers.reserve(static_cast<std::size_t>(record_count));
    for (int record_index = 0; record_index < record_count; ++record_index) {
        RtklibNavRecordInfo info{};
        if (!rtklib_nav_record_info(truth_nav, record_index, &info)) {
            set_error(error_message, "cannot inspect truth NAV while building satellite runtime");
            return false;
        }
        if ((info.kind == RtklibNavRecordKind::kEphemeris || info.kind == RtklibNavRecordKind::kGlonassEphemeris) &&
            info.satellite_number > 0) {
            satellite_numbers.push_back(info.satellite_number);
        }
    }
    std::sort(satellite_numbers.begin(), satellite_numbers.end());
    satellite_numbers.erase(std::unique(satellite_numbers.begin(), satellite_numbers.end()), satellite_numbers.end());
    if (satellite_numbers.empty()) {
        set_error(error_message, "truth NAV contains no supported broadcast satellites");
        return false;
    }

    std::size_t definition_count = 0;
    const SignalDefinition* definitions = signal_definitions(&definition_count);
    runtime->satellites.reserve(satellite_numbers.size());
    runtime->signals.reserve(satellite_numbers.size() * definition_count);
    for (int satellite_number : satellite_numbers) {
        char satellite_id[4]{};
        GnssConstellation constellation{};
        if (!rtklib_satellite_number_to_id(satellite_number, satellite_id) ||
            !constellation_from_satellite_id(satellite_id, &constellation)) {
            continue;
        }

        SatelliteRuntime satellite{};
        satellite.satellite_number = satellite_number;
        satellite.constellation = constellation;
        satellite.first_signal_index = static_cast<int>(runtime->signals.size());
        satellite.pending_truth_record_index = -1;
        satellite.active_truth_record_index = -1;
        const int satellite_index = static_cast<int>(runtime->satellites.size());
        for (std::size_t definition_index = 0; definition_index < definition_count; ++definition_index) {
            if (definitions[definition_index].constellation != constellation) {
                continue;
            }
            SignalRuntime signal{};
            signal.satellite_index = satellite_index;
            signal.definition = definitions + definition_index;
            runtime->signals.push_back(signal);
            ++satellite.signal_count;
        }
        if (satellite.signal_count > 0) {
            runtime->satellites.push_back(satellite);
        } else {
            runtime->signals.resize(static_cast<std::size_t>(satellite.first_signal_index));
        }
    }
    if (runtime->satellites.empty() || runtime->signals.empty()) {
        set_error(error_message, "truth NAV contains no satellites in the frozen V1 signal set");
        return false;
    }
    runtime->observations.reserve(runtime->signals.size());
    return true;
}

bool nav_entry_from_record(const RtklibNavStore* truth_nav, int truth_record_index, NavScheduleEntry* entry,
                           std::string* error_message) {
    RtklibNavRecordInfo info{};
    NavOutputRecord output{};
    if (!rtklib_nav_record_info(truth_nav, truth_record_index, &info) ||
        !rtklib_nav_output_record(truth_nav, truth_record_index, &output, error_message)) {
        return false;
    }
    SimTime transmit_time{};
    if (!sim_time_from_week_sow(info.gps_week, info.transmit_sow_sec, &transmit_time)) {
        set_error(error_message, "truth NAV transmission time cannot be represented");
        return false;
    }

    NavScheduleEntry result{};
    result.truth_record_index = truth_record_index;
    result.transmit_time = transmit_time;
    result.kind = info.kind;
    result.satellite_number = info.satellite_number;
    result.iode = info.iode;
    result.prn = info.prn;
    if (info.kind == RtklibNavRecordKind::kEphemeris) {
        result.system = output.ephemeris.system;
        result.family = output.ephemeris.message_family;
    } else if (info.kind == RtklibNavRecordKind::kGlonassEphemeris) {
        result.system = NavOutputSystem::kGlonass;
        result.family = RtklibBroadcastMessageFamily::kGlonassFdma;
    } else {
        result.system = output.ionosphere.system;
        result.family = RtklibBroadcastMessageFamily::kUnknown;
    }
    *entry = result;
    return true;
}

bool build_nav_schedule(SimulatorRuntime* runtime, std::string* error_message) {
    const RtklibNavStore* truth_nav = truth_navigation_store(runtime->navigation_state);
    const int record_count = navigation_truth_record_count(runtime->navigation_state);
    runtime->nav_schedule.clear();
    runtime->nav_schedule.reserve(static_cast<std::size_t>(record_count));
    for (int record_index = 0; record_index < record_count; ++record_index) {
        NavScheduleEntry entry{};
        if (!nav_entry_from_record(truth_nav, record_index, &entry, error_message)) {
            return false;
        }
        runtime->nav_schedule.push_back(entry);
    }
    std::sort(runtime->nav_schedule.begin(), runtime->nav_schedule.end(), [](const NavScheduleEntry& lhs,
                                                                              const NavScheduleEntry& rhs) {
        const int comparison = compare_sim_time(lhs.transmit_time, rhs.transmit_time);
        return comparison < 0 || (comparison == 0 && lhs.truth_record_index < rhs.truth_record_index);
    });
    return true;
}

bool emit_line(const SimulationOutputSink& sink, SimulationLogKind kind, const SimTime& time, const std::string& line,
               std::string* error_message) {
    if (sink.write_line == nullptr) {
        return true;
    }
    if (!sink.write_line(kind, time, line.data(), line.size(), sink.user_data)) {
        set_error(error_message, "simulation output sink rejected a log record");
        return false;
    }
    return true;
}

bool should_emit_nav_kind(const SimConfig& config, RtklibNavRecordKind kind) {
    if (kind == RtklibNavRecordKind::kIonosphere) {
        return config.output_ion;
    }
    return config.output_eph;
}

bool emit_receiver_nav_record(const SimConfig& config, const RtklibNavStore* receiver_nav, int output_record_index,
                              const SimTime& time, const SimulationOutputSink& sink, SimulationRunStats* stats,
                              std::string* error_message) {
    NavOutputRecord record{};
    if (!rtklib_nav_output_record(receiver_nav, output_record_index, &record, error_message)) {
        return false;
    }
    if (!should_emit_nav_kind(config, record.kind)) {
        return true;
    }
    std::string line;
    bool supported = false;
    if (!format_novatel_receiver_nav_record(receiver_nav, output_record_index, time, &line, &supported, error_message)) {
        return false;
    }
    if (!supported) {
        return true;
    }
    if (!emit_line(sink, SimulationLogKind::kNavigation, time, line, error_message)) {
        return false;
    }
    ++stats->navigation_log_count;
    return true;
}

bool emit_startup_navigation(const SimConfig& config, const RtklibNavStore* receiver_nav, const SimTime& time,
                             const SimulationOutputSink& sink, SimulationRunStats* stats, std::string* error_message) {
    const int count = rtklib_nav_output_record_count(receiver_nav);
    for (int index = 0; index < count; ++index) {
        if (!emit_receiver_nav_record(config, receiver_nav, index, time, sink, stats, error_message)) {
            return false;
        }
    }
    return true;
}

bool emit_navigation_event(const SimConfig& config, const NavigationUpdateEvent& event, const RtklibNavStore* receiver_nav,
                           const SimulationOutputSink& sink, SimulationRunStats* stats, std::string* error_message) {
    ++stats->navigation_update_count;
    return emit_receiver_nav_record(config, receiver_nav, event.receiver_record_index, event.availability_time, sink,
                                    stats, error_message);
}

void reset_nav_cycle_state(SimulatorRuntime* runtime) {
    runtime->next_nav_schedule_index = 0U;
    for (int& record_index : runtime->pending_ion_record) {
        record_index = -1;
    }
    for (SatelliteRuntime& satellite : runtime->satellites) {
        satellite.pending_truth_record_index = -1;
        satellite.active_truth_record_index = -1;
        satellite.nav_plan_active = false;
    }
}

void set_pending_entry(SimulatorRuntime* runtime, const NavScheduleEntry& entry) {
    if (entry.kind == RtklibNavRecordKind::kIonosphere) {
        runtime->pending_ion_record[output_system_index(entry.system)] = entry.truth_record_index;
        return;
    }
    const int satellite_index = find_satellite_index(*runtime, entry.satellite_number);
    if (satellite_index < 0) {
        return;
    }
    SatelliteRuntime& satellite = runtime->satellites[static_cast<std::size_t>(satellite_index)];
    if (satellite.pending_truth_record_index != entry.truth_record_index) {
        satellite.pending_truth_record_index = entry.truth_record_index;
        if (satellite.active_truth_record_index != entry.truth_record_index) {
            satellite.nav_plan_active = false;
            satellite.active_truth_record_index = -1;
        }
    }
}

void reset_schedule_for_startup(SimulatorRuntime* runtime, const SimTime& startup_time, StartupMode startup_mode) {
    reset_nav_cycle_state(runtime);
    while (runtime->next_nav_schedule_index < runtime->nav_schedule.size() &&
           compare_sim_time(runtime->nav_schedule[runtime->next_nav_schedule_index].transmit_time, startup_time) <= 0) {
        if (startup_mode == StartupMode::COLD) {
            set_pending_entry(runtime, runtime->nav_schedule[runtime->next_nav_schedule_index]);
        }
        ++runtime->next_nav_schedule_index;
    }
}

AcquisitionContext startup_acquisition_context(StartupMode mode) {
    switch (mode) {
        case StartupMode::HOT:
            return AcquisitionContext::kHot;
        case StartupMode::WARM:
            return AcquisitionContext::kWarm;
        case StartupMode::COLD:
            return AcquisitionContext::kCold;
    }
    return AcquisitionContext::kHot;
}

bool start_receiver_cycle(SimulatorRuntime* runtime, const ScenarioEpochState& scenario, const SimulationRequest& request,
                          const SimulationOutputSink& sink, SimulationRunStats* stats, std::string* error_message) {
    runtime->current_startup_mode = scenario.startup_mode;
    if (!initialize_receiver_navigation(runtime->navigation_state, scenario.startup_mode, scenario.time, error_message)) {
        return false;
    }
    reset_solution_engine_state(&runtime->solution_engine);
    if (!sample_receiver_startup_timing(scenario.startup_mode, runtime->tracking_config, &runtime->rng,
                                        &runtime->startup_timing)) {
        set_error(error_message, "cannot sample deterministic receiver startup timing");
        return false;
    }
    if (!add_time_ns(scenario.time, runtime->startup_timing.total_search_ready_delay_ns,
                     &runtime->startup_search_ready_time)) {
        set_error(error_message, "receiver startup timing overflows simulation time");
        return false;
    }
    for (SignalRuntime& signal : runtime->signals) {
        reset_signal_tracker(&signal.tracker, signal.definition->signal_id, scenario.time);
        reset_carrier_ambiguity_state(&signal.ambiguity);
        signal.startup_pending = true;
    }
    reset_schedule_for_startup(runtime, scenario.time, scenario.startup_mode);
    if (scenario.startup_mode != StartupMode::COLD &&
        !emit_startup_navigation(request.config, receiver_navigation_store(runtime->navigation_state), scenario.time, sink,
                                 stats, error_message)) {
        return false;
    }
    return true;
}

bool geometry_for_epoch(SimulatorRuntime* runtime, const SimConfig& config, const SimTime& time) {
    const RtklibNavStore* truth_nav = truth_navigation_store(runtime->navigation_state);
    for (SatelliteRuntime& satellite : runtime->satellites) {
        std::string ignored_error;
        satellite.geometry_valid = compute_satellite_geometry(truth_nav, runtime->receiver_truth, time,
                                                              satellite.satellite_number, config.elevation_mask_deg,
                                                              &satellite.geometry, &ignored_error);
    }
    return true;
}

bool schedule_and_update_tracking(SimulatorRuntime* runtime, const ScenarioEpochState& scenario,
                                  std::string* error_message) {
    for (SignalRuntime& signal : runtime->signals) {
        SatelliteRuntime& satellite = runtime->satellites[static_cast<std::size_t>(signal.satellite_index)];
        const double elevation_deg = satellite.geometry_valid ? satellite.geometry.elevation_rad * kRadiansToDegrees
                                                              : -90.0;
        const bool available = scenario.signal_available && satellite.geometry_valid && satellite.geometry.visible;
        if (available && !signal.tracker.scheduled) {
            AcquisitionContext context = AcquisitionContext::kReacquisition;
            SimTime search_ready_time = scenario.time;
            if (signal.startup_pending) {
                context = startup_acquisition_context(runtime->current_startup_mode);
                search_ready_time = compare_sim_time(runtime->startup_search_ready_time, scenario.time) > 0
                                        ? runtime->startup_search_ready_time
                                        : scenario.time;
                signal.startup_pending = false;
            }
            if (!schedule_signal_acquisition(&signal.tracker, context, scenario.time, search_ready_time, elevation_deg,
                                             runtime->cn0_model, runtime->tracking_config, &runtime->rng,
                                             error_message)) {
                return false;
            }
        }
        if (!update_signal_tracker(&signal.tracker, scenario.time, available, elevation_deg, runtime->cn0_model,
                                   error_message)) {
            return false;
        }
        if (!available) {
            reset_carrier_ambiguity_state(&signal.ambiguity);
        }
    }
    return true;
}

bool family_matches_signal(const NavScheduleEntry& entry, const SignalDefinition& signal) {
    switch (entry.family) {
        case RtklibBroadcastMessageFamily::kLegacy:
            if (entry.system == NavOutputSystem::kGps) {
                return signal.nav_message_family == NavMessageFamily::kGpsLnav;
            }
            if (entry.system == NavOutputSystem::kQzss) {
                return signal.nav_message_family == NavMessageFamily::kQzssLnav;
            }
            if (entry.system == NavOutputSystem::kBeidou) {
                return signal.nav_message_family == NavMessageFamily::kBeidouD1D2;
            }
            return false;
        case RtklibBroadcastMessageFamily::kCnav:
            return (entry.system == NavOutputSystem::kGps && signal.nav_message_family == NavMessageFamily::kGpsCnav) ||
                   (entry.system == NavOutputSystem::kQzss && signal.nav_message_family == NavMessageFamily::kQzssCnav);
        case RtklibBroadcastMessageFamily::kCnav2:
            return (entry.system == NavOutputSystem::kGps && signal.nav_message_family == NavMessageFamily::kGpsCnav2) ||
                   (entry.system == NavOutputSystem::kQzss && signal.nav_message_family == NavMessageFamily::kQzssCnav2);
        case RtklibBroadcastMessageFamily::kGalileoInav:
            return signal.nav_message_family == NavMessageFamily::kGalileoInav;
        case RtklibBroadcastMessageFamily::kGalileoFnav:
            return signal.nav_message_family == NavMessageFamily::kGalileoFnav;
        case RtklibBroadcastMessageFamily::kBeidouBcnav1:
            return signal.nav_message_family == NavMessageFamily::kBeidouBcnav1;
        case RtklibBroadcastMessageFamily::kBeidouBcnav2:
            return signal.nav_message_family == NavMessageFamily::kBeidouBcnav2;
        case RtklibBroadcastMessageFamily::kBeidouBcnav3:
            return signal.nav_message_family == NavMessageFamily::kBeidouBcnav3;
        case RtklibBroadcastMessageFamily::kGlonassFdma:
            return signal.nav_message_family == NavMessageFamily::kGlonassFdma;
        case RtklibBroadcastMessageFamily::kUnknown:
            return false;
    }
    return false;
}

const SignalRuntime* tracked_nav_signal(const SimulatorRuntime& runtime, const SatelliteRuntime& satellite,
                                        const NavScheduleEntry& entry) {
    const SignalRuntime* fallback = nullptr;
    for (int offset = 0; offset < satellite.signal_count; ++offset) {
        const SignalRuntime& signal = runtime.signals[static_cast<std::size_t>(satellite.first_signal_index + offset)];
        if (signal.tracker.phase != SignalTrackingPhase::kTracking) {
            continue;
        }
        if (fallback == nullptr || signal_single_point_priority(signal.definition->signal_id) >= 0) {
            fallback = &signal;
        }
        if (family_matches_signal(entry, *signal.definition)) {
            return &signal;
        }
    }
    return fallback;
}

NavScheduleVariant schedule_variant(const NavScheduleEntry& entry) {
    if (entry.system == NavOutputSystem::kBeidou && entry.family == RtklibBroadcastMessageFamily::kLegacy) {
        return entry.prn >= 1 && entry.prn <= 5 ? NavScheduleVariant::kBeidouD2 : NavScheduleVariant::kBeidouD1;
    }
    return NavScheduleVariant::kDefault;
}

const NavScheduleEntry* schedule_entry_by_truth_index(const SimulatorRuntime& runtime, int truth_record_index) {
    for (const NavScheduleEntry& entry : runtime.nav_schedule) {
        if (entry.truth_record_index == truth_record_index) {
            return &entry;
        }
    }
    return nullptr;
}

bool any_tracked_signal_for_system(const SimulatorRuntime& runtime, NavOutputSystem system) {
    for (const SignalRuntime& signal : runtime.signals) {
        const SatelliteRuntime& satellite = runtime.satellites[static_cast<std::size_t>(signal.satellite_index)];
        if (output_system_for_constellation(satellite.constellation) == system &&
            signal.tracker.phase == SignalTrackingPhase::kTracking) {
            return true;
        }
    }
    return false;
}

bool deliver_record_direct(SimulatorRuntime* runtime, int truth_record_index, const SimTime& time,
                           const SimulationRequest& request, const SimulationOutputSink& sink,
                           SimulationRunStats* stats, std::string* error_message) {
    NavigationUpdateEvent event{};
    bool emitted = false;
    if (!apply_truth_navigation_record(runtime->navigation_state, truth_record_index, time, &event, &emitted,
                                       error_message)) {
        return false;
    }
    if (emitted && !emit_navigation_event(request.config, event, receiver_navigation_store(runtime->navigation_state),
                                          sink, stats, error_message)) {
        return false;
    }
    return true;
}

bool queue_due_navigation(SimulatorRuntime* runtime, const ScenarioEpochState& scenario, const SimulationRequest& request,
                          const SimulationOutputSink& sink, SimulationRunStats* stats, std::string* error_message) {
    if (!scenario.signal_available) {
        return true;
    }
    while (runtime->next_nav_schedule_index < runtime->nav_schedule.size() &&
           compare_sim_time(runtime->nav_schedule[runtime->next_nav_schedule_index].transmit_time, scenario.time) <= 0) {
        const NavScheduleEntry& entry = runtime->nav_schedule[runtime->next_nav_schedule_index];
        if (runtime->current_startup_mode == StartupMode::COLD) {
            set_pending_entry(runtime, entry);
        } else if (!deliver_record_direct(runtime, entry.truth_record_index, scenario.time, request, sink, stats,
                                          error_message)) {
            return false;
        }
        ++runtime->next_nav_schedule_index;
    }
    return true;
}

bool process_cold_navigation(SimulatorRuntime* runtime, const ScenarioEpochState& scenario,
                             const SimulationRequest& request, const SimulationOutputSink& sink,
                             SimulationRunStats* stats, std::string* error_message) {
    if (runtime->current_startup_mode != StartupMode::COLD || !scenario.signal_available) {
        return true;
    }

    for (int system_index = 0; system_index < 7; ++system_index) {
        const int truth_record_index = runtime->pending_ion_record[system_index];
        if (truth_record_index < 0) {
            continue;
        }
        const NavScheduleEntry* entry = schedule_entry_by_truth_index(*runtime, truth_record_index);
        if (entry != nullptr && any_tracked_signal_for_system(*runtime, entry->system)) {
            if (!deliver_record_direct(runtime, truth_record_index, scenario.time, request, sink, stats, error_message)) {
                return false;
            }
            runtime->pending_ion_record[system_index] = -1;
        }
    }

    for (SatelliteRuntime& satellite : runtime->satellites) {
        if (satellite.pending_truth_record_index < 0) {
            continue;
        }
        const NavScheduleEntry* entry = schedule_entry_by_truth_index(*runtime, satellite.pending_truth_record_index);
        if (entry == nullptr) {
            set_error(error_message, "pending cold NAV record is missing from schedule");
            return false;
        }
        const SignalRuntime* nav_signal = tracked_nav_signal(*runtime, satellite, *entry);
        if (nav_signal == nullptr) {
            continue;
        }

        if (!satellite.nav_plan_active || satellite.active_truth_record_index != entry->truth_record_index) {
            const SimTime acquisition_time = compare_sim_time(nav_signal->tracker.tracking_start_time, entry->transmit_time) > 0
                                                 ? nav_signal->tracker.tracking_start_time
                                                 : entry->transmit_time;
            if (!build_cold_nav_acquisition_plan_with_variant(nav_signal->definition->signal_id, schedule_variant(*entry),
                                                              acquisition_time, entry->iode, &satellite.nav_plan,
                                                              error_message)) {
                return false;
            }
            satellite.active_truth_record_index = entry->truth_record_index;
            satellite.nav_plan_active = true;
        }

        NavigationUpdateEvent event{};
        bool emitted = false;
        if (!deliver_cold_nav_plan_if_complete(satellite.nav_plan, satellite.active_truth_record_index, scenario.time,
                                               runtime->navigation_state, &event, &emitted, error_message)) {
            return false;
        }
        if (emitted) {
            if (!emit_navigation_event(request.config, event, receiver_navigation_store(runtime->navigation_state), sink,
                                       stats, error_message)) {
                return false;
            }
            satellite.pending_truth_record_index = -1;
            satellite.active_truth_record_index = -1;
            satellite.nav_plan_active = false;
        }
    }
    return true;
}

bool generate_epoch_observations(SimulatorRuntime* runtime, const ScenarioEpochState& scenario,
                                 const SimulationRequest& request, std::string* error_message) {
    runtime->observations.clear();
    if (!scenario.signal_available) {
        return true;
    }
    const RtklibNavStore* truth_nav = truth_navigation_store(runtime->navigation_state);
    for (SignalRuntime& signal : runtime->signals) {
        const SatelliteRuntime& satellite = runtime->satellites[static_cast<std::size_t>(signal.satellite_index)];
        if (!satellite.geometry_valid || signal.tracker.phase != SignalTrackingPhase::kTracking) {
            continue;
        }
        AtmosphereCorrection atmosphere{};
        if (!compute_atmosphere_correction(request.config.atmosphere_mode, truth_nav, scenario.time,
                                           signal.definition->signal_id, 0, runtime->receiver_truth.position_ecef_m,
                                           satellite.geometry.azimuth_rad, satellite.geometry.elevation_rad, &atmosphere,
                                           error_message)) {
            return false;
        }
        MeasurementObservation observation{};
        if (!generate_zero_noise_measurement(truth_nav, satellite.geometry, signal.tracker, atmosphere,
                                             &signal.ambiguity, &observation, error_message)) {
            return false;
        }
        runtime->observations.push_back(observation);
    }
    return true;
}

int tracked_satellite_count(const SimulatorRuntime& runtime) {
    int count = 0;
    for (const SatelliteRuntime& satellite : runtime.satellites) {
        bool tracked = false;
        for (int offset = 0; offset < satellite.signal_count; ++offset) {
            const SignalRuntime& signal = runtime.signals[static_cast<std::size_t>(satellite.first_signal_index + offset)];
            if (signal.tracker.observation_available) {
                tracked = true;
                break;
            }
        }
        if (tracked) {
            ++count;
        }
    }
    return count;
}

bool emit_receiver_epoch_logs(SimulatorRuntime* runtime, const ScenarioEpochState& scenario,
                              const SimulationRequest& request, const SimulationOutputSink& sink,
                              SimulationRunStats* stats, std::string* error_message) {
    std::string line;
    const MeasurementObservation* observation_data = runtime->observations.empty() ? nullptr : runtime->observations.data();
    const int observation_count = static_cast<int>(runtime->observations.size());
    if (!format_novatel_rangea(scenario.time, observation_data, observation_count, &line, error_message) ||
        !emit_line(sink, SimulationLogKind::kRange, scenario.time, line, error_message)) {
        return false;
    }
    ++stats->range_log_count;

    SolutionEpoch solution{};
    if (!solve_receiver_epoch(receiver_navigation_store(runtime->navigation_state), scenario.time, observation_data,
                              observation_count, request.config.elevation_mask_deg, request.config.atmosphere_mode,
                              &runtime->solution_engine, &solution, error_message)) {
        return false;
    }
    if (!format_novatel_psrposa(solution, tracked_satellite_count(*runtime), &line, error_message) ||
        !emit_line(sink, SimulationLogKind::kPosition, scenario.time, line, error_message)) {
        return false;
    }
    ++stats->position_log_count;
    if (!format_novatel_psrvela(solution, &line, error_message) ||
        !emit_line(sink, SimulationLogKind::kVelocity, scenario.time, line, error_message)) {
        return false;
    }
    ++stats->velocity_log_count;
    return true;
}

bool initialize_runtime(const SimulationRequest& request, SimulatorRuntime* runtime, std::string* error_message) {
    runtime->navigation_state = create_navigation_state();
    if (runtime->navigation_state == nullptr) {
        set_error(error_message, "cannot allocate navigation state");
        return false;
    }
    if (!load_truth_navigation(runtime->navigation_state, request.rinex_nav_path, error_message) ||
        !make_static_receiver_truth(request.config.receiver, &runtime->receiver_truth, error_message) ||
        !initialize_scenario_engine(request.config, request.start_time, &runtime->scenario_engine, error_message) ||
        !build_runtime_satellites(runtime, error_message) || !build_nav_schedule(runtime, error_message)) {
        return false;
    }
    seed_rng(&runtime->rng, request.config.seed);
    runtime->cn0_model = make_builtin_cn0_model(request.config.seed);
    runtime->tracking_config = default_signal_tracking_model_config();
    if (!validate_signal_tracking_model_config(runtime->tracking_config, error_message)) {
        return false;
    }
    reset_solution_engine_state(&runtime->solution_engine);
    reset_nav_cycle_state(runtime);
    return true;
}

void destroy_runtime(SimulatorRuntime* runtime) {
    if (runtime != nullptr) {
        destroy_navigation_state(runtime->navigation_state);
        runtime->navigation_state = nullptr;
    }
}

} // namespace

bool run_simulation(const SimulationRequest& request, const SimulationOutputSink& sink, SimulationRunStats* stats,
                    std::string* error_message) {
    if (stats == nullptr || !valid_request(request, error_message)) {
        if (stats == nullptr) {
            set_error(error_message, "simulation stats output must not be null");
        }
        return false;
    }
    *stats = {};

    SimulatorRuntime* runtime = new (std::nothrow) SimulatorRuntime{};
    if (runtime == nullptr) {
        set_error(error_message, "cannot allocate simulator runtime");
        return false;
    }
    if (!initialize_runtime(request, runtime, error_message)) {
        destroy_runtime(runtime);
        delete runtime;
        return false;
    }
    stats->runtime_signal_count = static_cast<std::uint64_t>(runtime->signals.size());

    std::uint64_t epoch_count = 0U;
    if (!epoch_count_for_duration(request.config.duration_ns, request.config.sampling_rate_hz, &epoch_count)) {
        set_error(error_message, "cannot determine simulation epoch count");
        destroy_runtime(runtime);
        delete runtime;
        return false;
    }

    bool success = true;
    for (std::uint64_t epoch_index = 0; epoch_index < epoch_count; ++epoch_index) {
        SimTime epoch_time{};
        ScenarioEpochState scenario{};
        if (!epoch_time_at_index(request.start_time, request.config.sampling_rate_hz, epoch_index, &epoch_time) ||
            !update_scenario_engine(&runtime->scenario_engine, epoch_time, &scenario, error_message)) {
            success = false;
            break;
        }
        ++stats->total_epochs;
        stats->power_on_event_count += scenario.power_on_event ? 1U : 0U;
        stats->power_off_event_count += scenario.power_off_event ? 1U : 0U;
        stats->signal_on_event_count += scenario.signal_on_event ? 1U : 0U;
        stats->signal_off_event_count += scenario.signal_off_event ? 1U : 0U;
        stats->startup_event_count += scenario.startup_event ? 1U : 0U;

        if (scenario.startup_event &&
            !start_receiver_cycle(runtime, scenario, request, sink, stats, error_message)) {
            success = false;
            break;
        }
        if (!scenario.powered) {
            for (SignalRuntime& signal : runtime->signals) {
                if (!update_signal_tracker(&signal.tracker, scenario.time, false, -90.0, runtime->cn0_model,
                                           error_message)) {
                    success = false;
                    break;
                }
                reset_carrier_ambiguity_state(&signal.ambiguity);
            }
            if (!success) {
                break;
            }
            continue;
        }
        ++stats->powered_epochs;

        if (!geometry_for_epoch(runtime, request.config, scenario.time) ||
            !schedule_and_update_tracking(runtime, scenario, error_message) ||
            !queue_due_navigation(runtime, scenario, request, sink, stats, error_message) ||
            !process_cold_navigation(runtime, scenario, request, sink, stats, error_message) ||
            !generate_epoch_observations(runtime, scenario, request, error_message)) {
            success = false;
            break;
        }
        stats->maximum_observations_in_epoch =
            std::max(stats->maximum_observations_in_epoch, static_cast<std::uint64_t>(runtime->observations.size()));
        if (!emit_receiver_epoch_logs(runtime, scenario, request, sink, stats, error_message)) {
            success = false;
            break;
        }
    }

    destroy_runtime(runtime);
    delete runtime;
    return success;
}

const char* simulation_log_kind_name(SimulationLogKind kind) {
    switch (kind) {
        case SimulationLogKind::kRange:
            return "RANGE";
        case SimulationLogKind::kPosition:
            return "PSRPOS";
        case SimulationLogKind::kVelocity:
            return "PSRVEL";
        case SimulationLogKind::kNavigation:
            return "NAV";
    }
    return "UNKNOWN";
}

const char* simulator_version() {
    return "0.1.0-dev";
}

const char* rtklib_commit_sha() {
    return GNSS_SIM_RTKLIB_COMMIT;
}

} // namespace gnss_sim
