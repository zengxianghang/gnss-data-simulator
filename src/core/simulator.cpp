#include "gnss_sim/simulator.h"

#include "core/deterministic_rng.h"
#include "gnss/nav_message_scheduler.h"
#include "gnss/nav_output_record.h"
#include "gnss/navigation_state.h"
#include "gnss/rtklib_adapter.h"
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
#include "output/truth_writer.h"
#include "scenario/scenario_engine.h"
#include "solution/solution_engine.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

#ifndef GNSS_SIM_RTKLIB_COMMIT
#define GNSS_SIM_RTKLIB_COMMIT "unknown"
#endif
#ifndef GNSS_SIM_SOURCE_COMMIT
#define GNSS_SIM_SOURCE_COMMIT "unknown"
#endif

namespace gnss_sim {
namespace {

constexpr double kRadiansToDegrees = 180.0 / 3.141592653589793238462643383279502884;

struct SignalRuntime {
    SignalTracker tracker;
    CarrierAmbiguityState ambiguity;
    bool ever_scheduled;
};

struct ColdFamilyRuntime {
    NavMessageFamily family;
    bool acquired;
    bool plan_active;
    int target_truth_record_index;
    NavAcquisitionPlan plan;
};

struct SatelliteRuntime {
    int satellite_number;
    GnssConstellation constellation;
    std::vector<SignalRuntime> signals;
    std::vector<ColdFamilyRuntime> cold_families;
};

struct TruthScheduleEntry {
    int truth_record_index;
    RtklibNavRecordInfo info;
    SimTime transmit_time;
    bool has_family;
    NavMessageFamily family;
};

struct RuntimeState {
    NavigationState* navigation;
    ReceiverTruth receiver;
    DeterministicRng rng;
    Cn0Model cn0_model;
    SignalTrackingModelConfig tracking_config;
    SolutionEngineState solution_state;
    StartupMode startup_mode;
    SimTime startup_search_ready_time;
    std::vector<SatelliteRuntime> satellites;
    std::vector<TruthScheduleEntry> truth_schedule;
};

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool valid_run_options(const SimulatorRunOptions& options) {
    return options.rinex_nav_path != nullptr && options.rinex_nav_path[0] != '\0' &&
           options.output_log_path != nullptr && options.output_log_path[0] != '\0' &&
           options.start_time.gps_week >= 0 && options.start_time.tow_ns >= 0 &&
           options.start_time.tow_ns < GPS_WEEK_NANOSECONDS;
}

bool write_message(std::ofstream* output, const std::string& message, std::string* error_message) {
    if (output == nullptr || !output->is_open()) {
        set_error(error_message, "simulator output stream is not open");
        return false;
    }
    output->write(message.data(), static_cast<std::streamsize>(message.size()));
    if (!*output) {
        set_error(error_message, "failed to write simulated receiver log");
        return false;
    }
    return true;
}

bool constellation_from_satellite(int satellite_number, GnssConstellation* constellation) {
    if (constellation == nullptr) {
        return false;
    }
    char satellite_id[4]{};
    if (!rtklib_satellite_number_to_id(satellite_number, satellite_id)) {
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

bool nav_family_from_record(const NavOutputRecord& record, NavMessageFamily* family) {
    if (family == nullptr) {
        return false;
    }
    if (record.kind == RtklibNavRecordKind::kGlonassEphemeris) {
        *family = NavMessageFamily::kGlonassFdma;
        return true;
    }
    if (record.kind != RtklibNavRecordKind::kEphemeris) {
        return false;
    }

    const KeplerianNavOutputData& eph = record.ephemeris;
    switch (eph.message_family) {
        case RtklibBroadcastMessageFamily::kLegacy:
            if (eph.system == NavOutputSystem::kGps) {
                *family = NavMessageFamily::kGpsLnav;
                return true;
            }
            if (eph.system == NavOutputSystem::kQzss) {
                *family = NavMessageFamily::kQzssLnav;
                return true;
            }
            if (eph.system == NavOutputSystem::kBeidou) {
                *family = NavMessageFamily::kBeidouD1D2;
                return true;
            }
            return false;
        case RtklibBroadcastMessageFamily::kCnav:
            if (eph.system == NavOutputSystem::kGps) {
                *family = NavMessageFamily::kGpsCnav;
                return true;
            }
            if (eph.system == NavOutputSystem::kQzss) {
                *family = NavMessageFamily::kQzssCnav;
                return true;
            }
            return false;
        case RtklibBroadcastMessageFamily::kCnav2:
            if (eph.system == NavOutputSystem::kGps) {
                *family = NavMessageFamily::kGpsCnav2;
                return true;
            }
            if (eph.system == NavOutputSystem::kQzss) {
                *family = NavMessageFamily::kQzssCnav2;
                return true;
            }
            return false;
        case RtklibBroadcastMessageFamily::kGalileoInav:
            *family = NavMessageFamily::kGalileoInav;
            return true;
        case RtklibBroadcastMessageFamily::kGalileoFnav:
            *family = NavMessageFamily::kGalileoFnav;
            return true;
        case RtklibBroadcastMessageFamily::kBeidouBcnav1:
            *family = NavMessageFamily::kBeidouBcnav1;
            return true;
        case RtklibBroadcastMessageFamily::kBeidouBcnav2:
            *family = NavMessageFamily::kBeidouBcnav2;
            return true;
        case RtklibBroadcastMessageFamily::kBeidouBcnav3:
            *family = NavMessageFamily::kBeidouBcnav3;
            return true;
        case RtklibBroadcastMessageFamily::kGlonassFdma:
            *family = NavMessageFamily::kGlonassFdma;
            return true;
        case RtklibBroadcastMessageFamily::kUnknown:
            return false;
    }
    return false;
}

bool build_truth_schedule(const RtklibNavStore* truth_nav, std::vector<TruthScheduleEntry>* schedule,
                          std::string* error_message) {
    if (truth_nav == nullptr || schedule == nullptr) {
        set_error(error_message, "truth NAV schedule request has invalid arguments");
        return false;
    }
    schedule->clear();
    const int count = rtklib_nav_record_count(truth_nav);
    schedule->reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
        TruthScheduleEntry entry{};
        entry.truth_record_index = index;
        if (!rtklib_nav_record_info(truth_nav, index, &entry.info) ||
            !sim_time_from_week_sow(entry.info.gps_week, entry.info.transmit_sow_sec, &entry.transmit_time)) {
            set_error(error_message, "cannot build deterministic truth NAV transmission schedule");
            return false;
        }
        if (entry.info.kind != RtklibNavRecordKind::kIonosphere) {
            NavOutputRecord output_record{};
            if (!rtklib_nav_output_record(truth_nav, index, &output_record, error_message)) {
                return false;
            }
            entry.has_family = nav_family_from_record(output_record, &entry.family);
        }
        schedule->push_back(entry);
    }
    std::sort(schedule->begin(), schedule->end(), [](const TruthScheduleEntry& lhs, const TruthScheduleEntry& rhs) {
        const int time_order = compare_sim_time(lhs.transmit_time, rhs.transmit_time);
        if (time_order != 0) {
            return time_order < 0;
        }
        return lhs.truth_record_index < rhs.truth_record_index;
    });
    return true;
}

ColdFamilyRuntime* cold_family_state(SatelliteRuntime* satellite, NavMessageFamily family) {
    for (ColdFamilyRuntime& state : satellite->cold_families) {
        if (state.family == family) {
            return &state;
        }
    }
    ColdFamilyRuntime state{};
    state.family = family;
    state.target_truth_record_index = -1;
    satellite->cold_families.push_back(state);
    return &satellite->cold_families.back();
}

bool build_satellite_runtimes(const std::vector<TruthScheduleEntry>& schedule, const SimTime& reset_time,
                              std::vector<SatelliteRuntime>* satellites, std::string* error_message) {
    if (satellites == nullptr) {
        set_error(error_message, "satellite runtime output is null");
        return false;
    }
    std::vector<int> satellite_numbers;
    for (const TruthScheduleEntry& entry : schedule) {
        if (entry.info.satellite_number > 0 && std::find(satellite_numbers.begin(), satellite_numbers.end(),
                                                         entry.info.satellite_number) == satellite_numbers.end()) {
            satellite_numbers.push_back(entry.info.satellite_number);
        }
    }
    std::sort(satellite_numbers.begin(), satellite_numbers.end());
    satellites->clear();
    satellites->reserve(satellite_numbers.size());
    std::size_t definition_count = 0;
    const SignalDefinition* definitions = signal_definitions(&definition_count);
    for (int satellite_number : satellite_numbers) {
        SatelliteRuntime satellite{};
        satellite.satellite_number = satellite_number;
        if (!constellation_from_satellite(satellite_number, &satellite.constellation)) {
            continue;
        }
        for (std::size_t index = 0; index < definition_count; ++index) {
            if (definitions[index].constellation != satellite.constellation) {
                continue;
            }
            SignalRuntime signal{};
            reset_signal_tracker(&signal.tracker, definitions[index].signal_id, reset_time);
            reset_carrier_ambiguity_state(&signal.ambiguity);
            satellite.signals.push_back(signal);
        }
        for (const TruthScheduleEntry& entry : schedule) {
            if (entry.info.satellite_number == satellite_number && entry.has_family) {
                cold_family_state(&satellite, entry.family);
            }
        }
        satellites->push_back(satellite);
    }
    if (satellites->empty()) {
        set_error(error_message, "RINEX NAV contains no V1-supported constellation satellites");
        return false;
    }
    return true;
}

SatelliteRuntime* find_satellite(std::vector<SatelliteRuntime>* satellites, int satellite_number) {
    for (SatelliteRuntime& satellite : *satellites) {
        if (satellite.satellite_number == satellite_number) {
            return &satellite;
        }
    }
    return nullptr;
}

bool any_tracking(const std::vector<SatelliteRuntime>& satellites) {
    for (const SatelliteRuntime& satellite : satellites) {
        for (const SignalRuntime& signal : satellite.signals) {
            if (signal.tracker.phase == SignalTrackingPhase::kTracking) {
                return true;
            }
        }
    }
    return false;
}

bool family_tracking(const SatelliteRuntime& satellite, NavMessageFamily family) {
    for (const SignalRuntime& signal : satellite.signals) {
        const SignalDefinition* definition = find_signal_definition(signal.tracker.signal_id);
        if (definition != nullptr && definition->nav_message_family == family &&
            signal.tracker.phase == SignalTrackingPhase::kTracking) {
            return true;
        }
    }
    return false;
}

bool record_output_enabled(const SimConfig& config, const NavOutputRecord& record) {
    if (record.kind == RtklibNavRecordKind::kIonosphere) {
        return config.output_ion;
    }
    return config.output_eph;
}

bool emit_receiver_nav_record(const SimConfig& config, const RtklibNavStore* receiver_nav, int receiver_record_index,
                              const SimTime& output_time, std::ofstream* output, SimulatorRunSummary* summary,
                              std::string* error_message) {
    NavOutputRecord record{};
    if (!rtklib_nav_output_record(receiver_nav, receiver_record_index, &record, error_message)) {
        return false;
    }
    if (!record_output_enabled(config, record)) {
        return true;
    }
    std::string message;
    bool supported = false;
    if (!format_novatel_nav_output_record(record, output_time, &message, &supported, error_message)) {
        return false;
    }
    if (supported) {
        if (!write_message(output, message, error_message)) {
            return false;
        }
        ++summary->nav_messages;
    }
    return true;
}

bool emit_receiver_nav_snapshot(const SimConfig& config, const RtklibNavStore* receiver_nav, const SimTime& output_time,
                                std::ofstream* output, SimulatorRunSummary* summary, std::string* error_message) {
    const int count = rtklib_nav_output_record_count(receiver_nav);
    for (int index = 0; index < count; ++index) {
        if (!emit_receiver_nav_record(config, receiver_nav, index, output_time, output, summary, error_message)) {
            return false;
        }
    }
    return true;
}

StartupMode scenario_startup_mode(const SimConfig& config) {
    return config.scenario == ScenarioType::TTFF ? config.ttff.startup_mode : StartupMode::HOT;
}

AcquisitionContext startup_context(StartupMode mode) {
    switch (mode) {
        case StartupMode::WARM:
            return AcquisitionContext::kWarm;
        case StartupMode::COLD:
            return AcquisitionContext::kCold;
        case StartupMode::HOT:
            return AcquisitionContext::kHot;
    }
    return AcquisitionContext::kHot;
}

bool receiver_power_on(RuntimeState* runtime, const SimConfig& config, const SimTime& power_on_time,
                       std::ofstream* output, SimulatorRunSummary* summary, std::string* error_message) {
    runtime->startup_mode = scenario_startup_mode(config);
    if (!initialize_receiver_navigation(runtime->navigation, runtime->startup_mode, power_on_time, error_message)) {
        return false;
    }
    reset_solution_engine_state(&runtime->solution_state);
    ReceiverStartupTiming timing{};
    if (!sample_receiver_startup_timing(runtime->startup_mode, runtime->tracking_config, &runtime->rng, &timing) ||
        !add_time_ns(power_on_time, timing.total_search_ready_delay_ns, &runtime->startup_search_ready_time)) {
        set_error(error_message, "cannot schedule deterministic receiver startup timing");
        return false;
    }
    for (SatelliteRuntime& satellite : runtime->satellites) {
        for (SignalRuntime& signal : satellite.signals) {
            reset_signal_tracker(&signal.tracker, signal.tracker.signal_id, power_on_time);
            reset_carrier_ambiguity_state(&signal.ambiguity);
            signal.ever_scheduled = false;
        }
        for (ColdFamilyRuntime& family : satellite.cold_families) {
            family.acquired = false;
            family.plan_active = false;
            family.target_truth_record_index = -1;
            family.plan = NavAcquisitionPlan{};
        }
    }
    if (runtime->startup_mode != StartupMode::COLD) {
        return emit_receiver_nav_snapshot(config, receiver_navigation_store(runtime->navigation), power_on_time, output,
                                          summary, error_message);
    }
    return true;
}

void receiver_power_off(RuntimeState* runtime, const SimTime& time) {
    reset_solution_engine_state(&runtime->solution_state);
    for (SatelliteRuntime& satellite : runtime->satellites) {
        for (SignalRuntime& signal : satellite.signals) {
            reset_signal_tracker(&signal.tracker, signal.tracker.signal_id, time);
            reset_carrier_ambiguity_state(&signal.ambiguity);
        }
    }
}

void receiver_signal_off(RuntimeState* runtime, const SimTime& time) {
    for (SatelliteRuntime& satellite : runtime->satellites) {
        for (SignalRuntime& signal : satellite.signals) {
            static_cast<void>(update_signal_tracker(&signal.tracker, time, false, 0.0, runtime->cn0_model, nullptr));
            reset_carrier_ambiguity_state(&signal.ambiguity);
        }
    }
}

bool record_is_available(const TruthScheduleEntry& entry, const SimTime& current_time) {
    return compare_sim_time(entry.transmit_time, current_time) <= 0;
}

bool apply_and_emit_nav(RuntimeState* runtime, const SimConfig& config, const TruthScheduleEntry& entry,
                        const SimTime& availability_time, std::ofstream* output, SimulatorRunSummary* summary,
                        std::string* error_message) {
    NavigationUpdateEvent event{};
    bool emitted = false;
    if (!apply_truth_navigation_record(runtime->navigation, entry.truth_record_index, availability_time, &event,
                                       &emitted, error_message)) {
        return false;
    }
    if (!emitted) {
        return true;
    }
    return emit_receiver_nav_record(config, receiver_navigation_store(runtime->navigation), event.receiver_record_index,
                                    event.availability_time, output, summary, error_message);
}

bool best_cold_plan(const SatelliteRuntime& satellite, const TruthScheduleEntry& target, NavAcquisitionPlan* plan,
                    bool* found, std::string* error_message) {
    *found = false;
    for (const SignalRuntime& signal : satellite.signals) {
        const SignalDefinition* definition = find_signal_definition(signal.tracker.signal_id);
        if (definition == nullptr || !target.has_family || definition->nav_message_family != target.family ||
            signal.tracker.phase != SignalTrackingPhase::kTracking) {
            continue;
        }
        SimTime acquisition_time = target.transmit_time;
        if (compare_sim_time(signal.tracker.tracking_start_time, acquisition_time) > 0) {
            acquisition_time = signal.tracker.tracking_start_time;
        }
        NavScheduleVariant variant = NavScheduleVariant::kDefault;
        if (target.family == NavMessageFamily::kBeidouD1D2) {
            variant = target.info.prn <= 5 ? NavScheduleVariant::kBeidouD2 : NavScheduleVariant::kBeidouD1;
        }
        NavAcquisitionPlan candidate{};
        if (!build_cold_nav_acquisition_plan_with_variant(signal.tracker.signal_id, variant, acquisition_time,
                                                          target.info.iode, &candidate, error_message)) {
            continue;
        }
        if (!*found || compare_sim_time(candidate.availability_time, plan->availability_time) < 0) {
            *plan = candidate;
            *found = true;
        }
    }
    return true;
}

const TruthScheduleEntry* latest_cold_target(const RuntimeState& runtime, int satellite_number, NavMessageFamily family,
                                             const SimTime& current_time) {
    const TruthScheduleEntry* target = nullptr;
    for (const TruthScheduleEntry& entry : runtime.truth_schedule) {
        if (!record_is_available(entry, current_time)) {
            break;
        }
        if (entry.info.satellite_number == satellite_number && entry.has_family && entry.family == family &&
            !navigation_truth_record_delivered(runtime.navigation, entry.truth_record_index)) {
            target = &entry;
        }
    }
    return target;
}

bool suppress_superseded_cold_records(RuntimeState* runtime, const TruthScheduleEntry& target,
                                      std::string* error_message) {
    for (const TruthScheduleEntry& entry : runtime->truth_schedule) {
        if (compare_sim_time(entry.transmit_time, target.transmit_time) > 0) {
            break;
        }
        if (entry.truth_record_index != target.truth_record_index &&
            entry.info.satellite_number == target.info.satellite_number && entry.has_family && target.has_family &&
            entry.family == target.family &&
            !navigation_truth_record_delivered(runtime->navigation, entry.truth_record_index) &&
            !consume_truth_navigation_record_without_copy(runtime->navigation, entry.truth_record_index,
                                                          error_message)) {
            return false;
        }
    }
    return true;
}

bool process_cold_nav(RuntimeState* runtime, const SimConfig& config, const SimTime& current_time,
                      std::ofstream* output, SimulatorRunSummary* summary, std::string* error_message) {
    const bool have_any_tracking = any_tracking(runtime->satellites);
    for (const TruthScheduleEntry& entry : runtime->truth_schedule) {
        if (!record_is_available(entry, current_time)) {
            break;
        }
        if (entry.info.kind == RtklibNavRecordKind::kIonosphere && have_any_tracking &&
            !navigation_truth_record_delivered(runtime->navigation, entry.truth_record_index) &&
            !apply_and_emit_nav(runtime, config, entry, current_time, output, summary, error_message)) {
            return false;
        }
    }

    for (SatelliteRuntime& satellite : runtime->satellites) {
        for (ColdFamilyRuntime& family_state : satellite.cold_families) {
            if (family_state.acquired) {
                if (!family_tracking(satellite, family_state.family)) {
                    continue;
                }
                for (const TruthScheduleEntry& entry : runtime->truth_schedule) {
                    if (!record_is_available(entry, current_time)) {
                        break;
                    }
                    if (entry.info.satellite_number == satellite.satellite_number && entry.has_family &&
                        entry.family == family_state.family &&
                        !navigation_truth_record_delivered(runtime->navigation, entry.truth_record_index) &&
                        !apply_and_emit_nav(runtime, config, entry, current_time, output, summary, error_message)) {
                        return false;
                    }
                }
                continue;
            }

            const TruthScheduleEntry* target =
                latest_cold_target(*runtime, satellite.satellite_number, family_state.family, current_time);
            if (target == nullptr) {
                continue;
            }
            NavAcquisitionPlan candidate{};
            bool found_candidate = false;
            if (!best_cold_plan(satellite, *target, &candidate, &found_candidate, error_message) || !found_candidate) {
                continue;
            }
            if (!family_state.plan_active || family_state.target_truth_record_index != target->truth_record_index ||
                compare_sim_time(candidate.availability_time, family_state.plan.availability_time) < 0) {
                family_state.plan = candidate;
                family_state.plan_active = true;
                family_state.target_truth_record_index = target->truth_record_index;
            }

            NavigationUpdateEvent event{};
            bool emitted = false;
            if (!deliver_cold_nav_plan_if_complete(family_state.plan, family_state.target_truth_record_index,
                                                   current_time, runtime->navigation, &event, &emitted,
                                                   error_message)) {
                return false;
            }
            if (!emitted) {
                continue;
            }
            if (!emit_receiver_nav_record(config, receiver_navigation_store(runtime->navigation),
                                          event.receiver_record_index, event.availability_time, output, summary,
                                          error_message) ||
                !suppress_superseded_cold_records(runtime, *target, error_message)) {
                return false;
            }
            family_state.acquired = true;
            family_state.plan_active = false;
        }
    }
    return true;
}

bool process_normal_nav(RuntimeState* runtime, const SimConfig& config, const SimTime& current_time,
                        std::ofstream* output, SimulatorRunSummary* summary, std::string* error_message) {
    const bool have_any_tracking = any_tracking(runtime->satellites);
    for (const TruthScheduleEntry& entry : runtime->truth_schedule) {
        if (!record_is_available(entry, current_time)) {
            break;
        }
        if (navigation_truth_record_delivered(runtime->navigation, entry.truth_record_index)) {
            continue;
        }
        bool can_receive = false;
        if (entry.info.kind == RtklibNavRecordKind::kIonosphere) {
            can_receive = have_any_tracking;
        } else {
            const SatelliteRuntime* satellite = nullptr;
            for (const SatelliteRuntime& candidate : runtime->satellites) {
                if (candidate.satellite_number == entry.info.satellite_number) {
                    satellite = &candidate;
                    break;
                }
            }
            can_receive = satellite != nullptr && entry.has_family && family_tracking(*satellite, entry.family);
        }
        if (can_receive && !apply_and_emit_nav(runtime, config, entry, current_time, output, summary, error_message)) {
            return false;
        }
    }
    return true;
}

bool process_receiver_nav(RuntimeState* runtime, const SimConfig& config, const SimTime& current_time,
                          std::ofstream* output, SimulatorRunSummary* summary, std::string* error_message) {
    if (runtime->startup_mode == StartupMode::COLD) {
        return process_cold_nav(runtime, config, current_time, output, summary, error_message);
    }
    return process_normal_nav(runtime, config, current_time, output, summary, error_message);
}

bool update_tracking_and_measurements(RuntimeState* runtime, const SimConfig& config,
                                      const ScenarioEpochState& scenario,
                                      std::vector<MeasurementObservation>* measurements, int* tracked_satellites,
                                      TruthWriter* truth_writer, std::string* error_message) {
    measurements->clear();
    *tracked_satellites = 0;
    const RtklibNavStore* truth_nav = truth_navigation_store(runtime->navigation);
    const AcquisitionContext initial_context = startup_context(runtime->startup_mode);

    for (SatelliteRuntime& satellite : runtime->satellites) {
        SatelliteGeometry geometry{};
        if (!compute_satellite_geometry(truth_nav, runtime->receiver, scenario.time, satellite.satellite_number,
                                        config.elevation_mask_deg, &geometry, error_message)) {
            return false;
        }
        const double elevation_deg = geometry.elevation_rad * kRadiansToDegrees;
        bool satellite_tracking = false;
        for (SignalRuntime& signal : satellite.signals) {
            const bool signal_available = scenario.signal_available && geometry.visible;
            if (signal_available && !signal.tracker.scheduled) {
                const AcquisitionContext context =
                    signal.ever_scheduled ? AcquisitionContext::kReacquisition : initial_context;
                SimTime search_ready_time = scenario.time;
                if (!signal.ever_scheduled &&
                    compare_sim_time(runtime->startup_search_ready_time, search_ready_time) > 0) {
                    search_ready_time = runtime->startup_search_ready_time;
                }
                if (!schedule_signal_acquisition(&signal.tracker, context, scenario.time, search_ready_time,
                                                 elevation_deg, runtime->cn0_model, runtime->tracking_config,
                                                 &runtime->rng, error_message)) {
                    return false;
                }
                signal.ever_scheduled = true;
            }
            if (!update_signal_tracker(&signal.tracker, scenario.time, signal_available, elevation_deg,
                                       runtime->cn0_model, error_message)) {
                return false;
            }
            if (signal.tracker.phase != SignalTrackingPhase::kTracking) {
                reset_carrier_ambiguity_state(&signal.ambiguity);
                continue;
            }
            satellite_tracking = true;
            AtmosphereCorrection atmosphere{};
            if (!compute_atmosphere_correction(config.atmosphere_mode, truth_nav, scenario.time,
                                               signal.tracker.signal_id, 0, runtime->receiver.position_ecef_m,
                                               geometry.azimuth_rad, geometry.elevation_rad, &atmosphere,
                                               error_message)) {
                return false;
            }
            MeasurementObservation observation{};
            if (!generate_zero_noise_measurement(truth_nav, geometry, signal.tracker, atmosphere, &signal.ambiguity,
                                                 &observation, error_message) ||
                !truth_writer_write_observation(truth_writer, runtime->receiver, geometry, signal.tracker, observation,
                                                error_message)) {
                return false;
            }
            measurements->push_back(observation);
        }
        if (satellite_tracking) {
            ++(*tracked_satellites);
        }
    }
    return true;
}

bool emit_epoch_logs(const SimConfig& config, const ScenarioEpochState& scenario,
                     const std::vector<MeasurementObservation>& measurements, int tracked_satellites,
                     const SolutionEpoch& solution, std::ofstream* output, SimulatorRunSummary* summary,
                     std::string* error_message) {
    std::string message;
    const MeasurementObservation* data = measurements.empty() ? nullptr : measurements.data();
    if (!format_novatel_rangea(scenario.time, data, static_cast<int>(measurements.size()), &message, error_message) ||
        !write_message(output, message, error_message)) {
        return false;
    }
    ++summary->range_messages;

    if (!format_novatel_psrposa(solution, tracked_satellites, &message, error_message) ||
        !write_message(output, message, error_message)) {
        return false;
    }
    ++summary->psrpos_messages;

    if (!format_novatel_psrvela(solution, &message, error_message) || !write_message(output, message, error_message)) {
        return false;
    }
    ++summary->psrvel_messages;
    return true;
}

} // namespace

bool run_simulator(const SimConfig& config, const SimulatorRunOptions& options, SimulatorRunSummary* summary,
                   std::string* error_message) {
    if (summary == nullptr || !valid_run_options(options) || !validate_sim_config(config, error_message)) {
        if (summary == nullptr || !valid_run_options(options)) {
            set_error(error_message, "simulator run request has invalid arguments");
        }
        return false;
    }
    if (config.atmosphere_mode == AtmosphereMode::UNSPECIFIED) {
        set_error(error_message, "simulation requires atmosphere_mode to be explicitly none or broadcast");
        return false;
    }

    SimulatorRunSummary result{};
    RuntimeState runtime{};
    runtime.navigation = create_navigation_state();
    if (runtime.navigation == nullptr) {
        set_error(error_message, "cannot allocate simulator navigation state");
        return false;
    }

    bool ok =
        load_truth_navigation(runtime.navigation, options.rinex_nav_path, error_message) &&
        make_static_receiver_truth(config.receiver, &runtime.receiver, error_message) &&
        build_truth_schedule(truth_navigation_store(runtime.navigation), &runtime.truth_schedule, error_message) &&
        build_satellite_runtimes(runtime.truth_schedule, options.start_time, &runtime.satellites, error_message);
    if (!ok) {
        destroy_navigation_state(runtime.navigation);
        return false;
    }

    seed_rng(&runtime.rng, config.seed);
    runtime.cn0_model = make_builtin_cn0_model(config.seed);
    runtime.tracking_config = default_signal_tracking_model_config();

    ScenarioEngine scenario_engine{};
    if (!initialize_scenario_engine(config, options.start_time, &scenario_engine, error_message)) {
        destroy_navigation_state(runtime.navigation);
        return false;
    }

    std::ofstream output(options.output_log_path, std::ios::binary | std::ios::trunc);
    if (!output) {
        set_error(error_message, std::string("cannot open simulator output: ") + options.output_log_path);
        destroy_navigation_state(runtime.navigation);
        return false;
    }

    TruthWriter* truth_writer =
        create_truth_writer(options.output_log_path, options.rinex_nav_path, config, options.start_time, error_message);
    if (truth_writer == nullptr) {
        output.close();
        destroy_navigation_state(runtime.navigation);
        return false;
    }

    std::uint64_t epoch_count = 0;
    if (!epoch_count_for_duration(config.duration_ns, config.sampling_rate_hz, &epoch_count)) {
        set_error(error_message, "cannot compute simulator epoch count");
        destroy_truth_writer(truth_writer);
        destroy_navigation_state(runtime.navigation);
        return false;
    }
    result.scheduled_epochs = epoch_count;

    std::vector<MeasurementObservation> measurements;
    std::size_t signal_count = 0;
    static_cast<void>(signal_definitions(&signal_count));
    measurements.reserve(runtime.satellites.size() * signal_count);

    for (std::uint64_t epoch_index = 0; epoch_index < epoch_count; ++epoch_index) {
        SimTime current_time{};
        if (!epoch_time_at_index(options.start_time, config.sampling_rate_hz, epoch_index, &current_time)) {
            set_error(error_message, "cannot advance simulator integer epoch clock");
            ok = false;
            break;
        }
        ScenarioEpochState scenario{};
        if (!update_scenario_engine(&scenario_engine, current_time, &scenario, error_message) ||
            !truth_writer_write_scenario_events(truth_writer, scenario, scenario_startup_mode(config), error_message)) {
            ok = false;
            break;
        }
        if (scenario.power_on_transition) {
            ++result.power_on_events;
            if (!receiver_power_on(&runtime, config, current_time, &output, &result, error_message)) {
                ok = false;
                break;
            }
        }
        if (scenario.power_off_transition) {
            ++result.power_off_events;
            receiver_power_off(&runtime, current_time);
        }
        if (scenario.signal_on_transition) {
            ++result.signal_on_events;
        }
        if (scenario.signal_off_transition) {
            ++result.signal_off_events;
            if (scenario.receiver_powered) {
                receiver_signal_off(&runtime, current_time);
            }
        }

        if (!scenario.receiver_powered) {
            continue;
        }
        ++result.powered_epochs;
        if (scenario.signal_available) {
            ++result.signal_on_epochs;
        } else {
            ++result.signal_off_epochs;
        }

        int tracked_satellites = 0;
        if (!update_tracking_and_measurements(&runtime, config, scenario, &measurements, &tracked_satellites,
                                              truth_writer, error_message) ||
            !process_receiver_nav(&runtime, config, current_time, &output, &result, error_message)) {
            ok = false;
            break;
        }
        if (static_cast<int>(measurements.size()) > result.max_observations_per_epoch) {
            result.max_observations_per_epoch = static_cast<int>(measurements.size());
        }

        SolutionEpoch solution{};
        const MeasurementObservation* data = measurements.empty() ? nullptr : measurements.data();
        if (!solve_receiver_epoch(receiver_navigation_store(runtime.navigation), current_time, data,
                                  static_cast<int>(measurements.size()), config.elevation_mask_deg,
                                  config.atmosphere_mode, &runtime.solution_state, &solution, error_message) ||
            !truth_writer_write_solution(truth_writer, solution, tracked_satellites, error_message) ||
            !emit_epoch_logs(config, scenario, measurements, tracked_satellites, solution, &output, &result,
                             error_message)) {
            ok = false;
            break;
        }
        if (solution.position.valid) {
            ++result.valid_position_epochs;
        }
        if (solution.velocity.valid) {
            ++result.valid_velocity_epochs;
        }
    }

    output.flush();
    if (!output && ok) {
        set_error(error_message, "failed to flush simulated receiver log");
        ok = false;
    }
    output.close();
    if (ok && !finalize_truth_writer(truth_writer, result, simulator_version(), simulator_commit_sha(),
                                     rtklib_commit_sha(), error_message)) {
        ok = false;
    }
    destroy_truth_writer(truth_writer);
    destroy_navigation_state(runtime.navigation);
    if (!ok) {
        return false;
    }
    *summary = result;
    return true;
}

const char* simulator_version() {
    return "0.1.0-dev";
}

const char* simulator_commit_sha() {
    return GNSS_SIM_SOURCE_COMMIT;
}

const char* rtklib_commit_sha() {
    return GNSS_SIM_RTKLIB_COMMIT;
}

} // namespace gnss_sim
