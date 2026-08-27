#include "tools/build_cn0_model/rinex_obs_stream.h"

#include "gnss/rtklib_adapter.h"
#include "gnss/satellite_engine.h"
#include "model/receiver_truth.h"

#include "rtklib.h"
#include "rtklib_obs_ext.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace gnss_sim::cn0_builder {
namespace {

constexpr int kObservationSlotCount = NFREQ + NEXOBS;
static_assert(kObservationSlotCount >= 5, "CN0 builder needs enough RTKLIB observation slots for V1 signals");

struct FileCloser {
    void operator()(FILE* file) const {
        if (file != nullptr) {
            std::fclose(file);
        }
    }
};

struct NavStoreDeleter {
    void operator()(RtklibNavStore* store) const { destroy_rtklib_nav_store(store); }
};

struct RinexControlGuard {
    rnxctr_t value{};
    bool initialized{};

    ~RinexControlGuard() {
        if (initialized) {
            free_rnxctr(&value);
        }
    }
};

struct HeaderIndexEntry {
    unsigned char code{};
    int frequency{};
    int priority{};
    int slot{-1};
};

struct DeclaredSignal {
    int rtklib_system{};
    GnssConstellation constellation{GnssConstellation::kGps};
    int slot{-1};
    const SignalDefinition* definition{};
};

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

std::string trim(std::string value) {
    const std::size_t begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const std::size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

std::string uppercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        if (character >= 'a' && character <= 'z') {
            return static_cast<char>(character - 'a' + 'A');
        }
        return static_cast<char>(character);
    });
    return value;
}

bool scan_signal_strength_unit(const std::string& path, std::string* unit, SignalStrengthUnitStatus* status,
                               std::string* error_message) {
    if (unit == nullptr || status == nullptr) {
        set_error(error_message, "signal-strength metadata outputs must not be null");
        return false;
    }

    std::ifstream input(path);
    if (!input) {
        set_error(error_message, "cannot open RINEX observation file for header inspection: " + path);
        return false;
    }

    std::set<std::string> values;
    bool saw_record = false;
    bool saw_end = false;
    std::string line;
    while (std::getline(input, line)) {
        const std::string label = line.size() > 60 ? trim(line.substr(60)) : std::string{};
        if (label.find("SIGNAL STRENGTH UNIT") != std::string::npos) {
            saw_record = true;
            values.insert(uppercase(trim(line.substr(0, std::min<std::size_t>(20, line.size())))));
        }
        if (label.find("END OF HEADER") != std::string::npos) {
            saw_end = true;
            break;
        }
    }
    if (!saw_end) {
        set_error(error_message, "RINEX observation header has no END OF HEADER record");
        return false;
    }
    if (!saw_record) {
        unit->clear();
        *status = SignalStrengthUnitStatus::kMissing;
        return true;
    }
    if (values.size() != 1) {
        *unit = values.empty() ? std::string{} : *values.begin();
        *status = SignalStrengthUnitStatus::kConflicting;
        return true;
    }

    *unit = *values.begin();
    *status = *unit == "DBHZ" ? SignalStrengthUnitStatus::kDbHz : SignalStrengthUnitStatus::kUnsupported;
    return true;
}

bool constellation_from_rtklib(int system, GnssConstellation* constellation) {
    if (constellation == nullptr) {
        return false;
    }
    switch (system) {
    case SYS_GPS:
        *constellation = GnssConstellation::kGps;
        return true;
    case SYS_GLO:
        *constellation = GnssConstellation::kGlonass;
        return true;
    case SYS_GAL:
        *constellation = GnssConstellation::kGalileo;
        return true;
    case SYS_CMP:
        *constellation = GnssConstellation::kBeidou;
        return true;
    case SYS_QZS:
        *constellation = GnssConstellation::kQzss;
        return true;
    default:
        return false;
    }
}

std::string normalize_rinex_signal_code(GnssConstellation constellation, std::string code) {
    // The pinned RTKLIB parser normalizes RINEX-3 BeiDou B1I 2I to its
    // historical internal alias 1I. Convert that alias back to the simulator's
    // authoritative RINEX signal identifier.
    if (constellation == GnssConstellation::kBeidou && code == "1I") {
        return "2I";
    }
    return code;
}

std::string unsupported_key(int rtklib_system, const char* observation_type) {
    char system = '?';
    switch (rtklib_system) {
    case SYS_GPS:
        system = 'G';
        break;
    case SYS_GLO:
        system = 'R';
        break;
    case SYS_GAL:
        system = 'E';
        break;
    case SYS_QZS:
        system = 'J';
        break;
    case SYS_CMP:
        system = 'C';
        break;
    default:
        break;
    }
    return std::string(1, system) + ":" + (observation_type != nullptr ? observation_type : "");
}

void remap_beidou_frequency(int system, int* frequency) {
    if (system != SYS_CMP || frequency == nullptr) {
        return;
    }
    if (*frequency == 5) {
        *frequency = 2;
    } else if (*frequency == 4) {
        *frequency = 3;
    }
}

std::vector<HeaderIndexEntry> build_header_index(int system, const char tobs[MAXOBSTYPE][4]) {
    std::vector<HeaderIndexEntry> entries;
    for (int index = 0; index < MAXOBSTYPE && tobs[index][0] != '\0'; ++index) {
        int frequency = 0;
        const unsigned char code = obs2code_ext(tobs[index] + 1, &frequency);
        remap_beidou_frequency(system, &frequency);
        const int priority = code != CODE_NONE ? getcodepri_ext(system, code, "") : 0;
        entries.push_back({code, frequency, priority, -1});
    }

    for (int frequency_slot = 0; frequency_slot < NFREQ; ++frequency_slot) {
        int selected = -1;
        for (int index = 0; index < static_cast<int>(entries.size()); ++index) {
            if (entries[index].frequency != frequency_slot + 1 || entries[index].priority <= 0) {
                continue;
            }
            if (selected < 0 || entries[index].priority > entries[selected].priority) {
                selected = index;
            }
        }
        if (selected < 0) {
            continue;
        }
        const unsigned char selected_code = entries[selected].code;
        for (HeaderIndexEntry& entry : entries) {
            if (entry.code == selected_code) {
                entry.slot = frequency_slot;
            }
        }
    }

    for (int extended_slot = 0; extended_slot < NEXOBS; ++extended_slot) {
        int selected = -1;
        for (int index = 0; index < static_cast<int>(entries.size()); ++index) {
            if (entries[index].code != CODE_NONE && entries[index].priority > 0 && entries[index].slot < 0) {
                selected = index;
                break;
            }
        }
        if (selected < 0) {
            break;
        }
        const unsigned char selected_code = entries[selected].code;
        for (HeaderIndexEntry& entry : entries) {
            if (entry.code == selected_code) {
                entry.slot = NFREQ + extended_slot;
            }
        }
    }
    return entries;
}

std::vector<DeclaredSignal> declared_cn0_signals(const rnxctr_t& control, RinexObsStreamSummary* summary) {
    const int systems[6] = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_SBS, SYS_CMP};
    std::vector<DeclaredSignal> signals;
    std::set<std::string> unsupported;

    for (int system_index = 0; system_index < 6; ++system_index) {
        const int system = systems[system_index];
        GnssConstellation constellation{};
        const bool supported_constellation = constellation_from_rtklib(system, &constellation);
        const std::vector<HeaderIndexEntry> entries = build_header_index(system, control.tobs[system_index]);
        for (int type_index = 0; type_index < static_cast<int>(entries.size()); ++type_index) {
            const char* observation_type = control.tobs[system_index][type_index];
            if (observation_type[0] != 'S') {
                continue;
            }
            if (!supported_constellation) {
                unsupported.insert(unsupported_key(system, observation_type));
                continue;
            }

            const std::string rinex_code = normalize_rinex_signal_code(constellation, observation_type + 1);
            const SignalDefinition* definition = find_signal_definition_by_rinex(constellation, rinex_code.c_str());
            if (entries[type_index].code == CODE_NONE || entries[type_index].priority <= 0 ||
                entries[type_index].slot < 0 || definition == nullptr) {
                unsupported.insert(unsupported_key(system, observation_type));
                continue;
            }
            signals.push_back({system, constellation, entries[type_index].slot, definition});
        }
    }

    if (summary != nullptr) {
        summary->unsupported_observables.assign(unsupported.begin(), unsupported.end());
        summary->unsupported_signal_observables = summary->unsupported_observables.size();
    }
    return signals;
}

gtime_t observation_time_to_gpst(gtime_t time, int time_system) {
    if (time_system == TSYS_UTC || time_system == TSYS_GLO) {
        return utc2gpst(time);
    }
    if (time_system == TSYS_CMP) {
        return bdt2gpst(time);
    }
    // GPST, GST and QZST share the same coarse epoch timeline for this
    // simulator boundary. BeiDou and UTC/GLONASS require explicit conversion.
    return time;
}

const char* time_system_name(int time_system) {
    switch (time_system) {
    case TSYS_GPS:
        return "GPS";
    case TSYS_UTC:
        return "UTC";
    case TSYS_GLO:
        return "GLO";
    case TSYS_GAL:
        return "GAL";
    case TSYS_QZS:
        return "QZS";
    case TSYS_CMP:
        return "BDT";
    default:
        return "UNKNOWN";
    }
}

bool gtime_to_sim_time(gtime_t time, SimTime* sim_time) {
    if (sim_time == nullptr) {
        return false;
    }
    int week = 0;
    const double sow_sec = time2gpst(time, &week);
    return week >= 0 && std::isfinite(sow_sec) && sim_time_from_week_sow(week, sow_sec, sim_time);
}

bool make_receiver_truth(const sta_t& station, ReceiverTruth* receiver, std::string* error_message) {
    if (receiver == nullptr || !std::isfinite(station.pos[0]) || !std::isfinite(station.pos[1]) ||
        !std::isfinite(station.pos[2]) ||
        std::sqrt(station.pos[0] * station.pos[0] + station.pos[1] * station.pos[1] + station.pos[2] * station.pos[2]) <
            1.0) {
        set_error(error_message, "RINEX observation header has no usable APPROX POSITION XYZ station position");
        return false;
    }

    ReceiverTruth result{};
    for (int index = 0; index < 3; ++index) {
        result.position_ecef_m[index] = station.pos[index];
        result.velocity_ecef_mps[index] = 0.0;
    }
    if (!rtklib_ecef_to_llh(result.position_ecef_m, &result.latitude_deg, &result.longitude_deg, &result.height_m)) {
        set_error(error_message, "cannot convert RINEX station ECEF position to receiver LLH");
        return false;
    }
    *receiver = result;
    return true;
}

int satellite_prn(int satellite_number) {
    char identifier[4]{};
    if (!rtklib_satellite_number_to_id(satellite_number, identifier) || identifier[0] == '\0') {
        return 0;
    }
    char* end = nullptr;
    const long value = std::strtol(identifier + 1, &end, 10);
    if (end == identifier + 1 || value <= 0 || value > 999) {
        return 0;
    }
    return static_cast<int>(value);
}

bool slot_is_declared(const std::vector<DeclaredSignal>& signals, int system, int slot) {
    for (const DeclaredSignal& declared : signals) {
        if (declared.rtklib_system == system && declared.slot == slot) {
            return true;
        }
    }
    return false;
}

} // namespace

const char* signal_strength_unit_status_name(SignalStrengthUnitStatus status) {
    switch (status) {
    case SignalStrengthUnitStatus::kDbHz:
        return "DBHZ";
    case SignalStrengthUnitStatus::kMissing:
        return "MISSING";
    case SignalStrengthUnitStatus::kUnsupported:
        return "UNSUPPORTED";
    case SignalStrengthUnitStatus::kConflicting:
        return "CONFLICTING";
    }
    return "UNKNOWN";
}

const char* cn0_sample_validity_name(Cn0SampleValidity validity) {
    switch (validity) {
    case Cn0SampleValidity::kValidDbHz:
        return "VALID_DBHZ";
    case Cn0SampleValidity::kAmbiguousSignalStrengthUnit:
        return "AMBIGUOUS_SIGNAL_STRENGTH_UNIT";
    case Cn0SampleValidity::kGeometryUnavailable:
        return "GEOMETRY_UNAVAILABLE";
    }
    return "UNKNOWN";
}

const char* constellation_name(GnssConstellation constellation) {
    switch (constellation) {
    case GnssConstellation::kGps:
        return "GPS";
    case GnssConstellation::kGlonass:
        return "GLONASS";
    case GnssConstellation::kGalileo:
        return "GALILEO";
    case GnssConstellation::kBeidou:
        return "BEIDOU";
    case GnssConstellation::kQzss:
        return "QZSS";
    }
    return "UNKNOWN";
}

bool stream_rinex_cn0_samples(const std::string& observation_path, const std::string& navigation_path,
                              const RinexCn0SampleCallback& callback, RinexObsProvenance* provenance,
                              RinexObsStreamSummary* summary, std::string* error_message) {
    if (!callback || provenance == nullptr || summary == nullptr) {
        set_error(error_message, "CN0 stream callback, provenance and summary are required");
        return false;
    }

    *provenance = {};
    *summary = {};
    provenance->observation_path = observation_path;
    provenance->navigation_path = navigation_path;
    if (!scan_signal_strength_unit(observation_path, &provenance->signal_strength_unit,
                                   &provenance->signal_strength_unit_status, error_message)) {
        return false;
    }

    std::unique_ptr<RtklibNavStore, NavStoreDeleter> nav_store(create_rtklib_nav_store());
    if (!nav_store) {
        set_error(error_message, "cannot allocate RTKLIB navigation store");
        return false;
    }
    if (!load_rinex_nav_file(nav_store.get(), navigation_path.c_str(), error_message)) {
        return false;
    }

    std::unique_ptr<FILE, FileCloser> input(std::fopen(observation_path.c_str(), "r"));
    if (!input) {
        set_error(error_message, "cannot open RINEX observation file: " + observation_path);
        return false;
    }

    RinexControlGuard control;
    if (!init_rnxctr(&control.value)) {
        set_error(error_message, "RTKLIB could not allocate incremental RINEX control buffers");
        return false;
    }
    control.initialized = true;
    if (!open_rnxctr(&control.value, input.get()) || control.value.type != 'O') {
        set_error(error_message, "RTKLIB could not open the file as RINEX observation data");
        return false;
    }

    provenance->rinex_version = control.value.ver;
    provenance->station_name = control.value.sta.name;
    provenance->marker_number = control.value.sta.marker;
    provenance->receiver_type = control.value.sta.rectype;
    provenance->antenna_type = control.value.sta.antdes;
    provenance->observation_time_system = time_system_name(control.value.tsys);
    for (int index = 0; index < 3; ++index) {
        provenance->station_ecef_m[index] = control.value.sta.pos[index];
    }

    ReceiverTruth receiver{};
    if (!make_receiver_truth(control.value.sta, &receiver, error_message)) {
        return false;
    }

    const std::vector<DeclaredSignal> declared_signals = declared_cn0_signals(control.value, summary);
    bool have_previous_epoch = false;
    SimTime previous_epoch{};

    while (true) {
        const int status = input_rnxctr(&control.value, input.get());
        if (status == -2) {
            break;
        }
        if (status == 0) {
            continue;
        }
        if (status != 1) {
            set_error(error_message, "unexpected non-observation record while streaming RINEX observation file");
            return false;
        }

        ++summary->epochs;
        summary->peak_epoch_observations = std::max(summary->peak_epoch_observations, control.value.obs.n);
        const gtime_t epoch_gpst = observation_time_to_gpst(control.value.time, control.value.tsys);
        SimTime epoch{};
        if (!gtime_to_sim_time(epoch_gpst, &epoch)) {
            set_error(error_message, "cannot normalize RINEX observation epoch to GPST");
            return false;
        }
        if (have_previous_epoch && compare_sim_time(epoch, previous_epoch) < 0) {
            ++summary->out_of_order_epochs;
            std::ostringstream message;
            message << "RINEX observation epochs are out of order at GPST " << epoch.gps_week << ':'
                    << sim_time_sow_sec(epoch);
            set_error(error_message, message.str());
            return false;
        }
        previous_epoch = epoch;
        have_previous_epoch = true;

        for (int observation_index = 0; observation_index < control.value.obs.n; ++observation_index) {
            const obsd_t& observation = control.value.obs.data[observation_index];
            ++summary->observation_records;

            const int rtklib_system = satsys(observation.sat, nullptr);
            GnssConstellation constellation{};
            if (!constellation_from_rtklib(rtklib_system, &constellation)) {
                continue;
            }

            SatelliteGeometry geometry{};
            std::string geometry_error;
            const bool geometry_valid = compute_satellite_geometry(nav_store.get(), receiver, epoch, observation.sat,
                                                                    -90.0, &geometry, &geometry_error);
            if (!geometry_valid) {
                ++summary->geometry_failures;
            }

            for (int slot = 0; slot < kObservationSlotCount; ++slot) {
                if (observation.SNR[slot] != 0 && !slot_is_declared(declared_signals, rtklib_system, slot)) {
                    ++summary->unmapped_snr_slots;
                }
            }

            for (const DeclaredSignal& declared : declared_signals) {
                if (declared.rtklib_system != rtklib_system) {
                    continue;
                }
                if (declared.slot < 0 || declared.slot >= kObservationSlotCount || observation.SNR[declared.slot] == 0) {
                    ++summary->missing_signal_strength;
                    continue;
                }

                RinexCn0Sample sample{};
                sample.time = epoch;
                sample.constellation = constellation;
                sample.satellite_number = observation.sat;
                sample.prn = satellite_prn(observation.sat);
                sample.signal_id = declared.definition->signal_id;
                sample.rinex_signal_code = declared.definition->rinex_signal_code;
                sample.signal_strength_value = static_cast<double>(observation.SNR[declared.slot]) * 0.25;
                sample.cn0_dbhz = std::numeric_limits<double>::quiet_NaN();
                sample.azimuth_rad = std::numeric_limits<double>::quiet_NaN();
                sample.elevation_rad = std::numeric_limits<double>::quiet_NaN();
                sample.provenance = provenance;

                if (provenance->signal_strength_unit_status == SignalStrengthUnitStatus::kDbHz) {
                    sample.cn0_dbhz = sample.signal_strength_value;
                }
                if (geometry_valid) {
                    sample.azimuth_rad = geometry.azimuth_rad;
                    sample.elevation_rad = geometry.elevation_rad;
                }

                if (provenance->signal_strength_unit_status != SignalStrengthUnitStatus::kDbHz) {
                    sample.validity = Cn0SampleValidity::kAmbiguousSignalStrengthUnit;
                    ++summary->ambiguous_unit_samples;
                } else if (!geometry_valid) {
                    sample.validity = Cn0SampleValidity::kGeometryUnavailable;
                } else {
                    sample.validity = Cn0SampleValidity::kValidDbHz;
                    ++summary->valid_dbhz_samples;
                }

                ++summary->emitted_samples;
                if (!callback(sample)) {
                    set_error(error_message, "CN0 sample callback requested early termination");
                    return false;
                }
            }
        }
    }

    return true;
}

} // namespace gnss_sim::cn0_builder
