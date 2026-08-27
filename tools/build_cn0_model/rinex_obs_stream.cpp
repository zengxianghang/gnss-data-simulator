#include "tools/build_cn0_model/rinex_obs_stream.h"

#include "gnss/rtklib_adapter.h"
#include "gnss/satellite_engine.h"
#include "model/receiver_truth.h"

#include "rtklib.h"
#include "rtklib_obs_ext.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <utility>

namespace gnss_sim::cn0_builder {
namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
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

struct DeclaredSignal {
    int rtklib_system{};
    GnssConstellation constellation{GnssConstellation::kGps};
    unsigned char observation_code{};
    const SignalDefinition* definition{};
};

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

std::string trim(std::string value) {
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = value.find_last_not_of(" \t\r\n");
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
    // historical internal alias 1I. Convert that internal alias back to the
    // authoritative RINEX signal identifier used by the simulator table.
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

std::vector<DeclaredSignal> declared_cn0_signals(const rnxctr_t& control, RinexObsStreamSummary* summary) {
    const int systems[6] = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_SBS, SYS_CMP};
    std::vector<DeclaredSignal> signals;
    std::set<std::string> unsupported;

    for (int system_index = 0; system_index < 6; ++system_index) {
        GnssConstellation constellation{};
        const bool supported_constellation = constellation_from_rtklib(systems[system_index], &constellation);
        for (int type_index = 0; type_index < MAXOBSTYPE && control.tobs[system_index][type_index][0] != '\0';
             ++type_index) {
            const char* observation_type = control.tobs[system_index][type_index];
            if (observation_type[0] != 'S') {
                continue;
            }
            if (!supported_constellation) {
                unsupported.insert(unsupported_key(systems[system_index], observation_type));
                continue;
            }

            int frequency = 0;
            const unsigned char observation_code = obs2code_ext(observation_type + 1, &frequency);
            const std::string rinex_code = normalize_rinex_signal_code(constellation, observation_type + 1);
            const SignalDefinition* definition = find_signal_definition_by_rinex(constellation, rinex_code.c_str());
            if (observation_code == CODE_NONE || frequency <= 0 || definition == nullptr) {
                unsupported.insert(unsupported_key(systems[system_index], observation_type));
                continue;
            }
            signals.push_back({systems[system_index], constellation, observation_code, definition});
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
    // Match RTKLIB readrnxobs(): GPS, Galileo and QZSS observation epochs are
    // used directly on the GPST-aligned timeline; only UTC is converted there.
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
    if (week < 0 || !std::isfinite(sow_sec)) {
        return false;
    }

    std::int64_t tow_ns = static_cast<std::int64_t>(std::llround(sow_sec * static_cast<double>(kNanosecondsPerSecond)));
    while (tow_ns >= GPS_WEEK_NANOSECONDS) {
        tow_ns -= GPS_WEEK_NANOSECONDS;
        ++week;
    }
    while (tow_ns < 0) {
        if (week == 0) {
            return false;
        }
        tow_ns += GPS_WEEK_NANOSECONDS;
        --week;
    }
    sim_time->gps_week = week;
    sim_time->tow_ns = tow_ns;
    return true;
}

bool time_precedes(const SimTime& lhs, const SimTime& rhs) {
    return lhs.gps_week < rhs.gps_week || (lhs.gps_week == rhs.gps_week && lhs.tow_ns < rhs.tow_ns);
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

int find_observation_slot(const obsd_t& observation, unsigned char observation_code) {
    for (int slot = 0; slot < kObservationSlotCount; ++slot) {
        if (observation.code[slot] == observation_code) {
            return slot;
        }
    }
    return -1;
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
        if (have_previous_epoch && time_precedes(epoch, previous_epoch)) {
            ++summary->out_of_order_epochs;
            std::ostringstream message;
            message << "RINEX observation epochs are out of order at GPST " << epoch.gps_week << ':'
                    << static_cast<double>(epoch.tow_ns) / static_cast<double>(kNanosecondsPerSecond);
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
                if (observation.SNR[slot] != 0 && observation.code[slot] == CODE_NONE) {
                    ++summary->unmapped_snr_slots;
                }
            }

            for (const DeclaredSignal& declared : declared_signals) {
                if (declared.rtklib_system != rtklib_system) {
                    continue;
                }
                const int slot = find_observation_slot(observation, declared.observation_code);
                if (slot < 0 || observation.SNR[slot] == 0) {
                    ++summary->missing_signal_strength;
                    continue;
                }

                RinexCn0Sample sample{};
                sample.time = epoch;
                sample.constellation = constellation;
                sample.satellite_number = observation.sat;
                sample.prn = satellite_prn(observation.sat);
                sample.signal_id = declared.definition->id;
                sample.rinex_signal_code = declared.definition->rinex_signal_code;
                sample.signal_strength_value = static_cast<double>(observation.SNR[slot]) * 0.25;
                sample.cn0_dbhz = std::numeric_limits<double>::quiet_NaN();
                sample.azimuth_rad = std::numeric_limits<double>::quiet_NaN();
                sample.elevation_rad = std::numeric_limits<double>::quiet_NaN();

                if (geometry_valid) {
                    sample.azimuth_rad = geometry.azimuth_rad;
                    sample.elevation_rad = geometry.elevation_rad;
                }
                if (!geometry_valid) {
                    sample.validity = Cn0SampleValidity::kGeometryUnavailable;
                } else if (provenance->signal_strength_unit_status != SignalStrengthUnitStatus::kDbHz) {
                    sample.validity = Cn0SampleValidity::kAmbiguousSignalStrengthUnit;
                    ++summary->ambiguous_unit_samples;
                } else {
                    sample.validity = Cn0SampleValidity::kValidDbHz;
                    sample.cn0_dbhz = sample.signal_strength_value;
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
