#include "output/truth_writer.h"

#include "gnss/rtklib_adapter.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <new>
#include <sstream>
#include <string>

namespace gnss_sim {

struct TruthWriter {
    std::filesystem::path output_directory;
    std::ofstream event_stream;
    std::ofstream observation_stream;
    std::ofstream solution_stream;
    SimConfig config;
    SimTime start_time;
    std::string rinex_nav_name;
    std::string rinex_nav_hash;
    std::uint64_t rinex_nav_size;
    bool have_observation_time;
    SimTime last_observation_time;
    std::uint64_t observation_index;
    bool finalized;
};

namespace {

constexpr std::uint64_t kFnv1aOffsetBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnv1aPrime = 1099511628211ULL;
constexpr double kRadiansToDegrees = 180.0 / 3.141592653589793238462643383279502884;

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

const char* bool_json(bool value) {
    return value ? "true" : "false";
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

const char* broadcast_message_family_name(RtklibBroadcastMessageFamily family) {
    switch (family) {
        case RtklibBroadcastMessageFamily::kUnknown:
            return "UNKNOWN";
        case RtklibBroadcastMessageFamily::kLegacy:
            return "LEGACY";
        case RtklibBroadcastMessageFamily::kCnav:
            return "CNAV";
        case RtklibBroadcastMessageFamily::kCnav2:
            return "CNAV2";
        case RtklibBroadcastMessageFamily::kGalileoInav:
            return "GALILEO_INAV";
        case RtklibBroadcastMessageFamily::kGalileoFnav:
            return "GALILEO_FNAV";
        case RtklibBroadcastMessageFamily::kBeidouBcnav1:
            return "BEIDOU_BCNAV1";
        case RtklibBroadcastMessageFamily::kBeidouBcnav2:
            return "BEIDOU_BCNAV2";
        case RtklibBroadcastMessageFamily::kBeidouBcnav3:
            return "BEIDOU_BCNAV3";
        case RtklibBroadcastMessageFamily::kGlonassFdma:
            return "GLONASS_FDMA";
        case RtklibBroadcastMessageFamily::kGlonassL3Oc:
            return "GLONASS_L3OC";
    }
    return "UNKNOWN";
}

std::string json_escape(const char* text) {
    if (text == nullptr) {
        return std::string();
    }
    std::ostringstream escaped;
    for (const unsigned char character : std::string(text)) {
        switch (character) {
            case '"':
                escaped << "\\\"";
                break;
            case '\\':
                escaped << "\\\\";
                break;
            case '\b':
                escaped << "\\b";
                break;
            case '\f':
                escaped << "\\f";
                break;
            case '\n':
                escaped << "\\n";
                break;
            case '\r':
                escaped << "\\r";
                break;
            case '\t':
                escaped << "\\t";
                break;
            default:
                if (character < 0x20U) {
                    escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                            << static_cast<unsigned int>(character) << std::dec << std::setfill(' ');
                } else {
                    escaped << static_cast<char>(character);
                }
                break;
        }
    }
    return escaped.str();
}

std::string csv_escape(const char* text) {
    const std::string source = text != nullptr ? std::string(text) : std::string();
    if (source.find_first_of(",\"\r\n") == std::string::npos) {
        return source;
    }
    std::string result = "\"";
    for (char character : source) {
        if (character == '"') {
            result += "\"\"";
        } else {
            result += character;
        }
    }
    result += '"';
    return result;
}

std::string config_json(const SimConfig& config, int indent) {
    const std::string pad(static_cast<std::size_t>(indent), ' ');
    const std::string inner(static_cast<std::size_t>(indent + 2), ' ');
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(17);
    output << "{\n";
    output << inner << "\"schema_version\": " << config.schema_version << ",\n";
    output << inner << "\"scenario\": \"" << scenario_type_name(config.scenario) << "\",\n";
    output << inner << "\"duration_ns\": " << config.duration_ns << ",\n";
    output << inner << "\"sampling_rate_hz\": " << config.sampling_rate_hz << ",\n";
    output << inner << "\"elevation_mask_deg\": " << config.elevation_mask_deg << ",\n";
    output << inner << "\"solution_elevation_mask_deg\": " << config.solution_elevation_mask_deg << ",\n";
    output << inner << "\"output_eph\": " << bool_json(config.output_eph) << ",\n";
    output << inner << "\"output_ion\": " << bool_json(config.output_ion) << ",\n";
    output << inner << "\"measurement_noise_enabled\": " << bool_json(config.measurement_noise_enabled) << ",\n";
    output << inner << "\"multipath_enabled\": " << bool_json(config.multipath_enabled) << ",\n";
    output << inner << "\"receiver_clock_bias_m\": " << config.receiver_clock_bias_m << ",\n";
    output << inner << "\"receiver_clock_drift_mps\": " << config.receiver_clock_drift_mps << ",\n";
    output << inner << "\"atmosphere_mode\": \"" << atmosphere_mode_name(config.atmosphere_mode) << "\",\n";
    output << inner << "\"receiver\": {\n";
    output << inner << "  \"latitude_deg\": " << config.receiver.latitude_deg << ",\n";
    output << inner << "  \"longitude_deg\": " << config.receiver.longitude_deg << ",\n";
    output << inner << "  \"height_m\": " << config.receiver.height_m << "\n";
    output << inner << "},\n";
    output << inner << "\"ttff\": {\n";
    output << inner << "  \"startup_mode\": \"" << startup_mode_name(config.ttff.startup_mode) << "\",\n";
    output << inner << "  \"power_on_ns\": " << config.ttff.power_on_ns << ",\n";
    output << inner << "  \"power_off_ns\": " << config.ttff.power_off_ns << "\n";
    output << inner << "},\n";
    output << inner << "\"rea\": {\n";
    output << inner << "  \"signal_on_ns\": " << config.rea.signal_on_ns << ",\n";
    output << inner << "  \"signal_off_ns\": " << config.rea.signal_off_ns << "\n";
    output << inner << "},\n";
    output << inner << "\"bestpos_rtk\": {\n";
    output << inner << "  \"enabled\": " << bool_json(config.bestpos_rtk.enabled) << ",\n";
    output << inner << "  \"stable_duration_ns\": " << config.bestpos_rtk.stable_duration_ns << ",\n";
    output << inner << "  \"min_used_satellites\": " << config.bestpos_rtk.min_used_satellites << ",\n";
    output << inner << "  \"horizontal_std_m\": " << config.bestpos_rtk.horizontal_std_m << ",\n";
    output << inner << "  \"height_std_m\": " << config.bestpos_rtk.height_std_m << "\n";
    output << inner << "},\n";
    output << inner << "\"measurement_error\": {\n";
    output << inner << "  \"psr_sigma_m\": " << config.measurement_error.psr_sigma_m << ",\n";
    output << inner << "  \"doppler_sigma_mps\": " << config.measurement_error.doppler_sigma_mps << ",\n";
    output << inner << "  \"adr_sigma_m\": " << config.measurement_error.adr_sigma_m << ",\n";
    output << inner << "  \"cn0_sigma_dbhz\": " << config.measurement_error.cn0_sigma_dbhz << ",\n";
    output << inner << "  \"psr_correlation_tau_sec\": " << config.measurement_error.psr_correlation_tau_sec << ",\n";
    output << inner << "  \"ttff_hot\": {\"psr_extra_sigma_m\": " << config.measurement_error.ttff_hot.psr_extra_sigma_m
           << ", \"doppler_extra_sigma_mps\": " << config.measurement_error.ttff_hot.doppler_extra_sigma_mps
           << ", \"cn0_extra_sigma_dbhz\": " << config.measurement_error.ttff_hot.cn0_extra_sigma_dbhz
           << ", \"decay_tau_sec\": " << config.measurement_error.ttff_hot.decay_tau_sec << "},\n";
    output << inner
           << "  \"ttff_warm\": {\"psr_extra_sigma_m\": " << config.measurement_error.ttff_warm.psr_extra_sigma_m
           << ", \"doppler_extra_sigma_mps\": " << config.measurement_error.ttff_warm.doppler_extra_sigma_mps
           << ", \"cn0_extra_sigma_dbhz\": " << config.measurement_error.ttff_warm.cn0_extra_sigma_dbhz
           << ", \"decay_tau_sec\": " << config.measurement_error.ttff_warm.decay_tau_sec << "},\n";
    output << inner
           << "  \"ttff_cold\": {\"psr_extra_sigma_m\": " << config.measurement_error.ttff_cold.psr_extra_sigma_m
           << ", \"doppler_extra_sigma_mps\": " << config.measurement_error.ttff_cold.doppler_extra_sigma_mps
           << ", \"cn0_extra_sigma_dbhz\": " << config.measurement_error.ttff_cold.cn0_extra_sigma_dbhz
           << ", \"decay_tau_sec\": " << config.measurement_error.ttff_cold.decay_tau_sec << "},\n";
    output << inner << "  \"rea_reacquisition\": {\"psr_extra_sigma_m\": "
           << config.measurement_error.rea_reacquisition.psr_extra_sigma_m
           << ", \"doppler_extra_sigma_mps\": " << config.measurement_error.rea_reacquisition.doppler_extra_sigma_mps
           << ", \"cn0_extra_sigma_dbhz\": " << config.measurement_error.rea_reacquisition.cn0_extra_sigma_dbhz
           << ", \"decay_tau_sec\": " << config.measurement_error.rea_reacquisition.decay_tau_sec << "},\n";
    output << inner << "  \"rea_fade\": {\"duration_sec\": " << config.measurement_error.rea_fade.duration_sec
           << ", \"psr_extra_sigma_m\": " << config.measurement_error.rea_fade.psr_extra_sigma_m
           << ", \"doppler_extra_sigma_mps\": " << config.measurement_error.rea_fade.doppler_extra_sigma_mps
           << ", \"cn0_drop_db\": " << config.measurement_error.rea_fade.cn0_drop_db << "}\n";
    output << inner << "},\n";
    output << inner << "\"seed\": " << config.seed << "\n";
    output << pad << '}';
    return output.str();
}

bool write_text_file(const std::filesystem::path& path, const std::string& content, std::string* error_message) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) {
        set_error(error_message, std::string("cannot open truth output: ") + path.generic_string());
        return false;
    }
    output.write(content.data(), static_cast<std::streamsize>(content.size()));
    output.flush();
    if (!output) {
        set_error(error_message, std::string("failed to write truth output: ") + path.generic_string());
        return false;
    }
    return true;
}

bool input_identity(const char* path, std::string* name, std::string* hash, std::uint64_t* size,
                    std::string* error_message) {
    if (path == nullptr || name == nullptr || hash == nullptr || size == nullptr) {
        set_error(error_message, "truth input identity request has invalid arguments");
        return false;
    }
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        set_error(error_message, std::string("cannot open RINEX NAV for hashing: ") + path);
        return false;
    }
    std::uint64_t value = kFnv1aOffsetBasis;
    std::uint64_t byte_count = 0;
    char buffer[65536];
    while (input) {
        input.read(buffer, static_cast<std::streamsize>(sizeof(buffer)));
        const std::streamsize count = input.gcount();
        for (std::streamsize index = 0; index < count; ++index) {
            value ^= static_cast<unsigned char>(buffer[index]);
            value *= kFnv1aPrime;
        }
        byte_count += static_cast<std::uint64_t>(count);
    }
    if (!input.eof()) {
        set_error(error_message, "failed while hashing RINEX NAV input");
        return false;
    }
    std::ostringstream hash_stream;
    hash_stream << std::hex << std::nouppercase << std::setw(16) << std::setfill('0') << value;
    *hash = hash_stream.str();
    *size = byte_count;
    *name = std::filesystem::path(path).filename().generic_string();
    return true;
}

bool open_csv(std::ofstream* stream, const std::filesystem::path& path, const char* header,
              std::string* error_message) {
    stream->open(path, std::ios::binary | std::ios::trunc);
    if (!*stream) {
        set_error(error_message, std::string("cannot open truth CSV: ") + path.generic_string());
        return false;
    }
    stream->imbue(std::locale::classic());
    *stream << header << '\n';
    if (!*stream) {
        set_error(error_message, std::string("cannot write truth CSV header: ") + path.generic_string());
        return false;
    }
    return true;
}

bool stream_ok(std::ofstream& stream, const char* name, std::string* error_message) {
    if (stream) {
        return true;
    }
    set_error(error_message, std::string("failed to write ") + name);
    return false;
}

void write_time_prefix(std::ofstream& stream, const SimTime& time) {
    stream << time.gps_week << ',' << time.tow_ns << ',' << std::fixed << std::setprecision(9)
           << sim_time_sow_sec(time);
}

int satellite_prn(int satellite_number) {
    char satellite_id[4]{};
    if (!rtklib_satellite_number_to_id(satellite_number, satellite_id)) {
        return 0;
    }
    return std::atoi(satellite_id + 1);
}

bool write_event_row(TruthWriter* writer, const SimTime& time, const char* event_type,
                     const ScenarioEpochState& scenario, StartupMode startup_mode, std::string* error_message) {
    write_time_prefix(writer->event_stream, time);
    writer->event_stream << ',' << event_type << ',' << scenario.cycle_index << ','
                         << (scenario.receiver_powered ? 1 : 0) << ',' << (scenario.signal_available ? 1 : 0) << ','
                         << startup_mode_name(startup_mode) << '\n';
    return stream_ok(writer->event_stream, "event_truth.csv", error_message);
}

} // namespace

TruthWriter* create_truth_writer(const char* receiver_log_path, const char* rinex_nav_path, const SimConfig& config,
                                 const SimTime& start_time, std::string* error_message) {
    if (receiver_log_path == nullptr || receiver_log_path[0] == '\0' || rinex_nav_path == nullptr ||
        rinex_nav_path[0] == '\0') {
        set_error(error_message, "truth writer request has invalid paths");
        return nullptr;
    }

    TruthWriter* writer = new (std::nothrow) TruthWriter{};
    if (writer == nullptr) {
        set_error(error_message, "cannot allocate truth writer");
        return nullptr;
    }
    writer->config = config;
    writer->start_time = start_time;
    std::filesystem::path receiver_path(receiver_log_path);
    writer->output_directory = receiver_path.parent_path();
    if (writer->output_directory.empty()) {
        writer->output_directory = ".";
    }
    if (!input_identity(rinex_nav_path, &writer->rinex_nav_name, &writer->rinex_nav_hash, &writer->rinex_nav_size,
                        error_message)) {
        delete writer;
        return nullptr;
    }

    const char* event_header =
        "gps_week,tow_ns,sow_sec,event_type,cycle_index,receiver_powered,signal_available,startup_mode";
    const char* observation_header =
        "gps_week,tow_ns,sow_sec,observation_index,system,prn,satellite_number,signal_id,signal_name,glonass_fcn,"
        "wavelength_m,receiver_x_m,receiver_y_m,receiver_z_m,receiver_vx_mps,receiver_vy_mps,receiver_vz_mps,"
        "transmit_week,transmit_tow_ns,transmit_sow_sec,satellite_x_m,satellite_y_m,satellite_z_m,"
        "satellite_vx_mps,satellite_vy_mps,satellite_vz_mps,azimuth_deg,elevation_deg,geometric_range_m,"
        "range_rate_mps,satellite_clock_bias_m,satellite_clock_drift_mps,receiver_clock_bias_m,"
        "receiver_clock_drift_mps,ionosphere_m,troposphere_m,broadcast_message_family,tgd_sec_0,tgd_sec_1,"
        "tgd_sec_2,tgd_sec_3,isc_sec_0,isc_sec_1,isc_sec_2,isc_sec_3,isc_sec_4,isc_sec_5,glonass_dtaun_sec,"
        "code_bias_m,code_bias_status,cn0_dbhz,tracking_phase,lock_time_ns,pseudorange_valid,doppler_valid,adr_valid,"
        "ambiguity_cycles,ambiguity_epoch_week,ambiguity_epoch_tow_ns,cycle_slip,pseudorange_m,doppler_hz,adr_cycles";
    const char* solution_header =
        "gps_week,tow_ns,sow_sec,tracked_satellites,position_valid,position_status,position_type,"
        "latitude_deg,longitude_deg,height_m,position_x_m,position_y_m,position_z_m,latitude_std_m,"
        "longitude_std_m,height_std_m,receiver_clock_bias_m,position_used_satellites,position_diagnostic,"
        "velocity_valid,velocity_status,velocity_type,velocity_x_mps,velocity_y_mps,velocity_z_mps,"
        "horizontal_speed_mps,track_over_ground_deg,vertical_speed_mps,receiver_clock_drift_mps,"
        "velocity_used_satellites,velocity_diagnostic";

    if (!open_csv(&writer->event_stream, writer->output_directory / "event_truth.csv", event_header, error_message) ||
        !open_csv(&writer->observation_stream, writer->output_directory / "observation_truth.csv", observation_header,
                  error_message) ||
        !open_csv(&writer->solution_stream, writer->output_directory / "solution_truth.csv", solution_header,
                  error_message)) {
        delete writer;
        return nullptr;
    }

    std::ostringstream scenario;
    scenario.imbue(std::locale::classic());
    scenario << "{\n"
             << "  \"truth_schema_version\": " << TRUTH_OUTPUT_SCHEMA_VERSION << ",\n"
             << "  \"start_time\": {\"gps_week\": " << start_time.gps_week << ", \"tow_ns\": " << start_time.tow_ns
             << "},\n"
             << "  \"resolved_config\": " << config_json(config, 2) << "\n"
             << "}\n";
    if (!write_text_file(writer->output_directory / "scenario.json", scenario.str(), error_message)) {
        delete writer;
        return nullptr;
    }
    return writer;
}

void destroy_truth_writer(TruthWriter* writer) {
    delete writer;
}

bool truth_writer_write_scenario_events(TruthWriter* writer, const ScenarioEpochState& scenario,
                                        StartupMode startup_mode, std::string* error_message) {
    if (writer == nullptr || writer->finalized) {
        set_error(error_message, "truth scenario event writer is unavailable");
        return false;
    }
    if (scenario.power_on_transition &&
        !write_event_row(writer, scenario.time, "POWER_ON", scenario, startup_mode, error_message)) {
        return false;
    }
    if (scenario.power_off_transition &&
        !write_event_row(writer, scenario.time, "POWER_OFF", scenario, startup_mode, error_message)) {
        return false;
    }
    if (scenario.signal_on_transition &&
        !write_event_row(writer, scenario.time, "SIGNAL_ON", scenario, startup_mode, error_message)) {
        return false;
    }
    if (scenario.signal_off_transition &&
        !write_event_row(writer, scenario.time, "SIGNAL_OFF", scenario, startup_mode, error_message)) {
        return false;
    }
    return true;
}

bool truth_writer_write_observation(TruthWriter* writer, const ReceiverTruth& receiver,
                                    const SatelliteGeometry& geometry, const SignalTracker& tracker,
                                    const MeasurementObservation& observation, std::string* error_message) {
    if (writer == nullptr || writer->finalized) {
        set_error(error_message, "truth observation writer is unavailable");
        return false;
    }
    const SignalDefinition* definition = find_signal_definition(observation.signal_id);
    if (definition == nullptr || geometry.satellite_number != observation.satellite_number) {
        set_error(error_message, "truth observation has inconsistent signal/geometry metadata");
        return false;
    }
    if (!writer->have_observation_time || compare_sim_time(writer->last_observation_time, geometry.receive_time) != 0) {
        writer->have_observation_time = true;
        writer->last_observation_time = geometry.receive_time;
        writer->observation_index = 0;
    }

    SimTime transmit_time{};
    if (!sim_time_from_week_sow(geometry.transmit_gps_week, geometry.transmit_sow_sec, &transmit_time)) {
        set_error(error_message, "truth observation transmit time cannot be normalized");
        return false;
    }

    std::ofstream& output = writer->observation_stream;
    write_time_prefix(output, geometry.receive_time);
    output << ',' << writer->observation_index++ << ',' << constellation_name(definition->constellation) << ','
           << satellite_prn(observation.satellite_number) << ',' << observation.satellite_number << ','
           << static_cast<int>(observation.signal_id) << ',' << csv_escape(definition->name) << ','
           << observation.glonass_fcn << ',' << std::scientific << std::setprecision(17) << observation.wavelength_m
           << ',' << receiver.position_ecef_m[0] << ',' << receiver.position_ecef_m[1] << ','
           << receiver.position_ecef_m[2] << ',' << receiver.velocity_ecef_mps[0] << ','
           << receiver.velocity_ecef_mps[1] << ',' << receiver.velocity_ecef_mps[2] << ',' << transmit_time.gps_week
           << ',' << transmit_time.tow_ns << ',' << std::fixed << std::setprecision(9) << geometry.transmit_sow_sec
           << ',' << std::scientific << std::setprecision(17) << geometry.satellite_state.position_ecef_m[0] << ','
           << geometry.satellite_state.position_ecef_m[1] << ',' << geometry.satellite_state.position_ecef_m[2] << ','
           << geometry.satellite_state.velocity_ecef_mps[0] << ',' << geometry.satellite_state.velocity_ecef_mps[1]
           << ',' << geometry.satellite_state.velocity_ecef_mps[2] << ',' << geometry.azimuth_rad * kRadiansToDegrees
           << ',' << geometry.elevation_rad * kRadiansToDegrees << ',' << observation.geometric_range_m << ','
           << observation.range_rate_mps << ',' << observation.satellite_clock_bias_m << ','
           << observation.satellite_clock_drift_mps << ',' << writer->config.receiver_clock_bias_m << ','
           << writer->config.receiver_clock_drift_mps << ',' << observation.ionosphere_code_delay_m << ','
           << observation.troposphere_delay_m << ','
           << broadcast_message_family_name(observation.broadcast_message_family) << ',' << observation.tgd_sec[0]
           << ',' << observation.tgd_sec[1] << ',' << observation.tgd_sec[2] << ',' << observation.tgd_sec[3] << ','
           << observation.isc_sec[0] << ',' << observation.isc_sec[1] << ',' << observation.isc_sec[2] << ','
           << observation.isc_sec[3] << ',' << observation.isc_sec[4] << ',' << observation.isc_sec[5] << ','
           << observation.glonass_dtaun_sec << ',' << observation.code_bias_m << ','
           << broadcast_code_bias_status_name(observation.code_bias_status) << ',' << observation.cn0_dbhz << ','
           << signal_tracking_phase_name(tracker.phase) << ',' << observation.lock_time_ns << ','
           << (observation.pseudorange_valid ? 1 : 0) << ',' << (observation.doppler_valid ? 1 : 0) << ','
           << (observation.adr_valid ? 1 : 0) << ',' << observation.ambiguity_cycles << ','
           << tracker.tracking_start_time.gps_week << ',' << tracker.tracking_start_time.tow_ns << ",0,"
           << observation.pseudorange_m << ',' << observation.doppler_hz << ',' << observation.adr_cycles << '\n';
    return stream_ok(output, "observation_truth.csv", error_message);
}

bool truth_writer_write_solution(TruthWriter* writer, const SolutionEpoch& solution, int tracked_satellites,
                                 std::string* error_message) {
    if (writer == nullptr || writer->finalized || tracked_satellites < 0) {
        set_error(error_message, "truth solution writer request has invalid arguments");
        return false;
    }
    const PositionSolution& position = solution.position;
    const VelocitySolution& velocity = solution.velocity;
    std::ofstream& output = writer->solution_stream;
    write_time_prefix(output, solution.time);
    output << ',' << tracked_satellites << ',' << (position.valid ? 1 : 0) << ','
           << receiver_solution_status_name(position.status) << ',' << receiver_solution_type_name(position.type) << ','
           << std::scientific << std::setprecision(17) << position.latitude_deg << ',' << position.longitude_deg << ','
           << position.height_m << ',' << position.position_ecef_m[0] << ',' << position.position_ecef_m[1] << ','
           << position.position_ecef_m[2] << ',' << position.latitude_std_m << ',' << position.longitude_std_m << ','
           << position.height_std_m << ',' << position.receiver_clock_bias_m << ',' << position.used_satellites << ','
           << csv_escape(position.diagnostic) << ',' << (velocity.valid ? 1 : 0) << ','
           << receiver_solution_status_name(velocity.status) << ',' << receiver_solution_type_name(velocity.type) << ','
           << velocity.velocity_ecef_mps[0] << ',' << velocity.velocity_ecef_mps[1] << ','
           << velocity.velocity_ecef_mps[2] << ',' << velocity.horizontal_speed_mps << ','
           << velocity.track_over_ground_deg << ',' << velocity.vertical_speed_mps << ','
           << velocity.receiver_clock_drift_mps << ',' << velocity.used_satellites << ','
           << csv_escape(velocity.diagnostic) << '\n';
    return stream_ok(output, "solution_truth.csv", error_message);
}

bool finalize_truth_writer(TruthWriter* writer, const SimulatorRunSummary& summary, const char* version,
                           const char* commit_sha, const char* rtklib_sha, std::string* error_message) {
    if (writer == nullptr || writer->finalized || version == nullptr || commit_sha == nullptr ||
        rtklib_sha == nullptr) {
        set_error(error_message, "truth writer finalization request has invalid arguments");
        return false;
    }
    writer->event_stream.flush();
    writer->observation_stream.flush();
    writer->solution_stream.flush();
    if (!writer->event_stream || !writer->observation_stream || !writer->solution_stream) {
        set_error(error_message, "failed to flush streamed truth CSV output");
        return false;
    }
    writer->event_stream.close();
    writer->observation_stream.close();
    writer->solution_stream.close();

    const char* cn0_hash_algorithm = summary.cn0_model_hash.empty() ? "none" : "fnv1a64";
    std::ostringstream manifest;
    manifest.imbue(std::locale::classic());
    manifest << "{\n"
             << "  \"output_format_version\": " << TRUTH_OUTPUT_SCHEMA_VERSION << ",\n"
             << "  \"simulator_version\": \"" << json_escape(version) << "\",\n"
             << "  \"simulator_commit_sha\": \"" << json_escape(commit_sha) << "\",\n"
             << "  \"rtklib_commit_sha\": \"" << json_escape(rtklib_sha) << "\",\n"
             << "  \"rinex_nav\": {\n"
             << "    \"name\": \"" << json_escape(writer->rinex_nav_name.c_str()) << "\",\n"
             << "    \"hash_algorithm\": \"fnv1a64\",\n"
             << "    \"hash\": \"" << writer->rinex_nav_hash << "\",\n"
             << "    \"size_bytes\": " << writer->rinex_nav_size << "\n"
             << "  },\n"
             << "  \"cn0_model\": {\n"
             << "    \"source\": \"" << json_escape(summary.cn0_model_source.c_str()) << "\",\n"
             << "    \"schema_version\": \"" << json_escape(summary.cn0_model_schema_version.c_str()) << "\",\n"
             << "    \"name\": \"" << json_escape(summary.cn0_model_name.c_str()) << "\",\n"
             << "    \"hash_algorithm\": \"" << cn0_hash_algorithm << "\",\n"
             << "    \"hash\": \"" << summary.cn0_model_hash << "\",\n"
             << "    \"size_bytes\": " << summary.cn0_model_size_bytes << "\n"
             << "  },\n"
             << "  \"start_time\": {\"gps_week\": " << writer->start_time.gps_week
             << ", \"tow_ns\": " << writer->start_time.tow_ns << "},\n"
             << "  \"random_seed\": " << writer->config.seed << ",\n"
             << "  \"resolved_config\": " << config_json(writer->config, 2) << ",\n"
             << "  \"run_summary\": {\n"
             << "    \"scheduled_epochs\": " << summary.scheduled_epochs << ",\n"
             << "    \"powered_epochs\": " << summary.powered_epochs << ",\n"
             << "    \"signal_on_epochs\": " << summary.signal_on_epochs << ",\n"
             << "    \"signal_off_epochs\": " << summary.signal_off_epochs << ",\n"
             << "    \"range_messages\": " << summary.range_messages << ",\n"
             << "    \"psrpos_messages\": " << summary.psrpos_messages << ",\n"
             << "    \"psrvel_messages\": " << summary.psrvel_messages << ",\n"
             << "    \"bestpos_messages\": " << summary.bestpos_messages << ",\n"
             << "    \"nav_messages\": " << summary.nav_messages << ",\n"
             << "    \"valid_position_epochs\": " << summary.valid_position_epochs << ",\n"
             << "    \"valid_velocity_epochs\": " << summary.valid_velocity_epochs << ",\n"
             << "    \"max_observations_per_epoch\": " << summary.max_observations_per_epoch << "\n"
             << "  }\n"
             << "}\n";
    if (!write_text_file(writer->output_directory / "run_manifest.json", manifest.str(), error_message)) {
        return false;
    }
    writer->finalized = true;
    return true;
}

} // namespace gnss_sim
