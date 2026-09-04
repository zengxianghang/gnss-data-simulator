#include "output/urban_truth_writer.h"

#include "gnss_sim/sim_time.h"
#include "model/urban_rooftop_diffraction.h"
#include "model/urban_scene_geometry.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <new>
#include <string>

namespace gnss_sim {

struct UrbanTruthWriter {
    std::ofstream signal_stream;
    std::ofstream path_stream;
    bool finalized;
};

namespace {

constexpr double kRadiansToDegrees = 180.0 / 3.141592653589793238462643383279502884;

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool open_csv(std::ofstream* stream, const std::filesystem::path& path, const char* header,
              std::string* error_message) {
    stream->open(path, std::ios::binary | std::ios::trunc);
    if (!*stream) {
        set_error(error_message, std::string("cannot open urban truth CSV: ") + path.generic_string());
        return false;
    }
    stream->imbue(std::locale::classic());
    *stream << header << '\n';
    if (!*stream) {
        set_error(error_message, std::string("cannot write urban truth CSV header: ") + path.generic_string());
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

void write_time(std::ofstream& stream, const SimTime& time) {
    stream << time.gps_week << ',' << time.tow_ns << ',' << std::fixed << std::setprecision(9)
           << sim_time_sow_sec(time);
}

const char* root_search_status_name(CodeTrackingDllRootSearchStatus status) {
    switch (status) {
        case CodeTrackingDllRootSearchStatus::kRootsFound:
            return "ROOTS_FOUND";
        case CodeTrackingDllRootSearchStatus::kNoRoots:
            return "NO_ROOTS";
    }
    return "UNKNOWN";
}

const char* selection_mode_name(CodeTrackingDllSelectionMode mode) {
    switch (mode) {
        case CodeTrackingDllSelectionMode::TRACKED:
            return "TRACKED";
        case CodeTrackingDllSelectionMode::ACQUISITION:
            return "ACQUISITION";
    }
    return "UNKNOWN";
}

void write_complex(std::ofstream& stream, const std::complex<double>& value) {
    stream << std::scientific << std::setprecision(17) << value.real() << ',' << value.imag();
}

void write_point(std::ofstream& stream, const EnuPoint& point) {
    stream << std::scientific << std::setprecision(17) << point.east_m << ',' << point.north_m << ',' << point.up_m;
}

bool write_path_rows(UrbanTruthWriter* writer, const SatelliteGeometry& geometry, const SignalDefinition& signal,
                     const UrbanSignalEpochResult& epoch, std::string* error_message) {
    const UrbanReceivedPathSet& received = epoch.received_paths;
    if (received.path_count <= 0) {
        return true;
    }

    std::ofstream& output = writer->path_stream;
    output << URBAN_TRUTH_SCHEMA_VERSION << ',';
    write_time(output, geometry.receive_time);
    output << ',' << geometry.satellite_number << ',' << static_cast<int>(signal.signal_id) << ",0,DIRECT_ROOF,"
           << urban_wall_id_name(received.diffraction_status == UrbanRooftopDiffractionStatus::VALID
                                     ? received.diffraction.wall_id
                                     : received.direct_geometry.primary_wall)
           << ',';
    if (received.diffraction_status == UrbanRooftopDiffractionStatus::VALID) {
        write_point(output, received.diffraction.diffraction_point_enu_m);
        output << ',' << std::scientific << std::setprecision(17) << received.diffraction.model_path_range_m << ','
               << received.diffraction.excess_path_length_m << ',' << received.paths[0].code_delay_sec << ',';
    } else {
        output << ",,," << std::scientific << std::setprecision(17) << geometry.geometric_range_m << ",0,"
               << received.paths[0].code_delay_sec << ',';
    }
    write_complex(output, received.paths[0].complex_voltage);
    output << ',' << std::abs(received.paths[0].complex_voltage) << ',' << std::arg(received.paths[0].complex_voltage)
           << ',';
    if (received.diffraction_status == UrbanRooftopDiffractionStatus::VALID) {
        output << received.diffraction.fresnel_v << ',';
        write_complex(output, received.diffraction.fresnel_coefficient);
    } else {
        output << ",,";
    }
    output << ",,,,,,,,,\n";
    if (!stream_ok(output, "urban_path_truth.csv", error_message)) {
        return false;
    }

    for (int index = 0; index < received.reflections.path_count; ++index) {
        const UrbanFirstOrderReflectionPath& reflection = received.reflections.paths[index];
        const int received_index = index + 1;
        if (received_index >= received.path_count) {
            set_error(error_message, "urban truth reflection/path ordering is inconsistent");
            return false;
        }
        output << URBAN_TRUTH_SCHEMA_VERSION << ',';
        write_time(output, geometry.receive_time);
        output << ',' << geometry.satellite_number << ',' << static_cast<int>(signal.signal_id) << ',' << received_index
               << ",REFLECTION," << urban_wall_id_name(reflection.wall_id) << ',';
        write_point(output, reflection.reflection_point_enu_m);
        output << ',' << std::scientific << std::setprecision(17) << reflection.model_path_range_m << ','
               << reflection.excess_path_length_m << ',' << received.paths[received_index].code_delay_sec << ',';
        write_complex(output, received.paths[received_index].complex_voltage);
        output << ',' << std::abs(received.paths[received_index].complex_voltage) << ','
               << std::arg(received.paths[received_index].complex_voltage) << ",,,,"
               << reflection.incidence_angle_rad * kRadiansToDegrees << ',';
        write_complex(output, reflection.rf_response.gamma_rhcp_from_rhcp);
        output << ',';
        write_complex(output, reflection.rf_response.gamma_lhcp_from_rhcp);
        output << ',';
        write_complex(output, reflection.antenna_rhcp_voltage);
        output << ',';
        write_complex(output, reflection.antenna_lhcp_voltage);
        output << '\n';
        if (!stream_ok(output, "urban_path_truth.csv", error_message)) {
            return false;
        }
    }
    return true;
}

} // namespace

UrbanTruthWriter* create_urban_truth_writer(const char* receiver_log_path, std::string* error_message) {
    if (receiver_log_path == nullptr || receiver_log_path[0] == '\0') {
        set_error(error_message, "urban truth writer request has invalid output path");
        return nullptr;
    }
    UrbanTruthWriter* writer = new (std::nothrow) UrbanTruthWriter{};
    if (writer == nullptr) {
        set_error(error_message, "cannot allocate urban truth writer");
        return nullptr;
    }
    std::filesystem::path output_directory = std::filesystem::path(receiver_log_path).parent_path();
    if (output_directory.empty()) {
        output_directory = ".";
    }
    const char* signal_header =
        "truth_schema_version,gps_week,tow_ns,sow_sec,satellite_number,signal_id,signal_name,glonass_fcn,"
        "azimuth_deg,elevation_deg,propagation_evaluated,direct_los,blocking_wall,grazing_roof,diffraction_status,"
        "reflection_count,received_path_count,urban_state,tracking_phase,loss_reason,open_cn0_dbhz,effective_cn0_dbhz,"
        "effective_cn0_finite,composite_power_ratio,dll_root_count,root_search_status,selection_mode,"
        "selected_root_valid,dll_code_phase_sec,dll_code_phase_chips,code_bias_m,tracked_correlation_real,"
        "tracked_correlation_imag,lock_time_ns,observation_available,pseudorange_valid,doppler_valid,adr_valid,"
        "reacquisition_event,carrier_continuity_valid,wavelength_m,wrapped_phase_rad,unwrapped_phase_rad,"
        "carrier_range_bias_m,environmental_range_rate_mps,environmental_range_rate_valid,cycle_slip_event,"
        "receiver_observation_emitted,pseudorange_m,doppler_hz,adr_cycles";
    const char* path_header =
        "truth_schema_version,gps_week,tow_ns,sow_sec,satellite_number,signal_id,path_index,path_kind,wall_id,"
        "point_e_m,point_n_m,point_u_m,model_path_range_m,excess_path_m,code_delay_sec,voltage_real,voltage_imag,"
        "voltage_amplitude,voltage_phase_rad,fresnel_v,fresnel_real,fresnel_imag,incidence_angle_deg,"
        "gamma_rhcp_real,gamma_rhcp_imag,gamma_lhcp_real,gamma_lhcp_imag,antenna_rhcp_real,antenna_rhcp_imag,"
        "antenna_lhcp_real,antenna_lhcp_imag";
    if (!open_csv(&writer->signal_stream, output_directory / "urban_signal_truth.csv", signal_header, error_message) ||
        !open_csv(&writer->path_stream, output_directory / "urban_path_truth.csv", path_header, error_message)) {
        delete writer;
        return nullptr;
    }
    return writer;
}

void destroy_urban_truth_writer(UrbanTruthWriter* writer) {
    delete writer;
}

bool urban_truth_writer_write_signal(UrbanTruthWriter* writer, const SatelliteGeometry& geometry,
                                     const SignalDefinition& signal, int glonass_fcn,
                                     const UrbanSignalEpochResult& epoch, const UrbanCarrierTemporalResult& temporal,
                                     const MeasurementObservation* observation, std::string* error_message) {
    if (writer == nullptr || writer->finalized || geometry.satellite_number <= 0) {
        set_error(error_message, "urban truth signal writer request is invalid");
        return false;
    }
    const UrbanReceivedPathSet& received = epoch.received_paths;
    const bool propagation_evaluated = received.path_count > 0;
    std::ofstream& output = writer->signal_stream;
    output << URBAN_TRUTH_SCHEMA_VERSION << ',';
    write_time(output, geometry.receive_time);
    output << ',' << geometry.satellite_number << ',' << static_cast<int>(signal.signal_id) << ',' << signal.name << ','
           << glonass_fcn << ',' << std::scientific << std::setprecision(17) << geometry.azimuth_rad * kRadiansToDegrees
           << ',' << geometry.elevation_rad * kRadiansToDegrees << ',' << (propagation_evaluated ? 1 : 0) << ',';
    if (propagation_evaluated) {
        output << (received.direct_geometry.line_of_sight ? 1 : 0) << ','
               << urban_wall_id_name(received.direct_geometry.primary_wall) << ','
               << (received.direct_geometry.grazing_roof ? 1 : 0) << ','
               << urban_rooftop_diffraction_status_name(received.diffraction_status);
    } else {
        output << ",NONE,,NOT_EVALUATED";
    }
    output << ',' << received.reflections.path_count << ',' << received.path_count << ','
           << urban_signal_state_name(epoch.urban_state) << ',' << signal_tracking_phase_name(epoch.tracking_phase)
           << ',' << signal_tracking_loss_reason_name(epoch.loss_reason) << ',';
    if (propagation_evaluated) {
        output << received.open_cn0_dbhz << ',' << epoch.effective_cn0_dbhz << ','
               << (epoch.effective_cn0.finite_effective_cn0 ? 1 : 0) << ',' << epoch.effective_cn0.composite_power_ratio
               << ',' << epoch.dll_root_count << ',' << root_search_status_name(epoch.root_search_status) << ','
               << selection_mode_name(epoch.selection_mode) << ',';
    } else {
        output << ",,,,0,NOT_EVALUATED,NOT_EVALUATED,";
    }
    output << (epoch.selected_root_valid ? 1 : 0) << ',';
    if (epoch.selected_root_valid) {
        output << epoch.preselected_root.code_phase_sec << ',' << epoch.preselected_root.code_phase_chips;
    } else {
        output << ',';
    }
    output << ',' << epoch.code_bias_m << ',';
    write_complex(output, epoch.tracked_composite_correlation);
    output << ',' << epoch.lock_time_ns << ',' << (epoch.observation_available ? 1 : 0) << ','
           << (epoch.psr_valid ? 1 : 0) << ',' << (epoch.doppler_valid ? 1 : 0) << ',' << (epoch.adr_valid ? 1 : 0)
           << ',' << (epoch.reacquisition_event ? 1 : 0) << ',' << (epoch.carrier_continuity_valid ? 1 : 0) << ','
           << temporal.wavelength_m << ',' << temporal.wrapped_phase_rad << ',' << temporal.unwrapped_phase_rad << ','
           << temporal.carrier_range_bias_m << ',' << temporal.environmental_range_rate_mps << ','
           << (temporal.environmental_range_rate_valid ? 1 : 0) << ',' << (temporal.cycle_slip_event ? 1 : 0) << ','
           << (observation != nullptr ? 1 : 0) << ',';
    if (observation != nullptr) {
        output << observation->pseudorange_m << ',' << observation->doppler_hz << ',' << observation->adr_cycles;
    } else {
        output << ",,";
    }
    output << '\n';
    if (!stream_ok(output, "urban_signal_truth.csv", error_message)) {
        return false;
    }
    return write_path_rows(writer, geometry, signal, epoch, error_message);
}

bool finalize_urban_truth_writer(UrbanTruthWriter* writer, std::string* error_message) {
    if (writer == nullptr || writer->finalized) {
        set_error(error_message, "urban truth writer finalization request is invalid");
        return false;
    }
    writer->signal_stream.flush();
    writer->path_stream.flush();
    if (!writer->signal_stream || !writer->path_stream) {
        set_error(error_message, "failed to flush urban truth CSV output");
        return false;
    }
    writer->signal_stream.close();
    writer->path_stream.close();
    writer->finalized = true;
    return true;
}

} // namespace gnss_sim
