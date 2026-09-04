#include "output/carrier_tracking_truth_writer.h"

#include "gnss_sim/sim_time.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <new>
#include <string>

namespace gnss_sim {

struct CarrierTrackingTruthWriter {
    std::ofstream stream;
    bool finalized;
};

namespace {

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool open_csv(std::ofstream* stream, const std::filesystem::path& path, const char* header,
              std::string* error_message) {
    stream->open(path, std::ios::binary | std::ios::trunc);
    if (!*stream) {
        set_error(error_message, std::string("cannot open carrier tracking truth CSV: ") + path.generic_string());
        return false;
    }
    stream->imbue(std::locale::classic());
    *stream << header << '\n';
    if (!*stream) {
        set_error(error_message,
                  std::string("cannot write carrier tracking truth CSV header: ") + path.generic_string());
        return false;
    }
    return true;
}

void write_time(std::ofstream& stream, const SimTime& time) {
    stream << time.gps_week << ',' << time.tow_ns << ',' << std::fixed << std::setprecision(9)
           << sim_time_sow_sec(time);
}

const char* acquisition_context_name(AcquisitionContext context) {
    switch (context) {
        case AcquisitionContext::kHot:
            return "HOT";
        case AcquisitionContext::kWarm:
            return "WARM";
        case AcquisitionContext::kCold:
            return "COLD";
        case AcquisitionContext::kReacquisition:
            return "REACQUISITION";
    }
    return "UNKNOWN";
}

void write_observation(std::ofstream& output, bool snapshot_available, const MeasurementObservation& observation,
                       bool range_rate_valid) {
    output << (snapshot_available ? 1 : 0) << ',';
    if (!snapshot_available) {
        output << ",,,,,,,,,";
        return;
    }
    output << (observation.observation_available ? 1 : 0) << ',' << (observation.pseudorange_valid ? 1 : 0) << ','
           << (observation.doppler_valid ? 1 : 0) << ',' << (observation.adr_valid ? 1 : 0) << ','
           << (range_rate_valid ? 1 : 0) << ',' << std::scientific << std::setprecision(17)
           << observation.range_rate_mps << ',' << observation.doppler_hz << ',' << observation.adr_cycles << ',';
}

} // namespace

const char* carrier_tracking_truth_reset_reason_name(CarrierTrackingTruthResetReason reason) {
    switch (reason) {
        case CarrierTrackingTruthResetReason::kNone:
            return "NONE";
        case CarrierTrackingTruthResetReason::kFeatureDisabled:
            return "FEATURE_DISABLED";
        case CarrierTrackingTruthResetReason::kCodeNotTracking:
            return "CODE_NOT_TRACKING";
    }
    return "UNKNOWN";
}

CarrierTrackingTruthWriter* create_carrier_tracking_truth_writer(const char* receiver_log_path,
                                                                 std::string* error_message) {
    if (receiver_log_path == nullptr || receiver_log_path[0] == '\0') {
        set_error(error_message, "carrier tracking truth writer request has invalid output path");
        return nullptr;
    }
    CarrierTrackingTruthWriter* writer = new (std::nothrow) CarrierTrackingTruthWriter{};
    if (writer == nullptr) {
        set_error(error_message, "cannot allocate carrier tracking truth writer");
        return nullptr;
    }
    std::filesystem::path output_directory = std::filesystem::path(receiver_log_path).parent_path();
    if (output_directory.empty()) {
        output_directory = ".";
    }
    const char* header =
        "carrier_truth_schema_version,gps_week,tow_ns,sow_sec,satellite_number,signal_id,signal_name,glonass_fcn,"
        "tracking_phase,acquisition_context,signal_loss_reason,carrier_tracking_enabled,carrier_result_available,"
        "carrier_reset_reason,coherent_integration_sec,effective_cn0_dbhz,cn0_linear_hz,carrier_mode,fll_phase,"
        "active_bandwidth_hz,phase_sigma_rad,sigma_hz,sigma_mps,correlation_tau_sec,correlation_alpha,"
        "tracking_error_hz,tracking_error_mps,mode_age_sec,carrier_lock_age_sec,pll_age_sec,"
        "fll_enter_persistence_sec,fll_exit_persistence_sec,pll_enter_persistence_sec,pll_exit_persistence_sec,"
        "carrier_segment_id,phase_segment_id,adr_cycle_offset_cycles,mode_changed,new_carrier_segment,cycle_slip_event,"
        "environmental_range_rate_applicable,environmental_range_rate_valid,environmental_range_rate_mps,"
        "physical_snapshot_available,physical_observation_available,physical_code_valid,physical_doppler_valid,"
        "physical_adr_valid,physical_range_rate_valid,physical_range_rate_mps,physical_doppler_hz,physical_adr_cycles,"
        "post_carrier_snapshot_available,post_carrier_observation_available,post_carrier_code_valid,"
        "post_carrier_doppler_valid,post_carrier_adr_valid,post_carrier_range_rate_valid,post_carrier_range_rate_mps,"
        "post_carrier_doppler_hz,post_carrier_adr_cycles";
    if (!open_csv(&writer->stream, output_directory / "carrier_tracking_truth.csv", header, error_message)) {
        delete writer;
        return nullptr;
    }
    return writer;
}

void destroy_carrier_tracking_truth_writer(CarrierTrackingTruthWriter* writer) {
    delete writer;
}

bool carrier_tracking_truth_writer_write_signal(CarrierTrackingTruthWriter* writer,
                                                const SatelliteGeometry& geometry,
                                                const SignalDefinition& signal, int glonass_fcn,
                                                const SignalTracker& tracker,
                                                const CarrierTrackingTruthSnapshot& snapshot,
                                                std::string* error_message) {
    if (writer == nullptr || writer->finalized || geometry.satellite_number <= 0) {
        set_error(error_message, "carrier tracking truth signal writer request is invalid");
        return false;
    }

    std::ofstream& output = writer->stream;
    output << CARRIER_TRACKING_TRUTH_SCHEMA_VERSION << ',';
    write_time(output, geometry.receive_time);
    output << ',' << geometry.satellite_number << ',' << static_cast<int>(signal.signal_id) << ',' << signal.name << ','
           << glonass_fcn << ',' << signal_tracking_phase_name(tracker.phase) << ','
           << acquisition_context_name(tracker.acquisition_context) << ','
           << signal_tracking_loss_reason_name(tracker.loss_reason) << ','
           << (snapshot.carrier_tracking_enabled ? 1 : 0) << ',' << (snapshot.result_available ? 1 : 0) << ','
           << carrier_tracking_truth_reset_reason_name(snapshot.reset_reason) << ',' << std::scientific
           << std::setprecision(17) << snapshot.coherent_integration_sec << ',';

    if (snapshot.result_available) {
        const CarrierTrackingResult& result = snapshot.runtime_result.tracking;
        const CarrierTrackingJitter& jitter = result.jitter;
        const CarrierTrackingState& state = snapshot.runtime_state;
        output << snapshot.effective_cn0_dbhz << ',' << jitter.cn0_linear_hz << ','
               << carrier_tracking_mode_name(result.mode) << ',' << carrier_tracking_fll_phase_name(result.fll_phase)
               << ',' << jitter.active_bandwidth_hz << ',' << jitter.phase_sigma_rad << ',' << jitter.sigma_hz << ','
               << jitter.sigma_mps << ',' << jitter.correlation_tau_sec << ',' << jitter.correlation_alpha << ','
               << result.tracking_error_hz << ',' << result.tracking_error_mps << ',' << state.mode_age_sec << ','
               << state.carrier_lock_age_sec << ',' << state.pll_age_sec << ',' << state.fll_enter_persistence_sec << ','
               << state.fll_exit_persistence_sec << ',' << state.pll_enter_persistence_sec << ','
               << state.pll_exit_persistence_sec << ',' << result.carrier_segment_id << ','
               << snapshot.runtime_result.phase_segment_id << ',' << snapshot.runtime_result.adr_cycle_offset_cycles
               << ',' << (result.mode_changed ? 1 : 0) << ',' << (result.new_carrier_segment ? 1 : 0) << ','
               << (snapshot.runtime_result.cycle_slip_event ? 1 : 0) << ',';
    } else {
        output << ",,,,,,,,,,,,,,,,,,,,,,,,,";
    }

    output << (snapshot.environmental_range_rate_applicable ? 1 : 0) << ','
           << (snapshot.environmental_range_rate_valid ? 1 : 0) << ',';
    if (snapshot.environmental_range_rate_valid) {
        output << std::scientific << std::setprecision(17) << snapshot.environmental_range_rate_mps;
    }
    output << ',';

    write_observation(output, snapshot.physical_snapshot_available, snapshot.physical_observation,
                      snapshot.physical_range_rate_valid);
    write_observation(output, snapshot.post_carrier_snapshot_available, snapshot.post_carrier_observation,
                      snapshot.post_carrier_range_rate_valid);
    output.seekp(-1, std::ios_base::cur);
    output << '\n';
    if (!output) {
        set_error(error_message, "failed to write carrier_tracking_truth.csv");
        return false;
    }
    return true;
}

bool finalize_carrier_tracking_truth_writer(CarrierTrackingTruthWriter* writer, std::string* error_message) {
    if (writer == nullptr || writer->finalized) {
        set_error(error_message, "carrier tracking truth writer finalization request is invalid");
        return false;
    }
    writer->stream.flush();
    if (!writer->stream) {
        set_error(error_message, "failed to flush carrier_tracking_truth.csv");
        return false;
    }
    writer->stream.close();
    writer->finalized = true;
    return true;
}

} // namespace gnss_sim
