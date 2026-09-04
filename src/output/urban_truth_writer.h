#ifndef GNSS_SIM_SRC_OUTPUT_URBAN_TRUTH_WRITER_H_
#define GNSS_SIM_SRC_OUTPUT_URBAN_TRUTH_WRITER_H_

#include "gnss/satellite_engine.h"
#include "model/measurement_model.h"
#include "model/urban_carrier_temporal.h"
#include "model/urban_signal_epoch.h"

#include <string>

namespace gnss_sim {

struct UrbanTruthWriter;

constexpr int URBAN_TRUTH_SCHEMA_VERSION = 1;

UrbanTruthWriter* create_urban_truth_writer(const char* receiver_log_path, std::string* error_message);
void destroy_urban_truth_writer(UrbanTruthWriter* writer);

// Writes one per-signal summary plus the already-computed propagation paths.
// observation may be null for BLOCKED/searching/lost states. This function is
// serialization-only: it must never recompute geometry, propagation, DLL, CN0,
// tracking, carrier phase, or path rate.
bool urban_truth_writer_write_signal(UrbanTruthWriter* writer, const SatelliteGeometry& geometry,
                                     const SignalDefinition& signal, int glonass_fcn,
                                     const UrbanSignalEpochResult& epoch, const UrbanCarrierTemporalResult& temporal,
                                     const MeasurementObservation* observation, std::string* error_message);

bool finalize_urban_truth_writer(UrbanTruthWriter* writer, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_OUTPUT_URBAN_TRUTH_WRITER_H_
