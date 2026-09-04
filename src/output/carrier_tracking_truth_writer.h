#ifndef GNSS_SIM_SRC_OUTPUT_CARRIER_TRACKING_TRUTH_WRITER_H_
#define GNSS_SIM_SRC_OUTPUT_CARRIER_TRACKING_TRUTH_WRITER_H_

#include "gnss/satellite_engine.h"
#include "gnss/signal_definitions.h"
#include "model/carrier_tracking_runtime.h"
#include "model/measurement_model.h"
#include "model/signal_tracking.h"

#include <string>

namespace gnss_sim {

struct CarrierTrackingTruthWriter;

constexpr int CARRIER_TRACKING_TRUTH_SCHEMA_VERSION = 1;

enum class CarrierTrackingTruthResetReason {
    kNone,
    kFeatureDisabled,
    kCodeNotTracking,
};

// Exact serialization snapshot assembled by the simulator at the carrier-layer
// boundary. The writer must only serialize these values; it must not recompute
// loop state, jitter, validity, propagation, or measurement effects.
struct CarrierTrackingTruthSnapshot {
    bool carrier_tracking_enabled;
    bool result_available;
    CarrierTrackingTruthResetReason reset_reason;
    double coherent_integration_sec;
    double effective_cn0_dbhz;
    CarrierTrackingRuntimeResult runtime_result;
    CarrierTrackingState runtime_state;

    bool environmental_range_rate_applicable;
    bool environmental_range_rate_valid;
    double environmental_range_rate_mps;

    bool physical_snapshot_available;
    MeasurementObservation physical_observation;
    bool physical_range_rate_valid;

    bool post_carrier_snapshot_available;
    MeasurementObservation post_carrier_observation;
    bool post_carrier_range_rate_valid;
};

CarrierTrackingTruthWriter* create_carrier_tracking_truth_writer(const char* receiver_log_path,
                                                                 std::string* error_message);
void destroy_carrier_tracking_truth_writer(CarrierTrackingTruthWriter* writer);

bool carrier_tracking_truth_writer_write_signal(CarrierTrackingTruthWriter* writer, const SatelliteGeometry& geometry,
                                                const SignalDefinition& signal, int glonass_fcn,
                                                const SignalTracker& tracker,
                                                const CarrierTrackingTruthSnapshot& snapshot,
                                                std::string* error_message);

bool finalize_carrier_tracking_truth_writer(CarrierTrackingTruthWriter* writer, std::string* error_message);

const char* carrier_tracking_truth_reset_reason_name(CarrierTrackingTruthResetReason reason);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_OUTPUT_CARRIER_TRACKING_TRUTH_WRITER_H_
