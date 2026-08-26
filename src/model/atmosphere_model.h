#ifndef GNSS_SIM_SRC_MODEL_ATMOSPHERE_MODEL_H_
#define GNSS_SIM_SRC_MODEL_ATMOSPHERE_MODEL_H_

#include "gnss/rtklib_adapter.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_types.h"

#include <string>

namespace gnss_sim {

enum class AtmosphereMode {
    kNone,
    kBroadcast,
};

enum class IonosphereCorrectionStatus {
    kDisabled,
    kApplied,
    kMissingParameters,
    kUnsupportedBroadcastModel,
};

struct AtmosphereCorrection {
    AtmosphereMode mode;
    IonosphereCorrectionStatus ionosphere_status;
    double ionosphere_code_delay_m;
    double troposphere_delay_m;
};

bool compute_atmosphere_correction(AtmosphereMode mode, const RtklibNavStore* nav_store, const SimTime& time,
                                   SignalId signal_id, int glonass_fcn, const double receiver_ecef_m[3],
                                   double azimuth_rad, double elevation_rad, AtmosphereCorrection* correction,
                                   std::string* error_message);

const char* atmosphere_mode_name(AtmosphereMode mode);
const char* ionosphere_correction_status_name(IonosphereCorrectionStatus status);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_ATMOSPHERE_MODEL_H_
