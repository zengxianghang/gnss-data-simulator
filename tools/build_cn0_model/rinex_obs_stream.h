#ifndef GNSS_SIM_TOOLS_BUILD_CN0_MODEL_RINEX_OBS_STREAM_H_
#define GNSS_SIM_TOOLS_BUILD_CN0_MODEL_RINEX_OBS_STREAM_H_

#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_time.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace gnss_sim::cn0_builder {

enum class SignalStrengthUnitStatus {
    kDbHz,
    kMissing,
    kUnsupported,
    kConflicting,
};

enum class Cn0SampleValidity {
    kValidDbHz,
    kAmbiguousSignalStrengthUnit,
    kGeometryUnavailable,
};

struct RinexObsProvenance {
    double rinex_version{};
    std::string station_name;
    std::string marker_number;
    std::string receiver_type;
    std::string antenna_type;
    double station_ecef_m[3]{};
    std::string signal_strength_unit;
    SignalStrengthUnitStatus signal_strength_unit_status{SignalStrengthUnitStatus::kMissing};
    std::string observation_time_system;
    std::string observation_path;
    std::string navigation_path;
};

struct RinexCn0Sample {
    SimTime time{};
    GnssConstellation constellation{GnssConstellation::kGps};
    int satellite_number{};
    int prn{};
    SignalId signal_id{SignalId::kGpsL1Ca};
    std::string rinex_signal_code;
    double signal_strength_value{};
    double cn0_dbhz{};
    double azimuth_rad{};
    double elevation_rad{};
    Cn0SampleValidity validity{Cn0SampleValidity::kGeometryUnavailable};
    const RinexObsProvenance* provenance{};
};

struct RinexObsStreamSummary {
    std::uint64_t epochs{};
    std::uint64_t observation_records{};
    std::uint64_t emitted_samples{};
    std::uint64_t valid_dbhz_samples{};
    std::uint64_t ambiguous_unit_samples{};
    std::uint64_t missing_signal_strength{};
    std::uint64_t unsupported_signal_observables{};
    std::uint64_t unmapped_snr_slots{};
    std::uint64_t geometry_failures{};
    std::uint64_t out_of_order_epochs{};
    int peak_epoch_observations{};
    std::vector<std::string> unsupported_observables;
};

using RinexCn0SampleCallback = std::function<bool(const RinexCn0Sample&)>;

bool stream_rinex_cn0_samples(const std::string& observation_path, const std::string& navigation_path,
                              const RinexCn0SampleCallback& callback, RinexObsProvenance* provenance,
                              RinexObsStreamSummary* summary, std::string* error_message);

const char* signal_strength_unit_status_name(SignalStrengthUnitStatus status);
const char* cn0_sample_validity_name(Cn0SampleValidity validity);
const char* constellation_name(GnssConstellation constellation);

} // namespace gnss_sim::cn0_builder

#endif // GNSS_SIM_TOOLS_BUILD_CN0_MODEL_RINEX_OBS_STREAM_H_
