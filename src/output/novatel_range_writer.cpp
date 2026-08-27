#include "output/novatel_range_writer.h"

#include "gnss/rtklib_adapter.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_time.h"
#include "output/novatel_ascii.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <vector>

namespace gnss_sim {
namespace {

constexpr double kPseudorangeSigmaM = 0.500;
constexpr double kAdrSigmaCycles = 0.050;
constexpr int kGlonassMinFcn = -7;
constexpr int kGlonassMaxFcn = 6;

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

char constellation_id(GnssConstellation constellation) {
    switch (constellation) {
        case GnssConstellation::kGps:
            return 'G';
        case GnssConstellation::kGlonass:
            return 'R';
        case GnssConstellation::kGalileo:
            return 'E';
        case GnssConstellation::kBeidou:
            return 'C';
        case GnssConstellation::kQzss:
            return 'J';
    }
    return '\0';
}

unsigned int constellation_status_bits(GnssConstellation constellation) {
    switch (constellation) {
        case GnssConstellation::kGps:
            return 0U;
        case GnssConstellation::kGlonass:
            return 1U;
        case GnssConstellation::kGalileo:
            return 3U;
        case GnssConstellation::kBeidou:
            return 4U;
        case GnssConstellation::kQzss:
            return 5U;
    }
    return 7U;
}

bool satellite_fields(const SignalDefinition& definition, const MeasurementObservation& observation, int* range_prn,
                      int* glofreq) {
    if (range_prn == nullptr || glofreq == nullptr) {
        return false;
    }
    char satellite_id[4]{};
    if (!rtklib_satellite_number_to_id(observation.satellite_number, satellite_id) ||
        satellite_id[0] != constellation_id(definition.constellation) || satellite_id[1] < '0' ||
        satellite_id[1] > '9' || satellite_id[2] < '0' || satellite_id[2] > '9') {
        return false;
    }

    const int prn = (satellite_id[1] - '0') * 10 + satellite_id[2] - '0';
    switch (definition.constellation) {
        case GnssConstellation::kGlonass:
            if (observation.glonass_fcn < kGlonassMinFcn || observation.glonass_fcn > kGlonassMaxFcn) {
                return false;
            }
            *range_prn = prn + 37;
            *glofreq = observation.glonass_fcn + 7;
            return true;
        case GnssConstellation::kQzss:
            *range_prn = prn + 192;
            *glofreq = 0;
            return true;
        case GnssConstellation::kGps:
        case GnssConstellation::kGalileo:
        case GnssConstellation::kBeidou:
            *range_prn = prn;
            *glofreq = 0;
            return true;
    }
    return false;
}

unsigned int tracking_status(const SignalDefinition& definition, const MeasurementObservation& observation) {
    unsigned int status = observation.adr_valid ? 4U : 7U;
    if (observation.adr_valid) {
        status |= 1U << 10U;
        status |= 1U << 11U;
    }
    if (observation.pseudorange_valid) {
        status |= 1U << 12U;
    }
    status |= (constellation_status_bits(definition.constellation) & 7U) << 16U;
    status |= (static_cast<unsigned int>(definition.novatel_oem7_signal_type) & 0x1FU) << 21U;
    return status;
}

bool finite_observation(const MeasurementObservation& observation) {
    return std::isfinite(observation.cn0_dbhz) && observation.cn0_dbhz >= 0.0 && observation.lock_time_ns >= 0 &&
           (!observation.pseudorange_valid ||
            (std::isfinite(observation.pseudorange_m) && observation.pseudorange_m > 0.0)) &&
           (!observation.adr_valid || std::isfinite(observation.adr_cycles)) &&
           (!observation.doppler_valid || std::isfinite(observation.doppler_hz));
}

} // namespace

bool format_novatel_rangea(const SimTime& time, const MeasurementObservation* observations, int observation_count,
                           std::string* message, std::string* error_message) {
    if (observation_count < 0 || (observation_count > 0 && observations == nullptr) || message == nullptr) {
        set_error(error_message, "RANGEA writer received invalid arguments");
        return false;
    }

    std::vector<const MeasurementObservation*> emitted;
    emitted.reserve(static_cast<std::size_t>(observation_count));
    for (int index = 0; index < observation_count; ++index) {
        if (observations[index].observation_available) {
            emitted.push_back(&observations[index]);
        }
    }
    std::sort(emitted.begin(), emitted.end(), [](const MeasurementObservation* lhs, const MeasurementObservation* rhs) {
        if (lhs->satellite_number != rhs->satellite_number) {
            return lhs->satellite_number < rhs->satellite_number;
        }
        return static_cast<int>(lhs->signal_id) < static_cast<int>(rhs->signal_id);
    });

    std::ostringstream body;
    body.imbue(std::locale::classic());
    body << emitted.size();
    for (const MeasurementObservation* observation : emitted) {
        const SignalDefinition* definition = find_signal_definition(observation->signal_id);
        if (definition == nullptr || definition->novatel_oem7_signal_type < 0 ||
            definition->novatel_oem7_signal_type > 31 || !finite_observation(*observation)) {
            set_error(error_message, "RANGEA observation cannot be represented deterministically");
            return false;
        }
        int range_prn = 0;
        int glofreq = 0;
        if (!satellite_fields(*definition, *observation, &range_prn, &glofreq)) {
            set_error(error_message, "RANGEA satellite mapping is invalid");
            return false;
        }

        const double pseudorange_m = observation->pseudorange_valid ? observation->pseudorange_m : 0.0;
        const double pseudorange_sigma_m = observation->pseudorange_valid ? kPseudorangeSigmaM : 0.0;
        const double adr_cycles = observation->adr_valid ? observation->adr_cycles : 0.0;
        const double adr_sigma_cycles = observation->adr_valid ? kAdrSigmaCycles : 0.0;
        const double doppler_hz = observation->doppler_valid ? observation->doppler_hz : 0.0;
        const double lock_time_sec = static_cast<double>(observation->lock_time_ns) /
                                     static_cast<double>(NANOSECONDS_PER_SECOND);

        body << ',' << range_prn << ',' << glofreq << ',' << std::fixed << std::setprecision(3) << pseudorange_m << ','
             << pseudorange_sigma_m << ',' << std::setprecision(6) << adr_cycles << ',' << std::setprecision(3)
             << adr_sigma_cycles << ',' << doppler_hz << ',' << std::setprecision(1) << observation->cn0_dbhz << ','
             << std::setprecision(3) << lock_time_sec << ',' << std::hex << std::nouppercase << std::setw(8)
             << std::setfill('0') << tracking_status(*definition, *observation) << std::dec << std::setfill(' ');
    }

    if (!novatel_ascii::frame("RANGEA", time, body.str(), message)) {
        set_error(error_message, "RANGEA header time cannot be represented");
        return false;
    }
    return true;
}

} // namespace gnss_sim
