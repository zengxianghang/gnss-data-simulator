#ifndef GNSS_SIM_SRC_GNSS_SIGNAL_DEFINITIONS_H_
#define GNSS_SIM_SRC_GNSS_SIGNAL_DEFINITIONS_H_

#include <cstddef>
#include <string>

namespace gnss_sim {

enum class GnssConstellation {
    kGps,
    kGlonass,
    kGalileo,
    kBeidou,
    kQzss,
};

enum class SignalId {
    kGpsL1Ca,
    kGpsL1C,
    kGpsL2P,
    kGpsL2C,
    kGpsL5Q,
    kQzssL1Ca,
    kQzssL1C,
    kQzssL2C,
    kQzssL5Q,
    kGlonassG1,
    kGlonassG2,
    kGlonassG3,
    kGalileoE1,
    kGalileoE5A,
    kGalileoE5B,
    kGalileoE6,
    kBeidouB1I,
    kBeidouB3I,
    kBeidouB1C,
    kBeidouB2A,
    kBeidouB2B,
};

enum class CarrierFrequencyModel {
    kFixed,
    kGlonassFdmaG1,
    kGlonassFdmaG2,
};

enum class NavMessageFamily {
    kGpsLnav,
    kGpsCnav,
    kGpsCnav2,
    kQzssLnav,
    kQzssCnav,
    kQzssCnav2,
    kGlonassFdma,
    kGlonassL3Oc,
    kGalileoInav,
    kGalileoFnav,
    kGalileoCnav,
    kBeidouD1D2,
    kBeidouBcnav1,
    kBeidouBcnav2,
    kBeidouBcnav3,
};

// Semantic selector for the signal-specific broadcast group-delay/code-bias
// model. The measurement model resolves the selected semantic against the
// actual RINEX NAV message (TGD, ISC, BGD, or GLONASS dtaun) rather than
// exposing eph_t/geph_t slots outside the RTKLIB adapter boundary.
enum class CodeBiasModel {
    kGpsL1Ca,
    kGpsL1C,
    kGpsL2P,
    kGpsL2C,
    kGpsL5Q,
    kQzssL1Ca,
    kQzssL1C,
    kQzssL2C,
    kQzssL5Q,
    kGlonassG1,
    kGlonassG2,
    kGlonassG3,
    kGalileoE1,
    kGalileoE5A,
    kGalileoE5B,
    kGalileoE6,
    kBeidouB1I,
    kBeidouB3I,
    kBeidouB1C,
    kBeidouB2A,
    kBeidouB2B,
};

// Ideal code-domain correlation family used by the V1 DLL model. This is
// intentionally narrower than a full IF/baseband signal description.
enum class CodeCorrelationModel {
    kUnsupported = 0,
    kBpsk,
    kTmboc,
    kCboc,
    kQmboc,
};

// Relationship of a secondary subcarrier to the primary subcarrier for
// composite correlation models. TMBOC time multiplexing does not use a fixed
// phase relation and therefore uses kNotApplicable.
enum class CompositeSubcarrierPhase {
    kNotApplicable = 0,
    kInPhase,
    kAntiPhase,
    kQuadrature,
};

struct CodeCorrelationProfile {
    CodeCorrelationModel model;
    double chip_rate_hz;
    double primary_subcarrier_rate_hz;
    double secondary_subcarrier_rate_hz;
    double secondary_power_fraction;
    CompositeSubcarrierPhase secondary_phase;
};

struct SignalDefinition {
    GnssConstellation constellation;
    SignalId signal_id;
    const char* name;
    const char* rinex_signal_code;
    CarrierFrequencyModel carrier_model;
    double nominal_frequency_hz;
    NavMessageFamily nav_message_family;
    CodeBiasModel code_bias_model;
    int novatel_oem7_signal_type;
    CodeCorrelationProfile code_correlation;
};

const SignalDefinition* signal_definitions(std::size_t* count);
const SignalDefinition* find_signal_definition(SignalId signal_id);
const SignalDefinition* find_signal_definition_by_rinex(GnssConstellation constellation, const char* rinex_signal_code);
const SignalDefinition* find_signal_definition_by_oem7(GnssConstellation constellation, int novatel_oem7_signal_type);

bool validate_code_correlation_profile(const CodeCorrelationProfile& profile, std::string* error_message);
bool signal_has_supported_code_correlation(const SignalDefinition& definition);

bool signal_carrier_frequency_hz(const SignalDefinition& definition, int glonass_fcn, double* frequency_hz);
bool signal_wavelength_m(const SignalDefinition& definition, int glonass_fcn, double* wavelength_m);
bool signal_rtklib_observation_code(const SignalDefinition& definition, int* observation_code, int* frequency_index);

// Priority for the single-frequency RTKLIB SPP loopback input. Lower is
// preferred for a satellite; negative means the signal must not be selected as
// the primary SPP observation. The policy lives here so solution code does not
// duplicate constellation/signal mapping rules.
int signal_single_point_priority(SignalId signal_id);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_GNSS_SIGNAL_DEFINITIONS_H_
