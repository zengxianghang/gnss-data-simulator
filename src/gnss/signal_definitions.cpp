#include "gnss/signal_definitions.h"

#include "gnss/rtklib_adapter.h"

#include <cmath>
#include <cstring>

namespace gnss_sim {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kGlonassG1BaseHz = 1602.0e6;
constexpr double kGlonassG1StepHz = 0.5625e6;
constexpr double kGlonassG2BaseHz = 1246.0e6;
constexpr double kGlonassG2StepHz = 0.4375e6;
constexpr int kGlonassMinFcn = -7;
constexpr int kGlonassMaxFcn = 6;

constexpr CodeCorrelationProfile unsupported_correlation() {
    return {CodeCorrelationModel::kUnsupported, 0.0, 0.0, 0.0, 0.0, CompositeSubcarrierPhase::kNotApplicable};
}

constexpr CodeCorrelationProfile bpsk_correlation(double chip_rate_hz) {
    return {CodeCorrelationModel::kBpsk, chip_rate_hz, 0.0, 0.0, 0.0, CompositeSubcarrierPhase::kNotApplicable};
}

constexpr CodeCorrelationProfile tmboc_correlation(double chip_rate_hz, double primary_subcarrier_rate_hz,
                                                   double secondary_subcarrier_rate_hz,
                                                   double secondary_power_fraction) {
    return {CodeCorrelationModel::kTmboc, chip_rate_hz,
            primary_subcarrier_rate_hz,   secondary_subcarrier_rate_hz,
            secondary_power_fraction,     CompositeSubcarrierPhase::kNotApplicable};
}

constexpr CodeCorrelationProfile cboc_correlation(double chip_rate_hz, double primary_subcarrier_rate_hz,
                                                  double secondary_subcarrier_rate_hz, double secondary_power_fraction,
                                                  CompositeSubcarrierPhase secondary_phase) {
    return {CodeCorrelationModel::kCboc, chip_rate_hz,   primary_subcarrier_rate_hz, secondary_subcarrier_rate_hz,
            secondary_power_fraction,    secondary_phase};
}

constexpr CodeCorrelationProfile qmboc_correlation(double chip_rate_hz, double primary_subcarrier_rate_hz,
                                                   double secondary_subcarrier_rate_hz,
                                                   double secondary_power_fraction) {
    return {CodeCorrelationModel::kQmboc, chip_rate_hz,
            primary_subcarrier_rate_hz,   secondary_subcarrier_rate_hz,
            secondary_power_fraction,     CompositeSubcarrierPhase::kQuadrature};
}

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool finite_nonnegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool composite_rates_valid(const CodeCorrelationProfile& profile) {
    return std::isfinite(profile.chip_rate_hz) && profile.chip_rate_hz > 0.0 &&
           std::isfinite(profile.primary_subcarrier_rate_hz) && profile.primary_subcarrier_rate_hz > 0.0 &&
           std::isfinite(profile.secondary_subcarrier_rate_hz) &&
           profile.secondary_subcarrier_rate_hz > profile.primary_subcarrier_rate_hz &&
           std::isfinite(profile.secondary_power_fraction) && profile.secondary_power_fraction > 0.0 &&
           profile.secondary_power_fraction < 1.0;
}

// OEM7 signal types match the RANGE RINEX mapping table in the pinned RTKLIB
// fork. RINEX codes resolve through RTKLIB, including its modern-BDS extension.
//
// Code-correlation profiles are the tracked RINEX component, not a full RF
// composite waveform. Ambiguous profiles are explicitly unsupported rather than
// silently mapped to a generic BPSK triangle. See docs/SIGNAL_CORRELATION_METADATA.md.
constexpr SignalDefinition kSignalDefinitions[] = {
    {GnssConstellation::kGps, SignalId::kGpsL1Ca, "GPS L1 C/A", "1C", CarrierFrequencyModel::kFixed, 1575.42e6,
     NavMessageFamily::kGpsLnav, CodeBiasModel::kGpsL1Ca, 0, bpsk_correlation(1.023e6)},
    {GnssConstellation::kGps, SignalId::kGpsL1C, "GPS L1C", "1L", CarrierFrequencyModel::kFixed, 1575.42e6,
     NavMessageFamily::kGpsCnav2, CodeBiasModel::kGpsL1C, 16, tmboc_correlation(1.023e6, 1.023e6, 6.138e6, 4.0 / 33.0)},
    {GnssConstellation::kGps, SignalId::kGpsL2P, "GPS L2P", "2P", CarrierFrequencyModel::kFixed, 1227.60e6,
     NavMessageFamily::kGpsLnav, CodeBiasModel::kGpsL2P, 5, bpsk_correlation(10.23e6)},
    {GnssConstellation::kGps, SignalId::kGpsL2C, "GPS L2C", "2S", CarrierFrequencyModel::kFixed, 1227.60e6,
     NavMessageFamily::kGpsCnav, CodeBiasModel::kGpsL2C, 17, unsupported_correlation()},
    {GnssConstellation::kGps, SignalId::kGpsL5Q, "GPS L5Q", "5Q", CarrierFrequencyModel::kFixed, 1176.45e6,
     NavMessageFamily::kGpsCnav, CodeBiasModel::kGpsL5Q, 14, bpsk_correlation(10.23e6)},

    {GnssConstellation::kQzss, SignalId::kQzssL1Ca, "QZSS L1 C/A", "1C", CarrierFrequencyModel::kFixed, 1575.42e6,
     NavMessageFamily::kQzssLnav, CodeBiasModel::kQzssL1Ca, 0, bpsk_correlation(1.023e6)},
    {GnssConstellation::kQzss, SignalId::kQzssL1C, "QZSS L1C", "1L", CarrierFrequencyModel::kFixed, 1575.42e6,
     NavMessageFamily::kQzssCnav2, CodeBiasModel::kQzssL1C, 16, unsupported_correlation()},
    {GnssConstellation::kQzss, SignalId::kQzssL2C, "QZSS L2C", "2S", CarrierFrequencyModel::kFixed, 1227.60e6,
     NavMessageFamily::kQzssCnav, CodeBiasModel::kQzssL2C, 17, unsupported_correlation()},
    {GnssConstellation::kQzss, SignalId::kQzssL5Q, "QZSS L5Q", "5Q", CarrierFrequencyModel::kFixed, 1176.45e6,
     NavMessageFamily::kQzssCnav, CodeBiasModel::kQzssL5Q, 14, bpsk_correlation(10.23e6)},

    {GnssConstellation::kGlonass, SignalId::kGlonassG1, "GLONASS G1", "1C", CarrierFrequencyModel::kGlonassFdmaG1,
     kGlonassG1BaseHz, NavMessageFamily::kGlonassFdma, CodeBiasModel::kGlonassG1, 0, bpsk_correlation(0.511e6)},
    {GnssConstellation::kGlonass, SignalId::kGlonassG2, "GLONASS G2", "2C", CarrierFrequencyModel::kGlonassFdmaG2,
     kGlonassG2BaseHz, NavMessageFamily::kGlonassFdma, CodeBiasModel::kGlonassG2, 1, bpsk_correlation(0.511e6)},
    {GnssConstellation::kGlonass, SignalId::kGlonassG3, "GLONASS G3", "3Q", CarrierFrequencyModel::kFixed, 1202.025e6,
     NavMessageFamily::kGlonassL3Oc, CodeBiasModel::kGlonassG3, 6, bpsk_correlation(10.23e6)},

    {GnssConstellation::kGalileo, SignalId::kGalileoE1, "Galileo E1", "1C", CarrierFrequencyModel::kFixed, 1575.42e6,
     NavMessageFamily::kGalileoInav, CodeBiasModel::kGalileoE1, 2,
     cboc_correlation(1.023e6, 1.023e6, 6.138e6, 1.0 / 11.0, CompositeSubcarrierPhase::kAntiPhase)},
    {GnssConstellation::kGalileo, SignalId::kGalileoE5A, "Galileo E5a", "5Q", CarrierFrequencyModel::kFixed, 1176.45e6,
     NavMessageFamily::kGalileoFnav, CodeBiasModel::kGalileoE5A, 12, bpsk_correlation(10.23e6)},
    {GnssConstellation::kGalileo, SignalId::kGalileoE5B, "Galileo E5b", "7Q", CarrierFrequencyModel::kFixed, 1207.14e6,
     NavMessageFamily::kGalileoInav, CodeBiasModel::kGalileoE5B, 17, bpsk_correlation(10.23e6)},
    // Galileo HAS is transmitted on the E6-B data component, but the HAS Initial
    // Service code-bias observable is E6-C (RINEX C6C). OEM7 RANGE signal type
    // 7 is E6C; type 6 would be E6B/C6B and must not consume a C6C OSB.
    {GnssConstellation::kGalileo, SignalId::kGalileoE6, "Galileo E6", "6C", CarrierFrequencyModel::kFixed, 1278.75e6,
     NavMessageFamily::kGalileoCnav, CodeBiasModel::kGalileoE6, 7, bpsk_correlation(5.115e6)},

    {GnssConstellation::kBeidou, SignalId::kBeidouB1I, "BeiDou B1I", "2I", CarrierFrequencyModel::kFixed, 1561.098e6,
     NavMessageFamily::kBeidouD1D2, CodeBiasModel::kBeidouB1I, 0, bpsk_correlation(2.046e6)},
    {GnssConstellation::kBeidou, SignalId::kBeidouB3I, "BeiDou B3I", "6I", CarrierFrequencyModel::kFixed, 1268.52e6,
     NavMessageFamily::kBeidouD1D2, CodeBiasModel::kBeidouB3I, 2, bpsk_correlation(10.23e6)},
    {GnssConstellation::kBeidou, SignalId::kBeidouB1C, "BeiDou B1C", "1P", CarrierFrequencyModel::kFixed, 1575.42e6,
     NavMessageFamily::kBeidouBcnav1, CodeBiasModel::kBeidouB1C, 7,
     qmboc_correlation(1.023e6, 1.023e6, 6.138e6, 4.0 / 33.0)},
    {GnssConstellation::kBeidou, SignalId::kBeidouB2A, "BeiDou B2a", "5P", CarrierFrequencyModel::kFixed, 1176.45e6,
     NavMessageFamily::kBeidouBcnav2, CodeBiasModel::kBeidouB2A, 9, bpsk_correlation(10.23e6)},
    {GnssConstellation::kBeidou, SignalId::kBeidouB2B, "BeiDou B2b", "7D", CarrierFrequencyModel::kFixed, 1207.14e6,
     NavMessageFamily::kBeidouBcnav3, CodeBiasModel::kBeidouB2B, 11, bpsk_correlation(10.23e6)},
};

constexpr std::size_t kSignalDefinitionCount = sizeof(kSignalDefinitions) / sizeof(kSignalDefinitions[0]);

bool is_valid_glonass_fcn(int glonass_fcn) {
    return glonass_fcn >= kGlonassMinFcn && glonass_fcn <= kGlonassMaxFcn;
}

} // namespace

const SignalDefinition* signal_definitions(std::size_t* count) {
    if (count != nullptr) {
        *count = kSignalDefinitionCount;
    }
    return kSignalDefinitions;
}

const SignalDefinition* find_signal_definition(SignalId signal_id) {
    for (const SignalDefinition& definition : kSignalDefinitions) {
        if (definition.signal_id == signal_id) {
            return &definition;
        }
    }
    return nullptr;
}

const SignalDefinition* find_signal_definition_by_rinex(GnssConstellation constellation,
                                                        const char* rinex_signal_code) {
    if (rinex_signal_code == nullptr) {
        return nullptr;
    }
    for (const SignalDefinition& definition : kSignalDefinitions) {
        if (definition.constellation == constellation &&
            std::strcmp(definition.rinex_signal_code, rinex_signal_code) == 0) {
            return &definition;
        }
    }
    return nullptr;
}

const SignalDefinition* find_signal_definition_by_oem7(GnssConstellation constellation, int novatel_oem7_signal_type) {
    for (const SignalDefinition& definition : kSignalDefinitions) {
        if (definition.constellation == constellation &&
            definition.novatel_oem7_signal_type == novatel_oem7_signal_type) {
            return &definition;
        }
    }
    return nullptr;
}

bool validate_code_correlation_profile(const CodeCorrelationProfile& profile, std::string* error_message) {
    if (!finite_nonnegative(profile.chip_rate_hz) || !finite_nonnegative(profile.primary_subcarrier_rate_hz) ||
        !finite_nonnegative(profile.secondary_subcarrier_rate_hz) ||
        !finite_nonnegative(profile.secondary_power_fraction)) {
        set_error(error_message, "signal code-correlation profile contains a non-finite or negative numeric value");
        return false;
    }

    switch (profile.model) {
        case CodeCorrelationModel::kUnsupported:
            if (profile.chip_rate_hz != 0.0 || profile.primary_subcarrier_rate_hz != 0.0 ||
                profile.secondary_subcarrier_rate_hz != 0.0 || profile.secondary_power_fraction != 0.0 ||
                profile.secondary_phase != CompositeSubcarrierPhase::kNotApplicable) {
                set_error(error_message, "unsupported correlation profile must not carry guessed waveform parameters");
                return false;
            }
            return true;

        case CodeCorrelationModel::kBpsk:
            if (profile.chip_rate_hz <= 0.0 || profile.primary_subcarrier_rate_hz != 0.0 ||
                profile.secondary_subcarrier_rate_hz != 0.0 || profile.secondary_power_fraction != 0.0 ||
                profile.secondary_phase != CompositeSubcarrierPhase::kNotApplicable) {
                set_error(error_message, "BPSK correlation profile parameters are inconsistent");
                return false;
            }
            return true;

        case CodeCorrelationModel::kTmboc:
            if (!composite_rates_valid(profile) ||
                profile.secondary_phase != CompositeSubcarrierPhase::kNotApplicable) {
                set_error(error_message, "TMBOC correlation profile parameters are inconsistent");
                return false;
            }
            return true;

        case CodeCorrelationModel::kCboc:
            if (!composite_rates_valid(profile) || (profile.secondary_phase != CompositeSubcarrierPhase::kInPhase &&
                                                    profile.secondary_phase != CompositeSubcarrierPhase::kAntiPhase)) {
                set_error(error_message, "CBOC correlation profile parameters are inconsistent");
                return false;
            }
            return true;

        case CodeCorrelationModel::kQmboc:
            if (!composite_rates_valid(profile) || profile.secondary_phase != CompositeSubcarrierPhase::kQuadrature) {
                set_error(error_message, "QMBOC correlation profile parameters are inconsistent");
                return false;
            }
            return true;
    }

    set_error(error_message, "unknown signal code-correlation profile model");
    return false;
}

bool signal_has_supported_code_correlation(const SignalDefinition& definition) {
    return definition.code_correlation.model != CodeCorrelationModel::kUnsupported &&
           validate_code_correlation_profile(definition.code_correlation, nullptr);
}

bool signal_carrier_frequency_hz(const SignalDefinition& definition, int glonass_fcn, double* frequency_hz) {
    if (frequency_hz == nullptr) {
        return false;
    }

    switch (definition.carrier_model) {
        case CarrierFrequencyModel::kFixed:
            *frequency_hz = definition.nominal_frequency_hz;
            return true;
        case CarrierFrequencyModel::kGlonassFdmaG1:
            if (!is_valid_glonass_fcn(glonass_fcn)) {
                return false;
            }
            *frequency_hz = kGlonassG1BaseHz + static_cast<double>(glonass_fcn) * kGlonassG1StepHz;
            return true;
        case CarrierFrequencyModel::kGlonassFdmaG2:
            if (!is_valid_glonass_fcn(glonass_fcn)) {
                return false;
            }
            *frequency_hz = kGlonassG2BaseHz + static_cast<double>(glonass_fcn) * kGlonassG2StepHz;
            return true;
    }
    return false;
}

bool signal_wavelength_m(const SignalDefinition& definition, int glonass_fcn, double* wavelength_m) {
    if (wavelength_m == nullptr) {
        return false;
    }
    double frequency_hz = 0.0;
    if (!signal_carrier_frequency_hz(definition, glonass_fcn, &frequency_hz) || !std::isfinite(frequency_hz) ||
        frequency_hz <= 0.0) {
        return false;
    }
    *wavelength_m = kSpeedOfLightMps / frequency_hz;
    return true;
}

bool signal_rtklib_observation_code(const SignalDefinition& definition, int* observation_code, int* frequency_index) {
    return rtklib_observation_code(definition.rinex_signal_code, observation_code, frequency_index);
}

int signal_single_point_priority(SignalId signal_id) {
    switch (signal_id) {
        case SignalId::kGpsL1Ca:
        case SignalId::kQzssL1Ca:
        case SignalId::kGlonassG1:
        case SignalId::kGalileoE1:
        case SignalId::kBeidouB1I:
            return 0;
        case SignalId::kGpsL1C:
        case SignalId::kQzssL1C:
            return 1;
        default:
            return -1;
    }
}

} // namespace gnss_sim
