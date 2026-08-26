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

// OEM7 signal types match the RANGE RINEX mapping table in the pinned RTKLIB
// fork. RINEX codes resolve through RTKLIB, including its modern-BDS extension.
constexpr SignalDefinition kSignalDefinitions[] = {
    {GnssConstellation::kGps, SignalId::kGpsL1Ca, "GPS L1 C/A", "1C", CarrierFrequencyModel::kFixed, 1575.42e6,
     NavMessageFamily::kGpsLnav, CodeBiasModel::kGpsL1Ca, 0},
    {GnssConstellation::kGps, SignalId::kGpsL1C, "GPS L1C", "1L", CarrierFrequencyModel::kFixed, 1575.42e6,
     NavMessageFamily::kGpsCnav2, CodeBiasModel::kGpsL1C, 16},
    {GnssConstellation::kGps, SignalId::kGpsL2P, "GPS L2P", "2P", CarrierFrequencyModel::kFixed, 1227.60e6,
     NavMessageFamily::kGpsLnav, CodeBiasModel::kGpsL2P, 5},
    {GnssConstellation::kGps, SignalId::kGpsL2C, "GPS L2C", "2S", CarrierFrequencyModel::kFixed, 1227.60e6,
     NavMessageFamily::kGpsCnav, CodeBiasModel::kGpsL2C, 17},
    {GnssConstellation::kGps, SignalId::kGpsL5Q, "GPS L5Q", "5Q", CarrierFrequencyModel::kFixed, 1176.45e6,
     NavMessageFamily::kGpsCnav, CodeBiasModel::kGpsL5Q, 14},

    {GnssConstellation::kQzss, SignalId::kQzssL1Ca, "QZSS L1 C/A", "1C", CarrierFrequencyModel::kFixed, 1575.42e6,
     NavMessageFamily::kQzssLnav, CodeBiasModel::kQzssL1Ca, 0},
    {GnssConstellation::kQzss, SignalId::kQzssL1C, "QZSS L1C", "1L", CarrierFrequencyModel::kFixed, 1575.42e6,
     NavMessageFamily::kQzssCnav2, CodeBiasModel::kQzssL1C, 16},
    {GnssConstellation::kQzss, SignalId::kQzssL2C, "QZSS L2C", "2S", CarrierFrequencyModel::kFixed, 1227.60e6,
     NavMessageFamily::kQzssCnav, CodeBiasModel::kQzssL2C, 17},
    {GnssConstellation::kQzss, SignalId::kQzssL5Q, "QZSS L5Q", "5Q", CarrierFrequencyModel::kFixed, 1176.45e6,
     NavMessageFamily::kQzssCnav, CodeBiasModel::kQzssL5Q, 14},

    {GnssConstellation::kGlonass, SignalId::kGlonassG1, "GLONASS G1", "1C", CarrierFrequencyModel::kGlonassFdmaG1,
     kGlonassG1BaseHz, NavMessageFamily::kGlonassFdma, CodeBiasModel::kGlonassG1, 0},
    {GnssConstellation::kGlonass, SignalId::kGlonassG2, "GLONASS G2", "2C", CarrierFrequencyModel::kGlonassFdmaG2,
     kGlonassG2BaseHz, NavMessageFamily::kGlonassFdma, CodeBiasModel::kGlonassG2, 1},
    {GnssConstellation::kGlonass, SignalId::kGlonassG3, "GLONASS G3", "3Q", CarrierFrequencyModel::kFixed, 1202.025e6,
     NavMessageFamily::kGlonassL3Oc, CodeBiasModel::kGlonassG3, 6},

    {GnssConstellation::kGalileo, SignalId::kGalileoE1, "Galileo E1", "1C", CarrierFrequencyModel::kFixed, 1575.42e6,
     NavMessageFamily::kGalileoInav, CodeBiasModel::kGalileoE1, 2},
    {GnssConstellation::kGalileo, SignalId::kGalileoE5A, "Galileo E5a", "5Q", CarrierFrequencyModel::kFixed, 1176.45e6,
     NavMessageFamily::kGalileoFnav, CodeBiasModel::kGalileoE5A, 12},
    {GnssConstellation::kGalileo, SignalId::kGalileoE5B, "Galileo E5b", "7Q", CarrierFrequencyModel::kFixed, 1207.14e6,
     NavMessageFamily::kGalileoInav, CodeBiasModel::kGalileoE5B, 17},
    {GnssConstellation::kGalileo, SignalId::kGalileoE6, "Galileo E6", "6B", CarrierFrequencyModel::kFixed, 1278.75e6,
     NavMessageFamily::kGalileoCnav, CodeBiasModel::kGalileoE6, 6},

    {GnssConstellation::kBeidou, SignalId::kBeidouB1I, "BeiDou B1I", "2I", CarrierFrequencyModel::kFixed, 1561.098e6,
     NavMessageFamily::kBeidouD1D2, CodeBiasModel::kBeidouB1I, 0},
    {GnssConstellation::kBeidou, SignalId::kBeidouB3I, "BeiDou B3I", "6I", CarrierFrequencyModel::kFixed, 1268.52e6,
     NavMessageFamily::kBeidouD1D2, CodeBiasModel::kBeidouB3I, 2},
    {GnssConstellation::kBeidou, SignalId::kBeidouB1C, "BeiDou B1C", "1P", CarrierFrequencyModel::kFixed, 1575.42e6,
     NavMessageFamily::kBeidouBcnav1, CodeBiasModel::kBeidouB1C, 7},
    {GnssConstellation::kBeidou, SignalId::kBeidouB2A, "BeiDou B2a", "5P", CarrierFrequencyModel::kFixed, 1176.45e6,
     NavMessageFamily::kBeidouBcnav2, CodeBiasModel::kBeidouB2A, 9},
    {GnssConstellation::kBeidou, SignalId::kBeidouB2B, "BeiDou B2b", "7D", CarrierFrequencyModel::kFixed, 1207.14e6,
     NavMessageFamily::kBeidouBcnav3, CodeBiasModel::kBeidouB2B, 11},
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

} // namespace gnss_sim
