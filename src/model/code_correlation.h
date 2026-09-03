#ifndef GNSS_SIM_SRC_MODEL_CODE_CORRELATION_H_
#define GNSS_SIM_SRC_MODEL_CODE_CORRELATION_H_

#include "gnss/signal_definitions.h"

#include <complex>
#include <string>

namespace gnss_sim {

// Evaluate the normalized ideal same-code autocorrelation at a code-phase
// offset expressed in code chips. The V1 ideal-code model treats different PRN
// chips as uncorrelated, so the response is exactly zero for |delay| >= 1 chip.
//
// Composite profiles are evaluated from the tracked component metadata in the
// central SignalDefinition table; unsupported profiles fail explicitly.
bool ideal_code_correlation_chips(const CodeCorrelationProfile& profile, double delay_chips,
                                  std::complex<double>* correlation, std::string* error_message);

// SignalDefinition wrapper using delay in seconds. This is the narrow interface
// later DLL/path code should use when propagation supplies physical delays.
bool ideal_signal_code_correlation(const SignalDefinition& definition, double delay_seconds,
                                   std::complex<double>* correlation, std::string* error_message);

bool signal_code_chip_duration_s(const SignalDefinition& definition, double* chip_duration_s,
                                 std::string* error_message);
bool signal_code_chip_length_m(const SignalDefinition& definition, double* chip_length_m,
                               std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_CODE_CORRELATION_H_
