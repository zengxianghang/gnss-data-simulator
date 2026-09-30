#include "output/unicore_nav_writer.h"

#include "output/unicore_ascii.h"

#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

namespace gnss_sim {
namespace {

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

const char* bool_text(bool value) {
    return value ? "TRUE" : "FALSE";
}

bool modern_bds(const KeplerianNavOutputData& eph) {
    return eph.message_family == RtklibBroadcastMessageFamily::kBeidouBcnav1 ||
           eph.message_family == RtklibBroadcastMessageFamily::kBeidouBcnav2 ||
           eph.message_family == RtklibBroadcastMessageFamily::kBeidouBcnav3;
}

// BDS-SIS-ICD-B1C/B2a/B2b reference semi-major axis A_ref (m): B-CNAV
// broadcasts DeltaA = A - A_ref.
constexpr double kBdsMeoReferenceSemiMajorAxisM = 27906100.0;
constexpr double kBdsIgsoGeoReferenceSemiMajorAxisM = 42162200.0;
// B-CNAV t_op scale factor (s).
constexpr double kBdsTopScaleSec = 300.0;

int bds_frequency_type(const KeplerianNavOutputData& eph) {
    if (eph.message_family == RtklibBroadcastMessageFamily::kBeidouBcnav2) {
        return 1;
    }
    if (eph.message_family == RtklibBroadcastMessageFamily::kBeidouBcnav3) {
        return 2;
    }
    return 0;
}

std::string legacy_kepler_body(const KeplerianNavOutputData& eph, bool beidou) {
    std::ostringstream body;
    body.imbue(std::locale::classic());
    body << eph.prn << ',' << std::fixed << std::setprecision(1) << eph.transmit_sow_sec << ',' << eph.svh << ','
         << eph.iode << ',' << eph.iode << ',' << eph.toe_week << ',' << eph.toe_week << ',' << eph.toe_sow_sec << ','
         << std::scientific << std::setprecision(15) << eph.semi_major_axis_m << ',' << eph.delta_mean_motion_radps
         << ',' << eph.mean_anomaly_rad << ',' << eph.eccentricity << ',' << eph.argument_of_perigee_rad << ','
         << eph.cuc_rad << ',' << eph.cus_rad << ',' << eph.crc_m << ',' << eph.crs_m << ',' << eph.cic_rad << ','
         << eph.cis_rad << ',' << eph.inclination_rad << ',' << eph.inclination_dot_radps << ',' << eph.omega0_rad
         << ',' << eph.omega_dot_radps << ',' << eph.iodc << ',' << std::fixed << std::setprecision(1)
         << eph.toc_sow_sec << ',' << std::scientific << std::setprecision(15) << eph.tgd_sec[0];
    if (beidou) {
        body << ',' << eph.tgd_sec[1];
    }
    body << ',' << eph.clock_bias_sec << ',' << eph.clock_drift_sec_per_sec << ',' << eph.clock_drift_rate_sec_per_sec2
         << ',' << eph.corrected_mean_motion_radps << ',' << eph.sva;
    return body.str();
}

// An integer-valued field as its unsigned raw value of `bits` bits.  RINEX
// producers may write a signed reading of the same bits (SISAI_ocb 27 as -5).
bool unsigned_raw_field(double value, int bits, int* raw) {
    const double rounded = std::nearbyint(value);
    const int limit = 1 << bits;
    if (!std::isfinite(value) || rounded != value || rounded < -limit / 2 || rounded >= limit) {
        return false;
    }
    *raw = (static_cast<int>(rounded) + limit) % limit;
    return true;
}

// Unicore N4 BD3EPH (reference book 7.3.12), one B-CNAV1/B-CNAV2/B-CNAV3
// ephemeris: PRN, Health, SatType, SISMAI, IODE, IODC, Week, Zweek, Tow, Toe,
// DeltaA, dDeltaA, DeltaN, dDeltaN, M0, Ecc, omega, Cuc, Cus, Crc, Crs, Cic,
// Cis, I0, IDOT, Omega0, OmegaDot, toc, Tgdb1cp, Tgdb2ap, Tgdb2bI, Tgdb2bQ,
// ISCb2ad, ISCb1cd, af0, af1, af2, iTop, SISAIoe, SISAIocb, SISAIoc1,
// SISAIoc2, two reserved fields and FreqType.  Week is the GPS week of Toe;
// Tow, Toe and toc are native BDT seconds of week and iTop the raw t_op
// (300 s units), as in the N4 example.  Group delays a message family does
// not broadcast, and IODE/IODC of B-CNAV3 (reserved in N4), are zero.
// Returns false when the record cannot be represented (unknown SatType or a
// non-integral protocol field).
bool bd3_ephemeris_body(const KeplerianNavOutputData& eph, std::string* body_text) {
    const int sat_type = eph.flag; // RINEX 4 SatType: 1 GEO, 2 IGSO, 3 MEO
    if (sat_type < 1 || sat_type > 3) {
        return false;
    }
    const double reference_axis_m = sat_type == 3 ? kBdsMeoReferenceSemiMajorAxisM : kBdsIgsoGeoReferenceSemiMajorAxisM;
    const double top_units = eph.top_sow_sec / kBdsTopScaleSec;
    int sismai = 0;
    int sisai[4] = {0, 0, 0, 0};
    static const int kSisaiBits[4] = {5, 5, 3, 3}; // oe, ocb, oc1, oc2
    bool valid = unsigned_raw_field(eph.sva, 4, &sismai) && std::nearbyint(top_units) == top_units;
    for (int index = 0; valid && index < 4; ++index) {
        valid = unsigned_raw_field(eph.sisai[index], kSisaiBits[index], &sisai[index]);
    }
    if (!valid) {
        return false;
    }
    const int frequency_type = bds_frequency_type(eph);
    const bool b2b = frequency_type == 2;
    double tgd_b1cp = 0.0;
    double tgd_b2ap = 0.0;
    double tgd_b2bi = 0.0;
    double isc_b2ad = 0.0;
    double isc_b1cd = 0.0;
    if (b2b) {
        tgd_b2bi = eph.tgd_sec[0];
    } else {
        tgd_b1cp = eph.tgd_sec[0];
        tgd_b2ap = eph.tgd_sec[1];
        if (frequency_type == 0) {
            isc_b1cd = eph.isc_sec[0]; // B-CNAV1 ISC_B1Cd
        } else {
            isc_b2ad = eph.isc_sec[0]; // B-CNAV2 ISC_B2ad
        }
    }

    std::ostringstream body;
    body.imbue(std::locale::classic());
    body << eph.prn << ',' << eph.svh << ',' << sat_type << ',' << sismai << ',' << (b2b ? 0 : eph.iode) << ','
         << (b2b ? 0 : eph.iodc) << ',' << eph.toe_week << ',' << eph.toe_week << ',' << std::fixed
         << std::setprecision(1) << eph.bdt_transmit_sow_sec << ',' << eph.bdt_toe_sow_sec << ',' << std::scientific
         << std::setprecision(15) << eph.semi_major_axis_m - reference_axis_m << ',' << eph.semi_major_axis_rate_mps
         << ',' << eph.delta_mean_motion_radps << ',' << eph.delta_mean_motion_rate_radps2 << ','
         << eph.mean_anomaly_rad << ',' << eph.eccentricity << ',' << eph.argument_of_perigee_rad << ',' << eph.cuc_rad
         << ',' << eph.cus_rad << ',' << eph.crc_m << ',' << eph.crs_m << ',' << eph.cic_rad << ',' << eph.cis_rad
         << ',' << eph.inclination_rad << ',' << eph.inclination_dot_radps << ',' << eph.omega0_rad << ','
         << eph.omega_dot_radps << ',' << std::fixed << std::setprecision(1) << eph.bdt_toc_sow_sec << ','
         << std::scientific << std::setprecision(15) << tgd_b1cp << ',' << tgd_b2ap << ',' << tgd_b2bi << ',' << 0.0
         << ',' << isc_b2ad << ',' << isc_b1cd << ',' << eph.clock_bias_sec << ',' << eph.clock_drift_sec_per_sec << ','
         << eph.clock_drift_rate_sec_per_sec2 << ',' << static_cast<int>(std::llround(top_units)) << ',' << sisai[0]
         << ',' << sisai[1] << ',' << sisai[2] << ',' << sisai[3] << ",0,0," << frequency_type;
    *body_text = body.str();
    return true;
}

std::string galileo_body(const KeplerianNavOutputData& eph) {
    std::ostringstream body;
    body.imbue(std::locale::classic());
    body << eph.prn << ',' << bool_text(eph.galileo_fnav_received) << ',' << bool_text(eph.galileo_inav_received) << ','
         << eph.galileo_e1b_health << ',' << eph.galileo_e5a_health << ',' << eph.galileo_e5b_health << ','
         << eph.galileo_e1b_dvs << ',' << eph.galileo_e5a_dvs << ',' << eph.galileo_e5b_dvs << ','
         << static_cast<int>(std::llround(eph.sva)) << ',' << eph.svh << ',' << eph.iode << ',' << std::fixed
         << std::setprecision(0) << eph.toe_sow_sec << ',' << std::scientific << std::setprecision(15)
         << eph.sqrt_semi_major_axis_sqrt_m << ',' << eph.delta_mean_motion_radps << ',' << eph.mean_anomaly_rad << ','
         << eph.eccentricity << ',' << eph.argument_of_perigee_rad << ',' << eph.cuc_rad << ',' << eph.cus_rad << ','
         << eph.crc_m << ',' << eph.crs_m << ',' << eph.cic_rad << ',' << eph.cis_rad << ',' << eph.inclination_rad
         << ',' << eph.inclination_dot_radps << ',' << eph.omega0_rad << ',' << eph.omega_dot_radps << ',' << std::fixed
         << std::setprecision(0) << eph.galileo_fnav_toc_sow_sec << ',' << std::scientific << std::setprecision(15)
         << eph.galileo_fnav_clock[0] << ',' << eph.galileo_fnav_clock[1] << ',' << eph.galileo_fnav_clock[2] << ','
         << std::fixed << std::setprecision(0) << eph.galileo_inav_toc_sow_sec << ',' << std::scientific
         << std::setprecision(15) << eph.galileo_inav_clock[0] << ',' << eph.galileo_inav_clock[1] << ','
         << eph.galileo_inav_clock[2] << ',' << eph.tgd_sec[0] << ',' << eph.tgd_sec[1];
    return body.str();
}

std::string glonass_body(const GlonassNavOutputData& glo) {
    std::ostringstream body;
    body.imbue(std::locale::classic());
    const std::int64_t toe_ms = static_cast<std::int64_t>(std::llround(glo.toe_sow_sec * 1000.0));
    body << glo.slot_offset << ',' << glo.frequency_offset << ",1,0," << glo.toe_week << ',' << toe_ms << ','
         << glo.gps_glonass_time_offset_sec << ',' << glo.calendar_day_number << ",0,0," << glo.iode << ',' << glo.svh
         << ',' << std::scientific << std::setprecision(15) << glo.position_ecef_m[0] << ',' << glo.position_ecef_m[1]
         << ',' << glo.position_ecef_m[2] << ',' << glo.velocity_ecef_mps[0] << ',' << glo.velocity_ecef_mps[1] << ','
         << glo.velocity_ecef_mps[2] << ',' << glo.acceleration_ecef_mps2[0] << ',' << glo.acceleration_ecef_mps2[1]
         << ',' << glo.acceleration_ecef_mps2[2] << ',' << glo.clock_bias_sec << ',' << glo.relative_frequency_bias
         << ',' << glo.differential_delay_sec << ',' << std::fixed << std::setprecision(0)
         << glo.frame_time_glonass_day_sec << ',' << glo.flags << ',' << glo.sva << ',' << glo.age_days << ','
         << glo.flags;
    return body.str();
}

std::string legacy_ion_body(const IonosphereNavOutputData& ion) {
    std::ostringstream body;
    body.imbue(std::locale::classic());
    body << std::scientific << std::setprecision(15);
    for (int index = 0; index < 8; ++index) {
        if (index != 0) {
            body << ',';
        }
        body << ion.coefficients[index];
    }
    const std::int64_t transmit_ms = static_cast<std::int64_t>(std::llround(ion.transmit_sow_sec * 1000.0));
    body << ',' << ion.prn << ',' << std::dec << ion.transmit_week << ',' << transmit_ms << ",0";
    return body.str();
}

std::string bd3_ion_body(const IonosphereNavOutputData& ion) {
    std::ostringstream body;
    body.imbue(std::locale::classic());
    body << std::scientific << std::setprecision(15);
    for (int index = 0; index < 9; ++index) {
        if (index != 0) {
            body << ',';
        }
        body << ion.coefficients[index];
    }
    body << ',' << std::fixed << std::setprecision(0) << ion.region;
    return body.str();
}

std::string galileo_ion_body(const IonosphereNavOutputData& ion) {
    std::ostringstream body;
    body.imbue(std::locale::classic());
    body << std::scientific << std::setprecision(15) << ion.coefficients[0] << ',' << ion.coefficients[1] << ','
         << ion.coefficients[2] << ",0,0,0,0,0,0";
    return body.str();
}

} // namespace

bool format_unicore_nav_output_record(const NavOutputRecord& source, const SimTime& output_time, std::string* message,
                                      bool* supported, std::string* error_message) {
    if (message == nullptr || supported == nullptr) {
        set_error(error_message, "Unicore NAV writer request has invalid arguments");
        return false;
    }
    *supported = false;
    message->clear();
    NavOutputRecord record = source;
    if (!finalize_nav_output_record_metadata(&record)) {
        set_error(error_message, "cannot finalize Unicore NAV output metadata");
        return false;
    }

    const char* log_name = nullptr;
    std::string body;
    if (record.kind == RtklibNavRecordKind::kGlonassEphemeris) {
        if (record.glonass.message_family != RtklibBroadcastMessageFamily::kGlonassFdma) {
            return true;
        }
        log_name = "GLOEPHA";
        body = glonass_body(record.glonass);
    } else if (record.kind == RtklibNavRecordKind::kEphemeris) {
        const KeplerianNavOutputData& eph = record.ephemeris;
        switch (eph.system) {
            case NavOutputSystem::kGps:
                log_name = "GPSEPHA";
                body = legacy_kepler_body(eph, false);
                break;
            case NavOutputSystem::kQzss:
                log_name = "QZSSEPHA";
                body = legacy_kepler_body(eph, false);
                break;
            case NavOutputSystem::kGalileo:
                log_name = "GALEPHA";
                body = galileo_body(eph);
                break;
            case NavOutputSystem::kBeidou:
                if (modern_bds(eph)) {
                    if (!bd3_ephemeris_body(eph, &body)) {
                        return true;
                    }
                    log_name = "BD3EPHA";
                } else {
                    log_name = "BDSEPHA";
                    body = legacy_kepler_body(eph, true);
                }
                break;
            case NavOutputSystem::kNavic:
                log_name = "IRNSSEPHA";
                body = legacy_kepler_body(eph, false);
                break;
            default:
                break;
        }
    } else if (record.kind == RtklibNavRecordKind::kIonosphere) {
        const IonosphereNavOutputData& ion = record.ionosphere;
        if (ion.system == NavOutputSystem::kGps && ion.coefficient_count >= 8) {
            log_name = "GPSIONA";
            body = legacy_ion_body(ion);
        } else if (ion.system == NavOutputSystem::kBeidou && ion.coefficient_count >= 9 && !ion.legacy_metadata) {
            log_name = "BD3IONA";
            body = bd3_ion_body(ion);
        } else if (ion.system == NavOutputSystem::kBeidou && ion.coefficient_count >= 8) {
            log_name = "BDSIONA";
            body = legacy_ion_body(ion);
        } else if (ion.system == NavOutputSystem::kGalileo && ion.coefficient_count >= 3) {
            log_name = "GALIONA";
            body = galileo_ion_body(ion);
        }
    }

    if (log_name == nullptr) {
        return true;
    }
    if (!unicore_ascii::frame(log_name, output_time, body, message)) {
        set_error(error_message, "Unicore NAV header time cannot be represented");
        return false;
    }
    *supported = true;
    return true;
}

bool format_unicore_receiver_nav_record(const RtklibNavStore* receiver_nav, int output_record_index,
                                        const SimTime& output_time, std::string* message, bool* supported,
                                        std::string* error_message) {
    NavOutputRecord record{};
    if (!rtklib_nav_output_record(receiver_nav, output_record_index, &record, error_message)) {
        return false;
    }
    return format_unicore_nav_output_record(record, output_time, message, supported, error_message);
}

} // namespace gnss_sim
