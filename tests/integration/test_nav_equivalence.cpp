#include "gnss/nav_output_record.h"
#include "gnss/rtklib_adapter.h"
#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"
#include "output/novatel_nav_writer.h"
#include "rangea_roundtrip.h"
#include "serialized_nav_parser.h"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

// Issue 97: numerical equivalence validation of the navigation conversion path.
//
// reference path : real RINEX NAV -> RTKLIB nav_t (production loader)
// round-trip path: the same real RINEX NAV -> production EPH/ION serialization
//                  -> production serialized-NAV parser -> production RTKLIB input
//                  adapter -> reconstructed nav_t
//
// The round-trip store is built exclusively from parsed receiver-log messages. No
// eph_t/geph_t/ionosphere value is copied, retargeted, synthesized, or interpolated,
// and the reference store is never consulted while building or using the round-trip
// store.

namespace {

constexpr int kReferenceGpsWeek = 2347;
constexpr double kReferenceSowSec = 436500.0;
constexpr double kElevationMaskDeg = 5.0;
// Per-family compatible observation codes for the family-restricted state selection
// (RTKLIB eph_supports_code): L1C for GPS/QZSS/GLONASS/Galileo, B1I (L2I) for BeiDou
// legacy records.
constexpr int kL1CObservationCode = 1;
constexpr int kBeidouB1IObservationCode = 40;

// Frozen acceptance limits (see the issue 97 report for the justification). The
// serialized NovAtel NAV fields carry 15 fractional scientific digits, so the
// dominant loss is IEEE-754 double round-tripping at roughly 1e-15 relative.
// Measured maxima over the full real fixture (see the NAV_EQUIV_* summary lines):
// position differs by at most 1.3e-8 m (1 ulp of the ~2e7 m orbit-radius double),
// velocity by at most 1.3e-5 m/s (RTKLIB computes velocity by differentiating the
// propagated position, amplifying that 1-ulp round-off over its finite-difference
// step), and clock bias/drift round-trip exactly. The frozen limits are ~75x the
// measured maxima and remain 7+ orders below the state magnitudes themselves.
constexpr double kMaxSatellitePositionDiffM = 1.0e-6;
constexpr double kMaxSatelliteVelocityDiffMps = 1.0e-4;
constexpr double kMaxSatelliteClockBiasDiffS = 1.0e-12;
constexpr double kMaxSatelliteClockDriftDiffSps = 1.0e-14;
constexpr double kMaxCorrectionRelativeDiff = 1.0e-12;
constexpr double kMaxPositioningPositionDiffM = 1.0e-4;
constexpr double kMaxPositioningClockDiffM = 1.0e-5;

std::string nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

gnss_sim::SimTime output_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(kReferenceGpsWeek, kReferenceSowSec, &time));
    return time;
}

gnss_sim::SimConfig simulator_config() {
    gnss_sim::SimConfig value = gnss_sim::default_sim_config();
    value.scenario = gnss_sim::ScenarioType::KS;
    value.atmosphere_mode = gnss_sim::AtmosphereMode::BROADCAST;
    value.sampling_rate_hz = 1;
    value.duration_ns = 60LL * gnss_sim::NANOSECONDS_PER_SECOND;
    value.elevation_mask_deg = 0.0;
    value.solution_elevation_mask_deg = kElevationMaskDeg;
    value.receiver = {20.0, 120.0, 100.0};
    value.measurement_noise_enabled = false;
    value.multipath_enabled = false;
    value.output_eph = true;
    value.output_ion = true;
    value.seed = 0x60U;
    return value;
}

const char* family_name(gnss_sim::RtklibBroadcastMessageFamily family) {
    switch (family) {
        case gnss_sim::RtklibBroadcastMessageFamily::kLegacy:
            return "LNAV";
        case gnss_sim::RtklibBroadcastMessageFamily::kCnav:
            return "CNAV";
        case gnss_sim::RtklibBroadcastMessageFamily::kCnav2:
            return "CNAV2";
        case gnss_sim::RtklibBroadcastMessageFamily::kGlonassFdma:
            return "FDMA";
        case gnss_sim::RtklibBroadcastMessageFamily::kGlonassL3Oc:
            return "L3OC";
        case gnss_sim::RtklibBroadcastMessageFamily::kGalileoInav:
            return "INAV";
        case gnss_sim::RtklibBroadcastMessageFamily::kGalileoFnav:
            return "FNAV";
        case gnss_sim::RtklibBroadcastMessageFamily::kBeidouBcnav1:
            return "BCNAV1";
        case gnss_sim::RtklibBroadcastMessageFamily::kBeidouBcnav2:
            return "BCNAV2";
        case gnss_sim::RtklibBroadcastMessageFamily::kBeidouBcnav3:
            return "BCNAV3";
        case gnss_sim::RtklibBroadcastMessageFamily::kUnknown:
            return "UNKNOWN";
    }
    return "UNKNOWN";
}

double relative_diff(double left, double right) {
    const double scale = std::max(std::fabs(left), std::fabs(right));
    if (scale <= 0.0) {
        return std::fabs(left - right);
    }
    return std::fabs(left - right) / scale;
}

bool is_keplerian_ephemeris(const gnss_sim::NavOutputRecord& record) {
    return record.kind == gnss_sim::RtklibNavRecordKind::kEphemeris;
}

bool is_ephemeris_record(const gnss_sim::NavOutputRecord& record) {
    return is_keplerian_ephemeris(record) || record.kind == gnss_sim::RtklibNavRecordKind::kGlonassEphemeris;
}

std::string record_identity_key(const gnss_sim::NavOutputRecord& record) {
    const bool keplerian = is_keplerian_ephemeris(record);
    std::ostringstream key;
    key << (keplerian ? gnss_sim::nav_output_system_name(record.ephemeris.system) : "GLO") << ':'
        << (keplerian ? record.ephemeris.prn : record.glonass.prn) << ':'
        << static_cast<int>(keplerian ? record.ephemeris.message_family : record.glonass.message_family) << ':'
        << (keplerian ? record.ephemeris.iode : record.glonass.iode) << ':'
        << (keplerian ? record.ephemeris.toe_week : record.glonass.toe_week) << ':'
        << (keplerian ? record.ephemeris.toe_sow_sec : record.glonass.toe_sow_sec);
    return key.str();
}

// Builds the round-trip navigation store exclusively through the production chain:
// projected record -> NovAtel NAV writer -> independent serialized-NAV parser ->
// production RTKLIB input adapter. Records the writer rejects (frozen receiver
// output contract, e.g. modern GPS/QZSS CNAV families) are skipped and counted.
bool build_roundtrip_store(const gnss_sim::RtklibNavStore* reference, gnss_sim::RtklibNavStore** roundtrip,
                           std::uint64_t* serialized_count, std::uint64_t* serialized_ephemeris_count,
                           std::uint64_t* rejected_count, std::string* error_message) {
    *roundtrip = gnss_sim::create_rtklib_nav_store();
    if (*roundtrip == nullptr) {
        *error_message = "cannot allocate round-trip navigation store";
        return false;
    }
    std::uint64_t serialized = 0;
    std::uint64_t serialized_ephemeris = 0;
    std::uint64_t rejected = 0;
    const int total = gnss_sim::rtklib_nav_output_record_count(reference);
    for (int index = 0; index < total; ++index) {
        gnss_sim::NavOutputRecord record{};
        if (!gnss_sim::rtklib_nav_output_record(reference, index, &record, error_message)) {
            return false;
        }
        std::string message;
        bool supported = false;
        if (!gnss_sim::format_novatel_nav_output_record(record, output_time(), &message, &supported, error_message)) {
            return false;
        }
        if (!supported) {
            ++rejected;
            continue;
        }
        gnss_sim::ParsedSerializedNavRecord parsed{};
        bool recognized = false;
        if (!gnss_sim::parse_serialized_novatel_nav_line_independent(message, &parsed, &recognized, error_message)) {
            return false;
        }
        if (!recognized) {
            *error_message = "serialized NAV message was not recognized by the production parser";
            return false;
        }
        if (!gnss_sim::rtklib_append_nav_output_record(*roundtrip, parsed.record, error_message)) {
            return false;
        }
        ++serialized;
        if (record.kind != gnss_sim::RtklibNavRecordKind::kIonosphere) {
            ++serialized_ephemeris;
        }
    }
    *serialized_count = serialized;
    *serialized_ephemeris_count = serialized_ephemeris;
    *rejected_count = rejected;
    return true;
}

struct StateDifference {
    double position_per_axis_m[3];
    double position_3d_m;
    double velocity_per_axis_mps[3];
    double velocity_3d_mps;
    double clock_bias_diff_s;
    double clock_drift_diff_sps;
};

bool compute_state_difference(const gnss_sim::RtklibSatelliteState& reference,
                              const gnss_sim::RtklibSatelliteState& roundtrip, StateDifference* difference) {
    double velocity_3d = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        difference->position_per_axis_m[axis] = reference.position_ecef_m[axis] - roundtrip.position_ecef_m[axis];
        difference->velocity_per_axis_mps[axis] = reference.velocity_ecef_mps[axis] - roundtrip.velocity_ecef_mps[axis];
        velocity_3d += difference->velocity_per_axis_mps[axis] * difference->velocity_per_axis_mps[axis];
    }
    difference->position_3d_m = std::sqrt(difference->position_per_axis_m[0] * difference->position_per_axis_m[0] +
                                          difference->position_per_axis_m[1] * difference->position_per_axis_m[1] +
                                          difference->position_per_axis_m[2] * difference->position_per_axis_m[2]);
    difference->velocity_3d_mps = std::sqrt(velocity_3d);
    difference->clock_bias_diff_s = reference.clock_bias_sec - roundtrip.clock_bias_sec;
    difference->clock_drift_diff_sps = reference.clock_drift_sec_per_sec - roundtrip.clock_drift_sec_per_sec;
    return true;
}

void report_state_csv(int satellite_number, gnss_sim::RtklibBroadcastMessageFamily family, int toe_week, double toe_sow,
                      int test_week, double test_sow, int reference_iode, int roundtrip_iode,
                      const StateDifference& difference) {
    std::cout << "NAV_EQUIV_STATE_CSV," << satellite_number << ',' << family_name(family) << ',' << toe_week << ','
              << toe_sow << ',' << test_week << ',' << test_sow << ',' << reference_iode << ',' << roundtrip_iode << ','
              << difference.position_per_axis_m[0] << ',' << difference.position_per_axis_m[1] << ','
              << difference.position_per_axis_m[2] << ',' << difference.position_3d_m << ','
              << difference.velocity_per_axis_mps[0] << ',' << difference.velocity_per_axis_mps[1] << ','
              << difference.velocity_per_axis_mps[2] << ',' << difference.velocity_3d_mps << ','
              << difference.clock_bias_diff_s << ',' << difference.clock_drift_diff_sps << "\n";
}

int kBeidouOrL1cCode(const gnss_sim::NavOutputRecord& record) {
    return is_keplerian_ephemeris(record) && record.ephemeris.system == gnss_sim::NavOutputSystem::kBeidou
               ? kBeidouB1IObservationCode
               : kL1CObservationCode;
}

struct StateEquivalenceStats {
    std::uint64_t compared_states = 0;
    std::uint64_t validity_skipped_samples = 0;
    std::uint64_t availability_mismatch_samples = 0;
    double max_position_m = 0.0;
    double max_velocity_mps = 0.0;
    double max_clock_bias_s = 0.0;
    double max_clock_drift_sps = 0.0;
};

enum class StateSampleOutcome {
    Compared,
    BothUnavailableValiditySkip,
    AvailabilityMismatch,
};

// Compares the family-restricted satellite state of one real broadcast instance at one
// GPST epoch. When expected_source is provided (the mandatory Toe sample), both stores
// must select exactly that source record before any state value is compared. Both
// stores must agree on availability; state values must agree within the frozen
// tolerances.
StateSampleOutcome compare_state_at_epoch(const gnss_sim::RtklibNavStore* reference,
                                          const gnss_sim::RtklibNavStore* roundtrip, int satellite_number,
                                          gnss_sim::RtklibBroadcastMessageFamily family, int toe_week, double toe_sow,
                                          int test_week, double test_sow, int observation_code,
                                          const int* expected_iode, const double* expected_toe_sow,
                                          StateEquivalenceStats* stats, const char* context) {
    std::string error_message;
    gnss_sim::RtklibSatelliteState reference_state{};
    gnss_sim::RtklibSatelliteState roundtrip_state{};
    gnss_sim::RtklibSelectedEphemerisInfo reference_identity{};
    gnss_sim::RtklibSelectedEphemerisInfo roundtrip_identity{};
    const bool reference_available =
        gnss_sim::get_rtklib_signal_satellite_state(reference, test_week, test_sow, satellite_number, observation_code,
                                                    family, &reference_state, &error_message, &reference_identity);
    const bool roundtrip_available =
        gnss_sim::get_rtklib_signal_satellite_state(roundtrip, test_week, test_sow, satellite_number, observation_code,
                                                    family, &roundtrip_state, &error_message, &roundtrip_identity);
    if (!reference_available && !roundtrip_available) {
        stats->validity_skipped_samples += 1;
        return StateSampleOutcome::BothUnavailableValiditySkip;
    }
    if (reference_available != roundtrip_available) {
        stats->availability_mismatch_samples += 1;
        ADD_FAILURE() << "one-sided satellite-state availability at GPST " << test_week << "/" << test_sow << " ("
                      << context << "): reference=" << reference_available << " roundtrip=" << roundtrip_available;
        return StateSampleOutcome::AvailabilityMismatch;
    }
    // The selected broadcast instance identity must match before any state value is
    // compared; a close state from a different real ephemeris is not acceptable.
    EXPECT_EQ(reference_identity.satellite_number, roundtrip_identity.satellite_number)
        << "selected-identity satellite mismatch at GPST " << test_week << "/" << test_sow;
    EXPECT_EQ(reference_identity.message_family, roundtrip_identity.message_family)
        << "selected-identity family mismatch at GPST " << test_week << "/" << test_sow;
    EXPECT_EQ(reference_identity.iode, roundtrip_identity.iode)
        << "selected-identity IODE mismatch at GPST " << test_week << "/" << test_sow;
    // Galileo has no broadcast IODC; its ASCII contract carries IODC=IODE while the
    // pinned RTKLIB decode leaves eph.iodc at zero, so IODC is asserted only where it
    // is a real broadcast field (GPS/QZSS legacy, BeiDou legacy).
    if (family == gnss_sim::RtklibBroadcastMessageFamily::kLegacy) {
        EXPECT_EQ(reference_identity.iodc, roundtrip_identity.iodc)
            << "selected-identity IODC mismatch at GPST " << test_week << "/" << test_sow;
    }
    EXPECT_EQ(reference_identity.toe_week, roundtrip_identity.toe_week) << "selected-identity Toe week mismatch";
    EXPECT_DOUBLE_EQ(reference_identity.toe_sow_sec, roundtrip_identity.toe_sow_sec)
        << "selected-identity Toe SOW mismatch at GPST " << test_week << "/" << test_sow;
    EXPECT_EQ(reference_identity.toc_week, roundtrip_identity.toc_week) << "selected-identity Toc week mismatch";
    EXPECT_DOUBLE_EQ(reference_identity.toc_sow_sec, roundtrip_identity.toc_sow_sec)
        << "selected-identity Toc SOW mismatch";
    // The receiver-log contract re-stamps the transmit time with the log delivery time
    // (issue 84: week-boundary delivery causality), so transmit identity is deliberately
    // reported by the adapter but not asserted here.
    // At the mandatory Toe sample the selected identity must also equal the current
    // unchanged source record: this proves that the specific real ephemeris under test
    // (not an adjacent instance) actually participated in the comparison.
    if (expected_iode != nullptr && expected_toe_sow != nullptr) {
        EXPECT_EQ(reference_identity.iode, *expected_iode);
        EXPECT_DOUBLE_EQ(reference_identity.toe_sow_sec, *expected_toe_sow);
        EXPECT_EQ(roundtrip_identity.iode, *expected_iode);
        EXPECT_DOUBLE_EQ(roundtrip_identity.toe_sow_sec, *expected_toe_sow);
    }
    StateDifference difference{};
    compute_state_difference(reference_state, roundtrip_state, &difference);
    stats->compared_states += 1;
    stats->max_position_m = std::max(stats->max_position_m, difference.position_3d_m);
    stats->max_velocity_mps = std::max(stats->max_velocity_mps, difference.velocity_3d_mps);
    stats->max_clock_bias_s = std::max(stats->max_clock_bias_s, std::fabs(difference.clock_bias_diff_s));
    stats->max_clock_drift_sps = std::max(stats->max_clock_drift_sps, std::fabs(difference.clock_drift_diff_sps));
    EXPECT_EQ(reference_state.health, roundtrip_state.health) << "health/validity must survive the round trip";
    EXPECT_LE(difference.position_3d_m, kMaxSatellitePositionDiffM) << "satellite position mismatch";
    EXPECT_LE(difference.velocity_3d_mps, kMaxSatelliteVelocityDiffMps) << "satellite velocity mismatch";
    EXPECT_LE(std::fabs(difference.clock_bias_diff_s), kMaxSatelliteClockBiasDiffS) << "satellite clock bias mismatch";
    EXPECT_LE(std::fabs(difference.clock_drift_diff_sps), kMaxSatelliteClockDriftDiffSps)
        << "satellite clock drift mismatch";
    report_state_csv(satellite_number, family, toe_week, toe_sow, test_week, test_sow, reference_identity.iode,
                     roundtrip_identity.iode, difference);
    return StateSampleOutcome::Compared;
}

TEST(NavEquivalence, SatelliteStateMatchesAcrossProductionRoundTrip) {
    std::string error_message;
    gnss_sim::RtklibNavStore* reference = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(reference, nullptr);
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(reference, nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::RtklibNavStore* roundtrip = nullptr;
    std::uint64_t serialized_count = 0;
    std::uint64_t serialized_ephemeris_count = 0;
    std::uint64_t rejected_count = 0;
    ASSERT_TRUE(build_roundtrip_store(reference, &roundtrip, &serialized_count, &serialized_ephemeris_count,
                                      &rejected_count, &error_message))
        << error_message;
    ASSERT_GT(serialized_count, 0U) << "the real fixture must serialize navigation records";
    ASSERT_EQ(gnss_sim::rtklib_nav_record_count(roundtrip), static_cast<int>(serialized_ephemeris_count))
        << "the production parser/adapter must reconstruct exactly the serialized ephemeris records";

    StateEquivalenceStats stats{};
    std::set<std::string> systems;
    std::uint64_t attempted_epochs = 0;
    std::uint64_t validity_skipped_epochs = 0;
    std::uint64_t records_with_state_comparison = 0;
    std::uint64_t serialized_ephemeris_records = 0;
    const int record_count = gnss_sim::rtklib_nav_output_record_count(reference);
    for (int index = 0; index < record_count; ++index) {
        gnss_sim::NavOutputRecord record{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(reference, index, &record, &error_message)) << error_message;
        if (!is_ephemeris_record(record)) {
            continue;
        }
        std::string message;
        bool supported = false;
        ASSERT_TRUE(
            gnss_sim::format_novatel_nav_output_record(record, output_time(), &message, &supported, &error_message))
            << error_message;
        if (!supported) {
            continue;
        }
        ++serialized_ephemeris_records;
        const int satellite_number =
            is_keplerian_ephemeris(record) ? record.ephemeris.satellite_number : record.glonass.satellite_number;
        const gnss_sim::RtklibBroadcastMessageFamily family =
            is_keplerian_ephemeris(record) ? record.ephemeris.message_family : record.glonass.message_family;
        const int toe_week = is_keplerian_ephemeris(record) ? record.ephemeris.toe_week : record.glonass.toe_week;
        const double toe_sow =
            is_keplerian_ephemeris(record) ? record.ephemeris.toe_sow_sec : record.glonass.toe_sow_sec;
        const int iode = is_keplerian_ephemeris(record) ? record.ephemeris.iode : record.glonass.iode;
        const bool glonass = record.kind == gnss_sim::RtklibNavRecordKind::kGlonassEphemeris;
        const double delta_s = glonass ? 600.0 : 1800.0;
        systems.insert(is_keplerian_ephemeris(record) ? gnss_sim::nav_output_system_name(record.ephemeris.system)
                                                      : "GLO");

        // Toe first: every supported serialized record must produce a state comparison
        // on both paths at its own Toe epoch. Toe-delta / Toe+delta add coverage and
        // may be validity-skipped on both paths together, never one-sided.
        std::uint64_t attempted = 0;
        std::uint64_t compared = 0;
        std::uint64_t validity_skipped = 0;
        bool toe_compared = false;
        const std::uint64_t state_count_before_toe = stats.compared_states;
        const StateSampleOutcome toe_outcome =
            compare_state_at_epoch(reference, roundtrip, satellite_number, family, toe_week, toe_sow, toe_week, toe_sow,
                                   kBeidouOrL1cCode(record), &iode, &toe_sow, &stats, "five-system fixture");
        ++attempted;
        if (toe_outcome == StateSampleOutcome::Compared) {
            ++compared;
            toe_compared = true;
            ++records_with_state_comparison;
        }
        if (toe_compared) {
            for (const double offset : {-delta_s, delta_s}) {
                double test_sow = toe_sow + offset;
                int test_week = toe_week;
                if (test_sow < 0.0) {
                    test_sow += 604800.0;
                    --test_week;
                } else if (test_sow >= 604800.0) {
                    test_sow -= 604800.0;
                    ++test_week;
                }
                ++attempted;
                const StateSampleOutcome offset_outcome = compare_state_at_epoch(
                    reference, roundtrip, satellite_number, family, toe_week, toe_sow, test_week, test_sow,
                    kBeidouOrL1cCode(record), nullptr, nullptr, &stats, "five-system fixture");
                if (offset_outcome == StateSampleOutcome::Compared) {
                    ++compared;
                } else if (offset_outcome == StateSampleOutcome::BothUnavailableValiditySkip) {
                    ++validity_skipped;
                }
            }
        }

        std::cout << "NAV_EQUIV_COVERAGE_CSV," << serialized_ephemeris_records << ','
                  << (is_keplerian_ephemeris(record) ? gnss_sim::nav_output_system_name(record.ephemeris.system)
                                                     : "GLO")
                  << ',' << satellite_number << ',' << family_name(family) << ',' << iode << ',' << toe_week << ','
                  << toe_sow << ',' << attempted << ',' << compared << ',' << validity_skipped << ','
                  << (toe_compared ? 0 : 1) << "\n";
        attempted_epochs += attempted;
        validity_skipped_epochs += validity_skipped;
        EXPECT_TRUE(toe_compared) << "serialized record " << serialized_ephemeris_records << " (sat "
                                  << satellite_number << ") must compare at its own Toe";
    }

    std::cout << "NAV_EQUIV_STATE_SUMMARY,serialized_ephemeris_records=" << serialized_ephemeris_records
              << ",records_with_successful_state_comparison=" << records_with_state_comparison
              << ",attempted_state_epochs=" << attempted_epochs << ",compared_state_epochs=" << stats.compared_states
              << ",validity_skipped_epochs=" << validity_skipped_epochs
              << ",availability_mismatch_epochs=" << stats.availability_mismatch_samples
              << ",rejected_records=" << rejected_count << ",max_position_m=" << stats.max_position_m
              << ",max_velocity_mps=" << stats.max_velocity_mps << ",max_clock_bias_s=" << stats.max_clock_bias_s
              << ",max_clock_drift_sps=" << stats.max_clock_drift_sps << "\n";
    EXPECT_GT(stats.compared_states, 0U);
    EXPECT_GE(serialized_count, 55U) << "the five-system fixture must serialize its full legacy record set";
    EXPECT_EQ(systems.size(), 5U) << "every serialized constellation must be compared";
    EXPECT_EQ(records_with_state_comparison, serialized_ephemeris_records)
        << "every supported serialized ephemeris must be state-compared (Toe mandatory)";
    gnss_sim::destroy_rtklib_nav_store(roundtrip);
    gnss_sim::destroy_rtklib_nav_store(reference);
}

TEST(NavEquivalence, SatelliteStateSelectionMatchesAcrossRealEphemerisTransition) {
    std::string error_message;
    const std::string companion_path =
        std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_galileo_companion_nav.rnx";
    gnss_sim::RtklibNavStore* reference = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(reference, nullptr);
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(reference, companion_path.c_str(), &error_message)) << error_message;

    gnss_sim::RtklibNavStore* roundtrip = nullptr;
    std::uint64_t serialized_count = 0;
    std::uint64_t serialized_ephemeris_count = 0;
    std::uint64_t rejected_count = 0;
    ASSERT_TRUE(build_roundtrip_store(reference, &roundtrip, &serialized_count, &serialized_ephemeris_count,
                                      &rejected_count, &error_message))
        << error_message;

    // The companion fixture carries two consecutive real E02 INAV instances (IODnav 1 at
    // Toe 457800 and IODnav 2 at Toe 458400, 600 s apart). Sample before, at, between,
    // and after both instances so the older/newer record selection is exercised on both
    // navigation paths at the transition.
    std::vector<double> instance_toes;
    int satellite_number = 0;
    int toe_week = 0;
    for (int index = 0; index < gnss_sim::rtklib_nav_output_record_count(roundtrip); ++index) {
        gnss_sim::NavOutputRecord record{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(roundtrip, index, &record, &error_message)) << error_message;
        if (record.kind == gnss_sim::RtklibNavRecordKind::kEphemeris &&
            record.ephemeris.system == gnss_sim::NavOutputSystem::kGalileo && record.ephemeris.prn == 2 &&
            record.ephemeris.message_family == gnss_sim::RtklibBroadcastMessageFamily::kGalileoInav) {
            instance_toes.push_back(record.ephemeris.toe_sow_sec);
            satellite_number = record.ephemeris.satellite_number;
            toe_week = record.ephemeris.toe_week;
        }
    }
    ASSERT_EQ(instance_toes.size(), 2U) << "the fixture must provide two real E02 INAV instances";
    std::sort(instance_toes.begin(), instance_toes.end());

    StateEquivalenceStats stats{};
    for (const double sow : {instance_toes[0] - 600.0, instance_toes[0], (instance_toes[0] + instance_toes[1]) / 2.0,
                             instance_toes[1], instance_toes[1] + 600.0}) {
        compare_state_at_epoch(reference, roundtrip, satellite_number,
                               gnss_sim::RtklibBroadcastMessageFamily::kGalileoInav, toe_week, instance_toes[0],
                               toe_week, sow, kL1CObservationCode, nullptr, nullptr, &stats,
                               "real E02 instance transition");
    }

    std::cout << "NAV_EQUIV_TRANSITION_SUMMARY,compared_states=" << stats.compared_states
              << ",max_position_m=" << stats.max_position_m << ",max_velocity_mps=" << stats.max_velocity_mps
              << ",max_clock_bias_s=" << stats.max_clock_bias_s << ",max_clock_drift_sps=" << stats.max_clock_drift_sps
              << "\n";
    EXPECT_EQ(stats.compared_states, 5U) << "every transition epoch must be compared on both navigation paths";
    gnss_sim::destroy_rtklib_nav_store(roundtrip);
    gnss_sim::destroy_rtklib_nav_store(reference);
}

TEST(NavEquivalence, CorrectionParametersSurviveProductionRoundTrip) {
    std::string error_message;
    gnss_sim::RtklibNavStore* reference = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(reference, nullptr);
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(reference, nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::RtklibNavStore* roundtrip = nullptr;
    std::uint64_t serialized_count = 0;
    std::uint64_t serialized_ephemeris_count = 0;
    std::uint64_t rejected_count = 0;
    ASSERT_TRUE(build_roundtrip_store(reference, &roundtrip, &serialized_count, &serialized_ephemeris_count,
                                      &rejected_count, &error_message))
        << error_message;

    // Store-level record identity: the reconstructed store must contain exactly the
    // serialized ephemeris instances (satellite + family + IODE/IODnav + Toe week/SOW),
    // so both positioning paths select among the same real broadcast instances.
    std::vector<std::string> reference_keys;
    std::vector<std::string> roundtrip_keys;
    for (int index = 0; index < gnss_sim::rtklib_nav_output_record_count(reference); ++index) {
        gnss_sim::NavOutputRecord record{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(reference, index, &record, &error_message)) << error_message;
        std::string message;
        bool supported = false;
        ASSERT_TRUE(
            gnss_sim::format_novatel_nav_output_record(record, output_time(), &message, &supported, &error_message))
            << error_message;
        if (supported && is_ephemeris_record(record)) {
            reference_keys.push_back(record_identity_key(record));
        }
    }
    for (int index = 0; index < gnss_sim::rtklib_nav_output_record_count(roundtrip); ++index) {
        gnss_sim::NavOutputRecord record{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(roundtrip, index, &record, &error_message)) << error_message;
        if (is_ephemeris_record(record)) {
            roundtrip_keys.push_back(record_identity_key(record));
        }
    }
    std::sort(reference_keys.begin(), reference_keys.end());
    std::sort(roundtrip_keys.begin(), roundtrip_keys.end());
    EXPECT_EQ(reference_keys, roundtrip_keys)
        << "the production round trip must preserve the full broadcast instance identity set";

    std::uint64_t compared_tgd = 0;
    std::uint64_t compared_bgd = 0;
    std::uint64_t compared_ion = 0;
    std::uint64_t stage_b_bias_compared = 0;
    std::uint64_t stage_b_ion_compared = 0;
    std::uint64_t outside_contract = 0;
    const int record_count = gnss_sim::rtklib_nav_output_record_count(reference);
    for (int index = 0; index < record_count; ++index) {
        gnss_sim::NavOutputRecord record{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(reference, index, &record, &error_message)) << error_message;
        std::string message;
        bool supported = false;
        ASSERT_TRUE(
            gnss_sim::format_novatel_nav_output_record(record, output_time(), &message, &supported, &error_message))
            << error_message;
        if (!supported) {
            ++outside_contract;
            continue;
        }
        gnss_sim::ParsedSerializedNavRecord parsed{};
        bool recognized = false;
        ASSERT_TRUE(
            gnss_sim::parse_serialized_novatel_nav_line_independent(message, &parsed, &recognized, &error_message))
            << error_message;
        ASSERT_TRUE(recognized) << "every serialized NAV message must be recognized by the production parser";
        const bool keplerian = is_keplerian_ephemeris(record);
        const int satellite_number = keplerian ? record.ephemeris.satellite_number : record.glonass.satellite_number;
        const gnss_sim::RtklibBroadcastMessageFamily family =
            keplerian ? record.ephemeris.message_family : record.glonass.message_family;
        const int toe_week = keplerian ? record.ephemeris.toe_week : record.glonass.toe_week;
        const double toe_sow = keplerian ? record.ephemeris.toe_sow_sec : record.glonass.toe_sow_sec;

        if (record.kind == gnss_sim::RtklibNavRecordKind::kGlonassEphemeris) {
            const double reference_delay = record.glonass.differential_delay_sec;
            const double roundtrip_delay = parsed.record.glonass.differential_delay_sec;
            EXPECT_LE(relative_diff(reference_delay, roundtrip_delay), kMaxCorrectionRelativeDiff)
                << "GLONASS delay must survive the serialization boundary unchanged";
            ++compared_bgd;
        } else if (record.kind == gnss_sim::RtklibNavRecordKind::kIonosphere) {
            ASSERT_EQ(record.ionosphere.coefficient_count, parsed.record.ionosphere.coefficient_count);
            for (int coefficient = 0; coefficient < record.ionosphere.coefficient_count; ++coefficient) {
                EXPECT_LE(relative_diff(record.ionosphere.coefficients[coefficient],
                                        parsed.record.ionosphere.coefficients[coefficient]),
                          kMaxCorrectionRelativeDiff)
                    << "ionosphere coefficient " << coefficient << " must survive the serialization boundary";
            }
            EXPECT_EQ(record.ionosphere.leap_seconds, parsed.record.ionosphere.leap_seconds)
                << "leap seconds must survive the serialization boundary unchanged";
            ++compared_ion;

            // Stage B: the final reconstructed RTKLIB store must hold exactly the real
            // serialized BeiDou ION coefficients (and the GPST-UTC leap seconds after the
            // parser's documented BDT-UTC +14 s restoration). The reference store cannot
            // serve as the expected side here because pinned RTKLIB never projects
            // RINEX4 BeiDou ION into nav.ion_cmp; the unchanged real-RINEX projection is
            // the expected truth.
            if (record.ionosphere.system == gnss_sim::NavOutputSystem::kBeidou) {
                gnss_sim::RtklibIonosphereModelState model{};
                ASSERT_TRUE(gnss_sim::rtklib_broadcast_ionosphere_model_state(
                    roundtrip, gnss_sim::RtklibIonosphereSystem::kBeidouLegacy, &model, &error_message))
                    << error_message;
                for (int coefficient = 0; coefficient < 8; ++coefficient) {
                    EXPECT_LE(
                        relative_diff(record.ionosphere.coefficients[coefficient], model.coefficients[coefficient]),
                        kMaxCorrectionRelativeDiff)
                        << "stage-B BeiDou ionosphere coefficient " << coefficient
                        << " must match the real serialized record in the final store";
                }
                EXPECT_EQ(model.leap_seconds, record.ionosphere.leap_seconds)
                    << "stage-B leap seconds must match the real serialized record";
                ++stage_b_ion_compared;
            }
            // Stage B for the GPS legacy model state: both final stores must hold the
            // same (zero) legacy GPS Klobuchar state, so the solver ionosphere model is
            // identical on both positioning paths.
            gnss_sim::RtklibIonosphereModelState reference_gps_model{};
            gnss_sim::RtklibIonosphereModelState roundtrip_gps_model{};
            ASSERT_TRUE(gnss_sim::rtklib_broadcast_ionosphere_model_state(
                reference, gnss_sim::RtklibIonosphereSystem::kGps, &reference_gps_model, &error_message))
                << error_message;
            ASSERT_TRUE(gnss_sim::rtklib_broadcast_ionosphere_model_state(
                roundtrip, gnss_sim::RtklibIonosphereSystem::kGps, &roundtrip_gps_model, &error_message))
                << error_message;
            for (int coefficient = 0; coefficient < 8; ++coefficient) {
                EXPECT_DOUBLE_EQ(reference_gps_model.coefficients[coefficient],
                                 roundtrip_gps_model.coefficients[coefficient])
                    << "stage-B GPS ionosphere coefficient " << coefficient << " must match between stores";
            }
        } else {
            const double reference_delay = record.ephemeris.tgd_sec[0];
            const double roundtrip_delay = parsed.record.ephemeris.tgd_sec[0];
            EXPECT_LE(relative_diff(reference_delay, roundtrip_delay), kMaxCorrectionRelativeDiff)
                << "TGD must survive the serialization boundary unchanged";
            if (record.ephemeris.system == gnss_sim::NavOutputSystem::kBeidou) {
                EXPECT_LE(relative_diff(record.ephemeris.tgd_sec[1], parsed.record.ephemeris.tgd_sec[1]),
                          kMaxCorrectionRelativeDiff)
                    << "BeiDou BGD2 must survive the serialization boundary unchanged";
                ++compared_bgd;
            } else {
                ++compared_tgd;
            }
            EXPECT_EQ(record.ephemeris.svh, parsed.record.ephemeris.svh) << "health must survive unchanged";
        }

        if (is_ephemeris_record(record)) {
            // Stage B: the final reconstructed RTKLIB store must expose the same real
            // broadcast group-delay values as the reference store for the same selected
            // instance (queried through the production bias API at Toe).
            gnss_sim::RtklibBroadcastBiasData reference_bias{};
            gnss_sim::RtklibBroadcastBiasData roundtrip_bias{};
            ASSERT_TRUE(gnss_sim::rtklib_broadcast_bias_data_for_family(reference, toe_week, toe_sow, satellite_number,
                                                                        family, &reference_bias, &error_message))
                << error_message;
            ASSERT_TRUE(gnss_sim::rtklib_broadcast_bias_data_for_family(roundtrip, toe_week, toe_sow, satellite_number,
                                                                        family, &roundtrip_bias, &error_message))
                << error_message;
            EXPECT_EQ(reference_bias.iode, roundtrip_bias.iode) << "stage-B selected-instance IODE mismatch";
            for (int term = 0; term < 4; ++term) {
                EXPECT_LE(relative_diff(reference_bias.tgd_sec[term], roundtrip_bias.tgd_sec[term]),
                          kMaxCorrectionRelativeDiff)
                    << "stage-B TGD/BGD term " << term << " must match in the final reconstructed store";
            }
            EXPECT_LE(relative_diff(reference_bias.glonass_dtaun_sec, roundtrip_bias.glonass_dtaun_sec),
                      kMaxCorrectionRelativeDiff)
                << "stage-B GLONASS delay must match in the final reconstructed store";
            ++stage_b_bias_compared;
        }
    }

    // Stage B for the serialized ionosphere: the GPS legacy ionosphere state of the
    // final reconstructed store must produce the same production reference delay as the
    // reference store at identical geometry. The real serialized BeiDou ION record
    // restores nav.ion_cmp beyond the reference solver state (documented issue 84
    // behavior: the reference store never projects RINEX4 BeiDou ION), so it is reported
    // rather than asserted.
    gnss_sim::RtklibIonosphereResult reference_ion{};
    gnss_sim::RtklibIonosphereResult roundtrip_ion{};
    const double receiver_ecef_m[3] = {-2167434.0, 4392540.0, 4076320.0};
    ASSERT_TRUE(gnss_sim::rtklib_broadcast_ionosphere_reference_delay(
        reference, gnss_sim::RtklibIonosphereSystem::kGps, kReferenceGpsWeek, kReferenceSowSec, receiver_ecef_m, 0.5,
        0.7, &reference_ion, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::rtklib_broadcast_ionosphere_reference_delay(
        roundtrip, gnss_sim::RtklibIonosphereSystem::kGps, kReferenceGpsWeek, kReferenceSowSec, receiver_ecef_m, 0.5,
        0.7, &roundtrip_ion, &error_message))
        << error_message;
    EXPECT_DOUBLE_EQ(reference_ion.reference_delay_m, roundtrip_ion.reference_delay_m)
        << "stage-B GPS ionosphere state must match the reference store";
    ++compared_ion;

    std::cout << "NAV_EQUIV_CORRECTION_SUMMARY,serialized_records=" << serialized_count
              << ",outside_contract_records=" << outside_contract << ",compared_tgd=" << compared_tgd
              << ",compared_bgd=" << compared_bgd << ",compared_ion=" << compared_ion
              << ",stage_b_bias_compared=" << stage_b_bias_compared << ",stage_b_ion_compared=" << stage_b_ion_compared
              << ",stage_b_gps_ion_delay_ref_m=" << reference_ion.reference_delay_m
              << ",stage_b_gps_ion_delay_rt_m=" << roundtrip_ion.reference_delay_m << "\n";
    EXPECT_GT(compared_tgd, 0U) << "the real fixture must exercise broadcast group delays";
    EXPECT_GT(compared_ion, 0U) << "the real fixture must exercise serialized ionosphere records";
    EXPECT_GT(outside_contract, 0U) << "modern families are documented as outside the receiver output contract";
    gnss_sim::destroy_rtklib_nav_store(roundtrip);
    gnss_sim::destroy_rtklib_nav_store(reference);
}

TEST(NavEquivalence, PairedPositioningMatchesBetweenOriginalRinexAndConvertedNavigation) {
    std::string error_message;
    gnss_sim::RtklibNavStore* reference = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(reference, nullptr);
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(reference, nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::RtklibNavStore* roundtrip = nullptr;
    std::uint64_t serialized_count = 0;
    std::uint64_t serialized_ephemeris_count = 0;
    std::uint64_t rejected_count = 0;
    ASSERT_TRUE(build_roundtrip_store(reference, &roundtrip, &serialized_count, &serialized_ephemeris_count,
                                      &rejected_count, &error_message))
        << error_message;

    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "nav_equivalence_run";
    gnss_sim::SimulatorRunSummary simulator_summary{};
    std::error_code fs_error;
    std::filesystem::remove_all(directory, fs_error);
    fs_error.clear();
    if (!std::filesystem::create_directories(directory, fs_error) || fs_error) {
        FAIL() << "cannot create simulator working directory: " << fs_error.message();
    }
    ASSERT_TRUE(gnss_sim::run_simulator(
        simulator_config(), {nav_path().c_str(), (directory / "simulated.log").string().c_str(), output_time()},
        &simulator_summary, &error_message))
        << error_message;

    std::ifstream log((directory / "simulated.log").string(), std::ios::binary);
    ASSERT_TRUE(log) << "simulator must produce a receiver log";
    std::string line;
    double max_position_m = 0.0;
    double sum_position_squared_m2 = 0.0;
    double max_clock_m = 0.0;
    std::uint64_t compared_epochs = 0;
    std::uint64_t status_divergences = 0;
    std::uint64_t satellite_set_divergences = 0;
    while (std::getline(log, line)) {
        if (line.rfind("#RANGEA,", 0) != 0) {
            continue;
        }
        gnss_sim::ParsedRangeEpoch epoch{};
        ASSERT_TRUE(gnss_sim::parse_rangea_line_independent(line, &epoch, &error_message)) << error_message;
        std::vector<gnss_sim::RtklibRawCodeObservation> selected;
        ASSERT_TRUE(gnss_sim::select_rangea_position_observations(epoch, &selected, &error_message)) << error_message;
        if (selected.empty()) {
            continue;
        }

        gnss_sim::RtklibPositionSolution reference_solution{};
        gnss_sim::RtklibPositionSolution roundtrip_solution{};
        const bool reference_solved = gnss_sim::rtklib_solve_raw_single_position(
            reference, epoch.gps_week, epoch.sow_sec, selected.data(), static_cast<int>(selected.size()),
            kElevationMaskDeg, true, &reference_solution, &error_message);
        ASSERT_TRUE(reference_solved) << error_message;
        const bool roundtrip_solved = gnss_sim::rtklib_solve_raw_single_position(
            roundtrip, epoch.gps_week, epoch.sow_sec, selected.data(), static_cast<int>(selected.size()),
            kElevationMaskDeg, true, &roundtrip_solution, &error_message);
        ASSERT_TRUE(roundtrip_solved) << error_message;

        if (reference_solution.valid != roundtrip_solution.valid) {
            ++status_divergences;
            ADD_FAILURE() << "solution status diverged at GPST " << epoch.gps_week << "/" << epoch.sow_sec;
            continue;
        }
        if (!reference_solution.valid) {
            continue;
        }
        if (reference_solution.used_satellites != roundtrip_solution.used_satellites) {
            ++satellite_set_divergences;
            ADD_FAILURE() << "used-satellite count diverged at GPST " << epoch.gps_week << "/" << epoch.sow_sec;
        }
        // Exact used-satellite set equality: compare the RTKLIB-reported masks and
        // verify each mask's population matches the reported count.
        std::uint64_t reference_count_from_mask = 0;
        std::uint64_t roundtrip_count_from_mask = 0;
        bool set_match = true;
        for (int word = 0; word < 7; ++word) {
            if (reference_solution.used_satellite_mask[word] != roundtrip_solution.used_satellite_mask[word]) {
                set_match = false;
            }
            for (int bit = 0; bit < 64; ++bit) {
                reference_count_from_mask += (reference_solution.used_satellite_mask[word] >> bit) & 1ULL;
                roundtrip_count_from_mask += (roundtrip_solution.used_satellite_mask[word] >> bit) & 1ULL;
            }
        }
        EXPECT_EQ(reference_count_from_mask, static_cast<std::uint64_t>(reference_solution.used_satellites))
            << "used-satellite mask population must match the reported count";
        EXPECT_EQ(roundtrip_count_from_mask, static_cast<std::uint64_t>(roundtrip_solution.used_satellites))
            << "used-satellite mask population must match the reported count";
        if (!set_match) {
            ++satellite_set_divergences;
            for (const gnss_sim::RtklibRawCodeObservation& observation : selected) {
                const int index = observation.satellite_number - 1;
                const bool in_reference = (reference_solution.used_satellite_mask[index / 64] >> (index % 64)) & 1ULL;
                const bool in_roundtrip = (roundtrip_solution.used_satellite_mask[index / 64] >> (index % 64)) & 1ULL;
                if (in_reference != in_roundtrip) {
                    ADD_FAILURE() << "satellite " << observation.satellite_number << " used only by "
                                  << (in_reference ? "reference" : "roundtrip") << " at GPST " << epoch.gps_week << "/"
                                  << epoch.sow_sec;
                }
            }
        }
        double position_3d = 0.0;
        double per_axis[3] = {0.0, 0.0, 0.0};
        for (int axis = 0; axis < 3; ++axis) {
            per_axis[axis] = reference_solution.position_ecef_m[axis] - roundtrip_solution.position_ecef_m[axis];
            position_3d += per_axis[axis] * per_axis[axis];
        }
        position_3d = std::sqrt(position_3d);
        const double clock_diff_m = reference_solution.receiver_clock_bias_m - roundtrip_solution.receiver_clock_bias_m;
        ++compared_epochs;
        max_position_m = std::max(max_position_m, position_3d);
        sum_position_squared_m2 += position_3d * position_3d;
        max_clock_m = std::max(max_clock_m, std::fabs(clock_diff_m));
        std::cout << "NAV_EQUIV_POSITION_CSV," << epoch.gps_week << ',' << epoch.sow_sec << ','
                  << reference_solution.used_satellites << ',' << per_axis[0] << ',' << per_axis[1] << ','
                  << per_axis[2] << ',' << position_3d << ',' << clock_diff_m << ",set_match=" << (set_match ? 1 : 0)
                  << "\n";
        EXPECT_LE(position_3d, kMaxPositioningPositionDiffM) << "paired positioning must agree";
        EXPECT_LE(std::fabs(clock_diff_m), kMaxPositioningClockDiffM) << "receiver clock must agree";
    }

    const double rms_position_m =
        compared_epochs > 0 ? std::sqrt(sum_position_squared_m2 / static_cast<double>(compared_epochs)) : 0.0;
    std::cout << "NAV_EQUIV_POSITION_SUMMARY,compared_epochs=" << compared_epochs
              << ",max_position_m=" << max_position_m << ",rms_position_m=" << rms_position_m
              << ",max_clock_m=" << max_clock_m << ",status_divergences=" << status_divergences
              << ",satellite_set_divergences=" << satellite_set_divergences << "\n";
    EXPECT_GT(compared_epochs, 0U) << "the paired positioning comparison must cover real epochs";
    EXPECT_EQ(status_divergences, 0U);
    EXPECT_EQ(satellite_set_divergences, 0U);
    gnss_sim::destroy_rtklib_nav_store(roundtrip);
    gnss_sim::destroy_rtklib_nav_store(reference);
}

} // namespace
