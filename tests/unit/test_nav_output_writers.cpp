#include "gnss/nav_output_record.h"
#include "gnss/navigation_state.h"
#include "gnss/rtklib_adapter.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_time.h"
#include "output/novatel_ascii.h"
#include "output/novatel_nav_writer.h"
#include "output/unicore_nav_writer.h"

extern "C" {
#include <rtklib.h>
}

#ifdef lock
#undef lock
#endif
#ifdef unlock
#undef unlock
#endif

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <gtest/gtest.h>
#include <set>
#include <string>
#include <vector>

namespace {

std::string data_path(const char* name) {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/" + name;
}

gnss_sim::SimTime output_time() {
    gnss_sim::SimTime result{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180000.0, &result));
    return result;
}

std::string log_name(const std::string& message) {
    if (message.empty() || message[0] != '#') {
        return {};
    }
    const std::size_t comma = message.find(',');
    return comma == std::string::npos ? std::string{} : message.substr(1, comma - 1);
}

bool valid_ascii_crc(const std::string& message) {
    if (message.size() < 12 || message[0] != '#' || message.substr(message.size() - 2) != "\r\n") {
        return false;
    }
    const std::size_t star = message.rfind('*');
    if (star == std::string::npos || star + 11 != message.size()) {
        return false;
    }
    const std::string payload = message.substr(1, star - 1);
    const std::string crc_text = message.substr(star + 1, 8);
    char* end = nullptr;
    const unsigned long parsed = std::strtoul(crc_text.c_str(), &end, 16);
    return end != nullptr && *end == '\0' && parsed == gnss_sim::novatel_ascii::crc32(payload);
}

std::string body_between_semicolon_and_crc(const std::string& message) {
    const std::size_t semicolon = message.find(';');
    const std::size_t star = message.rfind('*');
    if (semicolon == std::string::npos || star == std::string::npos || star < semicolon) {
        return {};
    }
    return message.substr(semicolon + 1, star - semicolon - 1);
}

std::vector<std::string> split_body_fields(const std::string& body) {
    std::vector<std::string> fields;
    std::size_t begin = 0;
    while (true) {
        const std::size_t comma = body.find(',', begin);
        if (comma == std::string::npos) {
            fields.push_back(body.substr(begin));
            break;
        }
        fields.push_back(body.substr(begin, comma - begin));
        begin = comma + 1;
    }
    return fields;
}

// Rebuilds the ephemeris an independent serialized-log parser would obtain from a
// GALEPHEMERISA body: fields[13..27] hold the common orbit block, fields[32..35]
// hold the INAV clock block, and field 12 holds Toe SOW; the GPS week comes from
// the receiver-log header time, not from the body.
eph_t ephemeris_from_galileo_body_fields(const std::vector<std::string>& fields, int toe_week, int satellite_number) {
    eph_t eph{};
    eph.sat = satellite_number;
    const double sqrt_a = std::stod(fields[13]);
    eph.A = sqrt_a * sqrt_a;
    eph.deln = std::stod(fields[14]);
    eph.M0 = std::stod(fields[15]);
    eph.e = std::stod(fields[16]);
    eph.omg = std::stod(fields[17]);
    eph.cuc = std::stod(fields[18]);
    eph.cus = std::stod(fields[19]);
    eph.crc = std::stod(fields[20]);
    eph.crs = std::stod(fields[21]);
    eph.cic = std::stod(fields[22]);
    eph.cis = std::stod(fields[23]);
    eph.i0 = std::stod(fields[24]);
    eph.idot = std::stod(fields[25]);
    eph.OMG0 = std::stod(fields[26]);
    eph.OMGd = std::stod(fields[27]);
    eph.toe = gpst2time(toe_week, std::stod(fields[12]));
    eph.toc = gpst2time(toe_week, std::stod(fields[32]));
    eph.f0 = std::stod(fields[33]);
    eph.f1 = std::stod(fields[34]);
    eph.f2 = std::stod(fields[35]);
    return eph;
}

eph_t ephemeris_from_keplerian_record(const gnss_sim::KeplerianNavOutputData& data) {
    eph_t eph{};
    eph.sat = data.satellite_number;
    eph.A = data.semi_major_axis_m;
    eph.deln = data.delta_mean_motion_radps;
    eph.M0 = data.mean_anomaly_rad;
    eph.e = data.eccentricity;
    eph.omg = data.argument_of_perigee_rad;
    eph.cuc = data.cuc_rad;
    eph.cus = data.cus_rad;
    eph.crc = data.crc_m;
    eph.crs = data.crs_m;
    eph.cic = data.cic_rad;
    eph.cis = data.cis_rad;
    eph.i0 = data.inclination_rad;
    eph.idot = data.inclination_dot_radps;
    eph.OMG0 = data.omega0_rad;
    eph.OMGd = data.omega_dot_radps;
    eph.toe = gpst2time(data.toe_week, data.toe_sow_sec);
    eph.toc = gpst2time(data.toc_week, data.toc_sow_sec);
    eph.f0 = data.clock_bias_sec;
    eph.f1 = data.clock_drift_sec_per_sec;
    eph.f2 = data.clock_drift_rate_sec_per_sec2;
    return eph;
}

void collect_supported_names(const gnss_sim::RtklibNavStore* store, bool unicore, std::set<std::string>* names) {
    ASSERT_NE(store, nullptr);
    ASSERT_NE(names, nullptr);
    std::string error_message;
    const int count = gnss_sim::rtklib_nav_output_record_count(store);
    for (int index = 0; index < count; ++index) {
        std::string first;
        std::string second;
        bool first_supported = false;
        bool second_supported = false;
        const bool first_ok = unicore
                                  ? gnss_sim::format_unicore_receiver_nav_record(store, index, output_time(), &first,
                                                                                 &first_supported, &error_message)
                                  : gnss_sim::format_novatel_receiver_nav_record(store, index, output_time(), &first,
                                                                                 &first_supported, &error_message);
        ASSERT_TRUE(first_ok) << error_message;
        const bool second_ok = unicore
                                   ? gnss_sim::format_unicore_receiver_nav_record(store, index, output_time(), &second,
                                                                                  &second_supported, &error_message)
                                   : gnss_sim::format_novatel_receiver_nav_record(store, index, output_time(), &second,
                                                                                  &second_supported, &error_message);
        ASSERT_TRUE(second_ok) << error_message;
        EXPECT_EQ(first_supported, second_supported);
        EXPECT_EQ(first, second);
        if (first_supported) {
            EXPECT_TRUE(valid_ascii_crc(first));
            names->insert(log_name(first));
        }
    }
}

bool load_real_ephemeris_record(const char* fixture_name, gnss_sim::NavOutputSystem system,
                                gnss_sim::RtklibBroadcastMessageFamily family, gnss_sim::NavOutputRecord* record,
                                std::string* error_message) {
    if (fixture_name == nullptr || record == nullptr) {
        if (error_message != nullptr) {
            *error_message = "real ephemeris test helper received invalid arguments";
        }
        return false;
    }
    gnss_sim::RtklibNavStore* store = gnss_sim::create_rtklib_nav_store();
    if (store == nullptr) {
        if (error_message != nullptr) {
            *error_message = "cannot allocate real ephemeris test NAV store";
        }
        return false;
    }
    if (!gnss_sim::load_rinex_nav_file(store, data_path(fixture_name).c_str(), error_message)) {
        gnss_sim::destroy_rtklib_nav_store(store);
        return false;
    }

    const int count = gnss_sim::rtklib_nav_output_record_count(store);
    for (int index = 0; index < count; ++index) {
        gnss_sim::NavOutputRecord candidate{};
        if (!gnss_sim::rtklib_nav_output_record(store, index, &candidate, error_message)) {
            gnss_sim::destroy_rtklib_nav_store(store);
            return false;
        }
        if (candidate.kind == gnss_sim::RtklibNavRecordKind::kEphemeris && candidate.ephemeris.system == system &&
            candidate.ephemeris.message_family == family) {
            *record = candidate;
            gnss_sim::destroy_rtklib_nav_store(store);
            return true;
        }
    }

    gnss_sim::destroy_rtklib_nav_store(store);
    if (error_message != nullptr) {
        *error_message = "requested real RINEX ephemeris family is absent from fixture";
    }
    return false;
}

gnss_sim::SimTime ephemeris_output_time(const gnss_sim::NavOutputRecord& record) {
    gnss_sim::SimTime result{};
    EXPECT_EQ(record.kind, gnss_sim::RtklibNavRecordKind::kEphemeris);
    EXPECT_TRUE(
        gnss_sim::sim_time_from_week_sow(record.ephemeris.transmit_week, record.ephemeris.transmit_sow_sec, &result));
    return result;
}

TEST(NavOutputWriter, LegacyMixedRinexCoversFiveEphemerisFamilies) {
    gnss_sim::RtklibNavStore* store = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(store, data_path("mixed_nav_2019.rnx").c_str(), &error_message))
        << error_message;

    std::set<std::string> novatel_names;
    std::set<std::string> unicore_names;
    collect_supported_names(store, false, &novatel_names);
    collect_supported_names(store, true, &unicore_names);

    EXPECT_TRUE(novatel_names.count("GPSEPHEMA"));
    EXPECT_TRUE(novatel_names.count("GLOEPHEMERISA"));
    EXPECT_TRUE(novatel_names.count("GALEPHEMERISA"));
    EXPECT_TRUE(novatel_names.count("BD2EPHEMA"));
    EXPECT_TRUE(novatel_names.count("QZSSEPHEMERISA"));
    EXPECT_TRUE(unicore_names.count("GPSEPHA"));
    EXPECT_TRUE(unicore_names.count("GLOEPHA"));
    EXPECT_TRUE(unicore_names.count("GALEPHA"));
    EXPECT_TRUE(unicore_names.count("BDSEPHA"));
    EXPECT_TRUE(unicore_names.count("QZSSEPHA"));

    gnss_sim::destroy_rtklib_nav_store(store);
}

TEST(NavOutputWriter, LegacyIonosphereMetadataMapsToFrozenFamilies) {
    gnss_sim::RtklibNavStore* store = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(store, data_path("ionosphere_nav_2019.rnx").c_str(), &error_message))
        << error_message;

    std::set<std::string> novatel_names;
    std::set<std::string> unicore_names;
    collect_supported_names(store, false, &novatel_names);
    collect_supported_names(store, true, &unicore_names);
    EXPECT_TRUE(novatel_names.count("IONUTCA"));
    EXPECT_TRUE(novatel_names.count("BD2IONUTCA"));
    EXPECT_TRUE(unicore_names.count("GPSIONA"));
    EXPECT_TRUE(unicore_names.count("BDSIONA"));
    EXPECT_TRUE(unicore_names.count("GALIONA"));

    gnss_sim::destroy_rtklib_nav_store(store);
}

TEST(NavOutputWriter, ColdUsesReceiverAvailabilityAndSuppressesDuplicateDelivery) {
    gnss_sim::NavigationState* state = gnss_sim::create_navigation_state();
    ASSERT_NE(state, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_truth_navigation(state, data_path("nav_updates_2019.rnx").c_str(), &error_message))
        << error_message;

    gnss_sim::SimTime startup{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 179000.0, &startup));
    ASSERT_TRUE(gnss_sim::initialize_receiver_navigation(state, gnss_sim::StartupMode::COLD, startup, &error_message))
        << error_message;
    EXPECT_EQ(gnss_sim::rtklib_nav_output_record_count(gnss_sim::receiver_navigation_store(state)), 0);

    gnss_sim::SimTime available{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 179020.0, &available));
    gnss_sim::NavigationUpdateEvent event{};
    bool emitted = false;
    ASSERT_TRUE(gnss_sim::apply_truth_navigation_record(state, 0, available, &event, &emitted, &error_message))
        << error_message;
    ASSERT_TRUE(emitted);
    EXPECT_EQ(event.receiver_record_index, 0);

    std::string message;
    bool supported = false;
    ASSERT_TRUE(gnss_sim::format_novatel_receiver_nav_record(gnss_sim::receiver_navigation_store(state),
                                                             event.receiver_record_index, available, &message,
                                                             &supported, &error_message))
        << error_message;
    EXPECT_TRUE(supported);
    EXPECT_EQ(log_name(message), "GPSEPHEMA");
    EXPECT_TRUE(valid_ascii_crc(message));

    emitted = true;
    ASSERT_TRUE(gnss_sim::apply_truth_navigation_record(state, 0, available, &event, &emitted, &error_message))
        << error_message;
    EXPECT_FALSE(emitted);
    EXPECT_EQ(gnss_sim::rtklib_nav_output_record_count(gnss_sim::receiver_navigation_store(state)), 1);
    gnss_sim::destroy_navigation_state(state);
}

TEST(NavOutputWriter, HotAndWarmRestoreTheSameDeterministicReceiverNavBytes) {
    std::vector<std::string> baseline;
    for (const gnss_sim::StartupMode mode : {gnss_sim::StartupMode::HOT, gnss_sim::StartupMode::WARM}) {
        gnss_sim::NavigationState* state = gnss_sim::create_navigation_state();
        ASSERT_NE(state, nullptr);
        std::string error_message;
        ASSERT_TRUE(gnss_sim::load_truth_navigation(state, data_path("mixed_nav_2019.rnx").c_str(), &error_message))
            << error_message;
        gnss_sim::SimTime startup{};
        ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 300000.0, &startup));
        ASSERT_TRUE(gnss_sim::initialize_receiver_navigation(state, mode, startup, &error_message)) << error_message;

        std::vector<std::string> current;
        const gnss_sim::RtklibNavStore* receiver = gnss_sim::receiver_navigation_store(state);
        for (int index = 0; index < gnss_sim::rtklib_nav_output_record_count(receiver); ++index) {
            std::string message;
            bool supported = false;
            ASSERT_TRUE(gnss_sim::format_unicore_receiver_nav_record(receiver, index, startup, &message, &supported,
                                                                     &error_message))
                << error_message;
            if (supported) {
                current.push_back(message);
            }
        }
        if (baseline.empty()) {
            baseline = current;
        } else {
            EXPECT_EQ(current, baseline);
        }
        gnss_sim::destroy_navigation_state(state);
    }
    EXPECT_FALSE(baseline.empty());
}

TEST(NavOutputWriter, RealModernBdsAndNavicStayWithinFrozenOutputScope) {
    std::string message;
    std::string error_message;
    bool supported = false;

    gnss_sim::NavOutputRecord bds{};
    ASSERT_TRUE(load_real_ephemeris_record("brd400dlr_rinex4_acceptance_nav.rnx", gnss_sim::NavOutputSystem::kBeidou,
                                           gnss_sim::RtklibBroadcastMessageFamily::kBeidouBcnav1, &bds, &error_message))
        << error_message;
    const gnss_sim::SimTime bds_time = ephemeris_output_time(bds);
    ASSERT_TRUE(gnss_sim::format_unicore_nav_output_record(bds, bds_time, &message, &supported, &error_message))
        << error_message;
    EXPECT_TRUE(supported);
    EXPECT_EQ(log_name(message), "BD3EPHA");
    EXPECT_TRUE(valid_ascii_crc(message));
    ASSERT_TRUE(gnss_sim::format_novatel_nav_output_record(bds, bds_time, &message, &supported, &error_message))
        << error_message;
    EXPECT_FALSE(supported);
    EXPECT_TRUE(message.empty());

    gnss_sim::NavOutputRecord navic{};
    ASSERT_TRUE(load_real_ephemeris_record("multi_gnss_acceptance_nav.rnx", gnss_sim::NavOutputSystem::kNavic,
                                           gnss_sim::RtklibBroadcastMessageFamily::kLegacy, &navic, &error_message))
        << error_message;
    const gnss_sim::SimTime navic_time = ephemeris_output_time(navic);
    ASSERT_TRUE(gnss_sim::format_unicore_nav_output_record(navic, navic_time, &message, &supported, &error_message))
        << error_message;
    EXPECT_TRUE(supported);
    EXPECT_EQ(log_name(message), "IRNSSEPHA");
    EXPECT_TRUE(valid_ascii_crc(message));

    std::size_t signal_count = 0;
    gnss_sim::signal_definitions(&signal_count);
    EXPECT_EQ(signal_count, 21U);
}

TEST(NavOutputWriter, NovatelLegacyEphemerisLogsDoNotMasqueradeModernGpsOrQzssFamilies) {
    gnss_sim::RtklibNavStore* store = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(store, nullptr);
    std::string error_message;
    ASSERT_TRUE(
        gnss_sim::load_rinex_nav_file(store, data_path("brd400dlr_rinex4_acceptance_nav.rnx").c_str(), &error_message))
        << error_message;

    int legacy_supported = 0;
    int modern_rejected = 0;
    const int count = gnss_sim::rtklib_nav_output_record_count(store);
    for (int index = 0; index < count; ++index) {
        gnss_sim::NavOutputRecord record{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(store, index, &record, &error_message)) << error_message;
        if (record.kind != gnss_sim::RtklibNavRecordKind::kEphemeris ||
            (record.ephemeris.system != gnss_sim::NavOutputSystem::kGps &&
             record.ephemeris.system != gnss_sim::NavOutputSystem::kQzss)) {
            continue;
        }

        const gnss_sim::SimTime time = ephemeris_output_time(record);
        std::string message;
        bool supported = false;
        ASSERT_TRUE(gnss_sim::format_novatel_nav_output_record(record, time, &message, &supported, &error_message))
            << error_message;
        if (record.ephemeris.message_family == gnss_sim::RtklibBroadcastMessageFamily::kLegacy) {
            ++legacy_supported;
            EXPECT_TRUE(supported);
            EXPECT_TRUE(valid_ascii_crc(message));
            EXPECT_TRUE(log_name(message) == "GPSEPHEMA" || log_name(message) == "QZSSEPHEMERISA");
        } else {
            ++modern_rejected;
            EXPECT_FALSE(supported);
            EXPECT_TRUE(message.empty());
        }
    }

    EXPECT_GT(legacy_supported, 0);
    EXPECT_GT(modern_rejected, 0) << "real BRD400 fixture must contain modern GPS/QZSS ephemeris records";
    gnss_sim::destroy_rtklib_nav_store(store);
}

TEST(NavOutputWriter, RealRinex4BeiDouLegacyIonSerializesWithoutModernFamilyMasquerade) {
    gnss_sim::RtklibNavStore* store = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(store, nullptr);
    std::string error_message;
    ASSERT_TRUE(
        gnss_sim::load_rinex_nav_file(store, data_path("brd400dlr_rinex4_acceptance_nav.rnx").c_str(), &error_message))
        << error_message;

    int legacy_count = 0;
    int modern_count = 0;
    const int count = gnss_sim::rtklib_nav_output_record_count(store);
    for (int index = 0; index < count; ++index) {
        gnss_sim::NavOutputRecord record{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(store, index, &record, &error_message)) << error_message;
        if (record.kind != gnss_sim::RtklibNavRecordKind::kIonosphere ||
            record.ionosphere.system != gnss_sim::NavOutputSystem::kBeidou) {
            continue;
        }

        std::string message;
        bool supported = false;
        const gnss_sim::SimTime time = output_time();
        ASSERT_TRUE(gnss_sim::format_novatel_nav_output_record(record, time, &message, &supported, &error_message))
            << error_message;
        if (record.ionosphere.legacy_metadata) {
            ++legacy_count;
            EXPECT_TRUE(supported);
            EXPECT_EQ(log_name(message), "BD2IONUTCA");
            EXPECT_TRUE(valid_ascii_crc(message));
        } else {
            ++modern_count;
            EXPECT_FALSE(supported);
            EXPECT_TRUE(message.empty());
        }
    }

    EXPECT_GT(legacy_count, 0) << "real BRD400DLR fixture must contain legacy BeiDou D1/D2/D1D2 ION";
    EXPECT_GT(modern_count, 0) << "real BRD400DLR fixture must contain modern BeiDou ION that cannot masquerade as BD2";
    gnss_sim::destroy_rtklib_nav_store(store);
}

TEST(NavOutputWriter, RealRinex4ExplicitGpsIonDoesNotMasqueradeAsLegacyIonutca) {
    gnss_sim::RtklibNavStore* store = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(store, nullptr);
    std::string error_message;
    ASSERT_TRUE(
        gnss_sim::load_rinex_nav_file(store, data_path("brd400dlr_rinex4_acceptance_nav.rnx").c_str(), &error_message))
        << error_message;

    int explicit_gps_ion_count = 0;
    int legacy_gps_ion_count = 0;
    const int count = gnss_sim::rtklib_nav_output_record_count(store);
    for (int index = 0; index < count; ++index) {
        gnss_sim::NavOutputRecord record{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(store, index, &record, &error_message)) << error_message;
        if (record.kind != gnss_sim::RtklibNavRecordKind::kIonosphere ||
            record.ionosphere.system != gnss_sim::NavOutputSystem::kGps) {
            continue;
        }

        std::string message;
        bool supported = false;
        const gnss_sim::SimTime time = output_time();
        ASSERT_TRUE(gnss_sim::format_novatel_nav_output_record(record, time, &message, &supported, &error_message))
            << error_message;
        if (record.ionosphere.legacy_metadata) {
            ++legacy_gps_ion_count;
            EXPECT_TRUE(supported);
            EXPECT_EQ(log_name(message), "IONUTCA");
            EXPECT_TRUE(valid_ascii_crc(message));
        } else {
            ++explicit_gps_ion_count;
            EXPECT_FALSE(supported) << "explicit RINEX4 GPS ION must not masquerade as legacy IONUTCA";
            EXPECT_TRUE(message.empty());
        }
    }

    EXPECT_GT(explicit_gps_ion_count, 0) << "real BRD400DLR fixture must contain explicit GPS ION records";
    EXPECT_EQ(legacy_gps_ion_count, 0) << "BRD400DLR nav.ion_gps stays at the zero/fallback state";
    gnss_sim::destroy_rtklib_nav_store(store);
}

TEST(NavOutputWriter, Bd3IonHasByteLevelGoldenRecord) {
    gnss_sim::NavOutputRecord record{};
    record.kind = gnss_sim::RtklibNavRecordKind::kIonosphere;
    record.ionosphere.system = gnss_sim::NavOutputSystem::kBeidou;
    record.ionosphere.coefficient_count = 9;
    record.ionosphere.legacy_metadata = false;

    std::string message;
    std::string error_message;
    bool supported = false;
    ASSERT_TRUE(gnss_sim::format_unicore_nav_output_record(record, output_time(), &message, &supported, &error_message))
        << error_message;
    ASSERT_TRUE(supported);
    EXPECT_EQ(message, "#BD3IONA,0,GPS,FINE,2041,180000000,0,0,18,0;"
                       "0.000000000000000e+00,0.000000000000000e+00,0.000000000000000e+00,"
                       "0.000000000000000e+00,0.000000000000000e+00,0.000000000000000e+00,"
                       "0.000000000000000e+00,0.000000000000000e+00,0.000000000000000e+00,0*bdf5809e\r\n");
}

TEST(NavOutputWriter, GalileoHealthBitsAreDecodedFromRealRinexBeforeSerialization) {
    gnss_sim::RtklibNavStore* store = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(store, nullptr);
    std::string error_message;
    ASSERT_TRUE(
        gnss_sim::load_rinex_nav_file(store, data_path("brd400dlr_rinex4_acceptance_nav.rnx").c_str(), &error_message))
        << error_message;

    int galileo_records = 0;
    int nonzero_health_records = 0;
    const int count = gnss_sim::rtklib_nav_output_record_count(store);
    for (int index = 0; index < count; ++index) {
        gnss_sim::NavOutputRecord record{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(store, index, &record, &error_message)) << error_message;
        if (record.kind != gnss_sim::RtklibNavRecordKind::kEphemeris ||
            record.ephemeris.system != gnss_sim::NavOutputSystem::kGalileo) {
            continue;
        }
        ++galileo_records;
        const int svh = record.ephemeris.svh;
        if (svh != 0) {
            ++nonzero_health_records;
        }
        ASSERT_TRUE(gnss_sim::finalize_nav_output_record_metadata(&record));
        EXPECT_EQ(record.ephemeris.galileo_e1b_dvs, (svh >> 0) & 0x1);
        EXPECT_EQ(record.ephemeris.galileo_e1b_health, (svh >> 1) & 0x3);
        EXPECT_EQ(record.ephemeris.galileo_e5a_dvs, (svh >> 3) & 0x1);
        EXPECT_EQ(record.ephemeris.galileo_e5a_health, (svh >> 4) & 0x3);
        EXPECT_EQ(record.ephemeris.galileo_e5b_dvs, (svh >> 6) & 0x1);
        EXPECT_EQ(record.ephemeris.galileo_e5b_health, (svh >> 7) & 0x3);
    }

    EXPECT_GT(galileo_records, 0);
    EXPECT_GT(nonzero_health_records, 0)
        << "real BRD400 fixture must retain at least one nonzero Galileo health-status record";
    gnss_sim::destroy_rtklib_nav_store(store);
}

TEST(NavOutputWriter, RealGalileoFnavWithInavCompanionDoesNotEmitAmbiguousOrbit) {
    gnss_sim::RtklibNavStore* store = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(store, nullptr);
    std::string error_message;
    ASSERT_TRUE(
        gnss_sim::load_rinex_nav_file(store, data_path("brd400dlr_rinex4_acceptance_nav.rnx").c_str(), &error_message))
        << error_message;

    // Loading applies RTKLIB uniqnav(), so each satellite keeps only its latest record
    // and no real fixture presents an FNAV record whose store also holds an INAV
    // companion. Prove both real cases stay serializable: INAV-source records (E02, E05,
    // ...) and FNAV-only records (E23, and E25 whose latest record is FNAV).
    gnss_sim::NavOutputRecord real_fnav_record{};
    int inav_emitted = 0;
    int fnav_only_emitted = 0;
    const int count = gnss_sim::rtklib_nav_output_record_count(store);
    for (int index = 0; index < count; ++index) {
        gnss_sim::NavOutputRecord record{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(store, index, &record, &error_message)) << error_message;
        if (record.kind != gnss_sim::RtklibNavRecordKind::kEphemeris ||
            record.ephemeris.system != gnss_sim::NavOutputSystem::kGalileo) {
            continue;
        }

        const gnss_sim::SimTime time = output_time();
        std::string message;
        bool supported = false;
        ASSERT_TRUE(gnss_sim::format_novatel_nav_output_record(record, time, &message, &supported, &error_message))
            << error_message;
        if (record.ephemeris.message_family == gnss_sim::RtklibBroadcastMessageFamily::kGalileoInav) {
            ++inav_emitted;
            EXPECT_TRUE(supported);
            EXPECT_TRUE(valid_ascii_crc(message));
        } else {
            ++fnav_only_emitted;
            EXPECT_FALSE(record.ephemeris.galileo_inav_received)
                << "real store cannot hold an FNAV record with an INAV companion";
            EXPECT_TRUE(supported);
            EXPECT_TRUE(valid_ascii_crc(message));
            real_fnav_record = record;
        }
    }

    EXPECT_GT(inav_emitted, 0) << "real BRD400DLR fixture must contain Galileo INAV records";
    EXPECT_GT(fnav_only_emitted, 0) << "real BRD400DLR fixture must contain Galileo FNAV records";

    // Exercise the ambiguous store state the writer itself can produce when a store does
    // hold both families: copy the real FNAV record values unchanged and set only the
    // companion flag the adapter fills when an INAV ephemeris exists for the satellite.
    // The FNAV orbit must not masquerade as an INAV ephemeris through the common orbit
    // block, so the writer must reject it.
    ASSERT_TRUE(gnss_sim::finalize_nav_output_record_metadata(&real_fnav_record));
    real_fnav_record.ephemeris.galileo_inav_received = true;
    const gnss_sim::SimTime time = output_time();
    std::string message;
    bool supported = false;
    ASSERT_TRUE(
        gnss_sim::format_novatel_nav_output_record(real_fnav_record, time, &message, &supported, &error_message))
        << error_message;
    EXPECT_FALSE(supported) << "FNAV orbit with an INAV companion must not masquerade as an INAV ephemeris";
    EXPECT_TRUE(message.empty());
    gnss_sim::destroy_rtklib_nav_store(store);
}

TEST(NavOutputWriter, RealGalileoInavSerializedRecordRoundTripsPositionAndClock) {
    gnss_sim::RtklibNavStore* store = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(store, nullptr);
    std::string error_message;
    ASSERT_TRUE(
        gnss_sim::load_rinex_nav_file(store, data_path("brd400dlr_rinex4_acceptance_nav.rnx").c_str(), &error_message))
        << error_message;

    bool verified_inav_record = false;
    const int count = gnss_sim::rtklib_nav_output_record_count(store);
    for (int index = 0; index < count && !verified_inav_record; ++index) {
        gnss_sim::NavOutputRecord record{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(store, index, &record, &error_message)) << error_message;
        if (record.kind != gnss_sim::RtklibNavRecordKind::kEphemeris ||
            record.ephemeris.system != gnss_sim::NavOutputSystem::kGalileo ||
            record.ephemeris.message_family != gnss_sim::RtklibBroadcastMessageFamily::kGalileoInav) {
            continue;
        }

        const gnss_sim::SimTime time = output_time();
        std::string message;
        bool supported = false;
        ASSERT_TRUE(gnss_sim::format_novatel_nav_output_record(record, time, &message, &supported, &error_message))
            << error_message;
        ASSERT_TRUE(supported);
        ASSERT_TRUE(valid_ascii_crc(message));

        const std::vector<std::string> fields = split_body_fields(body_between_semicolon_and_crc(message));
        ASSERT_EQ(fields.size(), 38u) << "GALEPHEMERISA body layout changed";
        EXPECT_EQ(fields[2], "TRUE") << "serialized INAV-source record must flag INAV received";

        const eph_t serialized =
            ephemeris_from_galileo_body_fields(fields, record.ephemeris.toe_week, record.ephemeris.satellite_number);
        const eph_t source = ephemeris_from_keplerian_record(record.ephemeris);
        const gtime_t eval_time = timeadd(source.toe, 3600.0);
        double source_position[6] = {0};
        double serialized_position[6] = {0};
        double source_clock[2] = {0};
        double serialized_clock[2] = {0};
        double source_variance = 0.0;
        double serialized_variance = 0.0;
        eph2pos(eval_time, &source, source_position, source_clock, &source_variance);
        eph2pos(eval_time, &serialized, serialized_position, serialized_clock, &serialized_variance);
        for (int axis = 0; axis < 3; ++axis) {
            EXPECT_LT(std::fabs(source_position[axis] - serialized_position[axis]), 1e-6)
                << "serialized INAV orbit must round-trip to the same satellite position";
        }
        EXPECT_LT(std::fabs((source_clock[0] - serialized_clock[0]) * CLIGHT), 1e-6)
            << "serialized INAV clock must round-trip to the same satellite clock";
        verified_inav_record = true;
    }

    EXPECT_TRUE(verified_inav_record) << "real BRD400DLR fixture must contain a serializable Galileo INAV record";
    gnss_sim::destroy_rtklib_nav_store(store);
}

} // namespace
