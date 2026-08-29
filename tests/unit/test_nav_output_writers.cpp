#include "gnss/nav_output_record.h"
#include "gnss/navigation_state.h"
#include "gnss/rtklib_adapter.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_time.h"
#include "output/novatel_ascii.h"
#include "output/novatel_nav_writer.h"
#include "output/unicore_nav_writer.h"

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

} // namespace
