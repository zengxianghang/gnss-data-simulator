#include "gnss/nav_output_record.h"
#include "gnss/navigation_state.h"
#include "gnss/rtklib_adapter.h"
#include "gnss/signal_definitions.h"
#include "gnss_sim/sim_time.h"
#include "output/novatel_ascii.h"
#include "output/novatel_nav_writer.h"
#include "output/unicore_nav_writer.h"

#include <gtest/gtest.h>

#include <cstdint>
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
    if (star == std::string::npos || star + 10 != message.size()) {
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
        const bool first_ok = unicore ? gnss_sim::format_unicore_receiver_nav_record(
                                           store, index, output_time(), &first, &first_supported, &error_message)
                                     : gnss_sim::format_novatel_receiver_nav_record(
                                           store, index, output_time(), &first, &first_supported, &error_message);
        ASSERT_TRUE(first_ok) << error_message;
        const bool second_ok = unicore ? gnss_sim::format_unicore_receiver_nav_record(
                                            store, index, output_time(), &second, &second_supported, &error_message)
                                      : gnss_sim::format_novatel_receiver_nav_record(
                                            store, index, output_time(), &second, &second_supported, &error_message);
        ASSERT_TRUE(second_ok) << error_message;
        EXPECT_EQ(first_supported, second_supported);
        EXPECT_EQ(first, second);
        if (first_supported) {
            EXPECT_TRUE(valid_ascii_crc(first));
            names->insert(log_name(first));
        }
    }
}

gnss_sim::NavOutputRecord synthetic_ephemeris(gnss_sim::NavOutputSystem system,
                                                gnss_sim::RtklibBroadcastMessageFamily family) {
    gnss_sim::NavOutputRecord record{};
    record.kind = gnss_sim::RtklibNavRecordKind::kEphemeris;
    record.ephemeris.system = system;
    record.ephemeris.message_family = family;
    record.ephemeris.prn = 2;
    record.ephemeris.iode = 3;
    record.ephemeris.iodc = 4;
    record.ephemeris.sva = 1.0;
    record.ephemeris.toe_week = 2041;
    record.ephemeris.toc_week = 2041;
    record.ephemeris.transmit_week = 2041;
    record.ephemeris.toe_sow_sec = 180000.0;
    record.ephemeris.toc_sow_sec = 180000.0;
    record.ephemeris.transmit_sow_sec = 180006.0;
    record.ephemeris.semi_major_axis_m = 26560000.0;
    record.ephemeris.eccentricity = 0.01;
    record.ephemeris.inclination_rad = 0.95;
    record.ephemeris.omega0_rad = 1.0;
    record.ephemeris.argument_of_perigee_rad = 0.2;
    record.ephemeris.mean_anomaly_rad = 0.3;
    record.ephemeris.delta_mean_motion_radps = 1.0e-9;
    record.ephemeris.omega_dot_radps = -8.0e-9;
    record.ephemeris.inclination_dot_radps = 1.0e-10;
    record.ephemeris.clock_bias_sec = 1.0e-4;
    record.ephemeris.clock_drift_sec_per_sec = 2.0e-12;
    record.ephemeris.tgd_sec[0] = 1.0e-9;
    record.ephemeris.tgd_sec[1] = 2.0e-9;
    record.ephemeris.galileo_fnav_received = true;
    record.ephemeris.galileo_inav_received = true;
    record.ephemeris.galileo_fnav_toc_sow_sec = 180000.0;
    record.ephemeris.galileo_inav_toc_sow_sec = 180000.0;
    return record;
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
                                                             event.receiver_record_index, available, &message, &supported,
                                                             &error_message))
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

TEST(NavOutputWriter, SyntheticModernBdsAndNavicStayWithinFrozenOutputScope) {
    std::string message;
    std::string error_message;
    bool supported = false;

    gnss_sim::NavOutputRecord bds = synthetic_ephemeris(gnss_sim::NavOutputSystem::kBeidou,
                                                         gnss_sim::RtklibBroadcastMessageFamily::kBeidouBcnav1);
    ASSERT_TRUE(gnss_sim::format_unicore_nav_output_record(bds, output_time(), &message, &supported, &error_message))
        << error_message;
    EXPECT_TRUE(supported);
    EXPECT_EQ(log_name(message), "BD3EPHA");
    EXPECT_TRUE(valid_ascii_crc(message));
    ASSERT_TRUE(gnss_sim::format_novatel_nav_output_record(bds, output_time(), &message, &supported, &error_message))
        << error_message;
    EXPECT_FALSE(supported);
    EXPECT_TRUE(message.empty());

    gnss_sim::NavOutputRecord navic = synthetic_ephemeris(gnss_sim::NavOutputSystem::kNavic,
                                                           gnss_sim::RtklibBroadcastMessageFamily::kLegacy);
    ASSERT_TRUE(gnss_sim::format_unicore_nav_output_record(navic, output_time(), &message, &supported, &error_message))
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
    EXPECT_EQ(message,
              "#BD3IONA,0,GPS,FINE,2041,180000000,0,0,18,0;"
              "0.000000000000000e+00,0.000000000000000e+00,0.000000000000000e+00,"
              "0.000000000000000e+00,0.000000000000000e+00,0.000000000000000e+00,"
              "0.000000000000000e+00,0.000000000000000e+00,0.000000000000000e+00,0*bdf5809e\r\n");
}

TEST(NavOutputWriter, GalileoHealthBitsAreDecodedBeforeSerialization) {
    gnss_sim::NavOutputRecord record =
        synthetic_ephemeris(gnss_sim::NavOutputSystem::kGalileo, gnss_sim::RtklibBroadcastMessageFamily::kGalileoInav);
    record.ephemeris.svh = (1 << 0) | (2 << 1) | (1 << 3) | (3 << 4) | (0 << 6) | (1 << 7);
    ASSERT_TRUE(gnss_sim::finalize_nav_output_record_metadata(&record));
    EXPECT_EQ(record.ephemeris.galileo_e1b_dvs, 1);
    EXPECT_EQ(record.ephemeris.galileo_e1b_health, 2);
    EXPECT_EQ(record.ephemeris.galileo_e5a_dvs, 1);
    EXPECT_EQ(record.ephemeris.galileo_e5a_health, 3);
    EXPECT_EQ(record.ephemeris.galileo_e5b_dvs, 0);
    EXPECT_EQ(record.ephemeris.galileo_e5b_health, 1);
}

} // namespace
