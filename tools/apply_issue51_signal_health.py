from pathlib import Path


def replace_once(path, old, new):
    p = Path(path)
    text = p.read_text()
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one guarded match, found {count}")
    p.write_text(text.replace(old, new, 1))


replace_once(
    "src/gnss/rtklib_adapter.h",
    '''bool rtklib_observation_code(const char* rinex_signal_code, int* observation_code, int* frequency_index);\nbool get_rtklib_satellite_state''',
    '''bool rtklib_observation_code(const char* rinex_signal_code, int* observation_code, int* frequency_index);\nbool rtklib_signal_health_for_family(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,\n                                     const char* rinex_signal_code,\n                                     RtklibBroadcastMessageFamily requested_message_family, int* signal_health,\n                                     std::string* error_message);\nbool get_rtklib_satellite_state''',
)

replace_once(
    "src/gnss/rtklib_bias_adapter.cpp",
    '''extern "C" {\n#include <rtklib.h>\n}''',
    '''extern "C" {\n#include <rtklib.h>\n#include <rtklib_signal_bias_ext.h>\n}''',
)

replace_once(
    "src/gnss/rtklib_bias_adapter.cpp",
    '''bool rtklib_broadcast_bias_data(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,\n                                RtklibBroadcastBiasData* data, std::string* error_message) {\n    return rtklib_broadcast_bias_data_for_family(store, gps_week, sow_sec, satellite_number,\n                                                 RtklibBroadcastMessageFamily::kUnknown, data, error_message);\n}\n''',
    '''bool rtklib_broadcast_bias_data(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,\n                                RtklibBroadcastBiasData* data, std::string* error_message) {\n    return rtklib_broadcast_bias_data_for_family(store, gps_week, sow_sec, satellite_number,\n                                                 RtklibBroadcastMessageFamily::kUnknown, data, error_message);\n}\n\nbool rtklib_signal_health_for_family(const RtklibNavStore* store, int gps_week, double sow_sec, int satellite_number,\n                                     const char* rinex_signal_code,\n                                     RtklibBroadcastMessageFamily requested_message_family, int* signal_health,\n                                     std::string* error_message) {\n    if (store == nullptr || rinex_signal_code == nullptr || rinex_signal_code[0] == '\\0' || signal_health == nullptr ||\n        satellite_number <= 0 || !valid_gps_time(gps_week, sow_sec)) {\n        set_error(error_message, "signal-health request has invalid arguments");\n        return false;\n    }\n\n    int observation_code = 0;\n    int frequency_index = 0;\n    if (!rtklib_observation_code(rinex_signal_code, &observation_code, &frequency_index)) {\n        set_error(error_message, "signal-health request has unsupported observation code");\n        return false;\n    }\n    static_cast<void>(frequency_index);\n\n    int required_message_mask = 0;\n    switch (requested_message_family) {\n        case RtklibBroadcastMessageFamily::kCnav:\n            required_message_mask = NAV_CNAV;\n            break;\n        case RtklibBroadcastMessageFamily::kCnav2:\n            required_message_mask = NAV_CNV2;\n            break;\n        case RtklibBroadcastMessageFamily::kGalileoInav:\n            required_message_mask = NAV_INAV;\n            break;\n        case RtklibBroadcastMessageFamily::kGalileoFnav:\n            required_message_mask = NAV_FNAV;\n            break;\n        case RtklibBroadcastMessageFamily::kBeidouBcnav1:\n            required_message_mask = NAV_CNV1;\n            break;\n        case RtklibBroadcastMessageFamily::kBeidouBcnav2:\n            required_message_mask = NAV_CNV2;\n            break;\n        case RtklibBroadcastMessageFamily::kBeidouBcnav3:\n            required_message_mask = NAV_CNV3;\n            break;\n        case RtklibBroadcastMessageFamily::kGlonassFdma:\n            required_message_mask = NAV_FDMA;\n            break;\n        case RtklibBroadcastMessageFamily::kGlonassL3Oc:\n            required_message_mask = NAV_L3OC;\n            break;\n        case RtklibBroadcastMessageFamily::kLegacy:\n        case RtklibBroadcastMessageFamily::kUnknown:\n            required_message_mask = 0;\n            break;\n    }\n\n    const gtime_t time = gpst2time(gps_week, sow_sec);\n    eph_t eph{};\n    geph_t geph{};\n    rtklib_signal_bias_info_ext_t info{};\n    const int status = rtklib_signal_ephemeris_ext(\n        time, satellite_number, static_cast<unsigned char>(observation_code), required_message_mask, &store->nav, &eph,\n        &geph, &info);\n    if (status <= 0) {\n        set_error(error_message, "no matching signal/message-family ephemeris for health");\n        return false;\n    }\n\n    const int raw_health = info.system == SYS_GLO ? geph.svh : eph.svh;\n    *signal_health = rtklib_signal_health_ext(info.system, info.message_type,\n                                              static_cast<unsigned char>(observation_code), raw_health);\n    return true;\n}\n''',
)

replace_once(
    "src/core/simulator.cpp",
    '''        bool satellite_tracking = false;\n        for (SignalRuntime& signal : satellite.signals) {\n            const bool signal_available = scenario.signal_available && geometry.visible;\n            if (signal_available && !signal.tracker.scheduled) {''',
    '''        bool satellite_tracking = false;\n        for (SignalRuntime& signal : satellite.signals) {\n            const SignalDefinition* definition = find_signal_definition(signal.tracker.signal_id);\n            if (definition == nullptr) {\n                set_error(error_message, "signal definition is missing during tracking update");\n                return false;\n            }\n\n            bool signal_healthy = geometry.healthy;\n            RtklibBroadcastMessageFamily health_family = RtklibBroadcastMessageFamily::kUnknown;\n            if (definition->nav_message_family == NavMessageFamily::kGpsCnav) {\n                health_family = RtklibBroadcastMessageFamily::kCnav;\n            } else if (definition->nav_message_family == NavMessageFamily::kGpsCnav2) {\n                health_family = RtklibBroadcastMessageFamily::kCnav2;\n            }\n            if (health_family != RtklibBroadcastMessageFamily::kUnknown) {\n                int signal_health = 0;\n                if (!rtklib_signal_health_for_family(truth_nav, geometry.transmit_gps_week, geometry.transmit_sow_sec,\n                                                     satellite.satellite_number, definition->rinex_signal_code,\n                                                     health_family, &signal_health, error_message)) {\n                    return false;\n                }\n                signal_healthy = signal_health == 0;\n            }\n\n            const bool signal_available =\n                scenario.signal_available && geometry.above_elevation_mask && signal_healthy;\n            if (signal_available && !signal.tracker.scheduled) {''',
)

replace_once(
    "tests/unit/test_rtklib_adapter.cpp",
    '''TEST(RtklibAdapterCoordinates, LlhEcefRoundTripUsesRtklibReferenceFunctions) {''',
    '''TEST_F(RtklibAdapterTest, GpsCnavHealthIsInterpretedPerSignal) {\n    std::string error_message;\n    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(store_, rinex4_acceptance_nav_path().c_str(), &error_message))\n        << error_message;\n\n    int g17 = 0;\n    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G17", &g17));\n\n    int l2c_health = -1;\n    ASSERT_TRUE(gnss_sim::rtklib_signal_health_for_family(\n        store_, 2347, 437100.0, g17, "2S", gnss_sim::RtklibBroadcastMessageFamily::kCnav, &l2c_health,\n        &error_message))\n        << error_message;\n    EXPECT_EQ(l2c_health, 0);\n\n    int l5q_health = -1;\n    ASSERT_TRUE(gnss_sim::rtklib_signal_health_for_family(\n        store_, 2347, 437100.0, g17, "5Q", gnss_sim::RtklibBroadcastMessageFamily::kCnav, &l5q_health,\n        &error_message))\n        << error_message;\n    EXPECT_NE(l5q_health, 0);\n}\n\nTEST(RtklibAdapterCoordinates, LlhEcefRoundTripUsesRtklibReferenceFunctions) {''',
)

print("GPS modern per-signal health gating patch applied")
