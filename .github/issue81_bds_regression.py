from pathlib import Path

path = Path('tests/integration/test_rangea_roundtrip.cpp')
text = path.read_text()
include_anchor = '#include "gnss_sim/simulator.h"\n#include "rangea_roundtrip.h"\n'
include_replacement = '''#include "gnss_sim/simulator.h"
#include "gnss/nav_output_record.h"
#include "gnss/rtklib_adapter.h"
#include "output/novatel_nav_writer.h"
#include "rangea_roundtrip.h"
#include "serialized_nav_parser.h"
'''
assert include_anchor in text
text = text.replace(include_anchor, include_replacement, 1)

marker = '\n} // namespace\n'
assert text.endswith(marker)
test = r'''

TEST(RangeaRoundtripIntegration, RealBeidouLegacyAsciiRestoresRtklibBroadcastState) {
    gnss_sim::RtklibNavStore* full = gnss_sim::create_rtklib_nav_store();
    gnss_sim::RtklibNavStore* source_record = gnss_sim::create_rtklib_nav_store();
    gnss_sim::RtklibNavStore* rebuilt = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(full, nullptr);
    ASSERT_NE(source_record, nullptr);
    ASSERT_NE(rebuilt, nullptr);

    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(full, brd4_nav_path().c_str(), &error_message)) << error_message;

    bool compared = false;
    const int output_count = gnss_sim::rtklib_nav_output_record_count(full);
    for (int index = 0; index < output_count; ++index) {
        gnss_sim::NavOutputRecord source{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(full, index, &source, &error_message)) << error_message;
        if (source.kind != gnss_sim::RtklibNavRecordKind::kEphemeris ||
            source.ephemeris.system != gnss_sim::NavOutputSystem::kBeidou ||
            source.ephemeris.message_family != gnss_sim::RtklibBroadcastMessageFamily::kLegacy) {
            continue;
        }

        ASSERT_TRUE(gnss_sim::rtklib_copy_nav_record(full, index, source_record, &error_message)) << error_message;
        gnss_sim::SimTime output_time{};
        ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(source.ephemeris.transmit_week,
                                                     source.ephemeris.transmit_sow_sec, &output_time));

        std::string message;
        bool supported = false;
        ASSERT_TRUE(gnss_sim::format_novatel_nav_output_record(source, output_time, &message, &supported, &error_message))
            << error_message;
        ASSERT_TRUE(supported);
        ASSERT_EQ(message.rfind("#BD2EPHEMA,", 0), 0U);

        gnss_sim::ParsedSerializedNavRecord parsed{};
        bool recognized = false;
        ASSERT_TRUE(gnss_sim::parse_serialized_novatel_nav_line_independent(message, &parsed, &recognized,
                                                                            &error_message))
            << error_message;
        ASSERT_TRUE(recognized);
        ASSERT_TRUE(gnss_sim::rtklib_append_nav_output_record(rebuilt, parsed.record, &error_message)) << error_message;

        int observation_code = 0;
        int frequency_index = 0;
        ASSERT_TRUE(gnss_sim::rtklib_observation_code("2I", &observation_code, &frequency_index));
        static_cast<void>(frequency_index);

        int state_week = parsed.output_gps_week;
        double state_sow = parsed.output_sow_sec + 60.0;
        if (state_sow >= 604800.0) {
            state_sow -= 604800.0;
            ++state_week;
        }
        gnss_sim::RtklibSatelliteState expected{};
        gnss_sim::RtklibSatelliteState actual{};
        ASSERT_TRUE(gnss_sim::get_rtklib_signal_satellite_state(
            source_record, state_week, state_sow, source.ephemeris.satellite_number, observation_code,
            gnss_sim::RtklibBroadcastMessageFamily::kLegacy, &expected, &error_message))
            << error_message;
        ASSERT_TRUE(gnss_sim::get_rtklib_signal_satellite_state(
            rebuilt, state_week, state_sow, source.ephemeris.satellite_number, observation_code,
            gnss_sim::RtklibBroadcastMessageFamily::kLegacy, &actual, &error_message))
            << error_message;

        const double dx = expected.position_ecef_m[0] - actual.position_ecef_m[0];
        const double dy = expected.position_ecef_m[1] - actual.position_ecef_m[1];
        const double dz = expected.position_ecef_m[2] - actual.position_ecef_m[2];
        const double position_difference_m = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double clock_difference_m =
            std::abs(expected.clock_bias_sec - actual.clock_bias_sec) * 299792458.0;
        EXPECT_LT(position_difference_m, 1.0e-3);
        EXPECT_LT(clock_difference_m, 1.0e-6);
        EXPECT_EQ(expected.health, actual.health);
        compared = true;
        break;
    }

    EXPECT_TRUE(compared) << "real BRD400 fixture must contain a NovAtel-serializable BeiDou legacy ephemeris";
    gnss_sim::destroy_rtklib_nav_store(rebuilt);
    gnss_sim::destroy_rtklib_nav_store(source_record);
    gnss_sim::destroy_rtklib_nav_store(full);
}
'''
text = text[:-len(marker)] + test + marker
path.write_text(text)
