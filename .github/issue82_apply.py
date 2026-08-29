from pathlib import Path

writer = Path('src/output/novatel_nav_writer.cpp')
text = writer.read_text()
old_gps = '''            case NavOutputSystem::kGps:\n                log_name = "GPSEPHEMA";\n                body = generic_kepler_body(eph, false, false);\n                break;'''
new_gps = '''            case NavOutputSystem::kGps:\n                if (eph.message_family == RtklibBroadcastMessageFamily::kLegacy) {\n                    log_name = "GPSEPHEMA";\n                    body = generic_kepler_body(eph, false, false);\n                }\n                break;'''
old_qzss = '''            case NavOutputSystem::kQzss:\n                log_name = "QZSSEPHEMERISA";\n                body = generic_kepler_body(eph, true, false);\n                break;'''
new_qzss = '''            case NavOutputSystem::kQzss:\n                if (eph.message_family == RtklibBroadcastMessageFamily::kLegacy) {\n                    log_name = "QZSSEPHEMERISA";\n                    body = generic_kepler_body(eph, true, false);\n                }\n                break;'''
assert old_gps in text
assert old_qzss in text
text = text.replace(old_gps, new_gps).replace(old_qzss, new_qzss)
writer.write_text(text)

test = Path('tests/unit/test_nav_output_writers.cpp')
text = test.read_text()
marker = '\nTEST(NavOutputWriter, Bd3IonHasByteLevelGoldenRecord) {'
assert marker in text
insert = r'''

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
'''
text = text.replace(marker, insert + marker)
test.write_text(text)
