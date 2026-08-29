from pathlib import Path

path = Path('src/gnss/rtklib_nav_output_adapter.cpp')
text = path.read_text(encoding='utf-8')
anchor = '''bool fill_explicit_ion(const nav_t& nav, int index, NavOutputRecord* record) {
'''
helper = '''bool legacy_beidou_ion_message(int message_type) {
    return message_type == NAV_D1 || message_type == NAV_D2 || message_type == NAV_D1D2;
}

'''
if helper.strip() not in text:
    if anchor not in text:
        raise SystemExit('fill_explicit_ion anchor not found')
    text = text.replace(anchor, helper + anchor, 1)
old = '''    output.leap_seconds = nav.leaps;
    output.legacy_metadata = false;
    record->kind = RtklibNavRecordKind::kIonosphere;
'''
new = '''    output.leap_seconds = nav.leaps;
    output.legacy_metadata =
        ion.hdr.sys == SYS_CMP && legacy_beidou_ion_message(ion.hdr.msg_type);
    record->kind = RtklibNavRecordKind::kIonosphere;
'''
if new not in text:
    if old not in text:
        raise SystemExit('legacy_metadata assignment not found')
    text = text.replace(old, new, 1)
path.write_text(text, encoding='utf-8')

path = Path('tests/unit/test_nav_output_writers.cpp')
text = path.read_text(encoding='utf-8')
name = 'RealRinex4BeiDouLegacyIonSerializesWithoutModernFamilyMasquerade'
if name in text:
    raise SystemExit('issue85 test already present')
anchor = 'TEST(NavOutputWriter, Bd3IonHasByteLevelGoldenRecord) {'
if anchor not in text:
    raise SystemExit('unit test insertion anchor not found')
block = r'''TEST(NavOutputWriter, RealRinex4BeiDouLegacyIonSerializesWithoutModernFamilyMasquerade) {
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

'''
text = text.replace(anchor, block + anchor, 1)
path.write_text(text, encoding='utf-8')
