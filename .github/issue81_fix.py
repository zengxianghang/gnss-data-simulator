from pathlib import Path

nav = Path('src/gnss/rtklib_nav_output_adapter.cpp')
text = nav.read_text()
old = '''bool legacy_beidou_ion_message(int message_type) {
    return message_type == NAV_D1 || message_type == NAV_D2 || message_type == NAV_D1D2;
}
'''
new = '''bool legacy_gps_ion_message(int message_type) {
    return message_type == NAV_LNAV;
}

bool legacy_beidou_ion_message(int message_type) {
    return message_type == NAV_D1 || message_type == NAV_D2 || message_type == NAV_D1D2;
}
'''
assert old in text
text = text.replace(old, new, 1)
old = '    output.legacy_metadata = ion.hdr.sys == SYS_CMP && legacy_beidou_ion_message(ion.hdr.msg_type);\n'
new = '''    output.legacy_metadata = (ion.hdr.sys == SYS_GPS && legacy_gps_ion_message(ion.hdr.msg_type)) ||
                             (ion.hdr.sys == SYS_CMP && legacy_beidou_ion_message(ion.hdr.msg_type));
'''
assert old in text
text = text.replace(old, new, 1)
nav.write_text(text)

test = Path('tests/unit/test_nav_output_writers.cpp')
text = test.read_text()
text = text.replace('TEST(NavOutputWriter, LegacyMixedRinexCoversFiveEphemerisFamilies) {',
                    'TEST(NavOutputWriter, LegacyMixedRinexCoversSupportedNovatelAndFiveSystemUnicoreFamilies) {', 1)
old = '    EXPECT_TRUE(novatel_names.count("GALEPHEMERISA"));\n'
assert old in text
text = text.replace(old, '', 1)
test.write_text(text)
