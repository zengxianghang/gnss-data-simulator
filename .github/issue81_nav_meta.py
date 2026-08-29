from pathlib import Path

parser = Path('tools/rangea_roundtrip/serialized_nav_parser.cpp')
text = parser.read_text()
old = '    ion.leap_seconds = leap_a;\n'
new = ('    // BD2IONUTCA carries BDT-UTC, while RTKLIB nav.leaps is GPST-UTC.\n'
       '    ion.leap_seconds = name == "BD2IONUTCA" ? leap_a + 14 : leap_a;\n')
assert old in text
parser.write_text(text.replace(old, new, 1))

header = Path('tools/rangea_roundtrip/rangea_roundtrip.h')
text = header.read_text()
old = ('    std::uint64_t parsed_ionosphere_records;\n'
       '    std::uint64_t gps_ephemeris_records;\n')
new = ('    std::uint64_t parsed_ionosphere_records;\n'
       '    std::uint64_t gps_ionosphere_records;\n'
       '    std::uint64_t skipped_position_epochs_without_ionosphere;\n'
       '    std::uint64_t gps_ephemeris_records;\n')
assert old in text
header.write_text(text.replace(old, new, 1))

core = Path('tools/rangea_roundtrip/rangea_roundtrip.cpp')
text = core.read_text()
old = '    SerializedNavRoundtripSummary result{};\n    std::string line;\n'
new = ('    SerializedNavRoundtripSummary result{};\n'
       '    bool gps_broadcast_ion_available = false;\n'
       '    std::string line;\n')
assert old in text
text = text.replace(old, new, 1)
old = ('            if (parsed_nav.record.kind == RtklibNavRecordKind::kIonosphere) {\n'
       '                ++result.parsed_ionosphere_records;\n'
       '            } else {\n')
new = ('            if (parsed_nav.record.kind == RtklibNavRecordKind::kIonosphere) {\n'
       '                ++result.parsed_ionosphere_records;\n'
       '                if (parsed_nav.record.ionosphere.system == NavOutputSystem::kGps) {\n'
       '                    ++result.gps_ionosphere_records;\n'
       '                    gps_broadcast_ion_available = true;\n'
       '                }\n'
       '            } else {\n')
assert old in text
text = text.replace(old, new, 1)
old = ('        result.position.selected_position_observations += static_cast<std::uint64_t>(selected.size());\n'
       '        std::vector<RtklibRawCodeObservation> usable;\n')
new = ('        result.position.selected_position_observations += static_cast<std::uint64_t>(selected.size());\n'
       '        if (broadcast_atmosphere && !gps_broadcast_ion_available) {\n'
       '            ++result.skipped_position_epochs_without_ionosphere;\n'
       '            continue;\n'
       '        }\n'
       '        std::vector<RtklibRawCodeObservation> usable;\n')
assert old in text
text = text.replace(old, new, 1)
old = ('    if (result.parsed_ephemeris_records == 0U || result.parsed_ionosphere_records == 0U) {\n'
       '        set_error(error_message, "serialized receiver log contains no usable EPHA/IONA navigation set");\n'
       '        return false;\n'
       '    }\n')
new = ('    if (result.parsed_ephemeris_records == 0U) {\n'
       '        set_error(error_message, "serialized receiver log contains no usable EPHA navigation set");\n'
       '        return false;\n'
       '    }\n'
       '    if (broadcast_atmosphere && result.gps_ionosphere_records == 0U) {\n'
       '        set_error(error_message,\n'
       '                  "serialized receiver log contains no usable GPS IONUTCA for broadcast atmosphere");\n'
       '        return false;\n'
       '    }\n')
assert old in text
core.write_text(text.replace(old, new, 1))

test = Path('tests/integration/test_rangea_roundtrip.cpp')
text = test.read_text()
assert 'RealBeidouLegacyIonRestoresRtklibGpsUtcLeapMetadata' not in text
marker = '\n} // namespace\n'
assert text.endswith(marker)
addition = r'''

TEST(RangeaRoundtripIntegration, BroadcastAtmosphereRequiresSerializedGpsIonBeforePositioning) {
    const std::filesystem::path directory = "gnss_sim_serialized_nav_no_gps_ion";
    gnss_sim::SimulatorRunSummary simulator_summary{};
    std::string error_message;
    ASSERT_TRUE(run_simulator(directory, &simulator_summary, &error_message)) << error_message;

    std::istringstream input(read_file(directory / "simulated.log"));
    std::ostringstream stripped;
    std::string line;
    while (std::getline(input, line)) {
        if (line.rfind("#IONUTCA,", 0) != 0) {
            stripped << line << '\n';
        }
    }

    std::istringstream validation_input(stripped.str());
    gnss_sim::SerializedNavRoundtripSummary summary{};
    error_message.clear();
    EXPECT_FALSE(gnss_sim::validate_serialized_navigation_roundtrip_stream(
        &validation_input, 20.0, 120.0, 100.0, 5.0, true, &summary, &error_message));
    EXPECT_NE(error_message.find("GPS IONUTCA"), std::string::npos)
        << "serialized-NAV validation must not use RTKLIB's built-in default Klobuchar coefficients";

    cleanup(directory);
}

TEST(RangeaRoundtripIntegration, RealBeidouLegacyIonRestoresRtklibGpsUtcLeapMetadata) {
    gnss_sim::RtklibNavStore* full = gnss_sim::create_rtklib_nav_store();
    gnss_sim::RtklibNavStore* rebuilt = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(full, nullptr);
    ASSERT_NE(rebuilt, nullptr);

    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(full, brd4_nav_path().c_str(), &error_message)) << error_message;

    bool compared = false;
    const int output_count = gnss_sim::rtklib_nav_output_record_count(full);
    for (int index = 0; index < output_count; ++index) {
        gnss_sim::NavOutputRecord source{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(full, index, &source, &error_message)) << error_message;
        if (source.kind != gnss_sim::RtklibNavRecordKind::kIonosphere ||
            source.ionosphere.system != gnss_sim::NavOutputSystem::kBeidou || !source.ionosphere.legacy_metadata) {
            continue;
        }

        std::string message;
        bool supported = false;
        ASSERT_TRUE(gnss_sim::format_novatel_nav_output_record(source, start_time(), &message, &supported, &error_message))
            << error_message;
        ASSERT_TRUE(supported);
        ASSERT_EQ(message.rfind("#BD2IONUTCA,", 0), 0U);

        gnss_sim::ParsedSerializedNavRecord parsed{};
        bool recognized = false;
        ASSERT_TRUE(
            gnss_sim::parse_serialized_novatel_nav_line_independent(message, &parsed, &recognized, &error_message))
            << error_message;
        ASSERT_TRUE(recognized);
        EXPECT_EQ(parsed.record.ionosphere.leap_seconds, source.ionosphere.leap_seconds);
        ASSERT_TRUE(gnss_sim::rtklib_append_nav_output_record(rebuilt, parsed.record, &error_message)) << error_message;

        const int rebuilt_count = gnss_sim::rtklib_nav_output_record_count(rebuilt);
        for (int rebuilt_index = 0; rebuilt_index < rebuilt_count; ++rebuilt_index) {
            gnss_sim::NavOutputRecord restored{};
            ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(rebuilt, rebuilt_index, &restored, &error_message))
                << error_message;
            if (restored.kind == gnss_sim::RtklibNavRecordKind::kIonosphere &&
                restored.ionosphere.system == gnss_sim::NavOutputSystem::kBeidou) {
                EXPECT_EQ(restored.ionosphere.leap_seconds, source.ionosphere.leap_seconds);
                compared = true;
                break;
            }
        }
        break;
    }

    EXPECT_TRUE(compared) << "real BRD400 fixture must preserve BDS legacy ion leap metadata through ASCII NAV";
    gnss_sim::destroy_rtklib_nav_store(rebuilt);
    gnss_sim::destroy_rtklib_nav_store(full);
}
'''
test.write_text(text[:-len(marker)] + addition + marker)

doc = Path('docs/SERIALIZED_NAV_ROUNDTRIP.md')
text = doc.read_text()
anchor = ('The serialized-NAV import adapter therefore keeps the parsed absolute Toe epoch unchanged and converts only the internal raw Toe SOW from GPST to BDT by subtracting 14 seconds, with week wrap handling. Omitting this conversion rotates BeiDou broadcast positions by roughly 16–43 km in the compact real-NAV case. This conversion is an RTKLIB internal representation requirement; it is not a change to the serialized `BD2EPHEMA` fields.\n')
assert anchor in text
addition_doc = ('\n`BD2IONUTCA` similarly carries BDT-UTC leap seconds. The importer adds 14 seconds when restoring RTKLIB `nav.leaps`, whose convention is GPST-UTC. The validator also refuses broadcast-atmosphere positioning until a serialized GPS `IONUTCA` has appeared, so RTKLIB cannot silently fall back to its built-in default Klobuchar coefficients.\n')
doc.write_text(text.replace(anchor, anchor + addition_doc, 1))
