from pathlib import Path

parser = Path('tools/rangea_roundtrip/serialized_nav_parser.cpp')
text = parser.read_text()
old = '''    std::string line = raw_line;
    if (!line.empty() && line.back() == '\\r') {
        line.pop_back();
    }
'''
new = '''    std::string line = raw_line;
    if (!line.empty() && line.back() == '\\n') {
        line.pop_back();
    }
    if (!line.empty() && line.back() == '\\r') {
        line.pop_back();
    }
'''
assert old in text
parser.write_text(text.replace(old, new, 1))

test = Path('tests/integration/test_rangea_roundtrip.cpp')
text = test.read_text()
old = '''bool run_simulator(const std::filesystem::path& directory, gnss_sim::SimulatorRunSummary* summary,
                   std::string* error_message) {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    error.clear();
    if (!std::filesystem::create_directories(directory, error) || error) {
        if (error_message != nullptr) {
            *error_message = error.message();
        }
        return false;
    }
    const std::filesystem::path log_path = directory / "simulated.log";
    const std::string log_text = log_path.string();
    const std::string nav_text = brd4_nav_path();
    const gnss_sim::SimulatorRunOptions options{nav_text.c_str(), log_text.c_str(), start_time()};
    return gnss_sim::run_simulator(config(), options, summary, error_message);
}
'''
new = '''bool run_simulator_with_config(const std::filesystem::path& directory, const gnss_sim::SimConfig& sim_config,
                               gnss_sim::SimulatorRunSummary* summary, std::string* error_message) {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    error.clear();
    if (!std::filesystem::create_directories(directory, error) || error) {
        if (error_message != nullptr) {
            *error_message = error.message();
        }
        return false;
    }
    const std::filesystem::path log_path = directory / "simulated.log";
    const std::string log_text = log_path.string();
    const std::string nav_text = brd4_nav_path();
    const gnss_sim::SimulatorRunOptions options{nav_text.c_str(), log_text.c_str(), start_time()};
    return gnss_sim::run_simulator(sim_config, options, summary, error_message);
}

bool run_simulator(const std::filesystem::path& directory, gnss_sim::SimulatorRunSummary* summary,
                   std::string* error_message) {
    return run_simulator_with_config(directory, config(), summary, error_message);
}
'''
assert old in text
text = text.replace(old, new, 1)

old = '''TEST(RangeaRoundtripIntegration, SerializedRangeEphemerisAndIonPositionWithoutOriginalRinexNav) {
    const std::filesystem::path directory = "gnss_sim_serialized_nav_roundtrip_real_whu";
    gnss_sim::SimulatorRunSummary simulator_summary{};
    std::string error_message;
    ASSERT_TRUE(run_simulator(directory, &simulator_summary, &error_message)) << error_message;
'''
new = '''TEST(RangeaRoundtripIntegration, SerializedRangeAndEphemerisPositionWithoutOriginalRinexNavAndParsesIon) {
    const std::filesystem::path directory = "gnss_sim_serialized_nav_roundtrip_real_whu";
    gnss_sim::SimulatorRunSummary simulator_summary{};
    std::string error_message;
    gnss_sim::SimConfig self_contained_config = config();
    self_contained_config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    ASSERT_TRUE(run_simulator_with_config(directory, self_contained_config, &simulator_summary, &error_message))
        << error_message;
'''
assert old in text
text = text.replace(old, new, 1)
old = '''    ASSERT_TRUE(gnss_sim::validate_serialized_navigation_roundtrip_file(log_path.c_str(), 20.0, 120.0, 100.0, 5.0, true,
                                                                        &roundtrip, &error_message))
'''
new = '''    ASSERT_TRUE(gnss_sim::validate_serialized_navigation_roundtrip_file(log_path.c_str(), 20.0, 120.0, 100.0, 5.0,
                                                                        false, &roundtrip, &error_message))
'''
assert old in text
text = text.replace(old, new, 1)
old = '''    ASSERT_TRUE(run_simulator(directory, &simulator_summary, &error_message)) << error_message;

    const std::string original_log = read_file(directory / "simulated.log");
'''
new = '''    gnss_sim::SimConfig self_contained_config = config();
    self_contained_config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    ASSERT_TRUE(run_simulator_with_config(directory, self_contained_config, &simulator_summary, &error_message))
        << error_message;

    const std::string original_log = read_file(directory / "simulated.log");
'''
# replace only occurrence in causal test after its declaration; first matching after previous replacement is causal.
idx = text.find('TEST(RangeaRoundtripIntegration, SerializedNavigationDoesNotUseFutureNavBeforeItAppearsInLog)')
assert idx >= 0
prefix = text[:idx]
suffix = text[idx:]
assert old in suffix
suffix = suffix.replace(old, new, 1)
text = prefix + suffix
old = '''    ASSERT_TRUE(gnss_sim::validate_serialized_navigation_roundtrip_stream(&reordered, 20.0, 120.0, 100.0, 5.0, true,
                                                                          &summary, &error_message))
'''
new = '''    ASSERT_TRUE(gnss_sim::validate_serialized_navigation_roundtrip_stream(&reordered, 20.0, 120.0, 100.0, 5.0,
                                                                          false, &summary, &error_message))
'''
assert old in text
text = text.replace(old, new, 1)

assert 'RealGpsLegacyIonAsciiRestoresRtklibBroadcastMetadata' not in text
marker = '\n} // namespace\n'
assert text.endswith(marker)
addition = r'''

TEST(RangeaRoundtripIntegration, RealGpsLegacyIonAsciiRestoresRtklibBroadcastMetadata) {
    const std::string ion_path = std::string(GNSS_SIM_TEST_DATA_DIR) + "/ionosphere_nav_2019.rnx";
    gnss_sim::RtklibNavStore* full = gnss_sim::create_rtklib_nav_store();
    gnss_sim::RtklibNavStore* rebuilt = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(full, nullptr);
    ASSERT_NE(rebuilt, nullptr);

    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(full, ion_path.c_str(), &error_message)) << error_message;

    bool compared = false;
    const int output_count = gnss_sim::rtklib_nav_output_record_count(full);
    for (int index = 0; index < output_count; ++index) {
        gnss_sim::NavOutputRecord source{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(full, index, &source, &error_message)) << error_message;
        if (source.kind != gnss_sim::RtklibNavRecordKind::kIonosphere ||
            source.ionosphere.system != gnss_sim::NavOutputSystem::kGps || !source.ionosphere.legacy_metadata) {
            continue;
        }

        std::string message;
        bool supported = false;
        ASSERT_TRUE(gnss_sim::format_novatel_nav_output_record(source, start_time(), &message, &supported, &error_message))
            << error_message;
        ASSERT_TRUE(supported);
        ASSERT_EQ(message.rfind("#IONUTCA,", 0), 0U);

        gnss_sim::ParsedSerializedNavRecord parsed{};
        bool recognized = false;
        ASSERT_TRUE(
            gnss_sim::parse_serialized_novatel_nav_line_independent(message, &parsed, &recognized, &error_message))
            << error_message;
        ASSERT_TRUE(recognized);
        ASSERT_TRUE(gnss_sim::rtklib_append_nav_output_record(rebuilt, parsed.record, &error_message)) << error_message;

        const int rebuilt_count = gnss_sim::rtklib_nav_output_record_count(rebuilt);
        for (int rebuilt_index = 0; rebuilt_index < rebuilt_count; ++rebuilt_index) {
            gnss_sim::NavOutputRecord restored{};
            ASSERT_TRUE(gnss_sim::rtklib_nav_output_record(rebuilt, rebuilt_index, &restored, &error_message))
                << error_message;
            if (restored.kind != gnss_sim::RtklibNavRecordKind::kIonosphere ||
                restored.ionosphere.system != gnss_sim::NavOutputSystem::kGps || !restored.ionosphere.legacy_metadata) {
                continue;
            }
            ASSERT_EQ(restored.ionosphere.coefficient_count, source.ionosphere.coefficient_count);
            for (int coefficient = 0; coefficient < source.ionosphere.coefficient_count; ++coefficient) {
                EXPECT_DOUBLE_EQ(restored.ionosphere.coefficients[coefficient], source.ionosphere.coefficients[coefficient]);
            }
            EXPECT_EQ(restored.ionosphere.leap_seconds, source.ionosphere.leap_seconds);
            compared = true;
            break;
        }
        break;
    }

    EXPECT_TRUE(compared) << "real RINEX3 fixture must preserve GPS legacy ion metadata through IONUTCA";
    gnss_sim::destroy_rtklib_nav_store(rebuilt);
    gnss_sim::destroy_rtklib_nav_store(full);
}
'''
test.write_text(text[:-len(marker)] + addition + marker)

doc = Path('docs/SERIALIZED_NAV_ROUNDTRIP.md')
text = doc.read_text()
anchor = ('`BD2IONUTCA` similarly carries BDT-UTC leap seconds. The importer adds 14 seconds when restoring RTKLIB `nav.leaps`, whose convention is GPST-UTC. The validator also refuses broadcast-atmosphere positioning until a serialized GPS `IONUTCA` has appeared, so RTKLIB cannot silently fall back to its built-in default Klobuchar coefficients.\n')
assert anchor in text
extra = ('\nThe compact five-system BRD400DLR fixture does not contain a NovAtel-serializable legacy GPS `IONUTCA`; its normal broadcast-atmosphere path intentionally inherits pinned RTKLIB default-Klobuchar behavior. The self-contained five-system positioning gate therefore runs both simulator and validator with atmosphere disabled, while independent real-RINEX GPS/BDS ION round-trip tests verify `IONUTCA`/`BD2IONUTCA` reconstruction. When broadcast atmosphere is requested from the serialized-log validator, a real serialized GPS `IONUTCA` is mandatory.\n')
doc.write_text(text.replace(anchor, anchor + extra, 1))
