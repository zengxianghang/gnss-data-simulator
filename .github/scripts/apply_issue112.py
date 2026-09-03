from pathlib import Path

writer = Path("tools/build_cn0_model/cn0_normalized_builder.cpp")
text = writer.read_text()
old_reference = 'output << "{\\\"signal\\\":\\\"" << reference.rinex_signal_code << "\\\",\\\"status\\\":\\\""'
new_reference = (
    'output << "{\\\"constellation\\\":\\\"" << constellation_name(reference.constellation)\n'
    '       << "\\\",\\\"signal\\\":\\\"" << reference.rinex_signal_code << "\\\",\\\"status\\\":\\\""'
)
old_bin = 'output << "{\\\"signal\\\":\\\"" << bin.rinex_signal_code << "\\\",\\\"elevation_min_deg\\\":" << bin.elevation_min_deg'
new_bin = (
    'output << "{\\\"constellation\\\":\\\"" << constellation_name(bin.constellation) << "\\\",\\\"signal\\\":\\\""\n'
    '       << bin.rinex_signal_code << "\\\",\\\"elevation_min_deg\\\":" << bin.elevation_min_deg'
)
if text.count(old_reference) != 1:
    raise SystemExit(f"unexpected reference writer match count: {text.count(old_reference)}")
if text.count(old_bin) != 2:
    raise SystemExit(f"unexpected bin writer match count: {text.count(old_bin)}")
text = text.replace(old_reference, new_reference)
text = text.replace(old_bin, new_bin)
writer.write_text(text)

test = Path("tests/unit/test_cn0_normalized_builder.cpp")
text = test.read_text()
if "#include <fstream>\n" not in text:
    text = text.replace("#include <filesystem>\n", "#include <filesystem>\n#include <fstream>\n")
marker = "\n} // namespace\n"
if text.count(marker) != 1:
    raise SystemExit("unexpected namespace marker count")
addition = r'''

TEST(Cn0NormalizedBuilder, MetadataKeepsConstellationWhenRinexSignalCodesCollide) {
    gnss_sim::cn0_builder::Cn0NormalizedBuildResult result{};
    Cn0NormalizedSourceResult source{};
    source.metadata.observation_file.file_name = "fixture.obs";
    source.metadata.navigation_file.file_name = "fixture.nav";

    gnss_sim::cn0_builder::Cn0SignalReference gps_reference{};
    gps_reference.constellation = GnssConstellation::kGps;
    gps_reference.signal_id = SignalId::kGpsL1Ca;
    gps_reference.rinex_signal_code = "1C";
    gps_reference.status = gnss_sim::cn0_builder::Cn0ReferenceStatus::kReady;
    gps_reference.count = 100U;
    gps_reference.p50_dbhz = 48.0;
    gnss_sim::cn0_builder::Cn0SignalReference qzss_reference = gps_reference;
    qzss_reference.constellation = GnssConstellation::kQzss;
    qzss_reference.signal_id = SignalId::kQzssL1Ca;
    qzss_reference.p50_dbhz = 50.0;
    source.references = {gps_reference, qzss_reference};

    Cn0SourceNormalizedBin gps_bin{};
    gps_bin.constellation = GnssConstellation::kGps;
    gps_bin.signal_id = SignalId::kGpsL1Ca;
    gps_bin.rinex_signal_code = "1C";
    gps_bin.elevation_min_deg = 10.0;
    gps_bin.elevation_max_deg = 15.0;
    gps_bin.source_status = Cn0BinStatus::kReady;
    gps_bin.sample_count = 100U;
    gps_bin.reference_ready = true;
    gps_bin.delta_p50_db = -10.0;
    Cn0SourceNormalizedBin qzss_bin = gps_bin;
    qzss_bin.constellation = GnssConstellation::kQzss;
    qzss_bin.signal_id = SignalId::kQzssL1Ca;
    qzss_bin.delta_p50_db = -8.0;
    source.bins = {gps_bin, qzss_bin};
    result.sources.push_back(source);

    Cn0NormalizedBin gps_aggregate{};
    gps_aggregate.constellation = GnssConstellation::kGps;
    gps_aggregate.signal_id = SignalId::kGpsL1Ca;
    gps_aggregate.rinex_signal_code = "1C";
    gps_aggregate.elevation_min_deg = 10.0;
    gps_aggregate.elevation_max_deg = 15.0;
    gps_aggregate.status = Cn0BinStatus::kReady;
    gps_aggregate.contributing_source_count = 1U;
    gps_aggregate.delta_p50_db = -10.0;
    Cn0NormalizedBin qzss_aggregate = gps_aggregate;
    qzss_aggregate.constellation = GnssConstellation::kQzss;
    qzss_aggregate.signal_id = SignalId::kQzssL1Ca;
    qzss_aggregate.delta_p50_db = -8.0;
    result.bins = {gps_aggregate, qzss_aggregate};

    Cn0AggregationConfig aggregation{};
    Cn0NormalizationConfig normalization{};
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "gnss_sim_cn0_constellation_meta.json";
    std::string error;
    ASSERT_TRUE(gnss_sim::cn0_builder::write_normalized_cn0_metadata_json(path.string(), aggregation, normalization,
                                                                          result, &error))
        << error;
    std::ifstream input(path, std::ios::binary);
    const std::string metadata(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
    EXPECT_NE(metadata.find("\"constellation\":\"GPS\",\"signal\":\"1C\""), std::string::npos);
    EXPECT_NE(metadata.find("\"constellation\":\"QZSS\",\"signal\":\"1C\""), std::string::npos);
    std::remove(path.string().c_str());
}
'''
text = text.replace(marker, addition + marker)
test.write_text(text)
