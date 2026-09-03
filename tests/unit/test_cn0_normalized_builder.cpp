#include "model/cn0_model.h"
#include "tools/build_cn0_model/cn0_builder.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace {

using gnss_sim::GnssConstellation;
using gnss_sim::SignalId;
using gnss_sim::cn0_builder::Cn0AggregationConfig;
using gnss_sim::cn0_builder::Cn0BinStatus;
using gnss_sim::cn0_builder::Cn0NormalizationConfig;
using gnss_sim::cn0_builder::Cn0NormalizedBin;
using gnss_sim::cn0_builder::Cn0NormalizedSourceResult;
using gnss_sim::cn0_builder::Cn0SourceNormalizedBin;

Cn0NormalizedSourceResult source_with_delta(double absolute_p50_dbhz, double reference_p50_dbhz,
                                            Cn0BinStatus status = Cn0BinStatus::kReady) {
    Cn0NormalizedSourceResult source{};
    Cn0SourceNormalizedBin bin{};
    bin.constellation = GnssConstellation::kGps;
    bin.signal_id = SignalId::kGpsL1Ca;
    bin.rinex_signal_code = "1C";
    bin.elevation_min_deg = 10.0;
    bin.elevation_max_deg = 15.0;
    bin.source_status = status;
    bin.sample_count = status == Cn0BinStatus::kEmpty ? 0U : 100U;
    bin.reference_ready = true;
    bin.delta_p50_db = absolute_p50_dbhz - reference_p50_dbhz;
    source.bins.push_back(bin);
    return source;
}

TEST(Cn0NormalizedBuilder, ConstantStationOffsetCancelsBeforeCrossSourceAggregation) {
    const Cn0NormalizedSourceResult station_a = source_with_delta(38.0, 48.0);
    const Cn0NormalizedSourceResult station_b = source_with_delta(46.0, 56.0);
    Cn0NormalizationConfig config{};
    config.min_sources_per_bin = 2;
    std::vector<Cn0NormalizedBin> bins;
    std::string error;
    ASSERT_TRUE(gnss_sim::cn0_builder::aggregate_normalized_cn0_sources({station_b, station_a}, config, &bins, &error))
        << error;
    ASSERT_EQ(bins.size(), 1U);
    EXPECT_EQ(bins[0].status, Cn0BinStatus::kReady);
    EXPECT_EQ(bins[0].contributing_source_count, 2U);
    EXPECT_DOUBLE_EQ(bins[0].delta_p50_db, -10.0);
}

TEST(Cn0NormalizedBuilder, SparseSourceDoesNotContaminateReadyAggregate) {
    const Cn0NormalizedSourceResult ready = source_with_delta(40.0, 48.0);
    const Cn0NormalizedSourceResult sparse = source_with_delta(55.0, 56.0, Cn0BinStatus::kSparse);
    Cn0NormalizationConfig config{};
    config.min_sources_per_bin = 1;
    std::vector<Cn0NormalizedBin> bins;
    std::string error;
    ASSERT_TRUE(gnss_sim::cn0_builder::aggregate_normalized_cn0_sources({sparse, ready}, config, &bins, &error))
        << error;
    ASSERT_EQ(bins.size(), 1U);
    EXPECT_EQ(bins[0].contributing_source_count, 1U);
    EXPECT_DOUBLE_EQ(bins[0].delta_p50_db, -8.0);
}

TEST(Cn0NormalizedBuilder, V2WriterRoundTripsAsNormalizedSemantic) {
    const Cn0NormalizedSourceResult station = source_with_delta(40.0, 48.0);
    Cn0NormalizationConfig config{};
    std::vector<Cn0NormalizedBin> bins;
    std::string error;
    ASSERT_TRUE(gnss_sim::cn0_builder::aggregate_normalized_cn0_sources({station}, config, &bins, &error)) << error;

    const std::filesystem::path path = std::filesystem::temp_directory_path() / "gnss_sim_normalized_cn0_test.csv";
    ASSERT_TRUE(gnss_sim::cn0_builder::write_normalized_cn0_model_csv(path.string(), bins, &error)) << error;
    gnss_sim::Cn0Model model{};
    ASSERT_TRUE(gnss_sim::load_cn0_model_csv(path.string().c_str(), 1U, &model, &error)) << error;
    EXPECT_EQ(model.semantic, gnss_sim::Cn0ModelSemantic::kNormalizedElevationShape);
    ASSERT_EQ(model.calibrated_bins.size(), 1U);
    EXPECT_DOUBLE_EQ(model.calibrated_bins[0].delta_p50_db, -8.0);
    std::remove(path.string().c_str());
}

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
    const std::string metadata{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};

    EXPECT_NE(
        metadata.find(
            "\"constellation\":\"GPS\",\"signal\":\"1C\",\"status\":\"READY\",\"count\":100,\"p50_dbhz\":48.000000"),
        std::string::npos);
    EXPECT_NE(
        metadata.find(
            "\"constellation\":\"QZSS\",\"signal\":\"1C\",\"status\":\"READY\",\"count\":100,\"p50_dbhz\":50.000000"),
        std::string::npos);
    EXPECT_NE(metadata.find("\"constellation\":\"GPS\",\"signal\":\"1C\",\"elevation_min_deg\":10.000000,\"status\":"
                            "\"READY\",\"sample_count\":100,\"reference_ready\":true,\"delta_p50_db\":-10.000000"),
              std::string::npos);
    EXPECT_NE(metadata.find("\"constellation\":\"QZSS\",\"signal\":\"1C\",\"elevation_min_deg\":10.000000,\"status\":"
                            "\"READY\",\"sample_count\":100,\"reference_ready\":true,\"delta_p50_db\":-8.000000"),
              std::string::npos);
    EXPECT_NE(metadata.find("\"constellation\":\"GPS\",\"signal\":\"1C\",\"elevation_min_deg\":10.000000,\"status\":"
                            "\"READY\",\"contributing_source_count\":1,\"delta_p50_db\":-10.000000"),
              std::string::npos);
    EXPECT_NE(metadata.find("\"constellation\":\"QZSS\",\"signal\":\"1C\",\"elevation_min_deg\":10.000000,\"status\":"
                            "\"READY\",\"contributing_source_count\":1,\"delta_p50_db\":-8.000000"),
              std::string::npos);
    std::remove(path.string().c_str());
}

} // namespace
