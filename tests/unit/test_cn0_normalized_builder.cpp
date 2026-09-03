#include "model/cn0_model.h"
#include "tools/build_cn0_model/cn0_builder.h"

#include <cstdio>
#include <filesystem>
#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace {

using gnss_sim::GnssConstellation;
using gnss_sim::SignalId;
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

} // namespace
