#include "tools/build_cn0_model/cn0_builder.h"
#include "tools/build_cn0_model/cn0_statistics.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <vector>

namespace {

using gnss_sim::GnssConstellation;
using gnss_sim::SignalId;
using gnss_sim::cn0_builder::Cn0AggregationConfig;
using gnss_sim::cn0_builder::Cn0Ar1Status;
using gnss_sim::cn0_builder::Cn0BinStatistics;
using gnss_sim::cn0_builder::Cn0BinStatus;
using gnss_sim::cn0_builder::Cn0BuildResult;
using gnss_sim::cn0_builder::Cn0InputSource;
using gnss_sim::cn0_builder::Cn0SampleValidity;
using gnss_sim::cn0_builder::Cn0StatisticsAccumulator;
using gnss_sim::cn0_builder::RinexCn0Sample;

constexpr double kPi = 3.14159265358979323846;

RinexCn0Sample sample(double cn0_dbhz, double elevation_deg, std::int64_t tow_ns, int satellite_number = 1,
                      SignalId signal_id = SignalId::kGpsL1Ca) {
    RinexCn0Sample result{};
    result.time = {2253, tow_ns};
    result.constellation = GnssConstellation::kGps;
    result.satellite_number = satellite_number;
    result.prn = satellite_number;
    result.signal_id = signal_id;
    result.rinex_signal_code = "1C";
    result.signal_strength_value = cn0_dbhz;
    result.cn0_dbhz = cn0_dbhz;
    result.elevation_rad = elevation_deg * kPi / 180.0;
    result.azimuth_rad = 0.0;
    result.validity = Cn0SampleValidity::kValidDbHz;
    return result;
}

const Cn0BinStatistics* find_bin(const std::vector<Cn0BinStatistics>& bins, SignalId signal_id,
                                 double elevation_min_deg) {
    for (const Cn0BinStatistics& bin : bins) {
        if (bin.signal_id == signal_id && std::fabs(bin.elevation_min_deg - elevation_min_deg) < 1e-12) {
            return &bin;
        }
    }
    return nullptr;
}

std::string test_data(const char* file_name) {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/" + file_name;
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

TEST(Cn0Statistics, Type7QuantilesMeanPopulationStddevAndMadAreGolden) {
    Cn0AggregationConfig config{};
    config.elevation_max_deg = 10.0;
    config.elevation_bin_width_deg = 10.0;
    config.min_samples_per_bin = 4;
    Cn0StatisticsAccumulator accumulator(config);
    std::string error;
    ASSERT_TRUE(accumulator.valid(&error)) << error;
    ASSERT_TRUE(accumulator.begin_source(1.0, &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(10.0, 5.0, 0), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(20.0, 5.0, 1000000000LL), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(30.0, 5.0, 2000000000LL), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(40.0, 5.0, 3000000000LL), &error)) << error;
    accumulator.end_source();

    const std::vector<Cn0BinStatistics> bins = accumulator.finalize();
    const Cn0BinStatistics* bin = find_bin(bins, SignalId::kGpsL1Ca, 0.0);
    ASSERT_NE(bin, nullptr);
    EXPECT_EQ(bin->status, Cn0BinStatus::kReady);
    EXPECT_EQ(bin->count, 4U);
    EXPECT_DOUBLE_EQ(bin->p05_dbhz, 11.5);
    EXPECT_DOUBLE_EQ(bin->p10_dbhz, 13.0);
    EXPECT_DOUBLE_EQ(bin->p25_dbhz, 17.5);
    EXPECT_DOUBLE_EQ(bin->p50_dbhz, 25.0);
    EXPECT_DOUBLE_EQ(bin->p75_dbhz, 32.5);
    EXPECT_DOUBLE_EQ(bin->p90_dbhz, 37.0);
    EXPECT_DOUBLE_EQ(bin->p95_dbhz, 38.5);
    EXPECT_DOUBLE_EQ(bin->mean_dbhz, 25.0);
    EXPECT_NEAR(bin->stddev_dbhz, std::sqrt(125.0), 1e-12);
    EXPECT_DOUBLE_EQ(bin->mad_dbhz, 10.0);
}

TEST(Cn0Statistics, ElevationEdgesAreHalfOpenExceptFinalMaximum) {
    Cn0AggregationConfig config{};
    config.elevation_max_deg = 10.0;
    config.elevation_bin_width_deg = 5.0;
    config.min_samples_per_bin = 1;
    Cn0StatisticsAccumulator accumulator(config);
    std::string error;
    ASSERT_TRUE(accumulator.begin_source(1.0, &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(30.0, 0.0, 0), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(31.0, 4.999, 1000000000LL), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(32.0, 5.0, 2000000000LL), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(33.0, 10.0, 3000000000LL), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(34.0, -0.01, 4000000000LL), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(35.0, 10.01, 5000000000LL), &error)) << error;
    accumulator.end_source();

    const std::vector<Cn0BinStatistics> bins = accumulator.finalize();
    const Cn0BinStatistics* lower = find_bin(bins, SignalId::kGpsL1Ca, 0.0);
    const Cn0BinStatistics* upper = find_bin(bins, SignalId::kGpsL1Ca, 5.0);
    ASSERT_NE(lower, nullptr);
    ASSERT_NE(upper, nullptr);
    EXPECT_EQ(lower->count, 2U);
    EXPECT_EQ(upper->count, 2U);
    EXPECT_FALSE(lower->includes_upper_edge);
    EXPECT_TRUE(upper->includes_upper_edge);
    EXPECT_EQ(accumulator.summary().rejected_elevation_range, 2U);
}

TEST(Cn0Statistics, InvalidAndOffGridSamplesCannotContaminateBins) {
    Cn0AggregationConfig config{};
    config.elevation_max_deg = 10.0;
    config.elevation_bin_width_deg = 10.0;
    config.min_samples_per_bin = 2;
    Cn0StatisticsAccumulator accumulator(config);
    std::string error;
    ASSERT_TRUE(accumulator.begin_source(1.0, &error)) << error;

    RinexCn0Sample invalid = sample(30.0, 5.0, 0);
    invalid.validity = Cn0SampleValidity::kGeometryUnavailable;
    ASSERT_TRUE(accumulator.add_sample(invalid, &error)) << error;
    RinexCn0Sample nonfinite = sample(31.0, 5.0, 1000000000LL);
    nonfinite.cn0_dbhz = std::numeric_limits<double>::quiet_NaN();
    ASSERT_TRUE(accumulator.add_sample(nonfinite, &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(31.1, 5.0, 2000000000LL), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(32.0, 5.0, 3000000000LL), &error)) << error;
    accumulator.end_source();

    const std::vector<Cn0BinStatistics> bins = accumulator.finalize();
    const Cn0BinStatistics* bin = find_bin(bins, SignalId::kGpsL1Ca, 0.0);
    ASSERT_NE(bin, nullptr);
    EXPECT_EQ(bin->count, 1U);
    EXPECT_EQ(bin->status, Cn0BinStatus::kSparse);
    EXPECT_EQ(accumulator.summary().rejected_validity, 1U);
    EXPECT_EQ(accumulator.summary().rejected_nonfinite, 1U);
    EXPECT_EQ(accumulator.summary().rejected_cn0_grid, 1U);
}

TEST(Cn0Statistics, TemporalStatisticsRespectGapBinAndSourceBoundaries) {
    Cn0AggregationConfig config{};
    config.elevation_max_deg = 20.0;
    config.elevation_bin_width_deg = 10.0;
    config.min_samples_per_bin = 1;
    config.min_temporal_pairs = 3;
    Cn0StatisticsAccumulator accumulator(config);
    std::string error;
    ASSERT_TRUE(accumulator.begin_source(1.0, &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(30.0, 5.0, 0), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(31.0, 5.0, 1000000000LL), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(33.0, 5.0, 2000000000LL), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(32.0, 5.0, 3000000000LL), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(34.0, 5.0, 5000000000LL), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(35.0, 15.0, 6000000000LL), &error)) << error;
    accumulator.end_source();
    ASSERT_TRUE(accumulator.begin_source(1.0, &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(36.0, 5.0, 7000000000LL), &error)) << error;
    accumulator.end_source();

    const std::vector<Cn0BinStatistics> bins = accumulator.finalize();
    const Cn0BinStatistics* bin = find_bin(bins, SignalId::kGpsL1Ca, 0.0);
    ASSERT_NE(bin, nullptr);
    EXPECT_EQ(bin->delta_count, 3U);
    EXPECT_DOUBLE_EQ(bin->delta_p50_dbhz, 1.0);
    EXPECT_DOUBLE_EQ(bin->delta_p90_dbhz, 1.8);
    EXPECT_DOUBLE_EQ(bin->delta_p99_dbhz, 1.98);
    EXPECT_EQ(bin->ar1_status, Cn0Ar1Status::kAvailable);
    EXPECT_TRUE(std::isfinite(bin->ar1));
    EXPECT_EQ(accumulator.summary().temporal_pairs, 3U);
    EXPECT_EQ(accumulator.summary().temporal_rejected_gap, 1U);
    EXPECT_EQ(accumulator.summary().temporal_rejected_bin_change, 1U);
    EXPECT_EQ(accumulator.summary().sources, 2U);
}

TEST(Cn0Statistics, MissingIntervalAndZeroVarianceAreExplicit) {
    Cn0AggregationConfig config{};
    config.elevation_max_deg = 10.0;
    config.elevation_bin_width_deg = 10.0;
    config.min_samples_per_bin = 1;
    config.min_temporal_pairs = 2;
    std::string error;

    Cn0StatisticsAccumulator accumulator(config);
    ASSERT_TRUE(accumulator.begin_source(std::numeric_limits<double>::quiet_NaN(), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(30.0, 5.0, 0), &error)) << error;
    ASSERT_TRUE(accumulator.add_sample(sample(30.0, 5.0, 1000000000LL), &error)) << error;
    accumulator.end_source();
    EXPECT_EQ(accumulator.summary().temporal_rejected_no_interval, 1U);

    Cn0StatisticsAccumulator constant_accumulator(config);
    ASSERT_TRUE(constant_accumulator.begin_source(1.0, &error)) << error;
    ASSERT_TRUE(constant_accumulator.add_sample(sample(30.0, 5.0, 0), &error)) << error;
    ASSERT_TRUE(constant_accumulator.add_sample(sample(30.0, 5.0, 1000000000LL), &error)) << error;
    ASSERT_TRUE(constant_accumulator.add_sample(sample(30.0, 5.0, 2000000000LL), &error)) << error;
    constant_accumulator.end_source();
    const std::vector<Cn0BinStatistics> bins = constant_accumulator.finalize();
    const Cn0BinStatistics* bin = find_bin(bins, SignalId::kGpsL1Ca, 0.0);
    ASSERT_NE(bin, nullptr);
    EXPECT_EQ(bin->ar1_status, Cn0Ar1Status::kZeroVariance);
    EXPECT_FALSE(std::isfinite(bin->ar1));
}

TEST(Cn0Statistics, HistogramMemoryIsIndependentOfSampleCount) {
    Cn0AggregationConfig config{};
    Cn0StatisticsAccumulator accumulator(config);
    std::string error;
    ASSERT_TRUE(accumulator.begin_source(1.0, &error)) << error;
    const std::size_t cells_before = accumulator.bounded_histogram_cells();
    for (int index = 0; index < 10000; ++index) {
        ASSERT_TRUE(accumulator.add_sample(sample(30.0 + static_cast<double>(index % 20) * 0.25, 45.0,
                                                  static_cast<std::int64_t>(index) * 1000000000LL),
                                           &error))
            << error;
    }
    accumulator.end_source();
    EXPECT_EQ(accumulator.bounded_histogram_cells(), cells_before);
    EXPECT_EQ(accumulator.summary().accepted_samples, 10000U);
}

TEST(Cn0Statistics, BuilderModelAndMetadataAreByteIdenticalForFixedSources) {
    const std::vector<Cn0InputSource> sources = {
        {test_data("cn0_stream_acceptance_obs.rnx"), test_data("multi_gnss_acceptance_nav.rnx")}};
    Cn0AggregationConfig config{};
    config.min_samples_per_bin = 1;
    Cn0BuildResult first{};
    Cn0BuildResult second{};
    std::string error;
    ASSERT_TRUE(gnss_sim::cn0_builder::build_cn0_model(sources, config, &first, &error)) << error;
    ASSERT_TRUE(gnss_sim::cn0_builder::build_cn0_model(sources, config, &second, &error)) << error;

    const std::filesystem::path directory = std::filesystem::temp_directory_path();
    const std::filesystem::path model_a = directory / "gnss_sim_cn0_model_a.csv";
    const std::filesystem::path model_b = directory / "gnss_sim_cn0_model_b.csv";
    const std::filesystem::path meta_a = directory / "gnss_sim_cn0_meta_a.json";
    const std::filesystem::path meta_b = directory / "gnss_sim_cn0_meta_b.json";
    ASSERT_TRUE(gnss_sim::cn0_builder::write_cn0_model_csv(model_a.string(), config, first.aggregation_summary,
                                                           first.bins, &error))
        << error;
    ASSERT_TRUE(gnss_sim::cn0_builder::write_cn0_model_csv(model_b.string(), config, second.aggregation_summary,
                                                           second.bins, &error))
        << error;
    ASSERT_TRUE(gnss_sim::cn0_builder::write_cn0_metadata_json(meta_a.string(), config, first, &error)) << error;
    ASSERT_TRUE(gnss_sim::cn0_builder::write_cn0_metadata_json(meta_b.string(), config, second, &error)) << error;
    EXPECT_EQ(read_file(model_a), read_file(model_b));
    EXPECT_EQ(read_file(meta_a), read_file(meta_b));
    EXPECT_EQ(first.bins.size(), 21U * 18U);
    EXPECT_EQ(first.aggregation_summary.accepted_samples, 30U);

    std::remove(model_a.string().c_str());
    std::remove(model_b.string().c_str());
    std::remove(meta_a.string().c_str());
    std::remove(meta_b.string().c_str());
}

} // namespace
