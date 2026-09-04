#include "gnss_sim/sim_time.h"
#include "model/cn0_model.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <string>

namespace {

constexpr const char* kModelHeader =
    "schema_version,constellation,signal,elevation_min_deg,elevation_max_deg,upper_edge_inclusive,status,count,"
    "p05_dbhz,p10_dbhz,p25_dbhz,p50_dbhz,p75_dbhz,p90_dbhz,p95_dbhz,mean_dbhz,stddev_dbhz,mad_dbhz,"
    "delta_count,delta_p50_dbhz,delta_p90_dbhz,delta_p99_dbhz,ar1_status,ar1";
constexpr const char* kNormalizedModelHeader =
    "schema_version,model_semantic,constellation,signal,elevation_min_deg,elevation_max_deg,upper_edge_inclusive,"
    "status,contributing_source_count,delta_p50_db";

std::string runtime_model_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/runtime_cn0_model.csv";
}

double estimate_model(const gnss_sim::Cn0Model& model, gnss_sim::SignalId signal_id, double elevation_deg,
                      double sow_sec) {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2300, sow_sec, &time));
    double cn0_dbhz = 0.0;
    EXPECT_TRUE(gnss_sim::cn0_model_estimate_dbhz(model, signal_id, elevation_deg, time, &cn0_dbhz));
    return cn0_dbhz;
}

double estimate(gnss_sim::SignalId signal_id, double elevation_deg, double sow_sec, std::uint64_t seed = 1234U) {
    const gnss_sim::Cn0Model model = gnss_sim::make_builtin_cn0_model(seed);
    return estimate_model(model, signal_id, elevation_deg, sow_sec);
}

std::string model_row(double elevation_min_deg, double elevation_max_deg, bool inclusive, const char* status,
                      std::uint64_t count, double p50_dbhz) {
    std::ostringstream output;
    output << "gnss-cn0-model-v1,GPS,1C," << elevation_min_deg << ',' << elevation_max_deg << ',' << (inclusive ? 1 : 0)
           << ',' << status << ',' << count << ',' << (p50_dbhz - 1.0) << ',' << (p50_dbhz - 1.0) << ','
           << (p50_dbhz - 1.0) << ',' << p50_dbhz << ',' << (p50_dbhz + 1.0) << ',' << (p50_dbhz + 1.0) << ','
           << (p50_dbhz + 1.0) << ',' << p50_dbhz << ",1,1,0,,,,INSUFFICIENT_SUPPORT,";
    return output.str();
}

std::string normalized_model_row(const char* signal, double elevation_min_deg, double elevation_max_deg, bool inclusive,
                                 const char* status, std::uint64_t source_count, const char* delta_p50_db) {
    std::ostringstream output;
    output << "gnss-cn0-model-v2,NORMALIZED_ELEVATION_SHAPE,GPS," << signal << ',' << elevation_min_deg << ','
           << elevation_max_deg << ',' << (inclusive ? 1 : 0) << ',' << status << ',' << source_count << ','
           << delta_p50_db;
    return output.str();
}

class TemporaryModel {
  public:
    explicit TemporaryModel(const std::string& content) {
        static int sequence = 0;
        path_ =
            std::filesystem::temp_directory_path() / ("gnss_sim_runtime_cn0_" + std::to_string(++sequence) + ".csv");
        std::ofstream output(path_, std::ios::binary | std::ios::trunc);
        output << content;
    }

    ~TemporaryModel() {
        std::remove(path_.string().c_str());
    }

    std::string path() const {
        return path_.string();
    }

  private:
    std::filesystem::path path_;
};

TEST(Cn0Model, ElevationFallbackIsContinuousAndMonotonicAtPiecewiseBoundaries) {
    const double sow = 180000.0;
    const gnss_sim::SignalId signal = gnss_sim::SignalId::kGpsL1Ca;
    const double elevations[] = {0.0,    4.999, 5.0,    5.001,  14.999, 15.0,   15.001,
                                 29.999, 30.0,  30.001, 59.999, 60.0,   60.001, 90.0};
    double previous = estimate(signal, elevations[0], sow);
    for (std::size_t index = 1; index < sizeof(elevations) / sizeof(elevations[0]); ++index) {
        const double current = estimate(signal, elevations[index], sow);
        EXPECT_GE(current + 1.0e-12, previous);
        previous = current;
    }
    EXPECT_GT(estimate(signal, 90.0, sow), estimate(signal, 3.0, sow) + 10.0);
}

TEST(Cn0Model, SameSeedAndTimeAreExactlyRepeatable) {
    const double first = estimate(gnss_sim::SignalId::kGalileoE5A, 42.0, 200000.25, 99U);
    const double second = estimate(gnss_sim::SignalId::kGalileoE5A, 42.0, 200000.25, 99U);
    EXPECT_DOUBLE_EQ(first, second);

    const double later = estimate(gnss_sim::SignalId::kGalileoE5A, 42.0, 200001.25, 99U);
    EXPECT_NE(first, later);
    EXPECT_LT(std::abs(first - later), 1.0);
}

TEST(Cn0Model, SignalCalibrationOffsetsRemainVisible) {
    const double l1c = estimate(gnss_sim::SignalId::kGpsL1C, 45.0, 250000.0);
    const double l2p = estimate(gnss_sim::SignalId::kGpsL2P, 45.0, 250000.0);
    EXPECT_GT(l1c, l2p + 1.0);
}

TEST(Cn0Model, TemporalVariationIsContinuousAcrossGpsWeekBoundary) {
    const gnss_sim::Cn0Model model = gnss_sim::make_builtin_cn0_model(7U);
    gnss_sim::SimTime before{};
    gnss_sim::SimTime after{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 604799.9, &before));
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2301, 0.1, &after));
    double before_cn0 = 0.0;
    double after_cn0 = 0.0;
    ASSERT_TRUE(gnss_sim::cn0_model_estimate_dbhz(model, gnss_sim::SignalId::kBeidouB1C, 35.0, before, &before_cn0));
    ASSERT_TRUE(gnss_sim::cn0_model_estimate_dbhz(model, gnss_sim::SignalId::kBeidouB1C, 35.0, after, &after_cn0));
    EXPECT_LT(std::abs(after_cn0 - before_cn0), 0.2);
}

TEST(Cn0Model, InvalidArgumentsAreRejected) {
    const gnss_sim::Cn0Model model = gnss_sim::make_builtin_cn0_model(1U);
    gnss_sim::SimTime time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 1.0, &time));
    EXPECT_FALSE(gnss_sim::cn0_model_estimate_dbhz(model, gnss_sim::SignalId::kGpsL1Ca, NAN, time, nullptr));
}

TEST(Cn0Model, CalibratedCentersEdgesAndBetweenCenterInterpolationAreGolden) {
    gnss_sim::Cn0Model model{};
    std::string error;
    ASSERT_TRUE(gnss_sim::load_cn0_model_csv(runtime_model_path().c_str(), 1234U, &model, &error)) << error;
    EXPECT_EQ(model.source, gnss_sim::Cn0ModelSource::kCalibratedCsv);
    EXPECT_EQ(model.semantic, gnss_sim::Cn0ModelSemantic::kAbsoluteStationCn0);
    EXPECT_STREQ(gnss_sim::cn0_model_semantic_name(model.semantic), "ABSOLUTE_STATION_CN0");
    EXPECT_EQ(model.identity.schema_version, "gnss-cn0-model-v1");
    EXPECT_EQ(model.identity.file_name, "runtime_cn0_model.csv");
    EXPECT_FALSE(model.identity.hash.empty());
    EXPECT_GT(model.identity.size_bytes, 0U);

    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.cn0_high_dbhz.push_back({"GPS L1 C/A", 52.0});
    ASSERT_TRUE(gnss_sim::configure_cn0_model_runtime(config, &model, &error)) << error;

    const double sow = 180000.0;
    const double at_15 = estimate_model(model, gnss_sim::SignalId::kGpsL1Ca, 15.0, sow);
    const double at_30 = estimate_model(model, gnss_sim::SignalId::kGpsL1Ca, 30.0, sow);
    const double at_45 = estimate_model(model, gnss_sim::SignalId::kGpsL1Ca, 45.0, sow);
    const double at_75 = estimate_model(model, gnss_sim::SignalId::kGpsL1Ca, 75.0, sow);
    EXPECT_NEAR(at_30, 0.5 * (at_15 + at_45), 1e-12);
    EXPECT_NEAR(at_45 - at_15, 10.0, 1e-12);
    EXPECT_NEAR(at_75 - at_45, 10.0, 1e-12);
    EXPECT_NEAR(estimate_model(model, gnss_sim::SignalId::kGpsL1Ca, 0.0, sow) - at_15, 0.0, 1e-12);
    EXPECT_NEAR(estimate_model(model, gnss_sim::SignalId::kGpsL1Ca, 90.0, sow) - at_75, 0.0, 1e-12);
}

TEST(Cn0Model, NormalizedSchemaRequiresRuntimeBaselineBeforeAbsoluteEvaluation) {
    const std::string content = std::string(kNormalizedModelHeader) + "\n" +
                                normalized_model_row("1C", 0.0, 45.0, false, "READY", 3U, "-8.500000") + "\n" +
                                normalized_model_row("1C", 45.0, 90.0, true, "READY", 3U, "-0.500000") + "\n";
    const TemporaryModel file(content);
    gnss_sim::Cn0Model model{};
    std::string error;
    ASSERT_TRUE(gnss_sim::load_cn0_model_csv(file.path().c_str(), 11U, &model, &error)) << error;
    EXPECT_EQ(model.semantic, gnss_sim::Cn0ModelSemantic::kNormalizedElevationShape);
    EXPECT_STREQ(gnss_sim::cn0_model_semantic_name(model.semantic), "NORMALIZED_ELEVATION_SHAPE");
    EXPECT_EQ(model.identity.schema_version, "gnss-cn0-model-v2");
    ASSERT_EQ(model.calibrated_bins.size(), 2U);
    EXPECT_DOUBLE_EQ(model.calibrated_bins[0].delta_p50_db, -8.5);
    EXPECT_EQ(model.calibrated_bins[0].support_count, 3U);

    gnss_sim::SimTime time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2300, 100.0, &time));
    double cn0_dbhz = 0.0;
    EXPECT_FALSE(gnss_sim::cn0_model_estimate_dbhz(model, gnss_sim::SignalId::kGpsL1Ca, 20.0, time, &cn0_dbhz));

    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.cn0_high_dbhz.push_back({"GPS L1 C/A", 47.0});
    ASSERT_TRUE(gnss_sim::configure_cn0_model_runtime(config, &model, &error)) << error;
    EXPECT_TRUE(gnss_sim::cn0_model_estimate_dbhz(model, gnss_sim::SignalId::kGpsL1Ca, 20.0, time, &cn0_dbhz));
}

TEST(Cn0Model, NormalizedBaselineShiftTranslatesOnlySelectedSignalAndPreservesShape) {
    const std::string content = std::string(kNormalizedModelHeader) + "\n" +
                                normalized_model_row("1C", 0.0, 45.0, false, "READY", 4U, "-8.000000") + "\n" +
                                normalized_model_row("1C", 45.0, 90.0, true, "READY", 4U, "-1.000000") + "\n" +
                                normalized_model_row("5Q", 0.0, 45.0, false, "READY", 3U, "-6.000000") + "\n" +
                                normalized_model_row("5Q", 45.0, 90.0, true, "READY", 3U, "-0.500000") + "\n";
    const TemporaryModel file(content);
    gnss_sim::Cn0Model base{};
    gnss_sim::Cn0Model shifted{};
    std::string error;
    ASSERT_TRUE(gnss_sim::load_cn0_model_csv(file.path().c_str(), 77U, &base, &error)) << error;
    ASSERT_TRUE(gnss_sim::load_cn0_model_csv(file.path().c_str(), 77U, &shifted, &error)) << error;

    gnss_sim::SimConfig base_config = gnss_sim::default_sim_config();
    base_config.cn0_high_dbhz = {{"GPS L1 C/A", 47.0}, {"GPS L5Q", 49.0}};
    gnss_sim::SimConfig shifted_config = base_config;
    shifted_config.cn0_high_dbhz[0].cn0_dbhz += 3.0;
    ASSERT_TRUE(gnss_sim::configure_cn0_model_runtime(base_config, &base, &error)) << error;
    ASSERT_TRUE(gnss_sim::configure_cn0_model_runtime(shifted_config, &shifted, &error)) << error;

    const double sow = 200000.0;
    const double l1_low = estimate_model(base, gnss_sim::SignalId::kGpsL1Ca, 22.5, sow);
    const double l1_high = estimate_model(base, gnss_sim::SignalId::kGpsL1Ca, 67.5, sow);
    const double shifted_l1_low = estimate_model(shifted, gnss_sim::SignalId::kGpsL1Ca, 22.5, sow);
    const double shifted_l1_high = estimate_model(shifted, gnss_sim::SignalId::kGpsL1Ca, 67.5, sow);
    EXPECT_NEAR(shifted_l1_low - l1_low, 3.0, 1e-12);
    EXPECT_NEAR(shifted_l1_high - l1_high, 3.0, 1e-12);
    EXPECT_NEAR((shifted_l1_high - shifted_l1_low) - (l1_high - l1_low), 0.0, 1e-12);

    const double l5 = estimate_model(base, gnss_sim::SignalId::kGpsL5Q, 22.5, sow);
    const double shifted_l5 = estimate_model(shifted, gnss_sim::SignalId::kGpsL5Q, 22.5, sow);
    EXPECT_DOUBLE_EQ(l5, shifted_l5);
}

TEST(Cn0Model, NormalizedReadyBinsInterpolateButSparseGapFallsBackWithoutBridging) {
    const std::string interpolation_content =
        std::string(kNormalizedModelHeader) + "\n" +
        normalized_model_row("1C", 0.0, 30.0, false, "READY", 3U, "-9.000000") + "\n" +
        normalized_model_row("1C", 30.0, 60.0, false, "READY", 3U, "-3.000000") + "\n" +
        normalized_model_row("1C", 60.0, 90.0, true, "READY", 3U, "0.000000") + "\n";
    const TemporaryModel interpolation_file(interpolation_content);
    gnss_sim::Cn0Model interpolation{};
    std::string error;
    ASSERT_TRUE(gnss_sim::load_cn0_model_csv(interpolation_file.path().c_str(), 17U, &interpolation, &error)) << error;
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.cn0_high_dbhz = {{"GPS L1 C/A", 47.0}};
    ASSERT_TRUE(gnss_sim::configure_cn0_model_runtime(config, &interpolation, &error)) << error;
    const double sow = 210000.0;
    const double at_15 = estimate_model(interpolation, gnss_sim::SignalId::kGpsL1Ca, 15.0, sow);
    const double at_30 = estimate_model(interpolation, gnss_sim::SignalId::kGpsL1Ca, 30.0, sow);
    const double at_45 = estimate_model(interpolation, gnss_sim::SignalId::kGpsL1Ca, 45.0, sow);
    EXPECT_NEAR(at_30 - at_15, 3.0, 1e-12);
    EXPECT_NEAR(at_45 - at_30, 3.0, 1e-12);

    const std::string gap_content = std::string(kNormalizedModelHeader) + "\n" +
                                    normalized_model_row("1C", 0.0, 30.0, false, "READY", 3U, "-9.000000") + "\n" +
                                    normalized_model_row("1C", 30.0, 60.0, false, "SPARSE", 1U, "-4.000000") + "\n" +
                                    normalized_model_row("1C", 60.0, 90.0, true, "READY", 3U, "0.000000") + "\n";
    const TemporaryModel gap_file(gap_content);
    gnss_sim::Cn0Model gap{};
    ASSERT_TRUE(gnss_sim::load_cn0_model_csv(gap_file.path().c_str(), 17U, &gap, &error)) << error;
    ASSERT_TRUE(gnss_sim::configure_cn0_model_runtime(config, &gap, &error)) << error;
    const gnss_sim::Cn0Model builtin = gnss_sim::make_builtin_cn0_model(17U);
    EXPECT_DOUBLE_EQ(estimate_model(gap, gnss_sim::SignalId::kGpsL1Ca, 45.0, sow),
                     estimate_model(builtin, gnss_sim::SignalId::kGpsL1Ca, 45.0, sow));
    EXPECT_NE(estimate_model(gap, gnss_sim::SignalId::kGpsL1Ca, 15.0, sow),
              estimate_model(builtin, gnss_sim::SignalId::kGpsL1Ca, 15.0, sow));
}

TEST(Cn0Model, NormalizedReadySignalWithoutReceiverBaselineFailsFast) {
    const std::string content = std::string(kNormalizedModelHeader) + "\n" +
                                normalized_model_row("1C", 0.0, 90.0, true, "READY", 2U, "-2.000000") + "\n" +
                                normalized_model_row("5Q", 0.0, 90.0, true, "READY", 2U, "-1.000000") + "\n";
    const TemporaryModel file(content);
    gnss_sim::Cn0Model model{};
    std::string error;
    ASSERT_TRUE(gnss_sim::load_cn0_model_csv(file.path().c_str(), 31U, &model, &error)) << error;
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.cn0_high_dbhz = {{"GPS L1 C/A", 47.0}};
    EXPECT_FALSE(gnss_sim::configure_cn0_model_runtime(config, &model, &error));
    EXPECT_NE(error.find("GPS L5Q"), std::string::npos);
}

TEST(Cn0Model, MissingSignalAndSparseGapUseExactBuiltinBaselinePolicy) {
    const std::string content = std::string(kModelHeader) + "\n" + model_row(0.0, 30.0, false, "READY", 100, 30.0) +
                                "\n" + model_row(30.0, 60.0, false, "SPARSE", 1, 40.0) + "\n" +
                                model_row(60.0, 90.0, true, "READY", 100, 50.0) + "\n";
    const TemporaryModel file(content);
    gnss_sim::Cn0Model model{};
    std::string error;
    ASSERT_TRUE(gnss_sim::load_cn0_model_csv(file.path().c_str(), 77U, &model, &error)) << error;
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.cn0_high_dbhz = {{"GPS L1 C/A", 54.0}};
    ASSERT_TRUE(gnss_sim::configure_cn0_model_runtime(config, &model, &error)) << error;
    const gnss_sim::Cn0Model builtin = gnss_sim::make_builtin_cn0_model(77U);
    const double sow = 200000.0;

    EXPECT_DOUBLE_EQ(estimate_model(model, gnss_sim::SignalId::kGpsL1Ca, 45.0, sow),
                     estimate_model(builtin, gnss_sim::SignalId::kGpsL1Ca, 45.0, sow));
    EXPECT_DOUBLE_EQ(estimate_model(model, gnss_sim::SignalId::kGpsL2C, 45.0, sow),
                     estimate_model(builtin, gnss_sim::SignalId::kGpsL2C, 45.0, sow));
}

TEST(Cn0Model, ExplicitMalformedModelsFailWithoutFallback) {
    const std::string valid_row = model_row(0.0, 90.0, true, "READY", 100, 40.0);
    const std::string duplicate = std::string(kModelHeader) + "\n" + model_row(0.0, 45.0, false, "READY", 100, 30.0) +
                                  "\n" + model_row(0.0, 45.0, true, "READY", 100, 40.0) + "\n";
    const std::string nonfinite =
        std::string(kModelHeader) +
        "\ngnss-cn0-model-v1,GPS,1C,0,90,1,READY,100,29,29,29,nan,31,31,31,30,1,1,0,,,,INSUFFICIENT_SUPPORT,\n";
    const std::string missing_delta_statistics =
        std::string(kModelHeader) +
        "\ngnss-cn0-model-v1,GPS,1C,0,90,1,READY,100,39,39,39,40,41,41,41,40,1,1,1,,,,INSUFFICIENT_SUPPORT,\n";
    const std::string wrong_semantic =
        std::string(kNormalizedModelHeader) + "\ngnss-cn0-model-v2,ABSOLUTE_STATION_CN0,GPS,1C,0,90,1,READY,2,-1.0\n";
    const std::string cases[] = {std::string("bad-header\n") + valid_row + "\n", duplicate, nonfinite,
                                 missing_delta_statistics, wrong_semantic};

    for (const std::string& content : cases) {
        const TemporaryModel file(content);
        gnss_sim::Cn0Model model{};
        std::string error;
        EXPECT_FALSE(gnss_sim::load_cn0_model_csv(file.path().c_str(), 1U, &model, &error));
        EXPECT_FALSE(error.empty());
    }
}

} // namespace
