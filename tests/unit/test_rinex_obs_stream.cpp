#include "gnss/rtklib_adapter.h"
#include "gnss/satellite_engine.h"
#include "model/receiver_truth.h"
#include "tools/build_cn0_model/rinex_obs_stream.h"

#include <gtest/gtest.h>

extern "C" {
#include <rtklib.h>
}

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

using gnss_sim::cn0_builder::Cn0SampleValidity;
using gnss_sim::cn0_builder::RinexCn0Sample;
using gnss_sim::cn0_builder::RinexObsProvenance;
using gnss_sim::cn0_builder::RinexObsStreamSummary;
using gnss_sim::cn0_builder::SignalStrengthUnitStatus;

std::string acceptance_obs_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/cn0_stream_acceptance_obs.rnx";
}

std::string acceptance_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/multi_gnss_acceptance_nav.rnx";
}

bool collect_samples(const std::string& observation_path, RinexObsProvenance* provenance,
                     RinexObsStreamSummary* summary, std::vector<RinexCn0Sample>* samples, std::string* error) {
    return gnss_sim::cn0_builder::stream_rinex_cn0_samples(
        observation_path, acceptance_nav_path(),
        [&](const RinexCn0Sample& sample) {
            samples->push_back(sample);
            return true;
        },
        provenance, summary, error);
}

std::string header_line(const std::string& value, const std::string& label) {
    std::string line = value;
    if (line.size() < 60) {
        line.append(60 - line.size(), ' ');
    }
    line += label;
    if (line.size() < 80) {
        line.append(80 - line.size(), ' ');
    }
    line += '\n';
    return line;
}

std::string observation_field(double value) {
    std::ostringstream output;
    output << std::setw(14) << std::fixed << std::setprecision(3) << value << "  ";
    return output.str();
}

std::string single_signal_fixture(char system, const std::string& signal_observable, const std::string& satellite,
                                  const std::vector<std::string>& unit_records, const std::vector<std::string>& epochs,
                                  const std::vector<std::string>& fields, const std::string& time_system = "GPS") {
    std::ostringstream output;
    output << header_line("     3.04           OBSERVATION DATA    M", "RINEX VERSION / TYPE");
    output << header_line("GNSS-SIM TEST       OpenAI", "PGM / RUN BY / DATE");
    output << header_line("CN0_UNIT_TEST", "MARKER NAME");
    output << header_line("0001", "MARKER NUMBER");
    output << header_line("  1090852.8702  6186534.0507  1100265.9126", "APPROX POSITION XYZ");
    output << header_line(std::string(1, system) + "    1 " + signal_observable, "SYS / # / OBS TYPES");
    for (const std::string& unit : unit_records) {
        output << header_line(unit, "SIGNAL STRENGTH UNIT");
    }
    output << header_line("  2023    03    14    00    15    00.0000000    " + time_system, "TIME OF FIRST OBS");
    output << header_line("", "END OF HEADER");
    for (std::size_t index = 0; index < epochs.size(); ++index) {
        output << epochs[index] << '\n';
        output << satellite << fields[index] << '\n';
    }
    return output.str();
}

class TemporaryRinexFile {
  public:
    explicit TemporaryRinexFile(const std::string& content) {
        static int sequence = 0;
        path_ = std::filesystem::temp_directory_path() / ("gnss_sim_cn0_stream_" + std::to_string(++sequence) + ".rnx");
        std::ofstream output(path_, std::ios::binary);
        output << content;
    }

    ~TemporaryRinexFile() {
        std::remove(path_.string().c_str());
    }

    std::string path() const {
        return path_.string();
    }

  private:
    std::filesystem::path path_;
};

TEST(RinexObsStream, StreamsAllFrozenV1SignalsIncludingModernBeidouCodes) {
    static_assert(NEXOBS >= 16, "test requires the simulator's multi-signal RTKLIB layout");

    RinexObsProvenance provenance{};
    RinexObsStreamSummary summary{};
    std::vector<RinexCn0Sample> samples;
    std::string error;
    ASSERT_TRUE(collect_samples(acceptance_obs_path(), &provenance, &summary, &samples, &error)) << error;

    EXPECT_EQ(provenance.signal_strength_unit_status, SignalStrengthUnitStatus::kDbHz);
    EXPECT_EQ(provenance.signal_strength_unit, "DBHZ");
    EXPECT_EQ(provenance.station_name, "CN0_ACCEPTANCE");
    EXPECT_EQ(provenance.observation_time_system, "GPS");
    EXPECT_EQ(summary.epochs, 1U);
    EXPECT_EQ(summary.observation_records, 7U);
    EXPECT_EQ(summary.peak_epoch_observations, 7);
    EXPECT_EQ(summary.emitted_samples, 30U);
    EXPECT_EQ(summary.valid_dbhz_samples, 30U);
    EXPECT_EQ(summary.missing_signal_strength, 0U);
    EXPECT_EQ(summary.unsupported_signal_observables, 0U);
    EXPECT_EQ(summary.unmapped_snr_slots, 0U);
    EXPECT_EQ(summary.geometry_failures, 0U);

    std::set<int> signal_ids;
    bool saw_b2a = false;
    bool saw_b2b = false;
    for (const RinexCn0Sample& sample : samples) {
        signal_ids.insert(static_cast<int>(sample.signal_id));
        EXPECT_EQ(sample.validity, Cn0SampleValidity::kValidDbHz);
        EXPECT_TRUE(std::isfinite(sample.cn0_dbhz));
        EXPECT_TRUE(std::isfinite(sample.azimuth_rad));
        EXPECT_TRUE(std::isfinite(sample.elevation_rad));
        EXPECT_EQ(sample.provenance, &provenance);
        if (sample.rinex_signal_code == "5P") {
            saw_b2a = true;
        }
        if (sample.rinex_signal_code == "7D") {
            saw_b2b = true;
        }
    }
    EXPECT_EQ(signal_ids.size(), 21U);
    EXPECT_TRUE(saw_b2a);
    EXPECT_TRUE(saw_b2b);
}

TEST(RinexObsStream, ElevationMatchesSimulatorGeometryToOnePicoradian) {
    RinexObsProvenance provenance{};
    RinexObsStreamSummary summary{};
    std::vector<RinexCn0Sample> samples;
    std::string error;
    ASSERT_TRUE(collect_samples(acceptance_obs_path(), &provenance, &summary, &samples, &error)) << error;
    ASSERT_FALSE(samples.empty());

    gnss_sim::RtklibNavStore* nav = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(nav, nullptr);
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav, acceptance_nav_path().c_str(), &error)) << error;

    gnss_sim::ReceiverTruth receiver{};
    for (int index = 0; index < 3; ++index) {
        receiver.position_ecef_m[index] = provenance.station_ecef_m[index];
        receiver.velocity_ecef_mps[index] = 0.0;
    }
    ASSERT_TRUE(gnss_sim::rtklib_ecef_to_llh(receiver.position_ecef_m, &receiver.latitude_deg, &receiver.longitude_deg,
                                             &receiver.height_m));

    gnss_sim::SatelliteGeometry geometry{};
    ASSERT_TRUE(gnss_sim::compute_satellite_geometry(nav, receiver, samples.front().time,
                                                     samples.front().satellite_number, -90.0, &geometry, &error))
        << error;
    EXPECT_NEAR(samples.front().azimuth_rad, geometry.azimuth_rad, 1e-12);
    EXPECT_NEAR(samples.front().elevation_rad, geometry.elevation_rad, 1e-12);
    gnss_sim::destroy_rtklib_nav_store(nav);
}

TEST(RinexObsStream, FixedInputsProduceIdenticalSamples) {
    RinexObsProvenance first_provenance{};
    RinexObsStreamSummary first_summary{};
    std::vector<RinexCn0Sample> first;
    std::string error;
    ASSERT_TRUE(collect_samples(acceptance_obs_path(), &first_provenance, &first_summary, &first, &error)) << error;

    RinexObsProvenance second_provenance{};
    RinexObsStreamSummary second_summary{};
    std::vector<RinexCn0Sample> second;
    ASSERT_TRUE(collect_samples(acceptance_obs_path(), &second_provenance, &second_summary, &second, &error)) << error;
    ASSERT_EQ(first.size(), second.size());
    ASSERT_EQ(first_summary.emitted_samples, second_summary.emitted_samples);

    for (std::size_t index = 0; index < first.size(); ++index) {
        EXPECT_EQ(first[index].time.gps_week, second[index].time.gps_week);
        EXPECT_EQ(first[index].time.tow_ns, second[index].time.tow_ns);
        EXPECT_EQ(first[index].satellite_number, second[index].satellite_number);
        EXPECT_EQ(first[index].signal_id, second[index].signal_id);
        EXPECT_EQ(first[index].rinex_signal_code, second[index].rinex_signal_code);
        EXPECT_DOUBLE_EQ(first[index].signal_strength_value, second[index].signal_strength_value);
        EXPECT_DOUBLE_EQ(first[index].cn0_dbhz, second[index].cn0_dbhz);
        EXPECT_DOUBLE_EQ(first[index].azimuth_rad, second[index].azimuth_rad);
        EXPECT_DOUBLE_EQ(first[index].elevation_rad, second[index].elevation_rad);
        EXPECT_EQ(first[index].validity, second[index].validity);
    }
}

TEST(RinexObsStream, DecodesModernBeidouSignalStrengthWithoutCodeObservable) {
    const TemporaryRinexFile file(single_signal_fixture(
        'C', "S5P", "C01", {"DBHZ"}, {"> 2023 03 14 00 15  0.0000000  0  1"}, {observation_field(44.0)}));

    RinexObsProvenance provenance{};
    RinexObsStreamSummary summary{};
    std::vector<RinexCn0Sample> samples;
    std::string error;
    ASSERT_TRUE(collect_samples(file.path(), &provenance, &summary, &samples, &error)) << error;
    ASSERT_EQ(samples.size(), 1U);
    EXPECT_EQ(samples.front().rinex_signal_code, "5P");
    EXPECT_DOUBLE_EQ(samples.front().cn0_dbhz, 44.0);
    EXPECT_EQ(summary.unmapped_snr_slots, 0U);
}

TEST(RinexObsStream, MissingOrInvalidSignalStrengthIsSkippedDeterministically) {
    const TemporaryRinexFile file(single_signal_fixture('G', "S1C", "G01", {"DBHZ"},
                                                        {"> 2023 03 14 00 15  0.0000000  0  1"}, {"      invalid   "}));

    RinexObsProvenance provenance{};
    RinexObsStreamSummary summary{};
    std::vector<RinexCn0Sample> samples;
    std::string error;
    ASSERT_TRUE(collect_samples(file.path(), &provenance, &summary, &samples, &error)) << error;
    EXPECT_TRUE(samples.empty());
    EXPECT_EQ(summary.missing_signal_strength, 1U);
}

TEST(RinexObsStream, MissingUnsupportedAndConflictingUnitsRemainAmbiguous) {
    const std::vector<std::vector<std::string>> units = {{}, {"VOLT"}, {"DBHZ", "VOLT"}};
    const SignalStrengthUnitStatus statuses[] = {SignalStrengthUnitStatus::kMissing,
                                                 SignalStrengthUnitStatus::kUnsupported,
                                                 SignalStrengthUnitStatus::kConflicting};

    for (std::size_t index = 0; index < units.size(); ++index) {
        const TemporaryRinexFile file(single_signal_fixture(
            'G', "S1C", "G01", units[index], {"> 2023 03 14 00 15  0.0000000  0  1"}, {observation_field(38.0)}));
        RinexObsProvenance provenance{};
        RinexObsStreamSummary summary{};
        std::vector<RinexCn0Sample> samples;
        std::string error;
        ASSERT_TRUE(collect_samples(file.path(), &provenance, &summary, &samples, &error)) << error;
        ASSERT_EQ(samples.size(), 1U);
        EXPECT_EQ(provenance.signal_strength_unit_status, statuses[index]);
        EXPECT_EQ(samples.front().validity, Cn0SampleValidity::kAmbiguousSignalStrengthUnit);
        EXPECT_FALSE(std::isfinite(samples.front().cn0_dbhz));
        EXPECT_EQ(summary.ambiguous_unit_samples, 1U);
    }
}

TEST(RinexObsStream, UnsupportedSignalMappingIsReportedWithoutFallback) {
    const TemporaryRinexFile file(single_signal_fixture(
        'G', "S1W", "G01", {"DBHZ"}, {"> 2023 03 14 00 15  0.0000000  0  1"}, {observation_field(35.0)}));

    RinexObsProvenance provenance{};
    RinexObsStreamSummary summary{};
    std::vector<RinexCn0Sample> samples;
    std::string error;
    ASSERT_TRUE(collect_samples(file.path(), &provenance, &summary, &samples, &error)) << error;
    EXPECT_TRUE(samples.empty());
    ASSERT_EQ(summary.unsupported_observables.size(), 1U);
    EXPECT_EQ(summary.unsupported_observables.front(), "G:S1W");
    EXPECT_EQ(summary.unsupported_signal_observables, 1U);
    EXPECT_EQ(summary.unmapped_snr_slots, 1U);
}

TEST(RinexObsStream, RejectsOutOfOrderEpochs) {
    const TemporaryRinexFile file(single_signal_fixture(
        'G', "S1C", "G01", {"DBHZ"}, {"> 2023 03 14 00 15  1.0000000  0  1", "> 2023 03 14 00 15  0.0000000  0  1"},
        {observation_field(38.0), observation_field(39.0)}));

    RinexObsProvenance provenance{};
    RinexObsStreamSummary summary{};
    std::vector<RinexCn0Sample> samples;
    std::string error;
    EXPECT_FALSE(collect_samples(file.path(), &provenance, &summary, &samples, &error));
    EXPECT_EQ(summary.out_of_order_epochs, 1U);
    EXPECT_NE(error.find("out of order"), std::string::npos);
}

TEST(RinexObsStream, ConvertsBeidouTimeAcrossGpsWeekBoundary) {
    const TemporaryRinexFile file(single_signal_fixture(
        'C', "S2I", "C01", {"DBHZ"}, {"> 2023 03 11 23 59 50.0000000  0  1"}, {observation_field(37.0)}, "BDT"));

    RinexObsProvenance provenance{};
    RinexObsStreamSummary summary{};
    std::vector<RinexCn0Sample> samples;
    std::string error;
    ASSERT_TRUE(collect_samples(file.path(), &provenance, &summary, &samples, &error)) << error;
    ASSERT_EQ(samples.size(), 1U);
    EXPECT_EQ(provenance.observation_time_system, "BDT");
    EXPECT_EQ(samples.front().time.gps_week, 2253);
    EXPECT_EQ(samples.front().time.tow_ns, 4000000000LL);
    EXPECT_DOUBLE_EQ(samples.front().cn0_dbhz, 37.0);
    EXPECT_EQ(samples.front().validity, Cn0SampleValidity::kGeometryUnavailable);
}

} // namespace
