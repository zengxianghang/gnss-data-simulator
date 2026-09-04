#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"
#include "model/carrier_tracking.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kTruthLatitudeDeg = 20.0;
constexpr double kTruthLongitudeDeg = 120.0;
constexpr double kTruthHeightM = 100.0;
constexpr std::int64_t kStaticDurationSeconds = 12;
constexpr const char* kFilteredNavName = "brd400dlr_beidou_verbatim_nav.rnx";

std::string source_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

gnss_sim::SimTime start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 436500.0, &time));
    return time;
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

std::vector<std::string> parse_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for (std::size_t index = 0; index < line.size(); ++index) {
        const char character = line[index];
        if (quoted) {
            if (character == '"' && index + 1U < line.size() && line[index + 1U] == '"') {
                field.push_back('"');
                ++index;
            } else if (character == '"') {
                quoted = false;
            } else {
                field.push_back(character);
            }
        } else if (character == '"') {
            quoted = true;
        } else if (character == ',') {
            fields.push_back(field);
            field.clear();
        } else {
            field.push_back(character);
        }
    }
    fields.push_back(field);
    return fields;
}

std::vector<std::vector<std::string>> read_csv(const std::filesystem::path& path) {
    std::vector<std::vector<std::string>> rows;
    std::ifstream input(path, std::ios::binary);
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        rows.push_back(parse_csv_line(line));
    }
    return rows;
}

int column_index(const std::vector<std::string>& header, const char* name) {
    for (std::size_t index = 0; index < header.size(); ++index) {
        if (header[index] == name) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

std::string observation_key(int week, std::int64_t tow_ns, int satellite_number, int signal_id) {
    return std::to_string(week) + ":" + std::to_string(tow_ns) + ":" + std::to_string(satellite_number) + ":" +
           std::to_string(signal_id);
}

std::string signal_key(int satellite_number, int signal_id) {
    return std::to_string(satellite_number) + ":" + std::to_string(signal_id);
}

bool materialize_verbatim_beidou_nav(const std::filesystem::path& destination, std::string* error_message) {
    const std::string source = read_file(source_nav_path());
    if (source.empty() || source.find('\r') != std::string::npos) {
        if (error_message != nullptr) {
            *error_message = "BRD400DLR source fixture is empty or is not LF-normalized";
        }
        return false;
    }
    std::istringstream input(source);
    std::ofstream output(destination, std::ios::binary | std::ios::trunc);
    if (!output) {
        if (error_message != nullptr) {
            *error_message = "cannot create verbatim BeiDou NAV subset";
        }
        return false;
    }
    bool header = true;
    bool keep_record = false;
    bool saw_header_end = false;
    int ephemeris_records = 0;
    std::set<std::string> satellites;
    std::string line;
    while (std::getline(input, line)) {
        if (header) {
            output << line << '\n';
            if (line.find("END OF HEADER") != std::string::npos) {
                header = false;
                saw_header_end = true;
            }
            continue;
        }
        if (line.rfind("> ", 0) == 0) {
            keep_record = line.rfind("> EPH C", 0) == 0;
            if (keep_record) {
                ++ephemeris_records;
                if (line.size() >= 9U) {
                    satellites.insert(line.substr(6, 3));
                }
            }
        }
        if (keep_record) {
            output << line << '\n';
        }
    }
    output.close();
    if (!saw_header_end || ephemeris_records < 4 || satellites.size() < 4U || !output) {
        if (error_message != nullptr) {
            *error_message = "verbatim BeiDou NAV subset does not contain enough authentic ephemerides";
        }
        return false;
    }
    return true;
}

gnss_sim::SimConfig static_config(int sampling_rate_hz, bool carrier_tracking_enabled) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::KS;
    config.duration_ns = kStaticDurationSeconds * gnss_sim::NANOSECONDS_PER_SECOND;
    config.sampling_rate_hz = sampling_rate_hz;
    config.elevation_mask_deg = 0.0;
    config.solution_elevation_mask_deg = 5.0;
    config.output_eph = false;
    config.output_ion = false;
    config.measurement_noise_enabled = false;
    config.multipath_enabled = true;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    config.receiver = {kTruthLatitudeDeg, kTruthLongitudeDeg, kTruthHeightM};
    config.seed = 0x163U;
    config.carrier_tracking.enabled = carrier_tracking_enabled;
    return config;
}

gnss_sim::SimConfig reacquisition_config() {
    gnss_sim::SimConfig config = static_config(10, true);
    config.scenario = gnss_sim::ScenarioType::REA;
    config.rea.signal_on_ns = 6LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_off_ns = 2LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.duration_ns = 14LL * gnss_sim::NANOSECONDS_PER_SECOND;
    return config;
}

bool run_in_directory(const std::filesystem::path& directory, const gnss_sim::SimConfig& config,
                      gnss_sim::SimulatorRunSummary* summary, std::string* error_message) {
    std::error_code filesystem_error;
    std::filesystem::remove_all(directory, filesystem_error);
    filesystem_error.clear();
    if (!std::filesystem::create_directories(directory, filesystem_error) || filesystem_error) {
        if (error_message != nullptr) {
            *error_message = "cannot create carrier validation directory";
        }
        return false;
    }
    const std::filesystem::path nav_path = directory / kFilteredNavName;
    if (!materialize_verbatim_beidou_nav(nav_path, error_message)) {
        return false;
    }
    const std::string input_path = nav_path.string();
    const std::string output_path = (directory / "simulated.log").string();
    const gnss_sim::SimulatorRunOptions options{input_path.c_str(), output_path.c_str(), start_time()};
    return gnss_sim::run_simulator(config, options, summary, error_message);
}

void cleanup(const std::filesystem::path& directory) {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
}

struct DistributionMetrics {
    std::uint64_t count = 0;
    double rms = 0.0;
    double p50 = 0.0;
    double p95 = 0.0;
    double p99 = 0.0;
    double maximum = 0.0;
};

double percentile(std::vector<double> values, double probability) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    std::size_t rank = static_cast<std::size_t>(std::ceil(probability * static_cast<double>(values.size())));
    rank = std::max<std::size_t>(1U, std::min(rank, values.size()));
    return values[rank - 1U];
}

DistributionMetrics metrics(const std::vector<double>& values) {
    DistributionMetrics result{};
    result.count = static_cast<std::uint64_t>(values.size());
    if (values.empty()) {
        return result;
    }
    double sum_squares = 0.0;
    std::vector<double> absolute;
    absolute.reserve(values.size());
    for (double value : values) {
        sum_squares += value * value;
        absolute.push_back(std::abs(value));
        result.maximum = std::max(result.maximum, std::abs(value));
    }
    result.rms = std::sqrt(sum_squares / static_cast<double>(values.size()));
    result.p50 = percentile(absolute, 0.50);
    result.p95 = percentile(absolute, 0.95);
    result.p99 = percentile(absolute, 0.99);
    return result;
}

struct CarrierSample {
    int week = 0;
    std::int64_t tow_ns = 0;
    int satellite_number = 0;
    int signal_id = 0;
    std::string signal_name;
    std::string acquisition_context;
    std::string reset_reason;
    std::string mode;
    std::string fll_phase;
    bool result_available = false;
    double effective_cn0_dbhz = 0.0;
    double coherent_integration_sec = 0.0;
    double active_bandwidth_hz = 0.0;
    double sigma_hz = 0.0;
    double sigma_mps = 0.0;
    double tracking_error_hz = 0.0;
    double tracking_error_mps = 0.0;
    double carrier_lock_age_sec = 0.0;
    double pll_age_sec = 0.0;
    bool mode_changed = false;
    bool new_carrier_segment = false;
    bool cycle_slip_event = false;
    bool environmental_range_rate_valid = false;
    double environmental_range_rate_mps = 0.0;
    bool physical_snapshot_available = false;
    double physical_range_rate_mps = 0.0;
    double physical_doppler_hz = 0.0;
    bool post_snapshot_available = false;
    bool post_code_valid = false;
    bool post_doppler_valid = false;
    bool post_adr_valid = false;
    double post_range_rate_mps = 0.0;
    double post_doppler_hz = 0.0;
    double post_adr_cycles = 0.0;
};

bool parse_carrier_samples(const std::filesystem::path& path, std::vector<CarrierSample>* output) {
    if (output == nullptr) {
        return false;
    }
    const auto rows = read_csv(path);
    if (rows.size() <= 1U) {
        return false;
    }
    const auto& header = rows.front();
    const int week = column_index(header, "gps_week");
    const int tow = column_index(header, "tow_ns");
    const int satellite = column_index(header, "satellite_number");
    const int signal = column_index(header, "signal_id");
    const int signal_name = column_index(header, "signal_name");
    const int acquisition = column_index(header, "acquisition_context");
    const int result_available = column_index(header, "carrier_result_available");
    const int reset_reason = column_index(header, "carrier_reset_reason");
    const int coherent = column_index(header, "coherent_integration_sec");
    const int cn0 = column_index(header, "effective_cn0_dbhz");
    const int mode = column_index(header, "carrier_mode");
    const int fll_phase = column_index(header, "fll_phase");
    const int bandwidth = column_index(header, "active_bandwidth_hz");
    const int sigma_hz = column_index(header, "sigma_hz");
    const int sigma_mps = column_index(header, "sigma_mps");
    const int error_hz = column_index(header, "tracking_error_hz");
    const int error_mps = column_index(header, "tracking_error_mps");
    const int carrier_age = column_index(header, "carrier_lock_age_sec");
    const int pll_age = column_index(header, "pll_age_sec");
    const int mode_changed = column_index(header, "mode_changed");
    const int new_segment = column_index(header, "new_carrier_segment");
    const int slip = column_index(header, "cycle_slip_event");
    const int environmental_valid = column_index(header, "environmental_range_rate_valid");
    const int environmental_rate = column_index(header, "environmental_range_rate_mps");
    const int physical_snapshot = column_index(header, "physical_snapshot_available");
    const int physical_rate = column_index(header, "physical_range_rate_mps");
    const int physical_doppler = column_index(header, "physical_doppler_hz");
    const int post_snapshot = column_index(header, "post_carrier_snapshot_available");
    const int post_code = column_index(header, "post_carrier_code_valid");
    const int post_doppler_valid = column_index(header, "post_carrier_doppler_valid");
    const int post_adr_valid = column_index(header, "post_carrier_adr_valid");
    const int post_rate = column_index(header, "post_carrier_range_rate_mps");
    const int post_doppler = column_index(header, "post_carrier_doppler_hz");
    const int post_adr = column_index(header, "post_carrier_adr_cycles");
    if (week < 0 || tow < 0 || satellite < 0 || signal < 0 || signal_name < 0 || acquisition < 0 ||
        result_available < 0 || reset_reason < 0 || coherent < 0 || cn0 < 0 || mode < 0 || fll_phase < 0 ||
        bandwidth < 0 || sigma_hz < 0 || sigma_mps < 0 || error_hz < 0 || error_mps < 0 || carrier_age < 0 ||
        pll_age < 0 || mode_changed < 0 || new_segment < 0 || slip < 0 || environmental_valid < 0 ||
        environmental_rate < 0 || physical_snapshot < 0 || physical_rate < 0 || physical_doppler < 0 ||
        post_snapshot < 0 || post_code < 0 || post_doppler_valid < 0 || post_adr_valid < 0 || post_rate < 0 ||
        post_doppler < 0 || post_adr < 0) {
        return false;
    }

    std::vector<CarrierSample> samples;
    samples.reserve(rows.size() - 1U);
    for (std::size_t index = 1; index < rows.size(); ++index) {
        const auto& row = rows[index];
        if (row.size() != header.size()) {
            return false;
        }
        CarrierSample sample{};
        sample.week = std::stoi(row[static_cast<std::size_t>(week)]);
        sample.tow_ns = std::stoll(row[static_cast<std::size_t>(tow)]);
        sample.satellite_number = std::stoi(row[static_cast<std::size_t>(satellite)]);
        sample.signal_id = std::stoi(row[static_cast<std::size_t>(signal)]);
        sample.signal_name = row[static_cast<std::size_t>(signal_name)];
        sample.acquisition_context = row[static_cast<std::size_t>(acquisition)];
        sample.result_available = row[static_cast<std::size_t>(result_available)] == "1";
        sample.reset_reason = row[static_cast<std::size_t>(reset_reason)];
        sample.coherent_integration_sec = std::stod(row[static_cast<std::size_t>(coherent)]);
        sample.environmental_range_rate_valid = row[static_cast<std::size_t>(environmental_valid)] == "1";
        if (sample.environmental_range_rate_valid && !row[static_cast<std::size_t>(environmental_rate)].empty()) {
            sample.environmental_range_rate_mps = std::stod(row[static_cast<std::size_t>(environmental_rate)]);
        }
        sample.physical_snapshot_available = row[static_cast<std::size_t>(physical_snapshot)] == "1";
        if (sample.physical_snapshot_available) {
            sample.physical_range_rate_mps = std::stod(row[static_cast<std::size_t>(physical_rate)]);
            sample.physical_doppler_hz = std::stod(row[static_cast<std::size_t>(physical_doppler)]);
        }
        sample.post_snapshot_available = row[static_cast<std::size_t>(post_snapshot)] == "1";
        if (sample.post_snapshot_available) {
            sample.post_code_valid = row[static_cast<std::size_t>(post_code)] == "1";
            sample.post_doppler_valid = row[static_cast<std::size_t>(post_doppler_valid)] == "1";
            sample.post_adr_valid = row[static_cast<std::size_t>(post_adr_valid)] == "1";
            sample.post_range_rate_mps = std::stod(row[static_cast<std::size_t>(post_rate)]);
            sample.post_doppler_hz = std::stod(row[static_cast<std::size_t>(post_doppler)]);
            sample.post_adr_cycles = std::stod(row[static_cast<std::size_t>(post_adr)]);
        }
        if (sample.result_available) {
            sample.effective_cn0_dbhz = std::stod(row[static_cast<std::size_t>(cn0)]);
            sample.mode = row[static_cast<std::size_t>(mode)];
            sample.fll_phase = row[static_cast<std::size_t>(fll_phase)];
            sample.active_bandwidth_hz = std::stod(row[static_cast<std::size_t>(bandwidth)]);
            sample.sigma_hz = std::stod(row[static_cast<std::size_t>(sigma_hz)]);
            sample.sigma_mps = std::stod(row[static_cast<std::size_t>(sigma_mps)]);
            sample.tracking_error_hz = std::stod(row[static_cast<std::size_t>(error_hz)]);
            sample.tracking_error_mps = std::stod(row[static_cast<std::size_t>(error_mps)]);
            sample.carrier_lock_age_sec = std::stod(row[static_cast<std::size_t>(carrier_age)]);
            sample.pll_age_sec = std::stod(row[static_cast<std::size_t>(pll_age)]);
            sample.mode_changed = row[static_cast<std::size_t>(mode_changed)] == "1";
            sample.new_carrier_segment = row[static_cast<std::size_t>(new_segment)] == "1";
            sample.cycle_slip_event = row[static_cast<std::size_t>(slip)] == "1";
        }
        samples.push_back(sample);
    }
    *output = samples;
    return true;
}

struct UrbanContext {
    bool range_rate_valid = false;
    double range_rate_mps = 0.0;
    std::string state;
    std::string wall;
    int path_count = 0;
    int reflection_count = 0;
};

using UrbanContextMap = std::map<std::string, UrbanContext>;

bool parse_urban_contexts(const std::filesystem::path& path, UrbanContextMap* output) {
    if (output == nullptr) {
        return false;
    }
    const auto rows = read_csv(path);
    if (rows.size() <= 1U) {
        return false;
    }
    const auto& header = rows.front();
    const int week = column_index(header, "gps_week");
    const int tow = column_index(header, "tow_ns");
    const int satellite = column_index(header, "satellite_number");
    const int signal = column_index(header, "signal_id");
    const int valid = column_index(header, "environmental_range_rate_valid");
    const int rate = column_index(header, "environmental_range_rate_mps");
    const int state = column_index(header, "urban_state");
    const int wall = column_index(header, "blocking_wall");
    const int paths = column_index(header, "received_path_count");
    const int reflections = column_index(header, "reflection_count");
    if (week < 0 || tow < 0 || satellite < 0 || signal < 0 || valid < 0 || rate < 0 || state < 0 || wall < 0 ||
        paths < 0 || reflections < 0) {
        return false;
    }
    UrbanContextMap result;
    for (std::size_t index = 1; index < rows.size(); ++index) {
        const auto& row = rows[index];
        if (row.size() != header.size()) {
            return false;
        }
        const int gps_week = std::stoi(row[static_cast<std::size_t>(week)]);
        const std::int64_t tow_ns = std::stoll(row[static_cast<std::size_t>(tow)]);
        const int satellite_number = std::stoi(row[static_cast<std::size_t>(satellite)]);
        const int signal_id = std::stoi(row[static_cast<std::size_t>(signal)]);
        UrbanContext context{};
        context.range_rate_valid = row[static_cast<std::size_t>(valid)] == "1";
        if (!row[static_cast<std::size_t>(rate)].empty()) {
            context.range_rate_mps = std::stod(row[static_cast<std::size_t>(rate)]);
        }
        context.state = row[static_cast<std::size_t>(state)];
        context.wall = row[static_cast<std::size_t>(wall)];
        context.path_count = std::stoi(row[static_cast<std::size_t>(paths)]);
        context.reflection_count = std::stoi(row[static_cast<std::size_t>(reflections)]);
        result[observation_key(gps_week, tow_ns, satellite_number, signal_id)] = context;
    }
    *output = result;
    return true;
}

struct VelocityStats {
    std::vector<double> error_3d_mps;
    std::vector<double> clock_drift_error_mps;
};

bool parse_velocity_stats(const std::filesystem::path& path, double truth_clock_drift_mps, VelocityStats* output) {
    if (output == nullptr) {
        return false;
    }
    const auto rows = read_csv(path);
    if (rows.size() <= 1U) {
        return false;
    }
    const auto& header = rows.front();
    const int valid = column_index(header, "velocity_valid");
    const int vx = column_index(header, "velocity_x_mps");
    const int vy = column_index(header, "velocity_y_mps");
    const int vz = column_index(header, "velocity_z_mps");
    const int clock_drift = column_index(header, "receiver_clock_drift_mps");
    if (valid < 0 || vx < 0 || vy < 0 || vz < 0 || clock_drift < 0) {
        return false;
    }
    VelocityStats result;
    for (std::size_t index = 1; index < rows.size(); ++index) {
        const auto& row = rows[index];
        if (row.size() != header.size() || row[static_cast<std::size_t>(valid)] != "1") {
            continue;
        }
        const double x = std::stod(row[static_cast<std::size_t>(vx)]);
        const double y = std::stod(row[static_cast<std::size_t>(vy)]);
        const double z = std::stod(row[static_cast<std::size_t>(vz)]);
        result.error_3d_mps.push_back(std::sqrt(x * x + y * y + z * z));
        result.clock_drift_error_mps.push_back(std::stod(row[static_cast<std::size_t>(clock_drift)]) -
                                               truth_clock_drift_mps);
    }
    *output = result;
    return true;
}

const char* cn0_bin(double cn0_dbhz) {
    if (cn0_dbhz < 18.0) {
        return "LT18";
    }
    if (cn0_dbhz < 22.0) {
        return "18_22";
    }
    if (cn0_dbhz < 27.0) {
        return "22_27";
    }
    if (cn0_dbhz < 30.0) {
        return "27_30";
    }
    return "GE30";
}

struct TrackingDistribution {
    std::uint64_t rows = 0;
    std::uint64_t valid_doppler_rows = 0;
    std::vector<double> error_hz;
    std::vector<double> error_mps;
    std::vector<double> sigma_hz;
    std::vector<double> normalized_error;
};

void print_tracking_distribution(const char* category, const std::string& name,
                                 const TrackingDistribution& distribution) {
    const DistributionMetrics error_hz = metrics(distribution.error_hz);
    const DistributionMetrics error_mps = metrics(distribution.error_mps);
    const DistributionMetrics sigma_hz = metrics(distribution.sigma_hz);
    const DistributionMetrics normalized = metrics(distribution.normalized_error);
    std::cout << "CARRIER_AUTH_" << category << " name=" << name << " rows=" << distribution.rows
              << " valid=" << distribution.valid_doppler_rows << " error_rms_hz=" << error_hz.rms
              << " error_p50_hz=" << error_hz.p50 << " error_p95_hz=" << error_hz.p95
              << " error_p99_hz=" << error_hz.p99 << " error_max_hz=" << error_hz.maximum
              << " error_rms_mps=" << error_mps.rms << " error_p95_mps=" << error_mps.p95
              << " error_max_mps=" << error_mps.maximum << " sigma_rms_hz=" << sigma_hz.rms
              << " normalized_error_rms=" << normalized.rms << '\n';
}

TEST(CarrierTrackingAuthenticValidation, AuthenticNavObservationStatisticsCompatibilityAndVelocity) {
    const std::filesystem::path enabled_a_directory = "gnss_sim_carrier_auth_enabled_a";
    const std::filesystem::path enabled_b_directory = "gnss_sim_carrier_auth_enabled_b";
    const std::filesystem::path disabled_a_directory = "gnss_sim_carrier_auth_disabled_a";
    const std::filesystem::path disabled_b_directory = "gnss_sim_carrier_auth_disabled_b";
    const gnss_sim::SimConfig enabled = static_config(10, true);
    const gnss_sim::SimConfig disabled_a = static_config(10, false);
    gnss_sim::SimConfig disabled_b = disabled_a;
    disabled_b.carrier_tracking.pll_noise_bandwidth_hz = 6.0;
    disabled_b.carrier_tracking.fll_noise_bandwidth_hz = 5.0;
    disabled_b.carrier_tracking.fll_pull_in_bandwidth_hz = 9.0;

    gnss_sim::SimulatorRunSummary enabled_a_summary{};
    gnss_sim::SimulatorRunSummary enabled_b_summary{};
    gnss_sim::SimulatorRunSummary disabled_a_summary{};
    gnss_sim::SimulatorRunSummary disabled_b_summary{};
    std::string error_message;
    ASSERT_TRUE(run_in_directory(enabled_a_directory, enabled, &enabled_a_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_in_directory(enabled_b_directory, enabled, &enabled_b_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_in_directory(disabled_a_directory, disabled_a, &disabled_a_summary, &error_message))
        << error_message;
    ASSERT_TRUE(run_in_directory(disabled_b_directory, disabled_b, &disabled_b_summary, &error_message))
        << error_message;

    EXPECT_EQ(read_file(enabled_a_directory / "simulated.log"), read_file(enabled_b_directory / "simulated.log"));
    EXPECT_EQ(read_file(enabled_a_directory / "carrier_tracking_truth.csv"),
              read_file(enabled_b_directory / "carrier_tracking_truth.csv"));
    EXPECT_EQ(read_file(disabled_a_directory / "simulated.log"), read_file(disabled_b_directory / "simulated.log"));
    EXPECT_EQ(read_file(disabled_a_directory / "observation_truth.csv"),
              read_file(disabled_b_directory / "observation_truth.csv"));
    EXPECT_EQ(read_file(disabled_a_directory / "urban_signal_truth.csv"),
              read_file(disabled_b_directory / "urban_signal_truth.csv"));
    EXPECT_EQ(read_file(enabled_a_directory / "observation_truth.csv"),
              read_file(disabled_a_directory / "observation_truth.csv"));
    EXPECT_EQ(read_file(enabled_a_directory / "urban_signal_truth.csv"),
              read_file(disabled_a_directory / "urban_signal_truth.csv"));
    EXPECT_EQ(read_file(enabled_a_directory / "simulated.log").find("carrier_mode"), std::string::npos);
    EXPECT_EQ(read_file(enabled_a_directory / "simulated.log").find("tracking_error_hz"), std::string::npos);

    std::vector<CarrierSample> samples;
    UrbanContextMap urban_contexts;
    ASSERT_TRUE(parse_carrier_samples(enabled_a_directory / "carrier_tracking_truth.csv", &samples));
    ASSERT_TRUE(parse_urban_contexts(enabled_a_directory / "urban_signal_truth.csv", &urban_contexts));

    std::map<std::string, TrackingDistribution> modes;
    std::map<std::string, TrackingDistribution> cn0_bins;
    std::map<std::string, std::uint64_t> validity_combinations;
    std::vector<double> environmental_rates;
    std::uint64_t result_rows = 0;
    std::uint64_t valid_doppler_rows = 0;
    std::uint64_t fll_code_doppler_no_adr = 0;
    std::uint64_t unlocked_code_only = 0;
    std::uint64_t pull_in_rows = 0;
    std::uint64_t mode_changes = 0;
    std::uint64_t new_segments = 0;
    std::uint64_t cycle_slips = 0;
    double max_doppler_mapping_mismatch_hz = 0.0;
    double max_range_rate_mapping_mismatch_mps = 0.0;
    double max_environmental_truth_mismatch_mps = 0.0;
    double worst_error_mps = 0.0;
    CarrierSample worst_sample{};
    UrbanContext worst_context{};

    for (const CarrierSample& sample : samples) {
        const std::string key = observation_key(sample.week, sample.tow_ns, sample.satellite_number, sample.signal_id);
        if (sample.environmental_range_rate_valid) {
            environmental_rates.push_back(sample.environmental_range_rate_mps);
            const auto urban = urban_contexts.find(key);
            ASSERT_NE(urban, urban_contexts.end());
            EXPECT_TRUE(urban->second.range_rate_valid);
            max_environmental_truth_mismatch_mps =
                std::max(max_environmental_truth_mismatch_mps,
                         std::abs(sample.environmental_range_rate_mps - urban->second.range_rate_mps));
        }
        if (!sample.result_available) {
            continue;
        }
        ++result_rows;
        TrackingDistribution& mode_distribution = modes[sample.mode];
        TrackingDistribution& bin_distribution = cn0_bins[cn0_bin(sample.effective_cn0_dbhz)];
        ++mode_distribution.rows;
        ++bin_distribution.rows;
        if (sample.mode_changed) {
            ++mode_changes;
        }
        if (sample.new_carrier_segment) {
            ++new_segments;
        }
        if (sample.cycle_slip_event) {
            ++cycle_slips;
        }
        if (sample.fll_phase == "PULL_IN") {
            ++pull_in_rows;
        }
        if (sample.post_snapshot_available) {
            const std::string validity = std::string(sample.post_code_valid ? "1" : "0") +
                                         (sample.post_doppler_valid ? "1" : "0") +
                                         (sample.post_adr_valid ? "1" : "0");
            ++validity_combinations[validity];
            if (sample.mode == "FLL_TRACK" && sample.post_code_valid && sample.post_doppler_valid &&
                !sample.post_adr_valid) {
                ++fll_code_doppler_no_adr;
            }
            if (sample.mode == "CARRIER_UNLOCKED" && sample.post_code_valid && !sample.post_doppler_valid &&
                !sample.post_adr_valid) {
                ++unlocked_code_only;
            }
        }
        if (sample.physical_snapshot_available && sample.post_snapshot_available) {
            max_doppler_mapping_mismatch_hz =
                std::max(max_doppler_mapping_mismatch_hz,
                         std::abs((sample.post_doppler_hz - sample.physical_doppler_hz) - sample.tracking_error_hz));
            max_range_rate_mapping_mismatch_mps =
                std::max(max_range_rate_mapping_mismatch_mps,
                         std::abs((sample.post_range_rate_mps - sample.physical_range_rate_mps) +
                                  sample.tracking_error_mps));
        }
        if (!sample.post_doppler_valid) {
            continue;
        }
        ++valid_doppler_rows;
        ++mode_distribution.valid_doppler_rows;
        ++bin_distribution.valid_doppler_rows;
        mode_distribution.error_hz.push_back(sample.tracking_error_hz);
        mode_distribution.error_mps.push_back(sample.tracking_error_mps);
        mode_distribution.sigma_hz.push_back(sample.sigma_hz);
        bin_distribution.error_hz.push_back(sample.tracking_error_hz);
        bin_distribution.error_mps.push_back(sample.tracking_error_mps);
        bin_distribution.sigma_hz.push_back(sample.sigma_hz);
        if (sample.sigma_hz > 0.0) {
            mode_distribution.normalized_error.push_back(sample.tracking_error_hz / sample.sigma_hz);
            bin_distribution.normalized_error.push_back(sample.tracking_error_hz / sample.sigma_hz);
        }
        if (std::abs(sample.tracking_error_mps) >= worst_error_mps) {
            worst_error_mps = std::abs(sample.tracking_error_mps);
            worst_sample = sample;
            const auto urban = urban_contexts.find(key);
            if (urban != urban_contexts.end()) {
                worst_context = urban->second;
            }
        }
    }

    const DistributionMetrics environmental = metrics(environmental_rates);
    std::cout << "CARRIER_AUTH_SUMMARY result_rows=" << result_rows << " valid_doppler_rows=" << valid_doppler_rows
              << " fll_code_doppler_no_adr=" << fll_code_doppler_no_adr
              << " unlocked_code_only=" << unlocked_code_only << " pull_in_rows=" << pull_in_rows
              << " mode_changes=" << mode_changes << " new_segments=" << new_segments << " cycle_slips=" << cycle_slips
              << " env_rate_rms_mps=" << environmental.rms << " env_rate_p95_mps=" << environmental.p95
              << " env_rate_max_mps=" << environmental.maximum << '\n'
              << "CARRIER_AUTH_MAPPING max_doppler_mismatch_hz=" << max_doppler_mapping_mismatch_hz
              << " max_range_rate_mismatch_mps=" << max_range_rate_mapping_mismatch_mps
              << " max_environmental_truth_mismatch_mps=" << max_environmental_truth_mismatch_mps << '\n'
              << "CARRIER_AUTH_VALIDITY code_doppler_adr_111=" << validity_combinations["111"]
              << " code_doppler_adr_110=" << validity_combinations["110"]
              << " code_doppler_adr_100=" << validity_combinations["100"] << '\n'
              << "CARRIER_AUTH_WORST gpst=" << worst_sample.week << ':' << worst_sample.tow_ns
              << " sat=" << worst_sample.satellite_number << " signal=" << worst_sample.signal_name
              << " cn0_dbhz=" << worst_sample.effective_cn0_dbhz << " mode=" << worst_sample.mode
              << " fll_phase=" << worst_sample.fll_phase << " sigma_hz=" << worst_sample.sigma_hz
              << " error_hz=" << worst_sample.tracking_error_hz << " error_mps=" << worst_sample.tracking_error_mps
              << " environmental_range_rate_mps=" << worst_sample.environmental_range_rate_mps
              << " urban_state=" << worst_context.state << " wall=" << worst_context.wall
              << " paths=" << worst_context.path_count << " reflections=" << worst_context.reflection_count << '\n';
    for (const auto& entry : modes) {
        print_tracking_distribution("MODE", entry.first, entry.second);
    }
    for (const char* bin : {"LT18", "18_22", "22_27", "27_30", "GE30"}) {
        print_tracking_distribution("CN0", bin, cn0_bins[bin]);
    }

    VelocityStats enabled_velocity;
    VelocityStats disabled_velocity;
    ASSERT_TRUE(parse_velocity_stats(enabled_a_directory / "solution_truth.csv", enabled.receiver_clock_drift_mps,
                                     &enabled_velocity));
    ASSERT_TRUE(parse_velocity_stats(disabled_a_directory / "solution_truth.csv", disabled_a.receiver_clock_drift_mps,
                                     &disabled_velocity));
    const DistributionMetrics enabled_velocity_3d = metrics(enabled_velocity.error_3d_mps);
    const DistributionMetrics disabled_velocity_3d = metrics(disabled_velocity.error_3d_mps);
    const DistributionMetrics enabled_clock = metrics(enabled_velocity.clock_drift_error_mps);
    const DistributionMetrics disabled_clock = metrics(disabled_velocity.clock_drift_error_mps);
    std::cout << "CARRIER_AUTH_VELOCITY on_valid=" << enabled_velocity_3d.count
              << " on_rms_3d_mps=" << enabled_velocity_3d.rms << " on_p95_3d_mps=" << enabled_velocity_3d.p95
              << " on_max_3d_mps=" << enabled_velocity_3d.maximum << " off_valid=" << disabled_velocity_3d.count
              << " off_rms_3d_mps=" << disabled_velocity_3d.rms << " off_p95_3d_mps=" << disabled_velocity_3d.p95
              << " off_max_3d_mps=" << disabled_velocity_3d.maximum << " on_clock_rms_mps=" << enabled_clock.rms
              << " on_clock_max_mps=" << enabled_clock.maximum << " off_clock_rms_mps=" << disabled_clock.rms
              << " off_clock_max_mps=" << disabled_clock.maximum << '\n';

    EXPECT_GT(result_rows, 100U);
    EXPECT_GT(valid_doppler_rows, 100U);
    EXPECT_GT(fll_code_doppler_no_adr, 0U);
    EXPECT_GT(unlocked_code_only, 0U);
    EXPECT_GT(pull_in_rows, 0U);
    EXPECT_GT(modes["FLL_TRACK"].rows, 0U);
    EXPECT_GT(modes["PLL_TRACK"].rows, 0U);
    EXPECT_GT(modes["CARRIER_UNLOCKED"].rows, 0U);
    EXPECT_LT(max_doppler_mapping_mismatch_hz, 1.0e-10);
    EXPECT_LT(max_range_rate_mapping_mismatch_mps, 1.0e-10);
    EXPECT_LT(max_environmental_truth_mismatch_mps, 1.0e-12);
    EXPECT_GT(enabled_velocity_3d.count, 0U);
    EXPECT_GT(disabled_velocity_3d.count, 0U);

    cleanup(enabled_a_directory);
    cleanup(enabled_b_directory);
    cleanup(disabled_a_directory);
    cleanup(disabled_b_directory);
}

TEST(CarrierTrackingAuthenticValidation, FrozenThermalJitterRespondsToCn0BandwidthIntegrationAndWavelength) {
    const gnss_sim::CarrierTrackingConfig base = gnss_sim::default_carrier_tracking_config();
    gnss_sim::CarrierTrackingJitter cn0_low{};
    gnss_sim::CarrierTrackingJitter cn0_high{};
    gnss_sim::CarrierTrackingJitter wide_band{};
    gnss_sim::CarrierTrackingJitter long_integration{};
    gnss_sim::CarrierTrackingJitter long_wavelength{};
    gnss_sim::CarrierTrackingJitter base_35{};
    std::string error_message;

    ASSERT_TRUE(gnss_sim::compute_carrier_tracking_jitter(base, gnss_sim::CarrierTrackingMode::kFllTrack, false, 20.0,
                                                         0.19, 0.02, &cn0_low, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::compute_carrier_tracking_jitter(base, gnss_sim::CarrierTrackingMode::kFllTrack, false, 40.0,
                                                         0.19, 0.02, &cn0_high, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::compute_carrier_tracking_jitter(base, gnss_sim::CarrierTrackingMode::kFllTrack, false, 35.0,
                                                         0.19, 0.02, &base_35, &error_message))
        << error_message;

    gnss_sim::CarrierTrackingConfig wider = base;
    wider.fll_noise_bandwidth_hz = 8.0;
    ASSERT_TRUE(gnss_sim::compute_carrier_tracking_jitter(wider, gnss_sim::CarrierTrackingMode::kFllTrack, false, 35.0,
                                                         0.19, 0.02, &wide_band, &error_message))
        << error_message;

    gnss_sim::CarrierTrackingConfig longer = base;
    longer.coherent_integration_sec = 0.04;
    ASSERT_TRUE(gnss_sim::compute_carrier_tracking_jitter(longer, gnss_sim::CarrierTrackingMode::kFllTrack, false,
                                                         35.0, 0.19, 0.02, &long_integration, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::compute_carrier_tracking_jitter(base, gnss_sim::CarrierTrackingMode::kFllTrack, false, 35.0,
                                                         0.38, 0.02, &long_wavelength, &error_message))
        << error_message;

    std::cout << "CARRIER_JITTER_RESPONSE cn0_20_sigma_hz=" << cn0_low.sigma_hz
              << " cn0_40_sigma_hz=" << cn0_high.sigma_hz << " base_35_sigma_hz=" << base_35.sigma_hz
              << " wide_8hz_sigma_hz=" << wide_band.sigma_hz
              << " integration_40ms_sigma_hz=" << long_integration.sigma_hz
              << " wavelength_0p19_sigma_mps=" << base_35.sigma_mps
              << " wavelength_0p38_sigma_mps=" << long_wavelength.sigma_mps << '\n';

    EXPECT_LT(cn0_high.sigma_hz, cn0_low.sigma_hz);
    EXPECT_GT(wide_band.sigma_hz, base_35.sigma_hz);
    EXPECT_LT(long_integration.sigma_hz, base_35.sigma_hz);
    EXPECT_NEAR(long_wavelength.sigma_hz, base_35.sigma_hz, 1.0e-12);
    EXPECT_NEAR(long_wavelength.sigma_mps / base_35.sigma_mps, 2.0, 1.0e-12);
}

TEST(CarrierTrackingAuthenticValidation, AuthenticNavOneAndTenHertzHaveConsistentLoopScale) {
    const std::filesystem::path one_hz_directory = "gnss_sim_carrier_auth_1hz";
    const std::filesystem::path ten_hz_directory = "gnss_sim_carrier_auth_10hz";
    const gnss_sim::SimConfig one_hz = static_config(1, true);
    const gnss_sim::SimConfig ten_hz = static_config(10, true);
    gnss_sim::SimulatorRunSummary one_hz_summary{};
    gnss_sim::SimulatorRunSummary ten_hz_summary{};
    std::string error_message;
    ASSERT_TRUE(run_in_directory(one_hz_directory, one_hz, &one_hz_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_in_directory(ten_hz_directory, ten_hz, &ten_hz_summary, &error_message)) << error_message;

    std::vector<CarrierSample> one_samples;
    std::vector<CarrierSample> ten_samples;
    ASSERT_TRUE(parse_carrier_samples(one_hz_directory / "carrier_tracking_truth.csv", &one_samples));
    ASSERT_TRUE(parse_carrier_samples(ten_hz_directory / "carrier_tracking_truth.csv", &ten_samples));

    std::map<std::string, CarrierSample> ten_by_key;
    for (const CarrierSample& sample : ten_samples) {
        ten_by_key[observation_key(sample.week, sample.tow_ns, sample.satellite_number, sample.signal_id)] = sample;
    }

    std::vector<double> one_errors;
    std::vector<double> ten_errors;
    std::uint64_t matched_scale = 0;
    double max_sigma_mismatch_hz = 0.0;
    for (const CarrierSample& one : one_samples) {
        if (!one.result_available || !one.post_doppler_valid || one.mode != "PLL_TRACK") {
            continue;
        }
        const auto ten = ten_by_key.find(observation_key(one.week, one.tow_ns, one.satellite_number, one.signal_id));
        if (ten == ten_by_key.end() || !ten->second.result_available || !ten->second.post_doppler_valid ||
            ten->second.mode != one.mode) {
            continue;
        }
        ++matched_scale;
        max_sigma_mismatch_hz = std::max(max_sigma_mismatch_hz, std::abs(one.sigma_hz - ten->second.sigma_hz));
        one_errors.push_back(one.tracking_error_hz);
        ten_errors.push_back(ten->second.tracking_error_hz);
    }
    const DistributionMetrics one_metric = metrics(one_errors);
    const DistributionMetrics ten_metric = metrics(ten_errors);
    const double rms_ratio = ten_metric.rms > 0.0 ? one_metric.rms / ten_metric.rms : 0.0;

    std::cout << "CARRIER_RATE_SANITY matched_pll=" << matched_scale
              << " max_sigma_mismatch_hz=" << max_sigma_mismatch_hz << " one_hz_error_rms_hz=" << one_metric.rms
              << " ten_hz_error_rms_hz=" << ten_metric.rms << " rms_ratio=" << rms_ratio << '\n';

    EXPECT_GT(matched_scale, 10U);
    EXPECT_LT(max_sigma_mismatch_hz, 1.0e-10);
    EXPECT_GT(one_metric.rms, 0.0);
    EXPECT_GT(ten_metric.rms, 0.0);
    EXPECT_GT(rms_ratio, 0.1);
    EXPECT_LT(rms_ratio, 10.0);

    cleanup(one_hz_directory);
    cleanup(ten_hz_directory);
}

TEST(CarrierTrackingAuthenticValidation, AuthenticNavReacquisitionExercisesPersistenceAndValidity) {
    const std::filesystem::path directory = "gnss_sim_carrier_auth_reacquisition";
    const gnss_sim::SimConfig config = reacquisition_config();
    const gnss_sim::SimTime validation_start = start_time();
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run_in_directory(directory, config, &summary, &error_message)) << error_message;

    std::vector<CarrierSample> samples;
    ASSERT_TRUE(parse_carrier_samples(directory / "carrier_tracking_truth.csv", &samples));

    std::uint64_t reset_rows = 0;
    std::uint64_t reacquisition_result_rows = 0;
    std::uint64_t reacquisition_unlocked_code_only = 0;
    std::uint64_t reacquisition_fll_valid_no_adr = 0;
    std::uint64_t reacquisition_new_segments = 0;
    for (const CarrierSample& sample : samples) {
        if (sample.reset_reason == "CODE_NOT_TRACKING") {
            ++reset_rows;
        }
        if (sample.acquisition_context != "REACQUISITION" || !sample.result_available) {
            continue;
        }
        ++reacquisition_result_rows;
        if (sample.mode == "CARRIER_UNLOCKED" && sample.post_code_valid && !sample.post_doppler_valid &&
            !sample.post_adr_valid) {
            ++reacquisition_unlocked_code_only;
        }
        if (sample.mode == "FLL_TRACK" && sample.post_code_valid && sample.post_doppler_valid &&
            !sample.post_adr_valid) {
            ++reacquisition_fll_valid_no_adr;
        }
        if (sample.new_carrier_segment) {
            ++reacquisition_new_segments;
        }
    }

    std::map<std::string, std::vector<CarrierSample>> groups;
    for (const CarrierSample& sample : samples) {
        groups[signal_key(sample.satellite_number, sample.signal_id)].push_back(sample);
    }

    bool found_full_sequence = false;
    std::string selected_key;
    double first_result_sec = 0.0;
    double first_fll_sec = 0.0;
    double first_fll_bandwidth_hz = 0.0;
    double first_doppler_valid_sec = 0.0;
    double first_pll_sec = 0.0;
    double first_adr_valid_sec = 0.0;
    double adr_jump_cycles = 0.0;
    for (const auto& entry : groups) {
        bool have_pre_adr = false;
        double last_pre_adr = 0.0;
        bool have_first_result = false;
        bool have_fll = false;
        bool have_doppler = false;
        bool have_pll = false;
        bool have_adr = false;
        double local_first_result = 0.0;
        double local_fll = 0.0;
        double local_fll_bandwidth = 0.0;
        double local_doppler = 0.0;
        double local_pll = 0.0;
        double local_adr = 0.0;
        double first_post_adr = 0.0;

        for (const CarrierSample& sample : entry.second) {
            const double elapsed_sec = static_cast<double>(sample.tow_ns - validation_start.tow_ns) /
                                       static_cast<double>(gnss_sim::NANOSECONDS_PER_SECOND);
            if (elapsed_sec < 6.0 && sample.post_snapshot_available && sample.post_adr_valid) {
                have_pre_adr = true;
                last_pre_adr = sample.post_adr_cycles;
            }
            if (sample.acquisition_context != "REACQUISITION" || !sample.result_available) {
                continue;
            }
            if (!have_first_result) {
                have_first_result = true;
                local_first_result = elapsed_sec;
            }
            if (!have_fll && sample.mode == "FLL_TRACK") {
                have_fll = true;
                local_fll = elapsed_sec;
                local_fll_bandwidth = sample.active_bandwidth_hz;
            }
            if (!have_doppler && sample.post_doppler_valid) {
                have_doppler = true;
                local_doppler = elapsed_sec;
            }
            if (!have_pll && sample.mode == "PLL_TRACK") {
                have_pll = true;
                local_pll = elapsed_sec;
            }
            if (!have_adr && sample.post_adr_valid) {
                have_adr = true;
                local_adr = elapsed_sec;
                first_post_adr = sample.post_adr_cycles;
            }
        }
        if (have_pre_adr && have_first_result && have_fll && have_doppler && have_pll && have_adr) {
            found_full_sequence = true;
            selected_key = entry.first;
            first_result_sec = local_first_result;
            first_fll_sec = local_fll;
            first_fll_bandwidth_hz = local_fll_bandwidth;
            first_doppler_valid_sec = local_doppler;
            first_pll_sec = local_pll;
            first_adr_valid_sec = local_adr;
            adr_jump_cycles = first_post_adr - last_pre_adr;
            break;
        }
    }

    std::cout << "CARRIER_REACQUISITION reset_rows=" << reset_rows
              << " result_rows=" << reacquisition_result_rows
              << " unlocked_code_only=" << reacquisition_unlocked_code_only
              << " fll_valid_no_adr=" << reacquisition_fll_valid_no_adr
              << " new_segments=" << reacquisition_new_segments << " selected=" << selected_key
              << " first_result_s=" << first_result_sec << " first_fll_s=" << first_fll_sec
              << " first_fll_bandwidth_hz=" << first_fll_bandwidth_hz
              << " first_doppler_valid_s=" << first_doppler_valid_sec << " first_pll_s=" << first_pll_sec
              << " first_adr_valid_s=" << first_adr_valid_sec << " adr_jump_cycles=" << adr_jump_cycles << '\n';

    EXPECT_GT(reset_rows, 0U);
    EXPECT_GT(reacquisition_result_rows, 0U);
    EXPECT_GT(reacquisition_unlocked_code_only, 0U);
    EXPECT_GT(reacquisition_fll_valid_no_adr, 0U);
    EXPECT_GT(reacquisition_new_segments, 0U);
    ASSERT_TRUE(found_full_sequence);
    EXPECT_GE(first_fll_sec - first_result_sec, 0.1);
    EXPECT_NEAR(first_fll_bandwidth_hz, config.carrier_tracking.fll_pull_in_bandwidth_hz, 1.0e-12);
    EXPECT_GE(first_doppler_valid_sec - first_fll_sec, 0.1);
    EXPECT_GE(first_adr_valid_sec - first_pll_sec, 0.9);
    EXPECT_NE(adr_jump_cycles, 0.0);

    cleanup(directory);
}

} // namespace
