#include "gnss_sim/sim_config.h"
#include "gnss_sim/sim_time.h"
#include "gnss_sim/simulator.h"

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
#include <vector>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTruthLatitudeDeg = 20.0;
constexpr double kTruthLongitudeDeg = 120.0;
constexpr double kTruthHeightM = 100.0;
constexpr std::int64_t kDurationSeconds = 60;
constexpr const char* kFilteredNavName = "brd400dlr_beidou_verbatim_nav.rnx";

std::string source_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

gnss_sim::SimTime start_time() {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2347, 436500.0, &time));
    return time;
}

gnss_sim::SimConfig validation_config(int sampling_rate_hz, bool multipath_enabled) {
    gnss_sim::SimConfig config = gnss_sim::default_sim_config();
    config.scenario = gnss_sim::ScenarioType::KS;
    config.duration_ns = kDurationSeconds * gnss_sim::NANOSECONDS_PER_SECOND;
    config.sampling_rate_hz = sampling_rate_hz;
    config.elevation_mask_deg = 0.0;
    config.solution_elevation_mask_deg = 5.0;
    config.output_eph = false;
    config.output_ion = false;
    config.measurement_noise_enabled = false;
    config.multipath_enabled = multipath_enabled;
    config.atmosphere_mode = gnss_sim::AtmosphereMode::NONE;
    config.receiver = {kTruthLatitudeDeg, kTruthLongitudeDeg, kTruthHeightM};
    config.seed = 0x152U;
    return config;
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
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

std::string epoch_key(int week, std::int64_t tow_ns) {
    return std::to_string(week) + ":" + std::to_string(tow_ns);
}

struct ObservationValue {
    bool doppler_valid = false;
    double doppler_hz = 0.0;
    double range_rate_mps = 0.0;
};

using ObservationMap = std::map<std::string, ObservationValue>;

bool parse_observations(const std::filesystem::path& path, ObservationMap* output) {
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
    const int valid = column_index(header, "doppler_valid");
    const int doppler = column_index(header, "doppler_hz");
    const int range_rate = column_index(header, "range_rate_mps");
    if (week < 0 || tow < 0 || satellite < 0 || signal < 0 || valid < 0 || doppler < 0 || range_rate < 0) {
        return false;
    }
    ObservationMap result;
    for (std::size_t index = 1; index < rows.size(); ++index) {
        const auto& row = rows[index];
        if (row.size() != header.size()) {
            return false;
        }
        const int gps_week = std::stoi(row[static_cast<std::size_t>(week)]);
        const std::int64_t tow_ns = std::stoll(row[static_cast<std::size_t>(tow)]);
        const int satellite_number = std::stoi(row[static_cast<std::size_t>(satellite)]);
        const int signal_id = std::stoi(row[static_cast<std::size_t>(signal)]);
        ObservationValue value{};
        value.doppler_valid = row[static_cast<std::size_t>(valid)] == "1";
        value.doppler_hz = std::stod(row[static_cast<std::size_t>(doppler)]);
        value.range_rate_mps = std::stod(row[static_cast<std::size_t>(range_rate)]);
        result[observation_key(gps_week, tow_ns, satellite_number, signal_id)] = value;
    }
    *output = result;
    return true;
}

struct UrbanSample {
    int week = 0;
    std::int64_t tow_ns = 0;
    int satellite_number = 0;
    int signal_id = 0;
    std::string signal_name;
    std::string state;
    std::string wall;
    bool propagation_evaluated = false;
    int path_count = 0;
    int reflection_count = 0;
    double azimuth_deg = 0.0;
    double elevation_deg = 0.0;
    double open_cn0_dbhz = 0.0;
    double effective_cn0_dbhz = 0.0;
    double wavelength_m = 0.0;
    double environmental_range_rate_mps = 0.0;
    bool environmental_range_rate_valid = false;
    double carrier_range_bias_m = 0.0;
    std::int64_t lock_time_ns = 0;
    bool reacquisition_event = false;
    bool cycle_slip_event = false;
    double code_bias_m = 0.0;
};

using UrbanMap = std::map<std::string, UrbanSample>;

bool parse_urban_samples(const std::filesystem::path& path, UrbanMap* output) {
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
    const int state = column_index(header, "urban_state");
    const int wall = column_index(header, "blocking_wall");
    const int propagation = column_index(header, "propagation_evaluated");
    const int path_count = column_index(header, "received_path_count");
    const int reflection_count = column_index(header, "reflection_count");
    const int azimuth = column_index(header, "azimuth_deg");
    const int elevation = column_index(header, "elevation_deg");
    const int open_cn0 = column_index(header, "open_cn0_dbhz");
    const int effective_cn0 = column_index(header, "effective_cn0_dbhz");
    const int wavelength = column_index(header, "wavelength_m");
    const int environmental_rate = column_index(header, "environmental_range_rate_mps");
    const int environmental_valid = column_index(header, "environmental_range_rate_valid");
    const int carrier_bias = column_index(header, "carrier_range_bias_m");
    const int lock_time = column_index(header, "lock_time_ns");
    const int reacquisition = column_index(header, "reacquisition_event");
    const int cycle_slip = column_index(header, "cycle_slip_event");
    const int code_bias = column_index(header, "code_bias_m");
    if (week < 0 || tow < 0 || satellite < 0 || signal < 0 || signal_name < 0 || state < 0 || wall < 0 ||
        propagation < 0 || path_count < 0 || reflection_count < 0 || azimuth < 0 || elevation < 0 || open_cn0 < 0 ||
        effective_cn0 < 0 || wavelength < 0 || environmental_rate < 0 || environmental_valid < 0 || carrier_bias < 0 ||
        lock_time < 0 || reacquisition < 0 || cycle_slip < 0 || code_bias < 0) {
        return false;
    }
    UrbanMap result;
    for (std::size_t index = 1; index < rows.size(); ++index) {
        const auto& row = rows[index];
        if (row.size() != header.size()) {
            return false;
        }
        UrbanSample sample{};
        sample.week = std::stoi(row[static_cast<std::size_t>(week)]);
        sample.tow_ns = std::stoll(row[static_cast<std::size_t>(tow)]);
        sample.satellite_number = std::stoi(row[static_cast<std::size_t>(satellite)]);
        sample.signal_id = std::stoi(row[static_cast<std::size_t>(signal)]);
        sample.signal_name = row[static_cast<std::size_t>(signal_name)];
        sample.state = row[static_cast<std::size_t>(state)];
        sample.wall = row[static_cast<std::size_t>(wall)];
        sample.propagation_evaluated = row[static_cast<std::size_t>(propagation)] == "1";
        sample.path_count = std::stoi(row[static_cast<std::size_t>(path_count)]);
        sample.reflection_count = std::stoi(row[static_cast<std::size_t>(reflection_count)]);
        sample.azimuth_deg = std::stod(row[static_cast<std::size_t>(azimuth)]);
        sample.elevation_deg = std::stod(row[static_cast<std::size_t>(elevation)]);
        if (!row[static_cast<std::size_t>(open_cn0)].empty()) {
            sample.open_cn0_dbhz = std::stod(row[static_cast<std::size_t>(open_cn0)]);
        }
        if (!row[static_cast<std::size_t>(effective_cn0)].empty()) {
            sample.effective_cn0_dbhz = std::stod(row[static_cast<std::size_t>(effective_cn0)]);
        }
        sample.wavelength_m = std::stod(row[static_cast<std::size_t>(wavelength)]);
        sample.environmental_range_rate_mps = std::stod(row[static_cast<std::size_t>(environmental_rate)]);
        sample.environmental_range_rate_valid = row[static_cast<std::size_t>(environmental_valid)] == "1";
        sample.carrier_range_bias_m = std::stod(row[static_cast<std::size_t>(carrier_bias)]);
        sample.lock_time_ns = std::stoll(row[static_cast<std::size_t>(lock_time)]);
        sample.reacquisition_event = row[static_cast<std::size_t>(reacquisition)] == "1";
        sample.cycle_slip_event = row[static_cast<std::size_t>(cycle_slip)] == "1";
        sample.code_bias_m = std::stod(row[static_cast<std::size_t>(code_bias)]);
        result[observation_key(sample.week, sample.tow_ns, sample.satellite_number, sample.signal_id)] = sample;
    }
    *output = result;
    return true;
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
    if (rank == 0U) {
        rank = 1U;
    }
    if (rank > values.size()) {
        rank = values.size();
    }
    return values[rank - 1U];
}

DistributionMetrics metrics(const std::vector<double>& values) {
    DistributionMetrics result{};
    result.count = static_cast<std::uint64_t>(values.size());
    if (values.empty()) {
        return result;
    }
    double sum_squares = 0.0;
    for (double value : values) {
        sum_squares += value * value;
        result.maximum = std::max(result.maximum, std::abs(value));
    }
    std::vector<double> absolute;
    absolute.reserve(values.size());
    for (double value : values) {
        absolute.push_back(std::abs(value));
    }
    result.rms = std::sqrt(sum_squares / static_cast<double>(values.size()));
    result.p50 = percentile(absolute, 0.50);
    result.p95 = percentile(absolute, 0.95);
    result.p99 = percentile(absolute, 0.99);
    return result;
}

struct StateDistribution {
    std::vector<double> range_rate_mps;
    std::vector<double> doppler_hz;
};

struct VelocityStats {
    std::vector<double> error_3d_mps;
    std::vector<double> horizontal_error_mps;
    std::vector<double> vertical_error_mps;
    std::vector<double> clock_drift_error_mps;
    std::string worst_epoch;
    double worst_error_mps = 0.0;
    int worst_used_satellites = 0;
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
    const int week = column_index(header, "gps_week");
    const int tow = column_index(header, "tow_ns");
    const int valid = column_index(header, "velocity_valid");
    const int vx = column_index(header, "velocity_x_mps");
    const int vy = column_index(header, "velocity_y_mps");
    const int vz = column_index(header, "velocity_z_mps");
    const int horizontal = column_index(header, "horizontal_speed_mps");
    const int vertical = column_index(header, "vertical_speed_mps");
    const int clock_drift = column_index(header, "receiver_clock_drift_mps");
    const int used = column_index(header, "velocity_used_satellites");
    if (week < 0 || tow < 0 || valid < 0 || vx < 0 || vy < 0 || vz < 0 || horizontal < 0 || vertical < 0 ||
        clock_drift < 0 || used < 0) {
        return false;
    }
    VelocityStats result{};
    for (std::size_t index = 1; index < rows.size(); ++index) {
        const auto& row = rows[index];
        if (row.size() != header.size()) {
            return false;
        }
        if (row[static_cast<std::size_t>(valid)] != "1") {
            continue;
        }
        const double x = std::stod(row[static_cast<std::size_t>(vx)]);
        const double y = std::stod(row[static_cast<std::size_t>(vy)]);
        const double z = std::stod(row[static_cast<std::size_t>(vz)]);
        const double error_3d = std::sqrt(x * x + y * y + z * z);
        result.error_3d_mps.push_back(error_3d);
        result.horizontal_error_mps.push_back(std::abs(std::stod(row[static_cast<std::size_t>(horizontal)])));
        result.vertical_error_mps.push_back(std::abs(std::stod(row[static_cast<std::size_t>(vertical)])));
        result.clock_drift_error_mps.push_back(std::stod(row[static_cast<std::size_t>(clock_drift)]) -
                                               truth_clock_drift_mps);
        if (error_3d >= result.worst_error_mps) {
            result.worst_error_mps = error_3d;
            result.worst_epoch = epoch_key(std::stoi(row[static_cast<std::size_t>(week)]),
                                           std::stoll(row[static_cast<std::size_t>(tow)]));
            result.worst_used_satellites = std::stoi(row[static_cast<std::size_t>(used)]);
        }
    }
    *output = result;
    return true;
}

bool run_in_directory(const std::filesystem::path& directory, const gnss_sim::SimConfig& config,
                      gnss_sim::SimulatorRunSummary* summary, std::string* error_message) {
    std::error_code filesystem_error;
    std::filesystem::remove_all(directory, filesystem_error);
    filesystem_error.clear();
    if (!std::filesystem::create_directories(directory, filesystem_error) || filesystem_error) {
        if (error_message != nullptr) {
            *error_message = "cannot create static urban Doppler validation directory";
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

void print_distribution(const char* prefix, const StateDistribution& distribution) {
    const DistributionMetrics rate = metrics(distribution.range_rate_mps);
    const DistributionMetrics doppler = metrics(distribution.doppler_hz);
    std::cout << prefix << " count=" << rate.count << " rate_rms_mps=" << rate.rms << " rate_p50_mps=" << rate.p50
              << " rate_p95_mps=" << rate.p95 << " rate_p99_mps=" << rate.p99 << " rate_max_mps=" << rate.maximum
              << " doppler_rms_hz=" << doppler.rms << " doppler_p50_hz=" << doppler.p50
              << " doppler_p95_hz=" << doppler.p95 << " doppler_p99_hz=" << doppler.p99
              << " doppler_max_hz=" << doppler.maximum << '\n';
}

TEST(StaticUrbanDopplerValidation, AuthenticNavQuantifiesDopplerAndStaticVelocity) {
    const std::filesystem::path on_directory = "gnss_sim_static_doppler_on_1hz";
    const std::filesystem::path off_directory = "gnss_sim_static_doppler_off_1hz";
    const gnss_sim::SimConfig on_config = validation_config(1, true);
    const gnss_sim::SimConfig off_config = validation_config(1, false);
    gnss_sim::SimulatorRunSummary on_summary{};
    gnss_sim::SimulatorRunSummary off_summary{};
    std::string error_message;
    ASSERT_TRUE(run_in_directory(on_directory, on_config, &on_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_in_directory(off_directory, off_config, &off_summary, &error_message)) << error_message;

    UrbanMap urban;
    ObservationMap on_observations;
    ObservationMap off_observations;
    ASSERT_TRUE(parse_urban_samples(on_directory / "urban_signal_truth.csv", &urban));
    ASSERT_TRUE(parse_observations(on_directory / "observation_truth.csv", &on_observations));
    ASSERT_TRUE(parse_observations(off_directory / "observation_truth.csv", &off_observations));

    StateDistribution all;
    StateDistribution los;
    StateDistribution los_multipath;
    StateDistribution nlos_tracked;
    std::uint64_t blocked_with_valid_doppler = 0;
    std::uint64_t invalid_environmental_rate = 0;
    std::uint64_t reacquisition_rows = 0;
    std::uint64_t cycle_slip_rows = 0;
    std::uint64_t matched_on_off = 0;
    double max_doppler_mapping_mismatch_hz = 0.0;
    double max_range_rate_mapping_mismatch_mps = 0.0;
    double worst_absolute_rate_mps = 0.0;
    UrbanSample worst_sample{};
    std::map<std::string, std::map<std::string, std::uint64_t>> epoch_states;

    for (const auto& item : urban) {
        const UrbanSample& sample = item.second;
        ++epoch_states[epoch_key(sample.week, sample.tow_ns)][sample.state];
        if (sample.reacquisition_event) {
            ++reacquisition_rows;
        }
        if (sample.cycle_slip_event) {
            ++cycle_slip_rows;
        }
        if (!sample.environmental_range_rate_valid) {
            ++invalid_environmental_rate;
        }
        const auto on_it = on_observations.find(item.first);
        if (sample.state == "BLOCKED" && on_it != on_observations.end() && on_it->second.doppler_valid) {
            ++blocked_with_valid_doppler;
        }
        if (!sample.environmental_range_rate_valid || !(sample.wavelength_m > 0.0) || on_it == on_observations.end() ||
            !on_it->second.doppler_valid) {
            continue;
        }

        const double rate = sample.environmental_range_rate_mps;
        const double doppler_bias_hz = -rate / sample.wavelength_m;
        all.range_rate_mps.push_back(rate);
        all.doppler_hz.push_back(doppler_bias_hz);
        StateDistribution* state_distribution = nullptr;
        if (sample.state == "LOS") {
            state_distribution = &los;
        } else if (sample.state == "LOS_MULTIPATH") {
            state_distribution = &los_multipath;
        } else if (sample.state == "NLOS_TRACKED") {
            state_distribution = &nlos_tracked;
        }
        if (state_distribution != nullptr) {
            state_distribution->range_rate_mps.push_back(rate);
            state_distribution->doppler_hz.push_back(doppler_bias_hz);
        }
        if (std::abs(rate) >= worst_absolute_rate_mps) {
            worst_absolute_rate_mps = std::abs(rate);
            worst_sample = sample;
        }

        const auto off_it = off_observations.find(item.first);
        if (off_it == off_observations.end() || !off_it->second.doppler_valid) {
            continue;
        }
        ++matched_on_off;
        const double actual_doppler_delta = on_it->second.doppler_hz - off_it->second.doppler_hz;
        const double actual_range_rate_delta = on_it->second.range_rate_mps - off_it->second.range_rate_mps;
        max_doppler_mapping_mismatch_hz =
            std::max(max_doppler_mapping_mismatch_hz, std::abs(actual_doppler_delta - doppler_bias_hz));
        max_range_rate_mapping_mismatch_mps =
            std::max(max_range_rate_mapping_mismatch_mps, std::abs(actual_range_rate_delta - rate));
    }

    VelocityStats on_velocity{};
    VelocityStats off_velocity{};
    ASSERT_TRUE(
        parse_velocity_stats(on_directory / "solution_truth.csv", on_config.receiver_clock_drift_mps, &on_velocity));
    ASSERT_TRUE(
        parse_velocity_stats(off_directory / "solution_truth.csv", off_config.receiver_clock_drift_mps, &off_velocity));

    print_distribution("STATIC_DOPPLER_ALL", all);
    print_distribution("STATIC_DOPPLER_LOS", los);
    print_distribution("STATIC_DOPPLER_LOS_MULTIPATH", los_multipath);
    print_distribution("STATIC_DOPPLER_NLOS_TRACKED", nlos_tracked);

    const DistributionMetrics on_velocity_3d = metrics(on_velocity.error_3d_mps);
    const DistributionMetrics on_horizontal = metrics(on_velocity.horizontal_error_mps);
    const DistributionMetrics on_vertical = metrics(on_velocity.vertical_error_mps);
    const DistributionMetrics on_clock = metrics(on_velocity.clock_drift_error_mps);
    const DistributionMetrics off_velocity_3d = metrics(off_velocity.error_3d_mps);
    const auto worst_states = epoch_states.find(on_velocity.worst_epoch);

    std::cout << "STATIC_DOPPLER_MAPPING matched=" << matched_on_off
              << " max_doppler_mismatch_hz=" << max_doppler_mapping_mismatch_hz
              << " max_range_rate_mismatch_mps=" << max_range_rate_mapping_mismatch_mps << '\n'
              << "STATIC_DOPPLER_INVALID env_rate_invalid_rows=" << invalid_environmental_rate
              << " reacquisition_rows=" << reacquisition_rows << " cycle_slip_rows=" << cycle_slip_rows
              << " blocked_valid_doppler=" << blocked_with_valid_doppler << '\n'
              << "STATIC_DOPPLER_WORST gpst=" << worst_sample.week << ':' << worst_sample.tow_ns
              << " sat=" << worst_sample.satellite_number << " signal=" << worst_sample.signal_name
              << " state=" << worst_sample.state << " rate_mps=" << worst_sample.environmental_range_rate_mps
              << " doppler_hz=" << (-worst_sample.environmental_range_rate_mps / worst_sample.wavelength_m)
              << " az_deg=" << worst_sample.azimuth_deg << " el_deg=" << worst_sample.elevation_deg
              << " wall=" << worst_sample.wall << " paths=" << worst_sample.path_count
              << " reflections=" << worst_sample.reflection_count << " cn0_open=" << worst_sample.open_cn0_dbhz
              << " cn0_effective=" << worst_sample.effective_cn0_dbhz << " code_bias_m=" << worst_sample.code_bias_m
              << '\n'
              << "STATIC_VELOCITY_ON valid=" << on_velocity_3d.count << " rms_3d_mps=" << on_velocity_3d.rms
              << " p95_3d_mps=" << on_velocity_3d.p95 << " max_3d_mps=" << on_velocity_3d.maximum
              << " horizontal_rms_mps=" << on_horizontal.rms << " horizontal_p95_mps=" << on_horizontal.p95
              << " horizontal_max_mps=" << on_horizontal.maximum << " vertical_rms_mps=" << on_vertical.rms
              << " vertical_p95_mps=" << on_vertical.p95 << " vertical_max_mps=" << on_vertical.maximum
              << " clock_rms_mps=" << on_clock.rms << " clock_p95_mps=" << on_clock.p95
              << " clock_max_mps=" << on_clock.maximum << " worst_epoch=" << on_velocity.worst_epoch
              << " worst_used_sats=" << on_velocity.worst_used_satellites << '\n'
              << "STATIC_VELOCITY_OFF valid=" << off_velocity_3d.count << " rms_3d_mps=" << off_velocity_3d.rms
              << " p95_3d_mps=" << off_velocity_3d.p95 << " max_3d_mps=" << off_velocity_3d.maximum << '\n';
    if (worst_states != epoch_states.end()) {
        std::cout << "STATIC_VELOCITY_WORST_STATES LOS=" << worst_states->second["LOS"]
                  << " LOS_MULTIPATH=" << worst_states->second["LOS_MULTIPATH"]
                  << " NLOS_TRACKED=" << worst_states->second["NLOS_TRACKED"]
                  << " BLOCKED=" << worst_states->second["BLOCKED"] << '\n';
    }

    EXPECT_GT(all.range_rate_mps.size(), 100U);
    EXPECT_GT(los.range_rate_mps.size(), 0U);
    EXPECT_GT(los_multipath.range_rate_mps.size(), 0U);
    EXPECT_GT(nlos_tracked.range_rate_mps.size(), 0U);
    EXPECT_EQ(blocked_with_valid_doppler, 0U);
    EXPECT_GT(matched_on_off, 100U);
    EXPECT_LT(max_doppler_mapping_mismatch_hz, 1.0e-8);
    EXPECT_LT(max_range_rate_mapping_mismatch_mps, 1.0e-9);
    EXPECT_GT(on_velocity_3d.count, 0U);
    EXPECT_GT(off_velocity_3d.count, 0U);

    cleanup(on_directory);
    cleanup(off_directory);
}

TEST(StaticUrbanDopplerValidation, OneHertzMatchesTenHertzOneSecondCarrierDifference) {
    const std::filesystem::path one_hz_directory = "gnss_sim_static_doppler_alias_1hz";
    const std::filesystem::path ten_hz_directory = "gnss_sim_static_doppler_alias_10hz";
    const gnss_sim::SimConfig one_hz_config = validation_config(1, true);
    const gnss_sim::SimConfig ten_hz_config = validation_config(10, true);
    gnss_sim::SimulatorRunSummary one_hz_summary{};
    gnss_sim::SimulatorRunSummary ten_hz_summary{};
    std::string error_message;
    ASSERT_TRUE(run_in_directory(one_hz_directory, one_hz_config, &one_hz_summary, &error_message)) << error_message;
    ASSERT_TRUE(run_in_directory(ten_hz_directory, ten_hz_config, &ten_hz_summary, &error_message)) << error_message;

    UrbanMap one_hz;
    UrbanMap ten_hz;
    ASSERT_TRUE(parse_urban_samples(one_hz_directory / "urban_signal_truth.csv", &one_hz));
    ASSERT_TRUE(parse_urban_samples(ten_hz_directory / "urban_signal_truth.csv", &ten_hz));

    std::uint64_t matched_intervals = 0;
    double max_rate_mismatch_mps = 0.0;
    double sum_squared_mismatch = 0.0;
    double max_ten_hz_reconstructed_one_second_phase_change_rad = 0.0;
    UrbanSample worst_sample{};
    for (const auto& item : one_hz) {
        const UrbanSample& one = item.second;
        if (!one.environmental_range_rate_valid || one.lock_time_ns < gnss_sim::NANOSECONDS_PER_SECOND ||
            one.reacquisition_event || one.cycle_slip_event || !(one.wavelength_m > 0.0)) {
            continue;
        }
        const auto current_ten = ten_hz.find(item.first);
        if (current_ten == ten_hz.end() || !current_ten->second.environmental_range_rate_valid ||
            current_ten->second.lock_time_ns < gnss_sim::NANOSECONDS_PER_SECOND ||
            current_ten->second.reacquisition_event || current_ten->second.cycle_slip_event) {
            continue;
        }
        const std::int64_t previous_tow_ns = one.tow_ns - gnss_sim::NANOSECONDS_PER_SECOND;
        if (previous_tow_ns < 0) {
            continue;
        }
        const std::string previous_key =
            observation_key(one.week, previous_tow_ns, one.satellite_number, one.signal_id);
        const auto previous_ten = ten_hz.find(previous_key);
        if (previous_ten == ten_hz.end() || !previous_ten->second.environmental_range_rate_valid) {
            continue;
        }
        const double ten_hz_one_second_rate =
            current_ten->second.carrier_range_bias_m - previous_ten->second.carrier_range_bias_m;
        const double mismatch = one.environmental_range_rate_mps - ten_hz_one_second_rate;
        ++matched_intervals;
        sum_squared_mismatch += mismatch * mismatch;
        if (std::abs(mismatch) >= max_rate_mismatch_mps) {
            max_rate_mismatch_mps = std::abs(mismatch);
            worst_sample = one;
        }
        const double ten_hz_reconstructed_one_second_phase_change =
            std::abs(ten_hz_one_second_rate) * 2.0 * kPi / one.wavelength_m;
        max_ten_hz_reconstructed_one_second_phase_change_rad = std::max(
            max_ten_hz_reconstructed_one_second_phase_change_rad, ten_hz_reconstructed_one_second_phase_change);
    }
    const double rms_mismatch_mps =
        matched_intervals > 0 ? std::sqrt(sum_squared_mismatch / static_cast<double>(matched_intervals)) : 0.0;

    std::cout << "STATIC_DOPPLER_RATE_CHECK matched_intervals=" << matched_intervals
              << " rms_mismatch_mps=" << rms_mismatch_mps << " max_mismatch_mps=" << max_rate_mismatch_mps
              << " max_10hz_reconstructed_1s_phase_change_rad=" << max_ten_hz_reconstructed_one_second_phase_change_rad
              << " pi=" << kPi << " worst_gpst=" << worst_sample.week << ':' << worst_sample.tow_ns
              << " worst_sat=" << worst_sample.satellite_number << " worst_signal=" << worst_sample.signal_name
              << " worst_state=" << worst_sample.state << '\n';

    EXPECT_GT(matched_intervals, 100U);
    EXPECT_LT(max_rate_mismatch_mps, 1.0e-6);
    EXPECT_LT(max_ten_hz_reconstructed_one_second_phase_change_rad, kPi);

    cleanup(one_hz_directory);
    cleanup(ten_hz_directory);
}

} // namespace
