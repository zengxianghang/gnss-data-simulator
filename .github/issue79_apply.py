from pathlib import Path
import re


def read(path):
    return Path(path).read_text(encoding="utf-8")


def write(path, text):
    Path(path).write_text(text, encoding="utf-8")


def replace_once(path, old, new):
    text = read(path)
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one match, found {count}: {old[:80]!r}")
    write(path, text.replace(old, new, 1))


def regex_once(path, pattern, replacement):
    text = read(path)
    updated, count = re.subn(pattern, replacement, text, count=1, flags=re.S)
    if count != 1:
        raise RuntimeError(f"{path}: regex expected one match, found {count}: {pattern[:80]!r}")
    write(path, updated)


# Public configuration.
replace_once(
    "include/gnss_sim/sim_config.h",
    "struct ReaConfig {\n    std::int64_t signal_on_ns;\n    std::int64_t signal_off_ns;\n};\n\n",
    "struct ReaConfig {\n    std::int64_t signal_on_ns;\n    std::int64_t signal_off_ns;\n};\n\n"
    "struct BestposRtkConfig {\n"
    "    bool enabled;\n"
    "    std::int64_t stable_duration_ns;\n"
    "    int min_used_satellites;\n"
    "    double horizontal_std_m;\n"
    "    double height_std_m;\n"
    "};\n\n",
)
replace_once(
    "include/gnss_sim/sim_config.h",
    "    TtffConfig ttff;\n    ReaConfig rea;\n    MeasurementErrorConfig measurement_error;\n",
    "    TtffConfig ttff;\n    ReaConfig rea;\n    BestposRtkConfig bestpos_rtk;\n    MeasurementErrorConfig measurement_error;\n",
)

# Config parser/defaults/validation.
insert_parse = r'''bool parse_bestpos_rtk(const cJSON* root, SimConfig* config, std::string* error_message) {
    const cJSON* object = cJSON_GetObjectItemCaseSensitive(root, "bestpos_rtk");
    if (object == nullptr) {
        return true;
    }
    const char* const allowed_keys[] = {"enabled", "stable_duration_sec", "min_used_satellites", "horizontal_std_m",
                                        "height_std_m"};
    if (!validate_object_keys(object, "bestpos_rtk", allowed_keys, 5U, error_message)) {
        return false;
    }
    double stable_duration_sec = static_cast<double>(config->bestpos_rtk.stable_duration_ns) /
                                 static_cast<double>(NANOSECONDS_PER_SECOND);
    return read_optional_bool(object, "enabled", &config->bestpos_rtk.enabled, error_message) &&
           read_optional_number(object, "stable_duration_sec", &stable_duration_sec, error_message) &&
           read_optional_int(object, "min_used_satellites", &config->bestpos_rtk.min_used_satellites, error_message) &&
           read_optional_number(object, "horizontal_std_m", &config->bestpos_rtk.horizontal_std_m, error_message) &&
           read_optional_number(object, "height_std_m", &config->bestpos_rtk.height_std_m, error_message) &&
           seconds_to_ns(stable_duration_sec, "bestpos_rtk.stable_duration_sec",
                         &config->bestpos_rtk.stable_duration_ns, error_message);
}

'''
replace_once(
    "src/core/sim_config.cpp",
    "bool parse_measurement_transient(const cJSON* parent, const char* key, MeasurementTransientErrorConfig* config,\n",
    insert_parse + "bool parse_measurement_transient(const cJSON* parent, const char* key, MeasurementTransientErrorConfig* config,\n",
)
replace_once(
    "src/core/sim_config.cpp",
    "bool valid_measurement_error_config(const MeasurementErrorConfig& config) {\n",
    "bool valid_bestpos_rtk_config(const BestposRtkConfig& config) {\n"
    "    return config.stable_duration_ns >= 0 && config.min_used_satellites >= 4 &&\n"
    "           config.min_used_satellites <= 64 && finite_nonnegative(config.horizontal_std_m) &&\n"
    "           finite_nonnegative(config.height_std_m);\n"
    "}\n\n"
    "bool valid_measurement_error_config(const MeasurementErrorConfig& config) {\n",
)
replace_once(
    "src/core/sim_config.cpp",
    "    config.rea = {300LL * NANOSECONDS_PER_SECOND, 10LL * NANOSECONDS_PER_SECOND};\n    config.measurement_error = {0.08,\n",
    "    config.rea = {300LL * NANOSECONDS_PER_SECOND, 10LL * NANOSECONDS_PER_SECOND};\n"
    "    config.bestpos_rtk = {true, 5LL * NANOSECONDS_PER_SECOND, 6, 0.01, 0.02};\n"
    "    config.measurement_error = {0.08,\n",
)
replace_once(
    "src/core/sim_config.cpp",
    "    if (!valid_measurement_error_config(config.measurement_error)) {\n",
    "    if (!valid_bestpos_rtk_config(config.bestpos_rtk)) {\n"
    "        set_error(error_message, \"bestpos_rtk configuration is invalid\");\n"
    "        return false;\n"
    "    }\n"
    "    if (!valid_measurement_error_config(config.measurement_error)) {\n",
)
replace_once(
    "src/core/sim_config.cpp",
    "                                        \"measurement_noise_enabled\",\n                                        \"measurement_error\",\n",
    "                                        \"measurement_noise_enabled\",\n                                        \"bestpos_rtk\",\n                                        \"measurement_error\",\n",
)
replace_once(
    "src/core/sim_config.cpp",
    "    bool success = validate_object_keys(root, \"root\", allowed_keys, 18U, error_message);\n",
    "    bool success = validate_object_keys(root, \"root\", allowed_keys, 19U, error_message);\n",
)
replace_once(
    "src/core/sim_config.cpp",
    "            parse_receiver(root, &parsed, error_message) && parse_ttff(root, &parsed, error_message) &&\n            parse_rea(root, &parsed, error_message) && parse_measurement_error(root, &parsed, error_message) &&\n",
    "            parse_receiver(root, &parsed, error_message) && parse_ttff(root, &parsed, error_message) &&\n"
    "            parse_rea(root, &parsed, error_message) && parse_bestpos_rtk(root, &parsed, error_message) &&\n"
    "            parse_measurement_error(root, &parsed, error_message) &&\n",
)

# Default JSON documents the active model explicitly.
replace_once(
    "config/default_v1.json",
    "  \"rea\": {\n    \"signal_on_sec\": 300.0,\n    \"signal_off_sec\": 10.0\n  },\n  \"seed\": 1\n",
    "  \"rea\": {\n    \"signal_on_sec\": 300.0,\n    \"signal_off_sec\": 10.0\n  },\n"
    "  \"bestpos_rtk\": {\n"
    "    \"enabled\": true,\n"
    "    \"stable_duration_sec\": 5.0,\n"
    "    \"min_used_satellites\": 6,\n"
    "    \"horizontal_std_m\": 0.01,\n"
    "    \"height_std_m\": 0.02\n"
    "  },\n"
    "  \"seed\": 1\n",
)

# Reproducibility manifest.
replace_once(
    "src/output/truth_writer.cpp",
    "    output << inner << \"  \\\"signal_off_ns\\\": \" << config.rea.signal_off_ns << \"\\n\";\n    output << inner << \"},\\n\";\n    output << inner << \"\\\"measurement_error\\\": {\\n\";\n",
    "    output << inner << \"  \\\"signal_off_ns\\\": \" << config.rea.signal_off_ns << \"\\n\";\n"
    "    output << inner << \"},\\n\";\n"
    "    output << inner << \"\\\"bestpos_rtk\\\": {\\n\";\n"
    "    output << inner << \"  \\\"enabled\\\": \" << bool_json(config.bestpos_rtk.enabled) << \",\\n\";\n"
    "    output << inner << \"  \\\"stable_duration_ns\\\": \" << config.bestpos_rtk.stable_duration_ns << \",\\n\";\n"
    "    output << inner << \"  \\\"min_used_satellites\\\": \" << config.bestpos_rtk.min_used_satellites << \",\\n\";\n"
    "    output << inner << \"  \\\"horizontal_std_m\\\": \" << config.bestpos_rtk.horizontal_std_m << \",\\n\";\n"
    "    output << inner << \"  \\\"height_std_m\\\": \" << config.bestpos_rtk.height_std_m << \"\\n\";\n"
    "    output << inner << \"},\\n\";\n"
    "    output << inner << \"\\\"measurement_error\\\": {\\n\";\n",
)

# New deterministic BESTPOS RTK state model.
write("src/model/bestpos_rtk_model.h", r'''#ifndef GNSS_SIM_SRC_MODEL_BESTPOS_RTK_MODEL_H_
#define GNSS_SIM_SRC_MODEL_BESTPOS_RTK_MODEL_H_

#include "gnss_sim/sim_config.h"
#include "solution/solution_engine.h"

#include <string>

namespace gnss_sim {

struct BestposRtkState {
    bool stability_active;
    bool fixed;
    SimTime stable_since;
};

void reset_bestpos_rtk_state(BestposRtkState* state);
bool update_bestpos_rtk_state(const BestposRtkConfig& config, const SimTime& epoch_time,
                              const PositionSolution& position, BestposRtkState* state,
                              std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_MODEL_BESTPOS_RTK_MODEL_H_
''')
write("src/model/bestpos_rtk_model.cpp", r'''#include "model/bestpos_rtk_model.h"

#include "gnss_sim/sim_time.h"

namespace gnss_sim {
namespace {

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool qualifying_position(const BestposRtkConfig& config, const PositionSolution& position) {
    return position.valid && position.status == ReceiverSolutionStatus::kSolComputed &&
           position.type == ReceiverSolutionType::kSingle && position.used_satellites >= config.min_used_satellites;
}

} // namespace

void reset_bestpos_rtk_state(BestposRtkState* state) {
    if (state != nullptr) {
        *state = {};
    }
}

bool update_bestpos_rtk_state(const BestposRtkConfig& config, const SimTime& epoch_time,
                              const PositionSolution& position, BestposRtkState* state,
                              std::string* error_message) {
    if (state == nullptr || config.stable_duration_ns < 0 || config.min_used_satellites < 1) {
        set_error(error_message, "BESTPOS RTK state request has invalid arguments");
        return false;
    }
    if (!config.enabled || !qualifying_position(config, position)) {
        reset_bestpos_rtk_state(state);
        return true;
    }
    if (state->fixed) {
        return true;
    }
    if (!state->stability_active) {
        state->stability_active = true;
        state->stable_since = epoch_time;
        if (config.stable_duration_ns == 0) {
            state->fixed = true;
        }
        return true;
    }

    std::int64_t stable_ns = 0;
    if (!difference_time_ns(epoch_time, state->stable_since, &stable_ns) || stable_ns < 0) {
        set_error(error_message, "BESTPOS RTK stability time is not monotonic");
        return false;
    }
    if (stable_ns >= config.stable_duration_ns) {
        state->fixed = true;
    }
    return true;
}

} // namespace gnss_sim
''')

# Build the new model and unit test.
replace_once(
    "CMakeLists.txt",
    "    src/model/atmosphere_model.cpp\n    src/model/cn0_model.cpp\n",
    "    src/model/atmosphere_model.cpp\n    src/model/bestpos_rtk_model.cpp\n    src/model/cn0_model.cpp\n",
)
replace_once(
    "CMakeLists.txt",
    "        tests/unit/test_atmosphere_model.cpp\n        tests/unit/test_cn0_model.cpp\n",
    "        tests/unit/test_atmosphere_model.cpp\n        tests/unit/test_bestpos_rtk_model.cpp\n        tests/unit/test_cn0_model.cpp\n",
)

write("tests/unit/test_bestpos_rtk_model.cpp", r'''#include "gnss_sim/sim_time.h"
#include "model/bestpos_rtk_model.h"

#include <gtest/gtest.h>
#include <string>

namespace {

gnss_sim::PositionSolution valid_position(int used_satellites = 8) {
    gnss_sim::PositionSolution position{};
    position.valid = true;
    position.status = gnss_sim::ReceiverSolutionStatus::kSolComputed;
    position.type = gnss_sim::ReceiverSolutionType::kSingle;
    position.used_satellites = used_satellites;
    return position;
}

gnss_sim::SimTime time_at(double sow_sec) {
    gnss_sim::SimTime time{};
    EXPECT_TRUE(gnss_sim::sim_time_from_week_sow(2300, sow_sec, &time));
    return time;
}

TEST(BestposRtkModel, FixesOnlyAfterContinuousStableDuration) {
    gnss_sim::BestposRtkConfig config{true, 5LL * gnss_sim::NANOSECONDS_PER_SECOND, 6, 0.01, 0.02};
    gnss_sim::BestposRtkState state{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(100.0), valid_position(), &state, &error_message));
    EXPECT_TRUE(state.stability_active);
    EXPECT_FALSE(state.fixed);
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(104.999), valid_position(), &state, &error_message));
    EXPECT_FALSE(state.fixed);
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(105.0), valid_position(), &state, &error_message));
    EXPECT_TRUE(state.fixed);
}

TEST(BestposRtkModel, InvalidOrLowSatellitePositionResetsFix) {
    gnss_sim::BestposRtkConfig config{true, 0, 6, 0.01, 0.02};
    gnss_sim::BestposRtkState state{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(100.0), valid_position(), &state, &error_message));
    ASSERT_TRUE(state.fixed);
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(101.0), valid_position(5), &state, &error_message));
    EXPECT_FALSE(state.stability_active);
    EXPECT_FALSE(state.fixed);

    gnss_sim::PositionSolution invalid{};
    invalid.status = gnss_sim::ReceiverSolutionStatus::kInsufficientObs;
    invalid.type = gnss_sim::ReceiverSolutionType::kNone;
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(102.0), invalid, &state, &error_message));
    EXPECT_FALSE(state.fixed);
}

TEST(BestposRtkModel, DisabledModelNeverFixes) {
    gnss_sim::BestposRtkConfig config{false, 0, 4, 0.01, 0.02};
    gnss_sim::BestposRtkState state{true, true, time_at(90.0)};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::update_bestpos_rtk_state(config, time_at(100.0), valid_position(), &state, &error_message));
    EXPECT_FALSE(state.stability_active);
    EXPECT_FALSE(state.fixed);
}

} // namespace
''')

# BESTPOS writer now serializes either the SPP solution or a truth-backed fixed solution.
replace_once(
    "src/output/novatel_solution_writer.h",
    "bool format_novatel_bestposa(const SimTime& time, const ReceiverTruth& truth, std::string* message,\n                             std::string* error_message);\n",
    "bool format_novatel_bestposa(const SolutionEpoch& solution, int tracked_satellites, const ReceiverTruth& truth,\n"
    "                             bool rtk_fixed, const BestposRtkConfig& rtk_config, std::string* message,\n"
    "                             std::string* error_message);\n",
)
regex_once(
    "src/output/novatel_solution_writer.cpp",
    r"bool format_novatel_bestposa\(.*?\n\}\n\nbool format_novatel_psrvela",
    r'''bool format_novatel_bestposa(const SolutionEpoch& solution, int tracked_satellites, const ReceiverTruth& truth,
                             bool rtk_fixed, const BestposRtkConfig& rtk_config, std::string* message,
                             std::string* error_message) {
    if (message == nullptr || tracked_satellites < 0 || tracked_satellites > 255 ||
        !consistent_position(solution.position) || !consistent_receiver_truth(truth) ||
        (rtk_fixed && !solution.position.valid)) {
        set_error(error_message, "BESTPOSA solution metadata is invalid");
        return false;
    }

    const PositionSolution& position = solution.position;
    const bool valid = position.valid;
    const double latitude_deg = rtk_fixed ? truth.latitude_deg : (valid ? position.latitude_deg : 0.0);
    const double longitude_deg = rtk_fixed ? truth.longitude_deg : (valid ? position.longitude_deg : 0.0);
    const double height_m = rtk_fixed ? truth.height_m : (valid ? position.height_m : 0.0);
    const double latitude_std_m = rtk_fixed ? rtk_config.horizontal_std_m : (valid ? position.latitude_std_m : 0.0);
    const double longitude_std_m = rtk_fixed ? rtk_config.horizontal_std_m : (valid ? position.longitude_std_m : 0.0);
    const double height_std_m = rtk_fixed ? rtk_config.height_std_m : (valid ? position.height_std_m : 0.0);
    const int used_satellites = valid ? position.used_satellites : 0;
    const char* status = rtk_fixed ? "SOL_COMPUTED" : receiver_solution_status_name(position.status);
    const char* type = rtk_fixed ? "NARROW_INT" : receiver_solution_type_name(position.type);

    std::ostringstream body;
    body.imbue(std::locale::classic());
    body << status << ',' << type << ',' << std::fixed << std::setprecision(11) << latitude_deg << ',' << longitude_deg
         << ',' << std::setprecision(4) << height_m << ",0.0000,WGS84," << latitude_std_m << ',' << longitude_std_m
         << ',' << height_std_m << ",\"\",0.000,0.000," << tracked_satellites << ',' << used_satellites
         << ",0,0,00,00,00,00";

    if (!novatel_ascii::frame("BESTPOSA", solution.time, body.str(), message)) {
        set_error(error_message, "BESTPOSA header time cannot be represented");
        return false;
    }
    return true;
}

bool format_novatel_psrvela''',
)

# Simulator state integration.
replace_once(
    "src/core/simulator.cpp",
    "#include \"model/atmosphere_model.h\"\n#include \"model/cn0_model.h\"\n",
    "#include \"model/atmosphere_model.h\"\n#include \"model/bestpos_rtk_model.h\"\n#include \"model/cn0_model.h\"\n",
)
replace_once(
    "src/core/simulator.cpp",
    "    SignalTrackingModelConfig tracking_config;\n    SolutionEngineState solution_state;\n",
    "    SignalTrackingModelConfig tracking_config;\n    SolutionEngineState solution_state;\n    BestposRtkState bestpos_rtk_state;\n",
)
replace_once(
    "src/core/simulator.cpp",
    "    reset_solution_engine_state(&runtime->solution_state);\n    ReceiverStartupTiming timing{};\n",
    "    reset_solution_engine_state(&runtime->solution_state);\n    reset_bestpos_rtk_state(&runtime->bestpos_rtk_state);\n    ReceiverStartupTiming timing{};\n",
)
replace_once(
    "src/core/simulator.cpp",
    "void receiver_power_off(RuntimeState* runtime, const SimTime& time) {\n    reset_solution_engine_state(&runtime->solution_state);\n",
    "void receiver_power_off(RuntimeState* runtime, const SimTime& time) {\n"
    "    reset_solution_engine_state(&runtime->solution_state);\n"
    "    reset_bestpos_rtk_state(&runtime->bestpos_rtk_state);\n",
)
replace_once(
    "src/core/simulator.cpp",
    "void receiver_signal_off(RuntimeState* runtime, const SimTime& time) {\n    for (SatelliteRuntime& satellite : runtime->satellites) {\n",
    "void receiver_signal_off(RuntimeState* runtime, const SimTime& time) {\n"
    "    reset_bestpos_rtk_state(&runtime->bestpos_rtk_state);\n"
    "    for (SatelliteRuntime& satellite : runtime->satellites) {\n",
)
replace_once(
    "src/core/simulator.cpp",
    "bool emit_epoch_logs(const SimConfig& config, const ScenarioEpochState& scenario,\n                     const std::vector<MeasurementObservation>& measurements, int tracked_satellites,\n                     const SolutionEpoch& solution, const ReceiverTruth& receiver, std::ofstream* output,\n",
    "bool emit_epoch_logs(const SimConfig& config, const ScenarioEpochState& scenario,\n"
    "                     const std::vector<MeasurementObservation>& measurements, int tracked_satellites,\n"
    "                     const SolutionEpoch& solution, const ReceiverTruth& receiver, bool bestpos_rtk_fixed,\n"
    "                     std::ofstream* output,\n",
)
replace_once(
    "src/core/simulator.cpp",
    "    if (!format_novatel_bestposa(scenario.time, receiver, &message, error_message) ||\n        !write_message(output, message, error_message)) {\n",
    "    if (!format_novatel_bestposa(solution, tracked_satellites, receiver, bestpos_rtk_fixed, config.bestpos_rtk,\n"
    "                                &message, error_message) ||\n"
    "        !write_message(output, message, error_message)) {\n",
)
old_solve = '''        SolutionEpoch solution{};
        const MeasurementObservation* data = measurements.empty() ? nullptr : measurements.data();
        if (!solve_receiver_epoch(receiver_navigation_store(runtime.navigation), current_time, data,
                                  static_cast<int>(measurements.size()), config.solution_elevation_mask_deg,
                                  config.atmosphere_mode, &runtime.solution_state, &solution, error_message) ||
            !truth_writer_write_solution(truth_writer, solution, tracked_satellites, error_message) ||
            !emit_epoch_logs(config, scenario, measurements, tracked_satellites, solution, runtime.receiver, &output,
                             &result, error_message)) {
            ok = false;
            break;
        }
'''
new_solve = '''        SolutionEpoch solution{};
        const MeasurementObservation* data = measurements.empty() ? nullptr : measurements.data();
        if (!solve_receiver_epoch(receiver_navigation_store(runtime.navigation), current_time, data,
                                  static_cast<int>(measurements.size()), config.solution_elevation_mask_deg,
                                  config.atmosphere_mode, &runtime.solution_state, &solution, error_message) ||
            !update_bestpos_rtk_state(config.bestpos_rtk, current_time, solution.position,
                                      &runtime.bestpos_rtk_state, error_message) ||
            !truth_writer_write_solution(truth_writer, solution, tracked_satellites, error_message) ||
            !emit_epoch_logs(config, scenario, measurements, tracked_satellites, solution, runtime.receiver,
                             runtime.bestpos_rtk_state.fixed, &output, &result, error_message)) {
            ok = false;
            break;
        }
'''
replace_once("src/core/simulator.cpp", old_solve, new_solve)

# Config tests.
replace_once(
    "tests/unit/test_sim_config.cpp",
    "    EXPECT_DOUBLE_EQ(config.measurement_error.rea_fade.duration_sec, 0.25);\n    EXPECT_FALSE(config.multipath_enabled);\n",
    "    EXPECT_DOUBLE_EQ(config.measurement_error.rea_fade.duration_sec, 0.25);\n"
    "    EXPECT_TRUE(config.bestpos_rtk.enabled);\n"
    "    EXPECT_EQ(config.bestpos_rtk.stable_duration_ns, 5LL * gnss_sim::NANOSECONDS_PER_SECOND);\n"
    "    EXPECT_EQ(config.bestpos_rtk.min_used_satellites, 6);\n"
    "    EXPECT_DOUBLE_EQ(config.bestpos_rtk.horizontal_std_m, 0.01);\n"
    "    EXPECT_DOUBLE_EQ(config.bestpos_rtk.height_std_m, 0.02);\n"
    "    EXPECT_FALSE(config.multipath_enabled);\n",
)
replace_once(
    "tests/unit/test_sim_config.cpp",
    "TEST_F(SimConfigTest, RejectsInvalidMeasurementErrorConfiguration) {\n",
    r'''TEST_F(SimConfigTest, LoadsAndValidatesBestposRtkOverrides) {
    write_test_config(R"json({
        "bestpos_rtk": {
            "enabled": false,
            "stable_duration_sec": 3.5,
            "min_used_satellites": 7,
            "horizontal_std_m": 0.015,
            "height_std_m": 0.03
        }
    })json");

    gnss_sim::SimConfig config{};
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message)) << error_message;
    EXPECT_FALSE(config.bestpos_rtk.enabled);
    EXPECT_EQ(config.bestpos_rtk.stable_duration_ns, 3500000000LL);
    EXPECT_EQ(config.bestpos_rtk.min_used_satellites, 7);
    EXPECT_DOUBLE_EQ(config.bestpos_rtk.horizontal_std_m, 0.015);
    EXPECT_DOUBLE_EQ(config.bestpos_rtk.height_std_m, 0.03);

    write_test_config(R"json({"bestpos_rtk":{"min_used_satellites":3}})json");
    EXPECT_FALSE(gnss_sim::load_sim_config_json(TEST_CONFIG_PATH, &config, &error_message));
    EXPECT_NE(error_message.find("bestpos_rtk"), std::string::npos);
}

TEST_F(SimConfigTest, RejectsInvalidMeasurementErrorConfiguration) {
''',
)

# Writer tests replace the old always-truth expectation.
replace_once(
    "tests/unit/test_output_writers.cpp",
    "gnss_sim::MeasurementObservation observation(const char* satellite_id, gnss_sim::SignalId signal_id, int glonass_fcn,\n",
    "std::string ascii_body(const std::string& message) {\n"
    "    const std::size_t semicolon = message.find(';');\n"
    "    const std::size_t star = message.rfind('*');\n"
    "    if (semicolon == std::string::npos || star == std::string::npos || star <= semicolon) {\n"
    "        return std::string();\n"
    "    }\n"
    "    return message.substr(semicolon + 1, star - semicolon - 1);\n"
    "}\n\n"
    "gnss_sim::MeasurementObservation observation(const char* satellite_id, gnss_sim::SignalId signal_id, int glonass_fcn,\n",
)
regex_once(
    "tests/unit/test_output_writers.cpp",
    r"TEST\(NovatelSolutionWriter, BestPosATruthIsByteStableAndDailyAnalysisCompatible\).*?\n\}\n\nTEST\(NovatelSolutionWriter, ValidPsrPosAIsByteStable\)",
    r'''TEST(NovatelSolutionWriter, BestPosAMirrorsPsrPosBeforeRtkFix) {
    gnss_sim::SolutionEpoch solution{};
    solution.time = writer_time();
    solution.position.valid = true;
    solution.position.status = gnss_sim::ReceiverSolutionStatus::kSolComputed;
    solution.position.type = gnss_sim::ReceiverSolutionType::kSingle;
    solution.position.latitude_deg = 20.1;
    solution.position.longitude_deg = 120.1;
    solution.position.height_m = 101.0;
    solution.position.latitude_std_m = 0.25;
    solution.position.longitude_std_m = 0.30;
    solution.position.height_std_m = 0.50;
    solution.position.used_satellites = 6;
    gnss_sim::ReceiverTruth truth{};
    truth.latitude_deg = 20.0;
    truth.longitude_deg = 120.0;
    truth.height_m = 100.0;
    const gnss_sim::BestposRtkConfig config{true, 5000000000LL, 6, 0.01, 0.02};

    std::string bestpos;
    std::string psrpos;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_bestposa(solution, 8, truth, false, config, &bestpos, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_psrposa(solution, 8, &psrpos, &error_message)) << error_message;
    EXPECT_EQ(ascii_body(bestpos), ascii_body(psrpos));
}

TEST(NovatelSolutionWriter, BestPosAFixedEpochUsesTruthAndNarrowInt) {
    gnss_sim::SolutionEpoch solution{};
    solution.time = writer_time();
    solution.position.valid = true;
    solution.position.status = gnss_sim::ReceiverSolutionStatus::kSolComputed;
    solution.position.type = gnss_sim::ReceiverSolutionType::kSingle;
    solution.position.latitude_deg = 20.1;
    solution.position.longitude_deg = 120.1;
    solution.position.height_m = 101.0;
    solution.position.latitude_std_m = 0.25;
    solution.position.longitude_std_m = 0.30;
    solution.position.height_std_m = 0.50;
    solution.position.used_satellites = 6;
    gnss_sim::ReceiverTruth truth{};
    truth.latitude_deg = 20.0;
    truth.longitude_deg = 120.0;
    truth.height_m = 100.0;
    const gnss_sim::BestposRtkConfig config{true, 5000000000LL, 6, 0.01, 0.02};

    std::string message;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_bestposa(solution, 8, truth, true, config, &message, &error_message))
        << error_message;
    EXPECT_EQ(ascii_body(message),
              "SOL_COMPUTED,NARROW_INT,20.00000000000,120.00000000000,100.0000,0.0000,WGS84,"
              "0.0100,0.0100,0.0200,\"\",0.000,0.000,8,6,0,0,00,00,00,00");
}

TEST(NovatelSolutionWriter, InvalidBestPosMatchesInvalidPsrPosInsteadOfTruth) {
    gnss_sim::SolutionEpoch solution{};
    solution.time = writer_time();
    solution.position.status = gnss_sim::ReceiverSolutionStatus::kInsufficientObs;
    solution.position.type = gnss_sim::ReceiverSolutionType::kNone;
    gnss_sim::ReceiverTruth truth{};
    truth.latitude_deg = 20.0;
    truth.longitude_deg = 120.0;
    truth.height_m = 100.0;
    const gnss_sim::BestposRtkConfig config{true, 5000000000LL, 6, 0.01, 0.02};

    std::string bestpos;
    std::string psrpos;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_bestposa(solution, 3, truth, false, config, &bestpos, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_psrposa(solution, 3, &psrpos, &error_message)) << error_message;
    EXPECT_EQ(ascii_body(bestpos), ascii_body(psrpos));
}

TEST(NovatelSolutionWriter, ValidPsrPosAIsByteStable)''',
)

# Scenario-level FIX/reset/refix behavior.
replace_once(
    "tests/integration/test_scenarios.cpp",
    "#include <gtest/gtest.h>\n#include <string>\n",
    "#include <gtest/gtest.h>\n#include <string>\n#include <vector>\n",
)
replace_once(
    "tests/integration/test_scenarios.cpp",
    "std::size_t occurrence_count(const std::string& text, const std::string& needle) {\n",
    r'''std::vector<std::string> bestpos_states(const std::string& log) {
    std::vector<std::string> states;
    std::size_t position = 0;
    while ((position = log.find("#BESTPOSA,", position)) != std::string::npos) {
        const std::size_t semicolon = log.find(';', position);
        const std::size_t first_comma = semicolon == std::string::npos ? std::string::npos : log.find(',', semicolon + 1);
        const std::size_t second_comma = first_comma == std::string::npos ? std::string::npos : log.find(',', first_comma + 1);
        if (semicolon == std::string::npos || first_comma == std::string::npos || second_comma == std::string::npos) {
            break;
        }
        states.push_back(log.substr(semicolon + 1, second_comma - semicolon - 1));
        position = second_comma + 1;
    }
    return states;
}

bool has_fix_reset_refix(const std::vector<std::string>& states) {
    bool saw_fix = false;
    bool saw_reset = false;
    for (const std::string& state : states) {
        if (!saw_fix && state == "SOL_COMPUTED,NARROW_INT") {
            saw_fix = true;
        } else if (saw_fix && !saw_reset && state != "SOL_COMPUTED,NARROW_INT") {
            saw_reset = true;
        } else if (saw_fix && saw_reset && state == "SOL_COMPUTED,NARROW_INT") {
            return true;
        }
    }
    return false;
}

std::size_t occurrence_count(const std::string& text, const std::string& needle) {
''',
)
replace_once(
    "tests/integration/test_scenarios.cpp",
    "    config.duration_ns = 10LL * gnss_sim::NANOSECONDS_PER_SECOND;\n",
    "    config.duration_ns = 10LL * gnss_sim::NANOSECONDS_PER_SECOND;\n"
    "    config.bestpos_rtk.stable_duration_ns = 1LL * gnss_sim::NANOSECONDS_PER_SECOND;\n",
)
replace_once(
    "tests/integration/test_scenarios.cpp",
    "    EXPECT_EQ(occurrence_count(log, \"#BESTPOSA,\"), 50U);\n    EXPECT_EQ(occurrence_count(log, gnss_sim::simulator_device_marker()), 0U);\n",
    "    EXPECT_EQ(occurrence_count(log, \"#BESTPOSA,\"), 50U);\n"
    "    EXPECT_GT(occurrence_count(log, \";SOL_COMPUTED,SINGLE,\"), 0U);\n"
    "    EXPECT_GT(occurrence_count(log, \";SOL_COMPUTED,NARROW_INT,\"), 0U);\n"
    "    EXPECT_EQ(occurrence_count(log, gnss_sim::simulator_device_marker()), 0U);\n",
)
insert_scenario_tests = r'''
TEST(StreamingSimulator, ReaBestPosFixResetsOnSignalLossAndRefixesAfterStability) {
    gnss_sim::SimConfig config = base_config(gnss_sim::ScenarioType::REA);
    config.sampling_rate_hz = 10;
    config.duration_ns = 18LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_on_ns = 7LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.rea.signal_off_ns = 2LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.bestpos_rtk.stable_duration_ns = 1LL * gnss_sim::NANOSECONDS_PER_SECOND;
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run(config, "rea_bestpos_rtk", &summary, &error_message)) << error_message;
    const std::vector<std::string> states = bestpos_states(read_file(test_output_path("rea_bestpos_rtk")));
    EXPECT_TRUE(has_fix_reset_refix(states));
    std::remove(test_output_path("rea_bestpos_rtk").c_str());
}

TEST(StreamingSimulator, TtffBestPosFixResetsAcrossPowerCycleAndRefixes) {
    gnss_sim::SimConfig config = base_config(gnss_sim::ScenarioType::TTFF);
    config.ttff.startup_mode = gnss_sim::StartupMode::HOT;
    config.sampling_rate_hz = 10;
    config.duration_ns = 18LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.ttff.power_on_ns = 7LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.ttff.power_off_ns = 2LL * gnss_sim::NANOSECONDS_PER_SECOND;
    config.bestpos_rtk.stable_duration_ns = 1LL * gnss_sim::NANOSECONDS_PER_SECOND;
    gnss_sim::SimulatorRunSummary summary{};
    std::string error_message;
    ASSERT_TRUE(run(config, "ttff_bestpos_rtk", &summary, &error_message)) << error_message;
    const std::vector<std::string> states = bestpos_states(read_file(test_output_path("ttff_bestpos_rtk")));
    EXPECT_TRUE(has_fix_reset_refix(states));
    std::remove(test_output_path("ttff_bestpos_rtk").c_str());
}

'''
replace_once(
    "tests/integration/test_scenarios.cpp",
    "TEST(StreamingSimulator, FutureOnlyEphemerisDoesNotAbortEarlierEpochs) {\n",
    insert_scenario_tests + "TEST(StreamingSimulator, FutureOnlyEphemerisDoesNotAbortEarlierEpochs) {\n",
)

# Manifest test confirms reproducible model parameters are retained.
replace_once(
    "tests/integration/test_truth_outputs.cpp",
    "    EXPECT_NE(manifest.find(\"\\\"bestpos_messages\\\": 10\"), std::string::npos);\n",
    "    EXPECT_NE(manifest.find(\"\\\"bestpos_messages\\\": 10\"), std::string::npos);\n"
    "    EXPECT_NE(manifest.find(\"\\\"bestpos_rtk\\\": {\"), std::string::npos);\n"
    "    EXPECT_NE(manifest.find(\"\\\"stable_duration_ns\\\": 5000000000\"), std::string::npos);\n",
)

print("issue 79 patch applied")
