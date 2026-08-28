from pathlib import Path


def replace_once(path: str, old: str, new: str) -> None:
    target = Path(path)
    text = target.read_text(encoding="utf-8")
    if text.count(old) != 1:
        raise RuntimeError(f"guard failed for {path}: expected one anchor, got {text.count(old)}")
    target.write_text(text.replace(old, new, 1), encoding="utf-8")


# Public run summary exposes the new receiver-log stream count.
replace_once(
    "include/gnss_sim/simulator.h",
    "    std::uint64_t psrvel_messages;\n    std::uint64_t nav_messages;",
    "    std::uint64_t psrvel_messages;\n    std::uint64_t bestpos_messages;\n    std::uint64_t nav_messages;",
)

# Formatter API: BESTPOS is truth instrumentation and consumes ReceiverTruth directly.
replace_once(
    "src/output/novatel_solution_writer.h",
    '#include "solution/solution_engine.h"\n',
    '#include "model/receiver_truth.h"\n#include "solution/solution_engine.h"\n',
)
replace_once(
    "src/output/novatel_solution_writer.h",
    "bool format_novatel_psrvela(const SolutionEpoch& solution, std::string* message, std::string* error_message);",
    "bool format_novatel_psrvela(const SolutionEpoch& solution, std::string* message, std::string* error_message);\n"
    "bool format_novatel_bestposa(const SimTime& time, const ReceiverTruth& truth, std::string* message,\n"
    "                              std::string* error_message);",
)

# Implement a byte-stable, high-quality NARROW_INT truth record.
replace_once(
    "src/output/novatel_solution_writer.cpp",
    "bool consistent_velocity(const VelocitySolution& velocity) {",
    "bool consistent_receiver_truth(const ReceiverTruth& truth) {\n"
    "    return std::isfinite(truth.latitude_deg) && truth.latitude_deg >= -90.0 && truth.latitude_deg <= 90.0 &&\n"
    "           std::isfinite(truth.longitude_deg) && truth.longitude_deg >= -180.0 && truth.longitude_deg <= 180.0 &&\n"
    "           std::isfinite(truth.height_m);\n"
    "}\n\n"
    "bool consistent_velocity(const VelocitySolution& velocity) {",
)
replace_once(
    "src/output/novatel_solution_writer.cpp",
    "bool format_novatel_psrvela(const SolutionEpoch& solution, std::string* message, std::string* error_message) {",
    "bool format_novatel_bestposa(const SimTime& time, const ReceiverTruth& truth, std::string* message,\n"
    "                              std::string* error_message) {\n"
    "    if (message == nullptr || !consistent_receiver_truth(truth)) {\n"
    "        set_error(error_message, \"BESTPOSA truth metadata is invalid\");\n"
    "        return false;\n"
    "    }\n\n"
    "    std::ostringstream body;\n"
    "    body.imbue(std::locale::classic());\n"
    "    body << \"SOL_COMPUTED,NARROW_INT,\" << std::fixed << std::setprecision(11) << truth.latitude_deg << ','\n"
    "         << truth.longitude_deg << ',' << std::setprecision(4) << truth.height_m\n"
    "         << \",0.0000,WGS84,0.0010,0.0010,0.0010,\\\"\\\",0.000,0.000,0,0,0,0,00,00,00,00\";\n\n"
    "    if (!novatel_ascii::frame(\"BESTPOSA\", time, body.str(), message)) {\n"
    "        set_error(error_message, \"BESTPOSA header time cannot be represented\");\n"
    "        return false;\n"
    "    }\n"
    "    return true;\n"
    "}\n\n"
    "bool format_novatel_psrvela(const SolutionEpoch& solution, std::string* message, std::string* error_message) {",
)

# Emit BESTPOS on every powered epoch. The caller already suppresses the complete log set while TTFF power is off.
replace_once(
    "src/core/simulator.cpp",
    "                     const SolutionEpoch& solution, std::ofstream* output, SimulatorRunSummary* summary,\n"
    "                     std::string* error_message) {",
    "                     const SolutionEpoch& solution, const ReceiverTruth& receiver, std::ofstream* output,\n"
    "                     SimulatorRunSummary* summary, std::string* error_message) {",
)
replace_once(
    "src/core/simulator.cpp",
    "    ++summary->psrvel_messages;\n    return true;",
    "    ++summary->psrvel_messages;\n\n"
    "    if (!format_novatel_bestposa(scenario.time, receiver, &message, error_message) ||\n"
    "        !write_message(output, message, error_message)) {\n"
    "        return false;\n"
    "    }\n"
    "    ++summary->bestpos_messages;\n"
    "    return true;",
)
replace_once(
    "src/core/simulator.cpp",
    "            !emit_epoch_logs(config, scenario, measurements, tracked_satellites, solution, &output, &result,\n"
    "                             error_message)) {",
    "            !emit_epoch_logs(config, scenario, measurements, tracked_satellites, solution, runtime.receiver, &output,\n"
    "                             &result, error_message)) {",
)

# Persist the stream count in the existing run manifest.
replace_once(
    "src/output/truth_writer.cpp",
    '             << "    \\\"psrvel_messages\\\": " << summary.psrvel_messages << ",\\n"\n'
    '             << "    \\\"nav_messages\\\": " << summary.nav_messages << ",\\n"',
    '             << "    \\\"psrvel_messages\\\": " << summary.psrvel_messages << ",\\n"\n'
    '             << "    \\\"bestpos_messages\\\": " << summary.bestpos_messages << ",\\n"\n'
    '             << "    \\\"nav_messages\\\": " << summary.nav_messages << ",\\n"',
)

# Byte golden and downstream-contract fields.
replace_once(
    "tests/unit/test_output_writers.cpp",
    "TEST(NovatelSolutionWriter, ValidPsrPosAIsByteStable) {",
    "TEST(NovatelSolutionWriter, BestPosATruthIsByteStableAndDailyAnalysisCompatible) {\n"
    "    gnss_sim::ReceiverTruth truth{};\n"
    "    truth.latitude_deg = 20.0;\n"
    "    truth.longitude_deg = 120.0;\n"
    "    truth.height_m = 100.0;\n\n"
    "    std::string message;\n"
    "    std::string error_message;\n"
    "    ASSERT_TRUE(gnss_sim::format_novatel_bestposa(writer_time(), truth, &message, &error_message))\n"
    "        << error_message;\n"
    "    EXPECT_EQ(message, \"#BESTPOSA,COM1,0,0.0,FINE,2300,12345.678,00000000,0,0;\"\n"
    "                       \"SOL_COMPUTED,NARROW_INT,20.00000000000,120.00000000000,100.0000,0.0000,WGS84,\"\n"
    "                       \"0.0010,0.0010,0.0010,\\\"\\\",0.000,0.000,0,0,0,0,00,00,00,00*923a3330\\r\\n\");\n"
    "    EXPECT_NE(message.find(\";SOL_COMPUTED,NARROW_INT,\"), std::string::npos);\n"
    "}\n\n"
    "TEST(NovatelSolutionWriter, ValidPsrPosAIsByteStable) {",
)

# Scenario count/availability behavior.
replace_once(
    "tests/integration/test_scenarios.cpp",
    "bool file_contains(const std::string& path, const std::string& text) {\n"
    "    std::ifstream input(path, std::ios::binary);\n"
    "    const std::string contents((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());\n"
    "    return contents.find(text) != std::string::npos;\n"
    "}",
    "std::string read_file(const std::string& path) {\n"
    "    std::ifstream input(path, std::ios::binary);\n"
    "    return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());\n"
    "}\n\n"
    "bool file_contains(const std::string& path, const std::string& text) {\n"
    "    return read_file(path).find(text) != std::string::npos;\n"
    "}\n\n"
    "std::size_t occurrence_count(const std::string& text, const std::string& needle) {\n"
    "    std::size_t count = 0;\n"
    "    std::size_t position = 0;\n"
    "    while ((position = text.find(needle, position)) != std::string::npos) {\n"
    "        ++count;\n"
    "        position += needle.size();\n"
    "    }\n"
    "    return count;\n"
    "}",
)
replace_once(
    "tests/integration/test_scenarios.cpp",
    "    EXPECT_EQ(summary.psrvel_messages, 50U);\n    EXPECT_EQ(summary.power_on_events, 1U);",
    "    EXPECT_EQ(summary.psrvel_messages, 50U);\n"
    "    EXPECT_EQ(summary.bestpos_messages, 50U);\n"
    "    EXPECT_EQ(occurrence_count(read_file(test_output_path(\"ks\")), \"#BESTPOSA,\"), 50U);\n"
    "    EXPECT_EQ(summary.power_on_events, 1U);",
)
replace_once(
    "tests/integration/test_scenarios.cpp",
    "    EXPECT_EQ(summary.psrvel_messages, 60U);\n    EXPECT_EQ(summary.power_off_events, 0U);",
    "    EXPECT_EQ(summary.psrvel_messages, 60U);\n"
    "    EXPECT_EQ(summary.bestpos_messages, 60U);\n"
    "    EXPECT_EQ(occurrence_count(read_file(test_output_path(\"rea\")), \"#BESTPOSA,\"), 60U);\n"
    "    EXPECT_EQ(summary.power_off_events, 0U);",
)
replace_once(
    "tests/integration/test_scenarios.cpp",
    "    EXPECT_EQ(summary.psrvel_messages, 60U);\n    EXPECT_EQ(summary.power_on_events, 2U);",
    "    EXPECT_EQ(summary.psrvel_messages, 60U);\n"
    "    EXPECT_EQ(summary.bestpos_messages, 60U);\n"
    "    EXPECT_EQ(occurrence_count(read_file(test_output_path(name)), \"#BESTPOSA,\"), 60U);\n"
    "    EXPECT_EQ(summary.power_on_events, 2U);",
)

# Manifest explicitly records BESTPOS count.
replace_once(
    "tests/integration/test_truth_outputs.cpp",
    "    EXPECT_NE(manifest.find(\"\\\"random_seed\\\": 7\"), std::string::npos);",
    "    EXPECT_NE(manifest.find(\"\\\"random_seed\\\": 7\"), std::string::npos);\n"
    "    EXPECT_NE(manifest.find(\"\\\"bestpos_messages\\\": 10\"), std::string::npos);\n"
    "    EXPECT_EQ(summary.bestpos_messages, 10U);",
)

print("issue 54 BESTPOSA patch applied")
