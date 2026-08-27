from pathlib import Path

path = Path("tests/integration/test_all_signal_residuals.cpp")
text = path.read_text()

new_helper = '''int required_rtklib_message_type(const gnss_sim::SignalDefinition& definition) {
    switch (definition.nav_message_family) {
        case gnss_sim::NavMessageFamily::kGpsLnav:
        case gnss_sim::NavMessageFamily::kQzssLnav:
            return NAV_LNAV;
        case gnss_sim::NavMessageFamily::kGpsCnav:
        case gnss_sim::NavMessageFamily::kQzssCnav:
            return NAV_CNAV;
        case gnss_sim::NavMessageFamily::kGpsCnav2:
        case gnss_sim::NavMessageFamily::kQzssCnav2:
            return NAV_CNV2;
        case gnss_sim::NavMessageFamily::kGlonassFdma:
            return NAV_FDMA;
        case gnss_sim::NavMessageFamily::kGlonassL3Oc:
            return NAV_L3OC;
        case gnss_sim::NavMessageFamily::kGalileoInav:
            return NAV_INAV;
        case gnss_sim::NavMessageFamily::kGalileoFnav:
            return NAV_FNAV;
        case gnss_sim::NavMessageFamily::kGalileoCnav:
            return 0;
        case gnss_sim::NavMessageFamily::kBeidouD1D2:
            return NAV_D1 | NAV_D2 | NAV_D1D2;
        case gnss_sim::NavMessageFamily::kBeidouBcnav1:
            return NAV_CNV1;
        case gnss_sim::NavMessageFamily::kBeidouBcnav2:
            return NAV_CNV2;
        case gnss_sim::NavMessageFamily::kBeidouBcnav3:
            return NAV_CNV3;
    }
    return 0;
}'''
start_marker = "int required_rtklib_message_type("
end_marker = "\n\nTEST(V1Acceptance, EveryFrozenSignalRunsTruthStateCodeAndDopplerResidualChecks)"
start = text.find(start_marker)
end = text.find(end_marker, start)
if start < 0 or end < 0 or text.find(start_marker, start + 1) >= 0:
    raise RuntimeError("required_rtklib_message_type function boundary mismatch")
text = text[:start] + new_helper + text[end:]

old_family = '''        const std::string message_family = fields[column.at("broadcast_message_family")];
        const int required_message_type = required_rtklib_message_type(*definition, message_family);'''
new_family = '''        // Code residuals must validate the signal's requested NAV family. The
        // truth broadcast_message_family can describe a generic fallback used
        // to keep geometry/Doppler generation running when that family is absent.
        const int required_message_type = required_rtklib_message_type(*definition);'''
if text.count(old_family) != 1:
    raise RuntimeError("message-family call-site anchor mismatch")
text = text.replace(old_family, new_family, 1)

old_decl = '''    std::map<std::string, SignalResidualStats> stats;
    std::set<std::string> seen_signals;
    std::set<std::string> code_unavailable_signals;'''
new_decl = '''    std::map<std::string, SignalResidualStats> stats;
    std::set<std::string> seen_signals;'''
if text.count(old_decl) != 1:
    raise RuntimeError("stats declaration anchor mismatch")
text = text.replace(old_decl, new_decl, 1)

old_insert = '''            ++signal_stats.code_unavailable;
            code_unavailable_signals.insert(signal_name);'''
new_insert = '''            ++signal_stats.code_unavailable;'''
if text.count(old_insert) != 1:
    raise RuntimeError("unavailable insert anchor mismatch")
text = text.replace(old_insert, new_insert, 1)

old_checks = '''        EXPECT_GT(signal_stats.code_residuals + signal_stats.code_unavailable, 0U)
            << "every V1 frequency must execute a code-residual/bias availability check: " << signal_name;
        if (signal_stats.code_residuals > 0U) {
            EXPECT_LT(signal_stats.max_abs_code_m, 0.02)
                << "code residual exceeds the RANGEA millimetre serialization floor: " << signal_name;
        }
    }

    EXPECT_EQ(code_unavailable_signals, (std::set<std::string>{"Galileo E6"}));'''
new_checks = '''        EXPECT_GT(signal_stats.code_residuals + signal_stats.code_unavailable, 0U)
            << "every V1 frequency must execute a code-residual/bias availability check: " << signal_name;
        if (signal_stats.code_residuals > 0U) {
            EXPECT_LT(signal_stats.max_abs_code_m, 0.02)
                << "code residual exceeds the RANGEA millimetre serialization floor: " << signal_name;
        }
        std::fprintf(stderr, "CODE_COVERAGE signal=%s residual_rows=%llu unavailable_rows=%llu max_abs_code=%.9f\\n",
                     signal_name.c_str(), static_cast<unsigned long long>(signal_stats.code_residuals),
                     static_cast<unsigned long long>(signal_stats.code_unavailable), signal_stats.max_abs_code_m);
        if (signal_name == "Galileo E6") {
            EXPECT_EQ(signal_stats.code_residuals, 0U) << "E6 must remain unavailable until HAS code bias is modeled";
            EXPECT_GT(signal_stats.code_unavailable, 0U);
        } else {
            EXPECT_GT(signal_stats.code_residuals, 0U)
                << "compact fixture must exercise at least one strict code residual for every non-E6 signal: "
                << signal_name;
        }
    }'''
if text.count(old_checks) != 1:
    raise RuntimeError("coverage check anchor mismatch")
text = text.replace(old_checks, new_checks, 1)

path.write_text(text)
print("requested-family code validation and 20/21 coverage policy applied")
