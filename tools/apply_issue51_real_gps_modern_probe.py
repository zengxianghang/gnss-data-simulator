from pathlib import Path

path = Path("tests/integration/test_all_signal_residuals.cpp")
text = path.read_text()

if "#include <rtklib_signal_bias_ext.h>" not in text:
    old = '''#include <rtklib.h>
#include <rtklib_residual_ext.h>
}'''
    new = '''#include <rtklib.h>
#include <rtklib_residual_ext.h>
#include <rtklib_signal_bias_ext.h>
}'''
    if text.count(old) != 1:
        raise RuntimeError("RTKLIB include anchor mismatch")
    text = text.replace(old, new, 1)

marker = '''TEST(V1Acceptance, EveryFrozenSignalRunsTruthStateCodeAndDopplerResidualChecks) {'''
if text.count(marker) != 1:
    raise RuntimeError("V1 residual test marker mismatch")

probe = r'''TEST(V1Acceptance, RealDlrGpsModernCompanionsExposePinnedRtklibStatus) {
    struct ProbeSignal {
        const char* name;
        const char* rinex_code;
        int required_message_type;
    };
    const ProbeSignal cnav_signals[] = {
        {"GPS L2C", "2S", NAV_CNAV},
        {"GPS L5Q", "5Q", NAV_CNAV},
    };
    const ProbeSignal cnv2_signals[] = {
        {"GPS L1C", "1L", NAV_CNV2},
    };

    auto probe_nav = [](const char* path_text, const ProbeSignal* signals, std::size_t signal_count,
                        int target_message_type, const char* source_label) {
        nav_t nav{};
        ASSERT_TRUE(load_nav(path_text, &nav)) << path_text;
        int gps_family_records = 0;
        int healthy_records = 0;
        int healthy_bias_records = 0;
        for (int index = 0; index < nav.n; ++index) {
            const eph_t& source_eph = nav.eph[index];
            if (satsys(source_eph.sat, nullptr) != SYS_GPS || source_eph.hdr.msg_type != target_message_type) {
                continue;
            }
            ++gps_family_records;
            if (source_eph.svh == 0) {
                ++healthy_records;
            }
            char satellite_id[8]{};
            satno2id(source_eph.sat, satellite_id);
            int week = 0;
            const double toe_sow = time2gpst(source_eph.toe, &week);
            std::fprintf(stderr, "REAL_GPS_MODERN_RECORD source=%s sat=%s msg_type=%d svh=%d week=%d toe=%.3f\n",
                         source_label, satellite_id, source_eph.hdr.msg_type, source_eph.svh, week, toe_sow);

            bool all_bias_available = true;
            for (std::size_t signal_index = 0; signal_index < signal_count; ++signal_index) {
                int frequency_index = 0;
                const unsigned char code = obs2code_ext(signals[signal_index].rinex_code, &frequency_index);
                ASSERT_NE(code, CODE_NONE) << signals[signal_index].name;
                eph_t selected_eph{};
                geph_t unused_geph{};
                rtklib_signal_bias_info_ext_t eph_info{};
                const int eph_status = rtklib_signal_ephemeris_ext(
                    source_eph.toe, source_eph.sat, code, signals[signal_index].required_message_type, &nav,
                    &selected_eph, &unused_geph, &eph_info);
                double bias_m = 0.0;
                rtklib_signal_bias_info_ext_t bias_info{};
                const int bias_status = rtklib_signal_code_bias_ext(
                    source_eph.toe, source_eph.sat, code, signals[signal_index].required_message_type, &nav,
                    &bias_m, &bias_info);
                std::fprintf(stderr,
                             "REAL_GPS_MODERN_SIGNAL source=%s sat=%s signal=%s svh=%d eph_status=%d eph_type=%d "
                             "bias_status=%d bias_type=%d bias_m=%.12f\n",
                             source_label, satellite_id, signals[signal_index].name, source_eph.svh, eph_status,
                             eph_info.message_type, bias_status, bias_info.message_type, bias_m);
                all_bias_available = all_bias_available && eph_status == 1 && bias_status == 1;
            }
            if (source_eph.svh == 0 && all_bias_available) {
                ++healthy_bias_records;
            }
        }
        std::fprintf(stderr,
                     "REAL_GPS_MODERN_SUMMARY source=%s msg_type=%d records=%d healthy=%d healthy_bias=%d\n",
                     source_label, target_message_type, gps_family_records, healthy_records, healthy_bias_records);
        EXPECT_GT(gps_family_records, 0) << source_label;
        free_nav(&nav);
    };

    probe_nav("/tmp/issue51_brd400dlr_2026180_trim.rnx", cnav_signals,
              sizeof(cnav_signals) / sizeof(cnav_signals[0]), NAV_CNAV, "DLR_2026_CNAV");
    probe_nav("/tmp/issue51_orekit_dlr_2022278_gps.rnx", cnv2_signals,
              sizeof(cnv2_signals) / sizeof(cnv2_signals[0]), NAV_CNV2, "DLR_2022_CNV2");
}

'''
text = text.replace(marker, probe + marker, 1)
path.write_text(text)
print("real DLR GPS CNAV/CNV2 pinned-RTKLIB probe added")
