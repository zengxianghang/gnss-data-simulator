from pathlib import Path

path = Path("tests/integration/test_all_signal_residuals.cpp")
text = path.read_text()
old = '''    nav_t nav{};
    ASSERT_TRUE(load_nav(nav_text, &nav));
'''
new = '''    nav_t nav{};
    ASSERT_TRUE(load_nav(nav_text, &nav));

    nav_t original_nav{};
    ASSERT_TRUE(load_nav(brd4_nav_path(), &original_nav));
    const gtime_t debug_epoch = gpst2time(start.gps_week, static_cast<double>(start.tow_ns) / 1.0e9);
    const auto dump_nav_integrity = [&](const char* label, const nav_t& debug_nav) {
        int gps23_count = 0;
        int gps23_lnav = 0;
        int gps23_other = 0;
        double gps23_min_age_sec = 1.0e99;
        int glo_fdma = 0;
        int glo_l3oc = 0;
        int glo_other = 0;
        for (int index = 0; index < debug_nav.n; ++index) {
            const eph_t& eph = debug_nav.eph[index];
            if (eph.sat == 23) {
                ++gps23_count;
                const int message_type = eph.hdr.msg_type != 0 ? eph.hdr.msg_type : NAV_LNAV;
                if (message_type == NAV_LNAV) {
                    ++gps23_lnav;
                } else {
                    ++gps23_other;
                }
                gps23_min_age_sec = (std::min)(gps23_min_age_sec, std::fabs(timediff(eph.toe, debug_epoch)));
            }
        }
        for (int index = 0; index < debug_nav.ng; ++index) {
            const geph_t& geph = debug_nav.geph[index];
            const int message_type = geph.hdr.msg_type != 0 ? geph.hdr.msg_type : NAV_FDMA;
            if (message_type == NAV_FDMA) {
                ++glo_fdma;
            } else if (message_type == NAV_L3OC) {
                ++glo_l3oc;
            } else {
                ++glo_other;
            }
        }
        std::fprintf(stderr,
                     "NAV_INTEGRITY label=%s n=%d ng=%d gps23=%d gps23_lnav=%d gps23_other=%d "
                     "gps23_min_age=%.3f glo_fdma=%d glo_l3oc=%d glo_other=%d\\n",
                     label, debug_nav.n, debug_nav.ng, gps23_count, gps23_lnav, gps23_other, gps23_min_age_sec,
                     glo_fdma, glo_l3oc, glo_other);
    };
    dump_nav_integrity("original", original_nav);
    dump_nav_integrity("overlay", nav);
    free_nav(&original_nav);
'''
count = text.count(old)
if count != 1:
    raise RuntimeError(f"test_all_signal_residuals.cpp: expected nav-load anchor once, found {count}")
path.write_text(text.replace(old, new, 1))
print("overlay navigation integrity diagnostic injected")
