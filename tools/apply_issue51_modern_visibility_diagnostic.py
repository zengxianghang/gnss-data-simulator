from pathlib import Path

path = Path("tests/integration/test_all_signal_residuals.cpp")
text = path.read_text()
old = '''    nav_t nav{};
    ASSERT_TRUE(load_nav(nav_text, &nav));

    std::ifstream input(directory / "observation_truth.csv");'''
new = '''    nav_t nav{};
    ASSERT_TRUE(load_nav(nav_text, &nav));

    // Diagnostic only: distinguish absent modern NAV families from real
    // family records that are simply below the compact scenario horizon.
    const gtime_t diagnostic_time = gpst2time(start.gps_week,
                                               static_cast<double>(start.tow_ns) /
                                                   static_cast<double>(gnss_sim::NANOSECONDS_PER_SECOND));
    double receiver_llh[3] = {config.receiver.latitude_deg * D2R, config.receiver.longitude_deg * D2R,
                              config.receiver.height_m};
    double diagnostic_receiver[3]{};
    pos2ecef(receiver_llh, diagnostic_receiver);
    for (int eph_index = 0; eph_index < nav.n; ++eph_index) {
        const eph_t& eph = nav.eph[eph_index];
        const int type = eph.hdr.msg_type;
        if (type != NAV_CNAV && type != NAV_CNV2 && type != NAV_CNV1 && type != NAV_CNV3) {
            continue;
        }
        double rs[3]{};
        double dts = 0.0;
        double var = 0.0;
        double los[3]{};
        double azel[2]{};
        double position_rad_m[3]{};
        char satellite_id[16]{};
        eph2pos(diagnostic_time, &eph, rs, &dts, &var);
        const double range = geodist(rs, diagnostic_receiver, los);
        ecef2pos(diagnostic_receiver, position_rad_m);
        if (range > 0.0) {
            satazel(position_rad_m, los, azel);
        }
        satno2id(eph.sat, satellite_id);
        std::fprintf(stderr, "MODERN_EPH sat=%s type=%d toe_age=%.3f elev_deg=%.6f\\n", satellite_id, type,
                     std::fabs(timediff(eph.toe, diagnostic_time)), azel[1] * R2D);
    }

    std::ifstream input(directory / "observation_truth.csv");'''
count = text.count(old)
if count != 1:
    raise RuntimeError(f"modern visibility diagnostic anchor: expected 1, found {count}")
path.write_text(text.replace(old, new, 1))
print("modern NAV visibility diagnostic injected")
