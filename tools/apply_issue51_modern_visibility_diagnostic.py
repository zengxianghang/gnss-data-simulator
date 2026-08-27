from pathlib import Path

path = Path("tests/integration/test_all_signal_residuals.cpp")
text = path.read_text()
old = '''    dump_nav_integrity("original", original_nav);
    dump_nav_integrity("overlay", nav);
    free_nav(&original_nav);
'''
new = '''    dump_nav_integrity("original", original_nav);
    dump_nav_integrity("overlay", nav);
    free_nav(&original_nav);

    // Diagnostic only: distinguish absent modern NAV families from real
    // family records that are simply below the compact scenario horizon.
    const gtime_t diagnostic_time = gpst2time(start.gps_week,
                                               static_cast<double>(start.tow_ns) /
                                                   static_cast<double>(gnss_sim::NANOSECONDS_PER_SECOND));
    double receiver_llh[3] = {config.receiver.latitude_deg * D2R, config.receiver.longitude_deg * D2R,
                              config.receiver.height_m};
    double diagnostic_receiver[3]{};
    pos2ecef(receiver_llh, diagnostic_receiver);
    const eph_t* target_j04 = nullptr;
    const eph_t* target_g17 = nullptr;
    const eph_t* target_c22 = nullptr;
    const eph_t* target_c24 = nullptr;
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
        if (std::string(satellite_id) == "J04" && type == NAV_CNAV) target_j04 = &eph;
        if (std::string(satellite_id) == "G17" && type == NAV_CNAV) target_g17 = &eph;
        if (std::string(satellite_id) == "C22" && type == NAV_CNV1) target_c22 = &eph;
        if (std::string(satellite_id) == "C24" && type == NAV_CNV3) target_c24 = &eph;
    }

    ASSERT_NE(target_j04, nullptr);
    ASSERT_NE(target_g17, nullptr);
    ASSERT_NE(target_c22, nullptr);
    ASSERT_NE(target_c24, nullptr);
    double target_rs[4][3]{};
    const eph_t* targets[4] = {target_j04, target_g17, target_c22, target_c24};
    for (int target = 0; target < 4; ++target) {
        double dts = 0.0;
        double var = 0.0;
        eph2pos(diagnostic_time, targets[target], target_rs[target], &dts, &var);
    }
    double best_lat_deg = 0.0;
    double best_lon_deg = 0.0;
    double best_min_elevation_deg = -90.0;
    double best_elevation_deg[4]{};
    for (int latitude_deg = -60; latitude_deg <= 60; latitude_deg += 2) {
        for (int longitude_deg = -180; longitude_deg < 180; longitude_deg += 2) {
            const double candidate_llh[3] = {static_cast<double>(latitude_deg) * D2R,
                                             static_cast<double>(longitude_deg) * D2R, 100.0};
            double candidate_receiver[3]{};
            double candidate_elevation_deg[4]{};
            double minimum_elevation_deg = 90.0;
            pos2ecef(candidate_llh, candidate_receiver);
            for (int target = 0; target < 4; ++target) {
                double los[3]{};
                double azel[2]{};
                if (geodist(target_rs[target], candidate_receiver, los) <= 0.0) {
                    minimum_elevation_deg = -90.0;
                    break;
                }
                satazel(candidate_llh, los, azel);
                candidate_elevation_deg[target] = azel[1] * R2D;
                minimum_elevation_deg = (std::min)(minimum_elevation_deg, candidate_elevation_deg[target]);
            }
            if (minimum_elevation_deg > best_min_elevation_deg) {
                best_min_elevation_deg = minimum_elevation_deg;
                best_lat_deg = static_cast<double>(latitude_deg);
                best_lon_deg = static_cast<double>(longitude_deg);
                for (int target = 0; target < 4; ++target) best_elevation_deg[target] = candidate_elevation_deg[target];
            }
        }
    }
    std::fprintf(stderr,
                 "MODERN_GRID best_lat=%.1f best_lon=%.1f min_elev=%.6f J04=%.6f G17=%.6f C22=%.6f C24=%.6f\\n",
                 best_lat_deg, best_lon_deg, best_min_elevation_deg, best_elevation_deg[0], best_elevation_deg[1],
                 best_elevation_deg[2], best_elevation_deg[3]);
'''
count = text.count(old)
if count != 1:
    raise RuntimeError(f"modern visibility diagnostic anchor: expected 1, found {count}")
path.write_text(text.replace(old, new, 1))
print("modern NAV visibility and shared-position grid diagnostic injected")
