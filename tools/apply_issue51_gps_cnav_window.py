from pathlib import Path

path = Path("tests/integration/test_all_signal_residuals.cpp")
text = path.read_text()

old = '''    struct ResidualSite {
        const char* name;
        double latitude_deg;
        double longitude_deg;
    };
    const ResidualSite sites[] = {
        {"asia", 20.0, 120.0},
        {"gps_cnav", 36.272115, -19.973734},
        {"bds_bcnav12", -47.507042, -174.038033},
    };'''
new = '''    struct ResidualSite {
        const char* name;
        double latitude_deg;
        double longitude_deg;
        double start_sow_sec;
        int duration_sec;
    };
    const ResidualSite sites[] = {
        {"asia", 20.0, 120.0, 436500.0, 60},
        // G17's real CNAV record in the compact BRD400DLR fixture has Toc
        // 01:25:00. Start this coverage leg at that real modern-family window
        // rather than using the earlier 01:15 legacy-oriented epoch.
        {"gps_cnav", 36.272115, -19.973734, 437100.0, 120},
        {"bds_bcnav12", -47.507042, -174.038033, 436500.0, 60},
    };'''
if text.count(old) != 1:
    raise RuntimeError("ResidualSite anchor mismatch")
text = text.replace(old, new, 1)

old = '''        gnss_sim::SimConfig config = base_config;
        config.receiver = {site.latitude_deg, site.longitude_deg, 100.0};

        const std::filesystem::path site_directory = directory / site.name;'''
new = '''        gnss_sim::SimConfig config = base_config;
        config.receiver = {site.latitude_deg, site.longitude_deg, 100.0};
        config.duration_ns = static_cast<std::int64_t>(site.duration_sec) * gnss_sim::NANOSECONDS_PER_SECOND;
        gnss_sim::SimTime site_start{};
        ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(start.gps_week, site.start_sow_sec, &site_start));

        if (std::string(site.name) == "gps_cnav") {
            const gtime_t site_time = gpst2time(site_start.gps_week, gnss_sim::sim_time_sow_sec(site_start));
            double receiver_llh[3] = {site.latitude_deg * D2R, site.longitude_deg * D2R, 100.0};
            double receiver_ecef[3]{};
            pos2ecef(receiver_llh, receiver_ecef);
            double generic_rs[6]{};
            double generic_dts[2]{};
            double generic_var = 0.0;
            int generic_svh = 0;
            double generic_los[3]{};
            double generic_azel[2]{};
            const int generic_status = satpos(site_time, site_time, g17, EPHOPT_BRDC, &nav, generic_rs, generic_dts,
                                              &generic_var, &generic_svh);
            if (generic_status == 1 && geodist(generic_rs, receiver_ecef, generic_los) > 0.0) {
                satazel(receiver_llh, generic_los, generic_azel);
            }
            eph_t cnav_eph{};
            geph_t unused_geph{};
            rtklib_signal_bias_info_ext_t cnav_info{};
            int l2c_code = 0;
            int l2c_frequency_index = 0;
            const gnss_sim::SignalDefinition* l2c_definition =
                gnss_sim::find_signal_definition(gnss_sim::SignalId::kGpsL2C);
            ASSERT_NE(l2c_definition, nullptr);
            ASSERT_TRUE(gnss_sim::signal_rtklib_observation_code(*l2c_definition, &l2c_code, &l2c_frequency_index));
            static_cast<void>(l2c_frequency_index);
            const int cnav_status = rtklib_signal_ephemeris_ext(site_time, g17, static_cast<unsigned char>(l2c_code),
                                                                NAV_CNAV, &nav, &cnav_eph, &unused_geph, &cnav_info);
            double cnav_rs[3]{};
            double cnav_dts = 0.0;
            double cnav_var = 0.0;
            double cnav_los[3]{};
            double cnav_azel[2]{};
            if (cnav_status == 1) {
                eph2pos(site_time, &cnav_eph, cnav_rs, &cnav_dts, &cnav_var);
                if (geodist(cnav_rs, receiver_ecef, cnav_los) > 0.0) {
                    satazel(receiver_llh, cnav_los, cnav_azel);
                }
            }
            std::fprintf(stderr,
                         "GPS_CNAV_WINDOW sow=%.1f generic_status=%d generic_elev=%.6f cnav_status=%d cnav_elev=%.6f cnav_type=%d\\n",
                         site.start_sow_sec, generic_status, generic_azel[1] * R2D, cnav_status,
                         cnav_azel[1] * R2D, cnav_info.message_type);
        }

        const std::filesystem::path site_directory = directory / site.name;'''
if text.count(old) != 1:
    raise RuntimeError("site config anchor mismatch")
text = text.replace(old, new, 1)

old = '''        const gnss_sim::SimulatorRunOptions options{nav_text.c_str(), output_text.c_str(), start};'''
new = '''        const gnss_sim::SimulatorRunOptions options{nav_text.c_str(), output_text.c_str(), site_start};'''
if text.count(old) != 1:
    raise RuntimeError("site start options anchor mismatch")
text = text.replace(old, new, 1)

old = '''        } else if (signal_name == "GPS L1C" || signal_name == "GPS L2C" || signal_name == "GPS L5Q") {
            // Diagnostic phase for GPS modern-family coverage.
        } else {'''
new = '''        } else if (signal_name == "GPS L1C") {
            // The compact fixture has no real GPS CNV2 record; L1C remains
            // explicitly pending a real CNV2 companion from the daily product.
        } else {'''
if text.count(old) != 1:
    raise RuntimeError("GPS modern expectation anchor mismatch")
text = text.replace(old, new, 1)

path.write_text(text)
print("GPS CNAV coverage leg moved to the real G17 CNAV epoch; L2C/L5Q strict again")
