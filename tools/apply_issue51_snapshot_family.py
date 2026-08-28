from pathlib import Path


def replace_once(path, old, new):
    p = Path(path)
    text = p.read_text()
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one guarded match, found {count}")
    p.write_text(text.replace(old, new, 1))


replace_once(
    "src/gnss/rtklib_adapter.cpp",
    '''    int selected_eph[MAXSAT];
    int selected_geph[MAXSAT];
    for (int sat_index = 0; sat_index < MAXSAT; ++sat_index) {
        selected_eph[sat_index] = -1;
        selected_geph[sat_index] = -1;
    }

    for (int index = 0; index < source->nav.n; ++index) {
        const eph_t& eph = source->nav.eph[index];
        if (eph.sat <= 0 || eph.sat > MAXSAT || !record_is_available(eph_transmission_time(eph), snapshot_time)) {
            continue;
        }
        const int sat_index = eph.sat - 1;
        const int selected = selected_eph[sat_index];
        if (selected < 0 ||
            timediff(eph_transmission_time(eph), eph_transmission_time(source->nav.eph[selected])) > 0.0) {
            selected_eph[sat_index] = index;
        }
    }
    for (int index = 0; index < source->nav.ng; ++index) {
        const geph_t& geph = source->nav.geph[index];
        if (geph.sat <= 0 || geph.sat > MAXSAT || !record_is_available(geph_transmission_time(geph), snapshot_time)) {
            continue;
        }
        const int sat_index = geph.sat - 1;
        const int selected = selected_geph[sat_index];
        if (selected < 0 ||
            timediff(geph_transmission_time(geph), geph_transmission_time(source->nav.geph[selected])) > 0.0) {
            selected_geph[sat_index] = index;
        }
    }

    for (int index = 0; index < source->nav.n; ++index) {
        const int sat = source->nav.eph[index].sat;
        if (sat > 0 && sat <= MAXSAT && selected_eph[sat - 1] == index &&
            !append_eph(source->nav.eph[index], &destination->nav)) {
            reset_nav(&destination->nav);
            set_error(error_message, "cannot allocate receiver ephemeris snapshot");
            return false;
        }
    }
    for (int index = 0; index < source->nav.ng; ++index) {
        const int sat = source->nav.geph[index].sat;
        if (sat > 0 && sat <= MAXSAT && selected_geph[sat - 1] == index &&
            !append_geph(source->nav.geph[index], &destination->nav)) {
            reset_nav(&destination->nav);
            set_error(error_message, "cannot allocate receiver GLONASS ephemeris snapshot");
            return false;
        }
    }
''',
    '''    // A receiver can cache several broadcast families for the same satellite
    // at the same time (for example GPS LNAV + CNAV + CNV2). Keep the latest
    // available record independently for each exact RINEX/RTKLIB message type.
    // Selecting only by satellite silently discarded modern-family ephemerides
    // during HOT/WARM initialization.
    std::vector<int> selected_eph;
    std::vector<int> selected_geph;
    selected_eph.reserve(static_cast<std::size_t>(source->nav.n));
    selected_geph.reserve(static_cast<std::size_t>(source->nav.ng));

    for (int index = 0; index < source->nav.n; ++index) {
        const eph_t& eph = source->nav.eph[index];
        if (eph.sat <= 0 || eph.sat > MAXSAT || !record_is_available(eph_transmission_time(eph), snapshot_time)) {
            continue;
        }
        auto selected = std::find_if(selected_eph.begin(), selected_eph.end(), [&](int candidate) {
            return source->nav.eph[candidate].sat == eph.sat &&
                   source->nav.eph[candidate].hdr.msg_type == eph.hdr.msg_type;
        });
        if (selected == selected_eph.end()) {
            selected_eph.push_back(index);
        } else if (timediff(eph_transmission_time(eph), eph_transmission_time(source->nav.eph[*selected])) > 0.0) {
            *selected = index;
        }
    }
    for (int index = 0; index < source->nav.ng; ++index) {
        const geph_t& geph = source->nav.geph[index];
        if (geph.sat <= 0 || geph.sat > MAXSAT || !record_is_available(geph_transmission_time(geph), snapshot_time)) {
            continue;
        }
        auto selected = std::find_if(selected_geph.begin(), selected_geph.end(), [&](int candidate) {
            return source->nav.geph[candidate].sat == geph.sat &&
                   source->nav.geph[candidate].hdr.msg_type == geph.hdr.msg_type;
        });
        if (selected == selected_geph.end()) {
            selected_geph.push_back(index);
        } else if (timediff(geph_transmission_time(geph), geph_transmission_time(source->nav.geph[*selected])) > 0.0) {
            *selected = index;
        }
    }

    for (int selected : selected_eph) {
        if (!append_eph(source->nav.eph[selected], &destination->nav)) {
            reset_nav(&destination->nav);
            set_error(error_message, "cannot allocate receiver ephemeris snapshot");
            return false;
        }
    }
    for (int selected : selected_geph) {
        if (!append_geph(source->nav.geph[selected], &destination->nav)) {
            reset_nav(&destination->nav);
            set_error(error_message, "cannot allocate receiver GLONASS ephemeris snapshot");
            return false;
        }
    }
''',
)

replace_once(
    "tests/unit/test_rtklib_adapter.cpp",
    '''std::string invalid_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/invalid_nav.rnx";
}
''',
    '''std::string invalid_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/invalid_nav.rnx";
}

std::string rinex4_acceptance_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}
''',
)

replace_once(
    "tests/unit/test_rtklib_adapter.cpp",
    '''TEST(RtklibAdapterCoordinates, LlhEcefRoundTripUsesRtklibReferenceFunctions) {''',
    '''TEST_F(RtklibAdapterTest, SnapshotRetainsSameSatelliteLegacyAndModernMessageFamilies) {
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(store_, rinex4_acceptance_nav_path().c_str(), &error_message))
        << error_message;

    gnss_sim::RtklibNavStore* snapshot = gnss_sim::create_rtklib_nav_store();
    ASSERT_NE(snapshot, nullptr);
    ASSERT_TRUE(gnss_sim::rtklib_copy_nav_snapshot(store_, 2347, 437100.0, snapshot, &error_message)) << error_message;

    int g17 = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G17", &g17));
    bool have_lnav = false;
    bool have_cnav = false;
    int g17_ephemeris_count = 0;
    for (int index = 0; index < gnss_sim::rtklib_nav_record_count(snapshot); ++index) {
        gnss_sim::RtklibNavRecordInfo info{};
        ASSERT_TRUE(gnss_sim::rtklib_nav_record_info(snapshot, index, &info));
        if (info.kind != gnss_sim::RtklibNavRecordKind::kEphemeris || info.satellite_number != g17) {
            continue;
        }
        ++g17_ephemeris_count;
        have_lnav = have_lnav || (info.message_type & NAV_LNAV) != 0;
        have_cnav = have_cnav || (info.message_type & NAV_CNAV) != 0;
    }
    EXPECT_TRUE(have_lnav);
    EXPECT_TRUE(have_cnav);
    EXPECT_GE(g17_ephemeris_count, 2);

    gnss_sim::destroy_rtklib_nav_store(snapshot);
}

TEST(RtklibAdapterCoordinates, LlhEcefRoundTripUsesRtklibReferenceFunctions) {''',
)

print("family-aware HOT/WARM navigation snapshot patch applied")
