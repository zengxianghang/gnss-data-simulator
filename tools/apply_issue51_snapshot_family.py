from pathlib import Path


def replace_once(path, old, new):
    p = Path(path)
    text = p.read_text()
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one guarded match, found {count}")
    p.write_text(text.replace(old, new, 1))


adapter = Path("src/gnss/rtklib_adapter.cpp")
text = adapter.read_text()
include_anchor = "#include <new>\n"
if text.count(include_anchor) != 1:
    raise RuntimeError("rtklib_adapter.cpp include anchor mismatch")
text = text.replace(include_anchor, "#include <algorithm>\n#include <new>\n#include <vector>\n", 1)

start_marker = "bool rtklib_copy_nav_snapshot("
end_marker = "\nbool rtklib_copy_nav_record("
if text.count(start_marker) != 1 or text.count(end_marker) != 1:
    raise RuntimeError("rtklib_copy_nav_snapshot function anchors mismatch")
start = text.index(start_marker)
end = text.index(end_marker, start)
new_function = '''bool rtklib_copy_nav_snapshot(const RtklibNavStore* source, int gps_week, double sow_sec, RtklibNavStore* destination,
                              std::string* error_message) {
    if (source == nullptr || destination == nullptr || source == destination || !valid_gps_time(gps_week, sow_sec)) {
        set_error(error_message, "navigation snapshot request has invalid arguments");
        return false;
    }

    reset_nav(&destination->nav);
    copy_nav_metadata(source->nav, &destination->nav, true);
    const gtime_t snapshot_time = gpst2time(gps_week, sow_sec);

    // A receiver can cache several broadcast families for the same satellite
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
    for (int index = 0; index < source->nav.nion; ++index) {
        if (record_is_available(source->nav.ion[index].trans_time, snapshot_time) &&
            !append_ion(source->nav.ion[index], &destination->nav)) {
            reset_nav(&destination->nav);
            set_error(error_message, "cannot allocate receiver ionosphere snapshot");
            return false;
        }
    }
    return true;
}
'''
text = text[:start] + new_function + text[end:]
adapter.write_text(text)

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
