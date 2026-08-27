from pathlib import Path


def replace_once(path, old, new):
    p = Path(path)
    text = p.read_text()
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one guarded match, found {count}")
    p.write_text(text.replace(old, new, 1))


replace_once(
    "src/gnss/rtklib_adapter.h",
    "    kBeidouBcnav3,\n    kGlonassFdma,\n};",
    "    kBeidouBcnav3,\n    kGlonassFdma,\n    kGlonassL3Oc,\n};",
)
replace_once(
    "src/gnss/rtklib_adapter.h",
    "    double isc_sec[6];\n    double glonass_dtaun_sec;\n};",
    "    double isc_sec[6];\n    double glonass_dtaun_sec;\n    double glonass_isc_l3ocp_sec;\n};",
)

replace_once(
    "src/gnss/rtklib_bias_adapter.cpp",
    '''const geph_t* select_glonass_ephemeris(const nav_t& nav, gtime_t time, int satellite_number) {
    const geph_t* selected = nullptr;
    double selected_age_sec = MAXDTOE_GLO + 1.0;
    for (int index = 0; index < nav.ng; ++index) {
        const geph_t& geph = nav.geph[index];
        if (geph.sat != satellite_number) {
            continue;
        }
        const double age_sec = std::fabs(timediff(geph.toe, time));
        if (age_sec > MAXDTOE_GLO) {
            continue;
        }
        if (selected == nullptr || age_sec <= selected_age_sec) {
            selected = &geph;
            selected_age_sec = age_sec;
        }
    }
    return selected;
}
''',
    '''RtklibBroadcastMessageFamily glonass_message_family(const geph_t& geph) {
    const int message_type = geph.hdr.msg_type != 0 ? geph.hdr.msg_type : NAV_FDMA;
    if (message_type == NAV_L3OC) {
        return RtklibBroadcastMessageFamily::kGlonassL3Oc;
    }
    if (message_type == NAV_FDMA) {
        return RtklibBroadcastMessageFamily::kGlonassFdma;
    }
    return RtklibBroadcastMessageFamily::kUnknown;
}

const geph_t* select_glonass_ephemeris(const nav_t& nav, gtime_t time, int satellite_number,
                                       RtklibBroadcastMessageFamily requested_family) {
    if (requested_family == RtklibBroadcastMessageFamily::kUnknown) {
        requested_family = RtklibBroadcastMessageFamily::kGlonassFdma;
    }
    const geph_t* selected = nullptr;
    double selected_age_sec = MAXDTOE_GLO + 1.0;
    for (int index = 0; index < nav.ng; ++index) {
        const geph_t& geph = nav.geph[index];
        if (geph.sat != satellite_number || glonass_message_family(geph) != requested_family) {
            continue;
        }
        const double age_sec = std::fabs(timediff(geph.toe, time));
        if (age_sec > MAXDTOE_GLO) {
            continue;
        }
        if (selected == nullptr || age_sec < selected_age_sec ||
            (std::fabs(age_sec - selected_age_sec) < 1.0e-9 && timediff(geph.tof, selected->tof) > 0.0)) {
            selected = &geph;
            selected_age_sec = age_sec;
        }
    }
    return selected;
}
''',
)
replace_once(
    "src/gnss/rtklib_bias_adapter.cpp",
    '''    if (system == SYS_GLO) {
        const geph_t* geph = select_glonass_ephemeris(store->nav, time, satellite_number);
        if (geph == nullptr) {
            set_error(error_message, "no matching GLONASS ephemeris for broadcast bias");
            return false;
        }
        result.message_family = RtklibBroadcastMessageFamily::kGlonassFdma;
        result.iode = geph->iode;
        result.glonass_fcn = geph->frq;
        result.glonass_dtaun_sec = geph->dtaun;
        *data = result;
        return true;
    }
''',
    '''    if (system == SYS_GLO) {
        const geph_t* geph =
            select_glonass_ephemeris(store->nav, time, satellite_number, requested_message_family);
        if (geph == nullptr) {
            set_error(error_message, "no matching GLONASS ephemeris family for broadcast bias");
            return false;
        }
        result.message_family = glonass_message_family(*geph);
        result.iode = geph->iode;
        if (result.message_family == RtklibBroadcastMessageFamily::kGlonassFdma) {
            result.glonass_fcn = geph->frq;
            result.glonass_dtaun_sec = geph->dtaun;
        } else if (result.message_family == RtklibBroadcastMessageFamily::kGlonassL3Oc) {
            result.glonass_fcn = 0;
            result.glonass_isc_l3ocp_sec = geph->isc_l3ocp;
        }
        *data = result;
        return true;
    }
''',
)

replace_once(
    "src/model/measurement_model.cpp",
    '''        case NavMessageFamily::kGlonassFdma:
            return RtklibBroadcastMessageFamily::kGlonassFdma;
        case NavMessageFamily::kGalileoInav:''',
    '''        case NavMessageFamily::kGlonassFdma:
            return RtklibBroadcastMessageFamily::kGlonassFdma;
        case NavMessageFamily::kGlonassL3Oc:
            return RtklibBroadcastMessageFamily::kGlonassL3Oc;
        case NavMessageFamily::kGalileoInav:''',
)
replace_once(
    "src/model/measurement_model.cpp",
    '''        case NavMessageFamily::kBeidouBcnav3:
            return RtklibBroadcastMessageFamily::kBeidouBcnav3;
        case NavMessageFamily::kGlonassL3Oc:
        case NavMessageFamily::kGalileoCnav:''',
    '''        case NavMessageFamily::kBeidouBcnav3:
            return RtklibBroadcastMessageFamily::kBeidouBcnav3;
        case NavMessageFamily::kGalileoCnav:''',
)
replace_once(
    "src/model/measurement_model.cpp",
    '''        case CodeBiasModel::kGlonassG3:
            set_unavailable(code_bias_m, status);
            return true;''',
    '''        case CodeBiasModel::kGlonassG3:
            if (bias_data.message_family != RtklibBroadcastMessageFamily::kGlonassL3Oc) {
                set_unavailable(code_bias_m, status);
            } else {
                set_bias(-bias_data.glonass_isc_l3ocp_sec, code_bias_m, status);
            }
            return true;''',
)

replace_once(
    "src/output/truth_writer.cpp",
    '''        case RtklibBroadcastMessageFamily::kGlonassFdma:
            return "GLONASS_FDMA";
    }''',
    '''        case RtklibBroadcastMessageFamily::kGlonassFdma:
            return "GLONASS_FDMA";
        case RtklibBroadcastMessageFamily::kGlonassL3Oc:
            return "GLONASS_L3OC";
    }''',
)

replace_once(
    "src/gnss/nav_output_record.h",
    '''struct GlonassNavOutputData {
    int satellite_number;
    int prn;''',
    '''struct GlonassNavOutputData {
    RtklibBroadcastMessageFamily message_family;
    int message_type;
    int satellite_number;
    int prn;''',
)

replace_once(
    "src/gnss/rtklib_nav_output_adapter.cpp",
    '''RtklibBroadcastMessageFamily message_family(int system, int message_type) {
    if (system == SYS_GLO) {
        return RtklibBroadcastMessageFamily::kGlonassFdma;
    }''',
    '''RtklibBroadcastMessageFamily message_family(int system, int message_type) {
    if (system == SYS_GLO) {
        return message_type == NAV_L3OC ? RtklibBroadcastMessageFamily::kGlonassL3Oc
                                        : RtklibBroadcastMessageFamily::kGlonassFdma;
    }''',
)
replace_once(
    "src/gnss/rtklib_nav_output_adapter.cpp",
    '''    GlonassNavOutputData output{};
    output.satellite_number = geph.sat;
    output.prn = prn;''',
    '''    GlonassNavOutputData output{};
    output.message_type = geph.hdr.msg_type != 0 ? geph.hdr.msg_type : NAV_FDMA;
    output.message_family = message_family(SYS_GLO, output.message_type);
    output.satellite_number = geph.sat;
    output.prn = prn;''',
)

replace_once(
    "src/core/simulator.cpp",
    '''    if (record.kind == RtklibNavRecordKind::kGlonassEphemeris) {
        *family = NavMessageFamily::kGlonassFdma;
        return true;
    }''',
    '''    if (record.kind == RtklibNavRecordKind::kGlonassEphemeris) {
        if (record.glonass.message_family == RtklibBroadcastMessageFamily::kGlonassL3Oc) {
            *family = NavMessageFamily::kGlonassL3Oc;
            return true;
        }
        if (record.glonass.message_family == RtklibBroadcastMessageFamily::kGlonassFdma) {
            *family = NavMessageFamily::kGlonassFdma;
            return true;
        }
        return false;
    }''',
)
replace_once(
    "src/core/simulator.cpp",
    '''        case RtklibBroadcastMessageFamily::kGlonassFdma:
            *family = NavMessageFamily::kGlonassFdma;
            return true;
        case RtklibBroadcastMessageFamily::kUnknown:''',
    '''        case RtklibBroadcastMessageFamily::kGlonassFdma:
            *family = NavMessageFamily::kGlonassFdma;
            return true;
        case RtklibBroadcastMessageFamily::kGlonassL3Oc:
            *family = NavMessageFamily::kGlonassL3Oc;
            return true;
        case RtklibBroadcastMessageFamily::kUnknown:''',
)

replace_once(
    "src/output/novatel_nav_writer.cpp",
    '''    if (record.kind == RtklibNavRecordKind::kGlonassEphemeris) {
        log_name = "GLOEPHEMERISA";
        body = glonass_body(record.glonass);''',
    '''    if (record.kind == RtklibNavRecordKind::kGlonassEphemeris) {
        if (record.glonass.message_family != RtklibBroadcastMessageFamily::kGlonassFdma) {
            return true;
        }
        log_name = "GLOEPHEMERISA";
        body = glonass_body(record.glonass);''',
)
replace_once(
    "src/output/unicore_nav_writer.cpp",
    '''    if (record.kind == RtklibNavRecordKind::kGlonassEphemeris) {
        log_name = "GLOEPHA";
        body = glonass_body(record.glonass);''',
    '''    if (record.kind == RtklibNavRecordKind::kGlonassEphemeris) {
        if (record.glonass.message_family != RtklibBroadcastMessageFamily::kGlonassFdma) {
            return true;
        }
        log_name = "GLOEPHA";
        body = glonass_body(record.glonass);''',
)

replace_once(
    "src/gnss/rtklib_adapter.cpp",
    '#include <new>\n',
    '#include <map>\n#include <new>\n#include <utility>\n',
)
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
    '''    std::map<std::pair<int, int>, int> selected_eph;
    std::map<std::pair<int, int>, int> selected_geph;

    for (int index = 0; index < source->nav.n; ++index) {
        const eph_t& eph = source->nav.eph[index];
        if (eph.sat <= 0 || eph.sat > MAXSAT || !record_is_available(eph_transmission_time(eph), snapshot_time)) {
            continue;
        }
        int message_type = eph.hdr.msg_type;
        if (message_type == 0 && satsys(eph.sat, nullptr) == SYS_GAL) {
            if ((eph.code & (1 << 9)) != 0 || (eph.code & ((1 << 0) | (1 << 2))) != 0) {
                message_type = NAV_INAV;
            } else if ((eph.code & (1 << 8)) != 0 || (eph.code & (1 << 1)) != 0) {
                message_type = NAV_FNAV;
            }
        }
        const std::pair<int, int> key{eph.sat, message_type};
        const auto selected = selected_eph.find(key);
        if (selected == selected_eph.end() ||
            timediff(eph_transmission_time(eph), eph_transmission_time(source->nav.eph[selected->second])) > 0.0) {
            selected_eph[key] = index;
        }
    }
    for (int index = 0; index < source->nav.ng; ++index) {
        const geph_t& geph = source->nav.geph[index];
        if (geph.sat <= 0 || geph.sat > MAXSAT || !record_is_available(geph_transmission_time(geph), snapshot_time)) {
            continue;
        }
        const int message_type = geph.hdr.msg_type != 0 ? geph.hdr.msg_type : NAV_FDMA;
        const std::pair<int, int> key{geph.sat, message_type};
        const auto selected = selected_geph.find(key);
        if (selected == selected_geph.end() ||
            timediff(geph_transmission_time(geph), geph_transmission_time(source->nav.geph[selected->second])) > 0.0) {
            selected_geph[key] = index;
        }
    }

    for (const auto& selected : selected_eph) {
        if (!append_eph(source->nav.eph[selected.second], &destination->nav)) {
            reset_nav(&destination->nav);
            set_error(error_message, "cannot allocate receiver ephemeris snapshot");
            return false;
        }
    }
    for (const auto& selected : selected_geph) {
        if (!append_geph(source->nav.geph[selected.second], &destination->nav)) {
            reset_nav(&destination->nav);
            set_error(error_message, "cannot allocate receiver GLONASS ephemeris snapshot");
            return false;
        }
    }
''',
)

replace_once(
    "tests/unit/test_measurement_model.cpp",
    '''    value.glonass_dtaun_sec = 3.0e-9;
    return value;''',
    '''    value.glonass_dtaun_sec = 3.0e-9;
    value.glonass_isc_l3ocp_sec = 25.0e-9;
    return value;''',
)
replace_once(
    "tests/unit/test_measurement_model.cpp",
    '''    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGlonassG2), glonass, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * 3.0e-9, 1.0e-12);
}''',
    '''    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGlonassG2), glonass, &code_bias_m,
                                                        &status, &error_message));
    EXPECT_NEAR(code_bias_m, kSpeedOfLightMps * 3.0e-9, 1.0e-12);

    auto glonass_l3oc = bias(gnss_sim::RtklibBroadcastMessageFamily::kGlonassL3Oc);
    ASSERT_TRUE(gnss_sim::compute_broadcast_code_bias_m(signal(gnss_sim::SignalId::kGlonassG3), glonass_l3oc,
                                                        &code_bias_m, &status, &error_message));
    EXPECT_EQ(status, gnss_sim::BroadcastCodeBiasStatus::kApplied);
    EXPECT_NEAR(code_bias_m, -kSpeedOfLightMps * 25.0e-9, 1.0e-12);
}''',
)

replace_once(
    "tests/integration/test_all_signal_residuals.cpp",
    '#include <fstream>\n',
    '#include <fstream>\n#include <iomanip>\n',
)
replace_once(
    "tests/integration/test_all_signal_residuals.cpp",
    '''std::string brd4_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}
''',
    '''std::string brd4_nav_path() {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/brd400dlr_rinex4_acceptance_nav.rnx";
}

double rinex4_field(const std::string& line, std::size_t offset) {
    return std::stod(line.substr(offset, 19));
}

void write_rinex4_field(std::ostream* output, double value) {
    *output << std::scientific << std::setprecision(12) << std::setw(19) << value;
}

void write_rinex4_four(std::ostream* output, double a, double b, double c, double d) {
    *output << "    ";
    write_rinex4_field(output, a);
    write_rinex4_field(output, b);
    write_rinex4_field(output, c);
    write_rinex4_field(output, d);
    *output << '\\n';
}

bool write_g3_overlay_nav(const std::filesystem::path& directory, std::string* output_path) {
    if (output_path == nullptr) {
        return false;
    }
    std::ifstream source(brd4_nav_path(), std::ios::binary);
    if (!source) {
        return false;
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(source, line)) {
        if (!line.empty() && line.back() == '\\r') {
            line.pop_back();
        }
        lines.push_back(line);
    }

    const std::filesystem::path path = directory / "brd400dlr_plus_synthetic_glo_l3oc.rnx";
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) {
        return false;
    }
    for (const std::string& original : lines) {
        output << original << '\\n';
    }

    // Test-only overlay: create an L3OC companion for every real FDMA record.
    // Orbit/clock values are copied from the source record; only the nonzero
    // ISC_L3OCp is synthetic. The checked-in BRD400DLR fixture is not modified
    // and real-source validation must never use this overlay.
    constexpr double kSyntheticIscL3OcpSec = 25.0e-9;
    for (std::size_t index = 0; index + 5 < lines.size(); ++index) {
        std::istringstream header(lines[index]);
        std::string marker;
        std::string kind;
        std::string satellite;
        std::string family;
        header >> marker >> kind >> satellite >> family;
        if (marker != ">" || kind != "EPH" || satellite.size() != 3U || satellite[0] != 'R' || family != "FDMA") {
            continue;
        }
        const std::string& clock = lines[index + 1];
        const std::string& orbit1 = lines[index + 2];
        const std::string& orbit2 = lines[index + 3];
        const std::string& orbit3 = lines[index + 4];
        if (clock.rfind(satellite, 0) != 0 || clock.size() < 80U || orbit1.size() < 80U || orbit2.size() < 80U ||
            orbit3.size() < 80U) {
            return false;
        }

        std::istringstream epoch_stream(clock.substr(4, 19));
        double epoch[6]{};
        if (!(epoch_stream >> epoch[0] >> epoch[1] >> epoch[2] >> epoch[3] >> epoch[4] >> epoch[5])) {
            return false;
        }
        const double t_tm = time2gpst(epoch2time(epoch), nullptr);

        output << "> EPH " << satellite << " L3OC\\n";
        output << clock.substr(0, 23);
        write_rinex4_field(&output, rinex4_field(clock, 23));
        write_rinex4_field(&output, rinex4_field(clock, 42));
        write_rinex4_field(&output, 0.0);
        output << '\\n';
        write_rinex4_four(&output, rinex4_field(orbit1, 4), rinex4_field(orbit1, 23), rinex4_field(orbit1, 42),
                          rinex4_field(orbit1, 61));
        write_rinex4_four(&output, rinex4_field(orbit2, 4), rinex4_field(orbit2, 23), rinex4_field(orbit2, 42), 0.0);
        write_rinex4_four(&output, rinex4_field(orbit3, 4), rinex4_field(orbit3, 23), rinex4_field(orbit3, 42),
                          kSyntheticIscL3OcpSec);
        for (int extra = 0; extra < 4; ++extra) {
            write_rinex4_four(&output, 0.0, 0.0, 0.0, 0.0);
        }
        write_rinex4_four(&output, 0.0, 0.0, 0.0, t_tm);
    }
    output.flush();
    if (!output) {
        return false;
    }
    *output_path = path.string();
    return true;
}
''',
)
replace_once(
    "tests/integration/test_all_signal_residuals.cpp",
    '''    if (family == "GLONASS_FDMA")
        return NAV_FDMA;
    return 0;''',
    '''    if (family == "GLONASS_FDMA")
        return NAV_FDMA;
    if (family == "GLONASS_L3OC")
        return NAV_L3OC;
    return 0;''',
)
replace_once(
    "tests/integration/test_all_signal_residuals.cpp",
    '''    const std::filesystem::path output_path = directory / "simulated.log";
    const std::string output_text = output_path.string();
    const std::string nav_text = brd4_nav_path();
    const gnss_sim::SimulatorRunOptions options{nav_text.c_str(), output_text.c_str(), start};''',
    '''    const std::filesystem::path output_path = directory / "simulated.log";
    const std::string output_text = output_path.string();
    std::string nav_text;
    ASSERT_TRUE(write_g3_overlay_nav(directory, &nav_text));
    const gnss_sim::SimulatorRunOptions options{nav_text.c_str(), output_text.c_str(), start};''',
)
replace_once(
    "tests/integration/test_all_signal_residuals.cpp",
    '''                rtklib_resdop_signal_ext(&observation, &nav, &residual_options, receiver_position_m,
                                         receiver_velocity_mps, 0.0, wavelength_m, &doppler_residual_mps, nullptr);''',
    '''                rtklib_resdop_signal_ext(&observation, &nav, &residual_options, receiver_position_m,
                                         receiver_velocity_mps, 0.0, required_message_type, wavelength_m,
                                         &doppler_residual_mps, nullptr);''',
)
replace_once(
    "tests/integration/test_all_signal_residuals.cpp",
    '''    EXPECT_EQ(code_unavailable_signals, (std::set<std::string>{"GLONASS G3", "Galileo E6"}));''',
    '''    EXPECT_EQ(code_unavailable_signals, (std::set<std::string>{"Galileo E6"}));''',
)

print("issue #51 GLONASS L3OC patch applied")
