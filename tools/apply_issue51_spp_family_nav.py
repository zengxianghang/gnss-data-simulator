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
    '''struct RtklibSolutionObservation {
    int satellite_number;
    int observation_code;''',
    '''struct RtklibSolutionObservation {
    int satellite_number;
    int observation_code;
    RtklibBroadcastMessageFamily message_family;''',
)

replace_once(
    "src/solution/solution_engine.cpp",
    '''    result.satellite_number = source.satellite_number;
    result.observation_code = observation_code;
    result.pseudorange_m = source.pseudorange_m;''',
    '''    result.satellite_number = source.satellite_number;
    result.observation_code = observation_code;
    result.message_family = source.broadcast_message_family;
    result.pseudorange_m = source.pseudorange_m;''',
)

replace_once(
    "src/gnss/rtklib_solution_adapter.cpp",
    '''bool legacy_prange_adjustment_m(const nav_t& nav, int satellite_number, int observation_code, double* adjustment_m) {''',
    '''int required_message_mask(int system, RtklibBroadcastMessageFamily family) {
    switch (family) {
        case RtklibBroadcastMessageFamily::kLegacy:
            if (system == SYS_GPS || system == SYS_QZS) return NAV_LNAV;
            if (system == SYS_CMP) return NAV_D1 | NAV_D2 | NAV_D1D2;
            break;
        case RtklibBroadcastMessageFamily::kCnav:
            return NAV_CNAV;
        case RtklibBroadcastMessageFamily::kCnav2:
            return NAV_CNV2;
        case RtklibBroadcastMessageFamily::kGalileoInav:
            return NAV_INAV;
        case RtklibBroadcastMessageFamily::kGalileoFnav:
            return NAV_FNAV;
        case RtklibBroadcastMessageFamily::kBeidouBcnav1:
            return NAV_CNV1;
        case RtklibBroadcastMessageFamily::kBeidouBcnav2:
            return NAV_CNV2;
        case RtklibBroadcastMessageFamily::kBeidouBcnav3:
            return NAV_CNV3;
        case RtklibBroadcastMessageFamily::kGlonassFdma:
            return NAV_FDMA;
        case RtklibBroadcastMessageFamily::kGlonassL3Oc:
            return NAV_L3OC;
        case RtklibBroadcastMessageFamily::kUnknown:
            break;
    }
    return 0;
}

bool append_solution_ephemeris(const nav_t& source_nav, gtime_t time, const RtklibSolutionObservation& observation,
                               nav_t* solver_nav, eph_t solver_eph[MAXOBS], geph_t solver_geph[MAXOBS]) {
    if (solver_nav == nullptr || observation.satellite_number <= 0 || observation.observation_code <= 0 ||
        observation.observation_code > 255) {
        return false;
    }
    const int system = satsys(observation.satellite_number, nullptr);
    const int mask = required_message_mask(system, observation.message_family);
    if (mask == 0) return false;

    eph_t eph{};
    geph_t geph{};
    if (rtklib_signal_ephemeris_ext(time, observation.satellite_number,
                                    static_cast<unsigned char>(observation.observation_code), mask, &source_nav, &eph,
                                    &geph, nullptr) != 1) {
        return false;
    }
    if (system == SYS_GLO) {
        if (solver_nav->ng >= MAXOBS) return false;
        solver_geph[solver_nav->ng++] = geph;
    } else {
        if (solver_nav->n >= MAXOBS) return false;
        solver_eph[solver_nav->n++] = eph;
    }
    return true;
}

bool legacy_prange_adjustment_m(const nav_t& nav, int satellite_number, int observation_code, double* adjustment_m) {''',
)

replace_once(
    "src/gnss/rtklib_solution_adapter.cpp",
    '''        const int status = rtklib_signal_code_bias_ext(time, observation.satellite_number,
                                                       static_cast<unsigned char>(observation.observation_code), 0,
                                                       &nav, &rtklib_code_bias_m, nullptr);''',
    '''        const int system = satsys(observation.satellite_number, nullptr);
        const int mask = required_message_mask(system, observation.message_family);
        if (mask == 0) return false;
        const int status = rtklib_signal_code_bias_ext(time, observation.satellite_number,
                                                       static_cast<unsigned char>(observation.observation_code), mask,
                                                       &nav, &rtklib_code_bias_m, nullptr);''',
)

replace_once(
    "src/gnss/rtklib_solution_adapter.cpp",
    '''    RtklibPositionSolution result{};
    obsd_t rtklib_observations[MAXOBS]{};
    const gtime_t epoch_time = gpst2time(gps_week, sow_sec);
    bool used_satellite[MAXSAT]{};
    int usable_count = 0;

    for (int index = 0; index < observation_count && usable_count < MAXOBS; ++index) {''',
    '''    RtklibPositionSolution result{};
    obsd_t rtklib_observations[MAXOBS]{};
    eph_t solver_eph[MAXOBS]{};
    geph_t solver_geph[MAXOBS]{};
    nav_t solver_nav = receiver_nav->nav;
    solver_nav.eph = solver_eph;
    solver_nav.n = 0;
    solver_nav.nmax = MAXOBS;
    solver_nav.geph = solver_geph;
    solver_nav.ng = 0;
    solver_nav.ngmax = MAXOBS;
    const gtime_t epoch_time = gpst2time(gps_week, sow_sec);
    bool used_satellite[MAXSAT]{};
    int usable_count = 0;

    for (int index = 0; index < observation_count && usable_count < MAXOBS; ++index) {''',
)

replace_once(
    "src/gnss/rtklib_solution_adapter.cpp",
    '''        double pseudorange_m = 0.0;
        if (!solver_pseudorange_m(receiver_nav->nav, epoch_time, source, &pseudorange_m)) {
            continue;
        }''',
    '''        if (!append_solution_ephemeris(receiver_nav->nav, epoch_time, source, &solver_nav, solver_eph,
                                             solver_geph)) {
            continue;
        }
        double pseudorange_m = 0.0;
        if (!solver_pseudorange_m(solver_nav, epoch_time, source, &pseudorange_m)) {
            continue;
        }''',
)

replace_once(
    "src/gnss/rtklib_solution_adapter.cpp",
    '''    const int status = pntpos(rtklib_observations, usable_count, &receiver_nav->nav, &options, &rtklib_solution,
                              nullptr, nullptr, message);''',
    '''    const int status = pntpos(rtklib_observations, usable_count, &solver_nav, &options, &rtklib_solution,
                              nullptr, nullptr, message);''',
)

print("SPP now filters each primary satellite to its observation NAV family")
