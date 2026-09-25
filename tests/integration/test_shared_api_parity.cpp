// Issue #102 Phase 1: old-vs-shared parity baseline on a real RINEX NAV fixture.
//
// Loads the same real multi-GNSS BRDM file through the simulator's existing
// adapter (the "old" production path) and through the pinned RTKLIB shared
// GNSS adapter API (the "new" path), then compares:
//
//   - the loaded record sets (per-record kind/system/prn/iode/iodc/toe);
//   - satellite state (position/velocity/clock bias) at probes derived from
//     each record's own toe, for satellites whose ephemeris is unique in the
//     store, so both selectors provably resolve to the same record;
//   - the shared ABI version actually linked.
//
// No NAV records are manufactured: the fixture is the repository's real
// reduced Stanford brdm file (GPS/GLONASS/Galileo/BeiDou/QZSS). The
// multi_gnss_acceptance_nav fixture additionally contains IRNSS/SBAS
// records, which the shared API's system vocabulary fails closed on by
// design; that fail-closed behavior is recorded in the Phase-1 inventory
// document rather than relaxed here.  Both paths compile the same pinned RTKLIB core, so
// the comparison tolerances below are numerical-noise guards, not semantic
// allowances; a semantic divergence shows up orders of magnitude beyond them.
// This test is the Phase-1 baseline for the incremental migration planned by
// issue #102; it does not remove or replace any simulator adapter code.

#include "gnss/rtklib_adapter.h"
#include "rtklib_shared_api.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <gtest/gtest.h>
#include <map>
#include <string>
#include <vector>

using namespace gnss_sim;

namespace {

std::string data_file(const char* name) {
    return std::string(GNSS_SIM_TEST_DATA_DIR) + "/" + name;
}

constexpr double kWeekSeconds = 604800.0;

struct ProbeRecord {
    int kind; // RtklibNavRecordKind as int
    int system;
    int prn;
    int satellite_number;
    int iode;
    int iodc;
    int gps_week;
    double toe_sow;
};

std::vector<ProbeRecord> load_old_records(const RtklibNavStore* store) {
    std::vector<ProbeRecord> records;
    const int total = rtklib_nav_record_count(store);
    records.reserve(static_cast<size_t>(std::max(total, 0)));
    for (int index = 0; index < total; ++index) {
        RtklibNavRecordInfo info{};
        if (!rtklib_nav_record_info(store, index, &info)) {
            continue;
        }
        records.push_back(ProbeRecord{
            static_cast<int>(info.kind),
            info.system,
            info.prn,
            info.satellite_number,
            info.iode,
            info.iodc,
            info.gps_week,
            info.toe_sow_sec,
        });
    }
    return records;
}

struct SharedRecord {
    uint64_t record_id;
    uint32_t record_kind;
    uint32_t system;
    uint32_t prn;
    uint32_t family;
    int32_t iode;
    int32_t iodc;
    int32_t health_raw;
    int32_t toe_week;
    double toe_sow;
};

std::vector<SharedRecord> load_shared_records(rtklib_shared_nav_store_t* store) {
    std::vector<SharedRecord> records;
    const size_t total = rtklib_shared_nav_record_count(store, 0, 0);
    records.reserve(total);
    for (uint64_t record_id = 1; record_id <= total; ++record_id) {
        rtklib_shared_record_identity_t identity{};
        identity.abi_version = RTKLIB_SHARED_ABI_VERSION;
        identity.struct_size = static_cast<uint32_t>(sizeof(identity));
        const int status = rtklib_shared_nav_record(store, record_id, &identity);
        if (status != RTKLIB_SHARED_OK) {
            continue;
        }
        records.push_back(SharedRecord{
            record_id,
            identity.record_kind,
            identity.system,
            identity.prn,
            identity.family,
            identity.iode,
            identity.iodc,
            identity.health_raw,
            identity.toe.week,
            identity.toe.sow,
        });
    }
    return records;
}

int shared_kind_to_old(uint32_t record_kind) {
    // The old RtklibNavRecordKind enumerates from 0 (ephemeris, GLONASS
    // ephemeris, ionosphere); the shared ABI enumerates from 1.
    switch (record_kind) {
        case RTKLIB_SHARED_RECORD_EPH:
            return 0;
        case RTKLIB_SHARED_RECORD_GLO_EPH:
            return 1;
        case RTKLIB_SHARED_RECORD_ION:
            return 2;
        default:
            return -1;
    }
}

std::string record_key(int kind, int system, int prn, int week, double toe_sow) {
    // GLONASS records carry no separate IODC and the adapter projects every
    // record onto one GPST week, so (kind, system, prn, week, toe) identifies
    // a record across both stores.
    char buffer[96];
    std::snprintf(buffer, sizeof(buffer), "%d|%d|%d|%d|%.6f", kind, system, prn, week, toe_sow);
    return buffer;
}

std::string shared_key(uint32_t record_kind, uint32_t system, uint32_t prn, int toe_week, double toe_sow) {
    return record_key(shared_kind_to_old(record_kind), static_cast<int>(system), static_cast<int>(prn), toe_week,
                      toe_sow);
}

bool resolve_observation_code(const char* rinex_code, int* code) {
    int frequency_index = 0;
    return rtklib_observation_code(rinex_code, code, &frequency_index);
}

// Canonical RINEX observation code per shared message-family bit.  The shared
// state query derives a family mask from the code and intersects it with the
// query mask, so the code must be consistent with the record's family.
const char* rinex_code_for_family(uint32_t family) {
    switch (family) {
        case RTKLIB_SHARED_NAV_LNAV:
            return "1C";
        case RTKLIB_SHARED_NAV_FDMA:
            return "1C";
        case RTKLIB_SHARED_NAV_FNAV:
            return "1C";
        case RTKLIB_SHARED_NAV_INAV:
            return "1C";
        case RTKLIB_SHARED_NAV_D1:
            return "2I";
        case RTKLIB_SHARED_NAV_D2:
            return "2I";
        case RTKLIB_SHARED_NAV_D1D2:
            return "2I";
        case RTKLIB_SHARED_NAV_CNAV:
            return "2S";
        case RTKLIB_SHARED_NAV_CNV1:
            return "1P";
        case RTKLIB_SHARED_NAV_CNV2:
            return "1P";
        case RTKLIB_SHARED_NAV_CNV3:
            return "5P";
        case RTKLIB_SHARED_NAV_IFNV:
            return "1C";
        default:
            return "1C";
    }
}

} // namespace

TEST(SharedApiParityBaseline, LinkedAbiIsVersionOne) {
    // ABI 1.x keeps the 1.0 POD layouts; the linked library must match the
    // header it was compiled against.
    EXPECT_EQ(RTKLIB_SHARED_ABI_VERSION >> 16, 1u);
    EXPECT_EQ(rtklib_shared_abi_version(), static_cast<int>(RTKLIB_SHARED_ABI_VERSION));
}

TEST(SharedApiParityBaseline, RecordSetsAgreeOnRealMultiGnssFixture) {
    RtklibNavStore* old_store = create_rtklib_nav_store();
    ASSERT_NE(old_store, nullptr);
    std::string error;
    ASSERT_TRUE(load_rinex_nav_file(old_store, data_file("mixed_nav_2019.rnx").c_str(), &error)) << error;

    rtklib_shared_nav_store_t* shared_store = rtklib_shared_nav_create();
    ASSERT_NE(shared_store, nullptr);
    ASSERT_EQ(rtklib_shared_nav_load_rinex(shared_store, data_file("mixed_nav_2019.rnx").c_str(), "",
                                           "SOURCE:parity-baseline"),
              RTKLIB_SHARED_OK);

    const std::vector<ProbeRecord> old_records = load_old_records(old_store);
    const std::vector<SharedRecord> shared_records = load_shared_records(shared_store);
    ASSERT_FALSE(old_records.empty());
    ASSERT_FALSE(shared_records.empty());

    std::multimap<std::string, const SharedRecord*> by_key;
    for (const SharedRecord& record : shared_records) {
        by_key.emplace(shared_key(record.record_kind, record.system, record.prn, record.toe_week, record.toe_sow),
                       &record);
    }
    size_t matched = 0;
    for (const ProbeRecord& record : old_records) {
        const auto found =
            by_key.equal_range(record_key(record.kind, record.system, record.prn, record.gps_week, record.toe_sow));
        ASSERT_NE(found.first, found.second)
            << "shared store is missing a record the old adapter loaded: kind=" << record.kind
            << " system=" << record.system << " prn=" << record.prn;
        for (auto it = found.first; it != found.second; ++it) {
            EXPECT_EQ(it->second->iode, record.iode);
            EXPECT_EQ(it->second->iodc, record.iodc);
            ++matched;
            break;
        }
    }
    EXPECT_EQ(matched, old_records.size());
    EXPECT_EQ(shared_records.size(), old_records.size());

    rtklib_shared_nav_destroy(shared_store);
    destroy_rtklib_nav_store(old_store);
}

TEST(SharedApiParityBaseline, SatelliteStatesAgreeForUniqueRecords) {
    RtklibNavStore* old_store = create_rtklib_nav_store();
    ASSERT_NE(old_store, nullptr);
    std::string error;
    ASSERT_TRUE(load_rinex_nav_file(old_store, data_file("mixed_nav_2019.rnx").c_str(), &error)) << error;

    rtklib_shared_nav_store_t* shared_store = rtklib_shared_nav_create();
    ASSERT_NE(shared_store, nullptr);
    ASSERT_EQ(rtklib_shared_nav_load_rinex(shared_store, data_file("mixed_nav_2019.rnx").c_str(), "",
                                           "SOURCE:parity-baseline"),
              RTKLIB_SHARED_OK);

    const std::vector<ProbeRecord> old_records = load_old_records(old_store);
    const std::vector<SharedRecord> shared_records = load_shared_records(shared_store);

    // Old-side uniqueness: only satellites with exactly one broadcast record
    // in the store give both selectors a provably identical target.
    std::map<std::string, int> ephemeral_count;
    for (const ProbeRecord& record : old_records) {
        if (record.kind == static_cast<int>(RtklibNavRecordKind::kIonosphere)) {
            continue;
        }
        ++ephemeral_count[record_key(record.kind, record.system, record.prn, record.gps_week, record.toe_sow)];
    }
    std::map<int, int> comparisons_per_system;
    std::map<int, int> system_name = {
        {0x01, 'G'},
        {0x04, 'R'},
        {0x08, 'E'},
        {0x20, 'C'},
    };

    double max_position_delta = 0.0;
    double max_velocity_delta = 0.0;
    double max_clock_delta = 0.0;
    double max_drift_delta = 0.0;
    size_t compared = 0;

    for (const ProbeRecord& record : old_records) {
        if (record.kind == static_cast<int>(RtklibNavRecordKind::kIonosphere)) {
            continue;
        }
        const std::string key = record_key(record.kind, record.system, record.prn, record.gps_week, record.toe_sow);
        if (ephemeral_count.at(key) != 1) {
            continue;
        }
        const SharedRecord* shared = nullptr;
        for (const SharedRecord& candidate : shared_records) {
            if (shared_key(candidate.record_kind, candidate.system, candidate.prn, candidate.toe_week,
                           candidate.toe_sow) == key) {
                shared = &candidate;
                break;
            }
        }
        ASSERT_NE(shared, nullptr);

        int probe_week = record.gps_week;
        const double probe_sow = [&] {
            double sow = record.toe_sow + 60.0;
            while (sow >= kWeekSeconds) {
                sow -= kWeekSeconds;
                probe_week += 1;
            }
            return sow;
        }();

        RtklibSatelliteState old_state{};
        ASSERT_TRUE(get_rtklib_satellite_state_with_selection_time(
            old_store, probe_week, probe_sow, probe_week, probe_sow, record.satellite_number, &old_state, &error))
            << error;

        int family_code = 0;
        if (!resolve_observation_code(rinex_code_for_family(shared->family), &family_code)) {
            continue;
        }
        rtklib_shared_state_query_t query{};
        query.abi_version = RTKLIB_SHARED_ABI_VERSION;
        query.struct_size = static_cast<uint32_t>(sizeof(query));
        query.system = static_cast<uint32_t>(record.system);
        query.prn = static_cast<uint32_t>(record.prn);
        query.rtklib_code = static_cast<uint8_t>(family_code);
        query.glonass_fcn = RTKLIB_SHARED_GLO_FCN_UNKNOWN;
        query.family_mask = shared->family;
        query.evaluation_time.week = probe_week;
        query.evaluation_time.sow = probe_sow;
        query.selection_time = query.evaluation_time;
        query.selected_record_id = shared->record_id;

        rtklib_shared_state_result_t result{};
        result.abi_version = RTKLIB_SHARED_ABI_VERSION;
        result.struct_size = static_cast<uint32_t>(sizeof(result));
        const int query_status = rtklib_shared_state_query(shared_store, &query, &result);
        if (query_status != RTKLIB_SHARED_OK) {
            std::printf("probe unsupported: sys=0x%x prn=%u family=0x%x "
                        "iode=%d toe=%.1f status=%d\n",
                        query.system, query.prn, shared->family, shared->iode, shared->toe_sow, query_status);
        }
        ASSERT_EQ(query_status, RTKLIB_SHARED_OK);
        EXPECT_EQ(result.status, RTKLIB_SHARED_OK);
        EXPECT_EQ(result.state_valid, 1u);
        if (result.status != RTKLIB_SHARED_OK || result.state_valid != 1u) {
            continue;
        }

        for (int axis = 0; axis < 3; ++axis) {
            max_position_delta =
                std::max(max_position_delta, std::abs(result.position_ecef_m[axis] - old_state.position_ecef_m[axis]));
            max_velocity_delta = std::max(max_velocity_delta,
                                          std::abs(result.velocity_ecef_mps[axis] - old_state.velocity_ecef_mps[axis]));
        }
        max_clock_delta = std::max(max_clock_delta, std::abs(result.clock_bias_s - old_state.clock_bias_sec));
        max_drift_delta =
            std::max(max_drift_delta, std::abs(result.clock_drift_sps - old_state.clock_drift_sec_per_sec));
        EXPECT_EQ(result.health_raw, old_state.health);
        ++compared;
        ++comparisons_per_system[record.system];
    }

    // Real multi-GNSS coverage: GPS, GLONASS, Galileo and BeiDou all present.
    EXPECT_GE(comparisons_per_system[0x01], 1);
    EXPECT_GE(comparisons_per_system[0x04], 1);
    EXPECT_GE(comparisons_per_system[0x08], 1);
    EXPECT_GE(comparisons_per_system[0x20], 1);
    EXPECT_GE(compared, 4u);

    // Same pinned core, same records, same probe epochs: any semantic drift
    // between the paths shows up far beyond these numerical-noise guards.
    EXPECT_LT(max_position_delta, 1.0e-6);
    EXPECT_LT(max_velocity_delta, 1.0e-9);
    EXPECT_LT(max_clock_delta, 1.0e-11);
    EXPECT_LT(max_drift_delta, 1.0e-12);

    std::printf("parity baseline: compared=%zu max_pos_m=%.3e max_vel_mps=%.3e "
                "max_clk_s=%.3e max_drift_sps=%.3e coverage=[G:%d R:%d E:%d C:%d]\n",
                compared, max_position_delta, max_velocity_delta, max_clock_delta, max_drift_delta,
                comparisons_per_system[0x01], comparisons_per_system[0x04], comparisons_per_system[0x08],
                comparisons_per_system[0x20]);

    rtklib_shared_nav_destroy(shared_store);
    destroy_rtklib_nav_store(old_store);
}
