#include "gnss/galileo_has_adapter.h"

extern "C" {
#include <rtklib.h>
}

#include <cmath>
#include <cstdio>
#include <fstream>
#include <new>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace gnss_sim {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;

struct GalileoHasBiasRecord {
    int satellite_number;
    gtime_t start_time;
    gtime_t end_time;
    double code_osb_m;
};

void set_error(std::string* error_message, const std::string& message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool parse_bias_epoch(const std::string& text, gtime_t* time) {
    if (time == nullptr) {
        return false;
    }
    int year = 0;
    int day_of_year = 0;
    double second_of_day = 0.0;
    if (std::sscanf(text.c_str(), "%d:%d:%lf", &year, &day_of_year, &second_of_day) != 3 || year < 1980 ||
        day_of_year < 1 || day_of_year > 366 || !std::isfinite(second_of_day) || second_of_day < 0.0 ||
        second_of_day > 86400.0) {
        return false;
    }
    double epoch[6] = {static_cast<double>(year), 1.0, 1.0, 0.0, 0.0, 0.0};
    *time = timeadd(epoch2time(epoch), static_cast<double>(day_of_year - 1) * 86400.0 + second_of_day);
    return true;
}

bool parse_bias_file(const char* path, std::vector<GalileoHasBiasRecord>* records, std::string* error_message) {
    if (path == nullptr || records == nullptr) {
        set_error(error_message, "Galileo HAS bias path or output is invalid");
        return false;
    }
    std::ifstream input(path);
    if (!input) {
        set_error(error_message, std::string("cannot open Galileo HAS bias file: ") + path);
        return false;
    }

    bool gps_time_system = false;
    std::vector<GalileoHasBiasRecord> parsed;
    std::string line;
    while (std::getline(input, line)) {
        if (line.find("TIME_SYSTEM") != std::string::npos) {
            std::istringstream time_system_stream(line);
            std::string key;
            std::string value;
            time_system_stream >> key >> value;
            gps_time_system = (key == "TIME_SYSTEM" && value == "G");
            continue;
        }
        std::istringstream stream(line);
        std::string kind;
        std::string satellite_id;
        std::string observable;
        std::string start_text;
        std::string end_text;
        std::string unit;
        double value = 0.0;
        if (!(stream >> kind >> satellite_id >> observable >> start_text >> end_text >> unit >> value) || kind != "OSB" ||
            observable != "C6C") {
            continue;
        }
        if (satellite_id.size() != 3U || satellite_id[0] != 'E' || unit != "ns" || !std::isfinite(value)) {
            continue;
        }
        const int satellite_number = satid2no(satellite_id.c_str());
        gtime_t start_time{};
        gtime_t end_time{};
        if (satellite_number <= 0 || !parse_bias_epoch(start_text, &start_time) || !parse_bias_epoch(end_text, &end_time) ||
            timediff(end_time, start_time) <= 0.0) {
            continue;
        }
        parsed.push_back({satellite_number, start_time, end_time, kSpeedOfLightMps * value * 1.0e-9});
    }
    if (!gps_time_system) {
        set_error(error_message, "Galileo HAS Bias-SINEX TIME_SYSTEM must be GPS (G)");
        return false;
    }
    if (parsed.empty()) {
        set_error(error_message, "Galileo HAS bias file contains no Galileo C6C OSB records");
        return false;
    }
    *records = std::move(parsed);
    return true;
}

} // namespace

struct GalileoHasStore {
    nav_t precise_nav{};
    std::vector<GalileoHasBiasRecord> e6_bias_records;
    bool loaded = false;
};

GalileoHasStore* create_galileo_has_store() {
    return new (std::nothrow) GalileoHasStore();
}

void destroy_galileo_has_store(GalileoHasStore* store) {
    if (store != nullptr) {
        freenav(&store->precise_nav, 0xFF);
        delete store;
    }
}

bool load_galileo_has_products(GalileoHasStore* store, const char* sp3_path, const char* clock_path,
                               const char* bias_path, std::string* error_message) {
    if (store == nullptr || sp3_path == nullptr || clock_path == nullptr || bias_path == nullptr || sp3_path[0] == '\0' ||
        clock_path[0] == '\0' || bias_path[0] == '\0') {
        set_error(error_message, "Galileo HAS SP3/CLK/BIA product paths are required together");
        return false;
    }

    nav_t precise_nav{};
    readsp3(sp3_path, &precise_nav, 0);
    if (precise_nav.ne <= 0) {
        freenav(&precise_nav, 0xFF);
        set_error(error_message, std::string("Galileo HAS SP3 contains no precise ephemeris: ") + sp3_path);
        return false;
    }
    if (readrnxc(clock_path, &precise_nav) <= 0 || precise_nav.nc <= 0) {
        freenav(&precise_nav, 0xFF);
        set_error(error_message, std::string("Galileo HAS CLK contains no precise clocks: ") + clock_path);
        return false;
    }

    std::vector<GalileoHasBiasRecord> bias_records;
    if (!parse_bias_file(bias_path, &bias_records, error_message)) {
        freenav(&precise_nav, 0xFF);
        return false;
    }

    freenav(&store->precise_nav, 0xFF);
    store->precise_nav = precise_nav;
    store->e6_bias_records = std::move(bias_records);
    store->loaded = true;
    return true;
}

bool galileo_has_e6_correction(const GalileoHasStore* store, int gps_week, double sow_sec, int satellite_number,
                               GalileoHasE6Correction* correction, std::string* error_message) {
    if (store == nullptr || !store->loaded || correction == nullptr || gps_week < 0 || satellite_number <= 0 ||
        !std::isfinite(sow_sec) || sow_sec < 0.0 || sow_sec >= 604800.0) {
        set_error(error_message, "Galileo HAS E6 correction request has invalid arguments");
        return false;
    }

    const gtime_t time = gpst2time(gps_week, sow_sec);
    const GalileoHasBiasRecord* selected_bias = nullptr;
    for (const GalileoHasBiasRecord& record : store->e6_bias_records) {
        if (record.satellite_number == satellite_number && timediff(time, record.start_time) >= -1.0e-9 &&
            timediff(time, record.end_time) < -1.0e-9) {
            selected_bias = &record;
            break;
        }
    }
    if (selected_bias == nullptr) {
        set_error(error_message, "Galileo HAS E6 C6C OSB is unavailable at the requested epoch");
        return false;
    }

    double rs[6]{};
    double dts[2]{};
    double variance_m2 = 0.0;
    if (peph2pos(time, satellite_number, &store->precise_nav, 0, rs, dts, &variance_m2) == 0) {
        set_error(error_message, "Galileo HAS precise orbit/clock state is unavailable at the requested epoch");
        return false;
    }

    GalileoHasE6Correction result{};
    for (int index = 0; index < 3; ++index) {
        result.satellite_state.position_ecef_m[index] = rs[index];
        result.satellite_state.velocity_ecef_mps[index] = rs[index + 3];
    }
    result.satellite_state.clock_bias_sec = dts[0];
    result.satellite_state.clock_drift_sec_per_sec = dts[1];
    result.satellite_state.variance_m2 = variance_m2;
    result.satellite_state.health = 0;
    result.code_osb_m = selected_bias->code_osb_m;
    *correction = result;
    return true;
}

} // namespace gnss_sim
