#ifndef GNSS_SIM_SRC_GNSS_GALILEO_HAS_ADAPTER_H_
#define GNSS_SIM_SRC_GNSS_GALILEO_HAS_ADAPTER_H_

#include "gnss/rtklib_adapter.h"

#include <string>

namespace gnss_sim {

struct GalileoHasStore;

struct GalileoHasE6Correction {
    RtklibSatelliteState satellite_state;
    double code_osb_m;
};

GalileoHasStore* create_galileo_has_store();
void destroy_galileo_has_store(GalileoHasStore* store);

bool load_galileo_has_products(GalileoHasStore* store, const char* sp3_path, const char* clock_path,
                               const char* bias_path, std::string* error_message);

bool galileo_has_e6_correction(const GalileoHasStore* store, int gps_week, double sow_sec, int satellite_number,
                               GalileoHasE6Correction* correction, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_GNSS_GALILEO_HAS_ADAPTER_H_
