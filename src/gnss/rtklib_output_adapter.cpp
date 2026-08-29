#include "gnss/rtklib_adapter.h"

#include <cstring>

extern "C" {
#include <rtklib.h>
}
namespace gnss_sim {

bool rtklib_satellite_number_to_id(int satellite_number, char satellite_id[4]) {
    if (satellite_id == nullptr || satellite_number <= 0 || satellite_number > MAXSAT) {
        return false;
    }

    char rtklib_id[16]{};
    satno2id(satellite_number, rtklib_id);
    if (std::strlen(rtklib_id) != 3) {
        return false;
    }
    satellite_id[0] = rtklib_id[0];
    satellite_id[1] = rtklib_id[1];
    satellite_id[2] = rtklib_id[2];
    satellite_id[3] = '\0';
    return true;
}

} // namespace gnss_sim
