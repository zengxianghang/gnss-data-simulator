#include "gnss_sim/simulator.h"

#ifndef GNSS_SIM_RTKLIB_COMMIT
#define GNSS_SIM_RTKLIB_COMMIT "unknown"
#endif

namespace gnss_sim {

const char* simulator_version()
{
    return "0.1.0-dev";
}

const char* rtklib_commit_sha()
{
    return GNSS_SIM_RTKLIB_COMMIT;
}

}  // namespace gnss_sim
