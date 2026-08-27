/*
 * Compile the pinned RTKLIB RINEX implementation against the fork's extended
 * observation-code helpers. This keeps the submodule commit pinned while
 * allowing RINEX OBS ingestion to understand modern BeiDou codes such as 5P
 * (B2a) and 7D (B2b).
 */
#include "rtklib_obs_ext.h"

#define obs2code obs2code_ext
#define code2obs code2obs_ext
#define getcodepri getcodepri_ext

#include "../../third_party/RTKLIB/src/rinex.c"
