#ifndef GNSS_SIM_SRC_GNSS_RTKLIB_FAMILY_TAXONOMY_H_
#define GNSS_SIM_SRC_GNSS_RTKLIB_FAMILY_TAXONOMY_H_

#include "gnss/rtklib_adapter.h"
#include "gnss/signal_definitions.h"

extern "C" {
#include <rtklib.h>
}

namespace gnss_sim {

// Message-family masks shared by the simulator's RTKLIB adapter paths. Keep
// the explicit-family mapping independent from legacy handling: some callers
// intentionally pass mask 0 for legacy/unknown so the signal semantics choose
// the family, while state/SPP/raw-position require a system-specific legacy
// mask.
constexpr int rtklib_explicit_nav_message_mask(RtklibBroadcastMessageFamily family) {
    switch (family) {
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
        case RtklibBroadcastMessageFamily::kLegacy:
        case RtklibBroadcastMessageFamily::kUnknown:
            return 0;
    }
    return 0;
}

constexpr int rtklib_required_nav_message_mask(int system, RtklibBroadcastMessageFamily family) {
    if (family == RtklibBroadcastMessageFamily::kLegacy) {
        if (system == SYS_GPS || system == SYS_QZS) {
            return NAV_LNAV;
        }
        if (system == SYS_CMP) {
            return NAV_D1 | NAV_D2 | NAV_D1D2;
        }
        return 0;
    }
    return rtklib_explicit_nav_message_mask(family);
}

// Decode only the RTKLIB message type itself. This keeps the NAV-output
// adapter's established decoding contract (including CNV2's system-dependent
// meaning) separate from caller-specific legacy/fallback policy. GLONASS is
// special because its message namespace must not be interpreted as another
// constellation's modern family.
constexpr RtklibBroadcastMessageFamily rtklib_message_type_family(int system, int message_type) {
    if (system == SYS_GLO) {
        if (message_type == NAV_FDMA) {
            return RtklibBroadcastMessageFamily::kGlonassFdma;
        }
        if (message_type == NAV_L3OC) {
            return RtklibBroadcastMessageFamily::kGlonassL3Oc;
        }
        return RtklibBroadcastMessageFamily::kUnknown;
    }
    switch (message_type) {
        case NAV_CNAV:
            return RtklibBroadcastMessageFamily::kCnav;
        case NAV_CNV1:
            return RtklibBroadcastMessageFamily::kBeidouBcnav1;
        case NAV_CNV2:
            return system == SYS_CMP ? RtklibBroadcastMessageFamily::kBeidouBcnav2
                                     : RtklibBroadcastMessageFamily::kCnav2;
        case NAV_CNV3:
            return RtklibBroadcastMessageFamily::kBeidouBcnav3;
        case NAV_INAV:
            return RtklibBroadcastMessageFamily::kGalileoInav;
        case NAV_FNAV:
            return RtklibBroadcastMessageFamily::kGalileoFnav;
        default:
            return RtklibBroadcastMessageFamily::kUnknown;
    }
}

// Bias/selection code needs an explicit family only when that family is valid
// for the satellite system. Keep this validation separate from the raw
// message-type decoder so NAV-output compatibility is not silently tightened.
constexpr bool rtklib_message_family_matches_system(int system, RtklibBroadcastMessageFamily family) {
    if (system == SYS_GPS || system == SYS_QZS) {
        return family == RtklibBroadcastMessageFamily::kCnav || family == RtklibBroadcastMessageFamily::kCnav2;
    }
    if (system == SYS_GAL) {
        return family == RtklibBroadcastMessageFamily::kGalileoInav ||
               family == RtklibBroadcastMessageFamily::kGalileoFnav;
    }
    if (system == SYS_CMP) {
        return family == RtklibBroadcastMessageFamily::kBeidouBcnav1 ||
               family == RtklibBroadcastMessageFamily::kBeidouBcnav2 ||
               family == RtklibBroadcastMessageFamily::kBeidouBcnav3;
    }
    if (system == SYS_GLO) {
        return family == RtklibBroadcastMessageFamily::kGlonassFdma ||
               family == RtklibBroadcastMessageFamily::kGlonassL3Oc;
    }
    return false;
}

constexpr RtklibBroadcastMessageFamily rtklib_explicit_message_family_for_system(int system, int message_type) {
    const RtklibBroadcastMessageFamily family = rtklib_message_type_family(system, message_type);
    return rtklib_message_family_matches_system(system, family) ? family : RtklibBroadcastMessageFamily::kUnknown;
}

// Project the simulator-owned navigation family into the generic RTKLIB
// broadcast-family taxonomy. Keep unsupported Galileo CNAV explicit: the
// current RTKLIB broadcast-family contract has no corresponding family.
constexpr RtklibBroadcastMessageFamily rtklib_broadcast_message_family(NavMessageFamily family) {
    switch (family) {
        case NavMessageFamily::kGpsLnav:
        case NavMessageFamily::kQzssLnav:
        case NavMessageFamily::kBeidouD1D2:
            return RtklibBroadcastMessageFamily::kLegacy;
        case NavMessageFamily::kGpsCnav:
        case NavMessageFamily::kQzssCnav:
            return RtklibBroadcastMessageFamily::kCnav;
        case NavMessageFamily::kGpsCnav2:
        case NavMessageFamily::kQzssCnav2:
            return RtklibBroadcastMessageFamily::kCnav2;
        case NavMessageFamily::kGlonassFdma:
            return RtklibBroadcastMessageFamily::kGlonassFdma;
        case NavMessageFamily::kGlonassL3Oc:
            return RtklibBroadcastMessageFamily::kGlonassL3Oc;
        case NavMessageFamily::kGalileoInav:
            return RtklibBroadcastMessageFamily::kGalileoInav;
        case NavMessageFamily::kGalileoFnav:
            return RtklibBroadcastMessageFamily::kGalileoFnav;
        case NavMessageFamily::kBeidouBcnav1:
            return RtklibBroadcastMessageFamily::kBeidouBcnav1;
        case NavMessageFamily::kBeidouBcnav2:
            return RtklibBroadcastMessageFamily::kBeidouBcnav2;
        case NavMessageFamily::kBeidouBcnav3:
            return RtklibBroadcastMessageFamily::kBeidouBcnav3;
        case NavMessageFamily::kGalileoCnav:
            return RtklibBroadcastMessageFamily::kUnknown;
    }
    return RtklibBroadcastMessageFamily::kUnknown;
}

struct RtklibNavMessageFamilyProjection {
    bool supported;
    NavMessageFamily family;
};

// Preserve the reverse projection used by the simulator's NAV-delivery path.
// Legacy/CNAV/CNAV2 are system-sensitive. The already-classified modern
// Galileo/BeiDou/GLONASS families are intentionally projected without another
// system check because that is the established simulator behavior. Tightening
// those combinations is a separate semantic change, not part of this refactor.
constexpr RtklibNavMessageFamilyProjection rtklib_nav_message_family_projection(int system,
                                                                                RtklibBroadcastMessageFamily family) {
    switch (family) {
        case RtklibBroadcastMessageFamily::kLegacy:
            if (system == SYS_GPS) {
                return {true, NavMessageFamily::kGpsLnav};
            }
            if (system == SYS_QZS) {
                return {true, NavMessageFamily::kQzssLnav};
            }
            if (system == SYS_CMP) {
                return {true, NavMessageFamily::kBeidouD1D2};
            }
            break;
        case RtklibBroadcastMessageFamily::kCnav:
            if (system == SYS_GPS) {
                return {true, NavMessageFamily::kGpsCnav};
            }
            if (system == SYS_QZS) {
                return {true, NavMessageFamily::kQzssCnav};
            }
            break;
        case RtklibBroadcastMessageFamily::kCnav2:
            if (system == SYS_GPS) {
                return {true, NavMessageFamily::kGpsCnav2};
            }
            if (system == SYS_QZS) {
                return {true, NavMessageFamily::kQzssCnav2};
            }
            break;
        case RtklibBroadcastMessageFamily::kGalileoInav:
            return {true, NavMessageFamily::kGalileoInav};
        case RtklibBroadcastMessageFamily::kGalileoFnav:
            return {true, NavMessageFamily::kGalileoFnav};
        case RtklibBroadcastMessageFamily::kBeidouBcnav1:
            return {true, NavMessageFamily::kBeidouBcnav1};
        case RtklibBroadcastMessageFamily::kBeidouBcnav2:
            return {true, NavMessageFamily::kBeidouBcnav2};
        case RtklibBroadcastMessageFamily::kBeidouBcnav3:
            return {true, NavMessageFamily::kBeidouBcnav3};
        case RtklibBroadcastMessageFamily::kGlonassFdma:
            return {true, NavMessageFamily::kGlonassFdma};
        case RtklibBroadcastMessageFamily::kGlonassL3Oc:
            return {true, NavMessageFamily::kGlonassL3Oc};
        case RtklibBroadcastMessageFamily::kUnknown:
            break;
    }
    return {false, NavMessageFamily::kGpsLnav};
}

// Exhaust the current public simulator family enum at compile time. These
// assertions are intentionally next to the mapping so adding/reassigning a
// family cannot silently change the SPP/raw-position selection contract.
static_assert(rtklib_explicit_nav_message_mask(RtklibBroadcastMessageFamily::kUnknown) == 0);
static_assert(rtklib_explicit_nav_message_mask(RtklibBroadcastMessageFamily::kLegacy) == 0);
static_assert(rtklib_explicit_nav_message_mask(RtklibBroadcastMessageFamily::kCnav) == NAV_CNAV);
static_assert(rtklib_explicit_nav_message_mask(RtklibBroadcastMessageFamily::kCnav2) == NAV_CNV2);
static_assert(rtklib_explicit_nav_message_mask(RtklibBroadcastMessageFamily::kGalileoInav) == NAV_INAV);
static_assert(rtklib_explicit_nav_message_mask(RtklibBroadcastMessageFamily::kGalileoFnav) == NAV_FNAV);
static_assert(rtklib_explicit_nav_message_mask(RtklibBroadcastMessageFamily::kBeidouBcnav1) == NAV_CNV1);
static_assert(rtklib_explicit_nav_message_mask(RtklibBroadcastMessageFamily::kBeidouBcnav2) == NAV_CNV2);
static_assert(rtklib_explicit_nav_message_mask(RtklibBroadcastMessageFamily::kBeidouBcnav3) == NAV_CNV3);
static_assert(rtklib_explicit_nav_message_mask(RtklibBroadcastMessageFamily::kGlonassFdma) == NAV_FDMA);
static_assert(rtklib_explicit_nav_message_mask(RtklibBroadcastMessageFamily::kGlonassL3Oc) == NAV_L3OC);

static_assert(rtklib_required_nav_message_mask(SYS_GPS, RtklibBroadcastMessageFamily::kLegacy) == NAV_LNAV);
static_assert(rtklib_required_nav_message_mask(SYS_QZS, RtklibBroadcastMessageFamily::kLegacy) == NAV_LNAV);
static_assert(rtklib_required_nav_message_mask(SYS_CMP, RtklibBroadcastMessageFamily::kLegacy) ==
              (NAV_D1 | NAV_D2 | NAV_D1D2));
static_assert(rtklib_required_nav_message_mask(SYS_GAL, RtklibBroadcastMessageFamily::kLegacy) == 0);
static_assert(rtklib_required_nav_message_mask(SYS_GLO, RtklibBroadcastMessageFamily::kLegacy) == 0);
static_assert(rtklib_required_nav_message_mask(SYS_GPS, RtklibBroadcastMessageFamily::kUnknown) == 0);

static_assert(rtklib_explicit_message_family_for_system(SYS_GPS, NAV_CNAV) == RtklibBroadcastMessageFamily::kCnav);
static_assert(rtklib_explicit_message_family_for_system(SYS_QZS, NAV_CNV2) == RtklibBroadcastMessageFamily::kCnav2);
static_assert(rtklib_explicit_message_family_for_system(SYS_GAL, NAV_INAV) ==
              RtklibBroadcastMessageFamily::kGalileoInav);
static_assert(rtklib_explicit_message_family_for_system(SYS_GAL, NAV_FNAV) ==
              RtklibBroadcastMessageFamily::kGalileoFnav);
static_assert(rtklib_explicit_message_family_for_system(SYS_CMP, NAV_CNV1) ==
              RtklibBroadcastMessageFamily::kBeidouBcnav1);
static_assert(rtklib_explicit_message_family_for_system(SYS_CMP, NAV_CNV2) ==
              RtklibBroadcastMessageFamily::kBeidouBcnav2);
static_assert(rtklib_explicit_message_family_for_system(SYS_CMP, NAV_CNV3) ==
              RtklibBroadcastMessageFamily::kBeidouBcnav3);
static_assert(rtklib_explicit_message_family_for_system(SYS_GLO, NAV_FDMA) ==
              RtklibBroadcastMessageFamily::kGlonassFdma);
static_assert(rtklib_explicit_message_family_for_system(SYS_GLO, NAV_L3OC) ==
              RtklibBroadcastMessageFamily::kGlonassL3Oc);
static_assert(rtklib_explicit_message_family_for_system(SYS_GPS, NAV_LNAV) == RtklibBroadcastMessageFamily::kUnknown);
static_assert(rtklib_explicit_message_family_for_system(SYS_CMP, NAV_D1) == RtklibBroadcastMessageFamily::kUnknown);
static_assert(rtklib_explicit_message_family_for_system(SYS_GAL, NAV_CNAV) == RtklibBroadcastMessageFamily::kUnknown);
static_assert(rtklib_explicit_message_family_for_system(SYS_GLO, 0) == RtklibBroadcastMessageFamily::kUnknown);

static_assert(rtklib_broadcast_message_family(NavMessageFamily::kGpsLnav) == RtklibBroadcastMessageFamily::kLegacy);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kGpsCnav) == RtklibBroadcastMessageFamily::kCnav);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kGpsCnav2) == RtklibBroadcastMessageFamily::kCnav2);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kQzssLnav) == RtklibBroadcastMessageFamily::kLegacy);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kQzssCnav) == RtklibBroadcastMessageFamily::kCnav);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kQzssCnav2) == RtklibBroadcastMessageFamily::kCnav2);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kGlonassFdma) ==
              RtklibBroadcastMessageFamily::kGlonassFdma);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kGlonassL3Oc) ==
              RtklibBroadcastMessageFamily::kGlonassL3Oc);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kGalileoInav) ==
              RtklibBroadcastMessageFamily::kGalileoInav);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kGalileoFnav) ==
              RtklibBroadcastMessageFamily::kGalileoFnav);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kGalileoCnav) ==
              RtklibBroadcastMessageFamily::kUnknown);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kBeidouD1D2) == RtklibBroadcastMessageFamily::kLegacy);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kBeidouBcnav1) ==
              RtklibBroadcastMessageFamily::kBeidouBcnav1);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kBeidouBcnav2) ==
              RtklibBroadcastMessageFamily::kBeidouBcnav2);
static_assert(rtklib_broadcast_message_family(NavMessageFamily::kBeidouBcnav3) ==
              RtklibBroadcastMessageFamily::kBeidouBcnav3);

static_assert(rtklib_nav_message_family_projection(SYS_GPS, RtklibBroadcastMessageFamily::kLegacy).supported);
static_assert(rtklib_nav_message_family_projection(SYS_GPS, RtklibBroadcastMessageFamily::kLegacy).family ==
              NavMessageFamily::kGpsLnav);
static_assert(rtklib_nav_message_family_projection(SYS_QZS, RtklibBroadcastMessageFamily::kLegacy).family ==
              NavMessageFamily::kQzssLnav);
static_assert(rtklib_nav_message_family_projection(SYS_CMP, RtklibBroadcastMessageFamily::kLegacy).family ==
              NavMessageFamily::kBeidouD1D2);
static_assert(!rtklib_nav_message_family_projection(SYS_GAL, RtklibBroadcastMessageFamily::kLegacy).supported);
static_assert(rtklib_nav_message_family_projection(SYS_GPS, RtklibBroadcastMessageFamily::kCnav).family ==
              NavMessageFamily::kGpsCnav);
static_assert(rtklib_nav_message_family_projection(SYS_QZS, RtklibBroadcastMessageFamily::kCnav).family ==
              NavMessageFamily::kQzssCnav);
static_assert(!rtklib_nav_message_family_projection(SYS_CMP, RtklibBroadcastMessageFamily::kCnav).supported);
static_assert(rtklib_nav_message_family_projection(SYS_GPS, RtklibBroadcastMessageFamily::kCnav2).family ==
              NavMessageFamily::kGpsCnav2);
static_assert(rtklib_nav_message_family_projection(SYS_QZS, RtklibBroadcastMessageFamily::kCnav2).family ==
              NavMessageFamily::kQzssCnav2);
static_assert(!rtklib_nav_message_family_projection(SYS_CMP, RtklibBroadcastMessageFamily::kCnav2).supported);
static_assert(rtklib_nav_message_family_projection(SYS_GAL, RtklibBroadcastMessageFamily::kGalileoInav).family ==
              NavMessageFamily::kGalileoInav);
static_assert(rtklib_nav_message_family_projection(SYS_GAL, RtklibBroadcastMessageFamily::kGalileoFnav).family ==
              NavMessageFamily::kGalileoFnav);
static_assert(rtklib_nav_message_family_projection(SYS_CMP, RtklibBroadcastMessageFamily::kBeidouBcnav1).family ==
              NavMessageFamily::kBeidouBcnav1);
static_assert(rtklib_nav_message_family_projection(SYS_CMP, RtklibBroadcastMessageFamily::kBeidouBcnav2).family ==
              NavMessageFamily::kBeidouBcnav2);
static_assert(rtklib_nav_message_family_projection(SYS_CMP, RtklibBroadcastMessageFamily::kBeidouBcnav3).family ==
              NavMessageFamily::kBeidouBcnav3);
static_assert(rtklib_nav_message_family_projection(SYS_GLO, RtklibBroadcastMessageFamily::kGlonassFdma).family ==
              NavMessageFamily::kGlonassFdma);
static_assert(rtklib_nav_message_family_projection(SYS_GLO, RtklibBroadcastMessageFamily::kGlonassL3Oc).family ==
              NavMessageFamily::kGlonassL3Oc);
static_assert(!rtklib_nav_message_family_projection(SYS_GPS, RtklibBroadcastMessageFamily::kUnknown).supported);
// Freeze the current Keplerian reverse-projection asymmetry: modern family
// enums are already classified and are not revalidated against eph.system.
static_assert(rtklib_nav_message_family_projection(SYS_GPS, RtklibBroadcastMessageFamily::kGalileoInav).family ==
              NavMessageFamily::kGalileoInav);
static_assert(rtklib_nav_message_family_projection(SYS_GPS, RtklibBroadcastMessageFamily::kBeidouBcnav1).family ==
              NavMessageFamily::kBeidouBcnav1);
static_assert(rtklib_nav_message_family_projection(SYS_GPS, RtklibBroadcastMessageFamily::kGlonassFdma).family ==
              NavMessageFamily::kGlonassFdma);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_GNSS_RTKLIB_FAMILY_TAXONOMY_H_
