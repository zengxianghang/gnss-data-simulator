#ifndef GNSS_SIM_SRC_GNSS_RTKLIB_FAMILY_TAXONOMY_H_
#define GNSS_SIM_SRC_GNSS_RTKLIB_FAMILY_TAXONOMY_H_

#include "gnss/rtklib_adapter.h"

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

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_GNSS_RTKLIB_FAMILY_TAXONOMY_H_
