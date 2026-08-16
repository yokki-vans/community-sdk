#pragma once

#include <stdint.h>

namespace freeink::x3_refresh {

// UC82xx/UC81xx panels may assert BUSY noticeably later than the command write,
// especially after temperature sensing or a cold power-on.  Fifty milliseconds
// was short enough to turn a valid refresh into a sticky bus error on some X3
// units.  The legacy blocking path already allowed one second for this edge;
// keep the split/async path at the same proven bound.
constexpr uint32_t BUSY_START_TIMEOUT_MS = 1000;

// The UC8279 factory waveform distinguishes a normal update from a fast
// differential one through PTIN/PTOUT.  Partial mode is safe only when OLD RAM
// is known to contain the frame currently visible on the panel.
constexpr bool useOtpPartial(const bool fastRequested, const bool oldPlaneSynced, const bool fullSyncForced) {
  return fastRequested && oldPlaneSynced && !fullSyncForced;
}

static_assert(BUSY_START_TIMEOUT_MS >= 1000, "X3 BUSY-start tolerance must cover the validated blocking path");
static_assert(useOtpPartial(true, true, false));
static_assert(!useOtpPartial(true, false, false));
static_assert(!useOtpPartial(false, true, false));
static_assert(!useOtpPartial(true, true, true));

}  // namespace freeink::x3_refresh
