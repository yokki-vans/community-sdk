#pragma once

#include <stdint.h>

namespace freeink::x3_refresh {

// UC82xx/UC81xx panels may assert BUSY noticeably later than the command write,
// especially after temperature sensing or a cold power-on.  Fifty milliseconds
// was short enough to turn a valid refresh into a sticky bus error on some X3
// units; one second still tripped on field units whose OTP temperature re-sense
// pushed the start edge past the bound — every navigation frame then failed the
// handshake, reinitialized the panel and escalated to a forced strong waveform
// (black scrub + lag per keypress).  Bound the failure wait generously; a
// healthy panel returns as soon as BUSY asserts, so the extra slack only ever
// costs time on frames that were going to fail anyway.
constexpr uint32_t BUSY_START_TIMEOUT_MS = 3000;

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
