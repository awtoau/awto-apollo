/**
 * Host-native test of the main-loop hang watchdog's kick timing (hang_wd.h).
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "hang_wd.h"

static int failures = 0;

#define CHECK(cond, ...)                                                       \
	do {                                                                       \
		if (cond) { printf("  ok   - "); }                                     \
		else { printf("  FAIL - "); failures++; }                              \
		printf(__VA_ARGS__);                                                   \
		printf("\n");                                                          \
	} while (0)

int main(void)
{
	CHECK(!hang_wd_kick_due(0, 0), "no kick on the tick it was kicked");
	CHECK(!hang_wd_kick_due(HANG_WD_KICK_PERIOD_MS - 1, 0), "no kick one tick before the period");
	CHECK(hang_wd_kick_due(HANG_WD_KICK_PERIOD_MS, 0), "kick at the period");
	CHECK(hang_wd_kick_due(0x00000010u, 0xFFFFFFF0u) == (0x20u >= HANG_WD_KICK_PERIOD_MS),
	      "the 32-bit millisecond counter wrapping does not change the answer");
	CHECK(hang_wd_kick_due(0xFFFFFFF0u + HANG_WD_KICK_PERIOD_MS, 0xFFFFFFF0u),
	      "kick at the period across the wrap");

	// The timeout must outlast the longest legitimate stall with margin, and the kick period must
	// fit inside it at least twice, so one late kick does not reset a healthy device.
	CHECK(HANG_WD_TIMEOUT_MS >= 2 * HANG_WD_KICK_PERIOD_MS, "timeout holds at least two kick periods");
	CHECK(HANG_WD_TIMEOUT_MS >= HANG_WD_LONGEST_STALL_MS * 5 / 2,
	      "timeout is at least 2.5x the longest stall derived from the code");

	printf(failures ? "FAILED\n" : "all passed\n");
	return failures != 0;
}
