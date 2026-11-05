/**
 * Host-native test of the main-loop hang watchdog's timing constants (hang_wd.h).
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

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
	// The relations are also _Static_asserts in hang_wd.h; the figures are printed for the log.
	CHECK(HANG_WD_TIMEOUT_MS >= 5 * HANG_WD_LONGEST_STALL_MS, "timeout %u ms is at least 5x the longest stall %u ms",
	      HANG_WD_TIMEOUT_MS, HANG_WD_LONGEST_STALL_MS);
	CHECK(HANG_WD_TIMEOUT_MS >= 10 * HANG_WD_KICK_PERIOD_MS, "at least ten kicks (%u ms apart) fit in the timeout",
	      HANG_WD_KICK_PERIOD_MS);
	CHECK(HANG_WD_KICK_PERIOD_MS >= 3, "kicks are spaced past the ~3 ms WDT clear synchronisation");

	printf(failures ? "FAILED\n" : "all passed\n");
	return failures != 0;
}
