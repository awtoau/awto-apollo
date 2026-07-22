/**
 * Host-native unit test for the Apollo control-plane mode arbitration.
 *
 * The mode state machine (apollo_mode.c) is plain C with no MCU dependency, so
 * we compile it for the host and drive it directly. This tests the actual lock
 * logic that gates the shared JTAG/UART pins — that a JTAG session takes the
 * lock, that re-entry is refused, that release returns to HOLD, and that the
 * `jtag_active()` query the pinmux guards rely on tracks state correctly.
 *
 * Build & run:  make -C firmware/test        (or: firmware/test/Makefile)
 *
 * This file is part of Apollo.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdbool.h>

#include "apollo_mode.h"

static int failures = 0;
static int checks   = 0;

#define CHECK(cond, msg)                                                       \
	do {                                                                       \
		checks++;                                                               \
		if (cond) {                                                             \
			printf("  ok   - %s\n", msg);                                       \
		} else {                                                                \
			printf("  FAIL - %s  (%s:%d)\n", msg, __FILE__, __LINE__);          \
			failures++;                                                         \
		}                                                                       \
	} while (0)


/* The lock starts in HOLD and reports JTAG inactive. */
static void test_initial_state(void)
{
	printf("test_initial_state\n");
	CHECK(apollo_mode_current() == MODE_APOLLO_HOLD, "starts in MODE_APOLLO_HOLD");
	CHECK(apollo_mode_jtag_active() == false,        "JTAG not active at start");
}


/* Acquiring the JTAG lock succeeds from HOLD and flips state + query. */
static void test_acquire_succeeds_from_hold(void)
{
	printf("test_acquire_succeeds_from_hold\n");
	apollo_mode_release_jtag();  /* known state */

	bool got = apollo_mode_acquire_jtag();
	CHECK(got == true,                                "acquire returns true from HOLD");
	CHECK(apollo_mode_current() == MODE_JTAG_PROGRAM, "mode is MODE_JTAG_PROGRAM after acquire");
	CHECK(apollo_mode_jtag_active() == true,          "jtag_active() true while held");

	apollo_mode_release_jtag();
}


/* Re-acquiring while already in JTAG mode is refused (no silent nesting). */
static void test_reacquire_fails_while_held(void)
{
	printf("test_reacquire_fails_while_held\n");
	apollo_mode_release_jtag();

	bool first  = apollo_mode_acquire_jtag();
	bool second = apollo_mode_acquire_jtag();
	CHECK(first == true,                              "first acquire succeeds");
	CHECK(second == false,                            "second acquire is REFUSED while held");
	CHECK(apollo_mode_jtag_active() == true,          "still active after refused re-acquire");
	CHECK(apollo_mode_current() == MODE_JTAG_PROGRAM, "mode unchanged by refused re-acquire");

	apollo_mode_release_jtag();
}


/* Releasing returns to HOLD and clears the query the pin guards depend on. */
static void test_release_returns_to_hold(void)
{
	printf("test_release_returns_to_hold\n");
	apollo_mode_acquire_jtag();

	apollo_mode_release_jtag();
	CHECK(apollo_mode_current() == MODE_APOLLO_HOLD, "back in HOLD after release");
	CHECK(apollo_mode_jtag_active() == false,        "jtag_active() false after release");
}


/* After a full release, a fresh acquire is allowed again (clean cycling). */
static void test_reacquire_after_release(void)
{
	printf("test_reacquire_after_release\n");
	apollo_mode_acquire_jtag();
	apollo_mode_release_jtag();

	bool again = apollo_mode_acquire_jtag();
	CHECK(again == true,                     "acquire succeeds again after a clean release");
	CHECK(apollo_mode_jtag_active() == true, "active after re-acquire");

	apollo_mode_release_jtag();
}


/* Release is idempotent: releasing when already in HOLD is harmless. */
static void test_release_idempotent(void)
{
	printf("test_release_idempotent\n");
	apollo_mode_release_jtag();
	apollo_mode_release_jtag();  /* second release must not underflow/flip state */
	CHECK(apollo_mode_current() == MODE_APOLLO_HOLD, "double release stays in HOLD");
	CHECK(apollo_mode_jtag_active() == false,        "double release keeps JTAG inactive");

	/* And a normal acquire still works afterwards. */
	CHECK(apollo_mode_acquire_jtag() == true,        "acquire works after redundant releases");
	apollo_mode_release_jtag();
}


int main(void)
{
	printf("== apollo_mode lock unit tests ==\n");

	test_initial_state();
	test_acquire_succeeds_from_hold();
	test_reacquire_fails_while_held();
	test_release_returns_to_hold();
	test_reacquire_after_release();
	test_release_idempotent();

	printf("\n%d checks, %d failures\n", checks, failures);
	if (failures == 0) {
		printf("PASS\n");
		return 0;
	}
	printf("FAIL\n");
	return 1;
}
