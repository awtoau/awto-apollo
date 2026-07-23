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


/* Escalation to programming requires an open JTAG session. */
static void test_programming_requires_open_session(void)
{
	printf("test_programming_requires_open_session\n");
	apollo_mode_release_jtag();

	/* No session open: escalation must NOT grant the lock by side effect. */
	apollo_mode_enter_programming();
	CHECK(apollo_mode_current() == MODE_APOLLO_HOLD,   "escalation from HOLD is a no-op");
	CHECK(apollo_mode_jtag_active() == false,          "escalation from HOLD does not lock pins");
	CHECK(apollo_mode_programming_active() == false,   "not programming after no-op escalation");
}


/* Escalating an open session enters PROGRAMMING and keeps the pin lock. */
static void test_escalation_enters_programming(void)
{
	printf("test_escalation_enters_programming\n");
	apollo_mode_release_jtag();
	apollo_mode_acquire_jtag();

	CHECK(apollo_mode_programming_active() == false, "JTAG_START alone is not programming");

	apollo_mode_enter_programming();
	CHECK(apollo_mode_current() == MODE_JTAG_PROGRAMMING, "escalated to MODE_JTAG_PROGRAMMING");
	CHECK(apollo_mode_programming_active() == true,       "programming reported active");
	/* The critical invariant: escalating must NOT drop the pin lock. */
	CHECK(apollo_mode_jtag_active() == true,              "pins STILL locked while programming");

	apollo_mode_release_jtag();
}


/* Escalation is idempotent, and re-acquire is still refused while programming. */
static void test_programming_is_sticky(void)
{
	printf("test_programming_is_sticky\n");
	apollo_mode_release_jtag();
	apollo_mode_acquire_jtag();
	apollo_mode_enter_programming();

	apollo_mode_enter_programming();  /* second escalation must not regress */
	CHECK(apollo_mode_current() == MODE_JTAG_PROGRAMMING, "double escalation stays PROGRAMMING");
	CHECK(apollo_mode_acquire_jtag() == false,            "re-acquire refused while programming");
	CHECK(apollo_mode_jtag_active() == true,              "still locked after refused acquire");

	apollo_mode_release_jtag();
}


/* Releasing from PROGRAMMING returns all the way to HOLD. */
static void test_release_from_programming(void)
{
	printf("test_release_from_programming\n");
	apollo_mode_acquire_jtag();
	apollo_mode_enter_programming();

	apollo_mode_release_jtag();
	CHECK(apollo_mode_current() == MODE_APOLLO_HOLD,  "PROGRAMMING -> HOLD on release");
	CHECK(apollo_mode_jtag_active() == false,         "pins freed after release");
	CHECK(apollo_mode_programming_active() == false,  "programming cleared after release");

	/* And the next session starts un-escalated. */
	CHECK(apollo_mode_acquire_jtag() == true,         "fresh session acquires after programming");
	CHECK(apollo_mode_programming_active() == false,  "fresh session starts un-escalated");
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
	test_programming_requires_open_session();
	test_escalation_enters_programming();
	test_programming_is_sticky();
	test_release_from_programming();

	printf("\n%d checks, %d failures\n", checks, failures);
	if (failures == 0) {
		printf("PASS\n");
		return 0;
	}
	printf("FAIL\n");
	return 1;
}
