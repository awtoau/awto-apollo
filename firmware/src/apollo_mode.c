/**
 * Apollo control-plane mode arbitration.
 *
 * See apollo_mode.h for the rationale. This is deliberately a tiny, single-owner
 * state machine: the SAMD11 control path is cooperative and single-threaded
 * (main loop + USB/UART ISRs), so a plain volatile flag is sufficient — the only
 * concurrency of concern is an ISR/callback observing the flag, never two
 * writers racing to acquire.
 *
 * This file is part of Apollo.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "apollo_mode.h"

static volatile apollo_mode_t current_mode = MODE_APOLLO_HOLD;


bool apollo_mode_acquire_jtag(void)
{
	// Reject re-entry: a second acquire without an intervening release means the
	// caller's session bookkeeping is broken. Don't silently nest. Either JTAG
	// state counts as already-held.
	if (apollo_mode_jtag_active()) {
		return false;
	}

	current_mode = MODE_JTAG_PROGRAM;
	return true;
}


void apollo_mode_enter_programming(void)
{
	// Only meaningful inside an open JTAG session. If no session is open there
	// is nothing to escalate, and we must not grant the pin lock by side effect.
	if (current_mode == MODE_JTAG_PROGRAM) {
		current_mode = MODE_JTAG_PROGRAMMING;
	}
}


bool apollo_mode_programming_active(void)
{
	return current_mode == MODE_JTAG_PROGRAMMING;
}


void apollo_mode_release_jtag(void)
{
	current_mode = MODE_APOLLO_HOLD;
}


bool apollo_mode_jtag_active(void)
{
	// True in BOTH JTAG states: escalating to MODE_JTAG_PROGRAMMING must not
	// drop the pin lock -- that is precisely when it matters most.
	return (current_mode == MODE_JTAG_PROGRAM)
	    || (current_mode == MODE_JTAG_PROGRAMMING);
}


apollo_mode_t apollo_mode_current(void)
{
	return current_mode;
}
