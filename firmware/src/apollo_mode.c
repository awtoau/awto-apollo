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
	// caller's session bookkeeping is broken. Don't silently nest.
	if (current_mode == MODE_JTAG_PROGRAM) {
		return false;
	}

	current_mode = MODE_JTAG_PROGRAM;
	return true;
}


void apollo_mode_release_jtag(void)
{
	current_mode = MODE_APOLLO_HOLD;
}


bool apollo_mode_jtag_active(void)
{
	return current_mode == MODE_JTAG_PROGRAM;
}


apollo_mode_t apollo_mode_current(void)
{
	return current_mode;
}
