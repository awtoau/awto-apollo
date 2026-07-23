/**
 * Apollo control-plane mode arbitration.
 *
 * Enforces a sticky mutual-exclusion state machine between JTAG programming and
 * the UART console. On the Cynthion d11 board these two subsystems contend for
 * the same physical pins (PA10/PA11/PA14/PA15) with no hardware arbitration, and
 * the pins cannot be split (see docs/apollo_samd11_mcu/
 * apollo_serial_interface_and_mode_exclusivity_design.md and awtoau/
 * cynthion-workspace#65). While JTAG owns the pins, any attempt to (re)pinmux
 * them for the UART console must be refused so that a JTAG program/configure
 * sequence cannot be interrupted mid-flash — most importantly by a TinyUSB CDC
 * callback lazily re-initializing the UART.
 *
 * The module is board-independent: on boards that are not pin-starved the lock
 * is still taken for the duration of a JTAG session, but nothing else contends
 * for the pins, so the guard is simply inert.
 *
 * This file is part of Apollo.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __APOLLO_MODE_H__
#define __APOLLO_MODE_H__

#include <stdbool.h>

typedef enum {
	// Default mode: UART console may own the shared pins; JTAG is idle.
	MODE_APOLLO_HOLD = 0,

	// A JTAG session is open and owns the shared pins, but no programming
	// sequence has started yet (e.g. a plain jtag-scan). The pins are locked,
	// but the control plane is otherwise unrestricted.
	MODE_JTAG_PROGRAM = 1,

	// A JTAG programming/configure sequence is in flight. In addition to the
	// pin lock, conflicting control-plane requests are refused so the sequence
	// cannot be interrupted. Entered by escalation from MODE_JTAG_PROGRAM on
	// the first programming request; left only by ending the JTAG session.
	MODE_JTAG_PROGRAMMING = 2,
} apollo_mode_t;

/**
 * Attempt to enter MODE_JTAG_PROGRAM and take exclusive ownership of the shared
 * pins. Called at the single choke point where a JTAG session begins
 * (jtag_init()).
 *
 * @return true if the lock was acquired (caller may proceed with JTAG); false if
 *         JTAG mode was already active (should not happen for a well-behaved
 *         host, but the caller must not proceed on false).
 */
bool apollo_mode_acquire_jtag(void);

/**
 * Leave MODE_JTAG_PROGRAM and return to MODE_APOLLO_HOLD. Called at the single
 * choke point where a JTAG session ends (jtag_deinit()). Idempotent.
 */
void apollo_mode_release_jtag(void);

/**
 * @return true iff JTAG currently owns the shared pins (in either JTAG state).
 *         Non-JTAG pin consumers (uart_initialize(), the console CDC callbacks)
 *         must check this and refuse to repinmux while it is true.
 */
bool apollo_mode_jtag_active(void);

/**
 * Escalate an open JTAG session to MODE_JTAG_PROGRAMMING, marking a
 * programming/configure sequence as in flight. Called when the first
 * programming-class vendor request arrives.
 *
 * No-op if a JTAG session is not open (nothing to escalate) or if already
 * escalated.
 */
void apollo_mode_enter_programming(void);

/**
 * @return true iff a JTAG programming sequence is in flight. The vendor-request
 *         dispatcher uses this to refuse conflicting control-plane requests, so
 *         programming cannot be interrupted.
 */
bool apollo_mode_programming_active(void);

/**
 * @return the current control-plane mode.
 */
apollo_mode_t apollo_mode_current(void);

#endif
