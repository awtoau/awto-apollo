/**
 * Main-loop hang watchdog: timing constants. Host-testable.
 *
 * The SAMD11 WDT is enabled in main() and kicked from the main loop's housekeeping block, so a
 * firmware that stops iterating (a wait on hardware that never completes, a fault handler, a
 * stalled tick) resets the MCU instead of staying enumerated and silent.
 * Notes: docs/board-usb-recovery.md.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __HANG_WD_H__
#define __HANG_WD_H__

// Longest legitimate stretch without a main-loop iteration, from the code: usb_switch.c waits
// board_delay(100) in take_over_usb()/hand_off_usb(), and a sideband command waits up to ~21 ms
// (fpga_adv.c). Not measured on a board.
#define HANG_WD_LONGEST_STALL_MS 100u

// WDT_CONFIG_PER_16K at the 32.768 kHz WDT clock (the reset default that dfu.c also relies on): 500 ms,
// 5x the longest stall above. On expiry the MCU resets and Apollo re-enumerates.
#define HANG_WD_TIMEOUT_MS 500u

// The kick is the main loop's 20 ms housekeeping block. A WDT CLEAR is write-synchronised to the
// WDT clock, so kicking every loop pass could stall the bus behind it; 20 ms spacing does not.
#define HANG_WD_KICK_PERIOD_MS 20u

_Static_assert(HANG_WD_TIMEOUT_MS >= 5 * HANG_WD_LONGEST_STALL_MS, "timeout must clear the longest stall 5x");
_Static_assert(HANG_WD_TIMEOUT_MS >= 10 * HANG_WD_KICK_PERIOD_MS, "several kicks must fit in the timeout");

// Per-MCU: enable the hardware watchdog with HANG_WD_TIMEOUT_MS, and clear it.
void hang_wd_enable(void);
void hang_wd_kick(void);

#endif
